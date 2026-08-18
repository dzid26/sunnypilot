# first time: run tools/setup_nix.sh, then enter the dev shell with: nix develop
{
  description = "sunnypilot/openpilot reproducible development environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.11";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };

        python = pkgs.python312;

        # NO_VERIFY skips Ubuntu-only OS check so op.sh works on any distro
        op = pkgs.writeShellScriptBin "op" ''
          OPENPILOT_ROOT="''${OPENPILOT_ROOT:-$(pwd)}"
          export NO_VERIFY=1
          exec ${pkgs.bash}/bin/bash "$OPENPILOT_ROOT/tools/op.sh" "$@"
        '';

        runtimeLibs = with pkgs; [
          stdenv.cc.cc.lib    # libstdc++ for Python C extensions
          zlib                # numpy and other pip wheels
          bzip2               # pip wheels
          zstd                # pip wheels
          libusb1             # python libusb1 (panda) — ctypes dlopen by name
          portaudio           # sounddevice (micd, soundd)
          llvmPackages_18.libllvm  # tinygrad ctypes
          qt5.qtbase          # Qt + xcb plugin runtime (cabana has RPATH, PlotJuggler dlopens by name)
          qt5.qtsvg           # PlotJuggler
          qt5.qtx11extras     # PlotJuggler
          libx11              # raylib GLFW (desktop UI)
          libxrandr           # raylib GLFW
          libxinerama         # raylib GLFW
          libxcursor          # raylib GLFW
          libxi               # raylib GLFW
          libxcb              # PlotJuggler prebuilt binary
          elfutils            # libdw (PlotJuggler prebuilt binary)
          libGL               # GLVND dispatcher (libGL.so.1) — raylib GLFW
          mesa                # GLX vendor (libGLX_mesa) + D3D12 driver for WSL
        ];
      in
      {
        devShells.default = pkgs.mkShell {
          name = "openpilot";

          # Nix gcc wrapper injects _FORTIFY_SOURCE which requires -O1+.
          # Some test targets (libsafety) compile with -O0, causing errors.
          hardeningDisable = [ "fortify" ];

          nativeBuildInputs = with pkgs; [
            bashInteractive  # https://github.com/NixOS/nix/issues/6982
            gcc14
            clang   # used by test_generated_header
            gnumake
            cmake
            pkg-config
            qt5.qtbase.dev
          ];

          buildInputs = with pkgs; [
            python
            uv
            git
            git-lfs
            tmux
            qt5.qtbase
            qt5.qtcharts
            qt5.qtwayland
            libglvnd.dev       # GL/gl.h headers
            openssl
            libusb1
            libGL                 # GLVND dispatcher (libGL.so.1)
            mesa                  # GLX vendor (libGLX_mesa) + D3D12 driver for WSL
            zlib
            bzip2
            zstd
            llvmPackages_18.libllvm
            libva              # vendored ffmpeg VA-API (link-time)
            libdrm
            libx11
            libxext
            libxcb
            op
          ];

          shellHook = ''
            export OPENPILOT_ROOT="$(pwd)"

            # op checks fail on non-Ubuntu; NO_VERIFY keeps op usable
            export NO_VERIFY=1

            # .python-version pins 3.12.13; make uv use the nix python instead of
            # trying (and failing) to download a managed one
            export UV_PYTHON="${python}/bin/python3"

            # vendored ffmpeg libavutil.a references VA-API/DRM symbols
            export NIX_LDFLAGS="$NIX_LDFLAGS -lva -lva-drm -ldrm"

            # point non-FHS nix libs at vendored binaries
            export LD_LIBRARY_PATH="${pkgs.lib.makeLibraryPath runtimeLibs}"

            # replay's vendored ncurses has a wrong compiled-in terminfo path
            export TERMINFO_DIRS="${pkgs.ncurses}/share/terminfo:/usr/share/terminfo:/lib/terminfo:/usr/lib/terminfo"

            # use bundled fontconfig when the host lacks one
            if [[ ! -f /etc/fonts/fonts.conf ]]; then
              export FONTCONFIG_FILE="${pkgs.fontconfig.out}/etc/fonts/fonts.conf"
              export FONTCONFIG_PATH="${pkgs.fontconfig.out}/etc/fonts"
            fi

            # tinygrad ctypes overrides (see tinygrad/runtime/support/c.py findlib)
            export LLVM_PATH="${pkgs.llvmPackages_18.libllvm.lib}/lib/libLLVM.so"
            export LIBC_PATH="${pkgs.glibc}/lib/libc.so.6"

            export QT_QPA_PLATFORM_PLUGIN_PATH="${pkgs.qt5.qtbase.bin}/lib/qt-${pkgs.qt5.qtbase.version}/plugins/platforms"
            export QT_PLUGIN_PATH="${pkgs.qt5.qtbase.bin}/lib/qt-${pkgs.qt5.qtbase.version}/plugins:${pkgs.qt5.qtwayland.bin}/lib/qt-${pkgs.qt5.qtbase.version}/plugins:${pkgs.qt5.qtsvg.bin}/lib/qt-${pkgs.qt5.qtsvg.version}/plugins"

            # Submodules should be present before uv sync: pyproject path sources
            # (pandacan, opendbc, msgq, ...) live in the submodule checkouts.
            if git submodule status --recursive | grep -q '^-'; then
              git submodule update --init --recursive --jobs 4
            fi

            # venv sync, only when lockfiles change
            stamp=".venv/.nix-synced"
            hash="$(sha256sum uv.lock pyproject.toml | sha256sum | cut -d' ' -f1)"
            if [[ ! -f "$stamp" ]] || [[ "$(cat "$stamp")" != "$hash" ]]; then
              uv sync --frozen --all-extras
              mkdir -p .venv && echo "$hash" > "$stamp"
            fi

            # configure the LFS filter only on fresh checkouts (installed after first run)
            git config --get filter.lfs.process >/dev/null 2>&1 || git lfs install
            git lfs pull

            # post-commit hook (linter on commit), only when missing
            if [[ ! -f .git/hooks/post-commit ]]; then
              "$OPENPILOT_ROOT/tools/op.sh" --no-verify post-commit

            fi


            echo ""
            echo "openpilot dev shell ready. Try it out:"
            echo "  op build    Build openpilot"
            echo "  op cabana   Launch Cabana"
            echo "  op test     Run tests"
            echo "  code .      Open VSCode from nix dev shell"
            echo ""

            if [ -f .venv/bin/activate ]; then source .venv/bin/activate; fi
          '';
        };
      });
}
