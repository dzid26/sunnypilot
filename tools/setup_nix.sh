#!/usr/bin/env bash
set -euo pipefail

if ! command -v nix >/dev/null 2>&1; then
  echo "installing nix..."
  if command -v curl >/dev/null 2>&1; then
    sh <(curl -fsSL https://nixos.org/nix/install) --daemon
  else
    echo "error: install curl first (e.g. apt install curl)" >&2
    exit 1
  fi
  . /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh 2>/dev/null || true
fi

mkdir -p "$HOME/.config/nix"
if ! grep -q '^experimental-features' "$HOME/.config/nix/nix.conf" 2>/dev/null; then
  echo 'experimental-features = nix-command flakes' >> "$HOME/.config/nix/nix.conf"
fi

echo "Nix is ready."
echo "Enter dev shell with: nix develop"
