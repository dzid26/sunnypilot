#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
FALLBACK_LAUNCH_SH="$DIR/launch_chffrplus.sh"
C3_LAUNCH_SH="$DIR/sunnypilot/system/hardware/c3/launch_chffrplus.sh"
PATCH_FILE="$DIR/panda_changes.patch"

# On any failure, run the fallback launcher
trap 'exec "$FALLBACK_LAUNCH_SH"' ERR

apply_startup_patch() {
  if [ ! -f "$PATCH_FILE" ]; then
    return
  fi

  local patch_name
  local panda_dir
  patch_name="$(basename "$PATCH_FILE")"
  panda_dir="$DIR/panda"

  if git -C "$panda_dir" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "Resetting panda repo before startup patch: $patch_name"
    git -C "$panda_dir" reset --hard HEAD
    git -C "$panda_dir" clean -fd
  else
    echo "Panda repo not found at $panda_dir; skipping reset/clean"
  fi

  echo "Applying startup patch: $patch_name"
  if git -C "$DIR" apply "$PATCH_FILE"; then
    return
  fi
  echo "Startup patch apply failed, retrying with 3-way merge: $patch_name"

  if git -C "$DIR" apply -3 "$PATCH_FILE"; then
    echo "Startup patch applied with 3-way merge: $patch_name"
    return
  fi

  echo "Startup patch is not compatible with current code: $patch_name"
}

MODEL="$(tr -d '\0' < "/sys/firmware/devicetree/base/model")"
export MODEL

apply_startup_patch

if [ "$MODEL" = "comma tici" ]; then
  # Force a failure if the launcher doesn't exist
  [ -x "$C3_LAUNCH_SH" ] || false

  # If it exists, run it
  exec "$C3_LAUNCH_SH"
fi

exec "$FALLBACK_LAUNCH_SH"
