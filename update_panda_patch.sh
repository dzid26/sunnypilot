#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PANDA_DIR="$ROOT_DIR/panda"
OUTPUT_PATH="$ROOT_DIR/panda_changes.patch"
MODE="all"  # all (HEAD), cached, unstaged
PATHS=()

usage() {
  cat <<'EOF'
Usage: update_panda_patch.sh [options] [path...]

Create a top-level patch with paths prefixed as a/panda/... and b/panda/...

Options:
  --cached          include staged changes only
  --unstaged        include unstaged changes only
  -o, --output FILE output patch path (default: panda_changes.patch in repo root)
  -h, --help        show this help

Defaults:
  - Includes staged + unstaged changes by diffing against HEAD.
  - Includes all panda paths unless one or more paths are provided.

Examples:
  ./update_panda_patch.sh
  ./update_panda_patch.sh board/drivers/can_common.h -o panda_changes.patch
  ./update_panda_patch.sh --cached board/safety
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --cached)
      MODE="cached"
      shift
      ;;
    --unstaged)
      MODE="unstaged"
      shift
      ;;
    -o|--output)
      OUTPUT_PATH="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      PATHS+=("$@")
      break
      ;;
    *)
      PATHS+=("$1")
      shift
      ;;
  esac
done

if [[ ${#PATHS[@]} -eq 0 ]]; then
  PATHS=(".")
fi

DIFF_TARGET=()
case "$MODE" in
  all)
    DIFF_TARGET=(HEAD)
    ;;
  cached)
    DIFF_TARGET=(--cached)
    ;;
  unstaged)
    DIFF_TARGET=()
    ;;
  *)
    echo "Invalid mode: $MODE" >&2
    exit 1
    ;;
esac

git -C "$PANDA_DIR" diff \
  --src-prefix=a/panda/ --dst-prefix=b/panda/ \
  "${DIFF_TARGET[@]}" -- "${PATHS[@]}" > "$OUTPUT_PATH"

if [[ -s "$OUTPUT_PATH" ]]; then
  echo "Wrote $OUTPUT_PATH"
else
  echo "No matching panda changes; wrote empty patch: $OUTPUT_PATH"
fi
