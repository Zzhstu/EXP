#!/usr/bin/env bash
# Keep the existing example demo as the default; this is an explicit new scene.
set -euo pipefail
ASTRA_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$ASTRA_SCRIPT_DIR/dynamic_comparison_demo.sh" ours --scene cangku "$@"
