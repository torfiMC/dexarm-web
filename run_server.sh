#!/usr/bin/env bash
# Start dexarm-web. Installs what's missing first (see install.sh). Options are passed through, e.g.:
#   ./run_server.sh --port 9000
#   ./run_server.sh --host 127.0.0.1      # only reachable from this machine
set -euo pipefail
cd "$(dirname "$0")"

if ! command -v uv >/dev/null 2>&1 && [ -x "$HOME/.local/bin/uv" ]; then export PATH="$HOME/.local/bin:$PATH"; fi

if ! command -v uv >/dev/null 2>&1 || [ ! -d .venv ] || [ ! -f node_modules/three/package.json ] \
   || [ ! -f node_modules/urdf-loader/package.json ] || [ ! -f vendor/URDF-of-DexArm/URDF/DexArm.urdf ]; then
  echo "Some parts are missing; running install.sh first."
  ./install.sh
  export PATH="$HOME/.local/bin:$PATH"
fi

cmd=(uv run --quiet dexarm-web "$@")
# Added to the dialout group but not logged in again yet? Run with the group anyway.
if ! id -nG | tr ' ' '\n' | grep -qx dialout \
   && getent group dialout | cut -d: -f4 | tr ',' '\n' | grep -qx "$USER" && command -v sg >/dev/null; then
  exec sg dialout -c "$(printf '%q ' "${cmd[@]}")"
fi
exec "${cmd[@]}"
