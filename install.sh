#!/usr/bin/env bash
# Install everything dexarm-web needs. Safe to run again: finished steps are skipped.
#   - uv (Python package manager), if missing
#   - Python dependencies (into .venv, managed by uv)
#   - three.js and urdf-loader (into node_modules; npm if available, otherwise downloaded directly)
#   - the DexArm 3D model from Rotrics' GitHub (into vendor/)
set -euo pipefail
cd "$(dirname "$0")"

say()  { printf '\033[1;34m==>\033[0m %s\n' "$*"; }
ok()   { printf '    \033[32mok\033[0m  %s\n' "$*"; }
warn() { printf '    \033[33m!\033[0m   %s\n' "$*"; }
need() { command -v "$1" >/dev/null 2>&1; }

# ---------------------------------------------------------------- uv
say "Python tooling (uv)"
if ! need uv && [ -x "$HOME/.local/bin/uv" ]; then export PATH="$HOME/.local/bin:$PATH"; fi
if need uv; then
  ok "uv $(uv --version | cut -d' ' -f2)"
else
  need curl || { echo "curl is required to install uv. Install curl (or uv: https://docs.astral.sh/uv/) and run again."; exit 1; }
  warn "uv not found; installing it from https://astral.sh/uv (into ~/.local/bin)"
  curl -LsSf https://astral.sh/uv/install.sh | sh
  export PATH="$HOME/.local/bin:$PATH"
  need uv || { echo "uv installation failed; see https://docs.astral.sh/uv/"; exit 1; }
  ok "uv $(uv --version | cut -d' ' -f2)"
fi

# ---------------------------------------------------------------- Python packages
say "Python packages"
uv sync --quiet
ok "environment in .venv"

# ---------------------------------------------------------------- front-end libraries
say "3D libraries (three.js, urdf-loader)"
fetch_pkg() {  # name version: download a package tarball from the npm registry, no Node needed
  local name=$1 version=$2 dest="node_modules/$1"
  if [ -f "$dest/package.json" ] && grep -q "\"version\": \"$version\"" "$dest/package.json"; then
    ok "$name $version"; return
  fi
  rm -rf "$dest"; mkdir -p "$dest"
  curl -fsSL "https://registry.npmjs.org/$name/-/$name-$version.tgz" | tar -xz -C "$dest" --strip-components=1
  ok "$name $version (downloaded)"
}
if [ -f node_modules/three/package.json ] && [ -f node_modules/urdf-loader/package.json ]; then
  ok "already installed"
elif need npm; then
  npm ci --silent --no-audit --no-fund
  ok "installed with npm"
else
  warn "npm not found; downloading the packages directly"
  fetch_pkg three 0.186.1
  fetch_pkg urdf-loader 0.13.1
fi

# ---------------------------------------------------------------- 3D model
say "DexArm 3D model (Rotrics-Dev/URDF-of-DexArm)"
dest=vendor/URDF-of-DexArm
if [ -f "$dest/URDF/DexArm.urdf" ]; then
  ok "already in $dest"
elif need git; then
  mkdir -p vendor
  git clone --quiet --depth 1 https://github.com/Rotrics-Dev/URDF-of-DexArm "$dest"
  ok "cloned into $dest"
else
  mkdir -p "$dest"
  curl -fsSL https://github.com/Rotrics-Dev/URDF-of-DexArm/archive/HEAD.tar.gz | tar -xz -C "$dest" --strip-components=1
  ok "downloaded into $dest"
fi

# ---------------------------------------------------------------- serial port access
say "Serial port access"
if id -nG | tr ' ' '\n' | grep -qx dialout; then
  ok "you're in the dialout group"
elif getent group dialout | cut -d: -f4 | tr ',' '\n' | grep -qx "$USER"; then
  ok "you're in the dialout group (log out and back in to use it everywhere; run_server.sh works already)"
else
  warn "you're not in the dialout group, so the server can't open the arm's USB port. Run:"
  echo "        sudo usermod -aG dialout $USER"
  echo "    then log out and back in (or just use ./run_server.sh, which picks it up straight away)."
fi

echo
say "Done. Start the server with ./run_server.sh, then open http://localhost:8080"
