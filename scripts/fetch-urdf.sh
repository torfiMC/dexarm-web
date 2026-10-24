#!/usr/bin/env bash
# Fetch the DexArm 3D model (URDF + STL meshes) used by the 3D view. It isn't bundled with this repo.
set -euo pipefail
cd "$(dirname "$0")/.."
dest=vendor/URDF-of-DexArm
if [ -d "$dest/.git" ]; then
  git -C "$dest" pull --ff-only
else
  mkdir -p vendor
  git clone --depth 1 https://github.com/Rotrics-Dev/URDF-of-DexArm "$dest"
fi
test -f "$dest/URDF/DexArm.urdf" && echo "OK: $dest/URDF/DexArm.urdf"
