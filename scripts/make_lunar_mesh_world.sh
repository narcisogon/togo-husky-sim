#!/usr/bin/env bash
set -eo pipefail

cd /mnt/c/Users/Username/OneDrive/Desktop/husky
if [[ ! -f lunar_world/lunar_heightmap.png ]]; then
  bash scripts/make_lunar_heightmap_world.sh
fi
python3 scripts/make_lunar_mesh_world.py "$@"