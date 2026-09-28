#!/usr/bin/env bash
# SIH26126 Isaac Sim mission — one-command simulation start.
#   ./run_isaac.sh              GUI window (needs the desktop display)
#   ./run_isaac.sh --headless   no window (SSH-safe)
# Then, in a second terminal:
#   source /opt/ros/jazzy/setup.bash && source install/setup.bash
#   ros2 launch sih_isaac nav.launch.py
#   ros2 run sih_isaac utm_goal.py -e 403562.52 -n 1450024.12
set -euo pipefail
cd "$(dirname "$0")"

if [ ! -f src/sih_isaac/usd/sih26126_world.usd ]; then
  echo "[setup] generating world + GeoTIFF + USD (one-off, ~3 min)"
  PYTHONPATH="$PWD/src/sih_sim:$PWD/src/sih_demo:$PWD/src/sih_isaac" \
    .demoenv/bin/python src/sih_isaac/sih_isaac/world.py
  (cd /isaacsim && ./python.sh "$PWD/src/sih_isaac/scripts/build_scene.py") ||
  (cd /isaacsim && ./python.sh "$OLDPWD/src/sih_isaac/scripts/build_scene.py")
fi

set +u
source /opt/ros/jazzy/setup.bash
set -u
cd /isaacsim && exec ./python.sh "$OLDPWD/src/sih_isaac/scripts/run_sim.py" "$@"
