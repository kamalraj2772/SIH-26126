#!/usr/bin/env bash
# SIH26126 GPS-denied UGV navigation -- one-command demo.
#   ./run_demo.sh            full mission, writes src/sih_demo/output/sih26126_demo.mp4
#   ./run_demo.sh --duration 60    short run, for a quick check
set -euo pipefail
cd "$(dirname "$0")"

if [ ! -d .demoenv ]; then
  echo "[setup] creating .demoenv (one-off, ~60 s)"
  python3 -m venv --system-site-packages .demoenv
  .demoenv/bin/pip install --quiet --upgrade pip
  .demoenv/bin/pip install --quiet mujoco pyproj rasterio imageio imageio-ffmpeg \
                                   opencv-python-headless
fi

export PYTHONPATH="$PWD/src/sih_sim:$PWD/src/sih_demo"
export MUJOCO_GL="${MUJOCO_GL:-egl}"     # headless GPU rendering; use 'osmesa' with no GPU
exec .demoenv/bin/python -m sih_demo.run_demo "$@"
