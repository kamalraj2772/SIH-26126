#!/usr/bin/env bash
# SIH26126 Isaac Sim mission — terminal 1 of 3.
#
#   ./run_isaac.sh              headless + record this run   (the normal way)
#   ./run_isaac.sh --no-record  no recording; add --gui for the Isaac window
#
# Recording creates mission_recordings/report_<HHMM>/ and points the other two
# terminals at it. Then, in a second and third terminal:
#   source /opt/ros/jazzy/setup.bash && source install/setup.bash
#   ros2 launch sih_isaac nav.launch.py
#   ros2 run sih_isaac utm_goal.py --lat 13.1147 --lon 80.1098
#
# The GUI is off while recording on purpose: GUI + RViz on one 4090 drops the
# sim to ~0.33x real time and Nav2's TF timing then starves (missions abort).
# The four 1080p camera videos (chase, side, top, and the rover's own ZED) are
# rendered offscreen and do not need it. If the sim is frame-rate bound, pass
# --no-onboard to drop the rover-camera render product.
#
# Every recorded run ends with, in its report directory: the four camera
# videos, rviz.mp4, costmap.mp4, ekf.mp4, slam_graph.mp4, geo_pipeline.mp4
# (GeoTIFF -> DEM -> UTM sequence) and QSLAM_highlight.mp4 (<= 2 min), all
# pictures only (no words), indexed in VIDEOS.md.
set -euo pipefail
cd "$(dirname "$0")"
WS=$PWD
HELPER=src/sih_isaac/sih_isaac/mission_run.py

RECORD=1
SIM_ARGS=()
for arg in "$@"; do
  case $arg in
    --no-record) RECORD=0 ;;
    --gui)       ;;                      # default when not recording
    *)           SIM_ARGS+=("$arg") ;;
  esac
done

if [ ! -f src/sih_isaac/usd/sih26126_world.usd ]; then
  echo "[setup] generating world + GeoTIFF + USD (one-off, ~3 min)"
  PYTHONPATH="$WS/src/sih_sim:$WS/src/sih_demo:$WS/src/sih_isaac" \
    .demoenv/bin/python src/sih_isaac/sih_isaac/world.py
  (cd /isaacsim && ./python.sh "$WS/src/sih_isaac/scripts/build_scene.py")
fi

if [ "$RECORD" = 1 ]; then
  RUN=$(python3 "$HELPER" new)
  FF=$(python3 "$HELPER" ffmpeg)
  SIM_ARGS+=(--headless --record "$RUN" --ffmpeg "$FF" --rec-size 1920x1080)
  echo $$ > "$RUN/sim.pid"          # our process group; finish_mission.sh signals it
  echo "[run] recording this mission to $RUN"
  echo "[run] next: ros2 launch sih_isaac nav.launch.py   (terminal 2)"
  # keep a copy of this terminal alongside the videos
  # tee ignores SIGINT so a Ctrl-C here cannot cut the log before the sim
  # prints "recordings finalized" (finish_mission.sh waits for that line)
  exec > >(trap '' INT; exec tee -a "$RUN/sim.log") 2>&1
else
  echo "[run] not recording (--no-record)"
fi

set +u
source /opt/ros/jazzy/setup.bash
set -u
cd /isaacsim && exec ./python.sh "$WS/src/sih_isaac/scripts/run_sim.py" "${SIM_ARGS[@]}"
