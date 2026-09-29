#!/usr/bin/env bash
# SIH26126 recorded mission: headless sim writing four camera mp4s (three
# follow views + the rover's own camera), RViz screen capture, live costmap /
# EKF / SLAM-graph videos, telemetry CSV; then the GeoTIFF->UTM sequence,
# the <=2 min highlight reel (no words in any video) and a PDF report.
#   ./record_mission.sh [-e EASTING] [-n NORTHING] [output_dir]
# Defaults: the committed goal, mission_recordings/run_<timestamp>.
set -euo pipefail
cd "$(dirname "$0")"
WS=$PWD

E=403562.52
N=1450024.12
while getopts "e:n:" opt; do
  case $opt in
    e) E=$OPTARG ;;
    n) N=$OPTARG ;;
    *) exit 2 ;;
  esac
done
shift $((OPTIND - 1))
RUN=${1:-mission_recordings/run_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$RUN"
RUN=$(realpath "$RUN")
FF=$(.demoenv/bin/python -c "import imageio_ffmpeg; print(imageio_ffmpeg.get_ffmpeg_exe())")
DISP=${DISPLAY:-:1}

cleanup() {
  [ -n "${LOG_PID:-}" ] && kill -INT "$LOG_PID" 2>/dev/null || true
  [ -n "${REC_PID:-}" ] && kill -INT "$REC_PID" 2>/dev/null || true
  # the visualisation recorder drains its render queues before exiting, and it
  # is paced on /clock, so give it time to finish before the sim goes away
  if [ -n "${VIZ_PID:-}" ]; then
    kill -INT "$VIZ_PID" 2>/dev/null || true
    for _ in $(seq 1 90); do kill -0 "$VIZ_PID" 2>/dev/null || break; sleep 1; done
    kill -0 "$VIZ_PID" 2>/dev/null && kill -TERM "$VIZ_PID" 2>/dev/null || true
  fi
  sleep 2
  [ -n "${NAV_PID:-}" ] && kill -INT -- -"$NAV_PID" 2>/dev/null || true
  # SIGINT lets run_sim.py finalize the mp4 files before exiting
  [ -n "${SIM_PID:-}" ] && kill -INT -- -"$SIM_PID" 2>/dev/null || true
  for _ in $(seq 1 45); do
    kill -0 "$SIM_PID" 2>/dev/null || break
    sleep 2
  done
  kill -0 "${SIM_PID:-0}" 2>/dev/null && kill -TERM -- -"$SIM_PID" 2>/dev/null || true
  # ros2 launch can hang on SIGINT when a lifecycle node is stuck: escalate
  for _ in $(seq 1 10); do
    kill -0 "${NAV_PID:-0}" 2>/dev/null || break
    sleep 2
  done
  kill -0 "${NAV_PID:-0}" 2>/dev/null && kill -TERM -- -"$NAV_PID" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT

echo "[record] output: $RUN"
setsid bash -c "cd /isaacsim && source /opt/ros/jazzy/setup.bash && \
  exec ./python.sh '$WS/src/sih_isaac/scripts/run_sim.py' --headless \
  --record '$RUN' --ffmpeg '$FF' --rec-size 1920x1080" \
  > "$RUN/sim.log" 2>&1 &
SIM_PID=$!
until grep -q "\[sim\] running" "$RUN/sim.log" 2>/dev/null; do
  kill -0 "$SIM_PID" 2>/dev/null || { echo "[record] sim died, see $RUN/sim.log"; exit 1; }
  sleep 3
done
echo "[record] sim up (recording behind/side/top/onboard)"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

# Nav2's lifecycle bringup has a rare startup race (a change_state response
# times out and the manager waits forever at "Configuring map_server").
# It never self-heals; a relaunch always cures it, so retry automatically.
NAV_OK=""
for attempt in 1 2 3; do
  # record:=none -- this script starts its own capture and logger below
  setsid env DISPLAY="$DISP" ros2 launch sih_isaac nav.launch.py rviz:=true \
    record:=none \
    > "$RUN/nav.log" 2>&1 &
  NAV_PID=$!
  for _ in $(seq 1 30); do   # up to 90 s
    grep -q "Activating bt_navigator" "$RUN/nav.log" 2>/dev/null && { NAV_OK=1; break; }
    kill -0 "$NAV_PID" 2>/dev/null || break
    sleep 3
  done
  [ -n "$NAV_OK" ] && break
  echo "[record] nav bringup stuck (attempt $attempt), relaunching"
  kill -INT -- -"$NAV_PID" 2>/dev/null || true
  sleep 10
  kill -0 "$NAV_PID" 2>/dev/null && kill -TERM -- -"$NAV_PID" 2>/dev/null || true
  sleep 3
done
[ -n "$NAV_OK" ] || { echo "[record] nav failed to activate 3 times, see $RUN/nav.log"; exit 1; }
sleep 10
echo "[record] nav active"

# Capture the whole screen: it is natively 1080p, so the RViz recording is
# true 1920x1080 instead of an upscale of the smaller window rectangle.
SCREEN=$(DISPLAY="$DISP" xwininfo -root 2>/dev/null |
         grep -oE "geometry [0-9]+x[0-9]+" | grep -oE "[0-9]+x[0-9]+") || true
REC_PID=""
if [ -n "${SCREEN:-}" ]; then
  SW=$(( ${SCREEN%x*} / 2 * 2 )); SH=$(( ${SCREEN#*x} / 2 * 2 ))
  if [ "$SW" = "1920" ] && [ "$SH" = "1080" ]; then
    SCALE=()
  else   # normalise anything else to 1080p
    SCALE=(-vf "scale=1920:1080:flags=lanczos,format=yuv420p")
  fi
  date +%s.%N > "$RUN/rviz_start.txt"   # ties this wall-clock grab to sim time
  "$FF" -y -loglevel error -f x11grab -framerate 15 -video_size "${SW}x${SH}" \
    -i "${DISP}+0,0" "${SCALE[@]}" -c:v libx264 -preset veryfast -crf 21 \
    -pix_fmt yuv420p -g 30 -movflags +frag_keyframe+empty_moov \
    "$RUN/rviz.mp4" > "$RUN/rviz_ffmpeg.log" 2>&1 &
  REC_PID=$!
  echo "[record] rviz screen capture ${SW}x${SH} -> 1080p"
else
  echo "[record] WARNING: no X display on $DISP; skipping screen capture"
fi

python3 src/sih_isaac/scripts/mission_logger.py --out "$RUN/mission_log.csv" \
  > "$RUN/logger.log" 2>&1 &
LOG_PID=$!
python3 src/sih_isaac/scripts/viz_recorder.py --out "$RUN" \
  > "$RUN/viz.log" 2>&1 &
VIZ_PID=$!

echo "[record] mission: goal E $E  N $N"
ros2 run sih_isaac utm_goal.py -e "$E" -n "$N" 2>&1 | tee "$RUN/goal.log" || true
echo "[record] mission ended, shutting recorders down"

cleanup
trap - EXIT

.demoenv/bin/python src/sih_isaac/scripts/make_mission_videos.py "$RUN" \
  --ffmpeg "$FF" || echo "[record] some videos failed; re-run make_mission_videos.py"

.demoenv/bin/python src/sih_isaac/scripts/make_mission_report.py "$RUN" \
  --ffmpeg "$FF" || echo "[record] report generation failed; re-run make_mission_report.py"
echo "[record] done: $RUN"
ls -lh "$RUN"
