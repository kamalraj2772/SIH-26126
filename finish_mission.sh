#!/usr/bin/env bash
# SIH26126: stop a recording mission cleanly and build its report.
#
#   ./finish_mission.sh            the currently active run
#   ./finish_mission.sh RUN_DIR    a specific mission_recordings/report_HHMM
#
# utm_goal.py calls this by itself when the mission ends, so the three-terminal
# flow needs no fourth command. Run it by hand after --no-finish, or to rebuild
# a report whose generation failed.
#
# Ordering matters: every recorder is stopped and allowed to finalize its own
# file BEFORE the sim and nav stack are torn down, so no mp4 is ever truncated.
set -uo pipefail          # deliberately not -e: every stage runs even if one fails
cd "$(dirname "$0")"
WS=$PWD
HELPER=src/sih_isaac/sih_isaac/mission_run.py

RUN=${1:-$(python3 "$HELPER" active || true)}
if [ -z "$RUN" ] || [ ! -d "$RUN" ]; then
  echo "[finish] no run to finish. usage: $0 [mission_recordings/report_HHMM]" >&2
  exit 2
fi
RUN=$(realpath "$RUN")
FF=$(python3 "$HELPER" ffmpeg)

# Everything below also goes to <run>/finish.log (tee ignores SIGINT).
exec > >(trap '' INT; exec tee -a "$RUN/finish.log") 2>&1

# Interrupting this halfway is how a run loses its videos, so the first Ctrl-C
# only warns; a second one aborts. The video/report builders run in their own
# session and are not reached by Ctrl-C at all.
INTS=0
on_int() {
  INTS=$((INTS + 1))
  if [ "$INTS" -ge 2 ]; then
    echo "[finish] aborted. Resume any time with: $0 $RUN"
    exit 130
  fi
  echo "[finish] still finishing -- recorders are being closed and the videos"
  echo "[finish] built. Please wait. (Ctrl-C again aborts between steps.)"
}
trap on_int INT
# run a builder out of Ctrl-C's reach: own session, and the waiting wrapper
# ignores SIGINT so it cannot report a failure while the build carries on
shielded() { (trap '' INT; exec setsid --wait "$@"); }

echo "[finish] ============================================================"
echo "[finish] mission over. Closing recorders and building the videos and"
echo "[finish] report: this takes about 3-5 minutes. Do NOT press Ctrl-C or"
echo "[finish] close the terminals -- everything shuts down by itself."
echo "[finish] ============================================================"
echo "[finish] run: $RUN"

alive() { kill -0 "$1" 2>/dev/null; }

# Stop one recorder by its pid file and wait for it to finish its output.
# mission_logger.py (python) flushes its CSV on SIGINT. The screen-capture ffmpeg
# does not act on SIGINT or SIGTERM at all while grabbing x11 -- measured, 90 s
# and still recording -- so it is escalated to SIGKILL, which is safe only
# because nav.launch.py writes that capture as a fragmented mp4 (every fragment
# self-indexing, so the file plays back however the process dies).
stop_recorder() {
  local file=$RUN/$1 name=$2 limit=${3:-15} pid sig
  [ -f "$file" ] || return 0
  pid=$(cat "$file" 2>/dev/null) || return 0
  alive "$pid" || return 0
  echo "[finish] stopping $name (pid $pid)"
  for sig in INT TERM; do
    kill "-$sig" "$pid" 2>/dev/null || return 0
    for _ in $(seq 1 "$limit"); do
      alive "$pid" || { echo "[finish] $name finalized"; return 0; }
      sleep 1
    done
    echo "[finish] WARNING: $name ignored SIG$sig after ${limit}s"
  done
  echo "[finish] $name does not stop on signals; killing it (its mp4 is fragmented, so it stays playable)"
  kill -KILL "$pid" 2>/dev/null
  for _ in $(seq 1 10); do alive "$pid" || return 0; sleep 1; done
}

# SIGINT a whole terminal: its process group, since python.sh does not exec
signal_terminal() {
  local pid=$1 sig=$2
  kill "-$sig" -- "-$pid" 2>/dev/null || kill "-$sig" "$pid" 2>/dev/null
}

# 1. the recorders that own files, in the order they stop cleanest.
#    viz_recorder.py drains its render queues and closes three mp4s on SIGINT;
#    it is paced on /clock, so it must stop before the sim does.
stop_recorder logger.pid       "telemetry logger" 15
stop_recorder rviz_ffmpeg.pid  "RViz screen capture" 4   # ignores INT/TERM; fragmented
stop_recorder viz.pid          "costmap/EKF/SLAM-graph recorder" 90

# 2. the sim: run_sim.py closes its camera ffmpeg stdins in its finally block
#    and prints "recordings finalized" once the mp4s are safe on disk
SIM_PID=$(cat "$RUN/sim.pid" 2>/dev/null || true)
if [ -n "${SIM_PID:-}" ] && alive "$SIM_PID"; then
  echo "[finish] stopping sim (pid $SIM_PID) -- finalizing the camera mp4s"
  signal_terminal "$SIM_PID" INT
  for _ in $(seq 1 90); do
    grep -q "recordings finalized" "$RUN/sim.log" 2>/dev/null && break
    alive "$SIM_PID" || break
    sleep 2
  done
  if grep -q "recordings finalized" "$RUN/sim.log" 2>/dev/null; then
    echo "[finish] camera recordings finalized"
  else
    echo "[finish] WARNING: sim did not report finalizing; videos may be short"
  fi
  # Isaac's app.close() can hang on its crash handler; the mp4s are already safe
  for _ in $(seq 1 30); do alive "$SIM_PID" || break; sleep 2; done
  alive "$SIM_PID" && { echo "[finish] sim slow to exit, escalating"; signal_terminal "$SIM_PID" TERM; }
else
  echo "[finish] sim already stopped"
fi
# last resort: a sim whose process group we could not reach
pgrep -f "[r]un_sim.py" >/dev/null && { echo "[finish] reaping stray run_sim.py"; pkill -INT -f "[r]un_sim.py"; sleep 5; }

# 3. the nav stack and RViz
NAV_PID=$(cat "$RUN/nav.pid" 2>/dev/null || true)
if [ -n "${NAV_PID:-}" ] && alive "$NAV_PID"; then
  echo "[finish] stopping nav stack (pid $NAV_PID)"
  signal_terminal "$NAV_PID" INT
  for _ in $(seq 1 15); do alive "$NAV_PID" || break; sleep 2; done
  # ros2 launch can hang on SIGINT when a lifecycle node is stuck
  alive "$NAV_PID" && { echo "[finish] nav slow to exit, escalating"; signal_terminal "$NAV_PID" TERM; }
fi

# 4. keep terminal 2's node output with the run (it goes to ~/.ros/log, not here)
NAVLOG=$(ls -dt "$HOME"/.ros/log/*/ 2>/dev/null | head -1)
if [ -n "${NAVLOG:-}" ] && [ ! -e "$RUN/nav_ros_log" ]; then
  cp -r "$NAVLOG" "$RUN/nav_ros_log" 2>/dev/null &&
    echo "[finish] copied nav logs -> $RUN/nav_ros_log"
fi

# 5. the deliverables: the GeoTIFF/DEM/UTM sequence, the <=2 min highlight
#    reel (no words in any video), VIDEOS.md, and the PDF report
echo "[finish] building mission videos"
shielded .demoenv/bin/python src/sih_isaac/scripts/make_mission_videos.py "$RUN" --ffmpeg "$FF" ||
  echo "[finish] some videos failed; re-run make_mission_videos.py '$RUN'"
echo "[finish] building mission report"
shielded .demoenv/bin/python src/sih_isaac/scripts/make_mission_report.py "$RUN" --ffmpeg "$FF" ||
  echo "[finish] report failed; re-run make_mission_report.py '$RUN'"

python3 "$HELPER" clear "$RUN"
echo "[finish] done: $RUN"
ls -lh "$RUN"
