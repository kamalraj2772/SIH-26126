"""The recording run directory shared by the three mission terminals.

Terminal 1 (run_isaac.sh) creates mission_recordings/report_HHMM, drops its
process-group id in sim.pid and points mission_recordings/.current_run at it.
Terminal 2 (nav.launch.py) and terminal 3 (utm_goal.py) read that pointer and
attach their recorders and their report to the same directory.

The pointer is only honoured while terminal 1's process is actually alive, so a
stale pointer from a crashed or finished run never captures a later session.
"""
import datetime
import os
import pathlib
import shutil

WS = pathlib.Path(os.environ.get("SIH_WS", "/home/qbotix-rover/sih_ws"))
RECORDINGS = WS / "mission_recordings"
POINTER = RECORDINGS / ".current_run"

# no system ffmpeg on this box; imageio-ffmpeg ships a static build in .demoenv
FFMPEG_DIR = WS / ".demoenv/lib/python3.12/site-packages/imageio_ffmpeg/binaries"


def ffmpeg_bin() -> str:
    for cand in sorted(FFMPEG_DIR.glob("ffmpeg-linux-*")):
        if os.access(cand, os.X_OK):
            return str(cand)
    return shutil.which("ffmpeg") or "ffmpeg"


def pid_alive(pid: int) -> bool:
    if pid <= 0:
        return False
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True          # exists, just not ours to signal
    return True


def read_pid(run_dir: pathlib.Path, name: str) -> int | None:
    try:
        return int((run_dir / name).read_text().strip())
    except (OSError, ValueError):
        return None


def active_run() -> pathlib.Path | None:
    """The run directory of the currently recording sim, or None."""
    try:
        run = pathlib.Path(POINTER.read_text().strip())
    except OSError:
        return None
    if not run.is_dir():
        return None
    pid = read_pid(run, "sim.pid")
    if pid is None or not pid_alive(pid):
        return None          # stale pointer: that sim is gone
    return run


def new_run_dir(stamp: str | None = None) -> pathlib.Path:
    """Create and return mission_recordings/report_HHMM (suffixed if taken)."""
    stamp = stamp or datetime.datetime.now().strftime("%H%M")
    run = RECORDINGS / f"report_{stamp}"
    n = 2
    while run.exists():
        run = RECORDINGS / f"report_{stamp}_{n}"
        n += 1
    run.mkdir(parents=True)
    return run


def clear_pointer(run: pathlib.Path | None = None) -> None:
    """Drop the pointer, unless it has already moved on to a newer run."""
    try:
        if run is None or POINTER.read_text().strip() == str(run):
            POINTER.unlink()
    except OSError:
        pass


if __name__ == "__main__":       # tiny CLI so the shell scripts share this logic
    import sys
    cmd = sys.argv[1] if len(sys.argv) > 1 else "active"
    if cmd == "new":
        run = new_run_dir()
        POINTER.parent.mkdir(parents=True, exist_ok=True)
        POINTER.write_text(str(run) + "\n")
        print(run)
    elif cmd == "active":
        run = active_run()
        print(run or "", end="\n" if run else "")
        sys.exit(0 if run else 1)
    elif cmd == "ffmpeg":
        print(ffmpeg_bin())
    elif cmd == "clear":
        clear_pointer(pathlib.Path(sys.argv[2]) if len(sys.argv) > 2 else None)
    else:
        sys.exit(f"unknown command {cmd}")
