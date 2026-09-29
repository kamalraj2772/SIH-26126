# SIH26126 — GPS-denied UGV navigation in Isaac Sim 6.0

The same GPS-denied mission as the MuJoCo demo (`src/sih_demo`), rebuilt as a
full ROS 2 stack on Isaac Sim 6.0.1: an operator hands the rover a **UTM
easting/northing**, and it drives there across unstructured terrain — slopes, a
tunnel, drums, logs, boulders, trees, rocks and **humans** (two standing, two
walking patrols that cross the corridor) — with **no GNSS anywhere** in the
system.

```
operator UTM goal ──anchor subtraction──► map-frame goal          (utm_goal.py)
prior GeoTIFF (EPSG:32644) ──slope──► occupancy map ──► Smac A* global plan
RTX lidar ──► rolling costmap ──► MPPI local planner ──► /cmd_vel ──► wheels
VIO-grade odometry + IMU ──EKF──► odom->base_link  (seeded at surveyed pose)
```

Measured on the committed configuration (seed 42, goal E 403562.52
N 1450024.12): **goal reached through the tunnel**, initial distance 123.5 m,
final ground-truth distance to goal 0.23 m, localization error at the end
0.98 m, 1 recovery, ~460 s sim time.

## Run it (three terminals) — every run is recorded

```bash
# 1. simulation. Creates mission_recordings/report_<HHMM>/ and records the
#    three follow cameras and the rover's own camera at 1080p. Headless on
#    purpose (see Known issues).
cd ~/sih_ws && ./run_isaac.sh

# 2. after "[sim] running" appears: navigation stack + RViz. Attaches a
#    full-screen capture (RViz and all three terminals), the telemetry log and
#    the costmap / EKF / SLAM-graph video recorder to the same run directory.
source /opt/ros/jazzy/setup.bash && source ~/sih_ws/install/setup.bash
ros2 launch sih_isaac nav.launch.py

# 3. give it somewhere to go -- lat/lon, or UTM, or "2D Goal Pose" in RViz.
#    When the mission ends this stops the recorders, shuts terminals 1 and 2
#    down and builds the mission videos, the highlight reel and the PDF report.
ros2 run sih_isaac utm_goal.py --lat 13.1150092 --lon 80.1103155
ros2 run sih_isaac utm_goal.py -e 403562.52 -n 1450024.12      # equivalent
```

Start the nav stack shortly after the sim is up (see Known issues).

Each run lands in its own `mission_recordings/report_<HHMM>` (suffixed `_2` if
that minute is taken), holding the videos below, `mission_log.csv`, the three
terminals' logs and `SIH26126_Mission_Report.pdf`. Opting out:

```bash
./run_isaac.sh --no-record                         # terminal 1, GUI, no files
ros2 launch sih_isaac nav.launch.py record:=none   # terminal 2, no capture
ros2 run sih_isaac utm_goal.py --lat .. --lon ..  --no-finish   # leave it up
./finish_mission.sh [mission_recordings/report_HHMM]            # finish later
```

`./record_mission.sh` still runs the whole thing unattended as a single command.

## Mission videos — every run, pictures only (no words)

| file | what it shows | made by |
|---|---|---|
| `QSLAM_highlight.mp4` | the final reel, **at most 2 min**: input, drive, inside the stack, arrival | `make_highlight.py` |
| `isaac_onboard.mp4` | the rover's own mast camera (ZED), 1080p | `run_sim.py` |
| `isaac_behind/side/top.mp4` | chase, side and bird's-eye follow cameras, 1080p | `run_sim.py` |
| `rviz.mp4` | full-screen capture of RViz and the terminals | `nav.launch.py` |
| `costmap.mp4` | global costmap (GeoTIFF slope + lidar + inflation), Smac A* route, local costmap with MPPI candidates | `viz_recorder.py` (live) |
| `ekf.mp4` | EKF estimate vs ground truth vs wheel-only dead reckoning, error and heading histories | `viz_recorder.py` (live) |
| `slam_graph.mp4` | map built from lidar with the pose graph, and the live ROS 2 node/topic graph whose links light up with traffic | `viz_recorder.py` (live) |
| `geo_pipeline.mp4` | GeoTIFF → DEM → slope → occupancy grid → UTM 44N ↔ map → this run's goal and route | `make_geo_video.py` |

`VIDEOS.md` in the run directory lists them with their lengths. No video
carries words — no titles, captions, labels, legends or text cards. The camera
and RViz captures are kept exactly as recorded; the drawn videos show only
their maps, plots and graph, with numeric tick marks for scale
(`video.strip_words()` enforces this).

Timebase: the Isaac captures and the three live videos run on sim time
(`viz_start.txt` holds the sim time at which the live videos start), so one
mission timestamp means the same moment in all of them. `rviz.mp4` is
wall-clock; `timebase.csv` and `rviz_start.txt` let the tools convert.

Rebuild a run's videos:

```bash
.demoenv/bin/python src/sih_isaac/scripts/make_mission_videos.py mission_recordings/report_HHMM
.demoenv/bin/python src/sih_isaac/scripts/make_highlight.py mission_recordings/report_HHMM --max-seconds 90
```

The onboard camera is a fourth 1080p render product; if a run is frame-rate
bound, `./run_isaac.sh --no-onboard` drops it (the reel then uses RViz in its
four-way grid instead).

Lat/lon is converted by `sih_isaac/geodesy.py`, a dependency-free
transverse-Mercator series (pyproj is not importable from `ros2 run`, and
keeping it out means nothing at runtime imports it). It agrees with pyproj to
under a millimetre worldwide and reproduces the baked site anchor exactly;
`tests/test_geodesy.py` checks both. It is still arithmetic on numbers typed at
a keyboard -- no receiver, no `NavSatFix`, nothing measures a position.

## What you see

- **Isaac Sim window** — the photoreal world: terrain from the GeoTIFF, the
  berm with its lit tunnel bore, an 8-degree drivable plateau, a 36-degree
  mound the planner must skirt, scattered obstacles, humans walking.
- **RViz window** — the robot's mind: GeoTIFF prior map, global costmap,
  rolling local costmap, the blue **Smac A\* global plan**, the orange **MPPI
  local plan** and its candidate trajectories, the red lidar scan, the camera
  feed, EKF odometry arrows and the TF tree.
- **utm_goal.py** — mission progress in both frames (map metres and UTM
  easting/northing), distance remaining, recovery count, final result.

## The GeoTIFF is the single source of truth

`sih_isaac/world.py` writes `generated/prior_dem_utm44n.tif` (float32 DEM,
EPSG:32644, 0.59 m/px — open it in QGIS) and builds **both** the Isaac terrain
mesh and the Nav2 static map (`generated/map.pgm`, slope > 15° = blocked) from
that same file, so the simulated world and the planner's prior agree by
construction. Corridor obstacles and humans are deliberately absent from the
prior — the rover must find them with the lidar.

The UTM↔map conversion is a pure translation against a **surveyed site datum**
(the same Avadi anchor as the MuJoCo demo), computed offline by pyproj at
world-generation time and baked into `generated/world.json`. At runtime nothing
imports pyproj (an operator's lat/lon goes through `geodesy.py` instead), and
nothing anywhere subscribes to a fix: grep for `NavSatFix` or `gps` and you
will find only this sentence.

## TF ownership (exactly one publisher per edge)

| edge | owner |
|---|---|
| `map -> odom` | static identity (default) or slam_toolbox (`slam:=true`, experimental) |
| `odom -> base_link` | robot_localization EKF (odometry velocities + IMU) |
| `base_link -> lidar_link` | one static_transform_publisher |
| `base_link -> everything else` | robot_state_publisher (URDF) |

Isaac Sim publishes **no TF at all**. The static `map -> odom` identity is
legitimate because the EKF is seeded with the surveyed deployment pose
(`initial_state` in `config/ekf.yaml`), so odom coincides with map at t0.
`/ground_truth` is published for scoring only; nothing in the nav stack
subscribes to it.

## Localization honesty

The EKF's inputs are body-frame vx/vy and the IMU's yaw + yaw rate, all
synthesized in `run_sim.py` from the simulator's rigid-body state — i.e.
**idealized VIO/AHRS-grade sensors**, not wheel encoders. This was a measured
decision, not convenience: wheel odometry on this skid-steer platform was
tried first and failed three ways (yaw from wheel-speed difference is
meaningless under scrub; distance over-counts ~30 % on slopes from slip;
lateral crabbing on side slopes is invisible to encoders), producing 10–70 m
localization errors and aborted missions. The quantities fused are exactly
what `isaac_ros_visual_slam` (cuVSLAM, now installed) and a magnetometer AHRS
measure on the real platform; wiring cuVSLAM's odometry in place of the
synthesized topic is the designated next step, and the wheel/IMU raw signals
are still published for comparison.

## Files

```
scripts/build_scene.py   world.json + DEM -> sih26126_world.usd (run once)
scripts/run_sim.py       loads the USD, ROS 2 bridge graphs, drives wheels,
                         wheel odometry, IMU, ground truth, human patrols
scripts/calibrate_yaw.py measures achieved-vs-commanded yaw/speed ratios
sih_isaac/world.py       DEM + GeoTIFF + occupancy map + obstacle/human layout
sih_isaac/utm_goal.py    lat/lon or UTM goal CLI (Nav2 NavigateToPose client);
                         finishes the recording run when the mission ends
sih_isaac/geodesy.py     WGS84 -> UTM, dependency-free
sih_isaac/mission_run.py the report_HHMM run dir the three terminals share
sih_isaac/video.py       shared look, no-words enforcement, ffmpeg pipe
scripts/viz_recorder.py  live costmap / EKF / SLAM-graph videos (sim-time paced)
scripts/make_geo_video.py      GeoTIFF -> DEM -> UTM picture sequence
scripts/make_highlight.py      the <= 2 min highlight reel
scripts/make_mission_videos.py all of the above, in order, + VIDEOS.md
finish_mission.sh        stop recorders, build mission videos + PDF report
launch/nav.launch.py     EKF, map anchor, map_server, Nav2, RViz
config/                  ekf.yaml, nav2_params.yaml, slam_toolbox.yaml, sih.rviz
```

Regenerate the world (new seed/layout): edit `sih_isaac/world.py`, then
`.demoenv/bin/python src/sih_isaac/sih_isaac/world.py` (with PYTHONPATH set as
in `run_isaac.sh`) and re-run `build_scene.py`.

## Skid-steer calibration (measured on this stack)

PhysX makes a long-wheelbase skid-steer scrub exactly like the real platform:
commanded yaw achieved only ~6 % with stock friction. Fixes applied, measured
with `calibrate_yaw.py`: wheel physics material mu 0.45/0.40, drive damping
5000 / max force 900, `SKID_YAW_GAIN = 6.5` (spin ratio 0.398 at gain 2.6),
rolling efficiency 0.95 folded into the wheel odometry.

## Known issues

- **slam_toolbox** (`slam:=true`) is experimental here: the async node has a
  nondeterministic activation race under sim time (use the sync node, as the
  launch does), and even then karto's corrections degraded the pose on this
  terrain (map corrupted after ~50 m, suspected scan-vs-odometry geometry
  mismatch). cuVSLAM is the intended replacement, not karto tuning.
- **GUI + RViz on one 4090** drops the sim to ~0.33x real time and Nav2's TF
  timing then starves (aborts). For demos, either run the sim headless with
  RViz, or expect slower missions in full-GUI mode; every nav2 transform
  tolerance is already at 1.0 s.
- Isaac Sim may print a crash-handler backtrace when its window is closed;
  the run's data is already on disk by then.
