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

## Run it (two terminals)

```bash
# 1. simulation (Isaac Sim GUI window; add --headless for no window)
cd ~/sih_ws && ./run_isaac.sh

# 2. after "[sim] running" appears: navigation stack + RViz
source /opt/ros/jazzy/setup.bash && source ~/sih_ws/install/setup.bash
ros2 launch sih_isaac nav.launch.py

# 3. give it somewhere to go, as UTM (or click "2D Goal Pose" in RViz)
ros2 run sih_isaac utm_goal.py -e 403562.52 -n 1450024.12
```

Start the nav stack shortly after the sim is up (see Known issues).

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
imports pyproj, and nothing anywhere subscribes to a fix: grep for `NavSatFix`
or `gps` and you will find only this sentence.

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
sih_isaac/utm_goal.py    UTM goal CLI (Nav2 NavigateToPose client)
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
