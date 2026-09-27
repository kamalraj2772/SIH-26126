# SIH26126 — GPS-denied UGV navigation (demo prototype)

One command produces a narrated dashboard video showing the whole chain running
end to end:

```bash
./run_demo.sh                 # ~80 s video at 2x real time
./run_demo.sh --speed 1       # real time, ~2.5 min
./run_demo.sh --speed 3       # ~55 s
```

```bash
./run_demo.sh --live          # interactive 3D window instead of a video
```

`--live` opens MuJoCo's viewer and runs the identical estimation and control
path — only the output differs. Drag to orbit, scroll to zoom, double-click the
rover then Ctrl+drag to follow it. Needs a display; the video path is headless
and works over SSH.

The video plays at 2x real time by default (stated in the header bar); physics
and control always run at full rate, only dashboard frames are decimated.

Output lands in `src/sih_demo/output/`:

| file | what it is |
|---|---|
| `sih26126_demo.mp4` | the dashboard video — intro card, live run, results card (~80 s, 20 MB) |
| `demo_summary.json` | every measured number quoted in the video |
| `telemetry.json` | per-control-step log (pose, drift, clearance, commands) |
| `prior_dem_utm44n.tif` | the prior DEM, a real UTM-44N GeoTIFF — open it in QGIS |
| `scene.xml` | the generated MuJoCo scene |

## What the demo actually demonstrates

```
operator's UTM goal ──pyproj──► map frame ──┐
surveyed site datum ────────────────────────┤
                                            ▼
prior GeoTIFF (EPSG:32644) ──slope──► A* terrain route
                                            │
ZED 2i RGB-D ──ORB + PnP-RANSAC──► pose ────┤
             └──depth──► local costmap ─────▼
                                     DWA ──► wheel velocities
```

**No GNSS anywhere in the control path.** There is no `NavSatFix`, no
`navsat_transform`, no `gps_*` anything. The only geodetic input is a *surveyed
site datum* — one lat/lon fixed offline, like a benchmark pillar on a test
range. The vehicle never measures its own lat/lon; it estimates its pose in the
map frame with VSLAM and converts that to UTM only for reporting.

### 1. UTM conversion — `geo.py`
pyproj, WGS-84 → EPSG:32644 (UTM zone 44N). The operator issues the goal as an
easting/northing; it is converted into the local ENU map frame against the
surveyed anchor, and the round trip is measured and reported (0.000 mm).

### 2. GeoTIFF — `geo.py`
A north-up float32 DEM is written with rasterio, correctly georeferenced
(CRS + affine transform), then **read back off disk**. The planner only ever
sees the file, so the GeoTIFF path is genuinely exercised rather than
decorative. It supplies two things: the A* traversability map, and the ground
reference used to decide whether a depth return is an obstacle or terrain.

### 3. VSLAM — `vslam.py`
**RTAB-Map is the production SLAM in the ROS 2 stack (Phase 2).** This demo runs
an equivalent stereo front-end in-process rather than RTAB-Map itself, so the
whole mission fits in one script with no ROS graph to bring up — the video
labels it as "RTAB-Map class stereo SLAM" for exactly that reason. If a judge
asks, that is the honest answer: same class of algorithm, not the same package.

It is a real front-end, not a replay of ground truth:
ORB corners on the ZED left image → back-projected to 3D landmarks with the
depth map → matched to the next frame (Hamming + Lowe ratio) → camera pose from
`solvePnPRansac`, refined on inliers → chained across keyframes. Initialised
from the surveyed deployment pose; nothing else enters it.

### 4. Collision-free navigation — `planner.py`
The obstacles in the corridor are **absent from the prior GeoTIFF**. They exist
only in the depth stream, so every deviation from the global route is the local
planner reacting to something the ZED just discovered. DWA samples (v, ω),
rolls each out, and rejects any trajectory that puts the footprint into an
occupied cell or into prior-map terrain hazards.

## Honest notes on the model

These are deliberate, documented approximations — worth stating rather than
glossing over:

* **Drivetrain.** Real tyres grip far harder along the axle than across it, and
  that asymmetry is what lets a skid-steer pivot. MuJoCo's contact friction is
  isotropic in the tangent plane, so four equally-gripping wheels give a lateral
  slip moment (arm `√(a²+b²)`) that always exceeds the drive moment (arm `b`) —
  the vehicle physically cannot yaw. The front pair are therefore modelled as
  low-friction rollers, and a measured slip-compensation gain (`SKID_YAW_GAIN`,
  3.5) maps requested body yaw rate onto wheel-speed difference. Commanding
  0.40 rad/s yields 0.41 rad/s achieved; without it, 0.08 rad/s.
* **Depth noise is modelled, not idealised.** σ = z²·σ_d/(f·B), which is ≈0.5 m
  at 10 m for the ZED 2i's 120 mm baseline. Obstacle evidence is therefore only
  taken from returns inside 8 m, with a detection threshold that tracks 2.5σ.
* **VSLAM has no loop closure and no bundle adjustment**, so drift accumulates
  monotonically. The reported figure is the real one. RTAB-Map's loop closure
  and graph optimisation would bound it; this front-end has neither.
* **Run-to-run variation.** The scene, terrain, obstacles and route are fully
  deterministic from the seed. `solvePnPRansac` is not bit-exact across runs, so
  final drift has varied between roughly 1 % and 3 % of path on the same seed.
  Quote the number from the run you actually show.
* Ground truth is recorded to score the run and is **never fed back** into
  estimation, mapping or control.

## Relationship to the ROS 2 stack

The terrain is `sih_sim.terrain` — the same analytic surface the Gazebo world
generator rasterises — so the demo and the ROS 2 stack describe one world.
MuJoCo is used here because it steps and renders far faster than gz-sim, which
is what makes a full 130 m mission reproducible in a couple of minutes.


## Result of the committed run (seed 42)

| metric | value |
|---|---|
| goal reached | yes — final error **1.91 m** against the commanded UTM goal |
| path travelled | 121.5 m in 127 s |
| rover–obstacle contacts | **0**, across 21 unsurveyed obstacles |
| minimum true clearance | 0.58 m vs 0.55 m footprint (0.75 m desired margin) |
| VSLAM drift | 1.20 m = **0.98 % of path**, 0 tracking losses, 259 keyframes |
| UTM round trip | 0.000 mm |

`collision_free` is true (never touched anything); `safety_margin_maintained` is
false (the 0.75 m comfort margin was eroded to 0.58 m at the tightest gate).
The summary reports those separately rather than collapsing them.
