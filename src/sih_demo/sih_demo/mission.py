"""Mission constants for the SIH26126 demo run.

The goal is authored the way an operator would author it: as a UTM easting /
northing on the site grid. The rover converts it to the map frame at start-up
(sih_demo.geo) and never sees a lat/lon again.
"""
from __future__ import annotations

SEED = 42

# Start pose in the map frame (the rover's known deployment point -- surveyed,
# not measured by any receiver).
START_XY = (-62.0, 16.0)
START_YAW_DEG = 8.0

# The commanded goal, in map-frame metres. Converted to UTM in run_demo so the
# mission brief can show the operator-facing easting/northing.
GOAL_XY = (56.0, 34.0)
GOAL_TOLERANCE_M = 2.5

# Vehicle limits (skid-steer UGV).
MAX_LINEAR_MPS = 1.0
MAX_ANGULAR_RPS = 0.55
WHEEL_RADIUS = 0.16
# Skid-steer slip compensation. A tracked/skid platform loses most of its
# commanded yaw to tyre scrub, so the wheel-speed difference has to be scaled
# up to realise the requested body yaw rate. Measured open-loop on this model:
# without it, 0.40 rad/s commanded yields 0.08 rad/s achieved.
SKID_YAW_GAIN = 3.5
TRACK_WIDTH = 0.62

# Safety envelope used for the collision-free claim.
ROBOT_RADIUS_M = 0.55       # circumscribed radius of the chassis
SAFETY_MARGIN_M = 0.20      # required clearance beyond the footprint

# Terrain traversability, evaluated on the prior GeoTIFF.
MAX_TRAVERSABLE_SLOPE_DEG = 15.0

CONTROL_HZ = 20.0
# Dashboard frames are composed every Nth control step and played back at
# CONTROL_HZ, so the video runs at Nx real time. The header states the factor.
VIDEO_SPEEDUP = 2
SIM_DT = 0.004              # MuJoCo integrator step
MAX_MISSION_S = 300.0

# --- ZED 2i, HD720 ----------------------------------------------------------
ZED_WIDTH_FULL = 1280
ZED_HEIGHT_FULL = 720
ZED_HFOV_DEG = 102.0
ZED_BASELINE_M = 0.120          # ZED 2i stereo baseline
ZED_MIN_DEPTH_M = 0.30
ZED_MAX_DEPTH_M = 20.0
ZED_FPS = 20.0

# Rendered at half scale for speed; intrinsics are scaled to match, so the
# VSLAM and depth maths stay consistent with the real HD720 geometry.
ZED_RENDER_SCALE = 0.5
ZED_WIDTH = int(ZED_WIDTH_FULL * ZED_RENDER_SCALE)
ZED_HEIGHT = int(ZED_HEIGHT_FULL * ZED_RENDER_SCALE)

# Camera mount on the chassis (body frame: x fwd, y left, z up).
ZED_MOUNT_XYZ = (0.34, 0.0, 0.52)
ZED_PITCH_DEG = -10.0           # tilted down to see near terrain


def corridor_obstacles(route, seed: int) -> list[tuple[float, float, float, str]]:
    """Scatter boulders and trees *along the planned route*.

    These are the things the prior survey does not know about. Each station is
    one of two deliberately-posed problems, spaced far enough apart that the
    manoeuvres do not overlap:

      * blocker -- a single obstacle on the lane centreline. The only way past
        is a lateral detour off the surveyed route.
      * gate    -- a pair on opposite shoulders, offset so the free gap between
        their surfaces is comfortably wider than the vehicle but narrower than
        the route corridor, so the planner has to aim at it.

    Gaps are sized against the vehicle's own safety envelope rather than picked
    by eye, so the course is always solvable and the collision-free result
    means something.
    """
    import numpy as np

    rng = np.random.RandomState(seed + 4242)
    seg = np.linalg.norm(np.diff(route, axis=0), axis=1)
    s_along = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(s_along[-1])
    envelope = ROBOT_RADIUS_M + SAFETY_MARGIN_M     # 0.75 m

    out: list[tuple[float, float, float, str]] = []
    s = 12.0
    flip = 1.0
    while s < total - 10.0:
        i = int(np.searchsorted(s_along, s))
        i = min(max(i, 1), len(route) - 1)
        p = route[i]
        tangent = route[min(i + 2, len(route) - 1)] - route[max(i - 2, 0)]
        nrm = np.linalg.norm(tangent)
        tangent = tangent / nrm if nrm > 1e-6 else np.array([1.0, 0.0])
        normal = np.array([-tangent[1], tangent[0]])

        def place(lateral, radius, kind, along=0.0):
            q = p + normal * lateral + tangent * along
            out.append((round(float(q[0]), 2), round(float(q[1]), 2),
                        float(radius), kind))

        if rng.rand() < 0.45:
            # Blocker: sits on the centreline, must be driven around.
            kind = "tree" if rng.rand() < 0.35 else "boulder"
            r = float(rng.uniform(0.34, 0.46) if kind == "tree"
                      else rng.uniform(0.50, 0.75))
            place(float(rng.uniform(-0.35, 0.35)), r, kind)
        else:
            # Gate: half-gap = envelope + vehicle half-width + slack.
            r_l = float(rng.uniform(0.45, 0.75))
            r_r = float(rng.uniform(0.45, 0.75))
            slack = float(rng.uniform(0.60, 1.05))
            place(-(r_l + envelope + slack), r_l, "boulder")
            place(+(r_r + envelope + slack), r_r,
                  "tree" if rng.rand() < 0.3 else "boulder",
                  along=float(rng.uniform(-0.6, 0.6)))

        flip *= -1.0
        s += float(rng.uniform(7.0, 9.5))
    return out
