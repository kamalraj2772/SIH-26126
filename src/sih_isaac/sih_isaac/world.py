"""World pre-generation for the SIH26126 Isaac Sim mission.

Produces, into src/sih_isaac/generated/:
  dem.npy               257x257 float32 DEM, NORTH-UP (row 0 = +y), map-frame
  prior_dem_utm44n.tif  the same DEM as a georeferenced GeoTIFF (EPSG:32644)
  terrain_texture.png   colour texture for the terrain mesh
  map.pgm / map.yaml    slope-derived occupancy grid for the Nav2 static layer
  world.json            obstacles, humans, tunnel, spawn/goal, datum metadata

The GeoTIFF is the single source of truth: the Isaac terrain mesh, the Nav2
static map and the UTM goal conversion all derive from this one file, so the
simulated world and the planner's prior are guaranteed to agree.

GPS-denied by construction: the only geodetic input is the surveyed site datum
(sih_demo.geo.SITE_LAT/LON). Nothing here or downstream reads a receiver.
"""
from __future__ import annotations

import json
import pathlib

import numpy as np

from sih_demo import geo, scene
from sih_sim import terrain

GEN = pathlib.Path(__file__).resolve().parents[1] / "generated"

SEED = 42
N = 257
WORLD = float(terrain.WORLD_SIZE)          # 150 m
HALF = WORLD / 2.0

START_XY = (-62.0, 16.0)
START_YAW_DEG = 8.0
GOAL_XY = (56.0, 34.0)                     # default goal; operator overrides in UTM

MAX_SLOPE_DEG = 15.0

# --- Berm with a tunnel carved through it ------------------------------------
# A 3.2 m earth berm across the corridor; the only low-slope way past it is the
# carved passage at y=25, which build_scene.py roofs over into a real tunnel.
# A full wall across the site: the carved tunnel is the only way past.
BERM = dict(x0=2.0, x1=10.0, y0=-80.0, y1=80.0, height=3.2, edge=3.0)
TUNNEL = dict(y=25.0, half_width=2.8, edge=1.2,
              roof_z0=3.0, roof_z1=3.6, mouth_margin=1.5)

# --- A traversable 10 deg plateau on the route --------------------------------
PLATEAU = dict(cx=-30.0, cy=20.0, top_r=6.0, height=1.1, slope_deg=8.0)


def _smoothstep(e0, e1, x):
    t = np.clip((x - e0) / (e1 - e0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def _box_profile(v, v0, v1, edge):
    """1 inside [v0, v1], smooth rolloff over `edge` outside."""
    return _smoothstep(v0 - edge, v0, v) * (1.0 - _smoothstep(v1, v1 + edge, v))


def berm_height(x, y):
    b = BERM
    prof = (_box_profile(x, b["x0"], b["x1"], b["edge"])
            * _box_profile(y, b["y0"], b["y1"], b["edge"]))
    t = TUNNEL
    carve = _box_profile(y, t["y"] - t["half_width"], t["y"] + t["half_width"],
                         t["edge"])
    return b["height"] * prof * (1.0 - carve)


def plateau_height(x, y):
    p = PLATEAU
    r = np.hypot(x - p["cx"], y - p["cy"])
    run = p["height"] / np.tan(np.radians(p["slope_deg"]))
    return p["height"] * (1.0 - _smoothstep(p["top_r"], p["top_r"] + run, r))


def height_map_frame(x, y):
    """Full mission-terrain height at map-frame (x, y): base + berm + plateau."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    return (terrain.terrain_height(x, y, SEED)
            + berm_height(x, y) + plateau_height(x, y))


def build_dem(n: int = N) -> np.ndarray:
    """North-up DEM (row 0 = +y edge), matching the GeoTIFF convention."""
    coords = np.linspace(-HALF, HALF, n)
    xx, yy = np.meshgrid(coords, coords)
    return np.flipud(height_map_frame(xx, yy)).astype(np.float32)


def slope_deg_grid(dem_north_up: np.ndarray) -> np.ndarray:
    res = WORLD / (dem_north_up.shape[0] - 1)
    h = np.flipud(dem_north_up)              # back to bottom-up for d/dy sign
    dhdy, dhdx = np.gradient(h, res)
    return np.degrees(np.arctan(np.hypot(dhdx, dhdy)))


def occupancy(dem_north_up: np.ndarray) -> np.ndarray:
    """0 free / 100 occupied, north-up, from the slope threshold alone.

    Only terrain the prior survey knows about. Drums, humans and the rest are
    absent on purpose -- the rover must discover them with its own sensors.
    """
    blocked = np.flipud(slope_deg_grid(dem_north_up) > MAX_SLOPE_DEG)  # north-up
    return np.where(blocked, 100, 0).astype(np.uint8)


def _corridor_obstacles(rng) -> list[dict]:
    """Unsurveyed obstacles scattered in a band along the straight route."""
    sx, sy = START_XY
    gx, gy = GOAL_XY
    d = np.hypot(gx - sx, gy - sy)
    ux, uy = (gx - sx) / d, (gy - sy) / d       # along-route unit vector
    px, py = -uy, ux                            # perpendicular
    kinds = ["drum", "crate", "log", "boulder"]
    out = []
    n_placed = 0
    while n_placed < 14:
        s = rng.uniform(10.0, d - 10.0)
        t = rng.uniform(-6.0, 6.0)
        x, y = sx + ux * s + px * t, sy + uy * s + py * t
        # keep the tunnel bore and its approaches clear
        if (BERM["x0"] - 6.0 < x < BERM["x1"] + 6.0
                and abs(y - TUNNEL["y"]) < TUNNEL["half_width"] + 2.0):
            continue
        if np.hypot(x - PLATEAU["cx"], y - PLATEAU["cy"]) < 3.0:
            continue
        kind = kinds[n_placed % len(kinds)]
        r = dict(drum=0.30, crate=0.45, log=0.25, boulder=0.55)[kind]
        z = float(height_map_frame(x, y))
        out.append(dict(name=f"obs_{n_placed}", kind=kind,
                        x=round(float(x), 2), y=round(float(y), 2),
                        z=round(z, 3), radius=r,
                        yaw=round(float(rng.uniform(0, 3.14)), 2)))
        n_placed += 1
    return out


def _humans() -> list[dict]:
    """Two standing, two walking. Patrol A crosses the corridor repeatedly."""
    def z(x, y):
        return round(float(height_map_frame(x, y)), 3)
    return [
        dict(name="worker_standing", kind="static", x=18.0, y=19.0,
             z=z(18.0, 19.0)),
        dict(name="surveyor_standing", kind="static", x=40.0, y=39.0,
             z=z(40.0, 39.0)),
        dict(name="patrol_crossing", kind="patrol", speed=0.8,
             waypoints=[[28.0, 20.0], [28.0, 34.0]]),
        dict(name="patrol_along", kind="patrol", speed=0.7,
             waypoints=[[44.0, 30.0], [52.0, 26.0]]),
    ]


def generate() -> dict:
    GEN.mkdir(parents=True, exist_ok=True)
    dem = build_dem()
    np.save(GEN / "dem.npy", dem)
    tif = geo.write_geotiff(dem, GEN / "prior_dem_utm44n.tif", WORLD)

    tex = scene.terrain_texture(dem, SEED)
    import cv2
    cv2.imwrite(str(GEN / "terrain_texture.png"),
                cv2.cvtColor(tex, cv2.COLOR_RGB2BGR))

    occ = occupancy(dem)
    # PGM convention: 0 = black = occupied, 254 = white = free.
    pgm = np.where(occ > 50, 0, 254).astype(np.uint8)
    cv2.imwrite(str(GEN / "map.pgm"), pgm)
    res = WORLD / (N - 1)
    (GEN / "map.yaml").write_text(
        "image: map.pgm\n"
        "mode: trinary\n"
        f"resolution: {res:.6f}\n"
        f"origin: [{-HALF - res / 2.0:.4f}, {-HALF - res / 2.0:.4f}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")

    rng = np.random.RandomState(SEED + 7)
    objs = terrain.scene_objects(SEED)
    world = dict(
        seed=SEED, world_size=WORLD, dem_n=N,
        site_datum=dict(lat=geo.SITE_LAT, lon=geo.SITE_LON,
                        anchor_e=geo.ANCHOR_E, anchor_n=geo.ANCHOR_N,
                        epsg=32644),
        start=dict(x=START_XY[0], y=START_XY[1], yaw_deg=START_YAW_DEG,
                   z=round(float(height_map_frame(*START_XY)), 3)),
        goal_default=dict(x=GOAL_XY[0], y=GOAL_XY[1],
                          utm=dict(zip(("easting", "northing"),
                                       geo.map_to_utm(*GOAL_XY)))),
        berm=BERM, tunnel=TUNNEL, plateau=PLATEAU,
        max_slope_deg=MAX_SLOPE_DEG,
        obstacles=_corridor_obstacles(rng),
        humans=_humans(),
        trees=objs["trees"], rocks=objs["rocks"],
        geotiff=tif,
    )
    (GEN / "world.json").write_text(json.dumps(world, indent=1))
    return world


if __name__ == "__main__":
    w = generate()
    print(f"[world] DEM {N}x{N} over {WORLD:.0f} m, "
          f"GeoTIFF {w['geotiff']['path']}")
    print(f"[world] start {w['start']['x']:.1f},{w['start']['y']:.1f}  "
          f"default goal UTM E {w['goal_default']['utm']['easting']:.2f} "
          f"N {w['goal_default']['utm']['northing']:.2f}")
    print(f"[world] {len(w['obstacles'])} corridor obstacles, "
          f"{len(w['humans'])} humans, {len(w['trees'])} trees, "
          f"{len(w['rocks'])} rocks")
