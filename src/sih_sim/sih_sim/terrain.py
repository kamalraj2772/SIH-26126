"""Analytic terrain height function for the SIH26126 Phase 1 world.

Pure function of (x, y, seed) in world ENU meters, origin at the heightmap
centre. Shared by heightmap_generator.py (rasterizes it to a PNG + raw .npy
DEM) and world_generator.py (samples it to place trees/rocks/spawn/goal on
the actual surface, not at a guessed z).

World layout (all coordinates in metres, world spans [-75, 75] on x and y):
  - ridge   : traversable ramp-plateau-ramp, ~12 deg faces, SW quadrant
  - mound   : cone, ~35 deg half-angle, NE quadrant (deliberately un-climbable)
  - ditch   : 1.5 m deep x 2 m wide trench crossing the middle
  - low-texture patch: 30x30 m region (NE-of-centre) where fine surface
    micro-relief is suppressed, distinct from the trees/rocks/ridge/mound
  - tree grove and rock field: NOT part of the height field, see
    scene_objects() below -- they are separate SDF primitives placed on
    top of this surface.
"""
from __future__ import annotations

import numpy as np

WORLD_SIZE = 150.0          # metres, both x and y
SAMPLES = 513                # 2^n + 1, gz-sim heightmap requirement

RIDGE = dict(x0=-40.0, x1=-5.0, face_len=10.0, y0=-45.0, y1=-25.0,
             slope_deg=12.0, y_margin=3.0)
MOUND = dict(cx=35.0, cy=30.0, slope_deg=35.0, height=4.0)
DITCH = dict(x0=-12.0, x1=12.0, y=5.0, width=2.0, depth=1.5, end_margin=1.0)
LOW_TEXTURE_PATCH = dict(x0=10.0, x1=40.0, y0=-40.0, y1=-10.0, margin=3.0)

TREE_GROVE = dict(cx=-45.0, cy=45.0, half=12.5, count=40)
ROCK_FIELD = dict(cx=55.0, cy=-55.0, half=12.5, count=60)

SPAWN_XY = (-68.0, -68.0)
GOAL_XY = (68.0, 68.0)

_MICRO_RELIEF_AMPLITUDE = 0.06  # metres


def _smoothstep(edge0: np.ndarray, edge1: np.ndarray, x: np.ndarray) -> np.ndarray:
    t = np.clip((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def _value_noise_octave(x: np.ndarray, y: np.ndarray, seed: int, cell: float) -> np.ndarray:
    """Bilinear value noise at a given lattice cell size (metres)."""
    gx = np.floor(x / cell).astype(np.int64)
    gy = np.floor(y / cell).astype(np.int64)
    fx = x / cell - gx
    fy = y / cell - gy

    def lattice_value(ix: np.ndarray, iy: np.ndarray) -> np.ndarray:
        # Hash lattice coords to a deterministic pseudo-random value in [-1, 1].
        # Deliberate uint64 wraparound (defined behaviour) -- silence numpy's
        # scalar-overflow warning for it rather than suppress warnings globally.
        with np.errstate(over="ignore"):
            h = (ix.astype(np.uint64) * np.uint64(374761393)
                 + iy.astype(np.uint64) * np.uint64(668265263)
                 + np.uint64(seed) * np.uint64(2246822519))
            h = (h ^ (h >> np.uint64(13))) * np.uint64(1274126177)
            h = h ^ (h >> np.uint64(16))
        return (h.astype(np.float64) % 100000) / 50000.0 - 1.0

    v00 = lattice_value(gx, gy)
    v10 = lattice_value(gx + 1, gy)
    v01 = lattice_value(gx, gy + 1)
    v11 = lattice_value(gx + 1, gy + 1)

    sx = _smoothstep(np.zeros_like(fx), np.ones_like(fx), fx)
    sy = _smoothstep(np.zeros_like(fy), np.ones_like(fy), fy)

    a = v00 * (1 - sx) + v10 * sx
    b = v01 * (1 - sx) + v11 * sx
    return a * (1 - sy) + b * sy


def micro_relief(x: np.ndarray, y: np.ndarray, seed: int) -> np.ndarray:
    total = np.zeros_like(x, dtype=np.float64)
    amplitude = _MICRO_RELIEF_AMPLITUDE
    cell = 6.0
    for octave in range(4):
        total += amplitude * _value_noise_octave(x, y, seed + octave * 97, cell)
        amplitude *= 0.5
        cell *= 0.45
    return total


def low_texture_mask(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """1.0 = full micro-relief, 0.0 = suppressed (inside the low-texture patch)."""
    p = LOW_TEXTURE_PATCH
    depth_x = np.minimum(x - p["x0"], p["x1"] - x)
    depth_y = np.minimum(y - p["y0"], p["y1"] - y)
    depth = np.minimum(depth_x, depth_y)
    depth = np.maximum(depth, 0.0)
    return 1.0 - _smoothstep(0.0, p["margin"], depth)


def ridge_height(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    r = RIDGE
    slope = np.tan(np.deg2rad(r["slope_deg"]))
    plateau_h = r["face_len"] * slope
    x0, x1, fl = r["x0"], r["x1"], r["face_len"]

    up_end = x0 + fl
    down_start = x1 - fl

    h = np.zeros_like(x, dtype=np.float64)
    h = np.where((x >= x0) & (x < up_end), (x - x0) * slope, h)
    h = np.where((x >= up_end) & (x < down_start), plateau_h, h)
    h = np.where((x >= down_start) & (x <= x1), (x1 - x) * slope, h)

    y_fade = _smoothstep(r["y0"], r["y0"] + r["y_margin"], y) * \
        (1.0 - _smoothstep(r["y1"] - r["y_margin"], r["y1"], y))
    in_x = (x >= x0) & (x <= x1)
    return np.where(in_x, h * y_fade, 0.0)


def mound_height(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    m = MOUND
    slope = np.tan(np.deg2rad(m["slope_deg"]))
    r = np.hypot(x - m["cx"], y - m["cy"])
    return np.maximum(0.0, m["height"] - r * slope)


def ditch_height(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    d = DITCH
    half_w = d["width"] / 2.0
    dy = np.abs(y - d["y"])
    cross = -d["depth"] * (1.0 - _smoothstep(half_w - 0.5, half_w, dy))

    x_fade = _smoothstep(d["x0"], d["x0"] + d["end_margin"], x) * \
        (1.0 - _smoothstep(d["x1"] - d["end_margin"], d["x1"], x))
    in_x = (x >= d["x0"]) & (x <= d["x1"])
    return np.where(in_x, cross * x_fade, 0.0)


def terrain_height(x: np.ndarray, y: np.ndarray, seed: int) -> np.ndarray:
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    relief = micro_relief(x, y, seed) * low_texture_mask(x, y)
    return relief + ridge_height(x, y) + mound_height(x, y) + ditch_height(x, y)


def scene_objects(seed: int) -> dict:
    """Deterministic tree-trunk and rock placements, sampled on the surface."""
    rng = np.random.RandomState(seed + 1)

    def scatter(spec, min_sep):
        pts = []
        cx, cy, half = spec["cx"], spec["cy"], spec["half"]
        attempts = 0
        while len(pts) < spec["count"] and attempts < spec["count"] * 200:
            attempts += 1
            px = rng.uniform(cx - half, cx + half)
            py = rng.uniform(cy - half, cy + half)
            if all((px - qx) ** 2 + (py - qy) ** 2 >= min_sep ** 2 for qx, qy, in pts):
                pts.append((px, py))
        return pts

    trees = []
    for (px, py) in scatter(TREE_GROVE, min_sep=1.2):
        pz = float(terrain_height(np.array(px), np.array(py), seed))
        trunk_h = float(rng.uniform(2.5, 4.5))
        trunk_r = float(rng.uniform(0.12, 0.22))
        canopy_r = float(rng.uniform(0.8, 1.4))
        trees.append(dict(x=px, y=py, z=pz, trunk_height=trunk_h,
                           trunk_radius=trunk_r, canopy_radius=canopy_r))

    rocks = []
    for (px, py) in scatter(ROCK_FIELD, min_sep=0.5):
        pz = float(terrain_height(np.array(px), np.array(py), seed))
        size = float(rng.uniform(0.1, 0.6))
        shape = "sphere" if rng.uniform() < 0.5 else "box"
        rocks.append(dict(x=px, y=py, z=pz, size=size, shape=shape))

    return dict(trees=trees, rocks=rocks)


def spawn_and_goal(seed: int) -> dict:
    sx, sy = SPAWN_XY
    gx, gy = GOAL_XY
    sz = float(terrain_height(np.array(sx), np.array(sy), seed))
    gz = float(terrain_height(np.array(gx), np.array(gy), seed))
    distance = float(np.hypot(gx - sx, gy - sy))
    return dict(
        spawn=dict(x=sx, y=sy, z=sz),
        goal=dict(x=gx, y=gy, z=gz),
        distance_m=distance,
    )
