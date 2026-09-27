#!/usr/bin/env python3
"""Generate a reproducible 513x513 8-bit greyscale heightmap PNG (plus the raw
float DEM as .npy and a features.json describing every placed object) from a
single integer seed.

Usage:
    heightmap_generator.py --seed 42 --out-dir <dir>

Image row/column <-> world (x, y) convention (verify against the running
simulation before trusting it -- see Phase 1 report):
    column = round((x + WORLD_SIZE/2) / cell)   # x: -half..+half -> 0..N-1
    row    = (N-1) - round((y + WORLD_SIZE/2) / cell)  # north-up: row 0 = +y edge

gz-sim heightmap <size> = (WORLD_SIZE, WORLD_SIZE, z_range) with <pos>0 0 0</pos>,
so the world surface sits at z_range * pixel/255 = analytic height + z_offset.
Everything placed on the terrain (spawn, goal, trees, rocks) is written to
features.json already in world coordinates.
"""
import argparse
import json
import pathlib

import numpy as np
from PIL import Image

from sih_sim import terrain


def build_height_grid(seed: int) -> np.ndarray:
    half = terrain.WORLD_SIZE / 2.0
    cell = terrain.WORLD_SIZE / (terrain.SAMPLES - 1)
    idx = np.arange(terrain.SAMPLES)
    xs = -half + idx * cell
    ys = -half + idx * cell

    col_x, row_y_bottom_up = np.meshgrid(xs, ys)  # row_y_bottom_up[r,c] = ys[r]
    heights_bottom_up = terrain.terrain_height(col_x, row_y_bottom_up, seed)
    # Flip vertically: PNG row 0 (top) must be the +y (north) edge.
    heights = np.flipud(heights_bottom_up)
    return heights


def save_png(heights: np.ndarray, path: pathlib.Path) -> tuple[float, float]:
    """Rasterize to 8-bit greyscale. Returns (z_offset, z_range).

    Pixel 0 is the terrain minimum and pixel 255 the maximum, so in the world
    the surface sits at `z_range * pixel/255`, i.e. analytic height + z_offset.

    The SDF then uses <pos>0 0 0</pos>: gz-sim applies the heightmap <pos> z to
    the *visual* but NOT to the DART/Bullet *collision* heightfield, whose base
    always sits at z=0. Verified the hard way -- a non-zero <pos> put the
    collision surface 1.5 m above the visual one and the robot fell through the
    world. Baking the offset into the image keeps both representations aligned.
    """
    z_min = float(heights.min())
    z_max = float(heights.max())
    z_range = z_max - z_min
    if z_range < 1e-6:
        z_range = 1e-6
    normalized = (heights - z_min) / z_range
    pixels = np.clip(np.round(normalized * 255.0), 0, 255).astype(np.uint8)
    Image.fromarray(pixels, mode="L").save(path)
    return -z_min, z_range


def save_diffuse_texture(seed: int, path: pathlib.Path, size: int = 256) -> None:
    """A small tileable sandy-dirt diffuse texture, generated from the seed --
    avoids any network dependency on Fuel-hosted textures."""
    rng = np.random.RandomState(seed + 777)
    base = np.array([0.55, 0.47, 0.35])  # sandy-dirt RGB
    noise = rng.normal(0.0, 0.035, size=(size, size, 1))
    rgb = np.clip(base[None, None, :] + noise, 0.0, 1.0)
    pixels = (rgb * 255).astype(np.uint8)
    Image.fromarray(pixels, mode="RGB").save(path)


def save_flat_normal(path: pathlib.Path, size: int = 4) -> None:
    """Constant 'no bump' normal map (RGB 128,128,255)."""
    pixels = np.zeros((size, size, 3), dtype=np.uint8)
    pixels[..., 0] = 128
    pixels[..., 1] = 128
    pixels[..., 2] = 255
    Image.fromarray(pixels, mode="RGB").save(path)


def generate(seed: int, out_dir: pathlib.Path) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    heights = build_height_grid(seed)

    png_path = out_dir / f"heightmap_{seed}.png"
    npy_path = out_dir / f"heights_{seed}.npy"
    features_path = out_dir / f"features_{seed}.json"
    texture_path = out_dir / f"texture_{seed}.png"
    normal_path = out_dir / "flat_normal.png"

    z_offset, z_range = save_png(heights, png_path)
    # raw DEM stays in analytic coordinates; Phase 4 adds z_offset for world z
    np.save(npy_path, heights)
    save_diffuse_texture(seed, texture_path)
    save_flat_normal(normal_path)

    objects = terrain.scene_objects(seed)
    mission = terrain.spawn_and_goal(seed)

    # every placed object's z is stored in WORLD coordinates
    for t in objects["trees"]:
        t["z"] += z_offset
    for r in objects["rocks"]:
        r["z"] += z_offset
    mission["spawn"]["z"] += z_offset
    mission["goal"]["z"] += z_offset

    features = dict(
        seed=seed,
        world_size=terrain.WORLD_SIZE,
        samples=terrain.SAMPLES,
        z_offset=z_offset,
        z_range=z_range,
        heightmap_png=png_path.name,
        heights_npy=npy_path.name,
        texture_png=texture_path.name,
        normal_png=normal_path.name,
        ridge=terrain.RIDGE,
        mound=terrain.MOUND,
        ditch=terrain.DITCH,
        low_texture_patch=terrain.LOW_TEXTURE_PATCH,
        tree_grove=terrain.TREE_GROVE,
        rock_field=terrain.ROCK_FIELD,
        trees=objects["trees"],
        rocks=objects["rocks"],
        spawn=mission["spawn"],
        goal=mission["goal"],
        distance_m=mission["distance_m"],
    )
    with open(features_path, "w") as f:
        json.dump(features, f, indent=2)

    return features


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--out-dir", type=pathlib.Path, required=True)
    args = parser.parse_args()

    features = generate(args.seed, args.out_dir)
    print(f"seed={features['seed']} z_offset={features['z_offset']:.3f} "
          f"z_range={features['z_range']:.3f} trees={len(features['trees'])} "
          f"rocks={len(features['rocks'])} spawn->goal distance="
          f"{features['distance_m']:.1f} m")
    print(f"wrote {args.out_dir}")


if __name__ == "__main__":
    main()
