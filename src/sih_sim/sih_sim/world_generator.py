#!/usr/bin/env python3
"""Render the Phase 1 Gazebo world SDF from a seed's features.json (generating
the heightmap first if it's missing), so the whole world is reproducible from
a single integer.

Usage:
    world_generator.py --seed 42 --heightmap-dir <dir> --out <world.sdf>

Trees are cylinders (trunk) + a sphere (canopy). Rocks are spheres or boxes.
No external mesh assets, per spec.
"""
import argparse
import json
import pathlib

from sih_sim import heightmap_generator

WORLD_TEMPLATE_HEADER = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="qslam_world">
    <physics name="qslam_physics" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <dart>
        <collision_detector>bullet</collision_detector>
      </dart>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.4 0.2 -0.9</direction>
    </light>
"""

WORLD_TEMPLATE_FOOTER = """
  </world>
</sdf>
"""


def heightmap_model_sdf(features: dict, png_abs_path: pathlib.Path) -> str:
    size = features["world_size"]
    z_range = features["z_range"]
    uri = f"file://{png_abs_path}"
    return f"""
    <model name="terrain">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>{uri}</uri>
              <size>{size} {size} {z_range}</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <heightmap>
              <texture>
                <size>8</size>
                <diffuse>{{DIRT_TEXTURE}}</diffuse>
                <normal>{{FLAT_NORMAL}}</normal>
              </texture>
              <uri>{uri}</uri>
              <size>{size} {size} {z_range}</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </visual>
      </link>
    </model>
"""


def tree_model_sdf(idx: int, t: dict) -> str:
    trunk_top_z = t["z"] + t["trunk_height"] / 2.0
    canopy_z = t["z"] + t["trunk_height"] + t["canopy_radius"] * 0.5
    return f"""
    <model name="tree_{idx}">
      <static>true</static>
      <pose>{t['x']:.3f} {t['y']:.3f} 0 0 0 0</pose>
      <link name="link">
        <collision name="trunk_collision">
          <pose>0 0 {trunk_top_z:.3f} 0 0 0</pose>
          <geometry>
            <cylinder><radius>{t['trunk_radius']:.3f}</radius><length>{t['trunk_height']:.3f}</length></cylinder>
          </geometry>
        </collision>
        <visual name="trunk_visual">
          <pose>0 0 {trunk_top_z:.3f} 0 0 0</pose>
          <geometry>
            <cylinder><radius>{t['trunk_radius']:.3f}</radius><length>{t['trunk_height']:.3f}</length></cylinder>
          </geometry>
          <material>
            <ambient>0.36 0.25 0.15 1</ambient><diffuse>0.36 0.25 0.15 1</diffuse>
          </material>
        </visual>
        <visual name="canopy_visual">
          <pose>0 0 {canopy_z:.3f} 0 0 0</pose>
          <geometry><sphere><radius>{t['canopy_radius']:.3f}</radius></sphere></geometry>
          <material>
            <ambient>0.15 0.35 0.12 1</ambient><diffuse>0.15 0.35 0.12 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""


def rock_model_sdf(idx: int, r: dict) -> str:
    z = r["z"] + r["size"] / 2.0
    if r["shape"] == "sphere":
        geom = f"<sphere><radius>{r['size']/2.0:.3f}</radius></sphere>"
    else:
        geom = f"<box><size>{r['size']:.3f} {r['size']*0.8:.3f} {r['size']*0.7:.3f}</size></box>"
    return f"""
    <model name="rock_{idx}">
      <static>true</static>
      <pose>{r['x']:.3f} {r['y']:.3f} {z:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry>{geom}</geometry></collision>
        <visual name="visual">
          <geometry>{geom}</geometry>
          <material><ambient>0.45 0.44 0.42 1</ambient><diffuse>0.45 0.44 0.42 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def build_world(features: dict, heightmap_dir: pathlib.Path) -> str:
    png_abs = (heightmap_dir / features["heightmap_png"]).resolve()
    texture_abs = (heightmap_dir / features["texture_png"]).resolve()
    normal_abs = (heightmap_dir / features["normal_png"]).resolve()

    parts = [WORLD_TEMPLATE_HEADER]
    hm_sdf = heightmap_model_sdf(features, png_abs)
    hm_sdf = hm_sdf.replace("{DIRT_TEXTURE}", f"file://{texture_abs}")
    hm_sdf = hm_sdf.replace("{FLAT_NORMAL}", f"file://{normal_abs}")
    parts.append(hm_sdf)

    for i, t in enumerate(features["trees"]):
        parts.append(tree_model_sdf(i, t))
    for i, r in enumerate(features["rocks"]):
        parts.append(rock_model_sdf(i, r))

    parts.append(WORLD_TEMPLATE_FOOTER)
    return "".join(parts)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--heightmap-dir", type=pathlib.Path, required=True)
    parser.add_argument("--out", type=pathlib.Path, required=True)
    args = parser.parse_args()

    features_path = args.heightmap_dir / f"features_{args.seed}.json"
    if not features_path.exists():
        heightmap_generator.generate(args.seed, args.heightmap_dir)
    with open(features_path) as f:
        features = json.load(f)

    sdf = build_world(features, args.heightmap_dir)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(sdf)
    print(f"wrote {args.out} ({len(features['trees'])} trees, "
          f"{len(features['rocks'])} rocks, spawn->goal "
          f"{features['distance_m']:.1f} m)")


if __name__ == "__main__":
    main()
