"""Builds the MuJoCo scene for the SIH26126 demo.

The terrain is the *same analytic surface* used by the Gazebo world generator
(sih_sim.terrain), so the demo and the ROS 2 stack are looking at one world.
MuJoCo is used here because it renders and steps far faster than gz-sim and
its shadowed, textured output reads as a real field rather than a cartoon.
"""
from __future__ import annotations

import pathlib

import numpy as np

from sih_demo import mission

HFIELD_N = 257           # collision/render resolution of the heightfield
TEX_N = 2048             # terrain colour texture resolution


def _hash_noise(shape, seed, octaves=5, base_cell=8):
    """Cheap fractal value noise on a regular grid, in [-1, 1]."""
    rng = np.random.RandomState(seed)
    out = np.zeros(shape, dtype=np.float64)
    amp, cell = 1.0, base_cell
    total_amp = 0.0
    for _ in range(octaves):
        h = max(2, int(shape[0] / cell))
        w = max(2, int(shape[1] / cell))
        coarse = rng.rand(h, w) * 2.0 - 1.0
        ys = np.linspace(0, h - 1, shape[0])
        xs = np.linspace(0, w - 1, shape[1])
        y0 = np.floor(ys).astype(int).clip(0, h - 2)
        x0 = np.floor(xs).astype(int).clip(0, w - 2)
        fy = (ys - y0)[:, None]
        fx = (xs - x0)[None, :]
        fy = fy * fy * (3 - 2 * fy)
        fx = fx * fx * (3 - 2 * fx)
        c = coarse
        top = c[np.ix_(y0, x0)] * (1 - fx) + c[np.ix_(y0, x0 + 1)] * fx
        bot = c[np.ix_(y0 + 1, x0)] * (1 - fx) + c[np.ix_(y0 + 1, x0 + 1)] * fx
        out += amp * (top * (1 - fy) + bot * fy)
        total_amp += amp
        amp *= 0.5
        cell *= 0.5
    return out / total_amp


def terrain_texture(heights_north_up: np.ndarray, seed: int) -> np.ndarray:
    """Colour the terrain from its own elevation and slope: dry grass on the
    flats, exposed rock and scree on the steep faces, damp soil in the ditch."""
    n = TEX_N
    src = heights_north_up
    # Resample the DEM up to texture resolution.
    yi = np.linspace(0, src.shape[0] - 1, n)
    xi = np.linspace(0, src.shape[1] - 1, n)
    y0 = np.floor(yi).astype(int).clip(0, src.shape[0] - 2)
    x0 = np.floor(xi).astype(int).clip(0, src.shape[1] - 2)
    fy = (yi - y0)[:, None]
    fx = (xi - x0)[None, :]
    top = src[np.ix_(y0, x0)] * (1 - fx) + src[np.ix_(y0, x0 + 1)] * fx
    bot = src[np.ix_(y0 + 1, x0)] * (1 - fx) + src[np.ix_(y0 + 1, x0 + 1)] * fx
    h = top * (1 - fy) + bot * fy

    cell = mission_world_size() / n
    gy, gx = np.gradient(h, cell)
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))

    grain = _hash_noise((n, n), seed, octaves=7, base_cell=3)
    patch = _hash_noise((n, n), seed + 7, octaves=3, base_cell=96)
    scrub = _hash_noise((n, n), seed + 13, octaves=4, base_cell=28)
    ochre = _hash_noise((n, n), seed + 23, octaves=3, base_cell=48)

    grass_a = np.array([101, 112, 62], dtype=np.float64)     # olive/dry grass
    grass_b = np.array([146, 152, 92], dtype=np.float64)     # sun-bleached
    scrub_c = np.array([72, 88, 48], dtype=np.float64)       # dark scrub clumps
    ochre_c = np.array([158, 134, 88], dtype=np.float64)     # bare ochre earth
    rock = np.array([132, 124, 112], dtype=np.float64)       # grey scree
    soil = np.array([92, 74, 53], dtype=np.float64)          # damp soil

    t_patch = np.clip(patch * 0.6 + 0.5, 0, 1)[..., None]
    img = grass_a * (1 - t_patch) + grass_b * t_patch

    t_scrub = np.clip((scrub - 0.18) * 2.4, 0, 1)[..., None]
    img = img * (1 - t_scrub) + scrub_c * t_scrub

    t_ochre = np.clip((ochre - 0.30) * 2.8, 0, 1)[..., None]
    img = img * (1 - t_ochre) + ochre_c * t_ochre

    t_rock = np.clip((slope - 11.0) / 18.0, 0, 1)[..., None] * 0.78
    img = img * (1 - t_rock) + rock * t_rock

    t_soil = np.clip((-h - 0.30) / 0.9, 0, 1)[..., None]     # inside the ditch
    img = img * (1 - t_soil) + soil * t_soil

    img *= (1.0 + 0.20 * grain)[..., None]                   # surface grain
    # Hillshade so relief is legible even in flat light.
    shade = np.clip(1.0 + 0.85 * (gx * 0.6 + gy * 0.6), 0.62, 1.26)[..., None]
    img *= shade
    return np.clip(img, 0, 255).astype(np.uint8)


def mission_world_size() -> float:
    from sih_sim import terrain
    return float(terrain.WORLD_SIZE)


def build_dem(seed: int, n: int = HFIELD_N) -> np.ndarray:
    """Analytic DEM sampled north-up (row 0 = +y edge), matching sih_sim."""
    from sih_sim import terrain
    size = terrain.WORLD_SIZE
    half = size / 2.0
    coords = np.linspace(-half, half, n)
    xx, yy = np.meshgrid(coords, coords)
    h_bottom_up = terrain.terrain_height(xx, yy, seed)
    return np.flipud(h_bottom_up)


def _obstacle_bodies(seed: int, corridor) -> list[dict]:
    """Corridor obstacles plus the background grove/rock field from sih_sim."""
    from sih_sim import terrain
    out = []
    for i, (x, y, r, kind) in enumerate(corridor):
        z = float(terrain.terrain_height(np.array(x), np.array(y), seed))
        out.append(dict(name=f"obs_{i}", x=x, y=y, z=z, radius=r, kind=kind))

    objs = terrain.scene_objects(seed)
    for i, t in enumerate(objs["trees"]):
        out.append(dict(name=f"grove_{i}", x=t["x"], y=t["y"], z=t["z"],
                        radius=t["trunk_radius"], kind="tree",
                        trunk_height=t["trunk_height"],
                        canopy_radius=t["canopy_radius"]))
    for i, r in enumerate(objs["rocks"]):
        out.append(dict(name=f"rock_{i}", x=r["x"], y=r["y"], z=r["z"],
                        radius=max(0.12, r["size"]), kind="rock"))
    return out


def _obstacle_xml(o: dict, rng: np.random.RandomState) -> str:
    x, y, z, r = o["x"], o["y"], o["z"], o["radius"]
    if o["kind"] == "tree":
        th = o.get("trunk_height", 3.4)
        cr = o.get("canopy_radius", max(1.0, r * 4.0))
        return f"""
    <body name="{o['name']}" pos="{x:.3f} {y:.3f} {z:.3f}">
      <geom type="cylinder" size="{r:.3f} {th/2:.3f}" pos="0 0 {th/2:.3f}"
            material="bark" contype="1" conaffinity="1"/>
      <geom type="ellipsoid" size="{cr:.2f} {cr:.2f} {cr*0.78:.2f}"
            pos="0 0 {th + cr*0.55:.2f}" material="foliage"
            contype="1" conaffinity="1"/>
    </body>"""
    if o["kind"] == "boulder":
        sx, sy, sz = r, r * rng.uniform(0.8, 1.15), r * rng.uniform(0.62, 0.9)
        yaw = rng.uniform(0, 180)
        return f"""
    <body name="{o['name']}" pos="{x:.3f} {y:.3f} {z + sz*0.55:.3f}" euler="0 0 {yaw:.1f}">
      <geom type="ellipsoid" size="{sx:.3f} {sy:.3f} {sz:.3f}" material="boulder"
            contype="1" conaffinity="1"/>
    </body>"""
    sz = r * rng.uniform(0.6, 0.95)
    return f"""
    <body name="{o['name']}" pos="{x:.3f} {y:.3f} {z + sz*0.5:.3f}">
      <geom type="ellipsoid" size="{r:.3f} {r*0.9:.3f} {sz:.3f}" material="boulder"
            contype="1" conaffinity="1"/>
    </body>"""


def rover_xml() -> str:
    """Skid-steer UGV: 4 driven wheels, ZED 2i on a front mast."""
    # Short wheelbase relative to track: a skid-steer yaws by sliding its
    # tyres sideways, and the resisting moment scales with the wheelbase. At
    # 0.60 m wheelbase this vehicle could not turn at all -- the lateral slip
    # torque exceeded what four contact patches could deliver.
    hx, hy, hz = 0.38, 0.30, 0.11          # chassis half-extents
    wr, ww = mission.WHEEL_RADIUS, 0.09    # wheel radius / half-width
    wy = mission.TRACK_WIDTH / 2.0
    wz = -0.05
    cam_x, cam_y, cam_z = mission.ZED_MOUNT_XYZ
    pitch = mission.ZED_PITCH_DEG
    # Camera looks along body +x with body +z up, then pitched down by |pitch|.
    p = np.deg2rad(pitch)
    fwd = np.array([np.cos(p), 0.0, np.sin(p)])       # optical axis in body frame
    right = np.array([0.0, -1.0, 0.0])                # image +u
    up = np.cross(fwd, right) * -1.0                  # image +v is down, so -up
    up = np.array([-np.sin(p), 0.0, np.cos(p)])

    # Tyres are anisotropic in reality -- they grip far harder along the axle
    # than across it -- and that asymmetry is what lets a skid-steer pivot.
    # MuJoCo's contact friction is isotropic in the tangent plane, so with four
    # equally-gripping wheels the lateral slip moment (arm sqrt(a^2+b^2)) always
    # exceeds the drive moment (arm b) and the vehicle physically cannot yaw.
    # The front pair are therefore modelled as low-friction rollers, which puts
    # the instantaneous centre of rotation on the driven axle and recovers the
    # real platform's turn-in-place behaviour.
    wheels = []
    for nx, sx, mu, drive in (("f", 0.20, 0.04, False), ("r", -0.20, 1.25, True)):
        for ny, sy in (("l", wy), ("r", -wy)):
            nm = f"wheel_{nx}{ny}"
            wheels.append(f"""
        <body name="{nm}" pos="{sx:.3f} {sy:.3f} {wz:.3f}">
          <joint name="{nm}_j" type="hinge" axis="0 1 0" damping="{0.10 if drive else 0.01}"
                 armature="0.010" frictionloss="{0.06 if drive else 0.005}"/>
          <geom name="{nm}_g" type="cylinder" size="{wr:.3f} {ww:.3f}" zaxis="0 1 0"
                material="tyre" friction="{mu} 0.002 0.0001" condim="4"
                mass="1.5" contype="1" conaffinity="1"/>
        </body>""")

    return f"""
    <body name="rover" pos="0 0 0">
      <freejoint name="rover_free"/>
      <site name="base_link" pos="0 0 0" size="0.01" rgba="1 0 0 0"/>
      <geom name="chassis" type="box" size="{hx} {hy} {hz}" material="chassis"
            mass="17.0" contype="1" conaffinity="1"/>
      <geom name="deck" type="box" size="0.26 0.23 0.045" pos="0.02 0 {hz+0.045}"
            material="chassis_dark" mass="2.0" contype="0" conaffinity="0"/>
      <geom name="mast" type="cylinder" size="0.028 0.10" pos="{cam_x-0.02} 0 {hz+0.09+0.10}"
            material="chassis_dark" mass="0.3" contype="0" conaffinity="0"/>
      <!-- ZED 2i stereo bar: 175 x 30 x 33 mm, 120 mm baseline -->
      <geom name="zed_body" type="box" size="0.0875 0.0165 0.015"
            pos="{cam_x} {cam_y} {cam_z}" euler="0 {pitch} 0"
            material="zed" mass="0.17" contype="0" conaffinity="0"/>
      <geom name="zed_lens_l" type="cylinder" size="0.011 0.004" zaxis="1 0 0"
            pos="{cam_x+0.013} {cam_y + mission.ZED_BASELINE_M/2:.3f} {cam_z}"
            material="lens" mass="0.001" contype="0" conaffinity="0"/>
      <geom name="zed_lens_r" type="cylinder" size="0.011 0.004" zaxis="1 0 0"
            pos="{cam_x+0.013} {cam_y - mission.ZED_BASELINE_M/2:.3f} {cam_z}"
            material="lens" mass="0.001" contype="0" conaffinity="0"/>

      <!-- Left imager = the ZED's reference optical frame (REP-103 optical:
           z forward, x right, y down), expressed here as MuJoCo xyaxes. -->
      <camera name="zed_left" mode="fixed" fovy="{_fovy():.3f}"
              pos="{cam_x+0.02} {cam_y + mission.ZED_BASELINE_M/2:.3f} {cam_z}"
              xyaxes="{right[0]:.6f} {right[1]:.6f} {right[2]:.6f} {up[0]:.6f} {up[1]:.6f} {up[2]:.6f}"/>
      <camera name="zed_right" mode="fixed" fovy="{_fovy():.3f}"
              pos="{cam_x+0.02} {cam_y - mission.ZED_BASELINE_M/2:.3f} {cam_z}"
              xyaxes="{right[0]:.6f} {right[1]:.6f} {right[2]:.6f} {up[0]:.6f} {up[1]:.6f} {up[2]:.6f}"/>
      <camera name="chase" mode="trackcom" pos="-6.5 0 3.2" xyaxes="0 -1 0 0.42 0 0.91"/>
      {''.join(wheels)}
    </body>"""


def _fovy() -> float:
    """Vertical FOV implied by the ZED 2i HD720 horizontal FOV and 16:9 frame."""
    hf = np.deg2rad(mission.ZED_HFOV_DEG)
    aspect = mission.ZED_WIDTH / mission.ZED_HEIGHT
    return float(np.degrees(2.0 * np.arctan(np.tan(hf / 2.0) / aspect)))


def build_mjcf(seed: int, tex_path: pathlib.Path, dem: np.ndarray,
               corridor) -> str:
    size = mission_world_size()
    half = size / 2.0
    zmin, zmax = float(dem.min()), float(dem.max())
    elev = max(0.5, zmax - zmin)
    rng = np.random.RandomState(seed + 11)
    obstacles = "".join(_obstacle_xml(o, rng)
                        for o in _obstacle_bodies(seed, corridor))

    return f"""<mujoco model="sih26126_demo">
  <compiler angle="degree" autolimits="true" texturedir="{tex_path.parent}"/>
  <option timestep="{mission.SIM_DT}" gravity="0 0 -9.81" integrator="implicitfast"
          cone="elliptic" impratio="10"/>
  <size njmax="4000" nconmax="1500"/>

  <statistic extent="60" center="0 0 1" meansize="0.15"/>

  <visual>
    <headlight ambient="0.38 0.38 0.40" diffuse="0.32 0.32 0.32" specular="0.1 0.1 0.1"/>
    <rgba haze="0.72 0.78 0.86 1"/>
    <!-- fractions of stat.extent (60 m): near 0.30 m, far 300 m, fog 90-270 m -->
    <map znear="0.005" zfar="5.0" shadowclip="1.2" shadowscale="0.7"
         fogstart="1.5" fogend="4.5"/>
    <quality shadowsize="8192" offsamples="8"/>
    <global offwidth="1920" offheight="1080" azimuth="130" elevation="-22"/>
  </visual>

  <asset>
    <texture name="sky" type="skybox" builtin="gradient" width="512" height="1024"
             rgb1="0.35 0.52 0.78" rgb2="0.82 0.88 0.94"/>
    <texture name="terrain_tex" type="2d" file="{tex_path.name}"/>
    <material name="terrain_mat" texture="terrain_tex" texuniform="false"
              specular="0.05" shininess="0.02" reflectance="0.0"/>
    <material name="bark" rgba="0.31 0.24 0.17 1" specular="0.05" shininess="0.05"/>
    <material name="foliage" rgba="0.21 0.36 0.17 1" specular="0.08" shininess="0.1"/>
    <material name="boulder" rgba="0.46 0.44 0.41 1" specular="0.12" shininess="0.15"/>
    <material name="chassis" rgba="0.82 0.52 0.10 1" specular="0.35" shininess="0.5"/>
    <material name="chassis_dark" rgba="0.16 0.17 0.19 1" specular="0.4" shininess="0.6"/>
    <material name="tyre" rgba="0.09 0.09 0.10 1" specular="0.08" shininess="0.1"/>
    <material name="zed" rgba="0.13 0.14 0.16 1" specular="0.6" shininess="0.8"/>
    <material name="lens" rgba="0.05 0.09 0.16 1" specular="1.0" shininess="1.0"/>
    <texture name="outfield_tex" type="2d" builtin="flat" width="16" height="16"
             rgb1="0.44 0.48 0.31"/>
    <material name="outfield_mat" texture="outfield_tex" texrepeat="1 1"
              specular="0.02" shininess="0.01"/>
    <hfield name="terrain" nrow="{dem.shape[0]}" ncol="{dem.shape[1]}"
            size="{half} {half} {elev:.4f} 2.0"/>
  </asset>

  <worldbody>
    <light name="sun" directional="true" castshadow="true" pos="-30 -40 60"
           dir="0.42 0.56 -0.72" diffuse="0.95 0.92 0.85" specular="0.25 0.25 0.22"/>
    <light name="fill" directional="true" castshadow="false" pos="40 30 40"
           dir="-0.5 -0.4 -0.76" diffuse="0.22 0.24 0.30" specular="0 0 0"/>
    <geom name="outfield" type="plane" size="400 400 5" pos="0 0 {zmin - 0.02:.4f}"
          material="outfield_mat" contype="0" conaffinity="0"/>
    <geom name="terrain_geom" type="hfield" hfield="terrain" pos="0 0 {zmin:.4f}"
          material="terrain_mat" friction="1.0 0.01 0.0005" condim="3"
          contype="1" conaffinity="1"/>
    {obstacles}
    {rover_xml()}
  </worldbody>

  <actuator>
    <velocity name="m_fl" joint="wheel_fl_j" kv="40" ctrlrange="-16 16"/>
    <velocity name="m_fr" joint="wheel_fr_j" kv="40" ctrlrange="-16 16"/>
    <velocity name="m_rl" joint="wheel_rl_j" kv="40" ctrlrange="-16 16"/>
    <velocity name="m_rr" joint="wheel_rr_j" kv="40" ctrlrange="-16 16"/>
  </actuator>

  <sensor>
    <framepos name="gt_pos" objtype="site" objname="base_link"/>
    <framequat name="gt_quat" objtype="site" objname="base_link"/>
    <gyro name="imu_gyro" site="base_link"/>
    <accelerometer name="imu_acc" site="base_link"/>
  </sensor>
</mujoco>
"""
