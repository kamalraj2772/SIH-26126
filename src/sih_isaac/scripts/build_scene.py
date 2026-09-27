"""Build the SIH26126 Isaac Sim 6.0 world USD from the generated world files.

Run with Isaac's python:
    cd /isaacsim && ./python.sh <ws>/src/sih_isaac/scripts/build_scene.py

Reads  src/sih_isaac/generated/{dem.npy, world.json, terrain_texture.png}
Writes src/sih_isaac/usd/sih26126_world.usd  (+ terrain_texture.png alongside)
       src/sih_isaac/generated/scene_preview.png

The terrain mesh is built from the same DEM that was written to the prior
GeoTIFF, so simulated ground truth and the planner's map agree by construction.
"""
import json
import pathlib
import shutil
import sys

import numpy as np

WS = pathlib.Path("/home/qbotix-rover/sih_ws")
PKG = WS / "src/sih_isaac"
GEN = PKG / "generated"
USD_DIR = PKG / "usd"
USD_PATH = USD_DIR / "sih26126_world.usd"

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.kit.commands
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from pxr import (Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics,
                 UsdShade)

enable_extension("isaacsim.asset.importer.urdf")
enable_extension("isaacsim.sensors.rtx")

world = json.loads((GEN / "world.json").read_text())
dem = np.load(GEN / "dem.npy")
N = dem.shape[0]
SIZE = world["world_size"]
HALF = SIZE / 2.0
RES = SIZE / (N - 1)


def hmap(x, y):
    """Bilinear DEM sample at map-frame (x, y). dem row 0 = +y edge."""
    c = (x + HALF) / RES
    r = (HALF - y) / RES
    r0, c0 = int(np.clip(r, 0, N - 2)), int(np.clip(c, 0, N - 2))
    fr, fc = r - r0, c - c0
    return float(dem[r0, c0] * (1 - fr) * (1 - fc) + dem[r0, c0 + 1] * (1 - fr) * fc
                 + dem[r0 + 1, c0] * fr * (1 - fc) + dem[r0 + 1, c0 + 1] * fr * fc)


from isaacsim.core.utils.stage import create_new_stage

create_new_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

# ---------------------------------------------------------------- physics
pscene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
pscene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
pscene.CreateGravityMagnitudeAttr(9.81)

ground_mat = UsdShade.Material.Define(stage, "/World/Physics/groundMat")
pm = UsdPhysics.MaterialAPI.Apply(ground_mat.GetPrim())
pm.CreateStaticFrictionAttr(0.9)
pm.CreateDynamicFrictionAttr(0.8)
pm.CreateRestitutionAttr(0.0)

# ---------------------------------------------------------------- lights
dome = UsdLux.DomeLight.Define(stage, "/World/Lights/dome")
dome.CreateIntensityAttr(400.0)
sun = UsdLux.DistantLight.Define(stage, "/World/Lights/sun")
sun.CreateIntensityAttr(2200.0)
sun.CreateAngleAttr(0.53)
sun.AddRotateXYZOp().Set(Gf.Vec3f(-52.0, 12.0, 35.0))

# ---------------------------------------------------------------- terrain
pts = []
for i in range(N):
    y = HALF - i * RES
    for j in range(N):
        pts.append(Gf.Vec3f(-HALF + j * RES, y, float(dem[i, j])))
idx, cnt, st = [], [], []
for i in range(N - 1):
    for j in range(N - 1):
        a = i * N + j
        # CCW seen from +z (row 0 is the +y edge), so PhysX normals face up
        idx += [a, a + N, a + N + 1, a, a + N + 1, a + 1]
        cnt += [3, 3]
for i in range(N):
    for j in range(N):
        st.append(Gf.Vec2f(j / (N - 1), 1.0 - i / (N - 1)))

mesh = UsdGeom.Mesh.Define(stage, "/World/terrain")
mesh.CreatePointsAttr(pts)
mesh.CreateFaceVertexIndicesAttr(idx)
mesh.CreateFaceVertexCountsAttr(cnt)
mesh.CreateSubdivisionSchemeAttr("none")
pv = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
    "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
pv.Set(st)

USD_DIR.mkdir(parents=True, exist_ok=True)
shutil.copy(GEN / "terrain_texture.png", USD_DIR / "terrain_texture.png")
tmat = UsdShade.Material.Define(stage, "/World/Looks/terrainMat")
sh = UsdShade.Shader.Define(stage, "/World/Looks/terrainMat/pbr")
sh.CreateIdAttr("UsdPreviewSurface")
sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.95)
tex = UsdShade.Shader.Define(stage, "/World/Looks/terrainMat/tex")
tex.CreateIdAttr("UsdUVTexture")
tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set("./terrain_texture.png")
tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
rd = UsdShade.Shader.Define(stage, "/World/Looks/terrainMat/stReader")
rd.CreateIdAttr("UsdPrimvarReader_float2")
rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
    rd.CreateOutput("result", Sdf.ValueTypeNames.Float2))
sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3))
tmat.CreateSurfaceOutput().ConnectToSource(
    sh.CreateOutput("surface", Sdf.ValueTypeNames.Token))
UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(tmat)

UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
mca = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
mca.CreateApproximationAttr("none")            # exact trimesh, static
UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(
    ground_mat, UsdShade.Tokens.weakerThanDescendants, "physics")


def color(prim, rgb):
    UsdGeom.Gprim(prim).CreateDisplayColorAttr([Gf.Vec3f(*rgb)])


def collide(prim):
    UsdPhysics.CollisionAPI.Apply(prim.GetPrim() if hasattr(prim, "GetPrim") else prim)


def xform_set(xf, x, y, z, yaw_deg=0.0, pitch_deg=0.0):
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(0.0, pitch_deg, yaw_deg))


# ---------------------------------------------------------------- tunnel roof
t, b = world["tunnel"], world["berm"]
bx = (b["x0"] + b["x1"]) / 2.0
top_z = hmap(bx, t["y"] + 10.0)                 # berm crest height nearby
roof = UsdGeom.Cube.Define(stage, "/World/tunnel/roof")
roof.GetPrim().GetAttribute("size").Set(1.0)
xf = UsdGeom.Xformable(roof)
xf.AddTranslateOp().Set(Gf.Vec3d(bx, t["y"], top_z + 0.05))
xf.AddScaleOp().Set(Gf.Vec3d(b["x1"] - b["x0"] + 3.0,
                             2 * (t["half_width"] + t["edge"]) + 1.2, 0.6))
color(roof, (0.42, 0.38, 0.33))
collide(roof)
for k, dx in enumerate((-4.5, 0.0, 4.5)):       # lights inside the bore
    L = UsdLux.SphereLight.Define(stage, f"/World/tunnel/light_{k}")
    L.CreateIntensityAttr(9000.0)
    L.CreateRadiusAttr(0.07)
    UsdGeom.Xformable(L).AddTranslateOp().Set(
        Gf.Vec3d(bx + dx, t["y"], hmap(bx + dx, t["y"]) + 2.4))

# ---------------------------------------------------------------- obstacles
obst_root = UsdGeom.Xform.Define(stage, "/World/obstacles")
for o in world["obstacles"]:
    p = f"/World/obstacles/{o['name']}"
    if o["kind"] == "drum":
        g = UsdGeom.Cylinder.Define(stage, p)
        g.CreateRadiusAttr(0.30); g.CreateHeightAttr(0.90); g.CreateAxisAttr("Z")
        xform_set(UsdGeom.Xformable(g), o["x"], o["y"], o["z"] + 0.45)
        color(g, (0.85, 0.35, 0.05))
    elif o["kind"] == "crate":
        g = UsdGeom.Cube.Define(stage, p)
        g.GetPrim().GetAttribute("size").Set(1.0)
        xf = UsdGeom.Xformable(g)
        xf.AddTranslateOp().Set(Gf.Vec3d(o["x"], o["y"], o["z"] + 0.42))
        xf.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, np.degrees(o["yaw"])))
        xf.AddScaleOp().Set(Gf.Vec3d(0.85, 0.85, 0.85))
        color(g, (0.55, 0.42, 0.22))
    elif o["kind"] == "log":
        g = UsdGeom.Cylinder.Define(stage, p)
        g.CreateRadiusAttr(0.22); g.CreateHeightAttr(2.4); g.CreateAxisAttr("X")
        xf = UsdGeom.Xformable(g)
        xf.AddTranslateOp().Set(Gf.Vec3d(o["x"], o["y"], o["z"] + 0.22))
        xf.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, np.degrees(o["yaw"])))
        color(g, (0.35, 0.25, 0.15))
    else:                                        # boulder
        g = UsdGeom.Sphere.Define(stage, p)
        g.CreateRadiusAttr(o["radius"])
        xf = UsdGeom.Xformable(g)
        xf.AddTranslateOp().Set(Gf.Vec3d(o["x"], o["y"], o["z"] + o["radius"] * 0.55))
        xf.AddScaleOp().Set(Gf.Vec3d(1.0, 0.85, 0.62))
        color(g, (0.45, 0.44, 0.42))
    collide(g)

# ---------------------------------------------------------------- trees, rocks
for i, tr in enumerate(world["trees"]):
    base = f"/World/vegetation/tree_{i}"
    trunk = UsdGeom.Cylinder.Define(stage, base + "/trunk")
    trunk.CreateRadiusAttr(tr["trunk_radius"])
    trunk.CreateHeightAttr(tr["trunk_height"]); trunk.CreateAxisAttr("Z")
    xform_set(UsdGeom.Xformable(trunk), tr["x"], tr["y"],
              tr["z"] + tr["trunk_height"] / 2)
    color(trunk, (0.32, 0.22, 0.12)); collide(trunk)
    can = UsdGeom.Sphere.Define(stage, base + "/canopy")
    can.CreateRadiusAttr(tr["canopy_radius"])
    xf = UsdGeom.Xformable(can)
    xf.AddTranslateOp().Set(Gf.Vec3d(tr["x"], tr["y"],
                                     tr["z"] + tr["trunk_height"] + tr["canopy_radius"] * 0.55))
    xf.AddScaleOp().Set(Gf.Vec3d(1.0, 1.0, 0.8))
    color(can, (0.13, 0.35 + 0.1 * (i % 3), 0.10))
for i, r in enumerate(world["rocks"]):
    g = UsdGeom.Sphere.Define(stage, f"/World/rocks/rock_{i}")
    g.CreateRadiusAttr(max(0.12, r["size"]))
    xf = UsdGeom.Xformable(g)
    xf.AddTranslateOp().Set(Gf.Vec3d(r["x"], r["y"], r["z"] + r["size"] * 0.4))
    xf.AddScaleOp().Set(Gf.Vec3d(1.0, 0.8, 0.6))
    color(g, (0.48, 0.47, 0.45)); collide(g)

# ---------------------------------------------------------------- humans
from isaacsim.storage.native import get_assets_root_path, is_file

assets_root = get_assets_root_path()
candidates = ["F_Business_02", "M_Business_02", "F_Medical_01", "M_Medical_01",
              "F_Worker_01", "M_Worker_01", "F_Business_01", "M_Business_01"]
found = []
for c in candidates:
    url = f"{assets_root}/Isaac/People/Characters/{c}/{c}.usd"
    try:
        if is_file(url):
            found.append(url)
    except Exception:
        pass
    if len(found) >= len(world["humans"]):
        break
print(f"[humans] {len(found)} cloud characters resolved")

for i, h in enumerate(world["humans"]):
    root = UsdGeom.Xform.Define(stage, f"/World/humans/{h['name']}")
    if h["kind"] == "patrol":
        x, y = h["waypoints"][0]
        z = hmap(x, y)
    else:
        x, y, z = h["x"], h["y"], h["z"]
    UsdGeom.Xformable(root).AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    if i < len(found):
        geom = UsdGeom.Xform.Define(stage, f"/World/humans/{h['name']}/geom")
        geom.GetPrim().GetReferences().AddReference(found[i])
    else:                                       # primitive stand-in
        body = UsdGeom.Capsule.Define(stage, f"/World/humans/{h['name']}/body")
        body.CreateRadiusAttr(0.22); body.CreateHeightAttr(1.1)
        body.CreateAxisAttr("Z")
        UsdGeom.Xformable(body).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.85))
        color(body, (0.75, 0.30, 0.20))
        head = UsdGeom.Sphere.Define(stage, f"/World/humans/{h['name']}/head")
        head.CreateRadiusAttr(0.14)
        UsdGeom.Xformable(head).AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.62))
        color(head, (0.80, 0.62, 0.48))
    # invisible physics proxy the lidar-independent physics world collides with
    cap = UsdGeom.Capsule.Define(stage, f"/World/humans/{h['name']}/proxy")
    cap.CreateRadiusAttr(0.30); cap.CreateHeightAttr(1.0); cap.CreateAxisAttr("Z")
    UsdGeom.Xformable(cap).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.85))
    UsdGeom.Imageable(cap).MakeInvisible()
    collide(cap)
    if h["kind"] == "patrol":
        rb = UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
        rb.CreateKinematicEnabledAttr(True)

# ---------------------------------------------------------------- rover (URDF)
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

cfg = URDFImporterConfig(
    urdf_path=str(PKG / "sih_rover.urdf"),
    usd_path=str(USD_DIR / "rover"),
    merge_fixed_joints=False,
    fix_base=False,
    joint_target_type="Velocity",
    override_joint_stiffness=0.0,
    override_joint_damping=2000.0,
)
rover_usd = URDFImporter(cfg).import_urdf()
print("[urdf] rover usd:", rover_usd)

rover_path = "/World/sih_rover"
start = world["start"]
rover_prim = stage.DefinePrim(rover_path, "Xform")
rover_prim.GetReferences().AddReference(rover_usd)
xf = UsdGeom.Xformable(rover_prim)
xf.ClearXformOpOrder()
xf.AddTranslateOp().Set(Gf.Vec3d(start["x"], start["y"], start["z"] + 0.35))
xf.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, start["yaw_deg"]))

# velocity drives on the four wheels
for jname in ["wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint"]:
    jp = None
    for p in Usd.PrimRange(rover_prim):
        if p.GetName() == jname:
            jp = p
            break
    if jp is None:
        print("[urdf] MISSING JOINT", jname)
        continue
    drv = UsdPhysics.DriveAPI.Apply(jp, "angular")
    drv.CreateTypeAttr("force")
    drv.CreateStiffnessAttr(0.0)
    drv.CreateDampingAttr(5000.0)
    drv.CreateMaxForceAttr(900.0)

# Skid-steer needs lateral scrub: a moderate-friction material on the wheels
# (0.45 still climbs the <=12 deg route slopes, tan 12 = 0.21) or in-place
# turning stalls against the terrain trimesh.
wheel_mat = UsdShade.Material.Define(stage, "/World/Physics/wheelMat")
wpm = UsdPhysics.MaterialAPI.Apply(wheel_mat.GetPrim())
wpm.CreateStaticFrictionAttr(0.45)
wpm.CreateDynamicFrictionAttr(0.40)
wpm.CreateRestitutionAttr(0.0)
n_wheel_col = 0
for p in Usd.PrimRange(rover_prim):
    if "wheel" in str(p.GetPath()).lower() and p.HasAPI(UsdPhysics.CollisionAPI):
        UsdShade.MaterialBindingAPI.Apply(p).Bind(
            wheel_mat, UsdShade.Tokens.strongerThanDescendants, "physics")
        n_wheel_col += 1
print(f"[urdf] wheel material bound to {n_wheel_col} collision prims")

# camera on the camera mount, looking along body +X
cam_path = None
for p in Usd.PrimRange(rover_prim):
    if p.GetName() == "camera_link":
        cam_path = str(p.GetPath())
        break
cam_parent = cam_path or rover_path
cam = UsdGeom.Camera.Define(stage, cam_parent + "/zed_camera")
cam.CreateFocalLengthAttr(8.484)               # 102 deg HFOV on 20.955 mm
cam.CreateHorizontalApertureAttr(20.955)
cam.CreateVerticalApertureAttr(20.955 * 360.0 / 640.0)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.15, 120.0))
# USD camera looks along -Z with +Y up; this maps view dir to body +X,
# camera-up to body +Z, camera-right to body -Y (verified: R*(0,0,-1)=(1,0,0)).
q = Gf.Quatd(0.5, Gf.Vec3d(0.5, -0.5, -0.5))
xf = UsdGeom.Xformable(cam)
off = Gf.Vec3d(0, 0, 0) if cam_path else Gf.Vec3d(0.37, 0, 0.125)
xf.AddTranslateOp().Set(off)
xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(q)

# lidar mast + RTX lidar
base_path = rover_path
for p in Usd.PrimRange(rover_prim):
    if p.GetName() == "base_link":
        base_path = str(p.GetPath())
        break
mast = UsdGeom.Cylinder.Define(stage, base_path + "/mast")
mast.CreateRadiusAttr(0.03); mast.CreateHeightAttr(0.30); mast.CreateAxisAttr("Z")
UsdGeom.Xformable(mast).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.30))
color(mast, (0.15, 0.15, 0.15))
# The RTX lidar sensor prim is created at runtime by run_sim.py (a baked
# OmniLidar prim crashes the renderer when the stage is re-opened).
(GEN / "base_link_path.txt").write_text(base_path)

# ---------------------------------------------------------------- save
omni.usd.get_context().save_as_stage(str(USD_PATH))
print("[scene] saved", USD_PATH)

# ---------------------------------------------------------------- preview
from isaacsim.core.api import World as SimWorld
from isaacsim.sensors.camera import Camera as PreviewCam

sim = SimWorld(stage_units_in_meters=1.0)
sim.reset()
pv = PreviewCam("/World/preview_cam", resolution=(1280, 720))
pv.initialize()
pv.set_world_pose(np.array([start["x"] - 8.0, start["y"] - 14.0,
                            start["z"] + 9.0]), None)
import isaacsim.core.utils.numpy.rotations as rot_utils
look = np.array([np.radians(-28.0), 0.0, np.radians(62.0)])
pv.set_world_pose(None, rot_utils.euler_angles_to_quats(
    np.array([0.0, 28.0, 62.0]), degrees=True))
for _ in range(45):
    sim.step(render=True)
rgba = pv.get_rgba()
if rgba is not None and rgba.size:
    import cv2
    cv2.imwrite(str(GEN / "scene_preview.png"),
                cv2.cvtColor(rgba[..., :3], cv2.COLOR_RGB2BGR))
    print("[scene] preview written")
# rover must still be upright and on the ground after 45 physics steps
from isaacsim.core.prims import SingleArticulation
art = SingleArticulation(rover_path)
art.initialize()
pos, orn = art.get_world_pose()
print(f"[check] rover after settle: pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) "
      f"start z {start['z']:.2f}  dofs={art.num_dof}")
app.close()
print("[build] DONE")
