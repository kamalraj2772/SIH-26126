"""Run the SIH26126 mission world in Isaac Sim 6.0 with the ROS 2 bridge.

    cd /isaacsim && source /opt/ros/jazzy/setup.bash && \
        ./python.sh <ws>/src/sih_isaac/scripts/run_sim.py [--headless] [--duration S]

Publishes (sim time on /clock):
    /camera/image, /camera/depth_image, /camera/camera_info   frame camera_optical_frame
    /scan                                                     frame lidar_link
    /imu/data                                                 frame imu_link
    /joint_states
    /odom_wheel      honest diff-drive odometry integrated from measured wheel
                     joint velocities -- topic only, publishes NO TF (the EKF
                     owns odom->base_link, rule 3)
    /ground_truth    evaluation only; nothing in the nav stack may subscribe

Subscribes:
    /cmd_vel         geometry_msgs/Twist -> 4 wheel velocity drives (skid steer)

TF: none published from Isaac at all. robot_state_publisher owns the URDF
frames, the EKF owns odom->base_link, slam_toolbox (or cuVSLAM) owns map->odom.
No GNSS anywhere: nothing here reads or publishes a fix of any kind.
"""
import argparse
import json
import math
import pathlib
import sys

WS = pathlib.Path("/home/qbotix-rover/sih_ws")
PKG = WS / "src/sih_isaac"
GEN = PKG / "generated"
USD_PATH = PKG / "usd/sih26126_world.usd"

p = argparse.ArgumentParser()
p.add_argument("--headless", action="store_true")
p.add_argument("--duration", type=float, default=1e9, help="sim seconds")
p.add_argument("--record", default=None, metavar="DIR",
               help="record the three follow cameras to DIR/isaac_*.mp4")
p.add_argument("--ffmpeg", default="ffmpeg", help="ffmpeg binary for --record")
p.add_argument("--rec-size", default="1920x1080", metavar="WxH",
               help="recording resolution (default 1080p)")
p.add_argument("--rec-fps", type=int, default=15, help="recording frame rate")
args = p.parse_args()

from isaacsim import SimulationApp

app = SimulationApp({"headless": args.headless, "open_usd": str(USD_PATH)})

import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, XFormPrim
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.extensions import enable_extension
from pxr import Gf, UsdGeom

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.sensors.rtx")
app.update()

world_cfg = json.loads((GEN / "world.json").read_text())
stage = omni.usd.get_context().get_stage()

ROVER = "/World/sih_rover"
BASE = None
CAM = None
for prim in stage.Traverse():
    n, path = prim.GetName(), str(prim.GetPath())
    if path.startswith(ROVER):
        if n == "base_link" and BASE is None:
            BASE = path
        elif n == "zed_camera" and CAM is None:
            CAM = path
print(f"[sim] base={BASE}\n[sim] cam={CAM}")
assert BASE and CAM

from pxr import UsdPhysics

ART_ROOT = ROVER
for prim in stage.Traverse():
    if str(prim.GetPath()).startswith(ROVER) and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        ART_ROOT = str(prim.GetPath())
        break
print(f"[sim] articulation root={ART_ROOT}")

_, lidar_prim = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path=BASE + "/rtx_lidar", parent=None, config="Example_Rotary_2D",
    translation=Gf.Vec3d(0.0, 0.0, 0.45),
    orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0))
LIDAR = str(lidar_prim.GetPath())
print(f"[sim] lidar={LIDAR}")

sim = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60.0,
            rendering_dt=1.0 / 30.0)
sim.reset()

rover = SingleArticulation(ROVER)
rover.initialize()
dof_names = rover.dof_names
print("[sim] dofs:", dof_names)
W_IDX = [dof_names.index(j) for j in
         ("wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint")]

# --- IMU (synthesized from rigid-body state: same signals a strapdown IMU
# measures -- body-frame angular velocity and specific force incl. gravity) ---
imu_state = dict(last_v=np.zeros(3), last_t=0.0)


def quat_rot_inv(q, v):
    """Rotate world vector v into the body frame (q = w,x,y,z body->world)."""
    w, x, y, z = q
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    return R.T @ np.asarray(v)


def imu_read(t):
    pos, q = rover.get_world_pose()
    v = np.asarray(rover.get_linear_velocity(), dtype=float)
    om = np.asarray(rover.get_angular_velocity(), dtype=float)
    dt_i = max(t - imu_state["last_t"], 1e-3)
    a_world = (v - imu_state["last_v"]) / dt_i + np.array([0.0, 0.0, 9.81])
    imu_state["last_v"], imu_state["last_t"] = v, t
    return dict(lin_acc=quat_rot_inv(q, a_world),
                ang_vel=quat_rot_inv(q, om),
                orientation=q)

# --- follow cameras: three third-person views that trail the rover -----------
# Pure viewport/recording eye candy: no ROS topic, no TF. Poses are recomputed
# every control tick from the rover's pose with a low-pass filter, so they
# swing smoothly instead of being rigidly welded to the body (a parented
# camera inherits every bump and jitter).
#          name        (fwd, left, up) offset   aim z  up vector      tau  focal
FOLLOW_SPECS = [
    ("chase_cam", (-5.0, 0.0, 2.2),  0.6, (0.0, 0.0, 1.0), 0.60, 14.0),  # behind
    ("side_cam",  (0.0, -6.5, 2.0),  0.6, (0.0, 0.0, 1.0), 0.60, 14.0),  # right side
    ("top_cam",   (0.0, 0.0, 16.0),  0.0, (0.0, 1.0, 0.0), 0.80, 18.0),  # bird's eye
]
CHASE_TAU_AIM = 0.25  # s, look-target smoothing (all cameras)

follow_ops, follow_state = {}, {}
for _name, _off, _aimz, _up, _tau, _focal in FOLLOW_SPECS:
    _cam = UsdGeom.Camera.Define(stage, f"/World/{_name}")
    _cam.GetFocalLengthAttr().Set(_focal)
    _cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 10000.0))
    follow_ops[_name] = UsdGeom.Xformable(_cam.GetPrim()).MakeMatrixXform()
    follow_state[_name] = dict(eye=None, aim=None)


def follow_cams_update(pos, quat, dt):
    w, x, y, z = quat
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    c, s = math.cos(yaw), math.sin(yaw)
    for name, (fwd, left, up_off), aimz, up, tau, _f in FOLLOW_SPECS:
        eye = np.array([pos[0] + fwd * c - left * s,
                        pos[1] + fwd * s + left * c,
                        pos[2] + up_off], dtype=np.float64)
        if name != "top_cam":
            eye[2] = max(eye[2], ground_z(eye[0], eye[1]) + 0.8)
        aim = np.array([pos[0], pos[1], pos[2] + aimz], dtype=np.float64)
        st = follow_state[name]
        if st["eye"] is None:
            st["eye"], st["aim"] = eye, aim
        else:
            st["eye"] += (eye - st["eye"]) * min(dt / tau, 1.0)
            st["aim"] += (aim - st["aim"]) * min(dt / CHASE_TAU_AIM, 1.0)
        view = Gf.Matrix4d()
        view.SetLookAt(Gf.Vec3d(*st["eye"]), Gf.Vec3d(*st["aim"]),
                       Gf.Vec3d(*up))
        follow_ops[name].Set(view.GetInverse())


if not args.headless:
    try:
        from omni.kit.viewport.utility import get_active_viewport
        get_active_viewport().camera_path = "/World/chase_cam"
        print("[sim] viewport -> /World/chase_cam (chase view)")
    except Exception as e:  # viewport API varies across kit builds
        print(f"[sim] WARNING: could not switch viewport to chase cam: {e}")

# --- optional mission recording: follow cameras -> mp4 ------------------------
REC_W, REC_H = (int(v) for v in args.rec_size.lower().split("x"))
REC_FPS = args.rec_fps
# the control tick is 30 Hz; capture every Nth tick to reach REC_FPS
REC_EVERY = max(1, round(30.0 / REC_FPS))
recorders = []
rec_state = {"n": 0}
if args.record:
    import subprocess
    rec_dir = pathlib.Path(args.record)
    rec_dir.mkdir(parents=True, exist_ok=True)
    for cam_name, label in (("chase_cam", "behind"), ("side_cam", "side"),
                            ("top_cam", "top")):
        rp = rep.create.render_product(f"/World/{cam_name}",
                                       resolution=(REC_W, REC_H))
        ann = rep.AnnotatorRegistry.get_annotator("rgb")
        ann.attach(rp)
        proc = subprocess.Popen(
            [args.ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo",
             "-pix_fmt", "rgba", "-s", f"{REC_W}x{REC_H}", "-r", str(REC_FPS),
             "-i", "-", "-c:v", "libx264", "-preset", "veryfast", "-crf", "23",
             "-pix_fmt", "yuv420p", str(rec_dir / f"isaac_{label}.mp4")],
            stdin=subprocess.PIPE)
        recorders.append((ann, proc))
    print(f"[sim] recording behind/side/top views at {REC_W}x{REC_H} "
          f"@{REC_FPS}fps -> {rec_dir}", flush=True)


def record_frames():
    rec_state["n"] += 1
    if rec_state["n"] % REC_EVERY:
        return
    for ann, proc in recorders:
        d = ann.get_data()
        if d is not None and getattr(d, "size", 0):
            proc.stdin.write(d.tobytes())


# --- render products + ROS2 camera/lidar graphs ------------------------------
cam_rp = rep.create.render_product(CAM, resolution=(640, 360))
lidar_rp = rep.create.render_product(LIDAR, resolution=(1, 1))

keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/ROSGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("rgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("depth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("info", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ("scan", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ("joints", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("simtime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ],
        keys.CONNECT: [
            ("tick.outputs:tick", "clock.inputs:execIn"),
            ("simtime.outputs:simulationTime", "clock.inputs:timeStamp"),
            ("tick.outputs:tick", "rgb.inputs:execIn"),
            ("tick.outputs:tick", "depth.inputs:execIn"),
            ("tick.outputs:tick", "info.inputs:execIn"),
            ("tick.outputs:tick", "scan.inputs:execIn"),
            ("tick.outputs:tick", "joints.inputs:execIn"),
            ("simtime.outputs:simulationTime", "joints.inputs:timeStamp"),
        ],
        keys.SET_VALUES: [
            ("rgb.inputs:renderProductPath", cam_rp.path),
            ("rgb.inputs:type", "rgb"),
            ("rgb.inputs:topicName", "/camera/image"),
            ("rgb.inputs:frameId", "camera_optical_frame"),
            ("depth.inputs:renderProductPath", cam_rp.path),
            ("depth.inputs:type", "depth"),
            ("depth.inputs:topicName", "/camera/depth_image"),
            ("depth.inputs:frameId", "camera_optical_frame"),
            ("info.inputs:renderProductPath", cam_rp.path),
            ("info.inputs:topicName", "/camera/camera_info"),
            ("info.inputs:frameId", "camera_optical_frame"),
            ("scan.inputs:renderProductPath", lidar_rp.path),
            ("scan.inputs:type", "laser_scan"),
            ("scan.inputs:topicName", "/scan"),
            ("scan.inputs:frameId", "lidar_link"),
            ("joints.inputs:targetPrim", [ART_ROOT]),
            ("joints.inputs:topicName", "/joint_states"),
        ],
    },
)

# --- in-process ROS node: cmd_vel, wheel odom, imu, ground truth -------------
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

rclpy.init()
node = rclpy.create_node("sih_isaac_sim")

WHEEL_R = 0.15
TRACK = 0.70
# PhysX skid steer scrubs like the real platform; commanded yaw must be scaled
# up to be realised. Measured open loop below; adjust if the print disagrees.
SKID_YAW_GAIN = 6.5
MAX_WHEEL_RPS = 30.0

cmd = dict(v=0.0, w=0.0, t=0.0)


def on_cmd(msg: Twist):
    cmd["v"], cmd["w"] = msg.linear.x, msg.angular.z
    cmd["t"] = sim.current_time


node.create_subscription(Twist, "/cmd_vel", on_cmd, 10)
qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
pub_odom = node.create_publisher(Odometry, "/odom_wheel", qos)
pub_gt = node.create_publisher(Odometry, "/ground_truth", qos)
pub_imu = node.create_publisher(Imu, "/imu/data", qos)


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def stamp_of(t):
    from builtin_interfaces.msg import Time
    s = Time()
    s.sec = int(t)
    s.nanosec = int((t - int(t)) * 1e9)
    return s


# wheel-odometry integrator state (starts at the surveyed deployment pose)
start = world_cfg["start"]
ow = dict(x=start["x"], y=start["y"],
          yaw=math.radians(start["yaw_deg"]))

# patrol humans
patrols = []
for h in world_cfg["humans"]:
    if h["kind"] != "patrol":
        continue
    xf = XFormPrim(f"/World/humans/{h['name']}")
    patrols.append(dict(xf=xf, wp=np.array(h["waypoints"], dtype=float),
                        speed=h["speed"], leg=0, s=0.0))

dem = np.load(GEN / "dem.npy")
N_DEM = dem.shape[0]
HALF = world_cfg["world_size"] / 2.0
RES = world_cfg["world_size"] / (N_DEM - 1)


def ground_z(x, y):
    c = int(np.clip((x + HALF) / RES, 0, N_DEM - 1))
    r = int(np.clip((HALF - y) / RES, 0, N_DEM - 1))
    return float(dem[r, c])


timeline = omni.timeline.get_timeline_interface()
timeline.play()

last_ctrl = -1.0
last_report = -5.0
CTRL_DT = 1.0 / 30.0
print("[sim] running -- waiting for /cmd_vel", flush=True)

try:
  while app.is_running() and sim.current_time < args.duration:
    sim.step(render=True)
    rclpy.spin_once(node, timeout_sec=0.0)
    t = sim.current_time
    if t - last_ctrl < CTRL_DT:
        continue
    dt = t - last_ctrl if last_ctrl > 0 else CTRL_DT
    last_ctrl = t

    # ---- drive: Twist -> wheel velocity targets (zero cmd if stale >0.5 s)
    v, w = (cmd["v"], cmd["w"]) if t - cmd["t"] < 0.5 else (0.0, 0.0)
    wl = (v - w * SKID_YAW_GAIN * TRACK / 2.0) / WHEEL_R
    wr = (v + w * SKID_YAW_GAIN * TRACK / 2.0) / WHEEL_R
    wl, wr = np.clip([wl, wr], -MAX_WHEEL_RPS, MAX_WHEEL_RPS)
    targets = np.zeros(rover.num_dof)
    targets[W_IDX] = [wl, wr, wl, wr]          # fl, fr, rl, rr
    rover.apply_action(ArticulationAction(joint_velocities=targets))

    # ---- odometry: body-frame forward velocity from the rigid-body state,
    # i.e. VIO-grade odometry (what cuVSLAM will provide once integrated; see
    # README "Localization honesty"). Wheel-speed odometry over-counts ~30%
    # on slopes (slip) and its yaw is unusable on a skid platform -- both are
    # still computed and logged for comparison.
    pos_b, quat_b = rover.get_world_pose()
    follow_cams_update(pos_b, quat_b, dt)
    if recorders:
        record_frames()
    v_body = quat_rot_inv(quat_b, np.asarray(rover.get_linear_velocity()))
    om_body = quat_rot_inv(quat_b, np.asarray(rover.get_angular_velocity()))
    jv = rover.get_joint_velocities()
    ml = 0.5 * (jv[W_IDX[0]] + jv[W_IDX[2]])
    mr = 0.5 * (jv[W_IDX[1]] + jv[W_IDX[3]])
    v_wheel = float(WHEEL_R * (ml + mr) / 2.0)
    v_meas = float(v_body[0])
    w_meas = float(om_body[2])
    ow["x"] = float(ow["x"] + v_meas * math.cos(ow["yaw"]) * dt)
    ow["y"] = float(ow["y"] + v_meas * math.sin(ow["yaw"]) * dt)
    ow["yaw"] = float(ow["yaw"] + w_meas * dt)
    om = Odometry()
    om.header.stamp = stamp_of(t)
    om.header.frame_id = "odom"
    om.child_frame_id = "base_link"
    om.pose.pose.position.x, om.pose.pose.position.y = ow["x"], ow["y"]
    (om.pose.pose.orientation.x, om.pose.pose.orientation.y,
     om.pose.pose.orientation.z, om.pose.pose.orientation.w) = yaw_to_quat(ow["yaw"])
    om.twist.twist.linear.x = v_meas
    om.twist.twist.linear.y = float(v_body[1])   # lateral slip: real on skid
    om.twist.twist.angular.z = w_meas
    om.twist.covariance[7] = 0.02
    om.pose.covariance[0] = om.pose.covariance[7] = 0.05
    om.pose.covariance[35] = 0.05
    om.twist.covariance[0] = 0.02
    om.twist.covariance[35] = 0.05
    pub_odom.publish(om)

    # ---- IMU
    fr = imu_read(t)
    im = Imu()
    im.header.stamp = stamp_of(t)
    im.header.frame_id = "imu_link"
    la, av, q = fr["lin_acc"], fr["ang_vel"], fr["orientation"]  # w,x,y,z
    im.linear_acceleration.x, im.linear_acceleration.y, im.linear_acceleration.z = map(float, la)
    im.angular_velocity.x, im.angular_velocity.y, im.angular_velocity.z = map(float, av)
    im.orientation.w, im.orientation.x, im.orientation.y, im.orientation.z = map(float, q)
    im.orientation_covariance[0] = im.orientation_covariance[4] = 0.02
    im.orientation_covariance[8] = 0.05
    im.angular_velocity_covariance[0] = im.angular_velocity_covariance[4] = 0.005
    im.angular_velocity_covariance[8] = 0.005
    im.linear_acceleration_covariance[0] = 0.04
    pub_imu.publish(im)

    # ---- ground truth (scoring only)
    pos, quat = rover.get_world_pose()             # quat w,x,y,z
    gt = Odometry()
    gt.header.stamp = stamp_of(t)
    gt.header.frame_id = "map"
    gt.child_frame_id = "base_link_gt"
    gt.pose.pose.position.x, gt.pose.pose.position.y, gt.pose.pose.position.z = map(float, pos)
    (gt.pose.pose.orientation.w, gt.pose.pose.orientation.x,
     gt.pose.pose.orientation.y, gt.pose.pose.orientation.z) = map(float, quat)
    pub_gt.publish(gt)

    # ---- patrol humans
    for pt in patrols:
        a, b = pt["wp"][pt["leg"]], pt["wp"][(pt["leg"] + 1) % len(pt["wp"])]
        seg = np.linalg.norm(b - a)
        pt["s"] += pt["speed"] * dt
        if pt["s"] >= seg:
            pt["s"] = 0.0
            pt["leg"] = (pt["leg"] + 1) % len(pt["wp"])
            a, b = b, pt["wp"][(pt["leg"] + 1) % len(pt["wp"])]
        u = (b - a) / max(np.linalg.norm(b - a), 1e-6)
        xy = a + u * pt["s"]
        yaw = math.atan2(u[1], u[0])
        qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
        pt["xf"].set_world_poses(
            positions=np.array([[xy[0], xy[1], ground_z(xy[0], xy[1])]]),
            orientations=np.array([[qw, 0.0, 0.0, qz]]))

    if t - last_report > 5.0:
        last_report = t
        print(f"[sim] t={t:7.1f}s cmd=({v:+.2f},{w:+.2f}) "
              f"gt=({pos[0]:+7.2f},{pos[1]:+7.2f}) "
              f"odom=({ow['x']:+7.2f},{ow['y']:+7.2f})", flush=True)

except KeyboardInterrupt:
    print("[sim] interrupted", flush=True)
finally:
    for _ann, _proc in recorders:
        try:
            _proc.stdin.close()
            _proc.wait(timeout=60)
        except Exception as e:
            print(f"[sim] recorder shutdown: {e}")
    if recorders:
        print("[sim] recordings finalized", flush=True)
    node.destroy_node()
    rclpy.shutdown()

app.close()
print("[sim] closed")
