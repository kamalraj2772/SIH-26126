const {
  Document, Packer, Paragraph, TextRun, HeadingLevel, AlignmentType,
  Table, TableRow, TableCell, WidthType, BorderStyle, ShadingType,
  LevelFormat, PageBreak,
} = require("docx");
const fs = require("fs");

const MONO = "Consolas";
const ACCENT = "1F4E79";
const CODEBG = "F2F2F2";
const HDRBG = "DCE6F1";

const p = (text, opts = {}) =>
  new Paragraph({
    spacing: { after: 120 },
    ...opts.para,
    children: [new TextRun({ text, size: 22, ...opts.run })],
  });

const rich = (runs, opts = {}) =>
  new Paragraph({
    spacing: { after: 120 },
    ...opts,
    children: runs.map(r => new TextRun({ size: 22, ...r })),
  });

const h1 = t => new Paragraph({ heading: HeadingLevel.HEADING_1, spacing: { before: 320, after: 160 }, children: [new TextRun({ text: t, size: 30, bold: true, color: ACCENT })] });
const h2 = t => new Paragraph({ heading: HeadingLevel.HEADING_2, spacing: { before: 260, after: 120 }, children: [new TextRun({ text: t, size: 26, bold: true, color: ACCENT })] });
const h3 = t => new Paragraph({ heading: HeadingLevel.HEADING_3, spacing: { before: 200, after: 100 }, children: [new TextRun({ text: t, size: 23, bold: true, color: "2E74B5" })] });

const bullet = (text, level = 0) =>
  new Paragraph({
    numbering: { reference: "bullets", level },
    spacing: { after: 80 },
    children: [new TextRun({ text, size: 22 })],
  });
const bulletRich = (runs, level = 0) =>
  new Paragraph({
    numbering: { reference: "bullets", level },
    spacing: { after: 80 },
    children: runs.map(r => new TextRun({ size: 22, ...r })),
  });

const code = lines =>
  lines.map((l, i) =>
    new Paragraph({
      shading: { type: ShadingType.CLEAR, fill: CODEBG },
      spacing: { after: i === lines.length - 1 ? 160 : 0 },
      indent: { left: 240, right: 240 },
      children: [new TextRun({ text: l === "" ? " " : l, font: MONO, size: 18 })],
    })
  );

const NOBORD = { style: BorderStyle.SINGLE, size: 4, color: "BFBFBF" };
function table(headers, rows, widths) {
  const total = widths.reduce((a, b) => a + b, 0);
  const mk = (cells, hdr) =>
    new TableRow({
      tableHeader: hdr,
      children: cells.map((c, i) =>
        new TableCell({
          width: { size: widths[i], type: WidthType.DXA },
          shading: hdr ? { type: ShadingType.CLEAR, fill: HDRBG } : undefined,
          margins: { top: 60, bottom: 60, left: 100, right: 100 },
          borders: { top: NOBORD, bottom: NOBORD, left: NOBORD, right: NOBORD },
          children: [new Paragraph({
            children: [new TextRun({ text: String(c), size: 20, bold: hdr, font: hdr ? undefined : (String(c).match(/^[/a-z_0-9.]+$/) && String(c).includes("_") ? MONO : undefined) })],
          })],
        })
      ),
    });
  return new Table({
    width: { size: total, type: WidthType.DXA },
    columnWidths: widths,
    rows: [mk(headers, true), ...rows.map(r => mk(r, false))],
  });
}

const children = [];

// ---------------- Title ----------------
children.push(
  new Paragraph({ spacing: { before: 1200, after: 200 }, alignment: AlignmentType.CENTER,
    children: [new TextRun({ text: "SIH26126 — GPS-Denied UGV Navigation", size: 48, bold: true, color: ACCENT })] }),
  new Paragraph({ spacing: { after: 160 }, alignment: AlignmentType.CENTER,
    children: [new TextRun({ text: "Isaac Sim 6.0.1 + ROS 2 Jazzy + Nav2 — Complete System Report", size: 30, color: "444444" })] }),
  new Paragraph({ spacing: { after: 60 }, alignment: AlignmentType.CENTER,
    children: [new TextRun({ text: "Workspace: ~/sih_ws  ·  Package: src/sih_isaac  ·  Commit 54c211f", size: 22, font: MONO })] }),
  new Paragraph({ spacing: { after: 400 }, alignment: AlignmentType.CENTER,
    children: [new TextRun({ text: "27 September 2026 — RIG, KCT", size: 22, color: "666666" })] }),

  p("An operator hands the rover a UTM Zone 44N easting/northing. The rover plans over a georeferenced GeoTIFF prior, drives 123.5 m across unstructured terrain — up an 8° slope, through a lit tunnel bored in a 3.2 m berm wall, past drums, logs, boulders, trees and walking humans — and stops 0.23 m from the commanded coordinate. No GNSS receiver, no NavSatFix, no gps_* topic exists anywhere in the system.", { run: { italics: true } }),
  new Paragraph({ children: [new PageBreak()] }),
);

// ---------------- Contents ----------------
children.push(h1("Contents"));
[
  "1.  Measured results (committed run, seed 42)",
  "2.  Running it yourself — the three terminals",
  "3.  System architecture, start to end",
  "      3.1  The one-file world: a GeoTIFF as single source of truth",
  "      3.2  Georeferencing without a receiver",
  "      3.3  Scene construction (build_scene.py)",
  "      3.4  Simulation runtime (run_sim.py)",
  "      3.5  Localization — GPS-free, and what it took",
  "      3.6  Global planner — Smac A* over the GeoTIFF prior",
  "      3.7  Local planner — MPPI with live obstacle discovery",
  "      3.8  Mission flow, start to end",
  "      3.9  What you watch (RViz + Isaac)",
  "4.  Measured calibrations",
  "5.  File map",
  "6.  Known limitations and next steps",
].forEach(t => children.push(new Paragraph({
  spacing: { after: 60 },
  children: [new TextRun({ text: t, size: 22 })],
})));

children.push(h1("Data flow at a glance"));
children.push(...code([
  "  OFFLINE (world.py, once)",
  "    surveyed datum 13.1147N 80.1098E --pyproj--> UTM anchor (E0,N0)",
  "    analytic terrain + berm/tunnel/plateau --> 257x257 DEM",
  "    DEM --> prior_dem_utm44n.tif  (EPSG:32644, the single source of truth)",
  "              |                    |                       |",
  "              v                    v                       v",
  "        terrain mesh (USD)   map.pgm (slope>15 deg)   world.json (anchor)",
  "",
  "  ONLINE",
  "    operator UTM goal --(E,N)-(E0,N0)--> map goal --> NavigateToPose",
  "                                                          |",
  "    /prior_map (static layer) ---+                        v",
  "    RTX lidar /scan -------------+--> global costmap --> Smac 2D A* --> /plan",
  "                                 |                                        |",
  "                                 +--> local costmap  --> MPPI 15 Hz <-----+",
  "                                                            |",
  "                                                       /cmd_vel",
  "                                                            v",
  "                              skid-steer mixer (gain 6.5) --> 4 wheel drives",
  "                                                            |",
  "                                                         PhysX",
  "                                                            |",
  "        body vx,vy + IMU yaw --> EKF --> odom->base_link ----+  (loop closes)",
  "",
  "        /ground_truth ..... scoring only, never subscribed by the stack",
  "        NO NavSatFix, NO navsat_transform, NO gps_* anywhere",
]));
children.push(new Paragraph({ children: [new PageBreak()] }));

// ---------------- 1 Results ----------------
children.push(h1("1. Measured results (committed run, seed 42)"));
children.push(table(
  ["Metric", "Value"],
  [
    ["Commanded goal (operator input)", "UTM 44N  E 403562.52  N 1450024.12"],
    ["Goal in map frame", "(56.00, 34.00) m"],
    ["Initial distance to goal", "123.5 m"],
    ["Mission outcome", "GOAL REACHED (Nav2 status 4), through the tunnel"],
    ["Final ground-truth distance to goal", "0.23 m"],
    ["Localization error at mission end", "0.98 m"],
    ["Recovery behaviours triggered", "1"],
    ["Obstacle contacts", "0 observed (planner in control throughout)"],
    ["Mission duration", "~460 s simulated time"],
    ["UTM ↔ map round-trip error", "0.000 mm (pure anchor arithmetic)"],
  ],
  [4200, 5000]
));
children.push(p(""));
children.push(p("Ground truth is recorded by the simulator solely for scoring. Nothing in the navigation stack subscribes to it."));

// ---------------- 2 Quick start ----------------
children.push(h1("2. Running it yourself"));
children.push(p("Three terminals on the desktop (the RViz and Isaac windows need the display; over SSH, run terminal 1 with --headless and keep RViz on the desktop)."));

children.push(h3("Terminal 1 — the simulation"));
children.push(...code([
  "cd ~/sih_ws",
  "./run_isaac.sh              # Isaac Sim GUI window (first run builds the world, ~3 min)",
  "# or:  ./run_isaac.sh --headless    # no window, faster and more robust",
]));
children.push(p("Wait for the line \"[sim] running -- waiting for /cmd_vel\"."));

children.push(h3("Terminal 2 — navigation stack + RViz"));
children.push(...code([
  "source /opt/ros/jazzy/setup.bash",
  "source ~/sih_ws/install/setup.bash",
  "ros2 launch sih_isaac nav.launch.py            # rviz:=false to skip RViz",
]));
children.push(p("Wait until the lifecycle manager reports every node active (a few seconds)."));

children.push(h3("Terminal 3 — give it somewhere to go (your input)"));
children.push(...code([
  "source /opt/ros/jazzy/setup.bash && source ~/sih_ws/install/setup.bash",
  "",
  "# the committed mission goal:",
  "ros2 run sih_isaac utm_goal.py -e 403562.52 -n 1450024.12",
  "",
  "# any other UTM 44N coordinate on the 150 m site:",
  "ros2 run sih_isaac utm_goal.py -e 403520.00 -n 1450040.00",
  "",
  "# or in map-frame metres:",
  "ros2 run sih_isaac utm_goal.py --map-x 30 --map-y -20",
]));
children.push(p("Alternatively click \"2D Goal Pose\" in RViz. The CLI prints live progress in both frames (map metres and UTM easting/northing), distance remaining and recovery count, and announces GOAL REACHED at the end."));

children.push(h3("Useful extras"));
children.push(...code([
  "ros2 topic echo /ground_truth --once     # scoring reference (never used by the stack)",
  "ros2 run tf2_ros tf2_echo map base_link  # where the rover believes it is",
  "ros2 topic hz /scan /camera/image        # sensor rates",
  "./run_demo.sh                            # the original MuJoCo demo video (separate system)",
]));
children.push(p("To regenerate the world (new layout/seed): edit src/sih_isaac/sih_isaac/world.py, delete src/sih_isaac/usd/sih26126_world.usd, and re-run ./run_isaac.sh — it rebuilds automatically."));
children.push(new Paragraph({ children: [new PageBreak()] }));

// ---------------- 3 Architecture ----------------
children.push(h1("3. System architecture, start to end"));

children.push(h2("3.1 The one-file world: a GeoTIFF as single source of truth"));
children.push(p("Everything begins offline in world.py. It samples an analytic terrain function (shared with the Gazebo and MuJoCo variants of this project) into a 257×257 digital elevation model covering 150 m × 150 m, then bakes in the mission features: a 3.2 m earth berm spanning the entire site with a 5.6 m-wide passage carved at y = 25 (the tunnel), an 8° conical plateau the rover can climb, and the pre-existing 36° mound, ridge and ditch it must avoid."));
children.push(p("That DEM is written as a genuinely georeferenced GeoTIFF (EPSG:32644, WGS 84 / UTM zone 44N, 0.586 m/pixel, correct affine transform — it opens in QGIS). Three consumers then derive from this single file, which guarantees the simulated world and the planner's prior agree by construction:"));
children.push(bullet("The Isaac Sim terrain mesh (what the physics engine and sensors see),"));
children.push(bullet("The Nav2 static map (map.pgm/map.yaml): every cell whose slope exceeds 15° is marked occupied — berm flanks, tunnel walls, mound, ditch edges — while the tunnel bore, plateau and open field stay free,"));
children.push(bullet("The UTM anchor metadata used for goal conversion (world.json)."));
children.push(p("Deliberately absent from the prior: the 14 corridor obstacles (drums, crates, logs, boulders) and all 4 humans. They exist only in the simulated world, so the rover must discover them with its own sensors — exactly the unsurveyed-obstacle condition of the problem statement."));

children.push(h2("3.2 Georeferencing without a receiver"));
children.push(p("The only geodetic input in the whole system is a surveyed site datum: one latitude/longitude (13.1147 N, 80.1098 E, Avadi) fixed offline at survey time, like a benchmark pillar on a test range. pyproj converts it once, at world-generation time, to a UTM anchor (E₀, N₀). From then on the map frame is a local ENU grid whose origin is that anchor, and UTM ↔ map conversion is pure subtraction — measured round-trip error 0.000 mm. At runtime nothing imports pyproj and nothing measures latitude/longitude; the vehicle estimates its pose in the map frame and converts to UTM for reporting only."));

children.push(h2("3.3 Scene construction (build_scene.py, run once)"));
children.push(p("A standalone Isaac Sim script turns the generated data into a USD stage:"));
children.push(bullet("Terrain: a 66k-vertex mesh built from the DEM with an exact-triangle-mesh PhysX collider. Triangle winding is counter-clockwise seen from above — PhysX meshes are one-sided, and the first build had the rover fall straight through before this was fixed."));
children.push(bullet("The rover: the same URDF as the Gazebo phase (xacro-generated), imported with Isaac's 6.0 URDF-to-USD converter; four continuous wheel joints get velocity drives (damping 5000, max force 900)."));
children.push(bullet("Wheels get their own physics material (μ 0.45/0.40): with terrain-level friction a skid-steer of this geometry (0.8 m wheelbase vs 0.7 m track) achieved only ~6 % of commanded yaw. This value was calibrated, not guessed (scripts/calibrate_yaw.py)."));
children.push(bullet("Sensors: a camera on the URDF camera mount (102° HFOV, ZED-2i-like geometry; the quaternion mapping the USD camera's −Z view axis onto body +X is computed in the script), and an RTX lidar (2-D rotary, 3600 rays, created at runtime because a baked lidar prim crashed the renderer on stage reopen)."));
children.push(bullet("Environment dressing: the tunnel roof slab and three interior lights, orange drums, wooden crates, logs, squashed-sphere boulders, 40 trees, 60 rocks, and 4 humans — 3 official Isaac People characters streamed from NVIDIA's asset CDN plus 1 primitive stand-in, each with an invisible capsule collider; the two patrol humans are kinematic rigid bodies."));

children.push(h2("3.4 Simulation runtime (run_sim.py)"));
children.push(p("The runtime opens the stage, plays physics at 60 Hz with rendering at 30 Hz, and bridges to ROS 2 Jazzy two ways:"));
children.push(bullet("OmniGraph nodes (GPU path): /clock, RGB image, 32-bit depth image and CameraInfo (frame_id camera_optical_frame, honouring REP-103), /scan from the RTX lidar (frame lidar_link), and /joint_states."));
children.push(bullet("An in-process rclpy node (control path, 30 Hz): subscribes /cmd_vel and converts Twist to four wheel velocity targets with the calibrated skid-yaw gain of 6.5; publishes odometry, IMU and /ground_truth; and walks the two patrol humans along their waypoint legs at 0.7–0.8 m/s, one of them crossing the rover's corridor repeatedly."));
children.push(p("A stale-command guard zeroes the wheels if no /cmd_vel arrives for 0.5 s, so a dead controller can never leave the rover driving blind."));

children.push(h2("3.5 Localization — GPS-free, and what it took"));
children.push(p("TF ownership is strict: exactly one publisher per edge, and Isaac Sim publishes no TF at all."));
children.push(table(
  ["TF edge", "Sole owner"],
  [
    ["map → odom", "static identity (default) — legitimate because odom is seeded at the surveyed pose; slam_toolbox optional via slam:=true (experimental)"],
    ["odom → base_link", "robot_localization EKF, 30 Hz"],
    ["base_link → lidar_link", "one static_transform_publisher (mast top, +0.45 m)"],
    ["base_link → camera/imu/wheels", "robot_state_publisher from the URDF"],
  ],
  [2900, 6300]
));
children.push(p(""));
children.push(p("The EKF fuses body-frame vx and vy plus IMU absolute yaw and yaw rate, and is seeded with the surveyed deployment pose (−62, 16, 8°) so the odom frame coincides with the map frame at t0. The odometry and IMU are synthesized in run_sim.py from the simulator's rigid-body state — i.e. idealized VIO/AHRS-grade sensing. This is a documented, measured decision rather than a shortcut:"));
children.push(bullet("Wheel-difference yaw is meaningless on a skid platform — the scrub ratio varied so much with speed, load and slope that fusing it produced 20–70 m localization collapses and aborted missions."));
children.push(bullet("Wheel distance over-counted ~30 % on slope climbs (slip), putting the estimate 7–8 m ahead of truth at the tunnel mouth."));
children.push(bullet("Gyro-only heading drifted ~13° over 45 m; lateral crabbing on side slopes is invisible to encoders entirely. Each failure mode was isolated in its own mission attempt."));
children.push(p("The quantities actually fused are exactly what isaac_ros_visual_slam (cuVSLAM, Isaac ROS 4.6 — already installed on this machine) and a magnetometer AHRS measure on real hardware; swapping cuVSLAM's odometry in place of the synthesized topic is the designated next step. Raw wheel/IMU signals are still published for comparison. slam_toolbox (lidar SLAM) remains available behind slam:=true but degraded the pose on this terrain and is documented as experimental."));

children.push(h2("3.6 Global planner — Smac A* over the GeoTIFF prior"));
children.push(p("The global costmap (map frame, 0.29 m resolution) stacks three layers: the static layer subscribing to /prior_map (the GeoTIFF-derived slope map served by nav2_map_server), an obstacle layer marking and clearing from the live lidar, and inflation (0.95 m, cost scaling 3.0) around everything. The Smac 2-D A* planner searches this costmap; with the berm walling off the whole site, the only free corridor across is the tunnel bore, and the committed run's plan crosses it at y ≈ 23.7."));
children.push(p("One tuning lesson is baked into the config: cost_travel_multiplier must stay at 1.0. At 2.0, the inflated-but-free cells inside the 5.6 m bore were double-costed and A* preferred squeezing around the berm's end — the mission that exposed this aborted at the north tip. The occupancy grid was then verified directly: the only fully-free crossing rows are y = 22.9–27.0, the bore."));

children.push(h2("3.7 Local planner — MPPI with live obstacle discovery"));
children.push(p("The controller server runs Nav2's MPPI (Model Predictive Path Integral) at 15 Hz: 1800 sampled trajectories, 56 steps of 67 ms each, DiffDrive motion model, vx up to 0.9 m/s. Its critics balance staying on the Smac path, goal progress, forward preference, and collision cost against a rolling 14 m × 14 m local costmap fed only by the lidar. This is where the unsurveyed world gets handled: drums, logs, boulders and the walking humans appear as scan returns, inflate, and MPPI steers around them — including the patrol human that crosses the corridor. Behaviour servers (spin / backup / wait) recover when progress stalls; the committed run needed exactly one recovery."));

children.push(h2("3.8 Mission flow, start to end"));
children.push(bullet("1.  utm_goal.py reads the operator's easting/northing, subtracts the baked anchor, validates the goal is on-site, and sends a NavigateToPose action goal in the map frame."));
children.push(bullet("2.  bt_navigator ticks its behaviour tree: ComputePathToPose → Smac A* returns the 193-pose path through the tunnel."));
children.push(bullet("3.  FollowPath → MPPI turns the path plus the live local costmap into /cmd_vel at 15 Hz."));
children.push(bullet("4.  run_sim.py maps each Twist to four wheel velocity targets (skid-yaw gain 6.5); PhysX rolls the rover over the DEM terrain."));
children.push(bullet("5.  Sensors close the loop: odometry+IMU → EKF → odom→base_link; lidar → costmaps; camera → operator view."));
children.push(bullet("6.  Replanning at 1 Hz absorbs discovered obstacles; recoveries handle stalls; the goal checker declares success within 0.7 m."));
children.push(bullet("7.  Throughout, utm_goal.py reports pose in map metres and UTM, and /ground_truth silently records the truth for scoring."));

children.push(h2("3.9 What you watch (RViz + Isaac)"));
children.push(bullet("RViz: the GeoTIFF prior map, the global costmap, the rolling local costmap, the blue Smac A* global plan, the orange MPPI local plan with its candidate-trajectory fan, red lidar returns, EKF odometry arrows, the TF tree, the robot model and the live camera feed."));
children.push(bullet("Isaac Sim window (./run_isaac.sh without --headless): the photoreal world — textured terrain, the berm with its lit tunnel bore, obstacles, vegetation and the walking humans. Note: with both GUIs plus the stack on one RTX 4090 the sim drops to ~0.33× real time and missions run slower; headless simulation with RViz-only is the robust demo configuration."));

children.push(new Paragraph({ children: [new PageBreak()] }));

// ---------------- 4 calibration ----------------
children.push(h1("4. Measured calibrations"));
children.push(table(
  ["Parameter", "Value", "How it was measured"],
  [
    ["Wheel physics material μ (static/dynamic)", "0.45 / 0.40", "In-place spin achieved 6 % of commanded yaw at terrain friction; 0.45 restores control while tan 12° = 0.21 still climbs every route slope"],
    ["SKID_YAW_GAIN", "6.5", "calibrate_yaw.py: spin ratio 0.398 measured at gain 2.6 → 2.6/0.398"],
    ["Wheel drive damping / max force", "5000 / 900 N·m", "Wheels must track commanded velocity under scrub load"],
    ["Rolling efficiency", "0.95", "calibrate_yaw.py drive phase: 0.5706 m/s achieved for 0.6 commanded"],
    ["Nav2 transform_tolerance", "1.0 s (everywhere)", "Sim-time TF stamping races starved costmaps at defaults"],
    ["Smac cost_travel_multiplier", "1.0", "2.0 made A* dodge the inflated tunnel bore"],
    ["EKF inputs", "vx, vy, yaw, yaw rate, ax", "Each wheel-odometry channel removed after its measured failure (§3.5)"],
  ],
  [2900, 1900, 4400]
));

// ---------------- 5 files ----------------
children.push(h1("5. File map"));
children.push(table(
  ["Path", "Role"],
  [
    ["run_isaac.sh", "One-command simulation start (builds world + USD on first run)"],
    ["src/sih_isaac/sih_isaac/world.py", "DEM, GeoTIFF, occupancy map, obstacle/human layout → generated/"],
    ["src/sih_isaac/scripts/build_scene.py", "generated data → usd/sih26126_world.usd"],
    ["src/sih_isaac/scripts/run_sim.py", "Runtime: ROS 2 bridge, drive, odometry/IMU, ground truth, patrols"],
    ["src/sih_isaac/scripts/calibrate_yaw.py", "Measures achieved-vs-commanded yaw/speed ratios"],
    ["src/sih_isaac/sih_isaac/utm_goal.py", "Operator CLI: UTM goal → NavigateToPose, live progress in both frames"],
    ["src/sih_isaac/launch/nav.launch.py", "EKF, map anchor, map_server, Nav2 servers, RViz"],
    ["src/sih_isaac/config/", "ekf.yaml, nav2_params.yaml, slam_toolbox.yaml, sih.rviz"],
    ["src/sih_isaac/generated/ (gitignored)", "dem.npy, prior_dem_utm44n.tif, map.pgm/yaml, world.json"],
    ["src/sih_isaac/README.md", "Condensed version of this report, kept with the code"],
  ],
  [4100, 5100]
));

// ---------------- 6 next ----------------
children.push(h1("6. Known limitations and next steps"));
children.push(bullet("Localization sensing is idealized (VIO/AHRS-grade, synthesized from simulator state) — honest and documented, with cuVSLAM (Isaac ROS 4.6, installed) as the drop-in replacement: its RGBD mode needs tracking_mode:2, num_cameras:1, min_num_images:1, depth_camera_id:0, launched via a launch file (the bare binary fails on Bazel runfiles)."));
children.push(bullet("slam_toolbox is experimental here: use the sync node (the async variant has a nondeterministic activation race under sim time), and expect karto's corrections to degrade on this terrain."));
children.push(bullet("Full-GUI demos (Isaac window + RViz) run at ~0.33× real time on one GPU; headless + RViz is the robust configuration."));
children.push(bullet("nvblox (installed) can replace the 2-D lidar costmap with camera-based 3-D reconstruction for overhangs like the tunnel roof."));
children.push(bullet("Contact-force scoring and multi-goal waypoint missions (navigate_through_poses is already enabled in bt_navigator) are natural additions."));

const doc = new Document({

  numbering: {
    config: [{
      reference: "bullets",
      levels: [0, 1].map(l => ({
        level: l,
        format: LevelFormat.BULLET,
        text: l === 0 ? "•" : "◦",
        alignment: AlignmentType.LEFT,
        style: { paragraph: { indent: { left: 720 * (l + 1), hanging: 360 } } },
      })),
    }],
  },
  styles: { default: { document: { run: { font: "Calibri", size: 22 } } } },
  sections: [{
    properties: { page: { margin: { top: 1080, bottom: 1080, left: 1260, right: 1260 } } },
    children,
  }],
});

Packer.toBuffer(doc).then(buf => {
  fs.writeFileSync("/home/qbotix-rover/sih_ws/SIH26126_Isaac_System_Report.docx", buf);
  console.log("written", buf.length, "bytes");
});
