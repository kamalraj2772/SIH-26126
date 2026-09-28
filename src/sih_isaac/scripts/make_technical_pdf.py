"""Build SIH26126_QSLAM_Detailed_System_Description.pdf.

    cd src/sih_isaac/scripts
    ../../../.demoenv/bin/python techdoc_figs_data.py       # figures (data)
    ../../../.demoenv/bin/python techdoc_figs_diagrams.py   # figures (diagrams)
    ../../../.demoenv/bin/python make_technical_pdf.py

Part A (technical) explains every phase, algorithm, node and interface of
the QSLAM system as it exists in this workspace, labelling each part as
implemented (Isaac testbed / MuJoCo prototype / Gazebo Phase 1),
experimental, or QSLAM design. Part B explains the same system to a reader
with no robotics background. Numbers come from the repo's configs, generated
files and recorded telemetry (build/tech_doc_figs/data_stats.json).
"""
import json
import pathlib

from PIL import Image as PILImage
from reportlab.platypus import NextPageTemplate, PageBreak

import techdoc_pdf_lib as L
from techdoc_pdf_lib import (B, H1, H2, H3, P, TEXT_W, CondPageBreak,
                             KeepTogether, SetPart, Spacer, badge, callout,
                             eq, figure, mono, side_by_side, table)

WS = pathlib.Path("/home/qbotix-rover/sih_ws")
OUT = WS / "SIH26126_QSLAM_Detailed_System_Description.pdf"
REC = WS / "mission_recordings"
FIGS = WS / "build/tech_doc_figs"
STATS = json.loads((FIGS / "data_stats.json").read_text()) \
    if (FIGS / "data_stats.json").exists() else {}


def F(name):
    return L.fig_ref(name)


def m(text):
    return mono(text)


def stat(path, default="?", fmt="{:.2f}"):
    """Look up a dotted key in data_stats.json, formatted."""
    cur = STATS
    for k in path.split("."):
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    try:
        return fmt.format(cur)
    except (TypeError, ValueError):
        return str(cur)


# --------------------------------------------------------------- images --
def prepare_images():
    """Crop the RViz screenshot and compose a 2×2 still panel."""
    FIGS.mkdir(parents=True, exist_ok=True)
    run = REC / "run_1080p"
    rv = PILImage.open(run / "still_rviz.jpg").convert("RGB")
    rv.crop((630, 150, 1905, 1045)).save(FIGS / "img_rviz_view.jpg",
                                          quality=92)
    tiles = [PILImage.open(run / n).convert("RGB") for n in
             ("still_isaac_behind.jpg", "still_isaac_side.jpg",
              "still_isaac_top.jpg")]
    tiles.append(PILImage.open(FIGS / "img_rviz_view.jpg"))
    W, H = 960, 540
    sheet = PILImage.new("RGB", (2 * W + 12, 2 * H + 12), "white")
    for i, im in enumerate(tiles):
        im = im.copy()
        im.thumbnail((W, H * 2))
        if im.height > H:
            top = (im.height - H) // 2
            im = im.crop((0, top, im.width, top + H))
        x = (i % 2) * (W + 12) + (W - im.width) // 2
        y = (i // 2) * (H + 12)
        sheet.paste(im, (x, y))
    sheet.save(FIGS / "img_stills.jpg", quality=90)


# ---------------------------------------------------------------- cover --
def cover():
    from reportlab.lib.styles import ParagraphStyle
    t1 = ParagraphStyle("t1", fontName="Lato-Bold", fontSize=34, leading=38,
                        textColor=L.NAVY)
    t2 = ParagraphStyle("t2", fontName="Lato", fontSize=15.5, leading=20,
                        textColor=L.INK)
    t3 = ParagraphStyle("t3", fontName="Lato-Semibold", fontSize=10.5,
                        leading=14, textColor=L.ORANGE)
    s = [Spacer(1, 26),
         P("SIH 2026  ·  PROBLEM STATEMENT SIH26126", t3),
         Spacer(1, 6),
         P("QSLAM", t1),
         P("Vision-based, GPS-denied autonomous navigation for an unmanned "
           "ground vehicle in unstructured outdoor terrain", t2),
         Spacer(1, 6),
         P("<b>Detailed system description</b> — every phase, algorithm, "
           "ROS 2 node, map format and design decision, with the measured "
           "results, followed by a plain-language explanation for readers "
           "new to robotics.",
           ParagraphStyle("t4", parent=L.body, fontSize=10.5, leading=15,
                          textColor=L.GREY)),
         Spacer(1, 10)]
    img = REC / "run_1080p/still_isaac_behind.jpg"
    s.append(L.Image(str(img), width=PAGE_INNER, height=PAGE_INNER * 1080 / 1920))
    s.append(P("The rover approaching the 3.2 m earth berm and its tunnel "
               "bore in the Isaac Sim 6.0.1 proving ground — frame from a "
               "recorded GPS-free mission.",
               ParagraphStyle("cc", parent=L.caption, alignment=0)))
    s.append(Spacer(1, 8))
    ident = [
        ["Problem statement", "SIH26126 — “Vision Based Autonomous "
         "Navigation for Unmanned Ground Vehicle for Outdoor environment”"],
        ["Theme · category", "Smart Automation · Software"],
        ["Team", "QSLAM (Team ID 14)"],
        ["System", "ROS 2 Jazzy · Nav2 · robot_localization EKF · "
         "Isaac Sim 6.0.1 · MuJoCo · Gazebo Harmonic"],
        ["GNSS / GPS used", "None, at any stage — no receiver, no NavSatFix, "
         "no navsat_transform"],
        ["Document", "Version 1.0 · 28 September 2026 · generated by "
         "src/sih_isaac/scripts/make_technical_pdf.py"],
    ]
    s.append(table(ident, [118, PAGE_INNER - 118], header=False, size=9,
                   first_col_bold=True))
    s.append(Spacer(1, 10))
    s.append(table([["How to read this document"],
                    ["<b>Part A — Technical description</b> (Sections 1–17): "
                     "architecture, each development and mission phase, and "
                     "how every algorithm works — with equations, "
                     "flowcharts, node graphs and the EKF estimation "
                     "graphs from recorded runs.<br/>"
                     "<b>Part B — QSLAM for everyone</b> (last five pages): "
                     "what the system does, whether it uses GPS, whether it "
                     "is online or offline, and what has been proven — no "
                     "background needed."]],
                   [PAGE_INNER], header=True, size=9, zebra=False))
    return s


PAGE_INNER = L.PAGE_W - 80


# ------------------------------------------------------------- contents --
def contents():
    s = [P("Contents", L.h1_plain), L.toc(), Spacer(1, 6)]
    legend = [
        ["Status label", "Meaning in this document"],
        [badge("impl"), "Runs in this workspace in the Isaac Sim 6.0.1 "
         "integration testbed (src/sih_isaac) and was measured."],
        [badge("proto"), "Runs in the MuJoCo end-to-end prototype "
         "(src/sih_demo)."],
        [badge("gz"), "Gazebo Harmonic world, robot, sensors and bridge "
         "(src/sih_sim, src/sih_description)."],
        [badge("exp"), "Installed or wired but not part of the measured "
         "configuration (cuVSLAM, slam_toolbox)."],
        [badge("design"), "Part of the registered QSLAM design; specified "
         "here, not yet in the repository."],
    ]
    s.append(table(legend, [150, TEXT_W - 150], size=8.3,
                   cap="Status labels used throughout Part A."))
    s.append(callout([
        P("<b>Three answers up front.</b>", L.body_tight),
        B("<b>Does QSLAM use GPS?</b> No. There is no GNSS receiver in the "
          "system at all. Position comes from a surveyed start point, the "
          "vehicle's own motion sensing (visual-inertial odometry + IMU "
          "heading) and a preloaded georeferenced map."),
        B("<b>Online or offline?</b> Fully offline with respect to "
          "networks: no internet, cloud or radio link is needed. Maps are "
          "prepared once before the mission; during the mission everything "
          "runs on the vehicle's own computer in real time."),
        B("<b>What is proven?</b> In the Isaac Sim testbed the rover drove "
          "to UTM goals through a tunnel and a corridor of unsurveyed "
          "obstacles and walking people, with 0.98–1.42 m localization error "
          "at the end of 123–295 m "
          "missions without GPS (Section 16)."),
    ], kind="ok"))
    return s


# =================================================================== PART A
def part_a():
    s = [SetPart("Part A — Technical description")]

    # ------------------------------------------------------------ 1 ------
    s += [H1("1  Problem, requirements and design principles")]
    s.append(P(
        "SIH26126 asks for a software module that drives an unmanned ground "
        "vehicle (UGV) to a goal across unstructured outdoor terrain using "
        "vision-based navigation. The operator specifies the destination as "
        "a <b>UTM coordinate</b> (easting/northing), the terrain contains "
        "slopes, ditches, vegetation and man-made obstacles, and the "
        "vehicle must not rely on satellite positioning. GNSS is exactly "
        "what disappears where UGVs are most needed: in tunnels, under "
        "canopy, beside tall structures, and wherever signals are jammed or "
        "spoofed. QSLAM treats that as a design rule rather than a failure "
        "mode: <b>there is no GNSS receiver anywhere in the software</b>, so "
        "the system cannot silently fall back on one."))
    s.append(table([
        ["Requirement", "Design decision in QSLAM", "Where"],
        ["Goal given in UTM, no GNSS", "Surveyed site datum converted to a "
         "UTM anchor offline (pyproj); at run time the goal is anchor-"
         "subtracted — pure arithmetic, nothing measured", "§5, utm_goal.py"],
        ["Unstructured terrain (slopes, ditch, berm, tunnel)", "Prior "
         "digital elevation model stored as a GeoTIFF; slope > 15° marked "
         "non-traversable; global route planned on it", "§6–8"],
        ["Unknown and moving obstacles (drums, logs, people)", "Live lidar "
         "obstacle layer in rolling costmaps; MPPI local controller "
         "re-optimises 15 times per second", "§9, §11"],
        ["Reliable pose without GPS on a skid-steer", "EKF fusing "
         "VIO-grade body velocity with absolute IMU/AHRS heading — chosen "
         "after wheel odometry failed in measurement", "§13"],
        ["Vision-based localization", "Visual SLAM front-end (prototype), "
         "cuVSLAM stereo VIO (installed), map anchoring to satellite/DEM "
         "(QSLAM design)", "§14, §15"],
        ["Reproducible validation", "Isaac Sim world generated from the same "
         "GeoTIFF the planner reads; ground truth logged only for scoring",
         "§7, §16"],
        ["Transferable to hardware", "Standard ROS 2 Jazzy + Nav2 "
         "interfaces; Jetson AGX Orin as target computer", "§4, §17"],
    ], [128, 285, 78], size=8.1,
        cap="Requirements of the problem statement and how QSLAM meets them."))
    s.append(H2("1.1  Five engineering rules"))
    s.append(P(
        "A previous attempt at this problem lost time to wiring mistakes "
        "that looked like algorithm failures. The five rules below are "
        "enforced in every launch and configuration file of this workspace:"))
    for r in [
        "<b>No GNSS in the graph</b> — no <i>NavSatFix</i> subscriber, no "
        "<i>navsat_transform_node</i>, no gps_* topic, parameter or script.",
        "<b>REP-103 camera frames</b> — optical frame (z forward, x right, y "
        "down) separate from the body frame (x forward, y left, z up), "
        "joined by a static transform; the frame_id actually published is "
        "verified, not assumed.",
        "<b>Exactly one publisher per TF edge</b> — every non-owner node has "
        "TF publishing disabled (Section 4.3).",
        "<b>Never set odom_frame_id on the rtabmap node</b> — it would read "
        "odometry from TF and silently ignore the visual-odometry topic.",
        "<b>Build products are never committed</b> — build/, install/, "
        "log/, generated worlds and model weights are git-ignored.",
    ]:
        s.append(B(r))

    # ------------------------------------------------------------ 2 ------
    s += [CondPageBreak(170), H1("2  The system at a glance")]
    s.append(P(
        "QSLAM splits cleanly into what happens <b>once, before the "
        "mission</b> (offline preparation of maps and the georeference) and "
        "what happens <b>continuously during the mission</b> (onboard "
        "sensing, estimation, planning and control). "
        f"{F('context_offline_online')} shows the split and the three "
        "things the system deliberately does not use."))
    s.append(figure("context_offline_online",
                    "System context. Left: offline preparation, run once. "
                    "Right: the onboard real-time loop. No GNSS receiver, "
                    "internet, cloud service or mobile network is used at "
                    "any point.", width=TEXT_W * 0.9))
    s.append(table([
        ["Aspect", "QSLAM (this workspace)"],
        ["Vehicle", "4-wheel skid-steer UGV, 60 kg, wheelbase 0.80 m, track "
         "0.70 m, wheel radius 0.15 m, chassis 0.70 × 0.50 × 0.25 m"],
        ["Sensors", "RGB-D camera (ZED-class, 640 × 360 in Isaac), mast-top "
         "RTX lidar at 0.45 m (2-D scan), IMU; wheel joint states"],
        ["Localization", "robot_localization EKF (30 Hz, 2-D): body "
         "velocity + absolute heading; surveyed start pose. cuVSLAM "
         "installed as the visual odometry source"],
        ["Maps", "Prior DEM as GeoTIFF (EPSG:32644, 257 × 257 px, 0.586 "
         "m/px) → slope-derived occupancy map; live lidar costmaps"],
        ["Global planner", "Nav2 Smac Planner 2D (A* on the global costmap), "
         "replanned at 1 Hz"],
        ["Local planner", "Nav2 MPPI controller, 1800 samples × 56 steps "
         "(3.75 s), 15 Hz"],
        ["Supervision", "Nav2 behaviour tree with recovery (clear costmaps, "
         "spin, wait, back up)"],
        ["Middleware / tools", "ROS 2 Jazzy, Nav2, Isaac Sim 6.0.1 "
         "(PhysX, RTX sensors, OmniGraph ROS 2 bridge), RViz 2, "
         "rasterio, pyproj, OpenCV, MuJoCo, Gazebo Harmonic"],
        ["Positioning infrastructure", "None: no GNSS, no beacons, no "
         "network. Only a surveyed site datum and preloaded map files"],
    ], [118, TEXT_W - 118], size=8.2, first_col_bold=True,
        cap="QSLAM system summary."))

    # ------------------------------------------------------------ 3 ------
    s += [CondPageBreak(170), H1("3  Development phases and their architecture")]
    s.append(P(
        "QSLAM was built in gated phases: each phase had an exit checklist "
        "and its numbers were measured before the next began. "
        f"{F('phases_roadmap')} shows the sequence and the status of each "
        "phase; the table summarises what each phase contains."))
    s.append(figure("phases_roadmap",
                    "Development phases. Green and blue phases run in this "
                    "workspace; grey is installed but not yet wired; orange "
                    "is the registered QSLAM design still to be built.", width=TEXT_W * 0.94))
    s.append(table([
        ["Phase", "Architecture and algorithms", "Status and evidence"],
        ["0 · MuJoCo end-to-end prototype (src/sih_demo)",
         "ZED 2i RGB-D rendering → ORB + PnP-RANSAC visual odometry "
         "(§14.2); depth → log-odds local costmap; GeoTIFF prior → "
         "slope-weighted A* (§10.4); DWA local planner (§11.2); "
         "UTM↔map via pyproj", badge("proto") + "<br/>Goal reached, 0 "
         "contacts with 21 unsurveyed obstacles, VSLAM drift 0.98–4.3 % of "
         "path across runs, UTM round trip 0.000 mm"],
        ["1 · Gazebo Harmonic world (src/sih_sim, sih_description)",
         "Seeded analytic terrain rasterised to a 513 × 513 heightmap; "
         "xacro rover (single source for robot_state_publisher and "
         "spawn); RGB-D 1280 × 720 @ 30 Hz, IMU 200 Hz, joint states 50 Hz; "
         "ros_gz_bridge with verified topic names and frame_id overrides",
         badge("gz") + "<br/>Sensors verified against REP-103; point-cloud "
         "frame found to be body-convention and bridged as camera_link"],
        ["2 · Isaac Sim navigation stack (src/sih_isaac)",
         "GeoTIFF as single source of truth → USD terrain + Nav2 map; "
         "EKF (VIO-grade velocity + absolute heading); Nav2 Smac 2D A* + "
         "MPPI + behaviour tree; UTM goal client; telemetry and recording",
         badge("impl") + "<br/>Goal reached through the tunnel in every "
         "recorded run; 0.98–1.42 m localization error"],
        ["2b · Visual odometry in the loop",
         "Isaac ROS cuVSLAM (stereo/RGB-D VIO) replacing the synthesised "
         "velocity input of the EKF; RTAB-Map / slam_toolbox for map→odom",
         badge("exp") + "<br/>cuVSLAM 15.0.0 installed and configured; "
         "slam_toolbox wired behind slam:=true. <i>Target: ≤ 2 % arrival "
         "error of distance</i>"],
        ["3 · Perception", "Semantic segmentation (SegFormer, "
         "RELLIS-3D + RUGD, TensorRT) + GPU 2.5-D elevation mapping → "
         "traversability and localizability cost layers",
         badge("design") + "<br/>sih_perception package scaffolded. "
         "<i>Target: ≥ 15 FPS on Jetson AGX Orin</i>"],
        ["4 · Robustness", "Map-anchored re-localization (BEV ↔ "
         "satellite/DEM), localization integrity monitor, degraded-mode "
         "supervisor, Collision Monitor", badge("design") + "<br/><i>"
         "Target: ≤ 5 m / 2° start alignment, ≥ 90 % mission success</i>"],
    ], [104, 232, 155], size=7.9,
        cap="Development phases: content, architecture and status."))
    s.append(P(
        "The prototype and the testbed share one world definition: "
        f"{m('sih_sim/terrain.py')} is the analytic terrain used by the "
        "Gazebo generator, the MuJoCo demo and the Isaac world, so results "
        "across phases describe the same site (seed 42). The remainder of "
        "Part A describes Phase 2 in depth, and Phases 0, 2b, 3 and 4 where "
        "their algorithms differ. After Phase 4 comes the hardware port (ZED "
        "X, AHRS, Jetson AGX Orin) and field trials, target ≤ 100 ms "
        "sensing-to-command latency; the targets are those of the QSLAM idea "
        "submission.", L.small))

    # ------------------------------------------------------------ mission
    s += [CondPageBreak(420), H2("3.1  Mission phases — what happens when the system runs")]
    s.append(P(
        f"{F('mission_flowchart')} follows one mission from survey to "
        "arrival. <b>M0 Preparation</b> (offline): world.py writes the "
        "GeoTIFF, the occupancy map and world.json; build_scene.py turns "
        "the same DEM into the Isaac USD stage. <b>M1 Bring-up</b>: the "
        "simulator starts publishing /clock and sensors; nav.launch.py "
        "starts the EKF seeded at the surveyed pose, the map server and "
        "Nav2 under a lifecycle manager. <b>M2 Goal</b>: the operator "
        "types a UTM easting/northing, which is anchor-subtracted, "
        "bounds-checked and sent as a NavigateToPose action. <b>M3 "
        "Planning</b>: Smac 2D computes the route (and recomputes it every "
        "second). <b>M4 Autonomy loop</b>: sense → localize → update "
        "costmaps → MPPI command, with recoveries when progress stalls. "
        "<b>M5 Arrival</b>: success inside 0.7 m, reported in map metres "
        "and UTM."))
    s.append(figure("mission_flowchart", "Mission flowchart from offline "
                    "preparation to arrival, including the decision points "
                    "and the recovery loop.", max_h=560))

    # ------------------------------------------------------------ 4 ------
    s += [CondPageBreak(170), H1("4  Software architecture")]
    s.append(H2("4.1  Layered view"))
    s.append(P(
        "The software is organised in five layers "
        f"({F('layered_architecture')}). Each layer only consumes the "
        "outputs of the layers below it through ROS 2 topics, actions and "
        "the TF tree, which is what makes the simulator replaceable by a "
        "real vehicle: only the platform layer changes."))
    s.append(figure("layered_architecture",
                    "Layered architecture with the implementation status of "
                    "every component.", width=TEXT_W * 0.88))
    s.append(H2("4.2  ROS 2 computation graph"))
    s.append(P(
        f"{F('ros_graph')} is the node/topic graph of the running testbed. "
        "Isaac Sim hosts one in-process ROS node (sih_isaac_sim) plus "
        "OmniGraph bridge nodes; everything else runs from nav.launch.py. "
        "Note that <b>/ground_truth is consumed only by the evaluation "
        "logger</b> — no navigation node subscribes to it."))
    s.append(table([
        ["Topic", "Type", "Publisher → subscribers", "Rate / frame"],
        ["/clock", "rosgraph_msgs/Clock", "Isaac (OmniGraph) → every node "
         "(use_sim_time)", "every sim step"],
        ["/scan", "sensor_msgs/LaserScan", "Isaac RTX lidar → global & local "
         "costmaps", "render rate · lidar_link"],
        ["/camera/image, /camera/depth_image, /camera/camera_info",
         "sensor_msgs/Image, CameraInfo", "Isaac → RViz (cuVSLAM input in "
         "Phase 2b)", "30 Hz · camera_optical_frame"],
        ["/imu/data", "sensor_msgs/Imu", "sih_isaac_sim → EKF", "30 Hz · "
         "imu_link"],
        ["/odom_wheel", "nav_msgs/Odometry", "sih_isaac_sim → EKF (twist "
         "only; no TF)", "30 Hz · odom→base_link"],
        ["/joint_states", "sensor_msgs/JointState", "Isaac → "
         "robot_state_publisher", "render rate"],
        ["/odometry/filtered", "nav_msgs/Odometry", "EKF → controller, BT "
         "navigator, logger", "30 Hz · odom"],
        ["/prior_map", "nav_msgs/OccupancyGrid", "map_server → global "
         "costmap static layer", "latched (transient local) · map"],
        ["/plan, /local_plan, /trajectories", "Path, MarkerArray",
         "planner / controller → RViz", "1 Hz / 15 Hz"],
        ["/cmd_vel", "geometry_msgs/Twist", "controller_server or "
         "behavior_server → sih_isaac_sim", "15 Hz (0.5 s watchdog)"],
        ["/ground_truth", "nav_msgs/Odometry", "sih_isaac_sim → "
         "mission_logger only", "30 Hz · map (evaluation)"],
        ["navigate_to_pose", "nav2_msgs/action/NavigateToPose",
         "utm_goal.py (client) → bt_navigator (server)", "per mission"],
    ], [118, 105, 170, 98], size=7.6, cap="Topics and actions of the "
        "Isaac testbed."))
    s.append(figure("ros_graph", "ROS 2 nodes (boxes) and topics/actions "
                    "(labels) of the Isaac testbed.", max_h=470))

    s.append(H2("4.3  Transform tree and ownership"))
    s.append(P(
        "In ROS the coordinate frames form a tree that every node queries. "
        "Two writers on one edge corrupt it silently, so each edge has "
        f"exactly one owner ({F('tf_tree')}). Isaac Sim publishes no TF. "
        "The static identity on map→odom is legitimate because the EKF is "
        "seeded with the surveyed deployment pose — odom coincides with map "
        "at t = 0 and only drift separates them afterwards; the map-"
        "anchoring module of the QSLAM design is exactly the component that "
        "will own this edge and remove that drift."))
    s.append(figure("tf_tree", "TF tree with the sole owner of each edge.", width=TEXT_W * 0.9))
    s.append(H2("4.4  Time, lifecycle and rates"))
    s.append(P(
        "Every node runs on simulation time (use_sim_time: true) driven by "
        "/clock, so the stack behaves identically at any real-time factor. "
        "Nav2 servers are lifecycle nodes brought up in order by "
        "lifecycle_manager_navigation (autostart). All Nav2 "
        "transform_tolerance values are 1.0 s because a full GUI plus RViz "
        "on one GPU drops the simulator to ≈0.33× real time. "
        f"{F('loop_rates')} lists the loop rates."))
    s.append(figure("loop_rates", "Loop rates of the running system "
                    "(simulation time), with the file that sets each.",
                    width=TEXT_W * 0.86))

    # ------------------------------------------------------------ 5 ------
    s += [CondPageBreak(170), H1("5  Georeferencing without GPS: UTM and the surveyed datum")]
    s.append(P(
        "The operator thinks in UTM; the robot plans in a local metric map "
        "frame. The bridge between them is a single <b>surveyed site "
        "datum</b> — one latitude/longitude fixed at survey time, like a "
        "benchmark pillar on a test range (Avadi, Tamil Nadu: 13.114700 °N, "
        "80.109800 °E). It is a constant of the mission, not a measurement "
        "made by the vehicle."))
    s.append(H3("Universal Transverse Mercator"))
    s.append(P(
        "UTM divides the Earth into 60 zones 6° wide and projects each with "
        "a transverse Mercator projection onto a metric grid. The zone "
        "number follows from the longitude λ:"))
    s.append(eq(r"\mathrm{zone} = \left\lfloor \frac{\lambda + 180^\circ}"
                r"{6^\circ} \right\rfloor + 1 = \left\lfloor \frac{260.11}{6}"
                r"\right\rfloor + 1 = 44"))
    s.append(P(
        "so the site is in <b>zone 44 N, EPSG:32644</b> (WGS 84 / UTM 44N, "
        "central meridian 81 °E). pyproj converts the datum once, offline, "
        f"to the anchor (E₀, N₀) = ({m('403 506.52')}, {m('1 449 990.12')}) "
        "m, which is written into world.json. At run time the conversion "
        "between UTM and the map frame is a translation:"))
    s.append(eq(r"x = E - E_0,\qquad y = N - N_0,\qquad "
                r"E = x + E_0,\qquad N = y + N_0"))
    s.append(P(
        "Over a 150 m site this is exact to within the projection's local "
        "distortions: the grid convergence here is γ ≈ (λ − λ₀) sin φ ≈ "
        "−0.89° × sin 13.1° ≈ −0.2°, absorbed by defining the map frame's "
        "+y axis as <i>grid</i> north; the point scale factor k ≈ 0.9997 "
        "changes a 150 m distance by ≈ 4.3 cm. The round trip "
        "lat/lon → UTM → map → UTM → lat/lon was measured at 0.000 mm."))
    s.append(figure("site_map", "The site in UTM zone 44N: surveyed datum "
                    "(map origin), start and default goal with their UTM "
                    "coordinates, and the 150 m × 150 m extent of the prior "
                    "DEM.", width=TEXT_W * 0.84))
    s.append(P(
        f"<b>utm_goal.py</b> implements the operator interface: it reads "
        f"the anchor from world.json, computes (x, y) = (E − E₀, N − N₀), "
        "rejects goals outside the ±75 m site, sends a NavigateToPose goal "
        "in frame <i>map</i>, and prints feedback in both frames (map metres "
        "and UTM) with the distance remaining and the recovery count. The "
        f"committed goal E {m('403 562.52')} N {m('1 450 024.12')} maps to "
        "(56.00, 34.00) m; the start (−62, 16) m is E 403 444.52, "
        "N 1 450 006.12. Nothing at run time imports pyproj, and nothing "
        "subscribes to a position fix."))

    # ------------------------------------------------------------ 6 ------
    s += [CondPageBreak(170), H1("6  The digital elevation model (DEM)")]
    s.append(P(
        "A DEM is a raster whose cells store ground elevation. It is the "
        "robot's prior knowledge of the terrain: which slopes are too steep, "
        "where the ditch and the berm are, where the tunnel passes. In the "
        "field it comes from a survey or a national product (the QSLAM "
        "design preloads ISRO Bhuvan / CartoDEM tiles); in this workspace it "
        "is synthesised analytically so that the simulator and the planner "
        "can be built from the same numbers."))
    s.append(H3("Terrain synthesis"))
    s.append(P(
        "The height at map coordinates (x, y) is a sum of a fractal "
        "micro-relief and parametric landforms:"))
    s.append(eq(r"h(x,y) = m_{\mathrm{tex}}\,h_{\mathrm{fBm}} + h_{\mathrm{ridge}}"
                r" + h_{\mathrm{mound}} + h_{\mathrm{ditch}} + "
                r"h_{\mathrm{berm}}\,(1 - c_{\mathrm{tunnel}}) + "
                r"h_{\mathrm{plateau}}"))
    s.append(P(
        "The micro-relief is fractional Brownian motion built from four "
        "octaves of hashed-lattice value noise n(·), each octave halving "
        "the amplitude and shrinking the lattice cell by 0.45:"))
    s.append(eq(r"h_{\mathrm{fBm}}(\mathbf{p}) = \sum_{k=0}^{3} a_k\, "
                r"n\!\left(\mathbf{p}/c_k\right),\quad a_k = 0.06 \cdot "
                r"0.5^{k}\ \mathrm{m},\quad c_k = 6 \cdot 0.45^{k}\ \mathrm{m}"))
    s.append(P(
        "Each noise value is bilinearly interpolated between four lattice "
        "values with the smoothstep weight S(t) = 3t² − 2t³, which also "
        "shapes every landform edge so the surface has no creases. The "
        "tunnel is carved by multiplying the berm by (1 − c), where c is a "
        "smooth box profile across y = 25 m; build_scene.py then roofs the "
        "carved bore (roof 3.0–3.6 m) in the USD stage."))
    s.append(table([
        ["Feature", "Parameters (map frame, metres)", "Role in the mission"],
        ["Earth berm + tunnel", "x 2–10, full site width, 3.2 m high; bore "
         "at y = 25, half-width 2.8 m", "Wall across the site: the tunnel is "
         "the only way through"],
        ["Plateau", "centre (−30, 20), top radius 6, 1.1 m, 8° flanks",
         "Drivable slope on the route"],
        ["Mound", "cone at (35, 30), 4 m, 35°", "Must be skirted (> 15°)"],
        ["Ridge", "x −40…−5, y −45…−25, 12° faces", "Ramp–plateau–ramp"],
        ["Ditch", "y = 5, x −12…12, 1.5 m deep, 2 m wide", "Negative obstacle "
         "only the prior can reveal"],
        ["Low-texture patch", "x 10–40, y −40…−10", "Suppressed relief: "
         "stress case for visual odometry"],
        ["Trees / rocks", "40 trunks at (−45, 45); 60 rocks at (55, −55)",
         "Scene clutter"],
        ["Corridor obstacles", "14 drums, crates, logs, boulders along the "
         "route", "<b>Absent from the prior</b> — found by lidar"],
        ["Humans", "2 standing, 2 patrols at 0.8 and 0.7 m/s crossing the "
         "corridor", "Dynamic obstacles, absent from the prior"],
    ], [98, 212, 181], size=7.8, cap="Landforms and objects of the "
        "seed-42 proving ground."))
    s.append(figure("dem_features", "The prior DEM (257 × 257 px over "
                    "150 m, 0.586 m/px) with every landform, the corridor "
                    "obstacles and humans that the prior deliberately does "
                    "not contain, start and goal.", width=TEXT_W * 0.8))
    s.append(figure("dem_oblique", "Shape of the terrain: berm with the "
                    "tunnel notch, the 35° mound and the 8° plateau.", width=TEXT_W * 0.74))
    s.append(figure("terrain_synthesis", "Terrain synthesis: (a) value-noise "
                    "octaves and their fBm sum; (b) smoothstep and the box "
                    "profile used for landform edges; (c) cross-sections "
                    "through the tunnel and through the solid berm.", width=TEXT_W * 0.94))

    # ------------------------------------------------------------ 7 ------
    s += [CondPageBreak(170), H1("7  GeoTIFF — the single source of truth")]
    s.append(P(
        "A GeoTIFF is an ordinary TIFF image plus GeoKeys that state its "
        "coordinate reference system and how pixel indices map to ground "
        f"coordinates. world.py writes the DEM as {m('prior_dem_utm44n.tif')}: "
        "one float32 band “terrain elevation above site datum (m)”, "
        "257 × 257 pixels, CRS EPSG:32644, deflate-compressed in 256 × 256 "
        "tiles, with the datum stored as tags (SITE_LAT/LON, "
        "ANCHOR_E/N). It opens in QGIS like any survey product."))
    s.append(H3("Affine geotransform"))
    s.append(P(
        "Pixel (col, row) maps to UTM through a six-parameter affine "
        "transform; for a north-up raster the rotation terms are zero:"))
    s.append(eq(r"E = c + a\,\mathrm{col},\qquad N = f + e\,\mathrm{row},"
                r"\qquad a = 0.5859\ \mathrm{m},\ e = -0.5859\ \mathrm{m}"))
    s.append(P(
        "with (c, f) = (403 431.225, 1 450 065.409) the upper-left corner of "
        "the upper-left pixel — half a pixel beyond the outermost sample "
        "(“pixel is area”). Row 0 is the north edge. The inverse used when "
        "sampling, with bilinear interpolation between the four "
        "surrounding pixels (fractional offsets f<sub>r</sub>, f<sub>c</sub>):"))
    s.append(eq(r"\mathrm{col} = \frac{E - c}{a} - \frac{1}{2},\quad "
                r"\mathrm{row} = \frac{f - N}{a} - \frac{1}{2},\quad "
                r"h = (1-f_r)\left[(1-f_c)h_{00} + f_c h_{01}\right] + "
                r"f_r\left[(1-f_c)h_{10} + f_c h_{11}\right]"))
    s.append(figure("geotiff_structure", "GeoTIFF structure: (a) the "
                    "raster corner, pixel centres and affine transform; (b) "
                    "the metadata read back from the file with rasterio.", width=TEXT_W * 0.92))
    s.append(H3("Why one file feeds everything"))
    s.append(P(
        "The same GeoTIFF produces (1) the Isaac Sim terrain mesh "
        "(build_scene.py), (2) the Nav2 occupancy map (§8) and (3) the UTM "
        "anchor. The simulated ground and the planner's belief therefore "
        "agree by construction, and a whole class of “map does not match "
        "the world” bugs cannot occur. In the MuJoCo prototype the planner "
        "reads the GeoTIFF back off disk through rasterio (class PriorDEM) "
        "rather than touching simulator state, so the file path is "
        "genuinely exercised. In the field the same interface accepts a "
        "surveyed or satellite-derived DEM."))

    # ------------------------------------------------------------ 8 ------
    s += [H1("8  Traversability and the prior occupancy map")]
    s.append(P(
        "Traversability is derived from the slope of the DEM. Using central "
        "differences at the DEM's own resolution (a coarser stencil smooths "
        "steep faces into something that looks drivable):"))
    s.append(eq(r"\theta(x,y) = \arctan\sqrt{\left(\frac{\partial h}"
                r"{\partial x}\right)^{2} + \left(\frac{\partial h}{\partial y}"
                r"\right)^{2}},\qquad \mathrm{cell\ blocked} \Leftrightarrow "
                r"\theta > 15^\circ"))
    s.append(P(
        "15° is a conservative limit for a 60 kg skid-steer on loose soil: "
        "the 8° plateau remains drivable, while the 35° mound, the ditch "
        "walls and the berm faces are blocked. The result is written as "
        f"{m('map.pgm')} (0 = occupied, 254 = free) with {m('map.yaml')}: "
        "mode trinary, resolution 0.585938 m, origin (−75.293, −75.293), "
        "occupied_thresh 0.65, free_thresh 0.25. map_server publishes it "
        "latched on /prior_map in frame map; 5.3 % of the site is blocked. "
        "Obstacles and people are "
        "deliberately absent — the rover must discover them."))
    s.append(figure("slope_occupancy", "From elevation to cost: (a) slope "
                    "with the 15° threshold; (b) the occupancy map Nav2 "
                    "loads; (c) the resulting global costmap after "
                    "inflation (reconstruction).", width=TEXT_W * 0.94))

    # ------------------------------------------------------------ 9 ------
    s += [CondPageBreak(170), H1("9  Costmaps")]
    s.append(P(
        "Nav2 plans on <b>layered costmaps</b>: 2-D grids whose cells hold a "
        "cost 0–254, assembled from plugin layers. QSLAM uses two."))
    s.append(table([
        ["", "Global costmap", "Local costmap"],
        ["Frame / window", "map, whole site", "odom, rolling 14 × 14 m around "
         "the robot"],
        ["Resolution", "0.293 m (half a DEM pixel)", "0.15 m"],
        ["Layers", "static (/prior_map) + obstacle (/scan) + inflation",
         "obstacle (/scan) + inflation"],
        ["Update / publish", "1 Hz / 1 Hz", "8 Hz / 5 Hz"],
        ["Used by", "Smac 2D global planner", "MPPI controller, behaviours"],
    ], [92, 200, 199], size=8.1, first_col_bold=True,
        cap="The two costmaps."))
    s.append(P(
        "<b>Obstacle layer.</b> Each /scan beam is ray-traced through the "
        "grid (Bresenham line from the lidar origin): cells along the beam "
        "are <i>cleared</i> up to 20 m, the end cell is <i>marked</i> "
        "lethal up to 18 m, for returns below 2.5 m height. Clearing is what "
        "lets a walking person leave no ghost behind. <b>Inflation "
        "layer.</b> The robot is modelled as a disc of radius r = 0.55 m; "
        "costs spread outward from every lethal cell as a function of the "
        "distance d:"))
    s.append(eq(r"c(d) = 252\,e^{-k\,(d - r)}\ \ \mathrm{for}\ r < d \leq R,"
                r"\quad c = 253\ \mathrm{for}\ d \leq r,\quad c = 254\ "
                r"\mathrm{at\ the\ obstacle},\quad k = 3.0,\ R = 0.95\ "
                r"\mathrm{m}"))
    s.append(P(
        "A cell of cost 253 means the robot's centre there would put the "
        "body in collision; the exponential band lets planners trade "
        f"clearance against distance ({F('inflation_curve')})."))
    s.append(figure("inflation_curve", "Inflation cost as a function of "
                    "distance from an obstacle for the configured robot "
                    "radius 0.55 m, cost_scaling_factor 3.0 and inflation "
                    "radius 0.95 m.", width=TEXT_W * 0.86))

    # ----------------------------------------------------------- 10 ------
    s += [H1("10  Global planner — A* (Nav2 Smac Planner 2D)")]
    s.append(H2("10.1  How A* works"))
    s.append(P(
        "A* finds the cheapest path on a graph — here, grid cells connected "
        "to their 8 neighbours. It keeps an <b>open set</b> ordered by"))
    s.append(eq(r"f(n) = g(n) + h(n)"))
    s.append(P(
        "where g(n) is the exact cost from the start to cell n and h(n) an "
        "estimate of the remaining cost. Each iteration pops the cell with "
        "the lowest f, moves it to the <b>closed set</b>, and relaxes its "
        "neighbours: if g(n) + cost(n, m) improves g(m), the neighbour's "
        "parent and priority are updated. When the goal is popped the path "
        "is read back through the parents. If h never over-estimates "
        "(admissible) and satisfies h(n) ≤ cost(n, m) + h(m) (consistent), "
        "the first path found is optimal and no cell is expanded twice. "
        "The Euclidean distance to the goal satisfies both."))
    s.append(H2("10.2  Smac Planner 2D in QSLAM"))
    s.append(P(
        "Smac 2D is Nav2's cost-aware A* on the global costmap (Moore "
        "8-neighbourhood, Euclidean heuristic). The cost of stepping to a "
        "neighbour grows with that cell's costmap value c ∈ [0, 252]:"))
    s.append(eq(r"\mathrm{cost}(n, m) = \ell_{nm}\left(1 + \mu\,"
                r"\frac{c(m)}{252}\right),\qquad \ell_{nm} \in \{1, \sqrt{2}"
                r"\},\qquad \mu = \mathtt{cost\_travel\_multiplier} = 1.0"))
    s.append(P(
        "Lethal and inscribed cells (≥ 253) are not expanded. Configuration: "
        "tolerance 0.5 m, allow_unknown true, max_iterations 10⁶, "
        "max_planning_time 3.0 s, no costmap down-sampling, final approach "
        "orientation kept. The behaviour tree re-plans at 1 Hz, so newly "
        "discovered obstacles in the global costmap re-route the path "
        "within a second."))
    s.append(callout(
        "<b>The tunnel lesson (μ = 1.0).</b> The 5.6 m bore leaves only a "
        "narrow lane free of inscribed cost, and every cell in it carries "
        "inflation cost that μ multiplies along the whole bore. During "
        "tuning on the live stack, values above 1.0 made the planner refuse "
        "the tunnel, so μ is pinned at 1.0. The offline re-computation in "
        f"{F('astar_search')} uses only the static prior: there, because "
        "the berm spans the entire site, μ = 1, 2 and 4 all return the same "
        "125.8 m tunnel route (23 007 expansions at μ = 1; straight line "
        "119.4 m) — the refusal therefore came from live-costmap "
        "conditions this reconstruction does not include.", kind="warn"))
    s.append(figure("astar_search", "A* on the reconstructed global costmap "
                    "(textbook implementation of the Smac 2D cost model, "
                    "run offline): expanded cells and the resulting route "
                    "through the tunnel.", width=TEXT_W * 0.9))
    s.append(H2("10.3  Hybrid-A* — the QSLAM target planner  " + badge("design")))
    s.append(P(
        "For a larger vehicle that cannot pivot in place, QSLAM specifies "
        "Smac Hybrid-A*. The search state becomes (x, y, ψ) with the heading "
        "discretised (typically 72 bins); successors are generated by short "
        "kinematically feasible motion primitives — straight segments and "
        "arcs of the minimum turning radius (Dubins, or Reeds–Shepp when "
        "reversing is allowed) — rather than grid steps. The heuristic is "
        "the maximum of a 2-D obstacle-aware distance and the obstacle-free "
        "non-holonomic distance, and an analytic expansion tries to connect "
        "directly to the goal with a Reeds–Shepp curve whenever it is near. "
        "The result is a path the vehicle can actually track. The skid-steer "
        "testbed rover can turn on the spot, so the 2-D planner is "
        "sufficient there."))
    s.append(H2("10.4  Prototype planner (MuJoCo)  " + badge("proto")))
    s.append(P(
        "The prototype runs its own A* on a 1 m grid sampled from the "
        "GeoTIFF, with blocked cells dilated by the footprint plus a 0.20 m "
        "margin and a soft terrain cost that prefers flat ground: "
        "cost = ℓ · (1 + 2.5 (θ/15°)²). It also builds a metric distance "
        "field to non-traversable terrain, the channel through which the "
        "ditch — a hole the camera cannot see as an obstacle — reaches the "
        "local planner."))

    # ----------------------------------------------------------- 11 ------
    s += [CondPageBreak(170), H1("11  Local planner — Model Predictive Path Integral control (MPPI)")]
    s.append(P(
        "The global path ignores what the lidar has just discovered and "
        "what the wheels can do in the next second. The local planner closes "
        "that gap. QSLAM uses Nav2's MPPI controller, a sampling-based model "
        "predictive controller derived from information-theoretic optimal "
        "control (Williams et al., 2017). Instead of solving an optimisation "
        "analytically it <b>imagines thousands of futures, scores them, and "
        "averages the good ones</b>."))
    s.append(figure("mppi_cycle", "One MPPI control cycle, repeated at "
                    "15 Hz.", width=TEXT_W * 0.92))
    s.append(H2("11.1  The algorithm"))
    s.append(P(
        "The controller keeps a nominal control sequence U = (u₀ … u<sub>T−1</sub>) "
        "with u = (v<sub>x</sub>, ω<sub>z</sub>), T = 56 steps of Δt = 0.067 s — a 3.75 s "
        "horizon. Each cycle:"))
    s.append(B("<b>Sample</b> K = 1800 perturbed sequences, "
               "ε<sub>k,t</sub> ~ N(0, Σ), Σ = diag(0.2², 0.4²), and clamp them to "
               "v<sub>x</sub> ∈ [−0.25, 0.9] m/s, |ω<sub>z</sub>| ≤ 1.0 rad/s."))
    s.append(B("<b>Roll out</b> each sequence through the differential-drive "
               "model:"))
    s.append(eq(r"x_{t+1} = x_t + v_t\cos\psi_t\,\Delta t,\quad "
                r"y_{t+1} = y_t + v_t\sin\psi_t\,\Delta t,\quad "
                r"\psi_{t+1} = \psi_t + \omega_t\,\Delta t"))
    s.append(B("<b>Score</b> every trajectory τ<sub>k</sub> with the weighted critics "
               "(critic table below) plus a control-cost term that keeps the update "
               "consistent with the sampling distribution:"))
    s.append(eq(r"S_k = \sum_{i} w_i\, C_i(\tau_k) + \gamma \sum_{t=0}^{T-1}"
                r" \mathbf{u}_t^{\mathsf{T}} \Sigma^{-1} \boldsymbol{\epsilon}"
                r"_{k,t},\qquad \gamma = 0.015"))
    s.append(B("<b>Weight</b> the samples with a softmax of their cost "
               "(temperature λ = 0.3) and <b>update</b> the sequence:"))
    s.append(eq(r"w_k = \frac{\exp\!\left(-\frac{1}{\lambda}(S_k - "
                r"S_{\min})\right)}{\sum_{j=1}^{K}\exp\!\left(-\frac{1}{\lambda}"
                r"(S_j - S_{\min})\right)},\qquad \mathbf{U} \leftarrow "
                r"\mathbf{U} + \sum_{k=1}^{K} w_k\,\boldsymbol{\epsilon}_k"))
    s.append(B("<b>Smooth and act</b>: Nav2 applies a 9-point quadratic "
               "Savitzky–Golay filter to the sequence, publishes u₀ on "
               "/cmd_vel, shifts the sequence one step and repeats."))
    s.append(P(
        "A small λ makes the average follow only the very best samples; a "
        "large λ averages broadly. Because every sample is a whole "
        "time-varying sequence, MPPI can find manoeuvres such as "
        "“swing left, then straighten” that a constant-arc sampler cannot, "
        "and it needs no gradient of the cost, so non-convex costs such as "
        f"costmaps are handled naturally. In {F('mppi_rollouts')}, 119 of "
        "the 1 800 samples carry 90 % of the weight."))
    s.append(figure("mppi_rollouts", "MPPI around an obstacle (numerical "
                    "illustration with the configured horizon, batch size, "
                    "noise and temperature; simplified critic set): (a) "
                    "sampled rollouts coloured by cost and the weighted "
                    "result; (b) how few samples carry most of the weight.", width=TEXT_W * 0.86))
    s.append(table([
        ["Critic", "Weight", "What it scores"],
        ["ConstraintCritic", "4.0", "Violations of the velocity limits of "
         "the motion model"],
        ["CostCritic", "3.81", "Costmap cost along the trajectory; collision "
         "cost 10⁶, critical cost 300; circular footprint"],
        ["GoalCritic", "5.0", "Distance of the trajectory end to the goal "
         "(active within 1.4 m)"],
        ["GoalAngleCritic", "3.0", "Heading towards the goal (within 0.5 m)"],
        ["PathAlignCritic", "14.0", "Alignment of trajectory points with "
         "the path; switched off when > 5 % of the path is blocked, so the "
         "robot may leave it to pass an obstacle"],
        ["PathFollowCritic", "5.0", "Progress towards a point 5 poses ahead "
         "on the path"],
        ["PathAngleCritic", "2.0", "Heading deviating more than 1.0 rad from "
         "the path direction"],
        ["PreferForwardCritic", "5.0", "Reverse motion (allowed, but "
         "discouraged)"],
    ], [104, 40, 347], size=7.9, cap="MPPI critics and weights "
        "(nav2_params.yaml)."))
    s.append(P(
        "Around the controller: controller_frequency 15 Hz; progress checker "
        "requires 0.5 m of movement in 20 s; goal checker accepts 0.7 m with "
        "any final heading (yaw tolerance 6.28 rad, stateful); "
        "failure_tolerance 0.5 s. A stale-command watchdog in run_sim.py "
        "stops the wheels if no /cmd_vel arrives for 0.5 s."))
    s.append(H2("11.2  Prototype local planner — DWA  " + badge("proto")))
    s.append(P(
        "The MuJoCo prototype uses the Dynamic Window Approach: it samples "
        "constant (v, ω) pairs (6 speeds up to the acceleration-limited "
        "window plus two reverse speeds, 25 turn rates), rolls each out for "
        "2.4 s at 0.2 s steps, rejects any whose clearance to the depth-built "
        "costmap or to prior terrain hazards is below 0.65 m, and minimises"))
    s.append(eq(r"J = 2.4\,d_{\mathrm{goal}} + 4\,\frac{\max(0,\ r_s - c)}{r_s}"
                r" + 1.4\,(v_{\max} - v) + 0.3\,|\omega| + 1.1\,\min(\max("
                r"o - 4, 0), 10) + 2.5\,[v<0] + 3\,[|v|<0.05]"))
    s.append(P(
        "with r<sub>s</sub> = 0.75 m the soft clearance, c the rollout's minimum "
        "clearance and o the offset from the surveyed route. Making the "
        "margin soft (not hard) is what prevents the classic DWA deadlock "
        "where one noisy cell rejects every sample.", L.small))

    # ----------------------------------------------------------- 12 ------
    s += [CondPageBreak(170), H1("12  Behaviour tree, supervision and recovery")]
    s.append(P(
        "bt_navigator executes Nav2's default <i>navigate_to_pose_w_"
        f"replanning_and_recovery</i> behaviour tree ({F('bt_navigator')}). "
        "A PipelineSequence runs ComputePathToPose under a 1 Hz "
        "RateController and FollowPath continuously; each has its own "
        "single-retry recovery that clears the corresponding costmap. If "
        "both fail, the outer RecoveryNode (6 retries) runs a RoundRobin of "
        "recovery behaviours, one per failure: clear both costmaps → spin "
        "1.57 rad → wait 5 s → back up 0.30 m at 0.15 m/s. A new goal "
        "pre-empts recovery (GoalUpdated)."))
    s.append(figure("bt_navigator", "The navigate-to-pose behaviour tree "
                    "used by QSLAM (Nav2 Jazzy default).", width=TEXT_W * 0.9))
    s.append(P(
        "behavior_server provides spin, backup, drive_on_heading and wait at "
        "10 Hz with rotational limits 0.2–0.8 rad/s and 2.0 rad/s², checking "
        "2.0 s ahead against the local costmap. The committed and "
        "run_1080p missions needed one recovery each; run_final needed "
        f"seven, and {F('ekf_commands')} reconstructs them from nav.log: "
        "the rover stalled 1 m from an unsurveyed crate (five no-progress "
        "aborts), a spin freed it, and two later controller aborts were "
        "cleared before the goal succeeded at 986 s. Short reverse commands "
        "(amber) are MPPI's own choice, allowed down to −0.25 m/s."))
    s.append(figure("ekf_commands", "Velocity commands during the 189 m run "
                    "(run_final) with the recovery events taken from nav.log; "
                    "amber = MPPI reverse outputs, grey = no-progress window.", width=TEXT_W * 0.92))

    # ----------------------------------------------------------- 13 ------
    s += [CondPageBreak(170), H1("13  Localization — the Extended Kalman Filter")]
    s.append(P(
        "Without GPS, “where am I?” is the hardest question. QSLAM answers it "
        "with robot_localization's EKF, the sole owner of odom→base_link. "
        "A Kalman filter keeps a Gaussian belief — a mean state x̂ and "
        "covariance P — and alternates <b>prediction</b> (move the belief "
        "with a motion model, grow the uncertainty) and <b>update</b> "
        "(correct it with each measurement, weighted by relative "
        "confidence). The <i>extended</i> filter linearises the nonlinear "
        f"models with Jacobians ({F('ekf_loop')})."))
    s.append(figure("ekf_loop", "EKF predict/update cycle as configured in "
                    "config/ekf.yaml.", width=TEXT_W * 0.88))
    s.append(H2("13.1  State and prediction"))
    s.append(P(
        "The state has 15 elements — position, orientation, linear and "
        "angular velocity, linear acceleration: "
        "x = [x y z φ θ ψ v<sub>x</sub> v<sub>y</sub> v<sub>z</sub> ω<sub>x</sub> ω<sub>y</sub> ω<sub>z</sub> a<sub>x</sub> a<sub>y</sub> a<sub>z</sub>]ᵀ. With "
        "two_d_mode the vertical, roll and pitch components are held at "
        "zero. The filter runs at 30 Hz with an omnidirectional kinematic "
        "model; for the planar part:"))
    s.append(eq(r"x_{k+1} = x_k + (v_x\cos\psi - v_y\sin\psi)\,\Delta t,\quad "
                r"y_{k+1} = y_k + (v_x\sin\psi + v_y\cos\psi)\,\Delta t,\quad "
                r"\psi_{k+1} = \psi_k + \omega_z\,\Delta t"))
    s.append(P(
        "(plus ½aΔt² terms and v ← v + aΔt). The covariance is propagated "
        "with the Jacobian F = ∂f/∂x and the process noise Q:"))
    s.append(eq(r"\hat{\mathbf{x}}^{-}_{k+1} = f(\hat{\mathbf{x}}_k,\Delta t),"
                r"\qquad \mathbf{P}^{-}_{k+1} = \mathbf{F}\,\mathbf{P}_k\,"
                r"\mathbf{F}^{\mathsf{T}} + \mathbf{Q}\,\Delta t"))
    s.append(H2("13.2  Measurement update"))
    s.append(P(
        "Each sensor message selects the state elements it measures (a "
        "boolean mask per sensor, which forms H) and carries its own "
        "covariance R. The update computes the innovation ν, its covariance "
        "S and the Kalman gain K, and uses the Joseph form for numerical "
        "stability:"))
    s.append(eq(r"\boldsymbol{\nu} = \mathbf{z} - \mathbf{H}\hat{\mathbf{x}}^{-},"
                r"\quad \mathbf{S} = \mathbf{H}\mathbf{P}^{-}\mathbf{H}^{\mathsf{T}}"
                r" + \mathbf{R},\quad \mathbf{K} = \mathbf{P}^{-}\mathbf{H}"
                r"^{\mathsf{T}}\mathbf{S}^{-1}"))
    s.append(eq(r"\hat{\mathbf{x}} = \hat{\mathbf{x}}^{-} + \mathbf{K}\,"
                r"\boldsymbol{\nu},\qquad \mathbf{P} = (\mathbf{I} - \mathbf{K}"
                r"\mathbf{H})\,\mathbf{P}^{-}(\mathbf{I} - \mathbf{K}\mathbf{H})"
                r"^{\mathsf{T}} + \mathbf{K}\mathbf{R}\mathbf{K}^{\mathsf{T}}"))
    s.append(table([
        ["Input", "Fused state elements", "Covariance (R)", "Physical source"],
        ["odom0: /odom_wheel", "v<sub>x</sub>, v<sub>y</sub> (body frame)", "0.02 (m/s)²",
         "VIO-grade body velocity (cuVSLAM on hardware); synthesised from "
         "the rigid-body state in the testbed"],
        ["imu0: /imu/data", "ψ (absolute yaw), ω<sub>z</sub>, a<sub>x</sub>", "ψ 0.05 rad², "
         "ω<sub>z</sub> 0.005, a<sub>x</sub> 0.04", "AHRS / magnetometer-grade heading, gyro, "
         "accelerometer (gravity removed)"],
        ["initial_state", "x = −62.0, y = 16.0, ψ = 0.1396 rad (8°)", "—",
         "Surveyed deployment pose (launch point)"],
    ], [82, 108, 88, 213], size=7.9, cap="EKF inputs (config/ekf.yaml, "
        "covariances from run_sim.py)."))
    s.append(H2("13.3  Why these inputs — a measured decision"))
    s.append(P(
        "Wheel-encoder odometry was tried first and failed in three measured "
        "ways on this skid-steer: (1) yaw from the left/right wheel-speed "
        "difference is meaningless when the wheels scrub sideways in every "
        "turn; (2) distance over-counts by about 30 % on slopes because the "
        "wheels slip; (3) sideways crabbing on side slopes is invisible to "
        "encoders. Missions ended 10–70 m from where the robot believed it "
        "was. Integrating the gyro alone for heading drifted about 13° over "
        "45 m. The fused quantities are therefore exactly what a "
        "visual-inertial odometry system and an AHRS measure on the real "
        f"vehicle. {F('deadreckon_reconstruction')} reconstructs, from the "
        "recorded 189 m path, how each failure mode grows: gyro-only heading "
        "ends 57 m off, a 30 % wheel over-count on slopes 4.3 m, the adopted "
        "EKF 1.09 m."))
    s.append(figure("deadreckon_reconstruction", "Position error versus "
                    "distance: gyro-only heading and slipping wheel "
                    "odometry (reconstructed from the recorded ground-truth "
                    "path with the drift rates measured on this platform) "
                    "against the EKF actually used.", width=TEXT_W * 0.82))
    s.append(H2("13.4  EKF estimation results"))
    s.append(P(
        "The evaluation logger records ground truth, the EKF estimate and "
        "the commands at 5 Hz. Localization error is ‖p<sub>GT</sub> − p<sub>EKF</sub>‖; it is "
        f"what the robot does not know about its own position. "
        f"{F('ekf_trajectory')} overlays estimate and truth for the two "
        f"recorded runs and {F('ekf_error')} analyses the error."))
    s.append(figure("ekf_trajectory", "EKF estimate versus ground truth for "
                    "the two recorded missions. Insets: the final offset at "
                    "arrival.", width=TEXT_W * 0.8))
    s.append(figure("ekf_error", "EKF estimation error: (a) over time; (b) "
                    "against distance travelled; (c) x and y components; "
                    "(d) distribution.", width=TEXT_W * 0.82))
    s.append(table([
        ["Run", "Distance", "Mean", "Median", "95th pct", "Max", "Final",
         "Final / dist."],
        ["run_final (headless, 7 recoveries)",
         f"{stat('ekf.run_final.distance_m', '189.0', '{:.1f}')} m",
         f"{stat('ekf.run_final.mean_m', '0.49')} m",
         f"{stat('ekf.run_final.median_m', '0.48')} m",
         f"{stat('ekf.run_final.p95_m', '0.95')} m",
         f"{stat('ekf.run_final.max_m', '1.10')} m",
         f"{stat('ekf.run_final.final_m', '1.09')} m",
         f"{stat('ekf.run_final.final_pct_of_distance', '0.58')} %"],
        ["run_1080p (GUI recording, 1 recovery)",
         f"{stat('ekf.run_1080p.distance_m', '295.1', '{:.1f}')} m",
         f"{stat('ekf.run_1080p.mean_m', '0.70')} m",
         f"{stat('ekf.run_1080p.median_m', '0.77')} m",
         f"{stat('ekf.run_1080p.p95_m', '1.39')} m",
         f"{stat('ekf.run_1080p.max_m', '1.43')} m",
         f"{stat('ekf.run_1080p.final_m', '1.42')} m",
         f"{stat('ekf.run_1080p.final_pct_of_distance', '0.48')} %"],
        ["committed run (README; not re-recorded)", "123.5 m plan", "—",
         "—", "—", "—", "0.98 m", "—"],
    ], [136, 52, 44, 46, 50, 44, 44, 75], size=7.8, cap="EKF localization "
        "error per run (ground truth used for scoring only). Distances are "
        "summed from 5 Hz ground truth and include manoeuvring."))
    s.append(callout(
        "<b>Reading these numbers honestly.</b> The EKF's velocity and "
        "heading inputs are synthesised from the simulator's rigid-body "
        "state, i.e. idealised VIO/AHRS-grade sensors without their noise. "
        "The ≈1 m errors therefore measure the integration pipeline — 30 Hz "
        "discretisation, message timing, and the 2-D projection of 3-D "
        "motion (in two_d_mode a body speed on a slope θ is integrated as "
        "horizontal, over-counting by 1/cos θ − 1: 1 % at 8°, 3.5 % at "
        "15°) — not real sensor noise. Replacing the synthesised input with "
        "cuVSLAM (Phase 2b) and bounding drift by map anchoring (§15.1) are "
        "the designated next steps.", kind="warn"))

    # ----------------------------------------------------------- 14 ------
    s += [CondPageBreak(170), H1("14  Visual SLAM")]
    s.append(P(
        "Visual SLAM (simultaneous localization and mapping) estimates the "
        "camera's motion from the images themselves while building a map of "
        "landmarks. A <b>front-end</b> turns images into relative motion "
        "(visual odometry); a <b>back-end</b> keeps drift in check by "
        "optimising a graph of keyframes and closing loops when a place is "
        f"recognised. {F('vslam_pipeline')} shows the pipeline and which "
        "parts come from which engine."))
    s.append(figure("vslam_pipeline", "Visual SLAM pipeline. Front-end "
                    "stages as implemented in the prototype; back-end "
                    "optimisation and loop closure as provided by cuVSLAM / "
                    "RTAB-Map.", width=TEXT_W * 0.9))
    s.append(H2("14.1  Stereo depth"))
    s.append(P(
        "A stereo camera with baseline B and focal length f (pixels) "
        "measures depth from disparity d; a disparity error σ<sub>d</sub> grows "
        "quadratically into a depth error:"))
    s.append(eq(r"z = \frac{f\,B}{d},\qquad \sigma_z = \frac{z^{2}}{f\,B}\,"
                r"\sigma_d"))
    s.append(figure("stereo_depth_noise", "Depth uncertainty of a ZED 2i "
                    "(B = 0.12 m) versus range — σz = 0.51 m at 10 m; the "
                    "prototype only accepts obstacle evidence inside 8 m.", width=TEXT_W * 0.86))
    s.append(H2("14.2  The prototype front-end  " + badge("proto")))
    s.append(P(
        "The MuJoCo prototype runs a real stereo-depth VO front-end on the "
        "rendered ZED 2i images (not a replay of ground truth):"))
    for r in [
        "<b>Features.</b> ORB on the histogram-equalised left image: FAST "
        "corners (threshold 12) at 8 pyramid levels (scale 1.2), up to 1500 "
        "features, each with a 256-bit rotated-BRIEF descriptor (31 px "
        "patch).",
        "<b>3-D landmarks.</b> Each corner with valid depth (0.3–15 m) is "
        "back-projected with the pinhole model: X = (u − c<sub>x</sub>) z / f<sub>x</sub>, "
        "Y = (v − c<sub>y</sub>) z / f<sub>y</sub>, Z = z, in the keyframe's optical frame.",
        "<b>Matching.</b> Brute-force Hamming distance, two nearest "
        "neighbours, Lowe ratio test d₁ < 0.78 d₂; at least 14 matches.",
        "<b>Pose.</b> PnP-RANSAC on the 3-D↔2-D pairs minimises the "
        "reprojection error Σ‖u<sub>i</sub> − π(K(R X<sub>i</sub> + t))‖² with a 2.5 px inlier "
        "threshold, 220 iterations, confidence 0.995, then Levenberg–"
        "Marquardt refinement on the inliers (≥ 10). RANSAC needs "
        "N = log(1 − p) / log(1 − wˢ) iterations for inlier ratio w and "
        "sample size s.",
        "<b>Chaining.</b> T<sub>world,cam</sub> = T<sub>world,kf</sub> · (T<sub>cur,kf</sub>)⁻¹; a new "
        "keyframe is taken after 0.45 m, 11° or fewer than 40 inliers; 25 % "
        "of each keyframe's landmarks join a sparse map (≤ 9000 points).",
    ]:
        s.append(B(r))
    s.append(P(
        "It has no loop closure or bundle adjustment, so drift grows "
        "monotonically: 0.98 % of path in the committed run and up to 4.3 % "
        "in later runs (PnP-RANSAC is not bit-exact between runs), with 0 "
        "tracking losses over ≈260 keyframes."))
    s.append(H2("14.3  Production engines  " + badge("exp")))
    s.append(P(
        "<b>cuVSLAM</b> (Isaac ROS Visual SLAM, installed at version "
        "15.0.0 with Isaac ROS 4.6, CUDA 13.2) is NVIDIA's GPU-accelerated "
        "stereo/RGB-D visual-inertial odometry with sparse bundle "
        "adjustment and loop closure. Integration findings from this "
        "workspace: it must be started from a launch file (the bare binary "
        "fails on its Bazel runfiles), and single-camera RGB-D mode needs "
        f"all of {m('tracking_mode: 2')}, {m('num_cameras: 1')}, "
        f"{m('min_num_images: 1')}, {m('depth_camera_id: 0')} — with the "
        "default two cameras it silently publishes nothing. Its odometry "
        "will feed the EKF's odom0 input with TF publishing disabled "
        "(rule 3)."))
    s.append(P(
        "<b>RTAB-Map</b> was the Phase 2 choice for the Gazebo stack: "
        "graph-based SLAM with appearance-based loop closure (bag-of-words "
        "over visual words) and working/long-term memory management that "
        "bounds computation on long missions. Rule 4 applies: its odometry "
        "comes in on a topic, never through odom_frame_id. "
        "<b>slam_toolbox</b> (lidar SLAM: correlative scan matching, Ceres "
        "pose-graph optimisation, 0.10 m resolution, loop search within "
        "8 m) is wired behind slam:=true as an owner for map→odom; the sync "
        "node is used because the async node has an activation race under "
        "simulation time. On this 3-D terrain a 2-D scan disagrees with "
        "odometry geometry on slopes, and its corrections degraded the "
        "pose after ≈50 m — hence the visual route for map→odom."))

    # ----------------------------------------------------------- 15 ------
    s += [CondPageBreak(170), H1("15  QSLAM innovations — designed modules")]
    s.append(P(
        badge("design") + "  The registered QSLAM idea adds three modules on "
        "top of the proven "
        "components above. They are specified here so the design is "
        f"complete; none is in the repository yet ({F('qslam_innovations')})."))
    s.append(figure("qslam_innovations", "The three QSLAM innovations and "
                    "where they plug into the implemented EKF and Nav2 "
                    "stack.", width=TEXT_W * 0.84))
    s.append(H2("15.1  Map-anchored global localization"))
    s.append(P(
        "This module plays the role GPS used to play, passively. (1) A "
        "<b>bird's-eye view</b> (BEV) of the surroundings is built by "
        "projecting camera pixels (or their semantic classes) onto the "
        "ground using depth and the GPU 2.5-D elevation map. (2) A tile of "
        "the <b>preloaded orthoimage and DEM</b> is cropped around the EKF's "
        "predicted pose, sized by its covariance. (3) A cross-view matcher "
        "(learned embeddings in the style of Shi & Li 2022 and OrienterNet "
        "2023, or normalised cross-correlation on elevation shape) scores "
        "every candidate offset (Δx, Δy, Δψ); the peak gives a UTM pose and "
        "heading and its sharpness a covariance. (4) The match is accepted "
        "only if it is statistically consistent with the filter:"))
    s.append(eq(r"d^{2} = \boldsymbol{\nu}^{\mathsf{T}}\mathbf{S}^{-1}"
                r"\boldsymbol{\nu} < \chi^{2}_{3,\,0.99} = 11.34"))
    s.append(P(
        "(5) An accepted match is fused as an absolute (x, y, ψ) update and "
        "takes ownership of map→odom, re-anchoring every ~200 m. The same "
        "matcher also provides the initial pose and heading, removing the "
        "need for a surveyed start point and a compass. The team's proof-"
        "of-concept study (reported in the idea deck; not reproduced in "
        "this repository) cut 3 km GNSS-free position error from 89.6 m to "
        "10.6 m (−88 %)."))
    s.append(H2("15.2  Localizability-aware navigation"))
    s.append(P(
        "Visual odometry fails on featureless ground (sand, water, "
        "uniform grass — cf. the low-texture patch in §6). A custom Nav2 "
        "costmap layer (C++ plugin) scores each cell by predicted "
        "localizability L ∈ [0, 1] from orthoimage texture/feature density "
        "and from feature counts observed online, adding "
        "c<sub>loc</sub> = w · (1 − L) to the traversability cost. The planner then "
        "accepts a slightly longer route that keeps the camera tracking. "
        "PoC (idea deck): −59 % arrival error for +18 % path length."))
    s.append(H2("15.3  Localization integrity monitor"))
    s.append(P(
        "Every cycle the monitor cross-checks visual odometry against the "
        "IMU and wheel odometry: normalised innovation squared (NIS) against "
        "a χ² gate, VIO tracking status and feature count. It drives a "
        f"three-state supervisor ({F('integrity_fsm')}): NORMAL; DEGRADED — "
        "slow down, prefer high-localizability cells, request a re-anchor, "
        "bridge with IMU + wheels; SAFE-STOP — stop and hold when the error "
        "bound exceeds the mission limit. The system degrades visibly "
        "instead of drifting silently."))
    s.append(figure("integrity_fsm", "Integrity-monitor state machine "
                    "(QSLAM design).", width=TEXT_W * 0.84))
    s.append(H2("15.4  Perception and safety layers"))
    s.append(P(
        "Semantic segmentation (SegFormer fine-tuned on the RELLIS-3D and "
        "RUGD off-road datasets, TensorRT-optimised) labels drivable ground, "
        "vegetation, water and obstacles; elevation_mapping_cupy fuses depth "
        "into a GPU 2.5-D elevation map from which slope, step height and "
        "roughness become traversability costs, fused with the semantic "
        "classes. Nav2's Collision Monitor adds a last-line safety check: "
        "polygon zones around the footprint that slow or stop the vehicle "
        "independently of the planner. Target: ≥ 15 FPS perception on a "
        "Jetson AGX Orin."))

    # ----------------------------------------------------------- 16 ------
    s += [CondPageBreak(170), H1("16  Simulation platform, vehicle and results")]
    s.append(H2("16.1  Isaac Sim testbed  " + badge("impl")))
    s.append(P(
        "Isaac Sim 6.0.1 loads the USD stage written by build_scene.py: the "
        "terrain mesh from the DEM with its texture, the roofed tunnel, "
        "obstacles, trees, rocks, four humans and lighting. PhysX steps at "
        "60 Hz and rendering at 30 Hz. An OmniGraph action graph "
        "(OnPlaybackTick → ROS2Context, ROS2PublishClock, two "
        "ROS2CameraHelpers for RGB and depth, ROS2CameraInfoHelper, "
        "ROS2RtxLidarHelper, ROS2PublishJointState, IsaacReadSimulationTime) "
        "publishes sensors; an in-process rclpy node handles /cmd_vel, "
        "odometry, IMU, ground truth and moves the patrolling humans along "
        "their waypoints on the DEM surface."))
    s.append(P(
        "<b>Skid-steer drive.</b> A twist (v, ω) becomes wheel speeds "
        "ω<sub>L,R</sub> = (v ∓ ω·G·T/2)/r with track T = 0.70 m, r = 0.15 m, limited "
        "to 30 rad/s. PhysX reproduces the real platform's scrub: with stock "
        "friction only ≈6 % of the commanded yaw rate was achieved. Wheel "
        "friction μ 0.45/0.40, drive damping 5000, max force 900 and a "
        "measured yaw gain G = 6.5 (spin ratio 0.398 at G = 2.6, "
        f"calibrate_yaw.py) restore tracking ({F('skid_steer')})."))
    s.append(figure("skid_steer", "(a) Skid-steer geometry and the "
                    "command mapping; (b) yaw-rate tracking before and "
                    "after calibration.", width=TEXT_W * 0.76))
    s.append(H2("16.2  Results"))
    s.append(table([
        ["Measure", "Committed run", "run_final", "run_1080p",
         "MuJoCo prototype"],
        ["Goal reached", "yes, via tunnel", "yes, via tunnel",
         "yes, via tunnel", "yes"],
        ["Route / distance travelled", "123.5 m plan",
         f"{stat('ekf.run_final.distance_m', '189.0', '{:.1f}')} m",
         f"{stat('ekf.run_1080p.distance_m', '295.2', '{:.1f}')} m",
         "121.5–123.8 m"],
        ["Localization error at end", "0.98 m",
         f"{stat('ekf.run_final.final_m', '1.09')} m",
         f"{stat('ekf.run_1080p.final_m', '1.42')} m",
         "1.20–5.33 m (VO drift)"],
        ["Arrival error (truth → goal)", "0.23 m",
         f"{stat('ekf.run_final.gt_final_to_goal_m', '1.33')} m",
         f"{stat('ekf.run_1080p.gt_final_to_goal_m', '1.20')} m", "1.91–5.68 m"],
        ["Recoveries", "1", "7", "1", "—"],
        ["Mission time (sim)", "≈460 s",
         f"{stat('ekf.run_final.duration_s', '968', '{:.0f}')} s",
         f"{stat('ekf.run_1080p.duration_s', '1417', '{:.0f}')} s",
         "127–131 s"],
        ["Obstacle contacts", "not logged", "not logged", "not logged",
         "0 (21 obstacles)"],
    ], [118, 90, 88, 88, 107], size=7.9, first_col_bold=True,
        cap="Measured results. Isaac runs: seed 42, goal E 403 562.52 "
        "N 1 450 024.12. The committed run is the README's reference run "
        "(123.5 m = Nav2's initial plan length; straight line 119.4 m); "
        "run_final and run_1080p are the recorded runs, errors at the last "
        "logged sample."))

    s.append(P(
        "<b>Recording pipeline.</b> record_mission.sh runs the headless "
        "simulator with three follow cameras to 1080p MP4, the navigation "
        "stack, an RViz screen capture, the telemetry logger "
        "(mission_logger.py, 5 Hz CSV) and the goal client, then renders a "
        "highlight reel (make_highlight.py) and a PDF mission report "
        "(make_mission_report.py).", L.small))

    # ----------------------------------------------------------- 17 ------
    s += [CondPageBreak(170), H1("17  Technology stack, limitations and roadmap")]
    s.append(table([
        ["Layer", "Technologies", "Role"],
        ["Middleware", "ROS 2 Jazzy (rclpy/rclcpp, DDS, TF2, lifecycle, "
         "actions)", "Message passing, time, transforms"],
        ["Navigation", "Nav2: bt_navigator (BehaviorTree.CPP v4), Smac "
         "Planner 2D (Hybrid-A* planned), MPPI controller, costmap_2d, "
         "behaviours, map_server, lifecycle_manager", "Planning, control, "
         "supervision"],
        ["Localization", "robot_localization EKF; Isaac ROS cuVSLAM; "
         "RTAB-Map; slam_toolbox (Karto + Ceres)", "State estimation"],
        ["Geospatial", "GeoTIFF via rasterio/GDAL, pyproj (PROJ), EPSG:32644, "
         "QGIS-compatible outputs; Bhuvan/CartoDEM planned", "Prior maps, "
         "UTM↔map"],
        ["Vision", "OpenCV (ORB, BFMatcher, solvePnPRansac, LM refine); "
         "SegFormer + TensorRT, elevation_mapping_cupy (planned)",
         "Visual odometry, perception"],
        ["Simulation", "NVIDIA Isaac Sim 6.0.1 (USD, PhysX, RTX lidar, "
         "OmniGraph ROS 2 bridge); MuJoCo; Gazebo Harmonic + ros_gz",
         "Validation"],
        ["Compute", "RTX 4090 + i9-13900K (testbed); Jetson AGX Orin "
         "(target)", "Onboard execution"],
        ["Tooling", "Python 3.12, NumPy, Matplotlib, ReportLab, ffmpeg, "
         "RViz 2, colcon", "Build, analysis, reporting"],
    ], [72, 300, 119], size=7.3, first_col_bold=True,
        cap="Technologies implemented or specified in QSLAM."))
    s.append(H2("Limitations (stated, not hidden) and roadmap"))
    for r in [
        "Testbed EKF inputs are idealised (no VIO/AHRS noise yet — Phase "
        "2b); drift is bounded only by distance until map anchoring or loop "
        "closure (§15.1) is in the loop.",
        "The lidar is 2-D: negative obstacles come only from the prior DEM "
        "until the 2.5-D elevation map exists (Phase 3); slam_toolbox "
        "degraded on 3-D terrain and is not in the measured configuration. "
        "The roadmap and its targets are in Table 4.",
    ]:
        s.append(B(r))
    return s


# =================================================================== PART B
def part_b():
    pb, pbb = L.pb_body, L.pb_bullet

    def Q(text):
        return P(text, pb)

    def QB(text):
        return B(text, pbb)

    s = [SetPart("Part B — QSLAM for everyone")]

    # ------------------------------------------------------------ B1 -----
    s.append(P("Part B — QSLAM explained for everyone", L.pb_h1))
    s.append(P("B1  What QSLAM is, in one page", L.pb_h2))
    s.append(Q(
        "Picture a small robotic vehicle, about the size of a large suitcase "
        "on four wheels. Someone gives it one instruction: <b>“go to this "
        "point on the map.”</b> Nobody drives it. On the way it has to cross "
        "open ground, climb a gentle hill, keep away from a mound that is "
        "too steep, find the one tunnel through an earth wall, and steer "
        "around barrels, logs, rocks and people walking across its path — "
        "and it must do all of this <b>without GPS</b>."))
    s.append(Q(
        "Why without GPS? The GPS signal is a very weak radio signal from "
        "satellites about 20,000 km away. It does not reach inside tunnels "
        "and mines, it fades under thick trees and between tall buildings, "
        "and in a conflict it can be jammed (drowned out) or spoofed "
        "(faked). Those are exactly the places where soldiers, rescuers and "
        "miners would most like to send a robot instead of a person."))
    s.append(Q(
        "<b>QSLAM</b> is our team's system for this problem (SIH 2026, "
        "problem statement SIH26126). “SLAM” stands for <i>Simultaneous "
        "Localization and Mapping</i>: a robot working out where it is while "
        "it makes sense of what it sees. QSLAM is software — the “brain” — "
        "and it has been built and tested in a realistic 3-D simulation of "
        "a 150 m × 150 m outdoor site."))
    s.append(figure("plain_mission_story", "A QSLAM mission in five steps."))
    s.append(figure(REC / "run_1080p/still_isaac_side.jpg",
                    "The simulated rover passing a grove of trees during a "
                    "test mission (Isaac Sim).", width=TEXT_W * 0.72,
                    key="plain_side"))

    # ------------------------------------------------------------ B2 -----
    s += [PageBreak(), P("B2  Does it use GPS? No. So how does it know "
                         "where it is?", L.pb_h2)]
    s.append(callout([P(
        "<b>Straight answer: QSLAM does not use GPS at all.</b> There is no "
        "GPS receiver in the system — not switched off, simply not there. "
        "So it cannot lose a GPS signal, and nobody can jam or fake one to "
        "mislead it.", ParagraphStyle_(pb, spaceAfter=0))], kind="ok"))
    s.append(Q("It works out its position the way a careful hiker without a "
               "phone would, using four ideas:"))
    s.append(figure("plain_how_it_knows", "How QSLAM knows where it is "
                    "without GPS."))
    for r in [
        "<b>1. It starts from a known spot.</b> The launch point was "
        "measured once, in advance — like starting a walk from your own "
        "front door, whose address you know.",
        "<b>2. It feels and watches its own movement.</b> The camera sees "
        "the ground slide past (like watching the road from a car window), "
        "an inertial sensor feels every turn (like your inner ear) and a "
        "heading sensor gives direction (like a compass). A mathematical "
        "“referee” called a <i>Kalman filter</i> combines them, trusting "
        "each source as much as it deserves, into one best guess of "
        "position — updated 30 times a second.",
        "<b>3. It carries its own map.</b> Before the mission it is given "
        "a height map of the area, so it already knows where the slopes, "
        "the ditch and the tunnel are, and plans a sensible route.",
        "<b>4. It recognises places (being added).</b> Small errors add up "
        "over distance. The planned “map anchoring” step compares the "
        "ground the camera sees, viewed from above, with a satellite "
        "picture of the area — like a hiker matching the shape of a hill to "
        "the map — and corrects the drift.",
    ]:
        s.append(QB(r))
    s.append(Q(
        "<b>How good is it?</b> In the simulated tests, after driving about "
        "190 m entirely on its own, the robot's idea of its position was "
        "about 1 metre off — roughly the length of the vehicle plus a "
        "bit. (The sensors in the simulation are cleaner than real ones; "
        "real-world numbers will be measured next.)"))
    s.append(Q(
        "<b>How can it accept a UTM “GPS-style” coordinate without GPS?</b> "
        "UTM is simply a grid that names every place on Earth in metres "
        "(an easting and a northing), like a giant sheet of graph paper. "
        "The site's reference point was surveyed once, so turning the "
        "destination into “118 m east and 18 m north of where you start” "
        "is a subtraction. No satellite is needed for arithmetic."))

    # ------------------------------------------------------------ B3 -----
    s += [PageBreak(), P("B3  Is QSLAM an online or an offline system?",
                         L.pb_h2)]
    s.append(callout([P(
        "<b>Short answer: offline.</b> QSLAM needs no internet, no cloud "
        "server, no mobile network and no satellite link. Every calculation "
        "happens on the vehicle's own computer. If all communication is "
        "cut, the mission simply continues.",
        ParagraphStyle_(pb, spaceAfter=0))], kind="ok"))
    s.append(Q(
        "People use the word “online” in two different ways, so here is "
        "each one answered separately:"))
    s.append(table([
        ["Question", "Answer for QSLAM"],
        ["Is it connected to the internet or a server while it works?",
         "<b>No.</b> Nothing is sent or received over a network during the "
         "mission. The maps it needs are copied onto the vehicle "
         "beforehand."],
        ["Does it work live, in real time?", "<b>Yes.</b> It reads its "
         "sensors, updates its position and re-plans its steering many "
         "times per second (steering 15 times a second, position 30 times "
         "a second)."],
        ["Does a person have to steer it?", "<b>No.</b> A person only "
         "types the destination once. Everything after that is automatic."],
    ], [175, TEXT_W - 175], size=9.6, first_col_bold=True))
    s.append(Spacer(1, 4))
    s.append(figure("plain_needs", "What QSLAM needs, and what it does "
                    "not need."))
    s.append(table([
        ["Before the mission — done once, offline",
         "During the mission — on the vehicle, live"],
        ["Load the height map of the area (a “GeoTIFF” file)",
         "Read camera, laser scanner and motion sensors"],
        ["Record the surveyed reference point of the site",
         "Estimate its own position (Kalman filter)"],
        ["Prepare the planner's map: mark slopes steeper than 15° as "
         "no-go", "Plan the route and re-check it every second"],
        ["(Planned) Load satellite image tiles of the area",
         "Steer around obstacles and people, 15 times a second"],
        ["", "Report progress in map metres and in UTM"],
    ], [TEXT_W / 2, TEXT_W / 2], size=9.4))

    # ------------------------------------------------------------ B4 -----
    s += [PageBreak(), P("B4  One mission, step by step — and what has "
                         "been proven", L.pb_h2)]
    for r in [
        "<b>Preparation.</b> The site's height map and reference point are "
        "loaded. The planner marks everything steeper than 15° as no-go.",
        "<b>Destination.</b> The operator types one line: easting "
        "403 562.52, northing 1 450 024.12. QSLAM turns that into “118 m "
        "east, 18 m north of the start”.",
        "<b>Plan.</b> The route planner searches the map for the cheapest "
        "safe path. The earth wall is too steep everywhere except at the "
        "tunnel, so the route goes through the tunnel.",
        "<b>Drive.</b> A laser scanner spots things the map never "
        "contained — barrels, logs, rocks, walking people. The steering "
        "controller imagines 1800 possible manoeuvres for the next few "
        "seconds, fifteen times every second, and picks the safest one that "
        "still makes progress. If it gets stuck, it backs up, turns or waits, "
        "then tries again.",
        "<b>Arrive.</b> It stops within about a metre of the destination and "
        "reports success.",
    ]:
        s.append(QB(r))
    s.append(figure(FIGS / "img_stills.jpg", "A recorded test mission from "
                    "four viewpoints; bottom right is the robot's own view "
                    "of its maps and planned route.", width=TEXT_W * 0.86,
                    key="plain_stills"))
    s.append(P("What has been shown so far (in simulation)", L.pb_h2))
    s.append(table([
        ["Question", "Result"],
        ["Did it reach the destination?", "Yes — in every recorded test, "
         "going through the tunnel."],
        ["How far off was its own position estimate?", "About 1 to 1.4 m "
         "after 123–295 m of driving, with no GPS."],
        ["Did it hit anything or anyone?", "In the prototype, which counts "
         "every contact: none, across 21 obstacles. The Isaac test world does "
         "not log contacts yet."],
        ["Did it ever use GPS?", "Never — there is no GPS in the software."],
    ], [200, TEXT_W - 200], size=9.6, first_col_bold=True))
    s.append(Spacer(1, 7))
    s.append(Q(
        "<b>Not yet proven:</b> everything so far is in simulation; the "
        "camera-only position tracking is being connected in place of the "
        "simulator's idealised motion data, and the satellite map-matching "
        "step is designed but not built. Real-vehicle trials come after "
        "that."))

    # ------------------------------------------------------------ B5 -----
    s += [PageBreak(), P("B5  Questions people usually ask", L.pb_h2)]
    faq = [
        ("Could an enemy jam or fake its navigation?",
         "There is no GPS signal to jam or fake. Its camera and motion "
         "sensors only receive, they do not transmit; the test vehicle's "
         "laser scanner is used only to see nearby obstacles."),
        ("What if the camera cannot see — dust, darkness, a blank wall?",
         "The motion sensors carry it through short gaps. The planned "
         "integrity monitor notices when vision becomes unreliable, slows "
         "the vehicle down, and stops it safely if needed; a thermal "
         "camera is an optional upgrade for night."),
        ("How accurate does it need to be?", "The design goal is to arrive "
         "within 2 % of the distance driven — within 2 m after 100 m."),
        ("What happens if the way is completely blocked?", "It tries "
         "recovery moves and new routes. If no safe route exists, it stops "
         "and reports that the goal could not be reached, rather than "
         "taking a risk."),
        ("Can it work in a place it has never been?", "Yes, if a height map "
         "of the area exists — for India these are available from "
         "national sources such as ISRO's Bhuvan portal — plus one known "
         "reference point. With map anchoring, even that point comes from "
         "the satellite image."),
        ("What computer does it need?", "Tests run on a desktop graphics "
         "computer. The target is an NVIDIA Jetson AGX Orin — a "
         "palm-sized computer designed for robots."),
    ]
    for q, a in faq:
        s.append(KeepTogether([P(f"<b>{q}</b>", ParagraphStyle_(
            pb, spaceAfter=1, textColor=L.NAVY)), Q(a)]))
    s.append(P("Mini glossary", L.pb_h2))
    s.append(table([
        ["Word", "Plain meaning"],
        ["GPS / GNSS", "Satellite positioning. QSLAM does not use it."],
        ["UTM", "A world-wide grid of coordinates in metres (easting, "
         "northing)."],
        ["DEM / GeoTIFF", "A height map of the ground, stored as an image "
         "file that knows where on Earth it is."],
        ["SLAM / VSLAM", "Working out where you are while mapping what you "
         "see; “V” means using a camera."],
        ["Kalman filter (EKF)", "The maths that blends several imperfect "
         "sensors into one best position estimate."],
        ["Lidar", "A laser scanner that measures distances to nearby "
         "objects."],
        ["IMU", "A motion sensor that feels turning and acceleration, like "
         "the inner ear."],
        ["ROS 2 / Nav2", "Standard open-source robot software on which "
         "QSLAM is built."],
        ["Simulation", "A physics-accurate virtual world used to test the "
         "robot safely and repeatably."],
    ], [118, TEXT_W - 118], size=9.2, first_col_bold=True))
    return s


def ParagraphStyle_(parent, **kw):
    from reportlab.lib.styles import ParagraphStyle
    return ParagraphStyle("x", parent=parent, **kw)


# ---------------------------------------------------------------- build --
def flatten(items):
    out = []
    for it in items:
        if isinstance(it, list):
            out += flatten(it)
        else:
            out.append(it)
    return out


def build_story():
    story = cover() + [NextPageTemplate("body"), PageBreak()]
    story += contents() + [PageBreak()]
    story += part_a()
    story += [PageBreak()] + part_b()
    return flatten(story)


def main():
    prepare_images()
    build_story()               # numbering pass: fixes every figure number
    L.reset_counters()
    story = build_story()
    doc = L.DocTemplate(OUT, title="QSLAM — Detailed System Description "
                        "(SIH26126)", author="Team QSLAM (Team ID 14)",
                        subject="GPS-denied vision-based UGV navigation")
    doc.multiBuild(story)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
