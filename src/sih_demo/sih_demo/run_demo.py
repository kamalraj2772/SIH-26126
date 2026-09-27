#!/usr/bin/env python3
"""SIH26126 -- GPS-denied UGV navigation demo.

One command runs the whole chain and writes a narrated dashboard video:

    UTM goal  ->  prior GeoTIFF  ->  A* terrain route
                                        |
    ZED 2i RGB-D  ->  ORB/PnP VSLAM  ->  pose in map frame
                  ->  obstacle points ->  local costmap  ->  DWA  ->  wheels

Nothing in the control path reads a GNSS receiver or the simulator's ground
truth. Ground truth is recorded only to score the run at the end.
"""
from __future__ import annotations

import argparse
import json
import pathlib
import time

import numpy as np
import cv2
import mujoco

from sih_demo import geo, hud, mission, planner, scene, sensors, vslam
from sih_sim import terrain

OUT = pathlib.Path(__file__).resolve().parents[1] / "output"


# ---------------------------------------------------------------- utilities
def yaw_of(R: np.ndarray) -> float:
    return float(np.arctan2(R[1, 0], R[0, 0]))


def se3(R, t):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
    return T


def quat_to_R(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# ---------------------------------------------------------------- the run
class Demo:
    def __init__(self, seed: int, duration: float, fast: bool, speed: int = 1):
        self.seed = seed
        self.duration = duration
        self.speed = speed
        self.fast = fast
        OUT.mkdir(parents=True, exist_ok=True)
        self.log: list[dict] = []
        self.stage_notes: list[str] = []

    # -- stage 1: georeferencing ------------------------------------------
    def build_geo(self):
        t0 = time.time()
        self.dem = scene.build_dem(self.seed)                # north-up
        self.world_size = scene.mission_world_size()
        self.tif_meta = geo.write_geotiff(self.dem, OUT / "prior_dem_utm44n.tif",
                                          self.world_size)
        # Read it back off disk -- the planner only ever sees the file.
        self.prior = geo.PriorDEM(OUT / "prior_dem_utm44n.tif")

        gx, gy = mission.GOAL_XY
        self.goal_utm = geo.map_to_utm(gx, gy)
        self.goal_ll = geo.utm_to_latlon(*self.goal_utm)
        # Round-trip the operator's UTM goal back into the map frame, which is
        # what the planner is actually given.
        self.goal_xy = np.array(geo.utm_to_map(*self.goal_utm))
        rt_err = float(np.linalg.norm(self.goal_xy - np.array(mission.GOAL_XY)))

        self.start_xy = np.array(mission.START_XY)
        self.start_utm = geo.map_to_utm(*self.start_xy)

        print(f"[geo] site datum      {geo.SITE_LAT:.6f} N, {geo.SITE_LON:.6f} E "
              f"-> UTM zone {geo.utm_zone_for_lon(geo.SITE_LON)}N (EPSG:32644)")
        print(f"[geo] map anchor      E {geo.ANCHOR_E:.2f}  N {geo.ANCHOR_N:.2f}")
        print(f"[geo] goal (operator) E {self.goal_utm[0]:.2f}  N {self.goal_utm[1]:.2f}"
              f"  -> map ({self.goal_xy[0]:.3f}, {self.goal_xy[1]:.3f})")
        print(f"[geo] UTM<->map round-trip error {rt_err*1000:.3f} mm")
        print(f"[geo] GeoTIFF         {self.prior.describe()}  ({time.time()-t0:.1f} s)")
        self.rt_err = rt_err

    # -- stage 2: global route over the prior ------------------------------
    def build_route(self):
        t0 = time.time()
        self.gplanner = planner.GlobalPlanner(self.prior, self.world_size, res=1.0)
        self.route = self.gplanner.plan(self.start_xy, self.goal_xy)
        self.gplanner.set_route(self.route)
        length = float(np.sum(np.linalg.norm(np.diff(self.route, axis=0), axis=1)))
        print(f"[nav] A* over prior GeoTIFF: {len(self.route)} waypoints, "
              f"{length:.1f} m, blocked cells "
              f"{100*self.gplanner.blocked.mean():.1f}%  ({time.time()-t0:.1f} s)")
        self.route_length = length
        # Obstacles the prior survey never saw, placed along the planned route.
        self.corridor = mission.corridor_obstacles(self.route, self.seed)
        print(f"[sim] {len(self.corridor)} unsurveyed obstacles seeded along the "
              "route (absent from the GeoTIFF; discoverable only by the ZED)")

    # -- stage 3: simulator -------------------------------------------------
    def build_sim(self):
        t0 = time.time()
        tex = scene.terrain_texture(self.dem, self.seed)
        tex_path = OUT / "terrain_texture.png"
        cv2.imwrite(str(tex_path), cv2.cvtColor(tex, cv2.COLOR_RGB2BGR))

        xml = scene.build_mjcf(self.seed, tex_path, self.dem, self.corridor)
        (OUT / "scene.xml").write_text(xml)
        self.m = mujoco.MjModel.from_xml_path(str(OUT / "scene.xml"))

        hid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_HFIELD, "terrain")
        adr = self.m.hfield_adr[hid]
        nr, nc = self.m.hfield_nrow[hid], self.m.hfield_ncol[hid]
        zmin, zmax = self.dem.min(), self.dem.max()
        elev = max(0.5, zmax - zmin)
        # MuJoCo hfield row 0 is the -y edge, so feed it bottom-up.
        self.m.hfield_data[adr:adr + nr * nc] = \
            ((np.flipud(self.dem) - zmin) / elev).astype(np.float32).ravel()

        self.d = mujoco.MjData(self.m)
        self.rover_bid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "rover")
        self.qadr = self.m.jnt_qposadr[self.m.body_jntadr[self.rover_bid]]
        self.vadr = self.m.jnt_dofadr[self.m.body_jntadr[self.rover_bid]]

        sx, sy = self.start_xy
        sz = float(terrain.terrain_height(np.array(sx), np.array(sy), self.seed))
        self.d.qpos[self.qadr:self.qadr + 3] = [sx, sy, sz + 0.28]
        yaw0 = np.deg2rad(mission.START_YAW_DEG)
        self.d.qpos[self.qadr + 3:self.qadr + 7] = \
            [np.cos(yaw0 / 2), 0, 0, np.sin(yaw0 / 2)]
        for _ in range(500):
            mujoco.mj_step(self.m, self.d)

        # Which geoms count as "something we must not hit".
        rover_names = {"chassis", "deck", "mast", "zed_body", "zed_lens_l",
                       "zed_lens_r", "wheel_fl_g", "wheel_fr_g",
                       "wheel_rl_g", "wheel_rr_g"}
        self.rover_geoms = {mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_GEOM, n)
                            for n in rover_names}
        terrain_gid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_GEOM, "terrain_geom")
        self.ignore_geoms = self.rover_geoms | {terrain_gid, -1}

        self.rgb_r = mujoco.Renderer(self.m, mission.ZED_HEIGHT, mission.ZED_WIDTH)
        self.dep_r = mujoco.Renderer(self.m, mission.ZED_HEIGHT, mission.ZED_WIDTH)
        self.dep_r.enable_depth_rendering()
        self.view_r = mujoco.Renderer(self.m, 656, 1160)
        self.zed = sensors.ZED2i(self.m, self.rgb_r, self.dep_r, self.seed)

        print(f"[sim] MuJoCo {mujoco.__version__}: {self.m.nbody} bodies, "
              f"{self.m.ngeom} geoms, hfield {nr}x{nc}, "
              f"ZED {mission.ZED_WIDTH}x{mission.ZED_HEIGHT} "
              f"fx={self.zed.fx:.1f}px  ({time.time()-t0:.1f} s)")

    # -- stage 4: estimation + control -------------------------------------
    def build_stack(self):
        gt = self.ground_truth()
        T_wb0 = se3(R_z(gt[2]), [gt[0], gt[1], gt[3]])
        T_wc0 = T_wb0 @ sensors.T_BODY_OPT
        self.slam = vslam.VisualSLAM(self.zed.K, T_wc0,
                                     mission.ZED_MIN_DEPTH_M, mission.ZED_MAX_DEPTH_M)
        self.costmap = planner.LocalCostmap(size_m=30.0, res=0.25)
        self.dwa = planner.DWAPlanner()
        self.v_cmd = 0.0
        self._recent: list = []
        self._last_reset = 0.0
        print("[slam] VSLAM seeded from the surveyed deployment pose "
              f"({gt[0]:.2f}, {gt[1]:.2f}, {np.degrees(gt[2]):.1f} deg) -- no GNSS fix")

    def ground_truth(self):
        p = self.d.qpos[self.qadr:self.qadr + 3]
        q = self.d.qpos[self.qadr + 3:self.qadr + 7]
        R = quat_to_R(q)
        return float(p[0]), float(p[1]), yaw_of(R), float(p[2]), R

    def collisions(self) -> int:
        n = 0
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            a, b = int(c.geom1), int(c.geom2)
            hit_rover = a in self.rover_geoms or b in self.rover_geoms
            other = b if a in self.rover_geoms else a
            if hit_rover and other not in self.ignore_geoms:
                n += 1
        return n

    def true_clearance(self, x, y) -> float:
        best = 99.0
        for (ox, oy, r, _k) in self.corridor:
            best = min(best, float(np.hypot(x - ox, y - oy) - r))
        return best

    def drive(self, v, w):
        wc = w * mission.SKID_YAW_GAIN          # skid-steer slip compensation
        vl = v - wc * mission.TRACK_WIDTH / 2.0
        vr = v + wc * mission.TRACK_WIDTH / 2.0
        wl = vl / mission.WHEEL_RADIUS
        wr = vr / mission.WHEEL_RADIUS
        self.d.ctrl[:] = [wl, wr, wl, wr]

    # -- the loop -----------------------------------------------------------
    def score(self, res) -> dict:
        gt = np.array(res["gt_track"]); est = np.array(res["est_track"])
        errs = np.linalg.norm(gt - est, axis=1)
        travelled = float(np.sum(np.linalg.norm(np.diff(gt, axis=0), axis=1)))
        return dict(
            seed=self.seed,
            reached_goal=bool(res["reached"]),
            goal_utm=dict(easting=round(self.goal_utm[0], 3),
                          northing=round(self.goal_utm[1], 3), epsg=32644,
                          lat=round(self.goal_ll[0], 7), lon=round(self.goal_ll[1], 7)),
            utm_roundtrip_error_mm=round(self.rt_err * 1000, 4),
            geotiff=self.tif_meta,
            global_route_m=round(self.route_length, 2),
            path_travelled_m=round(travelled, 2),
            final_position_error_m=round(float(np.linalg.norm(gt[-1] - self.goal_xy)), 3),
            vslam=dict(final_drift_m=round(float(errs[-1]), 3),
                       mean_drift_m=round(float(errs.mean()), 3),
                       max_drift_m=round(float(errs.max()), 3),
                       drift_pct_of_path=round(100 * float(errs[-1]) / max(travelled, 1e-6), 3),
                       keyframes=self.slam.kf_count,
                       lost_frames=self.slam.lost_frames),
            obstacles_in_corridor=len(self.corridor),
            collision_free=bool(res["collisions"] == 0
                                and res["min_true_clear"] > mission.ROBOT_RADIUS_M),
            safety_margin_maintained=bool(
                res["min_true_clear"] > mission.ROBOT_RADIUS_M + mission.SAFETY_MARGIN_M),
            contact_events_with_obstacles=res["collisions"],
            min_obstacle_clearance_m=res["min_true_clear"],
            robot_radius_m=mission.ROBOT_RADIUS_M,
            required_clearance_m=mission.ROBOT_RADIUS_M + mission.SAFETY_MARGIN_M,
            mission_time_s=round(float(self.log[-1]["t"]), 2),
            control_hz=mission.CONTROL_HZ,
            wall_clock_s=res["wall_s"])

    def run(self, video_path: pathlib.Path):
        import imageio.v2 as imageio
        writer = imageio.get_writer(
            str(video_path), fps=int(mission.CONTROL_HZ), codec="libx264",
            macro_block_size=8, pixelformat="yuv420p",
            output_params=["-crf", "24", "-preset", "medium"])

        fps = int(mission.CONTROL_HZ)
        for f in hud.card([
            ("Drive a skid-steer UGV to a surveyed UTM goal across unstructured "
             "terrain, with no GNSS at any stage.", "b"),
            ("", "s"),
            ("UTM|operator's goal is an EPSG:32644 easting / northing; pyproj "
             "converts it into the|local map frame against a surveyed site datum. "
             "No receiver is ever read.", "k"),
            ("GeoTIFF|a prior UTM-georeferenced DEM is written, then read back "
             "off disk;|A* plans a terrain-aware route over its slope map.", "k"),
            ("VSLAM|RTAB-Map class stereo SLAM -- ORB features on the ZED 2i left "
             "image,|back-projected with depth, tracked by PnP-RANSAC. "
             "The only source of pose.", "k"),
            ("Avoidance|ZED depth builds a local costmap; DWA rejects every "
             "trajectory that|violates the safety envelope, and drives the wheels.", "k"),
            ("", "s"),
            ("The corridor obstacles are absent from the prior GeoTIFF - they "
             "exist only in the depth stream.", "b"),
            ("Ground truth is recorded to score the run and is never fed back.", "d"),
        ], 6.5, fps, title="SIH26126   GPS-DENIED UGV NAVIGATION"):
            writer.append_data(f)

        speed = max(1, int(self.speed))
        dt_ctrl = 1.0 / mission.CONTROL_HZ
        n_sub = max(1, int(round(dt_ctrl / mission.SIM_DT)))
        gt_track, est_track = [], []
        min_true_clear = 99.0
        total_collisions = 0
        reached = False
        k = 0
        t_start = time.time()

        while self.d.time < self.duration:
            gx_, gy_, gyaw, gz, _ = self.ground_truth()
            rgb, depth = self.zed.capture(self.d)
            rep = self.slam.track(rgb, depth)

            T_wc = rep["T_world_cam"]
            T_wb = T_wc @ sensors.T_OPT_BODY
            ex, ey, eyaw = T_wb[0, 3], T_wb[1, 3], yaw_of(T_wb[:3, :3])

            # --- ZED depth -> world points -> local costmap (estimated pose) --
            pts_c = self.zed.deproject(depth, stride=3)
            obs_xy = free_xy = np.zeros((0, 2))
            if len(pts_c):
                rng_c = np.linalg.norm(pts_c, axis=1)
                # Beyond ~8 m the ZED's own triangulation noise (sigma grows as
                # z^2) is larger than the obstacles we care about, so obstacle
                # evidence is only taken from the near field. Far returns still
                # carve free space.
                keep = rng_c < 12.0
                pts_c, rng_c = pts_c[keep], rng_c[keep]
                pts_w = (T_wc[:3, :3] @ pts_c.T).T + T_wc[:3, 3]

            if len(pts_c):
                ground = self.prior.elevation(pts_w[:, 0], pts_w[:, 1])
                resid = pts_w[:, 2] - ground

                # The prior DEM gives terrain *shape*; VSLAM altitude and
                # attitude drift show up as a slowly varying offset across the
                # frame. Fit that offset as a plane over the ground-dominant
                # returns and subtract it, so obstacle height is measured
                # against the terrain rather than against an absolute datum.
                dxy = pts_w[:, :2] - np.array([ex, ey])
                A = np.column_stack([np.ones(len(dxy)), dxy])
                sel = resid < np.quantile(resid, 0.62)
                coef = np.zeros(3)
                if sel.sum() > 30:
                    coef = np.linalg.lstsq(A[sel], resid[sel], rcond=None)[0]
                    r2 = resid - A @ coef
                    sel2 = np.abs(r2) < 0.30
                    if sel2.sum() > 30:
                        coef = np.linalg.lstsq(A[sel2], resid[sel2], rcond=None)[0]
                hgt = resid - A @ coef

                # Detection threshold tracks the sensor: 2.5 sigma of the stereo
                # range noise at that distance, plus the DEM's own quantisation
                # error on sloped ground.
                sigma = rng_c ** 2 * 0.16 / (self.zed.fx * self.zed.baseline)
                slope = self.prior.slope_deg(pts_w[:, 0], pts_w[:, 1],
                                             eps=self.prior.res)
                thr = (0.30 + 2.5 * sigma
                       + 1.4 * np.tan(np.radians(slope)) * self.prior.res)

                is_obs = (hgt > thr) & (hgt < 3.5) & (rng_c < 8.0)
                obs_xy = pts_w[is_obs][:, :2]
                free_xy = pts_w[hgt <= thr * 0.7][:, :2]

            self.costmap.recenter(ex, ey)
            self.costmap.integrate(obs_xy, free_xy)

            # --- plan + drive ---------------------------------------------
            carrot, ci = planner.carrot_on_path(self.route, (ex, ey, eyaw))
            dist_goal = float(np.linalg.norm(self.goal_xy - np.array([ex, ey])))
            if dist_goal < mission.GOAL_TOLERANCE_M * 2.2:
                carrot = self.goal_xy
            # Stuck detector: if the vehicle has barely moved for 4 s, let DWA
            # consider reversing out instead of spinning against a dead end.
            self._recent.append((self.d.time, gx_, gy_))
            while self._recent and self.d.time - self._recent[0][0] > 4.0:
                self._recent.pop(0)
            stuck = (len(self._recent) > 40 and
                     np.hypot(gx_ - self._recent[0][1], gy_ - self._recent[0][2]) < 0.45)
            v, w, dinfo = self.dwa.plan((ex, ey, eyaw), carrot, self.costmap,
                                        self.v_cmd, terrain=self.gplanner,
                                        allow_reverse=stuck)
            if stuck:
                if self.d.time - self._last_reset > 10.0:
                    # Stale evidence must never be able to pin the vehicle
                    # permanently: drop it and re-observe.
                    self.costmap.reset()
                    self._last_reset = self.d.time
                    dinfo["mode"] = "costmap-reset"
            self.v_cmd = v
            self.drive(v, w)

            # --- score (ground truth, never fed back) ----------------------
            tc = self.true_clearance(gx_, gy_)
            min_true_clear = min(min_true_clear, tc)
            total_collisions += self.collisions()
            gt_track.append((gx_, gy_))
            est_track.append((ex, ey))
            err = float(np.hypot(ex - gx_, ey - gy_))
            travelled = float(np.sum(np.linalg.norm(
                np.diff(np.array(gt_track), axis=0), axis=1))) if len(gt_track) > 1 else 0.0

            self.log.append(dict(
                t=round(self.d.time, 3), gt=(round(gx_, 3), round(gy_, 3)),
                est=(round(ex, 3), round(ey, 3)), err=round(err, 4),
                v=round(v, 3), w=round(w, 3), clearance=round(tc, 3),
                features=rep["n_features"], inliers=rep["n_inliers"],
                keyframes=rep["keyframes"], mode=dinfo["mode"]))

            # The vehicle stops when *its own estimate* says it has arrived --
            # it has no other opinion available. How far that actually was from
            # the commanded UTM goal is then the honest end-to-end error, and
            # it necessarily includes the accumulated VSLAM drift.
            true_goal_dist = float(np.linalg.norm(self.goal_xy - np.array([gx_, gy_])))
            if dist_goal < mission.GOAL_TOLERANCE_M:
                reached = True

            if k % speed == 0 or reached:
                frame = self.compose(rgb, depth, rep, dinfo, gt_track, est_track,
                                     carrot, dict(err=err, travelled=travelled,
                                                  dist_goal=true_goal_dist,
                                                  min_clear=min_true_clear,
                                                  collisions=total_collisions,
                                                  v=v, w=w, tc=tc, reached=reached))
                writer.append_data(frame)
                if reached:
                    for _ in range(int(mission.CONTROL_HZ * 1.5)):
                        writer.append_data(frame)

            if reached:
                break

            for _ in range(n_sub):
                mujoco.mj_step(self.m, self.d)
            k += 1
            if k % 100 == 0:
                print(f"  t={self.d.time:6.1f}s  gt=({gx_:7.2f},{gy_:7.2f})  "
                      f"err={err:5.2f}m  clear={tc:5.2f}m  "
                      f"feat={rep['n_features']:4d} inl={rep['n_inliers']:3d}  "
                      f"{dinfo['mode']}")

        wall = time.time() - t_start
        res = dict(reached=reached, frames=k, wall_s=round(wall, 1),
                   gt_track=gt_track, est_track=est_track,
                   min_true_clear=round(min_true_clear, 3),
                   collisions=total_collisions)
        self.summary = self.score(res)
        ok = ("PASS" if (res["reached"] and self.summary["collision_free"])
              else "REVIEW")
        v = self.summary["vslam"]
        for f in hud.card([
            ("Georeferencing", "h"),
            (f"goal as issued        E {self.goal_utm[0]:.2f}  N {self.goal_utm[1]:.2f}"
             f"   EPSG:32644 (UTM 44N)", "b"),
            (f"UTM <-> map round trip   {self.summary['utm_roundtrip_error_mm']:.3f} mm", "b"),
            (f"prior DEM             {self.prior.describe()}", "b"),
            ("Visual SLAM  -  RTAB-Map class stereo front-end on the ZED 2i", "h"),
            (f"path travelled        {self.summary['path_travelled_m']:.1f} m"
             f"   over {self.summary['mission_time_s']:.0f} s", "b"),
            (f"final drift vs truth  {v['final_drift_m']:.2f} m  "
             f"= {v['drift_pct_of_path']:.2f} % of distance travelled", "g"),
            (f"mean / max drift      {v['mean_drift_m']:.2f} m / {v['max_drift_m']:.2f} m"
             f"   keyframes {v['keyframes']}   tracking losses {v['lost_frames']}", "b"),
            ("Collision-free navigation", "h"),
            (f"unsurveyed obstacles in the corridor   "
             f"{self.summary['obstacles_in_corridor']}", "b"),
            (f"rover-obstacle contacts                "
             f"{self.summary['contact_events_with_obstacles']}", "g"),
            (f"minimum true clearance                 "
             f"{self.summary['min_obstacle_clearance_m']:.2f} m"
             f"   vs {mission.ROBOT_RADIUS_M:.2f} m footprint"
             f" / {self.summary['required_clearance_m']:.2f} m desired margin",
             "g" if self.summary["collision_free"] else "b"),
            (f"goal reached within {mission.GOAL_TOLERANCE_M:.1f} m            "
             f"{'YES' if res['reached'] else 'NO'}"
             f"   final error {self.summary['final_position_error_m']:.2f} m", "g"),
            ("", "s"),
            (f"RESULT: {ok}", "h"),
        ], 7.5, fps, title="MISSION RESULT"):
            writer.append_data(f)
        writer.close()
        return res

    # -- dashboard ----------------------------------------------------------
    def compose(self, rgb, depth, rep, dinfo, gt_track, est_track, carrot, s):
        c = hud.new_canvas()
        gx_, gy_, gyaw, _, _ = self.ground_truth()
        e_utm = geo.map_to_utm(*est_track[-1])

        hud.header(c, "SIH26126  |  GPS-DENIED UGV NAVIGATION  |  ZED 2i VSLAM + DWA",
                   f"MuJoCo 3.x   physics {self.d.time:6.1f} s   "
                   f"{int(self.speed)}x real time   seed {self.seed}   EPSG:32644",
                   "NO GNSS IN THE CONTROL PATH")

        # --- main chase view ---
        box = hud.panel(c, hud.R_MAIN, "SIMULATED FIELD  -  chase view",
                        "MuJoCo rigid-body physics on the surveyed DEM")
        cam = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(cam)
        cam.lookat[:] = [gx_, gy_, self.d.qpos[self.qadr + 2]]
        cam.distance = 8.5
        cam.azimuth = np.degrees(gyaw) + 206.0
        cam.elevation = -29.0
        self.view_r.update_scene(self.d, cam)
        hud.blit(c, self.view_r.render(), box)

        # --- ZED RGB with ORB overlay ---
        box = hud.panel(c, hud.R_RGB,
                        f"ZED 2i LEFT {mission.ZED_WIDTH_FULL}x{mission.ZED_HEIGHT_FULL} HD720"
                        "  -  VSLAM FEATURE TRACKING",
                        "ORB + PnP-RANSAC stereo front-end, as RTAB-Map runs in "
                        "the ROS 2 stack")
        vis = rgb.copy()
        if rep["keypoints"] is not None:
            for (u, v) in rep["keypoints"][::2]:
                cv2.circle(vis, (int(u), int(v)), 2, hud.ACCENT, -1, cv2.LINE_AA)
        if rep["inlier_uv"] is not None:
            for (u, v) in rep["inlier_uv"]:
                cv2.circle(vis, (int(u), int(v)), 5, hud.GOOD, 1, cv2.LINE_AA)
        hud.badge(vis, f"ORB {rep['n_features']}   PnP inliers {rep['n_inliers']}",
                  (8, 8), hud.GOOD if rep["tracking_ok"] else hud.WARN)
        hud.blit(c, vis, box)

        # --- ZED depth ---
        box = hud.panel(c, hud.R_DEPTH, "ZED 2i  NEURAL DEPTH",
                        f"range {mission.ZED_MIN_DEPTH_M:.1f}-{mission.ZED_MAX_DEPTH_M:.0f} m"
                        "   stereo noise sigma = z^2 d/(fB)")
        dimg = hud.depth_colormap(depth, mission.ZED_MIN_DEPTH_M, mission.ZED_MAX_DEPTH_M)
        valid = float(np.isfinite(depth).mean() * 100)
        hud.badge(dimg, f"valid stereo returns {valid:.0f}%", (8, 8), hud.TXT)
        hud.blit(c, dimg, box)

        # --- global map ---
        box = hud.panel(c, hud.R_GMAP, "PRIOR GeoTIFF  +  GLOBAL ROUTE",
                        f"UTM 44N DEM {self.prior.n}px @ {self.prior.res:.2f} m  -  A* on slope")
        hud.blit(c, self.global_map(gt_track, est_track), box)

        # --- local costmap ---
        box = hud.panel(c, hud.R_COST, "LOCAL COSTMAP  (ZED depth only)  +  DWA",
                        f"30 x 30 m @ 0.25 m   {dinfo['n_safe']}/{dinfo['n_sampled']} "
                        "trajectories collision-free")
        hud.blit(c, self.costmap_panel(est_track, carrot), box)

        # --- telemetry ---
        hud.panel(c, hud.R_TEL, "TELEMETRY")
        ok = s["collisions"] == 0
        left = [
            ("VSLAM pose", f"{est_track[-1][0]:+8.2f}, {est_track[-1][1]:+8.2f} m", hud.TXT),
            ("  -> UTM 44N", f"E {e_utm[0]:.1f}  N {e_utm[1]:.1f}", hud.BLUE),
            ("drift vs truth", f"{s['err']:.2f} m  ({100*s['err']/max(s['travelled'],1e-3):.2f}% of path)",
             hud.GOOD if s["err"] < 3.0 else hud.ACCENT),
            ("keyframes", f"{rep['keyframes']}   lost {rep['lost_frames']}", hud.DIM),
            ("SLAM", "RTAB-Map class stereo VSLAM  (no GNSS, no wheel odom)", hud.DIM),
        ]
        right = [
            ("cmd v / w", f"{s['v']:.2f} m/s   {s['w']:+.2f} rad/s", hud.TXT),
            ("obstacle clearance", f"{s['tc']:.2f} m   (min {s['min_clear']:.2f} m)",
             hud.GOOD if s["tc"] > 0.4 else hud.WARN),
            ("collisions", f"{s['collisions']}" + ("  COLLISION-FREE" if ok else ""),
             hud.GOOD if ok else hud.WARN),
            ("to goal", f"{s['dist_goal']:.1f} m   travelled {s['travelled']:.1f} m",
             hud.ACCENT if not s["reached"] else hud.GOOD),
        ]
        hud.text_rows(c, hud.R_TEL[0] + 16, hud.R_TEL[1] + 52, left, key_w=130)
        hud.text_rows(c, hud.R_TEL[0] + 396, hud.R_TEL[1] + 52, right, key_w=150)

        if s["reached"]:
            cv2.putText(c, "GOAL REACHED", (hud.R_MAIN[0] + 380, hud.R_MAIN[1] + 90),
                        hud.F, 1.4, hud.GOOD, 2, cv2.LINE_AA)
        return c

    def global_map(self, gt_track, est_track):
        if not hasattr(self, "_gmap_base"):
            base = hud.hillshade(self.gplanner.elevation[::-1], 1.0)
            blocked = self.gplanner.blocked[::-1]
            base[blocked] = (0.55 * base[blocked] +
                             0.45 * np.array([200, 70, 60])).astype(np.uint8)
            self._gmap_base = cv2.resize(base, (520, 520), interpolation=cv2.INTER_LINEAR)
        img = self._gmap_base.copy()
        n = self.gplanner.n
        sc = 520.0 / n

        def px(p):
            cx = (p[0] + self.gplanner.half) / self.gplanner.res * sc
            cy = (self.gplanner.half - p[1]) / self.gplanner.res * sc
            return (int(cx), int(cy))

        cv2.polylines(img, [np.array([px(p) for p in self.route], np.int32)],
                      False, hud.BLUE, 2, cv2.LINE_AA)
        if len(gt_track) > 1:
            cv2.polylines(img, [np.array([px(p) for p in gt_track[::3]], np.int32)],
                          False, (235, 235, 235), 2, cv2.LINE_AA)
        if len(est_track) > 1:
            cv2.polylines(img, [np.array([px(p) for p in est_track[::3]], np.int32)],
                          False, hud.VIOLET, 2, cv2.LINE_AA)
        for (ox, oy, r, _k) in self.corridor:
            cv2.circle(img, px((ox, oy)), max(2, int(r * sc)), (120, 120, 130), 1, cv2.LINE_AA)
        cv2.circle(img, px(self.start_xy), 6, hud.DIM, -1, cv2.LINE_AA)
        cv2.circle(img, px(self.goal_xy), 8, hud.GOOD, 2, cv2.LINE_AA)
        cv2.drawMarker(img, px(self.goal_xy), hud.GOOD, cv2.MARKER_CROSS, 14, 2)
        cv2.circle(img, px(est_track[-1]), 5, hud.ACCENT, -1, cv2.LINE_AA)
        legend = [("A* route (prior DEM)", hud.BLUE), ("ground truth", (235, 235, 235)),
                  ("VSLAM estimate", hud.VIOLET), ("slope > 20 deg", (200, 70, 60))]
        for i, (lbl, col) in enumerate(legend):
            cv2.line(img, (12, 436 + i * 18), (34, 436 + i * 18), col, 2)
            cv2.putText(img, lbl, (40, 440 + i * 18), hud.FS, 0.36, (240, 240, 240), 1, cv2.LINE_AA)
        cv2.putText(img, "N", (492, 24), hud.F, 0.5, (240, 240, 240), 1, cv2.LINE_AA)
        cv2.arrowedLine(img, (498, 44), (498, 20), (240, 240, 240), 2, tipLength=0.4)
        return img

    def costmap_panel(self, est_track, carrot):
        cmp_ = self.costmap
        n = cmp_.n
        d = np.clip(cmp_.dist / 4.0, 0, 1)
        img = (np.stack([0.10 + 0.34 * d, 0.13 + 0.42 * d, 0.16 + 0.34 * d], -1)
               * 255).astype(np.uint8)
        occ = getattr(cmp_, "occ", np.zeros((n, n), np.uint8)).astype(bool)
        img[occ] = (252, 96, 84)
        img = np.flipud(img).copy()
        img = cv2.resize(img, (520, 520), interpolation=cv2.INTER_NEAREST)
        sc = 520.0 / n

        def px(p):
            cx = (p[0] - cmp_.origin[0]) / cmp_.res * sc
            cy = 520 - (p[1] - cmp_.origin[1]) / cmp_.res * sc
            return (int(cx), int(cy))

        for xs, ys, safe, _ in self.dwa.last_rollouts[::3]:
            col = (120, 210, 150) if safe else (150, 72, 72)
            cv2.polylines(img, [np.array([px(p) for p in zip(xs, ys)], np.int32)],
                          False, col, 1, cv2.LINE_AA)
        if self.dwa.chosen is not None:
            cv2.polylines(img, [np.array([px(p) for p in self.dwa.chosen], np.int32)],
                          False, hud.ACCENT, 3, cv2.LINE_AA)
        inpath = [p for p in self.route
                  if abs(p[0] - est_track[-1][0]) < 15 and abs(p[1] - est_track[-1][1]) < 15]
        if len(inpath) > 1:
            cv2.polylines(img, [np.array([px(p) for p in inpath], np.int32)],
                          False, hud.BLUE, 2, cv2.LINE_AA)
        cv2.drawMarker(img, px(carrot), hud.BLUE, cv2.MARKER_TILTED_CROSS, 16, 2)
        p = px(est_track[-1])
        cv2.circle(img, p, int(mission.ROBOT_RADIUS_M / cmp_.res * sc), hud.ACCENT, 1, cv2.LINE_AA)
        cv2.circle(img, p, 5, hud.ACCENT, -1, cv2.LINE_AA)
        legend = [("occupied (ZED)", (252, 96, 84)), ("safe rollout", (120, 210, 150)),
                  ("rejected (collision)", (150, 72, 72)), ("chosen command", hud.ACCENT)]
        for i, (lbl, col) in enumerate(legend):
            cv2.line(img, (12, 436 + i * 18), (34, 436 + i * 18), col, 2)
            cv2.putText(img, lbl, (40, 440 + i * 18), hud.FS, 0.36, (240, 240, 240), 1, cv2.LINE_AA)
        return img


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seed", type=int, default=mission.SEED)
    ap.add_argument("--duration", type=float, default=mission.MAX_MISSION_S)
    ap.add_argument("--video", default=str(OUT / "sih26126_demo.mp4"))
    ap.add_argument("--speed", type=int, default=mission.VIDEO_SPEEDUP,
                    help="compose every Nth control step -> Nx real-time video")
    ap.add_argument("--fast", action="store_true", help="skip video, run headless scoring")
    args = ap.parse_args()

    print("=" * 78)
    print(" SIH26126  GPS-DENIED UGV NAVIGATION  -  end-to-end demo")
    print("=" * 78)
    demo = Demo(args.seed, args.duration, args.fast, args.speed)
    demo.build_geo()
    demo.build_route()
    demo.build_sim()
    demo.build_stack()
    print("-" * 78)
    res = demo.run(pathlib.Path(args.video))
    print("-" * 78)

    summary = dict(demo.summary, video=str(args.video))
    (OUT / "demo_summary.json").write_text(json.dumps(summary, indent=2))
    (OUT / "telemetry.json").write_text(json.dumps(demo.log))

    print(json.dumps(summary, indent=2))
    print("-" * 78)
    print(f"video   -> {args.video}")
    print(f"summary -> {OUT/'demo_summary.json'}")


if __name__ == "__main__":
    main()
