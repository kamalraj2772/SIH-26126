"""ZED 2i model: renders the left/right imagers out of MuJoCo and turns the
rendered z-buffer into a depth map with the ZED's real limits and noise.

The depth returned here is what the rest of the stack is allowed to use. The
simulator's ground truth is never passed to VSLAM, the costmap or the planner
-- it is only recorded, at the very end, to score the run.
"""
from __future__ import annotations

import numpy as np

from sih_demo import mission


class ZED2i:
    def __init__(self, model, renderer_rgb, renderer_depth, seed: int = 0):
        self.m = model
        self._rgb = renderer_rgb
        self._depth = renderer_depth
        self.w = mission.ZED_WIDTH
        self.h = mission.ZED_HEIGHT
        self.rng = np.random.RandomState(seed + 991)

        fovy = np.deg2rad(_camera_fovy(model, "zed_left"))
        self.fy = (self.h / 2.0) / np.tan(fovy / 2.0)
        self.fx = self.fy                      # square pixels
        self.cx = (self.w - 1) / 2.0
        self.cy = (self.h - 1) / 2.0
        self.K = np.array([[self.fx, 0, self.cx],
                           [0, self.fy, self.cy],
                           [0, 0, 1.0]], dtype=np.float64)
        self.baseline = mission.ZED_BASELINE_M

        # Pixel ray directions in the optical frame (z forward, x right, y down).
        us, vs = np.meshgrid(np.arange(self.w), np.arange(self.h))
        self._ray_x = (us - self.cx) / self.fx
        self._ray_y = (vs - self.cy) / self.fy

    def capture(self, data) -> tuple[np.ndarray, np.ndarray]:
        """Return (rgb uint8 HxWx3, depth float32 HxW in metres, NaN = no return)."""
        self._rgb.update_scene(data, "zed_left")
        rgb = self._rgb.render().copy()

        self._depth.update_scene(data, "zed_left")
        z = self._depth.render().astype(np.float64).copy()

        # A stereo camera reports nothing beyond its range or on the sky.
        invalid = ~np.isfinite(z) | (z < mission.ZED_MIN_DEPTH_M) | (z > mission.ZED_MAX_DEPTH_M)

        # Stereo triangulation error: sigma_z = z^2 * sigma_disp / (f * B).
        sigma_disp = 0.16                                   # px, ZED 2i NEURAL-ish
        sigma_z = (z ** 2) * sigma_disp / (self.fx * self.baseline)
        z = z + self.rng.randn(*z.shape) * np.clip(sigma_z, 0.0, 0.8)

        # Speckle dropout, as on any correlation-based stereo matcher.
        z[self.rng.rand(*z.shape) < 0.012] = np.nan
        z[invalid] = np.nan
        return rgb, z.astype(np.float32)

    def deproject(self, depth: np.ndarray, stride: int = 4) -> np.ndarray:
        """Depth map -> Nx3 points in the optical frame (x right, y down, z fwd)."""
        z = depth[::stride, ::stride]
        rx = self._ray_x[::stride, ::stride]
        ry = self._ray_y[::stride, ::stride]
        ok = np.isfinite(z)
        z = z[ok]
        return np.stack([rx[ok] * z, ry[ok] * z, z], axis=1)

    def pixel_to_point(self, u, v, z):
        """Single-pixel deprojection, used by the VSLAM front-end."""
        x = (np.asarray(u, dtype=np.float64) - self.cx) / self.fx * z
        y = (np.asarray(v, dtype=np.float64) - self.cy) / self.fy * z
        return np.stack([x, y, np.asarray(z, dtype=np.float64)], axis=-1)


def _camera_fovy(model, name: str) -> float:
    import mujoco
    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
    return float(model.cam_fovy[cid])


# --- Static extrinsic: body (x fwd, y left, z up) -> ZED left optical frame ---
# REP-103 optical convention, kept explicitly separate from the body frame.
def body_to_optical() -> np.ndarray:
    p = np.deg2rad(mission.ZED_PITCH_DEG)
    fwd = np.array([np.cos(p), 0.0, np.sin(p)])            # optical +z
    right = np.array([0.0, -1.0, 0.0])                     # optical +x
    down = np.cross(fwd, right)                            # optical +y
    R = np.column_stack([right, down, fwd])                # body <- optical
    t = np.array([mission.ZED_MOUNT_XYZ[0] + 0.02,
                  mission.ZED_MOUNT_XYZ[1] + mission.ZED_BASELINE_M / 2.0,
                  mission.ZED_MOUNT_XYZ[2]])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T                                               # T_body_optical


T_BODY_OPT = body_to_optical()
T_OPT_BODY = np.linalg.inv(T_BODY_OPT)
