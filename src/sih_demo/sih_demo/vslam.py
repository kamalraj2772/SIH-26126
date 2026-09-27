"""Stereo-depth visual SLAM front-end for the ZED 2i.

This is a real front-end, not a replay of ground truth:

  1. ORB corners + BRIEF-style binary descriptors on the left image.
  2. Each keyframe corner is back-projected with the ZED depth map into a 3D
     landmark in that keyframe's optical frame.
  3. The next frame is matched against the keyframe (Hamming + Lowe ratio) and
     the camera pose is recovered by PnP-RANSAC on those 3D<->2D pairs.
  4. Keyframe-relative poses are chained into a trajectory, and every landmark
     is pushed into a world-frame sparse map.

The only state that comes from outside is the *surveyed deployment pose* used
to initialise the chain. No GNSS, no simulator pose feedback.
"""
from __future__ import annotations

import numpy as np

try:
    import cv2
except ImportError:                                   # pragma: no cover
    cv2 = None


def _se3(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).reshape(3)
    return T


class VisualSLAM:
    def __init__(self, K: np.ndarray, T_world_cam0: np.ndarray,
                 min_depth: float, max_depth: float):
        self.K = K
        self.T_world_cam = T_world_cam0.copy()        # current camera pose
        self.min_depth = min_depth
        self.max_depth = max_depth

        self.orb = cv2.ORB_create(
            nfeatures=1500, scaleFactor=1.2, nlevels=8,
            edgeThreshold=21, fastThreshold=12, patchSize=31)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        # Keyframe state
        self.kf_kp = None
        self.kf_des = None
        self.kf_pts3d = None                          # in the keyframe's optical frame
        self.T_world_kf = T_world_cam0.copy()
        self.kf_count = 0

        # Telemetry
        self.n_features = 0
        self.n_matches = 0
        self.n_inliers = 0
        self.tracking_ok = True
        self.lost_frames = 0
        self.landmarks = np.zeros((0, 3))
        self.trajectory = [T_world_cam0[:3, 3].copy()]

    # -- helpers -------------------------------------------------------------
    def _detect(self, gray, depth):
        kp, des = self.orb.detectAndCompute(gray, None)
        if des is None or len(kp) < 12:
            return None, None, None
        uv = np.array([k.pt for k in kp], dtype=np.float64)
        u = np.clip(np.round(uv[:, 0]).astype(int), 0, depth.shape[1] - 1)
        v = np.clip(np.round(uv[:, 1]).astype(int), 0, depth.shape[0] - 1)
        z = depth[v, u].astype(np.float64)
        # Keep only corners with a usable stereo return.
        ok = np.isfinite(z) & (z > self.min_depth) & (z < self.max_depth * 0.75)
        if ok.sum() < 12:
            return None, None, None
        uv, des, z = uv[ok], des[ok], z[ok]
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        pts3d = np.stack([(uv[:, 0] - cx) / fx * z,
                          (uv[:, 1] - cy) / fy * z, z], axis=1)
        return uv, des, pts3d

    def _make_keyframe(self, uv, des, pts3d):
        self.kf_kp, self.kf_des, self.kf_pts3d = uv, des, pts3d
        self.T_world_kf = self.T_world_cam.copy()
        self.kf_count += 1
        world = (self.T_world_kf[:3, :3] @ pts3d.T).T + self.T_world_kf[:3, 3]
        keep = world[np.random.rand(len(world)) < 0.25]
        if len(keep):
            self.landmarks = np.vstack([self.landmarks, keep])[-9000:]

    # -- main entry ----------------------------------------------------------
    def track(self, rgb: np.ndarray, depth: np.ndarray) -> dict:
        """Process one ZED frame. Returns the current camera pose + telemetry."""
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        gray = cv2.equalizeHist(gray)
        uv, des, pts3d = self._detect(gray, depth)
        self.n_features = 0 if uv is None else len(uv)

        if uv is None:
            self.tracking_ok = False
            self.lost_frames += 1
            return self._report(uv, None)

        if self.kf_des is None:
            self._make_keyframe(uv, des, pts3d)
            self.tracking_ok = True
            return self._report(uv, None)

        knn = self.matcher.knnMatch(des, self.kf_des, k=2)
        good = [a for a, b in (p for p in knn if len(p) == 2)
                if a.distance < 0.78 * b.distance]
        self.n_matches = len(good)

        if len(good) < 14:
            self.tracking_ok = False
            self.lost_frames += 1
            self._make_keyframe(uv, des, pts3d)       # re-seed and carry on
            return self._report(uv, None)

        obj = np.array([self.kf_pts3d[g.trainIdx] for g in good], dtype=np.float64)
        img = np.array([uv[g.queryIdx] for g in good], dtype=np.float64)

        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            obj, img, self.K, None, flags=cv2.SOLVEPNP_ITERATIVE,
            reprojectionError=2.5, iterationsCount=220, confidence=0.995)

        if not ok or inliers is None or len(inliers) < 10:
            self.tracking_ok = False
            self.lost_frames += 1
            self._make_keyframe(uv, des, pts3d)
            return self._report(uv, None)

        # Refine on the inlier set only.
        idx = inliers.ravel()
        cv2.solvePnPRefineLM(obj[idx], img[idx], self.K, None, rvec, tvec)
        self.n_inliers = len(idx)
        self.tracking_ok = True

        R, _ = cv2.Rodrigues(rvec)
        T_cur_kf = _se3(R, tvec)                       # keyframe -> current cam
        self.T_world_cam = self.T_world_kf @ np.linalg.inv(T_cur_kf)
        self.trajectory.append(self.T_world_cam[:3, 3].copy())

        # New keyframe on enough parallax, rotation, or track attrition.
        d = np.linalg.norm(T_cur_kf[:3, 3])
        ang = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2.0, -1, 1)))
        if d > 0.45 or ang > 11.0 or self.n_inliers < 40:
            self._make_keyframe(uv, des, pts3d)

        return self._report(uv, img[idx])

    def _report(self, uv, inlier_uv) -> dict:
        return dict(T_world_cam=self.T_world_cam.copy(),
                    keypoints=uv, inlier_uv=inlier_uv,
                    n_features=self.n_features, n_matches=self.n_matches,
                    n_inliers=self.n_inliers, keyframes=self.kf_count,
                    tracking_ok=self.tracking_ok, lost_frames=self.lost_frames)
