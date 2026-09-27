"""Navigation: a global route over the prior GeoTIFF, and a DWA local planner
over a costmap built from live ZED 2i depth.

Split of responsibilities -- the part that makes the demo honest:

  * The GeoTIFF knows the *terrain*: where the ground is too steep to climb.
    It is a survey product, available before the vehicle rolls, and it is what
    A* plans over.
  * The GeoTIFF does NOT know the *obstacles*: the boulders and trees in the
    corridor were never surveyed. They exist only in the ZED depth stream and
    only enter the local costmap once the camera has actually seen them.

So the global route is smooth and terrain-aware, and every deviation from it
you see in the demo is the local planner reacting to something the camera just
discovered.
"""
from __future__ import annotations

import heapq

import numpy as np

try:
    import cv2
except ImportError:                                   # pragma: no cover
    cv2 = None

from sih_demo import mission


# --------------------------------------------------------------------------
# Global planner: A* over prior-DEM traversability
# --------------------------------------------------------------------------
class GlobalPlanner:
    def __init__(self, prior_dem, world_size: float, res: float = 1.0):
        self.res = res
        self.half = world_size / 2.0
        self.n = int(round(world_size / res)) + 1
        coords = np.linspace(-self.half, self.half, self.n)
        xx, yy = np.meshgrid(coords, coords)          # yy increases with row
        self.xx, self.yy = xx, yy

        # Evaluate slope at the DEM's own resolution, not the planning grid's:
        # a 1 m central difference smooths the ridge's steep lateral fade into
        # something that looks traversable and is not.
        self.slope = prior_dem.slope_deg(xx, yy, eps=prior_dem.res)
        self.elevation = prior_dem.elevation(xx, yy)
        blocked = self.slope > mission.MAX_TRAVERSABLE_SLOPE_DEG

        # Inflate the un-traversable terrain by the vehicle's footprint.
        infl_cells = int(np.ceil((mission.ROBOT_RADIUS_M + mission.SAFETY_MARGIN_M) / res))
        k = 2 * infl_cells + 1
        self.blocked = cv2.dilate(blocked.astype(np.uint8),
                                  np.ones((k, k), np.uint8)) > 0
        # Metric distance from every cell to the nearest un-traversable cell.
        # The ditch is a *negative* obstacle: the ZED sees a hole, not a wall,
        # so the local planner cannot discover it from depth. The prior survey
        # can, and this field is how that knowledge reaches DWA.
        self.hazard_dist = cv2.distanceTransform(
            (~blocked).astype(np.uint8), cv2.DIST_L2, 5) * res
        self.route_dist = np.zeros_like(self.hazard_dist)

        # Soft cost: prefer flat ground even where it is passable.
        self.terrain_cost = 1.0 + 2.5 * np.clip(
            self.slope / mission.MAX_TRAVERSABLE_SLOPE_DEG, 0, 1) ** 2

    def _cell(self, x, y):
        c = int(round((x + self.half) / self.res))
        r = int(round((y + self.half) / self.res))
        return (min(max(r, 0), self.n - 1), min(max(c, 0), self.n - 1))

    def _world(self, r, c):
        return (-self.half + c * self.res, -self.half + r * self.res)

    def set_route(self, route: np.ndarray):
        """Rasterise the chosen route into a distance field, so the local
        planner can be told how far it has wandered off the surveyed corridor."""
        grid = np.ones((self.n, self.n), np.uint8)
        for p in route:
            r, c = self._cell(p[0], p[1])
            grid[r, c] = 0
        self.route_dist = cv2.distanceTransform(grid, cv2.DIST_L2, 5) * self.res

    def _lookup(self, field, xs, ys, default):
        c = np.round((np.asarray(xs) + self.half) / self.res).astype(int)
        r = np.round((np.asarray(ys) + self.half) / self.res).astype(int)
        inside = (c >= 0) & (c < self.n) & (r >= 0) & (r < self.n)
        out = np.full(np.shape(xs), default, dtype=np.float64)
        out[inside] = field[r[inside], c[inside]]
        return out

    def hazard_clearance(self, xs, ys):
        """Distance to the nearest un-traversable terrain cell, metres."""
        return self._lookup(self.hazard_dist, xs, ys, 0.0)

    def route_offset(self, xs, ys):
        return self._lookup(self.route_dist, xs, ys, 50.0)

    def plan(self, start_xy, goal_xy) -> np.ndarray:
        start, goal = self._cell(*start_xy), self._cell(*goal_xy)
        nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1),
                (-1, -1), (-1, 1), (1, -1), (1, 1)]

        g = {start: 0.0}
        parent: dict = {}
        openq = [(0.0, start)]
        closed = set()

        def h(a):
            return float(np.hypot(a[0] - goal[0], a[1] - goal[1]))

        while openq:
            _, cur = heapq.heappop(openq)
            if cur in closed:
                continue
            closed.add(cur)
            if cur == goal:
                break
            for dr, dc in nbrs:
                nxt = (cur[0] + dr, cur[1] + dc)
                if not (0 <= nxt[0] < self.n and 0 <= nxt[1] < self.n):
                    continue
                if self.blocked[nxt] and nxt != goal:
                    continue
                step = self.res * (1.41421 if dr and dc else 1.0)
                ng = g[cur] + step * self.terrain_cost[nxt]
                if ng < g.get(nxt, np.inf):
                    g[nxt] = ng
                    parent[nxt] = cur
                    heapq.heappush(openq, (ng + h(nxt) * self.res, nxt))

        if goal not in parent and goal != start:
            return np.array([start_xy, goal_xy], dtype=np.float64)

        path, cur = [goal], goal
        while cur != start:
            cur = parent[cur]
            path.append(cur)
        path.reverse()
        return np.array([self._world(r, c) for r, c in path], dtype=np.float64)


# --------------------------------------------------------------------------
# Local costmap, populated purely from ZED depth
# --------------------------------------------------------------------------
class LocalCostmap:
    def __init__(self, size_m: float = 30.0, res: float = 0.25):
        self.res = res
        self.n = int(size_m / res)
        self.size_m = size_m
        self.origin = np.array([0.0, 0.0])            # map coords of cell (0,0)
        self.logodds = np.zeros((self.n, self.n), dtype=np.float32)
        self.dist = np.full((self.n, self.n), 99.0, dtype=np.float32)

    def recenter(self, x, y):
        new_origin = np.array([x - self.size_m / 2.0, y - self.size_m / 2.0])
        shift = np.round((new_origin - self.origin) / self.res).astype(int)
        if shift.any():
            self.logodds = np.roll(self.logodds, (-shift[1], -shift[0]), axis=(0, 1))
            if shift[0] > 0:
                self.logodds[:, -shift[0]:] = 0
            elif shift[0] < 0:
                self.logodds[:, :-shift[0]] = 0
            if shift[1] > 0:
                self.logodds[-shift[1]:, :] = 0
            elif shift[1] < 0:
                self.logodds[:-shift[1], :] = 0
            self.origin = self.origin + shift * self.res

    def _cells(self, xs, ys):
        c = np.floor((xs - self.origin[0]) / self.res).astype(int)
        r = np.floor((ys - self.origin[1]) / self.res).astype(int)
        ok = (c >= 0) & (c < self.n) & (r >= 0) & (r < self.n)
        return r[ok], c[ok]

    def reset(self):
        self.logodds[:] = 0.0
        self.dist[:] = 99.0
        self.occ = np.zeros((self.n, self.n), np.uint8)

    def integrate(self, obstacle_xy: np.ndarray, free_xy: np.ndarray):
        """Fold one frame of ZED evidence in.

        Evidence is counted per *cell per frame*, not per point: a cell that
        happens to catch fifty returns must not be able to swamp the map in a
        single frame, or one noisy reading becomes a permanent wall.
        """
        self.logodds *= 0.998                          # forget stale evidence slowly
        if len(free_xy):
            r, c = self._cells(free_xy[:, 0], free_xy[:, 1])
            cnt = np.zeros_like(self.logodds)
            np.add.at(cnt, (r, c), 1.0)
            self.logodds -= 0.55 * np.minimum(cnt / 4.0, 1.0)
        if len(obstacle_xy):
            r, c = self._cells(obstacle_xy[:, 0], obstacle_xy[:, 1])
            cnt = np.zeros_like(self.logodds)
            np.add.at(cnt, (r, c), 1.0)
            # A cell needs ~3 returns in a frame to count as fully observed.
            self.logodds += 0.85 * np.minimum(cnt / 3.0, 1.0)
        np.clip(self.logodds, -2.0, 4.0, out=self.logodds)

        # Two frames of solid evidence before a cell blocks the planner.
        occ = (self.logodds > 1.2).astype(np.uint8)
        free = (1 - occ).astype(np.uint8)
        self.dist = cv2.distanceTransform(free, cv2.DIST_L2, 5) * self.res
        self.occ = occ

    def clearance(self, xs, ys) -> np.ndarray:
        """Distance to the nearest known obstacle, metres. Off-map = optimistic."""
        c = np.floor((np.asarray(xs) - self.origin[0]) / self.res).astype(int)
        r = np.floor((np.asarray(ys) - self.origin[1]) / self.res).astype(int)
        inside = (c >= 0) & (c < self.n) & (r >= 0) & (r < self.n)
        out = np.full(np.shape(xs), 99.0, dtype=np.float64)
        out[inside] = self.dist[r[inside], c[inside]]
        return out


# --------------------------------------------------------------------------
# DWA local planner
# --------------------------------------------------------------------------
class DWAPlanner:
    HORIZON_S = 2.4
    DT = 0.2
    N_V = 6
    N_W = 25

    def __init__(self):
        self.safe_radius = mission.ROBOT_RADIUS_M + mission.SAFETY_MARGIN_M
        self.last_rollouts: list = []
        self.chosen: np.ndarray | None = None

    def _rollout(self, x, y, yaw, v, w):
        steps = int(self.HORIZON_S / self.DT)
        xs = np.empty(steps); ys = np.empty(steps); yaws = np.empty(steps)
        for i in range(steps):
            yaw += w * self.DT
            x += v * np.cos(yaw) * self.DT
            y += v * np.sin(yaw) * self.DT
            xs[i], ys[i], yaws[i] = x, y, yaw
        return xs, ys, yaws

    def plan(self, pose, carrot, costmap, v_prev, terrain=None,
             allow_reverse=False):
        """Pick (v, w) for the next control step.

        Two thresholds, deliberately different:

          * hard  -- ROBOT_RADIUS. A trajectory that takes the footprint into
            an occupied cell is a collision and is never selectable.
          * soft  -- ROBOT_RADIUS + SAFETY_MARGIN. Eroding the margin is
            expensive but allowed.

        Making the margin a hard constraint is what deadlocks a DWA: every
        rollout starts at the vehicle, so a single noise-inflated cell inside
        the margin rejects the entire sample set and the planner can only spin.
        If nothing clears even the hard threshold, the fallback drives towards
        open space instead of giving up.
        """
        x, y, yaw = pose
        gx, gy = carrot
        # A little above the bare footprint: the rollout is a 0.2 s-discretised
        # unicycle approximation of a slipping skid-steer, so admitting
        # trajectories that only just clear the body lets tracking error turn
        # into a graze. Still well under the desired margin, so the sample set
        # can never be emptied and the planner cannot deadlock.
        hard = mission.ROBOT_RADIUS_M + 0.10
        soft = self.safe_radius

        v_hi = min(mission.MAX_LINEAR_MPS, v_prev + 0.45)
        vs = list(np.linspace(0.0, max(v_hi, 0.2), self.N_V)) + [-0.3, -0.55]
        ws = np.linspace(-mission.MAX_ANGULAR_RPS, mission.MAX_ANGULAR_RPS, self.N_W)

        best = None          # satisfies the hard constraint
        fallback = None      # largest clearance, used only if nothing is safe
        rollouts = []
        n_safe = 0

        for v in vs:
            for w in ws:
                if abs(v) < 1e-9 and abs(w) < 1e-9:
                    continue
                xs, ys, _ = self._rollout(x, y, yaw, v, w)
                min_clr = float(costmap.clearance(xs, ys).min())
                off = 0.0
                if terrain is not None:
                    # Prior-map terrain hazards (the ditch, steep faces) are a
                    # hard constraint too: the ZED cannot see a hole.
                    min_clr = min(min_clr, float(terrain.hazard_clearance(xs, ys).min()))
                    off = float(terrain.route_offset(xs[-1:], ys[-1:])[0])

                safe = min_clr > hard
                rollouts.append((xs, ys, min_clr > soft, min_clr))
                d_goal = float(np.hypot(xs[-1] - gx, ys[-1] - gy))

                if fallback is None or min_clr > fallback[0] + 1e-6:
                    fallback = (min_clr, v, w, xs, ys, d_goal)

                if not safe:
                    continue
                n_safe += 1
                cost = (2.4 * d_goal
                        + 4.0 * max(0.0, soft - min_clr) / soft
                        + 1.4 * (mission.MAX_LINEAR_MPS - max(v, 0.0))
                        + 0.30 * abs(w)
                        + 1.1 * min(max(off - 4.0, 0.0), 10.0)
                        + (2.5 if v < 0 else 0.0)
                        # standing still must never be the cheapest option
                        + (3.0 if abs(v) < 0.05 else 0.0))
                if best is None or cost < best[0]:
                    best = (cost, v, w, xs, ys, min_clr)

        self.last_rollouts = rollouts
        if best is not None:
            _, v, w, xs, ys, min_clr = best
            self.chosen = np.stack([xs, ys], axis=1)
            return float(v), float(w), dict(mode="dwa", min_clearance=min_clr,
                                            n_safe=n_safe, n_sampled=len(rollouts))

        # Nothing clears the footprint: back away towards the most open space.
        min_clr, v, w, xs, ys, _ = fallback
        self.chosen = np.stack([xs, ys], axis=1)
        return float(v), float(w), dict(mode="escape", min_clearance=min_clr,
                                        n_safe=0, n_sampled=len(rollouts))


def carrot_on_path(path: np.ndarray, pose, lookahead: float = 4.0) -> tuple[np.ndarray, int]:
    """Pure-pursuit style lookahead point on the global route."""
    p = np.array(pose[:2])
    d = np.linalg.norm(path - p, axis=1)
    i = int(np.argmin(d))
    j = i
    while j < len(path) - 1 and np.linalg.norm(path[j] - p) < lookahead:
        j += 1
    return path[j], i
