"""
Given a path from planning:
    1. Break it into waypoints
    2. Smooth the path slightly
    3. Repeatedly navigate to the closest waypoint that progresses along the path
    4. Right before reaching the next waypoint, walk to the waypoint after it to avoid stopping
    5. When the path is complete, exit to neutral position

Path rows are [x, y, theta] with x,y in meters and theta in degrees (same convention as Walk).
"""

import logging
import numpy as np

from mujococodebase.utils.math_ops import MathOps

logger = logging.getLogger(__file__)


class PathFollower:
    def __init__(self, agent):
        self.agent = agent
        self.path = None

        # all constants in meters
        self._PATH_SPACING = 0.5
        self._SMOOTH_WEIGHT = 0.25
        self._PATH_COMPLETE_THRESHOLD = 0.1
        self._WAYPOINT_LOOKAHEAD = 0.28


    def set_path(self, path):
        self.path = self._break_path_into_waypoints(path)
        self.path = self._smooth_path(self.path)

    def follow_current_path(self):
        if self.path is None:
            return
        if self._is_path_complete():
            self.agent.skills_manager.execute("Neutral")
            return

        closest_waypoint = self._get_next_forward_waypoint(self.path)
        self._walk_to_waypoint(closest_waypoint)

    def is_path_complete(self) -> bool:
        if self.path is None:
            return True
        return self._is_path_complete()

    # HELPER FUNCTIONS

    def _break_path_into_waypoints(self, path):
        if path.ndim != 2 or path.shape[1] != 3:
            raise ValueError("path must be shape (N, 3) with columns [x, y, theta_deg]")
        if len(path) <= 1:
            return path.copy()

        spacing = self._PATH_SPACING
        new_path = []
        for i in range(len(path) - 1):
            p0, p1 = path[i], path[i + 1]
            dx, dy = p1[0] - p0[0], p1[1] - p0[1]
            segment_len = float(np.hypot(dx, dy))
            num_segments = max(1, int(np.ceil(segment_len / spacing)))
            heading = float(np.rad2deg(np.arctan2(dy, dx)))
            for j in range(num_segments):
                fraction = j / num_segments
                new_path.append(
                    [
                        p0[0] + fraction * dx,
                        p0[1] + fraction * dy,
                        heading,
                    ]
                )
        new_path.append(path[-1].copy())
        return np.asarray(new_path, dtype=float)

    def _smooth_path(self, path):
        if len(path) < 3:
            return path.copy()

        weight = self._SMOOTH_WEIGHT
        new_path = path.copy()
        xy = path[:, :2]
        theta = np.unwrap(path[:, 2], period=360.0)

        for i in range(1, len(path) - 1):
            new_path[i, 0] = (1 - weight) * xy[i, 0] + weight * 0.5 * (xy[i - 1, 0] + xy[i + 1, 0])
            new_path[i, 1] = (1 - weight) * xy[i, 1] + weight * 0.5 * (xy[i - 1, 1] + xy[i + 1, 1])
            new_path[i, 2] = (1 - weight) * theta[i] + weight * 0.5 * (theta[i - 1] + theta[i + 1])

        new_path[:, 2] = MathOps.normalize_deg(new_path[:, 2])
        return new_path

    def _get_next_forward_waypoint(self, path):
        agent_xy = self.agent.world.global_position[:2]
        best_idx = None
        best_dist = float("inf")
        for i in range(len(path)):
            if not self._is_waypoint_ahead(i) or self._should_advance_past_waypoint(i):
                continue
            dist = float(np.linalg.norm(path[i, :2] - agent_xy))
            if dist < best_dist:
                best_dist = dist
                best_idx = i
        if best_idx is None:
            return len(path) - 1
        return best_idx

    def _walk_to_waypoint(self, waypoint_index):
        waypoint = self.path[waypoint_index]
        self.agent.skills_manager.execute(
            "Walk",
            target_2d=waypoint[:2],
            is_target_absolute=True,
            orientation=waypoint[2],
        )

    # UTILITY FUNCTIONS

    def _is_path_complete(self):
        agent_location = self.agent.world.global_position[:2]
        target_location = self.path[-1][:2]
        return np.linalg.norm(target_location - agent_location) <= self._PATH_COMPLETE_THRESHOLD

    def _should_advance_past_waypoint(self, waypoint_index):
        waypoint = self.path[waypoint_index][:2]
        agent_location = self.agent.world.global_position[:2]
        return float(np.linalg.norm(waypoint - agent_location)) <= self._WAYPOINT_LOOKAHEAD

    def _is_waypoint_ahead(self, waypoint_index):
        if waypoint_index == len(self.path) - 1:
            return not self._is_path_complete()
        path_dir = self.path[waypoint_index + 1][:2] - self.path[waypoint_index][:2]
        agent_dir = self.path[waypoint_index][:2] - self.agent.world.global_position[:2]
        return np.dot(path_dir, agent_dir) > 0
