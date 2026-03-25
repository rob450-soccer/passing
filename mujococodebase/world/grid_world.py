from __future__ import annotations
import logging
import numpy as np

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__file__)


class GridWorld:
    """
    Grid representation of the environment.
    0 represents free space. 1 represents obstacles.
    (0, 0) is at the top left, with the x axis going down and the y axis going across.
    """

    def __init__(self, grid: list[list[int]]):
        self._grid = np.array(grid, dtype=float)
        self.inflate_obstacles()
        self.width, self.height = self._grid.shape
    
    def __init__(self, width: int, height: int):
        self._grid = np.zeros((width, height), dtype=float)
        self.width, self.height = self._grid.shape

    def inflate_obstacles(self, inflation_amount: int = 2) -> None:
        """
        Inflate the size of the obstacles to keep the robot a safe distance away.
        Inflated cells are given the value 0.5 to differentiate them from the true obstacles.

        Args:
            inflation_amount (int): Number of cells to inflate obstacles by. Default 2.
        """
        for r in range(self._grid.shape[0]):
            for c in range(self._grid.shape[1]):
                if self._grid[r, c] == 1:
                    # this is an obstacle, set nearby 0 cells to 0.5
                    inflation_section = self._grid[
                        max(r - inflation_amount, 0) : min(r + inflation_amount + 1, self._grid.shape[0]),
                        max(c - inflation_amount, 0) : min(c + inflation_amount + 1, self._grid.shape[1]),
                    ]
                    inflation_section[inflation_section == 0] = 0.5  # inflation_section is a view of self.grid

    def neighbors(self, pos: np.ndarray[int]) -> list[np.ndarray[int]]:
        """Get a list of the eight neighboring locations of the specified location."""
        headings = [0, np.pi/4, np.pi/2, 3*np.pi/4, np.pi, -3*np.pi/4, -np.pi/2, -np.pi/4]
        positions = [
            np.array([pos[0] + 1, pos[1]]),
            np.array([pos[0] - 1, pos[1]]),
            np.array([pos[0], pos[1] + 1]),
            np.array([pos[0], pos[1] - 1]),
            np.array([pos[0] + 1, pos[1] + 1]),
            np.array([pos[0] + 1, pos[1] - 1]),
            np.array([pos[0] - 1, pos[1] + 1]),
            np.array([pos[0] - 1, pos[1] - 1])
        ]

        idx = 0
        while idx < len(positions):
            if positions[idx][0] <= -self.width // 2 or positions[idx][0] >= self.width // 2:
                positions.pop(idx)
            elif positions[idx][1] <= -self.height // 2 or positions[idx][1] >= self.height // 2:
                positions.pop(idx)
            else:
                idx += 1
        
        if len(pos) == 2:
            # no orientation information
            return positions

        # include orientation information
        neighbors = []
        for p in positions:
            for h in headings:
                neighbors.append(np.array([p[0], p[1], h]))
        return neighbors

    def line_of_sight(self, position1: np.ndarray[int], position2: np.ndarray[int]) -> bool:
        """
        Checks line of sight using Bresenham's.

        Args:
            pos1 (np.ndarray[int]): Location of the starting cell of the line.
            pos2 (np.ndarray[int]): Location of the ending cell of the line.
        """
        pos1 = self._cartesian_to_array_index(position1)
        pos2 = self._cartesian_to_array_index(position2)
        x0, y0 = pos1
        x1, y1 = pos2
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        error = dx - dy
        
        x_step = 1 if x0 < x1 else -1
        y_step = 1 if y0 < y1 else -1

        while True:
            if self.is_occupied(np.array([x0, y0])):
                return False
                
            if x0 == x1 and y0 == y1:
                # Reached the target
                break

            e2 = 2 * error
            if e2 > -dy:
                error -= dy
                x0 += x_step
            if e2 < dx:
                error += dx
                y0 += y_step
                
        return True

    def update_grid(self, new_grid: list[list[int]]) -> None:
        """Update the map with new info."""
        self._grid = np.array(new_grid, dtype=float)
        self.inflate_obstacles()

    def update_location(self, position: np.ndarray[int], value: float) -> None:
        """Update a specific cell of the map."""
        pos = self._cartesian_to_array_index(position)
        self._grid[pos[0], pos[1]] = value
    
    def add_obstacle(self, position: np.ndarray[int], inflation_amount: int = 2) -> None:
        """Add an obstacle and inflate the area around it."""
        pos = self._cartesian_to_array_index(position)
        self._grid[pos[0], pos[1]] = 1
        inflation_section = self._grid[
            max(pos[0] - inflation_amount, 0) : min(pos[0] + inflation_amount + 1, self._grid.shape[0]),
            max(pos[1] - inflation_amount, 0) : min(pos[1] + inflation_amount + 1, self._grid.shape[1]),
        ]
        inflation_section[inflation_section == 0] = 0.5

    def is_occupied(self, position: np.ndarray[int]) -> bool:
        """Check if a cell of the map is occupied or not."""
        pos = self._cartesian_to_array_index(position)
        return self._grid[pos[0], pos[1]] > 0

    def is_obstacle(self, position: np.ndarray[int]) -> bool:
        """Check if a cell of the map is an obstacle or not."""
        pos = self._cartesian_to_array_index(position)
        return self._grid[pos[0], pos[1]] == 1

    def is_inflation(self, position: np.ndarray[int]) -> bool:
        """Check if a cell of the map is partof the inflation layer or not."""
        pos = self._cartesian_to_array_index(position)
        return self._grid[pos[0], pos[1]] == 0.5

    def is_free(self, position: np.ndarray[int]) -> bool:
        """Check if a cell of the map is free space or not."""
        pos = self._cartesian_to_array_index(position)
        return self._grid[pos[0], pos[1]] == 0

    def _cartesian_to_array_index(self, position: np.ndarray[int]) -> np.ndarray[int]:
        """Convert from Cartesian coordinates to array indices."""
        return np.array([position[0] + self.width // 2, self.height // 2 - position[1]])

class Node:
    """Node for an individual location."""

    def __init__(self, location: np.ndarray[int], parent: Node | None = None):
        self.location = location
        self.parent = parent
        self.g = float("inf")
        self.h = 0
        self.score = float("inf")

    def __lt__(self, other):
        if not isinstance(other, Node):
            return False
        return self.score < other.score

    def __eq__(self, other):
        if not isinstance(other, Node):
            return False
        return np.array_equal(self.location, other.location)

    def __hash__(self):
        return hash(tuple(self.location))

    def distance_to(self, other: Node) -> float:
        """Calculate Euclidean distance between two nodes."""
        diff = np.abs(self.location - other.location)
        if self.location.size == 3:
            # Wrap-around logic: get the shortest angular distance
            diff[2] = min(diff[2], 2 * np.pi - diff[2])
        return float(np.linalg.norm(diff))