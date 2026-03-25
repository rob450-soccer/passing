from __future__ import annotations
import math
import numpy


class GridWorld:
    """
    Grid representation of the environment.
    0 represents free space. 1 represents obstacles.
    (0, 0) is at the top left, with the x axis going down and the y axis going across.
    """

    def __init__(self, grid: list[list[int]]):
        self._grid = numpy.array(grid, dtype=float)
        self.inflate_obstacles()

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

    def neighbors(self, pos: tuple[int]) -> list[tuple[int]]:
        """Get a list of the eight neighboring locations of the specified location."""
        neighbors = []
        if pos[0] > 0:
            neighbors.append((pos[0] - 1, pos[1]))
            if pos[1] > 0:
                neighbors.append((pos[0] - 1, pos[1] - 1))
            if pos[1] < self._grid.shape[1] - 1:
                neighbors.append((pos[0] - 1, pos[1] + 1))
        if pos[0] < self._grid.shape[0] - 1:
            neighbors.append((pos[0] + 1, pos[1]))
            if pos[1] > 0:
                neighbors.append((pos[0] + 1, pos[1] - 1))
            if pos[1] < self._grid.shape[1] - 1:
                neighbors.append((pos[0] + 1, pos[1] + 1))
        if pos[1] > 0:
            neighbors.append((pos[0], pos[1] - 1))
        if pos[1] < self._grid.shape[1] - 1:
            neighbors.append((pos[0], pos[1] + 1))
        return neighbors

    def line_of_sight(self, pos1: tuple[int], pos2: tuple[int]) -> bool:
        """
        Checks line of sight using Bresenham's.

        Args:
            pos1 (tuple[int]): Location of the starting cell of the line.
            pos2 (tuple[int]): Location of the ending cell of the line.
        """
        x0, y0 = pos1
        x1, y1 = pos2
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        error = dx - dy
        
        x_step = 1 if x0 < x1 else -1
        y_step = 1 if y0 < y1 else -1

        while True:
            if self.is_occupied((x0, y0)):
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
        self._grid = numpy.array(new_grid, dtype=float)
        self.inflate_obstacles()

    def update_location(self, pos: tuple[int], value: float) -> None:
        """Update a specific cell of the map."""
        self._grid[pos[0], pos[1]] = value

    def is_occupied(self, pos: tuple[int]) -> bool:
        """Check if a cell of the map is occupied or not."""
        return self._grid[pos[0], pos[1]] > 0

    def is_obstacle(self, pos: tuple[int]) -> bool:
        """Check if a cell of the map is an obstacle or not."""
        return self._grid[pos[0], pos[1]] == 1

    def is_inflation(self, pos: tuple[int]) -> bool:
        """Check if a cell of the map is partof the inflation layer or not."""
        return self._grid[pos[0], pos[1]] == 0.5

    def is_free(self, pos: tuple[int]) -> bool:
        """Check if a cell of the map is free space or not."""
        return self._grid[pos[0], pos[1]] == 0


class Node:
    """Node for an individual location."""

    def __init__(self, location: tuple[int], parent: Node | None = None):
        self.location = location
        self.parent = parent
        self.g = float("inf")
        self.h = 0
        self.score = float("inf")

    def __lt__(self, other):
        return self.score < other.score

    def distance_to(self, other: Node) -> float:
        """Calculate Euclidean distance between two nodes."""
        return math.hypot(self.location[0] - other.location[0], self.location[1] - other.location[1])
