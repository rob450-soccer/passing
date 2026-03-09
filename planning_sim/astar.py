from __future__ import annotations
import heapq
import math
import numpy


class GridWorld:
    """
    Grid representation of the environment.
    0 represents free space.
    1 represents obstacles.
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

    def update_grid(self, new_grid: list[list[int]]) -> None:
        """Update the map with new info."""
        self._grid = numpy.array(new_grid, dtype=float)
        self.inflate_obstacles()

    def update_location(self, pos: list[int], value: float) -> None:
        """Update a specific cell of the map."""
        self._grid[pos[0], pos[1]] = value

    def is_occupied(self, pos: list[int]) -> bool:
        """Check if a cell of the map is occupied or not."""
        return self._grid[pos[0], pos[1]] > 0

    def is_obstacle(self, pos: list[int]) -> bool:
        """Check if a cell of the map is an obstacle or not."""
        return self._grid[pos[0], pos[1]] == 1
    
    def is_inflation(self, pos: list[int]) -> bool:
        """Check if a cell of the map is partof the inflation layer or not."""
        return self._grid[pos[0], pos[1]] == 0.5
    
    def is_free(self, pos: list[int]) -> bool:
        """Check if a cell of the map is free space or not."""
        return self._grid[pos[0], pos[1]] == 0


class Node:
    """Node for an individual location."""

    def __init__(self, location: list[int], parent: Node | None):
        self.location = location
        self.parent = parent
        self.g = float("inf")
        self.h = 0
        self.f = float("inf")

    def __lt__(self, other):
        return self.f < other.f

    def distance_to(self, other: Node) -> float:
        """Calculate Euclidean distance between two nodes."""
        return math.hypot(self.location[0] - other.location[0], self.location[1] - other.location[1])



def theta_star():
    pass