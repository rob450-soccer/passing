import heapq
import logging
import numpy as np

from mujococodebase.world.grid_world import GridWorld, Node

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__file__)


def ana_theta_star(world: GridWorld, start: np.ndarray[int], goal: np.ndarray[int], path_name: str, shared_paths: dict, shared_ready_events: dict) -> list[np.ndarray[int]] | None:
    """
    Anytime variant that makes progressively better any-angle paths if time is a limiting factor.
    
    Args:
        world (GridWorld): representation of the map
        start (np.ndarray[int]): starting location to plan from
        goal (np.ndarray[int]): target location to plan to
        current_path (list[np.ndarray[int]]): a variable to store and get the current path

    Returns: 
        path (list[np.ndarray[int]]): the final best path found
    """
    goal = _adjust_goal_if_occupied(world, start, goal)
    logger.debug(f"planning with ana_theta_star() from {start} to {goal}")
    # setup
    start_node = Node(start)
    goal_node = Node(goal)
    start_node.g = 0
    start_node.h = _distance_to_goal(start_node, goal)
    G = float('inf') # current best cost
    E = float('inf') # sub-optimality
    start_node.score = -1 * ((G - start_node.g) / start_node.h)

    open = [start_node]
    heapq.heapify(open)
    g_scores = {start_node: 0}

    def e(node):
        if node.h == 0:
            return -float('inf')
        if G == float('inf'):
            return node.h
        return -1 * ((G - node.g) / node.h)

    def improve_solution():
        nonlocal E, G
        while len(open) > 0:
            current_node = heapq.heappop(open)

            # update suboptimality bound
            if current_node.score < E:
                E = current_node.score

            # check if we've reached the goal:
            if np.any(np.all(current_node.location == np.atleast_2d(goal), axis=1)):
                G = current_node.g
                # reconstruct path
                path = []
                while current_node is not None:
                    path.append(current_node.location)
                    current_node = current_node.parent
                return path[::-1]

            for neighbor in world.neighbors(current_node.location):
                # skip occupied cells
                if world.is_occupied(neighbor):
                    continue

                # take the node if it introduces a shorter path
                neighbor_node = Node(neighbor, current_node)
                if current_node.parent and world.line_of_sight(current_node.parent.location, neighbor_node.location):
                    g = current_node.parent.g + current_node.parent.distance_to(neighbor_node)
                    neighbor_node.parent = current_node.parent
                else:
                    g = current_node.g + current_node.distance_to(neighbor_node)
                if g < g_scores.get(neighbor_node, float("inf")):
                    neighbor_node.g = g
                    neighbor_node.h = _distance_to_goal(neighbor_node, goal)
                    if neighbor_node.g + neighbor_node.h < G:
                        neighbor_node.score = e(neighbor_node)
                        g_scores[neighbor_node] = g
                        heapq.heappush(open, neighbor_node)

    # finding the path
    while len(open) > 0:
        new_path = improve_solution()
        if new_path is None:
            break
        shared_paths[path_name] = new_path
        shared_ready_events[path_name].set()
        # prune nodes that cannot give a shorter path
        open = [node for node in open if node.g + node.h < G]
        for node in open:
            node.score = e(node)
        heapq.heapify(open)

    logger.debug(f"[test1] {path_name} path: {[pos.tolist() for pos in shared_paths[path_name]]}")
    return shared_paths[path_name]


### Helper functions ###


def _distance_to_goal(start: Node, goal: np.ndarray[int]) -> float:
    """Get the distance to the goal, whether the goal is a single point or multiple."""
    if goal.ndim == 1:
        return np.linalg.norm(start.location - goal)
    return np.min(np.linalg.norm(start.location - goal, axis=1))


def _bresenham(x0: int, y0: int, x1: int, y1: int) -> list[tuple[int, int]]:
    """Bresenham line from (x0, y0) to (x1, y1), inclusive."""
    cells: list[tuple[int, int]] = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return cells


def _adjust_goal_if_occupied(world: GridWorld, start: np.ndarray, goal: np.ndarray) -> np.ndarray:
    """
    If the goal cell is occupied, move the goal to the free cell closest to the original goal
    along the line segment from the robot (start) to the goal (grid / Bresenham cells).
    Preserves any extra dimensions on goal (e.g. orientation).
    """
    x0, y0 = int(start[0]), int(start[1])
    x1, y1 = int(goal[0]), int(goal[1])

    def at_xy(x: int, y: int) -> np.ndarray:
        if goal.size > 2:
            g = goal.copy()
            g[0], g[1] = x, y
            return g
        return np.array([x, y], dtype=goal.dtype)

    if not world.is_occupied(at_xy(x1, y1)):
        return goal

    for x, y in reversed(_bresenham(x0, y0, x1, y1)):
        c = at_xy(x, y)
        if not world.is_occupied(c):
            return c
    return goal