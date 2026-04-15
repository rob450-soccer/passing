import heapq
import logging
import threading
import numpy as np

from mujococodebase.world.grid_world import GridWorld, Node

logger = logging.getLogger(__file__)

def distance_to_goal(start: Node, goal: np.ndarray[int]) -> float:
    """Get the distance to the goal, whether the goal is a single point or multiple."""
    if goal.ndim == 1:
        return np.linalg.norm(start.location - goal)
    return np.min(np.linalg.norm(start.location - goal, axis=1))


def a_star(world: GridWorld, start: np.ndarray[int], goal: np.ndarray[int]) -> list[np.ndarray[int]] | None:
    """The classic."""
    # setup
    start_node = Node(start)
    start_node.g = 0
    start_node.h = distance_to_goal(start_node, goal)
    start_node.score = start_node.g + start_node.h

    open = [start_node]
    heapq.heapify(open)
    g_scores = {start_node: 0}

    # finding the path
    while len(open) > 0:
        current_node = heapq.heappop(open)

        # check if we've reached the goal:
        if (current_node.location in goal if isinstance(goal, list) else current_node.location == goal):
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
            g = current_node.g + current_node.distance_to(neighbor_node)
            if g < g_scores.get(neighbor_node, float("inf")):
                neighbor_node.g = g
                neighbor_node.h = distance_to_goal(neighbor_node, goal)
                neighbor_node.score = neighbor_node.g + neighbor_node.h
                g_scores[neighbor_node] = g

                heapq.heappush(open, neighbor_node)

    # no path found
    return None


def theta_star(world: GridWorld, start: np.ndarray[int], goal: np.ndarray[int]) -> list[np.ndarray[int]] | None:
    """Calculates any-angle paths for more realistic and efficient movement."""
    # setup
    start_node = Node(start)
    goal_node = Node(goal)
    start_node.g = 0
    start_node.h = distance_to_goal(start_node, goal)
    start_node.score = start_node.g + start_node.h

    open = [start_node]
    heapq.heapify(open)
    g_scores = {start_node: 0}

    # finding the path
    while len(open) > 0:
        current_node = heapq.heappop(open)

        # check if we've reached the goal:
        if (current_node.location in goal if isinstance(goal, list) else current_node.location == goal):
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

            # take the node if it introduces a shorter path, considering the path from the parent as well for any-angle functionality
            neighbor_node = Node(neighbor, current_node)
            g = 0
            if current_node.parent and world.line_of_sight(current_node.parent.location, neighbor_node.location):
                g = current_node.parent.g + current_node.parent.distance_to(neighbor_node)
                neighbor_node.parent = current_node.parent
            else:
                g = current_node.g + current_node.distance_to(neighbor_node)
            
            if g < g_scores.get(neighbor_node, float("inf")):
                neighbor_node.g = g
                neighbor_node.h = distance_to_goal(neighbor_node, goal)
                neighbor_node.score = neighbor_node.g + neighbor_node.h
                g_scores[neighbor_node] = g

                heapq.heappush(open, neighbor_node)

    # no path found
    return None


def ana_star(world: GridWorld, start: np.ndarray[int], goal: np.ndarray[int], current_path: list[np.ndarray[int]]) -> list[np.ndarray[int]] | None:
    """
    Anytime variant that makes progressively better paths if time is a limiting factor.
    
    Args:
        world (GridWorld): representation of the map
        start (np.ndarray[int]): starting location to plan from
        goal (np.ndarray[int]): target location to plan to
        current_path (list[np.ndarray[int]]): a variable to store and get the current path

    Returns: 
        path (list[np.ndarray[int]]): the final best path found
    """
    # setup
    start_node = Node(start)
    goal_node = Node(goal)
    start_node.g = 0
    start_node.h = distance_to_goal(start_node, goal)
    G = float('inf') # current best cost
    E = float('inf') # sub-optimality
    start_node.score = -1 * ((G - start_node.g) / start_node.h)

    open = [start_node]
    heapq.heapify(open)
    g_scores = {start_node: 0}

    def e(node):
        if node.h == 0:
            return -float('inf')
        return -1 * ((G - node.g) / node.h)

    def improve_solution():
        nonlocal E, G
        while len(open) > 0:
            current_node = heapq.heappop(open)

            # update suboptimality bound
            if current_node.score < E:
                E = current_node.score

            # check if we've reached the goal:
            if (current_node.location in goal if isinstance(goal, list) else current_node.location == goal):
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
                g = current_node.g + current_node.distance_to(neighbor_node)
                if g < g_scores.get(neighbor_node, float("inf")):
                    neighbor_node.g = g
                    neighbor_node.h = distance_to_goal(neighbor_node, goal)
                    if neighbor_node.g + neighbor_node.h < G:
                        neighbor_node.score = e(neighbor_node)
                        g_scores[neighbor_node] = g
                        heapq.heappush(open, neighbor_node)

    # finding the path
    while len(open) > 0:
        new_path = improve_solution()
        if new_path is None:
            break
        current_path[:] = new_path
        # prune nodes that cannot give a shorter path
        i = 0
        while i < len(open):
            if open[i].g + open[i].h >= G:
                open.pop(i)
            else:
                # recalculate
                open[i].score = e(open[i])
            i += 1
        heapq.heapify(open)

    return current_path


def ana_theta_star(
    world: GridWorld,
    start: np.ndarray[int],
    goal: np.ndarray[int],
    path_name: str,
    shared_paths: dict,
    shared_ready_events: dict,
    cancel_event: threading.Event | None = None,
) -> list[np.ndarray[int]] | None:
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
    logger.debug(f"planning with ana_theta_star() from {start} to {goal}")
    # setup
    start_node = Node(start)
    goal_node = Node(goal)
    start_node.g = 0
    start_node.h = distance_to_goal(start_node, goal)
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
            if cancel_event is not None and cancel_event.is_set():
                return None
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
                if cancel_event is not None and cancel_event.is_set():
                    return None
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
                    neighbor_node.h = distance_to_goal(neighbor_node, goal)
                    if neighbor_node.g + neighbor_node.h < G:
                        neighbor_node.score = e(neighbor_node)
                        g_scores[neighbor_node] = g
                        heapq.heappush(open, neighbor_node)

    # finding the path
    while len(open) > 0:
        if cancel_event is not None and cancel_event.is_set():
            return None
        new_path = improve_solution()
        if new_path is None:
            break
        if cancel_event is not None and cancel_event.is_set():
            return None
        shared_paths[path_name] = new_path
        shared_ready_events[path_name].set()
        # prune nodes that cannot give a shorter path
        open = [node for node in open if node.g + node.h < G]
        for node in open:
            node.score = e(node)
        heapq.heapify(open)

    if cancel_event is not None and cancel_event.is_set():
        return None
    logger.debug(f"[test1] {path_name} path: {[pos.tolist() for pos in shared_paths[path_name]]}")
    return shared_paths[path_name]