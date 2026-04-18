import argparse
import heapq
import threading
import time
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch

from world import GridWorld, Node


@dataclass
class StepFrame:
    current: tuple[int, int] | None
    considered: list[tuple[int, int]]
    selected: list[tuple[int, int]]
    path: list[tuple[int, int]] | None = None


def distance_to_goal(start: Node, goal: np.ndarray | tuple[int, ...]) -> float:
    """Match mujococodebase/planning/planning.py: single goal or closest of multiple goals."""
    g = np.asarray(goal)
    loc = np.asarray(start.location)
    if g.ndim == 1:
        return float(np.linalg.norm(loc - g))
    return float(np.min(np.linalg.norm(loc - g, axis=1)))


def goal_reached(location: tuple[int, ...], goal: np.ndarray | tuple[int, ...]) -> bool:
    """Match planning.py goal test: np.any(np.all(current_node.location == np.atleast_2d(goal), axis=1))."""
    return bool(np.any(np.all(np.asarray(location) == np.atleast_2d(np.asarray(goal)), axis=1)))


def make_demo_grid(rows: int = 30, cols: int = 30) -> np.ndarray:
    grid = np.zeros((rows, cols), dtype=int)

    # Maze-like layout with two main channels:
    # - an upper channel that's discovered quickly but tends to be longer
    # - a lower channel that can yield a shorter path after more search
    # This helps demonstrate ANA* Theta* returning an early suboptimal path.
    grid[4:26, 8] = 1
    grid[2:23, 15] = 1
    grid[7:29, 21] = 1

    # Primary openings (upper route is easier to find early).
    grid[5:8, 8] = 0
    grid[9:12, 15] = 0
    grid[11:14, 21] = 0

    # Secondary openings that enable a shorter route once explored.
    grid[19:22, 8] = 0
    grid[23:26, 15] = 0
    grid[24:27, 21] = 0

    # Add blockers to make the upper route bend more (usually longer).
    grid[1:4, 24] = 1
    grid[14:18, 24] = 1
    grid[18, 22:27] = 1
    return grid


def ana_theta_star_trace(
    world: GridWorld,
    start: tuple[int, int],
    goal: tuple[int, int] | np.ndarray,
    cancel_event: threading.Event | None = None,
) -> tuple[list[tuple[int, int]] | None, list[StepFrame]]:
    start_node = Node(start)
    start_node.g = 0
    start_node.h = distance_to_goal(start_node, goal)
    G = float("inf")
    E = float("inf")
    start_node.score = -1 * ((G - start_node.g) / start_node.h)

    open_nodes = [start_node]
    heapq.heapify(open_nodes)
    g_scores: dict[Node, float] = {start_node: 0.0}
    frames: list[StepFrame] = []
    expansions = 0
    improve_calls = 0
    solution_timings_ms: list[float] = []
    trace_start_time = time.perf_counter()

    def e(node: Node) -> float:
        if node.h == 0:
            return -float("inf")
        if G == float("inf"):
            return node.h
        return -1 * ((G - node.g) / node.h)

    def reconstruct_path(last: Node) -> list[tuple[int, int]]:
        path = []
        node = last
        while node is not None:
            path.append(node.location)
            node = node.parent
        return path[::-1]

    best_path: list[tuple[int, int]] | None = None

    def improve_solution() -> tuple[list[tuple[int, int]] | None, str]:
        nonlocal E, G, expansions, improve_calls
        improve_calls += 1
        pass_start_expansions = expansions
        print(
            f"[trace] pass {improve_calls} start | open={len(open_nodes)} | "
            f"best_cost={'inf' if G == float('inf') else f'{G:.3f}'}"
        )
        while len(open_nodes) > 0:
            if cancel_event is not None and cancel_event.is_set():
                return None, "cancelled"
            current_node = heapq.heappop(open_nodes)
            expansions += 1
            if expansions % 100 == 0:
                print(
                    f"[trace] expansions={expansions}, current={current_node.location}, "
                    f"open={len(open_nodes)}, best_G={G}, frames={len(frames)}"
                )

            if current_node.score < E:
                E = current_node.score

            if goal_reached(current_node.location, goal):
                G = current_node.g
                found_path = reconstruct_path(current_node)
                elapsed_ms = (time.perf_counter() - trace_start_time) * 1000.0
                solution_timings_ms.append(elapsed_ms)
                print(
                    f"[trace] pass {improve_calls} found path | len={len(found_path)} | "
                    f"cost={G:.3f} | expanded_this_pass={expansions - pass_start_expansions} | "
                    f"open_after_goal={len(open_nodes)} | t={elapsed_ms:.1f} ms"
                )
                frames.append(
                    StepFrame(
                        current=current_node.location,
                        considered=[],
                        selected=[],
                        path=found_path,
                    )
                )
                return found_path, "goal_reached"

            considered: list[tuple[int, int]] = []
            selected: list[tuple[int, int]] = []

            for neighbor in world.neighbors(current_node.location):
                if cancel_event is not None and cancel_event.is_set():
                    return None, "cancelled"
                if world.is_occupied(neighbor):
                    continue

                considered.append(neighbor)

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
                        selected.append(neighbor)
                        heapq.heappush(open_nodes, neighbor_node)

            frames.append(
                StepFrame(
                    current=current_node.location,
                    considered=considered,
                    selected=selected,
                    path=None,
                )
            )

        print(
            f"[trace] pass {improve_calls} exhausted frontier | "
            f"expanded_this_pass={expansions - pass_start_expansions} | open=0"
        )
        return None, "frontier_exhausted"

    while len(open_nodes) > 0:
        if cancel_event is not None and cancel_event.is_set():
            print("[trace] stop: cancel_event set")
            break
        open_before_prune = len(open_nodes)
        new_path, reason = improve_solution()
        if new_path is None:
            print(f"[trace] stop: improve_solution returned None ({reason})")
            break
        if cancel_event is not None and cancel_event.is_set():
            print("[trace] stop: cancel_event set after pass")
            break
        best_path = new_path

        # Same pruning/re-scoring structure as ana_theta_star in mujococodebase/planning/planning.py
        open_nodes = [node for node in open_nodes if node.g + node.h < G]
        pruned_count = open_before_prune - len(open_nodes)
        for node in open_nodes:
            node.score = e(node)
        heapq.heapify(open_nodes)
        print(
            f"[trace] post-pass {improve_calls} prune | removed={pruned_count} | "
            f"remaining_open={len(open_nodes)} | best_cost={G:.3f}"
        )

    if cancel_event is not None and cancel_event.is_set():
        return None, frames

    if best_path is None:
        print(
            f"[trace] finished: no path found, expansions={expansions}, "
            f"frames={len(frames)}, g_scores={len(g_scores)}"
        )
    else:
        total_solutions = sum(1 for frame in frames if frame.path)
        print(
            f"[trace] finished: solutions={total_solutions}, best_path_len={len(best_path)}, "
            f"expansions={expansions}, frames={len(frames)}"
        )
        for idx, elapsed_ms in enumerate(solution_timings_ms, start=1):
            delta_ms = elapsed_ms - (solution_timings_ms[idx - 2] if idx > 1 else 0.0)
            print(
                f"[trace] solution {idx}: cumulative={elapsed_ms:.1f} ms, "
                f"since_prev={delta_ms:.1f} ms"
            )
    return best_path, frames


def animate_trace(
    grid_world: GridWorld,
    start: tuple[int, int],
    goal: tuple[int, int],
    frames: list[StepFrame],
    interval_ms: int,
    save_path: str | None,
) -> None:
    rows, cols = grid_world._grid.shape

    fig, ax = plt.subplots(figsize=(12, 10))
    # Minimize outer whitespace while keeping room for the legend.
    fig.subplots_adjust(left=0.02, right=0.88, bottom=0.02, top=0.96)
    cmap = ListedColormap(
        [
            "#ffffff",  # free
            "#e0e0e0",  # inflation
            "#2f2f2f",  # obstacle
            "#f6c344",  # considered
            "#55acee",  # selected
            "#2ecc71",  # current
            "#27ae60",  # start
            "#e74c3c",  # goal
        ]
    )

    base = np.zeros((rows, cols), dtype=int)
    base[grid_world._grid == 0.5] = 1
    base[grid_world._grid >= 1] = 2

    img = ax.imshow(base, cmap=cmap, vmin=0, vmax=7, interpolation="none")
    path_line, = ax.plot([], [], color="#d55e00", linewidth=12, solid_capstyle="round")
    old_paths_collection = LineCollection([], colors="#f4c7a1", linewidths=8, alpha=0.9)
    ax.add_collection(old_paths_collection)
    ax.set_xticks(np.arange(-0.5, cols, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, rows, 1), minor=True)
    ax.grid(which="minor", color="#d9d9d9", linewidth=0.5)
    ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
    legend_handles = [
        Patch(facecolor="#2f2f2f", label="Obstacle"),
        Patch(facecolor="#f6c344", label="Considered"),
        Patch(facecolor="#55acee", label="Selected"),
        Patch(facecolor="#2ecc71", label="Current"),
        Patch(facecolor="#f4c7a1", label="Old Path"),
        Patch(facecolor="#d55e00", label="Path"),
        Patch(facecolor="#27ae60", label="Start"),
        Patch(facecolor="#e74c3c", label="Goal"),
    ]
    ax.legend(
        handles=legend_handles,
        loc="upper left",
        bbox_to_anchor=(1.005, 1.0),
        frameon=True,
        borderaxespad=0.0,
        fontsize=12,
    )

    # Precompute cumulative selected history and path history for fast rendering.
    cumulative_selected_by_frame: list[set[tuple[int, int]]] = []
    old_paths_by_frame: list[list[list[tuple[int, int]]]] = []
    best_path_by_frame: list[list[tuple[int, int]] | None] = []
    selected_running: set[tuple[int, int]] = set()
    found_paths: list[list[tuple[int, int]]] = []
    for frame in frames:
        selected_running.update(frame.selected)
        # Old paths = superseded routes only, never the current best. On a solution frame,
        # `found_paths` still holds previous solutions, then we append the new one below.
        if frame.path:
            old_paths_by_frame.append([p[:] for p in found_paths])
            found_paths.append(frame.path[:])
        else:
            old_paths_by_frame.append([p[:] for p in found_paths[:-1]])
        best_path_by_frame.append(found_paths[-1][:] if found_paths else None)
        cumulative_selected_by_frame.append(set(selected_running))

    # Fast-forward after the last discovered solution to avoid long "no-improvement" tails.
    solution_indices = [i for i, frame in enumerate(frames) if frame.path]
    if solution_indices:
        last_solution_idx = solution_indices[-1]
        display_indices = list(range(0, last_solution_idx + 1))
        tail_start = last_solution_idx + 1
        tail_end = len(frames) - 1
        if tail_start < tail_end:
            step = max(1, (tail_end - tail_start) // 25)
            display_indices.extend(list(range(tail_start, tail_end, step)))
        if len(frames) > 1 and display_indices[-1] != len(frames) - 1:
            display_indices.append(len(frames) - 1)
    else:
        display_indices = list(range(len(frames)))

    # Hold the final frame for ~2.5 seconds before loop restart.
    final_hold_frames = max(1, int(round(2500 / max(1, interval_ms))))
    if display_indices:
        display_indices.extend([display_indices[-1]] * final_hold_frames)

    total_solutions = sum(1 for frame in frames if frame.path)

    def draw_step(animation_idx: int):
        idx = display_indices[animation_idx]
        frame = frames[idx]
        canvas = base.copy()
        is_final_frame = idx == len(frames) - 1
        is_terminal_no_improvement = is_final_frame and (total_solutions <= 1)

        # Keep history of previously selected cells blue.
        selected_history_prev = cumulative_selected_by_frame[idx]
        for r, c in selected_history_prev:
            if canvas[r, c] == 0:
                canvas[r, c] = 4

        if not is_final_frame:
            # Show currently considered cells in yellow, even if they were blue before.
            considered_set = set(frame.considered)
            for r, c in considered_set:
                if canvas[r, c] in (0, 4):
                    canvas[r, c] = 3

            # Current step selected cells also appear yellow now; they turn blue in later frames.
            if frame.current is not None:
                r, c = frame.current
                if canvas[r, c] in (0, 3, 4):
                    canvas[r, c] = 5

        canvas[start[0], start[1]] = 6
        canvas[goal[0], goal[1]] = 7
        img.set_data(canvas)

        old_segments = [[(c, r) for r, c in path] for path in old_paths_by_frame[idx] if len(path) > 1]
        if is_terminal_no_improvement:
            old_segments = []
        old_paths_collection.set_segments(old_segments)

        path_to_draw = best_path_by_frame[idx]
        if path_to_draw:
            xs = [c for r, c in path_to_draw]
            ys = [r for r, c in path_to_draw]
            path_line.set_data(xs, ys)
            if is_terminal_no_improvement:
                path_line.set_color("#d62728")
            else:
                path_line.set_color("#d55e00")
        else:
            path_line.set_data([], [])
            path_line.set_color("#d55e00")

        ax.set_title("ANA* Theta* search")
        return (img, path_line, old_paths_collection)

    ani = animation.FuncAnimation(
        fig,
        draw_step,
        frames=len(display_indices),
        interval=interval_ms,
        blit=False,
        repeat=True,
    )

    if save_path:
        ani.save(
            save_path,
            writer="pillow",
            fps=max(1, 1000 // interval_ms),
            dpi=240,
        )
        print(f"Saved animation to {save_path}")
    else:
        plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description="Animate ANA* Theta* search progression.")
    parser.add_argument("--interval-ms", type=int, default=160, help="Milliseconds between animation frames.")
    parser.add_argument(
        "--save",
        type=str,
        default="ana_theta_star_demo.gif",
        help="Output GIF path (Pillow writer). Ignored if --show is set.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Open an interactive window instead of saving a GIF.",
    )
    args = parser.parse_args()

    raw_grid = make_demo_grid()
    world = GridWorld(raw_grid.tolist())
    # Demo-only: remove inflated safety cells so only true obstacles remain.
    world._grid[world._grid == 0.5] = 0.0
    start = (2, 2)
    goal = (26, 27)

    path, frames = ana_theta_star_trace(world, start, goal)
    if path is None:
        print("No path found in demo map.")
    else:
        print(f"Path found with {len(path)} waypoints over {len(frames)} animated steps.")

    save_path = None if args.show else args.save
    animate_trace(world, start, goal, frames, args.interval_ms, save_path)


if __name__ == "__main__":
    main()
