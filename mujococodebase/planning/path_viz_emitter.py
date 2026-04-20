"""
path_viz_emit.py
Sends path planning overlay data to the path_viz.py visualizer over UDP.
Non-blocking and exception-safe — safe to call from inside the agent step loop.
"""

import json
import socket
import threading

_DEFAULT_HOST = "127.0.0.1"
_DEFAULT_PORT = 60002

_sock: socket.socket | None = None
_sock_lock = threading.Lock()


def _get_sock() -> socket.socket:
    """Return the shared UDP socket, creating it on first call."""
    global _sock
    with _sock_lock:
        if _sock is None:
            _sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return _sock


def emit(
    player_num: int,
    team: str,
    planned_path: list,              # list of [x, y] in grid coords
    grid_scale: int,                 # divide by this to get world-space metres
    current_step: int,               # index of the waypoint we're currently targeting
    current_pos: list,               # [x, y] world coords — agent's live position
    state: str = "",                 # FSM state name, e.g. "GO_TO_BALL"
    play_mode: str = "",             # current simulator play mode
    target_pos: list | None = None,  # [x, y] world coords — pass/goal target
    ball_pos: list | None = None,    # [x, y] world coords — ball position
    is_passer: bool | None = None,   # whether this agent is the designated passer
    host: str = _DEFAULT_HOST,
    port: int = _DEFAULT_PORT,
) -> None:
    """
    Convert a grid-space planned path to world-space and send it to path_viz.py.

    Call once per agent step from _follow_path() (or any state handler).
    Failures are silently swallowed so the visualizer can never affect the agent.
    """
    world_plan = [
        [pt[0] / grid_scale, pt[1] / grid_scale]
        for pt in planned_path
    ]

    payload: dict = {
        "player":       player_num,
        "team":         team,
        "plan":         world_plan,
        "current_step": current_step,
        "trail":        [current_pos],   # viz server accumulates these over time
        "state":        state,
        "play_mode":    play_mode,
        "target":       list(target_pos) if target_pos is not None else None,
        "ball":         list(ball_pos)   if ball_pos  is not None else None,
        "is_passer":    is_passer,
    }

    try:
        _get_sock().sendto(json.dumps(payload).encode(), (host, port))
    except OSError:
        pass  # never let the visualizer crash the agent


def emit_shutdown(
    player_num: int,
    team: str,
    host: str = _DEFAULT_HOST,
    port: int = _DEFAULT_PORT,
) -> None:
    """
    Send an explicit visualizer shutdown packet for one agent.

    This lets the viz server remove the agent immediately instead of waiting
    for any timeout-based expiry.
    """
    payload = {
        "event": "shutdown",
        "player": player_num,
        "team": team,
    }

    try:
        _get_sock().sendto(json.dumps(payload).encode(), (host, port))
    except OSError:
        pass