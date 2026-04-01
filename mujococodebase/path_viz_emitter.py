"""
Sends path planning overlay data to PathVizMonitor over UDP.
Non-blocking — safe to call from inside the agent step loop.
"""
import json
import socket
import threading
from collections import deque

_DEFAULT_HOST = "127.0.0.1"
_DEFAULT_PORT = 60002

_sock: socket.socket | None = None
_sock_lock = threading.Lock()


def _get_sock() -> socket.socket:
    global _sock
    if _sock is None:
        with _sock_lock:
            if _sock is None:
                _sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return _sock


def emit(
    player_num: int,
    team: str,
    planned_path: list,          # list of [x, y] grid coords
    grid_scale: int,             # divide by this to get world coords
    current_step: int,           # which waypoint index we're currently on
    current_pos: list,           # [x, y] world coords — agent's live position
    target_pos: list | None = None,  # [x, y] world coords — pass/goal target
    host: str = _DEFAULT_HOST,
    port: int = _DEFAULT_PORT,
) -> None:
    """
    Converts grid-space path to world-space and sends to PathVizMonitor.
    Call this once per step from _state_go_to_ball().
    """
    # Convert planned path from grid coords to world coords
    world_plan = [
        [pt[0] / grid_scale, pt[1] / grid_scale]
        for pt in planned_path
    ]

    # Highlight remaining waypoints differently from already-visited ones
    payload = {
        "player": player_num,
        "team": team,
        "plan": world_plan,               # full path in world coords
        "current_step": current_step,     # how far along we are
        "trail": [current_pos],           # server accumulates these over time
        "target": list(target_pos) if target_pos is not None else None,
    }
    try:
        raw = json.dumps(payload).encode()
        _get_sock().sendto(raw, (host, port))
    except OSError:
        pass  # never let viz crash the agent