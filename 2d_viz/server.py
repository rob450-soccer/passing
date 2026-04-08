#!/usr/bin/env python3
"""
server.py — Zero-dependency 2D path visualizer for RCSSServerMJ.

Usage:
    python server.py
    python server.py --udp-port 60002 --http-port 8765

Then open http://localhost:8765 in any browser.

UDP payload format (agents send JSON to --udp-port):
    {
        "player":       1,
        "team":         "TeamA",
        "plan":         [[x0,y0], [x1,y1], ...],
        "current_step": 3,
        "trail":        [[x, y], ...],
        "state":        "GO_TO_BALL",
        "target":       [x, y] | null,
        "ball":         [x, y] | null,
        "is_passer":    true | false | null
    }

All coordinates are world-space metres, origin at field centre, +X right, +Y up.
Field dimensions default to HLAdult2020 (14 m x 9 m).
"""

from __future__ import annotations

import argparse
import json
import mimetypes
import os
import socket
import threading
from collections import defaultdict
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from typing import Any

# ---------------------------------------------------------------------------
# Field geometry (HLAdult2020, metres)
# ---------------------------------------------------------------------------
FIELD_CONFIG = {
    "FIELD_W":       14.0,
    "FIELD_H":        9.0,
    "FIELD_BORDER":   1.5,
    "GOAL_DEPTH":     0.6,
    "GOAL_W":         2.6,
    "GOALIE_DEPTH":   1.0,
    "GOALIE_W":       4.0,
    "PENALTY_DEPTH":  3.0,
    "PENALTY_W":      6.0,
    "CENTER_R":       1.5,
    "PENALTY_SPOT_D": 2.1,
}

UDP_PORT      = 60002
HTTP_PORT     = 8765
TRAIL_HISTORY = 80

# Static files are served from the "static/" directory next to this script.
STATIC_DIR = Path(__file__).parent / "static"

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------
_lock   = threading.Lock()
_paths:  dict[str, dict]    = {}
_trails: dict[str, list]    = defaultdict(list)
_ball:   list[float] | None = None


def _key(team: str, player: int) -> str:
    return f"{team}:{player}"


# ---------------------------------------------------------------------------
# UDP listener thread
# ---------------------------------------------------------------------------
def _udp_thread(port: int) -> None:
    global _ball
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", port))
    sock.settimeout(1.0)
    print(f"[PathViz] UDP listening on :{port}")
    while True:
        try:
            raw, _ = sock.recvfrom(65535)
            payload = json.loads(raw.decode())
            team   = str(payload["team"])
            player = int(payload["player"])
            k      = _key(team, player)
            with _lock:
                if payload.get("ball"):
                    _ball = list(payload["ball"])
                if payload.get("trail"):
                    hist = _trails[k]
                    hist.extend([list(pt) for pt in payload["trail"]])
                    if len(hist) > TRAIL_HISTORY:
                        _trails[k] = hist[-TRAIL_HISTORY:]
                _paths[k] = payload
        except (TimeoutError, OSError):
            continue
        except (json.JSONDecodeError, KeyError, ValueError):
            continue


# ---------------------------------------------------------------------------
# HTTP handler
# ---------------------------------------------------------------------------
class Handler(BaseHTTPRequestHandler):

    def log_message(self, *_: Any) -> None:
        pass

    def do_GET(self) -> None:
        if self.path == "/state":
            self._serve_state()
        elif self.path == "/config":
            self._serve_config()
        else:
            self._serve_static()

    def _serve_state(self) -> None:
        with _lock:
            agents = []
            for k, payload in _paths.items():
                agents.append({
                    "team":         payload.get("team", ""),
                    "player":       payload.get("player", 0),
                    "plan":         payload.get("plan", []),
                    "current_step": payload.get("current_step", 0),
                    "target":       payload.get("target"),
                    "trail":        list(_trails.get(k, [])),
                    "state":        payload.get("state", ""),
                    "is_passer":    payload.get("is_passer"),
                })
            state = {"agents": agents, "ball": _ball}

        self._send_json(state)

    def _serve_config(self) -> None:
        """Expose field geometry constants to the browser."""
        self._send_json(FIELD_CONFIG)

    def _serve_static(self) -> None:
        # Resolve the requested path to a file under STATIC_DIR.
        # Default to index.html for bare /.
        req = self.path.lstrip("/") or "index.html"
        filepath = (STATIC_DIR / req).resolve()

        # Safety check: don't serve anything outside STATIC_DIR.
        try:
            filepath.relative_to(STATIC_DIR.resolve())
        except ValueError:
            self._send_error(403, "Forbidden")
            return

        if not filepath.is_file():
            self._send_error(404, "Not found")
            return

        mime, _ = mimetypes.guess_type(str(filepath))
        body = filepath.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", mime or "application/octet-stream")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, data: Any) -> None:
        body = json.dumps(data).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _send_error(self, code: int, message: str) -> None:
        body = message.encode()
        self.send_response(code)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main() -> None:
    parser = argparse.ArgumentParser(description="Zero-dep 2D path viz for RCSSServerMJ")
    parser.add_argument("--udp-port",  type=int, default=UDP_PORT,
                        help=f"UDP port agents send path data to (default {UDP_PORT})")
    parser.add_argument("--http-port", type=int, default=HTTP_PORT,
                        help=f"Browser UI port (default {HTTP_PORT})")
    args = parser.parse_args()

    t = threading.Thread(target=_udp_thread, args=(args.udp_port,), daemon=True)
    t.start()

    server = HTTPServer(("", args.http_port), Handler)
    print(f"[PathViz] open  http://localhost:{args.http_port}  in your browser")
    print("[PathViz] Ctrl-C to quit")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()