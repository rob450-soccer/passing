#!/usr/bin/env python3
"""
path_viz.py — Zero-dependency 2D path visualizer for RCSSServerMJ.

No pip installs required. Uses only Python stdlib.

Usage:
    python path_viz.py
    python path_viz.py --udp-port 60002 --http-port 8765

Then open  http://localhost:8765  in any browser.

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
import socket
import threading
from collections import defaultdict
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Any

# -- Field geometry (HLAdult2020, metres) ------------------------------------
FIELD_W         = 14.0
FIELD_H         = 9.0
FIELD_BORDER    = 1.5
GOAL_DEPTH      = 0.6
GOAL_W          = 2.6
GOALIE_DEPTH    = 1.0
GOALIE_W        = 4.0
PENALTY_DEPTH   = 3.0
PENALTY_W       = 6.0
CENTER_R        = 1.5
PENALTY_SPOT_D  = 2.1

UDP_PORT        = 60002
HTTP_PORT       = 8765
TRAIL_HISTORY   = 80

# -- Shared state ------------------------------------------------------------
_lock   = threading.Lock()
_paths:  dict[str, dict]    = {}
_trails: dict[str, list]    = defaultdict(list)
_ball:   list[float] | None = None


def _key(team: str, player: int) -> str:
    return f"{team}:{player}"


# -- UDP listener ------------------------------------------------------------
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


# -- HTTP handler ------------------------------------------------------------
class Handler(BaseHTTPRequestHandler):

    def log_message(self, *_: Any) -> None:
        pass

    def do_GET(self) -> None:
        if self.path == "/state":
            self._serve_state()
        else:
            self._serve_html()

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

        body = json.dumps(state).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _serve_html(self) -> None:
        html = _build_html()
        body = html.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def _build_html() -> str:
    return (
        '<!DOCTYPE html><html lang="en"><head>'
        '<meta charset="utf-8">'
        '<title>RCSSServerMJ \u2014 Path Viz</title>'
        '<style>'
        '* { margin:0; padding:0; box-sizing:border-box; }'
        'body { background:#111; display:flex; flex-direction:column;'
        '       align-items:center; justify-content:center; height:100vh;'
        '       font-family:monospace; color:#aaa; }'
        'canvas { border:1px solid #2a2a2a; display:block; }'
        '#status { margin-top:5px; font-size:11px; }'
        '</style></head><body>'
        '<canvas id="c"></canvas>'
        '<div id="status">connecting\u2026</div>'
        '<script>'
        + _JS_TEMPLATE.format(
            FIELD_W=FIELD_W, FIELD_H=FIELD_H, FIELD_BORDER=FIELD_BORDER,
            GOAL_DEPTH=GOAL_DEPTH, GOAL_W=GOAL_W,
            GOALIE_DEPTH=GOALIE_DEPTH, GOALIE_W=GOALIE_W,
            PENALTY_DEPTH=PENALTY_DEPTH, PENALTY_W=PENALTY_W,
            CENTER_R=CENTER_R, PENALTY_SPOT_D=PENALTY_SPOT_D,
        )
        + '</script></body></html>'
    )


_JS_TEMPLATE = """
const FW={FIELD_W}, FH={FIELD_H}, FB={FIELD_BORDER};
const GD={GOAL_DEPTH}, GW={GOAL_W};
const GAYD={GOALIE_DEPTH}, GAYW={GOALIE_W};
const PAD={PENALTY_DEPTH}, PAW={PENALTY_W};
const CR={CENTER_R}, PSD={PENALTY_SPOT_D};
const TW=FW+2*FB, TH=FH+2*FB;

// State colours — matches State enum in decision_maker.py
const STATE_COLOURS = {{
  GO_TO_BALL:            "#f5a623",
  DRIBBLE:               "#e74c3c",
  GO_TO_RECEIVE_POSITION:"#3498db",
  WAIT_FOR_PASS:         "#9b59b6",
  GETTING_UP:            "#e67e22",
  BEAMING:               "#1abc9c",
  NEUTRAL:               "#555",
}};
function stateCol(s){{ return STATE_COLOURS[s] || "#888"; }}

const PALETTE=["#29be87","#f05a2d","#3096e8","#d8a81e","#c83ec8","#30c8c8"];
const teamCol={{}};
let palIdx=0;
function col(team){{
  if(!(team in teamCol)) teamCol[team]=PALETTE[palIdx++%PALETTE.length];
  return teamCol[team];
}}

const canvas=document.getElementById("c");
const ctx=canvas.getContext("2d");

function resize(){{
  const m=16, maxW=window.innerWidth-m, maxH=window.innerHeight-44;
  const s=Math.min(maxW/TW, maxH/TH);
  canvas.width=Math.floor(TW*s); canvas.height=Math.floor(TH*s);
}}
window.addEventListener("resize",resize); resize();

function wx(x){{ return (x+TW/2)/TW*canvas.width; }}
function wy(y){{ return (TH/2-y)/TH*canvas.height; }}
function wl(m){{ return m/TW*canvas.width; }}

function field(){{
  ctx.fillStyle="#162916"; ctx.fillRect(0,0,canvas.width,canvas.height);
  const lw=Math.max(1,wl(0.05));
  ctx.lineWidth=lw; ctx.strokeStyle="#ddd";
  const hw=FW/2, hh=FH/2;
  ctx.strokeRect(wx(-hw),wy(hh),wl(FW),wl(FH));
  ctx.beginPath(); ctx.moveTo(wx(0),wy(hh)); ctx.lineTo(wx(0),wy(-hh)); ctx.stroke();
  ctx.beginPath(); ctx.arc(wx(0),wy(0),wl(CR),0,2*Math.PI); ctx.stroke();
  ctx.fillStyle="#ddd";
  ctx.beginPath(); ctx.arc(wx(0),wy(0),Math.max(2,wl(0.07)),0,2*Math.PI); ctx.fill();
  ctx.beginPath(); ctx.arc(wx(-hw+PSD),wy(0),Math.max(2,wl(0.07)),0,2*Math.PI); ctx.fill();
  ctx.beginPath(); ctx.arc(wx( hw-PSD),wy(0),Math.max(2,wl(0.07)),0,2*Math.PI); ctx.fill();
  const gah=GAYW/2;
  ctx.strokeRect(wx(-hw),    wy(gah), wl(GAYD), wl(GAYW));
  ctx.strokeRect(wx(hw-GAYD),wy(gah), wl(GAYD), wl(GAYW));
  const pah=PAW/2;
  ctx.strokeRect(wx(-hw),    wy(pah), wl(PAD), wl(PAW));
  ctx.strokeRect(wx(hw-PAD), wy(pah), wl(PAD), wl(PAW));
  const gw2=GW/2;
  ctx.strokeStyle="#cccc30";
  ctx.strokeRect(wx(-hw-GD),wy(gw2), wl(GD), wl(GW));
  ctx.strokeRect(wx( hw),   wy(gw2), wl(GD), wl(GW));
}}

function agents(list){{
  for(const ag of list){{
    const c=col(ag.team);
    const sc=stateCol(ag.state);

    // Trail
    if(ag.trail.length>=2){{
      ctx.beginPath();
      ctx.moveTo(wx(ag.trail[0][0]),wy(ag.trail[0][1]));
      for(let i=1;i<ag.trail.length;i++)
        ctx.lineTo(wx(ag.trail[i][0]),wy(ag.trail[i][1]));
      ctx.strokeStyle=c+"55"; ctx.lineWidth=Math.max(1,wl(0.045)); ctx.stroke();
    }}

    // Plan connectors
    if(ag.plan.length>=2){{
      ctx.beginPath();
      ctx.moveTo(wx(ag.plan[0][0]),wy(ag.plan[0][1]));
      for(let i=1;i<ag.plan.length;i++)
        ctx.lineTo(wx(ag.plan[i][0]),wy(ag.plan[i][1]));
      ctx.strokeStyle=c+"bb"; ctx.lineWidth=Math.max(1,wl(0.028)); ctx.stroke();
    }}

    // Waypoints — visited ones dimmer, upcoming brighter
    for(let i=0;i<ag.plan.length;i++){{
      const wp=ag.plan[i];
      const visited=i<ag.current_step;
      ctx.beginPath(); ctx.arc(wx(wp[0]),wy(wp[1]),Math.max(3,wl(0.12)),0,2*Math.PI);
      ctx.fillStyle=visited ? c+"44" : c+"cc"; ctx.fill();
      ctx.strokeStyle="#0008"; ctx.lineWidth=1; ctx.stroke();
    }}

    // Target
    if(ag.target){{
      const tx=wx(ag.target[0]), ty=wy(ag.target[1]), r=Math.max(4,wl(0.15));
      ctx.strokeStyle="#ffe020"; ctx.lineWidth=2;
      ctx.beginPath(); ctx.arc(tx,ty,r,0,2*Math.PI); ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(tx-r-4,ty); ctx.lineTo(tx+r+4,ty);
      ctx.moveTo(tx,ty-r-4); ctx.lineTo(tx,ty+r+4);
      ctx.stroke();
    }}

    // Agent dot at latest trail position, coloured by FSM state
    const lp=ag.trail.length ? ag.trail[ag.trail.length-1] : ag.plan[0];
    if(lp){{
      const ax=wx(lp[0]), ay=wy(lp[1]);
      const dotR=Math.max(5,wl(0.2));

      // Outer ring: team colour
      ctx.beginPath(); ctx.arc(ax,ay,dotR+2,0,2*Math.PI);
      ctx.fillStyle=c; ctx.fill();

      // Inner fill: FSM state colour
      ctx.beginPath(); ctx.arc(ax,ay,dotR,0,2*Math.PI);
      ctx.fillStyle=sc; ctx.fill();

      // Passer crown indicator
      if(ag.is_passer){{
        ctx.strokeStyle="#fff"; ctx.lineWidth=2;
        ctx.beginPath(); ctx.arc(ax,ay,dotR+5,Math.PI*1.1,Math.PI*1.9); ctx.stroke();
      }}

      // Label: team#number + state abbreviation
      const fs=Math.max(9,wl(0.45));
      ctx.font=`bold ${{fs}}px monospace`; ctx.textAlign="center";
      ctx.fillStyle="#fff";
      ctx.fillText(`${{ag.team.slice(0,4)}}#${{ag.player}}`,ax,ay-dotR-3);
      // State label below dot
      const shortState=(ag.state||"").replace(/_/g," ").toLowerCase();
      ctx.font=`${{Math.max(8,wl(0.38))}}px monospace`;
      ctx.fillStyle=sc;
      ctx.fillText(shortState,ax,ay+dotR+fs);
    }}
  }}
}}

function ball(b){{
  if(!b) return;
  const r=Math.max(4,wl(0.13));
  ctx.beginPath(); ctx.arc(wx(b[0]),wy(b[1]),r,0,2*Math.PI);
  ctx.fillStyle="#f5a623"; ctx.fill();
  ctx.strokeStyle="#fff9"; ctx.lineWidth=Math.max(1,wl(0.03)); ctx.stroke();
}}

function legend(){{
  const teams=Object.keys(teamCol); if(!teams.length) return;
  const fs=Math.max(9,wl(0.42)); ctx.textAlign="left";
  let y=8;
  // Team colours
  for(const t of teams){{
    ctx.fillStyle=teamCol[t]; ctx.fillRect(6,y,10,10);
    ctx.font=`bold ${{fs}}px monospace`; ctx.fillStyle="#ccc";
    ctx.fillText(t,20,y+10); y+=fs+5;
  }}
  // State colour key
  y+=6;
  for(const [name,c] of Object.entries(STATE_COLOURS)){{
    ctx.fillStyle=c; ctx.fillRect(6,y,10,10);
    ctx.font=`${{Math.max(8,fs-1)}}px monospace`; ctx.fillStyle="#aaa";
    ctx.fillText(name.replace(/_/g," ").toLowerCase(),20,y+10); y+=fs+3;
  }}
}}

const status=document.getElementById("status");
let ok=false;
async function poll(){{
  try{{
    const r=await fetch("/state"); if(!r.ok) throw 0;
    const d=await r.json();
    ctx.clearRect(0,0,canvas.width,canvas.height);
    field(); agents(d.agents); ball(d.ball); legend();
    status.textContent=`${{d.agents.length}} agent(s) \u2014 ${{new Date().toLocaleTimeString()}}`;
    ok=true;
  }}catch(e){{
    if(ok){{ status.textContent="connection lost \u2014 retrying\u2026"; ok=false; }}
  }}
  setTimeout(poll,80);
}}
poll();
"""


# -- Entry point -------------------------------------------------------------
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