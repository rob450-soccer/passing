// viz.js — canvas rendering for RCSSServerMJ Path Viz
// Fetches field config from /config, then polls /state every 80ms.

// ---------------------------------------------------------------------------
// Field geometry — loaded from /config at startup
// ---------------------------------------------------------------------------
let FW, FH, FB, GD, GW, GAYD, GAYW, PAD, PAW, CR, PSD;
let TW, TH; // total canvas world size including border

// ---------------------------------------------------------------------------
// State colours — keep in sync with State enum in decision_maker.py
// ---------------------------------------------------------------------------
const STATE_COLOURS = {
  GO_TO_BALL:             "#f5a623",
  DRIBBLE:                "#e74c3c",
  GO_TO_RECEIVE_POSITION: "#3498db",
  WAIT_FOR_PASS:          "#9b59b6",
  GETTING_UP:             "#e67e22",
  BEAMING:                "#1abc9c",
  NEUTRAL:                "#555",
};

function stateCol(s) {
  return STATE_COLOURS[s] || "#888";
}

// ---------------------------------------------------------------------------
// Team colour palette
// ---------------------------------------------------------------------------
const PALETTE = ["#29be87", "#f05a2d", "#3096e8", "#d8a81e", "#c83ec8", "#30c8c8"];
const teamCol = {};
let palIdx = 0;

function col(team) {
  if (!(team in teamCol)) teamCol[team] = PALETTE[palIdx++ % PALETTE.length];
  return teamCol[team];
}

// ---------------------------------------------------------------------------
// Canvas setup
// ---------------------------------------------------------------------------
const canvas = document.getElementById("c");
const ctx = canvas.getContext("2d");

function resize() {
  const m = 16;
  const maxW = window.innerWidth - m;
  const maxH = window.innerHeight - 44;
  const s = Math.min(maxW / TW, maxH / TH);
  canvas.width  = Math.floor(TW * s);
  canvas.height = Math.floor(TH * s);
}

window.addEventListener("resize", resize);

// ---------------------------------------------------------------------------
// World-to-canvas coordinate helpers
// ---------------------------------------------------------------------------
function wx(x) { return (x + TW / 2) / TW * canvas.width; }
function wy(y) { return (TH / 2 - y) / TH * canvas.height; }
function wl(m) { return m / TW * canvas.width; }

// ---------------------------------------------------------------------------
// Drawing: field
// ---------------------------------------------------------------------------
function drawField() {
  ctx.fillStyle = "#227522";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  const lw = Math.max(1, wl(0.05));
  ctx.lineWidth = lw;
  ctx.strokeStyle = "#ddd";

  const hw = FW / 2, hh = FH / 2;

  // Outer boundary
  ctx.strokeRect(wx(-hw), wy(hh), wl(FW), wl(FH));

  // Halfway line
  ctx.beginPath();
  ctx.moveTo(wx(0), wy(hh));
  ctx.lineTo(wx(0), wy(-hh));
  ctx.stroke();

  // Centre circle
  ctx.beginPath();
  ctx.arc(wx(0), wy(0), wl(CR), 0, 2 * Math.PI);
  ctx.stroke();

  // Centre spot + penalty spots
  ctx.fillStyle = "#ddd";
  for (const [x, y] of [[0, 0], [-hw + PSD, 0], [hw - PSD, 0]]) {
    ctx.beginPath();
    ctx.arc(wx(x), wy(y), Math.max(2, wl(0.07)), 0, 2 * Math.PI);
    ctx.fill();
  }

  // Goalie boxes
  const gah = GAYW / 2;
  ctx.strokeRect(wx(-hw),       wy(gah), wl(GAYD), wl(GAYW));
  ctx.strokeRect(wx(hw - GAYD), wy(gah), wl(GAYD), wl(GAYW));

  // Penalty boxes
  const pah = PAW / 2;
  ctx.strokeRect(wx(-hw),       wy(pah), wl(PAD), wl(PAW));
  ctx.strokeRect(wx(hw - PAD),  wy(pah), wl(PAD), wl(PAW));

  // Goals
  const gw2 = GW / 2;
  ctx.strokeStyle = "#cccc30";
  ctx.strokeRect(wx(-hw - GD), wy(gw2), wl(GD), wl(GW));
  ctx.strokeRect(wx(hw),       wy(gw2), wl(GD), wl(GW));
}

// ---------------------------------------------------------------------------
// Drawing: agents
// ---------------------------------------------------------------------------
function drawAgents(list) {
  for (const ag of list) {
    const c  = col(ag.team);
    const sc = stateCol(ag.state);

    // Trail
    if (ag.trail.length >= 2) {
      ctx.beginPath();
      ctx.moveTo(wx(ag.trail[0][0]), wy(ag.trail[0][1]));
      for (let i = 1; i < ag.trail.length; i++) {
        ctx.lineTo(wx(ag.trail[i][0]), wy(ag.trail[i][1]));
      }
      ctx.strokeStyle = c + "55";
      ctx.lineWidth = Math.max(1, wl(0.045));
      ctx.stroke();
    }

    // Planned path line
    if (ag.plan.length >= 2) {
      ctx.beginPath();
      ctx.moveTo(wx(ag.plan[0][0]), wy(ag.plan[0][1]));
      for (let i = 1; i < ag.plan.length; i++) {
        ctx.lineTo(wx(ag.plan[i][0]), wy(ag.plan[i][1]));
      }
      ctx.strokeStyle = c + "bb";
      ctx.lineWidth = Math.max(1, wl(0.028));
      ctx.stroke();
    }

    // Waypoint circles — visited ones are dimmed
    for (let i = 0; i < ag.plan.length; i++) {
      const wp = ag.plan[i];
      const visited = i < ag.current_step;
      ctx.beginPath();
      ctx.arc(wx(wp[0]), wy(wp[1]), Math.max(3, wl(0.12)), 0, 2 * Math.PI);
      ctx.fillStyle   = visited ? c + "44" : c + "cc";
      ctx.fill();
      ctx.strokeStyle = "#0008";
      ctx.lineWidth   = 1;
      ctx.stroke();
    }

    // Target crosshair
    if (ag.target) {
      const tx = wx(ag.target[0]), ty = wy(ag.target[1]);
      const r  = Math.max(4, wl(0.15));
      ctx.strokeStyle = "#ffe020";
      ctx.lineWidth   = 2;
      ctx.beginPath();
      ctx.arc(tx, ty, r, 0, 2 * Math.PI);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(tx - r - 4, ty); ctx.lineTo(tx + r + 4, ty);
      ctx.moveTo(tx, ty - r - 4); ctx.lineTo(tx, ty + r + 4);
      ctx.stroke();
    }

    // Agent dot — outer ring = team colour, inner fill = FSM state colour
    const lp = ag.trail.length ? ag.trail[ag.trail.length - 1] : ag.plan[0];
    if (lp) {
      const ax   = wx(lp[0]), ay = wy(lp[1]);
      const dotR = Math.max(5, wl(0.2));

      ctx.beginPath();
      ctx.arc(ax, ay, dotR + 2, 0, 2 * Math.PI);
      ctx.fillStyle = c;
      ctx.fill();

      ctx.beginPath();
      ctx.arc(ax, ay, dotR, 0, 2 * Math.PI);
      ctx.fillStyle = sc;
      ctx.fill();

      // Passer crown arc
      if (ag.is_passer) {
        ctx.strokeStyle = "#fff";
        ctx.lineWidth   = 2;
        ctx.beginPath();
        ctx.arc(ax, ay, dotR + 5, Math.PI * 1.1, Math.PI * 1.9);
        ctx.stroke();
      }

      // Label above dot
      const fs = Math.max(9, wl(0.45));
      ctx.font      = `bold ${fs}px monospace`;
      ctx.textAlign = "center";
      ctx.fillStyle = "#fff";
      ctx.fillText(`${ag.team.slice(0, 4)}#${ag.player}`, ax, ay - dotR - 3);

      // State label below dot
      const shortState = (ag.state || "").replace(/_/g, " ").toLowerCase();
      ctx.font      = `${Math.max(8, wl(0.38))}px monospace`;
      ctx.fillStyle = sc;
      ctx.fillText(shortState, ax, ay + dotR + fs);
    }
  }
}

// ---------------------------------------------------------------------------
// Drawing: ball
// ---------------------------------------------------------------------------
function drawBall(b) {
  if (!b) return;
  const r = Math.max(4, wl(0.13));
  ctx.beginPath();
  ctx.arc(wx(b[0]), wy(b[1]), r, 0, 2 * Math.PI);
  ctx.fillStyle   = "#f5a623";
  ctx.fill();
  ctx.strokeStyle = "#fff9";
  ctx.lineWidth   = Math.max(1, wl(0.03));
  ctx.stroke();
}

// ---------------------------------------------------------------------------
// Drawing: legend
// ---------------------------------------------------------------------------
function drawLegend() {
  const teams = Object.keys(teamCol);
  if (!teams.length) return;

  const fs = Math.max(9, wl(0.42));
  ctx.textAlign = "left";
  let y = 8;

  for (const t of teams) {
    ctx.fillStyle = teamCol[t];
    ctx.fillRect(6, y, 10, 10);
    ctx.font      = `bold ${fs}px monospace`;
    ctx.fillStyle = "#ccc";
    ctx.fillText(t, 20, y + 10);
    y += fs + 5;
  }

  y += 6;
  for (const [name, c] of Object.entries(STATE_COLOURS)) {
    ctx.fillStyle = c;
    ctx.fillRect(6, y, 10, 10);
    ctx.font      = `${Math.max(8, fs - 1)}px monospace`;
    ctx.fillStyle = "#aaa";
    ctx.fillText(name.replace(/_/g, " ").toLowerCase(), 20, y + 10);
    y += fs + 3;
  }
}

// ---------------------------------------------------------------------------
// Poll loop
// ---------------------------------------------------------------------------
const statusEl = document.getElementById("status");
let connected = false;

async function poll() {
  try {
    const r = await fetch("/state");
    if (!r.ok) throw new Error("bad response");
    const d = await r.json();

    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawField();
    drawAgents(d.agents);
    drawBall(d.ball);
    drawLegend();

    statusEl.textContent = `${d.agents.length} agent(s) — ${new Date().toLocaleTimeString()}`;
    connected = true;
  } catch (e) {
    if (connected) {
      statusEl.textContent = "connection lost — retrying…";
      connected = false;
    }
  }
  setTimeout(poll, 80);
}

// ---------------------------------------------------------------------------
// Startup — fetch field config first, then begin rendering
// ---------------------------------------------------------------------------
async function init() {
  const r = await fetch("/config");
  const cfg = await r.json();

  FW   = cfg.FIELD_W;
  FH   = cfg.FIELD_H;
  FB   = cfg.FIELD_BORDER;
  GD   = cfg.GOAL_DEPTH;
  GW   = cfg.GOAL_W;
  GAYD = cfg.GOALIE_DEPTH;
  GAYW = cfg.GOALIE_W;
  PAD  = cfg.PENALTY_DEPTH;
  PAW  = cfg.PENALTY_W;
  CR   = cfg.CENTER_R;
  PSD  = cfg.PENALTY_SPOT_D;
  TW   = FW + 2 * FB;
  TH   = FH + 2 * FB;

  resize();
  poll();
}

init();