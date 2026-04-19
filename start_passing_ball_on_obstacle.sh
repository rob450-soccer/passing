#!/usr/bin/env bash
# Run ./start_passing.sh with three static obstacle robots and the ball placed on
# obstacle 1's position (same x,y as that obstacle), so the ball sits inside an
# opponent body and is effectively unreachable for normal approach/dribble.
#
# Prerequisites: rcssservermj (or hatch equivalent) already listening for agents
# on the game port and the monitor on MONITOR_PORT (default 60001).
#
# Usage:
#   ./start_passing_ball_on_obstacle.sh [host] [agent_port]
# Environment:
#   MONITOR_PORT  monitor TCP port (default 60001)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OBSTACLES_DIR="${REPO_ROOT}/obstacles"
RCSSSMJ_DIR="${REPO_ROOT}/rcssservermj"

host="${1:-localhost}"
port="${2:-60000}"
monitor_port="${MONITOR_PORT:-60001}"

# Three obstacle poses (field coordinates, meters). Ball is offset from obstacle 1.
OBS1_X=2.5
OBS1_Y=2.0
OBS2_X=2.5
OBS2_Y=-2.0
OBS3_X=5.5
OBS3_Y=0.0

BALL_OFFSET_X=0.3
BALL_OFFSET_Y=0.0
BALL_X="$(awk "BEGIN {printf \"%.3f\", ${OBS1_X} + ${BALL_OFFSET_X}}")"
BALL_Y="$(awk "BEGIN {printf \"%.3f\", ${OBS1_Y} + ${BALL_OFFSET_Y}}")"

obstacle_pids=()

cleanup() {
  local pid
  for pid in "${obstacle_pids[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
}

trap cleanup EXIT INT TERM

echo "Starting Team players via start_passing.sh..."
"${SCRIPT_DIR}/start_passing.sh" "${host}" "${port}"

sleep 2

echo "Spawning 3 obstacle clients (Obstacles 1..3)..."
python3 "${OBSTACLES_DIR}/run_obstacles.py" --host "${host}" --port "${port}" --number 1 --x "${OBS1_X}" --y "${OBS1_Y}" &
obstacle_pids+=($!)
python3 "${OBSTACLES_DIR}/run_obstacles.py" --host "${host}" --port "${port}" --number 2 --x "${OBS2_X}" --y "${OBS2_Y}" &
obstacle_pids+=($!)
python3 "${OBSTACLES_DIR}/run_obstacles.py" --host "${host}" --port "${port}" --number 3 --x "${OBS3_X}" --y "${OBS3_Y}" &
obstacle_pids+=($!)

sleep 2

python3 "${RCSSSMJ_DIR}/monitor_client.py" -s "${host}" -p "${monitor_port}" \
  kickOff Left

# kickoff resets the ball to center, so place it after kickoff.
sleep 1
echo "Placing ball near obstacle 1 after kickoff: (${BALL_X}, ${BALL_Y})..."
python3 "${RCSSSMJ_DIR}/monitor_client.py" -s "${host}" -p "${monitor_port}" \
  ball "\"(pos ${BALL_X} ${BALL_Y} 0)\""

echo "Running. Obstacle PIDs: ${obstacle_pids[*]} (killed on script exit)."
echo "Stop with Ctrl+C; use pkill -f run_player.py if players outlive this script."

wait
