#!/bin/bash

set -e

positions_file="$1"
host="${2:-localhost}"
port="${3:-60000}"
field="${4:-hl_adult}"

if [[ -z "$positions_file" ]]; then
  echo "Error: positions_file argument is required."
  echo "Usage: $0 <positions_file> [host] [port] [field]"
  exit 1
fi

if [[ ! -f "$positions_file" ]]; then
  echo "Error: positions file '$positions_file' not found."
  exit 1
fi

export OMP_NUM_THREADS=1

# Track child PIDs and kill them on exit
pids=()

cleanup() {
  echo "Stopping all players..."
  for pid in "${pids[@]}"; do
    kill "$pid" 2>/dev/null
  done
  wait
  echo "All players stopped."
  exit 0
}

trap cleanup SIGINT SIGTERM

team=""
player_number=1
while IFS= read -r line || [[ -n "$line" ]]; do
  line="$(echo "$line" | xargs)"

  if [[ -z "$line" ]] || [[ "$line" == \#* ]]; then
    continue
  fi

  if [[ -z "$team" ]]; then
    team="$line"
    echo "Using team name '$team' from file."
    continue
  fi

  read -r x y rot <<<"$line"

  if [[ -z "$x" || -z "$y" ]]; then
    echo "Skipping invalid line (need at least x y): '$line'"
    continue
  fi

  if [[ -z "$rot" ]]; then
    rot=0.0
  fi

  echo "Starting player $player_number at ($x, $y, $rot) on team '$team'"
  python3 run_player.py \
    --host "$host" \
    --port "$port" \
    -n "$player_number" \
    -t "$team" \
    -f "$field" \
    --spawn-x "$x" \
    --spawn-y "$y" \
    --spawn-rot "$rot" &

  pids+=($!)  # capture PID of the last backgrounded process
  player_number=$((player_number + 1))
  sleep 1
done < "$positions_file"

wait