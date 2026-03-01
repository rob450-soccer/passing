#!/bin/bash

# Usage:
#   ./start_custom_players.sh <positions_file> [host] [port] [field]
#
# positions_file format:
#   - Lines starting with '#' or empty lines are ignored.
#   - The first non-empty, non-comment line is the TEAM NAME.
#   - Each subsequent non-empty, non-comment line should be:
#         x y
#     or  x y rot
#     where x, y, rot are floats (field coordinates, rot in degrees).
#
# Field can also be specified here or overridden via CLI:
#   - Field: 4th argument (default: hl_adult)

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

team=""
player_number=1
while IFS= read -r line || [[ -n "$line" ]]; do
  # Trim leading/trailing whitespace
  line="$(echo "$line" | xargs)"

  # Skip empty lines and comments
  if [[ -z "$line" ]] || [[ "$line" == \#* ]]; then
    continue
  fi

  # First meaningful line: team name
  if [[ -z "$team" ]]; then
    team="$line"
    echo "Using team name '$team' from file."
    continue
  fi

  # Split into tokens for positions
  read -r x y rot <<<"$line"

  if [[ -z "$x" || -z "$y" ]]; then
    echo "Skipping invalid line (need at least x y): '$line'"
    continue
  fi

  # Default rotation if not provided
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

  player_number=$((player_number + 1))
done < "$positions_file"

wait

