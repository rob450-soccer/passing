#!/bin/bash
export OMP_NUM_THREADS=1

host=${1:-localhost}
port=${2:-60000}

# Clean up stale shared-memory objects from previous aborted runs.
python3 - <<'PY'
from multiprocessing import shared_memory
for name in ("rob450_pose_1", "rob450_pose_2"):
    try:
        shm = shared_memory.SharedMemory(name=name, create=False)
    except FileNotFoundError:
        continue
    try:
        shm.close()
    finally:
        try:
            shm.unlink()
        except FileNotFoundError:
            pass
PY

pids=()

cleanup() {
  # First request graceful shutdown so Python can run cleanup handlers.
  for pid in "${pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null
    fi
  done

  # Give children a brief chance to exit gracefully.
  sleep 0.2

  # Force terminate remaining children.
  for pid in "${pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null
    fi
  done

  for pid in "${pids[@]}"; do
    wait "$pid" 2>/dev/null
  done
}

trap cleanup INT TERM

for i in {1..2}; do
  python3 run_player.py --host "$host" --port "$port" -n "$i" -t Team1 &
  pids+=("$!")
done

wait "${pids[@]}"