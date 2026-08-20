#!/bin/bash
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cleanup() {
    kill "$iris_0_pid" "$iris_1_pid" 2>/dev/null
    wait "$iris_0_pid" "$iris_1_pid" 2>/dev/null
}
trap cleanup EXIT INT TERM

python3 "$script_dir/ego_swarm_goal.py" iris 0 9 5 0.7 &
iris_0_pid=$!

python3 "$script_dir/ego_swarm_goal.py" iris 1 9 1.2 0.7 &
iris_1_pid=$!

wait "$iris_0_pid" "$iris_1_pid"
