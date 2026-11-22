#!/usr/bin/env bash
# Run a command in a clean ROS 2 Humble environment, isolated for testing.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -v -i 'conda' | paste -sd ':' -)"
export PATH
while read -r name; do unset "$name"; done < <(compgen -e | grep '^CONDA' || true)
unset PYTHONHOME PYTHONPATH

export ROS_DOMAIN_ID=77
export IGN_PARTITION=gecko_test

source /opt/ros/humble/setup.bash
if [[ -f "$ROOT/ros2_ws/install/setup.bash" ]]; then
    source "$ROOT/ros2_ws/install/setup.bash"
fi

exec "$@"
