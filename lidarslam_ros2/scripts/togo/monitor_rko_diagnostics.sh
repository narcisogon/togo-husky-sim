#!/usr/bin/env bash
set -euo pipefail

DURATION_SEC="${1:-10}"

if ! [[ "$DURATION_SEC" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "Usage: $0 [duration_seconds]" >&2
  exit 2
fi

run_for_duration() {
  local label="$1"
  shift
  echo
  echo "===== ${label} (${DURATION_SEC}s) ====="
  timeout "${DURATION_SEC}" "$@" || {
    local code=$?
    if [[ "$code" -ne 124 ]]; then
      echo "Command failed with exit code ${code}: $*" >&2
      return "$code"
    fi
  }
}

run_for_duration "runtime diagnostics" \
  ros2 topic echo /rko_lio/runtime_diagnostics

run_for_duration "registration diagnostics" \
  ros2 topic echo /rko_lio/registration_diagnostics

run_for_duration "raw odometry hz" \
  ros2 topic hz /rko_lio/odometry

run_for_duration "stable odometry hz" \
  ros2 topic hz /rko_lio/odometry_stable
