#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

IFACE="${G1_DASHBOARD_INTERFACE:-enP8p1s0}"
UNITREE_SERVICES="${G1_DASHBOARD_UNITREE_SERVICES:-auto}"
BASE_SENSING="${G1_DASHBOARD_BASE_SENSING:-auto}"
SYSTEM_HZ="${G1_DASHBOARD_SYSTEM_HZ:-5}"
PROCESS_ACTIONS="${G1_DASHBOARD_PROCESS_ACTIONS:-1}"

if [[ -n "${G1_DASHBOARD_MONITOR_PYTHON:-}" ]]; then
  MONITOR_PY="$G1_DASHBOARD_MONITOR_PYTHON"
elif [[ -x "$HOME/miniconda3/envs/g1_xr/bin/python" ]]; then
  MONITOR_PY="$HOME/miniconda3/envs/g1_xr/bin/python"
elif [[ -x "$HOME/miniforge3/envs/g1_xr/bin/python" ]]; then
  MONITOR_PY="$HOME/miniforge3/envs/g1_xr/bin/python"
elif [[ -x "$HOME/anaconda3/envs/g1_xr/bin/python" ]]; then
  MONITOR_PY="$HOME/anaconda3/envs/g1_xr/bin/python"
else
  MONITOR_PY="$(command -v python3)"
fi

BRIDGE_PY="${G1_DASHBOARD_BRIDGE_PYTHON:-$(command -v python3)}"
MONITOR_PID=""
BRIDGE_PID=""

cleanup() {
  local code=$?
  trap - EXIT INT TERM
  if [[ -n "${BRIDGE_PID:-}" ]] && kill -0 "$BRIDGE_PID" 2>/dev/null; then
    kill "$BRIDGE_PID" 2>/dev/null || true
    wait "$BRIDGE_PID" 2>/dev/null || true
  fi
  if [[ -n "${MONITOR_PID:-}" ]] && kill -0 "$MONITOR_PID" 2>/dev/null; then
    kill "$MONITOR_PID" 2>/dev/null || true
    wait "$MONITOR_PID" 2>/dev/null || true
  fi
  # Deliberately do NOT stop a controller launched by the dashboard here.
  # Browser/bridge/dashboard disconnects must not become a robot-control input.
  exit "$code"
}
trap cleanup EXIT INT TERM

BRIDGE_ARGS=()
if [[ "$PROCESS_ACTIONS" == "1" || "$PROCESS_ACTIONS" == "true" || "$PROCESS_ACTIONS" == "yes" ]]; then
  if [[ -z "${G1_DASHBOARD_ACTION_TOKEN:-}" ]]; then
    G1_DASHBOARD_ACTION_TOKEN="$("$BRIDGE_PY" - <<'PY'
import secrets
print(secrets.token_urlsafe(18))
PY
)"
  fi
  export G1_DASHBOARD_ACTION_TOKEN
  BRIDGE_ARGS+=(--enable-process-actions)
fi

echo "G1 dashboard Step 5.1 — controller process + XR action manager"
echo "  interface       : $IFACE"
echo "  monitor python  : $MONITOR_PY"
echo "  services        : $UNITREE_SERVICES (read-only)"
echo "  base sensing    : $BASE_SENSING (read-only)"
echo "  system rate     : ${SYSTEM_HZ} Hz"
if ((${#BRIDGE_ARGS[@]})); then
  echo "  process actions : ENABLED (start/stop + controller-validated XR actions)"
  echo "  management key  : $G1_DASHBOARD_ACTION_TOKEN"
  echo "                     the site will prompt for this key on first open"
else
  echo "  process actions : DISABLED"
fi
echo

"$MONITOR_PY" "$HERE/g1_dashboard_system_monitor.py" \
  --network-interface="$IFACE" \
  --unitree-services="$UNITREE_SERVICES" \
  --base-sensing="$BASE_SENSING" \
  --hz="$SYSTEM_HZ" &
MONITOR_PID=$!

# Give the monitor a moment to initialize DDS before the HTTP bridge starts.
sleep 0.25

echo "System monitor pid: $MONITOR_PID"
echo "Starting dashboard bridge. Ctrl+C stops dashboard processes only."
echo "A controller launched from the dashboard is intentionally left running."
echo

"$BRIDGE_PY" "$HERE/g1_dashboard_bridge.py" "${BRIDGE_ARGS[@]}" &
BRIDGE_PID=$!
wait "$BRIDGE_PID"
