#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

IFACE="${G1_DASHBOARD_INTERFACE:-enP8p1s0}"
UNITREE_SERVICES="${G1_DASHBOARD_UNITREE_SERVICES:-auto}"
BASE_SENSING="${G1_DASHBOARD_BASE_SENSING:-auto}"
SYSTEM_HZ="${G1_DASHBOARD_SYSTEM_HZ:-5}"
PROCESS_ACTIONS="${G1_DASHBOARD_PROCESS_ACTIONS:-1}"
SERVICE_ACTIONS="${G1_DASHBOARD_SERVICE_ACTIONS:-$PROCESS_ACTIONS}"

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
SERVICE_PID=""

cleanup() {
  local code=$?
  trap - EXIT INT TERM
  if [[ -n "${BRIDGE_PID:-}" ]] && kill -0 "$BRIDGE_PID" 2>/dev/null; then
    kill "$BRIDGE_PID" 2>/dev/null || true
    wait "$BRIDGE_PID" 2>/dev/null || true
  fi
  if [[ -n "${SERVICE_PID:-}" ]] && kill -0 "$SERVICE_PID" 2>/dev/null; then
    kill "$SERVICE_PID" 2>/dev/null || true
    wait "$SERVICE_PID" 2>/dev/null || true
  fi
  if [[ -n "${MONITOR_PID:-}" ]] && kill -0 "$MONITOR_PID" 2>/dev/null; then
    kill "$MONITOR_PID" 2>/dev/null || true
    wait "$MONITOR_PID" 2>/dev/null || true
  fi
  if [[ -n "${G1_DASHBOARD_SERVICE_SOCKET:-}" ]]; then
    rm -f "$G1_DASHBOARD_SERVICE_SOCKET" 2>/dev/null || true
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

SERVICE_ENABLED=0
if [[ "$SERVICE_ACTIONS" == "1" || "$SERVICE_ACTIONS" == "true" || "$SERVICE_ACTIONS" == "yes" ]]; then
  if ((${#BRIDGE_ARGS[@]} == 0)); then
    echo "ERROR: service actions require G1_DASHBOARD_PROCESS_ACTIONS=1 for management-key authentication" >&2
    exit 2
  fi
  if [[ -z "${G1_DASHBOARD_SERVICE_TOKEN:-}" ]]; then
    G1_DASHBOARD_SERVICE_TOKEN="$("$BRIDGE_PY" - <<'PY'
import secrets
print(secrets.token_urlsafe(24))
PY
)"
  fi
  export G1_DASHBOARD_SERVICE_TOKEN
  export G1_DASHBOARD_SERVICE_SOCKET="${G1_DASHBOARD_SERVICE_SOCKET:-${XDG_RUNTIME_DIR:-/tmp}/g1_dashboard_service_actions_${UID}.sock}"
  export G1_DASHBOARD_SERVICE_POLICY="${G1_DASHBOARD_SERVICE_POLICY:-$HERE/service_policy.json}"
  BRIDGE_ARGS+=(--enable-service-actions)
  SERVICE_ENABLED=1
fi

echo "G1 dashboard Step 5.2 — verified Unitree service controls"
echo "  interface       : $IFACE"
echo "  monitor python  : $MONITOR_PY"
echo "  service monitor : $UNITREE_SERVICES (read-only inventory)"
echo "  base sensing    : $BASE_SENSING (read-only)"
echo "  system rate     : ${SYSTEM_HZ} Hz"
if ((${#BRIDGE_ARGS[@]})); then
  echo "  process actions : ENABLED (listener/Inspire/camera + controller-validated XR actions)"
  if [[ "$SERVICE_ENABLED" == "1" ]]; then
    echo "  service actions : ENABLED (explicit allowlist + ServiceList post-verification)"
    echo "  service policy  : $G1_DASHBOARD_SERVICE_POLICY"
    if [[ -n "${G1_DASHBOARD_SERVICE_ALLOWLIST:-}" ]]; then
      echo "  runtime allow   : $G1_DASHBOARD_SERVICE_ALLOWLIST"
    fi
  else
    echo "  service actions : DISABLED"
  fi
  echo "  management key  : $G1_DASHBOARD_ACTION_TOKEN"
  echo "                     the site will prompt for this key on first open"
  if [[ -x "/usr/local/libexec/g1-dashboard/g1_dashboard_inspire_helper.py" ]]; then
    echo "  Inspire helper  : installed"
  else
    echo "  Inspire helper  : NOT INSTALLED — run: sudo ./install_inspire_helper.sh"
  fi
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

# Give the monitor time to construct its RobotState RPC entity first. The
# service action worker is a separate process/participant and initializes only
# after the read-only monitor is already up.
sleep 0.45

if [[ "$SERVICE_ENABLED" == "1" ]]; then
  STATE_DIR="${XDG_STATE_HOME:-$HOME/.local/state}/g1_dashboard"
  mkdir -p "$STATE_DIR"
  SERVICE_LOG="$STATE_DIR/service_actions.log"
  rm -f "$G1_DASHBOARD_SERVICE_SOCKET" 2>/dev/null || true
  "$MONITOR_PY" "$HERE/g1_dashboard_service_worker.py" \
    --network-interface="$IFACE" \
    --socket="$G1_DASHBOARD_SERVICE_SOCKET" \
    --policy="$G1_DASHBOARD_SERVICE_POLICY" \
    >"$SERVICE_LOG" 2>&1 &
  SERVICE_PID=$!
  for _ in $(seq 1 20); do
    [[ -S "$G1_DASHBOARD_SERVICE_SOCKET" ]] && break
    kill -0 "$SERVICE_PID" 2>/dev/null || break
    sleep 0.10
  done
  if [[ -S "$G1_DASHBOARD_SERVICE_SOCKET" ]]; then
    echo "Service action worker pid: $SERVICE_PID"
    echo "Service action log       : $SERVICE_LOG"
  else
    echo "WARNING: service action worker did not become ready; controls will show offline." >&2
    tail -20 "$SERVICE_LOG" 2>/dev/null || true
  fi
fi

echo "System monitor pid: $MONITOR_PID"
echo "Starting dashboard bridge. Ctrl+C stops dashboard processes only."
echo "Managed controller/camera processes are intentionally left running if the dashboard itself exits."
echo "A helper-managed Inspire service follows the controller lifecycle, not the browser lifecycle."
echo "The Unitree ServiceSwitch worker exists only while the dashboard stack is running and accepts explicit allowlisted requests only."
echo

"$BRIDGE_PY" "$HERE/g1_dashboard_bridge.py" "${BRIDGE_ARGS[@]}" &
BRIDGE_PID=$!
wait "$BRIDGE_PID"
