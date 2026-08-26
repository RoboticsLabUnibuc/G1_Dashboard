#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNITREE_USER="${SUDO_USER:-${USER:-unitree}}"
if [[ "$UNITREE_USER" == "root" ]]; then
  UNITREE_USER="unitree"
fi
SOURCE_BINARY="/home/unitree/dfx_inspire_service/build/inspire_g1"
INSTALL_DIR="/usr/local/libexec/g1-dashboard"
HELPER_DST="$INSTALL_DIR/g1_dashboard_inspire_helper.py"
BINARY_DST="$INSTALL_DIR/inspire_g1"
SUDOERS_FILE="/etc/sudoers.d/g1-dashboard-inspire"

if [[ "${1:-}" == "--uninstall" ]]; then
  if [[ $EUID -ne 0 ]]; then exec sudo "$0" --uninstall; fi
  rm -f "$SUDOERS_FILE" "$HELPER_DST" "$BINARY_DST" "$INSTALL_DIR/inspire_g1.sha256"
  rmdir "$INSTALL_DIR" 2>/dev/null || true
  echo "Removed G1 dashboard Inspire helper and sudoers rule."
  exit 0
fi

if [[ $EUID -ne 0 ]]; then
  exec sudo "$0" "$@"
fi

if [[ "$UNITREE_USER" != "unitree" ]]; then
  echo "Refusing unexpected dashboard user: $UNITREE_USER (expected unitree)" >&2
  exit 1
fi
if [[ ! -x "$SOURCE_BINARY" ]]; then
  echo "Missing working Inspire binary: $SOURCE_BINARY" >&2
  exit 1
fi
if ! command -v visudo >/dev/null 2>&1; then
  echo "visudo is required to validate the narrow sudoers rule." >&2
  exit 1
fi

install -d -o root -g root -m 0755 "$INSTALL_DIR"
install -o root -g root -m 0755 "$HERE/privileged/g1_dashboard_inspire_helper.py" "$HELPER_DST"
install -o root -g root -m 0755 "$SOURCE_BINARY" "$BINARY_DST"
sha256sum "$BINARY_DST" | awk '{print $1}' > "$INSTALL_DIR/inspire_g1.sha256"
chown root:root "$INSTALL_DIR/inspire_g1.sha256"
chmod 0644 "$INSTALL_DIR/inspire_g1.sha256"

TMP="$(mktemp)"
trap 'rm -f "$TMP"' EXIT
cat > "$TMP" <<EOF
# G1 Dashboard: exact RH56DFX Inspire service lifecycle only.
# No shell, arbitrary executable, environment, PID, or argument passthrough.
unitree ALL=(root) NOPASSWD: $HELPER_DST start, $HELPER_DST stop
EOF
chmod 0440 "$TMP"
visudo -cf "$TMP" >/dev/null
install -o root -g root -m 0440 "$TMP" "$SUDOERS_FILE"
visudo -cf "$SUDOERS_FILE" >/dev/null

mkdir -p /run/g1-dashboard /var/log/g1-dashboard
chmod 0755 /run/g1-dashboard /var/log/g1-dashboard

echo "Installed dashboard Inspire lifecycle helper."
echo "  helper : $HELPER_DST"
echo "  binary : $BINARY_DST"
echo "  source : $SOURCE_BINARY"
echo "  sha256 : $(cat "$INSTALL_DIR/inspire_g1.sha256")"
echo
echo "Check status without sudo:"
echo "  $HELPER_DST status"
echo "Test passwordless start/stop permission without launching:"
echo "  sudo -n -l $HELPER_DST start"
