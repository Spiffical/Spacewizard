#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd "$SCRIPT_DIR/.." && pwd)
SERVICE_NAME=${SPACEWIZARD_SERVICE_NAME:-spacewizard-production.service}
SOURCE_SERVICE="$REPO_DIR/systemd/user/$SERVICE_NAME"
USER_SYSTEMD_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
TARGET_SERVICE="$USER_SYSTEMD_DIR/$SERVICE_NAME"

if [[ ! -f "$SOURCE_SERVICE" ]]; then
    printf 'Missing service file: %s\n' "$SOURCE_SERVICE" >&2
    exit 1
fi

mkdir -p "$USER_SYSTEMD_DIR"
# The unit is a template: point it at wherever this checkout lives.
sed "s|@REPO_DIR@|$REPO_DIR|g" "$SOURCE_SERVICE" > "$TARGET_SERVICE"
chmod 644 "$TARGET_SERVICE"
chmod +x "$REPO_DIR/scripts/production_runner.sh" "$REPO_DIR/scripts/spacewizard_live.sh" "$REPO_DIR/scripts/set_mode.sh"

systemctl --user daemon-reload
systemctl --user enable "$SERVICE_NAME"

if command -v loginctl >/dev/null 2>&1; then
    if ! loginctl show-user "$USER" -p Linger 2>/dev/null | grep -q '=yes$'; then
        if command -v sudo >/dev/null 2>&1; then
            sudo loginctl enable-linger "$USER" || {
                printf 'Could not enable linger automatically. Run this once with sudo:\n' >&2
                printf '  sudo loginctl enable-linger %s\n' "$USER" >&2
            }
        else
            printf 'Install complete, but linger is not enabled. Run this once as root:\n' >&2
            printf '  loginctl enable-linger %s\n' "$USER" >&2
        fi
    fi
fi

cat <<EOF
Installed and enabled $SERVICE_NAME.

Start now:
  systemctl --user start $SERVICE_NAME

Check status/logs:
  systemctl --user status $SERVICE_NAME
  journalctl --user -u $SERVICE_NAME -f

Stop:
  systemctl --user stop $SERVICE_NAME
EOF
