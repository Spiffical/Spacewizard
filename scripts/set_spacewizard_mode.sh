#!/usr/bin/env bash
# Switch the Pi production service to another sound mode and restart it.
# The choice is stored in a systemd drop-in, so it survives reboots.
#   ./scripts/set_spacewizard_mode.sh day-o
#   ./scripts/set_spacewizard_mode.sh smooth-sine

set -euo pipefail

MODE=${1:-}
case "$MODE" in
    smooth-sine|day-o|ambient-flow|auduino|reese-bass|wow-voice) ;;
    *)
        printf 'Usage: %s smooth-sine|day-o|ambient-flow|auduino|reese-bass|wow-voice\n' "$(basename "$0")" >&2
        exit 1
        ;;
esac

SERVICE=${SPACEWIZARD_SERVICE_NAME:-spacewizard-production.service}
DROPIN_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user/$SERVICE.d"

mkdir -p "$DROPIN_DIR"
printf '[Service]\nEnvironment=SPACEWIZARD_MODE=%s\n' "$MODE" > "$DROPIN_DIR/mode.conf"
systemctl --user daemon-reload
systemctl --user restart "$SERVICE"
printf 'SpaceWizard mode set to %s; %s restarted.\n' "$MODE" "$SERVICE"
