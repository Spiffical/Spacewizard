#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
LAUNCHER="$SCRIPT_DIR/maestro-osc-version/start_spacewizard_demo.sh"

usage() {
    cat <<EOF
Usage: $(basename "$0") [start|stop|restart|help] [demo options]

Simple top-level wrapper for SpaceWizard demo mode.
This boots a SuperCollider-only demo or lab GUI, with no candle.

Commands:
  start            Start demo mode (default)
  stop             Stop demo mode
  restart          Stop, then start demo mode again
  help             Show this help text

Examples:
  ./spacewizard_demo.sh
  ./spacewizard_demo.sh restart --mode ambient-lab
  ./spacewizard_demo.sh stop
  ./spacewizard_demo.sh restart --output-device "MacBook Pro Speakers"

Any demo options are forwarded to:
  ./maestro-osc-version/start_spacewizard_demo.sh
EOF
}

if [[ ! -f "$LAUNCHER" ]]; then
    printf 'Missing demo launcher: %s\n' "$LAUNCHER" >&2
    exit 1
fi

command="${1:-start}"

if [[ $# -gt 0 ]]; then
    shift
fi

case "$command" in
    start|on)
        exec "$LAUNCHER" "$@"
        ;;
    stop|off)
        exec "$LAUNCHER" --stop
        ;;
    restart)
        "$LAUNCHER" --stop
        exec "$LAUNCHER" "$@"
        ;;
    help|-h|--help)
        usage
        exit 0
        ;;
    --*)
        exec "$LAUNCHER" "$command" "$@"
        ;;
    *)
        printf 'Unknown command: %s\n\n' "$command" >&2
        usage >&2
        exit 1
        ;;
esac
