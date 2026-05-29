#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
LAUNCHER="$SCRIPT_DIR/maestro-osc-version/start_spacewizard_live.sh"

usage() {
        cat <<EOF
Usage: $(basename "$0") [start|stop|restart|help] [launcher options]

Simple top-level wrapper for the SpaceWizard live rig.

Commands:
    start            Start the live rig (default)
    stop             Stop the live rig
    restart          Stop, then start the live rig again
    help             Show this help text

Examples:
    ./spacewizard_live.sh
    ./spacewizard_live.sh start --mode ambient-flow
    ./spacewizard_live.sh start --mode ambient-flow --ambient-variant forest
    ./spacewizard_live.sh stop
    ./spacewizard_live.sh restart --mode wow-voice

Any launcher options are forwarded to:
    ./maestro-osc-version/start_spacewizard_live.sh
EOF
}

if [[ ! -f "$LAUNCHER" ]]; then
    printf 'Missing launcher script: %s\n' "$LAUNCHER" >&2
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