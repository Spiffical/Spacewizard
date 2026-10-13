#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=${SPACEWIZARD_REPO_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}
LAUNCHER="$REPO_DIR/spacewizard_live.sh"
RUNTIME_DIR="$REPO_DIR/maestro-osc-version/.runtime"
SCLANG_PID_FILE="$RUNTIME_DIR/sclang.pid"
BRIDGE_PID_FILE="$RUNTIME_DIR/bridge.pid"

MODE=${SPACEWIZARD_MODE:-smooth-sine}
SMOOTH_PRESET=${SPACEWIZARD_SMOOTH_PRESET:-pi-quiet-pentatonic}
SERIAL_PORT=${SPACEWIZARD_SERIAL_PORT:-/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0}
MAESTRO_HOST=${SPACEWIZARD_MAESTRO_HOST:-192.168.37.1}
MAESTRO_PORT=${SPACEWIZARD_MAESTRO_PORT:-7672}
POLL_SECONDS=${SPACEWIZARD_SERVICE_POLL_SECONDS:-2}

stop_live_rig() {
    "$LAUNCHER" stop >/dev/null 2>&1 || true
}

pid_is_running() {
    local pid_file="$1"
    [[ -s "$pid_file" ]] || return 1

    local pid
    pid=$(<"$pid_file")
    [[ -n "$pid" ]] || return 1
    kill -0 "$pid" 2>/dev/null
}

if [[ ! -x "$LAUNCHER" ]]; then
    printf 'Missing executable launcher: %s\n' "$LAUNCHER" >&2
    exit 1
fi

trap 'stop_live_rig; exit 0' INT TERM

stop_live_rig

"$LAUNCHER" start \
    --mode "$MODE" \
    --smooth-preset "$SMOOTH_PRESET" \
    --port "$SERIAL_PORT" \
    --maestro-host "$MAESTRO_HOST" \
    --maestro-port "$MAESTRO_PORT"

health_grace_deadline=$((SECONDS + 10))
while true; do
    if ! pid_is_running "$SCLANG_PID_FILE"; then
        printf 'SuperCollider process is not running; requesting systemd restart.\n' >&2
        exit 1
    fi

    if ! pid_is_running "$BRIDGE_PID_FILE"; then
        printf 'OSC bridge process is not running; requesting systemd restart.\n' >&2
        exit 1
    fi

    if (( SECONDS >= health_grace_deadline )) && ! "$REPO_DIR/.venv/bin/python" "$SCRIPT_DIR/check_spacewizard_health.py" \
        "$RUNTIME_DIR/synth.health" "$RUNTIME_DIR/bridge.health"; then
        printf 'Live rig stopped responding; requesting systemd restart.\n' >&2
        exit 1
    fi

    sleep "$POLL_SECONDS"
done
