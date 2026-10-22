#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=${SPACEWIZARD_REPO_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}
LAUNCHER="$REPO_DIR/scripts/spacewizard_live.sh"
RUNTIME_DIR="$REPO_DIR/.runtime"
SCLANG_PID_FILE="$RUNTIME_DIR/sclang.pid"
BRIDGE_PID_FILE="$RUNTIME_DIR/bridge.pid"
HISTORY_LOG="$RUNTIME_DIR/history/rig.log"

MODE=${SPACEWIZARD_MODE:-smooth-sine}
SMOOTH_PRESET=${SPACEWIZARD_SMOOTH_PRESET:-pi-quiet-pentatonic}
SERIAL_PORT=${SPACEWIZARD_SERIAL_PORT:-/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0}
MAESTRO_HOST=${SPACEWIZARD_MAESTRO_HOST:-192.168.37.1}
MAESTRO_PORT=${SPACEWIZARD_MAESTRO_PORT:-7672}
POLL_SECONDS=${SPACEWIZARD_SERVICE_POLL_SECONDS:-2}

# Same line format as the bridge's history (bridge/history.py), so restarts sit beside its events.
log_history() {
    mkdir -p "${HISTORY_LOG%/*}"
    printf '%s.000 up %s  runner: %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" \
        "$(awk '{printf "%.1f", $1}' /proc/uptime 2>/dev/null || echo '?')" "$1" >> "$HISTORY_LOG" 2>/dev/null || true
}

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

trap 'log_history "stopping the rig"; stop_live_rig; exit 0' INT TERM

stop_live_rig

log_history "starting the rig (mode $MODE)"
if ! "$LAUNCHER" start \
    --mode "$MODE" \
    --smooth-preset "$SMOOTH_PRESET" \
    --port "$SERIAL_PORT" \
    --maestro-host "$MAESTRO_HOST" \
    --maestro-port "$MAESTRO_PORT"; then
    log_history "the launcher failed to start the rig; systemd will retry"
    exit 1
fi

health_grace_deadline=$((SECONDS + 10))
while true; do
    if ! pid_is_running "$SCLANG_PID_FILE"; then
        printf 'SuperCollider process is not running; requesting systemd restart.\n' >&2
        log_history "SuperCollider process is not running; restarting the rig"
        exit 1
    fi

    if ! pid_is_running "$BRIDGE_PID_FILE"; then
        printf 'OSC bridge process is not running; requesting systemd restart.\n' >&2
        log_history "bridge process is not running; restarting the rig"
        exit 1
    fi

    if (( SECONDS >= health_grace_deadline )) && ! "$REPO_DIR/.venv/bin/python" "$SCRIPT_DIR/check_health.py" \
        "$RUNTIME_DIR/synth.health" "$RUNTIME_DIR/bridge.health"; then
        printf 'Live rig stopped responding; requesting systemd restart.\n' >&2
        log_history "health check failed (no heartbeat); restarting the rig"
        exit 1
    fi

    sleep "$POLL_SECONDS"
done
