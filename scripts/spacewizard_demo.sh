#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
PYTHON_BIN=${PYTHON_BIN:-"$REPO_ROOT/.venv/bin/python"}
if [[ -z "${SCLANG_BIN:-}" ]]; then
    if [[ -x "/Applications/SuperCollider.app/Contents/MacOS/sclang" ]]; then
        SCLANG_BIN="/Applications/SuperCollider.app/Contents/MacOS/sclang"
    else
        SCLANG_BIN=$(command -v sclang || true)
    fi
fi
if [[ -z "${SESSION_SHELL:-}" ]]; then
    SESSION_SHELL=$(command -v bash || command -v sh || true)
fi
if [[ -z "${QT_QPA_PLATFORM:-}" ]] && [[ "$(uname -s)" != "Darwin" ]] && [[ -z "${DISPLAY:-}" ]] && [[ -z "${WAYLAND_DISPLAY:-}" ]]; then
    export QT_QPA_PLATFORM=offscreen
fi
if [[ -z "${SCLANG_WRAPPER:-}" ]] && [[ "$(uname -s)" != "Darwin" ]] && command -v pw-jack >/dev/null 2>&1; then
    SCLANG_WRAPPER=$(command -v pw-jack)
fi
SYNTH_FILE="$REPO_ROOT/synth/SpaceWizardSynth.scd"
BRIDGE_SCRIPT="$REPO_ROOT/scripts/run_bridge.py"
SOUND_PRESET_CONFIG_FILE=${SOUND_PRESET_CONFIG_FILE:-"$REPO_ROOT/synth/sound_presets.scd"}
RUNTIME_DIR="$REPO_ROOT/.runtime"
LOG_DIR="$RUNTIME_DIR/logs"
BOOT_FILE="$RUNTIME_DIR/spacewizard_demo_boot.scd"
CHOIR_SAMPLE_DIR="$REPO_ROOT/synth/samples/choir"
DEMO_LOG="$LOG_DIR/sclang_demo.log"
DEMO_PID_FILE="$RUNTIME_DIR/sclang_demo.pid"
DEMO_SCREEN_NAME="spacewizard-demo"
LIVE_SCLANG_PID_FILE="$RUNTIME_DIR/sclang.pid"
LIVE_BRIDGE_PID_FILE="$RUNTIME_DIR/bridge.pid"
DEFAULT_FALLBACK_OUTPUT_DEVICE="MacBook Pro Speakers"
INPUT_DEVICE=${SPACEWIZARD_INPUT_DEVICE:-"MacBook Pro Microphone"}

STOP_ONLY=0
OUTPUT_DEVICE_OVERRIDE=""
DEMO_MODE="ambient"

usage() {
    cat <<EOF
Usage: $(basename "$0") [start|restart|stop] [options]

Starts SpaceWizard demo mode with SuperCollider only.
This launches the ambient demo scene and opens the manual GUI, without the sensor,
receiver, or Python OSC bridge.

Options:
    --mode MODE          Demo mode: ambient, ambient-lab, or choir-textures
  --output-device NAME  Force a specific SuperCollider output device
  --stop                Stop the managed demo process
  -h, --help            Show this help text
EOF
}

log() {
    printf '[spacewizard-demo] %s\n' "$*"
}

require_file() {
    local path="$1"
    local description="$2"

    if [[ ! -e "$path" ]]; then
        printf 'Missing %s: %s\n' "$description" "$path" >&2
        exit 1
    fi
}

stop_pid_file() {
    local pid_file="$1"
    local label="$2"

    if [[ ! -f "$pid_file" ]]; then
        return
    fi

    local pid
    pid=$(<"$pid_file")
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
        log "Stopping $label (pid $pid)"
        kill "$pid" 2>/dev/null || true
    fi

    rm -f "$pid_file"
}

kill_matching() {
    local pattern="$1"
    local label="$2"

    if pgrep -f "$pattern" >/dev/null 2>&1; then
        log "Stopping existing $label"
        pkill -f "$pattern" 2>/dev/null || true
    fi
}

stop_screen_session() {
    if ! command -v screen >/dev/null 2>&1; then
        return
    fi

    if screen -ls | grep -q "[.]$DEMO_SCREEN_NAME[[:space:]]"; then
        log "Stopping demo screen session"
        screen -S "$DEMO_SCREEN_NAME" -X quit >/dev/null 2>&1 || true
    fi
}

wait_for_exit() {
    local pattern="$1"
    local timeout_seconds="$2"

    "$PYTHON_BIN" - "$pattern" "$timeout_seconds" <<'PY'
import subprocess
import sys
import time

pattern = sys.argv[1]
timeout_seconds = float(sys.argv[2])
deadline = time.monotonic() + timeout_seconds

while time.monotonic() < deadline:
    result = subprocess.run(
        ["pgrep", "-f", pattern],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    if result.returncode != 0:
        raise SystemExit(0)
    time.sleep(0.2)

raise SystemExit(1)
PY
}

stop_existing_processes() {
    mkdir -p "$LOG_DIR"

    stop_screen_session
    stop_pid_file "$DEMO_PID_FILE" "demo SuperCollider"
    stop_pid_file "$LIVE_SCLANG_PID_FILE" "live SuperCollider"
    stop_pid_file "$LIVE_BRIDGE_PID_FILE" "live bridge"

    kill_matching "$BRIDGE_SCRIPT" "bridge"
    kill_matching "$SCLANG_BIN" "SuperCollider language"
    kill_matching "(^|/)scsynth( |$)" "SuperCollider audio server"

    wait_for_exit "$BRIDGE_SCRIPT|$SCLANG_BIN|(^|/)scsynth( |$)" 4 || true
}

detect_output_device() {
    if [[ -n "$OUTPUT_DEVICE_OVERRIDE" ]]; then
        printf '%s\n' "$OUTPUT_DEVICE_OVERRIDE"
        return 0
    fi

    local audio_report
    audio_report=$(system_profiler SPAudioDataType 2>/dev/null || true)

    local detected_output
    detected_output=$(AUDIO_REPORT="$audio_report" "$PYTHON_BIN" - <<'PY'
import re
import os
import sys

audio_report = os.environ.get("AUDIO_REPORT", "")
device_name = None
device_indent = None

for raw_line in audio_report.splitlines():
    line = raw_line.rstrip("\n")
    stripped = line.strip()
    if not stripped:
        continue

    indent = len(line) - len(line.lstrip(" "))
    device_match = re.match(r"^(\s+)([^:][^\n]*):\s*$", line)
    if device_match and indent >= 8:
        device_name = device_match.group(2).strip()
        device_indent = indent
        continue

    if device_name is None:
        continue

    if indent <= (device_indent or 0):
        device_name = None
        device_indent = None
        continue

    if stripped == "Default Output Device: Yes" or stripped == "Default System Output Device: Yes":
        print(device_name)
        raise SystemExit(0)

raise SystemExit(1)
PY
    ) || true

    if [[ -n "$detected_output" ]]; then
        printf '%s\n' "$detected_output"
    else
        printf '%s\n' "$DEFAULT_FALLBACK_OUTPUT_DEVICE"
    fi
}

write_boot_file() {
    local output_device="$1"
    local requested_live_mode='\ambientFlow'
    local requested_smooth_preset='nil'
    local ready_condition='(~swAmbientScene.notNil) and: { ~swAmbientGUI.notNil }'
    local launch_body=$'        ~swAmbientScene.();\n        ~swAmbientGUI.();\n        "SpaceWizard demo GUI ready.".postln;'

    if [[ "$DEMO_MODE" == "choir-textures" ]]; then
        requested_live_mode='\smoothSine'
        requested_smooth_preset='\quietPentatonic'
        ready_condition='~swChoirTextureGUI.notNil'
        launch_body=$'        ~swChoirTextureGUI.();\n        ~swDemoGuiWatch = AppClock.sched(1.0, {\n            if(~swChoirTextureWindow.isNil) {\n                ~swChoirTextureGUI.();\n                "SpaceWizard choir texture GUI reopened.".postln;\n            };\n            1.0;\n        });\n        "SpaceWizard choir texture GUI ready.".postln;'
    elif [[ "$DEMO_MODE" == "ambient-lab" ]]; then
        requested_live_mode='\smoothSine'
        ready_condition='(~swAmbientLabScene.notNil) and: { ~swAmbientLabGUI.notNil }'
        launch_body=$'        ~swAmbientLabScene.();\n        ~swAmbientLabGUI.();\n        "SpaceWizard ambient lab GUI ready.".postln;'
    fi

    cat > "$BOOT_FILE" <<EOF
~swRequestedLiveMode = $requested_live_mode;
~swRequestedSmoothPreset = $requested_smooth_preset;
~swChoirSampleDir = "$CHOIR_SAMPLE_DIR";
~swSoundPresetConfigFile = "$SOUND_PRESET_CONFIG_FILE";
s.options.numInputBusChannels_(0);
s.options.inDevice_("$INPUT_DEVICE");
s.options.outDevice_("$output_device");
thisProcess.interpreter.executeFile("$SYNTH_FILE");
AppClock.sched(0.2, {
    if($ready_condition) {
$launch_body
        nil;
    } {
        0.2;
    };
});
~swDemoKeepAliveRoutine = Routine({
    loop { 1.0.wait; };
}).play(SystemClock);
EOF
}

wait_for_log_text() {
    local log_file="$1"
    local expected_text="$2"
    local timeout_seconds="$3"
    local pid="$4"

    "$PYTHON_BIN" - "$log_file" "$expected_text" "$timeout_seconds" "$pid" <<'PY'
from pathlib import Path
import os
import sys
import time

log_path = Path(sys.argv[1])
expected_text = sys.argv[2]
timeout_seconds = float(sys.argv[3])
pid = int(sys.argv[4])
deadline = time.monotonic() + timeout_seconds

while time.monotonic() < deadline:
    if log_path.exists():
        if expected_text in log_path.read_text(errors="ignore"):
            raise SystemExit(0)

    try:
        os.kill(pid, 0)
    except OSError:
        raise SystemExit(2)

    time.sleep(0.2)

raise SystemExit(1)
PY
}

start_demo() {
    local -a sclang_args
    local ready_text="SpaceWizard demo GUI ready."

    if [[ "$DEMO_MODE" == "choir-textures" ]]; then
        ready_text="SpaceWizard choir texture GUI ready."
    elif [[ "$DEMO_MODE" == "ambient-lab" ]]; then
        ready_text="SpaceWizard ambient lab GUI ready."
    fi

    : > "$DEMO_LOG"
    rm -f "$DEMO_PID_FILE"
    log "Starting SuperCollider demo ($DEMO_MODE)"
    if [[ -n "${SCLANG_WRAPPER:-}" ]]; then
        sclang_args=("$SCLANG_WRAPPER" "$SCLANG_BIN" "-D" "-r" "$BOOT_FILE")
    else
        sclang_args=("$SCLANG_BIN" "-D" "-r" "$BOOT_FILE")
    fi

    if command -v screen >/dev/null 2>&1; then
        screen -dmS "$DEMO_SCREEN_NAME" "$SESSION_SHELL" -lc 'pid_file=$1; log_file=$2; shift 2; printf "%s\n" "$$" > "$pid_file"; exec "$@" > "$log_file" 2>&1' _ "$DEMO_PID_FILE" "$DEMO_LOG" "${sclang_args[@]}"
    else
        nohup "$SESSION_SHELL" -lc 'pid_file=$1; log_file=$2; shift 2; printf "%s\n" "$$" > "$pid_file"; exec "$@" > "$log_file" 2>&1' _ "$DEMO_PID_FILE" "$DEMO_LOG" "${sclang_args[@]}" >/dev/null 2>&1 &
    fi

    for _ in {1..50}; do
        [[ -s "$DEMO_PID_FILE" ]] && break
        sleep 0.1
    done

    if [[ ! -s "$DEMO_PID_FILE" ]]; then
        printf 'Demo mode failed to start SuperCollider session.\n' >&2
        exit 1
    fi

    local pid
    pid=$(<"$DEMO_PID_FILE")

    if ! wait_for_log_text "$DEMO_LOG" "$ready_text" 30 "$pid"; then
        printf 'Demo mode failed to boot cleanly. Recent log output:\n' >&2
        tail -n 60 "$DEMO_LOG" >&2 || true
        exit 1
    fi

    log "Demo GUI ready (pid $pid)"
}

# Optional leading command: start (default), restart, or stop.
case "${1:-}" in
    start|on|restart)
        shift
        ;;
    stop|off)
        STOP_ONLY=1
        shift
        ;;
    help)
        usage
        exit 0
        ;;
esac

while [[ $# -gt 0 ]]; do
    case "$1" in
        --mode)
            [[ $# -ge 2 ]] || { usage >&2; exit 1; }
            DEMO_MODE="$2"
            shift 2
            ;;
        --output-device)
            [[ $# -ge 2 ]] || { usage >&2; exit 1; }
            OUTPUT_DEVICE_OVERRIDE="$2"
            shift 2
            ;;
        --stop)
            STOP_ONLY=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            printf 'Unknown option: %s\n\n' "$1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

case "$DEMO_MODE" in
    ambient|ambient-lab|choir-textures)
        ;;
    *)
        printf 'Unknown demo mode: %s\n\n' "$DEMO_MODE" >&2
        usage >&2
        exit 1
        ;;
esac

require_file "$PYTHON_BIN" "Python interpreter"
require_file "$SCLANG_BIN" "SuperCollider language executable"
require_file "$SESSION_SHELL" "session shell"
require_file "$SYNTH_FILE" "SpaceWizard synth patch"

stop_existing_processes

if [[ "$STOP_ONLY" -eq 1 ]]; then
    log "Stopped SpaceWizard demo mode"
    exit 0
fi

mkdir -p "$LOG_DIR"

OUTPUT_DEVICE=$(detect_output_device)
log "Using audio output $OUTPUT_DEVICE"
log "Using audio input $INPUT_DEVICE"
write_boot_file "$OUTPUT_DEVICE"
start_demo

cat <<EOF

SpaceWizard demo mode started.
Mode: $DEMO_MODE
Audio output: $OUTPUT_DEVICE
SuperCollider log: $DEMO_LOG

To stop demo mode:
  $(basename "$0") --stop
EOF
