#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
PYTHON_BIN=${PYTHON_BIN:-"$REPO_ROOT/.venv/bin/python"}
SCLANG_BIN=${SCLANG_BIN:-"/Applications/SuperCollider.app/Contents/MacOS/sclang"}
SYNTH_FILE="$SCRIPT_DIR/synth/SpaceWizardSynth.scd"
BRIDGE_SCRIPT="$SCRIPT_DIR/run_osc_bridge.py"
SOUND_PRESET_CONFIG_FILE=${SOUND_PRESET_CONFIG_FILE:-"$SCRIPT_DIR/sound_presets.scd"}
RUNTIME_DIR="$SCRIPT_DIR/.runtime"
LOG_DIR="$RUNTIME_DIR/logs"
BOOT_FILE="$RUNTIME_DIR/spacewizard_boot.scd"
CHOIR_SAMPLE_DIR="$SCRIPT_DIR/samples/choir"
CHOIR_ASSET_SAMPLE_DIR="$REPO_ROOT/assets"
SCLANG_LOG="$LOG_DIR/sclang.log"
BRIDGE_LOG="$LOG_DIR/bridge.log"
SCLANG_PID_FILE="$RUNTIME_DIR/sclang.pid"
BRIDGE_PID_FILE="$RUNTIME_DIR/bridge.pid"
DEFAULT_FALLBACK_OUTPUT_DEVICE="MacBook Pro Speakers"

VERBOSE_TEST=0
PORT_OVERRIDE=""
STOP_ONLY=0
SYNTH_MODE="auduino"
SMOOTH_PRESET="standard"
AMBIENT_VARIANT="flow"
OUTPUT_DEVICE_OVERRIDE=""
STATUS_GUI=0
SMOOTH_MIX_GUI=0

usage() {
        cat <<EOF
Usage: $(basename "$0") [options]

Starts the local SpaceWizard live rig by restarting SuperCollider and the
Python OSC bridge, then auto-detecting the receiver serial port.

Options:
    --port PORT            Use a specific receiver serial port
    --mode MODE            Live synth mode: auduino, ambient-flow, smooth-sine, reese-bass, or wow-voice
    --smooth-preset NAME   Smooth-sine preset: standard or quiet-pentatonic
    --ambient-variant NAME Ambient variant anchor for ambient-flow mode: flow, chordal, drone, or forest
                                                 Yaw then blends smoothly around the ambient variant ring from that starting point.
    --output-device NAME   Force a specific SuperCollider output device
    --status-gui           Open the live status GUI with current inputs and synth parameters
    --smooth-mix-gui       Open the smooth-sine mix GUI for pentatonic/ambient balance
    --verbose-test         Pass --verbose-test to the bridge
    --stop                 Stop the managed SuperCollider and bridge processes only
    -h, --help             Show this help text
EOF
}

log() {
    printf '[spacewizard-live] %s\n' "$*"
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

    stop_pid_file "$BRIDGE_PID_FILE" "bridge"
    stop_pid_file "$SCLANG_PID_FILE" "SuperCollider"

    kill_matching "$BRIDGE_SCRIPT" "bridge"
    kill_matching "$SCLANG_BIN" "SuperCollider language"
    kill_matching "(^|/)scsynth( |$)" "SuperCollider audio server"

    wait_for_exit "$BRIDGE_SCRIPT|$SCLANG_BIN|(^|/)scsynth( |$)" 4 || true
}

detect_receiver_port() {
    if [[ -n "$PORT_OVERRIDE" ]]; then
        printf '%s\n' "$PORT_OVERRIDE"
        return 0
    fi

    "$PYTHON_BIN" - <<'PY'
from pathlib import Path
import glob
import sys
import time

try:
    import serial
except Exception as exc:
    print(f"pyserial import failed: {exc}", file=sys.stderr)
    raise SystemExit(2)

markers = (
    "SpaceWizard OSC Receiver Starting",
    "Serial format:",
    "Radio initialized successfully",
    "AX=",
)

candidates = sorted(
    set(glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.usbmodem*"))
)

for port in candidates:
    try:
        connection = serial.Serial(port, 230400, timeout=0.25)
    except Exception:
        continue

    try:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            raw_line = connection.readline()
            if not raw_line:
                continue

            line = raw_line.decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            if any(marker in line for marker in markers):
                print(port)
                raise SystemExit(0)
    finally:
        try:
            connection.close()
        except Exception:
            pass

raise SystemExit(1)
PY
}

free_serial_port() {
    local port="$1"

    if ! command -v lsof >/dev/null 2>&1; then
        return
    fi

    local holder_pids
    holder_pids=$(lsof -t "$port" 2>/dev/null | sort -u || true)
    if [[ -z "$holder_pids" ]]; then
        return
    fi

    while IFS= read -r pid; do
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            log "Freeing serial port from pid $pid"
            kill "$pid" 2>/dev/null || true
        fi
    done <<< "$holder_pids"
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
    local sc_mode="\\auduino"
    local output_device="$1"
    local ambient_variant_symbol="nil"
    local ambient_yaw_blend="false"
    local ambient_yaw_offset="0.0"
    local smooth_preset_symbol="nil"
    local status_gui="false"
    local smooth_mix_gui="false"

    if [[ "$SYNTH_MODE" == "ambient-flow" ]]; then
        sc_mode="\\ambientFlow"
        ambient_yaw_blend="true"
    elif [[ "$SYNTH_MODE" == "smooth-sine" ]]; then
        sc_mode="\\smoothSine"
        case "$SMOOTH_PRESET" in
            standard)
                smooth_preset_symbol="\\standard"
                ;;
            quiet-pentatonic)
                smooth_preset_symbol="\\quietPentatonic"
                ;;
        esac
    elif [[ "$SYNTH_MODE" == "reese-bass" ]]; then
        sc_mode="\\reeseBass"
    elif [[ "$SYNTH_MODE" == "wow-voice" ]]; then
        sc_mode="\\wowVoice"
    fi

    if [[ "$SYNTH_MODE" == "ambient-flow" ]]; then
        case "$AMBIENT_VARIANT" in
            flow)
                ambient_variant_symbol="\\flow"
                ambient_yaw_offset="0.0"
                ;;
            chordal)
                ambient_variant_symbol="\\chordal"
                ambient_yaw_offset="0.25"
                ;;
            drone)
                ambient_variant_symbol="\\drone"
                ambient_yaw_offset="0.5"
                ;;
            forest)
                ambient_variant_symbol="\\forest"
                ambient_yaw_offset="0.75"
                ;;
        esac
    fi

    if [[ "$STATUS_GUI" -eq 1 ]]; then
        status_gui="true"
    fi
    if [[ "$SMOOTH_MIX_GUI" -eq 1 ]]; then
        smooth_mix_gui="true"
    fi

    cat > "$BOOT_FILE" <<EOF
~swRequestedLiveMode = $sc_mode;
~swRequestedAmbientVariant = $ambient_variant_symbol;
~swRequestedAmbientYawBlend = $ambient_yaw_blend;
~swRequestedAmbientYawOffset = $ambient_yaw_offset;
~swRequestedSmoothPreset = $smooth_preset_symbol;
~swRequestedStatusGUI = $status_gui;
~swRequestedSmoothMixGUI = $smooth_mix_gui;
~swChoirSampleDir = "$CHOIR_SAMPLE_DIR";
~swChoirAssetSampleDir = "$CHOIR_ASSET_SAMPLE_DIR";
~swSoundPresetConfigFile = "$SOUND_PRESET_CONFIG_FILE";
s.options.numInputBusChannels_(0);
s.options.memSize_(262144);
s.options.outDevice_("$output_device");
thisProcess.interpreter.executeFile("$SYNTH_FILE");
s.waitForBoot {
    AppClock.sched(0.25, {
        if (~swSetLiveMode.notNil) {
            ~swSetLiveMode.(~swRequestedLiveMode ? \auduino);
        };
        if (~swRequestedStatusGUI ? false) {
            { ~swLiveStatusGUI.(); }.defer;
        };
        if (~swRequestedSmoothMixGUI ? false) {
            { ~swSmoothMixGUI.(); }.defer;
        };
        if (~swSmoothChoirDebugOnly ? false) {
            ~swAuduinoStopAll.();
            ~swSmoothStop.();
            ~swAmbientStop.();
            ~swReeseStop.();
            ~swWowStop.();
            if (~swChoirDebugTrigger.notNil) {
                ~swChoirDebugTrigger.(
                    choirAmp: ~swAuduinoParams[\smoothChoirAmp] ? 0.46,
                    synthAmp: ~swAuduinoParams[\smoothChoirSynthAmp] ? 0.74,
                    sampleStart: ~swAuduinoParams[\smoothChoirSampleStart] ? 0.0
                );
            };
        };
        nil;
    });
};
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

start_supercollider() {
    : > "$SCLANG_LOG"
    log "Starting SuperCollider"
    nohup "$SCLANG_BIN" -D "$BOOT_FILE" > "$SCLANG_LOG" 2>&1 &
    local pid=$!
    printf '%s\n' "$pid" > "$SCLANG_PID_FILE"

    if ! wait_for_log_text "$SCLANG_LOG" "SpaceWizard synth loaded." 25 "$pid"; then
        printf 'SuperCollider failed to boot cleanly. Recent log output:\n' >&2
        tail -n 40 "$SCLANG_LOG" >&2 || true
        exit 1
    fi

    log "SuperCollider ready (pid $pid)"
}

start_bridge() {
    local port="$1"
    local -a bridge_args

    : > "$BRIDGE_LOG"
    bridge_args=("$PYTHON_BIN" "-u" "$BRIDGE_SCRIPT" "$port" "--test" "--synth")
    if [[ "$VERBOSE_TEST" -eq 1 ]]; then
        bridge_args+=("--verbose-test")
    fi

    log "Starting bridge on $port"
    nohup "${bridge_args[@]}" > "$BRIDGE_LOG" 2>&1 &
    local pid=$!
    printf '%s\n' "$pid" > "$BRIDGE_PID_FILE"

    if ! wait_for_log_text "$BRIDGE_LOG" "Connected to SuperCollider synth" 15 "$pid"; then
        printf 'Bridge failed to start cleanly. Recent log output:\n' >&2
        tail -n 60 "$BRIDGE_LOG" >&2 || true
        exit 1
    fi

    log "Bridge ready (pid $pid)"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --port)
            [[ $# -ge 2 ]] || { usage >&2; exit 1; }
            PORT_OVERRIDE="$2"
            shift 2
            ;;
        --mode)
            [[ $# -ge 2 ]] || { usage >&2; exit 1; }
            case "$2" in
                auduino|ambient-flow|smooth-sine|reese-bass|wow-voice)
                    SYNTH_MODE="$2"
                    ;;
                *)
                    printf 'Unsupported synth mode: %s\n' "$2" >&2
                    printf 'Supported modes: auduino, ambient-flow, smooth-sine, reese-bass, wow-voice\n' >&2
                    exit 1
                    ;;
            esac
            shift 2
            ;;
        --smooth-preset)
            [[ $# -ge 2 ]] || { usage >&2; exit 1; }
            case "$2" in
                standard|quiet-pentatonic)
                    SMOOTH_PRESET="$2"
                    ;;
                *)
                    printf 'Unsupported smooth preset: %s\n' "$2" >&2
                    printf 'Supported smooth presets: standard, quiet-pentatonic\n' >&2
                    exit 1
                    ;;
            esac
            shift 2
            ;;
        --ambient-variant)
            [[ $# -ge 2 ]] || { usage >&2; exit 1; }
            case "$2" in
                flow|chordal|drone|forest)
                    AMBIENT_VARIANT="$2"
                    ;;
                *)
                    printf 'Unsupported ambient variant: %s\n' "$2" >&2
                    printf 'Supported ambient variants: flow, chordal, drone, forest\n' >&2
                    exit 1
                    ;;
            esac
            shift 2
            ;;
        --output-device)
            [[ $# -ge 2 ]] || { usage >&2; exit 1; }
            OUTPUT_DEVICE_OVERRIDE="$2"
            shift 2
            ;;
        --status-gui)
            STATUS_GUI=1
            shift
            ;;
        --smooth-mix-gui)
            SMOOTH_MIX_GUI=1
            shift
            ;;
        --verbose-test)
            VERBOSE_TEST=1
            shift
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

require_file "$PYTHON_BIN" "Python interpreter"
require_file "$SCLANG_BIN" "SuperCollider language executable"
require_file "$SYNTH_FILE" "SpaceWizard synth patch"
require_file "$BRIDGE_SCRIPT" "OSC bridge script"

stop_existing_processes

if [[ "$STOP_ONLY" -eq 1 ]]; then
    log "Stopped SpaceWizard live processes"
    exit 0
fi

mkdir -p "$LOG_DIR"

OUTPUT_DEVICE=$(detect_output_device)
log "Using audio output $OUTPUT_DEVICE"
write_boot_file "$OUTPUT_DEVICE"

RECEIVER_PORT=$(detect_receiver_port) || {
    printf 'Could not auto-detect the receiver serial port. Use --port /dev/cu.usbserialXXX if needed.\n' >&2
    exit 1
}

log "Using receiver port $RECEIVER_PORT"
free_serial_port "$RECEIVER_PORT"
start_supercollider
start_bridge "$RECEIVER_PORT"

cat <<EOF

SpaceWizard live rig started.
Receiver: $RECEIVER_PORT
Mode: $SYNTH_MODE
Ambient variant anchor: $AMBIENT_VARIANT
Smooth preset: $( [[ "$SYNTH_MODE" == "smooth-sine" ]] && printf '%s' "$SMOOTH_PRESET" || printf 'n/a' )
Live status GUI: $( [[ "$STATUS_GUI" -eq 1 ]] && printf 'enabled' || printf 'disabled' )
Smooth mix GUI: $( [[ "$SMOOTH_MIX_GUI" -eq 1 ]] && printf 'enabled' || printf 'disabled' )
Audio output: $OUTPUT_DEVICE
SuperCollider log: $SCLANG_LOG
Bridge log: $BRIDGE_LOG

To stop everything:
  $(basename "$0") --stop
EOF
