"""Offline audit reproductions; no serial devices or OSC sockets are opened.

Run from the repository root with .venv/bin/python audits/2026-09-19/reproduce.py.
Assertions confirm the observed bugs in the audited version, NOT safe behavior.
After fixes, these assertions should fail and be replaced with regression tests.
"""

from contextlib import redirect_stdout
import csv
import io
import json
from pathlib import Path
import sys
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "maestro-osc-version"))

from bridge import app, metrics
from bridge.config import load_config
from bridge.serial_input import parse_serial_line

CONFIG_PATH = ROOT / "maestro-osc-version/config.json"
CONFIG = load_config(str(CONFIG_PATH)).config
RESULTS = {}


def frame(z=0.5, gx=0.0):
    return {"AX": 0.0, "AY": 0.0, "AZ": z, "GX": gx, "GY": 0.0, "GZ": 0.0}


def calculate(data, state, timestamp):
    with patch.object(metrics, "get_current_time_ms", return_value=timestamp):
        return metrics.calculate_motion_metrics(data, CONFIG, state)


state = metrics.ImpactState()
transitions = []
previous = False
for timestamp in range(1000, 10001, 10):
    result = calculate(frame(z=0.5 if timestamp < 1100 else 0.53), state, timestamp)
    if result["interaction_active"] != previous:
        transitions.append([timestamp, result["interaction_active"]])
        previous = result["interaction_active"]
assert len([t for t, active in transitions if active and t > 1110]) == 4
RESULTS["constant_rest_reactivates"] = transitions

state = metrics.ImpactState()
for timestamp, data in [(1000, parse_serial_line("GY=0 GZ=0 IMP=0")),
                        (1010, frame()), (1020, frame())]:
    result = calculate(data, state, timestamp)
assert result["interaction_active"]
RESULTS["partial_startup_then_stationary_frames"] = {"gate": result["interaction_gate"]}

assert parse_serial_line("AX=nan") is not None
try:
    parse_serial_line("FMT=2 IMP=inf")
except OverflowError:
    RESULTS["nonfinite_input"] = {"nan_accepted": True, "infinite_impact": "OverflowError"}
else:
    raise AssertionError("Expected infinite IMP to escape parser")

state = metrics.ImpactState()
calculate(frame(), state, 1000)
calculate(frame(gx=2), state, 1010)
result = calculate(frame(gx=2), state, 61010)
assert result["pickup_triggered"]
RESULTS["two_motion_frames_one_minute_apart"] = {"pickup_triggered": True}


class DummySerial:
    is_open = True

    def readline(self):
        return b""

    def close(self):
        self.is_open = False


def bridge_scenario(*, unplug=False, synth_error=False, recover=False):
    clock = [1000]
    records = []
    clients = []
    serial_opens = []
    stream = [(1000, frame()), (1100, frame(gx=2)), (1120, frame(gx=2)), (1160, frame(gx=2))]
    stream += [(t, None) for t in range(1200, 12001, 200)]
    if recover:
        stream += [(t, frame()) for t in range(12020, 15001, 20)]
    iterator = iter(stream)

    class DummyClient:
        def __init__(self, host, port):
            self.port = port
            clients.append(self)

        def send_message(self, address, value):
            if synth_error and self.port == 57120 and clock[0] >= 1160:
                raise OSError("simulated synth send failure")
            records.append((clock[0], self.port, address, value))

    def open_serial(**kwargs):
        serial_opens.append(kwargs["port"])
        return DummySerial()

    def read_line(_port):
        try:
            timestamp, data = next(iterator)
        except StopIteration:
            if unplug:
                raise app.serial.SerialException("simulated USB removal")
            raise KeyboardInterrupt
        clock[0] = timestamp
        return "" if data is None else " ".join(f"{k}={v}" for k, v in data.items())

    error = None
    with patch.object(app.serial, "Serial", side_effect=open_serial), \
         patch.object(app, "SimpleUDPClient", DummyClient), \
         patch.object(app, "_read_latest_serial_line", side_effect=read_line), \
         patch.object(metrics, "get_current_time_ms", side_effect=lambda: clock[0]), \
         patch.object(app.time, "monotonic", side_effect=lambda: clock[0] / 1000), \
         redirect_stdout(io.StringIO()):
        try:
            app.main(["mock-serial", "127.0.0.1", "7672", "--synth", "--config", str(CONFIG_PATH)])
        except (app.serial.SerialException, OSError) as exc:
            error = type(exc).__name__
    return {"serial_opens": len(serial_opens), "error": error,
            "last_brightness": [r for r in records if r[2] == "/global/brightness"][-1],
            "last_synth_control": [r for r in records if r[2] == "/spacewizard/auduino"][-1],
            "messages_during_silence": sum(1200 <= r[0] <= 12000 for r in records)}


silence = bridge_scenario()
assert silence["last_brightness"][3] == 1.0
assert silence["last_synth_control"][3][0] == 1.0
assert silence["messages_during_silence"] == 0
RESULTS["silent_radio_then_keyboard_stop"] = silence

unplug = bridge_scenario(unplug=True)
assert unplug["serial_opens"] == 1 and unplug["error"] == "SerialException"
RESULTS["usb_removal"] = unplug

synth_failure = bridge_scenario(synth_error=True)
assert synth_failure["error"] == "OSError"
assert synth_failure["last_brightness"][3] == 1.0
RESULTS["synth_send_error_stops_combined_bridge"] = synth_failure

recovered = bridge_scenario(recover=True)
assert recovered["last_brightness"][3] == 0.0
RESULTS["ordinary_radio_stream_resumption"] = recovered

scaled = parse_serial_line("FMT=2 AX=0 AY=0 AZ=8192 GX=164 GY=0 GZ=0 IMP=0")
RESULTS["dmp_scaling"] = {"reported_gravity_g": scaled["AZ"], "reported_gyro_dps": scaled["GX"]}
assert scaled["AZ"] == 0.5

trace_path = ROOT / "maestro-osc-version/timing_probe_20260428_204911.csv"
state = metrics.ImpactState()
stages = {}
for row in csv.DictReader(trace_path.open()):
    data = parse_serial_line(row["raw_line"])
    # Exclude incomplete records, so parser corruption cannot explain these results.
    if not data or not all(k in data for k in ("AX", "AY", "AZ", "GX", "GY", "GZ")):
        continue
    result = calculate(data, state, 100000 + int(row["elapsed_ms"]))
    totals = stages.setdefault(row["stage"], {"frames": 0, "active": 0, "pickups": 0})
    totals["frames"] += 1
    totals["active"] += int(result["interaction_active"])
    totals["pickups"] += int(result["pickup_triggered"])
assert stages["HORIZONTAL_REST"]["pickups"] == 1
assert stages["FINAL_REST"]["active"] == stages["FINAL_REST"]["frames"]
RESULTS["historical_trace_with_current_config"] = stages

print(json.dumps(RESULTS, indent=2))
