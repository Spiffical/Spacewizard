# SpaceWizard

A wireless motion sensor for interactive installations. Build it into an object that
people can pick up, and the way they move it drives MaestroDMX stage lighting and a
SuperCollider sound engine. The motion is also broadcast as an open OSC stream that
anyone on the same network can use. The defaults are tuned for the first installation,
a candle; see [Using another object](#using-another-object).

```
transmitter, inside the object: Arduino Nano + MPU6050 + nRF24L01
   |  2.4 GHz radio
receiver: Arduino Nano + nRF24L01 --USB--> Raspberry Pi (or a laptop)
                                            |- bridge --> MaestroDMX lights (OSC, UDP 7672)
                                            |- SuperCollider synth --> speakers
                                            '- public OSC stream (UDP 9000)
```

## Quick start

1. **Flash the firmware.** Calibrate the MPU6050 with `firmware/calibration/`, then flash
   `firmware/transmitter` (inside the object) and `firmware/receiver` to Arduino Nanos.
   Wiring and libraries are in [docs/hardware.md](docs/hardware.md).
2. **Install** [SuperCollider](https://supercollider.github.io) and Python 3, then:
   ```bash
   python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
   ```
3. **Run** with the receiver plugged in over USB:
   ```bash
   scripts/spacewizard_live.sh start --mode smooth-sine --smooth-preset pi-quiet-pentatonic
   scripts/spacewizard_live.sh start --mode day-o --maestro-host 192.168.37.1   # with lights
   scripts/spacewizard_live.sh stop
   ```
   Without `--maestro-host` the rig runs sound only. Logs are in `.runtime/logs/`.

## Using another object

The sensor works in anything that can be picked up and moved. Two things depend on the
object, both in `config/bridge.json`:

- **Orientation.** `synth_settings.tilt_mapping` gives the object's long axis in the
  sensor's coordinates, so pitch, roll and heading follow the object, not the circuit
  board. `pitch_axis_unit` is the accelerometer reading, scaled to length 1, while the
  object stands upright (read `accel` in `tools/osc_monitor.py`). `roll_axis_unit` is a
  perpendicular axis, such as the same reading while the object lies on its side. Both
  are currently measured for the candle.
- **Sensitivity.** `interaction_mode` and `song_gate` set how much movement counts as
  handling the object. Record a pickup, a move and a rest with
  `tools/capture_pickup_session.py` to tune them.

## Sound modes

| Mode | Behaviour |
|---|---|
| `smooth-sine` | A quiet pentatonic tone and an ambient bed that follow tilt, roll and heading, with a choir accent on pickup. Use the `pi-quiet-pentatonic` preset on the Pi. |
| `day-o` | Plays a recording from `synth/samples/song/` from its cue point when the object is moved. As movement stops the song slows down and drops in pitch, and moving again brings it back. Set up for "Day-O", but any song works: set `songFile` and `songStartSeconds` in `synth/sound_presets.scd`. No recording is included. |
| `ambient-flow`, `auduino`, `reese-bass`, `wow-voice` | Earlier experiments. |

If the object is left standing upright after being moved, a voice clip from
`synth/samples/voice/` can play, for example to ask visitors to put it back. No clip is
included, and without one the prompt is off.

## OSC stream

Any computer on the same network as the Pi can receive the motion data by listening for
OSC on **UDP port 9000**: one bundle per frame at 30 Hz, sent by broadcast. Check it with
`python3 tools/osc_monitor.py`, which needs only `pip install python-osc`.

Addresses start with `/spacewizard/sensor`; to use another name, set
`public_osc.address_prefix` in `config/bridge.json`.

| Address | Arguments |
|---|---|
| `/valid` | `i` 1 while sensor data arrives, 0 when it stops |
| `/active` | `i` 1 while the object is being handled |
| `/activity` | `f` movement right now, 0 to 1 |
| `/energy` | `f` averaged movement, 0 to 1 |
| `/orientation` | `fff` pitch -90 to 90, roll -180 to 180, yaw 0 to 360, in degrees |
| `/quaternion` | `ffff` sensor orientation w, x, y, z |
| `/accel` | `fff` acceleration in g, including gravity |
| `/gyro` | `fff` rotation rate in degrees per second |
| `/pickup` | `i` event: the object was just picked up |
| `/impact` | `f` event: a knock or drop, jerk in g per second |

On networks that block broadcast, list receivers as `"host:port"` in `public_osc.targets`.

## Raspberry Pi

The Pi runs the rig as a user service that starts at boot and restarts on failure.

```bash
scripts/install_pi_service.sh      # on the Pi, once
scripts/set_mode.sh day-o          # on the Pi: switch sound mode (persists across reboots)
scripts/deploy_to_pi.sh            # from a laptop: copy this checkout, run the tests, restart
systemctl --user status spacewizard-production.service
```

Bridge and synth logs are in `.runtime/logs/` on the Pi.

The serial port, Maestro address and default mode are set in
`systemd/user/spacewizard-production.service`. Connect the Pi to the Maestro by
Ethernet for performances; Maestro's manual advises against relying on its Wi-Fi hotspot.

## Configuration

- `config/bridge.json`: motion thresholds (`interaction_mode`), the song's motion energy
  (`song_gate`), orientation axes (`synth_settings.tilt_mapping`), the OSC stream
  (`public_osc`) and the Maestro address.
- `config/osc_mappings_pickup.json`: which motion values drive which Maestro controls.
- `synth/sound_presets.scd`: sound levels and behaviour for each mode.

## Layout

```
bridge/     Python bridge: serial in; Maestro, synth and public OSC out
config/     bridge settings and Maestro mappings
firmware/   transmitter, receiver and calibration sketches
synth/      SuperCollider patch, sound presets and samples
scripts/    launcher, Pi service, mode switch and deploy scripts
tools/      sensor capture tools and the OSC monitor
tests/      regression tests
data/       recorded sensor traces
docs/       hardware notes and audit history
legacy/     the original NeoPixel-only version
```

## Tests

```bash
.venv/bin/python -m unittest tests/test_reliability.py
```
