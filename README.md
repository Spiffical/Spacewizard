# SpaceWizard

A motion-sensing candle for an interactive installation. A visitor picks the candle up,
and its movement drives MaestroDMX stage lighting and a SuperCollider sound engine.
The candle's motion is also broadcast as an open OSC stream that anyone on the same
network can use.

```
candle: Arduino Nano + MPU6050 + nRF24L01
   |  2.4 GHz radio
receiver: Arduino Nano + nRF24L01 --USB--> Raspberry Pi (or a laptop)
                                            |- bridge --> MaestroDMX lights (OSC, UDP 7672)
                                            |- SuperCollider synth --> speakers
                                            '- public OSC stream (UDP 9000)
```

## Quick start

1. **Flash the firmware.** Calibrate the MPU6050 with `firmware/calibration/`, then flash
   `firmware/transmitter` (the candle) and `firmware/receiver` to Arduino Nanos.
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

## Sound modes

| Mode | Behaviour |
|---|---|
| `smooth-sine` | A quiet pentatonic tone and an ambient bed that follow tilt, roll and heading, with a choir accent on pickup. Use the `pi-quiet-pentatonic` preset on the Pi. |
| `day-o` | Plays `synth/samples/song/day-o.wav` from its cue point when the candle is moved. As movement stops the song slows down and drops in pitch, and moving again brings it back. The recording is not included. |
| `ambient-flow`, `auduino`, `reese-bass`, `wow-voice` | Earlier experiments. |

If the candle settles upright on its stand, a voice clip from `synth/samples/voice/` asks
the visitor to put it back on the ground. The clip is not included.

## OSC stream

Any computer on the same network as the Pi gets the candle's data by listening for OSC on
**UDP port 9000**: one bundle per frame at 30 Hz, sent by broadcast. Check it with
`python3 tools/osc_monitor.py`, which needs only `pip install python-osc`.

| Address | Arguments |
|---|---|
| `/spacewizard/candle/valid` | `i` 1 while sensor data arrives, 0 when it stops |
| `/spacewizard/candle/active` | `i` 1 while the candle is being handled |
| `/spacewizard/candle/activity` | `f` movement right now, 0 to 1 |
| `/spacewizard/candle/energy` | `f` averaged movement, 0 to 1 |
| `/spacewizard/candle/orientation` | `fff` pitch -90 to 90, roll -180 to 180, yaw 0 to 360, in degrees |
| `/spacewizard/candle/quaternion` | `ffff` sensor orientation w, x, y, z |
| `/spacewizard/candle/accel` | `fff` acceleration in g, including gravity |
| `/spacewizard/candle/gyro` | `fff` rotation rate in degrees per second |
| `/spacewizard/candle/pickup` | `i` event: the candle was just picked up |
| `/spacewizard/candle/impact` | `f` event: a knock or drop, jerk in g per second |

On networks that block broadcast, list receivers as `"host:port"` in `public_osc.targets`
in `config/bridge.json`.

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
  (`song_gate`), the OSC stream (`public_osc`) and the Maestro address.
- `config/osc_mappings_pickup.json`: which candle values drive which Maestro controls.
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
