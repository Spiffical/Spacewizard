# Reliability fixes — 2026-09-19

The bridge and SuperCollider fixes are installed on `sbialek@spacewizard-pi.local` in `/home/sbialek/Spacewizard-sound`. The Pi's original runtime files matched the local pre-fix versions; its sound preset and tuning were preserved. A rollback archive is at `/home/sbialek/spacewizard-backups/2026-09-19-reliability/runtime-before.tgz`. Existing local work was preserved; no Git commit was made.

## Implemented behavior

- Startup, input loss, USB disconnection, and normal shutdown send explicit off commands. After 0.5 seconds without valid sensor data, the bridge closes the gate and repeats off commands every 0.2 seconds. Motion state resets after gaps.
- Serial connections reopen automatically, including startup with no receiver. Failed OSC destinations retry independently. Logs distinguish configured UDP output from verified delivery.
- Incomplete, duplicate-field, nonfinite, out-of-range, invalid-quaternion, and explicitly invalid frames are rejected. Repeated sample sequence numbers do not refresh the input deadline.
- Confirmed rest adopts the settled acceleration baseline/reference. A constant stationary sample no longer repeatedly retriggers pickup after settling.
- SuperCollider independently closes the motion gate and cancels stand prompts after 0.75 seconds without live control. The configured sound fade still applies; this is not an instant audio mute.
- Cancelled stand-prompt callbacks cannot interfere with newer callbacks. Playback rechecks current motion, orientation, and input freshness. Repeated off packets no longer postpone voice cleanup indefinitely.
- The production runner checks bridge and synth heartbeats as well as PIDs. Audio-server failure stops the synth heartbeat even if the language process survives; systemd then restarts the rig. An intentionally switched-off candle does not cause restart loops.

## Firmware prepared locally, not flashed

Both Arduino sketches compile for `arduino:avr:nano` with AVR core 1.8.7, RF24 1.5.0, and MPU6050 1.4.4. RX uses 4,748 bytes of flash / 547 bytes of RAM; TX uses 15,664 / 704.

The new 32-byte radio payload adds a sample sequence and validity marker. TX marks stale sensor samples invalid, bounds I2C operations/FIFO draining, retries MPU initialization, and uses actual radio acknowledgements. Both sides retry radio initialization; alternate receive addresses use the required shared prefix. RX still accepts original 28-byte packets.

**Both boards need the updated firmware for the complete sensor freshness/recovery path.** With legacy firmware, a frozen sensor that keeps transmitting cannot be distinguished from genuine stationary data. Hardware watchdog resets were not enabled because the boards' bootloader compatibility has not been established.

## Verification

- Eight Python regressions passed on Mac and Pi. These include a real bridge subprocess, pseudo-terminal receiver removal/replacement, localhost OSC capture, lost/resumed input, malformed frames, safe startup/shutdown, serial/OSC retry, sequence handling, rest reactivation, and heartbeat expiry.
- The full synth patch compiled on Mac SuperCollider 3.14.1 and Pi 3.13.0. Tests of its current timer/watchdog functions passed.
- An isolated real synth server passed on Mac (BlackHole) and Pi (PipeWire/JACK, no speaker). Tests exercised actual voice creation/cleanup, input expiry/recovery, explicit invalid input, and heartbeat cessation after audio-server exit. Test ports were separate from production.
- Shell syntax and Git whitespace checks passed. Runtime files were compared before deployment.
- On the actual Pi production service, force-killing the bridge triggered automatic recovery in 37.0 seconds. Force-killing only `scsynth` while `sclang` survived triggered recovery in 45.1 seconds. Both heartbeats resumed; the service remained healthy with the actual RX connected and the candle off. The service was left enabled and running. These timings include service restart delay and complete synth startup.

Saved output is in [verification/](verification/), including Pi regression/integration output and production recovery results.

## Still requires the complete rig

1. Flash both updated Arduino sketches after confirming board/bootloader settings, then test RF interruption, both radios' power loss, sensor failure, and USB unplug/replug using the actual devices. No Arduino was flashed in this session.
2. Record stationary, pickup, rotation, and set-down motion with the candle on. Sensor scaling remains deliberately unchanged: correcting the DMP/raw-gyro units requires a coordinated sensitivity retune. Current sensitivity has not been physically validated.
3. Test Maestro restart/network loss and actual light output. UDP has no delivery acknowledgement; the bridge cannot extinguish lights while its network path to Maestro is down. A receiver-side timeout/failsafe is still needed if holding the last lighting state during a network outage is unacceptable.
4. Listen to the actual speaker output, including fades and stand prompts. Software voice/graph checks do not verify the amplifier, speaker, or perceived sound.

The original [AUDIT.md](AUDIT.md) and its reproductions are historical evidence. Its reproduction assertions intentionally demonstrate old bugs; use `tests/` for current regression checks.
