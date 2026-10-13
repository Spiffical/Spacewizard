**SpaceWizard reliability audit — 2026-09-19**

This is the original, pre-fix audit snapshot. See [FIXES.md](FIXES.md) for the subsequent changes, deployment, verification, and remaining hardware work. Line numbers below refer to the audited snapshot.

The current code has reproducible unintended activation and missing recovery safeguards. It should not yet be treated as reliably unattended.

Scope: the current working tree at base commit `1a8ab78`, including existing uncommitted changes, for the Maestro OSC transmitter/receiver, Python bridge, live SuperCollider control path, launch scripts, and Pi production service. The nested `sbialek@spacewizard-pi.local` copy and deprecated/LED implementations were not treated as the production source. This is a local source audit; the firmware actually flashed on the Arduinos and files installed on the Pi were not verified. Runtime source and configuration were not changed.

P1 means fix before unattended operation. P2 means a consequential correctness/reliability defect to address next.

1. **[P1] Lost input leaves the last lighting and sound state active; shutdown does not send off.**

   [app.py:482](/Users/spencerbialek/PersonalProjects/Spacewizard/maestro-osc-version/bridge/app.py:482) immediately continues on empty/invalid input. There is no last-valid-frame deadline or independent output heartbeat. [Cleanup:633](/Users/spencerbialek/PersonalProjects/Spacewizard/maestro-osc-version/bridge/app.py:633) only closes serial. The [synth OSC handler:7976](/Users/spencerbialek/PersonalProjects/Spacewizard/maestro-osc-version/synth/SpaceWizardSynth.scd:7976) also has no input timeout.

   Reproduction: activate the gate, simulate over ten seconds without radio data, then interrupt the bridge. Zero commands are sent during silence or shutdown; the last Maestro brightness and synth gate both remain `1`. A startup with no valid frames likewise sends no initial off command. Continuous synth voices can remain active, and the code never tells Maestro to extinguish the lights. Any independent Maestro timeout would need separate verification.

   Fix: independent monotonic freshness deadline, explicit off at startup/loss/shutdown, repeated safe-state output during outages, and a synth-side watchdog for bridge crashes. Invalidate pending pickup/rest candidates on gaps: two motion frames a minute apart currently satisfy the 10 ms pickup hold. If the network to Maestro itself is down, a Python off message cannot reach it; safe behavior during that outage requires a timeout at Maestro or downstream.

2. **[P1] Settling at a different acceleration baseline repeatedly reactivates a stationary candle.**

   [metrics.py:343](/Users/spencerbialek/PersonalProjects/Spacewizard/maestro-osc-version/bridge/metrics.py:343) freezes baseline adaptation while active. On confirmed rest, [line 456](/Users/spencerbialek/PersonalProjects/Spacewizard/maestro-osc-version/bridge/metrics.py:456) clears the stable reference without adopting the settled baseline. The old baseline still satisfies the activation threshold on subsequent frames.

   Reproduction with the current configuration: hold acceleration magnitude at `0.50`, change once to `0.53`, then keep all inputs perfectly constant and gyro zero. Gate opens at 1110 ms, closes at 3120 ms, reopens at 3140 ms, and repeats. There are four further pickups before ten seconds, with no further input changes. Rate limiting can also omit the brief off interval, making this appear continuously on.

   Fix: establish the new rest baseline and vector reference when rest is confirmed; require subsequent fresh movement to rearm. Preserve that reference rather than immediately discarding it.

3. **[P1] Incomplete serial frames are accepted as real motion and can trigger on.**

   [serial_input.py:51](/Users/spencerbialek/PersonalProjects/Spacewizard/maestro-osc-version/bridge/serial_input.py:51) accepts any parsed key/value subset, while [metrics.py:215](/Users/spencerbialek/PersonalProjects/Spacewizard/maestro-osc-version/bridge/metrics.py:215) defaults absent axes to zero. A complete newline is not proof of a complete sensor record. Dropping `FMT=2` also causes raw integer counts to be interpreted as engineering units.

   Reproduction: `GY=0 GZ=0 IMP=0`, then two identical stationary full frames, opens the gate. The saved timing trace contains an actual truncated initial record, so partial input is not merely hypothetical. `NaN` is accepted, and `FMT=2 IMP=inf` raises an uncaught `OverflowError`.

   Fix: require the full mandatory field set, explicit supported protocol version, finite numeric values, and valid ranges before updating any state. Reject malformed records completely and include them in the freshness timeout.

4. **[P1] Sensor/radio faults have no bounded recovery, and stale sensor data masquerades as fresh traffic.**

   The transmitter [loop:233](/Users/spencerbialek/PersonalProjects/Spacewizard/maestro-osc-version/Spacewizard2point0_osc_transmitter/Spacewizard2point0_osc_transmitter.ino:233) always transmits, even when MPU initialization failed or no new DMP packet was read. A stopped DMP can therefore produce endless copies of the last moving sample. No packet sequence, sample age, validity flag, or boot identity lets the receiver distinguish these from fresh measurements.

   MPU initialization is attempted only in setup. Both sketches ignore `radio.begin()` failure and configure the radio only once; a radio-only brownout can erase its configuration without restarting the Arduino. The transmitter also never enables an I2C timeout; the installed AVR Wire implementation defaults to timeout disabled. A stuck bus can block the loop indefinitely. Restarting the Pi bridge cannot fix a hung transmitter.

   Fix: transmit only valid fresh samples, include freshness/validity metadata within the radio payload budget, bound I2C operations, retry failed initialization with backoff, recover radio configuration after detected failures, and use an MCU watchdog with a safe startup state. Confirm these paths with physical power-fault tests.

5. **[P1] USB disconnects and synth send errors terminate the combined bridge.**

   The runtime loop catches only decoding errors and keyboard interrupts at [app.py:609](/Users/spencerbialek/PersonalProjects/Spacewizard/maestro-osc-version/bridge/app.py:609). Serial is opened once. [Synth sends:257](/Users/spencerbialek/PersonalProjects/Spacewizard/maestro-osc-version/bridge/osc.py:257) propagate network errors instead of handling them independently of Maestro output.

   Both failure paths were reproduced with in-memory I/O: USB removal exits after one serial-open attempt; an injected synth send error exits after Maestro was set to full brightness. The standalone live launcher provides no later restart. The Pi production runner should detect an exited bridge and request a systemd restart, so USB loss is not necessarily permanent there; that recovery restarts the entire sound stack and still lacks an off command.

   Fix: retry serial reopening on the stable device path while maintaining safe outputs, reset motion state after reconnection, and isolate synth/network failures so one destination cannot stop the other.

6. **[P2] The claimed radio fallback cannot detect missing delivery, and its alternate addresses do not match.**

   [Transmitter:214](/Users/spencerbialek/PersonalProjects/Spacewizard/maestro-osc-version/Spacewizard2point0_osc_transmitter/Spacewizard2point0_osc_transmitter.ino:214) disables auto-ack, but [line 316](/Users/spencerbialek/PersonalProjects/Spacewizard/maestro-osc-version/Spacewizard2point0_osc_transmitter/Spacewizard2point0_osc_transmitter.ino:316) treats `radio.write()` success as delivered. With auto-ack disabled, a successful transmit reports success even when the receiver is absent. Enabling auto-ack alone is insufficient: [receiver:89](/Users/spencerbialek/PersonalProjects/Spacewizard/maestro-osc-version/Spacewizard2point0_osc_receiver/Spacewizard2point0_osc_receiver.ino:89) opens pipes 2/3 with different address prefixes, but those pipes inherit pipe 1's upper address bytes. These behaviors are documented in the [RF24 API](https://nrf24.github.io/RF24/classRF24.html) and match the locally installed library implementation.

   Fix: use compatible addressing and acknowledgements or an explicit application heartbeat. Multiple addresses on one channel do not provide independent RF paths. Ordinary interference can clear and traffic resume without a reconnect handshake; this finding concerns missing delivery detection and ineffective fallback.

7. **[P2] A complete single FIFO packet is skipped and the old buffer is decoded.**

   [Transmitter:259](/Users/spencerbialek/PersonalProjects/Spacewizard/maestro-osc-version/Spacewizard2point0_osc_transmitter/Spacewizard2point0_osc_transmitter.ino:259) uses `getFIFOCount() > dmpPacketSize`. Exactly one complete packet does not enter the read loop, yet decoding proceeds. At startup the buffer is zero-initialized; later it contains a previous packet. At packet-aligned counts, draining also leaves one whole packet unread. Accelerometer/quaternion data can thus lag the gyro read.

   Fix: consume complete packets with `>=`, bound draining, and decode/transmit only after a successful new read. Handle overflow and invalid sizes explicitly. This finding is established by the control flow; it was not tested on physical firmware.

8. **[P2] Sensor scaling does not match the DMP/library output.**

   [serial_input.py:9](/Users/spencerbialek/PersonalProjects/Spacewizard/maestro-osc-version/bridge/serial_input.py:9) divides acceleration by 16384 and gyro by 131. The transmitter takes acceleration from the DMP FIFO, whose installed MPU6050 1.4.4 implementation documents 8192 counts per g, and raw gyro registers after DMP initialization selects ±2000°/s (16.4 counts per °/s). See the [local DMP initialization:342](/Users/spencerbialek/Documents/Arduino/libraries/MPU6050/src/MPU6050_6Axis_MotionApps20.cpp:342) and [gravity subtraction:508](/Users/spencerbialek/Documents/Arduino/libraries/MPU6050/src/MPU6050_6Axis_MotionApps20.cpp:508).

   Reproduction: a one-g DMP sample reports `0.5g`; a raw gyro count of 164 reports approximately `1.25°/s` rather than `10°/s`. Historical stationary captures also cluster near `0.5g`. Baseline adaptation partly hides this error, but physical thresholds, jerk, gyro activity, and fallback orientation calculations are mis-scaled.

   Fix: pin the firmware/library protocol and use the correct units, then retune thresholds using new stationary/movement recordings. Do not simply change divisors while retaining thresholds tuned against the old units. The exact libraries flashed on the devices remain unverified.

9. **[P2] Cancelled stand-prompt callbacks can invalidate newer cancellations.**

   [Synth:2936](/Users/spencerbialek/PersonalProjects/Spacewizard/maestro-osc-version/synth/SpaceWizardSynth.scd:2936) clears the shared pending flag even for an obsolete callback. [Cancellation:2876](/Users/spencerbialek/PersonalProjects/Spacewizard/maestro-osc-version/synth/SpaceWizardSynth.scd:2876) increments the token only when that flag is present.

   Reproduction in an isolated SuperCollider language process: schedule A, cancel A, schedule B, let A's obsolete callback fire, then cancel B. B still invokes playback. In live use the movement-triggered cancellation can consequently fail, allowing the stand voice while the candle is being used upright.

   Fix: invalidate the token on every cancellation, allow only the owning callback to clear pending state, and recheck current gate/activity/freshness before playback.

10. **[P2] Continuous off packets prevent scheduled voice cleanup from running.**

    [Synth:2859](/Users/spencerbialek/PersonalProjects/Spacewizard/maestro-osc-version/synth/SpaceWizardSynth.scd:2859) stops and recreates the cleanup routine on every gate update, including repeated `false`. At roughly 30 Hz, its fade-out-plus-0.35-second wait never completes. An isolated execution reproduced zero cleanup calls throughout continuous off updates. The motion bus still fades down, so this is not proof of audible sound remaining on; it retains voice processing/resources and delays the intended reset.

    Fix: schedule cleanup on the open-to-closed transition and cancel only on reopening, or preserve an absolute cleanup deadline.

11. **[P2] Production supervision checks process existence, not functioning audio or fresh data.**

    [Runner:49](/Users/spencerbialek/PersonalProjects/Spacewizard/scripts/spacewizard_production_runner.sh:49) checks only the sclang and bridge PID files. A crashed `scsynth` audio server with a surviving sclang process, or a silent/hung bridge, passes this test. The synth patch has no server-recovery hook that rebuilds its graph. Startup log messages also establish neither current sensor traffic nor actual Maestro receipt.

    Fix: monitor valid sensor freshness and SuperCollider server responsiveness, with bounded recovery and a safe state before restarts. Use a separate status for “sending UDP” versus verified destination health.

**Validation and limits**

The [Python reproduction script](/Users/spencerbialek/PersonalProjects/Spacewizard/audits/2026-09-19/reproduce.py) runs the actual bridge/metrics/parser code with mocked serial and OSC clients. It exercises silent input, clean interruption, USB removal, synth send failure, ordinary input resumption, partial/nonfinite records, stale pickup timing, rest reactivation, and historical trace replay. [Results](/Users/spencerbialek/PersonalProjects/Spacewizard/audits/2026-09-19/results.json) record the observations. Its assertions deliberately confirm the bugs in this snapshot; they are not passing safety tests.

The [SuperCollider reproduction](/Users/spencerbialek/PersonalProjects/Spacewizard/audits/2026-09-19/synth_timer_reproduction.scd) contains exact copies of the four relevant audited functions with shortened timers and mocked audio actions. It ran under SuperCollider 3.14.1 without booting an audio server. [Output](/Users/spencerbialek/PersonalProjects/Spacewizard/audits/2026-09-19/synth_timer_results.txt) confirms both timer defects. This copied-function harness must be regenerated after source changes.

Replaying `timing_probe_20260428_204911.csv` with the current config, excluding incomplete records, yielded one pickup during `HORIZONTAL_REST` (500/634 frames active) and 640/640 active frames during `FINAL_REST`. Stage labels are historical capture instructions, not independent proof of perfect physical stillness; this is evidence that the current tuning needs bench validation. The constant-input reactivation reproduction is independent of that uncertainty.

Maestro OSC uses UDP. The bridge resends current values on valid frames and catches Maestro send errors, so a temporary network outage to the same address ordinarily permits output to resume when delivery becomes possible. There is no reliable delivery acknowledgement or verified connection state, and the advertised retry/connection-timeout config entries are unused. A separate reconnection handshake is not the missing mechanism; freshness, safe-state delivery, and health monitoring are.

Suggested repair order: implement safe loss/shutdown behavior; correct rest rearming and frame validation; add serial and firmware recovery/freshness; fix radio/FIFO/scaling; fix synth timers and supervision. Before deployment, physically test radio removal/return, USB unplug/replug, transmitter/receiver brownouts, frozen sensor output, Maestro/network restart, audio-server failure, startup with the candle already resting, and long stationary runs. No hardware was flashed, live rig restarted, or device disconnected during this audit.
