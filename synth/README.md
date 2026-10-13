# SpaceWizard Synth Sketch

This folder contains a first synth prototype for the candle pickup piece.

The sound design goal is simple:
- the candle is quiet or nearly quiet at rest
- picking it up produces a bloom or ignition gesture
- ongoing motion turns into a sustained, playable texture
- tilt, roll, and spin can eventually shape the sound in real time

The current patch is written in SuperCollider because it is free, stable on macOS, and realistic to run later on Raspberry Pi.

It now also includes an Auduino-inspired mode based on Peter Knight's original Arduino granular synth design: two lo-fi triangle grains reset by a sync oscillator, with fast exponential decay and deliberate clipping/bit-depth reduction.

There is now also a second live mode, `smoothSine`, which keeps the same motion-control input path but turns it into short overlapping pentatonic notes instead of a sustained drone. The notes fade out naturally, and the live pitch follower now slews toward the target and advances one scale step at a time so slow tilt changes read more like a phrase than skipped jumps.

There is now also a third live mode, `reeseBass`, built around three detuned saw waves with glide, low-pass shaping, and a classic moving bass character.

There is now also a fourth live mode, `wowVoice`, which turns pitch movement into a drawn-out vocal-like `wow` gesture. Rising or falling pitch motion can trigger the start of the syllable, and the vowel then stretches until the next strong reversal or new burst of movement.

There is now also a fifth live mode, `ambientFlow`, which builds a slow, non-repeating soundscape from stochastic drift and irregular shimmer events. It was shaped after SuperCollider's dynamic-stochastic `Gendy` approach for the wandering bed, then paired with sparse resonant sparkles so tilt and roll change the space, color, and density instead of constantly dragging the harmony by pitch.

That ambient path now includes four switchable variants on one shared GUI: `Flow`, `Harmonic Chords`, `Drone Texture`, and `Forest Canopy`. The forest mode adds wind, leaf rustle, insects, and distant call-like gestures so the demo can sit in the actual installation setting rather than sounding like a generic synth pad.

The Auduino scenes now include the original 54-step D-E-G-A-B pentatonic lookup table from Peter Knight's code, plus the earlier SpaceWizard minor-pentatonic mapping as a switchable alternative.

## Files

- `SpaceWizardSynth.scd`: main patch, helper functions, and OSC listeners
- `SpaceWizardNoOscTests.scd`: quick audition script that does not require OSC or sensor hardware

## Install SuperCollider

On macOS:
- install the SuperCollider app from https://supercollider.github.io/downloads

On Raspberry Pi later:
- use the distro package or build instructions from the same project site

## First Run

1. Open `SpaceWizardSynth.scd` in SuperCollider.
2. Evaluate the whole file.
3. Wait for the server to boot.
4. Run one of the helper lines shown in the post window.

If you just want a fast audition, open `SpaceWizardNoOscTests.scd` and evaluate the first block. Then run the test lines one at a time.

## No-OSC Listening Tests

These are the first tests to try on the MacBook.

1. `~swTestPickup.()`
This is the shortest test. It gives you the pickup bloom and opens the sustained layer.

2. `~swTestTiltSweep.()`
This sweeps the tilt parameter from low to high. Listen for the filter opening and the harmonic center shifting upward.

3. `~swTestSpinStudy.()`
This pushes spin from nearly still to agitated. Listen for tremolo, FM roughness, and more animated high-frequency energy.

4. `~swTestAutopilot.()`
This is the most installation-like sketch. It starts gently, blooms, becomes active, then falls back toward rest.

5. `~swSawPentatonicScene.()`
This uses a dedicated distorted sawtooth lead voice. It first sweeps upward through a minor pentatonic scale, then starts choosing random notes from the same scale. The line is continuous, so each note sustains until the next pitch change.

6. `~swSawPentatonicSweep.()` and `~swSawPentatonicRandom.()`
These split the same idea into two parts if you want to hear them separately.

7. `~swAuduinoPentatonicScene.()`
This is the Auduino-inspired continuous version. It now defaults to the original Auduino D-E-G-A-B pentatonic table, then sweeps that set and moves into random notes from the same set.

8. `~swAuduinoMelodicScene.()`
This is the slower, more melodic continuous variant. It reduces the sync rate so the grains breathe longer and the pitch changes read more clearly as notes rather than textures.

9. `~swAuduinoSteppedScene.()` and `~swAuduinoMelodicSteppedScene.()`
These add discrete stepped-note versions with short gaps between notes, closer to the quantized stepped behavior from the original Auduino demos.

10. `~swSmoothSineScene.()` and `~swSmoothSineSweep.()`
These audition the new pentatonic note mode, first as a single note and then as a sweep through a wider pentatonic test range.

11. `~swReeseBassScene.()` and `~swReeseBassSweep.()`
These audition the new Reese-style bass mode with three detuned saw voices and portamento.

12. `~swWowScene.()` and `~swWowSweep.()`
These audition the new vocal-like `wow` mode and a manual rising/falling sweep that retriggers the syllable on reversal.

13. `~swAmbientScene.()`, `~swAmbientChordalScene.()`, `~swAmbientDroneScene.()`, `~swAmbientForestScene.()`, and `~swAmbientGUI.()`
These audition the ambient variants and open the dedicated manual test window. The GUI now has a mode selector for `Flow`, `Harmonic Chords`, `Drone Texture`, and `Forest Canopy`, plus a `Reset Mode` button that reloads the recommended preset for the currently selected variant. In live use, tilt now changes openness and darkness more than pitch, roll shifts width and left-right balance, activity increases event density, and spin adds shimmer or forest detail. The `Drone Texture` point in the live yaw ring also adds low pentatonic bass notes from the smooth-note synth layer.

14. `~swAuduinoGUI.()`
This opens a small live-control window for the Auduino voice with a scale selector, a play-mode selector, and a `Scale Step` slider. In stepped modes, moving that slider snaps the sound across the pentatonic notes instead of gliding continuously. Closing the window also stops any active Auduino sound.

## Manual Parameters

The patch exposes these controls through helper functions now, and later through OSC.

- `activity`: overall motion intensity from `0.0` to `1.0`
- `tilt`: bipolar control from `-1.0` to `1.0`; currently shifts tone and filter openness
- `roll`: bipolar control from `-1.0` to `1.0`; currently changes width and detune feel
- `spin`: from `0.0` to `1.0`; currently adds tremolo, FM motion, and air
- `presence`: from `0.0` to `1.0`; currently adds shimmer and reverb density
- `amp`: output level

Useful manual commands:

```supercollider
~swPickup.(activity: 0.3, tilt: 0.1, roll: 0.0, spin: 0.04, presence: 0.55, amp: 0.17);
~swSet.(activity: 0.6, tilt: 0.8, roll: -0.2, spin: 0.25, presence: 0.7, amp: 0.18);
~swRest.();
~swRelease.();
```

Useful saw-note commands:

```supercollider
~swSawPentatonicScene.();
~swSawPentatonicSweep.(rootMidi: 50, amp: 0.12);
~swSawPentatonicRandom.(rootMidi: 50, steps: 24, amp: 0.11);
```

Useful Auduino-style command:

```supercollider
~swAuduinoPentatonicScene.();
~swAuduinoPentatonicScene.(scaleMode: \originalAuduino);
~swAuduinoPentatonicScene.(scaleMode: \spacewizardMinor);
~swAuduinoSteppedScene.();
~swAuduinoSteppedScene.(scaleMode: \originalAuduino);
~swAuduinoMelodicScene.();
~swAuduinoMelodicScene.(scaleMode: \spacewizardMinor);
~swAuduinoMelodicSteppedScene.();
~swAuduinoGUI.();
```

Useful smooth-sine commands:

```supercollider
~swSetLiveMode.(\smoothSine);
~swSmoothSineScene.();
~swSmoothSineSweep.();
~swSetLiveMode.(\auduino);
```

Useful ambient commands:

```supercollider
~swSetLiveMode.(\ambientFlow);
~swAmbientScene.();
~swAmbientSweep.();
~swAmbientChordalScene.();
~swAmbientDroneScene.();
~swAmbientForestScene.();
~swAmbientGUI.();
~swSetLiveMode.(\auduino);
```

When the live bridge is running in `ambient-flow`, the launcher can now choose the ambient variant anchor with `--ambient-variant flow|chordal|drone|forest`. The bridge also sends normalized yaw, and the SuperCollider patch uses that yaw to crossfade smoothly around the four ambient variants in a circle. In the calibrated live setup, that yaw follows the DMP heading only when the candle is horizontal enough, while long-axis twisting feeds roll instead of yaw. The `drone` segment of that circle now also layers in low pentatonic bass notes from the short-note synth mode.

Useful Reese-bass commands:

```supercollider
~swSetLiveMode.(\reeseBass);
~swReeseBassScene.();
~swReeseBassSweep.();
~swSetLiveMode.(\auduino);
```

Useful wow-voice commands:

```supercollider
~swSetLiveMode.(\wowVoice);
~swWowScene.();
~swWowSweep.();
~swSetLiveMode.(\auduino);
```

The original Auduino source did not expose multiple pentatonic families. Its canonical stepped musical option was a single D-E-G-A-B table over many octaves. The new selector keeps that exact table available while still letting you compare it against the earlier SpaceWizard minor-pentatonic scene.

The saw helpers are now monophonic and legato by design. They change pitch on one continuously sounding lead rather than firing separate short notes.

## OSC Hooks For Later

The patch already installs three OSC listeners so it is ready for the next integration pass.

- `/spacewizard/pickup`
- `/spacewizard/gate`
- `/spacewizard/motion`

The intended `/spacewizard/motion` message shape is:

```text
/spacewizard/motion activity tilt roll spin presence amp
```

Example:

```text
/spacewizard/motion 0.42 -0.18 0.25 0.61 0.70 0.18
```

That is not wired into the Python bridge yet. This patch just makes the synth side testable first.

## What To Listen For

- Does the pickup bloom feel like a candle ignition rather than a generic synth stab?
- Is the sustained layer rich enough to hold attention after pickup?
- Does `spin` create excitement without turning harsh too fast?
- Does `tilt` feel musically meaningful, or should it map to pitch or filter differently?
- Should the resting state be silent, or leave a very faint ember behind?

## Live Launcher

If you are using the headless launcher from the main project folder, the default live mode remains Auduino grit:

```bash
scripts/spacewizard_live.sh start
```

To boot directly into the new pentatonic note mode instead, run:

```bash
scripts/spacewizard_live.sh start --mode smooth-sine

To boot the live rig and open the parameter status GUI at startup, run:

```bash
scripts/spacewizard_live.sh start --mode smooth-sine --status-gui
```
```

To boot directly into the new ambient flow mode instead, run:

```bash
scripts/spacewizard_live.sh start --mode ambient-flow
```

To boot directly into the new detuned saw Reese bass mode instead, run:

```bash
scripts/spacewizard_live.sh start --mode reese-bass
```

To boot directly into the new vocal-like wow mode instead, run:

```bash
scripts/spacewizard_live.sh start --mode wow-voice
```

## Next Integration Step

When you are ready, the bridge can send a second OSC stream to SuperCollider alongside Maestro:
- pickup event as a one-shot trigger
- interaction gate as the open or closed state
- normalized motion parameters derived from the gyro and accelerometer stream
