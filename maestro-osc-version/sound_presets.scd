// SpaceWizard live sound preset configuration.
//
// This file is loaded by start_spacewizard_live.sh before the live mode starts.
// It is SuperCollider syntax, so comments are allowed and symbols use a leading
// backslash, for example \flow or \triggeredNotes.
//
// Ranges used below:
// - amp / level style controls: usually 0.0 to 1.0, but keep live output below 0.3 unless noted.
// - pan: -1.0 left to 1.0 right.
// - normalized motion axes: pitch/roll -1.0 to 1.0, yaw/activity/spin 0.0 to 1.0.
// - seconds: non-negative seconds.
// - midi roots: MIDI note numbers.
//
// You can remove a key from a preset to fall back to the built-in default.
// Do not rename the top-level sections unless the synth code is updated too.

~swSoundPresetConfig = (
    // Smooth-sine presets. CLI names:
    // - --smooth-preset standard         -> \standard
    // - --smooth-preset quiet-pentatonic -> \quietPentatonic
    smoothSine: (
        standard: (
            // Triggering mode options: \triggeredNotes, \trackedMono, \directMono.
            smoothExperiment: \triggeredNotes,
            smoothRootMidi: 38,             // MIDI root for pentatonic note set
            smoothPitchSmoothing: 0.34,     // 0.0 to 1.0
            smoothIndexHysteresis: 0.22,    // 0.0 to 1.0

            // Quiet synth/pentatonic note level. Typical range 0.0 to 1.2.
            smoothAmpScale: 1.0,
            smoothHold: 0.06,               // seconds
            smoothDecay: 2.6,               // seconds
            smoothMinInterval: 0.07,        // seconds between triggered notes

            // Roll tone defaults and roll-response range.
            // smoothRollToneMaxRoll is the roll amount that reaches the max values below.
            smoothCrunch: 0.0,             // 0.0 to 0.65
            smoothReverb: 0.14,            // 0.0 to 0.8
            smoothRollToneMaxRoll: 0.65,   // 0.01 to 1.0
            smoothRollCrunchMin: 0.0,      // 0.0 to 0.65
            smoothRollCrunchMax: 0.18,     // 0.0 to 0.65
            smoothRollReverbMin: 0.14,     // 0.0 to 0.8
            smoothRollReverbMax: 0.34,     // 0.0 to 0.8

            // Delay/rise for quiet synth + ambient after the candle gate opens.
            smoothLayerFadeDelaySeconds: 0.0, // 0.0 to 10.0
            smoothLayerFadeInSeconds: 0.18,   // 0.01 to 12.0

            // Smooth ambient bed.
            smoothAmbientEnabled: true,
            smoothAmbientYawBlendEnabled: true,
            smoothAmbientAmpScale: 0.58,    // 0.0 to 3.0
            smoothAmbientAmpCeiling: 0.24,  // 0.0 to 0.5
            smoothAmbientPanScale: 0.6,     // 0.0 to 1.0
            smoothAmbientWidthScale: 0.92,  // 0.18 to 1.0
            smoothAmbientVariant: \flow,    // \flow, \chordal, \drone, \forest
            smoothAmbientRoot: 74,          // Hz root for ambient bed
            smoothAmbientYawOffset: 0.0,    // 0.0 to 1.0
            smoothAmbientLabEnabled: false,

            // Ambient drum hit triggered by impact/smack jerk detection.
            smoothImpactDrumEnabled: false,
            smoothImpactDrumAmp: 0.16,      // 0.0 to 0.6
            smoothImpactDrumFreq: 74,       // Hz, 32 to 180
            smoothImpactDrumDecay: 1.6,     // seconds, 0.25 to 4.5
            smoothImpactDrumTone: 0.42,     // 0.0 dark/round to 1.0 brighter skin
            smoothImpactDrumVerb: 0.72,     // 0.0 dry to 0.95 washed out
            smoothImpactDrumMaxJerk: 60.0,  // jerk g/s that maps to max hit force

            // Choir defaults for standard smooth mode.
            smoothChoirEnabled: true,
            smoothChoirSolo: false,
            smoothChoirAmp: 0.60,
            smoothChoirSynthAmp: 0.46,
            smoothChoirSpread: 0.78,
            smoothChoirShimmer: 0.12,
            smoothChoirBreath: 0.07,
            smoothChoirBrightness: 0.28,
            smoothChoirSampleStart: 0.0,
            smoothChoirTexture: 1,
            smoothChoirFadeIn: 0.5,
            smoothChoirHold: 0.28,
            smoothChoirFadeOut: 4.2,
            smoothChoirOctaveDown3: 0,
            smoothChoirOctaveDown2: 1,
            smoothChoirOctaveDown1: 1,
            smoothChoirOctaveUp1: 0,
            smoothChoirOctaveUp2: 0,
            smoothChoirOctaveUp3: 0
        ),

        quietPentatonic: (
            smoothExperiment: \triggeredNotes,
            smoothRootMidi: 38,
            smoothPitchSmoothing: 0.34,
            smoothIndexHysteresis: 0.22,
            smoothHold: 0.06,
            smoothDecay: 2.6,
            smoothMinInterval: 0.07,

            // Tuned quiet synth + ambient balance.
            smoothAmpScale: 0.439,          // quiet pentatonic notes, 0.0 to 1.2
            smoothAmbientEnabled: true,
            smoothAmbientYawBlendEnabled: true,
            smoothAmbientAmpScale: 3.0,     // louder ambient bed
            smoothAmbientAmpCeiling: 0.24,
            smoothAmbientPanScale: 0.6,
            smoothAmbientWidthScale: 0.92,
            smoothAmbientVariant: \flow,
            smoothAmbientRoot: 74,
            smoothAmbientYawOffset: 0.0,
            smoothAmbientLabEnabled: false,

            // Ambient drum hit triggered by impact/smack jerk detection.
            smoothImpactDrumEnabled: false,
            smoothImpactDrumAmp: 0.16,
            smoothImpactDrumFreq: 74,
            smoothImpactDrumDecay: 1.6,
            smoothImpactDrumTone: 0.42,
            smoothImpactDrumVerb: 0.72,
            smoothImpactDrumMaxJerk: 60.0,

            // Delay/rise for quiet synth + ambient after the choir pickup accent.
            smoothLayerFadeDelaySeconds: 1.0,
            smoothLayerFadeInSeconds: 0.65,

            // Roll changes the quiet synth tone.
            smoothCrunch: 0.0,
            smoothReverb: 0.14,
            smoothRollToneMaxRoll: 0.65,
            smoothRollCrunchMin: 0.0,
            smoothRollCrunchMax: 0.58,
            smoothRollReverbMin: 0.12,
            smoothRollReverbMax: 0.72,

            // Choir + accompanying synth accent.
            smoothChoirEnabled: true,
            smoothChoirSolo: false,
            smoothChoirAmp: 0.46,           // choir sample level, 0.0 to 0.6
            smoothChoirSynthAmp: 0.74,      // synth octave layer level, 0.0 to 0.8
            smoothChoirSpread: 0.78,        // 0.1 to 1.0
            smoothChoirShimmer: 0.12,       // 0.0 to 1.0
            smoothChoirBreath: 0.07,        // 0.0 to 0.18
            smoothChoirBrightness: 0.28,    // 0.0 to 1.0
            smoothChoirSampleStart: 0.0,    // seconds into sample
            smoothChoirTexture: 4,          // 0 Cathedral, 1 Flow, 2 Ribbon, 3 Glass, 4 Vowel, 5 Chime
            smoothChoirFadeIn: 0.5,         // seconds
            smoothChoirHold: 0.28,          // seconds
            smoothChoirFadeOut: 4.2,        // seconds

            // Choir synth octave layers, each 0 off or 1 on.
            smoothChoirOctaveDown3: 0,
            smoothChoirOctaveDown2: 1,
            smoothChoirOctaveDown1: 1,
            smoothChoirOctaveUp1: 0,
            smoothChoirOctaveUp2: 0,
            smoothChoirOctaveUp3: 0
        )
    ),

    // Ambient-flow presets. CLI: --mode ambient-flow --ambient-variant flow|chordal|drone|forest.
    ambientFlow: (
        flow: (
            ambientVariant: \flow,
            ambientRoot: 74,
            ambientLift: -0.1,      // -0.35 to 0.35
            ambientLean: -0.15,     // -1.0 to 1.0
            ambientMotion: 0.22,    // 0.04 to 1.0
            ambientSpin: 0.14,      // 0.02 to 1.0
            ambientAura: 0.62,      // 0.0 to 1.0
            ambientDarkness: 0.66,  // 0.0 to 1.0
            ambientShimmer: 0.34,   // 0.0 to 1.0
            ambientWidth: 0.78,     // 0.18 to 1.0
            amp: 0.1,
            pan: 0.0
        ),
        chordal: (
            ambientVariant: \chordal,
            ambientRoot: 82,
            ambientLift: -0.08,
            ambientLean: 0.12,
            ambientMotion: 0.16,
            ambientSpin: 0.12,
            ambientAura: 0.78,
            ambientDarkness: 0.4,
            ambientShimmer: 0.24,
            ambientWidth: 0.74,
            amp: 0.11,
            pan: 0.0
        ),
        drone: (
            ambientVariant: \drone,
            ambientRoot: 58,
            ambientLift: -0.34,
            ambientLean: -0.24,
            ambientMotion: 0.18,
            ambientSpin: 0.08,
            ambientAura: 0.72,
            ambientDarkness: 0.84,
            ambientShimmer: 0.12,
            ambientWidth: 0.66,
            amp: 0.1,
            pan: 0.0
        ),
        forest: (
            ambientVariant: \forest,
            ambientRoot: 66,
            ambientLift: 0.04,
            ambientLean: 0.08,
            ambientMotion: 0.36,
            ambientSpin: 0.34,
            ambientAura: 0.78,
            ambientDarkness: 0.54,
            ambientShimmer: 0.48,
            ambientWidth: 0.9,
            amp: 0.11,
            pan: 0.0
        )
    ),

    // Auduino grit mode. CLI: --mode auduino.
    auduino: (
        standard: (
            syncFreq: 42,        // Hz
            grain1Freq: 196,     // Hz
            grain2Freq: 294,     // Hz
            amp: 0.12,          // 0.0 to 0.4
            pan: 0.0,
            drive: 2.6,         // 1.0 to 6.0
            crushBits: 7,       // 5 to 10
            hp: 60,             // Hz
            lp: 4600,           // Hz
            decay1: 0.14,       // seconds
            decay2: 0.09,       // seconds
            rootMidi: 50,
            stepIndex: 10,
            scaleMode: \originalAuduino,
            playMode: \continuous
        )
    ),

    // Reese bass mode. CLI: --mode reese-bass.
    reeseBass: (
        standard: (
            amp: 0.12,
            pan: 0.0,
            reeseFreq: 50,          // Hz
            reeseDetune: 0.028,     // 0.0 to 0.12
            reeseGlide: 0.32,       // seconds
            reeseWobbleRate: 0.18,  // Hz
            reeseWobbleDepth: 0.12, // 0.0 to 0.5
            reeseDrive: 2.8,        // 1.0 to 6.0
            reeseLp: 900,           // Hz
            reeseRq: 0.24,          // 0.05 to 0.8
            reeseSubMix: 0.46       // 0.0 to 1.0
        )
    ),

    // Wow voice mode. CLI: --mode wow-voice.
    wowVoice: (
        standard: (
            amp: 0.12,
            pan: 0.0,
            wowFreq: 110,          // Hz
            wowGlide: 0.28,        // seconds
            wowVowelDepth: 0.82,   // 0.3 to 1.15
            wowBreath: 0.035,      // 0.0 to 0.12
            wowLp: 2400,           // Hz
            wowTone: 0.16          // 0.0 to 0.5
        )
    )
);

~swSoundPresetConfig;
