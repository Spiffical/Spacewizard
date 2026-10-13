// SpaceWizard live sound preset configuration.
//
// This file is loaded by scripts/spacewizard_live.sh before the live mode starts.
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
    // - --smooth-preset standard             -> \standard
    // - --smooth-preset quiet-pentatonic     -> \quietPentatonic
    // - --smooth-preset pi-quiet-pentatonic  -> \piQuietPentatonic
    smoothSine: (
        standard: (
            // Triggering mode options: \triggeredNotes, \trackedMono, \directMono.
            smoothExperiment: \triggeredNotes,
            smoothRootMidi: 38,             // MIDI root for pentatonic note set
            smoothPitchSmoothing: 0.34,     // 0.0 to 1.0
            smoothIndexHysteresis: 0.22,    // 0.0 to 1.0
            smoothTriggerUseRawPitch: false, // true makes note crossings follow unsmoothed pitch
            smoothGlide: 0.0,               // mono modes only; pitch glide in seconds

            // Quiet synth/pentatonic note level. Typical range 0.0 to 1.2.
            smoothAmpScale: 1.0,
            smoothAmpFollowsActivity: true, // true maps note volume to motion speed
            smoothFixedAmp: 0.10,           // used when smoothAmpFollowsActivity is false
            smoothHold: 0.06,               // seconds
            smoothDecay: 2.6,               // seconds
            smoothDecayMin: 1.7,            // live motion decay at low activity, seconds
            smoothDecayMax: 3.2,            // live motion decay at high activity, seconds
            smoothDecayYawMin: -0.1,        // yaw contribution at yaw 0.0, seconds
            smoothDecayYawMax: 1.0,         // yaw contribution at yaw 1.0, seconds
            smoothMinInterval: 0.07,        // seconds between triggered notes
            smoothSnapBypassInterval: true, // true lets large pitch jumps trigger immediately
            smoothFastJumpThreshold: 0.55,  // normalized pitch jump that counts as violent motion
            smoothFastMinInterval: 0.10,    // minimum interval during violent pitch jumps
            smoothFastJumpAmpScale: 0.85,   // note amp scale during violent pitch jumps
            smoothFastJumpDecayScale: 0.85, // note decay scale during violent pitch jumps
            smoothTailBudgetVoices: 12,     // max overlapping smooth note tails, 1 to 24
            smoothDensityReferenceVoices: 6, // where tail thinning reaches its reference point, 1 to 24

            // Roll tone defaults and roll-response range.
            // smoothRollToneMaxRoll is the roll amount that reaches the max values below.
            smoothCrunch: 0.0,             // 0.0 to 0.65
            smoothReverb: 0.14,            // 0.0 to 0.8
            smoothRollToneMaxRoll: 0.65,   // 0.01 to 1.0
            smoothRollCrunchMin: 0.0,      // 0.0 to 0.65
            smoothRollCrunchMax: 0.18,     // 0.0 to 0.65
            smoothRollReverbMin: 0.14,     // 0.0 to 0.8
            smoothRollReverbMax: 0.34,     // 0.0 to 0.8

            // Bass clarity curve. As candle pitch moves downward, this trims
            // sub/low-mid buildup while preserving the note and effects.
            smoothBassClarityAmount: 0.24,      // 0.0 none to 1.0 strongest
            smoothBassClarityPitchStart: -0.2,  // pitchNorm where cleanup starts
            smoothBassClarityPitchFull: -0.9,   // pitchNorm where cleanup reaches full strength
            smoothBassClarityHpMax: 54.0,       // Hz high-pass at full cleanup, 30 to 140
            smoothBassClarityMudFreq: 180.0,    // Hz center of low-mid cut, 90 to 420
            smoothBassClarityMudCutDb: 2.5,     // dB cut at full cleanup, 0 to 12
            smoothLowPitchAmpBoost: 0.0,        // extra amp at low pitch; 0.5 means +50%
            smoothLowPitchAmpStart: 0.0,        // pitchNorm where low-note boost starts
            smoothLowPitchAmpFull: -0.85,       // pitchNorm where low-note boost reaches full strength

            // Delay/rise for quiet synth + ambient after the candle gate opens.
            smoothLayerFadeDelaySeconds: 0.0, // 0.0 to 10.0
            smoothLayerFadeInSeconds: 0.18,   // 0.01 to 12.0

            // Smooth ambient bed.
            smoothAmbientEnabled: true,
            smoothAmbientYawBlendEnabled: true,
            smoothAmbientAmpScale: 0.58,    // 0.0 to 3.0
            smoothAmbientAmpCeiling: 0.24,  // 0.0 to 0.5
            smoothAmbientAmpFollowsActivity: true, // true maps bed volume to motion speed
            smoothAmbientFixedAmp: 0.10,    // pre-scale bed amp when not following activity
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
            smoothChoirFxMix: 0.32,        // choir sample/synth reverb halo, 0.0 to 1.0
            smoothChoirDelayMix: 0.14,     // choir echo trail, 0.0 to 1.0
            smoothChoirAir: 0.36,          // airy high shimmer on choir, 0.0 to 1.0
            smoothChoirBassAmp: 0.08,      // soft bass support behind choir, 0.0 to 0.45
            smoothChoirBassTone: 0.42,     // 0.0 round/dark to 1.0 brighter/clearer
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
            smoothChoirOctaveUp3: 0,

            // Stand failsafe prompt. Plays only after the candle settles upright.
            standPromptEnabled: true,
            standPromptPitchMin: 0.86,      // normalized pitch: 1.0 is +90 degrees
            standPromptDelaySeconds: 2.5,   // extra seconds after the motion/music fade-out completes
            standPromptAmp: 1.0,            // 0.0 to 1.2
            standPromptStopActivity: 0.12,  // jitter guard; gate-open still stops prompt immediately
            standPromptRequirePriorMotion: true // true waits until moved/off-stand after boot
        ),

        quietPentatonic: (
            smoothExperiment: \triggeredNotes,
            smoothRootMidi: 38,
            smoothPitchSmoothing: 0.78,     // higher is faster; live mode now preserves this value
            smoothIndexHysteresis: 0.08,    // lower reacts closer to pitch boundaries
            smoothTriggerUseRawPitch: true, // trigger from raw pitch, not delayed smoothed pitch
            smoothGlide: 0.0,               // mono pitch glide, seconds
            smoothHold: 0.06,
            smoothDecay: 2.6,
            smoothDecayMin: 1.7,            // live motion decay at low activity, seconds
            smoothDecayMax: 3.2,            // live motion decay at high activity, seconds
            smoothDecayYawMin: -0.1,        // yaw contribution at yaw 0.0, seconds
            smoothDecayYawMax: 1.0,         // yaw contribution at yaw 1.0, seconds
            smoothMinInterval: 0.025,
            smoothSnapBypassInterval: true,
            smoothFastJumpThreshold: 0.55,
            smoothFastMinInterval: 0.10,
            smoothFastJumpAmpScale: 0.85,
            smoothFastJumpDecayScale: 0.85,
            smoothTailBudgetVoices: 12,
            smoothDensityReferenceVoices: 6,

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

            // Bass clarity curve. More cleanup is applied as candle pitch
            // moves downward into the bass notes.
            smoothBassClarityAmount: 0.68,      // 0.0 none to 1.0 strongest
            smoothBassClarityPitchStart: 0.05,  // pitchNorm where cleanup starts
            smoothBassClarityPitchFull: -0.86,  // pitchNorm where cleanup reaches full strength
            smoothBassClarityHpMax: 62.0,       // Hz high-pass at full cleanup, 30 to 140
            smoothBassClarityMudFreq: 180.0,    // Hz center of low-mid cut, 90 to 420
            smoothBassClarityMudCutDb: 4.0,     // dB cut at full cleanup, 0 to 12
            smoothLowPitchAmpBoost: 0.0,        // restores audibility below 0 degrees pitch
            smoothLowPitchAmpStart: 0.0,
            smoothLowPitchAmpFull: -0.85,

            // Choir + accompanying synth accent.
            smoothChoirEnabled: true,
            smoothChoirSolo: false,
            smoothChoirAmp: 0.37,           // choir sample level, 0.0 to 0.6
            smoothChoirSynthAmp: 0.0,       // synth octave layer level, 0.0 to 0.8; 0 disables it
            smoothChoirSpread: 0.93,        // 0.1 to 1.0
            smoothChoirShimmer: 0.95,       // 0.0 to 1.0
            smoothChoirBreath: 0.10,        // 0.0 to 0.18
            smoothChoirBrightness: 0.96,    // 0.0 to 1.0
            smoothChoirFxMix: 1.0,          // choir sample/synth reverb halo, 0.0 to 1.0
            smoothChoirDelayMix: 0.99,      // choir echo trail, 0.0 to 1.0
            smoothChoirAir: 0.94,           // airy high shimmer on choir, 0.0 to 1.0
            smoothChoirBassAmp: 0.31,       // soft bass support behind choir, 0.0 to 0.45
            smoothChoirBassTone: 0.57,      // 0.0 round/dark to 1.0 brighter/clearer
            smoothChoirSampleStart: 0.0,    // seconds into sample
            smoothChoirTexture: 1,          // 0 Cathedral, 1 Flow, 2 Ribbon, 3 Glass, 4 Vowel, 5 Chime
            smoothChoirFadeIn: 0.5,         // seconds
            smoothChoirHold: 0.28,          // seconds
            smoothChoirFadeOut: 4.2,        // seconds

            // Choir synth octave layers, each 0 off or 1 on.
            smoothChoirOctaveDown3: 0,
            smoothChoirOctaveDown2: 1,
            smoothChoirOctaveDown1: 1,
            smoothChoirOctaveUp1: 0,
            smoothChoirOctaveUp2: 0,
            smoothChoirOctaveUp3: 0,

            // Stand failsafe prompt. If the candle becomes motionless while
            // upright on the stand, wait for the music to fade out, then play
            // the instruction sample. Any renewed motion stops it.
            standPromptEnabled: false,
            standPromptPitchMin: 0.86,      // normalized pitch: 1.0 is +90 degrees
            standPromptDelaySeconds: 2.5,   // extra seconds after the motion/music fade-out completes
            standPromptAmp: 1.0,            // 0.0 to 1.2
            standPromptStopActivity: 0.12,  // jitter guard; gate-open still stops prompt immediately
            standPromptRequirePriorMotion: true // true waits until moved/off-stand after boot
        ),

        piQuietPentatonic: (
            smoothExperiment: \trackedMono,
            smoothRootMidi: 38,
            smoothPitchSmoothing: 0.68,     // higher is faster; live mode now preserves this value
            smoothIndexHysteresis: 0.12,    // lower reacts closer to pitch boundaries
            smoothTriggerUseRawPitch: true, // trigger from raw pitch, not delayed smoothed pitch
            smoothGlide: 0.035,             // mono pitch glide, seconds
            smoothHold: 0.04,
            smoothDecay: 1.15,
            smoothDecayMin: 0.75,           // live motion decay at low activity, seconds
            smoothDecayMax: 1.20,           // live motion decay at high activity, seconds
            smoothDecayYawMin: -0.03,       // yaw contribution at yaw 0.0, seconds
            smoothDecayYawMax: 0.12,        // yaw contribution at yaw 1.0, seconds
            smoothMinInterval: 0.09,        // higher lowers Pi CPU during fast pitch sweeps
            smoothSnapBypassInterval: false, // keep fast sweeps from bypassing the Pi throttle
            smoothFastJumpThreshold: 0.42,  // fast pitch jumps get softened to avoid Pi audio edge
            smoothFastMinInterval: 0.16,
            smoothFastJumpAmpScale: 1.0,
            smoothFastJumpDecayScale: 0.68,
            smoothTailBudgetVoices: 5,      // cap overlapping tails so fast moves keep newest notes
            smoothDensityReferenceVoices: 3,

            // Pi-safe quiet synth + ambient balance.
            smoothAmpScale: 1.03,           // quiet pentatonic notes, 0.0 to 1.2
            smoothAmpFollowsActivity: false,
            smoothFixedAmp: 0.11,
            smoothAmbientEnabled: true,
            smoothAmbientYawBlendEnabled: true,
            smoothAmbientAmpScale: 3.6,
            smoothAmbientAmpCeiling: 0.43,
            smoothAmbientAmpFollowsActivity: false,
            smoothAmbientFixedAmp: 0.12,
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

            // Fast rise for the Pi continuous quiet synth + ambient layer.
            smoothLayerFadeDelaySeconds: 0.05,
            smoothLayerFadeInSeconds: 0.18,

            // Roll changes the quiet synth tone.
            smoothCrunch: 0.0,
            smoothReverb: 0.14,
            smoothRollToneMaxRoll: 0.65,
            smoothRollCrunchMin: 0.0,
            smoothRollCrunchMax: 0.34,
            smoothRollReverbMin: 0.12,
            smoothRollReverbMax: 0.72,

            // Bass clarity plus low-note compensation tuned for the Pi output.
            smoothBassClarityAmount: 0.42,
            smoothBassClarityPitchStart: 0.05,
            smoothBassClarityPitchFull: -0.86,
            smoothBassClarityHpMax: 48.0,
            smoothBassClarityMudFreq: 180.0,
            smoothBassClarityMudCutDb: 2.2,
            smoothLowPitchAmpBoost: 0.80,
            smoothLowPitchAmpStart: 0.0,
            smoothLowPitchAmpFull: -0.85,

            // Choir + accompanying synth accent.
            smoothChoirEnabled: true,
            smoothChoirSolo: false,
            smoothChoirAmp: 0.18,           // was 0.37; pickup choir accent lowered twice by ~30%
            smoothChoirSynthAmp: 0.0,
            smoothChoirSpread: 0.93,
            smoothChoirShimmer: 0.95,
            smoothChoirBreath: 0.10,
            smoothChoirBrightness: 0.96,
            smoothChoirFxMix: 1.0,
            smoothChoirDelayMix: 0.99,
            smoothChoirAir: 0.94,
            smoothChoirBassAmp: 0.31,
            smoothChoirBassTone: 0.57,
            smoothChoirSampleStart: 0.0,
            smoothChoirTexture: 1,
            smoothChoirFadeIn: 0.5,
            smoothChoirHold: 0.28,
            smoothChoirFadeOut: 4.2,

            // Choir synth octave layers, each 0 off or 1 on.
            smoothChoirOctaveDown3: 0,
            smoothChoirOctaveDown2: 1,
            smoothChoirOctaveDown1: 1,
            smoothChoirOctaveUp1: 0,
            smoothChoirOctaveUp2: 0,
            smoothChoirOctaveUp3: 0,

            // Stand failsafe prompt for the final Pi installation.
            standPromptEnabled: true,
            standPromptPitchMin: 0.75,
            standPromptDelaySeconds: 2.5,
            standPromptAmp: 1.0,
            standPromptStopActivity: 0.12,
            standPromptRequirePriorMotion: true
        )
    ),

    // Song mode. CLI: --mode day-o
    // Put the recording in samples/song (wav, aiff or flac). On every pickup it plays
    // from songStartSeconds, and it fades out when the candle is set down.
    song: (
        dayO: (
            songFile: "day-o.wav",          // file name in samples/song; falls back to the first audio file there
            songStartSeconds: 0.0,          // cue point in seconds; day-o.wav is already trimmed to the call (3.63 s into the CD track)
            songAmp: 0.2,                   // 0.0 to 1.0; 0.2 plays the song about 2 dB louder than the smooth-sine rig
            songFadeInSeconds: 0.01,        // near-instant start so the first word lands on pickup
            songFadeOutSeconds: 1.0,        // final release when the song stops or input is lost, 0.01 to 5.0 seconds

            // Wind-down. The bridge's song energy (config/bridge.json "song_gate") averages how much
            // the candle moves and falls to zero over about 3 s when it is held still.
            songFullSpeedEnergy: 0.8,       // energy at or above this plays at normal speed and pitch
            songMinRate: 0.2,               // speed (and pitch) as the energy nears zero; 0.5 is an octave down
            songFadeBelowEnergy: 0.25,      // volume fades to silence as the energy drops below this

            // Stand failsafe prompt, same as the Pi installation preset.
            standPromptEnabled: true,
            standPromptPitchMin: 0.75,
            standPromptDelaySeconds: 2.5,
            standPromptAmp: 1.0,
            standPromptStopActivity: 0.12,
            standPromptRequirePriorMotion: true
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
