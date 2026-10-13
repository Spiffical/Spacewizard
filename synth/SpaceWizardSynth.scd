(
// SpaceWizard synth patch.
// Evaluate this entire file in SuperCollider to boot the server,
// load the synth definitions, install OSC listeners, and expose
// helper functions for manual testing.

s.waitForBoot {
    SynthDef(\spacewizardVoice, {
        |out=0, gate=1, amp=0.18, activity=0.0, tilt=0.0, roll=0.0, spin=0.0, presence=0.55,
         motionBus=0, motionRise=3.2, motionFall=5.0|
        var gateEnv = EnvGen.kr(Env.asr(1.8, 1.0, 4.5), gate, doneAction: 2);
        var motionLevel = LagUD.kr(In.kr(motionBus, 1).clip(0.0, 1.0), motionRise.clip(0.01, 12.0), motionFall.clip(0.01, 12.0));
        var activityLag = Lag.kr(Clip.kr(activity, 0.0, 1.0), 0.25);
        var tiltLag = Lag.kr(Clip.kr(tilt, -1.0, 1.0), 0.18);
        var rollLag = Lag.kr(Clip.kr(roll, -1.0, 1.0), 0.18);
        var spinLag = Lag.kr(Clip.kr(spin, 0.0, 1.0), 0.22);
        var presenceLag = Lag.kr(Clip.kr(presence, 0.0, 1.0), 0.3);
        var cutoffScale = LinLin.kr(tiltLag, -1.0, 1.0, 0.68, 1.45);
        var baseMidi = 42 + (tiltLag * 6.0) + (rollLag * 4.0);
        var baseFreq = baseMidi.midicps;
        var fmRate = LinExp.kr(spinLag, 0.0, 1.0, 0.08, 8.5);
        var fmIndex = LinLin.kr(activityLag, 0.0, 1.0, 0.15, 5.0) + (spinLag * 1.5);
        var fm = SinOsc.ar(fmRate * [1.0, 1.37], 0.0, fmIndex).sum;
        var drone = Mix(VarSaw.ar(
            baseFreq * [0.995, 1.0, 1.008] * (1.0 + (fm * 0.003)),
            0.0,
            0.42 + (rollLag * 0.14),
            [0.07, 0.06, 0.05]
        ));
        var sub = SinOsc.ar(baseFreq * 0.5, 0.0, 0.05);
        var shimmer = SinOsc.ar(baseFreq * [2.01, 3.0], 0.0, 0.025 + (presenceLag * 0.04)).sum;
        var air = BPF.ar(
            WhiteNoise.ar(0.015 + (activityLag * 0.025)),
            baseFreq * (3.5 + (presenceLag * 2.0)),
            0.18 + (spinLag * 0.15)
        );
        var body = drone + sub + shimmer + air;
        var cutoff = LinExp.kr(activityLag, 0.0, 1.0, 380.0, 5200.0) * cutoffScale;
        var resonance = LinLin.kr(presenceLag, 0.0, 1.0, 0.24, 0.1);
        var trem = SinOsc.kr(LinExp.kr(spinLag, 0.0, 1.0, 0.1, 11.0), 0.0).range(0.78, 1.0);
        var motionRipple = SinOsc.kr(0.13 + (activityLag * 0.9), 0.0).range(0.92, 1.08);
        var sig = RLPF.ar(body * motionRipple, cutoff.clip(150.0, 8000.0), resonance);
        sig = LeakDC.ar(sig * trem);
        sig = Splay.ar(sig, 0.18 + (rollLag.abs * 0.7) + (activityLag * 0.08), amp * gateEnv * motionLevel);
        sig = FreeVerb2.ar(sig[0], sig[1], 0.18 + (activityLag * 0.32), 0.86, 0.25 + (presenceLag * 0.25));
        Out.ar(out, sig);
    }).add;

    SynthDef(\spacewizardPickupBloom, {
        |out=0, amp=0.24, tilt=0.0, spin=0.0, brightness=0.55|
        var tiltClip = Clip.kr(tilt, -1.0, 1.0);
        var spinClip = Clip.kr(spin, 0.0, 1.0);
        var brightClip = Clip.kr(brightness, 0.0, 1.0);
        var env = EnvGen.kr(Env.perc(0.01, 4.8, 1.0, -4), doneAction: 2);
        var sweepStart = 160.0;
        var sweepEnd = (1300.0 + (brightClip * 2600.0)) * LinLin.kr(tiltClip, -1.0, 1.0, 0.8, 1.2);
        var sweep = XLine.kr(sweepStart, sweepEnd.max(sweepStart + 1.0), 2.3);
        var tone = SinOsc.ar([sweep, sweep * 1.49], 0.0, [0.05, 0.04]).sum;
        var breath = BPF.ar(WhiteNoise.ar(0.06), sweep * (1.4 + spinClip), 0.18);
        var thunk = LPF.ar(PinkNoise.ar(0.04), 180.0 + (brightClip * 140.0));
        var sig = HPF.ar(tone + breath + thunk, 55.0);
        sig = FreeVerb.ar(sig, 0.34, 0.88, 0.4);
        Out.ar(out, Pan2.ar(sig * env * amp, tiltClip * 0.45));
    }).add;

    SynthDef(\spacewizardAmbientDrum, {
        |out=0, amp=0.16, freq=74, decay=1.6, tone=0.42, verb=0.72, pan=0.0, force=1.0|
        var forceCtl = force.clip(0.25, 1.35);
        var bodyEnv = EnvGen.kr(Env.perc(0.004, decay.clip(0.25, 4.5), 1.0, -5), doneAction: 2);
        var clickEnv = EnvGen.kr(Env.perc(0.001, 0.055, 1.0, -8));
        var toneCtl = tone.clip(0.0, 1.0);
        var baseFreq = freq.clip(32.0, 180.0);
        var pitchSweep = XLine.kr((baseFreq * (1.55 + (forceCtl * 0.35))).clip(45.0, 360.0), baseFreq, 0.085);
        var body = SinOsc.ar(pitchSweep, 0.0, 0.82) + SinOsc.ar(baseFreq * 0.5, 0.0, 0.26);
        var skin = BPF.ar(WhiteNoise.ar(0.34), (baseFreq * (5.0 + (toneCtl * 14.0))).clip(260.0, 4200.0), 0.32) * clickEnv;
        var air = LPF.ar(PinkNoise.ar(0.09), (360.0 + (toneCtl * 1600.0)).clip(180.0, 2600.0)) * bodyEnv;
        var sig = ((body * bodyEnv) + skin + air).tanh;

        sig = LeakDC.ar(sig);
        sig = FreeVerb.ar(sig, verb.clip(0.0, 0.95), 0.88, 0.28);
        Out.ar(out, Pan2.ar(sig * amp.clip(0.0, 0.6) * forceCtl, pan.clip(-1.0, 1.0)));
    }).add;

    SynthDef(\spacewizardStandPrompt, {
        |out=0, buf=0, amp=1.0, pan=0.0, gate=1|
        var play = PlayBuf.ar(1, buf, BufRateScale.kr(buf), 1, 0, 0);
        var env = EnvGen.kr(Env.asr(0.015, 1.0, 0.08), gate, doneAction: 2);
        var sig = LeakDC.ar(play) * env * amp.clip(0.0, 1.2);

        FreeSelf.kr(Done.kr(play));
        Out.ar(out, Pan2.ar(sig, pan.clip(-1.0, 1.0)));
    }).add;

    // Song mode players. The buffer is loaded from the song's cue point, so playback
    // starts at frame 0. `cut` quickly silences a fading copy on a fast re-pickup.
    SynthDef(\spacewizardSongPlayer1, {
        |out=0, buf=0, amp=0.4, gate=1, fadeIn=0.01, fadeOut=2.5, cut=0, rate=1|
        // rate changes speed and pitch together, like a turntable slowing down.
        var rateCtl = Lag.kr(rate.clip(0.05, 2.0), 0.08);
        var ampCtl = Lag.kr(amp.clip(0.0, 1.0), 0.08);
        var play = PlayBuf.ar(1, buf, BufRateScale.kr(buf) * rateCtl, 1, 0, 0);
        var env = EnvGen.kr(Env.asr(fadeIn.max(0.001), 1.0, fadeOut.max(0.01), -2), gate, doneAction: 2);
        var cutEnv = Lag.kr(1.0 - cut.clip(0.0, 1.0), 0.04);

        FreeSelf.kr(Done.kr(play));
        Out.ar(out, Pan2.ar(play * env * cutEnv * ampCtl, 0.0));
    }).add;

    SynthDef(\spacewizardSongPlayer2, {
        |out=0, buf=0, amp=0.4, gate=1, fadeIn=0.01, fadeOut=2.5, cut=0, rate=1|
        // rate changes speed and pitch together, like a turntable slowing down.
        var rateCtl = Lag.kr(rate.clip(0.05, 2.0), 0.08);
        var ampCtl = Lag.kr(amp.clip(0.0, 1.0), 0.08);
        var play = PlayBuf.ar(2, buf, BufRateScale.kr(buf) * rateCtl, 1, 0, 0);
        var env = EnvGen.kr(Env.asr(fadeIn.max(0.001), 1.0, fadeOut.max(0.01), -2), gate, doneAction: 2);
        var cutEnv = Lag.kr(1.0 - cut.clip(0.0, 1.0), 0.04);

        FreeSelf.kr(Done.kr(play));
        Out.ar(out, play * env * cutEnv * ampCtl);
    }).add;

    SynthDef(\spacewizardSawLead, {
        |out=0, gate=1, freq=220, amp=0.12, pan=0.0, cutoff=2600, rq=0.22, detune=1.004, drive=4.8,
         glide=0.035, motionBus=0, motionRise=3.2, motionFall=5.0|
        var env = EnvGen.kr(Env.asr(0.005, 1.0, 0.08), gate, doneAction: 2);
        var motionLevel = LagUD.kr(In.kr(motionBus, 1).clip(0.0, 1.0), motionRise.clip(0.01, 12.0), motionFall.clip(0.01, 12.0));
        var freqLag = Lag.kr(freq.max(40), glide.max(0.001));
        var raw = Mix(Saw.ar(freqLag * [1.0, detune], [0.55, 0.45]));
        var sub = Pulse.ar(freqLag * 0.5, 0.48, 0.16);
        var sig = HPF.ar(raw + sub, 70);
        sig = (sig * drive).tanh;
        sig = sig + ((sig * (drive * 1.8)).fold2(0.55) * 0.18);
        sig = RLPF.ar(sig, cutoff.clip(180, 9000), rq.clip(0.05, 0.95));
        sig = sig + (BPF.ar(WhiteNoise.ar(0.012), cutoff * 1.9, 0.18) * 0.08);
        sig = LeakDC.ar(sig * amp * env * motionLevel);
        sig = FreeVerb.ar(sig, 0.14, 0.26, 0.12);
        Out.ar(out, Pan2.ar(sig, pan));
    }).add;

    SynthDef(\spacewizardAuduino, {
        |out=0, gate=1, amp=0.14, pan=0.0, syncFreq=84, grain1Freq=220, grain2Freq=330,
         decay1=0.38, decay2=0.24, drive=2.8, crushBits=7, hp=70, lp=5200,
         motionBus=0, motionRise=3.2, motionFall=5.0|
        var gateEnv = EnvGen.kr(Env.asr(0.005, 1.0, 0.06), gate, doneAction: 2);
        var motionLevel = LagUD.kr(In.kr(motionBus, 1).clip(0.0, 1.0), motionRise.clip(0.01, 12.0), motionFall.clip(0.01, 12.0));
        var syncPhase = Phasor.ar(0, syncFreq.max(0.1) / SampleRate.ir, 0.0, 1.0, 0.0);
        var syncReset = (HPZ1.ar(syncPhase) < 0);
        var grain1Phase = Phasor.ar(syncReset, grain1Freq.max(1.0) / SampleRate.ir, 0.0, 1.0, 0.0);
        var grain2Phase = Phasor.ar(syncReset, grain2Freq.max(1.0) / SampleRate.ir, 0.0, 1.0, 0.0);
        var t = Sweep.ar(syncReset);
        var coeff1 = LinLin.kr(decay1.clip(0.0, 1.0), 0.0, 1.0, 4.0, 150.0);
        var coeff2 = LinLin.kr(decay2.clip(0.0, 1.0), 0.0, 1.0, 4.0, 220.0);
        var env1 = exp((t * SampleRate.ir * coeff1 / 256.0).neg);
        var env2 = exp((t * SampleRate.ir * coeff2 / 256.0).neg);
        var tri1 = ((1.0 - ((grain1Phase * 2.0) - 1.0).abs) * 2.0) - 1.0;
        var tri2 = ((1.0 - ((grain2Phase * 2.0) - 1.0).abs) * 2.0) - 1.0;
        var sig = (tri1 * env1) + (tri2 * env2);
        var levels = (2 ** crushBits.clip(4, 12)) - 1;

        sig = (sig * drive).clip2(1.0);
        sig = (sig * levels).round / levels;
        sig = LeakDC.ar(sig);
        sig = HPF.ar(sig, hp.max(20));
        sig = LPF.ar(sig, lp.clip(400, 12000));
        sig = sig * amp * gateEnv * motionLevel;
        Out.ar(out, Pan2.ar(sig, pan));
    }).add;

    SynthDef(\spacewizardSmoothSine, {
        |out=0, gate=1, freq=72, amp=0.1, pan=0.0, vibratoRate=0.18, vibratoDepth=0.006,
         motionDepth=0.06, bloom=0.18, lp=2200, hold=0.06, decay=2.6, crunch=0.0, reverb=0.14,
         bassClarity=0.0, bassClarityHpMax=62.0, bassClarityMudFreq=180.0, bassClarityMudCutDb=4.0,
         motionBus=0, motionRise=3.2, motionFall=5.0|
        var gateEnv = LagUD.kr(gate.clip(0.0, 1.0), 0.005, 0.14);
        var noteEnv = EnvGen.kr(Env.linen(0.01, hold.clip(0.01, 0.4), decay.clip(0.2, 8.0), 1.0, -4), doneAction: 2);
        var motionLevel = LagUD.kr(In.kr(motionBus, 1).clip(0.0, 1.0), motionRise.clip(0.01, 12.0), motionFall.clip(0.01, 12.0));
        var baseFreq = freq.max(25.0);
        var clarityCtl = Lag.kr(bassClarity.clip(0.0, 1.0), 0.08);
        var clarityHp = clarityCtl.linlin(0.0, 1.0, 24.0, bassClarityHpMax.clip(30.0, 140.0));
        var mudCut = 0.0 - (clarityCtl * bassClarityMudCutDb.clip(0.0, 12.0));
        var vibrato = SinOsc.kr(vibratoRate.clip(0.02, 5.0), 0.0, vibratoDepth.clip(0.0, 0.08), 1.0);
        var motion = SinOsc.kr((vibratoRate * 0.35).clip(0.02, 2.0), 0.0)
            .range(1.0 - motionDepth.clip(0.0, 0.25), 1.0);
        var crunchCtl = Lag.kr(crunch.clip(0.0, 0.65), 0.18);
        var reverbCtl = Lag.kr(reverb.clip(0.0, 0.8), 0.22);
        var bloomCtl = Lag.kr(bloom.clip(0.0, 0.4), 0.16);
        var lpCtl = Lag.kr(lp.clip(180, 7000), 0.12);
        var fundamental = Mix(SinOsc.ar(baseFreq * vibrato * [0.999, 1.0, 1.002], 0.0, [0.22, 0.34, 0.22]));
        var sub = SinOsc.ar(baseFreq * 0.5 * (1.0 + LFNoise2.kr(0.08).bipolar(0.0025)), 0.0, 0.18 * (1.0 - (clarityCtl * 0.55)));
        var upper = SinOsc.ar(baseFreq * 2.0, 0.0, bloomCtl * (0.1 + (clarityCtl * 0.08)));
        var sig = (fundamental + sub + upper) * motion;
        var crushed = (sig * (1.0 + (crunchCtl * 9.0))).tanh * (1.0 + (crunchCtl * 0.35));

        sig = (sig * (1.0 - (crunchCtl * 0.65))) + (crushed * (crunchCtl * 0.9));
        sig = LPF.ar(sig, lpCtl);
        sig = HPF.ar(sig, clarityHp);
        sig = BPeakEQ.ar(sig, bassClarityMudFreq.clip(90.0, 420.0), 0.75, mudCut);
        sig = LeakDC.ar(sig) * noteEnv * gateEnv * amp * motionLevel;
        sig = FreeVerb.ar(sig, (reverbCtl + (bloomCtl * 0.16)).clip(0.0, 0.82), (0.34 + (bloomCtl * 0.25) + (reverbCtl * 0.7)).clip(0.1, 0.96), 0.12);
        sig = HPF.ar(sig, clarityHp * 0.72);
        sig = Limiter.ar(LeakDC.ar(sig), 0.92, 0.005);
        Out.ar(out, Pan2.ar(sig, pan));
    }).add;

    SynthDef(\spacewizardSmoothSineMono, {
        |out=0, gate=1, freq=72, amp=0.1, pan=0.0, vibratoRate=0.18, vibratoDepth=0.006,
         motionDepth=0.06, bloom=0.18, lp=2200, glide=0.0, crunch=0.0, reverb=0.14,
         bassClarity=0.0, bassClarityHpMax=62.0, bassClarityMudFreq=180.0, bassClarityMudCutDb=4.0,
         motionBus=0, motionRise=3.2, motionFall=5.0|
        var gateEnv = EnvGen.kr(Env.asr(0.005, 1.0, 0.08, -4), gate, doneAction: 2);
        var motionLevel = LagUD.kr(In.kr(motionBus, 1).clip(0.0, 1.0), motionRise.clip(0.01, 12.0), motionFall.clip(0.01, 12.0));
        var baseFreq = Lag.kr(freq.max(25.0), glide.clip(0.0, 0.2));
        var clarityCtl = Lag.kr(bassClarity.clip(0.0, 1.0), 0.08);
        var clarityHp = clarityCtl.linlin(0.0, 1.0, 24.0, bassClarityHpMax.clip(30.0, 140.0));
        var mudCut = 0.0 - (clarityCtl * bassClarityMudCutDb.clip(0.0, 12.0));
        var vibrato = SinOsc.kr(vibratoRate.clip(0.02, 5.0), 0.0, vibratoDepth.clip(0.0, 0.08), 1.0);
        var motion = SinOsc.kr((vibratoRate * 0.35).clip(0.02, 2.0), 0.0)
            .range(1.0 - motionDepth.clip(0.0, 0.25), 1.0);
        var crunchCtl = Lag.kr(crunch.clip(0.0, 0.65), 0.18);
        var reverbCtl = Lag.kr(reverb.clip(0.0, 0.8), 0.22);
        var bloomCtl = Lag.kr(bloom.clip(0.0, 0.4), 0.16);
        var lpCtl = Lag.kr(lp.clip(180, 7000), 0.12);
        var fundamental = Mix(SinOsc.ar(baseFreq * vibrato * [0.999, 1.0, 1.002], 0.0, [0.22, 0.34, 0.22]));
        var sub = SinOsc.ar(baseFreq * 0.5 * (1.0 + LFNoise2.kr(0.08).bipolar(0.0025)), 0.0, 0.18 * (1.0 - (clarityCtl * 0.55)));
        var upper = SinOsc.ar(baseFreq * 2.0, 0.0, bloomCtl * (0.1 + (clarityCtl * 0.08)));
        var sig = (fundamental + sub + upper) * motion;
        var crushed = (sig * (1.0 + (crunchCtl * 9.0))).tanh * (1.0 + (crunchCtl * 0.35));

        sig = (sig * (1.0 - (crunchCtl * 0.65))) + (crushed * (crunchCtl * 0.9));
        sig = LPF.ar(sig, lpCtl);
        sig = HPF.ar(sig, clarityHp);
        sig = BPeakEQ.ar(sig, bassClarityMudFreq.clip(90.0, 420.0), 0.75, mudCut);
        sig = LeakDC.ar(sig) * gateEnv * amp * motionLevel;
        sig = FreeVerb.ar(sig, (reverbCtl + (bloomCtl * 0.16)).clip(0.0, 0.82), (0.34 + (bloomCtl * 0.25) + (reverbCtl * 0.7)).clip(0.1, 0.96), 0.12);
        sig = HPF.ar(sig, clarityHp * 0.72);
        sig = Limiter.ar(LeakDC.ar(sig), 0.92, 0.005);
        Out.ar(out, Pan2.ar(sig, pan));
    }).add;

    SynthDef(\spacewizardReeseBass, {
        |out=0, gate=1, freq=54, amp=0.12, pan=0.0, detune=0.028, glide=0.32,
         wobbleRate=0.18, wobbleDepth=0.12, drive=2.8, lp=900, rq=0.24, subMix=0.46,
         motionBus=0, motionRise=3.2, motionFall=5.0|
        var gateEnv = EnvGen.kr(Env.asr(0.02, 1.0, 0.55), gate, doneAction: 2);
        var motionLevel = LagUD.kr(In.kr(motionBus, 1).clip(0.0, 1.0), motionRise.clip(0.01, 12.0), motionFall.clip(0.01, 12.0));
        var baseFreq = Lag.kr(freq.max(25.0), glide.clip(0.01, 1.5));
        var wobble = SinOsc.kr(wobbleRate.clip(0.02, 8.0), 0.0).range(1.0 - wobbleDepth.clip(0.0, 0.45), 1.0 + wobbleDepth.clip(0.0, 0.45));
        var saws = Mix(Saw.ar(baseFreq * wobble * [1.0 - detune, 1.0, 1.0 + detune], [0.19, 0.3, 0.19]));
        var sub = SinOsc.ar(baseFreq * 0.5, 0.0, subMix.clip(0.0, 0.7));
        var body = LeakDC.ar((saws + sub) * drive.clip(1.0, 6.0));
        var filtered = RLPF.ar(body.tanh, lp.clip(120, 5000), rq.clip(0.08, 0.95));
        var air = BPF.ar(WhiteNoise.ar(0.003), (lp * 1.45).clip(150, 4200), 0.2) * 0.025;
        var sig = LeakDC.ar((filtered + air) * amp * gateEnv * motionLevel);
        sig = FreeVerb.ar(sig, 0.06, 0.18, 0.05);
        Out.ar(out, Pan2.ar(sig, pan));
    }).add;

    SynthDef(\spacewizardWowVoice, {
        |out=0, gate=1, freq=110, amp=0.12, pan=0.0, glide=0.28, wowTrig=0,
         direction=1, vowelDepth=0.8, breath=0.035, lp=2600, tone=0.16,
         motionBus=0, motionRise=3.2, motionFall=5.0|
        var gateEnv = EnvGen.kr(Env.asr(0.02, 1.0, 0.45), gate, doneAction: 2);
        var motionLevel = LagUD.kr(In.kr(motionBus, 1).clip(0.0, 1.0), motionRise.clip(0.01, 12.0), motionFall.clip(0.01, 12.0));
        var baseFreq = Lag.kr(freq.max(45.0), glide.clip(0.04, 1.6));
        var dir = Lag.kr(direction.clip(-1.0, 1.0), 0.08);
        var wowEnv = EnvGen.kr(
            Env([0.0, 1.0, 0.28], [0.045, 1.75], [-4, -2]),
            Changed.kr(wowTrig)
        );
        var vowelOpen = (wowEnv * vowelDepth.clip(0.0, 1.2)).clip(0.0, 1.0);
        var source = Mix(VarSaw.ar(
            baseFreq * [0.5, 1.0, 1.01],
            0.0,
            [0.56, 0.44, 0.48],
            [0.08, 0.12, 0.07]
        ));
        var f1 = LinExp.kr(vowelOpen, 0.0, 1.0, 260.0, 760.0) * (1.0 + (dir * 0.06));
        var f2 = LinExp.kr(vowelOpen, 0.0, 1.0, 540.0, 1320.0) * (1.0 - (dir * 0.04));
        var f3 = LinExp.kr(vowelOpen, 0.0, 1.0, 1180.0, 2550.0) * (1.0 + (dir * 0.03));
        var voiced = Mix(BPF.ar(source, [f1, f2, f3], [0.12, 0.09, 0.08]) * [1.15, 0.7, 0.28]);
        var body = SinOsc.ar(baseFreq * [1.0, 0.5], 0.0, [0.07, 0.05]).sum;
        var air = HPF.ar(PinkNoise.ar(breath.clip(0.0, 0.12)), 420) * (0.18 + (vowelOpen * 0.1));
        var sig = voiced + body + air;

        sig = LPF.ar(sig, lp.clip(500, 5200));
        sig = LeakDC.ar(sig * (1.0 + (tone.clip(0.0, 0.5) * 0.8)));
        sig = sig * amp * gateEnv * motionLevel;
        sig = FreeVerb.ar(sig, 0.1, 0.25, 0.08);
        Out.ar(out, Pan2.ar(sig, pan));
    }).add;

    SynthDef(\spacewizardHeavenlyChoirSynth, {
        |out=0, gate=1, rootFreq=220, amp=0.22, pan=0.0, spread=0.78,
         shimmer=0.12, breath=0.07, brightness=0.28, choirTrig=0,
         choirFxMix=0.32, choirDelayMix=0.14, choirAir=0.36, choirBassAmp=0.0, choirBassTone=0.42,
         fadeIn=0.5, holdTime=0.28, fadeOut=4.2,
         motionBus=0, motionRise=3.2, motionFall=5.0|
        var gateEnv = EnvGen.kr(Env.asr(0.01, 1.0, 0.18), gate, doneAction: 2);
        var motionLevel = LagUD.kr(In.kr(motionBus, 1).clip(0.0, 1.0), motionRise.clip(0.01, 12.0), motionFall.clip(0.01, 12.0));
        var choirEnv = EnvGen.kr(
            Env([0.0, 1.0, 0.9, 0.0], [fadeIn.clip(0.01, 1.5), holdTime.clip(0.0, 3.0), fadeOut.clip(0.2, 9.0)], [-4, -1, -5]),
            Trig1.kr(Changed.kr(choirTrig), 0.03)
        );
        var root = Lag.kr(rootFreq.clip(120.0, 620.0), 0.16);
        var ampCtl = Lag.kr(amp.clip(0.0, 0.34), 0.12);
        var spreadCtl = Lag.kr(spread.clip(0.1, 1.0), 0.25);
        var shimmerCtl = Lag.kr(shimmer.clip(0.0, 1.0), 0.25);
        var breathCtl = Lag.kr(breath.clip(0.0, 0.2), 0.18);
        var brightCtl = Lag.kr(brightness.clip(0.0, 1.0), 0.2);
        var fxMixCtl = Lag.kr(choirFxMix.clip(0.0, 1.0), 0.22);
        var delayMixCtl = Lag.kr(choirDelayMix.clip(0.0, 1.0), 0.2);
        var airCtl = Lag.kr(choirAir.clip(0.0, 1.0), 0.18);
        var bassAmpCtl = Lag.kr(choirBassAmp.clip(0.0, 0.45), 0.12);
        var bassToneCtl = Lag.kr(choirBassTone.clip(0.0, 1.0), 0.18);
        var singers = [0.5, 0.75, 1.0, 1.0, 1.26, 1.5].collect { |ratio, index|
            var singerFreq = (root * ratio).clip(120.0, 980.0);
            var vibrato = SinOsc.kr(
                4.8 + (index * 0.15) + LFNoise2.kr(0.05 + (index * 0.01)).range(-0.12, 0.12),
                0.0,
                0.0024 + (shimmerCtl * 0.0014),
                1.0
            );
            var detune = 1.0 + LFNoise2.kr(0.08 + (index * 0.017)).bipolar(0.0028 + (shimmerCtl * 0.0014));
            var voicedFreq = singerFreq * vibrato * detune;
            var aspiration = HPF.ar(PinkNoise.ar((breathCtl * 0.15) + (airCtl * 0.01) + 0.002), 900.0)
                * (0.06 + (shimmerCtl * 0.03));
            var formantShift = 1.0 + LFNoise2.kr(0.04 + (index * 0.015)).bipolar(0.05);
            var ooBody = Mix([
                Formant.ar(voicedFreq, 320.0 * formantShift, 90.0, 0.22),
                Formant.ar(voicedFreq, (820.0 + (brightCtl * 90.0)) * formantShift, 130.0, 0.12),
                Formant.ar(voicedFreq, (2280.0 + (brightCtl * 140.0)) * formantShift, 180.0, 0.024)
            ]);
            var warmFold = LPF.ar(
                Mix(VarSaw.ar(voicedFreq * [1.0, 2.0], 0.0, [0.8, 0.54], [0.022, 0.01])),
                1800.0 + (brightCtl * 700.0)
            ) * 0.14;
            var chest = SinOsc.ar(voicedFreq * 0.5, 0.0, 0.018);
            var presence = BPF.ar(aspiration, 1400.0 + (brightCtl * 600.0), 0.24, 0.04);

            ooBody + warmFold + chest + presence
        };
        var dry = Splay.ar(singers, spreadCtl, ampCtl * choirEnv, pan);
        var echo = CombC.ar(
            dry * 0.6,
            0.7,
            [0.19, 0.27] + (spreadCtl * 0.04) + (shimmerCtl * 0.02),
            3.2 + (shimmerCtl * 1.8)
        ) * (0.22 + (delayMixCtl * 0.52));
        var bassFreq = (root * 0.5).clip(40.0, 180.0);
        var bass = RLPF.ar(
            (SinOsc.ar(bassFreq, 0.0, 0.58) + VarSaw.ar(bassFreq, 0.0, 0.42, 0.16)).tanh,
            bassToneCtl.linexp(0.0, 1.0, 110.0, 840.0),
            0.32
        ) * bassAmpCtl * choirEnv;
        var halo;
        var sig;

        dry = HPF.ar(LPF.ar(dry, brightCtl.linexp(0.0, 1.0, 1300.0, 3600.0)), 110.0);
        dry = BHiShelf.ar(dry, 2400.0, 0.65, airCtl.linlin(0.0, 1.0, -1.5, 4.5));
        halo = FreeVerb2.ar(
            (dry[0] * 0.28) + echo[0],
            (dry[1] * 0.28) + echo[1],
            0.32 + (fxMixCtl * 0.32) + (shimmerCtl * 0.08),
            0.96,
            0.72 - (brightCtl * 0.12)
        );
        sig = LeakDC.ar(((dry * 1.08) + echo + (halo * (0.34 + (fxMixCtl * 0.58))) + Pan2.ar(bass, pan * 0.25)) * gateEnv * motionLevel);
        Out.ar(out, sig);
    }).add;

    SynthDef(\spacewizardHeavenlyChoirSamples, {
        |out=0, gate=1, rootFreq=220, amp=0.60, synthAmp=0.46, pan=0.0, spread=0.78,
         shimmer=0.12, breath=0.07, brightness=0.28, choirTrig=0,
         choirFxMix=0.32, choirDelayMix=0.14, choirAir=0.36, choirBassAmp=0.0, choirBassTone=0.42,
         sampleStart=0.0, textureMode=1, basePitchHz=176.47,
         fadeIn=0.5, holdTime=0.28, fadeOut=4.2,
         octaveDown3=0, octaveDown2=1, octaveDown1=1, octaveUp1=0, octaveUp2=0, octaveUp3=0,
         buf0=0, buf1=1, buf2=2, buf3=3, buf4=4,
         motionBus=0, motionRise=3.2, motionFall=5.0|
        var gateEnv = EnvGen.kr(Env.asr(0.01, 1.0, 0.18), gate, doneAction: 2);
        var motionLevel = LagUD.kr(In.kr(motionBus, 1).clip(0.0, 1.0), motionRise.clip(0.01, 12.0), motionFall.clip(0.01, 12.0));
        var choirTrigPulse = Trig1.kr(Changed.kr(choirTrig), 0.03);
        var choirEnv = EnvGen.kr(
            Env([0.0, 1.0, 0.88, 0.0], [fadeIn.clip(0.01, 1.5), holdTime.clip(0.0, 3.0), fadeOut.clip(0.2, 9.0)], [-4, -1, -5]),
            choirTrigPulse
        );
        var root = Lag.kr(rootFreq.clip(120.0, 320.0), 0.16);
        var basePitch = Lag.kr(basePitchHz.clip(120.0, 320.0), 0.08);
        var ampCtl = Lag.kr(amp.clip(0.0, 0.6), 0.12);
        var synthAmpCtl = Lag.kr(synthAmp.clip(0.0, 0.8), 0.12);
        var spreadCtl = Lag.kr(spread.clip(0.1, 1.0), 0.25);
        var shimmerCtl = Lag.kr(shimmer.clip(0.0, 1.0), 0.25);
        var breathCtl = Lag.kr(breath.clip(0.0, 0.2), 0.18);
        var brightCtl = Lag.kr(brightness.clip(0.0, 1.0), 0.2);
        var fxMixCtl = Lag.kr(choirFxMix.clip(0.0, 1.0), 0.22);
        var delayMixCtl = Lag.kr(choirDelayMix.clip(0.0, 1.0), 0.2);
        var airCtl = Lag.kr(choirAir.clip(0.0, 1.0), 0.18);
        var bassAmpCtl = Lag.kr(choirBassAmp.clip(0.0, 0.45), 0.12);
        var bassToneCtl = Lag.kr(choirBassTone.clip(0.0, 1.0), 0.18);
        var accentLevel = LagUD.kr(choirEnv.clip(0.0, 1.0), 0.02, 0.42);
        var startFrame = (sampleStart.clip(0.0, 8.5) * BufSampleRate.kr(buf0)).min(BufFrames.kr(buf0) - 2);
        var sampleRate = BufRateScale.kr(buf0) * (root / basePitch.max(1.0));
        var rawSample = LeakDC.ar(PlayBuf.ar(1, buf0, sampleRate, choirTrigPulse, startFrame, 0));
        var sampleTone = BHiShelf.ar(
            HPF.ar(LPF.ar(rawSample, brightCtl.linexp(0.0, 1.0, 1800.0, 6200.0)), 70.0),
            2400.0,
            0.65,
            airCtl.linlin(0.0, 1.0, -1.0, 4.5)
        );
        var choirDry = Pan2.ar(sampleTone, pan, ampCtl * accentLevel);
        var choirDelay = CombC.ar(
            choirDry * (0.2 + (delayMixCtl * 0.5)),
            0.8,
            [0.22, 0.31] + (spreadCtl * 0.035) + (shimmerCtl * 0.018),
            2.8 + (shimmerCtl * 2.2)
        ) * delayMixCtl;
        var choirHall = FreeVerb2.ar(
            choirDry[0] + choirDelay[0],
            choirDry[1] + choirDelay[1],
            0.34 + (fxMixCtl * 0.34),
            0.92 + (fxMixCtl * 0.07),
            0.56 - (brightCtl * 0.18)
        );
        var choirLayer = choirDry + choirDelay + (choirHall * fxMixCtl);
        var vowelLift = brightCtl.linlin(0.0, 1.0, 0.94, 1.12);
        var makeVowel = { |freq, gain=1.0|
            var shimmerDetune = 1.0 + LFNoise2.kr(0.11).bipolar(0.002 + (shimmerCtl * 0.003));
            var airy = HPF.ar(WhiteNoise.ar((breathCtl * 0.018) + (airCtl * 0.004) + 0.001), (1400.0 + (brightCtl * 1200.0)).clip(1200.0, 4200.0)) * 0.18;
            var presence = BHiShelf.ar(
                HPF.ar(
                    Mix(VarSaw.ar(freq * [1.0, 1.5, 2.0], 0.0, [0.54, 0.48, 0.42], [0.09, 0.05, 0.04])),
                    freq.clip(90.0, 240.0)
                ),
                1800.0,
                0.62,
                2.5 + (brightCtl * 3.8)
            );
            var voice = LeakDC.ar(
                Formant.ar(
                    freq * shimmerDetune,
                    ((freq * 6.0) * vowelLift).clip((freq + 20.0), 3600.0),
                    ((freq * 2.4) * vowelLift).clip(220.0, 2200.0),
                    0.22
                )
                + (Formant.ar(
                    (freq * 0.5) * shimmerDetune,
                    ((freq * 4.0) * vowelLift).clip(((freq * 0.5) + 20.0), 2800.0),
                    ((freq * 1.8) * vowelLift).clip(180.0, 1800.0),
                    0.10
                ) * 0.6)
                + (presence * 0.68)
                + airy
            );

            voice * gain;
        };
        var activeCount = 1
            + octaveDown3.clip(0.0, 1.0)
            + octaveDown2.clip(0.0, 1.0)
            + octaveDown1.clip(0.0, 1.0)
            + octaveUp1.clip(0.0, 1.0)
            + octaveUp2.clip(0.0, 1.0)
            + octaveUp3.clip(0.0, 1.0);
        var synthMono = Mix([
            makeVowel.(root * 0.125, octaveDown3.clip(0.0, 1.0) * 0.38),
            makeVowel.(root * 0.25, octaveDown2.clip(0.0, 1.0) * 0.52),
            makeVowel.(root * 0.5, octaveDown1.clip(0.0, 1.0) * 0.74),
            makeVowel.(root, 1.0),
            makeVowel.(root * 2.0, octaveUp1.clip(0.0, 1.0) * 0.64),
            makeVowel.(root * 4.0, octaveUp2.clip(0.0, 1.0) * 0.42),
            makeVowel.(root * 8.0, octaveUp3.clip(0.0, 1.0) * 0.26)
        ]) * activeCount.max(1.0).sqrt.reciprocal * synthAmpCtl * accentLevel * 1.65;
        var synthDry = Splay.ar([
            synthMono,
            DelayC.ar(synthMono, 0.08, 0.019 + (shimmerCtl * 0.004)),
            DelayC.ar(synthMono, 0.08, 0.033 + (spreadCtl * 0.006))
        ], spreadCtl * 0.24, 0.72, pan);
        var synthHall = FreeVerb2.ar(
            synthDry[0] + DelayC.ar(synthDry[0], 0.14, 0.041),
            synthDry[1] + DelayC.ar(synthDry[1], 0.14, 0.053),
            0.38 + (fxMixCtl * 0.24) + (shimmerCtl * 0.04),
            0.98,
            0.22 + (breathCtl * 0.06)
        );
        var bassFreq = (root * 0.5).clip(40.0, 180.0);
        var bassLayer = Pan2.ar(
            RLPF.ar(
                (SinOsc.ar(bassFreq, 0.0, 0.58) + VarSaw.ar(bassFreq, 0.0, 0.42, 0.16)).tanh,
                bassToneCtl.linexp(0.0, 1.0, 110.0, 840.0),
                0.32
            ) * bassAmpCtl * accentLevel,
            pan * 0.25
        );
        var blend = choirLayer + (synthDry * 1.05) + (synthHall * (0.34 + (fxMixCtl * 0.36))) + bassLayer;
        var sig = Limiter.ar(
            LeakDC.ar(blend * gateEnv),
            0.97,
            0.01
        );
        Out.ar(out, sig);
    }).add;

    SynthDef(\spacewizardChoirRawSamplePlayer, {
        |out=0, buf=0, amp=0.22, pan=0.0, sampleStart=0.0, fxMix=0.0, delayMix=0.0, air=0.0, brightness=0.28, spread=0.78|
        var startFrame = (sampleStart.clip(0.0, 8.5) * BufSampleRate.kr(buf)).min(BufFrames.kr(buf) - 2);
        var raw = PlayBuf.ar(1, buf, BufRateScale.kr(buf), 1, startFrame, 0, doneAction: 2);
        var fxMixCtl = fxMix.clip(0.0, 1.0);
        var delayMixCtl = delayMix.clip(0.0, 1.0);
        var brightCtl = brightness.clip(0.0, 1.0);
        var tone = BHiShelf.ar(
            HPF.ar(LPF.ar(LeakDC.ar(raw), brightCtl.linexp(0.0, 1.0, 1800.0, 6200.0)), 70.0),
            2400.0,
            0.65,
            air.clip(0.0, 1.0).linlin(0.0, 1.0, -1.0, 4.5)
        );
        var dry = Pan2.ar(tone, pan, amp.clip(0.0, 0.6));
        var echo = CombC.ar(
            dry * (0.2 + (delayMixCtl * 0.5)),
            0.8,
            [0.22, 0.31] + (spread.clip(0.1, 1.0) * 0.035),
            2.8
        ) * delayMixCtl;
        var hall = FreeVerb2.ar(
            dry[0] + echo[0],
            dry[1] + echo[1],
            0.34 + (fxMixCtl * 0.34),
            0.92 + (fxMixCtl * 0.07),
            0.56 - (brightCtl * 0.18)
        );
        var sig = Limiter.ar(dry + echo + (hall * fxMixCtl), 0.97, 0.01);

        Out.ar(out, sig);
    }).add;

    ~swMakeChoirPreviewSynthDef = { |name, voiceKind=\cathedral, hallMix=0.34, hallRoom=0.44, hallDamp=0.26|
        SynthDef(name, {
            |out=0, buf=0, synthAmp=0.14, pan=0.0, sampleStart=0.0, basePitchHz=176.47,
             shimmer=0.12, breath=0.07, brightness=0.28, spread=0.78,
             hallMixCtl=0.34, hallRoomCtl=0.44, hallDampCtl=0.26, bassAmp=0.0, bassTone=0.42,
             octaveDown3=0, octaveDown2=1, octaveDown1=1, octaveUp1=0, octaveUp2=0, octaveUp3=0|
            var synthAmpCtl = Lag.kr(synthAmp.clip(0.0, 0.8), 0.08);
            var baseFreq = Lag.kr(basePitchHz.clip(120.0, 320.0), 0.05);
            var sampleDur = (BufDur.kr(buf) - sampleStart.clip(0.0, 8.5)).max(0.25);
            var shimmerCtl = Lag.kr(shimmer.clip(0.0, 1.0), 0.2);
            var breathCtl = Lag.kr(breath.clip(0.0, 0.2), 0.16);
            var brightCtl = Lag.kr(brightness.clip(0.0, 1.0), 0.18);
            var spreadCtl = Lag.kr(spread.clip(0.1, 1.0), 0.2);
            var bassAmpCtl = Lag.kr(bassAmp.clip(0.0, 0.45), 0.12);
            var bassToneCtl = Lag.kr(bassTone.clip(0.0, 1.0), 0.16);
            var makeVoice = { |freq, gain=1.0|
                var voice;
                var presence;

                voice = if (voiceKind == \cathedral) {
                    BHiShelf.ar(
                        HPF.ar(
                            RLPF.ar(
                                Mix([
                                    VarSaw.ar(freq * [1.0, 1.004], 0.0, 0.48, [0.055, 0.045]),
                                    SinOsc.ar(freq, 0.0, 0.08),
                                    SinOsc.ar(freq * 2.0, 0.0, 0.03),
                                    SinOsc.ar(freq * 3.0, 0.0, 0.015)
                                ]),
                                (freq * 6.5).clip(1200.0, 4200.0),
                                0.42
                            ),
                            freq.clip(120.0, 260.0)
                        ),
                        1800.0,
                        0.6,
                        2.4
                    )
                } {
                    if (voiceKind == \flow) {
                        LPF.ar(
                            LeakDC.ar(
                                Mix([
                                    VarSaw.ar(
                                        freq * [1.0, 1.0025],
                                        0.0,
                                        LFNoise2.kr(0.08).range(0.52, 0.84),
                                        [0.12, 0.10]
                                    ),
                                    LFTri.ar(freq, 0.0, 0.10),
                                    SinOsc.ar(freq * 2.0, 0.0, 0.055),
                                    VarSaw.ar(freq * 2.01, 0.0, 0.42, 0.04)
                                ])
                                + (AllpassC.ar(SinOsc.ar(freq, 0.0, 0.075), 0.2, LFNoise2.kr(0.11).range(0.03, 0.12), 1.6) * 0.7)
                            ),
                            (freq * (6.4 + (brightCtl * 4.5))).clip(1100.0, 5600.0)
                        )
                    } {
                        if (voiceKind == \ribbon) {
                            BHiShelf.ar(
                                HPF.ar(
                                    LeakDC.ar(
                                        Mix([
                                            LFTri.ar(freq * [1.0, 2.0], 0.0, [0.05, 0.02]),
                                            VarSaw.ar(freq * [1.0, 1.5], 0.0, 0.72, [0.018, 0.014]),
                                            SinOsc.ar(freq * 3.0, 0.0, 0.012)
                                        ])
                                    ),
                                    freq.clip(140.0, 240.0)
                                ),
                                2400.0,
                                0.7,
                                4.0
                            )
                        } {
                            if (voiceKind == \glass) {
                                LeakDC.ar(
                                    BPF.ar(VarSaw.ar(freq * [1.0, 2.0], 0.0, 0.32, [0.03, 0.02]).sum, (freq * 4.0).clip(1100.0, 4200.0), 0.24)
                                    + SinOsc.ar(freq * 2.0, 0.0, 0.04)
                                    + SinOsc.ar(freq * 4.0, 0.0, 0.015)
                                )
                            } {
                                if (voiceKind == \vowel) {
                                    LeakDC.ar(
                                        Formant.ar(
                                            freq,
                                            (freq * 6.0).clip(freq + 20.0, 3600.0),
                                            (freq * 2.4).clip(220.0, 2200.0),
                                            0.11
                                        )
                                        + (Formant.ar(
                                            freq * 0.5,
                                            (freq * 4.0).clip((freq * 0.5) + 20.0, 2800.0),
                                            (freq * 1.8).clip(180.0, 1800.0),
                                            0.05
                                        ) * 0.6)
                                    )
                                } {
                                    LeakDC.ar(
                                        Klank.ar(
                                            `[
                                                [1.0, 2.01, 3.97, 5.43, 6.79] * freq.clip(90.0, 1200.0),
                                                [1.0, 0.7, 0.45, 0.28, 0.16],
                                                [2.8, 2.2, 1.8, 1.4, 1.2]
                                            ],
                                            PinkNoise.ar(0.004)
                                        ) * (44100 / SampleRate.ir)
                                    )
                                }
                            }
                        }
                    }
                };
                presence = BHiShelf.ar(
                    HPF.ar(
                        Mix(VarSaw.ar(freq * [1.0, 1.5, 2.0], 0.0, [0.54, 0.48, 0.42], [0.065, 0.04, 0.035])),
                        freq.clip(90.0, 240.0)
                    ),
                    2100.0,
                    0.62,
                    1.8 + (brightCtl * 3.4)
                );

                BHiShelf.ar(
                    voice + (presence * 0.55) + (HPF.ar(WhiteNoise.ar((breathCtl * 0.012) + 0.0008), 1600.0 + (brightCtl * 1200.0)) * (0.05 + (shimmerCtl * 0.04))),
                    2200.0,
                    0.65,
                    brightCtl.linlin(0.0, 1.0, -1.5, 4.5)
                ) * gain * 1.9;
            };
            var activeCount = 1
                + octaveDown3.clip(0.0, 1.0)
                + octaveDown2.clip(0.0, 1.0)
                + octaveDown1.clip(0.0, 1.0)
                + octaveUp1.clip(0.0, 1.0)
                + octaveUp2.clip(0.0, 1.0)
                + octaveUp3.clip(0.0, 1.0);
            var synthMonoBase = Mix([
                makeVoice.(baseFreq * 0.125, octaveDown3.clip(0.0, 1.0) * 0.38),
                makeVoice.(baseFreq * 0.25, octaveDown2.clip(0.0, 1.0) * 0.52),
                makeVoice.(baseFreq * 0.5, octaveDown1.clip(0.0, 1.0) * 0.74),
                makeVoice.(baseFreq, 1.0),
                makeVoice.(baseFreq * 2.0, octaveUp1.clip(0.0, 1.0) * 0.64),
                makeVoice.(baseFreq * 4.0, octaveUp2.clip(0.0, 1.0) * 0.42),
                makeVoice.(baseFreq * 8.0, octaveUp3.clip(0.0, 1.0) * 0.26)
            ]) * activeCount.max(1.0).sqrt.reciprocal * synthAmpCtl * 1.55;
            var synthMono = synthMonoBase + (DelayC.ar(synthMonoBase, 0.12, LFNoise2.kr(0.08).range(0.018, 0.09)) * shimmerCtl * 0.18);
            var synthStereo = Splay.ar([
                synthMono,
                DelayC.ar(synthMono, 0.08, 0.019 + (shimmerCtl * 0.004)),
                DelayC.ar(synthMono, 0.08, 0.033 + (spreadCtl * 0.006))
            ], spreadCtl * 0.28, 0.72, pan);
            var synthHall = FreeVerb2.ar(
                synthStereo[0] + DelayC.ar(synthStereo[0], 0.14, 0.041),
                synthStereo[1] + DelayC.ar(synthStereo[1], 0.14, 0.053),
                hallRoomCtl.clip(0.0, 1.0),
                0.98,
                hallDampCtl.clip(0.0, 1.0)
            );
            var bassFreq = (baseFreq * 0.5).clip(40.0, 180.0);
            var bassLayer = Pan2.ar(
                RLPF.ar(
                    (SinOsc.ar(bassFreq, 0.0, 0.58) + VarSaw.ar(bassFreq, 0.0, 0.42, 0.16)).tanh,
                    bassToneCtl.linexp(0.0, 1.0, 110.0, 840.0),
                    0.32
                ) * bassAmpCtl,
                pan * 0.25
            );
            var keepAlive = Line.kr(1.0, 1.0, sampleDur + 1.6, doneAction: 2);
            var sig = Limiter.ar(((synthStereo * 1.05) + (synthHall * (0.25 + (hallMixCtl.clip(0.0, 1.0) * 0.62))) + bassLayer) * keepAlive, 0.97, 0.01);

            Out.ar(out, sig);
        }).add;
    };

    ~swMakeChoirPreviewSynthDef.(\spacewizardChoirPreviewCathedral, \cathedral, 0.34, 0.44, 0.26);
    ~swMakeChoirPreviewSynthDef.(\spacewizardChoirPreviewFlow, \flow, 0.4, 0.56, 0.42);
    ~swMakeChoirPreviewSynthDef.(\spacewizardChoirPreviewRibbon, \ribbon, 0.28, 0.38, 0.24);
    ~swMakeChoirPreviewSynthDef.(\spacewizardChoirPreviewGlass, \glass, 0.22, 0.32, 0.18);
    ~swMakeChoirPreviewSynthDef.(\spacewizardChoirPreviewVowel, \vowel, 0.26, 0.41, 0.22);
    ~swMakeChoirPreviewSynthDef.(\spacewizardChoirPreviewChime, \chime, 0.38, 0.5, 0.3);

    SynthDef(\spacewizardAmbientFlow, {
        |out=0, gate=1, rootFreq=74, amp=0.1, pan=0.0, lift=0.0, lean=0.0,
         motion=0.28, spin=0.18, aura=0.62, darkness=0.58, shimmer=0.42, width=0.82,
         motionBus=0, motionRise=3.2, motionFall=5.0|
        var gateEnv = EnvGen.kr(Env.asr(2.6, 1.0, 5.8), gate, doneAction: 2);
        var motionLevel = LagUD.kr(In.kr(motionBus, 1).clip(0.0, 1.0), motionRise.clip(0.01, 12.0), motionFall.clip(0.01, 12.0));
        var root = Lag.kr(rootFreq.clip(38.0, 240.0), 3.4);
        var rootMidi = root.cpsmidi;
        var liftCtl = Lag.kr(lift.clip(-1.0, 1.0), 2.4);
        var leanCtl = Lag.kr(lean.clip(-1.0, 1.0), 2.0);
        var motionCtl = Lag.kr(motion.clip(0.0, 1.0), 3.0);
        var spinCtl = Lag.kr(spin.clip(0.0, 1.0), 1.8);
        var auraCtl = Lag.kr(aura.clip(0.0, 1.0), 2.8);
        var darkCtl = Lag.kr(darkness.clip(0.0, 1.0), 3.4);
        var shimmerCtl = Lag.kr(shimmer.clip(0.0, 1.0), 2.2);
        var widthCtl = Lag.kr(width.clip(0.0, 1.0), 2.0);
        var ampCtl = Lag.kr(amp.clip(0.0, 0.25), 0.8);
        var warmDegrees = [0, 3, 7, 10, 14];
        var brightDegrees = [0, 2, 7, 9, 14];
        var liftOffsets = [-7, -2, 0, 4, 9];
        var driftRates = [0.021, 0.017, 0.013, 0.011, 0.019] + (motionCtl * [0.055, 0.046, 0.061, 0.042, 0.058]);
        var degrees = Array.fill(5, { |index|
            SelectX.kr(leanCtl.linlin(-1.0, 1.0, 0.0, 1.0), [warmDegrees[index], brightDegrees[index]])
        });
        var freqJitter = Array.fill(5, { |index|
            LFNoise2.kr(driftRates[index]).range(-0.22, 0.22)
                + (SinOsc.kr(driftRates[index] * 0.37, index).range(-0.08, 0.08) * (0.35 + spinCtl))
        });
        var freqs = Array.fill(5, { |index|
            (rootMidi + degrees[index] + (liftCtl * liftOffsets[index]) + freqJitter[index]).midicps
        });
        var layers = Array.fill(5, { |index|
            var freq = freqs[index];
            var widthMod = LFNoise2.kr(0.03 + (index * 0.01) + (spinCtl * 0.05)).range(0.28, 0.82);
            var body = SinOsc.ar(freq * [0.5, 1.0, 2.0 + (liftCtl * 0.03)], 0.0, [0.03, 0.026, 0.011]).sum;
            var haze = VarSaw.ar(freq * [0.5, 1.0 + (index * 0.0014)], 0.0, [widthMod, 0.44], [0.017, 0.011]).sum;
            var cutoff = (freq * (2.4 + (auraCtl * 2.6) + (liftCtl * 0.9) + ((1.0 - darkCtl) * 2.2))).clip(180.0, 5200.0);
            LPF.ar(body + haze, cutoff)
        });
        var bed = Splay.ar(layers, widthCtl.linlin(0.0, 1.0, 0.22, 0.98), 1.0, pan + (leanCtl * 0.14));
        var sparkleTrig = Dust.kr(motionCtl.linexp(0.0, 1.0, 0.14, 3.2) + shimmerCtl.linlin(0.0, 1.0, 0.0, 2.6));
        var sparkleDegree = Demand.kr(
            sparkleTrig,
            0,
            Dwrand([0, 2, 5, 7, 9, 12, 14, 19], [0.16, 0.14, 0.16, 0.14, 0.1, 0.12, 0.1, 0.08], inf)
        );
        var sparkleFreq = (rootMidi + sparkleDegree + liftCtl.linlin(-1.0, 1.0, -5.0, 9.0) + (leanCtl * 2.5)).midicps
            * TExpRand.kr(0.992, 1.014, sparkleTrig);
        var sparkleEnv = Decay2.kr(sparkleTrig, 0.03, TExpRand.kr(1.8, 5.2, sparkleTrig));
        var sparkle = Mix(Ringz.ar(
            PinkNoise.ar(0.005) * sparkleEnv,
            sparkleFreq * [1.0, 1.5, 2.01],
            [3.8, 4.5, 5.1],
            [0.1, 0.07, 0.04]
        ));
        var sparkleStereo = Pan2.ar(HPF.ar(sparkle, 280), TRand.kr(-0.9, 0.9, sparkleTrig) * widthCtl);
        var mist = Gendy1.ar(
            2,
            3,
            0.2 + (motionCtl * 0.35),
            0.18 + (spinCtl * 0.25),
            (root * 0.25).clip(16.0, 120.0),
            (root * (1.5 + (auraCtl * 2.1))).clip(60.0, 1500.0),
            0.1 + (auraCtl * 0.12),
            0.05 + (motionCtl * 0.08),
            8
        );
        var mistFiltered = BPF.ar(
            mist,
            (root * (3.2 + (liftCtl * 0.8) + ((1.0 - darkCtl) * 2.3))).clip(260.0, 4200.0),
            0.18,
            0.05 + (auraCtl * 0.08)
        );
        var air = BPF.ar(
            WhiteNoise.ar(0.003 + (shimmerCtl * 0.0035)),
            (root * (5.2 + (liftCtl * 1.2) + (spinCtl * 2.0))).clip(900.0, 7200.0),
            0.22,
            0.04 + (shimmerCtl * 0.08)
        );
        var sig = bed + sparkleStereo + (mistFiltered ! 2) + (air ! 2);
        var globalLp = (darkCtl.linlin(0.0, 1.0, 7200.0, 1600.0) + liftCtl.linlin(-1.0, 1.0, -320.0, 900.0)).clip(1100.0, 8200.0);
        var verb;

        sig = LeakDC.ar(LPF.ar(sig, globalLp));
        verb = FreeVerb2.ar(sig[0], sig[1], 0.26 + (auraCtl * 0.16), 0.72 + (motionCtl * 0.2), darkCtl.linlin(0.0, 1.0, 0.78, 0.36));
        sig = ((sig * 0.68) + (verb * 0.92)) * ampCtl * gateEnv * motionLevel;
        Out.ar(out, sig);
    }).add;

    SynthDef(\spacewizardAmbientChords, {
        |out=0, gate=1, rootFreq=74, amp=0.1, pan=0.0, lift=0.0, lean=0.0,
         motion=0.28, spin=0.18, aura=0.62, darkness=0.58, shimmer=0.42, width=0.82,
         motionBus=0, motionRise=3.2, motionFall=5.0|
        var gateEnv = EnvGen.kr(Env.asr(3.2, 1.0, 6.4), gate, doneAction: 2);
        var motionLevel = LagUD.kr(In.kr(motionBus, 1).clip(0.0, 1.0), motionRise.clip(0.01, 12.0), motionFall.clip(0.01, 12.0));
        var root = Lag.kr(rootFreq.clip(40.0, 240.0), 4.0);
        var rootMidi = root.cpsmidi;
        var liftCtl = Lag.kr(lift.clip(-1.0, 1.0), 2.6);
        var leanCtl = Lag.kr(lean.clip(-1.0, 1.0), 2.4);
        var motionCtl = Lag.kr(motion.clip(0.0, 1.0), 3.4);
        var spinCtl = Lag.kr(spin.clip(0.0, 1.0), 2.4);
        var auraCtl = Lag.kr(aura.clip(0.0, 1.0), 3.0);
        var darkCtl = Lag.kr(darkness.clip(0.0, 1.0), 3.6);
        var shimmerCtl = Lag.kr(shimmer.clip(0.0, 1.0), 2.4);
        var widthCtl = Lag.kr(width.clip(0.0, 1.0), 2.0);
        var ampCtl = Lag.kr(amp.clip(0.0, 0.25), 0.8);
        var chordBlend = leanCtl.linlin(-1.0, 1.0, 0.0, 1.0);
        var chordTrig = Dust.kr(motionCtl.linexp(0.0, 1.0, 0.02, 0.34) + 0.015 + (spinCtl * 0.03));
        var chordIndex = Demand.kr(
            chordTrig,
            0,
            Dwrand([0, 1, 2, 3, 4], [0.18, 0.22, 0.2, 0.18, 0.22], inf)
        );
        var warmChords = [
            [0, 0, -2, 0, -5],
            [3, 5, 3, 7, 0],
            [7, 10, 7, 10, 3],
            [10, 14, 12, 15, 10]
        ];
        var brightChords = [
            [0, 0, -2, 0, -3],
            [4, 2, 5, 4, 0],
            [7, 7, 9, 9, 4],
            [11, 9, 12, 14, 11]
        ];
        var liftOffsets = [-7, -2, 3, 7];
        var widthAmt = widthCtl.linlin(0.0, 1.0, 0.16, 0.88);
        var voices = Array.fill(4, { |index|
            var degree = SelectX.kr(
                chordBlend,
                [
                    Select.kr(chordIndex, warmChords[index]),
                    Select.kr(chordIndex, brightChords[index])
                ]
            );
            var freq = (rootMidi + degree + (liftCtl * liftOffsets[index])
                + LFNoise2.kr(0.018 + (index * 0.006) + (motionCtl * 0.02)).range(-0.12, 0.12)).midicps;
            var pulseWidth = LFNoise2.kr(0.04 + (index * 0.01) + (spinCtl * 0.02)).range(0.24, 0.76);
            var source = Mix(VarSaw.ar(
                freq * [0.5, 1.0, 1.004],
                0.0,
                [pulseWidth, 0.42, 0.58],
                [0.02, 0.03, 0.016]
            ));
            source = source + Mix(SinOsc.ar(freq * [1.0, 2.0], 0.0, [0.012, 0.006]));
            source = RLPF.ar(
                source,
                (freq * (2.2 + (auraCtl * 2.6) + ((1.0 - darkCtl) * 1.7) + (liftCtl * 0.6))).clip(180.0, 5200.0),
                0.14 + (darkCtl * 0.1)
            );
            Pan2.ar(source, ((index / 1.5) - 1.0) * widthAmt + pan + (leanCtl * 0.08));
        });
        var glintTrig = Dust.kr(shimmerCtl.linexp(0.0, 1.0, 0.04, 1.8) + (motionCtl * 0.3));
        var glintEnv = Decay2.kr(glintTrig, 0.02, 2.4 + (auraCtl * 2.2));
        var glintDegree = Demand.kr(
            glintTrig,
            0,
            Dwrand([7, 10, 12, 14, 19], [0.22, 0.18, 0.22, 0.2, 0.18], inf)
        );
        var glintFreq = (rootMidi + glintDegree + liftCtl.linlin(-1.0, 1.0, -4.0, 8.0)).midicps
            * TExpRand.kr(0.995, 1.012, glintTrig);
        var glints = Ringz.ar(WhiteNoise.ar(0.004) * glintEnv, glintFreq * [1.0, 2.01], [3.8, 4.6], [0.08, 0.03]).sum;
        var glintStereo = Pan2.ar(HPF.ar(glints, 320), TRand.kr(-0.82, 0.82, glintTrig) * widthCtl);
        var floor = LPF.ar(SinOsc.ar(root * 0.5, 0.0, 0.035 + (darkCtl * 0.025)), root * (1.8 + (auraCtl * 0.8)));
        var sig = Mix(voices) + glintStereo + (floor ! 2);
        var verb;

        sig = LeakDC.ar(sig);
        verb = FreeVerb2.ar(sig[0], sig[1], 0.28 + (auraCtl * 0.14), 0.84 + (motionCtl * 0.08), darkCtl.linlin(0.0, 1.0, 0.75, 0.42));
        sig = ((sig * 0.7) + (verb * 0.95)) * ampCtl * gateEnv * motionLevel;
        Out.ar(out, sig);
    }).add;

    SynthDef(\spacewizardAmbientDrone, {
        |out=0, gate=1, rootFreq=74, amp=0.1, pan=0.0, lift=0.0, lean=0.0,
         motion=0.28, spin=0.18, aura=0.62, darkness=0.58, shimmer=0.42, width=0.82,
         motionBus=0, motionRise=3.2, motionFall=5.0|
        var gateEnv = EnvGen.kr(Env.asr(4.2, 1.0, 8.2), gate, doneAction: 2);
        var motionLevel = LagUD.kr(In.kr(motionBus, 1).clip(0.0, 1.0), motionRise.clip(0.01, 12.0), motionFall.clip(0.01, 12.0));
        var root = Lag.kr(rootFreq.clip(32.0, 180.0), 4.6);
        var rootMidi = root.cpsmidi - 12;
        var liftCtl = Lag.kr(lift.clip(-1.0, 1.0), 2.8);
        var leanCtl = Lag.kr(lean.clip(-1.0, 1.0), 2.4);
        var motionCtl = Lag.kr(motion.clip(0.0, 1.0), 3.8);
        var spinCtl = Lag.kr(spin.clip(0.0, 1.0), 2.6);
        var auraCtl = Lag.kr(aura.clip(0.0, 1.0), 3.0);
        var darkCtl = Lag.kr(darkness.clip(0.0, 1.0), 4.0);
        var shimmerCtl = Lag.kr(shimmer.clip(0.0, 1.0), 2.8);
        var widthCtl = Lag.kr(width.clip(0.0, 1.0), 2.2);
        var ampCtl = Lag.kr(amp.clip(0.0, 0.25), 0.8);
        var baseOffsets = [-17, -12, -5, 0, 7];
        var liftOffsets = [-6, -4, -1, 2, 5];
        var widthAmt = widthCtl.linlin(0.0, 1.0, 0.12, 0.78);
        var layers = Array.fill(5, { |index|
            var freq = (rootMidi + baseOffsets[index] + (liftCtl * liftOffsets[index])
                + LFNoise2.kr(0.012 + (index * 0.004) + (motionCtl * 0.018)).range(-0.18, 0.18)).midicps;
            var drift = SinOsc.kr(0.018 + (index * 0.005) + (motionCtl * 0.025), index)
                .range(0.996 - (spinCtl * 0.002), 1.004 + (spinCtl * 0.002));
            var body = Mix(VarSaw.ar(
                freq * drift * [0.5, 1.0, 1.004],
                0.0,
                [0.64, 0.36, 0.52],
                [0.016, 0.022, 0.014]
            ));
            var sine = Mix(SinOsc.ar(freq * [0.5, 1.0, 1.49], 0.0, [0.02, 0.018, 0.008]));
            var rasp = BPF.ar(
                PinkNoise.ar(0.002 + (motionCtl * 0.0025) + (shimmerCtl * 0.0015)),
                (freq * (1.2 + (darkCtl * 0.8) + (spinCtl * 0.3))).clip(90.0, 2400.0),
                0.16,
                0.018 + (motionCtl * 0.03)
            );
            var cutoff = (freq * (1.8 + (auraCtl * 1.4) + ((1.0 - darkCtl) * 1.2))).clip(120.0, 3200.0);
            Pan2.ar(LPF.ar(body + sine + rasp, cutoff), ((index / 2.0) - 1.0) * widthAmt + pan + (leanCtl * 0.06));
        });
        var undertow = LPF.ar(Mix(SinOsc.ar((root * 0.25) * [0.995, 1.005], 0.0, [0.08, 0.08])), 180 + (auraCtl * 120));
        var friction = BPF.ar(
            GrayNoise.ar(0.003 + (motionCtl * 0.004)),
            (root * (2.4 + (spinCtl * 3.4) + (liftCtl * 0.5))).clip(160.0, 4200.0),
            0.2,
            0.06 + (spinCtl * 0.08)
        );
        var fog = CombC.ar(Mix(layers) * (0.18 + (auraCtl * 0.08)), 1.4, LFNoise2.kr(0.07 + (motionCtl * 0.05)).range(0.18, 0.72), 4.2);
        var sig = Mix(layers) + (undertow ! 2) + (friction ! 2) + (fog ! 2);
        var globalLp = (darkCtl.linlin(0.0, 1.0, 5200.0, 1400.0) + (liftCtl * 420.0)).clip(900.0, 6200.0);
        var verb;

        sig = LeakDC.ar(LPF.ar(sig, globalLp));
        verb = FreeVerb2.ar(sig[0], sig[1], 0.24 + (auraCtl * 0.16), 0.92, 0.28 + (darkCtl * 0.12));
        sig = ((sig * 0.72) + (verb * 0.88)) * ampCtl * gateEnv * motionLevel;
        Out.ar(out, sig);
    }).add;

    SynthDef(\spacewizardAmbientForest, {
        |out=0, gate=1, rootFreq=74, amp=0.1, pan=0.0, lift=0.0, lean=0.0,
         motion=0.28, spin=0.18, aura=0.62, darkness=0.58, shimmer=0.42, width=0.82,
         motionBus=0, motionRise=3.2, motionFall=5.0|
        var gateEnv, root, rootMidi, liftCtl, leanCtl, motionCtl, spinCtl, auraCtl, darkCtl,
            shimmerCtl, widthCtl, widthAmt, windMotion, wind, canopy, leafTrig, leafEnv,
            leafFreq, leaves, leafStereo, insectTrig, insectEnv, insectFreq, insects,
            insectStereo, callTrig, callEnv, callGlide, callFreq, calls, callStereo,
            ampCtl, motionLevel,
            floor, sig, globalLp, verb;

        gateEnv = EnvGen.kr(Env.asr(2.6, 1.0, 6.4), gate, doneAction: 2);
        root = Lag.kr(rootFreq.clip(42.0, 200.0), 3.6);
        rootMidi = root.cpsmidi;
        liftCtl = Lag.kr(lift.clip(-1.0, 1.0), 2.0);
        leanCtl = Lag.kr(lean.clip(-1.0, 1.0), 1.8);
        motionCtl = Lag.kr(motion.clip(0.0, 1.0), 2.4);
        spinCtl = Lag.kr(spin.clip(0.0, 1.0), 1.8);
        auraCtl = Lag.kr(aura.clip(0.0, 1.0), 2.4);
        darkCtl = Lag.kr(darkness.clip(0.0, 1.0), 2.8);
        shimmerCtl = Lag.kr(shimmer.clip(0.0, 1.0), 1.8);
        widthCtl = Lag.kr(width.clip(0.0, 1.0), 1.8);
        ampCtl = Lag.kr(amp.clip(0.0, 0.25), 0.8);
        motionLevel = LagUD.kr(In.kr(motionBus, 1).clip(0.0, 1.0), motionRise.clip(0.01, 12.0), motionFall.clip(0.01, 12.0));
        widthAmt = widthCtl.linlin(0.0, 1.0, 0.42, 0.98);
        windMotion = LFNoise2.kr(0.035 + (motionCtl * 0.08)).range(0.28, 1.0);
        wind = HPF.ar(BrownNoise.ar(0.018 + (auraCtl * 0.01)) * windMotion, 90);
        canopy = Splay.ar(Array.fill(3, { |index|
            var freq = (rootMidi + [-12, 0, 7][index] + (liftCtl * [-5, 0, 4][index])
                + LFNoise2.kr(0.018 + (index * 0.008) + (motionCtl * 0.03)).range(-0.14, 0.14)).midicps;
            var tone = Mix(VarSaw.ar(freq * [0.5, 1.0], 0.0, [0.62, 0.44], [0.012, 0.018]));
            tone = tone + SinOsc.ar(freq * (2.0 + (index * 0.2)), 0.0, 0.005 + (auraCtl * 0.004));
            LPF.ar(tone, (freq * (3.0 + ((1.0 - darkCtl) * 2.2))).clip(160.0, 4200.0));
        }), widthAmt, 1.0, pan + (leanCtl * 0.1));
        wind = BPF.ar(wind, (darkCtl.linlin(0.0, 1.0, 5200.0, 1800.0) + (leanCtl * 400.0)).clip(900.0, 6400.0), 0.35, 0.22 + (auraCtl * 0.12));
        leafTrig = Dust.kr(motionCtl.linexp(0.0, 1.0, 1.0, 18.0) + (spinCtl * 3.5) + 0.3);
        leafEnv = Decay2.kr(leafTrig, 0.005, 0.05 + (motionCtl * 0.16));
        leafFreq = TExpRand.kr(480.0, 4400.0, leafTrig) * (1.0 + (liftCtl * 0.18));
        leaves = BPF.ar(WhiteNoise.ar(0.03) * leafEnv, leafFreq, 0.2, 0.14 + (motionCtl * 0.08));
        leafStereo = Pan2.ar(leaves, TRand.kr(-0.95, 0.95, leafTrig) * widthCtl);
        insectTrig = Dust.kr(shimmerCtl.linexp(0.0, 1.0, 0.1, 6.4) + (spinCtl * 2.6));
        insectEnv = Decay2.kr(insectTrig, 0.003, 0.03 + (shimmerCtl * 0.12));
        insectFreq = TExpRand.kr((root * 8.0).clip(1100.0, 2400.0), (root * 28.0).clip(2800.0, 9200.0), insectTrig);
        insects = Mix(SinOsc.ar(insectFreq * [1.0, 1.006], 0.0, [0.016, 0.012])) * insectEnv;
        insectStereo = Pan2.ar(HPF.ar(insects, 1200), TRand.kr(-0.85, 0.85, insectTrig) * widthCtl);
        callTrig = Dust.kr((0.012 + (auraCtl * 0.03) + ((1.0 - darkCtl) * 0.02)) * (0.5 + motionCtl));
        callEnv = Decay2.kr(callTrig, 0.02, 1.0 + (auraCtl * 1.4));
        callGlide = EnvGen.kr(Env([1.0, 1.22, 0.97], [0.07, 0.32], [2, -3]), callTrig);
        callFreq = TExpRand.kr((root * 2.2).clip(260.0, 680.0), (root * 5.0).clip(520.0, 2400.0), callTrig);
        calls = Mix(SinOsc.ar(callFreq * callGlide * [1.0, 1.5], 0.0, [0.03, 0.012])) * callEnv;
        callStereo = Pan2.ar(HPF.ar(calls, 240), TRand.kr(-0.88, 0.88, callTrig) * widthCtl);
        floor = LPF.ar(PinkNoise.ar(0.003) * LFNoise2.kr(0.09).range(0.2, 0.9), (root * (1.4 + darkCtl)).clip(80.0, 320.0));
        sig = canopy + (wind ! 2) + leafStereo + insectStereo + callStereo + (floor ! 2);
        globalLp = (darkCtl.linlin(0.0, 1.0, 7600.0, 1800.0) + (liftCtl * 600.0)).clip(1200.0, 8200.0);

        sig = LeakDC.ar(LPF.ar(sig, globalLp));
        verb = FreeVerb2.ar(sig[0], sig[1], 0.24 + (auraCtl * 0.18), 0.82 + (motionCtl * 0.08), darkCtl.linlin(0.0, 1.0, 0.72, 0.3));
        sig = ((sig * 0.76) + (verb * 0.82)) * ampCtl * gateEnv * motionLevel;
        Out.ar(out, sig);
    }).add;

    SynthDef(\spacewizardAmbientLabHalo, {
        |out=0, gate=1, rootFreq=74, amp=0.1, pan=0.0, lift=0.0, brightness=0.55,
         darkness=0.45, density=0.35, shimmer=0.35, width=0.82, grainScan=0.35,
         rhythm=0.22, ambience=0.72, overlay=0.0, sceneMorph=0.0,
         buf=0, basePitchHz=176.47, motionBus=0, motionRise=0.18, motionFall=5.0|
        var gateEnv = EnvGen.kr(Env.asr(4.0, 1.0, 8.0), gate, doneAction: 2);
        var motionLevel = LagUD.kr(In.kr(motionBus, 1).clip(0.0, 1.0), motionRise.clip(0.01, 12.0), motionFall.clip(0.01, 12.0));
        var root = Lag.kr(rootFreq.clip(36.0, 130.0), 2.4);
        var rootMidi = root.cpsmidi;
        var liftCtl = Lag.kr(lift.clip(-1.0, 1.0), 1.5);
        var brightCtl = Lag.kr(brightness.clip(0.0, 1.0), 1.2);
        var darkCtl = Lag.kr(darkness.clip(0.0, 1.0), 1.6);
        var densityCtl = Lag.kr(density.clip(0.0, 1.0), 1.2);
        var shimmerCtl = Lag.kr(shimmer.clip(0.0, 1.0), 1.0);
        var widthCtl = Lag.kr(width.clip(0.0, 1.0), 1.1);
        var rhythmCtl = Lag.kr(rhythm.clip(0.0, 1.0), 1.0);
        var ambienceCtl = Lag.kr(ambience.clip(0.0, 1.0), 1.8);
        var scanCtl = Lag.kr(grainScan.clip(0.0, 1.0), 1.6);
        var sceneCtl = Lag.kr(sceneMorph.clip(0.0, 1.0), 1.8);
        var ampCtl = Lag.kr(amp.clip(0.0, 0.5), 0.9);
        var warmDegrees = [0, 3, 7, 10, 14, 17, 22, 24];
        var brightDegrees = [0, 2, 7, 9, 12, 16, 21, 26];
        var liftOffsets = [-12, -7, -3, 0, 3, 7, 10, 14];
        var layers = Array.fill(8, { |index|
            var degree = SelectX.kr(sceneCtl, [warmDegrees[index], brightDegrees[index]])
                + ((scanCtl - 0.5) * [-3.0, -2.1, -1.2, -0.4, 0.7, 1.4, 2.4, 3.4][index]);
            var driftRate = 0.006 + (index * 0.0025) + (densityCtl * 0.018) + (scanCtl * 0.008);
            var pitchDrift = LFNoise2.kr(driftRate).range(-0.22, 0.22)
                + SinOsc.kr(driftRate * 0.37, index).range(-0.09, 0.09);
            var freq = (rootMidi + degree + (liftCtl * liftOffsets[index]) + pitchDrift).midicps;
            var pulseWidth = LFNoise2.kr(0.018 + (index * 0.008) + (shimmerCtl * 0.04)).range(0.34, 0.76);
            var body = Mix(SinOsc.ar(freq * [0.5, 1.0, 2.0], 0.0, [0.018, 0.015, 0.0045]));
            var haze = Mix(VarSaw.ar(
                freq * [0.5, 1.0 + (index * 0.0009)],
                0.0,
                [pulseWidth, 0.46],
                [0.011, 0.007]
            ));
            var color = (freq * (1.7 + (brightCtl * 4.2) + (ambienceCtl * 1.4) + (scanCtl * 0.8))).clip(140.0, 6800.0);
            Pan2.ar(RLPF.ar(body + haze, color, 0.22 + (darkCtl * 0.14)), ((index / 3.5) - 1.0) * widthCtl + pan);
        });
        var glintTrig = Dust.kr(rhythmCtl.linexp(0.0, 1.0, 0.002, 0.12) + shimmerCtl.linlin(0.0, 1.0, 0.0, 0.22));
        var glintDegree = SelectX.kr(
            ((sceneCtl * 7.0) + TRand.kr(-0.45, 0.45, glintTrig)).clip(0.0, 7.0),
            [7, 10, 12, 14, 17, 19, 24, 26]
        );
        var glintFreq = (rootMidi + glintDegree + liftCtl.linlin(-1.0, 1.0, -5.0, 11.0)).midicps
            * TExpRand.kr(0.992, 1.018, glintTrig);
        var glintEnv = Decay2.kr(glintTrig, 0.06, TExpRand.kr(2.4, 7.2, glintTrig));
        var glints = Mix(Ringz.ar(
            PinkNoise.ar(0.002) * glintEnv,
            glintFreq * [1.0, 1.5, 2.01],
            [2.8, 3.4, 4.2],
            [0.006, 0.004, 0.0018]
        ));
        var glintStereo = Pan2.ar(LPF.ar(HPF.ar(glints, 380.0), 3600.0), TRand.kr(-0.9, 0.9, glintTrig) * widthCtl);
        var breath = BPF.ar(
            PinkNoise.ar(0.002 + (densityCtl * 0.0015)),
            (root * (2.4 + (brightCtl * 6.0) + (shimmerCtl * 2.5) + (sceneCtl * 1.2))).clip(260.0, 4200.0),
            0.3,
            0.012 + (brightCtl * 0.012) + (ambienceCtl * 0.022)
        );
        var bed = Mix(layers);
        var comb = CombC.ar(bed * (0.08 + (ambienceCtl * 0.08)), 1.6, (0.2 + (scanCtl * 0.58) + LFNoise2.kr(0.025 + (densityCtl * 0.025)).range(-0.035, 0.035)).clip(0.16, 0.92), 3.8);
        var sig = bed + glintStereo + (breath ! 2) + comb;
        var globalLp = (darkCtl.linlin(0.0, 1.0, 4600.0, 900.0) + (brightCtl * 900.0) + (sceneCtl * 360.0) + (liftCtl * 260.0)).clip(650.0, 5200.0);
        var verb;

        sig = LeakDC.ar(LPF.ar(sig, globalLp));
        verb = FreeVerb2.ar(sig[0], sig[1], 0.22 + (ambienceCtl * 0.18), 0.78 + (ambienceCtl * 0.12), darkCtl.linlin(0.0, 1.0, 0.68, 0.34));
        sig = ((sig * 0.54) + (verb * 0.56)) * ampCtl * gateEnv * motionLevel;
        sig = CompanderD.ar(sig, 0.55, 1.0, 0.34, 0.01, 0.22);
        Out.ar(out, Limiter.ar(sig, 0.92, 0.01));
    }).add;

    SynthDef(\spacewizardAmbientLabAirports, {
        |out=0, gate=1, rootFreq=74, amp=0.1, pan=0.0, lift=0.0, brightness=0.55,
         darkness=0.45, density=0.35, shimmer=0.35, width=0.82, grainScan=0.35,
         rhythm=0.28, ambience=0.72, overlay=0.0, sceneMorph=0.0,
         buf=0, basePitchHz=176.47, motionBus=0, motionRise=0.18, motionFall=5.0|
        var gateEnv = EnvGen.kr(Env.asr(3.6, 1.0, 7.2), gate, doneAction: 2);
        var motionLevel = LagUD.kr(In.kr(motionBus, 1).clip(0.0, 1.0), motionRise.clip(0.01, 12.0), motionFall.clip(0.01, 12.0));
        var root = Lag.kr(rootFreq.clip(36.0, 130.0), 2.3);
        var rootMidi = root.cpsmidi;
        var liftCtl = Lag.kr(lift.clip(-1.0, 1.0), 1.4);
        var brightCtl = Lag.kr(brightness.clip(0.0, 1.0), 1.1);
        var darkCtl = Lag.kr(darkness.clip(0.0, 1.0), 1.5);
        var densityCtl = Lag.kr(density.clip(0.0, 1.0), 1.1);
        var shimmerCtl = Lag.kr(shimmer.clip(0.0, 1.0), 0.9);
        var widthCtl = Lag.kr(width.clip(0.0, 1.0), 1.0);
        var rhythmCtl = Lag.kr(rhythm.clip(0.0, 1.0), 1.0);
        var ambienceCtl = Lag.kr(ambience.clip(0.0, 1.0), 1.6);
        var scanCtl = Lag.kr(grainScan.clip(0.0, 1.0), 1.4);
        var sceneCtl = Lag.kr(sceneMorph.clip(0.0, 1.0), 1.6);
        var ampCtl = Lag.kr(amp.clip(0.0, 0.5), 0.9);
        var chordPath = ((sceneCtl * 3.45) + (scanCtl * 0.55) + LFNoise2.kr(0.008 + (densityCtl * 0.012)).range(-0.06, 0.06)).clip(0.0, 4.0);
        var warmChords = [
            [0, 0, -5, -2, 3],
            [3, 7, 0, 5, 10],
            [7, 10, 3, 7, 14],
            [10, 14, 7, 12, 17]
        ];
        var openChords = [
            [0, 0, -3, 2, 7],
            [2, 7, 4, 9, 14],
            [7, 12, 9, 16, 21],
            [11, 16, 14, 19, 26]
        ];
        var liftOffsets = [-8, -3, 0, 5];
        var voices = Array.fill(4, { |index|
            var warmDegree = SelectX.kr(chordPath, warmChords[index]);
            var openDegree = SelectX.kr(chordPath, openChords[index]);
            var degree = SelectX.kr(((sceneCtl * 0.6) + (brightCtl * 0.4)).clip(0.0, 1.0), [warmDegree, openDegree]);
            var freq = (rootMidi + degree + (liftCtl * liftOffsets[index])
                + LFNoise2.kr(0.015 + (index * 0.006) + (densityCtl * 0.018)).range(-0.1, 0.1)).midicps;
            var source = Mix(VarSaw.ar(
                freq * [0.5, 1.0, 1.004],
                0.0,
                LFNoise2.kr(0.028 + (index * 0.01)).range(0.28, 0.72),
                [0.016, 0.026, 0.012]
            ));
            var sine = Mix(SinOsc.ar(freq * [1.0, 2.0], 0.0, [0.014, 0.006]));
            source = RLPF.ar(
                source + sine,
                (freq * (2.2 + (brightCtl * 3.2) + (ambienceCtl * 1.1))).clip(180.0, 6200.0),
                0.12 + (darkCtl * 0.12)
            );
            Pan2.ar(source, ((index / 1.5) - 1.0) * widthCtl + pan);
        });
        var noteTrig = Dust.kr(rhythmCtl.linexp(0.0, 1.0, 0.006, 0.24) + shimmerCtl.linlin(0.0, 1.0, 0.0, 0.12));
        var noteDegree = SelectX.kr(
            ((sceneCtl * 8.0) + (Demand.kr(noteTrig, 0, Dseq([0, 1, -1, 2, 0, -2, 1, 0], inf)) * (0.16 + (rhythmCtl * 0.18)))).clip(0.0, 8.0),
            [0, 2, 3, 7, 9, 12, 14, 19, 24]
        );
        var noteFreq = (rootMidi + noteDegree + liftCtl.linlin(-1.0, 1.0, -5.0, 6.0)).midicps
            * TExpRand.kr(0.996, 1.01, noteTrig);
        var noteEnv = Decay2.kr(noteTrig, 0.12, TExpRand.kr(4.8, 12.0, noteTrig));
        var pluck = Pluck.ar(
            PinkNoise.ar(0.006) * noteEnv,
            noteTrig,
            0.12,
            (1.0 / noteFreq).clip(0.0008, 0.12),
            7.0 + (ambienceCtl * 5.0),
            0.28 + (darkCtl * 0.38)
        );
        var bell = Mix(Ringz.ar(pluck, noteFreq * [1.0, 2.01, 2.99], [3.2, 4.5, 5.0], [0.014, 0.004, 0.0014]));
        var bellStereo = Pan2.ar(LPF.ar(HPF.ar(bell, 140.0), 2800.0), LFNoise2.kr(0.07).range(-0.55, 0.55) * widthCtl);
        var floor = LPF.ar(SinOsc.ar(root * 0.5, 0.0, 0.035 + (darkCtl * 0.025)), root * (1.8 + (ambienceCtl * 1.0)));
        var air = BPF.ar(
            PinkNoise.ar(0.0015 + (brightCtl * 0.0018)),
            (root * (4.0 + (brightCtl * 9.0) + (sceneCtl * 2.0))).clip(520.0, 4600.0),
            0.26,
            0.008 + (brightCtl * 0.018)
        );
        var sig = (Mix(voices) * 1.08) + (bellStereo * 0.42) + (floor ! 2) + (air ! 2);
        var verb;

        sig = LeakDC.ar(LPF.ar(sig, (darkCtl.linlin(0.0, 1.0, 4600.0, 1000.0) + (brightCtl * 700.0)).clip(750.0, 5200.0)));
        verb = FreeVerb2.ar(sig[0], sig[1], 0.28 + (ambienceCtl * 0.18), 0.88 + (ambienceCtl * 0.08), darkCtl.linlin(0.0, 1.0, 0.72, 0.38));
        sig = ((sig * 0.58) + (verb * 0.55)) * ampCtl * gateEnv * motionLevel;
        sig = CompanderD.ar(sig, 0.55, 1.0, 0.34, 0.01, 0.22);
        Out.ar(out, Limiter.ar(sig, 0.92, 0.01));
    }).add;

    SynthDef(\spacewizardAmbientLabGranular, {
        |out=0, gate=1, rootFreq=74, amp=0.1, pan=0.0, lift=0.0, brightness=0.55,
         darkness=0.45, density=0.35, shimmer=0.35, width=0.82, grainScan=0.35,
         rhythm=0.22, ambience=0.78, overlay=0.0, sceneMorph=0.0,
         buf=0, basePitchHz=176.47, motionBus=0, motionRise=0.18, motionFall=5.0|
        var gateEnv = EnvGen.kr(Env.asr(4.0, 1.0, 8.0), gate, doneAction: 2);
        var motionLevel = LagUD.kr(In.kr(motionBus, 1).clip(0.0, 1.0), motionRise.clip(0.01, 12.0), motionFall.clip(0.01, 12.0));
        var root = Lag.kr(rootFreq.clip(36.0, 130.0), 2.1);
        var liftCtl = Lag.kr(lift.clip(-1.0, 1.0), 1.3);
        var brightCtl = Lag.kr(brightness.clip(0.0, 1.0), 1.0);
        var darkCtl = Lag.kr(darkness.clip(0.0, 1.0), 1.4);
        var densityCtl = Lag.kr(density.clip(0.0, 1.0), 0.9);
        var shimmerCtl = Lag.kr(shimmer.clip(0.0, 1.0), 0.8);
        var widthCtl = Lag.kr(width.clip(0.0, 1.0), 0.9);
        var rhythmCtl = Lag.kr(rhythm.clip(0.0, 1.0), 0.9);
        var ambienceCtl = Lag.kr(ambience.clip(0.0, 1.0), 1.5);
        var scanCtl = Lag.kr(grainScan.clip(0.0, 1.0), 1.3);
        var sceneCtl = Lag.kr(sceneMorph.clip(0.0, 1.0), 1.3);
        var ampCtl = Lag.kr(amp.clip(0.0, 0.5), 0.9);
        var hasBuf = (BufFrames.kr(buf) > 2);
        var baseRate = (root / basePitchHz.clip(80.0, 320.0)).clip(0.2, 1.2);
        var trigRate = densityCtl.linexp(0.0, 1.0, 4.0, 22.0) + rhythmCtl.linlin(0.0, 1.0, 0.0, 4.0);
        var trigA = Impulse.kr(trigRate);
        var trigB = Dust.kr((trigRate * 0.36) + shimmerCtl.linlin(0.0, 1.0, 0.1, 2.2));
        var posA = (scanCtl + LFNoise2.kr(0.04 + (densityCtl * 0.05)).range(-0.08, 0.08)).wrap(0.02, 0.96);
        var posB = (sceneCtl + LFNoise2.kr(0.025 + (shimmerCtl * 0.05)).range(-0.12, 0.12)).wrap(0.02, 0.96);
        var durA = densityCtl.linlin(0.0, 1.0, 0.42, 0.09);
        var durB = densityCtl.linlin(0.0, 1.0, 0.72, 0.18);
        var rateA = baseRate * (1.0 + (liftCtl * 0.12)) * TExpRand.kr(0.994, 1.018 + (shimmerCtl * 0.015), trigA);
        var rateB = baseRate * SelectX.kr(sceneCtl, [0.45, 1.1]) * TExpRand.kr(0.985, 1.014 + (shimmerCtl * 0.012), trigB);
        var grainA = GrainBuf.ar(2, trigA, durA, buf, rateA, posA, 2, LFNoise2.kr(0.1).range(widthCtl.neg, widthCtl));
        var grainB = GrainBuf.ar(2, trigB, durB, buf, rateB, posB, 2, LFNoise2.kr(0.13).range(widthCtl.neg, widthCtl));
        var fallbackRoot = root.cpsmidi;
        var fallback = Splay.ar(Array.fill(5, { |index|
            var freq = (fallbackRoot + [0, 3, 7, 10, 14][index] + (liftCtl * [-7, -3, 0, 4, 9][index])
                + LFNoise2.kr(0.018 + (index * 0.006)).range(-0.12, 0.12)).midicps;
            var voice = Formant.ar(
                freq,
                (freq * (3.2 + (brightCtl * 2.6))).clip(freq + 20.0, 2600.0),
                (freq * (1.8 + (sceneCtl * 1.2))).clip(160.0, 2400.0),
                0.026
            );
            LPF.ar(voice, (freq * 4.2).clip(420.0, 3400.0))
        }), widthCtl, 0.8, pan);
        var cloud = ((grainA * 0.68) + (grainB * 0.5)) * hasBuf + (fallback * 0.85 * (1.0 - hasBuf));
        var air = BPF.ar(
            PinkNoise.ar(0.0016 + (shimmerCtl * 0.0018)),
            (root * (4.2 + (brightCtl * 8.0))).clip(520.0, 4600.0),
            0.28,
            0.014 + (ambienceCtl * 0.024)
        );
        var sig = cloud + (air ! 2);
        var verb;

        sig = LeakDC.ar(LPF.ar(sig, (darkCtl.linlin(0.0, 1.0, 4600.0, 900.0) + (brightCtl * 650.0)).clip(700.0, 5200.0)));
        verb = FreeVerb2.ar(sig[0], sig[1], 0.32 + (ambienceCtl * 0.2), 0.9 + (ambienceCtl * 0.08), darkCtl.linlin(0.0, 1.0, 0.7, 0.34));
        sig = ((sig * 0.56) + (verb * 0.68)) * ampCtl * gateEnv * motionLevel;
        sig = CompanderD.ar(sig, 0.55, 1.0, 0.34, 0.01, 0.22);
        Out.ar(out, Limiter.ar(sig, 0.92, 0.01));
    }).add;

    SynthDef(\spacewizardAmbientLabHexBells, {
        |out=0, gate=1, rootFreq=74, amp=0.1, pan=0.0, lift=0.0, brightness=0.55,
         darkness=0.45, density=0.35, shimmer=0.35, width=0.82, grainScan=0.35,
         rhythm=0.22, ambience=0.78, overlay=0.0, sceneMorph=0.0,
         buf=0, basePitchHz=176.47, motionBus=0, motionRise=0.18, motionFall=5.0|
        var gateEnv = EnvGen.kr(Env.asr(2.8, 1.0, 6.0), gate, doneAction: 2);
        var motionLevel = LagUD.kr(In.kr(motionBus, 1).clip(0.0, 1.0), motionRise.clip(0.01, 12.0), motionFall.clip(0.01, 12.0));
        var root = Lag.kr(rootFreq.clip(36.0, 120.0), 2.1);
        var liftCtl = Lag.kr(lift.clip(-1.0, 1.0), 1.2);
        var brightCtl = Lag.kr(brightness.clip(0.0, 1.0), 0.9);
        var darkCtl = Lag.kr(darkness.clip(0.0, 1.0), 1.2);
        var densityCtl = Lag.kr(density.clip(0.0, 1.0), 0.9);
        var shimmerCtl = Lag.kr(shimmer.clip(0.0, 1.0), 0.8);
        var widthCtl = Lag.kr(width.clip(0.0, 1.0), 0.9);
        var rhythmCtl = Lag.kr(rhythm.clip(0.0, 1.0), 0.8);
        var ambienceCtl = Lag.kr(ambience.clip(0.0, 1.0), 1.4);
        var scanCtl = Lag.kr(grainScan.clip(0.0, 1.0), 1.0);
        var sceneCtl = Lag.kr(sceneMorph.clip(0.0, 1.0), 1.1);
        var ampCtl = Lag.kr(amp.clip(0.0, 0.45), 0.9);
        var sequence = [0, 2, 5, 3, 4, 1, 2, 5, 0];
        var durs = [1, 1, 2, 1, 1, 1, 2, 1, 2] / 4;
        var scale = (Array.fill(5, { |index| Scale.hexDorian.degrees + (index * 12) }).flatten + 24).collect { |midi| midi.midicps };
        var stepRate = (rhythmCtl.linexp(0.0, 1.0, 0.32, 1.25) + densityCtl.linlin(0.0, 1.0, 0.0, 0.38) + (shimmerCtl * 0.12)).clip(0.22, 1.65);
        var stepTrig = Impulse.kr(stepRate);
        var phraseTrig = PulseDivider.kr(stepTrig, 9);
        var prob = (0.22 + (densityCtl * 0.16) + (rhythmCtl * 0.06) + (shimmerCtl * 0.03)).clip(0.12, 0.52);
        var noteTrig = stepTrig * (TRand.kr(0.0, 1.0, stepTrig) < prob);
        var seqNote = Demand.kr(stepTrig, 0, Dseq(sequence, inf));
        var durShape = Demand.kr(stepTrig, 0, Dseq(durs, inf));
        var phraseTranspose = Demand.kr(phraseTrig, 0, Dseq([0, 6, 8, 12, 9, 6, 0, 12], inf));
        var travelTranspose = sceneCtl.linlin(0.0, 1.0, 0.0, 14.0) + scanCtl.linlin(0.0, 1.0, -2.2, 2.2);
        var transpose = (travelTranspose + (phraseTranspose * rhythmCtl * 0.1) + (liftCtl * 3.0)).clip(0, 18);
        var noteIndex = (seqNote + transpose).clip(0, 24);
        var freq = SelectX.kr(noteIndex, scale) * (root / 74.0).clip(0.55, 2.1) * TExpRand.kr(0.998, 1.004 + (shimmerCtl * 0.005), noteTrig);
        var env = EnvGen.kr(Env.perc(0.12, (durShape * (4.8 + (ambienceCtl * 4.2))).clip(1.4, 8.0), 1.0, -6.0), noteTrig);
        var strikeEnv = Decay2.kr(noteTrig, 0.012, 0.12 + (shimmerCtl * 0.04));
        var notePan = LFNoise2.kr(0.08 + (rhythmCtl * 0.08)).range(widthCtl.neg, widthCtl);
        var lfo = SinOsc.kr(5.5 + (shimmerCtl * 9.0)).range(0.06, 0.16);
        var level = transpose.linlin(0.0, 34.0, 1.0, 0.22);
        var travelBed = Splay.ar(Array.fill(4, { |index|
            var degree = SelectX.kr(sceneCtl, [[0, 3, 7, 10][index], [0, 7, 12, 19][index]])
                + (liftCtl * [-4, -1, 2, 6][index])
                + (scanCtl.linlin(0.0, 1.0, -1.8, 1.8) * [-0.5, 0.3, 0.8, 1.3][index]);
            var bedFreq = (root.cpsmidi + degree).midicps;
            var tone = Mix(SinOsc.ar(bedFreq * [0.5, 1.0, 1.005], 0.0, [0.014, 0.018, 0.006]));
            LPF.ar(tone, (bedFreq * (2.0 + (brightCtl * 2.8) + (sceneCtl * 1.2))).clip(150.0, 3200.0));
        }), widthCtl * 0.72, 0.58, pan);
        var partials = Mix(SinOsc.ar(
            freq * [1.0, 2.01, 2.99, 4.02],
            pi / 36,
            [0.2, 0.032, 0.011, 0.0038] * level
        ));
        var strike = Mix(Ringz.ar(
            PinkNoise.ar(0.0012) * strikeEnv,
            freq * [1.0, 2.0, 3.01],
            [0.42, 0.66, 0.84],
            [0.01, 0.004, 0.0015] * level
        ));
        var bell = Pan2.ar((partials * lfo * env) + strike, (pan + notePan).clip(-0.95, 0.95));
        var floor = LPF.ar(SinOsc.ar(root * 0.5, 0.0, 0.018 + (darkCtl * 0.014)), root * (1.6 + ambienceCtl));
        var delayTime = (0.25 + (sceneCtl * 0.34) + (scanCtl * 0.08)).clip(0.18, 0.72);
        var comb = CombC.ar(bell, 0.8, [delayTime, delayTime * 0.75].clip(0.08, 0.78), 1.5 + (ambienceCtl * 2.0)) * (0.16 + (ambienceCtl * 0.16));
        var space = AllpassC.ar(comb, 0.7, [0.19, 0.31] + (scanCtl * [0.05, 0.09]), 2.6 + (ambienceCtl * 2.2));
        var sig = (bell * 0.2) + (space * 0.42) + (travelBed * 1.62) + (floor ! 2);
        var globalLp = (darkCtl.linlin(0.0, 1.0, 4200.0, 820.0) + (brightCtl * 850.0) + (sceneCtl * 420.0) + (scanCtl * 260.0)).clip(620.0, 5200.0);
        var verb;

        sig = LeakDC.ar(LPF.ar(sig, globalLp));
        verb = FreeVerb2.ar(sig[0], sig[1], 0.24 + (ambienceCtl * 0.2), 0.82 + (ambienceCtl * 0.12), darkCtl.linlin(0.0, 1.0, 0.7, 0.38));
        sig = ((sig * 1.68) + (verb * 1.08)) * ampCtl * gateEnv * motionLevel;
        sig = CompanderD.ar(sig, 0.55, 1.0, 0.34, 0.01, 0.2);
        Out.ar(out, Limiter.ar(sig, 0.92, 0.01));
    }).add;

    SynthDef(\spacewizardAmbientLabTapeSwells, {
        |out=0, gate=1, rootFreq=74, amp=0.1, pan=0.0, lift=0.0, brightness=0.55,
         darkness=0.45, density=0.35, shimmer=0.35, width=0.82, grainScan=0.35,
         rhythm=0.22, ambience=0.78, overlay=0.0, sceneMorph=0.0,
         buf=0, basePitchHz=176.47, motionBus=0, motionRise=0.18, motionFall=5.0|
        var gateEnv = EnvGen.kr(Env.asr(4.0, 1.0, 8.0), gate, doneAction: 2);
        var motionLevel = LagUD.kr(In.kr(motionBus, 1).clip(0.0, 1.0), motionRise.clip(0.01, 12.0), motionFall.clip(0.01, 12.0));
        var root = Lag.kr(rootFreq.clip(36.0, 120.0), 2.1);
        var liftCtl = Lag.kr(lift.clip(-1.0, 1.0), 1.3);
        var brightCtl = Lag.kr(brightness.clip(0.0, 1.0), 1.0);
        var darkCtl = Lag.kr(darkness.clip(0.0, 1.0), 1.4);
        var densityCtl = Lag.kr(density.clip(0.0, 1.0), 1.1);
        var shimmerCtl = Lag.kr(shimmer.clip(0.0, 1.0), 0.9);
        var widthCtl = Lag.kr(width.clip(0.0, 1.0), 1.0);
        var rhythmCtl = Lag.kr(rhythm.clip(0.0, 1.0), 0.9);
        var ambienceCtl = Lag.kr(ambience.clip(0.0, 1.0), 1.6);
        var scanCtl = Lag.kr(grainScan.clip(0.0, 1.0), 1.1);
        var sceneCtl = Lag.kr(sceneMorph.clip(0.0, 1.0), 1.2);
        var ampCtl = Lag.kr(amp.clip(0.0, 0.45), 1.0);
        var notes = [28, 40, 45, 50, 55, 60, 64, 67, 71, 62, 65, 69, 72];
        var transposeScene = sceneCtl.linlin(0.0, 1.0, 0.0, 7.0) + scanCtl.linlin(0.0, 1.0, -2.0, 2.0);
        var transposeDrift = Demand.kr(Impulse.kr((0.018 + (rhythmCtl * 0.05)).clip(0.01, 0.08)), 0, Dseq([0, 0, 5, 7, 5, 0], inf));
        var transpose = (transposeScene + (transposeDrift * rhythmCtl * 0.2) + (liftCtl * 3.5)).clip(-5, 14);
        var voices = Array.fill(4, { |index|
            var rate = (0.08 + (densityCtl * 0.22) + (rhythmCtl * 0.08) + (shimmerCtl * 0.03)) * [0.66, 0.84, 1.0, 1.16][index];
            var trig = Dust.kr(rate.clip(0.006, 0.18));
            var walk = Demand.kr(trig, 0, Dbrown(0, notes.size - 1, 1, inf));
            var durChoice = Demand.kr(trig, 0, Dwrand([1, 2, 8], [0.7, 0.2, 0.1], inf));
            var durScale = (durChoice * (1.5 + (ambienceCtl * 2.4))).clip(1.0, 16.0);
            var note = SelectX.kr(walk, notes) + transpose;
            var freq = note.midicps * (root / 74.0).clip(0.55, 2.1) * TExpRand.kr(0.998, 1.006 + (shimmerCtl * 0.006), trig);
            var panPos = LFNoise2.kr(rate.clip(0.02, 0.3)).range(widthCtl.neg, widthCtl) + pan;
            var detuneEnv = EnvGen.kr(Env([0.0, 1.0], [1.0], 2), trig, levelScale: 0.002 + (shimmerCtl * 0.006), timeScale: durScale);
            var lfoRate = TExpRand.kr(4.0, 8.0 + (shimmerCtl * 6.0), trig);
            var lfo = SinOsc.kr(lfoRate).range(1.0 - detuneEnv, 1.0 + detuneEnv);
            var env = EnvGen.kr(
                Env([0.001, 1.0, 1.0, 0.001], [0.34, 0.36, 0.3], [2, 0, -5]),
                trig,
                timeScale: durScale
            );
            var filterEnv = EnvGen.kr(
                Env([1.0, 2.2 + (brightCtl * 5.5), 1.25 + (brightCtl * 2.0)], [0.52, 0.48], [2, -2]),
                trig,
                timeScale: durScale
            );
            var level = note.linlin(30.0, 96.0, 0.62, 0.12).clip(0.09, 0.62);
            var source = Mix(VarSaw.ar(
                freq * lfo * [0.5, 1.0, 1.004],
                0.0,
                [0.52, 0.45, 0.39],
                [0.028, 0.042, 0.016]
            ));
            var body = RLPF.ar(
                source,
                (freq * filterEnv).clip(100.0, 4200.0),
                0.16 + (darkCtl * 0.18)
            );
            Pan2.ar(body * env * level, panPos.clip(-0.95, 0.95));
        });
        var dry = Mix(voices);
        var travelBed = Splay.ar(Array.fill(4, { |index|
            var degree = SelectX.kr(sceneCtl, [[0, 5, 10, 15][index], [0, 7, 12, 19][index]])
                + (liftCtl * [-5, -1, 3, 7][index])
                + (scanCtl.linlin(0.0, 1.0, -2.0, 2.0) * [-0.5, 0.2, 0.8, 1.2][index]);
            var bedFreq = (root.cpsmidi + degree).midicps;
            var tone = Mix(VarSaw.ar(
                bedFreq * [0.5, 1.0, 1.003],
                0.0,
                [0.56, 0.46, 0.4],
                [0.018, 0.026, 0.008]
            ));
            RLPF.ar(tone, (bedFreq * (2.0 + (brightCtl * 2.4) + (sceneCtl * 1.0))).clip(130.0, 3000.0), 0.18 + (darkCtl * 0.16));
        }), widthCtl * 0.7, 0.54, pan);
        var low = LPF.ar(SinOsc.ar(root * 0.5, 0.0, 0.016 + (darkCtl * 0.012)), root * (1.4 + (ambienceCtl * 0.9)));
        var tapeMod = SinOsc.kr(0.035 + (shimmerCtl * 0.1)).range(0.0, 0.012 + (shimmerCtl * 0.015))
            + LFNoise2.kr(0.06 + (densityCtl * 0.08)).range(0.0, 0.006 + (scanCtl * 0.012));
        var tape = DelayC.ar(dry, 0.32, 0.08 + tapeMod);
        var delayTime = (0.18 + (scanCtl * 0.16) + (sceneCtl * 0.05)).clip(0.12, 0.42);
        var delay = LPF.ar(CombC.ar(tape, 0.8, [delayTime, delayTime * 1.333].clip(0.09, 0.72), 2.4 + (ambienceCtl * 4.2)), 900.0 + (brightCtl * 900.0));
        var cloud = AllpassC.ar(delay, 0.7, [0.17, 0.29] + (scanCtl * [0.035, 0.055]), 3.0 + (ambienceCtl * 3.4));
        var sig = (dry * 0.46) + (travelBed * 1.26) + (delay * (0.16 + (ambienceCtl * 0.18))) + (cloud * (0.12 + (ambienceCtl * 0.16))) + (low ! 2);
        var globalLp = (darkCtl.linlin(0.0, 1.0, 4200.0, 820.0) + (brightCtl * 700.0)).clip(620.0, 5000.0);
        var verb;

        sig = LeakDC.ar(HPF.ar(LPF.ar(sig, globalLp), 40.0));
        sig = CompanderD.ar(sig.tanh, 0.5, 1.0, 0.42, 0.01, 0.28);
        verb = FreeVerb2.ar(sig[0], sig[1], 0.28 + (ambienceCtl * 0.18), 0.84 + (ambienceCtl * 0.12), darkCtl.linlin(0.0, 1.0, 0.72, 0.38));
        sig = ((sig * 1.55) + (verb * 1.22)) * ampCtl * gateEnv * motionLevel;
        sig = CompanderD.ar(sig, 0.55, 1.0, 0.34, 0.01, 0.22);
        Out.ar(out, Limiter.ar(sig, 0.92, 0.01));
    }).add;

    SynthDef(\spacewizardAmbientLabTapeScript, {
        |out=0, gate=1, rootFreq=74, amp=0.1, pan=0.0, lift=0.0, brightness=0.55,
         darkness=0.45, density=0.35, shimmer=0.35, width=0.82, grainScan=0.35,
         rhythm=0.22, ambience=0.78, overlay=0.0, sceneMorph=0.0,
         buf=0, basePitchHz=176.47, motionBus=0, motionRise=0.18, motionFall=5.0|
        var gateEnv = EnvGen.kr(Env.asr(3.8, 1.0, 8.0), gate, doneAction: 2);
        var motionLevel = LagUD.kr(In.kr(motionBus, 1).clip(0.0, 1.0), motionRise.clip(0.01, 12.0), motionFall.clip(0.01, 12.0));
        var root = Lag.kr(rootFreq.clip(36.0, 120.0), 2.4);
        var rootRatio = (root / 74.0).clip(0.55, 1.25);
        var liftCtl = Lag.kr(lift.clip(-1.0, 1.0), 1.8);
        var brightCtl = Lag.kr(brightness.clip(0.0, 1.0), 1.4);
        var darkCtl = Lag.kr(darkness.clip(0.0, 1.0), 1.8);
        var densityCtl = Lag.kr(density.clip(0.0, 1.0), 1.4);
        var shimmerCtl = Lag.kr(shimmer.clip(0.0, 1.0), 1.1);
        var widthCtl = Lag.kr(width.clip(0.0, 1.0), 1.3);
        var rhythmCtl = Lag.kr(rhythm.clip(0.0, 1.0), 1.1);
        var ambienceCtl = Lag.kr(ambience.clip(0.0, 1.0), 1.8);
        var scanCtl = Lag.kr(grainScan.clip(0.0, 1.0), 1.4);
        var sceneCtl = Lag.kr(sceneMorph.clip(0.0, 1.0), 1.6);
        var ampCtl = Lag.kr(amp.clip(0.0, 0.45), 0.9);
        var notes = [33, 45, 50, 55, 60, 65, 71, 76, 81, 74, 79, 84, 89];
        var baseTranspose = (sceneCtl.linlin(0.0, 1.0, -12.0, 2.0) + scanCtl.linlin(0.0, 1.0, -2.0, 2.0) + (liftCtl * 5.0)).clip(-18.0, 8.0);
        var widthMotion = widthCtl.linlin(0.0, 1.0, 0.12, 0.82);
        var streams = Array.fill(3, { |index|
            var tempo = (0.62 + (densityCtl * 0.72) + (rhythmCtl * 0.45)) * [0.78, 1.0, 1.17][index];
            var trig = TDuty.kr(
                Dwrand([0.166667, 0.333333, 1.333333], [0.7, 0.2, 0.1], inf) / tempo.max(0.2),
                0,
                1
            );
            var dur = Demand.kr(trig, 0, Dwrand([1, 2, 8], [0.7, 0.2, 0.1], inf));
            var walk = Demand.kr(trig, 0, Dbrown(0, notes.size - 1, 1, inf));
            var longTrig = trig * (dur > 6.0) * (TRand.kr(0.0, 1.0, trig) < (0.18 + (sceneCtl * 0.24)));
            var phraseTranspose = Demand.kr(longTrig, 0, Dseq([0, 5, 7, 0, 5, 0, 7, 0], inf));
            var note = Select.kr(walk, notes) + baseTranspose + (phraseTranspose * (0.18 + (rhythmCtl * 0.22)));
            var durScale = (dur * (1.8 + (ambienceCtl * 3.2))).clip(1.2, 18.0);
            var freq = (note.midicps * rootRatio * TExpRand.kr(0.996, 1.004 + (shimmerCtl * 0.004), trig)).clip(38.0, 760.0);
            var lenv = EnvGen.kr(Env.new([0, 1], [durScale], 2), trig, levelScale: 0.003 + (shimmerCtl * 0.006));
            var lfo = SinOsc.kr(TExpRand.kr(3.0, 7.0 + (shimmerCtl * 4.0), trig)).range(1.0 - lenv, 1.0 + lenv);
            var env = EnvGen.kr(
                Env.new([0.001, 1, 1, 0.001], [0.28, 0.42, 0.3], [\lin, \exp, \exp]),
                trig,
                timeScale: durScale
            );
            var freqScale = (1.45 + (brightCtl * 3.2) + (sceneCtl * 0.55)).clip(1.1, 5.2);
            var fenv = EnvGen.kr(
                Env.new([1.0, freqScale, freqScale * 0.72], [0.5, 0.5], [\exp, \lin]),
                trig,
                timeScale: durScale
            );
            var level = note.linlin(25.0, 92.0, 0.72, 0.08).clip(0.08, 0.72);
            var source = Saw.ar(freq * lfo, 0.052);
            var filter = MoogFF.ar(
                source,
                (fenv * freq * (0.84 + (brightCtl * 0.48))).clip(95.0, 3600.0),
                2.2 + (darkCtl * 0.46)
            );
            var panPos = LFNoise2.kr((0.025 + (index * 0.01) + (densityCtl * 0.035))).range(widthMotion.neg, widthMotion) + pan;

            Pan2.ar(filter * env * level, panPos.clip(-0.9, 0.9));
        });
        var dry = Mix(streams);
        var floor = LPF.ar(SinOsc.ar(root * 0.5, 0.0, 0.02 + (darkCtl * 0.02)), root * (1.2 + ambienceCtl));
        var delayTime = (0.245 + sceneCtl.linlin(0.0, 1.0, -0.055, 0.085) + scanCtl.linlin(0.0, 1.0, -0.018, 0.018)).clip(0.12, 0.46);
        var delay = LPF.ar(CombC.ar(dry, 0.5, delayTime, 5.0 + (ambienceCtl * 2.5)), 900.0 + (brightCtl * 1100.0)) * (0.26 + (rhythmCtl * 0.2));
        var reverbIn = dry + delay + (floor ! 2);
        var reverb = FreeVerb2.ar(reverbIn[0], reverbIn[1], 0.2 + (ambienceCtl * 0.24), 0.84 + (ambienceCtl * 0.1), darkCtl.linlin(0.0, 1.0, 0.64, 0.38));
        var tapeTrig = Dust.kr(0.08 + (shimmerCtl * 0.22) + (densityCtl * 0.12));
        var tapeEnv = EnvGen.kr(
            Env.perc(
                TRand.kr(0.5, 1.3, tapeTrig),
                TRand.kr(1.0, 4.0, tapeTrig),
                TRand.kr(0.002, 0.009 + (shimmerCtl * 0.008), tapeTrig)
            ),
            tapeTrig
        );
        var tapeDelay = (0.08 + (widthCtl * 0.018) + (scanCtl * 0.035) + tapeEnv).clip(0.02, 0.92);
        var sig = (reverbIn * 0.64) + (reverb * (0.28 + (ambienceCtl * 0.22)));
        var globalLp = (darkCtl.linlin(0.0, 1.0, 3600.0, 780.0) + (brightCtl * 680.0)).clip(620.0, 4300.0);

        sig = DelayC.ar(sig, 1.0, tapeDelay);
        sig = Compander.ar(sig, sig, 0.5, 1.0, 0.125, 0.01, 0.01).tanh;
        sig = LeakDC.ar(HPF.ar(LPF.ar(sig, globalLp), 40.0));
        sig = Limiter.ar(sig * (16.dbamp) * ampCtl * gateEnv * motionLevel, 0.92, 0.01);
        Out.ar(out, sig);
    }).add;

    SynthDef(\spacewizardAmbientLabVowelOverlay, {
        |out=0, gate=1, rootFreq=110, amp=0.0, pan=0.0, brightness=0.55,
         shimmer=0.3, width=0.7, motionBus=0, motionRise=0.18, motionFall=5.0|
        var gateEnv = EnvGen.kr(Env.asr(1.2, 1.0, 2.6), gate, doneAction: 2);
        var motionLevel = LagUD.kr(In.kr(motionBus, 1).clip(0.0, 1.0), motionRise.clip(0.01, 12.0), motionFall.clip(0.01, 12.0));
        var root = Lag.kr(rootFreq.clip(70.0, 420.0), 0.8);
        var brightCtl = Lag.kr(brightness.clip(0.0, 1.0), 0.8);
        var shimmerCtl = Lag.kr(shimmer.clip(0.0, 1.0), 0.8);
        var widthCtl = Lag.kr(width.clip(0.0, 1.0), 0.8);
        var ampCtl = Lag.kr(amp.clip(0.0, 0.1), 0.6);
        var source = Mix(VarSaw.ar(
            root * [0.5, 1.0, 1.003],
            0.0,
            [0.62, 0.48, 0.44],
            [0.03, 0.035, 0.022]
        ));
        var vowel = Mix([
            Formant.ar(root, (root * (4.8 + (brightCtl * 3.4))).clip(root + 20.0, 3200.0), (root * 2.0).clip(220.0, 1800.0), 0.08),
            Formant.ar(root * 0.5, (root * (3.2 + (brightCtl * 2.4))).clip(root + 20.0, 2400.0), (root * 1.5).clip(180.0, 1400.0), 0.052)
        ]);
        var air = BPF.ar(PinkNoise.ar(0.0016 + (shimmerCtl * 0.0018)), 700.0 + (brightCtl * 1200.0), 0.34);
        var sig = LPF.ar((source * 0.45) + vowel + (air * 0.12), 1200.0 + (brightCtl * 1800.0));

        sig = Splay.ar([
            sig,
            DelayC.ar(sig, 0.08, 0.019 + (shimmerCtl * 0.004)),
            DelayC.ar(sig, 0.08, 0.035 + (widthCtl * 0.006))
        ], widthCtl * 0.36, ampCtl * gateEnv * motionLevel, pan);
        sig = FreeVerb2.ar(sig[0], sig[1], 0.28 + (shimmerCtl * 0.08), 0.86, 0.42);
        sig = CompanderD.ar(LeakDC.ar(sig), 0.55, 1.0, 0.38, 0.01, 0.18);
        Out.ar(out, Limiter.ar(sig, 0.92, 0.01));
    }).add;

    SynthDef(\spacewizardAmbientLabPentatonicOverlay, {
        |out=0, gate=1, freq=90, amp=0.0, pan=0.0, cutoff=1050, rq=0.32,
         drive=1.8, glide=0.8, width=0.56, tone=0.42, motionBus=0, motionRise=0.18, motionFall=5.0|
        var gateEnv = EnvGen.kr(Env.asr(0.75, 1.0, 2.8), gate, doneAction: 2);
        var motionLevel = LagUD.kr(In.kr(motionBus, 1).clip(0.0, 1.0), motionRise.clip(0.01, 12.0), motionFall.clip(0.01, 12.0));
        var freqCtl = Lag.kr(freq.clip(42.0, 190.0), glide.clip(0.18, 2.6));
        var ampCtl = Lag.kr(amp.clip(0.0, 0.35), 0.34);
        var cutoffCtl = Lag.kr(cutoff.clip(360.0, 2800.0), 0.62);
        var toneCtl = Lag.kr(tone.clip(0.0, 1.0), 0.9);
        var widthCtl = Lag.kr(width.clip(0.0, 1.0), 0.9);
        var body = Mix([
            Pulse.ar(freqCtl * 0.5, 0.48, 0.12),
            SinOsc.ar(freqCtl * 0.5, 0.0, 0.1),
            SinOsc.ar(freqCtl, 0.0, 0.085),
            VarSaw.ar(freqCtl * [0.5, 1.0, 1.003], 0.0, [0.58, 0.48, 0.42], [0.08, 0.12, 0.04]).sum
        ]);
        var shaped = ((body * drive.clip(0.8, 3.2)).tanh * 0.76) + (body * 0.24);
        var filtered = RLPF.ar(shaped, cutoffCtl, rq.clip(0.12, 0.72));
        var lowBloom = LPF.ar(SinOsc.ar(freqCtl * [0.25, 0.5], 0.0, [0.08, 0.055]).sum, 260.0 + (toneCtl * 360.0));
        var sig = filtered + lowBloom;

        sig = Splay.ar([
            sig,
            DelayC.ar(sig, 0.12, 0.028 + (widthCtl * 0.012)),
            DelayC.ar(sig, 0.12, 0.047 + (widthCtl * 0.016))
        ], widthCtl * 0.34, ampCtl * 3.2 * gateEnv * motionLevel, pan);
        sig = FreeVerb2.ar(sig[0], sig[1], 0.18 + (toneCtl * 0.1), 0.72, 0.48);
        sig = CompanderD.ar(LeakDC.ar(LPF.ar(sig, 3200.0)), 0.55, 1.0, 0.42, 0.01, 0.22);
        Out.ar(out, Limiter.ar(sig, 0.9, 0.01));
    }).add;

    SynthDef(\spacewizardAmbientLabAnalyzer, {
        |inBus=0, rate=10, replyID=9301|
        var sig = InFeedback.ar(inBus, 2);
        var left = LeakDC.ar(sig[0]);
        var right = LeakDC.ar(sig[1]);
        var energy = (left.abs + right.abs) * 0.5;
        var mono = LeakDC.ar((left + right) * 0.5);
        var mid = (sig[0] + sig[1]) * 0.5;
        var side = (sig[0] - sig[1]) * 0.5;
        var midAmp = Amplitude.kr(mid, 0.08, 0.45);
        var sideAmp = Amplitude.kr(side, 0.08, 0.45);
        var rms = Amplitude.kr(energy, 0.12, 0.45);
        var fast = Amplitude.kr(energy, 0.004, 0.05);
        var slow = Amplitude.kr(energy, 0.2, 0.7);
        var peak = Amplitude.kr(left.abs.max(right.abs), 0.001, 0.08);
        var hf = Amplitude.kr((HPF.ar(left, 2500.0).abs + HPF.ar(right, 2500.0).abs) * 0.5, 0.08, 0.35);
        var low = Amplitude.kr((LPF.ar(left, 180.0).abs + LPF.ar(right, 180.0).abs) * 0.5, 0.08, 0.35);
        var widthMetric = Lag.kr((sideAmp / (midAmp + sideAmp + 0.0001)).clip(0.0, 1.0), 0.35);
        var chain = FFT(LocalBuf(2048), mono);
        var centroid = Lag.kr(SpecCentroid.kr(chain).clip(40.0, 10000.0) / 10000.0, 0.35);
        var zcr = A2K.kr(ZeroCrossing.ar(mono)).clip(0.0, 12000.0) / 12000.0;
        var popMetric = (fast - (slow * 3.5)).max(0.0);

        SendReply.kr(
            Impulse.kr(rate.clip(1.0, 40.0)),
            '/sw/ambientLabAnalysis',
            [rms, peak, fast, slow, popMetric, hf, low, widthMetric, zcr, centroid],
            replyID
        );
    }).add;

    s.sync;

    ~swChoirBuffers = [];
    ~swChoirHasSamples = false;
    ~swFindChoirSamplePaths = {
        var candidateDirs = [
            (~swChoirSampleDir ? "").asString,
            (~swChoirAssetSampleDir ? "").asString
        ].select { |path| path.notNil and: { path.asString.size > 0 } };
        var paths = [];

        candidateDirs.do { |directoryPath|
            var directory = PathName(directoryPath);

            if (paths.isEmpty and: { directory.isFolder }) {
                paths = directory.files
                    .select { |pathName|
                        var fileName = pathName.fileName.asString.toLower;

                        [".wav", ".aiff", ".aif", ".flac"].any { |suffix| fileName.endsWith(suffix) }
                    }
                    .sort { |a, b| a.fileName.asString < b.fileName.asString }
                    .collect(_.fullPath);

                if (paths.notEmpty) {
                    ("Using choir samples from " ++ directoryPath).postln;
                };
            };
        };

        if (paths.isEmpty) {
            "No choir sample files found in configured sample folders. Using synth fallback choir.".postln;
        };

        paths;
    };
    ~swLoadChoirSamples = {
        ~swChoirBuffers.do { |buffer|
            if (buffer.notNil) {
                buffer.free;
            };
        };

        ~swChoirBuffers = ~swFindChoirSamplePaths.().collect { |path|

            if (PathName(path).isFile) {
                ("Loading choir sample " ++ path).postln;
                Buffer.readChannel(s, path, channels: [0]);
            } {
                ("Missing choir sample " ++ path).warn;
                nil
            }
        };

        s.sync;
        ~swChoirHasSamples = ~swChoirBuffers.detect { |buffer| buffer.notNil }.notNil;
        if (~swChoirHasSamples.not) {
            "No portable choir samples loaded. Using synth fallback choir.".postln;
        };
    };
    ~swChoirBufNumAt = { |index=0|
        var buffers = ~swChoirBuffers ? [];
        var buffer = buffers[index];

        if (buffer.isNil) {
            buffer = buffers.detect { |item| item.notNil };
        };

        if (buffer.notNil) {
            buffer.bufnum;
        } {
            0;
        }
    };
    ~swLoadChoirSamples.();

    ~swStandPromptBuffer = nil;
    ~swStandPromptHasSample = false;
    ~swFindStandPromptSamplePath = {
        var candidateDirs = [
            (~swStandPromptSampleDir ? "").asString
        ].select { |path| path.notNil and: { path.asString.size > 0 } };
        var paths = [];

        candidateDirs.do { |directoryPath|
            var directory = PathName(directoryPath);

            if (paths.isEmpty and: { directory.isFolder }) {
                paths = directory.files
                    .select { |pathName|
                        var fileName = pathName.fileName.asString.toLower;

                        [".wav", ".aiff", ".aif", ".flac"].any { |suffix| fileName.endsWith(suffix) }
                    }
                    .sort { |a, b| a.fileName.asString < b.fileName.asString }
                    .collect(_.fullPath);

                if (paths.notEmpty) {
                    ("Using stand prompt sample from " ++ directoryPath).postln;
                };
            };
        };

        paths.first;
    };
    ~swLoadStandPromptSample = {
        var path;

        if (~swStandPromptBuffer.notNil) {
            ~swStandPromptBuffer.free;
            ~swStandPromptBuffer = nil;
        };

        path = ~swFindStandPromptSamplePath.();
        if (path.notNil and: { PathName(path).isFile }) {
            ("Loading stand prompt sample " ++ path).postln;
            ~swStandPromptBuffer = Buffer.readChannel(s, path, channels: [0]);
            s.sync;
            ~swStandPromptHasSample = ~swStandPromptBuffer.notNil;
        } {
            ~swStandPromptHasSample = false;
            "No stand prompt sample found. Stand failsafe voice prompt disabled.".postln;
        };
    };
    ~swStandPromptBufNum = {
        if (~swStandPromptBuffer.notNil) {
            ~swStandPromptBuffer.bufnum;
        } {
            0;
        };
    };
    ~swLoadStandPromptSample.();

    // ---- Song mode ----------------------------------------------------------
    // Plays a recording from samples/song, starting at songStartSeconds, whenever the
    // bridge's song gate opens, and fades it out when that gate closes.
    // Own compiler scope: the main block is at SuperCollider's per-function literal limit.
    {
    ~swSongBuffer = nil;
    ~swSongReady = false;
    ~swSongVoice = nil;
    ~swSongFadingVoices = List.new;
    ~swFindSongPath = { |fileName=nil|
        var directory = PathName((~swSongSampleDir ? "").asString);
        var files = [];
        var match;

        if (directory.isFolder) {
            files = directory.files
                .select { |pathName|
                    var name = pathName.fileName.asString.toLower;

                    [".wav", ".aiff", ".aif", ".flac"].any { |suffix| name.endsWith(suffix) }
                }
                .sort { |a, b| a.fileName.asString < b.fileName.asString };
        };
        if (fileName.notNil) {
            match = files.detect { |pathName| pathName.fileName.asString == fileName.asString };
        };
        if (match.isNil and: { files.notEmpty }) {
            match = files.first;
        };
        if (match.notNil) { match.fullPath } { nil };
    };
    ~swLoadSongBuffer = { |params|
        var path = ~swFindSongPath.(params[\songFile]);
        var startSeconds = (params[\songStartSeconds] ? 0.0).max(0.0);
        var soundFile;
        var startFrame = 0;
        var channels = 2;

        ~swSongReady = false;
        if (~swSongBuffer.notNil) {
            ~swSongBuffer.free;
            ~swSongBuffer = nil;
        };
        if (path.isNil) {
            ("No song file found in " ++ (~swSongSampleDir ? "(unset)").asString
                ++ "; song mode stays silent until one is added.").postln;
        } {
            soundFile = SoundFile.openRead(path);
            if (soundFile.notNil) {
                startFrame = (startSeconds * soundFile.sampleRate).round.asInteger
                    .clip(0, (soundFile.numFrames - 1).max(0));
                channels = soundFile.numChannels;
                soundFile.close;
            };
            ("Loading song " ++ path ++ " from " ++ startSeconds.round(0.01).asString ++ " s").postln;
            ~swSongBuffer = Buffer.readChannel(s, path, startFrame, -1,
                (channels >= 2).if({ [0, 1] }, { [0] }),
                { |buffer|
                    ~swSongReady = true;
                    ("Song ready: " ++ (buffer.numFrames / buffer.sampleRate.max(1)).round(0.1).asString
                        ++ " s, " ++ buffer.numChannels.asString ++ " channel(s)").postln;
                });
        };
    };
    ~swSongCutFading = {
        ~swSongFadingVoices.copy.do { |voice|
            if (voice.isPlaying) {
                voice.set(\cut, 1);
                AppClock.sched(0.12, {
                    if (voice.isPlaying) { voice.free; };
                    nil;
                });
            };
            ~swSongFadingVoices.remove(voice);
        };
    };
    ~swSongStart = {
        var params = ~swAuduinoParams ? ();
        var buffer = ~swSongBuffer;
        var voice;

        if (~swSongVoice.notNil) {
            ~swSongFadingVoices.add(~swSongVoice);
            ~swSongVoice = nil;
        };
        ~swSongCutFading.();
        if ((~swSongReady ? false) and: { buffer.notNil }) {
            voice = Synth((buffer.numChannels == 1).if(\spacewizardSongPlayer1, \spacewizardSongPlayer2), [
                \buf, buffer.bufnum,
                \amp, ~swSongAmpFor.(~swSongEnergy ? 1.0, params),
                \rate, ~swSongRateFor.(~swSongEnergy ? 1.0, params),
                \fadeIn, (params[\songFadeInSeconds] ? 0.01).clip(0.001, 5.0),
                \fadeOut, (params[\songFadeOutSeconds] ? 2.5).clip(0.01, 5.0)
            ]);
            ~swSongVoice = voice;
            voice.onFree {
                if (~swSongVoice === voice) { ~swSongVoice = nil; };
                ~swSongFadingVoices.remove(voice);
            };
            "Song started from its cue point.".postln;
        } {
            "Song mode: no song loaded, nothing to play.".postln;
        };
    };
    ~swSongRelease = {
        if (~swSongVoice.notNil) {
            ~swSongVoice.set(\gate, 0);
            ~swSongFadingVoices.add(~swSongVoice);
            ~swSongVoice = nil;
            "Song fading out.".postln;
        };
    };
    // Song energy (9th OSC value, 0..1) from the bridge: how much the candle is being
    // moved, averaged. It sets playback speed and pitch together, and the volume near zero.
    ~swSongEnergy = 1.0;
    ~swSongRateFor = { |energy=1.0, params|
        var p = params ? ();
        var fullSpeed = (p[\songFullSpeedEnergy] ? 0.8).clip(0.05, 1.0);
        var minRate = (p[\songMinRate] ? 0.2).clip(0.05, 1.0);

        minRate + ((1.0 - minRate) * (energy / fullSpeed).clip(0.0, 1.0));
    };
    ~swSongAmpFor = { |energy=1.0, params|
        var p = params ? ();
        var fadeBelow = (p[\songFadeBelowEnergy] ? 0.25).clip(0.01, 1.0);

        (p[\songAmp] ? 0.4).clip(0.0, 1.0) * (energy / fadeBelow).clip(0.0, 1.0);
    };
    ~swSongSetEnergy = { |energy=1.0|
        var params = ~swAuduinoParams ? ();
        var e = energy.clip(0.0, 1.0);
        var rate = ~swSongRateFor.(e, params);
        var amp = ~swSongAmpFor.(e, params);

        ~swSongEnergy = e;
        if (~swSongVoice.notNil) { ~swSongVoice.set(\rate, rate, \amp, amp); };
        // A voice that is fading out keeps slowing down until it is gone.
        ~swSongFadingVoices.do { |voice| voice.set(\rate, rate, \amp, amp); };
    };

    // The song follows its own, less sensitive gate from the bridge (8th OSC value):
    // it restarts from the cue when that gate opens and fades when it closes.
    ~swSongGateOpen = false;
    ~swSongSetGate = { |isOpen=false|
        var open = isOpen.asBoolean;
        var wasOpen = ~swSongGateOpen ? false;

        ~swSongGateOpen = open;
        if (open and: { wasOpen.not }) { ~swSongStart.(); };
        if (open.not and: { wasOpen }) { ~swSongRelease.(); };
    };
    ~swSongStop = {
        if (~swSongVoice.notNil) {
            ~swSongFadingVoices.add(~swSongVoice);
            ~swSongVoice = nil;
        };
        ~swSongCutFading.();
    };
    }.value;

    ~swMotionFadeInSeconds = 0.18;
    ~swMotionFadeOutSeconds = 5.0;
    ~swMotionGateOpen = false;
    ~swMotionCleanupRoutine = nil;
    ~swMotionLevelBus = Bus.control(s, 1);
    ~swMotionLevelBus.set(0.0);
    ~swSmoothLayerFadeDelaySeconds = 1.0;
    ~swSmoothLayerFadeInSeconds = 0.65;
    ~swSmoothLayerGateOpen = false;
    ~swSmoothLayerGateToken = 0;
    ~swSmoothLayerGateRoutine = nil;
    ~swSmoothLayerLevelBus = Bus.control(s, 1);
    ~swSmoothLayerLevelBus.set(0.0);
    ~swStandPrompt = nil;
    ~swStandPromptRoutine = nil;
    ~swStandPromptToken = 0;
    ~swStandPromptArmed = false;
    ~swStandPromptPrompted = false;
    ~swStandPromptLastPitchNorm = 0.0;

    ~swVoice = nil;
    ~swSawLead = nil;
    ~swAuduino = nil;
    ~swSmoothVoices = List.new;
    ~swSmoothMonoVoice = nil;
    ~swSmoothChoir = nil;
    ~swSmoothChoirTriggerCount = 0;
    ~swSmoothChoirGateWasOpen = false;
    ~swChoirSamplePitchHz = 176.47;
        ~swSmoothTailBudgetVoices = 12;
        ~swSmoothDensityReferenceVoices = 6;
    ~swSmoothLastMidi = nil;
    ~swSmoothLastIndex = nil;
    ~swSmoothFilteredPitchNorm = nil;
    ~swSmoothLastTriggerTime = 0.0;
    ~swAmbientBassVariant = \drone;
    ~swAmbientBassLastTriggerTime = 0.0;
    ~swAmbientBassStep = 0;
    ~swReeseBass = nil;
    ~swWowVoice = nil;
    ~swAmbientVoices = IdentityDictionary.new;
    ~swAmbientLabVoice = nil;
    ~swAmbientLabVoiceEngine = nil;
    ~swAmbientLabAppliedEngine = nil;
    ~swAmbientLabOverlay = nil;
    ~swAmbientLabOverlayActive = false;
    ~swAmbientLabPentatonic = nil;
    ~swAmbientLabPentatonicVoices = List.new;
    ~swAmbientLabPentatonicLastIndex = nil;
    ~swAmbientLabPentatonicFilteredPitchNorm = nil;
    ~swAmbientLabPentatonicLastTriggerTime = 0.0;
    ~swAuduinoRoutine = nil;
    ~swAuduinoWindow = nil;
    ~swAmbientWindow = nil;
    ~swAmbientLabWindow = nil;
    ~swChoirTextureWindow = nil;
    ~swChoirBlendPreview = nil;
    ~swChoirRawPreview = nil;
    ~swSmoothMixWindow = nil;
    ~swLiveStatusWindow = nil;
    ~swLiveStatusText = nil;
    ~swLiveStatusRefreshRoutine = nil;
    ~swLiveStatusRefreshPeriod = 0.02;
    ~swLiveStatusNeedsRefresh = true;
    ~swLiveStatusLastSummary = nil;
    ~swWowTriggerCount = 0;
    ~swWowPrevPitchNorm = nil;
    ~swWowPrevDirection = 0;
    ~swWowPrevVelocity = 0.0;
    ~swWowTriggerCooldown = 0;
    ~swLiveInputState = (
        gate: 0.0,
        activity: 0.0,
        pitchNorm: 0.0,
        pitchDeg: 0.0,
        rollNorm: 0.0,
        rollDeg: 0.0,
        spinNorm: 0.0,
        yawNorm: 0.0,
        yawDeg: 0.0
    );
    ~swAuduinoParams = (
        syncFreq: 42,
        grain1Freq: 196,
        grain2Freq: 294,
        decay1: 0.14,
        decay2: 0.09,
        amp: 0.12,
        pan: 0.0,
        drive: 2.6,
        crushBits: 7,
        hp: 60,
        lp: 4600,
        scaleMode: \originalAuduino,
        playMode: \continuous,
        voiceMode: \auduino,
        rootMidi: 50,
        stepIndex: 10,
        smoothFreq: 72,
        smoothMidi: 43,
        smoothRootMidi: 38,
            smoothExperiment: \triggeredNotes,
        smoothPitchNorm: 0.0,
        smoothPitchSmoothing: 0.34,
        smoothIndexHysteresis: 0.22,
        smoothTriggerUseRawPitch: false,
        smoothGlide: 0.0,
        smoothVibratoRate: 0.18,
        smoothVibratoDepth: 0.006,
        smoothMotionDepth: 0.06,
        smoothBloom: 0.18,
        smoothLp: 2200,
        smoothCrunch: 0.0,
        smoothReverb: 0.14,
        smoothRollToneMaxRoll: 0.65,
        smoothRollCrunchMin: 0.0,
        smoothRollCrunchMax: 0.58,
        smoothRollReverbMin: 0.12,
        smoothRollReverbMax: 0.72,
        smoothBassClarityAmount: 0.0,
        smoothBassClarityPitchStart: 0.0,
        smoothBassClarityPitchFull: -0.9,
        smoothBassClarityHpMax: 62.0,
        smoothBassClarityMudFreq: 180.0,
        smoothBassClarityMudCutDb: 4.0,
        smoothLowPitchAmpBoost: 0.0,
        smoothLowPitchAmpStart: 0.0,
        smoothLowPitchAmpFull: -0.85,
        smoothLayerFadeDelaySeconds: 1.0,
        smoothLayerFadeInSeconds: 0.65,
        smoothHold: 0.06,
        smoothDecay: 2.6,
        smoothDecayMin: 1.7,
        smoothDecayMax: 3.2,
        smoothDecayYawMin: -0.1,
        smoothDecayYawMax: 1.0,
        smoothMinInterval: 0.07,
        smoothSnapBypassInterval: true,
        smoothFastJumpThreshold: 0.55,
        smoothFastMinInterval: 0.10,
        smoothFastJumpAmpScale: 0.85,
        smoothFastJumpDecayScale: 0.85,
        smoothTailBudgetVoices: 12,
        smoothDensityReferenceVoices: 6,
        smoothPreset: \standard,
        smoothAmpScale: 1.0,
        smoothAmpFollowsActivity: true,
        smoothFixedAmp: 0.10,
        smoothChoirEnabled: true,
        smoothChoirSolo: false,
        smoothChoirAmp: 0.60,
        smoothChoirSynthAmp: 0.46,
        smoothChoirSpread: 0.78,
        smoothChoirShimmer: 0.12,
        smoothChoirBreath: 0.07,
        smoothChoirBrightness: 0.28,
        smoothChoirFxMix: 0.32,
        smoothChoirDelayMix: 0.14,
        smoothChoirAir: 0.36,
        smoothChoirBassAmp: 0.0,
        smoothChoirBassTone: 0.42,
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
        standPromptEnabled: true,
        standPromptPitchMin: 0.86,
        standPromptDelaySeconds: 2.5,
        standPromptAmp: 1.0,
        standPromptStopActivity: 0.12,
        standPromptRequirePriorMotion: true,
        smoothAmbientEnabled: true,
        smoothAmbientYawBlendEnabled: true,
        smoothAmbientAmpScale: 0.58,
        smoothAmbientAmpCeiling: 0.24,
        smoothAmbientAmpFollowsActivity: true,
        smoothAmbientFixedAmp: 0.10,
        smoothAmbientPanScale: 0.6,
        smoothAmbientWidthScale: 0.92,
        smoothAmbientVariant: \flow,
        smoothAmbientRoot: 74,
        smoothAmbientYawOffset: 0.0,
        smoothAmbientLabEnabled: false,
        smoothImpactDrumEnabled: true,
        smoothImpactDrumAmp: 0.16,
        smoothImpactDrumFreq: 74,
        smoothImpactDrumDecay: 1.6,
        smoothImpactDrumTone: 0.42,
        smoothImpactDrumVerb: 0.72,
        smoothImpactDrumMaxJerk: 60.0,
        reeseFreq: 50,
        reeseDetune: 0.028,
        reeseGlide: 0.32,
        reeseWobbleRate: 0.18,
        reeseWobbleDepth: 0.12,
        reeseDrive: 2.8,
        reeseLp: 900,
        reeseRq: 0.24,
        reeseSubMix: 0.46,
        wowFreq: 110,
        wowGlide: 0.28,
        wowDirection: 1,
        wowVowelDepth: 0.82,
        wowBreath: 0.035,
        wowLp: 2400,
        wowTone: 0.16,
        wowTrig: 0,
        ambientVariant: \flow,
        ambientYawBlendEnabled: false,
        ambientYawOffset: 0.0,
        ambientBlendPosition: 0.0,
        ambientBlendPrimaryVariant: \flow,
        ambientBlendSecondaryVariant: \chordal,
        ambientBlendMix: 0.0,
        ambientRoot: 74,
        ambientLift: -0.1,
        ambientLean: -0.15,
        ambientMotion: 0.22,
        ambientSpin: 0.14,
        ambientAura: 0.62,
        ambientDarkness: 0.66,
        ambientShimmer: 0.34,
        ambientWidth: 0.78
    );
    ~swAuduinoDefaultParams = ~swAuduinoParams.copy;

    ~swSoundPresetConfig = ();
    ~swLoadSoundPresetConfig = {
        var configPath = (~swSoundPresetConfigFile ? "").asString;
        var loaded;

        if (configPath.size > 0 and: { PathName(configPath).isFile }) {
            loaded = thisProcess.interpreter.executeFile(configPath);
            if (loaded.notNil) {
                ~swSoundPresetConfig = loaded;
            };
            if (~swSoundPresetConfig.isNil) {
                ~swSoundPresetConfig = ();
            };
            ("Loaded sound preset config: " ++ configPath).postln;
        } {
            ~swSoundPresetConfig = ();
            if (configPath.size > 0) {
                ("No sound preset config found at " ++ configPath ++ "; using built-in defaults.").postln;
            };
        };
    };

    ~swSoundPresetOverridesFor = { |section, preset=\standard|
        var config = ~swSoundPresetConfig ? ();
        var sectionConfig = config[section] ? ();
        var overrides = sectionConfig[preset] ? ();

        overrides;
    };

    ~swLoadSoundPresetConfig.();

    ~swAmbientVariantOptions = [
        (key: \flow, label: "Expansive flow"),
        (key: \chordal, label: "Harmonic chords"),
        (key: \drone, label: "Drone texture"),
        (key: \forest, label: "Forest canopy")
    ];
    ~swAmbientVariantKeys = ~swAmbientVariantOptions.collect { |item| item[\key] };

    ~swAmbientLabEngineOptions = [
        (key: \halo, label: "Halo Drone"),
        (key: \airports, label: "Airports Chords"),
        (key: \granular, label: "Granular Choir Cloud"),
        (key: \hexBells, label: "Hex Bells"),
        (key: \tapeSwells, label: "Tape Swells"),
        (key: \tapeScript, label: "Tape Script"),
        (key: \hybrid, label: "Hybrid Flow")
    ];
    ~swAmbientLabEngineKeys = ~swAmbientLabEngineOptions.collect { |item| item[\key] };

    ~swAmbientLabTargetOptions = [
        (key: \none, label: "None"),
        (key: \sceneMorph, label: "Scene Morph"),
        (key: \harmonyLift, label: "Harmony Lift"),
        (key: \brightness, label: "Brightness"),
        (key: \darkness, label: "Darkness"),
        (key: \density, label: "Density"),
        (key: \shimmer, label: "Shimmer"),
        (key: \width, label: "Width"),
        (key: \grainScan, label: "Grain Scan"),
        (key: \rhythmDensity, label: "Rhythm Density"),
        (key: \overlayMix, label: "Overlay Mix")
    ];

    ~swAmbientLabCurveOptions = [
        (key: \linear, label: "Linear"),
        (key: \soft, label: "Soft"),
        (key: \strong, label: "Strong")
    ];

    ~swAmbientLabAxisOptions = [
        (key: \gate, label: "Gate", min: 0.0, max: 1.0),
        (key: \yaw, label: "Yaw", min: 0.0, max: 1.0),
        (key: \pitch, label: "Pitch", min: -1.0, max: 1.0),
        (key: \roll, label: "Roll", min: -1.0, max: 1.0),
        (key: \activity, label: "Activity", min: 0.0, max: 1.0),
        (key: \spin, label: "Spin/Accel", min: 0.0, max: 1.0)
    ];

    ~swAmbientLabPresetOptions = [
        (key: \beautifulDefault, label: "Beautiful Default"),
        (key: \yawSceneMorph, label: "Yaw Scene Morph"),
        (key: \pitchOpensSky, label: "Pitch Opens Sky"),
        (key: \rollSpace, label: "Roll Space"),
        (key: \accelRhythm, label: "Accel Rhythm")
    ];

    ~swAmbientLabIndexFor = { |options, key|
        options.detectIndex { |item| item[\key] == key } ? 0;
    };

    ~swAmbientLabMapping = { |target=\none, amount=0.0, curve=\linear, invert=false|
        (target: target, amount: amount, curve: curve, invert: invert);
    };

    ~swAmbientLabDefaultMappings = {
        (
            gate: ~swAmbientLabMapping.(\none, 0.0, \linear, false),
            yaw: ~swAmbientLabMapping.(\sceneMorph, 0.95, \linear, false),
            pitch: ~swAmbientLabMapping.(\brightness, 0.78, \linear, false),
            roll: ~swAmbientLabMapping.(\width, 0.85, \linear, false),
            activity: ~swAmbientLabMapping.(\density, 0.5, \soft, false),
            spin: ~swAmbientLabMapping.(\shimmer, 0.44, \soft, false)
        )
    };

    ~swAmbientLabDefaultState = {
        (
            engine: \halo,
            preset: \beautifulDefault,
            isPlaying: false,
            gate: 1.0,
            yaw: 0.34,
            pitch: 0.0,
            roll: 0.0,
            activity: 0.24,
            spin: 0.12,
            root: 58.0,
            level: 0.12,
            ambience: 0.74,
            rhythm: 0.14,
            overlay: 0.04,
            pentatonicEnabled: false,
            pentatonicLevel: 0.1,
            pentatonicRootMidi: 38,
            yawOverlayEnabled: true,
            yawOverlayStart: 0.68,
            yawOverlayEnd: 0.86,
            yawOverlayHysteresis: 0.04,
            hybridVariant: \flow,
            mappings: ~swAmbientLabDefaultMappings.()
        )
    };

    ~swAmbientLabState = ~swAmbientLabDefaultState.();

    ~swAmbientLabPresetFor = { |preset=\beautifulDefault|
        var state = ~swAmbientLabDefaultState.();
        var mappings = ~swAmbientLabDefaultMappings.();

        if (preset == \yawSceneMorph) {
            mappings[\yaw] = ~swAmbientLabMapping.(\sceneMorph, 0.82, \linear, false);
            mappings[\pitch] = ~swAmbientLabMapping.(\brightness, 0.22, \soft, false);
            mappings[\roll] = ~swAmbientLabMapping.(\width, 0.28, \soft, false);
            mappings[\activity] = ~swAmbientLabMapping.(\density, 0.28, \soft, false);
            mappings[\spin] = ~swAmbientLabMapping.(\shimmer, 0.28, \soft, false);
            state = state.putAll((
                engine: \hybrid,
                preset: \yawSceneMorph,
                root: 56.0,
                level: 0.12,
                ambience: 0.72,
                rhythm: 0.16,
                overlay: 0.035,
                yaw: 0.12,
                mappings: mappings
            ));
        } {
            if (preset == \pitchOpensSky) {
                mappings[\yaw] = ~swAmbientLabMapping.(\sceneMorph, 0.28, \linear, false);
                mappings[\pitch] = ~swAmbientLabMapping.(\brightness, 0.48, \soft, false);
                mappings[\roll] = ~swAmbientLabMapping.(\width, 0.22, \soft, false);
                mappings[\activity] = ~swAmbientLabMapping.(\density, 0.26, \soft, false);
                mappings[\spin] = ~swAmbientLabMapping.(\shimmer, 0.24, \soft, false);
                state = state.putAll((
                    engine: \halo,
                    preset: \pitchOpensSky,
                    root: 60.0,
                    level: 0.125,
                    ambience: 0.78,
                    rhythm: 0.12,
                    overlay: 0.035,
                    pitch: -0.18,
                    mappings: mappings
                ));
            } {
                if (preset == \rollSpace) {
                    mappings[\yaw] = ~swAmbientLabMapping.(\sceneMorph, 0.38, \linear, false);
                    mappings[\pitch] = ~swAmbientLabMapping.(\darkness, 0.22, \soft, true);
                    mappings[\roll] = ~swAmbientLabMapping.(\width, 0.5, \soft, false);
                    mappings[\activity] = ~swAmbientLabMapping.(\density, 0.26, \soft, false);
                    mappings[\spin] = ~swAmbientLabMapping.(\grainScan, 0.28, \linear, false);
                    state = state.putAll((
                        engine: \granular,
                        preset: \rollSpace,
                        root: 55.0,
                        level: 0.118,
                        ambience: 0.82,
                        rhythm: 0.14,
                        overlay: 0.025,
                        mappings: mappings
                    ));
                } {
                    if (preset == \accelRhythm) {
                        mappings[\yaw] = ~swAmbientLabMapping.(\sceneMorph, 0.42, \linear, false);
                        mappings[\pitch] = ~swAmbientLabMapping.(\brightness, 0.24, \soft, false);
                        mappings[\roll] = ~swAmbientLabMapping.(\width, 0.22, \soft, false);
                        mappings[\activity] = ~swAmbientLabMapping.(\density, 0.36, \soft, false);
                        mappings[\spin] = ~swAmbientLabMapping.(\rhythmDensity, 0.46, \linear, false);
                        state = state.putAll((
                            engine: \airports,
                            preset: \accelRhythm,
                            root: 58.0,
                            level: 0.12,
                            ambience: 0.78,
                            rhythm: 0.2,
                            overlay: 0.05,
                            spin: 0.12,
                            mappings: mappings
                        ));
                    } {
                        state = state.putAll((preset: \beautifulDefault, mappings: mappings));
                    }
                }
            }
        };

        state;
    };

    ~swLiveVoiceOptions = [
        (key: \auduino, label: "Auduino grit"),
        (key: \ambientFlow, label: "Ambient flow"),
        (key: \smoothSine, label: "Smooth bass notes"),
        (key: \reeseBass, label: "Reese bass"),
        (key: \wowVoice, label: "Wow voice")
    ];

    ~swSmoothExperimentOptions = [
        (key: \triggeredNotes, label: "Triggered notes"),
        (key: \trackedMono, label: "Tracked mono"),
        (key: \directMono, label: "Direct mono")
    ];

    ~swSmoothPresetFor = { |preset=\standard|
        var selected = preset ? \standard;
        var values;

        if (selected == \piQuietPentatonic) {
            values = (
                smoothPreset: \piQuietPentatonic,
                smoothExperiment: \trackedMono,
                smoothPitchSmoothing: 0.68,
                smoothIndexHysteresis: 0.12,
                smoothTriggerUseRawPitch: true,
                smoothGlide: 0.035,
                smoothHold: 0.04,
                smoothDecay: 1.15,
                smoothDecayMin: 0.75,
                smoothDecayMax: 1.20,
                smoothDecayYawMin: -0.03,
                smoothDecayYawMax: 0.12,
                smoothMinInterval: 0.09,
                smoothSnapBypassInterval: false,
                smoothFastJumpThreshold: 0.42,
                smoothFastMinInterval: 0.16,
                smoothFastJumpAmpScale: 1.0,
                smoothFastJumpDecayScale: 0.68,
                smoothTailBudgetVoices: 5,
                smoothDensityReferenceVoices: 3,
                smoothAmpScale: 1.03,
                smoothAmpFollowsActivity: false,
                smoothFixedAmp: 0.11,
                smoothCrunch: 0.0,
                smoothReverb: 0.14,
                smoothRollToneMaxRoll: 0.65,
                smoothRollCrunchMin: 0.0,
                smoothRollCrunchMax: 0.34,
                smoothRollReverbMin: 0.12,
                smoothRollReverbMax: 0.72,
                smoothBassClarityAmount: 0.42,
                smoothBassClarityPitchStart: 0.05,
                smoothBassClarityPitchFull: -0.86,
                smoothBassClarityHpMax: 48.0,
                smoothBassClarityMudFreq: 180.0,
                smoothBassClarityMudCutDb: 2.2,
                smoothLowPitchAmpBoost: 0.80,
                smoothLowPitchAmpStart: 0.0,
                smoothLowPitchAmpFull: -0.85,
                smoothLayerFadeDelaySeconds: 0.05,
                smoothLayerFadeInSeconds: 0.18,
                smoothChoirEnabled: true,
                smoothChoirAmp: 0.37,
                smoothChoirSynthAmp: 0.0,
                smoothChoirSampleStart: 0.0,
                smoothChoirTexture: 1,
                smoothChoirSpread: 0.93,
                smoothChoirShimmer: 0.95,
                smoothChoirBreath: 0.10,
                smoothChoirBrightness: 0.96,
                smoothChoirFxMix: 1.0,
                smoothChoirDelayMix: 0.99,
                smoothChoirAir: 0.94,
                smoothChoirBassAmp: 0.31,
                smoothChoirBassTone: 0.57,
                smoothChoirFadeIn: 0.5,
                smoothChoirHold: 0.28,
                smoothChoirFadeOut: 4.2,
                smoothChoirOctaveDown3: 0,
                smoothChoirOctaveDown2: 1,
                smoothChoirOctaveDown1: 1,
                smoothChoirOctaveUp1: 0,
                smoothChoirOctaveUp2: 0,
                smoothChoirOctaveUp3: 0,
                standPromptEnabled: true,
                standPromptPitchMin: 0.75,
                standPromptDelaySeconds: 2.5,
                standPromptAmp: 1.0,
                standPromptStopActivity: 0.12,
                standPromptRequirePriorMotion: true,
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
                smoothImpactDrumEnabled: true,
                smoothImpactDrumAmp: 0.16,
                smoothImpactDrumFreq: 74,
                smoothImpactDrumDecay: 1.6,
                smoothImpactDrumTone: 0.42,
                smoothImpactDrumVerb: 0.72,
                smoothImpactDrumMaxJerk: 60.0
            );
        } {
            if (selected == \quietPentatonic) {
                values = (
                    smoothPreset: \quietPentatonic,
                    smoothExperiment: \triggeredNotes,
                    smoothPitchSmoothing: 0.78,
                    smoothIndexHysteresis: 0.08,
                    smoothTriggerUseRawPitch: true,
                    smoothGlide: 0.0,
                    smoothHold: 0.06,
                    smoothDecay: 2.6,
                    smoothDecayMin: 1.7,
                    smoothDecayMax: 3.2,
                    smoothDecayYawMin: -0.1,
                    smoothDecayYawMax: 1.0,
                    smoothMinInterval: 0.025,
                    smoothSnapBypassInterval: true,
                    smoothFastJumpThreshold: 0.55,
                    smoothFastMinInterval: 0.10,
                    smoothFastJumpAmpScale: 0.85,
                    smoothFastJumpDecayScale: 0.85,
                    smoothTailBudgetVoices: 12,
                    smoothDensityReferenceVoices: 6,
                    smoothAmpScale: 0.439,
                    smoothCrunch: 0.0,
                    smoothReverb: 0.14,
                    smoothRollToneMaxRoll: 0.65,
                    smoothRollCrunchMin: 0.0,
                    smoothRollCrunchMax: 0.58,
                    smoothRollReverbMin: 0.12,
                    smoothRollReverbMax: 0.72,
                    smoothBassClarityAmount: 0.68,
                    smoothBassClarityPitchStart: 0.05,
                    smoothBassClarityPitchFull: -0.86,
                    smoothBassClarityHpMax: 62.0,
                    smoothBassClarityMudFreq: 180.0,
                    smoothBassClarityMudCutDb: 4.0,
                    smoothLowPitchAmpBoost: 0.0,
                    smoothLowPitchAmpStart: 0.0,
                    smoothLowPitchAmpFull: -0.85,
                    smoothLayerFadeDelaySeconds: 1.0,
                    smoothLayerFadeInSeconds: 0.65,
                    smoothChoirEnabled: true,
                    smoothChoirAmp: 0.37,
                    smoothChoirSynthAmp: 0.0,
                    smoothChoirSampleStart: 0.0,
                    smoothChoirTexture: 1,
                    smoothChoirSpread: 0.93,
                    smoothChoirShimmer: 0.95,
                    smoothChoirBreath: 0.10,
                    smoothChoirBrightness: 0.96,
                    smoothChoirFxMix: 1.0,
                    smoothChoirDelayMix: 0.99,
                    smoothChoirAir: 0.94,
                    smoothChoirBassAmp: 0.31,
                    smoothChoirBassTone: 0.57,
                    smoothChoirFadeIn: 0.5,
                    smoothChoirHold: 0.28,
                    smoothChoirFadeOut: 4.2,
                    smoothChoirOctaveDown3: 0,
                    smoothChoirOctaveDown2: 1,
                    smoothChoirOctaveDown1: 1,
                    smoothChoirOctaveUp1: 0,
                    smoothChoirOctaveUp2: 0,
                    smoothChoirOctaveUp3: 0,
                    standPromptEnabled: false,
                    standPromptPitchMin: 0.86,
                    standPromptDelaySeconds: 2.5,
                    standPromptAmp: 1.0,
                    standPromptStopActivity: 0.12,
                    standPromptRequirePriorMotion: true,
                    smoothAmbientEnabled: true,
                    smoothAmbientYawBlendEnabled: true,
                    smoothAmbientAmpScale: 3.0,
                    smoothAmbientAmpCeiling: 0.24,
                    smoothAmbientPanScale: 0.6,
                    smoothAmbientWidthScale: 0.92,
                    smoothAmbientVariant: \flow,
                    smoothAmbientRoot: 74,
                    smoothAmbientYawOffset: 0.0,
                    smoothAmbientLabEnabled: false,
                    smoothImpactDrumEnabled: true,
                    smoothImpactDrumAmp: 0.16,
                    smoothImpactDrumFreq: 74,
                    smoothImpactDrumDecay: 1.6,
                    smoothImpactDrumTone: 0.42,
                    smoothImpactDrumVerb: 0.72,
                    smoothImpactDrumMaxJerk: 60.0
                );
            } {
                values = (
                    smoothPreset: \standard,
                    smoothExperiment: \triggeredNotes,
                    smoothTriggerUseRawPitch: false,
                    smoothGlide: 0.0,
                    smoothDecayMin: 1.7,
                    smoothDecayMax: 3.2,
                    smoothDecayYawMin: -0.1,
                    smoothDecayYawMax: 1.0,
                    smoothSnapBypassInterval: true,
                    smoothFastJumpThreshold: 0.55,
                    smoothFastMinInterval: 0.10,
                    smoothFastJumpAmpScale: 0.85,
                    smoothFastJumpDecayScale: 0.85,
                    smoothTailBudgetVoices: 12,
                    smoothDensityReferenceVoices: 6,
                    smoothAmpScale: 1.0,
                    smoothCrunch: 0.0,
                    smoothReverb: 0.14,
                    smoothRollToneMaxRoll: 0.65,
                    smoothRollCrunchMin: 0.0,
                    smoothRollCrunchMax: 0.18,
                    smoothRollReverbMin: 0.14,
                    smoothRollReverbMax: 0.34,
                    smoothChoirFxMix: 0.32,
                    smoothChoirDelayMix: 0.14,
                    smoothChoirAir: 0.36,
                    smoothChoirBassAmp: 0.08,
                    smoothChoirBassTone: 0.42,
                    smoothBassClarityAmount: 0.24,
                    smoothBassClarityPitchStart: -0.2,
                    smoothBassClarityPitchFull: -0.9,
                    smoothBassClarityHpMax: 54.0,
                    smoothBassClarityMudFreq: 180.0,
                    smoothBassClarityMudCutDb: 2.5,
                    smoothLowPitchAmpBoost: 0.0,
                    smoothLowPitchAmpStart: 0.0,
                    smoothLowPitchAmpFull: -0.85,
                    smoothLayerFadeDelaySeconds: 0.0,
                    smoothLayerFadeInSeconds: 0.18,
                    smoothAmbientEnabled: true,
                    smoothAmbientYawBlendEnabled: true,
                    smoothAmbientAmpScale: 0.58,
                    smoothAmbientAmpCeiling: 0.24,
                    smoothAmbientPanScale: 0.6,
                    smoothAmbientWidthScale: 0.92,
                    smoothAmbientVariant: \flow,
                    smoothAmbientRoot: 74,
                    smoothAmbientYawOffset: 0.0,
                    smoothAmbientLabEnabled: false,
                    smoothImpactDrumEnabled: true,
                    smoothImpactDrumAmp: 0.16,
                    smoothImpactDrumFreq: 74,
                    smoothImpactDrumDecay: 1.6,
                    smoothImpactDrumTone: 0.42,
                    smoothImpactDrumVerb: 0.72,
                    smoothImpactDrumMaxJerk: 60.0
                );
            };
        };

        values.putAll(~swSoundPresetOverridesFor.(\smoothSine, selected));
    };

    ~swApplySmoothPresetGlobals = { |params|
        if (params[\smoothLayerFadeDelaySeconds].notNil) {
            ~swSmoothLayerFadeDelaySeconds = params[\smoothLayerFadeDelaySeconds].clip(0.0, 10.0);
        };
        if (params[\smoothLayerFadeInSeconds].notNil) {
            ~swSmoothLayerFadeInSeconds = params[\smoothLayerFadeInSeconds].clip(0.01, 12.0);
        };
        if (params[\smoothTailBudgetVoices].notNil) {
            ~swSmoothTailBudgetVoices = params[\smoothTailBudgetVoices].clip(1, 24).asInteger;
        };
        if (params[\smoothDensityReferenceVoices].notNil) {
            ~swSmoothDensityReferenceVoices = params[\smoothDensityReferenceVoices].clip(1, 24).asInteger;
        };
    };

    ~swSmoothRollToneFor = { |params, rollShape=0.0|
        var maxRoll = (params[\smoothRollToneMaxRoll] ? 0.65).clip(0.01, 1.0);
        var crunchMin = (params[\smoothRollCrunchMin] ? 0.0).clip(0.0, 0.65);
        var crunchMax = (params[\smoothRollCrunchMax] ? 0.58).clip(0.0, 0.65);
        var reverbMin = (params[\smoothRollReverbMin] ? 0.12).clip(0.0, 0.8);
        var reverbMax = (params[\smoothRollReverbMax] ? 0.72).clip(0.0, 0.8);
        var shapedRoll = rollShape.clip(0.0, 1.0);

        (
            crunch: shapedRoll.linlin(0.0, maxRoll, crunchMin, crunchMax).clip(crunchMin.min(crunchMax), crunchMin.max(crunchMax)),
            reverb: shapedRoll.linlin(0.0, maxRoll, reverbMin, reverbMax).clip(reverbMin.min(reverbMax), reverbMin.max(reverbMax))
        );
    };

    ~swApplySmoothPreset = { |preset=\standard|
        var params = (~swAuduinoParams ? ~swAuduinoDefaultParams.copy).copy;
        var selectedPreset = preset ? \standard;

        params = params.putAll(~swSmoothPresetFor.(selectedPreset));
        ~swApplySmoothPresetGlobals.(params);
        ~swAuduinoParams = params;

        if ((params[\voiceMode] ? \auduino) == \smoothSine) {
            ~swSmoothApplyMonoAmpScale.();
            ~swSmoothAmbientApplyCurrent.();
        };

        ("Smooth preset: " ++ (params[\smoothPreset] ? selectedPreset).asString
            ++ " smoothAmpScale=" ++ (params[\smoothAmpScale] ? 1.0).round(0.001).asString
            ++ " smoothAmbientAmpScale=" ++ (params[\smoothAmbientAmpScale] ? 0.58).round(0.001).asString).postln;
        ~swRefreshLiveStatusGUI.();
        params;
    };

    ~swResetWowTracking = {
        ~swWowTriggerCount = 0;
        ~swWowPrevPitchNorm = nil;
        ~swWowPrevDirection = 0;
        ~swWowPrevVelocity = 0.0;
        ~swWowTriggerCooldown = 0;
    };

    ~swResetSmoothTracking = {
        ~swSmoothLastMidi = nil;
        ~swSmoothLastIndex = nil;
        ~swSmoothFilteredPitchNorm = nil;
        ~swSmoothLastTriggerTime = 0.0;
    };

    ~swResetSmoothChoirTracking = {
        ~swSmoothChoirTriggerCount = 0;
        ~swSmoothChoirGateWasOpen = false;
    };

    ~swResetAmbientBassTracking = {
        ~swAmbientBassLastTriggerTime = 0.0;
        ~swAmbientBassStep = 0;
    };

    ~swFormatStatusValue = { |value|
        if (value.isNil) {
            "nil"
        } {
            if (value.isKindOf(Float)) {
                value.round(0.0001).asString
            } {
                value.asString
            }
        };
    };

    ~swMidiNoteLabel = { |midi|
        var noteNames = ["C", "C#", "D", "Eb", "E", "F", "F#", "G", "Ab", "A", "Bb", "B"];
        var roundedMidi;
        var noteIndex;
        var octave;

        if (midi.isNil) {
            "nil"
        } {
            roundedMidi = midi.round.asInteger;
            noteIndex = roundedMidi % 12;
            octave = ((roundedMidi / 12).floor - 1).asInteger;
            noteNames.wrapAt(noteIndex) ++ octave.asString ++ " (" ++ roundedMidi.asString ++ ")"
        }
    };

    ~swLiveVoiceIndexFor = { |voiceMode=\auduino|
        ~swLiveVoiceOptions.detectIndex { |item|
            item[\key] == (voiceMode ? \auduino)
        } ? 0;
    };

    ~swAuduinoScaleIndexFor = { |scaleMode=\originalAuduino|
        ~swAuduinoScaleOptions.detectIndex { |item|
            item[\key] == (scaleMode ? \originalAuduino)
        } ? 0;
    };

    ~swSmoothExperimentIndexFor = { |experiment=\triggeredNotes|
        ~swSmoothExperimentOptions.detectIndex { |item|
            item[\key] == (experiment ? \triggeredNotes)
        } ? 0;
    };

    ~swLiveStatusParamKeysForMode = { |voiceMode=\auduino|
        if (voiceMode == \smoothSine) {
            [
                \smoothExperiment,
                \smoothMidi,
                \smoothFreq,
                \smoothPitchNorm,
                \smoothAmpScale,
                \smoothAmbientAmpScale,
                \smoothAmbientAmpCeiling,
                \smoothVibratoRate,
                \smoothVibratoDepth,
                \smoothMotionDepth,
                \smoothBloom,
                \smoothLp,
                \smoothCrunch,
                \smoothReverb,
                \smoothHold,
                \smoothDecay,
                \smoothChoirAmp,
                \smoothChoirSynthAmp,
                \smoothChoirSpread,
                \smoothChoirShimmer,
                \smoothChoirBreath,
                \smoothChoirBrightness,
                \smoothChoirFxMix,
                \smoothChoirDelayMix,
                \smoothChoirAir,
                \smoothChoirBassAmp,
                \smoothChoirBassTone,
                \smoothChoirSampleStart,
                \smoothChoirTexture,
                \smoothChoirFadeIn,
                \smoothChoirHold,
                \smoothChoirFadeOut,
                \amp,
                \pan
            ]
        } {
            if (voiceMode == \ambientFlow) {
                [
                    \ambientVariant,
                    \ambientBlendPosition,
                    \ambientBlendPrimaryVariant,
                    \ambientBlendSecondaryVariant,
                    \ambientBlendMix,
                    \ambientRoot,
                    \ambientLift,
                    \ambientLean,
                    \ambientMotion,
                    \ambientSpin,
                    \ambientAura,
                    \ambientDarkness,
                    \ambientShimmer,
                    \ambientWidth,
                    \amp,
                    \pan
                ]
            } {
                if (voiceMode == \reeseBass) {
                    [
                        \reeseFreq,
                        \reeseDetune,
                        \reeseGlide,
                        \reeseWobbleRate,
                        \reeseWobbleDepth,
                        \reeseDrive,
                        \reeseLp,
                        \reeseRq,
                        \reeseSubMix,
                        \amp,
                        \pan
                    ]
                } {
                    if (voiceMode == \wowVoice) {
                        [
                            \wowFreq,
                            \wowGlide,
                            \wowDirection,
                            \wowVowelDepth,
                            \wowBreath,
                            \wowLp,
                            \wowTone,
                            \wowTrig,
                            \amp,
                            \pan
                        ]
                    } {
                        [
                            \scaleMode,
                            \playMode,
                            \stepIndex,
                            \syncFreq,
                            \grain1Freq,
                            \grain2Freq,
                            \decay1,
                            \decay2,
                            \drive,
                            \crushBits,
                            \hp,
                            \lp,
                            \amp,
                            \pan
                        ]
                    }
                }
            }
        };
    };

    ~swLiveStatusSummary = {
        var inputs = ~swLiveInputState ? ();
        var params = ~swAuduinoParams ? ();
        var voiceMode = params[\voiceMode] ? \auduino;
        var extraParamLines = [];
        var paramLines = ~swLiveStatusParamKeysForMode.(voiceMode).collect { |key|
            "  " ++ key.asString ++ ": " ++ ~swFormatStatusValue.(params[key])
        };

        if (voiceMode == \smoothSine) {
            extraParamLines = [
                "  smoothNote: " ++ ~swMidiNoteLabel.(params[\smoothMidi]),
                "  smoothFreqHz: " ++ ~swFormatStatusValue.(params[\smoothFreq]),
                "  smoothRoot: " ++ ~swMidiNoteLabel.(params[\smoothRootMidi] ? 38)
            ];
        } {
            if (voiceMode == \ambientFlow) {
                extraParamLines = [
                    "  ambientPrimary: " ++ ~swFormatStatusValue.(params[\ambientBlendPrimaryVariant]),
                    "  ambientSecondary: " ++ ~swFormatStatusValue.(params[\ambientBlendSecondaryVariant])
                ];
            };
        };

        ([
            "Mode: " ++ voiceMode.asString,
            "Motion gate open: " ++ (~swMotionGateOpen ? false).asString,
            "",
            "Inputs:",
            "  gate: " ++ ~swFormatStatusValue.(inputs[\gate] ? 0.0),
            "  activity: " ++ ~swFormatStatusValue.(inputs[\activity] ? 0.0),
            "  pitchNorm: " ++ ~swFormatStatusValue.(inputs[\pitchNorm] ? 0.0),
            "  pitchDeg: " ++ ~swFormatStatusValue.(inputs[\pitchDeg] ? 0.0),
            "  rollNorm: " ++ ~swFormatStatusValue.(inputs[\rollNorm] ? 0.0),
            "  rollDeg: " ++ ~swFormatStatusValue.(inputs[\rollDeg] ? 0.0),
            "  spinNorm: " ++ ~swFormatStatusValue.(inputs[\spinNorm] ? 0.0),
            "  yawNorm: " ++ ~swFormatStatusValue.(inputs[\yawNorm] ? 0.0),
            "  yawDeg: " ++ ~swFormatStatusValue.(inputs[\yawDeg] ? 0.0),
            "",
            "Params:"
        ] ++ extraParamLines ++ paramLines).join("\n");
    };

    ~swPerformLiveStatusGUIRefresh = {
        var summary;
        var params = ~swAuduinoParams ? ();
        var voiceMode = params[\voiceMode] ? \auduino;
        var scaleMode = params[\scaleMode] ? \originalAuduino;
        var smoothExperiment = params[\smoothExperiment] ? \triggeredNotes;
        var voiceIndex = ~swLiveVoiceIndexFor.(voiceMode);
        var scaleIndex = ~swAuduinoScaleIndexFor.(scaleMode);
        var experimentIndex = ~swSmoothExperimentIndexFor.(smoothExperiment);

        if (~swLiveStatusText.notNil) {
            summary = ~swLiveStatusSummary.();
            if (summary != ~swLiveStatusLastSummary) {
                ~swLiveStatusLastSummary = summary;
                ~swLiveStatusText.string_(summary);
            };
        };

        if (~swLiveStatusModeMenu.notNil and: { ~swLiveStatusModeMenu.value != voiceIndex }) {
            ~swLiveStatusModeMenu.value_(voiceIndex);
        };

        if (~swLiveStatusScaleMenu.notNil and: { ~swLiveStatusScaleMenu.value != scaleIndex }) {
            ~swLiveStatusScaleMenu.value_(scaleIndex);
        };

        if (~swLiveStatusSmoothExperimentMenu.notNil and: { ~swLiveStatusSmoothExperimentMenu.value != experimentIndex }) {
            ~swLiveStatusSmoothExperimentMenu.value_(experimentIndex);
        };

        ~swLiveStatusNeedsRefresh = false;
    };

    ~swRequestLiveStatusGUIRefresh = {
        ~swLiveStatusNeedsRefresh = true;
    };

    ~swStopLiveStatusGUIRefreshLoop = {
        if (~swLiveStatusRefreshRoutine.notNil) {
            ~swLiveStatusRefreshRoutine.stop;
            ~swLiveStatusRefreshRoutine = nil;
        };
    };

    ~swStartLiveStatusGUIRefreshLoop = {
        ~swStopLiveStatusGUIRefreshLoop.();
        ~swLiveStatusRefreshRoutine = Routine({
            loop {
                if (~swLiveStatusText.notNil and: { ~swLiveStatusNeedsRefresh ? true }) {
                    ~swPerformLiveStatusGUIRefresh.();
                };
                (~swLiveStatusRefreshPeriod ? 0.04).wait;
            };
        }).play(AppClock);
    };

    ~swRefreshLiveStatusGUI = {
        ~swRequestLiveStatusGUIRefresh.();
    };

    ~swLiveStatusGUI = {
        var win;
        var currentParams;

        if (~swLiveStatusWindow.notNil) {
            ~swLiveStatusWindow.front;
            ~swStartLiveStatusGUIRefreshLoop.();
            ~swRefreshLiveStatusGUI.();
            ^~swLiveStatusWindow;
        };

        win = Window("SpaceWizard Live Status", Rect(700, 80, 520, 660));
        win.alwaysOnTop_(true);
        ~swLiveStatusWindow = win;
        ~swLiveStatusLastSummary = nil;
        ~swLiveStatusNeedsRefresh = true;
        currentParams = ~swAuduinoParams ? ();

        StaticText(win, Rect(12, 12, 46, 20))
            .string_("Output")
            .align_(\left);

        ~swLiveStatusModeMenu = PopUpMenu(win, Rect(60, 10, 184, 24))
            .items_(~swLiveVoiceOptions.collect { |item| item[\label] })
            .value_(~swLiveVoiceIndexFor.(currentParams[\voiceMode] ? \auduino))
            .action_({ |menu|
                var selected = ~swLiveVoiceOptions[menu.value] ? ~swLiveVoiceOptions.first;
                ~swSetLiveMode.(selected[\key]);
            });

        StaticText(win, Rect(258, 12, 86, 20))
            .string_("Auduino scale")
            .align_(\left);

        ~swLiveStatusScaleMenu = PopUpMenu(win, Rect(346, 10, 162, 24))
            .items_(~swAuduinoScaleOptions.collect { |item| item[\label] })
            .value_(~swAuduinoScaleIndexFor.(currentParams[\scaleMode] ? \originalAuduino))
            .action_({ |menu|
                var params = (~swAuduinoParams ? ~swAuduinoDefaultParams.copy).copy;
                var selected = ~swAuduinoScaleOptions[menu.value] ? ~swAuduinoScaleOptions.first;
                params[\scaleMode] = selected[\key];
                ~swAuduinoParams = params;
                if ((~swAuduino.notNil) or: { ~swAmbientHasVoices.() } or: { ~swSmoothHasVoices.() } or: { ~swReeseBass.notNil } or: { ~swWowVoice.notNil }) {
                    ~swApplyLiveModeCurrent.();
                };
                ~swRefreshLiveStatusGUI.();
            });

        Button(win, Rect(12, 44, 112, 24))
            .states_([[
                "Auduino GUI"
            ]])
            .action_({ ~swAuduinoGUI.(); });

        Button(win, Rect(136, 44, 112, 24))
            .states_([[
                "Ambient GUI"
            ]])
            .action_({ ~swAmbientGUI.(); });

        StaticText(win, Rect(260, 46, 98, 20))
            .string_("Smooth exp")
            .align_(\left);

        ~swLiveStatusSmoothExperimentMenu = PopUpMenu(win, Rect(360, 44, 148, 24))
            .items_(~swSmoothExperimentOptions.collect { |item| item[\label] })
            .value_(~swSmoothExperimentIndexFor.(currentParams[\smoothExperiment] ? \triggeredNotes))
            .action_({ |menu|
                var params = (~swAuduinoParams ? ~swAuduinoDefaultParams.copy).copy;
                var selected = ~swSmoothExperimentOptions[menu.value] ? ~swSmoothExperimentOptions.first;
                params[\smoothExperiment] = selected[\key];
                ~swAuduinoParams = params;
                ~swSmoothStop.();
                if ((params[\voiceMode] ? \auduino) == \smoothSine) {
                    ~swApplyLiveModeCurrent.();
                };
                ~swRefreshLiveStatusGUI.();
            });

        ~swLiveStatusText = TextView(win, Rect(12, 110, 496, 502))
            .font_(Font("Menlo", 12))
            .editable_(false)
            .string_("");

        Button(win, Rect(12, 622, 92, 28))
            .states_([[
                "Refresh"
            ]])
            .action_({ ~swPerformLiveStatusGUIRefresh.(); });

        Button(win, Rect(116, 622, 92, 28))
            .states_([[
                "Close"
            ]])
            .action_({ win.close; });

        win.onClose_({
            ~swStopLiveStatusGUIRefreshLoop.();
            ~swLiveStatusWindow = nil;
            ~swLiveStatusText = nil;
            ~swLiveStatusModeMenu = nil;
            ~swLiveStatusScaleMenu = nil;
            ~swLiveStatusSmoothExperimentMenu = nil;
            ~swLiveStatusLastSummary = nil;
            ~swLiveStatusNeedsRefresh = true;
        });

        win.front;
        ~swStartLiveStatusGUIRefreshLoop.();
        ~swRefreshLiveStatusGUI.();
    };

    // Keep helper definitions in their own compiler scope: the main patch is
    // close to SuperCollider's per-function selector/literal limit.
    {
    ~swLiveInputValid = false;
    ~swLastLiveControlAt = nil;
    ~swLiveInputTimeoutSeconds = 0.75;
    if (~swLiveWatchdogRoutine.notNil) { ~swLiveWatchdogRoutine.stop; };
    ~swSetMotionGate = { |isOpen=false|
        var open = isOpen.asBoolean;
        var wasOpen = ~swMotionGateOpen ? false;

        ~swMotionGateOpen = open;
        if (~swMotionLevelBus.notNil) {
            ~swMotionLevelBus.set(open.if(1.0, 0.0));
        };
        ~swSetSmoothLayerGate.(open);
        if ((((~swAuduinoParams ? ())[\voiceMode] ? \auduino) == \song) and: { open.not }) {
            // Safety: the regular gate closing (set down, stale input, watchdog) always stops the song.
            ~swSongSetGate.(false);
        };
        if (open and: { ~swMotionCleanupRoutine.notNil }) {
            ~swMotionCleanupRoutine.stop;
            ~swMotionCleanupRoutine = nil;
        };
        if (open.not and: { wasOpen }) {
            ~swResetSmoothTracking.();
            ~swResetAmbientBassTracking.();
            ~swMotionCleanupRoutine = Routine({
                ((~swMotionFadeOutSeconds ? 5.0) + 0.35).wait;
                ~swMotionCleanupRoutine = nil;
                if (~swMotionGateOpen.not) {
                    ~swSilenceLiveVoices.();
                };
            }).play(AppClock);
        };
    };

    ~swStandPromptStop = {
        // Always invalidate callbacks, including ones whose pending marker was cleared.
        ~swStandPromptToken = (~swStandPromptToken ? 0) + 1;
        ~swStandPromptRoutine = nil;
        if (~swStandPrompt.notNil) {
            ~swStandPrompt.set(\gate, 0);
            ~swStandPrompt = nil;
        };
    };

    ~swStandPromptPlay = { |params|
        var enabled = (params[\standPromptEnabled] ? true);
        var amp = (params[\standPromptAmp] ? 1.0).clip(0.0, 1.2);
        var voice;

        if (enabled and: { ~swStandPromptHasSample ? false }) {
            ~swStandPromptStop.();
            "Stand prompt playing.".postln;
            voice = Synth(\spacewizardStandPrompt, [
                \buf, ~swStandPromptBufNum.(),
                \amp, amp,
                \pan, 0.0
            ]);
            ~swStandPrompt = voice;
            voice.onFree {
                if (~swStandPrompt === voice) {
                    ~swStandPrompt = nil;
                };
            };
        };
    };

    ~swStandPromptCancel = {
        ~swStandPromptStop.();
    };

    ~swStandPromptSchedule = { |params|
        var fadeOut = (~swMotionFadeOutSeconds ? 5.0).max(0.0);
        var postFadeDelay = (params[\standPromptDelaySeconds] ? 2.5).max(0.0);
        var delay = (fadeOut + postFadeDelay).clip(0.0, 30.0);
        var pitchMin = (params[\standPromptPitchMin] ? 0.86).clip(-1.0, 1.0);
        var token;

        if (~swStandPromptRoutine.isNil) {
            ~swStandPromptToken = (~swStandPromptToken ? 0) + 1;
            token = ~swStandPromptToken;
            ("Stand prompt scheduled in " ++ delay.round(0.01).asString
                ++ "s (fadeOut=" ++ fadeOut.round(0.01).asString
                ++ "s + postFadeDelay=" ++ postFadeDelay.round(0.01).asString
                ++ "s) at pitchNorm=" ++ (~swStandPromptLastPitchNorm ? 0.0).round(0.001).asString).postln;
            ~swStandPromptRoutine = token;
            AppClock.sched(delay, {
                if ((token == (~swStandPromptToken ? -1))
                    and: { ~swStandPromptRoutine == token }) {
                    // An obsolete callback must never clear a newer callback's marker.
                    ~swStandPromptRoutine = nil;
                    if ((~swLiveInputValid ? false)
                        and: { ~swLastLiveControlAt.notNil }
                        and: { (Main.elapsedTime - ~swLastLiveControlAt) < ~swLiveInputTimeoutSeconds }
                        and: { (~swMotionGateOpen ? false).not }
                        and: { ((~swLiveInputState ? ())[\activity] ? 0.0) <= (params[\standPromptStopActivity] ? 0.01) }
                        and: { (~swStandPromptLastPitchNorm ? 0.0) >= pitchMin }) {
                        ~swStandPromptPlay.(params);
                        ~swStandPromptPrompted = true;
                        ~swStandPromptArmed = false;
                    };
                };
                nil;
            });
        };
    };

    ~swStandPromptUpdate = { |params, gate=0.0, activity=0.0, pitchNorm=0.0|
        var enabled = params[\standPromptEnabled] ? true;
        var gateOpen = gate > 0.0;
        var motionStopActivity = (params[\standPromptStopActivity] ? 0.01).clip(0.0, 1.0);
        var pitchMin = (params[\standPromptPitchMin] ? 0.86).clip(-1.0, 1.0);
        var requirePriorMotion = params[\standPromptRequirePriorMotion] ? true;
        var hasPriorMotion = ~swStandPromptArmed ? false;
        var pitchOk = pitchNorm >= pitchMin;

        ~swStandPromptLastPitchNorm = pitchNorm;

        if (enabled.not) {
            ~swStandPromptCancel.();
            ~swStandPromptArmed = false;
            ~swStandPromptPrompted = false;
        } {
            if (gateOpen or: { activity > motionStopActivity }) {
                ~swStandPromptCancel.();
                ~swStandPromptArmed = true;
                ~swStandPromptPrompted = false;
            } {
                if (pitchOk.not) {
                    ~swStandPromptCancel.();
                    ~swStandPromptArmed = true;
                    ~swStandPromptPrompted = false;
                } {
                    if ((((hasPriorMotion or: { requirePriorMotion.not })
                        and: { pitchOk })
                        and: { (~swStandPromptPrompted ? false).not })
                        and: { ~swStandPrompt.isNil }) {
                        ~swStandPromptSchedule.(params.copy);
                    };
                };
            };
        };
    };


    ~swExpireLiveInput = {
        ~swLiveInputValid = false;
        ~swSetMotionGate.(false);
        ~swStandPromptCancel.();
        ~swStandPromptArmed = false;
        ~swStandPromptPrompted = false;
        ~swUpdateLiveInputState.(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    };
    ~swCheckLiveInput = { |now|
        now = now ? Main.elapsedTime;
        if (~swLastLiveControlAt.notNil
            and: { (now - ~swLastLiveControlAt) >= ~swLiveInputTimeoutSeconds }) {
            ~swExpireLiveInput.();
            ~swLastLiveControlAt = nil;
            "Live input timed out; motion gate closed and stand prompt cancelled.".postln;
        };
    };
    ~swStartLiveWatchdog = {
    ~swLiveWatchdogRoutine = Routine({
        var lastHealthWrite = -1.0;
        loop {
            ~swCheckLiveInput.();
            if (~swHealthFile.notNil and: { s.serverRunning }
                and: { (Main.elapsedTime - lastHealthWrite) >= 1.0 }) {
                File.use(~swHealthFile, "w", { |file| file.write("ready\n"); });
                lastHealthWrite = Main.elapsedTime;
            };
            0.1.wait;
        };
    }).play(AppClock);
    };
    }.value;

    ~swSmoothLayerBusIndex = {
        var bus = ~swSmoothLayerLevelBus ? ~swMotionLevelBus;
        var index = 0;
        if (bus.notNil) {
            index = bus.index;
        };
        index;
    };

    ~swSmoothLayerRiseSeconds = {
        (~swSmoothLayerFadeInSeconds ? ~swMotionFadeInSeconds ? 0.18).clip(0.01, 12.0);
    };

    ~swSetSmoothLayerGate = { |isOpen=false|
        var open = isOpen.asBoolean;
        var wasOpen = ~swSmoothLayerGateOpen ? false;

        if (wasOpen != open) {
            ~swSmoothLayerGateOpen = open;
            ~swSmoothLayerGateToken = (~swSmoothLayerGateToken ? 0) + 1;

            if (~swSmoothLayerGateRoutine.notNil) {
                ~swSmoothLayerGateRoutine.stop;
                ~swSmoothLayerGateRoutine = nil;
            };

            if (~swSmoothLayerLevelBus.notNil) {
                if (open) {
                    var token = ~swSmoothLayerGateToken;
                    var delay = (~swSmoothLayerFadeDelaySeconds ? 1.0).max(0.0);

                    ~swSmoothLayerLevelBus.set(0.0);
                    ~swSmoothLayerGateRoutine = Routine({
                        delay.wait;
                        if ((~swSmoothLayerGateOpen ? false) and: { token == (~swSmoothLayerGateToken ? -1) }) {
                            ~swSmoothLayerLevelBus.set(1.0);
                        };
                        ~swSmoothLayerGateRoutine = nil;
                    }).play(AppClock);
                } {
                    ~swSmoothLayerLevelBus.set(0.0);
                };
            };
        };
    };

    ~swSmoothHasVoices = {
        ((~swSmoothVoices.notNil) and: { ~swSmoothVoices.size > 0 }) or: { ~swSmoothMonoVoice.notNil } or: { ~swSmoothChoir.notNil };
    };

    ~swAmbientHasVoices = {
        (~swAmbientVoices.notNil) and: { ~swAmbientVoices.size > 0 };
    };

    ~swAmbientVariantIndexFor = { |variant=\flow|
        ~swAmbientVariantKeys.detectIndex { |key| key == variant } ? 0;
    };

    ~swAmbientYawOffsetForVariant = { |variant=\flow|
        (~swAmbientVariantIndexFor.(variant).asFloat / (~swAmbientVariantKeys.size.max(1)).asFloat);
    };

    ~swAmbientBlendStateFor = { |blendPosition=0.0|
        var count = ~swAmbientVariantKeys.size.max(1);
        var wrapped = blendPosition.wrap(0.0, 1.0);
        var scaled = wrapped * count;
        var primaryIndex = scaled.floor.asInteger.clip(0, count - 1);
        var mix = scaled - primaryIndex;
        var secondaryIndex = (primaryIndex + 1) % count;

        (
            position: wrapped,
            primaryVariant: ~swAmbientVariantKeys[primaryIndex],
            secondaryVariant: ~swAmbientVariantKeys[secondaryIndex],
            primaryWeight: 1.0 - mix,
            secondaryWeight: mix
        )
    };

    ~swAmbientDefFor = { |variant=\flow|
        if (variant == \chordal) { \spacewizardAmbientChords }
        {
            if (variant == \drone) { \spacewizardAmbientDrone }
            {
                if (variant == \forest) { \spacewizardAmbientForest } { \spacewizardAmbientFlow }
            }
        };
    };

    ~swAmbientPresetFor = { |variant=\flow|
        var selected = variant ? \flow;
        var preset;

        if (selected == \chordal) {
            preset = (
                voiceMode: \ambientFlow,
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
            );
        } {
            if (selected == \drone) {
                preset = (
                    voiceMode: \ambientFlow,
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
                );
            } {
                if (selected == \forest) {
                    preset = (
                        voiceMode: \ambientFlow,
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
                    );
                } {
                    preset = (
                        voiceMode: \ambientFlow,
                        ambientVariant: \flow,
                        ambientRoot: 74,
                        ambientLift: -0.1,
                        ambientLean: -0.15,
                        ambientMotion: 0.22,
                        ambientSpin: 0.14,
                        ambientAura: 0.62,
                        ambientDarkness: 0.66,
                        ambientShimmer: 0.34,
                        ambientWidth: 0.78,
                        amp: 0.1,
                        pan: 0.0
                    );
                }
            }
        };
        preset.putAll(~swSoundPresetOverridesFor.(\ambientFlow, selected));
    };

    ~swAmbientDescriptionFor = { |variant=\flow|
        if (variant == \chordal) {
            "Slow chordal pad with drifting voicings.\nTilt opens the harmony, roll warms or brightens the blend, motion nudges the changes, and spin wakes the glints."
        } {
            if (variant == \drone) {
                "Dense textural drone with undertow, friction, and low bass-note pulses in live yaw mode.\nTilt changes the weight, roll shifts the shadow, motion thickens the mass, and spin roughens the air."
            } {
                if (variant == \forest) {
                    "Forest canopy bed with wind, leaf rustle, insects, and distant calls.\nTilt opens the canopy, roll shifts the light, motion stirs the leaves, and spin wakes the shimmer."
                } {
                    "Non-repeating ambient bed.\nTilt opens or darkens the space, roll shifts the width, motion adds activity, and spin adds shimmer."
                }
            }
        };
    };

    ~swAmbientSweepStatesFor = { |variant=\flow|
        if (variant == \chordal) {
            [
                (ambientRoot: 62, ambientLift: -0.42, ambientLean: -0.18, ambientMotion: 0.06, ambientSpin: 0.05, ambientAura: 0.68, ambientDarkness: 0.64, ambientShimmer: 0.14, ambientWidth: 0.52, pan: -0.12, amp: 0.08),
                (ambientRoot: 74, ambientLift: -0.1, ambientLean: 0.0, ambientMotion: 0.16, ambientSpin: 0.12, ambientAura: 0.78, ambientDarkness: 0.46, ambientShimmer: 0.22, ambientWidth: 0.68, pan: -0.04, amp: 0.1),
                (ambientRoot: 88, ambientLift: 0.18, ambientLean: 0.22, ambientMotion: 0.3, ambientSpin: 0.18, ambientAura: 0.86, ambientDarkness: 0.28, ambientShimmer: 0.36, ambientWidth: 0.8, pan: 0.08, amp: 0.12),
                (ambientRoot: 110, ambientLift: 0.48, ambientLean: 0.56, ambientMotion: 0.44, ambientSpin: 0.3, ambientAura: 0.92, ambientDarkness: 0.14, ambientShimmer: 0.52, ambientWidth: 0.92, pan: 0.18, amp: 0.13)
            ]
        } {
            if (variant == \drone) {
                [
                    (ambientRoot: 48, ambientLift: -0.7, ambientLean: -0.46, ambientMotion: 0.08, ambientSpin: 0.04, ambientAura: 0.54, ambientDarkness: 0.92, ambientShimmer: 0.06, ambientWidth: 0.42, pan: -0.12, amp: 0.08),
                    (ambientRoot: 58, ambientLift: -0.38, ambientLean: -0.2, ambientMotion: 0.18, ambientSpin: 0.08, ambientAura: 0.62, ambientDarkness: 0.82, ambientShimmer: 0.12, ambientWidth: 0.56, pan: -0.05, amp: 0.095),
                    (ambientRoot: 72, ambientLift: 0.0, ambientLean: 0.12, ambientMotion: 0.3, ambientSpin: 0.16, ambientAura: 0.74, ambientDarkness: 0.66, ambientShimmer: 0.2, ambientWidth: 0.68, pan: 0.04, amp: 0.11),
                    (ambientRoot: 92, ambientLift: 0.38, ambientLean: 0.36, ambientMotion: 0.48, ambientSpin: 0.28, ambientAura: 0.86, ambientDarkness: 0.44, ambientShimmer: 0.34, ambientWidth: 0.82, pan: 0.12, amp: 0.125)
                ]
            } {
                if (variant == \forest) {
                    [
                        (ambientRoot: 56, ambientLift: -0.22, ambientLean: -0.46, ambientMotion: 0.24, ambientSpin: 0.18, ambientAura: 0.64, ambientDarkness: 0.8, ambientShimmer: 0.26, ambientWidth: 0.72, pan: -0.14, amp: 0.08),
                        (ambientRoot: 68, ambientLift: 0.02, ambientLean: -0.12, ambientMotion: 0.38, ambientSpin: 0.3, ambientAura: 0.76, ambientDarkness: 0.62, ambientShimmer: 0.42, ambientWidth: 0.84, pan: -0.06, amp: 0.095),
                        (ambientRoot: 84, ambientLift: 0.24, ambientLean: 0.16, ambientMotion: 0.56, ambientSpin: 0.48, ambientAura: 0.88, ambientDarkness: 0.44, ambientShimmer: 0.6, ambientWidth: 0.92, pan: 0.08, amp: 0.11),
                        (ambientRoot: 108, ambientLift: 0.62, ambientLean: 0.42, ambientMotion: 0.74, ambientSpin: 0.7, ambientAura: 0.96, ambientDarkness: 0.24, ambientShimmer: 0.82, ambientWidth: 0.98, pan: 0.16, amp: 0.13)
                    ]
                } {
                    [
                        (ambientRoot: 58, ambientLift: -0.78, ambientLean: -0.72, ambientMotion: 0.14, ambientSpin: 0.06, ambientAura: 0.42, ambientDarkness: 0.86, ambientShimmer: 0.18, ambientWidth: 0.44, pan: -0.18, amp: 0.08),
                        (ambientRoot: 72, ambientLift: -0.22, ambientLean: -0.18, ambientMotion: 0.28, ambientSpin: 0.14, ambientAura: 0.56, ambientDarkness: 0.72, ambientShimmer: 0.3, ambientWidth: 0.62, pan: -0.08, amp: 0.092),
                        (ambientRoot: 98, ambientLift: 0.26, ambientLean: 0.18, ambientMotion: 0.46, ambientSpin: 0.28, ambientAura: 0.72, ambientDarkness: 0.5, ambientShimmer: 0.48, ambientWidth: 0.82, pan: 0.08, amp: 0.108),
                        (ambientRoot: 136, ambientLift: 0.84, ambientLean: 0.72, ambientMotion: 0.72, ambientSpin: 0.56, ambientAura: 0.9, ambientDarkness: 0.24, ambientShimmer: 0.78, ambientWidth: 0.96, pan: 0.22, amp: 0.128)
                    ]
                }
            }
        };
    };

    ~swEnsureVoice = {
        if (~swVoice.isNil) {
            ~swVoice = Synth(\spacewizardVoice, [
                \motionBus, ~swMotionLevelBus.index,
                \motionRise, ~swMotionFadeInSeconds,
                \motionFall, ~swMotionFadeOutSeconds
            ]);
            ~swVoice.onFree { ~swVoice = nil; };
        };
        ~swVoice;
    };

    ~swPickup = { |activity=0.35, tilt=0.0, roll=0.0, spin=0.08, presence=0.6, amp=0.18|
        var voice = ~swEnsureVoice.();
        Synth(\spacewizardPickupBloom, [
            \tilt, tilt,
            \spin, spin,
            \brightness, activity.max(presence),
            \amp, amp * 1.2
        ]);
        voice.set(
            \gate, 1,
            \activity, activity,
            \tilt, tilt,
            \roll, roll,
            \spin, spin,
            \presence, presence,
            \amp, amp
        );
    };

    ~swSet = { |activity=0.3, tilt=0.0, roll=0.0, spin=0.08, presence=0.58, amp=0.18|
        var voice = ~swEnsureVoice.();
        voice.set(
            \activity, activity,
            \tilt, tilt,
            \roll, roll,
            \spin, spin,
            \presence, presence,
            \amp, amp
        );
    };

    ~swRest = {
        if (~swVoice.notNil) {
            ~swVoice.set(
                \activity, 0.02,
                \spin, 0.0,
                \presence, 0.28,
                \amp, 0.1
            );
        };
    };

    ~swRelease = {
        if (~swVoice.notNil) {
            ~swVoice.set(\gate, 0);
            ~swVoice = nil;
        };
    };

    ~swSilenceLiveVoices = {
        if (~swMotionCleanupRoutine.notNil) {
            ~swMotionCleanupRoutine.stop;
            ~swMotionCleanupRoutine = nil;
        };
        ~swSongStop.();
        ~swRelease.();
        if (~swSawLead.notNil) {
            ~swSawLead.set(\gate, 0);
            ~swSawLead = nil;
        };
        ~swSmoothStop.();
        ~swSmoothChoirStop.();
        if (~swReeseBass.notNil) {
            ~swReeseBass.set(\gate, 0);
            ~swReeseBass = nil;
        };
        if (~swWowVoice.notNil) {
            ~swWowVoice.set(\gate, 0);
            ~swWowVoice = nil;
        };
        ~swAmbientStop.();
        ~swAmbientLabStop.();
        ~swResetWowTracking.();
        ~swResetAmbientBassTracking.();
        ~swAuduinoStopAll.();
    };

    ~swStopAll = {
        ~swSilenceLiveVoices.();
        s.freeAll;
    };

    ~swPentatonicMidis = { |rootMidi=50|
        [0, 3, 5, 7, 10, 12, 15, 17, 19, 22].collect { |degree| rootMidi + degree };
    };

    ~swScaleMidiSpan = { |rootMidi=50, count=24, degrees, startStep=0|
        var scaleDegrees = degrees ? [0, 3, 5, 7, 10];
        Array.fill(count, { |index|
            var relativeIndex = index + startStep;
            rootMidi + scaleDegrees.wrapAt(relativeIndex) + ((relativeIndex / scaleDegrees.size).floor * 12)
        });
    };

    ~swAuduinoStepCount = 24;
    ~swAuduinoStepStart = -3;
    ~swAuduinoStepSyncMinHz = 8.0;
    ~swAuduinoStepSyncMaxHz = 160.0;
    ~swAuduinoOriginalScaleDegrees = [0, 2, 5, 7, 9];

    ~swAuduinoOriginalPentatonicPhaseIncs = [
        19, 22, 26, 29, 32, 38, 43, 51, 58, 65, 77, 86, 103, 115, 129,
        154, 173, 206, 231, 259, 308, 346, 411, 461, 518, 616, 691, 822,
        923, 1036, 1232, 1383, 1644, 1845, 2071, 2463, 2765, 3288, 3691,
        4143, 4927, 5530, 6577, 7382, 8286, 9854, 11060, 13153, 14764,
        16572, 19708, 22121, 26306
    ];

    ~swAuduinoScaleOptions = [
        (key: \originalAuduino, label: "Original Auduino D-E-G-A-B"),
        (key: \spacewizardMinor, label: "SpaceWizard minor pentatonic")
    ];

    ~swAuduinoPhaseIncToHz = { |phaseInc|
        (phaseInc * 31250.0) / 65536.0;
    };

    ~swAuduinoOriginalPentatonicMidis = {
        ~swAuduinoOriginalPentatonicPhaseIncs.collect { |phaseInc|
            ~swAuduinoPhaseIncToHz.(phaseInc).cpsmidi;
        };
    };

    ~swAuduinoNotesForScale = { |scaleMode=\originalAuduino, rootMidi=50, count=10|
        var selectedMode = scaleMode ? \originalAuduino;

        if (selectedMode == \originalAuduino) {
            var source = ~swAuduinoOriginalPentatonicMidis.();
            var safeCount = count.clip(1, source.size).asInteger;
            var distances = source.collect { |midi| (midi - rootMidi).abs };
            var nearestDistance = distances.minItem;
            var nearestIndex = distances.indexOf(nearestDistance) ? 0;
            var start = (nearestIndex - (safeCount / 2).floor).clip(0, source.size - safeCount).asInteger;
            source.copyRange(start, start + safeCount - 1);
        } {
            ~swPentatonicMidis.(rootMidi);
        };
    };

    ~swAuduinoStepNotesForScale = { |scaleMode=\originalAuduino, rootMidi=50|
        var selectedMode = scaleMode ? \originalAuduino;

        if (selectedMode == \originalAuduino) {
            ~swScaleMidiSpan.(rootMidi, ~swAuduinoStepCount, ~swAuduinoOriginalScaleDegrees, ~swAuduinoStepStart);
        } {
            ~swScaleMidiSpan.(rootMidi, ~swAuduinoStepCount, [0, 3, 5, 7, 10], ~swAuduinoStepStart);
        };
    };

    ~swAuduinoStepMidiForIndex = { |stepIndex=0, scaleMode=\originalAuduino, rootMidi=50|
        var notes = ~swAuduinoStepNotesForScale.(scaleMode, rootMidi);
        var clippedIndex = stepIndex.round.clip(0, notes.size - 1).asInteger;
        notes[clippedIndex];
    };

    ~swAuduinoStepIndexForSyncHz = { |syncFreq=42.0|
        syncFreq.clip(~swAuduinoStepSyncMinHz, ~swAuduinoStepSyncMaxHz)
            .explin(~swAuduinoStepSyncMinHz, ~swAuduinoStepSyncMaxHz, 0.0, (~swAuduinoStepCount - 1).asFloat)
            .round
            .asInteger;
    };

    ~swAuduinoApplySteppedCurrent = {
        var params = ~swAuduinoParams;
        var playMode = params[\playMode] ? \stepped;
        var melodic = playMode == \melodicStepped;
        var stepIndex = ~swAuduinoStepIndexForSyncHz.(params[\syncFreq] ? 42.0);
        var midi = ~swAuduinoStepMidiForIndex.(
            stepIndex,
            params[\scaleMode] ? \originalAuduino,
            params[\rootMidi] ? 50
        );
        var baseFreq = midi.midicps;
        var syncBaseMultiplier = melodic.if(0.16, 0.36);
        var grain1Scale = (params[\grain1Freq] ? 196.0) / 196.0;
        var grain2Scale = (params[\grain2Freq] ? 294.0) / 294.0;

        params[\stepIndex] = stepIndex;
        ~swAuduinoParams = params;

        ~swAuduinoSet.(
            syncFreq: (baseFreq * syncBaseMultiplier).clip(8, 1200),
            grain1Freq: (baseFreq * grain1Scale).clip(40, 12000),
            grain2Freq: (baseFreq * grain2Scale).clip(40, 14000),
            decay1: params[\decay1] ? 0.14,
            decay2: params[\decay2] ? 0.09,
            amp: params[\amp] ? 0.12,
            pan: params[\pan] ? 0.0,
            drive: params[\drive] ? 2.6,
            crushBits: params[\crushBits] ? 7,
            hp: params[\hp] ? 60,
            lp: params[\lp] ? 4600,
            scaleMode: params[\scaleMode] ? \originalAuduino,
            playMode: playMode,
            rootMidi: params[\rootMidi] ? 50,
            stepIndex: stepIndex
        );
    };

    ~swSawLeadEnsure = { |rootMidi=50, amp=0.12, drive=4.8|
        if (~swSawLead.isNil) {
            ~swSawLead = Synth(\spacewizardSawLead, [
                \freq, rootMidi.midicps,
                \amp, amp,
                \drive, drive,
                \motionBus, ~swMotionLevelBus.index,
                \motionRise, ~swMotionFadeInSeconds,
                \motionFall, ~swMotionFadeOutSeconds
            ]);
            ~swSawLead.onFree { ~swSawLead = nil; };
        };
        ~swSawLead;
    };

    ~swSawLeadStart = { |midi=50, amp=0.12, pan=0.0, cutoff=2200, rq=0.2, drive=4.8, glide=0.035|
        var lead = ~swSawLeadEnsure.(midi, amp, drive);
        lead.set(
            \gate, 1,
            \freq, midi.midicps,
            \amp, amp,
            \pan, pan,
            \cutoff, cutoff,
            \rq, rq,
            \drive, drive,
            \glide, glide
        );
    };

    ~swSawLeadSet = { |midi=50, amp=0.12, pan=0.0, cutoff=2600, rq=0.2, drive=4.8, glide=0.035|
        var lead = ~swSawLeadEnsure.(midi, amp, drive);
        lead.set(
            \freq, midi.midicps,
            \amp, amp,
            \pan, pan,
            \cutoff, cutoff,
            \rq, rq,
            \drive, drive,
            \glide, glide
        );
    };

    ~swSawLeadStop = {
        if (~swSawLead.notNil) {
            ~swSawLead.set(\gate, 0);
            ~swSawLead = nil;
        };
    };

    ~swSmoothBassMidis = { |rootMidi=38|
        ~swScaleMidiSpan.(rootMidi, 15, [0, 3, 5, 7, 10], 0);
    };

    ~swSmoothIndexForPitch = { |pitchNorm=0.0, rootMidi=38|
        var notes = ~swSmoothBassMidis.(rootMidi);

        pitchNorm.linlin(-1.0, 1.0, 0.0, (notes.size - 1).asFloat)
            .clip(0.0, (notes.size - 1).asFloat);
    };

    ~swSmoothMidiForIndex = { |index=0.0, rootMidi=38|
        var notes = ~swSmoothBassMidis.(rootMidi);
        var clippedIndex = index.round.clip(0, notes.size - 1).asInteger;

        notes[clippedIndex];
    };

    ~swSmoothMidiForPitch = { |pitchNorm=0.0, rootMidi=38|
        ~swSmoothMidiForIndex.(~swSmoothIndexForPitch.(pitchNorm, rootMidi), rootMidi);
    };

    ~swSmoothBassClarityForPitch = { |params, pitchNorm=0.0|
        var amount = ((params ? ())[\smoothBassClarityAmount] ? 0.0).clip(0.0, 1.0);
        var start = ((params ? ())[\smoothBassClarityPitchStart] ? 0.0).clip(-1.0, 1.0);
        var full = ((params ? ())[\smoothBassClarityPitchFull] ? -0.9).clip(-1.0, 1.0);
        var span = (start - full).abs.max(0.001);
        var shaped = ((start - pitchNorm.clip(-1.0, 1.0)) / span).clip(0.0, 1.0);

        amount * shaped;
    };

    ~swSmoothLiveDecayFor = { |params, activity=0.0, yawNorm=0.0|
        var decayMin = ((params ? ())[\smoothDecayMin] ? 1.7).clip(0.1, 8.0);
        var decayMax = ((params ? ())[\smoothDecayMax] ? 3.2).clip(0.1, 8.0);
        var yawMin = ((params ? ())[\smoothDecayYawMin] ? -0.1).clip(-2.0, 2.0);
        var yawMax = ((params ? ())[\smoothDecayYawMax] ? 1.0).clip(-2.0, 2.0);
        var lo = decayMin.min(decayMax);
        var hi = decayMin.max(decayMax);

        activity.clip(0.0, 1.0).linlin(0.0, 1.0, lo, hi)
            + yawNorm.clip(0.0, 1.0).linlin(0.0, 1.0, yawMin, yawMax);
    };

    ~swSmoothSpawnNote = { |freq=72, amp=0.1, pan=0.0, vibratoRate=0.18, vibratoDepth=0.006,
        motionDepth=0.06, bloom=0.18, lp=2200, hold=0.06, decay=2.6, crunch=0.0, reverb=0.14,
        bassClarity=0.0, bassClarityHpMax=62.0, bassClarityMudFreq=180.0, bassClarityMudCutDb=4.0|
        var voices = ~swSmoothVoices ? List.new;
        var tailBudget = (~swSmoothTailBudgetVoices ? 12).max(1).asInteger;
        var activeCount = voices.size.asFloat;
        var densityReference = (~swSmoothDensityReferenceVoices ? 6).max(1).asFloat;
        var density = activeCount / densityReference;
        var ampScale = density.linlin(0.0, 1.0, 1.0, 0.58).clip(0.32, 1.0);
        var holdScale = density.linlin(0.0, 1.0, 1.0, 0.86).clip(0.72, 1.0);
        var decayScale = density.linlin(0.0, 1.0, 1.0, 0.72).clip(0.42, 1.0);
        var bloomScale = density.linlin(0.0, 1.0, 1.0, 0.82).clip(0.7, 1.0);
        var lpScale = density.linlin(0.0, 1.0, 1.0, 0.88).clip(0.76, 1.0);
        var stolenVoice;
        var voice;
        var scaledHold = (hold * holdScale).max(0.02);
        var scaledDecay = (decay * decayScale).max(0.2);

        // Replace the oldest lingering tail so fast motion always gets the newest note.
            while { voices.size >= tailBudget } {
            stolenVoice = voices.removeAt(0);
            if (stolenVoice.notNil) {
                stolenVoice.set(\gate, 0);
            };
        };

        voice = Synth(\spacewizardSmoothSine, [
            \freq, freq,
            \amp, amp * ampScale,
            \pan, pan,
            \vibratoRate, vibratoRate,
            \vibratoDepth, vibratoDepth,
            \motionDepth, motionDepth,
            \bloom, bloom * bloomScale,
            \lp, lp * lpScale,
            \hold, scaledHold,
            \decay, scaledDecay,
            \crunch, crunch,
            \reverb, reverb,
            \bassClarity, bassClarity,
            \bassClarityHpMax, bassClarityHpMax,
            \bassClarityMudFreq, bassClarityMudFreq,
            \bassClarityMudCutDb, bassClarityMudCutDb,
            \motionBus, ~swSmoothLayerBusIndex.(),
            \motionRise, ~swSmoothLayerRiseSeconds.(),
            \motionFall, ~swMotionFadeOutSeconds
        ]);

        voices.add(voice);
        ~swSmoothVoices = voices;
        voice.onFree {
            if (~swSmoothVoices.notNil) {
                ~swSmoothVoices.remove(voice);
            };
        };
        AppClock.sched((scaledHold + scaledDecay + 0.6).clip(0.4, 10.0), {
            if (~swSmoothVoices.notNil) {
                ~swSmoothVoices.remove(voice);
            };
            nil;
        });
        voice;
    };

    ~swSmoothLowPitchAmpScaleFor = { |params|
        var pitchNorm = ((params ? ())[\smoothPitchNorm] ? 0.0).clip(-1.0, 1.0);
        var boost = ((params ? ())[\smoothLowPitchAmpBoost] ? 0.0).clip(0.0, 2.0);
        var start = ((params ? ())[\smoothLowPitchAmpStart] ? 0.0).clip(-1.0, 1.0);
        var full = ((params ? ())[\smoothLowPitchAmpFull] ? -0.85).clip(-1.0, 1.0);
        var span = (start - full).abs.max(0.001);
        var shaped = ((start - pitchNorm) / span).clip(0.0, 1.0);

        1.0 + (boost * shaped);
    };

    ~swSmoothLiveAmpFor = { |params, activity=0.0|
        if (((params ? ())[\smoothAmpFollowsActivity] ? true).not) {
            (((params ? ())[\smoothFixedAmp] ? 0.10).clip(0.0, 0.2));
        } {
            activity.clip(0.0, 1.0).linlin(0.0, 1.0, 0.05, 0.13);
        };
    };

    ~swSmoothScaledAmp = { |params|
        (((params ? ())[\amp] ? 0.1)
            * (((params ? ())[\smoothAmpScale]) ? 1.0)
            * ~swSmoothLowPitchAmpScaleFor.(params)).clip(0.0, 0.4);
    };

    ~swSmoothSet = { |freq=72, amp=0.1, pan=0.0, vibratoRate=0.18, vibratoDepth=0.006,
        motionDepth=0.06, bloom=0.18, lp=2200, hold=0.06, decay=2.6, crunch=0.0, reverb=0.14|
        var params = (~swAuduinoParams ? ~swAuduinoDefaultParams.copy).copy;
        var clarity;

        params[\voiceMode] = \smoothSine;
        params[\smoothFreq] = freq;
        params[\smoothMidi] = freq.cpsmidi.round.asInteger;
        params[\smoothVibratoRate] = vibratoRate;
        params[\smoothVibratoDepth] = vibratoDepth;
        params[\smoothMotionDepth] = motionDepth;
        params[\smoothBloom] = bloom;
        params[\smoothLp] = lp;
        params[\smoothCrunch] = crunch;
        params[\smoothReverb] = reverb;
        params[\smoothHold] = hold;
        params[\smoothDecay] = decay;
        params[\amp] = amp;
        params[\pan] = pan;
        ~swAuduinoParams = params;
        clarity = ~swSmoothBassClarityForPitch.(params, params[\smoothPitchNorm] ? 0.0);

        ~swSmoothSpawnNote.(
            freq: freq,
            amp: ~swSmoothScaledAmp.(params),
            pan: pan,
            vibratoRate: vibratoRate,
            vibratoDepth: vibratoDepth,
            motionDepth: motionDepth,
            bloom: bloom,
            lp: lp,
            hold: hold,
            decay: decay,
            crunch: crunch,
            reverb: reverb,
            bassClarity: clarity,
            bassClarityHpMax: params[\smoothBassClarityHpMax] ? 62.0,
            bassClarityMudFreq: params[\smoothBassClarityMudFreq] ? 180.0,
            bassClarityMudCutDb: params[\smoothBassClarityMudCutDb] ? 4.0
        );
    };

    ~swSmoothMonoSet = { |freq=72, amp=0.1, pan=0.0, vibratoRate=0.18, vibratoDepth=0.006,
        motionDepth=0.06, bloom=0.18, lp=2200, glide=0.0, crunch=0.0, reverb=0.14,
        bassClarity=0.0, bassClarityHpMax=62.0, bassClarityMudFreq=180.0, bassClarityMudCutDb=4.0|
        var voice = ~swSmoothMonoVoice;

        if (voice.isNil) {
            voice = Synth(\spacewizardSmoothSineMono, [
                \freq, freq,
                \amp, amp,
                \pan, pan,
                \vibratoRate, vibratoRate,
                \vibratoDepth, vibratoDepth,
                \motionDepth, motionDepth,
                \bloom, bloom,
                \lp, lp,
                \glide, glide,
                \crunch, crunch,
                \reverb, reverb,
                \bassClarity, bassClarity,
                \bassClarityHpMax, bassClarityHpMax,
                \bassClarityMudFreq, bassClarityMudFreq,
                \bassClarityMudCutDb, bassClarityMudCutDb,
                \motionBus, ~swSmoothLayerBusIndex.(),
                \motionRise, ~swSmoothLayerRiseSeconds.(),
                \motionFall, ~swMotionFadeOutSeconds
            ]);
            ~swSmoothMonoVoice = voice;
            voice.onFree {
                if (~swSmoothMonoVoice === voice) {
                    ~swSmoothMonoVoice = nil;
                };
            };
        } {
            voice.set(
                \freq, freq,
                \amp, amp,
                \pan, pan,
                \vibratoRate, vibratoRate,
                \vibratoDepth, vibratoDepth,
                \motionDepth, motionDepth,
                \bloom, bloom,
                \lp, lp,
                \glide, glide,
                \crunch, crunch,
                \reverb, reverb,
                \bassClarity, bassClarity,
                \bassClarityHpMax, bassClarityHpMax,
                \bassClarityMudFreq, bassClarityMudFreq,
                \bassClarityMudCutDb, bassClarityMudCutDb
            );
        };

        voice;
    };

    ~swSmoothApplyMonoAmpScale = {
        if (~swSmoothMonoVoice.notNil) {
            ~swSmoothMonoVoice.set(\amp, ~swSmoothScaledAmp.(~swAuduinoParams ? ()));
        };
    };

    ~swSmoothApplyActiveTone = { |params|
        var crunch = (params[\smoothCrunch] ? 0.0).clip(0.0, 0.65);
        var reverb = (params[\smoothReverb] ? 0.14).clip(0.0, 0.8);
        var bloom = (params[\smoothBloom] ? 0.18).clip(0.0, 0.4);
        var lp = (params[\smoothLp] ? 2200).clip(180.0, 7000.0);

        if (~swSmoothVoices.notNil) {
            ~swSmoothVoices = ~swSmoothVoices.select { |voice| voice.notNil };
        };
        if (~swSmoothMonoVoice.notNil) {
            ~swSmoothMonoVoice.set(\crunch, crunch, \reverb, reverb, \bloom, bloom, \lp, lp);
        };
    };

    ~swSmoothApplyMonoParams = { |params, midi, noteIndex=nil|
        var nextParams = params.copy;
        var freq = midi.midicps;
        var clarity = ~swSmoothBassClarityForPitch.(nextParams, nextParams[\smoothPitchNorm] ? 0.0);
        var scaledAmp;

        nextParams[\voiceMode] = \smoothSine;
        nextParams[\smoothFreq] = freq;
        nextParams[\smoothMidi] = midi;
        ~swAuduinoParams = nextParams;
        scaledAmp = ~swSmoothScaledAmp.(nextParams);

        if ((~swLiveOscDebugEnabled ? false) and: { (Main.elapsedTime - (~swSmoothMonoDebugAt ? 0.0)) > 0.5 }) {
            ("Smooth mono note midi=" ++ midi.asString
                ++ " freq=" ++ freq.round(0.01).asString
                ++ " amp=" ++ scaledAmp.round(0.001).asString
                ++ " pitch=" ++ (nextParams[\smoothPitchNorm] ? 0.0).round(0.001).asString
                ++ " pan=" ++ (nextParams[\pan] ? 0.0).round(0.001).asString
                ++ " layerOpen=" ++ (~swSmoothLayerGateOpen ? false).asString
                ++ " bus=" ++ ~swSmoothLayerBusIndex.().asString).postln;
            ~swSmoothMonoDebugAt = Main.elapsedTime;
        };

        ~swSmoothMonoSet.(
            freq: freq,
            amp: scaledAmp,
            pan: nextParams[\pan] ? 0.0,
            vibratoRate: nextParams[\smoothVibratoRate] ? 0.18,
            vibratoDepth: nextParams[\smoothVibratoDepth] ? 0.006,
            motionDepth: nextParams[\smoothMotionDepth] ? 0.06,
            bloom: nextParams[\smoothBloom] ? 0.18,
            lp: nextParams[\smoothLp] ? 2200,
            glide: nextParams[\smoothGlide] ? 0.0,
            crunch: nextParams[\smoothCrunch] ? 0.0,
            reverb: nextParams[\smoothReverb] ? 0.14,
            bassClarity: clarity,
            bassClarityHpMax: nextParams[\smoothBassClarityHpMax] ? 62.0,
            bassClarityMudFreq: nextParams[\smoothBassClarityMudFreq] ? 180.0,
            bassClarityMudCutDb: nextParams[\smoothBassClarityMudCutDb] ? 4.0
        );

        if (noteIndex.notNil) {
            ~swSmoothLastIndex = noteIndex;
        };
        ~swSmoothLastMidi = midi;
    };

    ~swSmoothMonoDirect = { |params|
        var rootMidi = params[\smoothRootMidi] ? 38;
        var notes = ~swSmoothBassMidis.(rootMidi);
        var targetPitchNorm = params[\smoothPitchNorm] ? 0.0;
        var noteIndex = ~swSmoothIndexForPitch.(targetPitchNorm, rootMidi).round.clip(0, notes.size - 1).asInteger;
        var midi = notes[noteIndex];

        ~swSmoothFilteredPitchNorm = targetPitchNorm;
        ~swSmoothApplyMonoParams.(params, midi, noteIndex);
    };

    ~swSmoothMonoTracked = { |params|
        var rootMidi = params[\smoothRootMidi] ? 38;
        var notes = ~swSmoothBassMidis.(rootMidi);
        var targetPitchNorm = params[\smoothPitchNorm] ? 0.0;
        var smoothing = (params[\smoothPitchSmoothing] ? 0.34).clip(0.0, 1.0);
        var hysteresis = (params[\smoothIndexHysteresis] ? 0.22).clip(0.05, 1.0);
        var filteredPitchNorm = ~swSmoothFilteredPitchNorm;
        var targetIndex;
        var currentIndex = ~swSmoothLastIndex;
        var nextIndex;
        var midi;

        if (filteredPitchNorm.isNil) {
            filteredPitchNorm = targetPitchNorm;
        } {
            filteredPitchNorm = filteredPitchNorm + ((targetPitchNorm - filteredPitchNorm) * smoothing);
        };
        ~swSmoothFilteredPitchNorm = filteredPitchNorm;

        targetIndex = ~swSmoothIndexForPitch.(filteredPitchNorm, rootMidi);

        if (currentIndex.isNil) {
            nextIndex = targetIndex.round.clip(0, notes.size - 1).asInteger;
        } {
            nextIndex = currentIndex;
            if (targetIndex >= (currentIndex + hysteresis)) {
                nextIndex = targetIndex.round.clip(0, notes.size - 1).asInteger;
            } {
                if (targetIndex <= (currentIndex - hysteresis)) {
                    nextIndex = targetIndex.round.clip(0, notes.size - 1).asInteger;
                };
            };
        };

        midi = notes[nextIndex.clip(0, notes.size - 1).asInteger];
        ~swSmoothApplyMonoParams.(params, midi, nextIndex);
    };

    ~swSmoothMaybeTrigger = { |params, force=false|
        var rootMidi = params[\smoothRootMidi] ? 38;
        var notes = ~swSmoothBassMidis.(rootMidi);
        var targetPitchNorm = params[\smoothPitchNorm] ? 0.0;
        var smoothing = (params[\smoothPitchSmoothing] ? 0.34).clip(0.0, 1.0);
        var hysteresis = (params[\smoothIndexHysteresis] ? 0.22).clip(0.05, 1.0);
        var useRawTrigger = params[\smoothTriggerUseRawPitch] ? false;
        var filteredPitchNorm = ~swSmoothFilteredPitchNorm;
        var pitchJump = 0.0;
        var triggerPitchNorm;
        var targetIndex;
        var currentIndex = ~swSmoothLastIndex;
        var nextIndex;
        var snapThreshold = 2.5;
        var indexDistance = 0.0;
        var immediateTrigger = force;
        var midi;
        var now = Main.elapsedTime;
        var minInterval = params[\smoothMinInterval] ? 0.07;
        var fastJumpThreshold = (params[\smoothFastJumpThreshold] ? 0.55).clip(0.05, 2.0);
        var fastMinInterval = (params[\smoothFastMinInterval] ? minInterval).clip(0.01, 1.0);
        var fastJumpAmpScale = (params[\smoothFastJumpAmpScale] ? 1.0).clip(0.0, 1.0);
        var fastJumpDecayScale = (params[\smoothFastJumpDecayScale] ? 1.0).clip(0.1, 1.0);
        var rapidPitchJump = false;
        var effectiveMinInterval;
        var snapBypassInterval = params[\smoothSnapBypassInterval] ? true;
        var forcedTrigger = force;
        var intervalReady;
        var shouldTrigger;

        if (filteredPitchNorm.isNil) {
            filteredPitchNorm = targetPitchNorm;
        } {
            pitchJump = (targetPitchNorm - filteredPitchNorm).abs;
            if (pitchJump >= 0.24) {
                filteredPitchNorm = targetPitchNorm;
            } {
                filteredPitchNorm = filteredPitchNorm + ((targetPitchNorm - filteredPitchNorm) * smoothing);
            };
        };
        ~swSmoothFilteredPitchNorm = filteredPitchNorm;
        rapidPitchJump = pitchJump >= fastJumpThreshold;
        effectiveMinInterval = if (rapidPitchJump) { fastMinInterval.max(minInterval) } { minInterval };
        intervalReady = (now - (~swSmoothLastTriggerTime ? 0.0)) >= effectiveMinInterval;

        triggerPitchNorm = if (useRawTrigger) { targetPitchNorm } { filteredPitchNorm };
        targetIndex = ~swSmoothIndexForPitch.(triggerPitchNorm, rootMidi);

        if (currentIndex.isNil) {
            nextIndex = targetIndex.round.clip(0, notes.size - 1).asInteger;
            shouldTrigger = true;
        } {
            nextIndex = currentIndex;
            indexDistance = (targetIndex - currentIndex).abs;
            if (indexDistance >= snapThreshold) {
                nextIndex = targetIndex.round.clip(0, notes.size - 1).asInteger;
                immediateTrigger = true;
            } {
                if (targetIndex >= (currentIndex + hysteresis)) {
                    nextIndex = targetIndex.round.clip(0, notes.size - 1).asInteger;
                } {
                    if (targetIndex <= (currentIndex - hysteresis)) {
                        nextIndex = targetIndex.round.clip(0, notes.size - 1).asInteger;
                    };
                };
            };
            shouldTrigger = immediateTrigger or: { nextIndex != currentIndex };
        };

        midi = notes[nextIndex.clip(0, notes.size - 1).asInteger];

        if (shouldTrigger and: { intervalReady or: { forcedTrigger or: { immediateTrigger and: { snapBypassInterval } } } }) {
            ~swSmoothSpawnNote.(
                freq: midi.midicps,
                amp: ~swSmoothScaledAmp.(params) * (if (rapidPitchJump) { fastJumpAmpScale } { 1.0 }),
                pan: params[\pan] ? 0.0,
                vibratoRate: params[\smoothVibratoRate] ? 0.18,
                vibratoDepth: params[\smoothVibratoDepth] ? 0.006,
                motionDepth: params[\smoothMotionDepth] ? 0.06,
                bloom: params[\smoothBloom] ? 0.18,
                lp: params[\smoothLp] ? 2200,
                hold: params[\smoothHold] ? 0.06,
                decay: (params[\smoothDecay] ? 2.6) * (if (rapidPitchJump) { fastJumpDecayScale } { 1.0 }),
                crunch: params[\smoothCrunch] ? 0.0,
                reverb: params[\smoothReverb] ? 0.14,
                bassClarity: ~swSmoothBassClarityForPitch.(params, targetPitchNorm),
                bassClarityHpMax: params[\smoothBassClarityHpMax] ? 62.0,
                bassClarityMudFreq: params[\smoothBassClarityMudFreq] ? 180.0,
                bassClarityMudCutDb: params[\smoothBassClarityMudCutDb] ? 4.0
            );
            ~swSmoothLastIndex = nextIndex.clip(0, notes.size - 1).asInteger;
            ~swSmoothLastMidi = midi;
            ~swSmoothLastTriggerTime = now;
        };
    };

    ~swSmoothStop = {
        if (~swSmoothVoices.notNil) {
            ~swSmoothVoices.do { |voice|
                if (voice.notNil) {
                    voice.set(\gate, 0);
                };
            };
            ~swSmoothVoices = List.new;
        };
        if (~swSmoothMonoVoice.notNil) {
            ~swSmoothMonoVoice.set(\gate, 0);
            ~swSmoothMonoVoice = nil;
        };
        ~swResetSmoothTracking.();
    };

    ~swAmbientEnsureVariant = { |variant=\flow, motionBus=nil|
        var voices = ~swAmbientVoices ? IdentityDictionary.new;
        var voice = voices[variant];
        var bus = motionBus ? ~swMotionLevelBus;
        var busIndex = 0;

        if (bus.notNil) {
            busIndex = bus.index;
        };

        if (voice.isNil) {
            voice = Synth(~swAmbientDefFor.(variant), [
                \motionBus, busIndex,
                \motionRise, ~swMotionFadeInSeconds,
                \motionFall, ~swMotionFadeOutSeconds
            ]);
            voices[variant] = voice;
            ~swAmbientVoices = voices;
            voice.onFree {
                if (~swAmbientVoices.notNil) {
                    if (~swAmbientVoices[variant] === voice) {
                        ~swAmbientVoices.removeAt(variant);
                    };
                };
            };
        } {
            ~swAmbientVoices = voices;
        };

        voice;
    };

    ~swAmbientSetVariant = { |variant=\flow, rootFreq=74, amp=0.1, pan=0.0, lift=0.0, lean=0.0,
        motion=0.28, spin=0.18, aura=0.62, darkness=0.58, shimmer=0.42, width=0.82, motionBus=nil, motionRise=nil|
        var bus = motionBus ? ~swMotionLevelBus;
        var busIndex = 0;
        var rise = motionRise ? ~swMotionFadeInSeconds;
        var voice;

        if (bus.notNil) {
            busIndex = bus.index;
        };
        voice = ~swAmbientEnsureVariant.(variant, bus);

        voice.set(
            \gate, 1,
            \motionBus, busIndex,
            \motionRise, rise,
            \rootFreq, rootFreq,
            \amp, amp,
            \pan, pan,
            \lift, lift,
            \lean, lean,
            \motion, motion,
            \spin, spin,
            \aura, aura,
            \darkness, darkness,
            \shimmer, shimmer,
            \width, width
        );
    };

    ~swAmbientReleaseVariantsExcept = { |allowedVariants=nil|
        var keep = allowedVariants ? [];
        var voices = ~swAmbientVoices ? IdentityDictionary.new;

        voices.keys.do { |variant|
            if (keep.includes(variant).not) {
                var voice = voices[variant];
                if (voice.notNil) {
                    voice.set(\gate, 0);
                };
                voices.removeAt(variant);
            };
        };

        ~swAmbientVoices = voices;
    };

    ~swAmbientSet = { |variant=\flow, rootFreq=74, amp=0.1, pan=0.0, lift=0.0, lean=0.0,
        motion=0.28, spin=0.18, aura=0.62, darkness=0.58, shimmer=0.42, width=0.82|
        var params = (~swAuduinoParams ? ~swAuduinoDefaultParams.copy).copy;

        params[\voiceMode] = \ambientFlow;
        params[\ambientVariant] = variant;
        params[\ambientRoot] = rootFreq;
        params[\ambientLift] = lift;
        params[\ambientLean] = lean;
        params[\ambientMotion] = motion;
        params[\ambientSpin] = spin;
        params[\ambientAura] = aura;
        params[\ambientDarkness] = darkness;
        params[\ambientShimmer] = shimmer;
        params[\ambientWidth] = width;
        params[\amp] = amp;
        params[\pan] = pan;
        ~swAuduinoParams = params;

        ~swAmbientSetVariant.(
            variant: variant,
            rootFreq: rootFreq,
            amp: amp,
            pan: pan,
            lift: lift,
            lean: lean,
            motion: motion,
            spin: spin,
            aura: aura,
            darkness: darkness,
            shimmer: shimmer,
            width: width
        );
        ~swAmbientReleaseVariantsExcept.([variant]);
    };

    ~swAmbientStop = {
        if (~swAmbientVoices.notNil) {
            ~swAmbientVoices.keysValuesDo { |variant, voice|
                if (voice.notNil) {
                    voice.set(\gate, 0);
                };
            };
            ~swAmbientVoices = IdentityDictionary.new;
        };
        ~swResetAmbientBassTracking.();
    };

    ~swReeseEnsure = {
        if (~swReeseBass.isNil) {
            ~swReeseBass = Synth(\spacewizardReeseBass, [
                \motionBus, ~swMotionLevelBus.index,
                \motionRise, ~swMotionFadeInSeconds,
                \motionFall, ~swMotionFadeOutSeconds
            ]);
            ~swReeseBass.onFree { ~swReeseBass = nil; };
        };
        ~swReeseBass;
    };

    ~swReeseSet = { |freq=50, amp=0.12, pan=0.0, detune=0.028, glide=0.32,
        wobbleRate=0.18, wobbleDepth=0.12, drive=2.8, lp=900, rq=0.24, subMix=0.46|
        var voice = ~swReeseEnsure.();
        var params = (~swAuduinoParams ? ~swAuduinoDefaultParams.copy).copy;

        params[\voiceMode] = \reeseBass;
        params[\reeseFreq] = freq;
        params[\reeseDetune] = detune;
        params[\reeseGlide] = glide;
        params[\reeseWobbleRate] = wobbleRate;
        params[\reeseWobbleDepth] = wobbleDepth;
        params[\reeseDrive] = drive;
        params[\reeseLp] = lp;
        params[\reeseRq] = rq;
        params[\reeseSubMix] = subMix;
        params[\amp] = amp;
        params[\pan] = pan;
        ~swAuduinoParams = params;

        voice.set(
            \gate, 1,
            \freq, freq,
            \amp, amp,
            \pan, pan,
            \detune, detune,
            \glide, glide,
            \wobbleRate, wobbleRate,
            \wobbleDepth, wobbleDepth,
            \drive, drive,
            \lp, lp,
            \rq, rq,
            \subMix, subMix
        );
    };

    ~swReeseStop = {
        if (~swReeseBass.notNil) {
            ~swReeseBass.set(\gate, 0);
            ~swReeseBass = nil;
        };
    };

    ~swWowEnsure = {
        if (~swWowVoice.isNil) {
            ~swWowVoice = Synth(\spacewizardWowVoice, [
                \motionBus, ~swMotionLevelBus.index,
                \motionRise, ~swMotionFadeInSeconds,
                \motionFall, ~swMotionFadeOutSeconds
            ]);
            ~swWowVoice.onFree {
                ~swWowVoice = nil;
                ~swResetWowTracking.();
            };
        };
        ~swWowVoice;
    };

    ~swWowSet = { |freq=110, amp=0.12, pan=0.0, glide=0.28, direction=1,
        vowelDepth=0.82, breath=0.035, lp=2400, tone=0.16, trigger=false|
        var voice = ~swWowEnsure.();
        var params = (~swAuduinoParams ? ~swAuduinoDefaultParams.copy).copy;

        if (trigger) {
            ~swWowTriggerCount = (~swWowTriggerCount ? 0) + 1;
        };

        params[\voiceMode] = \wowVoice;
        params[\wowFreq] = freq;
        params[\wowGlide] = glide;
        params[\wowDirection] = direction;
        params[\wowVowelDepth] = vowelDepth;
        params[\wowBreath] = breath;
        params[\wowLp] = lp;
        params[\wowTone] = tone;
        params[\wowTrig] = ~swWowTriggerCount;
        params[\amp] = amp;
        params[\pan] = pan;
        ~swAuduinoParams = params;

        voice.set(
            \gate, 1,
            \freq, freq,
            \amp, amp,
            \pan, pan,
            \glide, glide,
            \direction, direction,
            \vowelDepth, vowelDepth,
            \breath, breath,
            \lp, lp,
            "tone", tone,
            \wowTrig, ~swWowTriggerCount
        );
    };

    ~swWowStop = {
        if (~swWowVoice.notNil) {
            ~swWowVoice.set(\gate, 0);
            ~swWowVoice = nil;
        };
        ~swResetWowTracking.();
    };

    ~swAuduinoEnsure = {
        if (~swAuduino.isNil) {
            ~swAuduino = Synth(\spacewizardAuduino, [
                \motionBus, ~swMotionLevelBus.index,
                \motionRise, ~swMotionFadeInSeconds,
                \motionFall, ~swMotionFadeOutSeconds
            ]);
            ~swAuduino.onFree { ~swAuduino = nil; };
        };
        ~swAuduino;
    };

    ~swAuduinoStart = { |syncFreq=84, grain1Freq=220, grain2Freq=330, decay1=0.38, decay2=0.24,
        amp=0.14, pan=0.0, drive=2.8, crushBits=7, hp=70, lp=5200, scaleMode=\originalAuduino,
        playMode=\continuous, rootMidi=50, stepIndex=10|
        var voice = ~swAuduinoEnsure.();
        ~swAuduinoParams = (
            syncFreq: syncFreq,
            grain1Freq: grain1Freq,
            grain2Freq: grain2Freq,
            decay1: decay1,
            decay2: decay2,
            amp: amp,
            pan: pan,
            drive: drive,
            crushBits: crushBits,
            hp: hp,
            lp: lp,
            scaleMode: scaleMode,
            playMode: playMode,
            rootMidi: rootMidi,
            stepIndex: stepIndex
        );
        voice.set(
            \gate, 1,
            \syncFreq, syncFreq,
            \grain1Freq, grain1Freq,
            \grain2Freq, grain2Freq,
            \decay1, decay1,
            \decay2, decay2,
            \amp, amp,
            \pan, pan,
            \drive, drive,
            \crushBits, crushBits,
            \hp, hp,
            \lp, lp
        );
    };

    ~swAuduinoSet = { |syncFreq=84, grain1Freq=220, grain2Freq=330, decay1=0.38, decay2=0.24,
        amp=0.14, pan=0.0, drive=2.8, crushBits=7, hp=70, lp=5200, scaleMode=\originalAuduino,
        playMode=\continuous, rootMidi=50, stepIndex=10|
        var voice = ~swAuduinoEnsure.();
        ~swAuduinoParams = (
            syncFreq: syncFreq,
            grain1Freq: grain1Freq,
            grain2Freq: grain2Freq,
            decay1: decay1,
            decay2: decay2,
            amp: amp,
            pan: pan,
            drive: drive,
            crushBits: crushBits,
            hp: hp,
            lp: lp,
            scaleMode: scaleMode,
            playMode: playMode,
            rootMidi: rootMidi,
            stepIndex: stepIndex
        );
        voice.set(
            \syncFreq, syncFreq,
            \grain1Freq, grain1Freq,
            \grain2Freq, grain2Freq,
            \decay1, decay1,
            \decay2, decay2,
            \amp, amp,
            \pan, pan,
            \drive, drive,
            \crushBits, crushBits,
            \hp, hp,
            \lp, lp
        );
    };

    ~swAuduinoStop = {
        if (~swAuduino.notNil) {
            ~swAuduino.set(\gate, 0);
            ~swAuduino = nil;
        };
    };

    ~swAuduinoCancelRoutine = {
        if (~swAuduinoRoutine.notNil) {
            ~swAuduinoRoutine.stop;
            ~swAuduinoRoutine = nil;
        };
    };

    ~swAuduinoStopAll = {
        ~swAuduinoCancelRoutine.();
        ~swAuduinoStop.();
    };

    ~swAuduinoPlayRoutine = { |routine|
        ~swAuduinoCancelRoutine.();
        ~swAuduinoRoutine = routine;
        routine.play(AppClock);
    };

    ~swAuduinoSetMidi = { |midi=50, amp=0.13, pan=0.0, drive=2.9, crushBits=7|
        var freq = midi.midicps;
        ~swAuduinoSet.(
            syncFreq: (freq * 0.36).clip(18, 220),
            grain1Freq: freq,
            grain2Freq: (freq * 1.498),
            decay1: 0.33 + (midi % 5 * 0.05),
            decay2: 0.18 + (midi % 7 * 0.04),
            amp: amp,
            pan: pan,
            drive: drive,
            crushBits: crushBits,
            hp: 65,
            lp: 5600,
            scaleMode: ~swAuduinoParams[\scaleMode] ? \originalAuduino,
            playMode: ~swAuduinoParams[\playMode] ? \continuous,
            rootMidi: ~swAuduinoParams[\rootMidi] ? 50,
            stepIndex: ~swAuduinoParams[\stepIndex] ? 10
        );
    };

    ~swAuduinoSetMelodicMidi = { |midi=50, amp=0.12, pan=0.0, drive=2.5, crushBits=7|
        var freq = midi.midicps;
        ~swAuduinoSet.(
            syncFreq: (freq * 0.16).clip(10, 90),
            grain1Freq: freq,
            grain2Freq: (freq * 1.334),
            decay1: 0.10 + ((midi % 5) * 0.015),
            decay2: 0.06 + ((midi % 7) * 0.012),
            amp: amp,
            pan: pan,
            drive: drive,
            crushBits: crushBits,
            hp: 55,
            lp: 4300,
            scaleMode: ~swAuduinoParams[\scaleMode] ? \originalAuduino,
            playMode: ~swAuduinoParams[\playMode] ? \continuous,
            rootMidi: ~swAuduinoParams[\rootMidi] ? 50,
            stepIndex: ~swAuduinoParams[\stepIndex] ? 10
        );
    };

    ~swAuduinoStepMidi = { |midi=50, amp=0.13, pan=0.0, drive=2.9, crushBits=7,
        noteDur=0.14, gapDur=0.06, melodic=false|
        if (melodic) {
            ~swAuduinoSetMelodicMidi.(
                midi: midi,
                amp: amp,
                pan: pan,
                drive: drive,
                crushBits: crushBits
            );
        } {
            ~swAuduinoSetMidi.(
                midi: midi,
                amp: amp,
                pan: pan,
                drive: drive,
                crushBits: crushBits
            );
        };
        noteDur.wait;
        ~swAuduinoStop.();
        gapDur.wait;
    };

    ~swAuduinoApplyCurrent = {
        var params = ~swAuduinoParams;
        var playMode = params[\playMode] ? \continuous;

        if ((playMode == \stepped) or: { playMode == \melodicStepped }) {
            ~swAuduinoApplySteppedCurrent.();
        } {
            ~swAuduinoSet.(
                syncFreq: params[\syncFreq],
                grain1Freq: params[\grain1Freq],
                grain2Freq: params[\grain2Freq],
                decay1: params[\decay1],
                decay2: params[\decay2],
                amp: params[\amp],
                pan: params[\pan],
                drive: params[\drive],
                crushBits: params[\crushBits],
                hp: params[\hp],
                lp: params[\lp],
                scaleMode: params[\scaleMode] ? \originalAuduino,
                playMode: playMode,
                rootMidi: params[\rootMidi] ? 50,
                stepIndex: params[\stepIndex] ? 10
            );
        };
    };

    ~swSmoothApplyCurrent = {
        var params = ~swAuduinoParams;
        var experiment = params[\smoothExperiment] ? \triggeredNotes;
        var choirSolo = params[\smoothChoirSolo] ? false;

        ~swSmoothStop.();
        if (choirSolo.not) {
            if (experiment == \triggeredNotes) {
                ~swSmoothSet.(
                    freq: params[\smoothFreq] ? 72,
                    amp: params[\amp] ? 0.1,
                    pan: params[\pan] ? 0.0,
                    vibratoRate: params[\smoothVibratoRate] ? 0.18,
                    vibratoDepth: params[\smoothVibratoDepth] ? 0.006,
                    motionDepth: params[\smoothMotionDepth] ? 0.06,
                    bloom: params[\smoothBloom] ? 0.18,
                    lp: params[\smoothLp] ? 2200,
                    hold: params[\smoothHold] ? 0.06,
                    decay: params[\smoothDecay] ? 2.6,
                    crunch: params[\smoothCrunch] ? 0.0,
                    reverb: params[\smoothReverb] ? 0.14
                );
            } {
                ~swSmoothApplyMonoParams.(params, (params[\smoothMidi] ? 43).asInteger, ~swSmoothLastIndex);
            };
            ~swSmoothApplyActiveTone.(params);
            ~swSmoothAmbientApplyCurrent.();
        } {
            ~swAmbientStop.();
        };

        ~swSmoothChoirApplyCurrent.();
    };

    ~swAmbientApplyCurrent = {
        var params = ~swAuduinoParams;

        ~swAmbientSet.(
            variant: params[\ambientVariant] ? \flow,
            rootFreq: params[\ambientRoot] ? 74,
            amp: params[\amp] ? 0.1,
            pan: params[\pan] ? 0.0,
            lift: params[\ambientLift] ? 0.0,
            lean: params[\ambientLean] ? 0.0,
            motion: params[\ambientMotion] ? 0.28,
            spin: params[\ambientSpin] ? 0.18,
            aura: params[\ambientAura] ? 0.62,
            darkness: params[\ambientDarkness] ? 0.58,
            shimmer: params[\ambientShimmer] ? 0.42,
            width: params[\ambientWidth] ? 0.82
        );
    };

    ~swAmbientMapFromMotion = { |params, activity, pitchNorm, rollNorm, spinNorm, variantOverride=nil|
        var mapped = params.copy;
        var variant = variantOverride ? (mapped[\ambientVariant] ? \flow);
        var tiltShape = pitchNorm.abs;
        var rollShape = rollNorm.abs;

        mapped[\ambientRoot] = mapped[\ambientRoot] ? 74;

        if (variant == \chordal) {
            mapped[\ambientLift] = ((pitchNorm * 0.16) - 0.02).clip(-0.2, 0.2);
            mapped[\ambientLean] = rollNorm * 0.72;
            mapped[\ambientMotion] = activity.linlin(0.0, 1.0, 0.06, 0.54);
            mapped[\ambientSpin] = spinNorm.linlin(0.0, 1.0, 0.04, 0.58);
            mapped[\ambientAura] = (0.54 + (tiltShape * 0.24) + (activity * 0.18)).clip(0.0, 1.0);
            mapped[\ambientDarkness] = pitchNorm.linlin(-1.0, 1.0, 0.84, 0.22).clip(0.0, 1.0);
            mapped[\ambientShimmer] = ((spinNorm * 0.52) + (tiltShape * 0.18) + (activity * 0.08)).clip(0.0, 1.0);
            mapped[\ambientWidth] = (0.44 + (rollShape * 0.18) + (tiltShape * 0.18)).clip(0.0, 1.0);
            mapped[\pan] = rollNorm.linlin(-1.0, 1.0, -0.22, 0.22);
            mapped[\amp] = activity.linlin(0.0, 1.0, 0.05, 0.15);
        } {
            if (variant == \drone) {
                mapped[\ambientLift] = ((pitchNorm * 0.1) - 0.08).clip(-0.18, 0.12);
                mapped[\ambientLean] = rollNorm * 0.36;
                mapped[\ambientMotion] = activity.linlin(0.0, 1.0, 0.05, 0.58);
                mapped[\ambientSpin] = spinNorm.linlin(0.0, 1.0, 0.02, 0.24);
                mapped[\ambientAura] = (0.42 + (activity * 0.2) + (tiltShape * 0.16)).clip(0.0, 1.0);
                mapped[\ambientDarkness] = pitchNorm.linlin(-1.0, 1.0, 0.96, 0.46).clip(0.0, 1.0);
                mapped[\ambientShimmer] = ((spinNorm * 0.22) + (activity * 0.08)).clip(0.0, 1.0);
                mapped[\ambientWidth] = (0.28 + (rollShape * 0.16) + (tiltShape * 0.1)).clip(0.0, 1.0);
                mapped[\pan] = rollNorm.linlin(-1.0, 1.0, -0.14, 0.14);
                mapped[\amp] = activity.linlin(0.0, 1.0, 0.05, 0.14);
            } {
                if (variant == \forest) {
                    mapped[\ambientLift] = ((pitchNorm * 0.14) + 0.02).clip(-0.12, 0.2);
                    mapped[\ambientLean] = rollNorm * 0.6;
                    mapped[\ambientMotion] = activity.linlin(0.0, 1.0, 0.16, 0.96);
                    mapped[\ambientSpin] = spinNorm;
                    mapped[\ambientAura] = (0.46 + (pitchNorm.linlin(-1.0, 1.0, 0.0, 0.24)) + (activity * 0.16) + (spinNorm * 0.1)).clip(0.0, 1.0);
                    mapped[\ambientDarkness] = pitchNorm.linlin(-1.0, 1.0, 0.88, 0.24).clip(0.0, 1.0);
                    mapped[\ambientShimmer] = ((spinNorm * 0.72) + (tiltShape * 0.14) + (activity * 0.12)).clip(0.0, 1.0);
                    mapped[\ambientWidth] = (0.62 + (rollShape * 0.18) + (tiltShape * 0.1)).clip(0.0, 1.0);
                    mapped[\pan] = rollNorm.linlin(-1.0, 1.0, -0.28, 0.28);
                    mapped[\amp] = activity.linlin(0.0, 1.0, 0.04, 0.13);
                } {
                    mapped[\ambientLift] = (pitchNorm * 0.12).clip(-0.16, 0.16);
                    mapped[\ambientLean] = rollNorm * 0.46;
                    mapped[\ambientMotion] = activity.linlin(0.0, 1.0, 0.08, 0.92);
                    mapped[\ambientSpin] = spinNorm;
                    mapped[\ambientAura] = (0.38 + (activity * 0.34) + (tiltShape * 0.18) + (spinNorm * 0.08)).clip(0.0, 1.0);
                    mapped[\ambientDarkness] = pitchNorm.linlin(-1.0, 1.0, 0.9, 0.2).clip(0.0, 1.0);
                    mapped[\ambientShimmer] = ((spinNorm * 0.68) + (activity * 0.18)).clip(0.0, 1.0);
                    mapped[\ambientWidth] = (0.48 + (rollShape * 0.26) + (tiltShape * 0.1)).clip(0.0, 1.0);
                    mapped[\pan] = rollNorm.linlin(-1.0, 1.0, -0.26, 0.26);
                    mapped[\amp] = activity.linlin(0.0, 1.0, 0.045, 0.14);
                }
            }
        };

        mapped;
    };

    ~swAmbientApplyMappedParams = { |mappedParams, variantOverride=nil|
        var variant = variantOverride ? (mappedParams[\ambientVariant] ? \flow);

        ~swAmbientSetVariant.(
            variant: variant,
            rootFreq: mappedParams[\ambientRoot] ? 74,
            amp: mappedParams[\amp] ? 0.1,
            pan: mappedParams[\pan] ? 0.0,
            lift: mappedParams[\ambientLift] ? 0.0,
            lean: mappedParams[\ambientLean] ? 0.0,
            motion: mappedParams[\ambientMotion] ? 0.28,
            spin: mappedParams[\ambientSpin] ? 0.18,
            aura: mappedParams[\ambientAura] ? 0.62,
            darkness: mappedParams[\ambientDarkness] ? 0.58,
            shimmer: mappedParams[\ambientShimmer] ? 0.42,
            width: mappedParams[\ambientWidth] ? 0.82,
            motionBus: mappedParams[\motionBus],
            motionRise: mappedParams[\motionRise]
        );
    };

    ~swAmbientLabDefFor = { |engine=\halo|
        if (engine == \airports) {
            \spacewizardAmbientLabAirports
        } {
            if (engine == \granular) {
                \spacewizardAmbientLabGranular
            } {
                if (engine == \hexBells) {
                    \spacewizardAmbientLabHexBells
                } {
                    if (engine == \tapeSwells) {
                        \spacewizardAmbientLabTapeSwells
                    } {
                        if (engine == \tapeScript) {
                            \spacewizardAmbientLabTapeScript
                        } {
                            \spacewizardAmbientLabHalo
                        }
                    }
                }
            }
        };
    };

    ~swAmbientLabStopVoiceOnly = {
        if (~swAmbientLabVoice.notNil) {
            ~swAmbientLabVoice.set(\gate, 0);
            ~swAmbientLabVoice = nil;
            ~swAmbientLabVoiceEngine = nil;
        };
    };

    ~swAmbientLabResetAudioState = {
        ~swAmbientLabStopVoiceOnly.();
        ~swAmbientLabStopOverlay.();
        ~swAmbientLabStopPentatonicOverlay.();
        ~swAmbientStop.();
        ~swAmbientLabAppliedEngine = nil;
    };

    ~swAmbientLabStopOverlay = {
        if (~swAmbientLabOverlay.notNil) {
            ~swAmbientLabOverlay.set(\gate, 0);
            ~swAmbientLabOverlay = nil;
        };
        ~swAmbientLabOverlayActive = false;
    };

    ~swAmbientLabStopPentatonicOverlay = {
        if (~swAmbientLabPentatonic.notNil) {
            ~swAmbientLabPentatonic.set(\gate, 0);
            ~swAmbientLabPentatonic = nil;
        };
        if (~swAmbientLabPentatonicVoices.notNil) {
            ~swAmbientLabPentatonicVoices.do { |voice|
                if (voice.notNil) {
                    voice.set(\gate, 0);
                };
            };
            ~swAmbientLabPentatonicVoices = List.new;
        };
        ~swAmbientLabPentatonicLastIndex = nil;
        ~swAmbientLabPentatonicFilteredPitchNorm = nil;
        ~swAmbientLabPentatonicLastTriggerTime = 0.0;
    };

    ~swAmbientLabStop = {
        ~swAmbientLabState = (~swAmbientLabState ? ~swAmbientLabDefaultState.()).copy.put(\isPlaying, false);
        ~swAmbientLabResetAudioState.();
    };

    ~swAmbientLabEnsureVoice = { |engine=\halo|
        var voice = ~swAmbientLabVoice;

        if (engine == \hybrid) {
            ~swAmbientLabStopVoiceOnly.();
            nil
        } {
            if (voice.notNil and: { ~swAmbientLabVoiceEngine != engine }) {
                ~swAmbientLabStopVoiceOnly.();
                voice = nil;
            };
            if (voice.isNil) {
                voice = Synth(~swAmbientLabDefFor.(engine), [
                    \motionBus, ~swMotionLevelBus.index,
                    \motionRise, ~swMotionFadeInSeconds,
                    \motionFall, ~swMotionFadeOutSeconds,
                    \buf, ~swChoirBufNumAt.(0),
                    \basePitchHz, ~swChoirSamplePitchHz
                ]);
                ~swAmbientLabVoice = voice;
                ~swAmbientLabVoiceEngine = engine;
                voice.onFree {
                    if (~swAmbientLabVoice === voice) {
                        ~swAmbientLabVoice = nil;
                        ~swAmbientLabVoiceEngine = nil;
                    };
                };
            };
            voice;
        };
    };

    ~swAmbientLabCopyState = { |state|
        var source = state ? ~swAmbientLabDefaultState.();
        var copy = source.copy;
        var mappings = ();

        (source[\mappings] ? ()).keysValuesDo { |axis, mapping|
            var mappingCopy = (mapping ? ~swAmbientLabMapping.()).copy;
            if ((mappingCopy[\target] ? \none) == \pan) {
                mappingCopy[\target] = \width;
            };
            mappings[axis] = mappingCopy;
        };
        copy[\mappings] = mappings;
        copy;
    };

    ~swAmbientLabAxisIsBipolar = { |axis|
        (axis == \pitch) or: { axis == \roll };
    };

    ~swAmbientLabShapedValue = { |axis, rawValue=0.0, curve=\linear, invert=false|
        var bipolar = ~swAmbientLabAxisIsBipolar.(axis);
        var value = rawValue.asFloat;
        var sign = 1.0;
        var mag;
        var shaped;

        if (bipolar) {
            value = value.clip(-1.0, 1.0);
            if (invert) {
                value = value.neg;
            };
            sign = value.sign;
            mag = value.abs;
        } {
            value = value.clip(0.0, 1.0);
            if (invert) {
                value = 1.0 - value;
            };
            mag = value;
        };

        if (curve == \soft) {
            mag = mag.squared;
        } {
            if (curve == \strong) {
                mag = mag.sqrt;
            };
        };

        shaped = if (bipolar) { sign * mag } { mag };
        shaped;
    };

    ~swAmbientLabApplyMappingTarget = { |mapped, target=\none, axis=\yaw, shaped=0.0, amount=0.0|
        var bipolar = ~swAmbientLabAxisIsBipolar.(axis);
        var signed = if (bipolar) { shaped.clip(-1.0, 1.0) } { shaped.linlin(0.0, 1.0, -1.0, 1.0) };
        var uni = if (bipolar) { shaped.abs.clip(0.0, 1.0) } { shaped.clip(0.0, 1.0) };
        var sceneInput = if (bipolar) { shaped.linlin(-1.0, 1.0, 0.0, 1.0) } { shaped.clip(0.0, 1.0) };
        var amt = amount.clip(0.0, 1.0);

        if (target == \sceneMorph) {
            mapped[\sceneMorph] = (((mapped[\sceneMorph] ? 0.0) * (1.0 - amt)) + (sceneInput * amt)).clip(0.0, 1.0);
            mapped[\grainScan] = (((mapped[\grainScan] ? 0.35) * (1.0 - (amt * 0.85))) + (sceneInput * amt * 0.85)).clip(0.0, 1.0);
            mapped[\brightness] = ((mapped[\brightness] ? 0.5) + (signed * amt * 0.16)).clip(0.18, 0.82);
            mapped[\darkness] = ((mapped[\darkness] ? 0.5) - (signed * amt * 0.13)).clip(0.22, 0.82);
            mapped[\shimmer] = ((mapped[\shimmer] ? 0.2) + (uni * amt * 0.12)).clip(0.0, 0.6);
            mapped[\width] = ((mapped[\width] ? 0.7) + (uni * amt * 0.08)).clip(0.35, 0.96);
        } {
            if (target == \harmonyLift) {
                mapped[\lift] = ((mapped[\lift] ? 0.0) + (signed * amt * 0.34)).clip(-0.62, 0.62);
            } {
                if (target == \brightness) {
                    mapped[\brightness] = ((mapped[\brightness] ? 0.5) + (signed * amt * 0.38)).clip(0.18, 0.82);
                    mapped[\darkness] = ((mapped[\darkness] ? 0.5) - (signed * amt * 0.3)).clip(0.22, 0.82);
                    mapped[\lift] = ((mapped[\lift] ? 0.0) + (signed * amt * 0.24)).clip(-0.62, 0.62);
                    mapped[\ambience] = ((mapped[\ambience] ? 0.74) + (signed * amt * 0.08)).clip(0.0, 0.9);
                } {
                    if (target == \darkness) {
                        mapped[\darkness] = ((mapped[\darkness] ? 0.5) + (signed * amt * 0.28)).clip(0.22, 0.82);
                        mapped[\brightness] = ((mapped[\brightness] ? 0.5) - (signed * amt * 0.14)).clip(0.18, 0.82);
                        mapped[\lift] = ((mapped[\lift] ? 0.0) - (signed * amt * 0.08)).clip(-0.62, 0.62);
                    } {
                        if (target == \density) {
                            mapped[\density] = ((mapped[\density] ? 0.3) + (uni * amt * 0.32)).clip(0.0, 0.68);
                        } {
                            if (target == \shimmer) {
                                mapped[\shimmer] = ((mapped[\shimmer] ? 0.2) + (uni * amt * 0.3)).clip(0.0, 0.6);
                            } {
                                if (target == \width) {
                                    mapped[\width] = ((mapped[\width] ? 0.7) + (uni * amt * 0.42)).clip(0.35, 0.96);
                                    mapped[\grainScan] = ((mapped[\grainScan] ? 0.35) + (signed * amt * 0.48)).clip(0.0, 1.0);
                                    mapped[\sceneMorph] = ((mapped[\sceneMorph] ? 0.0) + (signed * amt * 0.2)).clip(0.0, 1.0);
                                    mapped[\lift] = ((mapped[\lift] ? 0.0) + (signed * amt * 0.16)).clip(-0.62, 0.62);
                                    mapped[\shimmer] = ((mapped[\shimmer] ? 0.2) + (uni * amt * 0.14)).clip(0.0, 0.6);
                                    mapped[\density] = ((mapped[\density] ? 0.3) + (uni * amt * 0.1)).clip(0.0, 0.68);
                                } {
                                    if (target == \pan) {
                                        mapped[\pan] = ((mapped[\pan] ? 0.0) + (signed * amt * 0.08)).clip(-0.12, 0.12);
                                    } {
                                        if (target == \grainScan) {
                                            mapped[\grainScan] = (((mapped[\grainScan] ? 0.35) * (1.0 - amt)) + (sceneInput * amt)).clip(0.0, 1.0);
                                            mapped[\sceneMorph] = (((mapped[\sceneMorph] ? 0.0) * (1.0 - (amt * 0.35))) + (sceneInput * amt * 0.35)).clip(0.0, 1.0);
                                        } {
                                            if (target == \rhythmDensity) {
                                                mapped[\rhythm] = ((mapped[\rhythm] ? 0.2) + (uni * amt * 0.34)).clip(0.0, 0.58);
                                                mapped[\density] = ((mapped[\density] ? 0.3) + (uni * amt * 0.1)).clip(0.0, 0.68);
                                            } {
                                                if (target == \overlayMix) {
                                                    mapped[\overlay] = ((mapped[\overlay] ? 0.0) + (uni * amt * 0.35)).clip(0.0, 0.55);
                                                };
                                            };
                                        };
                                    };
                                };
                            };
                        };
                    };
                };
            };
        };

        mapped;
    };

    ~swAmbientLabMappedForState = { |state|
        var source = state ? ~swAmbientLabState ? ~swAmbientLabDefaultState.();
        var gate = (source[\gate] ? 1.0).clip(0.0, 1.0);
        var activity = (source[\activity] ? 0.24).clip(0.0, 1.0);
        var spin = (source[\spin] ? 0.12).clip(0.0, 1.0);
        var pitch = (source[\pitch] ? 0.0).clip(-1.0, 1.0);
        var yaw = (source[\yaw] ? 0.0).clip(0.0, 1.0);
        var mappings = source[\mappings] ? ~swAmbientLabDefaultMappings.();
        var mapped = (
            engine: source[\engine] ? \halo,
            gate: gate,
            yaw: yaw,
            pitch: pitch,
            roll: (source[\roll] ? 0.0).clip(-1.0, 1.0),
            activity: activity,
            spin: spin,
            rootFreq: (source[\root] ? 58.0).clip(36.0, 130.0),
            amp: (((source[\level] ? 0.12).clip(0.0, 0.2) * 3.4) * gate).clip(0.0, 0.5),
            pan: 0.0,
            lift: 0.0,
            brightness: 0.48,
            darkness: 0.54,
            density: 0.14,
            shimmer: 0.12,
            width: 0.7,
            grainScan: 0.35,
            rhythm: (source[\rhythm] ? 0.14).clip(0.0, 0.58),
            ambience: (source[\ambience] ? 0.74).clip(0.0, 0.9),
            overlay: (source[\overlay] ? 0.0).clip(0.0, 0.55),
            sceneMorph: 0.0,
            pentatonicEnabled: source[\pentatonicEnabled] ? false,
            pentatonicLevel: (source[\pentatonicLevel] ? 0.1).clip(0.0, 0.24),
            pentatonicRootMidi: (source[\pentatonicRootMidi] ? 38).clip(30, 44),
            yawOverlayEnabled: source[\yawOverlayEnabled] ? true,
            yawOverlayStart: source[\yawOverlayStart] ? 0.68,
            yawOverlayEnd: source[\yawOverlayEnd] ? 0.86,
            yawOverlayHysteresis: source[\yawOverlayHysteresis] ? 0.04
        );

        mappings.keysValuesDo { |axis, mapping|
            var rawValue = source[axis] ? 0.0;
            var shaped = ~swAmbientLabShapedValue.(
                axis,
                rawValue,
                mapping[\curve] ? \linear,
                mapping[\invert] ? false
            );
            mapped = ~swAmbientLabApplyMappingTarget.(
                mapped,
                mapping[\target] ? \none,
                axis,
                shaped,
                mapping[\amount] ? 0.0
            );
        };

        mapped[\brightness] = (mapped[\brightness] ? 0.5).clip(0.18, 0.82);
        mapped[\darkness] = (mapped[\darkness] ? 0.5).clip(0.22, 0.82);
        mapped[\density] = (mapped[\density] ? 0.3).clip(0.0, 0.68);
        mapped[\shimmer] = (mapped[\shimmer] ? 0.2).clip(0.0, 0.6);
        mapped[\width] = (mapped[\width] ? 0.7).clip(0.35, 0.96);
        mapped[\pan] = (mapped[\pan] ? 0.0).clip(-0.12, 0.12);
        mapped[\grainScan] = (mapped[\grainScan] ? 0.35).clip(0.0, 1.0);
        mapped[\rhythm] = (mapped[\rhythm] ? 0.14).clip(0.0, 0.58);
        mapped[\overlay] = (mapped[\overlay] ? 0.0).clip(0.0, 0.55);
        mapped[\sceneMorph] = (mapped[\sceneMorph] ? 0.0).clip(0.0, 1.0);
        mapped;
    };

    ~swAmbientLabApplyOverlay = { |mapped|
        var enabled = mapped[\yawOverlayEnabled] ? true;
        var yaw = (mapped[\yaw] ? 0.0).clip(0.0, 1.0);
        var start = (mapped[\yawOverlayStart] ? 0.68).clip(0.0, 1.0);
        var finish = (mapped[\yawOverlayEnd] ? 0.86).clip(0.0, 1.0);
        var hysteresis = (mapped[\yawOverlayHysteresis] ? 0.04).clip(0.0, 0.2);
        var overlayLevel = (mapped[\overlay] ? 0.0).clip(0.0, 1.0);
        var gate = (mapped[\gate] ? 1.0).clip(0.0, 1.0);
        var wasActive = ~swAmbientLabOverlayActive ? false;
        var inWindow = (yaw >= start) and: { yaw <= finish };
        var heldWindow = (yaw >= (start - hysteresis).clip(0.0, 1.0)) and: { yaw <= (finish + hysteresis).clip(0.0, 1.0) };
        var shouldPlay = enabled and: { overlayLevel > 0.001 } and: { gate > 0.001 } and: { wasActive.if(heldWindow, inWindow) };
        var amp;
        var root;

        if (shouldPlay) {
            if (~swAmbientLabOverlay.isNil) {
                ~swAmbientLabOverlay = Synth(\spacewizardAmbientLabVowelOverlay, [
                    \motionBus, ~swMotionLevelBus.index,
                    \motionRise, ~swMotionFadeInSeconds,
                    \motionFall, ~swMotionFadeOutSeconds
                ]);
                ~swAmbientLabOverlay.onFree {
                    ~swAmbientLabOverlay = nil;
                    ~swAmbientLabOverlayActive = false;
                };
            };
            amp = (overlayLevel * 0.045 * gate).clip(0.0, 0.08);
            root = ((mapped[\rootFreq] ? 74.0) * 1.5).clip(72.0, 360.0);
            ~swAmbientLabOverlay.set(
                \gate, 1,
                \rootFreq, root,
                \amp, amp,
                \pan, 0.0,
                \brightness, mapped[\brightness] ? 0.55,
                \shimmer, mapped[\shimmer] ? 0.3,
                \width, mapped[\width] ? 0.7
            );
            ~swAmbientLabOverlayActive = true;
        } {
            ~swAmbientLabStopOverlay.();
        };
    };

    ~swAmbientLabApplyPentatonicOverlay = { |mapped|
        var enabled = mapped[\pentatonicEnabled] ? false;
        var gate = (mapped[\gate] ? 1.0).clip(0.0, 1.0);
        var level = (mapped[\pentatonicLevel] ? 0.1).clip(0.0, 0.24);
        var shouldPlay = enabled and: { level > 0.001 } and: { gate > 0.001 };

        if (shouldPlay) {
            var rootMidi = (mapped[\pentatonicRootMidi] ? 38).clip(30, 44).asInteger;
            var notes = ~swSmoothBassMidis.(rootMidi);
            var targetPitchNorm = (mapped[\pitch] ? 0.0).clip(-1.0, 1.0);
            var roll = (mapped[\roll] ? 0.0).clip(-1.0, 1.0);
            var rollShape = roll.abs;
            var yawCentered = (mapped[\yaw] ? 0.34).clip(0.0, 1.0).linlin(0.0, 1.0, -1.0, 1.0);
            var yawShape = yawCentered.abs;
            var activity = (mapped[\activity] ? 0.24).clip(0.0, 1.0);
            var spin = (mapped[\spin] ? 0.12).clip(0.0, 1.0);
            var smoothing = 0.34;
            var hysteresis = 0.22;
            var filteredPitchNorm = ~swAmbientLabPentatonicFilteredPitchNorm;
            var pitchJump = 0.0;
            var targetIndex;
            var currentIndex = ~swAmbientLabPentatonicLastIndex;
            var nextIndex;
            var snapThreshold = 2.5;
            var indexDistance = 0.0;
            var immediateTrigger = false;
            var now = Main.elapsedTime;
            var intervalReady = (now - (~swAmbientLabPentatonicLastTriggerTime ? 0.0)) >= 0.07;
            var shouldTrigger;
            var midi;

            if (filteredPitchNorm.isNil) {
                filteredPitchNorm = targetPitchNorm;
            } {
                pitchJump = (targetPitchNorm - filteredPitchNorm).abs;
                if (pitchJump >= 0.24) {
                    filteredPitchNorm = targetPitchNorm;
                } {
                    filteredPitchNorm = filteredPitchNorm + ((targetPitchNorm - filteredPitchNorm) * smoothing);
                };
            };
            ~swAmbientLabPentatonicFilteredPitchNorm = filteredPitchNorm;

            targetIndex = ~swSmoothIndexForPitch.(filteredPitchNorm, rootMidi);

            if (currentIndex.isNil) {
                nextIndex = targetIndex.round.clip(0, notes.size - 1).asInteger;
                shouldTrigger = true;
            } {
                nextIndex = currentIndex;
                indexDistance = (targetIndex - currentIndex).abs;
                if (indexDistance >= snapThreshold) {
                    nextIndex = targetIndex.round.clip(0, notes.size - 1).asInteger;
                    immediateTrigger = true;
                } {
                    if (targetIndex >= (currentIndex + hysteresis)) {
                        nextIndex = targetIndex.round.clip(0, notes.size - 1).asInteger;
                    } {
                        if (targetIndex <= (currentIndex - hysteresis)) {
                            nextIndex = targetIndex.round.clip(0, notes.size - 1).asInteger;
                        };
                    };
                };
                shouldTrigger = immediateTrigger or: { nextIndex != currentIndex };
            };

            midi = notes[nextIndex.clip(0, notes.size - 1).asInteger];

            if (shouldTrigger and: { immediateTrigger or: intervalReady }) {
                var voices = ~swAmbientLabPentatonicVoices ? List.new;
                var tailBudget = 8;
                var activeCount = voices.size.asFloat;
                var density = activeCount / 6.0;
                var ampScale = density.linlin(0.0, 1.0, 1.0, 0.58).clip(0.32, 1.0);
                var holdScale = density.linlin(0.0, 1.0, 1.0, 0.86).clip(0.72, 1.0);
                var decayScale = density.linlin(0.0, 1.0, 1.0, 0.72).clip(0.42, 1.0);
                var bloomScale = density.linlin(0.0, 1.0, 1.0, 0.82).clip(0.7, 1.0);
                var lpScale = density.linlin(0.0, 1.0, 1.0, 0.88).clip(0.76, 1.0);
                var stolenVoice;
                var voice;
                var amp = (level * gate * (0.88 + (activity * 0.22)) * ampScale).clip(0.0, 0.24);
                var lp = (roll.linexp(-1.0, 1.0, 480.0, 2600.0) * lpScale).clip(180.0, 7000.0);
                var bloom = ((rollShape.linlin(0.0, 1.0, 0.07, 0.18) + (yawShape * 0.08)) * bloomScale).clip(0.0, 0.4);
                var tone = ~swSmoothRollToneFor.(~swAuduinoParams ? ~swAuduinoDefaultParams.copy, rollShape);
                var crunch = tone[\crunch];
                var reverb = tone[\reverb];
                var hold = ((0.06) * holdScale).max(0.02);
                var decay = ((2.6) * decayScale).max(0.2);

                while { voices.size >= tailBudget } {
                    stolenVoice = voices.removeAt(0);
                    if (stolenVoice.notNil) {
                        stolenVoice.set(\gate, 0);
                    };
                };

                voice = Synth(\spacewizardSmoothSine, [
                    \freq, midi.midicps,
                    \amp, amp,
                    \pan, (((roll * 0.72) + (yawCentered * 0.28)).clip(-1.0, 1.0)).linlin(-1.0, 1.0, -0.26, 0.26),
                    \vibratoRate, 0.18,
                    \vibratoDepth, 0.006,
                    \motionDepth, 0.06,
                    \bloom, bloom,
                    \lp, lp,
                    \hold, hold,
                    \decay, decay,
                    \crunch, crunch,
                    \reverb, reverb,
                    \motionBus, ~swMotionLevelBus.index,
                    \motionRise, ~swMotionFadeInSeconds,
                    \motionFall, ~swMotionFadeOutSeconds
                ]);
                voices.add(voice);
                ~swAmbientLabPentatonicVoices = voices;
                voice.onFree {
                    if (~swAmbientLabPentatonicVoices.notNil) {
                        ~swAmbientLabPentatonicVoices.remove(voice);
                    };
                };

                ~swAmbientLabPentatonicLastIndex = nextIndex.clip(0, notes.size - 1).asInteger;
                ~swAmbientLabPentatonicLastTriggerTime = now;
            };
        } {
            ~swAmbientLabStopPentatonicOverlay.();
        };
    };

    ~swAmbientLabApplyMapped = { |mapped|
        var engine = mapped[\engine] ? \halo;
        var voice;
        var blendState;
        var variantsToKeep;

        if (~swAmbientLabAppliedEngine.notNil and: { ~swAmbientLabAppliedEngine != engine }) {
            ~swAmbientLabResetAudioState.();
        };
        ~swAmbientLabAppliedEngine = engine;

        if (engine == \hybrid) {
            ~swAmbientLabStopVoiceOnly.();
            blendState = ~swAmbientBlendStateFor.(mapped[\sceneMorph] ? 0.0);
            variantsToKeep = List.new;
            variantsToKeep.add(blendState[\primaryVariant]);
            if (blendState[\secondaryVariant] != blendState[\primaryVariant]) {
                variantsToKeep.add(blendState[\secondaryVariant]);
            };
            variantsToKeep.do { |variant|
                var weight = 0.0;
                var ambient = (
                    ambientRoot: mapped[\rootFreq] ? 74.0,
                    ambientLift: mapped[\lift] ? 0.0,
                    ambientLean: (((mapped[\grainScan] ? 0.35) - 0.35) * 1.5 + ((mapped[\lift] ? 0.0) * 0.5)).clip(-0.8, 0.8),
                    ambientMotion: ((mapped[\density] ? 0.3) + (((mapped[\grainScan] ? 0.35) - 0.35).abs * 0.2) + (((mapped[\brightness] ? 0.5) - 0.5).abs * 0.1)).clip(0.0, 1.0),
                    ambientSpin: ((mapped[\rhythm] ? 0.2) + ((mapped[\shimmer] ? 0.2) * 0.4) + (((mapped[\grainScan] ? 0.35) - 0.35).abs * 0.12) + (((mapped[\brightness] ? 0.5) - 0.5).max(0.0) * 0.18)).clip(0.0, 1.0),
                    ambientAura: ((mapped[\ambience] ? 0.76) + (((mapped[\brightness] ? 0.5) - 0.5) * 0.22)).clip(0.0, 1.0),
                    ambientDarkness: ((mapped[\darkness] ? 0.5) - (((mapped[\grainScan] ? 0.35) - 0.35) * 0.16)).clip(0.0, 1.0),
                    ambientShimmer: ((mapped[\shimmer] ? 0.2) + (((mapped[\grainScan] ? 0.35) - 0.35).abs * 0.14) + (((mapped[\brightness] ? 0.5) - 0.5).max(0.0) * 0.22)).clip(0.0, 1.0),
                    ambientWidth: mapped[\width] ? 0.7,
                    pan: 0.0,
                    amp: mapped[\amp] ? 0.1
                );

                if (variant == blendState[\primaryVariant]) {
                    weight = weight + (blendState[\primaryWeight] ? 0.0);
                };
                if (variant == blendState[\secondaryVariant]) {
                    weight = weight + (blendState[\secondaryWeight] ? 0.0);
                };
                ambient[\amp] = (ambient[\amp] * weight).clip(0.0, 0.22);
                ~swAmbientApplyMappedParams.(ambient, variant);
            };
            ~swAmbientReleaseVariantsExcept.(variantsToKeep.asArray);
        } {
            ~swAmbientStop.();
            voice = ~swAmbientLabEnsureVoice.(engine);
            if (voice.notNil) {
                voice.set(
                    \gate, 1,
                    \rootFreq, mapped[\rootFreq] ? 74.0,
                    \amp, mapped[\amp] ? 0.1,
                    \pan, mapped[\pan] ? 0.0,
                    \lift, mapped[\lift] ? 0.0,
                    \brightness, mapped[\brightness] ? 0.55,
                    \darkness, mapped[\darkness] ? 0.45,
                    \density, mapped[\density] ? 0.35,
                    \shimmer, mapped[\shimmer] ? 0.35,
                    \width, mapped[\width] ? 0.82,
                    \grainScan, mapped[\grainScan] ? 0.35,
                    \rhythm, mapped[\rhythm] ? 0.22,
                    \ambience, mapped[\ambience] ? 0.72,
                    \overlay, mapped[\overlay] ? 0.0,
                    \sceneMorph, mapped[\sceneMorph] ? 0.0,
                    \buf, ~swChoirBufNumAt.(0),
                    \basePitchHz, ~swChoirSamplePitchHz
                );
            };
        };

        ~swAmbientLabApplyOverlay.(mapped);
        ~swAmbientLabApplyPentatonicOverlay.(mapped);
        mapped;
    };

    ~swAmbientLabApplyMotion = { |yaw=nil, pitch=nil, roll=nil, activity=nil, spin=nil, gate=nil, sourceState=nil|
        var useGlobal = sourceState.isNil;
        var state = ~swAmbientLabCopyState.(sourceState ? ~swAmbientLabState ? ~swAmbientLabDefaultState.());
        var mapped;

        if (yaw.notNil) { state[\yaw] = yaw.clip(0.0, 1.0); };
        if (pitch.notNil) { state[\pitch] = pitch.clip(-1.0, 1.0); };
        if (roll.notNil) { state[\roll] = roll.clip(-1.0, 1.0); };
        if (activity.notNil) { state[\activity] = activity.clip(0.0, 1.0); };
        if (spin.notNil) { state[\spin] = spin.clip(0.0, 1.0); };
        if (gate.notNil) { state[\gate] = gate.clip(0.0, 1.0); };
        state[\isPlaying] = true;
        mapped = ~swAmbientLabMappedForState.(state);
        state[\lastMapped] = mapped.copy;
        if (useGlobal) {
            ~swAmbientLabState = state;
        };
        ~swAmbientLabApplyMapped.(mapped);
    };

    ~swAmbientLabApplyCurrent = {
        ~swAmbientLabApplyMotion.(sourceState: ~swAmbientLabState);
    };

    ~swAmbientLabStateForSmoothParams = { |params, gate=1.0, activity=0.0, pitchNorm=0.0, rollNorm=0.0, spinNorm=0.0, yawNorm=0.0|
        var saved = params[\smoothAmbientLabState] ? ~swAmbientLabState ? ~swAmbientLabDefaultState.();
        var state = ~swAmbientLabCopyState.(saved);

        state[\gate] = gate.clip(0.0, 1.0);
        state[\activity] = activity.clip(0.0, 1.0);
        state[\pitch] = pitchNorm.clip(-1.0, 1.0);
        state[\roll] = rollNorm.clip(-1.0, 1.0);
        state[\spin] = spinNorm.clip(0.0, 1.0);
        state[\yaw] = yawNorm.clip(0.0, 1.0);
        state[\isPlaying] = true;
        state;
    };

    ~swAmbientLabApplySmoothMotion = { |params, activity=0.0, pitchNorm=0.0, rollNorm=0.0, spinNorm=0.0, yawNorm=0.0, gate=1.0|
        var state = ~swAmbientLabStateForSmoothParams.(params, gate, activity, pitchNorm, rollNorm, spinNorm, yawNorm);
        var mapped = ~swAmbientLabMappedForState.(state);
        var overlay = params.copy.putAll((
            ambientLabEngine: mapped[\engine],
            ambientLabRoot: mapped[\rootFreq],
            ambientLabBrightness: mapped[\brightness],
            ambientLabDarkness: mapped[\darkness],
            ambientLabDensity: mapped[\density],
            ambientLabShimmer: mapped[\shimmer],
            ambientLabWidth: mapped[\width],
            ambientLabRhythm: mapped[\rhythm],
            ambientLabOverlay: mapped[\overlay],
            ambientBlendPosition: mapped[\sceneMorph]
        ));

        ~swAmbientLabApplyMapped.(mapped);
        overlay;
    };

    ~swAmbientLabPromoteToSmoothLive = {
        var state = ~swAmbientLabCopyState.(~swAmbientLabState ? ~swAmbientLabDefaultState.());
        var params = (~swAuduinoParams ? ~swAuduinoDefaultParams.copy).copy;

        state[\isPlaying] = true;
        params[\smoothAmbientLabEnabled] = true;
        params[\smoothAmbientLabState] = state;
        params[\smoothAmbientEnabled] = true;
        ~swAuduinoParams = params;
        "Ambient lab mapping promoted to smooth live ambient. Use smoothSine mode to hear it from candle motion.".postln;
        ~swRefreshLiveStatusGUI.();
    };

    ~swAmbientLabScene = {
        var state = ~swAmbientLabCopyState.(~swAmbientLabState ? ~swAmbientLabDefaultState.());

        ~swSetMotionGate.(true);
        state[\isPlaying] = true;
        ~swAmbientLabState = state;
        ~swAmbientLabApplyCurrent.();
    };

    ~swAmbientLabAnalysisDb = { |amp=0.0|
        amp.asFloat.max(0.0000001).ampdb;
    };

    ~swAmbientLabAnalysisMean = { |records, key|
        var total = 0.0;
        var count = records.size;

        if (count <= 0) {
            ^0.0;
        };
        records.do { |record|
            total = total + ((record[key] ? 0.0).asFloat);
        };
        total / count;
    };

    ~swAmbientLabAnalysisMax = { |records, key|
        var value = 0.0;

        records.do { |record|
            value = value.max((record[key] ? 0.0).asFloat);
        };
        value;
    };

    ~swAmbientLabAnalysisRange = { |records, key|
        var minValue = nil;
        var maxValue = nil;

        records.do { |record|
            var value = (record[key] ? 0.0).asFloat;
            if (minValue.isNil) {
                minValue = value;
            } {
                minValue = minValue.min(value);
            };
            if (maxValue.isNil) {
                maxValue = value;
            } {
                maxValue = maxValue.max(value);
            };
        };
        if (minValue.isNil) { 0.0 } { maxValue - minValue };
    };

    ~swAmbientLabAnalysisContrast = { |lowRecords, highRecords|
        var lowRms = ~swAmbientLabAnalysisDb.(~swAmbientLabAnalysisMean.(lowRecords, \rms));
        var highRms = ~swAmbientLabAnalysisDb.(~swAmbientLabAnalysisMean.(highRecords, \rms));
        var rmsDelta = (lowRms - highRms).abs / 24.0;
        var hfDelta = (~swAmbientLabAnalysisMean.(lowRecords, \hfRatio) - ~swAmbientLabAnalysisMean.(highRecords, \hfRatio)).abs * 1.4;
        var centroidDelta = (~swAmbientLabAnalysisMean.(lowRecords, \centroid) - ~swAmbientLabAnalysisMean.(highRecords, \centroid)).abs * 1.8;
        var widthDelta = (~swAmbientLabAnalysisMean.(lowRecords, \width) - ~swAmbientLabAnalysisMean.(highRecords, \width)).abs * 0.7;
        var zcrDelta = (~swAmbientLabAnalysisMean.(lowRecords, \zcr) - ~swAmbientLabAnalysisMean.(highRecords, \zcr)).abs * 1.0;

        (rmsDelta + hfDelta + centroidDelta + widthDelta + zcrDelta).clip(0.0, 9.0);
    };

    ~swAmbientLabAnalysisAxisDelta = { |records, stepLabel, axis|
        var stepRecords = records.select { |record| (record[\step] == stepLabel) and: { (record[\age] ? 0.0) > 0.45 } };
        var minValue = nil;
        var maxValue = nil;
        var lowRecords;
        var highRecords;
        var span;

        stepRecords.do { |record|
            var value = (record[axis] ? 0.0).asFloat;
            if (minValue.isNil) {
                minValue = value;
                maxValue = value;
            } {
                minValue = minValue.min(value);
                maxValue = maxValue.max(value);
            };
        };

        if (minValue.isNil) {
            ^0.0;
        };
        span = (maxValue - minValue).abs;
        if (span < 0.001) {
            ^0.0;
        };

        lowRecords = stepRecords.select { |record| (record[axis] ? 0.0).asFloat <= (minValue + (span * 0.3)) };
        highRecords = stepRecords.select { |record| (record[axis] ? 0.0).asFloat >= (maxValue - (span * 0.3)) };
        ~swAmbientLabAnalysisContrast.(lowRecords, highRecords);
    };

    ~swAmbientLabAnalysisMaxJump = { |records, key|
        ~swAmbientLabAnalysisMax.(records, key);
    };

    ~swAmbientLabAnalysisWriteCsv = { |path, records|
        var file;

        if (path.notNil) {
            file = File(path, "w");
            file.write("elapsed,engine,step,age,yaw,pitch,roll,activity,spin,rms,rms_db,peak,peak_db,fast,slow,pop,hf,low,hf_ratio,width,zcr,centroid,mapped_brightness,mapped_darkness,mapped_density,mapped_shimmer,mapped_width,mapped_grain_scan,mapped_rhythm,mapped_scene,mapped_lift,rms_jump_db,hf_jump,width_jump,zcr_jump,pop_flag,silence_flag,abrupt_flag\n");
            records.do { |record|
                file.write([
                    record[\elapsed], record[\engine], record[\step], record[\age],
                    record[\yaw], record[\pitch], record[\roll], record[\activity], record[\spin],
                    record[\rms], record[\rmsDb], record[\peak], record[\peakDb],
                    record[\fast], record[\slow], record[\pop], record[\hf], record[\low],
                    record[\hfRatio], record[\width], record[\zcr], record[\centroid],
                    record[\mappedBrightness], record[\mappedDarkness], record[\mappedDensity], record[\mappedShimmer],
                    record[\mappedWidth], record[\mappedGrainScan], record[\mappedRhythm], record[\mappedScene], record[\mappedLift],
                    record[\rmsJumpDb], record[\hfJump], record[\widthJump], record[\zcrJump],
                    record[\popFlag], record[\silenceFlag], record[\abruptFlag]
                ].collect { |item| item.asString }.join(",") ++ "\n");
            };
            file.close;
        };
    };

    ~swAmbientLabAnalysisWriteSummary = { |path, summary|
        var file;

        if (path.notNil) {
            file = File(path, "w");
            file.write(summary ++ "\n");
            file.close;
        };
    };

    ~swAmbientLabAnalysisSummarize = { |records, engines|
        var lines = List.new;

        lines.add("SpaceWizard Ambient Lab Analysis");
        lines.add("Target: audible RMS roughly -43 to -18 dBFS, peak below -2 dBFS, low silence/pop/abrupt flags, and clear continuous yaw/pitch/roll travel response.");
        lines.add("");
        engines.do { |engine|
            var engineRecords = records.select { |record| (record[\engine] == engine) and: { (record[\age] ? 0.0) > 0.45 } };
            var rms = ~swAmbientLabAnalysisMean.(engineRecords, \rms);
            var rmsDb = ~swAmbientLabAnalysisDb.(rms);
            var peak = ~swAmbientLabAnalysisMax.(engineRecords, \peak);
            var peakDb = ~swAmbientLabAnalysisDb.(peak);
            var popCount = engineRecords.count { |record| record[\popFlag] ? false };
            var abruptCount = engineRecords.count { |record| record[\abruptFlag] ? false };
            var silenceCount = engineRecords.count { |record| record[\silenceFlag] ? false };
            var silencePercent = if (engineRecords.size > 0) { (silenceCount / engineRecords.size) * 100.0 } { 100.0 };
            var rmsRange = ~swAmbientLabAnalysisRange.(engineRecords, \rmsDb);
            var hfRange = ~swAmbientLabAnalysisRange.(engineRecords, \hfRatio);
            var centroidRange = ~swAmbientLabAnalysisRange.(engineRecords, \centroid);
            var widthRange = ~swAmbientLabAnalysisRange.(engineRecords, \width);
            var yawDelta = ~swAmbientLabAnalysisAxisDelta.(engineRecords, \yaw_sweep, \yaw);
            var pitchDelta = ~swAmbientLabAnalysisAxisDelta.(engineRecords, \pitch_sweep, \pitch);
            var rollDelta = ~swAmbientLabAnalysisAxisDelta.(engineRecords, \roll_sweep, \roll);
            var maxRmsJump = ~swAmbientLabAnalysisMaxJump.(engineRecords, \rmsJumpDb);
            var maxHfJump = ~swAmbientLabAnalysisMaxJump.(engineRecords, \hfJump);
            var maxWidthJump = ~swAmbientLabAnalysisMaxJump.(engineRecords, \widthJump);
            var warnings = List.new;

            if (engineRecords.isEmpty) {
                warnings.add("NO_DATA");
            };
            if (rmsDb < -43.0) {
                warnings.add("TOO_QUIET");
            };
            if (rmsDb > -14.0) {
                warnings.add("LOUD_AVERAGE");
            };
            if (peakDb > -2.0) {
                warnings.add("HOT_PEAKS");
            };
            if (silencePercent > 15.0) {
                warnings.add("SILENCE_GAPS");
            };
            if (popCount > (engineRecords.size * 0.03).max(2)) {
                warnings.add("POP_TRANSIENTS");
            };
            if (abruptCount > (engineRecords.size * 0.025).max(2)) {
                warnings.add("ABRUPT_CHANGES");
            };
            if ((rmsRange + hfRange + (centroidRange * 1.8) + widthRange) < 0.18) {
                warnings.add("LOW_MUSICAL_CHANGE");
            };
            if (yawDelta < 0.03) {
                warnings.add("YAW_WEAK");
            };
            if (pitchDelta < 0.03) {
                warnings.add("PITCH_WEAK");
            };
            if (rollDelta < 0.03) {
                warnings.add("ROLL_WEAK");
            };

            lines.add(
                engine.asString
                ++ " | rms=" ++ rmsDb.round(0.1).asString ++ " dB"
                ++ " peak=" ++ peakDb.round(0.1).asString ++ " dB"
                ++ " silence=" ++ silencePercent.round(0.1).asString ++ "%"
                ++ " pops=" ++ popCount.asString
                ++ " abrupt=" ++ abruptCount.asString
                ++ " change(rms/hf/cent/width)=" ++ rmsRange.round(0.1).asString ++ "/" ++ hfRange.round(0.001).asString ++ "/" ++ centroidRange.round(0.001).asString ++ "/" ++ widthRange.round(0.001).asString
                ++ " axes(y/p/r)=" ++ yawDelta.round(0.001).asString ++ "/" ++ pitchDelta.round(0.001).asString ++ "/" ++ rollDelta.round(0.001).asString
                ++ " jumps(db/hf/width)=" ++ maxRmsJump.round(0.1).asString ++ "/" ++ maxHfJump.round(0.001).asString ++ "/" ++ maxWidthJump.round(0.001).asString
                ++ " => " ++ (warnings.isEmpty.if("PASS", warnings.asArray.join("+")))
            );
        };
        lines.asArray.join("\n");
    };

    ~swAmbientLabAnalysisRun = { |engines=nil, stepSeconds=3.0, sampleRate=12.0, outputPath=nil, summaryPath=nil|
        var selectedEngines = engines ? ~swAmbientLabEngineKeys;
        var basePoint = (yaw: 0.34, pitch: 0.0, roll: 0.0, activity: 0.32, spin: 0.14);
        var segments = [
            (label: \center_hold, from: basePoint, to: basePoint, seconds: 1.2),
            (label: \yaw_sweep, from: basePoint.copy.put(\yaw, 0.06), to: basePoint.copy.put(\yaw, 0.94), seconds: stepSeconds),
            (label: \yaw_return, from: basePoint.copy.put(\yaw, 0.94), to: basePoint.copy.put(\yaw, 0.34), seconds: 1.4),
            (label: \pitch_sweep, from: basePoint.copy.put(\pitch, -0.9), to: basePoint.copy.put(\pitch, 0.9), seconds: stepSeconds),
            (label: \pitch_return, from: basePoint.copy.put(\pitch, 0.9), to: basePoint, seconds: 1.4),
            (label: \roll_sweep, from: basePoint.copy.put(\roll, -0.9), to: basePoint.copy.put(\roll, 0.9), seconds: stepSeconds),
            (label: \roll_return, from: basePoint.copy.put(\roll, 0.9), to: basePoint, seconds: 1.4),
            (label: \activity_rise, from: basePoint.copy.put(\activity, 0.12), to: basePoint.copy.put(\activity, 0.86), seconds: 2.0),
            (label: \spin_rise, from: basePoint.copy.put(\spin, 0.05), to: basePoint.copy.put(\spin, 0.9), seconds: 2.0)
        ];
        var records = List.new;
        var current = (
            engine: \none, step: \idle, stepStart: Main.elapsedTime,
            yaw: 0.0, pitch: 0.0, roll: 0.0, activity: 0.0, spin: 0.0, mapped: ()
        );
        var startTime = Main.elapsedTime;
        var analyzer;
        var axisValue;

        outputPath = outputPath ? ~swAmbientLabAnalysisOutputPath ? (Platform.userHomeDir +/+ "spacewizard_ambient_lab_analysis.csv");
        summaryPath = summaryPath ? ~swAmbientLabAnalysisSummaryPath ? (Platform.userHomeDir +/+ "spacewizard_ambient_lab_analysis.summary.txt");
        axisValue = { |from, to, key, blend=0.0, low=0.0, high=1.0|
            var start = from[key];
            var finish = to[key];

            if (start.isNil) {
                start = basePoint[key] ? 0.0;
            };
            if (finish.isNil) {
                finish = start;
            };
            (start + ((finish - start) * blend)).clip(low, high);
        };

        ~swAmbientLabStop.();
        ~swSetMotionGate.(true);
        analyzer = Synth.tail(s, \spacewizardAmbientLabAnalyzer, [\rate, sampleRate]);

        OSCdef(\swAmbientLabAnalysisCapture, { |msg|
            var rms = msg[3].asFloat;
            var peak = msg[4].asFloat;
            var fast = msg[5].asFloat;
            var slow = msg[6].asFloat;
            var pop = msg[7].asFloat;
            var hf = msg[8].asFloat;
            var low = msg[9].asFloat;
            var width = msg[10].asFloat;
            var zcr = msg[11].asFloat;
            var centroid = msg[12].asFloat;
            var now = Main.elapsedTime;
            var rmsDb = ~swAmbientLabAnalysisDb.(rms);
            var peakDb = ~swAmbientLabAnalysisDb.(peak);
            var hfRatio = hf / (rms + 0.0001);
            var popFlag = (pop > 0.035) or: { (fast > (slow * 5.0)) and: { peak > 0.08 } };
            var silenceFlag = rmsDb < -58.0;
            var mapped = current[\mapped] ? ();
            var prevRecord = nil;
            var rmsJumpDb = 0.0;
            var hfJump = 0.0;
            var widthJump = 0.0;
            var zcrJump = 0.0;
            var abruptFlag = false;

            if (records.size > 0) {
                prevRecord = records.at(records.size - 1);
            };
            if (
                prevRecord.notNil
                and: { prevRecord[\engine] == (current[\engine] ? \none) }
                and: { prevRecord[\step] == (current[\step] ? \idle) }
            ) {
                rmsJumpDb = (rmsDb - (prevRecord[\rmsDb] ? rmsDb)).abs;
                hfJump = (hfRatio - (prevRecord[\hfRatio] ? hfRatio)).abs;
                widthJump = (width - (prevRecord[\width] ? width)).abs;
                zcrJump = (zcr - (prevRecord[\zcr] ? zcr)).abs;
                abruptFlag = ((now - (current[\stepStart] ? now)) > 0.45) and: {
                    popFlag
                    or: { (rmsJumpDb > 11.0) and: { peakDb > -20.0 } }
                    or: { (hfJump > 0.32) and: { peakDb > -24.0 } }
                    or: { (zcrJump > 0.14) and: { peakDb > -24.0 } }
                };
            };

            records.add((
                elapsed: now - startTime,
                engine: current[\engine] ? \none,
                step: current[\step] ? \idle,
                age: now - (current[\stepStart] ? now),
                yaw: current[\yaw] ? 0.0,
                pitch: current[\pitch] ? 0.0,
                roll: current[\roll] ? 0.0,
                activity: current[\activity] ? 0.0,
                spin: current[\spin] ? 0.0,
                rms: rms,
                rmsDb: rmsDb,
                peak: peak,
                peakDb: peakDb,
                fast: fast,
                slow: slow,
                pop: pop,
                hf: hf,
                low: low,
                hfRatio: hfRatio,
                width: width,
                zcr: zcr,
                centroid: centroid,
                mappedBrightness: mapped[\brightness] ? 0.0,
                mappedDarkness: mapped[\darkness] ? 0.0,
                mappedDensity: mapped[\density] ? 0.0,
                mappedShimmer: mapped[\shimmer] ? 0.0,
                mappedWidth: mapped[\width] ? 0.0,
                mappedGrainScan: mapped[\grainScan] ? 0.0,
                mappedRhythm: mapped[\rhythm] ? 0.0,
                mappedScene: mapped[\sceneMorph] ? 0.0,
                mappedLift: mapped[\lift] ? 0.0,
                rmsJumpDb: rmsJumpDb,
                hfJump: hfJump,
                widthJump: widthJump,
                zcrJump: zcrJump,
                popFlag: popFlag,
                silenceFlag: silenceFlag,
                abruptFlag: abruptFlag
            ));
        }, '/sw/ambientLabAnalysis');

        Routine({
            0.5.wait;
            selectedEngines.do { |engine|
                var state = ~swAmbientLabDefaultState.();
                state[\engine] = engine;
                state[\isPlaying] = true;
                ~swAmbientLabState = ~swAmbientLabCopyState.(state);
                ~swAmbientLabApplyCurrent.();
                0.4.wait;

                segments.do { |segment|
                    var from = segment[\from] ? basePoint;
                    var to = segment[\to] ? from;
                    var seconds = segment[\seconds] ? stepSeconds;
                    var updates = ((seconds / 0.12).ceil.asInteger).max(2);

                    current = current.copy.put(\step, segment[\label]).put(\stepStart, Main.elapsedTime);
                    updates.do { |index|
                        var blend = index / (updates - 1).max(1);
                        var yaw = axisValue.(from, to, \yaw, blend, 0.0, 1.0);
                        var pitch = axisValue.(from, to, \pitch, blend, -1.0, 1.0);
                        var roll = axisValue.(from, to, \roll, blend, -1.0, 1.0);
                        var activity = axisValue.(from, to, \activity, blend, 0.0, 1.0);
                        var spin = axisValue.(from, to, \spin, blend, 0.0, 1.0);
                        var mapped;

                        mapped = ~swAmbientLabApplyMotion.(yaw, pitch, roll, activity, spin, 1.0);
                        current = current.copy.putAll((
                            engine: engine,
                            yaw: yaw,
                            pitch: pitch,
                            roll: roll,
                            activity: activity,
                            spin: spin,
                            mapped: mapped.copy
                        ));
                        0.12.wait;
                    };
                };
                ~swAmbientLabResetAudioState.();
                0.35.wait;
            };

            analyzer.free;
            OSCdef(\swAmbientLabAnalysisCapture).free;
            ~swAmbientLabStop.();
            ~swSetMotionGate.(false);
            ~swAmbientLabAnalysisWriteCsv.(outputPath, records);
            ~swAmbientLabAnalysisLastSummary = ~swAmbientLabAnalysisSummarize.(records, selectedEngines);
            ~swAmbientLabAnalysisWriteSummary.(summaryPath, ~swAmbientLabAnalysisLastSummary);
            ~swAmbientLabAnalysisLastSummary.postln;
            ("Ambient lab analysis CSV: " ++ outputPath).postln;
            ("Ambient lab analysis summary: " ++ summaryPath).postln;
            "SpaceWizard ambient analysis complete.".postln;
            if (~swAmbientLabAnalysisQuitOnComplete ? false) {
                AppClock.sched(0.5, {
                    s.quit;
                    0.exit;
                    nil;
                });
            };
            nil;
        }).play(AppClock);
    };

    ~swAmbientBassMidis = { |rootFreq=58|
        var rootMidi = rootFreq.cpsmidi.round.clip(30, 40);
        [0, 0, 3, 5, 7, 5, 3, 10].collect { |degree| rootMidi + degree };
    };

    ~swAmbientApplyBassLayer = { |params, activity, rollNorm, spinNorm, weight=0.0|
        var now = Main.elapsedTime;
        var activityThreshold = 0.12;
        var interval = (activity.linlin(0.0, 1.0, 2.4, 0.78) - (weight * 0.32) - (spinNorm * 0.12)).clip(0.52, 2.6);
        var pattern;
        var midi;
        var amp;

        if ((weight <= 0.18) or: { activity < activityThreshold }) {
            ~swResetAmbientBassTracking.();
        } {
            pattern = ~swAmbientBassMidis.(params[\ambientRoot] ? 58);
            if ((now - (~swAmbientBassLastTriggerTime ? 0.0)) >= interval) {
                midi = pattern.wrapAt(~swAmbientBassStep ? 0);
                amp = ((0.035 + (activity * 0.04)) * weight).clip(0.0, 0.11);
                ~swAmbientBassStep = (~swAmbientBassStep ? 0) + 1;
                ~swAmbientBassLastTriggerTime = now;
                ~swSmoothSpawnNote.(
                    freq: midi.midicps,
                    amp: amp,
                    pan: rollNorm.linlin(-1.0, 1.0, -0.16, 0.16) * weight,
                    vibratoRate: 0.06 + (spinNorm * 0.12),
                    vibratoDepth: 0.001 + (spinNorm * 0.0025),
                    motionDepth: 0.02 + (activity * 0.03),
                    bloom: 0.06 + (weight * 0.08),
                    lp: rollNorm.abs.linlin(0.0, 1.0, 720.0, 1450.0),
                    hold: 0.04 + (activity * 0.05),
                    decay: 1.8 + (activity * 1.6)
                );
            };
        };
    };

    ~swAmbientApplyMotionSingle = { |params, activity, pitchNorm, rollNorm, spinNorm|
        var variant = params[\ambientVariant] ? \flow;
        var mapped = ~swAmbientMapFromMotion.(params, activity, pitchNorm, rollNorm, spinNorm, variant);

        ~swAmbientApplyMappedParams.(mapped, variant);
        ~swAmbientReleaseVariantsExcept.([variant]);
        ~swAmbientApplyBassLayer.(mapped, activity, rollNorm, spinNorm, (variant == ~swAmbientBassVariant).if(1.0, 0.0));
        mapped;
    };

    ~swAmbientApplyMotionBlend = { |params, activity, pitchNorm, rollNorm, spinNorm, yawNorm=0.0|
        var blendPosition = ((yawNorm ? 0.0) + (params[\ambientYawOffset] ? 0.0)).wrap(0.0, 1.0);
        var blendState = ~swAmbientBlendStateFor.(blendPosition);
        var blended = params.copy.putAll((
            ambientBlendPosition: blendState[\position],
            ambientBlendPrimaryVariant: blendState[\primaryVariant],
            ambientBlendSecondaryVariant: blendState[\secondaryVariant],
            ambientBlendMix: blendState[\secondaryWeight]
        ));

        ~swAmbientVariantKeys.do { |variant|
            var mapped = ~swAmbientMapFromMotion.(blended, activity, pitchNorm, rollNorm, spinNorm, variant);
            var weight = 0.0;

            if (variant == blendState[\primaryVariant]) {
                weight = weight + (blendState[\primaryWeight] ? 0.0);
            };
            if (variant == blendState[\secondaryVariant]) {
                weight = weight + (blendState[\secondaryWeight] ? 0.0);
            };

            mapped[\amp] = (mapped[\amp] ? 0.1) * weight;
            ~swAmbientApplyMappedParams.(mapped, variant);
            if (variant == ~swAmbientBassVariant) {
                ~swAmbientApplyBassLayer.(mapped, activity, rollNorm, spinNorm, weight);
            };
        };

        blended;
    };

    ~swSmoothAmbientApplyMotion = { |params, activity=0.0, pitchNorm=0.0, rollNorm=0.0, spinNorm=0.0, yawNorm=0.0, gate=1.0|
        var overlay = params.copy;
        var enabled = overlay[\smoothAmbientEnabled];
        var labEnabled = overlay[\smoothAmbientLabEnabled] ? false;
        var blendEnabled = overlay[\smoothAmbientYawBlendEnabled];
        var ambientGain = overlay[\smoothAmbientAmpScale];
        var ambientCeiling = (overlay[\smoothAmbientAmpCeiling] ? 0.24).clip(0.0, 0.5);
        var panScale = overlay[\smoothAmbientPanScale];
        var widthScale = overlay[\smoothAmbientWidthScale];
        var baseVariant = overlay[\smoothAmbientVariant] ? (overlay[\ambientVariant] ? \flow);
        var baseRoot = overlay[\smoothAmbientRoot] ? (overlay[\ambientRoot] ? 74);
        var yawOffset = overlay[\smoothAmbientYawOffset];
        var blendState;
        var singleVariant;
        var singleMapped;
        var pitchShape = pitchNorm.abs;
        var rollShape = rollNorm.abs;
        var yawCentered = yawNorm.linlin(0.0, 1.0, -1.0, 1.0);
        var yawShape = yawCentered.abs;
        var smoothAmbientDecorate;

        if (enabled.isNil) {
            enabled = true;
        };

        if (enabled.not) {
            ~swAmbientStop.();
            ~swAmbientLabStopVoiceOnly.();
            ~swAmbientLabStopOverlay.();
            overlay
        } {
            if (labEnabled) {
                ^~swAmbientLabApplySmoothMotion.(overlay, activity, pitchNorm, rollNorm, spinNorm, yawNorm, gate);
            };
            ~swAmbientLabStopVoiceOnly.();
            ~swAmbientLabStopOverlay.();

            if (blendEnabled.isNil) {
                blendEnabled = true;
            };
            if (ambientGain.isNil) {
                ambientGain = 0.58;
            };
            if (panScale.isNil) {
                panScale = 0.6;
            };
            if (widthScale.isNil) {
                widthScale = 0.92;
            };

            overlay[\ambientVariant] = baseVariant;
            overlay[\ambientRoot] = (
                baseRoot.asFloat.max(20.0).cpsmidi.clip(28.0, 52.0)
                + pitchNorm.linlin(-1.0, 1.0, -4.0, 7.0)
                + (yawCentered * 2.0)
            ).midicps.clip(42.0, 160.0);
            overlay[\ambientYawBlendEnabled] = blendEnabled;

            smoothAmbientDecorate = { |mapped|
                var decorated = mapped.copy;

                decorated[\ambientLift] = ((decorated[\ambientLift] ? 0.0) + (pitchNorm * 0.08)).clip(-0.35, 0.35);
                decorated[\ambientLean] = ((decorated[\ambientLean] ? 0.0) + (yawCentered * 0.24)).clip(-1.0, 1.0);
                decorated[\ambientMotion] = ((decorated[\ambientMotion] ? 0.28) + (yawShape * 0.14) + (rollShape * 0.08)).clip(0.04, 1.0);
                decorated[\ambientSpin] = ((decorated[\ambientSpin] ? 0.18) + (yawShape * 0.12)).clip(0.02, 1.0);
                decorated[\ambientAura] = ((decorated[\ambientAura] ? 0.62) + pitchNorm.linlin(-1.0, 1.0, -0.04, 0.12) + (yawShape * 0.08)).clip(0.0, 1.0);
                decorated[\ambientDarkness] = ((decorated[\ambientDarkness] ? 0.58) + pitchNorm.linlin(-1.0, 1.0, 0.16, -0.16)).clip(0.0, 1.0);
                decorated[\ambientShimmer] = ((decorated[\ambientShimmer] ? 0.42) + yawNorm.linlin(0.0, 1.0, -0.04, 0.22) + (rollShape * 0.08) + (pitchShape * 0.04)).clip(0.0, 1.0);
                decorated[\ambientWidth] = (((decorated[\ambientWidth] ? 0.82) * widthScale) + (yawShape * 0.06)).clip(0.18, 1.0);
                decorated[\pan] = (((decorated[\pan] ? 0.0) * panScale) + (yawCentered * 0.08)).clip(-0.4, 0.4);
                decorated[\motionBus] = ~swSmoothLayerLevelBus;
                decorated[\motionRise] = ~swSmoothLayerRiseSeconds.();
                if ((overlay[\smoothAmbientAmpFollowsActivity] ? true).not) {
                    decorated[\amp] = (overlay[\smoothAmbientFixedAmp] ? 0.10).clip(0.0, 0.2);
                };
                decorated
            };

            if (yawOffset.isNil) {
                yawOffset = overlay[\ambientYawOffset];
            };
            if (yawOffset.isNil) {
                yawOffset = ~swAmbientYawOffsetForVariant.(baseVariant);
            };
            overlay[\ambientYawOffset] = yawOffset;

            if (blendEnabled) {
                blendState = ~swAmbientBlendStateFor.((yawNorm + yawOffset).wrap(0.0, 1.0));
                overlay = overlay.copy.putAll((
                    ambientBlendPosition: blendState[\position],
                    ambientBlendPrimaryVariant: blendState[\primaryVariant],
                    ambientBlendSecondaryVariant: blendState[\secondaryVariant],
                    ambientBlendMix: blendState[\secondaryWeight]
                ));

                // Keep every ring variant alive and crossfade by blend weight (as the
                // ambient-flow mode does). Releasing variants that left the blend pair and
                // re-creating them when the heading swung back stacked ~20 fading voices
                // during fast waving and overloaded the Pi's audio server (Sep 2026).
                // Each voice already smooths amp changes with a 0.8 s lag.
                ~swAmbientVariantKeys.do { |variant|
                    var mapped = ~swAmbientMapFromMotion.(overlay, activity, pitchNorm, rollNorm, spinNorm, variant);
                    var weight = 0.0;

                    if (variant == blendState[\primaryVariant]) {
                        weight = weight + (blendState[\primaryWeight] ? 0.0);
                    };
                    if (variant == blendState[\secondaryVariant]) {
                        weight = weight + (blendState[\secondaryWeight] ? 0.0);
                    };

                    mapped = smoothAmbientDecorate.(mapped);
                    mapped[\amp] = ((mapped[\amp] ? 0.1) * ambientGain * weight).clip(0.0, ambientCeiling);
                    ~swAmbientApplyMappedParams.(mapped, variant);
                };

                ~swAmbientReleaseVariantsExcept.(~swAmbientVariantKeys);
            } {
                singleVariant = overlay[\ambientVariant] ? \flow;
                singleMapped = ~swAmbientMapFromMotion.(overlay, activity, pitchNorm, rollNorm, spinNorm, singleVariant);
                singleMapped = smoothAmbientDecorate.(singleMapped);
                singleMapped[\amp] = ((singleMapped[\amp] ? 0.1) * ambientGain).clip(0.0, ambientCeiling);
                ~swAmbientApplyMappedParams.(singleMapped, singleVariant);
                ~swAmbientReleaseVariantsExcept.([singleVariant]);
                overlay = overlay.copy.putAll((
                    ambientBlendPosition: (yawNorm + yawOffset).wrap(0.0, 1.0),
                    ambientBlendPrimaryVariant: singleVariant,
                    ambientBlendSecondaryVariant: singleVariant,
                    ambientBlendMix: 0.0
                ));
            };

            overlay
        }
    };

    ~swSmoothAmbientApplyCurrent = {
        var params = ~swAuduinoParams;
        var inputs = ~swLiveInputState ? ();

        ~swSmoothAmbientApplyMotion.(
            params,
            inputs[\activity] ? 0.0,
            inputs[\pitchNorm] ? 0.0,
            inputs[\rollNorm] ? 0.0,
            inputs[\spinNorm] ? 0.0,
            inputs[\yawNorm] ? 0.0,
            inputs[\gate] ? 1.0
        );
    };

    ~swSmoothMixGUI = {
        var win;
        var params = (~swAuduinoParams ? ~swAuduinoDefaultParams.copy).copy;
        var readout;
        var updateReadout;
        var applyValues;
        var makeSlider;
        var dumpValues;
        var reopenWithPreset;

        if (~swSmoothMixWindow.notNil) {
            ~swSmoothMixWindow.front;
            ^~swSmoothMixWindow;
        };

        if (params[\smoothAmpScale].isNil) {
            params[\smoothAmpScale] = 1.0;
        };
        if (params[\smoothAmbientAmpScale].isNil) {
            params[\smoothAmbientAmpScale] = 0.58;
        };
        if (params[\smoothAmbientAmpCeiling].isNil) {
            params[\smoothAmbientAmpCeiling] = 0.24;
        };

        updateReadout = {
            if (readout.notNil) {
                readout.string_(
                    "smoothAmpScale: " ++ ~swFormatStatusValue.(params[\smoothAmpScale]) ++ "\n"
                    ++ "smoothAmbientAmpScale: " ++ ~swFormatStatusValue.(params[\smoothAmbientAmpScale]) ++ "\n"
                    ++ "smoothAmbientAmpCeiling: " ++ ~swFormatStatusValue.(params[\smoothAmbientAmpCeiling]) ++ "\n"
                    ++ "current note base amp: " ++ ~swFormatStatusValue.(params[\amp] ? 0.0) ++ "\n"
                    ++ "current note effective amp: " ++ ~swFormatStatusValue.(~swSmoothScaledAmp.(params))
                );
            };
        };

        applyValues = {
            var next = (~swAuduinoParams ? ~swAuduinoDefaultParams.copy).copy;
            next[\smoothAmpScale] = params[\smoothAmpScale];
            next[\smoothAmbientAmpScale] = params[\smoothAmbientAmpScale];
            next[\smoothAmbientAmpCeiling] = params[\smoothAmbientAmpCeiling];
            ~swAuduinoParams = next;
            params = next.copy;
            ~swSmoothApplyMonoAmpScale.();
            if ((next[\voiceMode] ? \auduino) == \smoothSine) {
                ~swSmoothAmbientApplyCurrent.();
            };
            ~swRefreshLiveStatusGUI.();
            updateReadout.();
        };

        dumpValues = {
            ("Smooth mix values: smoothAmpScale=" ++ (params[\smoothAmpScale] ? 1.0).round(0.001).asString
                ++ ", smoothAmbientAmpScale=" ++ (params[\smoothAmbientAmpScale] ? 0.58).round(0.001).asString
                ++ ", smoothAmbientAmpCeiling=" ++ (params[\smoothAmbientAmpCeiling] ? 0.24).round(0.001).asString).postln;
        };

        reopenWithPreset = { |preset|
            var selected = preset ? \standard;
            params = (~swAuduinoParams ? ~swAuduinoDefaultParams.copy).copy.putAll(~swSmoothPresetFor.(selected));
            ~swAuduinoParams = params.copy;
            ~swSmoothApplyMonoAmpScale.();
            if ((params[\voiceMode] ? \auduino) == \smoothSine) {
                ~swSmoothAmbientApplyCurrent.();
            };
            ~swRefreshLiveStatusGUI.();
            win.close;
            AppClock.sched(0.05, {
                ~swSmoothMixGUI.();
                nil;
            });
        };

        makeSlider = { |label, key, spec, top|
            EZSlider(
                win,
                Rect(12, top, 456, 30),
                label,
                spec,
                { |view|
                    params[key] = view.value;
                    applyValues.();
                },
                params[key],
                layout: 'horz'
            );
        };

        win = Window("SpaceWizard Smooth Mix", Rect(160, 120, 500, 270));
        win.alwaysOnTop_(true);
        ~swSmoothMixWindow = win;

        StaticText(win, Rect(12, 10, 456, 22))
            .string_("Smooth sine live mix")
            .align_(\left);

        makeSlider.("Pent Notes", \smoothAmpScale, ControlSpec(0.0, 1.2, \lin, 0.001, params[\smoothAmpScale] ? 1.0), 44);
        makeSlider.("Ambient Bed", \smoothAmbientAmpScale, ControlSpec(0.0, 3.0, \lin, 0.001, params[\smoothAmbientAmpScale] ? 0.58), 82);

        readout = TextView(win, Rect(12, 124, 456, 82))
            .font_(Font("Menlo", 12))
            .editable_(false)
            .string_("");

        Button(win, Rect(12, 220, 108, 28))
            .states_([["Print Values"]])
            .action_({ dumpValues.(); });

        Button(win, Rect(132, 220, 108, 28))
            .states_([["Quiet Preset"]])
            .action_({ reopenWithPreset.(\quietPentatonic); });

        Button(win, Rect(252, 220, 98, 28))
            .states_([["Standard"]])
            .action_({ reopenWithPreset.(\standard); });

        Button(win, Rect(362, 220, 82, 28))
            .states_([["Close"]])
            .action_({ win.close; });

        win.onClose_({
            ~swSmoothMixWindow = nil;
        });

        win.front;
        updateReadout.();
    };

    ~swSmoothChoirEnsure = {
        if (~swSmoothChoir.isNil) {
            var synthName = (~swChoirHasSamples ? false).if(\spacewizardHeavenlyChoirSamples, \spacewizardHeavenlyChoirSynth);
            var synthArgs = [
                \motionBus, ~swMotionLevelBus.index,
                \motionRise, ~swMotionFadeInSeconds,
                \motionFall, ~swMotionFadeOutSeconds
            ];

            if (~swChoirHasSamples ? false) {
                synthArgs = [
                    \buf0, ~swChoirBufNumAt.(0),
                    \buf1, ~swChoirBufNumAt.(1),
                    \buf2, ~swChoirBufNumAt.(2),
                    \buf3, ~swChoirBufNumAt.(3),
                    \buf4, ~swChoirBufNumAt.(4)
                ] ++ synthArgs;
            };

            ~swSmoothChoir = Synth(synthName, synthArgs);
            ~swSmoothChoir.onFree { ~swSmoothChoir = nil; };
        };
        ~swSmoothChoir;
    };

    ~swSmoothChoirSet = { |rootFreq=220, amp=0.60, synthAmp=0.46, pan=0.0, spread=0.78, shimmer=0.12, breath=0.07, brightness=0.28,
        choirFxMix=0.32, choirDelayMix=0.14, choirAir=0.36, choirBassAmp=0.0, choirBassTone=0.42,
        sampleStart=0.0, textureMode=4, basePitchHz=176.47, trigger=false, fadeIn=0.5, holdTime=0.28, fadeOut=4.2,
        octaveDown3=0, octaveDown2=1, octaveDown1=1, octaveUp1=0, octaveUp2=0, octaveUp3=0|
        var voice = ~swSmoothChoirEnsure.();

        ~swSmoothChoirTriggerCount = ~swSmoothChoirTriggerCount ? 0;
        if (trigger) {
            ~swSmoothChoirTriggerCount = ~swSmoothChoirTriggerCount + 1;
        };

        if (~swChoirHasSamples ? false) {
            voice.set(
                \gate, 1,
                \rootFreq, rootFreq,
                \amp, amp,
                \synthAmp, synthAmp,
                \pan, pan,
                \spread, spread,
                \shimmer, shimmer,
                \breath, breath,
                \brightness, brightness,
                \choirFxMix, choirFxMix,
                \choirDelayMix, choirDelayMix,
                \choirAir, choirAir,
                \choirBassAmp, choirBassAmp,
                \choirBassTone, choirBassTone,
                \sampleStart, sampleStart,
                \basePitchHz, basePitchHz,
                \fadeIn, fadeIn,
                \holdTime, holdTime,
                \fadeOut, fadeOut,
                \octaveDown3, octaveDown3,
                \octaveDown2, octaveDown2,
                \octaveDown1, octaveDown1,
                \octaveUp1, octaveUp1,
                \octaveUp2, octaveUp2,
                \octaveUp3, octaveUp3,
                'textureMode', textureMode,
                \choirTrig, ~swSmoothChoirTriggerCount
            );
        } {
            voice.set(
                \gate, 1,
                \rootFreq, rootFreq,
                \amp, amp,
                \pan, pan,
                \spread, spread,
                \shimmer, shimmer,
                \breath, breath,
                \brightness, brightness,
                \choirFxMix, choirFxMix,
                \choirDelayMix, choirDelayMix,
                \choirAir, choirAir,
                \choirBassAmp, choirBassAmp,
                \choirBassTone, choirBassTone,
                \sampleStart, sampleStart,
                \fadeIn, fadeIn,
                \holdTime, holdTime,
                \fadeOut, fadeOut,
                'textureMode', textureMode,
                \choirTrig, ~swSmoothChoirTriggerCount
            );
        };
    };

    ~swSmoothChoirStop = {
        if (~swSmoothChoir.notNil) {
            ~swSmoothChoir.set(\gate, 0);
            ~swSmoothChoir = nil;
        };
        ~swResetSmoothChoirTracking.();
    };

    ~swChoirTextureDemoPlay = { |choirAmp=0.60, synthAmp=0.46, sampleStart=0.0, synthMode=4,
        spread=0.78, shimmer=0.12, breath=0.07, brightness=0.28,
        choirFxMix=0.32, choirDelayMix=0.14, choirAir=0.36, choirBassAmp=0.0, choirBassTone=0.42,
        octaveDown3=0, octaveDown2=1, octaveDown1=1, octaveUp1=0, octaveUp2=0, octaveUp3=0|
        var choirPitchHz = ~swChoirSamplePitchHz ? 176.47;
        var synthName = [
            \spacewizardChoirPreviewCathedral,
            \spacewizardChoirPreviewFlow,
            \spacewizardChoirPreviewRibbon,
            \spacewizardChoirPreviewGlass,
            \spacewizardChoirPreviewVowel,
            \spacewizardChoirPreviewChime
        ].wrapAt(synthMode.asInteger.clip(0, 5));
        ~swChoirTextureDemoStop.();

        if (~swChoirHasSamples ? false) {
            ~swChoirRawPreview = Synth(\spacewizardChoirRawSamplePlayer, [
                \buf, ~swChoirBufNumAt.(0),
                \amp, choirAmp.clip(0.0, 0.6),
                \sampleStart, sampleStart.clip(0.0, 5.0),
                \fxMix, choirFxMix.clip(0.0, 1.0),
                \delayMix, choirDelayMix.clip(0.0, 1.0),
                \air, choirAir.clip(0.0, 1.0),
                \brightness, brightness.clip(0.0, 1.0),
                \spread, spread.clip(0.1, 1.0)
            ]);
            ~swChoirRawPreview.onFree { ~swChoirRawPreview = nil; };

            ~swChoirBlendPreview = Synth(synthName, [
                \buf, ~swChoirBufNumAt.(0),
                \synthAmp, synthAmp.clip(0.0, 0.8),
                \sampleStart, sampleStart.clip(0.0, 5.0),
                \basePitchHz, choirPitchHz,
                \spread, spread.clip(0.1, 1.0),
                \shimmer, shimmer.clip(0.0, 1.0),
                \breath, breath.clip(0.0, 0.2),
                \brightness, brightness.clip(0.0, 1.0),
                \hallMixCtl, choirFxMix.clip(0.0, 1.0),
                \hallRoomCtl, (0.42 + (choirFxMix.clip(0.0, 1.0) * 0.38)).clip(0.0, 1.0),
                \hallDampCtl, (0.58 - (brightness.clip(0.0, 1.0) * 0.22)).clip(0.0, 1.0),
                \bassAmp, choirBassAmp.clip(0.0, 0.45),
                \bassTone, choirBassTone.clip(0.0, 1.0),
                \octaveDown3, octaveDown3.clip(0.0, 1.0),
                \octaveDown2, octaveDown2.clip(0.0, 1.0),
                \octaveDown1, octaveDown1.clip(0.0, 1.0),
                \octaveUp1, octaveUp1.clip(0.0, 1.0),
                \octaveUp2, octaveUp2.clip(0.0, 1.0),
                \octaveUp3, octaveUp3.clip(0.0, 1.0)
            ]);
            ~swChoirBlendPreview.onFree { ~swChoirBlendPreview = nil; };
        } {
            "No choir sample loaded; cannot play original-plus-synth preview.".warn;
        };

        ~swRequestLiveStatusGUIRefresh.();
    };

    ~swChoirRawDemoPlay = { |amp=0.24, sampleStart=0.0|
        if (~swChoirHasSamples ? false) {
            ~swChoirTextureDemoStop.();
            ~swChoirRawPreview = Synth(\spacewizardChoirRawSamplePlayer, [
                \buf, ~swChoirBufNumAt.(0),
                \amp, amp.clip(0.05, 0.34),
                \sampleStart, sampleStart.clip(0.0, 5.0)
            ]);
            ~swChoirRawPreview.onFree { ~swChoirRawPreview = nil; };
        } {
            "No choir sample loaded; cannot play original sample preview.".warn;
        };
        ~swRequestLiveStatusGUIRefresh.();
    };

    ~swChoirRawDemoStop = {
        if (~swChoirRawPreview.notNil) {
            ~swChoirRawPreview.free;
            ~swChoirRawPreview = nil;
        };
    };

    ~swChoirTextureDemoStop = {
        if (~swChoirBlendPreview.notNil) {
            ~swChoirBlendPreview.free;
            ~swChoirBlendPreview = nil;
        };
        ~swChoirRawDemoStop.();
        ~swSmoothChoirStop.();
        ~swSetMotionGate.(false);
        ~swRequestLiveStatusGUIRefresh.();
    };

    ~swSmoothChoirApplyMotion = { |params, pitchNorm=0.0, rollNorm=0.0, yawNorm=0.0, gate=0.0|
        var enabled = params[\smoothChoirEnabled];
        var gateOpen = gate > 0.0;
        var gateJustOpened = gateOpen and: { (~swSmoothChoirGateWasOpen ? false).not };
        var shouldTrigger = gateJustOpened or: { gateOpen and: { ~swSmoothChoir.isNil } };
        var needsDeferredTrigger = gateOpen and: { ~swSmoothChoir.isNil };
        var yawCentered = yawNorm.linlin(0.0, 1.0, -1.0, 1.0);
        var rollShape = rollNorm.abs;
        var choirMidi = (params[\smoothMidi] ? ~swSmoothMidiForPitch.(pitchNorm, params[\smoothRootMidi] ? 38)) + 19 + (yawCentered * 2.0);
        var choirRootFreq = (~swChoirHasSamples ? false).if(~swChoirSamplePitchHz ? 176.47, choirMidi.midicps);
        var playChoir;

        if (enabled.isNil) {
            enabled = true;
        };

        playChoir = { |triggerNow=true|
            ~swSmoothChoirSet.(
                rootFreq: choirRootFreq,
                amp: params[\smoothChoirAmp] ? 0.60,
                synthAmp: params[\smoothChoirSynthAmp] ? 0.46,
                pan: yawCentered * 0.12,
                spread: ((params[\smoothChoirSpread] ? 0.78) + (yawNorm * 0.08)).clip(0.3, 1.0),
                shimmer: ((params[\smoothChoirShimmer] ? 0.12) + (yawNorm * 0.18) + (rollShape * 0.06)).clip(0.0, 1.0),
                breath: ((params[\smoothChoirBreath] ? 0.07) + (pitchNorm.abs * 0.01)).clip(0.0, 0.18),
                brightness: ((params[\smoothChoirBrightness] ? 0.28) + pitchNorm.linlin(-1.0, 1.0, -0.06, 0.14) + (yawNorm * 0.06)).clip(0.0, 1.0),
                choirFxMix: params[\smoothChoirFxMix] ? 0.32,
                choirDelayMix: params[\smoothChoirDelayMix] ? 0.14,
                choirAir: params[\smoothChoirAir] ? 0.36,
                choirBassAmp: params[\smoothChoirBassAmp] ? 0.0,
                choirBassTone: params[\smoothChoirBassTone] ? 0.42,
                sampleStart: params[\smoothChoirSampleStart] ? 0.0,
                basePitchHz: ~swChoirSamplePitchHz ? 176.47,
                textureMode: params[\smoothChoirTexture] ? 4,
                fadeIn: params[\smoothChoirFadeIn] ? 0.5,
                holdTime: params[\smoothChoirHold] ? 0.28,
                fadeOut: params[\smoothChoirFadeOut] ? 4.2,
                octaveDown3: params[\smoothChoirOctaveDown3] ? 0,
                octaveDown2: params[\smoothChoirOctaveDown2] ? 1,
                octaveDown1: params[\smoothChoirOctaveDown1] ? 1,
                octaveUp1: params[\smoothChoirOctaveUp1] ? 0,
                octaveUp2: params[\smoothChoirOctaveUp2] ? 0,
                octaveUp3: params[\smoothChoirOctaveUp3] ? 0,
                trigger: triggerNow
            );
        };

        if (enabled.not) {
            ~swSmoothChoirStop.();
        } {
            if (shouldTrigger) {
                if (needsDeferredTrigger) {
                    playChoir.(false);
                    AppClock.sched(0.05, {
                        var inputs = ~swLiveInputState ? ();
                        var stillInSmoothSine = (((~swAuduinoParams ? ())[\voiceMode] ? \auduino) == \smoothSine);
                        if (stillInSmoothSine and: { (inputs[\gate] ? 0.0) > 0.0 }) {
                            playChoir.(true);
                        };
                        nil;
                    });
                } {
                    playChoir.(true);
                };
            };
            ~swSmoothChoirGateWasOpen = gateOpen;
        };
    };

    ~swSmoothChoirApplyCurrent = {
        var params = ~swAuduinoParams;
        var inputs = ~swLiveInputState ? ();

        ~swSmoothChoirApplyMotion.(
            params,
            inputs[\pitchNorm] ? (params[\smoothPitchNorm] ? 0.0),
            inputs[\rollNorm] ? 0.0,
            inputs[\yawNorm] ? 0.0,
            inputs[\gate] ? 0.0
        );
    };

    ~swReeseApplyCurrent = {
        var params = ~swAuduinoParams;

        ~swReeseSet.(
            freq: params[\reeseFreq] ? 50,
            amp: params[\amp] ? 0.12,
            pan: params[\pan] ? 0.0,
            detune: params[\reeseDetune] ? 0.028,
            glide: params[\reeseGlide] ? 0.32,
            wobbleRate: params[\reeseWobbleRate] ? 0.18,
            wobbleDepth: params[\reeseWobbleDepth] ? 0.12,
            drive: params[\reeseDrive] ? 2.8,
            lp: params[\reeseLp] ? 900,
            rq: params[\reeseRq] ? 0.24,
            subMix: params[\reeseSubMix] ? 0.46
        );
    };

    ~swWowApplyCurrent = {
        var params = ~swAuduinoParams;

        ~swWowSet.(
            freq: params[\wowFreq] ? 110,
            amp: params[\amp] ? 0.12,
            pan: params[\pan] ? 0.0,
            glide: params[\wowGlide] ? 0.28,
            direction: params[\wowDirection] ? 1,
            vowelDepth: params[\wowVowelDepth] ? 0.82,
            breath: params[\wowBreath] ? 0.035,
            lp: params[\wowLp] ? 2400,
            tone: params[\wowTone] ? 0.16,
            trigger: false
        );
    };

    ~swApplyLiveModeCurrent = {
        var params = ~swAuduinoParams;
        var voiceMode = params[\voiceMode] ? \auduino;

        if (voiceMode == \song) {
            // Song mode has no continuous voice; the song itself follows the motion gate.
            ~swAuduinoStopAll.();
            ~swAmbientStop.();
            ~swAmbientLabStopVoiceOnly.();
            ~swAmbientLabStopOverlay.();
            ~swSmoothStop.();
            ~swSmoothChoirStop.();
            ~swReeseStop.();
            ~swWowStop.();
        } {
        if (voiceMode == \ambientFlow) {
            ~swAuduinoStopAll.();
            ~swSmoothStop.();
            ~swSmoothChoirStop.();
            ~swReeseStop.();
            ~swWowStop.();
            ~swAmbientLabStopVoiceOnly.();
            ~swAmbientLabStopOverlay.();
            ~swAmbientApplyCurrent.();
        } {
            if (voiceMode == \smoothSine) {
                ~swAuduinoStopAll.();
                ~swReeseStop.();
                ~swWowStop.();
                ~swSmoothApplyCurrent.();
            } {
                if (voiceMode == \reeseBass) {
                    ~swAuduinoStopAll.();
                    ~swAmbientStop.();
                    ~swAmbientLabStopVoiceOnly.();
                    ~swAmbientLabStopOverlay.();
                    ~swSmoothStop.();
                    ~swSmoothChoirStop.();
                    ~swWowStop.();
                    ~swReeseApplyCurrent.();
                } {
                    if (voiceMode == \wowVoice) {
                        ~swAuduinoStopAll.();
                        ~swAmbientStop.();
                        ~swAmbientLabStopVoiceOnly.();
                        ~swAmbientLabStopOverlay.();
                        ~swSmoothStop.();
                        ~swSmoothChoirStop.();
                        ~swReeseStop.();
                        ~swWowApplyCurrent.();
                    } {
                        ~swAmbientStop.();
                        ~swAmbientLabStopVoiceOnly.();
                        ~swAmbientLabStopOverlay.();
                        ~swSmoothStop.();
                        ~swSmoothChoirStop.();
                        ~swReeseStop.();
                        ~swWowStop.();
                        ~swAuduinoApplyCurrent.();
                    };
                };
            };
        };
        };
    };

    ~swSetLiveMode = { |voiceMode=\auduino|
        var selectedMode =
            if (voiceMode == \ambientFlow) { \ambientFlow }
            { if (voiceMode == \smoothSine) { \smoothSine }
                { if (voiceMode == \reeseBass) { \reeseBass }
                    { if (voiceMode == \wowVoice) { \wowVoice }
                        { if (voiceMode == \song) { \song } { \auduino } } } } };
        var params = (~swAuduinoParams ? ~swAuduinoDefaultParams.copy).copy;

        params[\voiceMode] = selectedMode;
        if (selectedMode == \auduino) {
            params = params.putAll(~swSoundPresetOverridesFor.(\auduino, \standard));
        };
        if (selectedMode == \reeseBass) {
            params = params.putAll(~swSoundPresetOverridesFor.(\reeseBass, \standard));
        };
        if (selectedMode == \wowVoice) {
            params = params.putAll(~swSoundPresetOverridesFor.(\wowVoice, \standard));
        };
        if (selectedMode == \smoothSine) {
            params = params.putAll(~swSmoothPresetFor.(params[\smoothPreset] ? (~swRequestedSmoothPreset ? \standard)));
            ~swApplySmoothPresetGlobals.(params);
        };
        if (selectedMode == \ambientFlow) {
            params = params.putAll(~swAmbientPresetFor.(params[\ambientVariant] ? (~swRequestedAmbientVariant ? \flow)));
        };
        if (selectedMode == \song) {
            params = params.putAll(~swSoundPresetOverridesFor.(\song, ~swRequestedSongPreset ? \dayO));
            ~swLoadSongBuffer.(params);
        };
        params[\voiceMode] = selectedMode;
        ~swAuduinoParams = params;
        ("Live synth mode: " ++ selectedMode.asString).postln;
        ~swRefreshLiveStatusGUI.();

        if ((~swAuduino.notNil) or: { ~swAmbientHasVoices.() } or: { ~swSmoothHasVoices.() } or: { ~swReeseBass.notNil } or: { ~swWowVoice.notNil }) {
            ~swApplyLiveModeCurrent.();
        };

        selectedMode;
    };

    ~swSmoothSineScene = { |freq=nil, amp=0.1|
        var startFreq = freq ? 38.midicps;
        ~swSetMotionGate.(true);
        ~swSetLiveMode.(\smoothSine);
        ~swSmoothAmbientApplyCurrent.();
        ~swSmoothSet.(
            freq: startFreq,
            amp: amp,
            pan: 0.0,
            vibratoRate: 0.12,
            vibratoDepth: 0.004,
            motionDepth: 0.05,
            bloom: 0.15,
            lp: 1700
        );
    };

    ~swSmoothSineSweep = {
        Routine({
            var notes = ~swSmoothBassMidis.(~swAuduinoParams[\smoothRootMidi] ? 38);
            ~swSetMotionGate.(true);
            ~swSetLiveMode.(\smoothSine);
            ~swSmoothAmbientApplyCurrent.();
            notes.size.do { |step|
                var blend = step / (notes.size - 1).max(1);
                ~swSmoothSet.(
                    freq: notes[step].midicps,
                    amp: 0.085 + (blend * 0.035),
                    pan: blend.linlin(0.0, 1.0, -0.18, 0.18),
                    vibratoRate: blend.linexp(0.0, 1.0, 0.08, 0.6),
                    vibratoDepth: 0.003 + (blend * 0.005),
                    motionDepth: 0.04 + (blend * 0.05),
                    bloom: 0.12 + (blend * 0.1),
                    lp: blend.linexp(0.0, 1.0, 700.0, 1800.0),
                    hold: 0.07,
                    decay: 2.4 + (blend * 0.8)
                );
                0.75.wait;
            };
        }).play(AppClock);
    };

    ~swAmbientScene = { |rootFreq=nil, amp=nil, variant=nil|
        var selectedVariant = variant ? (~swAuduinoParams[\ambientVariant] ? \flow);
        var preset = ~swAmbientPresetFor.(selectedVariant).copy;

        ~swSetMotionGate.(true);

        if (rootFreq.notNil) {
            preset[\ambientRoot] = rootFreq;
        };
        if (amp.notNil) {
            preset[\amp] = amp;
        };

        ~swAuduinoParams = ~swAuduinoParams.copy.putAll(preset);
        ~swSetLiveMode.(\ambientFlow);
        ~swAmbientApplyCurrent.();
    };

    ~swAmbientSweep = { |variant=nil|
        Routine({
            var selectedVariant = variant ? (~swAuduinoParams[\ambientVariant] ? \flow);
            var states = ~swAmbientSweepStatesFor.(selectedVariant);

            ~swSetMotionGate.(true);
            ~swSetLiveMode.(\ambientFlow);
            states.do { |state|
                ~swAuduinoParams = ~swAuduinoParams.copy
                    .putAll(state)
                    .put(\voiceMode, \ambientFlow)
                    .put(\ambientVariant, selectedVariant);
                ~swAmbientApplyCurrent.();
                5.5.wait;
            };
        }).play(AppClock);
    };

    ~swAmbientChordalScene = { |rootFreq=nil, amp=nil|
        ~swAmbientScene.(rootFreq, amp, \chordal);
    };

    ~swAmbientChordalSweep = {
        ~swAmbientSweep.(\chordal);
    };

    ~swAmbientDroneScene = { |rootFreq=nil, amp=nil|
        ~swAmbientScene.(rootFreq, amp, \drone);
    };

    ~swAmbientDroneSweep = {
        ~swAmbientSweep.(\drone);
    };

    ~swAmbientForestScene = { |rootFreq=nil, amp=nil|
        ~swAmbientScene.(rootFreq, amp, \forest);
    };

    ~swAmbientForestSweep = {
        ~swAmbientSweep.(\forest);
    };

    ~swReeseBassScene = { |freq=46, amp=0.12|
        ~swSetMotionGate.(true);
        ~swSetLiveMode.(\reeseBass);
        ~swReeseSet.(
            freq: freq,
            amp: amp,
            pan: 0.0,
            detune: 0.032,
            glide: 0.4,
            wobbleRate: 0.14,
            wobbleDepth: 0.1,
            drive: 3.0,
            lp: 760,
            rq: 0.22,
            subMix: 0.5
        );
    };

    ~swReeseBassSweep = {
        Routine({
            ~swSetMotionGate.(true);
            ~swSetLiveMode.(\reeseBass);
            16.do { |step|
                var blend = step / 15;
                ~swReeseSet.(
                    freq: blend.linexp(0.0, 1.0, 32.0, 108.0),
                    amp: 0.09 + (blend * 0.05),
                    pan: blend.linlin(0.0, 1.0, -0.14, 0.14),
                    detune: blend.linlin(0.0, 1.0, 0.012, 0.06),
                    glide: blend.linlin(0.0, 1.0, 0.48, 0.12),
                    wobbleRate: blend.linexp(0.0, 1.0, 0.06, 3.6),
                    wobbleDepth: blend.linlin(0.0, 1.0, 0.04, 0.26),
                    drive: blend.linlin(0.0, 1.0, 2.0, 4.8),
                    lp: blend.linexp(0.0, 1.0, 320.0, 2200.0),
                    rq: blend.linlin(0.0, 1.0, 0.22, 0.44),
                    subMix: blend.linlin(0.0, 1.0, 0.52, 0.26)
                );
                0.55.wait;
            };
        }).play(AppClock);
    };

    ~swWowScene = { |freq=108, amp=0.12|
        ~swSetMotionGate.(true);
        ~swSetLiveMode.(\wowVoice);
        ~swWowSet.(
            freq: freq,
            amp: amp,
            pan: 0.0,
            glide: 0.32,
            direction: 1,
            vowelDepth: 0.9,
            breath: 0.04,
            lp: 2200,
            tone: 0.18,
            trigger: true
        );
    };

    ~swWowSweep = {
        Routine({
            var dir = 1;
            ~swSetMotionGate.(true);
            ~swSetLiveMode.(\wowVoice);
            12.do { |step|
                var blend = step / 11;
                ~swWowSet.(
                    freq: blend.linexp(0.0, 1.0, 82.0, 180.0),
                    amp: 0.09 + (blend * 0.05),
                    pan: blend.linlin(0.0, 1.0, -0.18, 0.18),
                    glide: 0.36 - (blend * 0.18),
                    direction: dir,
                    vowelDepth: 0.72 + (blend * 0.24),
                    breath: 0.025 + (blend * 0.03),
                    lp: blend.linexp(0.0, 1.0, 1200.0, 3200.0),
                    tone: 0.12 + (blend * 0.12),
                    trigger: (step == 0)
                );
                0.35.wait;
            };
            dir = -1;
            ~swWowSet.(freq: 152, amp: 0.13, glide: 0.25, direction: dir, vowelDepth: 0.95, breath: 0.05, lp: 2400, tone: 0.22, trigger: true);
            10.do { |step|
                var blend = step / 9;
                ~swWowSet.(
                    freq: blend.linexp(0.0, 1.0, 152.0, 86.0),
                    amp: 0.12,
                    pan: blend.linlin(0.0, 1.0, 0.14, -0.14),
                    glide: 0.22 + (blend * 0.18),
                    direction: dir,
                    vowelDepth: 0.92 - (blend * 0.18),
                    breath: 0.05 - (blend * 0.012),
                    lp: blend.linexp(0.0, 1.0, 2500.0, 1500.0),
                    tone: 0.2,
                    trigger: false
                );
                0.38.wait;
            };
        }).play(AppClock);
    };

    ~swTestPickup = {
        ~swPickup.(activity: 0.32, tilt: 0.12, roll: -0.08, spin: 0.06, presence: 0.58, amp: 0.18);
    };

    ~swTestTiltSweep = {
        Routine({
            ~swPickup.(activity: 0.28, tilt: -0.85, roll: -0.1, spin: 0.06, presence: 0.52, amp: 0.17);
            16.do { |step|
                var blend = step / 15;
                var tilt = blend.linlin(0.0, 1.0, -0.9, 0.95);
                ~swSet.(
                    activity: 0.24 + (blend * 0.28),
                    tilt: tilt,
                    roll: -0.25 + (blend * 0.5),
                    spin: 0.05 + (blend * 0.08),
                    presence: 0.48 + (blend * 0.28),
                    amp: 0.18
                );
                0.55.wait;
            };
            1.5.wait;
            ~swRest.();
            2.0.wait;
            ~swRelease.();
        }).play(AppClock);
    };

    ~swTestSpinStudy = {
        Routine({
            ~swPickup.(activity: 0.42, tilt: 0.05, roll: 0.0, spin: 0.04, presence: 0.62, amp: 0.18);
            10.do { |step|
                var spin = step.linlin(0, 9, 0.02, 1.0);
                ~swSet.(
                    activity: 0.35 + (spin * 0.25),
                    tilt: 0.1,
                    roll: spin.linlin(0.02, 1.0, -0.2, 0.25),
                    spin: spin,
                    presence: 0.58 + (spin * 0.25),
                    amp: 0.18
                );
                0.7.wait;
            };
            1.0.wait;
            ~swRest.();
            1.2.wait;
            ~swRelease.();
        }).play(AppClock);
    };

    ~swTestAutopilot = {
        Routine({
            ~swPickup.(activity: 0.26, tilt: -0.25, roll: -0.12, spin: 0.04, presence: 0.5, amp: 0.17);
            8.do { |step|
                var phase = step / 7;
                ~swSet.(
                    activity: 0.22 + (phase * 0.15),
                    tilt: phase.linlin(0.0, 1.0, -0.35, 0.1),
                    roll: phase.linlin(0.0, 1.0, -0.18, 0.12),
                    spin: 0.03 + (phase * 0.05),
                    presence: 0.46 + (phase * 0.18),
                    amp: 0.16
                );
                0.75.wait;
            };

            Synth(\spacewizardPickupBloom, [\tilt, 0.0, \spin, 0.3, \brightness, 0.75, \amp, 0.22]);

            12.do { |step|
                var phase = step / 11;
                ~swSet.(
                    activity: 0.38 + (0.32 * sin(phase * 2pi).abs),
                    tilt: sin(phase * 2pi * 0.7) * 0.75,
                    roll: cos(phase * 2pi * 0.9) * 0.55,
                    spin: 0.18 + (phase * 0.55),
                    presence: 0.65 + (phase * 0.18),
                    amp: 0.19
                );
                0.65.wait;
            };

            6.do { |step|
                var falloff = 1.0 - (step / 5);
                ~swSet.(
                    activity: 0.08 + (falloff * 0.18),
                    tilt: falloff * -0.3,
                    roll: falloff * 0.16,
                    spin: falloff * 0.12,
                    presence: 0.3 + (falloff * 0.1),
                    amp: 0.12 + (falloff * 0.03)
                );
                0.8.wait;
            };

            ~swRest.();
            2.5.wait;
            ~swRelease.();
        }).play(AppClock);
    };

    ~swSawPentatonicSweep = { |rootMidi=50, amp=0.12, stepDur=0.24|
        Routine({
            var notes = ~swPentatonicMidis.(rootMidi);
            ~swSawLeadStart.(
                midi: notes.first,
                amp: amp,
                pan: -0.3,
                cutoff: 1500,
                rq: 0.18,
                drive: 5.2,
                glide: 0.03
            );
            notes.do { |midi, index|
                ~swSawLeadSet.(
                    midi: midi,
                    amp: amp,
                    pan: index.linlin(0, notes.size - 1, -0.35, 0.35),
                    cutoff: 1500 + (index * 300),
                    rq: 0.18,
                    drive: 5.2,
                    glide: 0.04
                );
                stepDur.wait;
            };
            0.15.wait;
            ~swSawLeadStop.();
        }).play(AppClock);
    };

    ~swSawPentatonicRandom = { |rootMidi=50, amp=0.11, steps=18|
        Routine({
            var notes = ~swPentatonicMidis.(rootMidi);
            ~swSawLeadStart.(
                midi: notes.choose,
                amp: amp,
                pan: 0.0,
                cutoff: 2400,
                rq: 0.18,
                drive: 5.8,
                glide: 0.02
            );
            steps.do {
                var midi = notes.choose;
                var wait = [0.12, 0.16, 0.2, 0.28, 0.36].choose;
                ~swSawLeadSet.(
                    midi: midi,
                    amp: amp * [0.9, 1.0, 1.1].choose,
                    pan: 0.65.rand2,
                    cutoff: exprand(1700, 5600),
                    rq: exprand(0.1, 0.26),
                    drive: exprand(4.8, 7.0),
                    glide: wait * 0.18
                );
                wait.wait;
            };
            0.12.wait;
            ~swSawLeadStop.();
        }).play(AppClock);
    };

    ~swSawPentatonicScene = { |rootMidi=50, amp=0.115|
        Routine({
            var notes = ~swPentatonicMidis.(rootMidi);
            ~swSawLeadStart.(
                midi: notes.first,
                amp: amp,
                pan: -0.25,
                cutoff: 1550,
                rq: 0.16,
                drive: 5.4,
                glide: 0.035
            );
            notes.do { |midi, index|
                ~swSawLeadSet.(
                    midi: midi,
                    amp: amp,
                    pan: index.linlin(0, notes.size - 1, -0.3, 0.3),
                    cutoff: 1550 + (index * 260),
                    rq: 0.16,
                    drive: 5.4,
                    glide: 0.045
                );
                0.22.wait;
            };

            0.45.wait;

            20.do {
                var midi = notes.choose;
                var wait = [0.12, 0.16, 0.19, 0.24, 0.32].choose;
                ~swSawLeadSet.(
                    midi: midi,
                    amp: amp * [0.9, 1.0, 1.15].choose,
                    pan: 0.7.rand2,
                    cutoff: exprand(1800, 6000),
                    rq: exprand(0.1, 0.24),
                    drive: exprand(5.2, 7.4),
                    glide: wait * 0.2
                );
                wait.wait;
            };

            0.15.wait;
            ~swSawLeadStop.();
        }).play(AppClock);
    };

    ~swAuduinoPentatonicScene = { |rootMidi=50, amp=0.13, scaleMode|
        var routine = Routine({
            var selectedScaleMode = scaleMode ? (~swAuduinoParams[\scaleMode] ? \originalAuduino);
            var notes = ~swAuduinoNotesForScale.(selectedScaleMode, rootMidi, 10);
            ~swAuduinoStart.(
                syncFreq: (notes.first.midicps * 0.36).clip(18, 220),
                grain1Freq: notes.first.midicps,
                grain2Freq: notes.first.midicps * 1.498,
                decay1: 0.32,
                decay2: 0.22,
                amp: amp,
                pan: -0.15,
                drive: 3.0,
                crushBits: 7,
                hp: 65,
                lp: 5200,
                scaleMode: selectedScaleMode,
                playMode: \continuous,
                rootMidi: rootMidi,
                stepIndex: 10
            );

            notes.do { |midi, index|
                ~swAuduinoSetMidi.(
                    midi: midi,
                    amp: amp,
                    pan: index.linlin(0, notes.size - 1, -0.25, 0.25),
                    drive: 2.8 + (index * 0.08),
                    crushBits: 7
                );
                0.22.wait;
            };

            0.4.wait;

            18.do {
                var midi = notes.choose;
                var wait = [0.11, 0.15, 0.18, 0.24, 0.3].choose;
                ~swAuduinoSetMidi.(
                    midi: midi,
                    amp: amp * [0.9, 1.0, 1.1].choose,
                    pan: 0.5.rand2,
                    drive: exprand(2.6, 3.8),
                    crushBits: [6, 7, 7, 8].choose
                );
                wait.wait;
            };

            0.2.wait;
            ~swAuduinoStop.();
            ~swAuduinoRoutine = nil;
        });
        ~swAuduinoPlayRoutine.(routine);
    };

    ~swAuduinoSteppedScene = { |rootMidi=50, amp=0.13, scaleMode, noteDur=0.12, gapDur=0.08|
        var routine = Routine({
            var selectedScaleMode = scaleMode ? (~swAuduinoParams[\scaleMode] ? \originalAuduino);
            var notes = ~swAuduinoNotesForScale.(selectedScaleMode, rootMidi, 10);

            notes.do { |midi, index|
                ~swAuduinoStepMidi.(
                    midi: midi,
                    amp: amp,
                    pan: index.linlin(0, notes.size - 1, -0.25, 0.25),
                    drive: 2.8 + (index * 0.08),
                    crushBits: 7,
                    noteDur: noteDur,
                    gapDur: gapDur,
                    melodic: false
                );
            };

            14.do {
                var midi = notes.choose;
                var localNoteDur = [0.08, 0.11, 0.13, 0.16].choose;
                var localGapDur = [0.05, 0.07, 0.09, 0.12].choose;
                ~swAuduinoStepMidi.(
                    midi: midi,
                    amp: amp * [0.9, 1.0, 1.1].choose,
                    pan: 0.5.rand2,
                    drive: exprand(2.6, 3.8),
                    crushBits: [6, 7, 7, 8].choose,
                    noteDur: localNoteDur,
                    gapDur: localGapDur,
                    melodic: false
                );
            };

            ~swAuduinoStop.();
            ~swAuduinoRoutine = nil;
        });
        ~swAuduinoPlayRoutine.(routine);
    };

    ~swAuduinoMelodicScene = { |rootMidi=50, amp=0.12, scaleMode|
        var routine = Routine({
            var selectedScaleMode = scaleMode ? (~swAuduinoParams[\scaleMode] ? \originalAuduino);
            var notes = ~swAuduinoNotesForScale.(selectedScaleMode, rootMidi, 10);
            ~swAuduinoStart.(
                syncFreq: (notes.first.midicps * 0.16).clip(10, 90),
                grain1Freq: notes.first.midicps,
                grain2Freq: notes.first.midicps * 1.334,
                decay1: 0.10,
                decay2: 0.07,
                amp: amp,
                pan: -0.12,
                drive: 2.4,
                crushBits: 7,
                hp: 55,
                lp: 4300,
                scaleMode: selectedScaleMode,
                playMode: \melodicContinuous,
                rootMidi: rootMidi,
                stepIndex: 10
            );

            notes.do { |midi, index|
                ~swAuduinoSetMelodicMidi.(
                    midi: midi,
                    amp: amp,
                    pan: index.linlin(0, notes.size - 1, -0.2, 0.2),
                    drive: 2.3 + (index * 0.04),
                    crushBits: 7
                );
                0.42.wait;
            };

            0.8.wait;

            14.do {
                var midi = notes.choose;
                var wait = [0.28, 0.36, 0.45, 0.56, 0.68].choose;
                ~swAuduinoSetMelodicMidi.(
                    midi: midi,
                    amp: amp * [0.92, 1.0, 1.06].choose,
                    pan: 0.35.rand2,
                    drive: exprand(2.2, 3.0),
                    crushBits: [7, 7, 8].choose
                );
                wait.wait;
            };

            0.35.wait;
            ~swAuduinoStop.();
            ~swAuduinoRoutine = nil;
        });
        ~swAuduinoPlayRoutine.(routine);
    };

    ~swAuduinoMelodicSteppedScene = { |rootMidi=50, amp=0.12, scaleMode, noteDur=0.22, gapDur=0.12|
        var routine = Routine({
            var selectedScaleMode = scaleMode ? (~swAuduinoParams[\scaleMode] ? \originalAuduino);
            var notes = ~swAuduinoNotesForScale.(selectedScaleMode, rootMidi, 10);

            notes.do { |midi, index|
                ~swAuduinoStepMidi.(
                    midi: midi,
                    amp: amp,
                    pan: index.linlin(0, notes.size - 1, -0.2, 0.2),
                    drive: 2.3 + (index * 0.04),
                    crushBits: 7,
                    noteDur: noteDur,
                    gapDur: gapDur,
                    melodic: true
                );
            };

            10.do {
                var midi = notes.choose;
                var localNoteDur = [0.18, 0.22, 0.28, 0.34].choose;
                var localGapDur = [0.09, 0.12, 0.15, 0.18].choose;
                ~swAuduinoStepMidi.(
                    midi: midi,
                    amp: amp * [0.92, 1.0, 1.06].choose,
                    pan: 0.35.rand2,
                    drive: exprand(2.2, 3.0),
                    crushBits: [7, 7, 8].choose,
                    noteDur: localNoteDur,
                    gapDur: localGapDur,
                    melodic: true
                );
            };

            ~swAuduinoStop.();
            ~swAuduinoRoutine = nil;
        });
        ~swAuduinoPlayRoutine.(routine);
    };

    ~swAuduinoGUI = {
        var win;
        var params = (~swAuduinoParams ? ~swAuduinoDefaultParams.copy).copy;
        var makeSlider;
        var scaleOptions = ~swAuduinoScaleOptions;
        var scaleLabels = scaleOptions.collect { |item| item[\label] };
        var modeOptions = [
            (key: \continuous, label: "Continuous"),
            (key: \melodicContinuous, label: "Melodic continuous"),
            (key: \stepped, label: "Stepped"),
            (key: \melodicStepped, label: "Melodic stepped")
        ];
        var modeLabels = modeOptions.collect { |item| item[\label] };
        var currentScaleIndex;
        var currentModeIndex;
        var updateRunning;

        if (~swAuduinoWindow.notNil) {
            ~swAuduinoWindow.close;
        };

        updateRunning = {
            ~swAuduinoParams = params.copy;
            if ((~swAuduino.notNil) or: { ~swAmbientHasVoices.() } or: { ~swSmoothHasVoices.() } or: { ~swReeseBass.notNil } or: { ~swWowVoice.notNil }) {
                ~swApplyLiveModeCurrent.();
            };
        };

        currentScaleIndex = scaleOptions.detectIndex { |item|
            item[\key] == (params[\scaleMode] ? \originalAuduino)
        } ? 0;

        currentModeIndex = modeOptions.detectIndex { |item|
            item[\key] == (params[\playMode] ? \continuous)
        } ? 0;

        win = Window("SpaceWizard Auduino Controls", Rect(120, 120, 430, 500));
        win.alwaysOnTop_(true);
        ~swAuduinoWindow = win;

        StaticText(win, Rect(12, 8, 400, 20))
            .string_("Auduino-style controls with original D-E-G-A-B table option")
            .align_(\left);

        StaticText(win, Rect(12, 30, 70, 18))
            .string_("Scale")
            .align_(\left);

        PopUpMenu(win, Rect(82, 28, 260, 24))
            .items_(scaleLabels)
            .value_(currentScaleIndex)
            .action_({ |menu|
                var selected = scaleOptions[menu.value] ? scaleOptions.first;
                params[\scaleMode] = selected[\key];
                updateRunning.();
            });

        StaticText(win, Rect(12, 58, 70, 18))
            .string_("Mode")
            .align_(\left);

        PopUpMenu(win, Rect(82, 56, 260, 24))
            .items_(modeLabels)
            .value_(currentModeIndex)
            .action_({ |menu|
                var selected = modeOptions[menu.value] ? modeOptions.first;
                params[\playMode] = selected[\key];
                updateRunning.();
            });

        makeSlider = { |label, key, spec, top|
            EZSlider(
                win,
                Rect(12, top, 406, 28),
                label,
                spec,
                { |view|
                    params[key] = view.value;
                    updateRunning.();
                },
                params[key],
                layout: 'horz'
            );
        };

        makeSlider.("Sync Hz / Step", \syncFreq, ControlSpec(~swAuduinoStepSyncMinHz, ~swAuduinoStepSyncMaxHz, \exp, 0.01, params[\syncFreq]), 92);
        makeSlider.("Grain 1 Hz", \grain1Freq, ControlSpec(60, 1200, \exp, 0.01, params[\grain1Freq]), 126);
        makeSlider.("Grain 2 Hz", \grain2Freq, ControlSpec(60, 1600, \exp, 0.01, params[\grain2Freq]), 160);
        makeSlider.("Decay 1", \decay1, ControlSpec(0.02, 0.6, \lin, 0.001, params[\decay1]), 194);
        makeSlider.("Decay 2", \decay2, ControlSpec(0.02, 0.6, \lin, 0.001, params[\decay2]), 228);
        makeSlider.("Drive", \drive, ControlSpec(1.0, 5.0, \lin, 0.01, params[\drive]), 262);
        makeSlider.("Bit Crush", \crushBits, ControlSpec(4, 10, \lin, 1, params[\crushBits]), 296);
        makeSlider.("Level", \amp, ControlSpec(0.03, 0.25, \lin, 0.001, params[\amp]), 330);

        Button(win, Rect(12, 384, 110, 28))
            .states_([[
                "Play Current"
            ]])
            .action_({
                ~swAuduinoParams = params.copy;
                ~swApplyLiveModeCurrent.();
            });

        Button(win, Rect(132, 384, 70, 28))
            .states_([[
                "Stop"
            ]])
            .action_({ ~swAuduinoStopAll.(); });

        Button(win, Rect(214, 384, 120, 28))
            .states_([[
                "Reset Melodic"
            ]])
            .action_({
                params = (
                    syncFreq: 42,
                    grain1Freq: 196,
                    grain2Freq: 294,
                    decay1: 0.14,
                    decay2: 0.09,
                    amp: 0.12,
                    pan: 0.0,
                    drive: 2.6,
                    crushBits: 7,
                    hp: 60,
                    lp: 4600,
                    scaleMode: \originalAuduino,
                    playMode: \continuous,
                    rootMidi: 50,
                    stepIndex: 10
                );
                ~swAuduinoParams = params.copy;
                win.close;
                ~swAuduinoGUI.();
            });

        win.onClose_({
            ~swAuduinoWindow = nil;
            ~swAuduinoStopAll.();
        });
        win.front;
    };

    ~swAmbientGUI = {
        var win;
        var params = (~swAuduinoParams ? ~swAuduinoDefaultParams.copy).copy;
        var makeSlider;
        var updateRunning;
        var variantOptions = ~swAmbientVariantOptions;
        var variantLabels = variantOptions.collect { |item| item[\label] };
        var currentVariantIndex;
        var descriptionView;
        var updateDescription;
        var stopOnClose = true;
        var reopenWithParams;

        if (~swAmbientWindow.notNil) {
            ~swAmbientWindow.close;
        };

        params[\voiceMode] = \ambientFlow;
        params[\ambientVariant] = params[\ambientVariant] ? \flow;

        updateRunning = {
            params[\voiceMode] = \ambientFlow;
            ~swAuduinoParams = params.copy;
            if (~swAmbientHasVoices.()) {
                ~swAmbientApplyCurrent.();
            };
        };

        updateDescription = {
            descriptionView.string_(~swAmbientDescriptionFor.(params[\ambientVariant] ? \flow));
        };

        reopenWithParams = { |nextParams|
            stopOnClose = false;
            params = ~swAuduinoParams.copy.putAll(nextParams);
            params[\voiceMode] = \ambientFlow;
            params[\ambientVariant] = params[\ambientVariant] ? \flow;
            ~swAuduinoParams = params.copy;
            if (~swAmbientHasVoices.()) {
                ~swAmbientApplyCurrent.();
            };
            win.close;
            AppClock.sched(0.05, {
                ~swAmbientGUI.();
                nil;
            });
        };

        currentVariantIndex = variantOptions.detectIndex { |item|
            item[\key] == (params[\ambientVariant] ? \flow)
        } ? 0;

        win = Window("SpaceWizard Ambient Controls", Rect(140, 90, 560, 570));
        win.alwaysOnTop_(true);
        ~swAmbientWindow = win;

        StaticText(win, Rect(12, 8, 536, 34))
            .string_("Ambient variants for demo mode.\nUse Root, Lift, Lean, Activity, and Spin to reshape the bed; live motion now favors space, color, and density over big pitch swings.")
            .align_(\left);

        StaticText(win, Rect(12, 50, 56, 18))
            .string_("Mode")
            .align_(\left);

        PopUpMenu(win, Rect(70, 48, 220, 24))
            .items_(variantLabels)
            .value_(currentVariantIndex)
            .action_({ |menu|
                var selected = variantOptions[menu.value] ? variantOptions.first;
                params[\ambientVariant] = selected[\key];
                updateDescription.();
                updateRunning.();
            });

        descriptionView = StaticText(win, Rect(12, 78, 536, 42))
            .align_(\left);
        updateDescription.();

        makeSlider = { |label, key, spec, top|
            EZSlider(
                win,
                Rect(12, top, 536, 28),
                label,
                spec,
                { |view|
                    params[key] = view.value;
                    updateRunning.();
                },
                params[key],
                layout: 'horz'
            );
        };

        makeSlider.("Root Hz", \ambientRoot, ControlSpec(40, 220, \exp, 0.01, params[\ambientRoot] ? 74), 128);
        makeSlider.("Pitch Lift", \ambientLift, ControlSpec(-1, 1, \lin, 0.001, params[\ambientLift] ? -0.1), 162);
        makeSlider.("Roll Lean", \ambientLean, ControlSpec(-1, 1, \lin, 0.001, params[\ambientLean] ? -0.15), 196);
        makeSlider.("Activity", \ambientMotion, ControlSpec(0, 1, \lin, 0.001, params[\ambientMotion] ? 0.22), 230);
        makeSlider.("Spin", \ambientSpin, ControlSpec(0, 1, \lin, 0.001, params[\ambientSpin] ? 0.14), 264);
        makeSlider.("Aura", \ambientAura, ControlSpec(0, 1, \lin, 0.001, params[\ambientAura] ? 0.62), 298);
        makeSlider.("Darkness", \ambientDarkness, ControlSpec(0, 1, \lin, 0.001, params[\ambientDarkness] ? 0.66), 332);
        makeSlider.("Shimmer", \ambientShimmer, ControlSpec(0, 1, \lin, 0.001, params[\ambientShimmer] ? 0.34), 366);
        makeSlider.("Width", \ambientWidth, ControlSpec(0, 1, \lin, 0.001, params[\ambientWidth] ? 0.78), 400);
        makeSlider.("Level", \amp, ControlSpec(0.03, 0.22, \lin, 0.001, params[\amp] ? 0.1), 434);

        Button(win, Rect(12, 490, 132, 30))
            .states_([[
                "Play Current"
            ]])
            .action_({
                ~swAuduinoParams = params.copy;
                ~swSetLiveMode.(\ambientFlow);
                ~swAmbientApplyCurrent.();
            });

        Button(win, Rect(156, 490, 84, 30))
            .states_([[
                "Stop"
            ]])
            .action_({ ~swAmbientStop.(); });

        Button(win, Rect(252, 490, 120, 30))
            .states_([[
                "Sweep Demo"
            ]])
            .action_({
                ~swAuduinoParams = params.copy;
                ~swAmbientSweep.(params[\ambientVariant] ? \flow);
            });

        Button(win, Rect(384, 490, 140, 30))
            .states_([[
                "Reset Mode"
            ]])
            .action_({
                reopenWithParams.(~swAmbientPresetFor.(params[\ambientVariant] ? \flow));
            });

        win.onClose_({
            ~swAmbientWindow = nil;
            if (stopOnClose) {
                ~swAmbientStop.();
            };
        });
        win.front;
    };

    ~swAmbientLabInfoText = (
        halo: "Halo Drone: continuous meandering sine and saw partials, glints, comb space, and slow color changes. Good for the sustained beautiful bed after the choir accent.",
        airports: "Airports Chords: slow stochastic chord changes and long plucked/bell events. Good for subtle rhythmic qualities without becoming a beat.",
        granular: "Granular Choir Cloud: stretches and scans the loaded choir sample with stock GrainBuf, falling back to a formant cloud if the sample is missing.",
        hexBells: "Hex Bells: motion-controlled hex-Dorian bell sequence inspired by Tomer Baruch's generative patch. Yaw steers phrase transposition, pitch opens the tone, roll widens the bell field, and activity/spin shape sparse rhythm.",
        tapeSwells: "Tape Swells: slow brown-walk saw swells inspired by the shared patch, with stock delay, reverb, and tape-style wobble. Yaw chooses transposition scenes, pitch opens the filter, roll widens the field, and activity/spin control event flow and tape motion.",
        tapeScript: "Tape Script: close stock-UGen version of the shared Tdef/Ndef swell patch. Yaw moves through transposition and delay time, pitch opens the Moog-style filter/freqScale, roll changes width and tape wobble, and activity/spin alter event flow.",
        hybrid: "Hybrid Flow: routes this lab mapping into the existing Flow, Chordal, Drone, and Forest engines, with Scene Morph crossfading around that variant ring.",
        yaw: "Yaw is unipolar from 0 to 1. It works best for scene morphing, grain scan position, or opening the yaw-window vowel overlay.",
        pitch: "Pitch is bipolar from -1 to 1. It works best for sky/opening gestures: brightness, darkness, and gentle harmony lift.",
        roll: "Roll is bipolar from -1 to 1. It now changes internal width and diffusion instead of panning, so rolling the candle feels like opening the space rather than moving the whole sound left or right.",
        activity: "Activity is unipolar from 0 to 1. It works best for density, motion, loudness support, and how much the bed wakes up.",
        spin: "Spin/Accel is unipolar from 0 to 1. In this phase it represents motion energy and is useful for shimmer, rhythm density, or grain movement.",
        sceneMorph: "Scene Morph moves through the active harmonic/textural scene. In Hybrid Flow it crossfades through Flow, Chordal, Drone, and Forest; in Hex Bells, Tape Swells, and Tape Script it biases phrase transposition.",
        harmonyLift: "Harmony Lift raises or lowers chord color without making the whole ambient layer track pitch too literally.",
        brightness: "Brightness opens filters and upper formants. Use subtly so the candle feels expressive instead of harsh.",
        darkness: "Darkness closes the top end and emphasizes weight. Invert pitch here for a natural down-is-darker gesture.",
        density: "Density controls how busy the layer is: partial drift, grains, event rate, and motion in the texture.",
        shimmer: "Shimmer controls glints, air, detune, and high-frequency motion.",
        width: "Width controls stereo spread and internal diffusion while keeping the center stable. It is especially useful with roll because the gesture is physically spatial without turning into a pan control.",
        pan: "Pan is legacy-only in this lab. New mappings should use Width, Scene Morph, Grain Scan, or Shimmer so motion changes how the music is generated rather than where it sits.",
        grainScan: "Grain Scan moves through the choir sample cloud or acts as a secondary morph control in non-granular engines. In Hex Bells, Tape Swells, and Tape Script it blends phrase/tape choices toward yaw-selected colors.",
        rhythmDensity: "Rhythm Density increases sparse plucks, glints, and pulse-like events while keeping them ambient.",
        overlayMix: "Overlay Mix controls the subtle vowel synth that fades in only inside the yaw window.",
        pentatonicSynth: "Pent Synth adds the existing smoothSine triggered pentatonic notes on top of any ambient engine. Sweeping Pitch steps through the pentatonic notes; each note is short, with Pent Level setting how present the hits are.",
        overview: "Ambient Motion Lab: pick an engine, move the manual motion sliders, assign axes to targets, and promote a good mapping when it is ready for smooth live mode."
    );

    ~swAmbientLabInfoFor = { |key=\overview|
        ~swAmbientLabInfoText[key] ? ~swAmbientLabInfoText[\overview];
    };

    ~swAmbientLabGUI = {
        var win;
        var state = ~swAmbientLabCopyState.(~swAmbientLabState ? ~swAmbientLabDefaultState.());
        var engineLabels = ~swAmbientLabEngineOptions.collect { |item| item[\label] };
        var presetLabels = ~swAmbientLabPresetOptions.collect { |item| item[\label] };
        var targetLabels = ~swAmbientLabTargetOptions.collect { |item| item[\label] };
        var curveLabels = ~swAmbientLabCurveOptions.collect { |item| item[\label] };
        var infoView;
        var setInfo;
        var applyIfPlaying;
        var reopenWithState;
        var makeInfoButton;
        var makeSlider;
        var makeMappingRow;
        var stopOnClose = true;

        if (~swAmbientLabWindow.notNil) {
            ~swAmbientLabWindow.close;
        };
        if (state[\mappings].isNil) {
            state[\mappings] = ~swAmbientLabDefaultMappings.();
        };

        setInfo = { |key|
            if (infoView.notNil) {
                infoView.string_(~swAmbientLabInfoFor.(key));
            };
        };

        applyIfPlaying = {
            ~swAmbientLabState = ~swAmbientLabCopyState.(state);
            if (state[\isPlaying] ? false) {
                ~swAmbientLabApplyCurrent.();
            };
        };

        reopenWithState = { |nextState|
            var wasPlaying = state[\isPlaying] ? false;
            stopOnClose = false;
            state = ~swAmbientLabCopyState.(nextState);
            state[\isPlaying] = wasPlaying;
            ~swAmbientLabState = ~swAmbientLabCopyState.(state);
            if (wasPlaying) {
                ~swAmbientLabApplyCurrent.();
            };
            win.close;
            AppClock.sched(0.05, {
                ~swAmbientLabGUI.();
                nil;
            });
        };

        makeInfoButton = { |left, top, key|
            Button(win, Rect(left, top, 24, 22))
                .states_([["?"]])
                .action_({ setInfo.(key); });
        };

        makeSlider = { |label, key, spec, top|
            EZSlider(
                win,
                Rect(12, top, 360, 24),
                label,
                spec,
                { |view|
                    state[key] = view.value;
                    applyIfPlaying.();
                },
                state[key],
                layout: 'horz'
            );
        };

        makeMappingRow = { |axisItem, top|
            var axis = axisItem[\key];
            var mapping = (state[\mappings][axis] ? ~swAmbientLabMapping.()).copy;
            var targetIndex = ~swAmbientLabIndexFor.(~swAmbientLabTargetOptions, mapping[\target] ? \none);
            var curveIndex = ~swAmbientLabIndexFor.(~swAmbientLabCurveOptions, mapping[\curve] ? \linear);

            StaticText(win, Rect(12, top + 3, 68, 18))
                .string_(axisItem[\label])
                .align_(\left);
            makeInfoButton.(78, top, axis);

            PopUpMenu(win, Rect(110, top, 156, 22))
                .items_(targetLabels)
                .value_(targetIndex)
                .action_({ |menu|
                    var selected = ~swAmbientLabTargetOptions[menu.value] ? ~swAmbientLabTargetOptions.first;
                    mapping[\target] = selected[\key];
                    state[\mappings][axis] = mapping;
                    setInfo.(selected[\key]);
                    applyIfPlaying.();
                });

            EZSlider(
                win,
                Rect(274, top, 160, 22),
                "Amt",
                ControlSpec(0, 1, \lin, 0.001, mapping[\amount] ? 0.0),
                { |view|
                    mapping[\amount] = view.value;
                    state[\mappings][axis] = mapping;
                    applyIfPlaying.();
                },
                mapping[\amount] ? 0.0,
                layout: 'horz'
            );

            PopUpMenu(win, Rect(442, top, 86, 22))
                .items_(curveLabels)
                .value_(curveIndex)
                .action_({ |menu|
                    var selected = ~swAmbientLabCurveOptions[menu.value] ? ~swAmbientLabCurveOptions.first;
                    mapping[\curve] = selected[\key];
                    state[\mappings][axis] = mapping;
                    applyIfPlaying.();
                });

            Button(win, Rect(536, top, 52, 22))
                .states_([["Norm"], ["Inv"]])
                .value_((mapping[\invert] ? false).if(1, 0))
                .action_({ |button|
                    mapping[\invert] = button.value == 1;
                    state[\mappings][axis] = mapping;
                    applyIfPlaying.();
                });

            makeInfoButton.(596, top, mapping[\target] ? \none);
        };

        win = Window("SpaceWizard Ambient Motion Lab", Rect(110, 70, 760, 772));
        win.alwaysOnTop_(true);
        ~swAmbientLabWindow = win;

        StaticText(win, Rect(12, 8, 390, 22))
            .string_("Ambient Motion Lab")
            .align_(\left);

        StaticText(win, Rect(12, 38, 52, 18))
            .string_("Engine")
            .align_(\left);
        PopUpMenu(win, Rect(72, 36, 190, 24))
            .items_(engineLabels)
            .value_(~swAmbientLabIndexFor.(~swAmbientLabEngineOptions, state[\engine] ? \halo))
            .action_({ |menu|
                var selected = ~swAmbientLabEngineOptions[menu.value] ? ~swAmbientLabEngineOptions.first;
                state[\engine] = selected[\key];
                setInfo.(selected[\key]);
                applyIfPlaying.();
            });
        makeInfoButton.(270, 37, state[\engine] ? \halo);

        StaticText(win, Rect(318, 38, 52, 18))
            .string_("Preset")
            .align_(\left);
        PopUpMenu(win, Rect(376, 36, 194, 24))
            .items_(presetLabels)
            .value_(~swAmbientLabIndexFor.(~swAmbientLabPresetOptions, state[\preset] ? \beautifulDefault))
            .action_({ |menu|
                var selected = ~swAmbientLabPresetOptions[menu.value] ? ~swAmbientLabPresetOptions.first;
                reopenWithState.(~swAmbientLabPresetFor.(selected[\key]));
            });

        makeSlider.("Root Hz", \root, ControlSpec(36, 120, \exp, 0.01, state[\root] ? 58), 78);
        makeSlider.("Level", \level, ControlSpec(0.0, 0.2, \lin, 0.001, state[\level] ? 0.12), 106);
        makeSlider.("Ambience", \ambience, ControlSpec(0, 0.9, \lin, 0.001, state[\ambience] ? 0.74), 134);
        makeSlider.("Rhythm", \rhythm, ControlSpec(0, 0.7, \lin, 0.001, state[\rhythm] ? 0.14), 162);
        makeSlider.("Overlay", \overlay, ControlSpec(0, 0.6, \lin, 0.001, state[\overlay] ? 0.04), 190);
        makeSlider.("Yaw Start", \yawOverlayStart, ControlSpec(0, 1, \lin, 0.001, state[\yawOverlayStart] ? 0.68), 218);
        makeSlider.("Yaw End", \yawOverlayEnd, ControlSpec(0, 1, \lin, 0.001, state[\yawOverlayEnd] ? 0.86), 246);
        makeSlider.("Pent Level", \pentatonicLevel, ControlSpec(0, 0.24, \lin, 0.001, state[\pentatonicLevel] ? 0.1), 274);

        Button(win, Rect(400, 78, 96, 28))
            .states_([["Play"]])
            .action_({
                state[\isPlaying] = true;
                ~swAmbientLabState = ~swAmbientLabCopyState.(state);
                ~swAmbientLabApplyCurrent.();
            });
        Button(win, Rect(508, 78, 84, 28))
            .states_([["Stop"]])
            .action_({
                state[\isPlaying] = false;
                ~swAmbientLabState = ~swAmbientLabCopyState.(state);
                ~swAmbientLabStop.();
            });
        Button(win, Rect(604, 78, 118, 28))
            .states_([["Promote Live"]])
            .action_({
                ~swAmbientLabState = ~swAmbientLabCopyState.(state);
                ~swAmbientLabPromoteToSmoothLive.();
                setInfo.(\overview);
            });
        Button(win, Rect(400, 116, 96, 28))
            .states_([["Reset"]])
            .action_({ reopenWithState.(~swAmbientLabPresetFor.(\beautifulDefault)); });
        Button(win, Rect(508, 116, 84, 28))
            .states_([["Info"]])
            .action_({ setInfo.(\overview); });
        Button(win, Rect(604, 116, 118, 28))
            .states_([["Pent Synth Off"], ["Pent Synth On"]])
            .value_((state[\pentatonicEnabled] ? false).if(1, 0))
            .action_({ |button|
                state[\pentatonicEnabled] = button.value == 1;
                setInfo.(\pentatonicSynth);
                if (state[\pentatonicEnabled]) {
                    if ((state[\pentatonicLevel] ? 0.0) < 0.1) {
                        state[\pentatonicLevel] = 0.1;
                    };
                    ~swSetMotionGate.(true);
                    state[\isPlaying] = true;
                    ~swAmbientLabState = ~swAmbientLabCopyState.(state);
                    ~swAmbientLabApplyCurrent.();
                } {
                    applyIfPlaying.();
                };
            });

        StaticText(win, Rect(12, 310, 220, 18))
            .string_("Manual motion")
            .align_(\left);

        ~swAmbientLabAxisOptions.do { |axisItem, index|
            makeSlider.(
                axisItem[\label],
                axisItem[\key],
                ControlSpec(axisItem[\min], axisItem[\max], \lin, 0.001, state[axisItem[\key]] ? 0.0),
                334 + (index * 28)
            );
            makeInfoButton.(380, 334 + (index * 28), axisItem[\key]);
        };

        StaticText(win, Rect(12, 508, 220, 18))
            .string_("Axis mapping")
            .align_(\left);
        StaticText(win, Rect(110, 508, 120, 18))
            .string_("Target")
            .align_(\left);
        StaticText(win, Rect(274, 508, 90, 18))
            .string_("Amount")
            .align_(\left);
        StaticText(win, Rect(442, 508, 70, 18))
            .string_("Curve")
            .align_(\left);

        ~swAmbientLabAxisOptions.do { |axisItem, index|
            makeMappingRow.(axisItem, 532 + (index * 30));
        };

        infoView = StaticText(win, Rect(12, 724, 724, 42))
            .string_(~swAmbientLabInfoFor.(\overview))
            .align_(\left);

        win.onClose_({
            ~swAmbientLabWindow = nil;
            if (stopOnClose) {
                ~swAmbientLabStop.();
            };
        });

        win.front;
    };

    ~swChoirTextureGUI = {
        var defaultParams;
        var win;
        var synthOptions = [
            (key: 0, label: "Cathedral"),
            (key: 1, label: "Flow"),
            (key: 2, label: "Ribbon"),
            (key: 3, label: "Glass"),
            (key: 4, label: "Vowel"),
            (key: 5, label: "Chime")
        ];
        var synthLabels = synthOptions.collect { |item| item[\label] };
        var params;
        var makeSlider;
        var makeToggle;
        var infoView;
        var updateInfo;
        var updateActivePreview;
        var applyToLiveParams;
        var valuesText;
        var printValues;
        var playRaw;
        var playChoirFx;
        var playSynthOnly;
        var playBlend;

        defaultParams = (
            choirAmp: (~swAuduinoParams[\smoothChoirAmp] ? 0.46).clip(0.0, 0.6),
            synthAmp: (~swAuduinoParams[\smoothChoirSynthAmp] ? 0.74).clip(0.0, 0.8),
            sampleStart: (~swAuduinoParams[\smoothChoirSampleStart] ? 0.0).clip(0.0, 5.0),
            synthMode: (~swAuduinoParams[\smoothChoirTexture] ? 4).asInteger.clip(0, 5),
            spread: (~swAuduinoParams[\smoothChoirSpread] ? 0.78).clip(0.1, 1.0),
            shimmer: (~swAuduinoParams[\smoothChoirShimmer] ? 0.18).clip(0.0, 1.0),
            breath: (~swAuduinoParams[\smoothChoirBreath] ? 0.08).clip(0.0, 0.2),
            brightness: (~swAuduinoParams[\smoothChoirBrightness] ? 0.34).clip(0.0, 1.0),
            choirFxMix: (~swAuduinoParams[\smoothChoirFxMix] ? 0.42).clip(0.0, 1.0),
            choirDelayMix: (~swAuduinoParams[\smoothChoirDelayMix] ? 0.22).clip(0.0, 1.0),
            choirAir: (~swAuduinoParams[\smoothChoirAir] ? 0.52).clip(0.0, 1.0),
            choirBassAmp: (~swAuduinoParams[\smoothChoirBassAmp] ? 0.16).clip(0.0, 0.45),
            choirBassTone: (~swAuduinoParams[\smoothChoirBassTone] ? 0.42).clip(0.0, 1.0),
            fadeIn: (~swAuduinoParams[\smoothChoirFadeIn] ? 0.5).clip(0.01, 1.5),
            holdTime: (~swAuduinoParams[\smoothChoirHold] ? 0.28).clip(0.0, 3.0),
            fadeOut: (~swAuduinoParams[\smoothChoirFadeOut] ? 4.2).clip(0.2, 9.0),
            octaveDown3: (~swAuduinoParams[\smoothChoirOctaveDown3] ? 0).asInteger,
            octaveDown2: (~swAuduinoParams[\smoothChoirOctaveDown2] ? 1).asInteger,
            octaveDown1: (~swAuduinoParams[\smoothChoirOctaveDown1] ? 1).asInteger,
            octaveUp1: (~swAuduinoParams[\smoothChoirOctaveUp1] ? 0).asInteger,
            octaveUp2: (~swAuduinoParams[\smoothChoirOctaveUp2] ? 0).asInteger,
            octaveUp3: (~swAuduinoParams[\smoothChoirOctaveUp3] ? 0).asInteger
        );
        params = defaultParams.copy.putAll(~swChoirBlendParams ? ());

        if (~swChoirTextureWindow.notNil) {
            ~swChoirTextureWindow.close;
        };

        valuesText = {
            "smoothChoirAmp: " ++ params[\choirAmp].round(0.001).asString ++ ",\n"
            ++ "smoothChoirSynthAmp: " ++ params[\synthAmp].round(0.001).asString ++ ",\n"
            ++ "smoothChoirSpread: " ++ params[\spread].round(0.001).asString ++ ",\n"
            ++ "smoothChoirShimmer: " ++ params[\shimmer].round(0.001).asString ++ ",\n"
            ++ "smoothChoirBreath: " ++ params[\breath].round(0.001).asString ++ ",\n"
            ++ "smoothChoirBrightness: " ++ params[\brightness].round(0.001).asString ++ ",\n"
            ++ "smoothChoirFxMix: " ++ params[\choirFxMix].round(0.001).asString ++ ",\n"
            ++ "smoothChoirDelayMix: " ++ params[\choirDelayMix].round(0.001).asString ++ ",\n"
            ++ "smoothChoirAir: " ++ params[\choirAir].round(0.001).asString ++ ",\n"
            ++ "smoothChoirBassAmp: " ++ params[\choirBassAmp].round(0.001).asString ++ ",\n"
            ++ "smoothChoirBassTone: " ++ params[\choirBassTone].round(0.001).asString ++ ",\n"
            ++ "smoothChoirSampleStart: " ++ params[\sampleStart].round(0.001).asString ++ ",\n"
            ++ "smoothChoirTexture: " ++ params[\synthMode].asInteger.asString ++ ",\n"
            ++ "smoothChoirFadeIn: " ++ params[\fadeIn].round(0.001).asString ++ ",\n"
            ++ "smoothChoirHold: " ++ params[\holdTime].round(0.001).asString ++ ",\n"
            ++ "smoothChoirFadeOut: " ++ params[\fadeOut].round(0.001).asString ++ ",\n"
            ++ "smoothChoirOctaveDown3: " ++ params[\octaveDown3].asInteger.asString ++ ",\n"
            ++ "smoothChoirOctaveDown2: " ++ params[\octaveDown2].asInteger.asString ++ ",\n"
            ++ "smoothChoirOctaveDown1: " ++ params[\octaveDown1].asInteger.asString ++ ",\n"
            ++ "smoothChoirOctaveUp1: " ++ params[\octaveUp1].asInteger.asString ++ ",\n"
            ++ "smoothChoirOctaveUp2: " ++ params[\octaveUp2].asInteger.asString ++ ",\n"
            ++ "smoothChoirOctaveUp3: " ++ params[\octaveUp3].asInteger.asString;
        };

        updateInfo = { |message=nil|
            var sourceSummary = (~swChoirHasSamples ? false).if(
                "Sample-backed choir loaded.",
                "No sample loaded; using synth fallback choir."
            );

            infoView.string_(
                message ? (sourceSummary ++ " Move sliders while a preview is playing; use Print Values when you find a good setting.")
            );
        };

        updateActivePreview = {
            if (~swChoirRawPreview.notNil) {
                ~swChoirRawPreview.set(
                    \amp, params[\choirAmp],
                    \fxMix, params[\choirFxMix],
                    \delayMix, params[\choirDelayMix],
                    \air, params[\choirAir],
                    \brightness, params[\brightness],
                    \spread, params[\spread]
                );
            };
            if (~swChoirBlendPreview.notNil) {
                ~swChoirBlendPreview.set(
                    \synthAmp, params[\synthAmp],
                    \spread, params[\spread],
                    \shimmer, params[\shimmer],
                    \breath, params[\breath],
                    \brightness, params[\brightness],
                    \hallMixCtl, params[\choirFxMix],
                    \hallRoomCtl, (0.42 + (params[\choirFxMix] * 0.38)).clip(0.0, 1.0),
                    \hallDampCtl, (0.58 - (params[\brightness] * 0.22)).clip(0.0, 1.0),
                    \bassAmp, params[\choirBassAmp],
                    \bassTone, params[\choirBassTone],
                    \octaveDown3, params[\octaveDown3],
                    \octaveDown2, params[\octaveDown2],
                    \octaveDown1, params[\octaveDown1],
                    \octaveUp1, params[\octaveUp1],
                    \octaveUp2, params[\octaveUp2],
                    \octaveUp3, params[\octaveUp3]
                );
            };
            ~swChoirBlendParams = params.copy;
        };

        applyToLiveParams = {
            var live = (~swAuduinoParams ? ~swAuduinoDefaultParams.copy).copy;
            live[\smoothChoirAmp] = params[\choirAmp];
            live[\smoothChoirSynthAmp] = params[\synthAmp];
            live[\smoothChoirSpread] = params[\spread];
            live[\smoothChoirShimmer] = params[\shimmer];
            live[\smoothChoirBreath] = params[\breath];
            live[\smoothChoirBrightness] = params[\brightness];
            live[\smoothChoirFxMix] = params[\choirFxMix];
            live[\smoothChoirDelayMix] = params[\choirDelayMix];
            live[\smoothChoirAir] = params[\choirAir];
            live[\smoothChoirBassAmp] = params[\choirBassAmp];
            live[\smoothChoirBassTone] = params[\choirBassTone];
            live[\smoothChoirSampleStart] = params[\sampleStart];
            live[\smoothChoirTexture] = params[\synthMode];
            live[\smoothChoirFadeIn] = params[\fadeIn];
            live[\smoothChoirHold] = params[\holdTime];
            live[\smoothChoirFadeOut] = params[\fadeOut];
            live[\smoothChoirOctaveDown3] = params[\octaveDown3];
            live[\smoothChoirOctaveDown2] = params[\octaveDown2];
            live[\smoothChoirOctaveDown1] = params[\octaveDown1];
            live[\smoothChoirOctaveUp1] = params[\octaveUp1];
            live[\smoothChoirOctaveUp2] = params[\octaveUp2];
            live[\smoothChoirOctaveUp3] = params[\octaveUp3];
            ~swAuduinoParams = live;
            ~swSmoothChoirApplyCurrent.();
            updateInfo.("Applied to current live choir params.\n" ++ valuesText.());
        };

        printValues = {
            var text = valuesText.();
            ("Choir preset values:\n" ++ text).postln;
            updateInfo.(text);
        };

        playRaw = {
            ~swChoirBlendParams = params.copy;
            ~swChoirRawDemoPlay.(
                amp: params[\choirAmp],
                sampleStart: params[\sampleStart]
            );
            updateInfo.("Original sample only. Blend/effects sliders do not affect this raw preview.");
        };

        playChoirFx = {
            ~swChoirBlendParams = params.copy;
            ~swChoirTextureDemoPlay.(
                choirAmp: params[\choirAmp],
                synthAmp: 0.0,
                sampleStart: params[\sampleStart],
                synthMode: params[\synthMode],
                spread: params[\spread],
                shimmer: params[\shimmer],
                breath: params[\breath],
                brightness: params[\brightness],
                choirFxMix: params[\choirFxMix],
                choirDelayMix: params[\choirDelayMix],
                choirAir: params[\choirAir],
                choirBassAmp: 0.0,
                choirBassTone: params[\choirBassTone],
                octaveDown3: 0,
                octaveDown2: 0,
                octaveDown1: 0,
                octaveUp1: 0,
                octaveUp2: 0,
                octaveUp3: 0
            );
            updateInfo.("Choir sample with effects only.");
        };

        playSynthOnly = {
            ~swChoirBlendParams = params.copy;
            ~swChoirTextureDemoPlay.(
                choirAmp: 0.0,
                synthAmp: params[\synthAmp],
                sampleStart: params[\sampleStart],
                synthMode: params[\synthMode],
                spread: params[\spread],
                shimmer: params[\shimmer],
                breath: params[\breath],
                brightness: params[\brightness],
                choirFxMix: params[\choirFxMix],
                choirDelayMix: params[\choirDelayMix],
                choirAir: params[\choirAir],
                choirBassAmp: 0.0,
                choirBassTone: params[\choirBassTone],
                octaveDown3: params[\octaveDown3],
                octaveDown2: params[\octaveDown2],
                octaveDown1: params[\octaveDown1],
                octaveUp1: params[\octaveUp1],
                octaveUp2: params[\octaveUp2],
                octaveUp3: params[\octaveUp3]
            );
            updateInfo.("Synth only. Choir sample and soft bass are muted.");
        };

        playBlend = {
            ~swChoirBlendParams = params.copy;
            ~swChoirTextureDemoPlay.(
                choirAmp: params[\choirAmp],
                synthAmp: params[\synthAmp],
                sampleStart: params[\sampleStart],
                synthMode: params[\synthMode],
                spread: params[\spread],
                shimmer: params[\shimmer],
                breath: params[\breath],
                brightness: params[\brightness],
                choirFxMix: params[\choirFxMix],
                choirDelayMix: params[\choirDelayMix],
                choirAir: params[\choirAir],
                choirBassAmp: params[\choirBassAmp],
                choirBassTone: params[\choirBassTone],
                octaveDown3: params[\octaveDown3],
                octaveDown2: params[\octaveDown2],
                octaveDown1: params[\octaveDown1],
                octaveUp1: params[\octaveUp1],
                octaveUp2: params[\octaveUp2],
                octaveUp3: params[\octaveUp3]
            );
            updateInfo.();
        };

        win = Window("SpaceWizard Choir Lab", Rect(140, 40, 680, 830));
        win.alwaysOnTop_(true);
        ~swChoirTextureWindow = win;

        StaticText(win, Rect(12, 8, 656, 34))
            .string_("Tune the choir sample effects, the added synth voice, octave layers, and soft bass support. Sliders update a playing preview where possible.")
            .align_(\left);

        StaticText(win, Rect(12, 50, 72, 18))
            .string_("Synth")
            .align_(\left);

        PopUpMenu(win, Rect(84, 48, 200, 24))
            .items_(synthLabels)
            .value_(params[\synthMode].asInteger)
            .action_({ |menu|
                var selected = synthOptions[menu.value] ? synthOptions.first;
                params[\synthMode] = selected[\key];
                updateInfo.("Synth texture changed. Replay the blend to hear the new synth model.");
            });

        makeSlider = { |label, key, spec, top|
            EZSlider(
                win,
                Rect(12, top, 656, 24),
                label,
                spec,
                { |view|
                    params[key] = view.value;
                    updateActivePreview.();
                },
                params[key],
                layout: 'horz'
            );
        };

        makeToggle = { |label, key, left, top|
            Button(win, Rect(left, top, 102, 28))
                .states_([
                    [label ++ " off"],
                    [label ++ " on"]
                ])
                .value_(params[key].asInteger)
                .action_({ |button|
                    params[key] = button.value.asInteger;
                    updateActivePreview.();
                });
        };

        makeSlider.("Choir Level", \choirAmp, ControlSpec(0.0, 0.6, \lin, 0.001, params[\choirAmp]), 84);
        makeSlider.("Synth Level", \synthAmp, ControlSpec(0.0, 0.8, \lin, 0.001, params[\synthAmp]), 112);
        makeSlider.("Soft Bass", \choirBassAmp, ControlSpec(0.0, 0.45, \lin, 0.001, params[\choirBassAmp]), 140);
        makeSlider.("Bass Tone", \choirBassTone, ControlSpec(0.0, 1.0, \lin, 0.001, params[\choirBassTone]), 168);

        makeSlider.("Spread", \spread, ControlSpec(0.1, 1.0, \lin, 0.001, params[\spread]), 208);
        makeSlider.("Shimmer", \shimmer, ControlSpec(0.0, 1.0, \lin, 0.001, params[\shimmer]), 236);
        makeSlider.("Breath", \breath, ControlSpec(0.0, 0.2, \lin, 0.001, params[\breath]), 264);
        makeSlider.("Brightness", \brightness, ControlSpec(0.0, 1.0, \lin, 0.001, params[\brightness]), 292);
        makeSlider.("Choir Reverb", \choirFxMix, ControlSpec(0.0, 1.0, \lin, 0.001, params[\choirFxMix]), 320);
        makeSlider.("Choir Echo", \choirDelayMix, ControlSpec(0.0, 1.0, \lin, 0.001, params[\choirDelayMix]), 348);
        makeSlider.("Choir Air", \choirAir, ControlSpec(0.0, 1.0, \lin, 0.001, params[\choirAir]), 376);

        makeSlider.("Sample Start", \sampleStart, ControlSpec(0.0, 4.5, \lin, 0.01, params[\sampleStart]), 416);
        makeSlider.("Fade In", \fadeIn, ControlSpec(0.01, 1.5, \lin, 0.01, params[\fadeIn]), 444);
        makeSlider.("Hold", \holdTime, ControlSpec(0.0, 3.0, \lin, 0.01, params[\holdTime]), 472);
        makeSlider.("Fade Out", \fadeOut, ControlSpec(0.2, 9.0, \lin, 0.01, params[\fadeOut]), 500);

        StaticText(win, Rect(12, 536, 656, 18))
            .string_("Synth octave layers")
            .align_(\left);

        makeToggle.("-3 oct", \octaveDown3, 12, 560);
        makeToggle.("-2 oct", \octaveDown2, 124, 560);
        makeToggle.("-1 oct", \octaveDown1, 236, 560);
        makeToggle.("+1 oct", \octaveUp1, 348, 560);
        makeToggle.("+2 oct", \octaveUp2, 460, 560);
        makeToggle.("+3 oct", \octaveUp3, 572, 560);

        Button(win, Rect(12, 604, 122, 34))
            .states_([["Original"]])
            .action_({ playRaw.(); });

        Button(win, Rect(144, 604, 122, 34))
            .states_([["Choir FX"]])
            .action_({ playChoirFx.(); });

        Button(win, Rect(276, 604, 122, 34))
            .states_([["Synth Only"]])
            .action_({ playSynthOnly.(); });

        Button(win, Rect(408, 604, 122, 34))
            .states_([["Full Blend"]])
            .action_({ playBlend.(); });

        Button(win, Rect(542, 604, 82, 34))
            .states_([["Stop"]])
            .action_({ ~swChoirTextureDemoStop.(); });

        Button(win, Rect(12, 648, 150, 30))
            .states_([["Replay Blend"]])
            .action_({ playBlend.(); });

        Button(win, Rect(174, 648, 150, 30))
            .states_([["Apply Live"]])
            .action_({ applyToLiveParams.(); });

        Button(win, Rect(336, 648, 150, 30))
            .states_([["Print Values"]])
            .action_({ printValues.(); });

        infoView = StaticText(win, Rect(12, 690, 656, 130))
            .align_(\left);
        updateInfo.();

        win.onClose_({
            ~swChoirTextureWindow = nil;
            ~swChoirTextureDemoStop.();
        });
        win.front;
    };

    ~swImpactDrum = { |jerk=0.0, deviation=0.0, activity=0.0|
        var params = ~swAuduinoParams ? ~swAuduinoDefaultParams.copy;
        var enabled = params[\smoothImpactDrumEnabled];
        var maxJerk = (params[\smoothImpactDrumMaxJerk] ? 60.0).max(1.0);
        var force = jerk.asFloat.linlin(0.0, maxJerk, 0.35, 1.0).clip(0.35, 1.2);
        var activityBoost = activity.asFloat.clip(0.0, 1.0).linlin(0.0, 1.0, 0.86, 1.1);
        var deviationBoost = deviation.asFloat.clip(0.0, 1.0).linlin(0.0, 1.0, 0.9, 1.12);

        if (enabled.isNil) {
            enabled = true;
        };

        if (enabled) {
            Synth(\spacewizardAmbientDrum, [
                \amp, (params[\smoothImpactDrumAmp] ? 0.16).clip(0.0, 0.6),
                \freq, (params[\smoothImpactDrumFreq] ? 74).clip(32.0, 180.0),
                \decay, (params[\smoothImpactDrumDecay] ? 1.6).clip(0.25, 4.5),
                \tone, (params[\smoothImpactDrumTone] ? 0.42).clip(0.0, 1.0),
                \verb, (params[\smoothImpactDrumVerb] ? 0.72).clip(0.0, 0.95),
                \force, (force * activityBoost * deviationBoost).clip(0.25, 1.35)
            ]);
        };
    };

    OSCdef(\swPickupTrigger, { |msg|
        var trig = msg[1] ? 0;
        if (trig > 0) {
            ~swSetMotionGate.(true);
            ~swPickup.();
        } {
            ~swRest.();
        };
    }, '/spacewizard/pickup');

    OSCdef(\swImpactDrum, { |msg|
        var trig = msg[1] ? 0;
        if (trig > 0) {
            ~swImpactDrum.(msg[2] ? 0.0, msg[3] ? 0.0, msg[4] ? 0.0);
        };
    }, '/spacewizard/impact');

    OSCdef(\swPing, { |msg|
        var freq = (msg[1] ? 880).clip(80.0, 2000.0);
        var amp = (msg[2] ? 0.14).clip(0.0, 0.4);
        var ping = Synth(\default, [\freq, freq, \amp, amp]);
        ("SpaceWizard OSC ping freq=" ++ freq.asString ++ " amp=" ++ amp.asString).postln;
        AppClock.sched(0.5, {
            if (ping.notNil) {
                ping.set(\gate, 0);
            };
            nil;
        });
    }, '/spacewizard/ping');

    OSCdef(\swDebugControl, { |msg|
        var enabled = (msg[1] ? 1) > 0;
        ~swLiveOscDebugEnabled = enabled;
        ("SpaceWizard live OSC debug " ++ enabled.asString).postln;
    }, '/spacewizard/debug');

    OSCdef(\swSmoothProbe, { |msg|
        var freq = (msg[1] ? 110).clip(40.0, 600.0);
        var amp = (msg[2] ? 0.32).clip(0.0, 0.6);
        var seconds = (msg[3] ? 4.0).clip(0.5, 20.0);
        var bus = Bus.control(s, 1);
        var voice;

        bus.set(1.0);

        voice = Synth(\spacewizardSmoothSineMono, [
            \freq, freq,
            \amp, amp,
            \pan, 0.0,
            \vibratoRate, 0.3,
            \vibratoDepth, 0.002,
            \motionDepth, 0.02,
            \bloom, 0.12,
            \lp, 2600,
            \glide, 0.02,
            \crunch, 0.0,
            \reverb, 0.18,
            \bassClarity, 0.0,
            \motionBus, bus.index,
            \motionRise, 0.01,
            \motionFall, 1.0
        ]);
        ("SpaceWizard smooth probe freq=" ++ freq.asString
            ++ " amp=" ++ amp.asString
            ++ " seconds=" ++ seconds.asString
            ++ " bus=" ++ bus.index.asString).postln;
        AppClock.sched(seconds, {
            if (voice.notNil) {
                voice.set(\gate, 0);
            };
            if (bus.notNil) {
                bus.free;
            };
            nil;
        });
    }, '/spacewizard/smooth_probe');

    OSCdef(\swGate, { |msg|
        var gate = (msg[1] ? 0).clip(0.0, 1.0);
        ~swSetMotionGate.(gate > 0.0);
        if (gate > 0.0) {
            ~swSet.(activity: gate.max(0.1));
        } {
            ~swRest.();
        };
    }, '/spacewizard/gate');

    OSCdef(\swMotion, { |msg|
        ~swSet.(
            activity: (msg[1] ? 0.0).clip(0.0, 1.0),
            tilt: (msg[2] ? 0.0).clip(-1.0, 1.0),
            roll: (msg[3] ? 0.0).clip(-1.0, 1.0),
            spin: (msg[4] ? 0.0).clip(0.0, 1.0),
            presence: (msg[5] ? 0.55).clip(0.0, 1.0),
            amp: (msg[6] ? 0.18).clip(0.0, 0.4)
        );
    }, '/spacewizard/motion');

    ~swUpdateLiveInputState = { |gate=0.0, activity=0.0, pitchNorm=0.0, rollNorm=0.0, spinNorm=0.0, yawNorm=0.0|
        ~swLiveInputState = ~swLiveInputState.copy.putAll((
            gate: gate,
            activity: activity,
            pitchNorm: pitchNorm,
            pitchDeg: pitchNorm * 90.0,
            rollNorm: rollNorm,
            rollDeg: rollNorm * 180.0,
            spinNorm: spinNorm,
            yawNorm: yawNorm,
            yawDeg: yawNorm * 360.0
        ));

        ~swRequestLiveStatusGUIRefresh.();
    };



    OSCdef(\swLiveStatus, { |msg|
        var gate = (msg[1] ? 0.0).clip(0.0, 1.0);
        var activity = (msg[2] ? 0.0).clip(0.0, 1.0);
        var pitchNorm = (msg[3] ? 0.0).clip(-1.0, 1.0);
        var rollNorm = (msg[4] ? 0.0).clip(-1.0, 1.0);
        var spinNorm = (msg[5] ? 0.0).clip(0.0, 1.0);
        var yawNorm = (msg[6] ? 0.0).clip(0.0, 1.0);

        ~swUpdateLiveInputState.(gate, activity, pitchNorm, rollNorm, spinNorm, yawNorm);
    }, '/spacewizard/live_status');

    OSCdef(\swAuduinoControl, { |msg|
        var gate = (msg[1] ? 0.0).clip(0.0, 1.0);
        var activity = (msg[2] ? 0.0).clip(0.0, 1.0);
        var pitchNorm = (msg[3] ? 0.0).clip(-1.0, 1.0);
        var rollNorm = (msg[4] ? 0.0).clip(-1.0, 1.0);
        var spinNorm = (msg[5] ? 0.0).clip(0.0, 1.0);
        var yawNorm = (msg[6] ? 0.0).clip(0.0, 1.0);
        var params = (~swAuduinoParams ? ~swAuduinoDefaultParams.copy).copy;
        var voiceMode = params[\voiceMode] ? \auduino;
        var playMode = params[\playMode] ? \continuous;
        var rollShape = rollNorm.abs;
        var yawCentered = yawNorm.linlin(0.0, 1.0, -1.0, 1.0);
        var yawShape = yawCentered.abs;
        var blendState;
        var tone;

        // The optional seventh value distinguishes missing input from genuine rest.
        // Six-value packets remain compatible with existing bridge/test senders.
        ~swLastLiveControlAt = Main.elapsedTime;
        ~swLiveInputValid = (msg[7] ? 1.0) > 0.0;
        if (~swLiveInputValid.not) { gate = 0.0; activity = 0.0; spinNorm = 0.0; };

        if ((~swLiveOscDebugEnabled ? false) and: { (Main.elapsedTime - (~swAuduinoOscDebugAt ? 0.0)) > 0.5 }) {
            ("Auduino OSC recv gate=" ++ gate.round(0.001).asString
                ++ " activity=" ++ activity.round(0.001).asString
                ++ " pitch=" ++ pitchNorm.round(0.001).asString
                ++ " roll=" ++ rollNorm.round(0.001).asString
                ++ " spin=" ++ spinNorm.round(0.001).asString
                ++ " mode=" ++ voiceMode.asString
                ++ " exp=" ++ (params[\smoothExperiment] ? \none).asString).postln;
            ~swAuduinoOscDebugAt = Main.elapsedTime;
        };

        ~swUpdateLiveInputState.(gate, activity, pitchNorm, rollNorm, spinNorm, yawNorm);
        if ((~swLiveOscDebugEnabled ? false) and: { (Main.elapsedTime - (~swAuduinoFlowDebugAt ? 0.0)) > 0.5 }) {
            "Auduino after live input update".postln;
        };

        if (voiceMode == \song) {
            // Regular gate: lights/stand prompt semantics and cleanup. Song gate (msg[8]):
            // the less sensitive on/off that actually starts and stops the song.
            ~swSetMotionGate.(gate > 0.0);
            ~swSongSetEnergy.(if (~swLiveInputValid) { msg[9] ? 1.0 } { 0.0 });
            ~swSongSetGate.((gate > 0.0) and: { (msg[8] ? gate) > 0.0 });
            ~swAuduinoParams = params;
        } {
        if (voiceMode == \smoothSine) {
            if ((~swLiveOscDebugEnabled ? false) and: { (Main.elapsedTime - (~swAuduinoFlowDebugAt ? 0.0)) > 0.5 }) {
                "Auduino entering smoothSine branch".postln;
            };
            ~swSetMotionGate.(gate > 0.0);
            if ((~swLiveOscDebugEnabled ? false) and: { (Main.elapsedTime - (~swAuduinoFlowDebugAt ? 0.0)) > 0.5 }) {
                "Auduino after motion gate".postln;
                ~swAuduinoFlowDebugAt = Main.elapsedTime;
            };
            params[\smoothPitchNorm] = pitchNorm;
            params[\smoothExperiment] = params[\smoothExperiment] ? \triggeredNotes;
            ~swAuduinoParams = params;

            if (gate > 0.0) {
                params[\smoothMidi] = ~swSmoothMidiForPitch.(pitchNorm, params[\smoothRootMidi] ? 38);
                params[\smoothFreq] = (params[\smoothMidi] ? 43).midicps;
                params[\smoothVibratoRate] = spinNorm.linexp(0.0, 1.0, 0.05, 1.9) * yawNorm.linexp(0.0, 1.0, 0.88, 1.42);
                params[\smoothVibratoDepth] = spinNorm.linlin(0.0, 1.0, 0.0012, 0.0085) + (activity * 0.0022) + (yawShape * 0.0018);
                params[\smoothMotionDepth] = activity.linlin(0.0, 1.0, 0.02, 0.1) + (yawShape * 0.03);
                params[\smoothBloom] = rollShape.linlin(0.0, 1.0, 0.07, 0.18) + (yawShape * 0.08);
                params[\smoothLp] = rollNorm.linexp(-1.0, 1.0, 480.0, 2600.0);
                tone = ~swSmoothRollToneFor.(params, rollShape);
                params[\smoothCrunch] = tone[\crunch];
                params[\smoothReverb] = tone[\reverb];
                params[\smoothHold] = activity.linlin(0.0, 1.0, 0.04, 0.11) + (yawShape * 0.025);
                params[\smoothDecay] = ~swSmoothLiveDecayFor.(params, activity, yawNorm);
                params[\pan] = (((rollNorm * 0.72) + (yawCentered * 0.28)).clip(-1.0, 1.0)).linlin(-1.0, 1.0, -0.26, 0.26);
                params[\amp] = ~swSmoothLiveAmpFor.(params, activity);
                if (params[\smoothChoirSolo] ? false) {
                    ~swSmoothStop.();
                    ~swAmbientStop.();
                    ~swSmoothChoirApplyMotion.(params, pitchNorm, rollNorm, yawNorm, gate);
                    ~swAuduinoParams = params;
                } {
                    params = ~swSmoothAmbientApplyMotion.(params, activity, pitchNorm, rollNorm, spinNorm, yawNorm, gate);
                    ~swSmoothChoirApplyMotion.(params, pitchNorm, rollNorm, yawNorm, gate);
                    ~swAuduinoParams = params;
                    ~swSmoothApplyActiveTone.(params);

                    if ((~swLiveOscDebugEnabled ? false) and: { (Main.elapsedTime - (~swSmoothBranchDebugAt ? 0.0)) > 0.5 }) {
                        ("Smooth branch exp=" ++ (params[\smoothExperiment] ? \none).asString
                            ++ " ampBase=" ++ (params[\amp] ? 0.0).round(0.001).asString
                            ++ " ampScale=" ++ (params[\smoothAmpScale] ? 1.0).round(0.001).asString
                            ++ " layerOpen=" ++ (~swSmoothLayerGateOpen ? false).asString).postln;
                        ~swSmoothBranchDebugAt = Main.elapsedTime;
                    };

                    if ((params[\smoothExperiment] ? \triggeredNotes) == \triggeredNotes) {
                        ~swSmoothMaybeTrigger.(params);
                    } {
                        if ((params[\smoothExperiment] ? \triggeredNotes) == \trackedMono) {
                            ~swSmoothMonoTracked.(params);
                        } {
                            ~swSmoothMonoDirect.(params);
                        };
                    };
                };
            } {
                ~swAuduinoParams = params;
                ~swSmoothChoirApplyMotion.(params, pitchNorm, rollNorm, yawNorm, gate);
            };
        } {
            ~swSetMotionGate.(gate > 0.0);
            if (voiceMode == \ambientFlow) {
                if (gate > 0.0) {
                    if (params[\ambientYawBlendEnabled] ? false) {
                        params = ~swAmbientApplyMotionBlend.(params, activity, pitchNorm, rollNorm, spinNorm, yawNorm);
                    } {
                        params = ~swAmbientApplyMotionSingle.(params, activity, pitchNorm, rollNorm, spinNorm);
                    };
                } {
                    if (params[\ambientYawBlendEnabled] ? false) {
                        blendState = ~swAmbientBlendStateFor.(((params[\ambientYawOffset] ? 0.0) + yawNorm).wrap(0.0, 1.0));
                        params = params.copy.putAll((
                            ambientBlendPosition: blendState[\position],
                            ambientBlendPrimaryVariant: blendState[\primaryVariant],
                            ambientBlendSecondaryVariant: blendState[\secondaryVariant],
                            ambientBlendMix: blendState[\secondaryWeight]
                        ));
                    };
                };
                ~swAuduinoParams = params;
            } {
                if (voiceMode == \reeseBass) {
                    params[\reeseFreq] = pitchNorm.linexp(-1.0, 1.0, 30.0, 118.0);
                    params[\reeseDetune] = rollShape.linlin(0.0, 1.0, 0.012, 0.072);
                    params[\reeseGlide] = activity.linlin(0.0, 1.0, 0.46, 0.12);
                    params[\reeseWobbleRate] = spinNorm.linexp(0.0, 1.0, 0.04, 6.5);
                    params[\reeseWobbleDepth] = activity.linlin(0.0, 1.0, 0.05, 0.24) + (spinNorm * 0.08);
                    params[\reeseDrive] = rollShape.linlin(0.0, 1.0, 2.0, 5.0);
                    params[\reeseLp] = rollNorm.linexp(-1.0, 1.0, 260.0, 2200.0);
                    params[\reeseRq] = activity.linlin(0.0, 1.0, 0.2, 0.46);
                    params[\reeseSubMix] = rollShape.linlin(0.0, 1.0, 0.54, 0.28);
                    params[\pan] = rollNorm.linlin(-1.0, 1.0, -0.16, 0.16);
                    params[\amp] = activity.linlin(0.0, 1.0, 0.055, 0.17);
                } {
                    if (voiceMode == \wowVoice) {
                        var prevPitchNorm = ~swWowPrevPitchNorm ? pitchNorm;
                        var prevDirection = ~swWowPrevDirection ? 0;
                        var delta = pitchNorm - prevPitchNorm;
                        var velocity = delta.abs;
                        var velocityThreshold = 0.012;
                        var directionThreshold = 0.008;
                        var direction = (velocity >= directionThreshold).if(delta.sign, prevDirection);
                        var directionChanged = (direction != 0) and: { direction != prevDirection };
                        var velocityBurst = (velocity >= velocityThreshold) and: { (~swWowPrevVelocity ? 0.0) < velocityThreshold };
                        var triggerWow = false;
                        var cooldown = ~swWowTriggerCooldown ? 0;

                        if (cooldown > 0) {
                            ~swWowTriggerCooldown = cooldown - 1;
                        } {
                            if (directionChanged or: { velocityBurst }) {
                                triggerWow = true;
                                ~swWowTriggerCooldown = 10;
                            };
                        };

                        if (direction == 0) {
                            direction = (prevDirection == 0).if(1, prevDirection);
                        };

                        params[\wowFreq] = pitchNorm.linexp(-1.0, 1.0, 82.0, 220.0);
                        params[\wowGlide] = activity.linlin(0.0, 1.0, 0.42, 0.14);
                        params[\wowDirection] = direction;
                        params[\wowVowelDepth] = activity.linlin(0.0, 1.0, 0.58, 1.0) + (velocity * 2.5);
                        params[\wowBreath] = spinNorm.linlin(0.0, 1.0, 0.015, 0.075);
                        params[\wowLp] = rollNorm.linexp(-1.0, 1.0, 1200.0, 3600.0);
                        params[\wowTone] = rollShape.linlin(0.0, 1.0, 0.08, 0.28);
                        params[\pan] = rollNorm.linlin(-1.0, 1.0, -0.18, 0.18);
                        params[\amp] = activity.linlin(0.0, 1.0, 0.05, 0.16);

                        ~swWowPrevPitchNorm = pitchNorm;
                        ~swWowPrevDirection = direction;
                        ~swWowPrevVelocity = velocity;
                        ~swAuduinoParams = params;

                        if (gate > 0.0) {
                            ~swWowSet.(
                                freq: params[\wowFreq],
                                amp: params[\amp],
                                pan: params[\pan],
                                glide: params[\wowGlide],
                                direction: params[\wowDirection],
                                vowelDepth: params[\wowVowelDepth].clip(0.3, 1.15),
                                breath: params[\wowBreath],
                                lp: params[\wowLp],
                                tone: params[\wowTone],
                                trigger: triggerWow
                            );
                        };
                    } {
                        if ((playMode == \continuous) or: { playMode == \melodicContinuous }) {
                            playMode = \stepped;
                        };

                        params[\playMode] = playMode;
                        params[\syncFreq] = pitchNorm.linexp(-1.0, 1.0, ~swAuduinoStepSyncMinHz, ~swAuduinoStepSyncMaxHz);
                        params[\drive] = rollShape.linlin(0.0, 1.0, 2.0, 5.6);
                        params[\grain2Freq] = 294 * rollNorm.linexp(-1.0, 1.0, 0.62, 2.2);
                        params[\crushBits] = rollShape.linlin(0.0, 1.0, 9.0, 5.0).round.clip(5, 10);
                        params[\lp] = rollNorm.linexp(-1.0, 1.0, 1500.0, 9000.0);
                        params[\hp] = rollShape.linlin(0.0, 1.0, 45.0, 240.0);
                        params[\pan] = rollNorm.linlin(-1.0, 1.0, -0.18, 0.18);
                        params[\decay1] = activity.linlin(0.0, 1.0, 0.08, 0.22) + (spinNorm * 0.08) + (rollShape * 0.04);
                        params[\decay2] = activity.linlin(0.0, 1.0, 0.05, 0.16) + (spinNorm * 0.06) + (rollShape * 0.03);
                        params[\amp] = activity.linlin(0.0, 1.0, 0.07, 0.18);
                    };
                };
            };
        };
        if ((voiceMode != \wowVoice) and: { voiceMode != \ambientFlow } and: { voiceMode != \smoothSine }) {
            ~swAuduinoParams = params;

            if (gate > 0.0) {
                ~swApplyLiveModeCurrent.();
            };
        };
        };
        if (~swLiveInputValid) {
            ~swStandPromptUpdate.(params, gate, activity.max(spinNorm), pitchNorm);
        } {
            ~swExpireLiveInput.();
        };
    }, '/spacewizard/auduino');

    OSCdef(\swSmoothDemoControl, { |msg|
        var gate = (msg[1] ? 0.0).clip(0.0, 1.0);
        var activity = (msg[2] ? 0.0).clip(0.0, 1.0);
        var pitchNorm = (msg[3] ? 0.0).clip(-1.0, 1.0);
        var rollNorm = (msg[4] ? 0.0).clip(-1.0, 1.0);
        var spinNorm = (msg[5] ? 0.0).clip(0.0, 1.0);
        var yawNorm = (msg[6] ? 0.0).clip(0.0, 1.0);
        var params = (~swAuduinoParams ? ~swAuduinoDefaultParams.copy).copy;
        var didArm = false;
        var rollShape = rollNorm.abs;
        var yawCentered = yawNorm.linlin(0.0, 1.0, -1.0, 1.0);
        var yawShape = yawCentered.abs;
        var tone;

        if ((~swLiveOscDebugEnabled ? false) and: { (Main.elapsedTime - (~swSmoothDemoOscDebugAt ? 0.0)) > 0.5 }) {
            ("Smooth demo OSC recv gate=" ++ gate.round(0.001).asString
                ++ " activity=" ++ activity.round(0.001).asString
                ++ " pitch=" ++ pitchNorm.round(0.001).asString
                ++ " roll=" ++ rollNorm.round(0.001).asString
                ++ " spin=" ++ spinNorm.round(0.001).asString).postln;
            ~swSmoothDemoOscDebugAt = Main.elapsedTime;
        };

        ~swUpdateLiveInputState.(gate, activity, pitchNorm, rollNorm, spinNorm, yawNorm);

        if ((params[\voiceMode] ? \auduino) != \smoothSine) {
            params[\voiceMode] = \smoothSine;
            ~swAuduinoParams = params;
            ~swApplyLiveModeCurrent.();
            didArm = true;
        };

        ~swSetMotionGate.(gate > 0.0);
        if ((~swLiveOscDebugEnabled ? false) and: { (Main.elapsedTime - (~swSmoothDemoEarlyDebugAt ? 0.0)) > 0.5 }) {
            ("Smooth demo after motion gate gate=" ++ gate.round(0.001).asString).postln;
        };
        params[\smoothPitchNorm] = pitchNorm;
        params[\smoothExperiment] = params[\smoothExperiment] ? \triggeredNotes;
        ~swAuduinoParams = params;

        if (gate > 0.0) {
            params[\smoothMidi] = ~swSmoothMidiForPitch.(pitchNorm, params[\smoothRootMidi] ? 38);
            params[\smoothFreq] = (params[\smoothMidi] ? 43).midicps;
            params[\smoothVibratoRate] = spinNorm.linexp(0.0, 1.0, 0.05, 1.9) * yawNorm.linexp(0.0, 1.0, 0.88, 1.42);
            params[\smoothVibratoDepth] = spinNorm.linlin(0.0, 1.0, 0.0012, 0.0085) + (activity * 0.0022) + (yawShape * 0.0018);
            params[\smoothMotionDepth] = activity.linlin(0.0, 1.0, 0.02, 0.1) + (yawShape * 0.03);
            params[\smoothBloom] = rollShape.linlin(0.0, 1.0, 0.07, 0.18) + (yawShape * 0.08);
            params[\smoothLp] = rollNorm.linexp(-1.0, 1.0, 480.0, 2600.0);
            tone = ~swSmoothRollToneFor.(params, rollShape);
            if ((~swLiveOscDebugEnabled ? false) and: { (Main.elapsedTime - (~swSmoothDemoEarlyDebugAt ? 0.0)) > 0.5 }) {
                ("Smooth demo after roll tone crunch=" ++ (tone[\crunch] ? 0.0).round(0.001).asString
                    ++ " reverb=" ++ (tone[\reverb] ? 0.0).round(0.001).asString).postln;
                ~swSmoothDemoEarlyDebugAt = Main.elapsedTime;
            };
            params[\smoothCrunch] = tone[\crunch];
            params[\smoothReverb] = tone[\reverb];
            params[\smoothHold] = activity.linlin(0.0, 1.0, 0.04, 0.11) + (yawShape * 0.025);
            params[\smoothDecay] = ~swSmoothLiveDecayFor.(params, activity, yawNorm);
            params[\pan] = (((rollNorm * 0.72) + (yawCentered * 0.28)).clip(-1.0, 1.0)).linlin(-1.0, 1.0, -0.26, 0.26);
            params[\amp] = ~swSmoothLiveAmpFor.(params, activity);

            if (params[\smoothChoirSolo] ? false) {
                ~swSmoothStop.();
                ~swAmbientStop.();
                ~swSmoothChoirApplyMotion.(params, pitchNorm, rollNorm, yawNorm, gate);
                ~swAuduinoParams = params;
            } {
                if ((Main.elapsedTime - (~swSmoothDemoStepDebugAt ? 0.0)) > 0.5) {
                    ("Smooth demo before ambient midi=" ++ (params[\smoothMidi] ? 43).asString
                        ++ " experiment=" ++ (params[\smoothExperiment] ? \nil).asString).postln;
                };
                params = ~swSmoothAmbientApplyMotion.(params, activity, pitchNorm, rollNorm, spinNorm, yawNorm, gate);
                if ((Main.elapsedTime - (~swSmoothDemoStepDebugAt ? 0.0)) > 0.5) {
                    ("Smooth demo after ambient paramsNil=" ++ params.isNil.asString
                        ++ " experiment=" ++ ((params ? ())[\smoothExperiment] ? \nil).asString).postln;
                };
                ~swSmoothChoirApplyMotion.(params, pitchNorm, rollNorm, yawNorm, gate);
                if ((Main.elapsedTime - (~swSmoothDemoStepDebugAt ? 0.0)) > 0.5) {
                    "Smooth demo after choir".postln;
                };
                ~swAuduinoParams = params;
                ~swSmoothApplyActiveTone.(params);
                if ((Main.elapsedTime - (~swSmoothDemoStepDebugAt ? 0.0)) > 0.5) {
                    ("Smooth demo before note branch experiment=" ++ (params[\smoothExperiment] ? \nil).asString).postln;
                    ~swSmoothDemoStepDebugAt = Main.elapsedTime;
                };

                if ((params[\smoothExperiment] ? \triggeredNotes) == \triggeredNotes) {
                    ("SpaceWizard smooth demo note midi=" ++ (params[\smoothMidi] ? 43).asString).postln;
                    ~swSmoothMaybeTrigger.(params, true);
                } {
                    if ((params[\smoothExperiment] ? \triggeredNotes) == \trackedMono) {
                        ~swSmoothMonoTracked.(params);
                    } {
                        ~swSmoothMonoDirect.(params);
                    };
                };
            };
        } {
            ~swAuduinoParams = params;
            ~swSmoothChoirApplyMotion.(params, pitchNorm, rollNorm, yawNorm, gate);
        };

        params = (~swAuduinoParams ? params ? ~swAuduinoDefaultParams.copy).copy;
        params[\voiceMode] = \smoothSine;
        ~swAuduinoParams = params;

        if (didArm) {
            "SpaceWizard smooth demo mode armed.".postln;
        };
        ~swStandPromptUpdate.(params, gate, activity.max(spinNorm), pitchNorm);
    }, '/spacewizard/smooth_demo');


    ~swStartLiveWatchdog.();

    "SpaceWizard synth loaded.".postln;
    "OSC listening on SuperCollider language port: ".post; NetAddr.langPort.postln;
    "Try ~swTestPickup.(), ~swTestTiltSweep.(), ~swTestSpinStudy.(), or ~swTestAutopilot.()".postln;
    "New saw test: ~swSawPentatonicScene.()".postln;
    "Auduino-style test: ~swAuduinoPentatonicScene.()".postln;
    "Ambient tests: ~swAmbientScene.(), ~swAmbientChordalScene.(), ~swAmbientDroneScene.(), ~swAmbientForestScene.()".postln;
    "Smooth bass-note test: ~swSmoothSineScene.()".postln;
    "Reese bass test: ~swReeseBassScene.()".postln;
    "Wow voice test: ~swWowScene.()".postln;
    "Melodic Auduino test: ~swAuduinoMelodicScene.()".postln;
    "Open GUI: ~swAuduinoGUI.()".postln;
    "Ambient GUI: ~swAmbientGUI.() with Flow / Chordal / Drone / Forest selector".postln;
    "Smooth mix GUI: ~swSmoothMixGUI.()".postln;
    "Ambient motion lab: ~swAmbientLabScene.(); ~swAmbientLabGUI.()".postln;
    "Choir texture GUI: ~swChoirTextureGUI.()".postln;
    "Bridge synth OSC: /spacewizard/auduino gate activity pitch roll spin yaw".postln;
    "Smooth demo OSC: /spacewizard/smooth_demo gate activity pitch roll spin yaw".postln;
    "Bridge status OSC: /spacewizard/live_status gate activity pitch roll spin yaw".postln;

    ~swAuduinoParams = (~swAuduinoParams ? ~swAuduinoDefaultParams.copy).copy;

    if (~swRequestedAmbientVariant.notNil) {
        ~swAuduinoParams = ~swAuduinoParams.copy.putAll(~swAmbientPresetFor.(~swRequestedAmbientVariant));
    };
    if (~swRequestedAmbientYawOffset.notNil) {
        ~swAuduinoParams = ~swAuduinoParams.copy.put(\ambientYawOffset, ~swRequestedAmbientYawOffset);
    };
    if (~swRequestedAmbientYawBlend.notNil) {
        ~swAuduinoParams = ~swAuduinoParams.copy.put(\ambientYawBlendEnabled, ~swRequestedAmbientYawBlend);
    };

    if (~swRequestedSmoothPreset.notNil) {
        ~swApplySmoothPreset.(~swRequestedSmoothPreset);
    };

    if (~swRequestedStatusGUI ? false) {
        { ~swLiveStatusGUI.(); }.defer;
    };
};
)
