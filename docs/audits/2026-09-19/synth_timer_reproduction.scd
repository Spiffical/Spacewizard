// Isolated copies of audited functions; mocks perform no audio or network sends.
(
~swSetSmoothLayerGate = {};
~swResetSmoothTracking = {};
~swResetAmbientBassTracking = {};
~auditCleanup = 0;
~swSilenceLiveVoices = { ~auditCleanup = ~auditCleanup + 1; };
~swMotionFadeOutSeconds = 0.0;
~auditPromptCount = 0;
~swStandPromptPlay = { |params| ~auditPromptCount = ~auditPromptCount + 1; };
~swStandPromptLastPitchNorm = 1.0;
    ~swSetMotionGate = { |isOpen=false|
        var open = isOpen.asBoolean;

        ~swMotionGateOpen = open;
        if (~swMotionLevelBus.notNil) {
            ~swMotionLevelBus.set(open.if(1.0, 0.0));
        };
        ~swSetSmoothLayerGate.(open);
        if (~swMotionCleanupRoutine.notNil) {
            ~swMotionCleanupRoutine.stop;
            ~swMotionCleanupRoutine = nil;
        };
        if (open.not) {
            ~swResetSmoothTracking.();
            ~swResetAmbientBassTracking.();
            ~swMotionCleanupRoutine = Routine({
                ((~swMotionFadeOutSeconds ? 5.0) + 0.35).wait;
                if (~swMotionGateOpen.not) {
                    ~swSilenceLiveVoices.();
                };
            }).play(AppClock);
        };
    };
    ~swStandPromptStop = {
        if (~swStandPromptRoutine.notNil) {
            ~swStandPromptToken = (~swStandPromptToken ? 0) + 1;
            ~swStandPromptRoutine = nil;
        };
        if (~swStandPrompt.notNil) {
            ~swStandPrompt.set(\gate, 0);
            ~swStandPrompt = nil;
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
            ~swStandPromptRoutine = true;
            AppClock.sched(delay, {
                if ((token == (~swStandPromptToken ? -1))
                    and: { (~swStandPromptLastPitchNorm ? 0.0) >= pitchMin }) {
                    ~swStandPromptPlay.(params);
                    ~swStandPromptPrompted = true;
                    ~swStandPromptArmed = false;
                } {
                    ("Stand prompt skipped at playback check; pitchNorm=" ++ (~swStandPromptLastPitchNorm ? 0.0).round(0.001).asString).postln;
                };
                ~swStandPromptRoutine = nil;
                nil;
            });
        };
    };
~swStandPromptSchedule.((standPromptDelaySeconds: 0.2));
AppClock.sched(0.05, { ~swStandPromptCancel.(); nil; });
AppClock.sched(0.06, { ~swStandPromptSchedule.((standPromptDelaySeconds: 0.2)); nil; });
AppClock.sched(0.22, { ~swStandPromptCancel.(); nil; });
Routine({
    35.do { ~swSetMotionGate.(false); 0.02.wait; };
    ("AUDIT cleanup_calls_during_continuous_off=" ++ ~auditCleanup).postln;
    ("AUDIT canceled_prompt_play_count=" ++ ~auditPromptCount).postln;
    if ((~auditCleanup == 0) and: { ~auditPromptCount == 1 }) {
        "AUDIT reproduced both timer bugs".postln;
        0.exit;
    } { 1.exit; };
}).play(AppClock);
)
