"""Compile the full patch; test current timer/watchdog functions without audio."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'synth/SpaceWizardSynth.scd'
sclang = os.environ.get('SCLANG_BIN') or shutil.which('sclang') or '/Applications/SuperCollider.app/Contents/MacOS/sclang'
source = SOURCE.read_text()
names = ['swSetMotionGate', 'swStandPromptStop', 'swStandPromptCancel',
         'swStandPromptSchedule', 'swExpireLiveInput', 'swCheckLiveInput']
chunks = []
for name in names:
    start = source.index('    ~' + name + ' =')
    end = source.index('\n    };', start) + 7
    chunks.append(source[start:end])
prelude = r'''(
~auditCheck = { |condition, description|
    if (condition) { ("PASS: " ++ description).postln; }
    { ("FAIL: " ++ description).postln; 1.exit; };
};
~auditCheck.(File.readAllString("SOURCE_PATH").compile.notNil, "full synth patch compiles");
~swSetSmoothLayerGate = {};
~swResetSmoothTracking = {};
~swResetAmbientBassTracking = {};
~auditCleanup = 0;
~swSilenceLiveVoices = { ~auditCleanup = ~auditCleanup + 1; };
~swMotionFadeOutSeconds = 0.0;
~auditPromptCount = 0;
~swStandPromptPlay = { |params| ~auditPromptCount = ~auditPromptCount + 1; };
~swStandPromptLastPitchNorm = 1.0;
~swLiveInputValid = true;
~swLastLiveControlAt = Main.elapsedTime;
~swLiveInputTimeoutSeconds = 0.75;
~swLiveInputState = (activity: 0.0);
~swUpdateLiveInputState = { |gate, activity| ~swLiveInputState = (gate: gate, activity: activity); };
'''.replace('SOURCE_PATH', str(SOURCE))
ending = r'''
~swSetMotionGate.(true);
~swSetMotionGate.(false);
~swStandPromptSchedule.((standPromptDelaySeconds: 0.2));
AppClock.sched(0.05, { ~swStandPromptCancel.(); nil; });
AppClock.sched(0.06, { ~swStandPromptSchedule.((standPromptDelaySeconds: 0.2)); nil; });
AppClock.sched(0.22, { ~swStandPromptCancel.(); nil; });
AppClock.sched(0.30, {
    ~auditCheck.(~auditPromptCount == 0, "cancelled/replaced prompts do not play");
    ~swLastLiveControlAt = Main.elapsedTime;
    ~swStandPromptSchedule.((standPromptDelaySeconds: 0.2));
    nil;
});
Routine({
    35.do { ~swSetMotionGate.(false); 0.02.wait; };
    ~auditCheck.(~auditCleanup == 1, "continuous off packets run cleanup exactly once");
    ~auditCheck.(~auditPromptCount == 1, "valid stationary prompt still plays");
    ~swStandPromptSchedule.((standPromptDelaySeconds: 0.2));
    ~swSetMotionGate.(true);
    ~swStandPromptArmed = true;
    ~swCheckLiveInput.(~swLastLiveControlAt + 1.0);
    ~auditCheck.(~swMotionGateOpen.not, "input watchdog closes the gate");
    ~auditCheck.(~swLiveInputValid.not and: { ~swStandPromptArmed.not }, "input loss disarms stand prompts");
    0.3.wait;
    ~auditCheck.(~auditPromptCount == 1, "input loss cancels pending playback");
    "All synth reliability checks passed.".postln;
    0.exit;
}).play(AppClock);
)
'''
with tempfile.TemporaryDirectory() as directory:
    script = Path(directory) / 'reliability.scd'
    script.write_text(prelude + '\n'.join(chunks) + ending)
    result = subprocess.run([sclang, '-D', '-u', '58999', str(script)],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=20)
    print(result.stdout)
    if result.returncode or 'All synth reliability checks passed.' not in result.stdout:
        raise SystemExit(1)
