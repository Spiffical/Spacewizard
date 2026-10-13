"""Boot an isolated real synth server using a specified silent/virtual output.

Usage: .venv/bin/python tests/run_synth_integration.py --output-device 'BlackHole 2ch'
Uses private OSC ports and never stops another synth process.
"""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--output-device', required=True)
args = parser.parse_args()
sclang = os.environ.get('SCLANG_BIN') or shutil.which('sclang') or '/Applications/SuperCollider.app/Contents/MacOS/sclang'

def sc_string(value):
    return '"' + str(value).replace('\\', '\\\\').replace('"', '\\"') + '"'

with tempfile.TemporaryDirectory() as directory:
    health = Path(directory) / 'synth.health'
    script = Path(directory) / 'integration.scd'
    code = r'''(
s = Server(\spacewizardAudit, NetAddr("127.0.0.1", 58998));
Server.default = s;
s.options.numInputBusChannels = 0;
s.options.memSize = 262144;
s.options.outDevice = OUTPUT_DEVICE;
~swHealthFile = HEALTH_FILE;
~swRequestedSmoothPreset = \piQuietPentatonic;
~swSoundPresetConfigFile = PRESET_FILE;
~swChoirSampleDir = CHOIR_DIR;
~swStandPromptSampleDir = VOICE_DIR;
~auditPassed = true;
~auditCheck = { |condition, label|
    (condition.if("PASS: ", "FAIL: ") ++ label).postln;
    if (condition.not) { ~auditPassed = false; };
};
thisProcess.interpreter.executeFile(SYNTH_FILE);
s.waitForBoot {
    Routine({
        var packet = ['/spacewizard/auduino', 1.0, 0.4, 0.2, 0.1, 0.2, 0.3, 1.0];
        var attempts = 0;
        var healthAfterQuit;
        while { ~swCheckLiveInput.isNil and: { attempts < 100 } } { 0.1.wait; attempts = attempts + 1; };
        if (~swCheckLiveInput.isNil) { "FAIL: synth initialization".postln; s.quit; 1.exit; };
        ~swSetLiveMode.(\smoothSine);
        ~swMotionFadeOutSeconds = 0.1;
        12.do { OSCdef(\swAuduinoControl).func.value(packet); 0.1.wait; };
        ~auditCheck.(~swMotionGateOpen and: { ~swLiveInputValid }, "live control opens real synth gate");
        ~auditCheck.(~swSmoothHasVoices.(), "Pi preset creates real voices");
        1.5.wait;
        ~auditCheck.(~swMotionGateOpen.not and: { ~swLiveInputValid.not }, "lost OSC closes real synth gate");
        ~auditCheck.(~swSmoothHasVoices.().not and: { ~swAmbientHasVoices.().not }, "watchdog cleans up real voices");
        ~auditCheck.(PathName(HEALTH_FILE).isFile, "audio server heartbeat written");
        OSCdef(\swAuduinoControl).func.value(packet);
        ~auditCheck.(~swMotionGateOpen, "fresh OSC recovers after timeout");
        packet[7] = 0.0;
        OSCdef(\swAuduinoControl).func.value(packet);
        ~auditCheck.(~swMotionGateOpen.not and: { ~swStandPromptArmed.not }, "invalid-input packet closes gate and disarms prompt");
        ~swSilenceLiveVoices.();
        s.quit;
        0.5.wait;
        healthAfterQuit = File.mtime(HEALTH_FILE);
        1.2.wait;
        ~auditCheck.(File.mtime(HEALTH_FILE) == healthAfterQuit, "audio-server exit stops heartbeat while language stays alive");
        ~swLiveWatchdogRoutine.stop;
        if (~auditPassed) { "All real synth integration checks passed.".postln; 0.exit; } { 1.exit; };
    }).play(AppClock);
};
)
'''
    for name, value in dict(OUTPUT_DEVICE=args.output_device, HEALTH_FILE=health,
                            PRESET_FILE=ROOT/'maestro-osc-version/sound_presets.scd',
                            CHOIR_DIR=ROOT/'maestro-osc-version/samples/choir',
                            VOICE_DIR=ROOT/'maestro-osc-version/samples/voice',
                            SYNTH_FILE=ROOT/'maestro-osc-version/synth/SpaceWizardSynth.scd').items():
        code = code.replace(name, sc_string(value))
    script.write_text(code)
    process = subprocess.Popen([sclang, '-D', '-u', '58997', str(script)], stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        output, _ = process.communicate(timeout=40)
        print(output)
        if process.returncode or 'All real synth integration checks passed.' not in output:
            raise SystemExit(1)
    finally:
        # Isolated process group contains only this test's language/audio processes.
        if process.poll() is None:
            import signal
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
