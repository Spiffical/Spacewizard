(
// SpaceWizard no-OSC audition script.
// Evaluate this file after opening it in SuperCollider.
// It loads the main patch file automatically from the same folder.

var basePath = PathName(thisProcess.nowExecutingPath).pathOnly;
(basePath +/+ "SpaceWizardSynth.scd").load;
)

(
// Wait until the server finishes booting, then run any of these helpers.

~swNoOscPickup = {
	~swTestPickup.();
};

~swNoOscTiltSweep = {
	~swTestTiltSweep.();
};

~swNoOscSpinStudy = {
	~swTestSpinStudy.();
};

~swNoOscAutopilot = {
	~swTestAutopilot.();
};

~swNoOscSawPentatonic = {
	~swSawPentatonicScene.();
};

~swNoOscSawSweep = {
	~swSawPentatonicSweep.();
};

~swNoOscSawRandom = {
	~swSawPentatonicRandom.();
};

~swNoOscAuduino = {
	~swAuduinoPentatonicScene.();
};

~swNoOscAuduinoStepped = {
	~swAuduinoSteppedScene.();
};

~swNoOscAuduinoOriginalScale = {
	~swAuduinoPentatonicScene.(scaleMode: \originalAuduino);
};

~swNoOscAuduinoSpaceWizardScale = {
	~swAuduinoPentatonicScene.(scaleMode: \spacewizardMinor);
};

~swNoOscAuduinoMelodic = {
	~swAuduinoMelodicScene.();
};

~swNoOscAuduinoMelodicStepped = {
	~swAuduinoMelodicSteppedScene.();
};

~swNoOscSmoothSine = {
	~swSmoothSineScene.();
};

~swNoOscSmoothSineSweep = {
	~swSmoothSineSweep.();
};

~swNoOscAmbient = {
	~swAmbientScene.();
};

~swNoOscAmbientSweep = {
	~swAmbientSweep.();
};

~swNoOscAmbientChordal = {
	~swAmbientChordalScene.();
};

~swNoOscAmbientChordalSweep = {
	~swAmbientChordalSweep.();
};

~swNoOscAmbientDrone = {
	~swAmbientDroneScene.();
};

~swNoOscAmbientDroneSweep = {
	~swAmbientDroneSweep.();
};

~swNoOscAmbientForest = {
	~swAmbientForestScene.();
};

~swNoOscAmbientForestSweep = {
	~swAmbientForestSweep.();
};

~swNoOscAmbientGUI = {
	~swAmbientGUI.();
};

~swNoOscReeseBass = {
	~swReeseBassScene.();
};

~swNoOscReeseBassSweep = {
	~swReeseBassSweep.();
};

~swNoOscWow = {
	~swWowScene.();
};

~swNoOscWowSweep = {
	~swWowSweep.();
};

~swNoOscAuduinoGUI = {
	~swAuduinoGUI.();
};

~swManualStart = {
	~swPickup.(activity: 0.3, tilt: -0.2, roll: 0.0, spin: 0.04, presence: 0.5, amp: 0.17);
};

~swManualRise = {
	~swSet.(activity: 0.55, tilt: 0.65, roll: -0.2, spin: 0.3, presence: 0.7, amp: 0.18);
};

~swManualAgitate = {
	~swSet.(activity: 0.75, tilt: -0.4, roll: 0.5, spin: 0.85, presence: 0.8, amp: 0.2);
};

~swManualRest = {
	~swRest.();
};

~swManualRelease = {
	~swRelease.();
};

~swManualStopAll = {
	~swStopAll.();
};

"No-OSC helpers loaded: ~swNoOscPickup.(), ~swNoOscTiltSweep.(), ~swNoOscSpinStudy.(), ~swNoOscAutopilot.(), ~swNoOscSawPentatonic.(), ~swNoOscAuduino.(), ~swNoOscAuduinoStepped.(), ~swNoOscAuduinoOriginalScale.(), ~swNoOscAuduinoSpaceWizardScale.(), ~swNoOscAuduinoMelodic.(), ~swNoOscAuduinoMelodicStepped.(), ~swNoOscSmoothSine.(), ~swNoOscSmoothSineSweep.(), ~swNoOscAmbient.(), ~swNoOscAmbientSweep.(), ~swNoOscAmbientChordal.(), ~swNoOscAmbientChordalSweep.(), ~swNoOscAmbientDrone.(), ~swNoOscAmbientDroneSweep.(), ~swNoOscAmbientForest.(), ~swNoOscAmbientForestSweep.(), ~swNoOscAmbientGUI.(), ~swNoOscReeseBass.(), ~swNoOscReeseBassSweep.(), ~swNoOscWow.(), ~swNoOscWowSweep.(), ~swNoOscAuduinoGUI.()".postln;
)
