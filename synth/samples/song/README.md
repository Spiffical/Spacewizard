Song mode recording (`--mode day-o`).

Put the recording here as `day-o.wav` (WAV, AIFF or FLAC). If that name is
missing, the rig plays the first audio file in this folder alphabetically.

Playback starts at `songStartSeconds` from the `song`/`dayO` preset in
`synth/sound_presets.scd` when the candle is moved deliberately. When the candle is
held still, the song slows down and drops in pitch until it stops, unless the
candle moves again first. Trim the file so it begins at the "Day-O" call, or set
`songStartSeconds` to that time.

Audio files in this folder are ignored by git, because commercial recordings
must not be pushed to the public repository.
