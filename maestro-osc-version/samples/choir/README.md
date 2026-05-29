Place portable choir samples for SuperCollider in this folder.

Requirements:
- Use mono `.wav`, `.aiff`, `.aif`, or `.flac` files.
- Use sustained choir vowels such as `ooh` or `aah`.
- Dry or lightly reverberant recordings work best.
- 3 to 8 distinct singers is enough for the current layered choir engine.
- Longer sustained notes are better than short attacks.

Current active sample:
- `00_heavenly_ah_random_intruder_cc0.wav`
- Source: Freesound `AHHHH` by `random_intruder`
- License: Creative Commons 0
- Notes: downloaded from the public high-quality preview and converted to mono WAV for SuperCollider.

Behavior:
- `start_spacewizard_live.sh` passes this folder to SuperCollider at boot.
- `SpaceWizardSynth.scd` auto-loads every supported audio file here in alphabetical order.
- If this folder is empty, the live rig falls back to the built-in synthesized choir.

Suggested naming:
- `alto_ooh_01.wav`
- `alto_ooh_02.wav`
- `soprano_ooh_01.wav`
- `soprano_ooh_02.wav`
- `tenor_ooh_01.wav`

Licensing:
- Only add samples you created yourself or are licensed for redistribution and live performance.
