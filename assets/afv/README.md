# AFV-Native audio assets

`Click_f32.wav` and `Crackle_f32.wav` are from
[AFV-Native](https://github.com/xsquawkbox/AFV-Native) (Audio for VATSIM),
used under the 3-Clause BSD License (see `AFV_COPYING.md`). The click is the
squelch transient VATSIM clients play when a station un-keys; the ATC radio
effect splices it into the tower's transmissions
(`XCOPILOT_SQUELCH_WAV` selects a different recording; empty falls back to
the synthesized burst in `src/radiofx.rs`).
