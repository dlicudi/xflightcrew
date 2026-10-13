# Configuration

Settings come from the environment or `.env`, then `prefs.json`, then the
defaults. [.env.example](../../.env.example) lists them all. Most can also be
changed live from the panel's Settings tab or the terminal; one set in the
environment is fixed and the screen says which variable to change.

| Variable | Default | |
|---|---|---|
| `XCOPILOT_MODE` | `both` | `both`, `copilot`, `atc`, `instructor` |
| `XCOPILOT_UI` | `plain` | `plain` or `tui` |
| `XCOPILOT_LISTEN` | `continuous` | `continuous`, `ptt`, or `off` |
| `XCOPILOT_TTS` | `onnx` | `onnx`, `say`, `piper`, or `off` |
| `XCOPILOT_CONFIRM` | `true` | ask before safety-critical commands |
| `XCOPILOT_MONITOR` | `true` | background callouts |
| `XCOPILOT_AIRCRAFT` | `auto` | a profile id to force one |
| `XCOPILOT_ATC_AIRPORT` | `LXGB` | the starting field for ATC |
| `XCOPILOT_ATC_READBACK` | `true` | require readbacks of runway clearances |
| `XCOPILOT_SIMBRIEF_ID` | | your SimBrief username |
| `XCOPILOT_TRAFFIC` | `off` | `ours`, `real`, or `off` |
| `XCOPILOT_EFB` | `false` | serve the EFB page on your network |
| `XCOPILOT_MODELS_DIR` | `models` | where voices and speech models live |

## Voices and speech models

`xfc voices` lists every Piper voice and whisper model, installs and
removes them, and previews a voice (`p`). `u` makes an installed voice the
copilot's or the cabin crew's, the same pick as the panel's voice picker;
a running xfc picks it up on restart. The ATC voices stay in `.env`
(`XCOPILOT_ATC_PIPER_VOICE`, `_APPROACH_`, `_DEPARTURE_`, `_ATIS_PIPER_VOICE`,
`XCOPILOT_ATC_VOICE_POOL`) and the speech model is `WHISPER_MODEL`. A model
something is configured to use cannot be removed until it is moved off it.

## Names

The `xcopilot/*` datarefs and `XCOPILOT_*` variables keep the project's old
name so existing setups keep working.
