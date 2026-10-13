# Installation and setup

## Requirements

- X-Plane 12.4.4 or newer, with the Web API enabled (Settings, Network,
  port 8086)
- macOS on Apple Silicon (the speech model runs on Metal)
- Rust, to build the app
- a whisper.cpp model, a ggml `.bin` such as `ggml-base.en.bin`, in `models/`

## Build and install

```sh
cp .env.example .env
make -C plugin install
cargo run
```

`make -C plugin install` builds the plugin and copies it to
`X-Plane 12/Resources/plugins/xflightcrew/`. It also removes an old
`plugins/xcopilot/` folder if there is one, since two copies would both load.
Install it with X-Plane closed, or reload plugins afterwards.

The plugin and the app work together: the app does the listening, thinking
and speaking, and the plugin gives it push-to-talk, the in-sim panel, FMS
route loading, terrain queries, traffic drawing and the control channels the
copilot flies through.

## Talking to it

Listening is continuous by default. For push-to-talk, set
`XCOPILOT_LISTEN=ptt` and bind the plugin's commands in X-Plane: the radio
key transmits to ATC, the intercom key talks to the crew.

To type instead of speak, set `XCOPILOT_UI=tui`, or use the typed line on
the panel's Transcript tab.

## The panel

The plugin draws a window with Flight, Transcript, ATC, Route, Checklist,
Brief, State, Board and Settings tabs: the live situation, everything said,
the frequencies, the plan, the aircraft's checklists, the airport's
departures and arrivals, and the settings.

## Aircraft

The profile is picked automatically from the loaded aircraft. Profiles ship
for the Airbus A330-300, Boeing 737-800, Zibo 738, Boeing 747-400, MD-82,
Citation X, Cirrus Vision SF50, Cessna 172SP, Cirrus SR22, Beechcraft Baron
58, King Air C90B, Twin Otter, DC-3, Lancair Evolution, Van's RV-10 and
Sikorsky S-76C. Any other aircraft gets a basic profile read from its `.acf`
file, and the copilot says so.

`xfc conform` checks a profile against the loaded aircraft while parked:
every control it declares is moved, read back and put back. See
[conform.md](../conform.md).

## Modes

Set with `XCOPILOT_MODE` or live from the panel's Settings tab:

- `both` (default): copilot and ATC
- `copilot`: no simulated ATC, for use with online ATC
- `atc`: ATC and traffic without the crew
- `instructor`: teaches and grades one task at a time without touching the
  controls

## Testing

```sh
cargo test --workspace
```

This runs without X-Plane. Tests that need an X-Plane install or a running
sim are marked ignored. With the sim running:

```sh
xfc sortie bench/sorties/c172-g1000-ils13.yaml
xfc debrief
```

`sortie` flies a scripted flight and scores it; `debrief` reviews the last
flight. Every flight is logged to `logs/blackbox.jsonl`, and `xfc analyze`
lists the phrases the grammar missed.

## Limitations

- X-Plane 12 only.
- The Cessna 172 is the reference aircraft. Other profiles work but have had
  less flight testing.
- Approach and landing are done and flown on the C172, SR22, King Air,
  Lancair Evolution, Baron 58 and SF50; jets use the same controller but are
  not yet validated. Takeoff, circuits, taxi and full auto get only safety
  fixes until their turn.
- Traffic needs CSL models installed, pauses under time acceleration, and
  does not yet account for wind.
- Phrasing the grammar does not know is refused, not guessed.
