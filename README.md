# XFlightCrew

A copilot, ATC, AI traffic and cabin crew for X-Plane 12, driven by voice and
running entirely on your own machine.

```
"flaps ten"  ->  speech to text  ->  grammar  ->  write the dataref  ->  read it back  ->  "Flaps ten."
```

- **Copilot**: works the panel, runs the aircraft's checklists, watches for
  problems and can fly any phase of the flight.
- **ATC**: Delivery, Ground, Tower and Approach on the airport's real
  frequencies, with readbacks, sequencing and taxi routes.
- **AI traffic**: airline and GA flights handled by the same controllers as
  you.
- **Cabin crew**: on the aircraft that carry them.

There is no language model and nothing is sent to a server. Speech
recognition (whisper.cpp), the command grammar and the voices all run
locally. A phrase the grammar does not know is refused, never guessed, and
every action is read back from the aircraft's actual state.

## Install

You need:

- X-Plane 12 with the Web API enabled (Settings, Network, port 8086)
- a whisper.cpp model such as `ggml-base.en.bin` in `models/`
- Rust, to build the app

```sh
cp .env.example .env
make -C plugin install
cargo run
```

`make -C plugin install` builds the XFlightCrew plugin and installs it into
X-Plane. It provides push-to-talk, the in-sim panel, FMS loading, traffic
drawing and the control channels the copilot flies through.

Details: [installation and setup](docs/guide/install.md).

## Guides

- [Installation and setup](docs/guide/install.md)
- [The copilot](docs/guide/copilot.md): commands, questions, checklists,
  letting it fly
- [ATC](docs/guide/atc.md)
- [AI traffic](docs/guide/traffic.md)
- [Configuration](docs/guide/configuration.md)
- [The EFB](docs/efb.md): the tablet page
- Per-aircraft command references: [Cessna 172SP](docs/cessna_172sp.md),
  [Cirrus SR22](docs/cirrus_sr22.md)

## Status

The Cessna 172 is the reference aircraft. Sixteen aircraft have profiles; any
other gets a basic profile read from its `.acf`. The automated flying is
being rebuilt one phase at a time: approach and landing are done, the other
phases take only safety fixes until their turn. See
[limitations](docs/guide/install.md#limitations).

## Development

`cargo test --workspace` runs everything that does not need the sim. The
design rules are in [CLAUDE.md](CLAUDE.md); the history behind them is in
[docs/dev-history.md](docs/dev-history.md).

## Licence

GPL-3.0-or-later ([LICENSE](LICENSE)). Third-party parts keep their own
licences, all GPL-compatible: the X-Plane SDK (MIT/BSD), Dear ImGui (MIT),
XPMP2 (MIT) and the B612 font (SIL OFL). CSL models are not bundled.
