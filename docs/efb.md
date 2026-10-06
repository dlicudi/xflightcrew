# The EFB

A browser Electronic Flight Bag served by `xvl` on your LAN, for an iPad, an iPhone or a second
screen: the copilot's knowledge as seven pages (Now, Radio, Airport, Checklist, Plan, Weather,
Settings, in the order a flight uses them), the crew's warnings and cautions, a hold-to-talk
button, and buttons that speak for you. Code: `src/efb/` (`mod.rs` the server, `state.rs` the page
state, `feed.rs` the taps, `ptt.rs` the push to talk, `airport.rs` and `diagram.js` the airport
map, `page.html` the page, `icons/` the home-screen icons).

A wide screen shows every page in its rail, an iPad upright in its top bar (words only). A phone's
bottom bar shows Now, Radio, Airport, Checklist and More, which opens Plan, Weather and Settings;
a phone on its side shows the same five as icons in a rail that never scrolls. MY CONTROLS and HOLD
TO TALK are one tap on every size.

## Turn it on

Off by default. Any one of:

- `.env` or the shell: `XCOPILOT_EFB=true` (pins it; the terminal and panel then refuse to change it)
- the terminal's Settings screen or the panel's Settings tab: `EFB` on
- `prefs.json` (`settings.efb = "on"`), written by either of the above

It applies live: turning it on starts the server within two seconds, off stops it, and a new port
rebinds.

| Setting | Env | Default |
|---|---|---|
| `efb` | `XCOPILOT_EFB` | off |
| `efb_port` | `XCOPILOT_EFB_PORT` | 8787 |
| `efb_token` | `XCOPILOT_EFB_TOKEN` | generated at the first enable, saved in `prefs.json` |

## The URL and the token

At the first bind `xvl` prints (plain mode) and logs (`EFB serving`, both modes) the full link:

```
http://192.168.1.20:8787/?t=3f9a0c1d2b7e4a6f8c5d1e0b9a7f6c3d
```

Open it once on the device. Every request needs the token: the page takes it from `?t=`, sets a
cookie, and keeps it in the device's local storage; the API calls send it as `X-EFB-Token`. A wrong
or missing token is a 403 and a blackbox `efb` `auth_failed` record (the peer address and the path,
never the token tried). The token is the `EFB token` setting (shown on the terminal's Settings
screen and the panel's Settings tab): blank it and the next enable makes a new one, which retires
every old link.

The server binds `0.0.0.0`: anyone on your LAN who has the link can press the buttons. Keep the
link to yourself, and leave the EFB off on a network you do not trust.

## Add it to the home screen (iPhone, iPad)

Open the link in Safari, then Share, then **Add to Home Screen**. The page carries a web app
manifest (`/manifest.webmanifest`, whose start URL holds the token) and
`apple-mobile-web-app-capable`, so the icon opens full screen with no browser bars. The icons
(`/apple-touch-icon.png` 180, `/icon-192.png`, `/icon-512.png`) are PNGs drawn by
`python3 tools/efb_icon.py` (deterministic: a re-run on an unchanged script changes no byte) and
served without the token, since iOS fetches the touch icon without the page's cookie; they hold
nothing but the mark. A home-screen
app has its own cookie jar: the token rides in its start URL and in local storage, so it keeps
working after a restart. If you ever change the token, delete the icon and add it again from the
new link.

## What each page reads

Nothing on the EFB reads X-Plane. Every number is one the app already holds; a fact nobody holds
is a dash, never a guess.

| Page | Shows | From |
|---|---|---|
| Header | callsign, type, route, phase, who flies, COM1 | the monitor's callsign resolve (`feed::note_callsign`), the live profile, the plan on the shelf, `flight::published()`, `flight_state::crew_has_aircraft` / `crew_mission`, the runway watch's COM1 read (`feed::note_com1`) |
| Now | phase strip, NEXT (the follower's top of descent, else its fix, else the plan's ring or the destination along the route), who flies and the automation feed, IAS / ALT / GS / ETA / fuel, the last 8 lines said | `flight::published()`, `procedure::planned_tod` / `active_leg` / `fix_distance_nm`, `auto_status::compose`, the monitor's tick (`feed::note_tick`), `monitor::route_remaining_nm` over the plan, the speakers (`feed::note_intercom`, `note_station`) and the command loop (`note_pilot`) |
| Radio | COM1 and who it is, the station's frequencies, the accepted IFR clearance, the tower's state for us, a readback owed, the controller's last call to us, the frequency log | `feed::com1`, `station::tuned_station`, `station::active_station_in_range`, `party_line::assigned_clearance`, the flight model's clearance fact, `party_line::readback_owed`, `Voice::last_controller_line`, the radio lines of the conversation |
| Plan | source and when it was shelved, the legs with planned altitude and navlog stage (the follower's fix highlighted), SID / STAR / runways, plan fuel, the load sheet, the last plan check and the arm's preflight | `simbrief::loaded_plan` / `plan_shelved_at`, `procedure::active_leg`, `feed::AUDIT` (kept by `plan_audit::check`), `feed::PREFLIGHT` (kept at full auto's arm) |
| Checklist | this flight's current or last list item by item (done, held and by whom, not as listed, skipped, pending), its flavour, the copilot's proposal, and only the replies the list is waiting for | `flow::sheet` (a record the runner keeps as it decides each item, flight-scoped; nothing reads it back), `flow::proposal_title`, `flow::waiting_for` |
| Settings | every row of the settings registry, as the terminal's Settings screen shows it | `settings::ALL` |
| Every page | the crew's warning as a banner until acknowledged on the page, the latest caution as an amber chip, a dot for idle / listening / thinking / speaking | `feed::note_alert` (tapped where `Voice::alert`, `narrate` and `say_crew` mirror to the panel), `feed::note_activity` (the reporter's status phase), `Voice::is_speaking` |
| Weather | METAR raw and plain for destination, origin, alternate, with the runway picked or cleared | the sim's real-weather file (`weather::latest_file`, re-read at most once a minute), `full_auto::arrival_runway`, `party_line::clearance_runway`, else the plan's runways |
| Airport | the field's diagram (runways, pavement, taxi lines, taxiway names, holding positions, stands), the aircraft, the taxi route held (driven part dimmed, crossings and its end marked), the turn card as the panel draws it, the cleared or arrival runway | apt.dat as `nav::find_airport` and `nav::find_taxi_net` parse it (built once per field, `efb::airport`), the monitor's tick with its true heading, `party_line::with_taxi_route` / `taxi_to_stand`, the guide's `TurnCard` (`feed::note_turn_card`), `party_line::clearance_runway`, `full_auto::arrival_runway`, the plan's runways |

The plan check and preflight were spoken once and dropped before the EFB; they are now kept in
flight-scoped cells (`efb::feed::AUDIT`, `PREFLIGHT`: a `Scoped` cell, so a new flight never shows
the last one's verdict).

## The Airport page

Which field: in the air, the plan's destination (else the station's field); on the ground, the
field we are at (the plan's origin or destination, or the station's, within 5 nm), else the
departure (`airport::wanted_field`). The info box says which and why.

The geometry is `GET /airport?icao=XXXX` (token required), JSON in a local frame: east and north
metres from the middle of the field's runways, so the page does no geodesy. It is built once per
field from apt.dat off the server's thread (about 0.1 to 0.4 s; LEMG is 76 kB, EGKK 60 kB, rings
and lines thinned to 0.4 m) and kept for the last three fields; its `version` (a hash) is the
ETag, and the page state names the field and version (`airport.icao`, `airport.version`), so the
page fetches it only when the field changes. The layers: runways (100 rows, both ends, width),
pavement (110 polygons; where a field draws none, the 1202 network at a nominal 20 m, said on the
page), taxi centrelines (120, types 1 and 51), taxiway names (on each name's longest 1202 edges),
holding positions (where a 1204 runway zone meets a taxiway outside it), stands (1300).

The live picture rides the page state (`airport`): the aircraft (`own`, true heading and track),
the route (`route.driven`, `route.remaining`, `holds`, `dest`; drawn only when it starts within
15 km), the card (`card`: kind, banner text, aim bearing true, cross-track signed, positive right),
the runway (`runway`: cleared, arrival, or the plan's). The card exists only while the copilot's
taxi guide runs (the crew on, `taxi_guide` on, a taxi clearance held).

On the page: pinch or wheel to zoom, drag to pan, double tap to follow the aircraft again (or to
see the whole field with no aircraft on it). The buttons: zoom, the whole field, north up or track
up, follow. The aircraft glides between the one-second updates; a jump over 400 m is a jump.
The script is `/diagram.js` (code, no data, served without the token like the icon). To look at
it without a sim: `EFB_DIAGRAM_OUT=target/efb cargo test --release -- --ignored
real_install_page_states` writes the real LEMG and EGKK diagrams and a taxiing page state.

## The buttons

Every button POSTs the words a pilot would say (`/say`), and the line goes onto the command channel
exactly like the panel's typed line (`pollers::queue_command`; blackbox `typed` `taken` with
`source: "efb"`, and `efb` `say`). The grammar picks the tool, with its gates and readback. There
is no other way in: the EFB cannot do anything the voice cannot. The phrases are in
`efb::state::PHRASES`, each pinned by `every_button_routes_where_it_says` to the branch or tool the
command loop takes it to. "Fly the flight plan", "Land the aircraft" and "Load the aircraft" need a
second tap within three seconds; "My controls" is always one tap, on every page and size.

The type-a-line box sends any line, as typed. One line per 500 ms (a 429 otherwise), 200
characters at most.

The Checklist page's buttons are the flow's closed vocabulary ("go ahead", "talk me through it",
"not now" for a proposal; "yes" / "no" at a hazard confirm; "next", "skip", "wait", "help me with
that" at an instructor's item; "stop the checklist" while one runs), shown only when the list is
waiting for that answer, and pinned in the same routing test.

## Alerts

Every line the crew says through `Voice::alert` (and `narrate`, `say_crew`) is kept with its level,
this flight's only (40 at most). A WARNING is a red banner on every page until you tap Acknowledge
(the acknowledgement lives on that device only; the app never hears it), for half an hour at most.
A CAUTION is the amber chip in the header for five minutes. An advisory is in the conversation only.

## Push to talk

HOLD TO TALK (beside MY CONTROLS) opens the mic exactly as the plugin's `xcopilot/ptt_active` does,
the intercom key: the words still go through STT and the grammar. It moves nothing. The page sends
`/ptt` press, then a heartbeat every 250 ms while your finger is down, then release; the app treats
it as held only while the heartbeat is fresh (600 ms) and never past 60 s, so a lost touch-up
(iOS cancels touches freely) can never leave the mic open. Blackbox `efb` `ptt` (press, release
with `held_ms`, lapsed with why). The button shows only when this run captures on push to talk (a
bound PTT dataref or key); continuous listening and text-only have nothing for it to open. The
radio key (`xcopilot/radio_ptt_active`) is not on the page yet.

## Settings

The Settings page is the registry (`settings::ALL`), the same rows as the terminal's Settings
screen. A change POSTs `/setting {key, value}` and goes through `settings::set` like the terminal's:
validated, applied live, saved to `prefs.json`. A setting pinned by its environment variable is
shown read-only, naming it. The EFB's port and token are read-only here (the token masked), and the
app mode and turning the EFB off ask for a confirm, which the server checks too (409 without it).
One change per 300 ms. Blackbox `efb` `setting` (key and value, never the token) or
`setting_refused`.

The panel's Settings tab now publishes a change made elsewhere instead of reading the difference
as a click (`pollers::panel_sync`): a mode set from the terminal or the EFB is no longer reverted by
the panel's stale value half a second later.

The voice pickers, the audio bus, camera snap, the negative tone and the UI scale are panel-only
(not in the registry) and are not on the page yet: they move onto the registry first.

## Not on the EFB yet (would need a new sim read or more of the OFP)

- Per-fix airway, ETA and fuel on the Plan page: the OFP navlog has them, the app does not keep them
  (`simbrief::Waypoint` is ident, position, altitude). An OFP parse change, not a sim read.
- The aircraft's fuel against the plan at this fix: needs the navlog's planned fuel per fix (above).
- Flight category (VFR, MVFR) on the weather cards: the parsed METAR keeps cloud as words.
- The tower's runway in use at a field other than the one we are cleared at.
- Wind and temperature at the aircraft, V speeds, landing performance: each needs a new read or
  a new publisher.
- On the Airport page: other traffic, the auto taxi's paint-ridden path (the page draws the route
  as the party line holds it), Custom Scenery airports (only Global Airports' apt.dat is read, as
  everywhere else in the app).
