# The EFB

A browser Electronic Flight Bag served by `xvl` on your LAN, for an iPad, an iPhone or a second
screen: the copilot's knowledge as four tabs (Flight, Map, Plan, Radio) and Settings behind the
gear, the crew's warnings and cautions, a hold-to-talk button, and buttons that speak for you.
Code: `src/efb/` (`mod.rs` the server, `state.rs` the page state, `flight_tab.rs` the Flight tab's
phase layouts, `transcript.rs` the FROM -> TO labels, `route_map.rs` the route picture and its fit, `basemap.rs` the water under it, `feed.rs`
the taps, `ptt.rs` the push to talk, `airport.rs` and `diagram.js` the airport map, `page.html`
the page, `icons/` the home-screen icons). The approved design is
`docs/tasks/efb-v3/mockup.html` (2026-10-06).

A wide screen shows the tabs in a left rail, a phone in a bottom bar; the gear opens Settings.
A fixed action bar holds MY CONTROLS, HOLD TO TALK and the phase's ONE suggested action (pilot
English like every button), one tap on every size, and the type-a-line box (behind the keyboard
button on a phone). The status strip: callsign, type, route, phase (with the altitude in the air),
who flies, the caution chip, COM1 with the station it is on. The page is night first, with a day
palette from the device's setting, or picked on the Settings tab (this device only).

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
| Strip | callsign (written, "WAT519"), type, route, phase and altitude, who flies, COM1 and its station | the monitor's callsign resolve (`feed::note_callsign`) written by `transcript::written_callsign` over the override and the plan's id, the live profile, the plan on the shelf, `flight::published()`, `flight_state::crew_has_aircraft` / `crew_mission`, the runway watch's COM1 read (`feed::note_com1`) named by the station's services |
| Flight | the layout for the phase (below): a headline, four readouts, the cards; the running checklist as a card in any phase; the intercom log | `flight_tab::build` over the same facts; `flow::sheet`, `flow::proposal_title`, `flow::waiting_for`; the transcript's intercom lines |
| Map | on the ground or low near the destination, the airport diagram; elsewhere the route over water and coastlines: legs in magenta, flown legs dimmed, the active fix ringed, the top of descent, the plan's fields, the aircraft (below) | `airport::page` (below); `route_map::build` over the plan's legs, `procedure::active_leg`, the monitor's tick, `procedure::planned_tod` (else the plan's ring measured back from the destination), the field directory (`station::field_position`); `GET /basemap` from X-Plane's installed map data |
| Plan | source and when shelved, the route string, the plan check and preflight, route and procedures, plan fuel, the load sheet, the legs (the follower's fix in magenta), the weather for destination, origin and alternate with the runway picked or cleared | `simbrief::loaded_plan` / `plan_shelved_at`, `procedure::active_leg`, `feed::AUDIT`, `feed::PREFLIGHT`, the sim's real-weather file (`weather::latest_file`, re-read at most once a minute), `full_auto::arrival_runway`, `party_line::clearance_runway` |
| Radio | COM1 and who it is, the accepted clearance (or the tower's state for us), a readback owed, the controller's last call to us with Say again, the station's frequencies, the frequency log (Radio or All) | `feed::com1`, `station::tuned_station`, `station::active_station_in_range`, `party_line::assigned_clearance`, the flight model's clearance fact, `party_line::readback_owed`, `Voice::last_controller_line`, the labelled transcript |
| Settings | every row of the settings registry, as the terminal's Settings screen shows it, and this device's theme | `settings::ALL` |
| Every page | the crew's warning as a banner until acknowledged on the page, the latest caution as an amber chip, a dot for idle / listening / thinking / speaking | `feed::note_alert` (tapped where the voice mirrors to the panel; a banner or chip only for a `Voice::hazard`, below), `feed::note_activity`, `Voice::is_speaking` |

### The Flight tab, phase by phase

The layout follows the flight model's phase (`flight_tab::layout_for`, pinned by
`each_phase_has_its_layout`): no model yet, Parked, Engine start, Ready and Pushback read as
**Parked**; Taxi out, Holding, Lined up, the takeoff roll, a reject and Taxi in as **Taxi**;
Initial climb, Climb, Cruise and Hover as **Climb and cruise**; Descent, Approach, Go around,
Landing and Rollout as **Descent and approach**.

| Layout | Headline | Readouts | Cards | Suggested action |
|---|---|---|---|---|
| Parked | the accepted IFR clearance (squawk in mono), else the tower's state, else "LEMG to EGKK, no clearance yet" and what to ask for, else "no flight plan" | fuel on board against the plan's block (amber under the minimum takeoff fuel), the plan's takeoff weight against its limit, the plan's flight time, the squawk | the running checklist (else Departure: SID, initial climb, cruise, COM1), the departure runway: available (the drawn field's length), needed (`aircraft::takeoff_needed_m` at the plan's weight, "fits" or "short by"), the origin METAR's wind with its head and cross components, QNH | Fetch the flight plan, else Fly the flight plan (two taps), else How are we doing? |
| Taxi | the guide's turn card ("Next: onto B in 100 m", the taxiway in magenta), else holding or lined up on the runway, else the route's holding point | next turn ("L B"), metres to the hold or stand along the held route with the time at this groundspeed, groundspeed, the departure runway with its length and wind | the taxi route by name, each leg's length, done or to come, the metres to its end, then the hold; the running checklist, else the runway card | "<Field> Tower, ready for departure" while the pilot flies, holding or taxiing with a tower tuned; else How are we doing? |
| Climb and cruise | "Next NORMI in 64 nm, at 10:08Z", then the next fix and the top of descent in the sub line | the active fix's distance and ETA, the top of descent in minutes (its miles, fix and level), the destination's ETA and miles to go along the route, fuel at the destination at this burn against the plan's arrival fuel and reserve | the route ahead (the leg flown, the active fix in magenta, four more with planned level, navlog stage and ETA), Flight: speed, who has the lateral (LNAV or the follower's bug, `feed::note_lateral`), the destination's METAR, the arrival runway and STAR; the running checklist | Top of descent? |
| Descent and approach | "Runway 26L at EGKK", the tower's state, the STAR and the destination's weather | the runway with its length, the wind with head and cross components, QNH, the landing distance (`aircraft::planned_landing_needed_ft`, the book figure with its margin) against the strip | the stopping margin as a bar (needed against available) with the margin in metres, else an Arrival card; the landing checklist | Go around (two taps) |

Readouts the mockup shows that the app does not hold are left out, never filled: engine N2 and
the start's progress, the ILS frequency, identifier and DA, the wind aloft, the before-takeoff
configuration read back (flaps, trim, autobrake), per-fix airway and ETA from the OFP, TA/RA.

### The transcript

Every line the taps kept is labelled FROM -> TO by one pure rule (`transcript::label`, pinned by
the tests in `src/efb/transcript.rs` on real lines) and coloured by speaker: you white, the
copilot cyan, the cabin violet, ATC green. The Flight tab shows the intercom, the Radio tab the
frequency grouped under each frequency's header, or All (both, each tagged RADIO or INTERCOM).

- Intercom: COPILOT -> YOU, or COPILOT -> CABIN when the line opens with the cabin address
  ("Cabin crew, seats for landing."); CABIN -> FLIGHT DECK; YOU -> COPILOT, or YOU -> CABIN when
  the grammar's cabin address leads (`cabin::Cabin::addressed`).
- Ours on the radio: OUR CALLSIGN -> the position COM1 was on ("WAT519 -> TOWER"), never
  "copilot"; tagged "by copilot" when the crew keyed it (the radio drain's call,
  `feed::crew_transmits`), "readback" for a readback.
- A controller: the position -> the callsign that opens its line ("TOWER -> EASY 8791"), ours
  or another's; a line addressed to someone else, or the voice's `overheard`, is dimmed. The last
  call to us carries "readback owed" while `party_line::readback_owed` holds.
- Traffic: the ghost's callsign -> the position, dimmed. A "say again" either way is tagged.

The sim panel's transcript carries the same labels: the app writes the row's "FROM > TO" to
`xcopilot/status/fromto` just before the row (`transcript::panel_label`), and the plugin shows
the FROM on the chip and "> TO" before the text. An older plugin never registers the dataref:
the app skips the label and the rows keep their tags.

The plan check and preflight were spoken once and dropped before the EFB; they are now kept in
flight-scoped cells (`efb::feed::AUDIT`, `PREFLIGHT`: a `Scoped` cell, so a new flight never shows
the last one's verdict).

## The Map tab: which picture

The page's own choice (`route_map::mode`, pinned by `the_map_turns_to_the_field_low_near_it`):
on the ground, the airport diagram; airborne, the destination's diagram once within 15 nm of it
(where the diagram still draws the aircraft) below 3,000 ft above the ground, or in the landing;
the route otherwise; airborne with no route to draw, the airport. The Airport and Route buttons
pick for you until the page's own choice changes. Airborne on the diagram, "follow" frames the
field and the aircraft together, north up.

## The Map tab: the route

The fit is the state's (`route_map::fit`, pinned by `the_cruise_fits_what_is_ahead`,
`the_last_fix_near_the_field_fits_the_terminal`, `the_ground_fits_the_whole_plan`):

- Airborne within 40 nm of the destination, or within 60 nm below 10,000 ft: the **terminal**
  view, the aircraft and the field only.
- Airborne elsewhere: what is **ahead**, the aircraft, the next fix and the legs still to fly.
  Flown legs stay drawn, dimmed, and never drive the fit.
- On the ground, or with no aircraft: the whole **route**.
- Never narrower than 15 nm either way.

Under the route, water and coastlines from X-Plane's OWN installed map data,
`Resources/map data/water/<+LL+LLL>/<+LL+LLL>.shp`: one ESRI Shapefile of water polygons per 1x1
degree tile, named by its south-west corner, in 10x10 degree folders (355 folders, 664 MB). An
installed file like apt.dat, never a sim read (`efb::basemap`). `GET /basemap?bbox=lat0,lon0,lat1,lon1&px=<width>`
(token required, ETag'd) reads only the tiles under the view, off the server's thread: the
rings simplified (Douglas-Peucker) to about a pixel of the view, each vertex on a tile's side
kept so neighbours meet edge to edge, sent as flat `[lat, lon, ...]` arrays at 4 decimals, under
400 kB (coarser until it fits). A tile with no file is all water or all land: a neighbour whose
shared side is water makes it water, a land side land, the answer spreads among missing tiles,
and none at all is open sea. Tiles are cached (96 read, 1,024 simplified, 16 answers). Measured
on the real install: the EGKK terminal view 69 kB at 1,400 px (22 kB on a phone), a cruise view
from Malaga to Gatwick 150 to 200 kB, a few to 30 ms. The page asks again only once its view
leaves the tiles it holds or its zoom moves by about a step. No map data: the route alone,
said nowhere. Land is the page's background, water a muted blue grey (`--water`), the coast a
faint line (`--coast`), day and night.

Fields: the plan's origin, destination and alternate are ringed and labelled with their ICAO
always (from the field directory the region index loads from apt.dat, else the plan's waypoint).
Once the view is under 6 degrees across, `/basemap` also sends the towered fields in it (every
land field under 1.5 degrees): drawn as small dots near the legs still to fly (or anywhere in a
terminal view), the towered ones lightly labelled.

Labels never overlap: each is placed at the first free spot of eight round its point, clear of
the labels already placed, the aircraft, the map's buttons, the info box and the scale bar; in
order the active fix, the destination and the top of descent (never dropped), the origin and
alternate, the fixes ahead nearest first, the towered fields, the flown fixes. One with no free
spot is dropped.

## The Map tab: the airport

Which field: in the air, the plan's destination (else the station's field); on the ground, the
field we are at (the plan's origin or destination, or the station's, within 5 nm), else the
departure (`airport::wanted_field`). The info box says which and why.

The diagram is drawn in the night palette in either theme. The geometry is `GET /airport?icao=XXXX` (token required), JSON in a local frame: east and north
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
command loop takes it to; the two built from a fact (a field's weather, the station's "say
again" and "ready for departure") are pinned in the same test. "Fly the flight plan", "Land the
aircraft", "Load the aircraft" and "Go around" need a second tap within three seconds; "My
controls" is always one tap, on every page and size.

The type-a-line box sends any line, as typed. One line per 500 ms (a 429 otherwise), 200
characters at most.

The checklist card's buttons are the flow's closed vocabulary ("go ahead", "talk me through it",
"not now" for a proposal; "yes" / "no" at a hazard confirm; "next", "skip", "wait", "help me with
that" at an instructor's item; "stop the checklist" while one runs), shown only when the list is
waiting for that answer, and pinned in the same routing test.

## Alerts

The banner and the chip are for HAZARDS, said through `Voice::hazard` with a severity
(`monitor::Severity`): the monitor's checks at their own Warning or Caution (stall and airspeed,
sink rate, terrain, overspeed, cabin altitude, fuel...), the safe hold's terrain warning, its
holds and its take-over, the recovery's lines, "Not stabilised, ..., going around", an aborted
start, a stale plugin, automation stopped, the sim gone. A line spoken at `Priority::Warning` or
`Caution` is NOT a hazard by that alone: the priority is how hard a line interrupts (autoland
says "I'll land at EGKK runway 26L, 8 miles. I have the approach." at Warning only so the radio
cannot cut it), and such lines stay in the conversation (pinned by
`only_a_hazard_reaches_the_efb_banner`).

Hazards are kept with their level, this flight's only (40 at most). A WARNING is a red banner
under the strip on every page until you tap Acknowledge (the acknowledgement lives on that device
only; the app never hears it), for half an hour at most. A CAUTION is the amber chip in the
header for five minutes. Everything else the crew says is in the conversation only.

A frequency is set with its point tightened ("124.230", not the mono face's "124. 230") on the
strip, the Radio tab, the frequency log's headers and the cards.

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
  (`simbrief::Waypoint` is ident, position, altitude). An OFP parse change, not a sim read. The
  Flight tab's leg ETAs are the distance at this groundspeed.
- Engine start progress (N2 per engine), the ILS frequency, identifier and DA, the before-takeoff
  configuration as read back, TA/RA: the Flight tab leaves these readouts out until a loop
  publishes them.
- The aircraft's fuel against the plan at this fix: needs the navlog's planned fuel per fix (above).
- Flight category (VFR, MVFR) on the weather cards: the parsed METAR keeps cloud as words.
- The tower's runway in use at a field other than the one we are cleared at.
- Wind and temperature at the aircraft, V speeds, landing performance beyond the book figure:
  each needs a new read or a new publisher.
- On the Airport page: other traffic, the auto taxi's paint-ridden path (the page draws the route
  as the party line holds it), Custom Scenery airports (only Global Airports' apt.dat is read, as
  everywhere else in the app).
