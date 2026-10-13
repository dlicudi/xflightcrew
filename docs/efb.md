# The EFB

A browser Electronic Flight Bag served by `xfc` on your LAN, for an iPad, an iPhone or a second
screen: the copilot's knowledge as four tabs (Flight, Map, Plan, Radio) and Settings behind the
gear, the crew's warnings and cautions, a hold-to-talk button, and buttons that speak for you.
Code: `src/efb/` (`mod.rs` the server, `state.rs` the page state, `flight_tab.rs` the Flight tab's
phase layouts, `transcript.rs` the FROM -> TO labels, `map.rs` the map's live picture and its overview, `basemap.rs` the water under it, `feed.rs`
the taps, `ptt.rs` the push to talk, `airport.rs` every field's diagram, `map.js` the world map, `page.html`
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
- the terminal's Settings screen: `EFB` on (the panel's Settings tab says so where the link
  would be)
- `prefs.json` (`settings.efb = "on"`), written by either of the above

It applies live: turning it on starts the server within two seconds, off stops it, and a new port
rebinds.

| Setting | Env | Default |
|---|---|---|
| `efb` | `XCOPILOT_EFB` | off |
| `efb_port` | `XCOPILOT_EFB_PORT` | 8787 |
| `efb_token` | `XCOPILOT_EFB_TOKEN` | generated at the first enable, saved in `prefs.json` |

## The URL and the token

At the first bind `xfc` prints (plain mode) and logs (`EFB serving`, both modes) the full link:

```
http://192.168.1.20:8787/?t=3f9a0c1d2b
```

The panel's Settings tab shows the same link under the `EFB token` field, as text to select or
copy (the Copy link button) and as a QR code to scan with the iPad's camera (2026-10-13). The app
publishes it on `xcopilot/status/efb_url` (`efb::qr::panel_payload`, from the monitor's tick: on
a change and every few seconds, since a plugin reload zeroes it) as `url|N|hex`: the link, the
side in modules with the four-module quiet zone, and the modules row-major, dark = 1, most
significant bit first. The matrix is made in the app by the `qrcode` crate (MIT OR Apache-2.0, no
dependencies with its default features off) at error correction M, and a test decodes it back
(`rqrr`, a dev dependency); the plugin only paints it, dark modules on a white field whatever
the panel's theme, whole pixels a module. Empty while the EFB is off or not yet listening: the
tab then says how to turn it on. A plugin without the dataref is simply not sent it; an app that
has not written it in 30 s (older than the link) gets nothing drawn rather than a false "off".

Open it once on the device. Every request needs the token: the page takes it from `?t=`, sets a
cookie, and keeps it in the device's local storage; the API calls send it as `X-EFB-Token`. A wrong
or missing token is a 403 and a blackbox `efb` `auth_failed` record (the peer address and the path,
never the token tried). The token is the `EFB token` setting (shown on the terminal's Settings
screen and the panel's Settings tab): blank it and the next enable makes a new one, which retires
every old link. A generated token is 10 hex digits (2026-10-13, was 32: the QR code means nobody
types it, and a wrong guess is a 403, blackboxed: 40 bits is out of reach of guessing on a LAN); one
you set yourself (`XCOPILOT_EFB_TOKEN`, 8 to 64 letters, digits, '-' or '_') stays as it is, and
so does a long one saved before: blank it for a short one.

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
| Map | one world map: water and coastlines, the fields, the route (legs in magenta, flown legs dimmed, the active fix ringed, the top of descent, the plan's fields), our traffic and the aircraft at every zoom; closer in, every field's diagram with its stands free or taken, and the taxi route and turn card (below) | `map::build` over the plan's legs, `procedure::active_leg`, the monitor's tick, `procedure::planned_tod` (else the plan's ring measured back from the destination), the field directory (`station::field_position`), `feed::traffic` (`TrafficWorld::note_efb_traffic`); `GET /basemap` from X-Plane's installed map data; `GET /airport` from apt.dat; `airport::page` for the wanted field |
| Plan | source and when shelved, the route string, the plan check and preflight, route and procedures, plan fuel, the load sheet, the legs (the follower's fix in magenta), the weather for destination, origin and alternate with the runway picked or cleared | `simbrief::loaded_plan` / `plan_shelved_at`, `procedure::active_leg`, `feed::AUDIT`, `feed::PREFLIGHT`, the sim's real-weather file (`weather::latest_file`, re-read at most once a minute), `full_auto::arrival_runway`, `party_line::clearance_runway` |
| Board | its own tab (2026-10-13): every field our traffic simulates, Departures and Arrivals as the panel's Board tab shows them, the field picked on the device, a dash with no traffic running | `feed::boards`, each simulated field's `board_payload` tapped by the traffic world every 5 s while the EFB serves (`TrafficWorld::note_efb_boards`), read by `efb::board` |
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

## The Map tab: one world map (2026-10-13)

The Map tab is ONE continuous map, like a phone's map app (`src/efb/map.js`): drag to pan (a
flick carries on and slows), pinch or wheel to zoom, double tap to zoom in, smoothly from a whole
region (about zoom 5.5) down to single stands (zoom 20.5), north up. **Follow** is on by
default: the map keeps our aircraft in the middle and a pinch zooms about it; a pan turns it off,
the follow button turns it back on. The other buttons: zoom in, zoom out, and **Overview**, which
frames what matters now (`map::fit`, below) and stops following. The view and the follow choice
are kept on the device. The first view: on the ground the field round us (about 2.5 km across),
airborne the overview centred on us.

The projection is Web Mercator, the one every world map's tiles use. It is conformal: a field's
diagram, drawn in its own metres, stays true at any latitude, so a whole field is placed with one
transform at every zoom, and the water, the route, every field, our traffic and our aircraft
share one set of world coordinates. The server's diagram frame is east and north metres from the
field's reference point; the page re-measures north on Mercator's stretch once when the diagram
arrives, so the field lands within a centimetre of where the server measured each point (pinned
under node by `the_page_places_a_field_where_the_server_measured_it`; without the re-measure a
point 3 km out at 60 degrees north was metres off). The numbers handed to the canvas stay local
(each layer is a `Path2D` in its own small frame), so nothing is lost to precision at stand zoom.

Layers by zoom, each faded in rather than switched:

- Always: the water and coast, the plan's legs and fixes (faded out as a field fills the screen,
  where its taxi route takes over), our traffic (smaller far out) and our aircraft.
- Fields: towered fields as dots, labelled from zoom 7; every field as a dot from about zoom 8.5,
  labelled from zoom 10. The plan's origin, destination and alternate ringed and labelled always.
- Close to a field (about 0.02 px a metre, a 3 km field 60 px across): its diagram is asked for,
  whichever field it is; its runways show once they are a few pixels long, its pavement fades in
  (0.025 to 0.05 px a metre) as its dot fades out, then the centrelines, the taxiway names, the
  holding positions, the stands (coloured, tappable) and their names, at the same scales the old
  diagram used.

The live picture: `map.own` (lat/lon, heading, track), `map.traffic` (every simulated field's
aircraft in lat/lon, wherever they are), `map.stands` (every field our traffic simulates, and the
wanted field with ours on a stand), the plan (`map.fixes`, `map.tod`, `map.fields`, `map.view`),
and for the wanted field (`airport`): the taxi route, the turn card and the runway mark, in its
frame. Our aircraft and the traffic glide between updates; a jump over 400 m is a jump. Taps on
an aircraft or a stand open their cards at every zoom the stand is drawn; Move here works on any
field's stand (the setup tool's gates decide).

Performance: the canvas redraws only when something changed (a finger, a zoom, a glide at most
20 times a second); every layer is a `Path2D` built once per answer or per field; the page asks
`/basemap` and `/airport` only after the view has stood still a quarter of a second, only for
what is under it, two diagrams at a time, and keeps 24 fields. The 500 ms state poll is unchanged.

## The Map tab: the overview and the water

The Overview button frames the state's fit (`map::fit`, pinned by `the_cruise_fits_what_is_ahead`,
`the_last_fix_near_the_field_fits_the_terminal`, `the_ground_fits_the_whole_plan`):

- Airborne within 40 nm of the destination, or within 60 nm below 10,000 ft: the **terminal**
  view, the aircraft and the field only.
- Airborne elsewhere: what is **ahead**, the aircraft, the next fix and the legs still to fly.
  Flown legs stay drawn, dimmed, and never drive the fit.
- On the ground, or with no aircraft: the whole **route**.
- Never narrower than 15 nm either way.

Under everything, water and coastlines from X-Plane's OWN installed map data,
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
leaves the tiles it holds or its zoom moves by about a step, with a margin of 30% (10%, then
none, to stay within 1,200 tiles).

Wider than 1,200 tiles, `/basemap` answers from a coarse **world layer** (`basemap::World`),
in the same wire format, so the coast is drawn at every zoom. It is built once per run when the
EFB first serves (never while it is off; turning the EFB off mid-build cancels it), on its own
thread (`efb-world`), paced to half a core, never on a request or the command loop: every
installed tile read through the same reader, thinned (its sides kept), then each 10x10 degree
block DISSOLVED: the tiles' shared sides cancel (an even-odd fill: a side drawn twice is no
side), the open sea's missing tiles join in, and what is left (the coast, the lakes, the block's
own sides) is chained into rings and thinned on Mercator to 2^-5 degrees of longitude, rings
smaller than that dropped. A wide view gets the blocks under it, thinned again to its own pixel
and rounded to the decimals that pixel needs, under the same 400 kB (coarser until it fits); the
answer's `bbox` is the blocks' tiles. Until the layer is built a wide view is a 503 with
`Retry-After: 5` and the page asks again; the water already drawn stays until a new answer
arrives, so zooming in or out never blanks the map, and the detailed tiles take over as soon as
the view is within 1,200 tiles. The antimeridian is the world's edge (nothing wraps), and
everything beyond the page's 85.05 degrees is flattened onto it for thinning. Logged once
("EFB world basemap built") and blackboxed `efb` `basemap_world` (`ms`, `tiles`, `rings`,
`bytes`). Measured on the real install (17,848 tiles, release): 5.8 s unpaced (16.9 s paced as it
runs live), 53,680 rings, 4.9 MB held; the whole world 322 kB at 2^-2 on 2,560 px or 1,400 px
(102 kB at 2^0 on a phone), Europe 317 kB at 2^-4 on 1,400 px, under 50 ms built, cached after.

No map data: no water,
the fields still come (the field directory is apt.dat's). Land is the map's background, water a muted blue grey (`--water`), the coast a
faint line (`--coast`), in the night palette whatever the theme.

Fields: the plan's origin, destination and alternate are ringed and labelled with their ICAO
always (from the field directory the region index loads from apt.dat, else the plan's waypoint).
Once the view is under 6 degrees across, `/basemap` also sends the towered fields in it (every
land field under 1.5 degrees): drawn as small dots anywhere in the view, the towered ones
lightly labelled; these are also the fields whose diagrams the page asks for once close.

Labels never overlap: a point's label takes the first free spot of eight round it, a diagram's
tag its own box, clear of the labels already placed, the aircraft, the map's buttons, the info
box and a tap card; in order the taxi route's marks and the wanted field's runways (never
dropped), the other fields' runways, taxiway and stand names, the active fix, the destination
and the top of descent (never dropped), the origin and alternate, the fixes ahead nearest first,
the towered fields, the traffic's callsigns, the flown fixes, every other field. One with no
free spot is dropped.

## The Map tab: the fields

Which field: in the air, the plan's destination (else the station's field); on the ground, the
field we are at (the plan's origin or destination, or the station's, within 5 nm), else the
departure (`airport::wanted_field`). The info box says which and why. That field carries the
taxi route, the turn card and the runway mark; every other field in view is drawn too.

Any field's geometry is `GET /airport?icao=XXXX` (token required; 3 to 7 letters and digits),
JSON in a local frame: east and north metres from the middle of the field's runways. It is built
on demand from apt.dat off the server's thread (about 0.1 to 0.4 s; LEMG is 76 kB, EGKK 60 kB,
rings and lines thinned to 0.4 m), two builds at a time: 202 while it is being built (the page
asks again), 404 for a field with no diagram (asked again after two minutes), 200 once built.
The server keeps 24 fields (the wanted field is never evicted: the state's poll keeps it fresh);
its `version` (a hash) is the ETag, and the page state names the wanted field and version
(`airport.icao`, `airport.version`), so the page fetches it again only when it changes. The layers: runways (100 rows, both ends, width),
pavement (110 polygons; where a field draws none, the 1202 network at a nominal 20 m, said on the
page), taxi centrelines (120, types 1 and 51), taxiway names (on each name's longest 1202 edges),
holding positions (where a 1204 runway zone meets a taxiway outside it), stands (1300).

The wanted field's live picture rides the page state (`airport`): the route (`route.driven`, `route.remaining`, `holds`, `dest`; drawn only when it starts within
15 km), the card (`card`: kind, banner text, aim bearing true, cross-track signed, positive right),
the runway (`runway`: cleared, arrival, or the plan's). The card exists only while the copilot's
taxi guide runs (the crew on, `taxi_guide` on, a taxi clearance held).

### Our traffic and the stands (2026-10-13)

With our own traffic running (`XCOPILOT_TRAFFIC=ours`), the map draws every aircraft of the
traffic world, every simulated field's, wherever it is (`map.traffic`, lat/lon), as a violet airliner rotated to its heading, smaller than our own white chevron,
gliding between updates like ours; its callsign beside it in the air, and on the ground once
zoomed in; smaller far out. Tap one for its card: callsign, airline and the callsign as spoken, ICAO type, from and
to (the board's flight, else its plan), what it is doing in plain words (parked, starting up,
taxiing out, holding short, lined up, taking off, climbing out, departing, inbound, downwind, base,
on final, landing, vacating, taxiing in, going around), its stand, its timetable slot and today's
estimate, its altitude in the air and its speed. Tap anywhere else to close it.

**The whole flight, like FlightAware (2026-10-13).** Every timetabled flight to or from a
simulated field is on the map from its off-block at the origin to its on-block at the
destination, not only while the full simulation flies it near the field: far out the
traffic world's whole-flight model places it (`traffic::track`, a climb at the type's rate, a
cruise level by distance, a 3 degree descent, along the great circle), drawn smaller and named
only from zoom 8 (`model: true`). An arrival comes out of the model into the simulation, and a
departure goes the other way, under the same id and at the same point: no aircraft appears or
vanishes. Each aircraft carries its `journey` (`airport::JourneyOut`: `from`, `to`, both ends'
`from_pos`/`to_pos`, the off-block `off` on the origin's clock and the on-block `on` (estimated)
on the destination's, `since_min`, `to_go_min`, `progress` 0 to 1 along the great circle).
Selected, its whole route is drawn: the great circle from the origin to where it is solid (cyan,
under its trail), its route ahead dashed (violet: the simulated route, or for a modelled arrival
the great circle to its entry), then on to the destination, both ends ringed and named. Its card
adds Off blocks (the time and how long ago), Arrives (the time and how long to go) and a Progress
bar, and says "Beyond our simulated airspace: placed by its timetable." for a modelled one.
Nothing new is read from X-Plane: the model is a pure function of the timetable and time.

The stands (apt.dat 1300) are coloured once zoomed in a little: green FREE, red TAKEN, white
ours, grey UNKNOWN. Taken is the traffic world's own rule (`traffic::fleet::Fleet::stand_holder`,
the one the fleet spawns and parks by): an aircraft standing on it, one taxiing in to it, or an
arrival not yet down whose planned stand it is; and our own aircraft on the ground within 20 m of
one. With no traffic world simulating the field nothing is known free: the stands are grey.
Nothing new is read from X-Plane: the traffic world taps its fleets into the EFB's feed about once
a second while the EFB serves (`TrafficWorld::note_efb_traffic`, `efb::feed::note_traffic`,
expiring in 5 s), and the page state carries `map.traffic` and `map.stands` (per field: `icao`,
`known` and the `taken` list; a field nobody knows anything of is left out and drawn grey).

Tap a stand for its card: its name, kind (gate, tie down, hangar, other), the types it takes and
its size code (1301) where apt.dat gives them, free or taken and by whom, and **Move here**
(greyed on a taken stand). Move here asks first ("Move to Stand 24? The flight reloads.", with
the field named when it is not the wanted one) and
then `POST /reposition {"icao", "stand"}` (token required, one per 3 s, blackbox `efb`
`reposition` with `asked`, then the outcome). It is SIM MANAGEMENT, not a crew action: it does not
go through `/say` or the grammar, and no phrase starts it. The endpoint answers 202 at once and
runs `tools::setup::reposition_to_stand` on its own thread, which owns every gate, in order: no
other move under way, no new flight loading, no loop of ours holding the aircraft (the taxi and
the tug included: "Say my controls first"), not in replay, stopped on the ground, the stand in the
field's apt.dat, not on it already, not taken by our traffic. Then a ramp start through the
plugin's mailbox exactly as "set up the flight" does, with THIS aircraft and its engines as they
are (running or not), waited on through the load (`init_seq`/`init_result`), the new flight begun
(`flight_state::flight_begins`), the station for the field put up, and the position READ BACK: on
the stand within 30 m of its 1300 point, else "The move didn't take: we're N metres from Stand
24." The outcome comes back as a toast (`airport.reposition`), and a move or a failure is said
on the intercom as a status line ("We're on stand 24."); a refusal is shown only.

The script is `/map.js` (code, no data, served without the token like the icon). To look at
it without a sim: `EFB_DIAGRAM_OUT=target/efb cargo test --release -- --ignored
real_install_page_states` writes the real LEMG and EGKK diagrams and a taxiing page state.

## The buttons

Every button POSTs the words a pilot would say (`/say`), and the line goes onto the command channel
exactly like the panel's typed line (`pollers::queue_command`; blackbox `typed` `taken` with
`source: "efb"`, and `efb` `say`). The grammar picks the tool, with its gates and readback. There
is no other way in for a crew action: the EFB cannot do anything the voice cannot. The phrases are in
`efb::state::PHRASES`, each pinned by `every_button_routes_where_it_says` to the branch or tool the
command loop takes it to; the two built from a fact (a field's weather, the station's "say
again" and "ready for departure") are pinned in the same test. The one exception is sim
management, not a crew action: the Map tab's "Move here" calls the setup tool's gated function
through its own endpoint (`/reposition`, above), never a phrase. "Fly the flight plan", "Land the
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
- On the Map tab: traffic other than our own (LiveTraffic's, X-Plane's AI), the auto taxi's paint-ridden path (the page draws the route
  as the party line holds it), Custom Scenery airports (only Global Airports' apt.dat is read, as
  everywhere else in the app).
