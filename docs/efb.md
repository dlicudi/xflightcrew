# The EFB

A browser Electronic Flight Bag served by `xvl` on your LAN, for an iPad, an iPhone or a second
screen: the copilot's knowledge as four pages (Now, Radio, Plan, Weather), and buttons that speak
for you. Code: `src/efb/` (`mod.rs` the server, `state.rs` the page state, `feed.rs` the taps,
`page.html` the page).

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
`apple-mobile-web-app-capable`, so the icon opens full screen with no browser bars. A home-screen
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
| Weather | METAR raw and plain for destination, origin, alternate, with the runway picked or cleared | the sim's real-weather file (`weather::latest_file`, re-read at most once a minute), `full_auto::arrival_runway`, `party_line::clearance_runway`, else the plan's runways |

The plan check and preflight were spoken once and dropped before the EFB; they are now kept in
flight-scoped cells (`efb::feed::AUDIT`, `PREFLIGHT`: a `Scoped` cell, so a new flight never shows
the last one's verdict).

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

## Not on the EFB yet (would need a new sim read or more of the OFP)

- Per-fix airway, ETA and fuel on the Plan page: the OFP navlog has them, the app does not keep them
  (`simbrief::Waypoint` is ident, position, altitude). An OFP parse change, not a sim read.
- The aircraft's fuel against the plan at this fix: needs the navlog's planned fuel per fix (above).
- Flight category (VFR, MVFR) on the weather cards: the parsed METAR keeps cloud as words.
- The tower's runway in use at a field other than the one we are cleared at.
- Wind and temperature at the aircraft, V speeds, landing performance, the checklist and airport
  pages of the design: each needs a new read or a new publisher; left out of v1.
