# Multi-field traffic: design (2026-10-13; stages 1 to 3 built the same day, unflown)

The pilot's ask: every field with a timetable within about 100 nm SIMULATED (its flights
exist, keep their schedule, talk to their own tower, heard only when tuned) and aircraft
DRAWN within about 40 nm (TCAS range). Drawing costs frame time per aircraft; simulating does
not.

## Today (one field)

- `src/traffic_link.rs` `TrafficWorld` holds ONE `link: Option<Link>`, for
  `wanted_field(mode, atc_on, party_line::installed_icao())`: the installed station's field.
  A change of field stands the whole fleet down (`stand_down`, every strip retired) and builds
  a new `Link` off the tick (`build`, `build_link`: apt.dat scan, taxi net, CIFP, then
  `timetable_start` / `traffic::midflow::plan` for a timetable field).
- `Link` owns a `traffic::Fleet` (one `FieldModel`), its due list, its `Outbox` (frequency
  pacing), the board. `Link::tick` steps the fleet, posts calls to the station
  (`crate::world::*`, through `party_line`), reads back orders (`world::traffic_order`).
- The station (`crates/atc/src/station`, `party_line.rs`) is ONE installed station with
  process-wide side state; traffic strips live in `party_line` `STRIPS` keyed by
  `FlightId::Traffic(u32)` with no field in the key; the runway book (`RunwayBook`) is the
  installed field's.
- Rendering: `src/traffic_sink.rs` packs every fleet aircraft (`traffic::wire`) to the plugin;
  `plugin/src/traffic.cpp` makes one XPMP2 aircraft per target. Parked and dormant aircraft
  are drawn too (`schedule::PARKED_MAX` 80 caps the start's parked fill for frame cost).
- The fleet's clock is the sim's running time; above 1x it freezes (`FAST_OVER`).

## What must change

1. **Several fleets at once.** `TrafficWorld.links: BTreeMap<Icao, Link>`: one `Link` per
   timetable field within `SIM_RANGE_NM` (100) of us, built off the tick as today, dropped
   (strips retired) past `SIM_RANGE_NM + 20` (hysteresis so a field on the edge does not
   churn). Field order by ICAO keeps the tick deterministic.
2. **Each field's tower works its own traffic; one station is the pilot's.** The station
   stays the pilot's ONE active station (party_line unchanged as the crew's seam). The other
   fields' traffic is worked by a light **traffic desk per field**: the tower FSM already runs
   per strip (`Station::respond_for`, `StripDesk`, `_for(strip, state)` cores), so a field
   that is not the installed one gets its own `FieldDesk { runway_book, strips }` in the
   world crate, answered synchronously, never spoken. Strips gain the field in their key:
   `FlightId::Traffic { field, id }` (or ids globally unique with a `field_of(id)` map; the
   smaller change, chosen: `Link` ids are allocated from one counter in `TrafficWorld`).
   When the pilot's station changes field, that field's desk is HANDED to the station
   (`Station::install` adopts its strips) instead of retiring them: no flight forgets its
   clearance because we tuned another tower.
3. **Heard only when tuned.** Unchanged in kind: a link's lines go through
   `voice.overhear_checked` with its field's speaker; `tuned_speaker` drops what COM1 is not
   on. A non-installed field's lines need the field in `StationSpeaker` (or the frequency):
   the speech arbiter renders a line only when COM1 is on THAT field's frequency.
4. **Flights between fields.** v1: flown on their timetables, not handed over. A departure
   leaves its fleet on its way out (`Phase::Leave`, `Despawn`) as today; the arrival at the
   other field is that field's own rotation (the timetables do not share airframes). The LXGB
   and LEMG tables already name each other's carriers but not each other as routes, so no
   flight is duplicated. v2 (later): a departure whose destination is a simulated field is
   handed to that link as an arrival at its entry (`Movement` with `Start::Approach`), one
   aircraft, one callsign.
5. **The render cull at 40 nm.** The sink sends only aircraft within `DRAW_RANGE_NM` (40) of
   us, plus every aircraft on the ground of the field we are at; beyond it, nothing (XPMP2
   destroys the plane, `traffic.cpp`). Hysteresis 40 in / 44 out so an aircraft on the ring
   does not flicker. TCAS sees what is drawn. The station's picture
   (`publish_traffic_picture`) carries only the installed field's fleet, as today.
6. **Frame-cost budget.** The plugin's cost is per drawn aircraft; at LEMG's 10:00 the start
   draws about 40 (34 to 44 in the harness mixes) plus 80 parked at most. Budget: `DRAW_MAX`
   120 aircraft, nearest first, parked ones dropped first beyond 15 nm (a dark parked aircraft
   15 nm away is invisible); measure with `api_load` and the fps dataref before raising it.
   Simulation cost is Rust-side: `Fleet::step` substeps each aircraft (`clock::SUBSTEP_S`)
   and spacing is pairwise per field; fields do not see each other's ground, so cost is the
   sum over fields, about linear. Dormant parked aircraft cost nothing but the draw.
7. **Determinism.** Each field's seed from `field_seed(icao, session_seed)` rather than the
   `built` counter, so a field's day is the same whichever field was built first; the board's
   `REMOTE_BOARD_SEED` becomes that seed and a remote board equals the simulated one.
8. **The mid-flow start per field.** Each link starts with `midflow::plan` at its own local
   time (`timetable_clock`); the own-aircraft clearances (`OWN_CLEAR_NM`,
   `OWN_GROUND_CLEAR_M`) apply only where we are.
9. **The board.** `RemoteBoard` becomes unnecessary for simulated fields: the selector shows
   any link's live `board_payload`; the clock board stays for fields in range without a link.

## Smallest architecture

```
TrafficWorld
  links: BTreeMap<String, Link>        one per timetable field in SIM_RANGE_NM
  next_id: u32                         ids unique across links
  sink: TrafficSink                    one, culled at DRAW_RANGE_NM
Link (unchanged inside) + desk: FieldDesk when its field is not the installed station's
party_line: the installed station adopts its field's FieldDesk strips on install
```

## Risks

- The station's side state is process-wide; a second field's strips in `STRIPS` must never
  reach the crew's queries (`player_blocked_strip`, `own_strip_claim`): keep other fields'
  strips OUT of party_line until their field is installed (the desk holds them).
- Handing a desk to the station on a COM1 change mid-clearance: test with the reinstall rule
  (`left_frequency`) harness.
- Frame cost past 60 drawn aircraft is unmeasured (`schedule::PARKED_MAX` note).
- Two apt.dat loads at start (LEMG and LXGB) take seconds off the tick; already off-thread.

## Stages

1. Cull the sink at 40 nm (hysteresis, `DRAW_MAX`), measured live with fps and `api_load`.
   Independent of everything else.
2. Seeds per field (`field_seed`), ids from one counter: no behaviour change at one field.
3. `links` map with fields in range; non-installed links run with NO radio (calls dropped,
   runway decisions by a minimal `FieldDesk`: one landing or departure on the runway at a
   time, the rotation's own runway): the world sweep runs two fields at once, 0 conflicts.
4. `FieldDesk` = the tower FSM per strip; the station adopts it on install; speech per
   field's frequency.
5. (Later) hand a departure to its destination's link.

## Built (2026-10-13, stages 1 to 3, unflown)

1. **The cull** (`src/traffic_sink.rs` `DrawCull`): drawn within `DRAW_RANGE_NM` 40, kept to
   `DRAW_OUT_NM` 44 once drawn, at most `DRAW_MAX` 120 (parked beyond `PARKED_FAR_NM` 15
   dropped first, then the farthest), the roster in id order so the same set is the same
   roster. The "plus every aircraft on the ground of the field we are at" clause was left
   out: every field is far smaller than 40 nm, so it is already inside. The blackbox
   `traffic` `draw` record carries `simulated`, `drawn`, `parked_drawn`, on a change at most
   every 5 s and every 30 s regardless (a record per 3 Hz tick would drown the log). No
   plugin change. Tests: `the_draw_ring_has_hysteresis`,
   `the_draw_cap_keeps_the_nearest_and_drops_far_parked_first`.
2. **Seeds and ids**: `traffic::field_seed(icao, session)` (FNV-1a of the ICAO, one splitmix
   round) with `traffic_link::SESSION_SEED` = 1, a constant, so a field is the same day every
   session and on a revisit (the smallest choice; a per-session seed can come later). The
   remote board takes the same seed (`REMOTE_BOARD_SEED` is gone). `traffic_link::Ids`, an
   `Arc<AtomicU32>` cloned into every link (`Link::share_ids`), peeked for a spawn and bumped
   only when it takes. Tests: `a_fields_seed_is_its_own`,
   `every_fleet_takes_its_ids_from_one_counter`, `a_fields_day_is_the_same_whoever_builds_it`
   (the board by the clock equals the fleet's board).
3. **Several fleets** (`TrafficWorld.links: BTreeMap`, `wanted_fields`, `next_build`):
   the station's field plus every timetable field within `SIM_RANGE_NM` 100 of us, kept to
   `SIM_OUT_NM` 120; still nothing without `ours`, ATC on and an installed station. Built one
   at a time off the tick, the station's field first, then ICAO order; a build that fails is
   not retried while the field stays in range. Only timetable fields are simulated beyond the
   station's (a generated field stays the station's alone), and a desk field always flies its
   timetable (the pilot's `XCOPILOT_TRAFFIC_FILE` is the station's field's only).
   - **The desk** is `traffic::FieldDesk` (`crates/traffic/src/desk.rs`, pure, in the traffic
     crate rather than the world crate: it needs no station and tests as plain data). Its
     claims are computed from the fleet each tick: a takeoff cleared and on the ground, a
     landing cleared from its final until off the pavement, anybody on the strip, a crossing
     cleared in the last 60 s. It clears the first landing asked, then the first takeoff
     (120 s after the last, as `WAKE_SECS`; no arrival inside 90 s of the threshold), then
     crossings (another strip's at once). Taxi and taxi in are cleared at once on the
     network's own route; no readbacks; the runway is the one the station last cleared, else
     the field's default (`schedule::default_runway`, the rotations' own).
   - **No radio**: a desk link (`Link::tick_desk`) never touches `crate::world` or
     `party_line`: no strip, no state, no call posted, no tower line, no type noted; its
     outbox stays empty. Silent whatever COM1 is on (stage 4 brings speech by frequency).
   - **The station moving**: when the installed station's field changes, the old field's fleet
     goes to a desk if it stays in range (`Link::make_desk`: strips retired, the desk adopting
     each flight where its phase says it was cleared, nobody leaves), else retires; the new
     field's fleet, if simulated, goes to the station (`Link::make_station`): its moving
     aircraft leave the frequency (the reinstall rule) until stage 4 hands the strips over.
   - **The board**: the selector's field shows its link's LIVE board when simulated; the
     clock board (`RemoteBoard`) only for a field in range with no link yet.
   - **The blackbox**: every `traffic` record of a fleet names its `field`; `to_desk`,
     `to_station` and `stand_down` (with `why`) mark the lifecycle.
   - Tests: `the_fields_in_range_join_and_leave_with_hysteresis`,
     `the_stations_field_is_built_first`, `a_desk_answers_the_obvious_way`,
     `one_on_the_runway_at_a_time`, `two_fields_fly_at_once_on_their_desks` (XTST and ULMM
     generated hours, every movement done, 0 runway conflicts, 0 overlaps, no id at two
     fields), and the real-install `real_install_two_fields_at_once` (below).

**The sweep, LEMG's station and LXGB's desk at once** (`real_install_two_fields_at_once`,
10:00 local on 2026-07-14, both mid-flow, ids from one counter, never one id at both):

| window | field | completed | touchdowns | conflicts | overlaps | go-arounds | tows |
|---|---|---|---|---|---|---|---|
| 30 min | LEMG (station) | 6 | 10 | 0 | 1 | 0 | 9 |
| 30 min | LXGB (desk) | 1 | 1 | 0 | 0 | 0 | 0 |
| 3 h | LEMG (station) | 26 | 42 | 0 | 6 | 11 | 56 |
| 3 h | LXGB (desk) | 3 | 2 | 0 | 1 | 0 | 0 |

LEMG flown alone over the same windows gives the same numbers (the test asserts completed,
touchdowns, go-arounds and tows equal; 330 calls alone against 329 beside LXGB over 3 h).
The 30 min LEMG row is `real_install_midflow_start`'s at 10:00. Every overlap is a taxi-out
18 to 19 m past a parked aircraft, the stand plan's owed clearance (dev-history 2026-10-13),
LXGB's included; LXGB's desk flew its one 10:20 arrival to the gate where the station's
mid-flow run had it go around once.

Owed from these stages: stage 4 (the station adopts a desk's strips, speech per field's
frequency), the frame cost measured live (`traffic` `draw` against `api_load` and fps).

## Built: the whole flight (2026-10-13, unflown)

The pilot: "an aircraft popped up out of nowhere on the map ... London to Malaga, we should see
its route and its progress, like FlightAware". An arrival spawned 25 to 40 nm out on a random
side, a departure despawned 12 nm out on a random bearing. Now every timetabled flight of a
simulated field exists from its off-block at the origin to its on-block at the destination.

- **The model** (`crates/traffic/src/track.rs`, pure): `Track` from one `End` to another along
  the great circle, taxi `TAXI_OUT_S` 10 min before the wheels up and `TAXI_IN_S` 5 min after
  the wheels on at a field end, a climb at 0.6 of the class's `climb_fpm` over its average
  speed, a cruise at `cruise_level_ft` (100 ft a mile over 5,000, capped by the class: FL370
  narrowbody, FL390 widebody, FL250 turboprop, 9,000 ft piston, on the semicircular rule), a 3
  degree descent (318 ft a mile); a short hop peaks where the climb meets the descent. Each of
  the three segments varies its ground speed linearly with distance from the end's own speed to
  the class's cruise and back, so position by time inverts in closed form. An air end
  past its time HOLDS there (a spawn retried, a runway not yet known): never a jump. `Whole`
  adds the two fields and the block times. No randomness: the flight is the timetable's.
- **One notion of a flight, the model only a source of position.** An arrival still to enter IS
  its `Due` movement in the link; its model (`traffic_whole::Modeled`) ENDS at the very aircraft
  its spawn will make (`Movement::entry_anchor`, the same `at_entry`), at its first due time,
  built once per runway once the entry is within 9 h. The first time it shows it takes an id
  from the world's counter, and the spawn keeps that id (`Due.id`): one roster entry from the
  origin to the gate. A departure that leaves the area (`Fleet::take_left`, its state and the
  substep's fleet time) is flown on by the model from exactly that state, the same aircraft
  value and id (`Link.beyond`), to its destination's blocks. Departures gone before the start
  (`Start::Away`, `midflow::plan` looking back `AWAY_BACK_S` 9 h, keyed by number AND time, a
  route's numbers repeat daily) are modelled from their wheels up at the field. No second
  fleet.
- **The ends face the flight.** `arrival::entry_toward`: an instrument arrival's entry is on
  the bearing from the threshold toward its ORIGIN (`places`), turned onto the approach side
  within `ENTRY_SIDE_DEG` 80; a departure's `exit_bearing` is the great circle toward its
  destination. Unplaced ends keep the old draws (one draw either way: the rest of the start is
  unchanged).
- **The draw ring**: the model's airborne aircraft join the sink's candidates
  (`Link::modeled_drawn`), culled by the same `DrawCull` (40 nm in, 44 out): a departure past
  12 nm and an arrival before its entry stay in TCAS range instead of popping. Never in the
  station's picture (`Link::targets`). The `traffic` `draw` record gains `modeled`.
- **The EFB**: `efb::feed::TrafficAc` gains `journey` (`Journey`: ends, positions, off-block
  on the origin's clock, on-block on the destination's, `places::utc_offset_h`, since, to go,
  progress) and `modeled`; the map draws the selected flight's whole route (`docs/efb.md`).
- **The blackbox**: `traffic` `handover` with `dir` `in` (`off_m`, `off_ft`: the model's fix at
  the spawn against the spawned aircraft, `held_s`, `shown`) or `out` (`to`, `alt_ft`,
  `cruise_ft`, `eta_s`).
- **Cost** (release, on the development MacBook): a track built 0.1 us, a position 0.02 us
  (`the_model_is_cheap`); LEMG at 10:00 on a summer Tuesday (`real_install_lemg_whole_flights`):
  41 departures gone before the start, up to 71 flights modelled at once, a model tick (every
  arrival due within 9 h and every flight beyond) 70 us once a second, the EFB tap 115 us a
  second, the map's traffic up to 47 kB a state; over 90 minutes 26 arrivals out of the model
  and 10 departures into it, no flight ever moving more than 0.2 m or 1 ft beyond a second's
  flight between two seconds.
- Tests: `a_long_flight_climbs_cruises_and_descends_on_the_great_circle`,
  `the_level_follows_the_distance_and_the_direction`,
  `a_short_hop_peaks_where_the_climb_meets_the_descent`, `an_air_anchor_is_met_exactly_and_held`
  (the model); `an_arrival_enters_the_simulation_where_the_model_puts_it`,
  `a_departure_leaves_the_simulation_where_the_model_takes_it` (each handover, within 1 m and
  1 ft); `a_whole_flight_never_jumps_at_either_handover` (a ULMM-Heathrow link on its desk for
  4 h: every id moves no further than its speed each second, model or simulation, never both);
  `a_whole_flight_rides_the_map_with_its_journey` and the map's node test (the page's great
  circle is the server's, point for point).

Owed: the model has no wind, no separation from the simulated traffic or between modelled
flights, and its origin's elevation is taken as sea level (the places table has none); a field
leaving `SIM_OUT_NM` takes its whole flights with it; a remote board (`RemoteBoard`) shows no
model.

## Code facts behind the design (read 2026-10-13)

- One station: `Session.atc_station` (`src/main/station_work.rs`), swapped through the single
  `party_line` `PENDING_STATION` by `region_tick` (`crates/atc/src/station/region.rs`,
  `AT_FIELD_NM` 4 on the ground, `SWITCH_MARGIN_NM` 5), `arm_station_for_freq` (COM1) and
  `arm_new_flight_at`. Every station static is a global singleton (`state.rs` lifetime
  groups); `Station::install` resets `TrafficStrips`, and the reinstall rule then despawns
  every moving traffic aircraft (`Link::tick`, `despawn_moving`).
- Ids: `Link.next_id` starts at 1, `STRIPS` is `BTreeMap<u32, _>`: two links collide at once.
  `RunwayBook::now` (`runway_book.rs`), the sequence number (`ctx.rs`), `stands_assigned` and
  `takeoff_hold.rs` iterate EVERY traffic strip, whatever its field; `traffic_tick` takes AGL
  from `active_field()`; `TOWER_WIND` and `TRAFFIC_TYPES` (keyed by callsign) are single.
  Hence stage 3 keeps other fields' strips out of party_line entirely.
- Speech: `Outbox` and `Voice::pump_overheard` key a line by `StationSpeaker` ROLE (Tower,
  Ground, Approach), resolved through the installed field's `CONTROLLER_FREQS`: a second
  field's "Tower" would play on our tower's frequency. Lines must carry the frequency (kHz).
- Seeds: `TrafficWorld::build` seeds with `self.built` (links built this session), so a field
  revisited gets another day: stage 2.
- Render: `traffic_sink.rs` sends every fleet aircraft, parked included, no distance filter;
  the wire caps at 341 roster / 409 frames (`wire.rs`, silently truncated); `FLAG_HIDDEN` is
  defined and never set; `traffic.cpp` creates or destroys an XPMP2 aircraft per roster id
  and probes terrain per ground aircraft per frame. The cull belongs in the sink (a roster
  change destroys the plane), never by `FLAG_HIDDEN` (a hidden plane still costs its update).
- Cost: `Fleet::step` substeps at 0.1 s; `spacing::apply` is all-pairs per field, so fields
  apart cost the sum, not the square. `FieldModel::load` is two streaming scans of the 400 MB
  apt.dat, uncached: build fields one at a time off the tick.
- Monitor tick about 3 Hz nominal (`XCOPILOT_MONITOR_HZ`), 1.25 s seen live.
- `docs/tasks/pillars-audit/stage-d-traffic.md` lists as not in v1 "strips re-adopted across a
  reinstall; departure or approach handoff for traffic"; this design is that follow-up.
