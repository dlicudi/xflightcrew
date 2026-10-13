# Multi-field traffic: design (2026-10-13, not implemented)

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
