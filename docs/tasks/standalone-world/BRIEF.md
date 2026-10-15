# The world runs without X-Plane (2026-10-15)

## The pilot's ask

`xfc` starts and the world starts with it: the home field's traffic pushes, taxis, departs and
arrives on real UTC, watched on the EFB with X-Plane closed. Loading X-Plane later puts us INTO
that world, already moving. The EFB is how we observe it, spot traffic and ATC issues, and see
which stands are free before choosing where to load.

No new command, no mode, no state file. The world is already deterministic from the clock
(`traffic::field_seed(icao, SESSION_SEED)`, `traffic_whole` closed form), so restarting `xfc`
at any time rebuilds the same world.

## Decided

1. **The world has its own clock: UTC.** The fleets step on the wall clock, never the sim's
   running time, zulu or date.
2. **The world ticks on its own**, whether or not the sim is there. Our aircraft is an input
   that may be absent.
3. **Loading into the world**: nothing new. A stand held by traffic is never cleared for us;
   the EFB shows the taken stands and "Move here" (`tools::setup::reposition_to_stand`) moves
   us to a free one.

## Today (what ties it to the sim)

- `TrafficWorld::tick` (`src/traffic_link.rs`) is called from the monitor through
  `world_tick::WorldTick` (`src/world_tick.rs`, `src/monitor/run.rs`). No sim, no monitor
  tick, no world.
- Its clock: `TOTAL_RUNNING_TIME_SEC` for `dt`, `sim_zulu_s` / `sim_date_days` for building
  and topping up schedules, `follow_sim_clock` (`CLOCK_JUMP_S`, `Link::reanchor`) to follow a
  sim clock that jumps, `FAST_OVER` to freeze under time acceleration.
- Its focus: `wanted_fields(&active, here, ..)` with `here` our position from the snapshot.
- Already sim-free: the station installs at boot from `XCOPILOT_ATC_AIRPORT`
  (`install_home_station`, `src/main.rs`); `Link::tick` already takes `Option<&OwnState>`; the
  EFB is one pure build over what the app holds.

## What changes

1. **The clock.** One world clock: UTC seconds and date from `SystemTime`. `dt` is its step;
   `zulu` and `day` for building and top-ups come from it. Delete `follow_sim_clock`, its
   reanchor path if nothing else uses it, and the `FAST_OVER` freeze with its spoken line:
   the world never waits for the sim, paused or fast. Code should go DOWN here.
2. **The tick leaves the monitor.** `TrafficWorld` gets its own loop at the monitor's cadence,
   started at boot when `XCOPILOT_TRAFFIC=ours`, ATC is on and a station is installed (as
   `wanted_field` gates it today). The monitor, when the sim is attached, publishes our state
   (what `TrafficWorld::tick` builds into `OwnState` and `note_own_state` today, plus the
   track and the true elevation) into one slot with a timestamp; the world reads it and
   treats a stale or missing one as "no aircraft of ours". `own_arrival`, `note_own_times`
   and `note_own_transmission` run only with our state present.
   - **Must check first**: the ORDER of the world's tick against the monitor's other world
     calls (`runway_watch`, compliance, landing hold) and the command loop's `drain_calls`.
     All meet through `party_line` statics; moving the tick to another thread must not let a
     strip be read half-updated or change who answers first. If a separate loop cannot keep
     that order, say so before building: the fallback is the monitor ticking the world with
     `own = None` while the sim is absent.
3. **Focus without us.** `here` is our position when attached, else the station's field's
   reference point. Arriving in the sim at another field is today's station change path
   (`station::arm_new_flight_at`), unchanged.
4. **Sim-only outputs stay sim-only.** The plugin sink (`traffic_sink`), the Board payload to
   the plugin (`publish_board`) and the spoken lines (`voice.overhear_checked`) run only
   while the sim is attached. Unattached, the calls still reach the transcript the EFB shows.
5. **A sim clock that disagrees is said, never followed.** On attach (and only then), read the
   sim's zulu and date once; if they part from UTC by more than `CLOCK_JUMP_S`, say once:
   "The sim's clock is N minutes off the world's. Set X-Plane to the real time." Blackbox
   `traffic` `clock_mismatch` with both values.

## Not in scope

No persistence file, no EFB field picker (the focus is `XCOPILOT_ATC_AIRPORT`), no plugin
change, no voices while unattached, no change to the traffic's laws or the tower.

## Tests

- Plain `cargo test`: the world with no aircraft of ours, a fixed UTC start, ticked N minutes
  twice, gives the same trace; a departure taxis and leaves, an arrival lands and parks.
- The world clock: a schedule built at a UTC instant equals the one built today from the same
  zulu and date.
- Our state slot: present, stale, absent; stale reads as absent.
- `cargo test --workspace` with `XCOPILOT_BLACKBOX=off`; `atc_world_sweep` still passes.

## Verification (the pilot's)

1. X-Plane closed, `XCOPILOT_TRAFFIC=ours`, EFB on: start `xfc`, the EFB map shows the home
   field's traffic moving within a minute, the stands filling and emptying.
2. Load X-Plane at a free stand there: our aircraft appears, the traffic is drawn where the EFB
   showed it, the tower answers us with its traffic already in the picture.
3. Set the sim's clock an hour off: the mismatch is said once, the world does not jump.

Blackbox to check: `traffic` (`clock_mismatch`, density and build events), `efb`.
