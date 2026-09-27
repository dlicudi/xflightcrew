# Task: IFR clearance delivery (the tower clears the route, the crew flies what it was given)

Read docs/tasks/COMMON.md first, then CLAUDE.md's "ATC: the world, not the crew"
and "The crew picks its own procedures" sections, and grep docs/dev-history.md
for `RequestClearance`, `choose_departure`, `assigned_squawk` before you start.

## The gap

The station (`crates/atc`) has a tower, approach vectors, a departure position,
ATIS, squawks and readback enforcement, but no IFR clearance. Today
`Intent::RequestClearance` (`crates/atc/src/tower.rs` about line 1076) answers
every "request clearance" with the VFR zone clearance: "cleared to leave the
zone, not above two thousand feet, squawk NNNN", with the readback only
suggested. The crew meanwhile files SimBrief plans, picks its own SID
(`src/full_auto.rs` `choose_departure`, event `departure_rechosen`) and sets
its own initial altitude (`src/takeoff_config.rs` `initial_altitude_ft`: the
caller's level, else field + 3,000). Nobody ever says "cleared to Edinburgh via
the GOSAM one alpha departure, climb five thousand feet, squawk 4521".

## What to build

1. **A new intent** `Intent::RequestIfrClearance { dest: Option<String> }` in
   `tower.rs` `parse_intent`. Shapes: "request IFR clearance to EGPH", "request
   clearance to Edinburgh", "ready to copy", "request clearance as filed",
   "IFR to Edinburgh, ready to copy", with the destination as an ICAO (3 letters
   also with K, as `src/flightplan.rs` does) or as name words matched against
   the airport index the station already has. A bare "request clearance" with
   no IFR word and no destination STAYS the zone clearance: nothing existing
   changes its answer. Add every new phrasing to
   `crates/atc/bench/atc_corpus.yaml`.

2. **The clearance** composed by the tower, dialect-aware through the existing
   dialect seam (`suggest_call`, `qnh_clause` are the models):
   - ICAO / CAP 413: "{callsign}, cleared to {dest} via the {SID} departure,
     runway {rwy}, climb {alt} feet, squawk {code}{qnh_clause}."
   - FAA: "{callsign}, cleared to {dest} airport via the {SID} departure, then
     as filed, climb and maintain {alt}, departure frequency {khz}, squawk
     {code}."
   - The SID from `crates/atc/src/nav.rs` (`sid_for_runway`, `sids_for_route`
     with the runway in use; the crew's CIFP dir is what `sid_minimums` takes).
     No CIFP or no SID for that runway: "after departure fly runway heading" and
     no SID. Never invent a name.
   - The initial altitude: the higher of 5,000 ft and field elevation + 3,000,
     rounded UP to the thousand. One function with a test, so the crew and the
     tower agree on it.
   - The squawk is the one the station already issues (`ctx.squawk`,
     `party_line::assigned_squawk`). The destination unknown to the index is
     answered "say again destination", never cleared.

3. **Readback ENFORCED** on the runway-critical items, the way the runway
   clearances are (`Awaited`, `XCOPILOT_ATC_READBACK`): the SID name (if one),
   the altitude and the squawk must come back; "readback correct" when they
   do; a wrong or missing item earns the correction and another wait; a
   mishear earns "say again", never a clearance (CLAUDE.md rule). The zone
   clearance's suggested-only readback is untouched.

4. **The party line seam** (`crates/atc/src/party_line.rs`): a crate-private
   static and ONE accessor `assigned_clearance() -> Option<IfrClearance>` with
   `dest`, `sid: Option<String>`, `runway`, `initial_alt_ft`, `squawk`. Set
   only after the readback is accepted, retired by `uninstall()` and by the
   swap the same way the squawk is. Never a `pub static`.

5. **The crew flies what it was given** (`src/full_auto.rs`):
   - `get_zone_clearance` asks for the IFR clearance instead of the zone
     clearance when the mission has an IFR plan: a `PlanSource::SimBrief` plan,
     or a local plan whose cruise is above the field's transition altitude.
     The call carries the destination ICAO. The crew reads back the SID,
     altitude and squawk (`circuit::radio_call` is the transmit path; look at
     how the takeoff readback is spoken and owed).
   - `choose_departure` takes the assigned SID when there is one and it serves
     the cleared runway; its own pick is the fallback and stays the behaviour
     with no clearance. The `departure_rechosen` event names the source
     ("cleared" vs "chosen").
   - `takeoff_config` uses the cleared altitude as the caller's level; the
     climb-out targets it; the departure position's "climb {alt}" on check-in
     still supersedes it as today.
   - Blackbox one `atc` event `ifr_clearance` with every item, on both sides.

6. **The panel coach** (`suggest_call`): when the pilot is on the ground with
   a discrete squawk not yet issued and an ATIS letter reported, the suggested
   call may stay the zone clearance; do NOT try to read the crew's plan from
   the atc crate (the seam is one way, crew reads world). Leave a one-line
   note in the report if you think the coach should change.

## Tests

- `tower.rs` matrix: the IFR request at an ICAO field and a FAA field, with and
  without a SID, readback correct, readback wrong altitude, readback missing
  squawk, mishear.
- `station.rs`: the accessor is set only after the accepted readback and is
  gone after `uninstall()` (hold `SIDE_STATE_LOCK`).
- `full_auto`: with a fake station handing down GOSAM 1A the departure is
  GOSAM 1A and the event says "cleared"; with no clearance the old pick stands.
- The corpus gains every phrasing you added; run the STT eval only if the
  models are present, otherwise say so.

## Rules that bite here

- The station returns text and can NEVER actuate.
- Frequencies from apt.dat rows, never hardcoded.
- No em or en dashes in any string.
- Do not touch the sim.
