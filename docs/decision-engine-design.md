# The flight-planning decision engine: investigation and design

Status: proposal, 2026-09-24. Nothing here is built yet. Every fact about the
current code is cited `file:line` as of commit `8dddc5b`; every fact about the
data was checked in the installed X-Plane 12 tree (Navigraph AIRAC 2609 in
`Custom Data/`).

The goal is a second pilot that *knows* the flight: which runway, which
departure, which arrival, which approach, what the terrain allows, and why.
Everything it says must come from a computed fact, and it must be quiet unless
something operationally meaningful changed. No model at runtime; every
decision reproducible from its inputs and testable in plain `cargo test`.

---

## 1. What XFlightCrew already does

The short answer: **a lot, in many places, and they disagree.** Today's EDLP-LOWI
sortie failed in exactly the seams between them (the tower gave the grass
26R, the crew armed 08 while the tower said 26, a circling procedure was
treated as a straight-in).

### Runway selection: seventeen deciders, one shared primitive

| Decider | Where | Wind source | Hard filters | Ranking |
|---|---|---|---|---|
| Wind pick | `crates/atc/src/nav.rs:2087` `best_runway_for_wind` | caller | hard surface, ≥ ½ longest (`assignable_runways`, :2114) | headwind only |
| Tower runway in use | `crates/atc/src/station.rs:1203` | datarefs; field METAR when airborne | frozen when committed; hysteresis 3 kt | headwind; arrival stream and the pilot's stated runway **bypass** the hard filter |
| ATIS runway | `station.rs:2166` | datarefs | wind pick | headwind, **no hysteresis**, can disagree with the tower |
| Full-auto arrival | `src/full_auto.rs:2686` `build_arrival` | destination METAR else datarefs | assignable; flyable approach; other end ≤ 10 kt tail | headwind ≥ 4 kt, else alignment; **no landing-distance check** |
| Tower's arrival runway | `full_auto.rs:2801` | – | must yield a flyable plan | – |
| Local planner | `src/flightplan.rs:582` | METAR | length ≥ landing distance (×1.0), **no surface filter** | wind angle / alignment |
| Autoland self-pick | `src/autoland.rs:235,277` | METAR per field | hard surface, ≥ landing distance ×1.3 (min 1,200 m) | distance + turn + tailwind penalty |
| Planned/cleared end | `autoland.rs:337`, `:2165` | – | hard surface, **no length check** | nearest |
| Procedure tool | `src/tools/procedure.rs:171` | **none** | min length **0**; forced ident has **no ICAO check** | distance + turn |
| Line-up fit | `src/autotakeoff.rs:446` | – | aligned, offset, remaining ≥ needed; **no surface** | heading error |
| Line-up target | `src/autotaxi.rs:1136` | – | cleared ident; backtrack rules; **no surface** | nearest |
| "Which runway?" | `src/answers.rs:557` | **datarefs, even for the destination** | wind pick; calm lists every end incl. grass | headwind |
| Approach guide | `src/tools/approach.rs:187` | – | ident by **number only** | nearest |
| Preflight | `full_auto.rs:3004`, `flightplan.rs:846` | – | **longest runway of any surface**, ×1.0 | – |

Inconsistencies found:
- Two hard-surface definitions: `nav::Runway::is_hard` accepts codes 0 and 15, `autoland::is_hard_surface` (`autoland.rs:48`) rejects both. The former is mine, from today, and is wrong about 15.
- Calm thresholds of 3, 4 and 5 kt in different deciders.
- Landing factors ×1.0, ×1.3, or none.
- **No decider uses crosswind.** The profiles carry `max_crosswind_kts` (`src/aircraft.rs:1319`), but only a spoken answer reads it (`answers.rs:1088`).
- Ident matching treats "26" as matching "26R" (`nav.rs:2419`).

### Procedures: approaches yes, SIDs barely, STARs never

- **Data parsed**: CIFP APPCH/SID legs (`nav.rs:2307 parse_proc_legs`). It keeps only the fix, leg type, course, distance, altitude, descriptor and FAF flag.
- **Approach choice** (`nav.rs:2436 best_approach`): rank by ident letter (I > R/X/Q > L > D/S/V), gated since today by `straight_in_legs` (exact runway fix, no RF before it, cut at threshold). Falls back to a DME-fix reconstruction (`nav.rs:2634`). Interceptability is checked (`procedure.rs:854 faf_off_course`, `:977 approach_flyable_from`).
  - **Approach transitions are never flown**: loaded with `transition=None`, so the plan starts at the fix before the FAF.
- **SID** (`nav.rs:2487 sid_for_runway`): the **first SID in the file** for the runway, voice-only ("fly the SID"). Full auto never uses it. Enroute transitions are excluded, and parallel "RW26B" transitions are dropped.
- **STAR**: `ProcKind::Star` exists (`nav.rs:2216`) and nothing calls it.
- **SimBrief SID/STAR** reach the follower only as navlog waypoints (`simbrief.rs:381`, `procedure.rs:908 enroute_legs`). They are flown direct-to with SimBrief's profile altitudes; no CIFP constraints apply.
- **The follower has no leg-type dispatch** (`procedure.rs:1090-1775`; `leg_type` is only logged). Every leg with a fix is flown direct-to. RF arcs are chorded, holds are not flown, and CI/VI/VM/FM legs with no altitude are skipped instantly.
- **Autoland flies a fixed 3° path** (`autoland.rs:2338,2565`). LOWI's charted glideslope is 3.77°.

### Terrain: one excellent sensor, used locally

- **The sensor** is the plugin's `XPLMProbeTerrainXYZ` against the sim's own mesh (`plugin/src/drefs.cpp:563-605`, `src/terrain.rs`). It takes at most 24 points per query, runs one query at a time, takes ≥ 100 ms plus a frame, and has **no cache**.
- **Consumers**:
  - the follower's 15 nm floor ahead (`procedure.rs:443-474`), which is **skipped on approaches** (`:1419`);
  - the circuit's side pick, per-leg probes and a "TAWS-lite" forward guard (`circuit.rs:1879,1405,2359`);
  - the planner's cruise level (`flightplan.rs:450`);
  - the local route and airborne rejoin, both fail-closed;
  - the helicopter route;
  - autoland's threshold elevation (`autoland.rs:2743`).
- **Unused but installed**:
  - `Custom Data/earth_msa.dat`: sector MSAs per fix. LOWI's four 25 nm sectors are at 10,700 / 11,300 / 13,500 / 14,300 ft.
  - `earth_mora.dat`: a 1° grid MORA.
  - `earth_hold.dat`: published holds.
  - `earth_aptmeta.dat`: transition altitude and level per airport.
  - `Resources/default data/DOF.DAT`: FAA obstacles, US only.
  - The repo references none of them.

### Weather

- **Source**: the newest `Output/real weather/metar-*.txt`, read on demand (`src/weather.rs:39-54`). It counts as stale after 2 h.
- **Parsed**: wind, gust, visibility, CAVOK, clouds, temperature, dew point, QNH.
- **Not parsed**:
  - clouds as numbers (they're kept only as spoken strings);
  - the variable-wind band;
  - RVR;
  - anything after TEMPO/BECMG;
  - TAFs.
- **Uses**:
  - The tower's surface wind is fed once a minute (`monitor.rs:3919`).
  - The arrival wind is read **once**, when the arrival is built (`full_auto.rs:1995`).
- **No weather-against-limits check exists anywhere.**

### Reassessment and announcing

- **The monitor's `Check` trait** (`monitor.rs:588`) gives sustained, once-per-episode callouts with repeat levels and fix actions.
- **Rapport** (`rapport.rs`) holds asides until the flight deck is calm (`CALM_S` 8 s, "By the way" after 20 s, dropped after 5 min), but has **no public API** for other modules to queue one.
- **`tempo::busy_now`** and **`persona` chatter levels** gate what gets volunteered.
- **Autoland watches the landing clearance every tick** and re-arms on a new end (`autoland.rs:2165`). That is the only continuous reassessment of runway or procedure today. Everything else is decided once.

### Explainability

The blackbox records many decisions with their numbers:
- `self_pick`, `arrival_procedure` (with wind source), `runway_from_clearance`, `tower_runway_unflyable`, `preflight`;
- the autotakeoff `NoFit` reasons.

What's missing:
- The tower's runway pick is not logged with its reasons.
- No decider records the *alternatives it rejected*.
- Debrief does not summarise terrain events.

---

## 2. Data we already have (and mostly ignore)

| Data | Where | Used today | Useful for |
|---|---|---|---|
| Runway geometry, width, surface | apt.dat row 100 (two parsers: `nav.rs:448`, `autoland.rs:87`; the latter drops width) | geometry, surface | suitability |
| Displaced threshold, blastpad, markings, approach lights | apt.dat 100 per-end fields | **no** | landing distance available, aim point |
| **Threshold elevation, gradient, displaced threshold, ILS ident + category** | CIFP `RWY:` records (`RWY:RW26 ,+0198, ,01894, ,OEV,1`) | **no** | slope, LDA, ILS category, aim elevation without a probe |
| Approach service levels (LPV/LNAV/VNAV) | CIFP `PRDAT:` | **no** | approach capability match |
| RNP per leg | CIFP field 10 (e.g. `302` = 0.30 nm) | **no** | RNP AR detection (a hard filter) |
| Speed limits | CIFP fields 26/27 (113k legs) | **no** (deliberately skipped, `nav.rs:2206`) | energy planning, STAR/SID compliance |
| Vertical path angle | CIFP field 28 (e.g. `-350` = 3.5°) | **no** | glidepath, TOD on approach |
| Turn direction | CIFP field 9 | **read from the wrong field** (`nav.rs:2343` reads 12) | holds, course reversals |
| RF centre fix and radius | CIFP fields 17, 30-33 | **no** | flying arcs, or knowing we can't |
| Glideslope angle | earth_nav row 6 (`377257.811` = 3.77°) | tower vectors only | autoland path |
| SBAS/GBAS endpoints with TCH | earth_nav rows 14/16 | **no** | LPV path, threshold crossing height |
| MSA sectors | `earth_msa.dat` | **no** | terrain sector summary, arrival sanity |
| Grid MORA | `earth_mora.dat` | **no** | enroute terrain floor without a probe |
| Holds | `earth_hold.dat` | **no** | published holding |
| Transition alt/level | `earth_aptmeta.dat` | **no** (uses SimBrief's, else the profile's) | altimeter rule without a plan |
| Obstacles | `DOF.DAT` (US only) | **no** | US obstacle margins |
| Terrain mesh | plugin probe | yes, locally | everything terrain |
| METAR | real-weather files | wind (and a spoken briefing) | runway wind, weather gates |
| SimBrief OFP | `simbrief.rs:180-282` | navlog, fuel, weights, transitions | parsed but **not**: `plan_rwy`, SID/STAR idents, navlog `stage`/`is_sid_star`, OFP weather (METAR/TAF), alternates' detail |
| Aircraft capability/performance | `profiles/*.yaml` | distances, speeds, `ils` flag (tool registration only) | the "aircraft" half of every filter |

Profile coverage of performance is uneven:
- **Full set** (roll, takeoff distance, landing distance, crosswind): A330, 737, 747, SF50, Citation X.
- **Partial**: the MD-82 lacks a crosswind figure; the Baron, 172, Lancair, RV-10, SR22 and King Air each have some.
- **None**: DC-3, Twin Otter and S-76 carry no performance figures.

---

## 3. What is missing

1. **One place where runway suitability is decided.** This is the root of today's bugs.
2. **A decision record**: chosen, reasons, rejected alternatives with their reasons. Needed for the blackbox, the panel, "why?" answers and tests.
3. **Crosswind and tailwind limits as filters.** The data exists; no decider uses it.
4. **Correct procedure parsing**: turn direction, speed limits, vertical angle, RNP and RF data.
5. **A leg-type-correct follower.** TF should fly the track, CF the course, RF the arc, and holds should be held. Until it exists, STARs and SIDs cannot be flown *as published*, only as waypoint chains.
6. **Approach transitions and STAR joining.** Today the approach starts at the IF with no path onto it.
7. **Minima: they are not in the data.** There are no DA/MDA/visibility values anywhere in the installed files. Any ceiling/visibility-versus-minima judgement would be invented; see §10.
8. **Aircraft capability flags**: RNP/RNP AR, LPV/SBAS, GLS, autoland category. The profiles only carry `ils: bool`.
9. **A terrain service**: cache, batching, and published MSA/MORA as a first layer.
10. **Change detection** for METAR, runway in use, route deviation and destination. Everything is polled ad hoc or decided once.

**Where the brief's assumptions need correcting:**
- *"Select SIDs and STARs."* In practice **SimBrief already selects them** (real dispatch logic, the runway it planned, the route). ATC assigns them in the real world, and when the pilot flies with real ATC (copilot mode) the frequency owns them. The engine's high-value job is **validation and repair**: is the planned SID/STAR compatible with the runway in use, does it connect, does it clear terrain; re-select a transition or procedure only when the runway changes. Choosing from scratch is the fallback, for local plans and "direct" plans.
- *"Visibility/ceiling versus minima."* See §7 and §10: no minima data. We can compare against **published categories we do know** (an ILS CAT I needs roughly 200 ft and 550 m; a non-precision approach needs substantially more), but only as a coarse, explicitly labelled heuristic. Better: say nothing about minima.
- *"Terrain awareness needs a new data source."* It mostly doesn't: the plugin probe **is** the sim's ground truth, and MSA/MORA are already installed. An offline DEM is only worth it for wide-area summaries the probe cannot afford (§6).
- *"ILS preferred to RNAV."* This is already the rank. Note on RNP: LOWI's RNAV 26 Z carries RNP 0.30 on its final legs, which is **standard for RNAV (GNSS) approaches**, not an authorisation-required marker. RNP AR approaches are their own ARINC route type ("H"), and none exist in the installed data. The open question is the copilot's own lateral accuracy on a 0.3 nm final in a valley, which is a flight-test item, not a data filter.

---

## 4. Proposed architecture

Three layers, placed so that nothing breaks the existing crate boundaries. The
`atc` crate must stay free of crew concerns and of whisper/cpal/ort, and the
tower must be able to use the same runway rules.

```
crates/atc/src/                       (pure data + pure rules; tower and crew both use)
  nav.rs            fix parse bugs; full CIFP leg fields (turn, speed, VPA, RNP, RF)
  navdata/
    rwy_meta.rs     CIFP RWY + apt.dat per-end fields → RunwayEnd { tora/lda, thr elev,
                    gradient, displaced, surface, width, ils cat, lighting }
    msa.rs          earth_msa.dat sectors         (lazy, per airport)
    mora.rs         earth_mora.dat 1° grid        (loaded once)
    holds.rs        earth_hold.dat
  suitability.rs    RunwayNeeds + wind components → Verdict for ONE runway end
                    (the single hard-filter rule set; the tower calls it with generic needs)

src/advisor/                          (crew side: the engine)
  facts.rs          Situation: aircraft (profile, weight, capability, position, energy),
                    airport facts, weather facts (METAR + age), ATC facts (party_line),
                    plan facts (SimBrief/local); built from IO, then pure
  decision.rs       Decision<T> { chosen, reasons: Vec<Fact>, rejected: Vec<(T, Vec<Reject>)>,
                    inputs_fingerprint }
  runway.rs         candidates → filters (suitability.rs) → rank → Decision<RunwayEnd>
  approach.rs       per runway: procedures → filters (straight-in, capability, RNP, RF,
                    interceptable, terrain) → rank → Decision<Approach>
  departure.rs      SID validation/selection + transition to the route
  arrival.rs        STAR validation/selection + transition + approach join
  terrain.rs        TerrainService (MSA/MORA layer + cached probe layer), summaries
  reassess.rs       the loop: fingerprint, materiality, hysteresis, announce/act
  explain.rs        Decision → spoken line / panel lines / blackbox JSON
```

**Principles:**
- **Pure core, IO at the edges.** This is the pattern `instructor::SyllabusIo` and `FakeXPlane` already use. Every rule is a pure function of a `Situation` snapshot, so every decision is a unit test.
- **The advisor recommends; loops act.** Full auto, autoland, the follower and the tower *consume* decisions; the advisor never touches a dataref. This keeps "one writer per dataref" and the actuation gates intact.
- **One decision, many consumers.** The seventeen deciders become callers of `runway::decide` / `approach::decide`, with the caller's own constraints (the line-up still checks it is *pointing down* the runway; it just stops re-deciding suitability).
- **Facts are typed.** A `Fact` is an enum carrying its numbers (`Headwind { kts }`, `Tailwind { kts, limit }`, `Crosswind { kts, limit }`, `LandingDistance { need_m, avail_m }`, `ProcedureClass(Ils { cat })`, `TrackMiles { extra_nm }`, `TerrainClearance { ft, where_ }`…). A spoken line is rendered *from* facts, so a sentence cannot contain a number that was not computed.

---

## 5. The deterministic decision model

### Shape

```
candidates ──► hard filters (each → Pass | Reject(Fact))
            ──► rank survivors LEXICOGRAPHICALLY on bucketed keys
            ──► hysteresis against the incumbent
            ──► Decision { chosen, reasons, rejected }
```

**Why lexicographic over bucketed keys rather than a weighted sum:**
- It is explainable: "27 over 09 because tailwind; 27 ILS over 27 RNAV because class".
- It is stable: buckets absorb noise.
- It is testable: monotonic by construction.

Weighted sums hide why and flip on noise, which is exactly the parallel-runway 27L/27R flip-flop fixed in `station.rs:1266`.

### Runway (landing; takeoff mirrors it with takeoff distance)

Hard filters (each rejection carries its numbers):
1. Surface hard, unless the airframe declares grass capability, or the field has only soft runways.
2. Length available (LDA from displaced threshold, CIFP/apt.dat) ≥ landing distance × factor (one factor, the airframe's, default 1.3 → `aircraft.rs`); takeoff: TORA ≥ `takeoff_needed_m`.
3. Width ≥ airframe minimum (new optional profile field; skipped when unknown).
4. Crosswind ≤ `max_crosswind_kts` (gust-included when a gust is reported); **skipped, and said so, when the profile lacks it**.
5. Tailwind ≤ airframe limit (new field, default 10 kt).
6. Closed or unusable: not in data. Assumed open.

Rank (lexicographic):
1. Assigned by ATC (a clearance or the tower's runway in use): the tower is the authority when it runs. Our job is then to *check* it and speak up only if it fails a hard filter.
2. Headwind bucket (5 kt buckets; any tailwind below any headwind).
3. Best available approach class for the airframe (ILS > LPV > RNAV/LNAV > LOC > VOR > visual), a lower key only when the weather is not VMC (see §10 on what "not VMC" can honestly mean).
4. Route fit: track miles from the arrival point (STAR end / plan's last fix) to the FAF.
5. Length margin bucket.
6. Stable ident order (deterministic tie-break).

Hysteresis: an incumbent is replaced only by a candidate that wins on a *higher* key, or on the same key by more than one bucket.

### Approach (for the chosen runway)

Hard filters:
- Straight-in to the exact runway (today's `straight_in_legs`).
- Leg types the follower can fly (no RF until it flies arcs).
- **RNP AR** (ARINC route type "H") needs a declared airframe capability. Default: none, so AR approaches are rejected with that reason. Ordinary RNAV finals at RNP 0.3 are normal and pass.
- Interceptable from the present state (`faf_off_course`, `approach_flyable_from`).
- Energy: the FAF altitude is reachable from here at the airframe's descent rate within the track miles available (numbers exist in `procedure.rs` TOD logic).
- Terrain: each segment's published altitude ≥ probed terrain + margin (a *data sanity check*, since the published altitudes already embed obstacle clearance).

Rank:
1. Class: ILS (by CAT from `RWY:`), then LPV (PRDAT), then LNAV/VNAV, then LNAV, then LOC, then VOR/NDB.
2. Continuity with the arrival: track miles from the STAR/transition end to the IF.
3. Vertical guidance (a VPA published).
4. Ident order.

### SID (validate first, select second)

- **Validate** the plan's SID (new: parse the SimBrief SID ident and `plan_rwy`):
  - runway compatibility (including "RW26B" parallel transitions);
  - its end fix is the route's next fix, or within N nm of the route line;
  - altitude constraints are achievable at `best_climb_fpm` and the weight-scaled climb gradient;
  - terrain along the SID ≥ constraints.
- **Select** only when there is no SID, or the runway changed:
  - candidates are SIDs with a transition for the runway;
  - filter by endpoint connectivity (end fix on the route within X nm and ahead of us along it);
  - rank by track miles (end fix to the first enroute fix), then constraint difficulty, then ident.

### STAR (validate first, select second)

Same shape as the SID:
- endpoint to the approach's IAF/IF connectivity (route type A transitions);
- entry fix on the route;
- constraints achievable from the cruise level and TOD;
- terrain.

Transition choice: the one whose first fix is the plan's entry fix, else the nearest-ahead one.

### Worked example (LOWI today, with stage 0 applied)

Runway 08:
- 6 kt headwind, 2,000 m, ILS: none.
- Approaches:
  - RNAV 08 V: rejected, no runway fix (circling).
  - RNAV 08 Z: rejected, RF before the threshold.

Runway 26:
- 6 kt tailwind (≤ 10 kt limit), 2,000 m.
- Approaches:
  - RNAV 26 Z: straight TF legs to RW26, RNP 0.30 (standard for RNAV): **valid**.
  - RNAV 26 E: rejected, no runway fix.
  - LOC 26: rejected, no runway fix.

Decision: **RNAV 26 Z**, with reasons "08 has no approach we can fly; 26 has a
straight-in RNAV; 6 kt tailwind within the 10 kt limit; 2,000 m against the
landing distance". This is what the arrival builder now picks (stage 0 work
done the same day).

---

## 6. Terrain awareness

**Layered sources, cheapest first:**

| Layer | Source | Cost | Use |
|---|---|---|---|
| Published | `earth_msa.dat` (per-fix sectors, 25 nm), `earth_mora.dat` (1° grid) | file read, once | sector summaries, enroute floor sanity, "don't descend below MSA off-procedure" |
| Probe | plugin `xcopilot/terrain/*`, the sim's own mesh | ≥ 100 ms per 24 points, frame time | precise profiles along a path, corridors, thresholds |
| Obstacles | `DOF.DAT` (US only) | file read | US-only additive margin; elsewhere, not claimed |

**TerrainService** (`src/advisor/terrain.rs`) wraps `src/terrain.rs`:
- **Cache**: positions are quantised to about 250 m cells; LRU of a few thousand cells, cleared on a large reposition. Profiles along the plan are computed once and reused.
- **Batching**: queue points and send 24 per query. Background priority, never on a control tick; the existing `MAILBOX` serialisation stays.
- **Honest failure**: `None` is "no measurement", never a number (the CLAUDE.md rule). Consumers decide fail-closed (planning) or fail-open (advisory), as today.

**Services**, designed around the existing consumers:
- `profile(path, spacing) -> Profile { samples, max_ft, max_at, min_clearance_vs(alts) }`: replaces the ad hoc samplers in `flightplan.rs`, `circuit.rs`, `heli_route.rs` and the full-auto rejoin.
- `corridor(path, half_width_nm)`: two offset profiles plus the centreline. Used for "significant terrain north of track".
- `sector_summary(airport) -> [(bearing_from, bearing_to, msa_ft)]`: straight from `earth_msa.dat`, which is where "terrain north" statements should come from (published, not inferred).
- `procedure_summary(plan)`: per leg, the published altitude minus the probed maximum along the leg. This is the sanity check that replaces today's blanket *skip terrain on approaches* (`procedure.rs:1419`) with *check the approach once, at build, and refuse on a violation*.
- `predicted_clearance(trajectory)`: the follower's projected path (VS × time, GS × time) against the profile. It generalises the circuit's TAWS-lite and the follower's floor.

**The deterministic observations this enables** (each a rendered `Fact`):
- "MSA north of the field is 14,300 ft; we're 11,000 ft, 18 nm north. Staying on the arrival." Here `sector_summary`, the aircraft position and the plan's lateral path are all computed.
- "Terrain along the next leg peaks at 9,850 ft, 1,150 ft below our level." From `profile`.

---

## 7. Continuous reassessment

**Inputs and their change detectors** (all polled cheaply at 1 Hz in one `reassess` task, not scattered):
- METAR file mtime, and parsed wind/visibility deltas.
- Tower runway in use. This needs a new `party_line::runway_in_use()` accessor, per the one-page seam rule.
- Clearance runway and state (`party_line::clearance_*`).
- Plan identity: destination, SimBrief OFP id, divert.
- Aircraft state:
  - cross-track from the plan beyond a limit;
  - energy vs the path (existing `high_on_profile` math);
  - terrain clearance ahead.

**The loop**:
1. Build a `Situation` and hash the *decision-relevant bucketed inputs* (the fingerprint).
2. If the fingerprint is unchanged, do nothing.
3. If it changed, re-run `decide()`.
4. If the decision's **chosen item** changed, apply materiality (below).
5. If the decision is the same but a reason crossed a limit (tailwind now over the limit), that is also material.

**Materiality** (rules, not scores), in increasing weight:
1. A hard-filter failure of the current plan: announce now (Caution), whatever the phase.
2. A changed runway or approach, stable for 60 s: announce as an aside when calm (rapport's machinery, which needs a public `rapport::aside()` entry); act if an automation owns the arrival and the new choice is flyable.
3. A changed rank only within buckets: never announced, blackboxed.
4. Sterile or busy (`tempo::busy_now`, below 1,000 ft, a landing loop running): only (1) is spoken.

Rate limit: at most one advisor line per 3 minutes unless severity rises. Nothing is said twice without an input change (the monitor's episode rule).

---

## 8. Integration with the copilot, voice and events

- **Consumers switch to the engine incrementally.** Each of the seventeen deciders keeps its call shape but asks `runway::decide` / `approach::decide`. The tower calls the atc-crate `suitability` with generic needs, so tower and crew share rules without the tower reading the crew.
- **Voice**:
  - **Asked**: "why runway 26?", "what approach are we flying?", "what are the alternatives?" answer from the last `Decision` (`explain.rs`). They are deterministic: the closed-vocabulary grammar pattern in `answers.rs`.
  - **Volunteered**: only through reassessment materiality, through rapport asides, gated by persona chatter (advisories only; safety lines never gated).
- **Panel**: the Route tab gets a "Plan" block (runway, approach, one line of reasons, rejected count) fed through the existing `auto_status` mailbox style.
- **Blackbox**: `advisor` events carry the full `Decision` JSON (chosen, reasons, rejected, fingerprint). **Debrief** gets a section listing each decision and each change, with its reason. The tower's runway pick gets the same treatment.
- **Full auto**: `brief_arrival` renders the arrival `Decision` at the gate (refusal included); `build_arrival` *is* `approach::decide`; the arm refuses when no approach survives and the destination is in terrain (MSA above field + 3,000 ft, say), instead of falling back to a visual.

---

## 9. Data sources, APIs, libraries

**Already installed (prefer these; no network, no licence change):**
- The CIFP full fields, `RWY:`, `PRDAT:`;
- earth_nav rows 4/5/6/14/16;
- `earth_msa.dat`, `earth_mora.dat`, `earth_hold.dat`, `earth_aptmeta.dat`;
- apt.dat full row 100 plus 1302;
- `DOF.DAT`;
- the METAR files;
- the plugin probe.

The Navigraph cycle 2609 is current; the per-*file* fallback to `Resources/default data` misses 1,487 airports that exist only there. Fix it to fall back **per airport** (`src/config.rs:280-320`).

**SimBrief OFP (already fetched, fields unparsed):**
- `origin/plan_rwy`, `destination/plan_rwy`;
- the SID/STAR idents in the `general`/`navlog` records;
- navlog `stage`, `is_sid_star`, `via_airway`;
- `weather` (origin/destination METAR and TAF);
- alternates.

This is the cheapest high-value addition: the planned runways and procedures, and a TAF.

**Network (policy decision needed).** CLAUDE.md allows one outbound call besides X-Plane (SimBrief).
- `aviationweather.gov` METAR/TAF would add a second. The OFP's weather section may be enough.

**Offline DEM (only if wide-area summaries prove necessary).**
- Copernicus GLO-30 or SRTM tiles, read with the pure-Rust `tiff` crate.
- Heavy on disk, and the sim's mesh is what the aircraft actually meets.
- Not recommended before the probe cache is proven insufficient.

**Not recommended:**
- Navigraph Charts API: it would supply minima, but it is licensed, needs the user's account, and breaks "entirely local".
- OurAirports: duplicates apt.dat.
- Any library for procedure logic: ARINC 424 leg semantics are small, and ours to own and test.

---

## 10. Where we must NOT recommend

The engine says "I don't have the data for that" rather than guessing:
- **Weather vs approach minima.** No DA/MDA/RVR minima exist in the data. At most, report the METAR ceiling and visibility plainly. Never "above minima".
- **RNP AR / RF / LPV capability.** Not in profiles today. Default to *not capable*, and say which approaches that excluded.
- **Runway condition**: contamination, braking action, closures, NOTAMs, work in progress. No data. Assume open and dry, and say so if asked.
- **Performance for airframes without figures** (DC-3, Twin Otter, S-76, partially others). Length checks are skipped with an explicit "no landing distance on file".
- **Crosswind limits without a profile figure**: skip the filter and mention it; no invented default.
- **Obstacles outside the US**: DOF is US-only. Terrain statements elsewhere are about terrain, not obstacles.
- **ATC preference, noise abatement, slot/flow**: not modelled. When real ATC is in use (copilot mode), their assignment wins and we only check it.
- **A stale METAR** (> 2 h): wind-based choices fall back to the tower's runway or alignment, and the reason says "weather stale".
- **Procedures whose legs we cannot fly** (RF, holds, CI/VI/VM before the follower learns them): never selected; named as rejected.

---

## 11. Staged implementation plan

Highest value first. Every stage ends with plain `cargo test` green and a blackbox event to verify in flight.

**Stage 0: correctness fixes found by this investigation (small, now)**
- `nav.rs:2343`: turn direction from field 9, not 12.
- One hard-surface definition (autoland's; drop 0 and 15 from `Runway::is_hard`).
- `runway_matches`: a side letter must match when either side names one (26 ≠ 26R).
- **RNP AR filter on approaches** (route type "H" without a declared capability), and a gate refusal for a terrain destination with no flyable approach. (Built since: the mountain-field hand-over.)
- `tools/procedure.rs` forced ident: ICAO-scoped, hard surface, landing length.
- Parse the CIFP speed limit (fields 26/27) and the vertical angle (field 28); unused yet.
- Localizer bearing decode (`nav.rs:63,137`) and `find_ils` side match (`nav.rs:49`).
- CIFP/nav data fallback per airport, not per file.
- Autoland glidepath from the charted angle (earth_nav GS / CIFP VPA), capped to the airframe's limit.

**Stage 1: runway suitability and decision records**
- `atc::suitability` plus `RunwayEnd` metadata from CIFP `RWY:` and apt.dat per-end fields (LDA, threshold elevation, gradient, displaced threshold).
- Profile fields: `max_tailwind_kts`, `min_runway_width_m`, `landing_factor`.
- `Decision` / `Fact` types, blackbox `advisor`, a debrief section.
- Migrate the tower, ATIS, `build_arrival`, local planner, self-pick, procedure tool, preflight and "which runway?". The line-up and the clearance switch keep their geometry checks but call suitability.

**Stage 2: approach decision**
- `approach::decide` with capability (profile `rnp_approach_nm`, `lpv`, `ils_cat`), class ranking from `RWY:` / `PRDAT:`, energy and interceptability.
- Approach transitions (route type A) chosen from the arrival point.
- "Why this runway / approach?" answers; the panel Plan block.

**Stage 3: terrain service**
- MSA/MORA parsers; the cached, batched probe; `profile`, `corridor`, `sector_summary`, `procedure_summary`, `predicted_clearance`.
- Migrate the existing samplers; replace "skip terrain on approach" with the build-time procedure check.

**Stage 4: SID/STAR validation (with SimBrief fields)**
- Parse `plan_rwy`, the SID/STAR idents and navlog stages.
- Validate against the runway in use; re-select a transition or procedure on a runway change.
- **Prerequisite for flying them as published**: the leg-type follower (TF track, CF course, RF arcs, HM holds, CI/VI intercepts). This is a flight-law change, so it gets its own sortie-test campaign (second-order plants, per CLAUDE.md).

**Stage 5: continuous reassessment**
- The `reassess` task, fingerprints, materiality, a rapport aside API, the `party_line::runway_in_use` accessor, and METAR change detection.

**Stage 6 (optional): data beyond the install**
- OFP weather/TAF; a METAR network source (a policy decision); a DEM only if stage 3 proves the probe insufficient.

---

## 12. Tests that prove it

- **Pure filters**: a table test per filter, including every boundary (tailwind exactly at the limit, LDA exactly at need × factor, crosswind with a gust).
- **Monotonicity (property tests)**:
  - more tailwind never improves a runway's rank;
  - more runway never worsens it;
  - a hard-filtered candidate never outranks a passing one;
  - removing a rejected candidate never changes the choice.
- **Hysteresis**: noise within a bucket never flips the decision (the 27L/27R lesson); a sequence test over 1,000 jittered wind samples yields at most one change.
- **Explanations are facts**: every rendered sentence is produced from `Fact` values; a test renders all `Fact` variants and asserts that each number in the text equals a field of the fact (no free text with numbers).
- **Golden decisions on real data** (`--ignored real_install_*`, like `real_install_lowi_arrival`):
  - LOWI at 080/6, 260/12 and calm, with and without RNP capability;
  - EDDM 26 vs 08 (ILS 26R/L, parallels);
  - EGLL (27L/R alternation, ILS CAT);
  - LXGB (no procedures: the DME reconstruction or a refusal);
  - ETND (the grass 26R never chosen);
  - EDLP (the vacate/stand cases already pinned).
  - Snapshot the `Decision` JSON.
- **Reassessment sequences**: scripted input timelines (METAR shift, a tower runway change, a deviation) assert the exact announcements: count, timing, severity, and silence when busy.
- **Terrain**: a fake probe (a grid function) for profiles and corridors; MSA parsing against the LOWI rows; `procedure_summary` flags a synthetic approach with a leg under terrain; the cache returns the same numbers without a second probe.
- **Regression cases from today**, each a named test: ETND grass, LOWI circling-as-approach, LOWI 26R grass, tower/crew runway mismatch, RNP 0.3 selection, a resume ceiling, a transition flip at FL180.
- **Sortie tests for the leg-type follower** (stage 4): fly TF/CF/RF/HM through the real law with a lagged heading plant and transport delay, and assert cross-track bounds per RNP value.
