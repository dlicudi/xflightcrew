# Global route tables (2026-10-15)

Goal: every airport with scheduled service flies its real airlines to their real destinations, at believable frequencies. Evidence: `docs/tasks/global-routes-spike/REPORT.md` (network) and `docs/tasks/global-routes-frequency-spike/REPORT.md` (frequencies).

Decided:
- **Network:** Wikipedia `{{Airport destination list}}` plus Wikidata (CC BY-SA 4.0 and CC0).
- **Frequency:** BTS T-100 for every route touching the US (public domain, per operator). Eurostat `avia_par_*` pair totals for every route with an EU end, the UK's included (attribution, no carrier: split by the heuristic). A fitted heuristic for the rest.
- **UK CAA punctuality data is used ONLY to fit the heuristic's parameters, never written into a shipped file**, until the CAA says otherwise.
- **Generated offline, reviewed, committed.** Never fetched at runtime (the `schedule.rs` rule).
- Fields with no scheduled service keep the hour generator (`schedule::generate`), unchanged.

## Stage 1: the data format (Rust, runtime)

Today's timetables are three per-field files, with carriers, places and clocks as Rust tables (`airlines.rs` constants via `carrier_by_name`, `places::PLACES`, `places::clock`). None of these scale, so all four become data:

```
routes/fields.yaml            every field a route names
  - {icao: LEMG, city: "Malaga", lat: 36.67, lon: -4.50, utc_offset_h: 1.0, dst: true}
  - {icao: EGKK, city: "London Gatwick", ..., top_up: true}
routes/carriers/EZY.yaml      one file per operator
  code: EZY
  name: "easyJet"
  iata: U2
  telephony: "Easy"
  types: [[A320, 40], [A20N, 35], [A319, 25]]
  numbers: [11, 8999]
  routes:
    - {at: LXGB, other: EGKK, number: 8901, summer: 9, winter: 6, windows: [[11.5, 12.5], [17.5, 18.5]]}
```

- **A route row keeps today's meaning exactly.** `at` is the field whose file it was written in, so `number` (inbound at `at`), `windows`, `based` and `types` mean what they mean now. `derive` still builds the other end's view, which keeps `rotation.rs` untouched.
- **Each pair is written once per carrier**: a load error names the duplicate.
- **The loader builds the same `FieldRoutes`.** `places::position` / `clock` read `fields.yaml`. `airline_name` reads the carrier's `name`/`iata`.
- **Telephony from data.** The station learns the routes' carrier telephony through the existing crew-to-station provider seam (as `callsign::user_telephony` does), never a second table. The bin's telephony test then checks the data.
- **Convert the three existing tables, changing nothing.** A golden test pins the LXGB, LEMG and EGKK days (every movement's callsign, type, time) before the change, and that test must still pass after it. The Rust tables `carrier_by_name`, `PLACES` and `clock` are deleted. The hour generator's carrier mixes (`field_mix`, `country_mix`, `FALLBACK`) stay Rust for now.
- **Load cost:** parse once, as today. Measure the parse time of a 50k-row synthetic set in a test and report it.

## Stage 2: the generator (offline, Python stdlib, `tools/routes/`)

`python3 -I tools/routes/build.py <cache_dir>` writes `routes/fields.yaml` and `routes/carriers/*.yaml`. Steps:

1. **Fields:** every article transcluding `{{Airport destination list}}` (MediaWiki `embeddedin`).
2. **Network:** parse each table (the first spike's parser). Resolve destinations and airlines through Wikidata:
   - ICAO from P239 (airports) and P230 (airlines); telephony from P432; coordinates from P625; time zone from P421, turned into `utc_offset_h` and `dst` with `zoneinfo`.
3. **Overrides** (`tools/routes/overrides.yaml`, hand-kept, small):
   - brand to operator (Delta Connection, United Express, Air Nostrum for Iberia rows)
   - airline code fixes (NOZ to NAX)
   - hand rows that win over generated ones (Gibraltar's published slots, BA's real numbers)
4. **Frequency, first source that has it:**
   - BTS per operator (July = summer, January = winter)
   - Eurostat pair total, split across the pair's carriers by the heuristic's weights
   - the heuristic
5. **Heuristic:** a few parameters (pair gravity from each end's destination count and the distance, a carrier-class weight, a seasonal-flag winter factor). They are fitted on BTS, Eurostat and CAA and stored in `tools/routes/params.json`, with the fit's error reported.
6. **Types:** from BTS where present, else the carrier's fleet. Every type must be one the performance tables know; map it or drop the row, logged, never a silent 172.
7. **Numbers:** assigned deterministically inside the carrier's range.
8. **Output:**
   - Every file is headed with its sources, the Wikipedia revision ids and the CC BY-SA attribution.
   - The run is deterministic: the same cache gives the same bytes.
   - Every dropped row and unresolved airline is listed in one log.

## Stage 3: check

- Compare the generated LEMG, EGKK and LXGB with the hand tables and the spike numbers.
- Fly one sortie at LEMG and one at a US field. Add the KJFK-class load to the `api_load` and frame-time watch.
