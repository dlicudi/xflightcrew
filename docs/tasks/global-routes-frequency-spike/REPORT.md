# Route frequency spike (2026-10-15)

Can open statistics give per-route weekly frequencies for the US and Europe? Follows `docs/tasks/global-routes-spike/REPORT.md` (network from Wikipedia plus Wikidata; OpenSky ruled out by its terms). Script: `spike.py` (Python 3.9, stdlib). Commands: `bts <cache> <y> <m>`, `btscov`, `euro <cache> <network.json> <cc> <field>`, `caa <network.json> <field> <NAME> <csv>...`. Calls were made 1 s apart with a descriptive UA, cached, no account.

## 1. BTS T-100 Segment (all carriers): usable

- **Download:** an anonymous POST of the TranStats DL_SelectFields form (`gnoyr_VQ=FMG`, viewstate plus field checkboxes) returns a zip of CSV. 49,604 rows for July 2025.
- **Licence:** US federal work, public domain. **Latency:** about 3 months; latest July 2026 on 15 Oct. Foreign carriers are partly late (no Emirates JFK in July 2026).
- **Fields:** DEPARTURES_PERFORMED per UNIQUE_CARRIER (the OPERATOR, IATA code), ORIGIN, DEST, AIRCRAFT_TYPE, month; CLASS F is scheduled passenger. Foreign carriers are included: 73 at JFK.
- **Coverage, carrier-pair flown in July 2025 or January 2026:**
  - KJFK 272/346 (79%): 30 Delta Connection rows miss.
  - KSEA 194/259 (75%): 31 Alaska rows miss (Horizon operates).
  - KASE 0/10: all regional brands (SkyWest operates them).
  - With a pair fallback for brands: 330/346, 250/259 and 10/10.
- **Examples:** JFK-LHR BA 55/wk (July), 46/wk (January); SEA-ANC Alaska 113/wk, 80/wk.

## 2. Eurostat avia_par_es / _de: partial

- **Download:** SDMX-CSV bulk, 26 MB gz for Spain. The units are FLIGHT (CAF_PAS, _ARR, _DEP), SEAT and PAS, per `airp_pr` (ES_LEMG_UK_EGKK) and month.
- **No carrier dimension.**
- **Licence:** reuse with attribution, commercial too (Decision 2011/833/EU). **Latency:** data ends 2024-12, last updated 19/11/25 (Spain) and 22/10/25 (Germany): 10 to 22 months stale.
- **Coverage:** LEMG 80 of 165 Wikipedia destinations; EDDF 182 of 299.
- **LEMG totals, weekly summer/winter (ours in brackets):** EGKK 82/40 (68/44); EIDW 57/26 (56/42); EGCC 41/22 (40/27); EGSS 29/18 (28/18); EGGD 28/13 (32/17).

## 3. UK CAA punctuality Full Analysis CSV: usable, licence to clear

- **Download:** monthly CSV, no account. 26 UK reporting airports.
- **Fields:** reporting_airport, origin_destination (a NAME, no codes), airline_name, scheduled_charter, number_flights_matched (movements, both ways; weekly rotations = movements / 2 x 7 / days in month).
- **Latency:** about 2 months (August 2026 published).
- **Licence:** not stated as OGL. "Referenced in any publication"; "No statistical data provided by CAA maybe sold on to a third party". Ask the CAA before shipping derived numbers.
- **EGKK July 2025:** 206 destinations, 348 airline-route rows. Wikipedia's 429 carrier-pairs: destination matched 306, carrier plus destination matched 214 (name matching, approximate). January 2026: 266 and 182.
- **EGKK-LEMG, July 2025 / January 2026 (ours in brackets):** easyJet 39/11 (28/21), Vueling 21/12 (7/7), BA Euroflyer 19/11 (14/7), Wizz 10/4 (7/4), TUI 3/0 (5/2).
- **Long-haul:** Emirates 21/21 (ours 28 since Feb 2026), Turkish 28/20 (ours 28/28).

## 4. Verdict

1. **Network:** Wikipedia plus Wikidata (carrier, pair, seasonal flag).
2. **US airports and every route touching the US:** BTS per carrier, summer (July) and winter (January), with aircraft types; brands mapped to operators (Delta Connection to 9E/OO, Alaska to QX/OO).
3. **UK airports:** CAA per carrier, once the licence is cleared.
4. **EU airports:** Eurostat pair totals, split across the pair's Wikipedia carriers by a heuristic (base carrier, LCC versus legacy); data 1 to 2 years old.
5. **Rest of world and the uncovered long tail** (about 20 to 50% of EU pairs): the heuristic over the Wikipedia network (carrier class, distance, seasonal flag summer-only), calibrated on the three sources above.

**Our own tables:** the route totals hold, but the per-carrier splits are wrong in places (Vueling LEMG-EGKK about 3x too low, easyJet's seasonal swing too small).

Sources: [Eurostat copyright notice](https://ec.europa.eu/eurostat/help/copyright-notice), [Eurostat licence policy](https://ec.europa.eu/eurostat/statistics-explained/index.php/Copyright/licence_policy), [CAA statistics guide](https://caa.co.uk/data-and-analysis/uk-aviation-market/flight-punctuality/statistics-guide), [CAA punctuality 2025](https://www.caa.co.uk/data-and-analysis/uk-aviation-market/flight-punctuality/uk-flight-punctuality-statistics/2025/).
