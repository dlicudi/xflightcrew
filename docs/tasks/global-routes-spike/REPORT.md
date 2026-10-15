# Global route network spike (2026-10-15)

Can real data give our AI traffic a global airline network? Measured on 20 sample airports. Script: `spike.py` (Python 3.9, stdlib only). Run: `python3 -I spike.py <cache_dir> <airlines.dat>`. It made 79 API calls in all (Wikipedia parse and pageprops, Wikidata wbgetentities), 1 s apart, with a descriptive User-Agent, cached. No personal data was sent.

## 1. Wikipedia network: parse table

All 20 articles use the same template, `{{Airport destination list}}`, inside the article itself. The script's fallbacks (a plain wikitable, a `{{Main}}` list article) were never needed. Each destination link goes through Wikipedia pageprops to Wikidata and its ICAO code (P239). Flags come from the cell's bold headers (Seasonal:, Charter:) and the text after a link (begins/resumes, ends, suspended).

| Airport | Rows | Dest to ICAO | Airlines | Seasonal | Charter | Begins/ends/susp. |
|---|---:|---:|---:|---:|---:|---:|
| LXGB | 5 | 5 | 2 | 0 | 0 | 0 |
| LEMG | 278 | 278 | 48 | 58 | 3 | 11 |
| EGKK | 429 | 429 | 60 | 195 | 0 | 30 |
| EGLL | 340 | 340 | 85 | 45 | 0 | 12 |
| EDDF | 496 | 496 | 84 | 144 | 2 | 11 |
| LFPG | 436 | 436 | 113 | 85 | 2 | 10 |
| EIDW | 306 | 306 | 45 | 107 | 16 | 4 |
| LOWI | 57 | 57 | 17 | 49 | 9 | 0 |
| EGPE | 16 | 16 | 8 | 4 | 2 | 2 |
| KJFK | 357 | 357 | 80 | 79 | 0 | 15 |
| KSEA | 259 | 259 | 39 | 33 | 0 | 6 |
| KASE | 15 | 15 | 4 | 8 | 2 | 0 |
| CYYZ | 407 | 407 | 60 | 139 | 0 | 19 |
| SBGR | 207 | 207 | 37 | 16 | 0 | 9 |
| FAOR | 167 | 166 | 48 | 0 | 0 | 1 |
| OMDB | 484 | 484 | 99 | 27 | 1 | 32 |
| VHHH | 307 | 307 | 78 | 17 | 5 | 7 |
| RJTT | 198 | 198 | 44 | 0 | 0 | 0 |
| YSSY | 188 | 188 | 54 | 6 | 0 | 8 |
| NZQN | 26 | 26 | 11 | 2 | 0 | 0 |
| Total | 4,978 | 4,977 (99.98%) | 313 distinct | | | |

The one miss is FAOR's Sishen Airport, whose Wikidata item has no P239. Not measured: a destination written as plain text without a link would be dropped silently. The template's style requires links, and spot checks found none. RJTT and FAOR mark nothing seasonal.

## 2. Airline resolution (313 distinct airlines)

| Source | ICAO | Telephony |
|---|---:|---:|
| OpenFlights airlines.dat, name/alias match | 173 (55%), 160 active | 165 |
| Wikidata via article link (P230, P432) | 282 (90%) | 261 (83%) |
| Either | 283 (90.4%) | |

- **OpenFlights misses 140:** KLM, Vueling, Virgin Atlantic, TUI Airways, JetBlue, SAS, Saudia, ITA, Riyadh Air, Norse, Transavia and every LATAM unit.
- **13 disagreements where both answer, with errors on both sides:**
  - OpenFlights gives defunct codes: Brussels DAT, Condor CIB, IndiGo IBU.
  - Wikidata's first value can be a sister operator: Norwegian Air Shuttle NOZ instead of NAX, AJet THY.
- **Both miss 30:** brands that are not operators (United Express, Delta Connection, American Eagle, QantasLink, LATAM Chile), new start-ups, and NZQN's scenic and helicopter operators.

## 3. Ground truth (carrier ICAO, other-end ICAO; charters excluded)

| Field | Ours | Wikipedia | Match | Ours only | Match if NOZ read as NAX |
|---|---:|---:|---:|---:|---:|
| LXGB | 5 | 5 | 5 | 0 | 5/5 |
| LEMG | 212 | 275 | 193 (91%) | 19 | 208/212 (98%) |
| EGKK | 120 | 426 | 115 (96%) | 5 | 117/120 (98%) |

- **Ours only, after the NOZ fix:**
  - LEMG Air Nostrum to GEML, LEMD and LEVC: Wikipedia lists these under the Iberia brand, not the operator.
  - LEMG TUI to EGGD: not listed.
  - EGKK TUI to KMCO: Wikipedia has KMLB.
  - EGKK UBT to KJFK and KMCO: not listed.
- **Wikipedia only:**
  - LEMG: 82 pairs, including Gulf Air, Kuwait, Royal Air Maroc, airBaltic, Luxair, KM Malta and more Iberia.
  - EGKK: 311 pairs. By design our table models 120 of 426 pairs; the rest is the weekly and seasonal long tail.

## 4. OpenSky

- **Anonymous access is refused.** Arrival and departure queries for EGKK and LXGB, 1 to 10 days back, all return `403 You cannot access historical flights`. The spike stopped there and made no account.
- **What access needs:**
  - An account and an API client: OAuth2 client credentials only, basic auth is no longer accepted, tokens last 30 min.
  - Flights cost credits: 4,000 a day standard, 8,000 for feeders; 30 credits for 1 to 2 day partitions, 60 per day for 3 or more.
  - At most 2 days per call (the docs contradict themselves on /departure). Data is available from the previous day back.
- **The terms rule it out as a shipped source.** Quotes:
  - "solely for the purpose of non-profit research and non-profit education"
  - "Operational use of the REST API in any live product, service, or automated system also requires a written license"
  - "You will not distribute, disclose, transfer or otherwise make available the data set(s)"
  - Aircraft identifiers must be anonymised in any publication.
- Derived per-route frequencies shipped in a GPL repository would be a redistribution and need written permission from contact[at]opensky-network.org.
- No frequency comparison was possible.

## Licensing

- **Wikipedia:** CC BY-SA 4.0. Treat a generated table as a derivative: attribute the article and revision id, and share alike. One-way compatible with GPLv3, so a labelled BY-SA YAML data file can ship beside GPL code.
- **Wikidata:** CC0 (airport and airline ICAO codes, callsigns).
- **OpenFlights:** ODbL 1.0 (attribution, share-alike on a derived database). Not needed, since Wikidata does better.
- **OpenSky:** a non-profit research licence with no redistribution and a written licence for operational use.

## Verdict

**Wikipedia plus Wikidata for the NETWORK: viable globally.** One template everywhere, 99.98% of destinations resolved, 90% of airlines resolved, and 98% recall of our hand-checked pairs once one code alias is fixed. A build-time generator (never at runtime) could emit route rows for any field.

**OpenSky for FREQUENCY: not viable** without a written licence, and not reachable anonymously. Frequencies, windows and types stay heuristic (carrier class, seasonal flag, distance, field size), or come from per-field passenger statistics as LXGB's did.

## Risks

1. **No frequency at all:** a weekly seasonal route and a 12-a-day shuttle look identical (EGKK: 426 pairs, 195 seasonal).
2. **Brand vs operator:** Iberia rows flown by Air Nostrum, United Express, Delta Connection, QantasLink. Callsigns and liveries need the operator, so an override table is required.
3. **Wikidata code ambiguity** (NOZ/NAX, AJet/THY): needs a short override list.
4. **Currency:** the tables are edited constantly. Pin a revision id per generated file and regenerate deliberately.
5. **Silent losses:** scenic and helicopter fields (NZQN) and plain-text cells degrade quietly, so the generator should log every dropped row.
