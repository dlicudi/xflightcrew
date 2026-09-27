# A profile per aircraft MODEL, not per ICAO type (2026-09-27)

Read `docs/tasks/COMMON.md` first; its rules apply.

## The problem

Detection keys on `sim/aircraft/view/acf_ICAO` alone: `detect::known_profile`
(src/detect.rs) is a hardcoded ICAO -> profile table, called by the startup
probe (src/main.rs ~394) and the swap poller (`spawn_aircraft_detector`,
src/detect.rs ~155). The stock Laminar 737 and the Zibo 737 both say `B738`,
so the Zibo would fly on `boeing_737_800`: stock datarefs, stock xlua
assumptions, the stock `scripted_auto_start`. Same for Laminar A330 vs ToLiss,
etc. That is the RV-10-on-the-172 bug class (CLAUDE.md "An unknown type is
never silently the 172") one level down: **a different model with the same
ICAO must never silently fly another model's profile.**

## The design

1. **The profile declares what it matches**, in its YAML, replacing the Rust
   table. New top-level block in every `profiles/*.yaml`:

   ```yaml
   match:
     icao: [B738, B737, B739]                   # acf_ICAO, case-insensitive
     acf: ["Laminar Research/Boeing 737-800/"]  # substrings of acf_relative_path
   ```

   Optional further keys `author:` / `description:` (substrings of
   `sim/aircraft/view/acf_author` / `acf_descrip`, case-insensitive) for the
   add-on whose folder name the user may have renamed. Every key given must
   hit; `icao` is required. Normalise the path (backslashes to `/`,
   case-insensitive) before matching.

2. **Resolution is one pure function** (e.g. `detect::resolve(&AcfIdentity,
   &[ProfileMatch]) -> Resolution`), `AcfIdentity { icao, relative_path,
   author, descrip }` read in one `tokio::join!`. Rules:
   - Candidates = profiles whose `icao` list holds the ICAO.
   - A candidate matches only if all its declared keys hit. Most specific wins
     (more keys matched); a tie between two profiles is a profile-authoring
     error: pick deterministically (by id), `tracing::warn!` naming both, and
     pin "no two shipped profiles tie on any installed .acf" in a test.
   - **ICAO hit, no candidate's `acf`/`author` matches -> NOT the ICAO's
     profile.** It is an unknown model: `generic_profile` from its own .acf,
     and the pilot hears a line naming it, e.g. "This is a B738 but not one I
     have a profile for. I'm flying it on the placards from its own aircraft
     file. Treat every automation as untested." Blackbox it.
   - A profile with only `icao` (no `acf`/`author`/`description`) is a
     type-wide catch-all. Allowed, but every shipped profile gets an `acf`
     matcher now (list below), so none is.
   - `XCOPILOT_AIRCRAFT=<id>` pinning is unchanged.

3. **The detector's change key is the model, not the ICAO.** Swapping between
   two .acf files with the same ICAO (stock 737 -> Zibo) must re-resolve and
   swap. Key the poller on (icao, relative_path); the generic id becomes
   `generic_<icao>_<acf file stem>` lowercased/sanitised so two unknown models
   of one type are distinct. Livery changes (`acf_livery_path`) are NOT a
   change and must not be read (slow dataref).

4. **Type-level callers stay type-level.** `src/tools/setup.rs` ~153 (jet or
   heli from a SimBrief plan's ICAO) and `plan_audit::gather_suitability`
   compare a PLAN's type code, which has no model. Give them an explicit
   `aircraft::profile_for_type(icao)` (any profile whose `match.icao` holds
   it) so the intent reads, and keep `plan_audit`'s "another type" refusal on
   ICAO. Do not make them model-aware.

5. **Loading**: the match blocks come from the YAMLs (they are embedded or read
   the way `aircraft::load` already reads profiles; follow that, do not add a
   second loader). Delete the hardcoded table in `known_profile`; keep the
   startup path and the poller on the same resolver (today they are two
   copies of the same decision; make it one).

6. **Probe it**: the profile_probe at swap should blackbox the identity that
   resolved it (icao, relative_path, author, which keys matched), so the next
   sortie shows why a profile was picked.

## Matchers for the shipped profiles

From the installed .acf files (`~/X-Plane 12/Aircraft`, `find -name '*.acf'`).
Verify each YAML's header comment for which airframe it was written against
before assigning; the ICAO lists are today's `known_profile` table.

| profile | acf substring |
|---|---|
| cessna_172sp | `Laminar Research/Cessna 172 SP/` |
| cirrus_sr22 | `Laminar Research/Cirrus SR22/` |
| md82 | `Laminar Research/McDonnell Douglas MD-82/` |
| citation_x | `Laminar Research/Cessna Citation X/` |
| cirrus_vision_sf50 | `Laminar Research/Cirrus Vision SF50/` |
| dc3 | `AWXDC3/` (check the YAML: if it was written for a Laminar DC-3, say so and ask) |
| king_air_c90b | `Laminar Research/Beechcraft King Air C90B/` |
| beechcraft_baron_58 | `Laminar Research/Beechcraft Baron 58/` |
| lancair_evolution | `Laminar Research/Lancair Evolution/` |
| dhc6_twin_otter | `DHC6-300 Twin Otter V2/` |
| vans_rv10 | `Laminar Research/Van's Aircraft RV-10/` |
| boeing_737_800 | `Laminar Research/Boeing 737-800/` |
| airbus_a330_300 | `Laminar Research/Airbus A330-300/` |
| boeing_747_400 | `Sparky/747-400/` (add an `author:` too if the .acf's `acf/_author` line gives one) |
| sikorsky_s76c | `Laminar Research/Sikorsky S-76/` |

A matcher on the folder is brittle to a user's renamed folder; where the .acf
has a distinctive `P acf/_author` or `P acf/_descrip` line, prefer adding
`author:` as a second route (an `any_of` is NOT wanted: keep "every key must
hit" and let the folder be the key; note in the YAML comment what to change if
the folder is renamed). Keep it simple.

## Tests

- `resolve` table test over every installed .acf path above: each lands on its
  profile; `ToLissA319_V1p12p1/a319.acf` (ICAO A319) is generic; a synthetic
  `Aircraft/Zibo/B737-800X/b738_4k.acf` with ICAO B738 is generic, NOT
  `boeing_737_800`; a Windows-style backslash path matches.
- Every shipped profile has a `match:` with `icao` and at least one of
  `acf`/`author`/`description` (pin it, like the other every-profile tests).
- No two shipped profiles tie on any row of the table.
- The detector swaps on a path change with the same ICAO (pure key test).
- Existing tests that call `known_profile` move to the resolver.

## Docs

One dev-history entry (why: stock vs Zibo share B738; the rule; blackbox
field) and rewrite the CLAUDE.md "Aircraft profiles drive the tool surface"
opening sentence: detected from the profile's `match:` (acf_ICAO + .acf path),
never ICAO alone; an unmatched model of a known type is generic, said aloud.
Also update the stale "detected from `acf_ICAO`" wording in main.rs comments.
