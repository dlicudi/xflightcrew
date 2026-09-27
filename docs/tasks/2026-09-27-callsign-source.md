# Callsign: one resolver, override > SimBrief > tail number

Read `docs/tasks/COMMON.md` and `CLAUDE.md` first.

## Today

The callsign is ONLY the tail number (`sim/aircraft/view/acf_tailnum`, `datarefs::TAILNUM`),
spoken by `tower::spoken_callsign` / `spoken_callsign_wake` (heavy suffix from MTOW). It is
resolved in three separate places:

- `crates/atc/src/station.rs` `build_ctx` (~line 1546): the tower's `ctx.callsign`
- `src/circuit.rs` `crew_callsign` (~line 764): the crew's own radio calls (full auto etc.)
- `src/bin/atc.rs` (~line 289): the standalone ATC binary

An airline flight (a 737 on a SimBrief OFP as BAW306) is called by its registration; there is no
way to set a callsign manually.

## Want

ONE resolver used by all three sites, in this order:

1. **Manual override**: new setting `XCOPILOT_CALLSIGN` through the settings registry
   (`src/settings.rs`: env > pref > default, env-pinned refused on screen; add a panel field if the
   Settings tab has text fields, else env/pref only and say so). Accept either a raw id
   ("BAW306", "N172SP", "G-ABCD") or already-spoken words; raw ids get spoken.
2. **SimBrief**: when a SimBrief plan is loaded (`PlanSource` SimBrief, not Local), take the OFP's
   ATC callsign. Verify the real field name against the OFP JSON (believed `atc.callsign`; also
   `general.icao_airline` + `general.flight_number`); if you cannot verify from a fixture or the
   SimBrief API docs, parse defensively (atc.callsign first, else airline+flight number) and say
   so in the report. Add `callsign: Option<String>` to `FlightPlan`. A local plan has none.
3. **Tail number**, as now.
4. Heavy suffix applied last from MTOW, whatever the source.

**Speaking an airline id**: reuse the existing telephony tables in `crates/atc/src/station.rs`
(`TELEPHONY`, `TELEPHONY_IATA`, and the ghost-speaking function near line 3811) so "BAW306"
becomes "Speedbird three zero six". Move/expose them from one place in the atc crate; do not copy
them. Unknown airline prefix: spell the letters NATO, digits spoken. Decide airline-vs-registration
shape by pattern (3 letters + digits, optional trailing letter = airline; else registration).

**Facts must not outlive their context**: the SimBrief callsign lives with the plan (replaced plan =
new callsign, cleared plan = none); a swap clears it via the existing plan/swap paths
(`flight_state::publish_airframe` / `stand_down` if a static is needed; prefer reading it off the
current plan so no new static exists at all). The atc crate must NOT reach into the app; if the
station needs the value, pass it in or add an accessor on the party line (`party_line.rs`), never a
`pub static`.

**The tower must recognise the pilot's spoken callsign in the new form.** Find how the tower/station
checks an utterance is addressed to us (search tower.rs/station.rs for where `ctx.callsign` or the
tail is matched against transcripts, "say callsign" paths). A pilot saying "Speedbird three zero
six, request taxi" must parse as ours; add the whisper-ish variants to
`crates/atc/bench/atc_corpus.yaml` if that corpus covers addressing.

Blackbox: one `atc` (or suitable kind) record `callsign` with `{spoken, source: override|simbrief|tail}`
when the resolved callsign changes. The Health/terminal may show it if trivial.

## Tests (plain cargo test, `XCOPILOT_BLACKBOX=off`)

- resolver order: override beats plan beats tail; empty override ignored
- BAW306 -> "Speedbird three zero six"; unknown "XYZ123" spelled; "N172SP", "G-ABCD" unchanged
- heavy suffix on each source
- SimBrief JSON parse with and without the atc callsign field
- local plan has no callsign; replacing a plan changes it
- tower addressing with an airline callsign (station route test)

## Done

`cargo test --workspace` green, `cargo build`, format only the files you edited (never bare `cargo
fmt`), commit on your branch with a message in the repo's style (what flew/why, "offline only").
Add a short `docs/dev-history.md` entry and, if it earns one, one CLAUDE.md phrase (the ATC section).
No em dashes anywhere. Report: files changed, the OFP field you used and how verified, anything
left undone.
