# LEMG 737, 2026-10-05: core fixes (first principles, no patches)

Flight: stock Laminar 737-800, LFRG to LEMG (SimBrief plan), full auto, sortie
`1791187965-full_auto`. Evidence: `logs/blackbox.jsonl` (kinds `state`, `autoland`,
`full_auto`, `said`, `typed`, `ap_pitch_authority`, `flight`). Times local (CEST,
epoch = 2026-10-05 HH:MM:SS +02:00).

## What happened

| Time | Event |
|---|---|
| 12:20:18 | Autoland armed for LEMG 13 at 4,579 ft; `alt_dial_parked` was 4,200, now 5,600 (park ABOVE rule). |
| 12:21:18 | `reversal`: left downwind 3,400 ft, raised for high ground. |
| 12:25:33 | "Autopilot is not holding the descent": VS dial -570, aircraft level (window 5,600 above). Stock 737 MCP will not fly a V/S descent away from a window above it. |
| 12:28:25 | Go-around 1: too high (550 ft over the path at 3.7 nm). |
| 12:30 | Pilot set the window to 0 by hand: VS descents and the go-around climb then followed the dial. |
| 12:37:47 | Full flap at 3.5 nm: VS gave -100 to -400 against -1,200 for ~12 s (flap pitch-up). |
| 12:38:18 | Go-around 2: `unstabilised_high` at the 1,000 ft gate. |
| 12:42:27 | xvl restarted mid-flight (to load 89fe7a6). The AP was left holding autoland's last VS (-330 fpm), window 0, 2,900 ft, heading 306 toward high ground. Nothing managed it. |
| 12:42:59, 12:43:09, 12:43:54 | "fly the flight plan": refused "No plan is loaded", then "Climb to 9,000 feet first, the sector altitude ... I haven't taken control", then "9 miles right of course ... outside the 3 mile rejoin corridor". The aircraft kept descending toward terrain. |
| ~12:44:50 | Lowest point ~270 ft AGL (y_agl 82 m); pilot pulled up by hand, AP servos out; recovered with CMD A, LVL CHG, A/T SPEED (manually, outside xvl). |
| 12:52:22 | "land the aircraft" self-picked **LXGB 09** (26 nm) over LEMG (plan not loaded after restart). |
| 12:53:07 | Cancelled; plan reloaded; aircraft left descending 1,300 fpm on autoland's last VS with window 0. |
| 12:55:56 | "fly the flight plan" refused: "not abeam any leg of the route". |
| 13:02:38 | "land the aircraft" (now nearer LEMG) picked LEMG 13; window parked 0 (fix works). |
| 13:08 | Descent not held again: the A/T was in SPEED 230 (set during the manual recovery) and overrode autoland's throttle writes; aircraft could not slow and descend. |
| 13:16:22 | Go-around 3: joined 150 ft high at 9 nm, VS -900 achieved vs -1,300 asked while also decelerating 198 to 178 kt at idle: a 737 cannot slow and descend steeply at once. |
| 13:16:53 | Autoland re-engaged by the pilot at 2 nm without landing flap: immediate go-around ("no landing flap"). |
| 13:24 to 13:29:50 | Landed by the **737's own APP mode** (LOC/GS captured, flaps 30, A/T SPEED 148), driven by hand through MCP datarefs: clean glideslope tracking. Pressing CMD B toggled CMD A OFF (servos 0) at 4.2 nm, re-engaged. A/T SPEED stayed engaged through touchdown. Touchdown 319 fpm. |

## Root causes (first principles)

1. **The crew can leave an airborne aircraft in a state that flies into the ground.** A refusal, a cancel, a give-up or a process start all leave whatever the AP was last told. There is no invariant "airborne and nobody flying => the aircraft is held safe (level at or above the safe altitude, heading clear of terrain)".
2. **Getting to the destination is a set of narrow resume gates, not a planner.** "Fly the flight plan" refuses off-route, below MSA, not abeam; "land the aircraft" self-picks by distance and ignores the destination when no plan is loaded. From ANY state the crew should compute a safe path to the destination's arrival or approach (climb to the safe altitude if needed, direct to an entry, then the approach), refusing only what is physically impossible (fuel, runway).
3. **Autoflight state has many writers and no ownership.** Autoland writes VS and window but assumes the autothrottle, CMD channels and MCP modes are as it expects; toggle commands (CMD A/B, HDG SEL, LVL CHG, VS) are pressed blind; a pilot's or a recovery's leftover modes silently fight the loop. One autoflight module should own the MCP: read, decide, press only on a difference, read back, and take over (or disengage) the autothrottle when a loop flies the throttles.
4. **Approach energy is not planned.** A jet must be slowed and configured before it is asked to hold a 3 degree path; the VS law asks for descent and deceleration at the same time on final. Airframes with a working coupled approach (the 737's APP) should fly it (`ils_coupled`, #81) with the loop managing speed, configuration and the landing.

## Already shipped in the air (89fe7a6)
`alt_dial_park_field`, `landing_flap_lead_nm` on the 737 profile. Keep, or subsume into the core design.
