    use super::*;

    /// Every test that publishes the fuel unit holds this. The fuel unit is
    /// process-wide, cargo runs these threads in parallel, and without it
    /// they overwrite each other's airframe mid-assertion — the same trap
    /// the station's `SIDE_STATE_LOCK` exists for.
    static FUEL_TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    /// An array element is never written by name-with-a-subscript.
    ///
    /// `set_float("sim/.../eng_master[0]", 1.0)` names a dataref that does not
    /// exist — the API answers "not found" and the caller, using `let _ =`,
    /// hears nothing. On the A330 the APU came up, the bleed came on, all
    /// eight fuel pumps came on, and both master switches sat at zero while
    /// the start waited two minutes for an idle that was never coming
    /// (2026-09-20). `set_float_array_elem` reads, patches the index and
    /// writes the whole array back.
    ///
    /// Cheap to write by accident and invisible when you do, which is the
    /// definition of a thing that belongs in a test rather than a comment.
    #[test]
    fn no_array_element_is_written_by_subscripted_name() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
        let mut files = Vec::new();
        fn walk(dir: &std::path::Path, out: &mut Vec<std::path::PathBuf>) {
            for e in std::fs::read_dir(dir).unwrap().flatten() {
                let p = e.path();
                if p.is_dir() {
                    walk(&p, out);
                } else if p.extension().is_some_and(|x| x == "rs") {
                    out.push(p);
                }
            }
        }
        walk(&root, &mut files);
        let mut offenders = Vec::new();
        for f in &files {
            let text = std::fs::read_to_string(f).unwrap();
            for (i, line) in text.lines().enumerate() {
                let t = line.trim_start();
                // A set_* whose name argument is a format! building a
                // subscript. The array writers take an index instead.
                let writes_scalar = t.contains("set_float(") || t.contains("set_int(");
                if writes_scalar && t.contains("format!(") && t.contains('[') && t.contains("{idx")
                {
                    offenders.push(format!("{}:{}", f.display(), i + 1));
                }
            }
        }
        assert!(
            offenders.is_empty(),
            "use set_float_array_elem / set_int_array_elem for an element, not \
             a subscripted name — the API does not know that dataref and the \
             write is lost silently: {offenders:?}"
        );
        assert!(files.len() > 10, "the walk found the tree: {}", files.len());
    }

    /// EVERY speed and EVERY rate in the flight laws, classified once.
    ///
    /// This is the rule CLAUDE.md already states — "all automation numbers
    /// are per-airframe, defaulting to the old GA figures when a profile is
    /// silent" — with something behind it at last. The rule was written
    /// before 2026-09-20 and on that date a 737 still flew a 0.8 nm capture
    /// radius, a 700 fpm descent, a 0.10 nose-down limit, avgas density and
    /// an 800 fpm go-around, because prose does not fail a build.
    ///
    /// A constant named in knots, feet per minute or degrees per second is
    /// a number the AIRCRAFT achieves, and those are the ones that were
    /// wrong. Each is listed here with what it is, and anything new or
    /// renamed fails until someone decides which it is. That decision is
    /// cheap at the moment of writing and expensive at 34,000 feet.
    #[test]
    fn every_speed_and_rate_in_the_laws_is_classified() {
        // (module, NAME) -> why it may be a shared constant.
        const REVIEWED: &[(&str, &str, &str)] = &[
            ("procedure", "NOT_CLIMBING_FPM", "a rate that is not a climb, any airframe"),
            ("autoland", "DEADBAND_FPM", "the energy throttle's hand: an energy-rate error it leaves alone (about 1.5 kt at a 172's approach speed), the learnt power still trims it; any airframe (2026-10-11)"),
            ("autoland", "LEVEL_RETURN_MAX_FPM", "the rate an engaged altitude hold may return to its altitude at and still be holding it, a watchdog band, not a commanded rate; any airframe (2026-10-11)"),
            ("autoland", "SLOW_KTS", "a band under the airframe's own approach target where the energy throttle closes the deficit faster, not a commanded speed; any airframe (2026-10-12)"),
            ("autoland", "URGENT_SLOW_KTS", "a band under the airframe's own approach target where the energy throttle's hand moves faster, not a commanded speed; any airframe (2026-10-12)"),
            ("autoland", "MAX_OFF_PATH_FPM", "how much steeper than the path's own rate a visual final's dial may go to rejoin it from above, a bound, not a commanded rate; any airframe (2026-10-12)"),
            ("autoland", "SPEED_TERM_MAX_FPM", "the most energy rate a speed error may ask of the energy throttle, a bound on the request, not a commanded rate; any airframe (2026-10-11)"),
            ("autoland", "FLARE_BLEED_KTS_S", "the landing call's float for speed over the airframe's own approach speed, a prediction margin, not a commanded rate; only airframes declaring landing_roll_ft (LXGB 2026-10-06)"),
            ("autoland", "GA_CLIMB_ESTABLISHED_FPM", "a go-around's positive climb, the test for reducing thrust, not a commanded rate; any airframe (#199)"),
            ("autoland", "GA_FAST_KTS", "a band about the airframe's own go-around speed, where the thrust comes back at once; any airframe (#199)"),
            ("autoland", "GA_SLOW_KTS", "a band under the airframe's own go-around speed, where go-around thrust returns; any airframe (#199)"),
            ("circuit", "TERRAIN_HOLD_CREDIT_FPM", "a give-up clock's credit, not a commanded rate: the slowest climb any airframe is allowed before the guard hands back"),
            ("procedure", "ARRIVAL_DECEL_KTS_S", "planning slow-down for a STAR speed limit: a jet's descent rate, so a slower airframe only starts early"),
            ("autotakeoff", "ROLL_BRAKE_FULL_KTS", "brake-steer fade band, groundspeed; only brake_steer airframes"),
            ("autotakeoff", "ROLL_BRAKE_OFF_KTS", "brake-steer fade band, groundspeed; only brake_steer airframes"),
            ("autotakeoff", "ROLL_ALIGN_KTS", "walking-pace pivot at a crooked roll start; only brake_steer airframes"),
            ("autotakeoff", "ROLL_ALIGN_HOLD_KTS", "walking pace under which a crooked roll start waits for the nose; only brake_steer airframes"),
            ("autotakeoff", "ROLL_ALIGN_CREEP_KTS", "the line-up's own creep, while a crooked roll start is held; only brake_steer airframes"),
            ("autotakeoff", "ROLL_ALIGN_OVER_KTS", "the line-up's own creep margin, while a crooked roll start is held; only brake_steer airframes"),
            ("autotakeoff", "ROLL_ALIGN_RUNAWAY_KTS", "a held roll start past a walking pace is a reject, never a roll; only brake_steer airframes"),
            ("autotaxi", "ALIGN_SETTLED_KTS", "a stopped aircraft, for the line-up's aligned verdict; any airframe"),
            ("autotaxi", "ALIGN_TRIM_MIN_KTS", "rolling at all, for the line-up alignment's standing-yaw trim; any airframe"),
            ("autotaxi", "ALIGN_TRIM_PER_DEG_S", "the alignment trim's integration rate, brake per degree second; tuned on the yaw plant, any castoring nose"),
            ("autotakeoff", "CLIMB_ACCEL_MAX_KTS_S", "the climb hold's asked-for trend: a speed ERROR closed, and only on airframes declaring climb_pitch_max_deg"),
            ("autotakeoff", "CLIMB_PITCH_DOWN_DEG_S", "the climb hold's target walk down, a law rate below the rotation's own 2.5 deg/s; declared airframes only"),
            ("autotakeoff", "CLIMB_HOLD_FLOOR_FPM", "the positive floor climb_vs_fpm already keeps (airborne, it climbs); only airframes declaring a climb-out"),
            ("speedhold", "IDLE_FAST_KTS", "margin over the pilot's own target, not an airframe speed"),
            ("speedhold", "BETA_SPENT_OVER_KTS", "margin over the taxi pace before the spent-beta brake; only airframes declaring beta_max"),
            ("speedhold", "UNKNOWN_CEILING_KTS", "FALLBACK only for an airframe declaring no Vne; with one the ceiling is speed_ceiling_kts"),
            // --- Properties of the SIM or the SENSORS, not the airframe ---
            (
                "autotakeoff",
                "STAGNANT_GS_KTS",
                "the ASI is dead below ~20 kt on any airframe",
            ),
            (
                "autotakeoff",
                "TRIM_MIN_GS_KTS",
                "integrator arming: below this nothing steers yet",
            ),
            (
                "autotaxi",
                "STRAIGHTEN_HOLD_KTS",
                "walking pace; a stop is a stop",
            ),
            (
                "autotaxi",
                "CREEP_KTS",
                "the speed below which the sim reads as stopped",
            ),
            (
                "autotaxi",
                "CORNER_MIN_KTS",
                "floor under a declared max_yaw_dps corner cap: walking pace",
            ),
            (
                "speedhold",
                "SURGE_KTS_S",
                "acceleration that means a surge, not a schedule",
            ),
            (
                "speedhold",
                "SETTLING_KTS_S",
                "deceleration that means settling, likewise",
            ),
            // --- Rules and ceilings that bind every aircraft ---
            (
                "speedhold",
                "TAXI_MAX_KTS",
                "a safety ceiling on taxi, never a target",
            ),
            (
                "autoland",
                "VACATE_HANDOFF_KTS",
                "taxi speed off the runway, not a flight speed",
            ),
            (
                "autoland",
                "ROLLOUT_BRAKE_STEER_MIN_KTS",
                "the rollout's brake split stops at walking pace, the taxi's own floor; only brake_steer airframes",
            ),
            // --- Aerodynamic ramps: airflow over a surface, not performance ---
            (
                "autotakeoff",
                "LOW_SPEED_FULL_KTS",
                "rudder authority builds with airflow",
            ),
            (
                "autotakeoff",
                "XTK_GAIN_REF_KTS",
                "not a speed the aircraft flies: the pace the centreline gain \
                 is QUOTED at, above which it scales by ref/gs to hold the \
                 closure time - the same geometry on every airframe",
            ),
            (
                "autotakeoff",
                "POWER_LEAD_FADE_KTS",
                "throttle lead fades as the surface bites",
            ),
            (
                "autotakeoff",
                "THROTTLE_HOLD_MAX_GS_KTS",
                "holds power while the nose swings",
            ),
            (
                "autotaxi",
                "BRAKE_STEER_MIN_KTS",
                "differential brake bites above walking pace",
            ),
            (
                "autotaxi",
                "BRAKE_STEER_FULL_KTS",
                "...and is fully effective above this",
            ),
            (
                "speedhold",
                "RUNAWAY_OVER_KTS",
                "a taxi this far over its target is a runaway: the brake gets a stop's authority",
            ),
            (
                "speedhold",
                "BETA_BITING_KTS_S",
                "slowing this fast, a beta depth has bitten: hold it (any turboprop taxi)",
            ),
            // --- Handling conventions, flown the same in anything ---
            (
                "autotakeoff",
                "ROTATE_RATE_DEG_S",
                "2-3 deg/s is how a rotation is flown",
            ),
            // --- GA DEFAULTS with a per-airframe override already in place ---
            (
                "autotaxi",
                "STRAIGHT_KTS",
                "default: Pace::from_profile overrides",
            ),
            (
                "autotaxi",
                "TURN_KTS",
                "default: Pace::from_profile overrides",
            ),
            (
                "autotaxi",
                "SHARP_KTS",
                "default: Pace::from_profile overrides",
            ),
            (
                "procedure",
                "GA_VS_FPM",
                "default: publish_vs_limits overrides",
            ),
            (
                "autoland",
                "ROUNDOUT_VS_FPM",
                "default: arrest_targets_fpm overrides",
            ),
            (
                "autoland",
                "FLARE_VS_FPM",
                "default: arrest_targets_fpm overrides",
            ),
            (
                "autoland",
                "VS_LEAD_MAX_CLIMB_FPM",
                "default: ArrestLeadProfile overrides",
            ),
            (
                "autoland",
                "MAX_CLIMB_FPM",
                "floor only: max_climb_fpm() takes the airframe's",
            ),
            (
                "circuit",
                "AUTHORED_CLIMB_FPM",
                "floor only: authored_climb_fpm() takes the airframe's",
            ),
            // --- Scaling REFERENCES, not targets: the law divides by them ---
            (
                "autoland",
                "GLIDEPATH_GAIN_REF_KTS",
                "gain scales by actual/this, so it self-corrects",
            ),
            (
                "autotakeoff",
                "PITCH_TRIM_PER_DEG_S",
                "integrator rate; authority is per-airframe",
            ),
            // --- MARGINS in knots. Honest debt: a 10 kt margin is a sixth of
            // a 172's approach speed and a fourteenth of a 737's. None has
            // been caught misbehaving, and each is reviewed here rather than
            // silently shared.
            (
                "autoland",
                "SPEED_FLOOR_MARGIN_KTS",
                "MARGIN: above the airframe's own floor",
            ),
            (
                "autoland",
                "APPROACH_FAST_KTS",
                "MARGIN: over the approach speed",
            ),
            (
                "autoland",
                "RETARD_SLOW_KTS",
                "MARGIN: default only; the profile's retard_slow_kts",
            ),
            (
                "autoland",
                "RETARD_SINK_FLOOR_FPM",
                "floor only: time to the wheels is never infinite",
            ),
            (
                "autoland",
                "CLIMB_SPEED_MARGIN_KTS",
                "MARGIN: surplus before a climb engages",
            ),
            (
                "autoland",
                "CLIMB_ENERGY_BONUS_KTS",
                "MARGIN: speed asked for under the floor",
            ),
            (
                "autotakeoff",
                "OVERSPEED_GUARD_KTS",
                "MARGIN: below Vne, which is per-airframe",
            ),
            (
                "circuit",
                "APPROACH_LEG_MARGIN_KTS",
                "MARGIN: over the approach speed",
            ),
            // --- Known debt, named. Suspect on a jet, not yet measured wrong.
            (
                "autotakeoff",
                "OVERSPEED_VS_MAX_FPM",
                "DEBT: 1500 is a jet's ordinary climb",
            ),
            (
                "circuit",
                "ALT_MAX_EXTRA_DESCENT_FPM",
                "DEBT: 500 extra is little for a jet",
            ),
            (
                "autoland",
                "FLAP_EXTEND_MARGIN_KTS",
                "MARGIN: under the airframe's own per-detent Vfe",
            ),
            (
                "autoland",
                "FLAP_RETRACT_MARGIN_KTS",
                "MARGIN: over the airframe's own per-detent Vfe",
            ),
            (
                "autoland",
                "PLACARD_TARGET_MARGIN_KTS",
                "MARGIN: under the airframe's own per-detent Vfe",
            ),
            (
                "autoland",
                "PATH_CATCH_UP_FPM",
                "MARGIN: over the path's own sink at the groundspeed",
            ),
            (
                "procedure",
                "DRAG_OUT_KTS",
                "MARGIN: over the governor's own target",
            ),
            (
                "procedure",
                "DRAG_IN_KTS",
                "MARGIN: over the governor's own target",
            ),
            (
                "autotakeoff",
                "GEAR_WARN_MARGIN_KTS",
                "MARGIN: over the airframe's own .acf gear-warning speed",
            ),
            (
                "procedure",
                "FLAPS_ON_SPEED_MARGIN_KTS",
                "MARGIN: over the airframe's own clean floor",
            ),
            (
                "autotakeoff",
                "TRIM_SCALE_FROM_KTS",
                "PHYSICS: where a rudder is aerodynamic, on any airframe",
            ),
            (
                "autotakeoff",
                "REJECT_STOPPED_KTS",
                "STANDSTILL: a rejected roll has stopped, the parking brake goes on",
            ),
            (
                "autotakeoff",
                "REJECT_STEER_MIN_KTS",
                "STANDSTILL-ISH: a walk, the stop lets go of the steering",
            ),
            (
                "speedhold",
                "BREAKAWAY_BELOW_KTS",
                "STANDSTILL: the wheels are not turning, on any airframe",
            ),
            (
                "autotakeoff",
                "GEAR_VLO_MARGIN_KTS",
                "MARGIN: under the airframe's own gear placard",
            ),
            (
                "autotakeoff",
                "ROTATE_MODEL_MIN_KTS",
                "FLOOR on a dead ASI's (ref/ias)^2 in the airframe's own declared rotation model",
            ),
            (
                "procedure",
                "DRAG_MIN_DESCENT_FPM",
                "a gate on the COMMANDED rate, which is already the airframe's descent_fpm",
            ),
            (
                "procedure",
                "DRAG_KEEP_BELOW_FPM",
                "a gate on the COMMANDED rate: only a commanded climb stows a brake already out",
            ),
            (
                "procedure",
                "DRAG_SLOW_KTS",
                "MARGIN: under the speed owed, which is the airframe's (or its plan's)",
            ),
            (
                "procedure",
                "IDLE_DESCENT_ON_FPM",
                "a gate on the rate the vertical law WANTS, which is the airframe's descent_fpm",
            ),
            (
                "procedure",
                "IDLE_DESCENT_OFF_FPM",
                "a gate on the rate the vertical law WANTS, which is the airframe's descent_fpm",
            ),
            (
                "procedure",
                "SPEED_PROTECT_FROM_KTS",
                "MARGIN: over the speed owed, which is the airframe's (or its plan's)",
            ),
            (
                "procedure",
                "SPEED_PROTECT_SPAN_KTS",
                "MARGIN: over the speed owed; the descent it scales is the airframe's descent_fpm",
            ),
            (
                "circuit",
                "GO_AROUND_PLACARD_MARGIN_KTS",
                "MARGIN: under the airframe's own flap placard, cleaning up a go-around",
            ),
            (
                "circuit",
                "GO_AROUND_RETRACT_MARGIN_KTS",
                "MARGIN: over the airframe's own flap retraction speed",
            ),
            (
                "circuit",
                "CLEANUP_STEP_UNDER_CAP_KTS",
                "MARGIN: under the airframe's own flap placard less its margin",
            ),
        ];

        let files = [
            ("autotakeoff", "autotakeoff"),
            ("autotaxi", "autotaxi"),
            ("autoland", "autoland"),
            ("circuit", "circuit"),
            ("procedure", "procedure"),
            ("pushback", "pushback"),
            ("taxi_in", "taxi_in"),
            ("speedhold", "tools/speedhold"),
        ];
        let mut found = Vec::new();
        for (module, path) in &files {
            let text = crate::test_support::module_source(path);
            for line in text.lines() {
                // Laws only, and a law is declared at the top level. A
                // constant indented inside a function is that test's or that
                // helper's own fixture and binds no aircraft. (Truncating at
                // `#[cfg(test)]` instead cut several real laws out of the
                // scan, because these files gate items throughout.)
                // Widened only so a moved law can read it (`roll_steer`).
                let Some(rest) = line
                    .strip_prefix("const ")
                    .or_else(|| line.strip_prefix("pub(crate) const "))
                    // A law moved whole into its module's own file
                    // (`autoland/flare.rs`) is `pub(super)` for its parent.
                    .or_else(|| line.strip_prefix("pub(super) const "))
                else {
                    continue;
                };
                let Some((name, _)) = rest.split_once(": f64 = ") else {
                    continue;
                };
                if name == "MS_TO_KTS" {
                    continue; // a unit, not a speed
                }
                if name.ends_with("_KTS")
                    || name.ends_with("_FPM")
                    || name.ends_with("_DEG_S")
                    || name.ends_with("_KTS_S")
                {
                    found.push((*module, name.to_string()));
                }
            }
        }
        assert!(found.len() > 25, "the scan found the laws: {}", found.len());
        let unreviewed: Vec<String> = found
            .iter()
            .filter(|(m, n)| !REVIEWED.iter().any(|(rm, rn, _)| rm == m && rn == n))
            .map(|(m, n)| format!("{m}::{n}"))
            .collect();
        assert!(
            unreviewed.is_empty(),
            "new speed/rate constants must be per-airframe or reviewed here \
             (see CLAUDE.md: all automation numbers are per-airframe): {unreviewed:?}"
        );
        // ...and the list must not rot: an entry for a constant that no
        // longer exists is a reason nobody has read in a while.
        let stale: Vec<&str> = REVIEWED
            .iter()
            .filter(|(m, n, _)| !found.iter().any(|(fm, fn_)| fm == m && fn_ == n))
            .map(|(_, n, _)| *n)
            .collect();
        assert!(stale.is_empty(), "reviewed but gone: {stale:?}");
    }

    /// The A330 can start its own engines, and the block PARSES — every name
    /// and value out of the aircraft's own autostart script.
    ///
    /// It could not until 2026-09-20: neither existing start block could
    /// express an ENG MODE knob and guarded master switches, so the profile
    /// said "engine start is the pilot's" and the copilot had no route to it
    /// at all. Found sitting on the ramp unable to start.
    #[test]
    fn the_a330_declares_an_airliner_start() {
        let p = load("airbus_a330_300").expect("the A330 loads");
        let a = p.airliner_start.as_ref().expect("an airliner start block");
        assert!(a.mode_selector.ends_with("eng_mode_selector"));
        assert_eq!((a.mode_start, a.mode_norm), (1.0, 0.0), "IGN/START, NORM");
        assert!(a.masters.ends_with("eng_master"));
        // One guard per engine, or none at all — a guard list that covers
        // only some of the switches would silently no-op the rest.
        assert!(
            a.master_guards.is_empty() || a.master_guards.len() == p.engines,
            "{} guards for {} engines",
            a.master_guards.len(),
            p.engines
        );
        assert_eq!(a.guard_lift, 0.002, "the script's own lift value");
        assert!(a.guard_secs > 0.0, "and it must be dropped again");
        // The APU is a sequence of COMMANDS, and the momentary START button
        // is the one step with no position to read — it does not latch.
        assert!(
            a.apu_steps.len() >= 4,
            "batteries, APU battery, master, START"
        );
        assert!(
            a.apu_steps.iter().any(|s| s.held_secs.is_some()),
            "the START button is a press-and-hold"
        );
        assert!(
            a.apu_steps
                .iter()
                .filter(|s| s.held_secs.is_none())
                .all(|s| s.position.is_some()),
            "every LATCHING step reads before it fires: a toggle at a switch \
             already on turns it off"
        );
        // The FUEL PUMPS. The first version of this block went APU straight
        // to master switches — a start with no fuel at the engine. The
        // aircraft's own sequence spends eight of its twenty-four steps here,
        // so a pre-start list without them is not a start sequence.
        let pumps = a
            .pre_start_steps
            .iter()
            .filter(|s| s.command.contains("pump"))
            .count();
        assert_eq!(pumps, 8, "all eight fuel pumps, as the aircraft does it");
        assert!(
            a.pre_start_steps
                .iter()
                .any(|s| s.command.contains("apu_toggle")),
            "start air has to be selected"
        );
        // THE ADIRS. Without them the engines run and the aircraft has no
        // attitude, heading or air data to fly on: every PFD tape flags red
        // and the ND reads MAP NOT AVAIL. Three IRs, three ADRs, three
        // selectors to NAV.
        let count = |frag: &str| {
            a.pre_start_steps
                .iter()
                .filter(|s| s.command.contains(frag))
                .count()
        };
        assert_eq!(
            count("/ir1_toggle") + count("/ir2_toggle") + count("/ir3_toggle"),
            3
        );
        assert_eq!(
            count("adr1_toggle") + count("adr2_toggle") + count("adr3_toggle"),
            3
        );
        let knobs: Vec<_> = a
            .pre_start_steps
            .iter()
            .filter(|s| s.command.contains("knob"))
            .collect();
        assert_eq!(knobs.len(), 3, "one selector per ADIRS");
        // A three-position knob needs a direction. Told only "press right
        // until it reads NAV", one left in ATT walks away from it.
        assert!(
            knobs.iter().all(|s| s.down_command.is_some()),
            "every knob can step both ways"
        );
        // ...and the ADIRS come BEFORE the engines are cranked, so alignment
        // runs while they start rather than after.
        let first_adirs = a
            .pre_start_steps
            .iter()
            .position(|s| s.command.contains("adirs"))
            .expect("adirs present");
        let beacon = a
            .pre_start_steps
            .iter()
            .position(|s| s.command.contains("beacon"))
            .expect("beacon present");
        assert!(first_adirs < beacon, "alignment starts first");

        // ...and the copilot WATCHES them for the rest of the flight, not
        // just at the start. A start can be missed, and an aircraft can be
        // loaded with them off — and the failure is silent until every tape
        // on the PFD is red. These rows fix rather than ask: there is no
        // judgement in "the inertial reference should be on".
        let watched: Vec<_> = p
            .advisor
            .iter()
            .filter(|r| r.dataref.contains("adirs"))
            .collect();
        assert_eq!(watched.len(), 9, "three IRs, three ADRs, three selectors");
        assert!(
            watched.iter().all(|r| r.command.is_some()),
            "every ADIRS row is actioned, not merely advised"
        );
        assert!(
            watched.iter().all(|r| r.expect_on),
            "off is always the failure"
        );
        // Watched on the GROUND too — before it matters, not once airborne
        // with nothing to fly on.
        assert!(
            watched
                .iter()
                .all(|r| r.phases.iter().any(|p| p == "ground")),
            "caught before we go"
        );
        // ...and the APU stops carrying the bleed once the engines have it.
        assert!(
            a.post_start_steps.iter().any(|s| s.want == 0.0),
            "the APU is shut down, not left running on its bleed"
        );
        assert!(
            a.post_start_steps
                .iter()
                .filter(|s| s.command.contains("pack"))
                .count()
                == 2,
            "both packs come back on"
        );
        // The START button is a command, and its name is ALSO a dataref on
        // this aircraft — the button's position, rewritten every frame.
        // Declaring it as a dataref is what made the APU never move.
        assert!(a
            .apu_steps
            .iter()
            .any(|s| s.command.ends_with("APU_start") && s.position.is_none()));
        assert!(a.spool_idle > 0.0 && a.apu_ready_n1 > 0.0);
        // ...and it must not also claim a start block it cannot fly: two
        // tools called `start_engines` is a registry that answers twice.
        assert!(
            !p.turbine_start,
            "the MD-82 sequence is not this aircraft's"
        );
        assert!(p.turboprop_start.is_none(), "nor are condition levers");
    }

    /// The ground roll is scaled to what the aircraft WEIGHS, and the
    /// scaling errs upward.
    ///
    /// Edinburgh 24 is 2,549 m; the A330's profile roll of 6,000 ft times the
    /// caller's 1.4 factor asks 2,560. Eleven metres, at MAX takeoff weight,
    /// on a light sector — and the refusal came from every position on the
    /// runway, having sent the pilot to line up and then to backtrack
    /// (2026-09-20). Real A330s operate into Edinburgh.
    #[test]
    fn the_takeoff_roll_follows_the_weight_and_errs_upward() {
        let mtow = 533_519.0 * 0.453_592; // the A330's, kg
        let roll = 6000.0 * 0.3048; // its MTOW ground roll, m
                                    // At max weight nothing changes — the profile figure IS that case.
        assert_eq!(takeoff_roll_m(roll, Some(mtow), Some(mtow)), roll);
        // On a light sector it asks for less, and EGPH becomes flyable.
        let light = takeoff_roll_m(roll, Some(180_000.0), Some(mtow));
        assert!(
            light * 1.4 < 2549.0,
            "EGPH fits at 180 t: {:.0}",
            light * 1.4
        );
        // ...but NOT as little as the square law would allow. Measured
        // against published field lengths the real exponent is about 1.1, so
        // linear asks slightly MORE than the aircraft needs and the square
        // law asks far less. When a margin is wrong it is wrong upward.
        let square = roll * (180_000.0 / mtow).powi(2);
        assert!(
            light > square * 1.2,
            "linear must be well above the square law: {light:.0} vs {square:.0}"
        );
        // A floor, because nothing here models density altitude, a wet
        // runway or a rejected takeoff.
        assert_eq!(takeoff_roll_m(roll, Some(1.0), Some(mtow)), roll * 0.5);
        // Over max weight asks no less than the figure itself.
        assert_eq!(takeoff_roll_m(roll, Some(mtow * 2.0), Some(mtow)), roll);
        // An unknown weight or MTOW keeps the old behaviour exactly.
        assert_eq!(takeoff_roll_m(roll, None, Some(mtow)), roll);
        assert_eq!(takeoff_roll_m(roll, Some(180_000.0), None), roll);
        assert_eq!(takeoff_roll_m(roll, Some(f64::NAN), Some(mtow)), roll);
    }

    /// A retraction schedule has to be FLYABLE: the speed that lets a detent
    /// be selected must sit under the placard of the detent being left. The
    /// A330 cleaned up "above 230" out of a CONF 1 whose Vfe is 215, so the
    /// clean-up could never legally happen and never did: flap 1 rode the
    /// climb to 330 kt (EGPH, 2026-09-20).
    /// Every fixed-wing profile declares its landing approach speed, and it
    /// sits where a crew flies one: 1.1 to 1.7 times its landing stall (a
    /// jet's Vref is about 1.15 to 1.23; the
    /// light singles' POH finals run 1.5 to 1.6: the 172SP's 65 on a Vso of
    /// 40, the RV-10's 80 on 50).
    /// Undeclared, the SR22 fell back to 1.3 x its CLEAN stall (91 against the
    /// POH's 80) and floated and bounced every landing (LEMG 13, 2026-09-29).
    #[test]
    fn every_aeroplane_declares_a_landing_speed_near_its_landing_stall() {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("profiles");
        let mut checked = 0;
        for e in std::fs::read_dir(dir).unwrap().flatten() {
            let path = e.path();
            if path.extension().is_none_or(|x| x != "yaml") {
                continue;
            }
            let text = std::fs::read_to_string(&path).unwrap();
            let Ok(profile) = serde_yaml::from_str::<AircraftProfile>(&text) else {
                continue;
            };
            let name = path.file_stem().unwrap().to_string_lossy().to_string();
            // Rotorcraft have no landing speed.
            if profile.vs_clean_kts.is_none() && profile.flaps.is_none() {
                continue;
            }
            let r = profile.reference.as_ref();
            let app = r.and_then(|r| r.approach_kts);
            assert!(app.is_some(), "{name}: no reference approach_kts");
            if let Some(vso) = r.and_then(|r| r.vs0_kts) {
                let k = app.unwrap() / vso;
                assert!((1.1..=1.7).contains(&k), "{name}: approach {app:?} is {k:.2} x Vso {vso}");
            }
            checked += 1;
        }
        assert!(checked >= 10, "{checked}");
    }

    #[test]
    fn every_profiles_flap_schedule_can_be_flown_inside_its_placards() {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("profiles");
        let mut checked = 0;
        for e in std::fs::read_dir(dir).unwrap().flatten() {
            let path = e.path();
            if path.extension().is_none_or(|x| x != "yaml") {
                continue;
            }
            let text = std::fs::read_to_string(&path).unwrap();
            let Ok(profile) = serde_yaml::from_str::<AircraftProfile>(&text) else {
                continue;
            };
            let Some(flaps) = &profile.flaps else {
                continue;
            };
            let mut detents = flaps.detents.clone();
            detents.sort_by(|a, b| a.ratio.total_cmp(&b.ratio));
            for pair in detents.windows(2) {
                let (cleaner, dirtier) = (&pair[0], &pair[1]);
                if let (Some(gate), Some(vfe)) = (cleaner.retract_above_kts, dirtier.vfe_kts) {
                    checked += 1;
                    assert!(
                        gate < vfe,
                        "{}: retracting to {} waits for {gate} kt, past the {vfe} kt placard of {}",
                        path.display(),
                        cleaner.label,
                        dirtier.label
                    );
                }
            }
        }
        assert!(checked > 0, "no schedule was checked");
    }

    /// The pavement rule is ONE function. Full auto kept its own copy without
    /// the weight scaling and said "I couldn't get us onto the runway" to an
    /// A330 on Edinburgh 24's centreline, which the spoken takeoff accepted
    /// (2026-09-20, about 200 t on the wheels).
    #[test]
    fn one_pavement_rule_and_edinburgh_fits_it_at_the_weight_flown() {
        let mtow = 533_519.0 * 0.453_592;
        let roll = Some(6000.0 * 0.3048);
        // As flown: fits the 2,549 m strip, and the 2,338 m left from the
        // displaced threshold the lead-on joins at.
        let flown = takeoff_needed_m(roll, Some(200_000.0), Some(mtow), 500.0);
        assert!(flown < 2338.0, "{flown:.0}");
        // At max weight, or with the weight unread, the old refusal stands.
        assert!(takeoff_needed_m(roll, Some(mtow), Some(mtow), 500.0) > 2549.0);
        assert!(takeoff_needed_m(roll, None, Some(mtow), 500.0) > 2549.0);
        // No roll on file is the caller's floor; a tiny roll never goes under it.
        assert_eq!(
            takeoff_needed_m(None, Some(1000.0), Some(1100.0), 400.0),
            400.0
        );
        assert_eq!(takeoff_needed_m(Some(100.0), None, None, 500.0), 500.0);
    }

    /// EVERY jet and turboprop declares the numbers the laws would otherwise
    /// take from a Cessna 172, and they PARSE.
    ///
    /// The 737 was fixed on 2026-09-20 and five more turbine profiles were
    /// still silent that evening — each one would have repeated the whole
    /// sortie: a 0.8 nm capture radius, a 700 fpm descent, a 0.10 nose-down
    /// limit, avgas density and an 800 fpm go-around. Silence is a legal
    /// default (it has to be, so nothing GA moves), which is exactly why the
    /// turbines have to be named here instead.
    #[test]
    fn every_turbine_profile_declares_its_own_numbers() {
        const TURBINES: &[&str] = &[
            "boeing_737_800",
            "airbus_a330_300",
            "boeing_747_400",
            "md82",
            "citation_x",
            "cirrus_vision_sf50",
            "dhc6_twin_otter",
            "king_air_c90b",
            "lancair_evolution",
        ];
        for id in TURBINES {
            let p = load(id).unwrap_or_else(|| panic!("{id} loads"));
            let kind = p
                .reference
                .as_ref()
                .and_then(|r| r.kind.as_deref())
                .unwrap_or("");
            assert!(
                kind.contains("jet") || kind.contains("turboprop"),
                "{id} is listed as a turbine but says {kind:?}"
            );
            assert!(
                p.pitch_authority_down.is_some_and(|a| a > 0.10),
                "{id}: the rotation needs more push than a 172's tenth"
            );
            let f = p
                .fuel
                .as_ref()
                .unwrap_or_else(|| panic!("{id} has no fuel block, so it briefs in avgas"));
            assert_eq!(f.kg_per_gal, Some(3.04), "{id} burns Jet A-1, not 100LL");
            assert!(f.spoken_unit.is_some(), "{id} must name its crew's unit");
            let r = p.reference.as_ref().expect("reference");
            let climb = r.enroute_climb_fpm.unwrap_or(0.0);
            let descent = r.enroute_descent_fpm.unwrap_or(0.0);
            assert!(
                climb > 700.0 && descent > 700.0,
                "{id}: {climb}/{descent} is the GA figure or worse"
            );
            // The optimum-altitude advisory is all-or-nothing: a partial
            // declaration would silently produce no advice at all.
            let opt = [
                r.optimum_alt_ft,
                r.optimum_alt_at_kg,
                r.optimum_alt_ft_per_1000kg,
            ];
            assert!(
                opt.iter().all(Option::is_some) || opt.iter().all(Option::is_none),
                "{id}: optimum altitude is half-declared, which advises nothing"
            );
        }
    }

    /// Everything this airframe was given today, read back out of its YAML —
    /// a profile field that does not parse is a fix that never shipped.
    #[test]
    fn the_737_profile_declares_its_jet_numbers() {
        let p = load("boeing_737_800").expect("the 737 profile loads");
        assert_eq!(p.pitch_authority_down, Some(0.5), "nose-down authority");
        assert_eq!(p.tail_strike_deg, Some(11.0));
        let f = p.fuel.as_ref().expect("fuel block");
        assert_eq!(f.kg_per_gal, Some(3.04), "Jet A-1, not 100LL");
        assert_eq!(f.spoken_unit.as_deref(), Some("kg"));
        let r = p.reference.as_ref().expect("reference block");
        assert_eq!(r.enroute_climb_fpm, Some(2000.0));
        assert_eq!(r.enroute_descent_fpm, Some(2000.0));
        assert_eq!(r.optimum_alt_ft, Some(38_000.0));
        // The retraction schedule the climb-out walks: 5, then 1 at 190,
        // then up at 210, cleanest first. Detent 2 declared a speed too and
        // added a step the profile's own comment did not have (Alghero,
        // 2026-09-27).
        assert_eq!(
            flap_schedule(p.flaps.as_ref()),
            vec![(0.0, 210.0), (0.125, 190.0)]
        );
        assert_eq!(flap_detent_spoken(p.flaps.as_ref(), 0.125).as_deref(), Some("one"));
        assert_eq!(flap_detent_spoken(p.flaps.as_ref(), 0.0).as_deref(), Some("up"));
        assert_eq!(flap_detent_spoken(p.flaps.as_ref(), 0.3), None);
    }

    /// The BIKF-EGPH brief, verbatim from the panel (2026-09-20): 3,126 kg in
    /// a 737's tanks read back as "1,149 gallons". That figure is 3126/2.72 —
    /// AVGAS density, on an airframe that burns Jet A-1.
    #[test]
    fn a_jets_fuel_is_not_spoken_in_avgas_gallons() {
        let _g = FUEL_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let jet = FuelProfile {
            reserve_caution_min: 60.0,
            reserve_warning_min: 30.0,
            cruise_burn_kg_hr: 2500.0,
            kg_per_gal: Some(3.04),
            spoken_unit: Some("kg".into()),
        };
        publish_fuel_unit(Some(&jet));
        assert_eq!(say_fuel_kg(3126.0), "3,126 kilos", "the crew's own unit");
        // Had it stayed in gallons, it would at least be the right ones.
        let jet_gal = FuelProfile {
            spoken_unit: Some("gallons".into()),
            ..jet.clone()
        };
        publish_fuel_unit(Some(&jet_gal));
        assert_eq!(say_fuel_kg(3126.0), "1,028 gallons", "Jet A, not 100LL");
        // A piston declares nothing and is unchanged: 100LL, gallons — the
        // 1,149 that started all this is CORRECT for a light aircraft.
        publish_fuel_unit(None);
        assert_eq!(say_fuel_kg(3126.0), "1,149 gallons");
        assert_eq!(fuel_unit().0, AVGAS_KG_PER_GAL);
        // And a swap off the jet must not leave its density behind.
        publish_fuel_unit(Some(&jet));
        publish_fuel_unit(None);
        assert_eq!(
            say_fuel_kg(3126.0),
            "1,149 gallons",
            "never outlives its airframe"
        );
    }

    /// The figures in the brief are rounded into the airframe's unit BEFORE
    /// they are subtracted, so they add up on screen. The panel said "You
    /// have 1149 aboard, but the plan needs 3002; short about 1852" — and
    /// 3002 minus 1149 is 1853.
    #[test]
    fn the_spoken_fuel_figures_agree_with_each_other() {
        let _g = FUEL_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        publish_fuel_unit(None);
        let aboard = fuel_in_unit_kg(3126.0).round();
        let need = fuel_in_unit_kg(8165.0).round();
        assert_eq!(say_fuel_figure(aboard), "1,149 gallons");
        assert_eq!(say_fuel_figure(need), "3,002 gallons");
        assert_eq!(
            say_fuel_figure(need - aboard),
            "1,853 gallons",
            "a difference of rounded figures, never a rounded difference"
        );
    }

    #[test]
    fn a_bad_fuel_declaration_is_refused_rather_than_flown() {
        let _g = FUEL_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let bad = FuelProfile {
            reserve_caution_min: 60.0,
            reserve_warning_min: 30.0,
            cruise_burn_kg_hr: 2500.0,
            kg_per_gal: Some(0.0),
            spoken_unit: Some("furlongs".into()),
        };
        publish_fuel_unit(Some(&bad));
        assert_eq!(fuel_unit().0, AVGAS_KG_PER_GAL, "a zero density is refused");
        assert_eq!(
            fuel_unit().1,
            FuelSpoken::Gallons,
            "an unknown unit falls back"
        );
        publish_fuel_unit(None);
    }

    #[test]
    fn cessna_yaml_loads_and_matches() {
        // load() prefers profiles/cessna_172sp.yaml (cwd is the crate root in tests).
        let p = load("cessna_172sp").expect("profile loads");
        assert_eq!(p.id, "cessna_172sp");
        assert!(p.autopilot && p.engine && p.barometer && p.com_radio);
        assert!(!p.retractable_gear);
        assert_eq!(p.lights.len(), 5);
        assert_eq!(p.taxi.as_ref().and_then(|t| t.max_yaw_dps), Some(10.0));
        // SOP advisor rows: taxi light on the ground, strobes airborne, landing
        // light for takeoff/arrival.
        assert_eq!(p.advisor.len(), 3);
        assert!(p.advisor.iter().all(|a| a.expect_on));
        // The taxi light auto-comes-on for taxi (the ground phase)...
        let taxi = p
            .advisor
            .iter()
            .find(|a| a.command.as_deref() == Some("sim/lights/taxi_lights_on"))
            .expect("a taxi-light SOP row");
        assert_eq!(taxi.phases, vec!["ground".to_string()]);
        // ...and the landing light is never expected on the ground (it dazzles
        // other crews — that's the taxi light's job).
        let landing = p
            .advisor
            .iter()
            .find(|a| a.command.as_deref() == Some("sim/lights/landing_lights_on"))
            .expect("a landing-light SOP row");
        assert!(!landing.phases.iter().any(|ph| ph == "ground"));
        let flaps = p.flaps.expect("has flaps");
        assert_eq!(flaps.detents.len(), 4);
        assert_eq!(flaps.detents[0].label, "up");
        assert_eq!(flaps.detents[3].label, "full");
        assert_eq!(flaps.detents[1].vfe_kts, Some(110.0));
        let fuel = p.fuel.expect("has fuel thresholds");
        assert_eq!(fuel.reserve_caution_min, 45.0);
        assert_eq!(fuel.reserve_warning_min, 30.0);
        // Reference figures load from YAML (and match the built-in), so the
        // copilot can recite V-speeds/limits without hardcoded literals.
        let r = p.reference.as_ref().expect("has reference figures");
        assert_eq!(r.best_glide_kts, Some(68.0));
        assert_eq!(r.va_kts, Some(99.0));
        assert_eq!(r.vx_kts, cessna_172sp().reference.unwrap().vx_kts);
        // The checklist action map loads, and the YAML matches the built-in (same
        // matches, in the same order) — so behaviour is identical either source.
        let yaml_map = &p.checklist_map;
        let builtin_map = &cessna_172sp().checklist_map;
        assert!(!yaml_map.is_empty());
        assert_eq!(yaml_map.len(), builtin_map.len());
        for (a, b) in yaml_map.iter().zip(builtin_map) {
            assert_eq!(a.match_, b.match_, "map entry mismatch (YAML vs built-in)");
        }
        // Electrical loads from YAML (not a silent fall-back to the built-in) and
        // matches it key-for-key.
        let builtin = cessna_172sp();
        let yaml_elec: Vec<&str> = p.electrical.iter().map(|e| e.key.as_str()).collect();
        let builtin_elec: Vec<&str> = builtin.electrical.iter().map(|e| e.key.as_str()).collect();
        assert_eq!(
            yaml_elec, builtin_elec,
            "electrical mismatch (YAML vs built-in)"
        );
    }

    #[test]
    fn yaml_profiles_parse_strictly() {
        // load() falls back to the built-in on a parse error, which silently
        // masks a malformed profile (a bad dataref, an un-tagged action enum, a
        // bare `off` collapsing to YAML 1.1 boolean false). Deserialize each file
        // directly so any such breakage fails loudly here instead of vanishing
        // behind the fallback.
        for id in ["cessna_172sp", "cirrus_sr22"] {
            let text = std::fs::read_to_string(format!("profiles/{id}.yaml")).unwrap();
            let p: AircraftProfile = serde_yaml::from_str(&text)
                .unwrap_or_else(|e| panic!("{id}.yaml failed to parse: {e:#}"));
            assert_eq!(p.id, id);
            let fs = p.fuel_selector.expect("fuel selector present");
            assert!(fs
                .positions
                .iter()
                .all(|p| !p.key.is_empty() && !p.command.is_empty()));
            assert!(!p.checklist_map.is_empty());
        }
    }

    /// An airframe whose own lua OWNS the stock mixture handle must declare the
    /// lever it is really cut with (2026-09-20). The 737's B738.systems.lua
    /// rewrites `mixture_ratio[0..1]` every frame from `condition_lever{1,2}`,
    /// so the stock write is accepted and then erased, and "fuel levers cut"
    /// was a lie at the EIDW stand. Probed live: `condition_lever1` is_writable
    /// FALSE, `_target` TRUE — the TARGET is the control.
    #[test]
    fn the_737_declares_the_lever_it_is_actually_cut_with() {
        let text = std::fs::read_to_string("profiles/boeing_737_800.yaml").unwrap();
        let p: AircraftProfile = serde_yaml::from_str(&text).unwrap();
        assert_eq!(
            p.fuel_cutoff_levers.len(),
            p.engines,
            "one cutoff lever per engine, or the second one runs on"
        );
        for lever in &p.fuel_cutoff_levers {
            assert!(
                !lever.contains("mixture_ratio"),
                "{lever}: the lua rewrites the stock handle every frame"
            );
            assert!(
                lever.ends_with("_target"),
                "{lever}: condition_lever itself is an animation output, read-only"
            );
        }
        assert_eq!(
            p.fuel_cutoff_value, 0.0,
            "the lua lights the engine at >= 0.75"
        );
        // And the probe covers them, so a renamed lever is said at boot rather
        // than found at a stand.
        let refs = profile_references(&p);
        for lever in &p.fuel_cutoff_levers {
            assert!(refs.contains(lever), "{lever} is not probed");
        }
    }

    /// The stock airliners (2026-09-16): parsed strictly, and the numbers the
    /// automation flies are the airframe's own, never the GA defaults (the
    /// RV-10 lesson at 174,000 lb).
    #[test]
    fn stock_airliner_yamls_parse_strictly_and_carry_their_own_numbers() {
        for (id, vr, roll) in [
            ("boeing_737_800", 145.0, 5500.0),
            ("airbus_a330_300", 148.0, 6000.0),
        ] {
            let text = std::fs::read_to_string(format!("profiles/{id}.yaml")).unwrap();
            let p: AircraftProfile = serde_yaml::from_str(&text)
                .unwrap_or_else(|e| panic!("{id}.yaml failed to parse: {e:#}"));
            assert_eq!(p.id, id);
            assert_eq!(p.engines, 2);
            assert_eq!(
                p.pressurised,
                Some(true),
                "{id}: cabin alerts need the real answer"
            );
            assert!(p.cabin_crew && p.retractable_gear && p.throttle && !p.engine);
            assert!(
                !p.autothrottle,
                "{id}: the md82 autothrottle tool would withhold hold_speed"
            );
            assert!(
                !p.turbine_start && !p.auto_prep,
                "{id}: the MD-82's APU/bleed sequence is not this aircraft's"
            );
            assert_eq!(
                p.scripted_auto_start,
                id == "boeing_737_800",
                "{id}: the 737 scripts its own start, the A330 does not (yet)"
            );
            let r = p.reference.as_ref().expect("reference card");
            assert_eq!(r.vr_kts, Some(vr));
            assert_eq!(
                r.takeoff_roll_ft,
                Some(roll),
                "{id}: the roll the runway check uses"
            );
            assert!(r.landing_distance_ft.unwrap() > 4000.0);
            assert!(r.approach_kts.unwrap() >= 140.0 && p.vs_clean_kts.unwrap() > 130.0);
            let f = p.flaps.as_ref().expect("flap detents");
            let to = f.takeoff_ratio.unwrap();
            assert!(
                f.detents.iter().any(|d| (d.ratio - to).abs() < 1e-9),
                "{id}: takeoff flap is a gate"
            );
            assert!(f.detents.last().unwrap().vfe_kts.is_some());
            assert!(p.fuel.as_ref().unwrap().cruise_burn_kg_hr > 1000.0);
            assert!(p.tail_strike_deg.is_some());
            // A jet's INITIAL climb is not its enroute climb speed, and the
            // takeoff's climb law protects the number it is given: fed the
            // enroute 210, it read the 737's real 174 kt as thirty-six knots
            // slow and commanded 150 fpm at takeoff thrust (2026-09-17).
            let ic = r
                .initial_climb_kts
                .expect("an airliner declares its initial climb");
            assert!(
                ic >= vr && ic < r.vy_kts.unwrap(),
                "{id}: initial climb {ic} must sit between Vr and the enroute climb"
            );
        }
    }

    #[test]
    fn dc3_yaml_loads_and_is_conservative() {
        // YAML-only profile (no built-in). Deserialize the file directly so a
        // malformed field fails loudly here rather than vanishing behind load()'s
        // built-in fallback (which would return None for "dc3").
        let text = std::fs::read_to_string("profiles/dc3.yaml").unwrap();
        let p: AircraftProfile = serde_yaml::from_str(&text)
            .unwrap_or_else(|e| panic!("dc3.yaml failed to parse: {e:#}"));
        assert_eq!(p.id, "dc3");
        // Twin radial: retractable gear with a VLO gate, throttle-only (the piston
        // group stays off — mixture is xlua-overridden, start/ignition are custom).
        assert!(p.retractable_gear);
        assert_eq!(p.vlo_kts, Some(140.0));
        assert!(p.throttle && !p.engine);
        assert!(p.fuel_selector.is_none()); // uses the dual cycle selectors instead
                                            // Twin per-engine accessories, verified live: magnetos (via ignition_on),
                                            // carb heat, and the boost pumps, all written across both engines.
        assert_eq!(p.engines, 2);
        assert!(p.magnetos && p.magnetos_per_engine && p.carb_heat && p.fuel_pump);
        // No functional X-Plane autopilot on this airframe (servos never latch,
        // verified live), so autopilot + ILS are off; the rest of the universal
        // tools stay on.
        assert!(!p.autopilot && !p.ils);
        assert!(p.barometer && p.transponder && p.pitch_trim);
        // Flaps: up + 1/4, 1/2, 3/4, full with descending Vfe.
        let flaps = p.flaps.expect("has flaps");
        assert_eq!(flaps.detents.len(), 5);
        assert_eq!(flaps.detents[0].label, "up");
        assert_eq!(flaps.detents[4].label, "full");
        assert_eq!(flaps.detents[1].vfe_kts, Some(130.0));
        assert_eq!(flaps.detents[4].vfe_kts, Some(97.0));
        assert_eq!(p.vne_kts, Some(178.0));
        assert_eq!(p.vs_clean_kts, Some(65.0));
        let r = p.reference.as_ref().expect("dc3 reference");
        assert_eq!(r.vr_kts, Some(84.0));
        assert_eq!(r.max_gross_lb, Some(25200.0));
        // Custom cycle-switch systems wired from config: power switch, dual fuel
        // selectors, and the twin piston start.
        let power = p.power_switch.as_ref().expect("dc3 power switch");
        assert_eq!(power.switch.up, "awx/c47/powersw_up");
        assert!(!power.switch.wrap); // clamped at the ends
        assert_eq!(power.states.first().map(|s| s.value), Some(0)); // battery first
        assert_eq!(p.fuel_selectors.len(), 2);
        let left = &p.fuel_selectors[0];
        assert_eq!(left.key, "left");
        assert_eq!(left.engine, 0);
        assert_eq!(left.start_value, Some(2)); // left main
        assert!(left.switch.wrap); // selectors wrap 0<->4
        assert_eq!(p.fuel_selectors[1].start_value, Some(3)); // right main
        let start = p.piston_start.as_ref().expect("dc3 piston start");
        assert_eq!(start.starters.len(), 2);
        assert_eq!(start.mixture_start_notch, 2.0);
        assert_eq!(start.mixture_cutoff_notch, 0.0);
        let mix = p.mixture_notch.as_ref().expect("dc3 mixture notch");
        assert_eq!(mix.notches.len(), 4);
        assert_eq!(mix.notches.first().map(|n| n.value), Some(0)); // cutoff
        assert_eq!(mix.notches.last().map(|n| n.value), Some(3)); // full rich
    }

    /// The Baron is the first STOCK piston twin: unlike the DC-3 (every system
    /// xlua-overridden) the generic tools drive it directly, so what this test
    /// guards is that the twin-ness actually reaches them.
    #[test]
    fn beechcraft_baron_58_yaml_loads() {
        let text = std::fs::read_to_string("profiles/beechcraft_baron_58.yaml").unwrap();
        let p: AircraftProfile = serde_yaml::from_str(&text)
            .unwrap_or_else(|e| panic!("beechcraft_baron_58.yaml failed to parse: {e:#}"));
        assert_eq!(p.id, "beechcraft_baron_58");

        // Twin, generic piston group. `engines` is what makes the magneto and
        // start tools write both engines rather than only number one.
        assert_eq!(p.engines, 2);
        assert!(p.engine && p.magnetos && p.fuel_pump);
        assert!(!p.magnetos_per_engine); // ignition_key, not the DC-3's ignition_on
        assert!(!p.carb_heat); // IO-550, fuel injected
        assert!(!p.turbine_start && p.piston_start.is_none());

        // Per-side fuel selectors, cycle-driven. Without these the start
        // sequence cranks two dry engines: they sit at OFF cold, and their
        // position dataref is read-only, so only the command can open them.
        assert_eq!(p.fuel_selectors.len(), 2);
        let (l, r) = (&p.fuel_selectors[0], &p.fuel_selectors[1]);
        assert_eq!((l.engine, r.engine), (0, 1));
        assert_eq!((l.start_value, r.start_value), (Some(1), Some(1)));
        assert!(l.switch.up.ends_with("_L_up") && r.switch.up.ends_with("_R_up"));
        // Never step onto a position we have not proven exists in the sim.
        assert_eq!((l.switch.min, l.switch.max), (0, 1));
        assert!(!l.switch.wrap, "off must not wrap round from on");

        // Retractable gear with a limit — the 172's whole reason for having no
        // gear tool is that it has none.
        assert!(p.retractable_gear);
        assert_eq!(p.vlo_kts, Some(152.0));

        // Per-notch Vfe is the point of a flap profile: approach flap is good to
        // 152 KIAS and full only to 122, and one figure for both would put full
        // flap out thirty knots early.
        let flaps = p.flaps.as_ref().expect("baron flaps");
        assert_eq!(flaps.detents.len(), 3);
        assert_eq!(flaps.takeoff_ratio, Some(0.0)); // clean takeoff
        let vfe: Vec<Option<f64>> = flaps.detents.iter().map(|d| d.vfe_kts).collect();
        assert_eq!(vfe, vec![None, Some(152.0), Some(122.0)]);

        // The twin speeds a multi pilot actually asks for.
        let r = p.reference.as_ref().expect("baron reference");
        assert_eq!(r.vyse_kts, Some(100.0)); // blue line
        assert_eq!(r.vmca_kts, Some(84.0));
        assert_eq!(r.kind.as_deref(), Some("twin-engine aircraft"));
        // The ground roll, not a factored field length (the MD-82 lesson).
        assert_eq!(r.takeoff_roll_ft, Some(1373.0));

        // Two independent landing-light switches, written together: switching
        // one leaves the aircraft half lit with a readback that confirms the
        // half it looked at (the Citation X trap).
        let landing = p
            .lights
            .iter()
            .find(|l| l.key == "landing")
            .expect("baron landing light");
        assert_eq!(landing.also_datarefs.len(), 1);
        assert!(landing.command_on.is_none(), "written, never commanded");

        // Roof floods: a left and a right pod, written together for the same
        // reason, and the one light on this airframe the copilot sets itself
        // after dark.
        let roof = p
            .lights
            .iter()
            .find(|l| l.key == "roof")
            .expect("baron roof lights");
        assert_eq!(roof.dataref, "laminar/b58/lighting/roof_L_on");
        assert_eq!(
            roof.also_datarefs,
            vec!["laminar/b58/lighting/roof_R_on".to_string()],
            "one pod lit is a readback confirming the half it looked at"
        );
        assert!(roof.auto_at_night);
        // Interior only. Switching an EXTERIOR light on the pilot's behalf
        // changes what other traffic sees, so nothing out there is ever
        // flagged: this assertion is the guard on that rule, not a restatement
        // of the profile.
        for l in p.lights.iter().filter(|l| l.auto_at_night) {
            assert_eq!(l.key, "roof", "{} is not an interior light", l.key);
        }

        // The flood switch is a COMMAND with a read-only position dataref —
        // probing the live sim showed no `flood_light_switch` DATAREF exists,
        // so the profile's old `flood_lights_dataref` wrote into nothing and
        // read back nothing (2026-08-02).
        assert!(p.flood_lights);
        assert!(
            p.flood_lights_dataref.is_none(),
            "there is no such dataref on this airframe"
        );
        let flood = p.flood_lights_toggle.as_ref().expect("baron flood toggle");
        assert_eq!(flood.command, "laminar/b58/lighting/flood_light_switch");
        assert_eq!(flood.dataref, "laminar/b58/flood_light_toggle/switch_pos");

        // The GNS430's welcome page, cleared once per power-up.
        assert_eq!(p.gps_ack_command.as_deref(), Some("sim/GPS/g430n1_ent"));

        // One battery, one alternator per engine, plus the pair and avionics.
        let keys: Vec<&str> = p.electrical.iter().map(|e| e.key.as_str()).collect();
        assert_eq!(
            keys,
            vec![
                "battery",
                "alternator1",
                "alternator2",
                "alternators",
                "avionics"
            ]
        );
    }

    #[test]
    fn king_air_c90b_yaml_loads() {
        // YAML-only profile. Deserialize directly so a malformed field fails here
        // rather than silently falling back.
        let text = std::fs::read_to_string("profiles/king_air_c90b.yaml").unwrap();
        let p: AircraftProfile = serde_yaml::from_str(&text)
            .unwrap_or_else(|e| panic!("king_air_c90b.yaml failed to parse: {e:#}"));
        assert_eq!(p.id, "king_air_c90b");
        // Twin turboprop: power-lever throttle (not the piston group), retractable
        // gear, no piston/turbine-start/AP tools for v1.
        assert!(p.throttle && !p.engine);
        assert!(p.retractable_gear);
        // Real AP, verified live (needs engines running); no piston/jet-start tools.
        assert!(p.autopilot && p.ils);
        assert!(!p.turbine_start && !p.magnetos && !p.carb_heat);
        // PT6 turboprop start, verified live: two engines, condition lever +
        // ign/start switch commands + per-engine generator.
        // Electrical wired to parity with the other profiles: battery, dual
        // generators (+ both), avionics.
        let elec: Vec<&str> = p.electrical.iter().map(|e| e.key.as_str()).collect();
        for k in [
            "battery",
            "generator1",
            "generator2",
            "generators",
            "avionics",
        ] {
            assert!(elec.contains(&k), "c90b electrical missing {k}");
        }
        assert!(p.flood_lights); // cockpit floods wired (generic_lights_switch[0])
        assert!(p.fuel_pump); // standby boost pumps (verified)
        let ts = p.turboprop_start.as_ref().expect("c90b turboprop start");
        assert_eq!(ts.start_up.len(), 2);
        assert_eq!(ts.start_dn.len(), 2);
        // dn+up per generator (the 3-pos GEN switch only latches ON on the
        // dn→center transition), so 4 commands for the pair.
        assert_eq!(ts.generators.len(), 4);
        assert_eq!(ts.condition_start, 0.5);
        assert_eq!(ts.condition_run, 1.0);
        // Modeled limits from the .acf.
        assert_eq!(p.vne_kts, Some(226.0));
        assert_eq!(p.vs_clean_kts, Some(100.0));
        // Flaps up/approach/down.
        let flaps = p.flaps.expect("has flaps");
        assert_eq!(flaps.detents.len(), 3);
        assert_eq!(flaps.detents[1].label, "approach");
        assert_eq!(flaps.detents[2].label, "down");
        let r = p.reference.as_ref().expect("c90b reference");
        assert_eq!(r.max_gross_lb, Some(9650.0));
        assert_eq!(r.vr_kts, Some(90.0));
    }

    /// Every installed airframe whose nose castors declares brake steering
    /// (the Evolution and the RV-10 did not, 2026-09-25). Needs the install:
    /// `cargo test -- --ignored real_install_castoring_noses_brake_steer`.
    #[test]
    #[ignore]
    fn real_install_castoring_noses_brake_steer() {
        let root = crate::config::xplane_root();
        for (id, dir, acf) in [
            ("lancair_evolution", "Laminar Research/Lancair Evolution", "N844X.acf"),
            ("vans_rv10", "Laminar Research/Van's Aircraft RV-10", "RV-10.acf"),
            ("cirrus_sr22", "Laminar Research/Cirrus SR22", "Cirrus SR22.acf"),
            ("cirrus_vision_sf50", "Laminar Research/Cirrus Vision SF50", "CirrusSF50.acf"),
            ("cessna_172sp", "Laminar Research/Cessna 172 SP", "Cessna_172SP.acf"),
            ("king_air_c90b", "Laminar Research/Beechcraft King Air C90B", "C90B.acf"),
        ] {
            let path = std::path::Path::new(&root).join("Aircraft").join(dir).join(acf);
            let text = String::from_utf8_lossy(&std::fs::read(&path).expect("acf on disk")).to_string();
            let p = load(id).unwrap_or_else(|| panic!("{id} loads"));
            let declared = p.taxi.as_ref().is_some_and(|t| t.brake_steer);
            assert_eq!(nose_castors_from_acf(&text), declared, "{id}");
        }
    }

    #[test]
    fn a_castoring_nose_is_read_from_the_acf() {
        assert!(nose_castors_from_acf("P _gear/0/_gear_castors 1\nP _gear/1/_gear_castors 0"));
        assert!(!nose_castors_from_acf("P _gear/0/_gear_castors 0\nP _gear/1/_gear_castors 1"));
        assert!(!nose_castors_from_acf(""));
        let evo = "P _gear/0/_gear_castors 1\nP _gear/0/_steerdeg_lospeed 0.000000000";
        let sf50 = "P _gear/0/_gear_castors 1\nP _gear/0/_steerdeg_lospeed 40.000000000";
        assert!(nose_free_from_acf(evo));
        assert!(!nose_free_from_acf(sf50));
        assert!(!nose_free_from_acf("P _gear/0/_gear_castors 0\nP _gear/0/_steerdeg_lospeed 0.0"));
    }

    #[test]
    fn lancair_evolution_yaml_loads() {
        // YAML-only profile. Deserialize directly so a malformed field fails here
        // rather than silently falling back.
        let text = std::fs::read_to_string("profiles/lancair_evolution.yaml").unwrap();
        let p: AircraftProfile = serde_yaml::from_str(&text)
            .unwrap_or_else(|e| panic!("lancair_evolution.yaml failed to parse: {e:#}"));
        assert_eq!(p.id, "lancair_evolution");
        // Single turboprop: power lever (not the piston group), retractable gear
        // with the POM's 150 KIAS gear speed, standard autopilot.
        assert!(p.throttle && !p.engine);
        assert!(p.retractable_gear);
        assert_eq!(p.vlo_kts, Some(150.0));
        assert!(p.autopilot && p.ils);
        assert!(!p.turbine_start && !p.magnetos && !p.carb_heat);
        // PT6 start with the press-and-hold starter: one engine, no start_dn
        // (the held button releases itself), igniter armed, boost pump fed.
        let ts = p
            .turboprop_start
            .as_ref()
            .expect("evolution turboprop start");
        assert!(ts.starter_held);
        assert_eq!(ts.start_up.len(), 1);
        assert!(ts.start_dn.is_empty());
        assert!(ts.boost_pumps);
        assert_eq!(ts.ignition_arm.len(), 1);
        assert_eq!(ts.condition_start, 0.5);
        // The real POH: low idle for flight, never high idle (LIME taxi).
        assert_eq!(ts.condition_run, 0.5);
        // Dual battery + generator + standby alternator + DC power. No "both
        // generators" pair (POM: GEN and ALT are not normally used together) and
        // no avionics switch (the xlua forces avionics on at flight start).
        let elec: Vec<&str> = p.electrical.iter().map(|e| e.key.as_str()).collect();
        for k in [
            "battery1",
            "battery2",
            "batteries",
            "generator",
            "alternator",
            "dc_power",
        ] {
            assert!(elec.contains(&k), "evolution electrical missing {k}");
        }
        assert!(!elec.contains(&"avionics") && !elec.contains(&"generators"));
        // After-start brings the GENERATOR + DC POWER online, not the alternator.
        assert_eq!(ts.generators.len(), 2);
        // The AP panel's own power switch is engaged before the servos.
        let prep = p.autopilot_prep.as_ref().expect("evolution autopilot prep");
        assert_eq!(prep.flight_director.len(), 1);
        assert!(!prep.ac_powered);
        // No beacon or taxi light on this airframe (POM) — only nav/strobe/landing.
        let lights: Vec<&str> = p.lights.iter().map(|l| l.key.as_str()).collect();
        assert_eq!(lights, ["nav", "strobe", "landing"]);
        // POM / .acf limits.
        assert_eq!(p.vne_kts, Some(256.0));
        assert_eq!(p.vs_clean_kts, Some(76.0));
        let flaps = p.flaps.expect("has flaps");
        assert_eq!(flaps.detents.len(), 3);
        assert_eq!(flaps.detents[1].vfe_kts, Some(160.0));
        assert_eq!(flaps.detents[2].vfe_kts, Some(140.0));
        // Fuel selector: off/left/right (the POH's engine-failure drills call
        // for OFF; no "both" — the wing tanks don't interconnect).
        let fsel = p.fuel_selector.as_ref().expect("evolution fuel selector");
        let pos: Vec<&str> = fsel.positions.iter().map(|p| p.key.as_str()).collect();
        assert_eq!(pos, ["off", "left", "right"]);
        let r = p.reference.as_ref().expect("evolution reference");
        assert_eq!(r.vr_kts, Some(77.0));
        assert_eq!(r.max_gross_lb, Some(4300.0));
        assert_eq!(r.cruise_kts, Some(300.0));
        let prop = p.prop.as_ref().expect("evolution prop");
        assert_eq!(prop.max_rpm, 2000.0);
    }

    #[test]
    fn dhc6_twin_otter_yaml_loads() {
        // YAML-only profile (RWDesigns add-on). Deserialize directly so a
        // malformed field fails loudly here rather than silently falling back.
        let text = std::fs::read_to_string("profiles/dhc6_twin_otter.yaml").unwrap();
        let p: AircraftProfile = serde_yaml::from_str(&text)
            .unwrap_or_else(|e| panic!("dhc6_twin_otter.yaml failed to parse: {e:#}"));
        assert_eq!(p.id, "dhc6_twin_otter");
        // Twin turboprop: power levers (not the piston group), FIXED gear.
        assert!(p.throttle && !p.engine);
        assert!(!p.retractable_gear);
        assert_eq!(p.engines, 2);
        assert!(!p.magnetos && !p.carb_heat && !p.turbine_start);
        // Stock-driven AP (the panel buttons fire sim/autopilot commands).
        assert!(p.autopilot && p.ils);
        assert!(p.autopilot_prep.is_none());
        // Boost pumps are COMMAND-driven (the universal array is a no-op on
        // this add-on): fuel_pump off, the override wired for both pumps.
        assert!(!p.fuel_pump);
        let pc = p
            .fuel_pump_commands
            .as_ref()
            .expect("twotter pump commands");
        assert_eq!(pc.on_commands.len(), 2);
        assert_eq!(pc.off_commands.len(), 2);
        assert_eq!(pc.readback.len(), 2);
        // Electrical: battery + DC master + generators (each/both) + bleed air.
        let elec: Vec<&str> = p.electrical.iter().map(|e| e.key.as_str()).collect();
        for k in [
            "battery",
            "master",
            "generator1",
            "generator2",
            "generators",
            "bleed",
        ] {
            assert!(elec.contains(&k), "twotter electrical missing {k}");
        }
        // PT6 start with the fire-once starter: no start_dn (the model latches
        // its own 16 s starter run), boost pumps fed per engine via ignition_arm.
        let ts = p.turboprop_start.as_ref().expect("twotter turboprop start");
        assert_eq!(ts.start_up.len(), 2);
        assert!(ts.start_dn.is_empty());
        assert!(!ts.starter_held);
        assert!(!ts.boost_pumps);
        assert_eq!(ts.ignition_arm.len(), 2);
        assert_eq!(ts.generators.len(), 2);
        // Reverse thrust: rollout tool, gated + self-stowing (STOL bushplane —
        // reverse is core to its short landings).
        assert!(p.reverse_thrust);
        // Modeled .acf limits: flaps 0/10/20/30/40, Vfe 105/95, Vne 202, Vs 60.
        let flaps = p.flaps.expect("has flaps");
        assert_eq!(flaps.detents.len(), 5);
        assert_eq!(flaps.detents[0].label, "up");
        assert_eq!(flaps.detents[4].label, "full");
        assert_eq!(flaps.detents[1].vfe_kts, Some(105.0));
        assert_eq!(flaps.detents[4].vfe_kts, Some(95.0));
        assert_eq!(p.vne_kts, Some(202.0));
        assert_eq!(p.vs_clean_kts, Some(60.0));
        let prop = p.prop.as_ref().expect("twotter prop");
        assert_eq!(prop.max_rpm, 2200.0);
        let r = p.reference.as_ref().expect("twotter reference");
        assert_eq!(r.max_gross_lb, Some(12500.0));
        assert_eq!(r.vr_kts, Some(75.0));
        assert_eq!(p.lights.len(), 5);
    }

    #[test]
    fn unknown_profile_is_none() {
        // No YAML, no built-in → None (caller surfaces a clear error).
        assert!(load("does_not_exist").is_none());
    }

    #[test]
    fn sr22_yaml_loads_and_is_conservative() {
        let p = load("cirrus_sr22").expect("SR22 profile loads");
        assert_eq!(p.id, "cirrus_sr22");
        // Universal-dataref tools on (incl. engine — throttle/mixture/start), plus
        // the rustdecks-verified boost pump, key-start ignition and pitch trim.
        assert!(p.autopilot && p.parking_brake && p.pitot_heat && p.transponder && p.engine);
        assert!(p.magnetos && p.fuel_pump && p.pitch_trim);
        // Still off: carb heat (fuel-injected) and the vacuum DG (G1000 glass).
        assert!(!p.carb_heat && !p.directional_gyro);
        // Fuel selector is wired (verified deck commands): OFF / LEFT / RIGHT,
        // no "both".
        let fsel = p.fuel_selector.as_ref().expect("SR22 has a fuel selector");
        let pos: Vec<&str> = fsel.positions.iter().map(|p| p.key.as_str()).collect();
        assert_eq!(pos, ["off", "left", "right"]);
        // Dual electrical: BAT1/BAT2/ALT1/ALT2 + the pair switches + avionics.
        let elec: Vec<&str> = p.electrical.iter().map(|e| e.key.as_str()).collect();
        for k in [
            "battery1",
            "battery2",
            "alternator1",
            "alternator2",
            "batteries",
            "alternators",
            "avionics",
        ] {
            assert!(elec.contains(&k), "SR22 electrical missing {k}");
        }
        // Flaps 0/50/100.
        let flaps = p.flaps.expect("has flaps");
        assert_eq!(flaps.detents.len(), 3);
        assert_eq!(flaps.detents[2].label, "full");
        assert_eq!(flaps.detents[1].vfe_kts, Some(150.0));
        assert_eq!(p.vs_clean_kts, Some(70.0));
        assert_eq!(p.vne_kts, Some(204.0));
        // Reference card loads from YAML — the SR22's own numbers.
        let r = p.reference.as_ref().expect("sr22 reference");
        assert_eq!(r.best_glide_kts, Some(88.0));
        assert_eq!(r.max_gross_lb, Some(3600.0));
        assert_eq!(p.lights.len(), 3);
        // Conservative map: only the universal-safe entries, and the YAML matches
        // the built-in.
        let yaml_map = &p.checklist_map;
        let builtin_map = &cirrus_sr22().checklist_map;
        assert_eq!(yaml_map.len(), builtin_map.len());
        for (a, b) in yaml_map.iter().zip(builtin_map) {
            assert_eq!(a.match_, b.match_);
        }
        // Still call-out, not mapped: the dual battery/alternator (their file
        // conditions are compound `&&`).
        for e in yaml_map {
            assert!(!e.match_.contains("battery") && !e.match_.contains("generator"));
        }
        // The boost pump, key-start ignition, pitch trim and fuel selector ARE
        // now mapped.
        let matches: Vec<&str> = yaml_map.iter().map(|e| e.match_.as_str()).collect();
        for m in [
            "boost_pump",
            "ignition_key",
            "ignition_on",
            "elevator_trim",
            "fuel_tank_selector",
        ] {
            assert!(matches.contains(&m), "SR22 map missing {m}");
        }
        // Fuel-selector positions carry their dataref values so the checklist can
        // decode `fuel_tank_selector:N` (0=off/1=left/2=right).
        let fsel = p.fuel_selector.as_ref().unwrap();
        assert_eq!(
            fsel.readback.as_deref(),
            Some("sim/cockpit2/fuel/fuel_tank_selector")
        );
        assert_eq!(fsel.positions.iter().filter_map(|p| p.value).count(), 3);
    }

    /// The Zibo 737 (2026-10-06): its own profile, never the stock 737's
    /// names. The stock profile's start, cutoff and baro paths are the stock
    /// aircraft's xlua (`condition_lever*_target`, `barometer/capt_setting`,
    /// the scripted `auto_start`), none of which exist on the Zibo (probed
    /// live, parked). Its toggles are each read from their own position.
    #[test]
    fn the_zibo_flies_its_own_names_never_the_stock_737s() {
        let text = std::fs::read_to_string("profiles/zibo_738.yaml").unwrap();
        let p: AircraftProfile = serde_yaml::from_str(&text)
            .unwrap_or_else(|e| panic!("zibo_738.yaml failed to parse: {e:#}"));
        assert_eq!(p.id, "zibo_738");
        let m = p.matcher.as_ref().expect("a match block");
        assert!(m.claims_type("B738"));
        assert!(m.acf.iter().any(|a| a.contains("B737-800X")));
        assert!(
            !p.scripted_auto_start,
            "auto_start is not the Zibo's: X-Plane's generic start under its systems"
        );
        assert_eq!(p.fuel_cutoff_levers.len(), p.engines);
        let refs = profile_references(&p);
        // The matcher is a folder, never a name to probe.
        assert!(
            !refs.iter().any(|r| r.contains("B737-800X")),
            "the match block was probed as a name: {refs:?}"
        );
        for stock_only in [
            "condition_lever",
            "laminar/B738/barometer/capt_setting",
            "sim/operation/auto_start",
        ] {
            assert!(
                !refs.iter().any(|r| r.contains(stock_only)),
                "{stock_only}: the stock 737's, not on the Zibo"
            );
        }
        for lever in &p.fuel_cutoff_levers {
            assert!(refs.contains(lever), "{lever} is not probed");
        }
        // Read before fire: every anti-ice toggle reads its own switch.
        let ice = p.anti_ice.as_ref().expect("its own anti-ice switches");
        for s in &ice.systems {
            assert_eq!(s.on_commands.len(), 1, "{:?}: one toggle per entry", s.kind);
            assert!(s.readback.is_some(), "{:?}: a toggle needs its read", s.kind);
        }
        // The panel: its own buttons, LNAV with its lamp, never CMD B. CMD B
        // is named only as the second channel of the landing the autopilot
        // flies (`autopilot_lands`), pressed by `ap_lands` alone.
        let a = p.autoflight.as_ref().expect("its own MCP buttons");
        assert!(a.lnav_command.is_some() && a.lnav_status.is_some());
        assert!(!serde_yaml::to_string(a).unwrap().contains("cmd_b"));
        let lands = p.autopilot_lands.as_ref().expect("the autopilot lands it");
        let second = [lands.second_cmd.command.as_str(), lands.second_cmd.position.as_str()];
        assert!(refs
            .iter()
            .filter(|r| r.contains("cmd_b"))
            .all(|r| second.contains(&r.as_str())));
        assert!(p.seat_belt_signs.is_some() && p.transponder_mode_knob.is_some());
    }
