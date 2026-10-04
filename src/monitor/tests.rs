    /// Issue #63 (2026-09-26): `quiet` includes `autoland::ACTIVE`, and the
    /// compliance watch rode it, so an uncleared landing under an autoland
    /// was called 26.6 s after the touchdown, at the handoff. The watch now
    /// ticks and speaks through an autoland; the chatter stays quiet; a
    /// recovery and the panel's mute still silence it; a wreck stops it.
    #[test]
    fn the_compliance_watch_runs_through_an_autoland() {
        let g = super::world_gates(false, true, false, false);
        assert!(g.quiet, "chatter stays quiet under an autoland");
        assert!(g.compliance_ticks && g.compliance_speaks, "{g:?}");
        assert!(!g.cabin_quiet);
        // Hand-flown: the same watch, the same voice.
        let h = super::world_gates(false, false, false, false);
        assert!(!h.quiet && h.compliance_ticks && h.compliance_speaks);
        // A recovery or the panel's mute: ticked (edges current), unsaid.
        for g in [
            super::world_gates(true, false, false, false),
            super::world_gates(false, false, true, false),
        ] {
            assert!(g.quiet && g.cabin_quiet && g.compliance_ticks && !g.compliance_speaks);
        }
        // A wreck: nothing ticks, nothing speaks.
        let w = super::world_gates(false, false, false, true);
        assert!(w.quiet && !w.compliance_ticks && !w.compliance_speaks);
    }

    /// The watch's call site sits outside the `if !quiet` world block: a
    /// refactor that moves it back in brings issue #63 back.
    #[test]
    fn the_compliance_tick_is_not_under_the_quiet_gate() {
        let src = crate::test_support::module_source("monitor");
        let tick = src
            .find("crate::station::compliance_tick_facing(")
            .expect("call site");
        let gate = src[..tick]
            .rfind("if gates.compliance_ticks {")
            .expect("its own gate");
        assert!(
            !src[gate..tick].contains("if !quiet"),
            "the compliance watch is back under `quiet`"
        );
        let quiet_block = src.find("            if !quiet {\n").expect("world block");
        assert!(tick < quiet_block, "compliance tick inside the quiet block");
    }

    /// Sortie 1790601822-full_auto (EGKK-EGBB 15, 2026-09-28, #155): the
    /// landing hold armed at +3314.5 on the final autoland was flying, and
    /// the go-around judgement and the release both lived inside the `if
    /// !quiet` world block, which `autoland::ACTIVE` closes: no
    /// `landing_released`, no `go_around_deferred`, no `go_around_judged`
    /// in 158 s, and the tower then called the landing uncleared. Both call
    /// sites sit ahead of that block, under the compliance gate.
    #[test]
    fn the_landing_runway_watch_is_not_under_the_quiet_gate() {
        let all = crate::test_support::module_source("monitor");
        let src = &all[..all.find("\nmod tests;").expect("tests")];
        let quiet_block = src.find("            if !quiet {\n").expect("world block");
        for call in [
            "crate::station::go_around_tick(",
            "crate::station::landing_hold_tick(",
        ] {
            let at: Vec<usize> = src.match_indices(call).map(|(i, _)| i).collect();
            assert_eq!(at.len(), 1, "{call}: one call site");
            assert!(at[0] < quiet_block, "{call} is back under `quiet`");
            let gate = src[..at[0]]
                .rfind("if gates.compliance_ticks\n")
                .expect("the runway watch's gate");
            assert!(!src[gate..at[0]].contains("if !quiet"), "{call}");
        }
    }

    /// 1790441293 (737, LEMG 13): the `state` records' `in` at the four
    /// `yoke_under_ap` interventions, every one the crew's own channel read
    /// back (the takeoff's climb_manual pitch -0.371 at the AP engage, then
    /// the arrest and flare's flare_elev 0.26 / 0.32 / 0.21). Under our own
    /// fresh write they are no hand; with no write of ours the same yoke is.
    #[test]
    fn the_crews_own_flare_is_not_a_pilot_on_the_yoke() {
        use crate::flight::Control;
        let yoke = |yp: f64, yr: f64| super::Inputs {
            yoke_pitch: Some(yp),
            yoke_roll: Some(yr),
            ap_on: Some(1.0),
            steer_yield: Some(0.0),
            pitch_yield: Some(0.0),
            brake_yield: Some(0.0),
            ..Default::default()
        };
        let recorded = [
            (yoke(-0.37, -0.0), Control::Automation("takeoff")),
            (yoke(0.26, -0.11), Control::Automation("autoland")),
            (yoke(0.32, 0.05), Control::Automation("autoland")),
            (yoke(0.21, 0.05), Control::Automation("autoland")),
        ];
        for (i, c) in &recorded {
            assert_eq!(super::interventions_now(i, *c, true, false), [false; 4], "{i:?}");
            assert!(super::interventions_now(i, *c, false, false)[3], "{i:?}");
        }
        // A real hand while our channel is on is the plugin's latch to see.
        let mut grab = yoke(0.3, 0.0);
        grab.pitch_yield = Some(1.0);
        let now = super::interventions_now(&grab, Control::Automation("autoland"), true, false);
        assert!(now[1] && !now[3]);
        // The live stamp: a write just now owns the yoke reading.
        super::note_crew_yoke();
        assert!(super::crew_yoke_fresh());
    }

    #[test]
    fn the_overspeed_warning_never_blames_a_descent_in_a_climb() {
        assert!(super::overspeed_line(-1500.0).contains("descent"));
        assert!(!super::overspeed_line(2600.0).contains("descent"));
        assert!(!super::overspeed_line(0.0).contains("descent"));
        assert!(!super::overspeed_line(f64::NAN).contains("descent"));
    }

    use super::stall_speed_at_flap;

    #[test]
    fn the_flap_stall_is_the_profiles_vso_when_it_has_one() {
        // A330: Vs 175 clean, Vso 125 full flap. Full flap is Vso, half flap
        // halfway, clean is clean; the MCDU's VAPP of 154 sits above the 1.2
        // x 125 caution line, where the old 17% rule (145) had put it inside
        // the WARNING band.
        assert_eq!(stall_speed_at_flap(175.0, Some(125.0), 1.0), 125.0);
        assert_eq!(stall_speed_at_flap(175.0, Some(125.0), 0.5), 150.0);
        assert_eq!(stall_speed_at_flap(175.0, Some(125.0), 0.0), 175.0);
        assert!(154.0 > 1.2 * stall_speed_at_flap(175.0, Some(125.0), 1.0));
        assert!(
            154.0 <= 1.2 * stall_speed_at_flap(175.0, None, 1.0),
            "the old rule flagged it"
        );
        assert!(
            154.0 <= 1.1 * stall_speed_at_flap(175.0, None, 1.0),
            "...as a WARNING"
        );
        // No Vso on file: the trainer's 17% reduction, as before.
        let c172 = stall_speed_at_flap(48.0, None, 1.0);
        assert!((c172 - 48.0 * 0.83).abs() < 1e-9);
        // A nonsense Vso (above clean) is ignored, never a raised stall.
        assert_eq!(stall_speed_at_flap(48.0, Some(60.0), 1.0), c172);
    }

    use super::*;

    #[test]
    fn cabin_copy_does_not_invent_pressurisation() {
        assert!(cabin_message(Some(false), false).contains("unpressurised"));
        assert!(!cabin_message(Some(false), false).contains("Check pressurisation"));
        assert!(cabin_message(Some(true), false).contains("Check pressurisation"));
        assert!(!cabin_message(None, false).contains("pressurisation"));
    }

    #[test]
    fn cabin_episode_repeats_only_for_a_higher_band() {
        let t = Instant::now();
        let mut st = FireState::new();
        st.observe_level(Some(13));
        st.spoke_this_episode = true;
        st.last_spoken = Some(t);
        st.observe_level(Some(13));
        assert!(!st.should_speak(Severity::Caution, t + RENOTIFY_COOLDOWN));
        st.observe_level(Some(14));
        assert!(st.should_speak(Severity::Caution, t + RENOTIFY_COOLDOWN));
        st.spoke_this_episode = true;
        st.observe_level(Some(13));
        st.observe_level(Some(14));
        assert!(!st.should_speak(Severity::Caution, t + RENOTIFY_COOLDOWN));
        // Critical airspeed/sink warnings retain their timed reminder.
        assert!(st.should_speak(Severity::Warning, t + RENOTIFY_COOLDOWN));
    }

    #[test]
    fn avionics_gate_reads_the_right_dataref_and_fails_open() {
        // Airframes that model an avionics master gate the radio on its
        // readback dataref (the avionics/audio bus, not the battery master).
        for id in ["cessna_172sp", "cirrus_sr22", "king_air_c90b"] {
            let p = crate::aircraft::load(id).expect(id);
            let dref = avionics_readback_dref(&p).unwrap_or_else(|| panic!("{id} has avionics"));
            assert!(dref.contains("avionics"), "{id}: {dref}");
        }
        // Airframes with no avionics system → None → fail open (radio stays
        // powered; we never silence a controller we can't verify).
        for id in ["dc3", "dhc6_twin_otter", "lancair_evolution"] {
            let p = crate::aircraft::load(id).expect(id);
            assert_eq!(avionics_readback_dref(&p), None, "{id} must fail open");
        }
    }

    /// Fail open has to be RESTORED on every tick, not just defaulted at boot:
    /// `radio_powered` is process-wide and outlives an airframe switch. A
    /// cold-and-dark 172 publishes false; swapping to an airframe with no
    /// avionics master leaves nothing to publish, and the stale false used to
    /// bin every crew radio call for the rest of the session (Heathrow,
    /// 2026-07-27 — full auto's clearance and taxi requests never left the
    /// aircraft and it blamed the tower).
    #[test]
    fn an_airframe_switch_cannot_latch_a_dead_radio() {
        // The 172, cold and dark: its avionics master reads off.
        let c172 = avionics_readback_dref(&crate::aircraft::load("cessna_172sp").unwrap());
        assert!(!radio_powered_from(c172.as_deref(), Some(0.0)));
        assert!(radio_powered_from(c172.as_deref(), Some(1.0)));
        // A failed read on the SAME airframe fails open rather than holding
        // the last answer — one tick of the same trap.
        assert!(radio_powered_from(c172.as_deref(), None));
        // Swap to the Twin Otter: no avionics master, so nothing to read. The
        // decision must be "powered" on its own, never inherited from the 172
        // that was cold and dark a moment ago.
        let otter = avionics_readback_dref(&crate::aircraft::load("dhc6_twin_otter").unwrap());
        assert_eq!(otter, None);
        assert!(
            radio_powered_from(otter.as_deref(), None),
            "an airframe with no avionics master must fail OPEN, not inherit"
        );
    }

    fn snap(engine_running: bool, alternator_on: bool) -> Snapshot {
        Snapshot {
            on_ground: true,
            groundspeed_kts: 0.0,
            heading_deg: 0.0,
            bank_deg: 0.0,
            ias_kts: 0.0,
            ias_trend_kts_per_s: 0.0,
            flap_ratio: 0.0,
            agl_ft: 0.0,
            vsi_fpm: 0.0,
            vs_fpm: None,
            elevator_trim: 0.0,
            seatbelt_sign: None,
            tug_attached: false,
            pushing_back: false,
            engine_running,
            alternator_on,
            generator_amps_max: 0.0,
            gyro_drift_deg: 0.0,
            endurance_min: None,
            fuel_flow_kg_sec: None,
            fuel_kg: None,
            all_up_kg: None,
            winds_aloft: None,
            oil_pressure_psi: None,
            oil_temp_f: None,
            cht_f: None,
            oat_c: 15.0,
            dewpoint_c: -30.0,
            carb_heat_on: false,
            rpm: 0.0,
            low_vacuum: false,
            battery_amps: 0.0,
            park_brake: 0.0,
            crew_brake: false,
            cabin_altitude_ft: 0.0,
            fuel_selector: None,
            alt_ft: 0.0,
            ap_alt_dial_ft: None,
            ap_servos_on: false,
            gear_down: false,
            advisor_values: vec![],
            lat: 0.0,
            lon: 0.0,
            pos_source: None,
            track_deg: None,
            fps: None,
            dest_dist_nm: None,
            dest_elev_ft: None,
            tod_ring_nm: None,
            dest_route_nm: None,
            follower_tod: None,
            follower_flying: false,
            flight_peak_ft: None,
            baro_inhg: None,
            qnh_pa: None,
            automation_flying: false,
            fuel_tanks: None,
            alt_true_ft: None,
            icing: None,
            phase: Phase::Ground,
        }
    }

    /// The LINTO altimeter of 2026-09-14: 30.15 set, QNH 30.10, an
    /// automation flying indicated altitude 46 ft wrong.
    /// EGKK-EGBB, 2026-09-28 (sortie 1790601822-full_auto, #156), the state
    /// records' (t, altitude, vsi): the climb off the 6,000 ft level at LAM
    /// through EGKK's 6,000 ft transition altitude on QNH 30.13...
    #[rustfmt::skip]
    const EGKK_CLIMB: &[(f64, f64, f64)] = &[
        (1646.2, 6072.0, 20.0), (1647.8, 6073.0, 75.0), (1649.4, 6076.0, 109.0), (1650.8, 6079.0, 110.0),
        (1652.2, 6081.0, 94.0), (1653.2, 6084.0, 279.0), (1654.6, 6091.0, 598.0), (1656.1, 6112.0, 1211.0),
        (1657.8, 6149.0, 1798.0), (1659.3, 6210.0, 2235.0), (1660.9, 6279.0, 2414.0), (1662.4, 6350.0, 2458.0),
        (1664.0, 6421.0, 2448.0), (1665.5, 6491.0, 2425.0), (1666.6, 6555.0, 2413.0), (1668.0, 6609.0, 2414.0),
        (1669.3, 6672.0, 2395.0), (1670.8, 6730.0, 2375.0), (1672.2, 6796.0, 2363.0), (1673.6, 6859.0, 2353.0),
        (1674.9, 6918.0, 2377.0), (1676.1, 6974.0, 2428.0), (1677.5, 7028.0, 2440.0), (1679.0, 7092.0, 2387.0),
        (1680.3, 7152.0, 2317.0), (1681.7, 7207.0, 2272.0), (1683.1, 7261.0, 2234.0), (1684.6, 7314.0, 2195.0),
        (1686.2, 7371.0, 2150.0), (1687.6, 7435.0, 2112.0), (1689.0, 7488.0, 2096.0), (1690.4, 7544.0, 2094.0),
        (1691.6, 7595.0, 2089.0), (1692.9, 7636.0, 2092.0), (1694.0, 7685.0, 2114.0), (1695.4, 7730.0, 2140.0),
        (1696.9, 7783.0, 2177.0), (1698.2, 7837.0, 2196.0), (1699.6, 7886.0, 2207.0), (1701.0, 7940.0, 2206.0),
        (1702.4, 7993.0, 2204.0), (1703.7, 8049.0, 2206.0), (1705.0, 8100.0, 2191.0), (1706.5, 8155.0, 2141.0),
        (1707.7, 8210.0, 2064.0), (1709.1, 8251.0, 2021.0), (1710.4, 8297.0, 1987.0),
    ];
    /// ...and the descent on standard through EGBB's FL75 transition level
    /// to GROVE's FL070 and on down the join (QNH 30.15).
    #[rustfmt::skip]
    const EGBB_DESCENT: &[(f64, f64, f64)] = &[
        (2827.2, 7764.0, -800.0), (2828.9, 7752.0, -134.0), (2830.6, 7755.0, 323.0), (2832.2, 7769.0, 534.0),
        (2833.6, 7781.0, 398.0), (2835.0, 7784.0, -151.0), (2836.8, 7774.0, -824.0), (2838.7, 7734.0, -1699.0),
        (2840.5, 7673.0, -2177.0), (2842.3, 7606.0, -2246.0), (2844.0, 7543.0, -2120.0), (2845.7, 7489.0, -1950.0),
        (2847.4, 7434.0, -1800.0), (2849.3, 7384.0, -1702.0), (2851.1, 7331.0, -1688.0), (2852.8, 7282.0, -1852.0),
        (2854.5, 7226.0, -2051.0), (2856.4, 7167.0, -1992.0), (2858.0, 7110.0, -1685.0), (2859.6, 7070.0, -1287.0),
        (2861.6, 7042.0, -875.0), (2863.3, 7022.0, -464.0), (2865.3, 7013.0, -102.0), (2867.2, 7008.0, -366.0),
        (2869.1, 6984.0, -1140.0), (2871.0, 6937.0, -1787.0), (2872.8, 6875.0, -2110.0), (2874.7, 6813.0, -2111.0),
        (2876.6, 6748.0, -1915.0), (2878.3, 6697.0, -1581.0), (2880.2, 6656.0, -1276.0), (2882.2, 6619.0, -1037.0),
        (2884.1, 6589.0, -911.0), (2886.1, 6561.0, -826.0), (2888.1, 6535.0, -777.0), (2890.1, 6511.0, -718.0),
        (2892.3, 6488.0, -638.0), (2894.2, 6467.0, -544.0), (2896.3, 6451.0, -466.0), (2898.0, 6436.0, -403.0),
        (2899.9, 6425.0, -446.0), (2902.0, 6410.0, -425.0), (2904.2, 6396.0, -482.0), (2906.4, 6367.0, -1039.0),
        (2908.5, 6317.0, -1696.0), (2910.5, 6254.0, -2057.0), (2912.7, 6180.0, -2112.0),
    ];

    /// The first time `wanted` has held for `sustain_s` over the recorded
    /// ticks, and the setting it wanted: the monitor's sustain, replayed.
    fn replay_altimeter(
        ticks: &[(f64, f64, f64)],
        baro: f64,
        qnh_inhg: f64,
        sustain_s: f64,
        wanted: impl Fn(&Snapshot) -> Option<(f64, f64)>,
    ) -> Option<(f64, f64)> {
        let mut s = linto_altimeter(true);
        s.baro_inhg = Some(baro);
        s.qnh_pa = Some(qnh_inhg * crate::tools::instruments::PA_PER_INHG);
        let mut since: Option<f64> = None;
        for &(t, alt, vsi) in ticks {
            s.alt_ft = alt;
            s.vsi_fpm = vsi;
            match wanted(&s) {
                Some((_, want)) => {
                    let t0 = *since.get_or_insert(t);
                    if t - t0 >= sustain_s {
                        return Some((t, want));
                    }
                }
                None => since = None,
            }
        }
        None
    }

    /// #156: standard went in 38 s after the climb passed EGKK's 6,000 ft
    /// transition altitude (at 8,300 ft), QNH more than a minute under
    /// EGBB's FL75 transition level on standard (at 6,180 ft). Replayed: the
    /// old drift rule alone is late on both; the crossing check sets each AT
    /// its transition, and the drift check stays silent through both.
    #[test]
    fn the_altimeter_is_set_at_the_transition_not_after_it() {
        let sustain = ALTIMETER_CROSSING_SUSTAIN_MS as f64 / 1000.0;
        // The rule as flown: the band, then 20 s.
        let old = |ta: f64| {
            move |s: &Snapshot| {
                let (baro, pa) = (s.baro_inhg?, s.qnh_pa?);
                let want = crate::tools::instruments::correct_setting_inhg(s.alt_ft, ta, pa);
                let qnh = pa / crate::tools::instruments::PA_PER_INHG;
                if in_transition_band(s.alt_ft, ta)
                    && ((baro - qnh).abs() < 0.03 || (baro - 29.92).abs() < 0.03)
                {
                    return None;
                }
                ((baro - want).abs() >= 0.03).then_some((baro, want))
            }
        };
        let (t_old, _) = replay_altimeter(EGKK_CLIMB, 30.13, 30.13, 20.0, old(6_000.0)).unwrap();
        assert!(t_old > 1_685.0, "as flown: late, {t_old}");
        let (t, want) =
            replay_altimeter(EGKK_CLIMB, 30.13, 30.13, sustain, |s| altimeter_wanted(s, 6_000.0, true))
                .expect("standard owed climbing through 6,000");
        assert_eq!(want, crate::tools::instruments::STANDARD_INHG);
        assert!(t <= 1_657.8, "standard at {t}, the climb began at 1653");
        assert_eq!(
            replay_altimeter(EGKK_CLIMB, 30.13, 30.13, 20.0, |s| altimeter_wanted(s, 6_000.0, false)),
            None,
            "the drift check leaves the crossing to its own check"
        );
        // The descent: FL75 on standard.
        let (t_old, _) = replay_altimeter(EGBB_DESCENT, 29.92, 30.15, 20.0, old(7_500.0)).unwrap();
        assert!(t_old > 2_890.0, "as flown: late, {t_old}");
        let (t, want) =
            replay_altimeter(EGBB_DESCENT, 29.92, 30.15, sustain, |s| altimeter_wanted(s, 7_500.0, true))
                .expect("QNH owed descending through FL75");
        assert!((want - 30.15).abs() < 0.005, "{want}");
        assert!(t <= 2_849.3, "QNH at {t}, FL75 passed at 2845.7");
        // Set, the knob's own jump never undoes it: QNH read 220 ft higher
        // just under FL75, standard 200 ft lower just over 6,000.
        let mut s = linto_altimeter(true);
        s.qnh_pa = Some(30.15 * crate::tools::instruments::PA_PER_INHG);
        (s.baro_inhg, s.alt_ft, s.vsi_fpm) = (Some(30.15), 7_710.0, -1_800.0);
        assert_eq!(altimeter_wanted(&s, 7_500.0, true), None);
        assert_eq!(altimeter_wanted(&s, 7_500.0, false), None);
        s.qnh_pa = Some(30.50 * crate::tools::instruments::PA_PER_INHG);
        (s.baro_inhg, s.alt_ft, s.vsi_fpm) = (Some(29.92), 5_440.0, 2_000.0);
        assert_eq!(altimeter_wanted(&s, 6_000.0, true), None);
        assert_eq!(altimeter_wanted(&s, 6_000.0, false), None, "a high QNH's jump");
        // Level at the transition is no crossing: either setting stands.
        (s.baro_inhg, s.alt_ft, s.vsi_fpm) = (Some(30.50), 6_080.0, 40.0);
        assert_eq!(altimeter_wanted(&s, 6_000.0, true), None);
    }

    fn linto_altimeter(automation_flying: bool) -> Snapshot {
        let mut s = snap(true, true);
        s.on_ground = false;
        s.alt_ft = 6_838.0;
        s.baro_inhg = Some(30.15);
        s.qnh_pa = Some(101_946.0);
        s.automation_flying = automation_flying;
        s
    }

    /// The stock 737 at the stand as probed live on 2026-09-27: engines
    /// running, centre (tank 1) empty, every pump switch on.
    fn b738_dry_centre(automation_flying: bool) -> Snapshot {
        let mut s = snap(true, true);
        s.on_ground = true;
        s.automation_flying = automation_flying;
        s.fuel_tanks = Some(crate::tools::fuel_pumps::Tanks {
            kg: vec![2916.8, 0.0, 2918.1],
            pumps: vec![1.0, 1.0, 1.0],
            lateral_m: vec![-7.01, 0.0, 7.01],
        });
        s
    }

    /// #94, EGSS 2026-09-22: 4,675 ft indicated at 16,884 true with the
    /// knob at 17.50. That is the KNOB (the setting check says it and fixes
    /// it); the instrument check stays silent. A frozen static port with the
    /// knob right is the instrument's and is said.
    #[test]
    fn the_altimeter_is_cross_checked_against_the_true_height() {
        let c = AltimeterTruthCheck {
            transition_altitude_ft: 6_000.0,
        };
        let mut egss = snap(true, true);
        egss.on_ground = false;
        egss.alt_ft = 4_675.0;
        egss.alt_true_ft = Some(16_884.0);
        egss.baro_inhg = Some(17.50);
        egss.qnh_pa = Some(30.40 * crate::tools::instruments::PA_PER_INHG);
        egss.oat_c = -18.0;
        let t = altimeter_truth_of(&egss, 6_000.0).expect("all read");
        assert!(t.setting_gross(), "{t:?}");
        assert!(c.eval(&egss).is_none(), "the knob is the setting check's");
        let mut frozen = egss;
        frozen.alt_ft = 5_000.0;
        frozen.alt_true_ft = Some(9_000.0);
        frozen.baro_inhg = Some(30.10);
        frozen.qnh_pa = Some(30.10 * crate::tools::instruments::PA_PER_INHG);
        frozen.oat_c = -5.0;
        let line = c.eval(&frozen).expect("a static fault is said");
        assert!(line.contains("about 4200 feet higher"), "{line}");
        assert!(c.min_duration() >= Duration::from_secs(20));
        // No truth read: no measurement, no word.
        frozen.alt_true_ft = None;
        assert!(c.eval(&frozen).is_none());
        // On the ground the static is the field's.
        frozen.alt_true_ft = Some(9_000.0);
        frozen.on_ground = true;
        assert!(c.eval(&frozen).is_none());
    }

    /// #100: icing conditions with an automation flying switch the systems
    /// through the sentinel; a pilot flying is advised and nothing written;
    /// warm air or no read is silence.
    #[test]
    fn icing_is_switched_by_the_crew_or_advised_to_the_pilot() {
        use crate::aircraft::IceKind;
        let _g = crate::mode::TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut s = snap(true, true);
        s.on_ground = false;
        s.oat_c = -4.0;
        s.dewpoint_c = -5.0;
        s.icing = Some(crate::tools::anti_ice::IcingRead {
            oat_c: -4.0,
            dewpoint_c: -5.0,
            switches: vec![(IceKind::Engine, Some(false)), (IceKind::Pitot, Some(true))],
            ..Default::default()
        });
        s.automation_flying = true;
        assert_eq!(
            Icing.eval(&s).as_deref(),
            Some("Icing conditions, engine anti-ice on.")
        );
        assert_eq!(
            Icing.fix_write(&s),
            Some((crate::tools::anti_ice::MONITOR_FIX, 1.0))
        );
        s.automation_flying = false;
        assert_eq!(
            Icing.eval(&s).as_deref(),
            Some("Icing conditions, recommend engine anti-ice on.")
        );
        assert!(Icing.fix_write(&s).is_none());
        s.icing.as_mut().unwrap().switches = vec![(IceKind::Engine, Some(true))];
        assert!(Icing.eval(&s).is_none(), "already on");
        s.icing = None;
        assert!(Icing.eval(&s).is_none());
    }

    #[test]
    fn an_automation_flying_switches_the_dry_centre_pumps_off() {
        let c = CentreTankDry { declared: vec![1] };
        let s = b738_dry_centre(true);
        assert_eq!(c.eval(&s).as_deref(), Some("Centre tank's empty, centre pumps off."));
        assert_eq!(
            c.fix_write(&s),
            Some((crate::tools::fuel_pumps::TANK_PUMP_ON, 0.0))
        );
        assert!(c.min_duration() >= Duration::from_secs(3));
    }

    #[test]
    fn a_pilot_flying_is_advised_and_nothing_is_written() {
        let c = CentreTankDry { declared: vec![1] };
        let s = b738_dry_centre(false);
        assert_eq!(
            c.eval(&s).as_deref(),
            Some("Centre tank's empty, centre pumps are still on.")
        );
        assert!(c.fix_write(&s).is_none());
    }

    #[test]
    fn the_centre_check_is_silent_on_a_failed_read_or_with_fuel_aboard() {
        let c = CentreTankDry { declared: vec![1] };
        let mut s = b738_dry_centre(true);
        s.fuel_tanks = None;
        assert!(c.eval(&s).is_none() && c.fix_write(&s).is_none());
        let mut s = b738_dry_centre(true);
        s.fuel_tanks.as_mut().unwrap().kg[1] = 4_000.0;
        assert!(c.eval(&s).is_none() && c.fix_write(&s).is_none());
        let mut s = b738_dry_centre(true);
        s.engine_running = false;
        assert!(c.eval(&s).is_none());
    }

    /// The overspeed checks' verdicts for a profile at an IAS and altitude:
    /// (caution, warning) fired.
    fn overspeed_at(p: &crate::aircraft::AircraftProfile, ias: f64, alt_ft: f64) -> (bool, bool) {
        let mut s = snap(true, true);
        s.on_ground = false;
        s.ias_kts = ias;
        s.ias_trend_kts_per_s = 0.0;
        s.alt_ft = alt_ft;
        let checks = build_checks(p);
        let fired = |id: &str| checks.iter().any(|c| c.id() == id && c.eval(&s).is_some());
        (fired("overspeed_caution"), fired("overspeed_warning"))
    }

    #[test]
    fn the_citation_x_overspeed_is_judged_at_its_altitude() {
        let p = crate::aircraft::load("citation_x").expect("citation_x profile");
        // Low: Vmo 270, the caution from 243.
        assert_eq!(overspeed_at(&p, 250.0, 5_000.0), (true, false));
        assert_eq!(overspeed_at(&p, 275.0, 5_000.0), (false, true));
        // At FL200 the limit is 350 (the Mmo's airspeed there is 442): 300
        // is a cruise, not an overspeed; the caution from 315.
        assert_eq!(overspeed_at(&p, 300.0, 20_000.0), (false, false));
        assert_eq!(overspeed_at(&p, 320.0, 20_000.0), (true, false));
        assert_eq!(overspeed_at(&p, 355.0, 20_000.0), (false, true));
        // At FL430 the Mmo is the limit: M0.935 is about 268 KIAS there.
        assert_eq!(overspeed_at(&p, 275.0, 43_000.0), (false, true));
        // An altitude not read is the low (conservative) limit.
        assert_eq!(overspeed_at(&p, 250.0, f64::NAN), (true, false));
    }

    #[test]
    fn an_overspeed_without_a_high_vmo_is_one_vne_at_every_altitude() {
        let p = crate::aircraft::load("boeing_737_800").expect("737 profile");
        assert!(p.vmo_high.is_none());
        let vne = p.vne_kts.expect("737 Vne");
        for alt in [2_000.0, 20_000.0, 37_000.0] {
            assert_eq!(overspeed_at(&p, vne * 0.95, alt), (true, false), "{alt}");
            assert_eq!(overspeed_at(&p, vne + 1.0, alt), (false, true), "{alt}");
            assert_eq!(overspeed_at(&p, vne * 0.85, alt), (false, false), "{alt}");
        }
    }

    #[test]
    fn only_a_profile_that_declares_tanks_gets_the_centre_check() {
        let b738 = crate::aircraft::load("boeing_737_800").expect("737 profile");
        assert_eq!(b738.fuel_tank_pumps_off_when_empty, vec![1]);
        assert!(build_checks(&b738).iter().any(|c| c.id() == CENTRE_TANK_DRY));
        let c172 = crate::aircraft::load("cessna_172sp").expect("172 profile");
        assert!(!build_checks(&c172).iter().any(|c| c.id() == CENTRE_TANK_DRY));
        // The swap probe covers the names the declaration relies on.
        let names = crate::aircraft::profile_references(&b738);
        for n in [
            crate::tools::fuel_pumps::TANK_PUMP_ON,
            crate::tools::fuel_pumps::TANK_FUEL_KG,
            crate::tools::fuel_pumps::TANK_LATERAL_M,
        ] {
            assert!(names.iter().any(|x| x == n), "{n} not probed");
        }
    }

    /// The monitor's fix end to end on the recorded numbers: the decision
    /// names tank 1, the one writer switches element 1 to 0 and reads it back.
    #[tokio::test]
    async fn the_fix_switches_element_one_off_and_reads_it_back() {
        use crate::tools::fuel_pumps as fp;
        let s = b738_dry_centre(true);
        let tanks = fp::pumps_to_switch_off(&[1], s.fuel_tanks.as_ref(), s.engine_running);
        let xp = sim::xplane::FakeXPlane::new();
        xp.set_read_array(fp::TANK_FUEL_KG, &[2916.8, 0.0, 2918.1]);
        xp.set_read_array(fp::TANK_PUMP_ON, &[1.0, 1.0, 1.0]);
        xp.apply_array_writes(fp::TANK_PUMP_ON);
        let done = fp::switch_pumps(&xp, &tanks, false).await;
        assert_eq!(xp.index_writes(fp::TANK_PUMP_ON), vec![(1, 0.0)]);
        assert_eq!(done.len(), 1);
        assert_eq!((done[0].tank, done[0].kg, done[0].before, done[0].after), (1, Some(0.0), Some(1.0), Some(0.0)));
        assert!(fp::all_took(&done) && fp::failed_line(&done).is_none());
    }

    /// The 737 mid-Atlantic on EIDW-BIKF (2026-09-20): FL340, 62.9 tonnes,
    /// level, an optimum of 38,600 four and a half thousand feet overhead,
    /// and not one word said about it for the whole crossing.
    fn mid_atlantic_737(weight_kg: f64, alt_ft: f64) -> Snapshot {
        let mut s = snap(true, true);
        s.on_ground = false;
        s.phase = Phase::Cruise;
        s.vsi_fpm = 40.0;
        s.alt_ft = alt_ft;
        s.all_up_kg = Some(weight_kg);
        s
    }

    fn jet_reference() -> crate::aircraft::ReferenceProfile {
        let mut r = crate::aircraft::ReferenceProfile::default();
        r.optimum_alt_ft = Some(38_000.0);
        r.optimum_alt_at_kg = Some(65_000.0);
        r.optimum_alt_ft_per_1000kg = Some(290.0);
        r
    }

    #[test]
    fn the_step_climb_is_offered_once_the_weight_has_earned_it() {
        let c = StepClimb {
            reference: jet_reference(),
        };
        let said = c
            .eval(&mid_atlantic_737(62_935.0, 34_000.0))
            .expect("4,600 ft below optimum is worth a word");
        assert!(said.contains("below optimum"), "{said}");
        assert!(
            said.contains("three six zero"),
            "one step, not four: {said}"
        );
        assert!(said.contains("4,600"), "it names the gap it found: {said}");
        // With no wind profile there is simply no wind clause, never a guess.
        assert!(!said.contains("wind"), "nothing invented: {said}");
        // Heavy off the blocks: optimum is BELOW us, nothing to offer.
        assert_eq!(c.eval(&mid_atlantic_737(79_000.0, 34_000.0)), None);
        // Already at optimum: silent.
        assert_eq!(c.eval(&mid_atlantic_737(62_935.0, 38_000.0)), None);
        // Inside one step of it: still silent, because there is no step to ask
        // for that would not overshoot.
        assert_eq!(c.eval(&mid_atlantic_737(62_935.0, 37_000.0)), None);
    }

    #[test]
    fn the_step_climb_never_speaks_outside_level_cruise() {
        let c = StepClimb {
            reference: jet_reference(),
        };
        let climbing = Snapshot {
            vsi_fpm: 1800.0,
            ..mid_atlantic_737(62_935.0, 34_000.0)
        };
        assert_eq!(c.eval(&climbing), None, "already going somewhere");
        let approaching = Snapshot {
            phase: Phase::Approach,
            ..mid_atlantic_737(62_935.0, 34_000.0)
        };
        assert_eq!(c.eval(&approaching), None, "not a cruise problem any more");
        // No weight read is no optimum: it says nothing rather than guessing.
        let blind = Snapshot {
            all_up_kg: None,
            ..mid_atlantic_737(62_935.0, 34_000.0)
        };
        assert_eq!(c.eval(&blind), None);
        // A cruise that wanders under the autopilot is still a cruise: at the
        // old 300 fpm gate every wobble ended the episode and re-armed it,
        // and the same advice went out three times in six minutes.
        let wobbling = Snapshot {
            vsi_fpm: 352.0,
            ..mid_atlantic_737(62_935.0, 34_000.0)
        };
        assert!(
            c.eval(&wobbling).is_some(),
            "a 352 fpm wobble is level cruise, not a climb"
        );
        // The plan is taking us DOWN shortly: a climb is arguing with the
        // flight plan the copilot is itself flying.
        let going_down = Snapshot {
            follower_tod: Some(crate::procedure::PlannedTod {
                fix: "PETAX".into(),
                dist_nm: 40.0,
                to_tod_nm: 12.0,
                target_ft: 20_200.0,
                started: false,
                held_ft: None,
            }),
            ..mid_atlantic_737(62_935.0, 34_000.0)
        };
        assert_eq!(
            c.eval(&going_down),
            None,
            "the plan's own descent outranks the money"
        );
        // And it never actuates: a level belongs to ATC.
        assert_eq!(
            c.fix_write(&mid_atlantic_737(62_935.0, 34_000.0)),
            None,
            "the copilot asks, it does not climb"
        );
    }

    /// EGBB-EGKK (sortie 1790534902, 2026-09-27, +1405 s): a 737 at 63,976 kg
    /// holding the plan's 15,000 ft, 68 nm from Gatwick at 303 kt with the
    /// follower flying, heard "23,300 feet below optimum ... flight level one
    /// seven zero would burn less". Three gates, each alone enough.
    #[test]
    fn no_step_climb_on_a_short_hop_or_over_the_plans_level() {
        let c = StepClimb {
            reference: jet_reference(),
        };
        let egbb_egkk = || {
            let mut s = mid_atlantic_737(63_976.0, 14_997.0);
            s.groundspeed_kts = 303.0;
            s.dest_dist_nm = Some(68.0);
            s.dest_elev_ft = Some(202.0);
            s
        };
        // The optimum the line quoted is the profile's own at that weight.
        let optimum = crate::aircraft::optimum_altitude_ft(&jet_reference(), 63_976.0).unwrap();
        assert_eq!(((optimum - 14_997.0) / 100.0).round() * 100.0, 23_300.0);
        // 1: a gap of eleven levels is a level PLANNED low, not a drift one
        // step answers: the gap and the level asked for no longer disagree
        // because the pair is never said.
        assert_eq!(crate::aircraft::step_climb_ft(14_997.0, optimum), None);
        assert_eq!(c.eval(&egbb_egkk()), None, "planned low, not drifted");
        // 2: a follower flying the plan's level is never argued with, even on
        // a leg with no top of descent published and a gap one step answers.
        let mut following = mid_atlantic_737(62_935.0, 34_000.0);
        following.follower_flying = true;
        assert_eq!(following.follower_tod, None);
        assert_eq!(c.eval(&following), None, "the plan's level is the plan's");
        // 3: ten minutes of cruise left does not pay for a climb, whoever flies.
        let mut short = mid_atlantic_737(62_935.0, 34_000.0);
        short.groundspeed_kts = 450.0;
        short.dest_dist_nm = Some(180.0);
        short.dest_elev_ft = Some(202.0);
        short.tod_ring_nm = Some(105.0);
        assert!(step_cruise_left_min(&short).unwrap() < STEP_CLIMB_MIN_CRUISE_MIN);
        assert_eq!(c.eval(&short), None, "10 minutes to the top of descent");
        // The same drift with two hours of cruise ahead is still worth a word.
        short.dest_dist_nm = Some(1_000.0);
        assert!(c.eval(&short).is_some_and(|l| l.contains("three six zero")));
    }

    /// EGBB-EGKK (sortie 1790534902, 2026-09-27): +1684 Benson, +1864
    /// Heathrow, +2045 Farnborough, +2226 Gatwick, each "Closest field is X,
    /// tower is on ...", none an answer (the one question, "when do we
    /// land?" at +1677, was answered at +1678 by the ETA).
    #[test]
    fn the_closest_field_is_not_volunteered_on_a_normal_flight() {
        use crate::persona::Chatter;
        for c in [Chatter::Quiet, Chatter::Normal] {
            assert!(!region_note_said(false, false, c), "{c:?}: normal flight, airborne");
        }
        assert!(region_note_said(false, false, Chatter::Chatty), "a chatty copilot volunteers");
        for c in [Chatter::Quiet, Chatter::Normal, Chatter::Chatty] {
            assert!(region_note_said(false, true, c), "{c:?}: with a failure it is the question");
            assert!(region_note_said(true, false, c), "{c:?}: the ground relocation note");
        }
    }

    /// The climb is only worth taking if the wind up there allows it, so the
    /// advisory says which it is. EIDW-BIKF's jet stream was flat across the
    /// band — the fact that made the climb free.
    #[test]
    fn the_step_climb_says_what_the_wind_does_up_there() {
        let c = StepClimb {
            reference: jet_reference(),
        };
        let with = |w: crate::weather::WindsAloft| {
            let mut s = mid_atlantic_737(62_935.0, 34_000.0);
            s.winds_aloft = Some(w);
            c.eval(&s).expect("still below optimum")
        };
        // Flat core: FL340 and FL360 within a knot or two of each other.
        let flat = crate::weather::WindsAloft::from_layers(
            &[9160.2, 10362.9, 11887.2],
            &[50.88, 61.16, 57.40],
            &[280.9, 278.9, 284.9],
        )
        .expect("layers");
        let said = with(flat);
        assert!(said.contains("much the same"), "{said}");
        // A profile that strengthens sharply with height must say so, because
        // then the step is NOT free and the pilot should weigh it.
        let shear = crate::weather::WindsAloft::from_layers(
            &[9000.0, 12000.0],
            &[20.0, 80.0],
            &[280.0, 280.0],
        )
        .expect("layers");
        let said = with(shear);
        assert!(said.contains("windier"), "{said}");
    }

    /// A piston declares no optimum, so the check is never even built for it.
    #[test]
    fn a_piston_is_never_given_a_step_climb_check() {
        let c172 = crate::aircraft::builtin("cessna_172sp").expect("the 172");
        let r = c172
            .reference
            .clone()
            .expect("the 172 has reference figures");
        assert_eq!(r.optimum_alt_ft, None);
        assert!(
            !build_checks(&c172).iter().any(|c| c.id() == "step_climb"),
            "no optimum declared, no advisory built"
        );
    }

    #[test]
    fn altimeter_check_advises_the_pilot_and_resets_for_an_automation() {
        let c = AltimeterSetting {
            transition_altitude_ft: 18_000.0,
            crossing: false,
        };
        let pilot = linto_altimeter(false);
        let said = c.eval(&pilot).expect("mis-set altimeter is a callout");
        assert!(
            said.starts_with("Altimeter check, QNH is three zero one zero"),
            "{said}"
        );
        assert!(said.contains("you have three zero one five"), "{said}");
        assert!(
            c.fix_write(&pilot).is_none(),
            "the pilot's knob is never turned"
        );

        let auto = linto_altimeter(true);
        assert_eq!(
            c.eval(&auto).as_deref(),
            Some("Altimeter reset, three zero one zero.")
        );
        let (dref, v) = c
            .fix_write(&auto)
            .expect("an automation's altimeter is set");
        assert_eq!(dref, BARO_KNOB);
        assert!((v - 30.10).abs() < 0.005, "{v}");
    }

    #[test]
    fn altimeter_check_is_quiet_when_set_or_cold() {
        let c = AltimeterSetting {
            transition_altitude_ft: 18_000.0,
            crossing: false,
        };
        let mut s = linto_altimeter(false);
        s.baro_inhg = Some(30.11); // a hundredth off: within tolerance
        assert!(c.eval(&s).is_none());
        s.baro_inhg = Some(30.15);
        s.engine_running = false; // cold and dark: nobody's flying yet
        assert!(c.eval(&s).is_none());
        s.engine_running = true;
        s.qnh_pa = None; // no pressure from the sim: nothing to compare
        assert!(c.eval(&s).is_none());
    }

    #[test]
    fn altimeter_check_wants_standard_above_the_transition_altitude() {
        let c = AltimeterSetting {
            transition_altitude_ft: 6_000.0,
            crossing: false,
        };
        let mut s = linto_altimeter(false);
        s.baro_inhg = Some(30.10); // QNH set, but we are in the flight levels
        let said = c
            .eval(&s)
            .expect("QNH above the transition altitude is wrong");
        assert!(
            said.starts_with("Altimeter check, standard is two niner niner two"),
            "{said}"
        );
    }

    /// EDLP-LOWI (2026-09-24): hovering at FL180 on an 18,000 ft transition
    /// the knob was reset QNH, standard, QNH. Inside the band either setting
    /// stands; a knob on neither is still corrected.
    #[test]
    fn altimeter_check_leaves_either_setting_alone_near_the_transition() {
        let c = AltimeterSetting {
            transition_altitude_ft: 18_000.0,
            crossing: false,
        };
        let mut s = linto_altimeter(false);
        let qnh = s.qnh_pa.unwrap() / crate::tools::instruments::PA_PER_INHG;
        for alt in [17_700.0, 18_000.0, 18_300.0] {
            s.alt_ft = alt;
            s.baro_inhg = Some(qnh);
            assert!(c.eval(&s).is_none(), "QNH at {alt}");
            s.baro_inhg = Some(29.92);
            assert!(c.eval(&s).is_none(), "standard at {alt}");
        }
        s.alt_ft = 18_000.0;
        s.baro_inhg = Some(28.50);
        assert!(c.eval(&s).is_some(), "a knob on neither is still wrong");
        s.alt_ft = 20_000.0;
        s.baro_inhg = Some(qnh);
        assert!(c.eval(&s).is_some(), "clear of the band, QNH is wrong");
    }

    /// Airborne at cruise with a known destination — the descent-advisor base.
    /// Roughly today's LEAL–LEMD picture: 14,000 ft, Madrid at ~2,000 ft,
    /// planned TOD ring 42 nm out.
    fn cruise_with_dest(dist_nm: f64) -> Snapshot {
        let mut s = snap(true, true);
        s.on_ground = false;
        s.alt_ft = 14_000.0;
        s.agl_ft = 12_000.0;
        s.phase = Phase::Cruise;
        s.vsi_fpm = 0.0;
        s.groundspeed_kts = 300.0;
        s.ias_kts = 240.0;
        s.dest_dist_nm = Some(dist_nm);
        s.dest_elev_ft = Some(2_000.0);
        s.tod_ring_nm = Some(42.0);
        s
    }

    #[test]
    fn tod_ahead_speaks_in_the_runup_band() {
        let c = TodAhead;
        assert!(c.eval(&cruise_with_dest(80.0)).is_none(), "far out → quiet");
        let msg = c
            .eval(&cruise_with_dest(50.0))
            .expect("8 nm before TOD → advise");
        assert!(msg.contains("Top of descent"), "{msg}");
        assert!(
            c.eval(&cruise_with_dest(40.0)).is_none(),
            "past TOD → this check yields"
        );
        // No destination known → silent.
        let mut s = cruise_with_dest(50.0);
        s.dest_dist_nm = None;
        assert!(c.eval(&s).is_none());
    }

    #[test]
    fn tod_passed_names_the_recovery_rate() {
        let c = TodPassed;
        assert!(
            c.eval(&cruise_with_dest(45.0)).is_none(),
            "before TOD → quiet"
        );
        // Today's miss: level at 14,000, 38 nm out, TOD at 42. Height to lose
        // 10,000 ft over (38-5)/300kt = 6.6 min → ~1,500 fpm.
        let msg = c
            .eval(&cruise_with_dest(38.0))
            .expect("past TOD and level → caution");
        assert!(msg.contains("1500 feet a minute"), "{msg}");
        // Already descending → moot, quiet.
        let mut s = cruise_with_dest(38.0);
        s.vsi_fpm = -2000.0;
        assert!(c.eval(&s).is_none());
    }

    /// LEJR-LXGB, 2026-09-14: level at 7,000 with the SimBrief ring long
    /// passed, but the follower pacing its own descent to 27D10 at 3,143 ft.
    /// The advisor speaks the follower's point and altitude, and "past the
    /// descent point" is only ever the follower's own miss.
    #[test]
    fn follower_tod_outranks_the_plan_ring() {
        let tod = |to_tod_nm: f64, started: bool| crate::procedure::PlannedTod {
            fix: "27D10".into(),
            dist_nm: to_tod_nm + 11.2,
            to_tod_nm,
            target_ft: 3143.0,
            started,
            held_ft: None,
        };
        // The ring says "past" (38 nm out, ring 42); the follower says 8 to go.
        let mut s = cruise_with_dest(38.0);
        s.follower_tod = Some(tod(8.0, false));
        assert!(
            TodPassed.eval(&s).is_none(),
            "the hands are correctly waiting"
        );
        let msg = TodAhead.eval(&s).expect("the follower's point, 8 nm ahead");
        assert!(msg.contains("8 miles ahead"), "{msg}");
        assert!(msg.contains("down to 3,100 feet"), "{msg}");
        // At the cruise it is said; one step down, the next step is not
        // (LIBP-LEMG 2026-09-26: seven calls down one arrival).
        s.flight_peak_ft = Some(s.alt_ft + 200.0);
        assert!(TodAhead.eval(&s).is_some(), "still at the cruise");
        s.flight_peak_ft = Some(s.alt_ft + 5000.0);
        assert!(TodAhead.eval(&s).is_none(), "a step of a descent already begun");
        // The ring says "coming up" (50 nm out); the follower is 20 nm away.
        let mut s = cruise_with_dest(50.0);
        s.follower_tod = Some(tod(20.0, false));
        assert!(TodAhead.eval(&s).is_none(), "far from the follower's point");
        // Two miles past its own point and still level: the follower missed.
        let mut s = cruise_with_dest(38.0);
        s.follower_tod = Some(tod(-2.0, false));
        let msg = TodPassed.eval(&s).expect("the follower's own miss");
        assert!(msg.contains("feet a minute makes 3,100 feet"), "{msg}");
        // Descent commanded: both quiet, whatever the ring thinks.
        let mut s = cruise_with_dest(38.0);
        s.follower_tod = Some(tod(0.0, true));
        assert!(TodAhead.eval(&s).is_none());
        assert!(TodPassed.eval(&s).is_none());
        // Mid-descent on the follower's slope: the ring's slope never judges it.
        let mut s = cruise_with_dest(30.0);
        s.vsi_fpm = -700.0;
        s.alt_ft = 13_000.0;
        assert!(HighOnProfile.eval(&s).is_some(), "the ring's rule alone");
        s.follower_tod = Some(tod(-1.0, true));
        assert!(
            HighOnProfile.eval(&s).is_none(),
            "the follower owns its profile"
        );
    }

    /// LIME-LIPE, 2026-09-25: "climb to 19,000" held the follower level over
    /// the plan's descents. One reminder that names who holds the level, and
    /// no "past the descent point" Caution about a level the pilot chose.
    /// And no top of descent is ever called in a climb.
    #[test]
    fn a_pilot_held_level_is_no_missed_descent_and_a_climb_has_no_tod() {
        let held = |to_tod_nm: f64| crate::procedure::PlannedTod {
            fix: "IDREK".into(),
            dist_nm: to_tod_nm + 3.0,
            to_tod_nm,
            target_ft: 18_000.0,
            started: false,
            held_ft: Some(19_000.0),
        };
        let mut s = cruise_with_dest(38.0);
        s.follower_tod = Some(held(5.0));
        let msg = TodAhead.eval(&s).expect("the plan's descent, reminded");
        assert!(msg.contains("19,000 held") && msg.contains("IDREK"), "{msg}");
        s.follower_tod = Some(held(-3.0));
        assert!(TodPassed.eval(&s).is_none(), "the pilot chose the level");
        // Climbing: neither, ring or follower.
        let mut s = cruise_with_dest(38.0);
        s.vsi_fpm = 700.0;
        assert!(TodPassed.eval(&s).is_none(), "a climb is not 'still level'");
        let mut s = cruise_with_dest(50.0);
        s.vsi_fpm = 450.0;
        assert!(TodAhead.eval(&s).is_none(), "no top of descent in a climb");
    }

    /// EGLF-LFPB, 2026-09-22: between the legs that HAVE a descent the
    /// follower publishes no point at all, and the ring filled the silence.
    /// Four calls on one arrival, from two geometries: the follower's "about
    /// a minute, 2 miles, down to 23,000", then the ring's "12 miles ahead",
    /// then the ring's "we're past the descent point" as a CAUTION, then the
    /// follower's "7 miles ahead, down to 20,200". Nothing was wrong: the
    /// descent was being flown leg by leg the whole time.
    ///
    /// While a follower flies, the ring is not a fallback. It is silence.
    #[test]
    fn the_ring_never_fills_the_follower_s_silence() {
        // The ring's own geometry says all three of these things.
        let mut s = cruise_with_dest(38.0);
        assert!(TodPassed.eval(&s).is_some(), "the ring alone still speaks");
        let ahead = cruise_with_dest(50.0);
        assert!(TodAhead.eval(&ahead).is_some(), "and so does its run-up");
        // Now a follower is flying, on a leg with nothing to descend to yet.
        s.follower_flying = true;
        assert_eq!(s.follower_tod, None, "no point to publish on this leg");
        assert_eq!(
            TodPassed.eval(&s),
            None,
            "the ring must not call a correctly flown plan late"
        );
        let mut ahead = ahead;
        ahead.follower_flying = true;
        assert_eq!(TodAhead.eval(&ahead), None, "nor announce its own point");
        // ...and the high-on-profile advisor keeps the same rule.
        let mut high = cruise_with_dest(30.0);
        high.vsi_fpm = -700.0;
        high.alt_ft = 13_000.0;
        assert!(HighOnProfile.eval(&high).is_some(), "the ring's rule alone");
        high.follower_flying = true;
        assert_eq!(HighOnProfile.eval(&high), None, "the follower owns it");
        // With no follower flying, nothing above has changed.
        let plain = cruise_with_dest(38.0);
        assert!(TodPassed.eval(&plain).is_some(), "the ring is still there");
    }

    /// LEMG-LXGB, S-76, iMac 2026-09-29 (#191): the helicopter route flew its
    /// own approach down to the gate while the ring said "High on the
    /// profile" and "past the descent point, about 1600 feet a minute". The
    /// route owns the descent as the follower does: the same silence.
    #[test]
    fn a_helicopter_route_owns_its_descent_too() {
        assert!(!follower_flying(false, false));
        assert!(follower_flying(true, false));
        assert!(follower_flying(false, true));
        let mut s = cruise_with_dest(38.0);
        let mut high = cruise_with_dest(30.0);
        high.vsi_fpm = -700.0;
        high.alt_ft = 13_000.0;
        assert!(TodPassed.eval(&s).is_some() && HighOnProfile.eval(&high).is_some());
        s.follower_flying = follower_flying(false, true);
        high.follower_flying = follower_flying(false, true);
        assert_eq!(TodPassed.eval(&s), None, "the route's descent is not late");
        assert_eq!(HighOnProfile.eval(&high), None, "nor high");
    }

    /// LSZS-LILO, SR22, 2026-09-24 (issue #68): "We're past the descent
    /// point. About 300 feet a minute makes the profile." at t 1790265228.8,
    /// 51 ft AGL, five seconds after liftoff from Samedan (5,600 ft). The
    /// plan's ring (SimBrief's along-route TOD down a winding valley route)
    /// was longer than the 23.6 nm straight line to Sondrio (898 ft): the
    /// recorded 300 fpm puts the ring past 24.6 nm, so the aircraft was
    /// "past the descent point" on the runway. The recorded ticks replayed
    /// through the flight model: the climb-out is silent, and so is a level
    /// held low (139 ft, the resume refusal's height, long enough for the
    /// model to call it a level); a real cruise still calls.
    #[test]
    fn a_climb_out_under_a_ring_behind_the_departure_calls_no_top_of_descent() {
        use crate::flight::{Flight, Sample};
        type Row = (f64, bool, f64, f64, f64, f64, f64, f64, f64);
        const LILO: (f64, f64, f64) = (46.1546, 9.7952, 898.0);
        const RING_NM: f64 = 25.0;
        // (t, ground, gs, ias, vsi, agl, alt, lat, lon), the state records.
        #[rustfmt::skip]
        let recorded: &[Row] = &[
            (1790265186.1, true, 12.8, 9.4, -8.1, 0.0, 5617.9, 46.52844, 9.87915),
            (1790265191.4, true, 32.4, 26.8, -16.1, 0.0, 5616.7, 46.52895, 9.87953),
            (1790265194.8, true, 44.2, 42.5, -26.1, 0.1, 5615.5, 46.52950, 9.87998),
            (1790265198.3, true, 56.6, 55.6, -22.2, 0.0, 5614.4, 46.53018, 9.88062),
            (1790265201.7, true, 68.1, 65.0, -37.5, 0.0, 5612.6, 46.53106, 9.88131),
            (1790265204.9, true, 78.8, 70.6, -49.0, 0.1, 5610.4, 46.53214, 9.88192),
            (1790265208.2, true, 83.6, 80.8, -68.4, -0.0, 5607.6, 46.53325, 9.88263),
            (1790265211.5, true, 88.4, 82.2, -59.1, 0.1, 5604.3, 46.53436, 9.88368),
            (1790265215.0, true, 95.2, 91.5, -43.8, 0.1, 5602.0, 46.53553, 9.88487),
            (1790265218.4, true, 102.6, 99.1, -11.4, 0.1, 5600.3, 46.53694, 9.88614),
            (1790265221.8, true, 110.3, 107.4, 7.6, 0.1, 5600.1, 46.53857, 9.88747),
            (1790265223.3, false, 113.1, 107.4, 82.0, 2.6, 5600.7, 46.53936, 9.88808),
            (1790265225.0, false, 114.6, 108.0, 372.7, 12.8, 5606.0, 46.54005, 9.88862),
            (1790265226.9, false, 115.9, 111.1, 536.0, 29.2, 5620.2, 46.54087, 9.88931),
            (1790265228.7, false, 116.4, 109.9, 730.4, 51.3, 5638.5, 46.54171, 9.89004),
            (1790265230.6, false, 117.2, 110.6, 724.3, 74.0, 5661.4, 46.54257, 9.89080),
            (1790265232.3, false, 118.1, 110.2, 689.3, 94.6, 5681.6, 46.54339, 9.89159),
            (1790265233.7, false, 118.9, 106.4, 665.3, 114.5, 5700.6, 46.54421, 9.89237),
            (1790265235.7, false, 119.6, 109.7, 990.4, 139.2, 5720.1, 46.54491, 9.89306),
        ];
        let mut flight = Flight::new();
        let mut tick = |r: Row| {
            let (t, ground, gs, ias, vsi, agl, alt, lat, lon) = r;
            flight.tick(&Sample {
                t,
                on_ground: ground,
                gs_kts: gs,
                ias_kts: ias,
                vsi_fpm: vsi,
                agl_ft: agl,
                alt_ft: alt,
                engine_running: true,
                lat: Some(lat),
                lon: Some(lon),
                ..Sample::default()
            });
            let mut s = snap(true, true);
            s.on_ground = ground;
            s.groundspeed_kts = gs;
            s.ias_kts = ias;
            s.vsi_fpm = vsi;
            s.agl_ft = agl;
            s.alt_ft = alt;
            s.lat = lat;
            s.lon = lon;
            s.dest_dist_nm = Some(crate::nav::haversine_nm(lat, lon, LILO.0, LILO.1));
            s.dest_elev_ft = Some(LILO.2);
            s.tod_ring_nm = Some(RING_NM);
            s.phase = coarse_phase(flight.phase(), agl);
            s
        };
        for &r in recorded {
            let s = tick(r);
            assert_eq!(TodPassed.eval(&s), None, "t {}: {:?}", r.0, s.phase);
            assert_eq!(TodAhead.eval(&s), None, "t {}", r.0);
        }
        // Level for an instant at 139 ft (the vertical-speed gate alone let
        // this speak): the ring is behind us, the old rule's numbers hold
        // (23.6 nm out, 116 kt, "about 300 feet a minute").
        #[rustfmt::skip]
        let row: Row = (1790265237.0, false, 119.6, 109.7, 0.0, 139.2, 5720.1, 46.54491, 9.89306);
        let mut level = tick(row);
        assert!((level.dest_dist_nm.unwrap() - 23.6).abs() < 0.3);
        assert_eq!(level.phase, Phase::Climb, "the initial climb, by the model");
        assert_eq!(TodPassed.eval(&level), None, "no top of descent in the climb-out");
        // Held level at 139 ft for 40 s: the model calls it "levelled low"
        // (a cruise), and the height still keeps the family silent.
        for i in 1..=20 {
            let t = row.0 + 2.0 * i as f64;
            level = tick((t, false, 120.0, 110.0, 0.0, 139.2, 5720.1, 46.5450, 9.8931));
        }
        assert_eq!(level.phase, Phase::Cruise, "levelled low");
        assert_eq!(TodPassed.eval(&level), None, "139 ft AGL is no cruise to leave");
        // A real cruise: climb to 10,400 ft (4,800 ft over the valley),
        // level for 40 s, 22 nm out under the 25 nm ring: now it speaks.
        let mut t = 1790265300.0;
        let mut alt: f64 = 5720.0;
        while alt < 10_400.0 {
            alt = (alt + 700.0 / 30.0).min(10_400.0);
            t += 2.0;
            tick((t, false, 120.0, 110.0, 700.0, alt - 5_581.0, alt, 46.50, 9.87));
        }
        let mut cruise = level;
        for _ in 0..20 {
            t += 2.0;
            cruise = tick((t, false, 120.0, 110.0, 0.0, 4_819.0, 10_400.0, 46.50, 9.87));
        }
        assert_eq!(cruise.phase, Phase::Cruise);
        assert!(cruise.dest_dist_nm.unwrap() < RING_NM - 1.0);
        let msg = TodPassed.eval(&cruise).expect("past the ring in a real cruise");
        assert!(msg.contains("feet a minute makes the profile"), "{msg}");
    }

    /// The LSZS-LILO plan's points (SimBrief, 2026-09-24; the same route
    /// `heli_route` `start` logged on 1790279339.9): 63.0 nm of valley legs
    /// to a field 23.1 nm away in a straight line.
    const LSZS_LILO: &[(f64, f64)] = &[
        (46.5344, 9.8841),
        (46.6, 9.0 + 59.0 / 60.0),
        (46.7, 10.1),
        (46.65, 10.0 + 11.0 / 60.0),
        (46.0 + 34.0 / 60.0, 10.0 + 10.0 / 60.0),
        (46.0 + 26.0 / 60.0, 10.05),
        (46.0 + 25.0 / 60.0, 10.05),
        (46.4, 10.0 + 2.0 / 60.0),
        (46.35, 10.05),
        (46.0 + 17.0 / 60.0, 10.1),
        (46.0 + 13.0 / 60.0, 10.15),
        (46.0 + 10.0 / 60.0, 10.1),
        (46.0 + 10.0 / 60.0, 9.95),
        (46.1546, 9.7952),
    ];

    /// The route's remaining miles: the whole 63 nm from the origin, the
    /// straight line only on the last straight legs, and never growing as
    /// the aircraft flies the legs.
    #[test]
    fn the_remaining_route_is_measured_along_the_legs() {
        let r = LSZS_LILO;
        let dest = r[r.len() - 1];
        let at_origin = route_remaining_nm(r, r[0].0, r[0].1).unwrap();
        assert!((at_origin - 63.0).abs() < 0.3, "{at_origin}");
        let straight = crate::nav::haversine_nm(r[0].0, r[0].1, dest.0, dest.1);
        assert!((straight - 23.1).abs() < 0.3, "{straight}");
        // Flown leg by leg, ten points a leg: never more to go than before.
        let mut last = f64::INFINITY;
        for w in r.windows(2) {
            for k in 0..10 {
                let f = k as f64 / 10.0;
                let (lat, lon) = (w[0].0 + f * (w[1].0 - w[0].0), w[0].1 + f * (w[1].1 - w[0].1));
                let rem = route_remaining_nm(r, lat, lon).unwrap();
                assert!(rem <= last + 0.05, "{lat},{lon}: {rem} after {last}");
                last = rem;
            }
        }
        // The last leg is the straight line.
        let (lat, lon) = r[r.len() - 2];
        let rem = route_remaining_nm(r, lat, lon).unwrap();
        assert!((rem - crate::nav::haversine_nm(lat, lon, dest.0, dest.1)).abs() < 0.01);
        // No route, no measurement: the straight line stands in.
        assert_eq!(route_remaining_nm(&r[..1], 46.5, 9.9), None);
        assert_eq!(route_remaining_nm(&[], 46.5, 9.9), None);
    }

    /// #109: the plan's ring (SimBrief's along-route TOD) against the
    /// straight line to the field called LSZS-LILO's top of descent at
    /// every point of the route's northern loop, and "past" long before it.
    /// Measured along the legs, it speaks only where the ring really is.
    #[test]
    fn the_along_route_ring_is_compared_with_the_route_still_to_fly() {
        const RING_NM: f64 = 25.0;
        let r = LSZS_LILO;
        let dest = r[r.len() - 1];
        let cruise_at = |i: usize, route: bool| {
            let mut s = snap(true, true);
            s.on_ground = false;
            s.phase = Phase::Cruise;
            s.alt_ft = 10_400.0;
            s.agl_ft = 4_800.0;
            s.vsi_fpm = 0.0;
            s.groundspeed_kts = 150.0;
            s.ias_kts = 140.0;
            (s.lat, s.lon) = r[i];
            s.dest_dist_nm = Some(crate::nav::haversine_nm(r[i].0, r[i].1, dest.0, dest.1));
            s.dest_elev_ft = Some(898.0);
            s.tod_ring_nm = Some(RING_NM);
            s.dest_route_nm = if route { route_remaining_nm(r, r[i].0, r[i].1) } else { None };
            s
        };
        // The old comparison, the straight line: "coming up" 2.9 nm out at
        // the first valley fix, 57 nm of route before the real point...
        let old = cruise_at(1, false);
        assert!(TodAhead.eval(&old).is_some(), "the straight line calls it");
        // ...and "past the descent point" 31 nm of route out, at fix 5.
        let old = cruise_at(5, false);
        assert!(TodPassed.eval(&old).is_some(), "the straight line calls it past");
        // Along the legs: nothing on the northern loop...
        for i in 1..=4 {
            let s = cruise_at(i, true);
            assert_eq!(TodAhead.eval(&s), None, "fix {i}");
            assert_eq!(TodPassed.eval(&s), None, "fix {i}");
        }
        // ...the run-up where the ring is (30.7 nm of route at fix 5)...
        let s = cruise_at(5, true);
        assert_eq!(TodPassed.eval(&s), None, "not past it yet");
        let msg = TodAhead.eval(&s).expect("the ring 5.7 nm ahead along the route");
        assert!(msg.contains("6 miles ahead"), "{msg}");
        // ...and past it once the route to fly is inside the ring (fix 9).
        let s = cruise_at(9, true);
        assert!(TodPassed.eval(&s).is_some(), "20.9 nm of route under a 25 nm ring");
    }

    /// #109 on a real SimBrief plan (EGKK-EGBB, the stock 737's OFP, bench
    /// copy): the ring the monitor gets (`plan::tod_ring_nm`) and the route
    /// it measures along (the plan's waypoints, as `plan.rs` publishes
    /// them). The navlog's `tod_to_dest_nm` is 61 nm, the arrival as flown;
    /// the legs from its TOD are 42, so against them it called the descent
    /// 19 nm early. Measured from the TOD's position the ring is reached at
    /// the OFP's own TOD; the straight line to EGBB is inside it 24 nm of
    /// route before that, where the old comparison called the descent point
    /// "passed". Flown through the checks at FL180 cruise.
    #[test]
    fn a_real_plans_ring_is_reached_at_its_own_top_of_descent() {
        let v: serde_json::Value =
            serde_json::from_str(include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/ofp/egkk-egbb-738.trimmed.json"))).unwrap();
        let fp = crate::simbrief::plan_from_ofp(&v).unwrap();
        let route: Vec<(f64, f64)> = fp.waypoints.iter().map(|w| (w.lat, w.lon)).collect();
        assert!((fp.tod_to_dest_nm - 61.0).abs() < 0.5, "{}", fp.tod_to_dest_nm);
        let ring = crate::plan::tod_ring_nm(&fp, &route);
        assert!((ring - 42.2).abs() < 0.5, "{ring}");
        let fixes = v.pointer("/navlog/fix").and_then(|f| f.as_array()).unwrap();
        let at = |id: &str| {
            let f = fixes.iter().find(|f| f["ident"] == id).unwrap();
            let n = |k: &str| f[k].as_str().unwrap().parse::<f64>().unwrap();
            (n("pos_lat"), n("pos_long"))
        };
        let (tod, dest) = (at("TOD"), *route.last().unwrap());
        // At the OFP's TOD the route still to fly IS the ring.
        let rem = route_remaining_nm(&route, tod.0, tod.1).unwrap();
        let straight = crate::nav::haversine_nm(tod.0, tod.1, dest.0, dest.1);
        assert!((rem - ring).abs() < 2.0, "route {rem:.1} vs ring {ring}");
        assert!(straight < ring - 20.0, "the straight line is {straight:.1}");
        // Flown from the origin in quarter miles: where each comparison
        // first says "inside the ring", as route miles before the TOD.
        let total = crate::route_finder::path_nm(&route);
        let rem_tod = route_remaining_nm(&route, tod.0, tod.1).unwrap();
        let (mut along_first, mut straight_first) = (None, None);
        let mut s = 0.0;
        while s <= total {
            let p = crate::route_finder::point_along(&route, s);
            let r = route_remaining_nm(&route, p.0, p.1).unwrap();
            if along_first.is_none() && r <= ring {
                along_first = Some(r - rem_tod);
            }
            let d = crate::nav::haversine_nm(p.0, p.1, dest.0, dest.1);
            if straight_first.is_none() && d <= ring {
                straight_first = Some(r - rem_tod);
            }
            s += 0.25;
        }
        let (along_first, straight_first) = (along_first.unwrap(), straight_first.unwrap());
        assert!(along_first.abs() < 2.0, "along the legs, {along_first:.1} nm from the TOD");
        assert!(straight_first > 20.0, "straight line, {straight_first:.1} nm early");
        // Through the checks, 5 nm of route before the TOD and 5 after.
        let cruise_at = |before_nm: f64, route_known: bool| {
            let p = crate::route_finder::point_along(&route, total - rem_tod - before_nm);
            let mut s = snap(true, true);
            s.on_ground = false;
            s.phase = Phase::Cruise;
            s.alt_ft = 18_000.0;
            s.agl_ft = 17_500.0;
            s.vsi_fpm = 0.0;
            s.groundspeed_kts = 300.0;
            s.ias_kts = 250.0;
            (s.lat, s.lon) = p;
            s.dest_dist_nm = Some(crate::nav::haversine_nm(p.0, p.1, dest.0, dest.1));
            s.dest_elev_ft = Some(339.0);
            s.tod_ring_nm = Some(ring);
            s.dest_route_nm = if route_known { route_remaining_nm(&route, p.0, p.1) } else { None };
            s
        };
        let before = cruise_at(5.0, true);
        assert_eq!(TodPassed.eval(&before), None, "5 nm before the TOD");
        let msg = TodAhead.eval(&before).expect("the run-up to the TOD");
        assert!(msg.contains("ahead"), "{msg}");
        assert!(TodPassed.eval(&cruise_at(-5.0, true)).is_some(), "5 nm past it");
        // The straight line, 20 nm of route before the TOD: "passed".
        assert!(TodPassed.eval(&cruise_at(20.0, false)).is_some(), "the old comparison");
        assert_eq!(TodPassed.eval(&cruise_at(20.0, true)), None);
        assert_eq!(TodAhead.eval(&cruise_at(20.0, true)), None);
    }

    /// #127: EGKK-EGBB (sortie 1790541304-full_auto +1604) said "Top of
    /// descent in about a minute, 1 miles ahead" at 300 kt. A mile is said
    /// "a mile", and the time is the distance at this groundspeed.
    #[test]
    fn the_top_of_descent_says_a_mile_and_a_time_that_agrees() {
        let mut s = cruise_with_dest(38.0);
        s.groundspeed_kts = 300.0;
        s.follower_tod = Some(crate::procedure::PlannedTod {
            fix: "BUZAD".into(),
            dist_nm: 20.0,
            to_tod_nm: 1.0,
            target_ft: 18_000.0,
            started: false,
            held_ft: None,
        });
        let msg = TodAhead.eval(&s).expect("the follower's point, a mile ahead");
        assert_eq!(
            msg,
            "Descent coming up. Top of descent in under a minute, a mile ahead, down to 18,000 feet."
        );
        assert_eq!(tod_ahead_words(0.3, 300.0), "in under a minute, under a mile ahead");
        assert_eq!(tod_ahead_words(4.0, 300.0), "in about a minute, 4 miles ahead");
        assert_eq!(tod_ahead_words(12.0, 480.0), "in about 2 minutes, 12 miles ahead");
        assert_eq!(tod_ahead_words(1.0, 0.0), "a mile ahead");
        assert_eq!(spoken_miles(8.4), "8 miles");
        // The two halves never disagree by more than the rounding: over the
        // whole run-up band at every groundspeed a descent is flown at.
        for gs in (120..=520).step_by(20) {
            for tenth in 0..=120 {
                let nm = tenth as f64 / 10.0;
                let words = tod_ahead_words(nm, gs as f64);
                assert!(!words.contains(", 1 miles") && !words.contains('\u{2014}'), "{words}");
                let secs = nm / gs as f64 * 3600.0;
                if words.starts_with("in under a minute") {
                    assert!(secs < 45.0, "{words} at {gs} kt is {secs:.0} s");
                } else {
                    let n: f64 = if words.starts_with("in about a minute") {
                        1.0
                    } else {
                        words["in about ".len()..].split(' ').next().unwrap().parse().unwrap()
                    };
                    assert!((secs - n * 60.0).abs() <= 30.0, "{words} at {gs} kt is {secs:.0} s");
                }
            }
        }
    }

    /// The descent advisor never judges a takeoff, a climb or a flare, and
    /// never within the initial climb height of the ground.
    #[test]
    fn high_on_profile_is_a_descent_from_the_cruise() {
        let mut s = cruise_with_dest(30.0);
        s.vsi_fpm = -700.0;
        s.alt_ft = 13_000.0;
        assert!(HighOnProfile.eval(&s).is_some(), "mid-descent, high");
        for p in [Phase::Ground, Phase::Takeoff, Phase::Climb, Phase::Landing] {
            s.phase = p;
            assert_eq!(HighOnProfile.eval(&s), None, "{p:?}");
        }
        s.phase = Phase::Approach;
        assert!(HighOnProfile.eval(&s).is_some(), "a descent on the arrival");
        s.agl_ft = 900.0;
        assert_eq!(HighOnProfile.eval(&s), None, "too near the ground");
    }

    #[test]
    fn high_on_profile_advises_mid_descent() {
        let c = HighOnProfile;
        // On the slope: 30 nm out, profile ≈ 2000+2000+22*300 = 10,600.
        let mut s = cruise_with_dest(30.0);
        s.vsi_fpm = -1800.0;
        s.alt_ft = 10_600.0;
        assert!(c.eval(&s).is_none(), "on profile → quiet");
        // 3,400 ft above it, fast → boards call.
        s.alt_ft = 14_000.0;
        s.ias_kts = 258.0;
        let msg = c.eval(&s).expect("well above the slope");
        assert!(msg.contains("Boards"), "{msg}");
        // Level flight is TodPassed's problem, not this check's.
        s.vsi_fpm = 0.0;
        assert!(c.eval(&s).is_none());
    }

    #[test]
    fn phase_expectation_advises_on_the_delta() {
        let c = PhaseExpectation {
            id: "advisor_test",
            index: 0,
            expect_on: true,
            phases: vec![Phase::Climb, Phase::Cruise],
            say: "Yaw damper's still off.".into(),
            command: None,
        };
        let mut s = snap(true, true);
        // Wrong phase → silent even when the switch is off.
        s.phase = Phase::Ground;
        s.advisor_values = vec![Some(0.0)];
        assert!(c.eval(&s).is_none());
        // Right phase, switch off → advise.
        s.phase = Phase::Cruise;
        assert_eq!(c.eval(&s).as_deref(), Some("Yaw damper's still off."));
        // Switch on → satisfied.
        s.advisor_values = vec![Some(1.0)];
        assert!(c.eval(&s).is_none());
        // Failed read → silent, never advise off stale air.
        s.advisor_values = vec![None];
        assert!(c.eval(&s).is_none());
    }

    /// LIPK stand (2026-09-25): "Taxi light on." six seconds after the
    /// engine was shut down. A ground row wanting a switch ON waits for an
    /// engine.
    #[test]
    fn a_ground_row_is_silent_with_the_engine_off() {
        let c = PhaseExpectation {
            id: "advisor_test",
            index: 0,
            expect_on: true,
            phases: vec![Phase::Ground],
            say: "Taxi light on.".into(),
            command: Some("sim/lights/taxi_lights_on".into()),
        };
        let mut s = snap(false, false);
        s.phase = Phase::Ground;
        s.advisor_values = vec![Some(0.0)];
        assert!(c.eval(&s).is_none(), "parked, engine off");
        let mut s = snap(true, true);
        s.phase = Phase::Ground;
        s.advisor_values = vec![Some(0.0)];
        assert_eq!(c.eval(&s).as_deref(), Some("Taxi light on."));
    }

    #[test]
    fn advisor_rows_build_from_the_profile() {
        let p = crate::aircraft::load("md82").expect("md82 profile");
        assert!(!p.advisor.is_empty(), "md82 declares an SOP table");
        let checks = build_checks(&p);
        let advisors = checks
            .iter()
            .filter(|c| c.id().starts_with("advisor_"))
            .count();
        assert_eq!(advisors, p.advisor.len());
    }

    #[test]
    fn sink_rate_scales_with_height() {
        let c = SinkRate;
        let mut s = snap(true, true);
        s.on_ground = false;
        // Normal 700-fpm final at 400 AGL: silent.
        s.agl_ft = 400.0;
        s.vsi_fpm = -700.0;
        assert!(c.eval(&s).is_none());
        // The LEAL-LEMD dive: ~1,960 fpm at 187 AGL → fires.
        s.agl_ft = 187.0;
        s.vsi_fpm = -1958.0;
        assert!(c.eval(&s).is_some());
        // Same rate up high is a normal descent.
        s.agl_ft = 3000.0;
        assert!(c.eval(&s).is_none());
        s.on_ground = true;
        s.agl_ft = 0.0;
        assert!(c.eval(&s).is_none(), "on the ground → silent");
    }

    #[test]
    fn altitude_bust_fires_through_the_dial_only() {
        let c = AltitudeBust;
        let mut s = snap(true, true);
        s.on_ground = false;
        s.ap_servos_on = true;
        s.ap_alt_dial_ft = Some(5000.0);
        // Descending toward the dial from above: normal, silent.
        s.alt_ft = 8000.0;
        s.vsi_fpm = -2000.0;
        assert!(c.eval(&s).is_none());
        // Blown through it: fires.
        s.alt_ft = 4500.0;
        assert!(c.eval(&s).is_some());
        // Landing configuration (gear down = dial holds the missed-approach
        // altitude while descending below it): silent.
        s.gear_down = true;
        assert!(c.eval(&s).is_none());
        s.gear_down = false;
        // Hand-flying (servos off): the AP isn't going to capture anything.
        s.ap_servos_on = false;
        assert!(c.eval(&s).is_none());
        s.ap_servos_on = true;
        // Climbing through the dial fires too.
        s.alt_ft = 5600.0;
        s.vsi_fpm = 1500.0;
        assert!(c.eval(&s).is_some());
    }

    #[test]
    fn fuel_selector_off_gating() {
        let c = FuelSelectorOff;
        let mut s = snap(true, true);
        assert!(c.eval(&s).is_none(), "no reading → silent");
        s.fuel_selector = Some(0.0);
        assert!(c.eval(&s).is_some(), "selector OFF, engine running → warn");
        s.fuel_selector = Some(1.0);
        assert!(c.eval(&s).is_none(), "tank selected → silent");
        s.fuel_selector = Some(0.0);
        s.engine_running = false;
        assert!(c.eval(&s).is_none(), "engine off (parked) → silent");
    }

    #[test]
    fn not_charging_trusts_a_producing_generator() {
        let c = AlternatorNotCharging { noun: "Generator" };
        let mut s = snap(true, true);
        s.battery_amps = -7.0;
        assert!(c.eval(&s).is_some(), "discharge, nothing producing → warn");
        // MD-82 case: standing battery-bus discharge while both generators charge.
        s.generator_amps_max = 39.0;
        assert!(
            c.eval(&s).is_none(),
            "a producing generator overrules the ammeter"
        );
        s.generator_amps_max = 0.0;
        s.engine_running = false;
        assert!(c.eval(&s).is_none(), "engine off → silent");
    }

    #[test]
    fn engine_health_gating_and_disjoint_bands() {
        let z = Duration::ZERO;
        // Oil pressure (PSI): warn below the 20 PSI redline; idle lows (e.g. 40)
        // are NOT alarmed (that idle nuisance was the original false positive).
        let low = EngineHealth {
            id: "l",
            severity: Severity::Warning,
            param: EngineParam::OilPressure,
            low_side: true,
            trip: 20.0,
            clear: None,
            min_dur: z,
        };

        let mut s = snap(true, true);
        assert!(low.eval(&s).is_none(), "no reading → silent");
        s.oil_pressure_psi = Some(40.0);
        assert!(low.eval(&s).is_none(), "idle-low 40 PSI is fine, no alarm");
        s.oil_pressure_psi = Some(15.0);
        assert!(low.eval(&s).is_some(), "below 20 PSI redline → warn");
        s.engine_running = false;
        assert!(low.eval(&s).is_none(), "engine off suppresses");

        // Oil-temp bands are disjoint (°F thresholds): caution 240–245, warn >245.
        let caution = EngineHealth {
            id: "c",
            severity: Severity::Caution,
            param: EngineParam::OilTemp,
            low_side: false,
            trip: 240.0,
            clear: Some(245.0),
            min_dur: z,
        };
        let warn = EngineHealth {
            id: "w",
            severity: Severity::Warning,
            param: EngineParam::OilTemp,
            low_side: false,
            trip: 245.0,
            clear: None,
            min_dur: z,
        };
        let mut s = snap(true, true);
        s.oil_temp_f = Some(205.0);
        assert!(
            caution.eval(&s).is_none() && warn.eval(&s).is_none(),
            "205°F cruise → silent"
        );
        s.oil_temp_f = Some(242.0);
        assert!(
            caution.eval(&s).is_some() && warn.eval(&s).is_none(),
            "caution band only"
        );
        s.oil_temp_f = Some(250.0);
        assert!(
            warn.eval(&s).is_some() && caution.eval(&s).is_none(),
            "warning band only"
        );
    }

    #[test]
    fn carb_ice_advisory_conditions() {
        let c = CarbIce;
        let mut s = snap(true, true);
        s.on_ground = false;
        s.oat_c = 10.0;
        s.dewpoint_c = 5.0; // spread 5 → humid
        s.rpm = 1500.0; // throttled back
        assert!(c.eval(&s).is_some(), "ripe conditions → advise carb heat");
        s.carb_heat_on = true;
        assert!(c.eval(&s).is_none(), "carb heat already on → silent");
        s.carb_heat_on = false;
        s.dewpoint_c = -30.0; // dry
        assert!(c.eval(&s).is_none(), "dry air → silent");
        s.dewpoint_c = 5.0;
        s.rpm = 2400.0; // cruise power
        assert!(c.eval(&s).is_none(), "high power → silent");
    }

    #[test]
    fn low_vacuum_and_alternator_ammeter() {
        let v = LowVacuum;
        let mut s = snap(true, true);
        s.on_ground = false;
        assert!(v.eval(&s).is_none(), "vacuum ok → silent");
        s.low_vacuum = true;
        assert!(
            v.eval(&s).is_some(),
            "low vacuum airborne → warn unreliable gyros"
        );
        // On the ground the light is NORMAL at idle RPM (POH checks vacuum at
        // run-up power) — cautioning during taxi is noise, never spoken.
        s.on_ground = true;
        assert!(v.eval(&s).is_none(), "on the ground → silent");
        s.on_ground = false;
        s.engine_running = false;
        assert!(v.eval(&s).is_none(), "engine off → no vacuum check");

        let a = AlternatorNotCharging { noun: "Alternator" };
        let mut s = snap(true, true); // alternator switch on
        s.battery_amps = 8.0; // charging
        assert!(a.eval(&s).is_none(), "charging → silent");
        s.battery_amps = -10.0; // discharging with alternator on
        assert!(a.eval(&s).is_some(), "not charging → caution");
        let mut s = snap(true, false); // alternator off — AlternatorOffline's job
        s.battery_amps = -10.0;
        assert!(a.eval(&s).is_none(), "switch off → handled elsewhere");
    }

    /// EGPF 2026-09-21, the A330's circuit (pilot bands, the trend
    /// projected): the four "getting slow" calls in CONF 1 and 2, from the
    /// recorded `state` ticks (ias, flap, ias trend over the preceding
    /// ticks). On the straight line each is in the caution band; on the
    /// profile's declared table none is, and a genuinely slow CONF 2 still is.
    #[test]
    fn the_a330s_recorded_conf_2_base_is_not_slow() {
        let p = crate::aircraft::load("airbus_a330_300").expect("the A330 loads");
        let checks = build_checks(&p);
        let caution = checks
            .iter()
            .find(|c| c.id() == "airspeed_caution")
            .expect("the A330 has a stall caution");
        let line = StallSpeed {
            id: "line",
            severity: Severity::Caution,
            vs_clean_kts: 175.0,
            vs0_kts: Some(125.0),
            detent_stall: Vec::new(),
            at_or_below_factor: 1.20,
            above_factor: Some(1.10),
        };
        let mut s = snap(true, true);
        s.on_ground = false;
        s.agl_ft = 900.0;
        s.automation_flying = false;
        // t 1789976359.2, 751.7, 788.6, 837.3.
        for (ias, flap, trend) in [(183.5, 0.25, 0.0), (175.2, 0.5, -1.7), (177.2, 0.5, -0.8), (177.3, 0.5, -0.5)] {
            s.ias_kts = ias;
            s.flap_ratio = flap;
            s.ias_trend_kts_per_s = trend;
            assert!(line.eval(&s).is_some(), "the old line called {ias} at {flap}");
            assert!(caution.eval(&s).is_none(), "{ias} kt at flap {flap} is not slow");
        }
        s.ias_kts = 160.0;
        s.flap_ratio = 0.5;
        s.ias_trend_kts_per_s = 0.0;
        assert!(caution.eval(&s).is_some(), "160 in CONF 2 is 1.15 x 139");
    }

    #[test]
    fn airspeed_bands_and_flare_and_ground_suppression() {
        // Vs=48 clean → caution ≤57.6 (and >52.8), warning ≤52.8.
        let caution = StallSpeed {
            id: "c",
            severity: Severity::Caution,
            vs_clean_kts: 48.0,
            vs0_kts: None,
            detent_stall: Vec::new(),
            at_or_below_factor: 1.20,
            above_factor: Some(1.10),
        };
        let warn = StallSpeed {
            id: "w",
            severity: Severity::Warning,
            vs_clean_kts: 48.0,
            vs0_kts: None,
            detent_stall: Vec::new(),
            at_or_below_factor: 1.10,
            above_factor: None,
        };
        let mut s = snap(true, true);
        s.on_ground = false;
        s.agl_ft = 500.0; // airborne, above the flare

        s.ias_kts = 65.0; // normal approach — silent
        assert!(caution.eval(&s).is_none() && warn.eval(&s).is_none());
        s.ias_kts = 55.0; // getting slow — caution only
        assert!(caution.eval(&s).is_some() && warn.eval(&s).is_none());
        s.ias_kts = 50.0; // near stall — warning only (bands disjoint)
        assert!(warn.eval(&s).is_some() && caution.eval(&s).is_none());

        // Rate term: a fast washout trips the caution while the speed is still
        // well above the line. 67 kt is silent steady, but decaying at 3 kt/s it
        // projects to 67 − 3×4.0 = 55 → caution.
        s.ias_kts = 67.0;
        s.ias_trend_kts_per_s = 0.0;
        assert!(caution.eval(&s).is_none() && warn.eval(&s).is_none());
        s.ias_trend_kts_per_s = -3.0;
        assert!(caution.eval(&s).is_some() && warn.eval(&s).is_none());
        // Same 67 kt accelerating away (recovering) stays silent — projection
        // only ever lowers the effective speed, never raises it.
        s.ias_trend_kts_per_s = 3.0;
        assert!(caution.eval(&s).is_none() && warn.eval(&s).is_none());
        s.ias_trend_kts_per_s = 0.0;

        // Flaps lower the stall speed, so the same 55 kt is now safe and silent;
        // the band only fires once you're slow *for that configuration*.
        s.flap_ratio = 1.0; // full flaps → Vs ≈ 39.8, caution ≤47.8
        s.ias_kts = 55.0;
        assert!(caution.eval(&s).is_none() && warn.eval(&s).is_none());
        s.ias_kts = 46.0; // slow for full flaps — caution
        assert!(caution.eval(&s).is_some() && warn.eval(&s).is_none());
        s.flap_ratio = 0.0;

        // Slow but in the flare → suppressed (slow-and-low is intended there).
        s.ias_kts = 50.0;
        s.agl_ft = 20.0;
        assert!(caution.eval(&s).is_none() && warn.eval(&s).is_none());
        // On the ground → suppressed.
        s.agl_ft = 500.0;
        s.on_ground = true;
        assert!(caution.eval(&s).is_none() && warn.eval(&s).is_none());
    }

    /// EGSS, 2026-09-22: the governor walked the estimate over the 60 minute
    /// edge and the same caution was said every 31 s. One episode, one line,
    /// then one more per ten minutes lost, in different words.
    #[test]
    fn a_fuel_band_is_held_over_a_wobble_and_said_again_only_per_ten_minutes() {
        let caution = fuel_caution(); // 45 down to 30
        let mut s = snap(true, true);
        s.on_ground = false;
        s.fuel_flow_kg_sec = None;
        s.fuel_kg = None;
        s.endurance_min = Some(44.0);
        let first = caution.eval(&s).unwrap();
        assert!(first.starts_with("Fuel's getting low"), "{first}");
        assert_eq!(caution.repeat_level(&s), Some(0));
        // The wobble back over the edge does not end the episode...
        s.endurance_min = Some(47.0);
        assert!(caution.eval(&s).is_some());
        assert_eq!(caution.repeat_level(&s), Some(-1), "and is no new level");
        // ...a real change (fuel found, a big power change) does.
        s.endurance_min = Some(56.0);
        assert!(caution.eval(&s).is_none());
        s.endurance_min = Some(47.0);
        assert!(
            caution.eval(&s).is_none(),
            "outside the band and never latched"
        );
        // Ten minutes on is a new level with its own words.
        s.endurance_min = Some(34.0);
        assert_eq!(caution.repeat_level(&s), Some(1));
        assert_eq!(
            caution.eval(&s).unwrap(),
            "Fuel check, about 35 minutes now."
        );
        let warn = fuel_warning();
        s.endurance_min = Some(9.0);
        assert_eq!(
            warn.eval(&s).unwrap(),
            "About 10 minutes of fuel. We land now."
        );
        assert!(!warn.eval(&s).unwrap().contains('\u{2014}'));
    }

    /// The 172's fuel bands: caution ≤ 45 min, warning ≤ 30 min.
    fn fuel_caution() -> FuelEndurance {
        FuelEndurance {
            id: "fuel_caution",
            severity: Severity::Caution,
            at_or_below_min: 45.0,
            above_min: Some(30.0),
            cruise_burn_kg_hr: None,
            entered: Default::default(),
        }
    }
    fn fuel_warning() -> FuelEndurance {
        FuelEndurance {
            id: "fuel_warning",
            severity: Severity::Warning,
            at_or_below_min: 30.0,
            above_min: None,
            cruise_burn_kg_hr: None,
            entered: Default::default(),
        }
    }

    #[test]
    fn rolling_on_park_brake_fires_only_when_moving_on_ground() {
        let c = RollingOnParkBrake;
        let mut s = snap(true, true);
        // Parked with the brake set (a run-up): silent.
        s.park_brake = 1.0;
        s.groundspeed_kts = 0.0;
        assert!(c.eval(&s).is_none(), "stationary is fine");
        // Rolling with it set: caution.
        s.groundspeed_kts = 20.0;
        assert!(c.eval(&s).is_some(), "rolling with the brake must fire");
        // Rolling with it released: silent.
        s.park_brake = 0.0;
        assert!(c.eval(&s).is_none());
        // Airborne (stale ratio): silent.
        s.park_brake = 1.0;
        s.on_ground = false;
        assert!(c.eval(&s).is_none(), "not a thing in the air");
        // The taxi-in's own arrest of a rolling handoff: the copilot set the
        // brake and announced it; it does not then nag the pilot about it.
        s.on_ground = true;
        s.groundspeed_kts = 38.0;
        assert!(c.eval(&s).is_some());
        crate::taxi_in::ARREST_HOLD.store(true, std::sync::atomic::Ordering::Relaxed);
        assert!(
            c.eval(&s).is_none(),
            "the copilot's own brake is not a caution"
        );
        crate::taxi_in::ARREST_HOLD.store(false, std::sync::atomic::Ordering::Relaxed);
        assert!(c.eval(&s).is_some());
    }

    /// EGKK 26L (1790524628, 2026-09-27): the reject's `brakes_max` read
    /// `park` 0.7 at 87 kt and 1.0 at 69 kt, and the caution told the crew
    /// to release the brake that was stopping the aircraft. Our own brake
    /// command is not a forgotten parking brake; the same reading with our
    /// brakes quiet still is.
    #[test]
    fn a_rejects_own_brake_is_not_a_rolling_parking_brake() {
        let c = RollingOnParkBrake;
        let mut s = snap(true, true);
        for (gs, park) in [(87.7, 0.7), (69.0, 1.0), (48.4, 1.0)] {
            s.groundspeed_kts = gs;
            s.park_brake = park;
            s.crew_brake = true;
            assert!(c.eval(&s).is_none(), "the reject's brake at {gs} kt, park {park}");
            s.crew_brake = false;
            assert!(c.eval(&s).is_some(), "a pilot's brake at {gs} kt, park {park}");
        }
    }

    /// EGKK (1790524628): the reject's brake command latched the plugin's
    /// `brake_yield` 0.5 s later, and the latch stayed up through the stop.
    /// A latch that rose on our command is ours for its whole episode; one
    /// that rose with our brakes quiet is a hand.
    /// #181: the servo, the elevator surface and the dials ride every
    /// airborne `state` tick the autopilot or our column flies, and only
    /// those (eight reads a tick are the sim's frame time).
    #[test]
    fn the_pitch_channel_is_logged_airborne_under_a_servo_or_our_column() {
        assert!(super::pitch_reads_due(false, true, false));
        assert!(super::pitch_reads_due(false, false, true));
        assert!(!super::pitch_reads_due(false, false, false));
        assert!(!super::pitch_reads_due(true, true, true), "never on the wheels");
        let i = super::Inputs {
            pitch: Some(super::PitchInputs {
                servo: Some(0.1234),
                elev_deg: Some(-3.456),
                vs_dial: Some(-500.0),
                ovr_pitch: Some(0.0),
                ..Default::default()
            }),
            ..Default::default()
        };
        let j = i.json();
        assert_eq!(j["pitch"]["servo"], 0.123);
        assert_eq!(j["pitch"]["elev"], -3.46);
        assert_eq!(j["pitch"]["vs_dial"], -500.0);
        assert_eq!(j["pitch"]["ovr"], false);
        assert!(j["pitch"]["tot"].is_null(), "a failed read is null");
        assert!(super::Inputs::default().json()["pitch"].is_null());
    }

    #[test]
    fn the_crews_own_reject_brake_is_not_a_pilot_on_the_brakes() {
        use crate::flight::Control;
        let tick = |latched: bool| super::Inputs {
            steer_yield: Some(0.0),
            pitch_yield: Some(0.0),
            brake_yield: Some(if latched { 1.0 } else { 0.0 }),
            ..Default::default()
        };
        let ctl = Control::Automation("takeoff");
        let mut latch = super::CrewBrakeLatch::default();
        // (latched, crew brake in flight, a hand): the roll, the reject's
        // command, the latch rising with it, the stop, our brake quiet with
        // the latch still up, the steer channel stale (the latch drops),
        // then a pilot's key with our brakes quiet, still up when ours fire.
        let ticks = [
            (false, false, false),
            (false, true, false),
            (true, true, false),
            (true, true, false),
            (true, false, false),
            (false, false, false),
            (true, false, true),
            (true, true, true),
        ];
        for (k, (latched, crew, hand)) in ticks.iter().enumerate() {
            let crews = latch.step(*latched, *crew);
            let now = super::interventions_now(&tick(*latched), ctl, false, crews);
            assert_eq!(now[2], *hand, "tick {k}");
        }
    }

    #[test]
    fn cabin_altitude_bands_are_disjoint_and_airborne_only() {
        let caution = CabinAltitude {
            id: "c",
            severity: Severity::Caution,
            band_ft: (10_000.0, Some(12_500.0)),
            message: "caution",
            min_dur: Duration::from_secs(10),
        };
        let warning = CabinAltitude {
            id: "w",
            severity: Severity::Warning,
            band_ft: (12_500.0, None),
            message: "warning",
            min_dur: Duration::from_secs(4),
        };
        let mut s = snap(true, true);
        s.on_ground = false;
        // Below 10k: nothing.
        s.cabin_altitude_ft = 8_000.0;
        assert!(caution.eval(&s).is_none() && warning.eval(&s).is_none());
        // 10k–12.5k: caution only (disjoint bands).
        s.cabin_altitude_ft = 11_000.0;
        assert!(caution.eval(&s).is_some() && warning.eval(&s).is_none());
        // Above 12.5k: warning only.
        s.cabin_altitude_ft = 14_000.0;
        assert!(caution.eval(&s).is_none() && warning.eval(&s).is_some());
        // On the ground it never fires, however high the reading.
        s.on_ground = true;
        assert!(caution.eval(&s).is_none() && warning.eval(&s).is_none());
    }

    #[test]
    fn alternator_fires_only_when_running_without_charge() {
        let c = AlternatorOffline { noun: "Alternator" };
        assert!(c.eval(&snap(true, false)).is_some()); // running, alt off → alert
        assert!(c.eval(&snap(true, true)).is_none()); // running, alt on → ok
        assert!(c.eval(&snap(false, false)).is_none()); // shut down → ok
        assert_eq!(c.min_duration(), ALTERNATOR_SUSTAIN); // sustained before warning
    }

    #[test]
    fn sustain_suppresses_transient_then_fires_when_held() {
        let t0 = Instant::now();
        let d = ALTERNATOR_SUSTAIN;
        let mut st = FireState::new();

        // The start transient: alternator offline for a few seconds, then the
        // ALT switch goes on and the condition clears — must never reach speak.
        assert!(!st.sustained(d, t0)); // rising edge: timer starts, not yet due
        assert!(!st.sustained(d, t0 + Duration::from_secs(3)));
        st.clear(); // alternator came online

        // A genuinely forgotten alternator: held continuously past the sustain.
        let t1 = t0 + Duration::from_secs(10);
        assert!(!st.sustained(d, t1)); // new rising edge — times from here
        assert!(!st.sustained(d, t1 + d - Duration::from_secs(1))); // still short
        assert!(st.sustained(d, t1 + d)); // held a full minute → now warn
    }

    #[test]
    fn fuel_bands_are_disjoint_and_airborne_only() {
        let caution = fuel_caution();
        let warning = fuel_warning();
        let mut s = snap(true, true);
        s.on_ground = false;

        // Plenty of fuel: nothing fires.
        s.endurance_min = Some(90.0);
        assert!(caution.eval(&s).is_none() && warning.eval(&s).is_none());

        // Caution band (30 < min ≤ 45): only caution.
        s.endurance_min = Some(40.0);
        assert!(caution.eval(&s).is_some());
        assert!(warning.eval(&s).is_none());

        // Warning band (≤ 30): only warning, not caution (disjoint handoff).
        s.endurance_min = Some(25.0);
        assert!(caution.eval(&s).is_none());
        assert!(warning.eval(&s).is_some());

        // On the ground: silent even when low (idle burn makes endurance moot).
        s.on_ground = true;
        assert!(caution.eval(&s).is_none() && warning.eval(&s).is_none());

        // Engine off, or endurance uncomputable: silent.
        s.on_ground = false;
        s.engine_running = false;
        assert!(warning.eval(&s).is_none());
        s.engine_running = true;
        s.endurance_min = None;
        assert!(warning.eval(&s).is_none());
    }

    #[test]
    fn fuel_pretakeoff_cautions_on_ground_only_when_low() {
        // 45 min caution; 24 kg/hr cruise → 18 kg ≈ 45 min, 14 kg ≈ 35 min.
        let c = FuelPreTakeoff {
            caution_min: 45.0,
            cruise_burn_kg_hr: 24.0,
        };
        let mut s = snap(true, true); // on ground, engine running

        // Plenty for a flight: silent.
        s.fuel_kg = Some(40.0); // 100 min at cruise
        assert!(c.eval(&s).is_none());

        // Low for departure: cautions.
        s.fuel_kg = Some(14.0); // ~35 min at cruise
        assert!(c.eval(&s).is_some());

        // Airborne: this check is ground-only (FuelEndurance owns the air).
        s.on_ground = false;
        assert!(c.eval(&s).is_none());

        // Engine off, or no fuel read: silent.
        s.on_ground = true;
        s.engine_running = false;
        assert!(c.eval(&s).is_none());
        s.engine_running = true;
        s.fuel_kg = None;
        assert!(c.eval(&s).is_none());
    }

    #[test]
    fn gyro_sync_took_needs_the_drift_to_fall() {
        // A real sync zeroes the drift.
        assert!(gyro_sync_took(13.0, 0.2));
        assert!(gyro_sync_took(-12.0, 1.0));
        // The stock 737 (LEBL-LFBD 2026-09-27): accepted, nothing moved.
        assert!(!gyro_sync_took(11.8, 11.8));
        assert!(!gyro_sync_took(11.8, 11.9));
        assert!(!gyro_sync_took(15.6, 15.6));
        // A token dent is not a sync.
        assert!(!gyro_sync_took(12.0, 9.0));
    }

    #[test]
    fn gyro_sync_says_only_what_it_read_back() {
        let t0 = Instant::now();
        let mut g = GyroSync::default();
        assert!(!g.due(5.0, t0), "under the advisory");
        assert!(g.due(11.8, t0));
        g.last = Some(t0);
        g.fired(11.8, t0);
        assert!(!g.due(11.8, t0), "readback owed");
        assert!(!g.verify_due(t0 + Duration::from_millis(500)));
        let t1 = t0 + GYRO_VERIFY_AFTER;
        assert!(g.verify_due(t1));
        // A failed read is no measurement: still owed, nothing said.
        assert_eq!(g.settle(None, t1), None);
        assert!(g.verify_due(t1));
        // The 737's readback: the drift did not move. Silent, latched.
        assert_eq!(
            g.settle(Some(11.9), t1),
            Some(GyroSyncOutcome::DidNotTake {
                before: 11.8,
                after: 11.9
            })
        );
        assert!(!g.verify_due(t1));
        // Fourteen calls in the sortie; now none, whatever the drift does.
        for k in 1..=14u32 {
            let t = t0 + GYRO_SYNC_COOLDOWN * k;
            assert!(!g.due(11.8 + 0.3 * f64::from(k), t), "latched at {k}");
        }
        // A swap resets it (the loop builds a fresh one).
        g = GyroSync::default();
        assert!(g.due(15.6, t0));
    }

    #[test]
    fn gyro_sync_that_took_is_said_and_can_come_again() {
        let t0 = Instant::now();
        let mut g = GyroSync::default();
        g.last = Some(t0);
        g.fired(-13.0, t0);
        let t1 = t0 + GYRO_VERIFY_AFTER;
        assert_eq!(
            g.settle(Some(0.1), t1),
            Some(GyroSyncOutcome::Took {
                before: -13.0,
                after: 0.1
            })
        );
        assert!(!g.due(12.0, t1), "cooldown");
        assert!(g.due(12.0, t0 + GYRO_SYNC_COOLDOWN));
    }

    #[test]
    fn gyro_sync_unread_gives_up_without_latching() {
        let t0 = Instant::now();
        let mut g = GyroSync::default();
        g.last = Some(t0);
        g.fired(12.0, t0);
        let late = t0 + GYRO_VERIFY_GIVE_UP;
        assert_eq!(
            g.settle(None, late),
            Some(GyroSyncOutcome::Unmeasured { before: 12.0 })
        );
        assert!(g.due(12.0, t0 + GYRO_SYNC_COOLDOWN), "not latched");
    }

    #[test]
    fn gyro_sync_line_rounds_to_nearest_five() {
        // The monitor now SYNCS the DG itself and announces it; the spoken
        // figure sounds like an estimate (either drift sign).
        let msg = gyro_sync_line(-13.0);
        assert!(msg.contains("about 15 degrees"), "got: {msg}");
        assert!(msg.contains("re-synced"), "got: {msg}");
        assert!(gyro_sync_line(11.0).contains("about 10 degrees"));
    }

    #[test]
    fn warning_speaks_on_edge_then_repeats() {
        let mut st = FireState::new();
        let t0 = Instant::now();
        assert!(st.should_speak(Severity::Warning, t0)); // rising edge
        st.spoke_this_episode = true;
        st.last_spoken = Some(t0);
        assert!(!st.should_speak(Severity::Warning, t0 + Duration::from_secs(1))); // too soon
        assert!(st.should_speak(
            Severity::Warning,
            t0 + RENOTIFY_COOLDOWN + Duration::from_secs(1)
        )); // due
    }

    #[test]
    fn advisory_speaks_once_and_does_not_repeat() {
        let mut st = FireState::new();
        let t0 = Instant::now();
        assert!(st.should_speak(Severity::Advisory, t0)); // rising edge
        st.spoke_this_episode = true;
        st.last_spoken = Some(t0);
        // Even long after, a still-firing Advisory doesn't nag.
        assert!(!st.should_speak(Severity::Advisory, t0 + RENOTIFY_COOLDOWN * 3));
    }

    #[test]
    fn fiddling_within_cooldown_does_not_renag() {
        let mut st = FireState::new();
        let t0 = Instant::now();
        // First occurrence speaks.
        assert!(st.should_speak(Severity::Warning, t0));
        st.spoke_this_episode = true;
        st.last_spoken = Some(t0);
        // Pilot fixes it.
        st.clear();
        assert!(!st.spoke_this_episode);
        // Recurs almost immediately → suppressed (still within cooldown).
        assert!(!st.should_speak(Severity::Warning, t0 + Duration::from_secs(3)));
        // Recurs well after the cooldown → speaks again.
        assert!(st.should_speak(
            Severity::Warning,
            t0 + RENOTIFY_COOLDOWN + Duration::from_secs(1)
        ));
    }

    #[test]
    fn sustained_caution_respeaks_on_a_fresh_episode() {
        // The exact call pattern active() runs: on every bad tick, sustained()
        // first (arming the rising edge), should_speak() only once the sustain
        // has elapsed. A recurred Caution must survive that wait — the old
        // every-tick firing flag consumed the edge during it, so a stalling
        // 172's second "getting slow" was never spoken (2026-07-13).
        let mut st = FireState::new();
        let d = Duration::from_millis(700);
        let t0 = Instant::now();
        let tick = Duration::from_secs(1);
        // Episode 1: sustain tick, then the speak tick.
        assert!(!st.sustained(d, t0));
        assert!(st.sustained(d, t0 + tick) && st.should_speak(Severity::Caution, t0 + tick));
        st.last_spoken = Some(t0 + tick);
        st.spoke_this_episode = true;
        // Condition clears (speed recovered).
        st.clear();
        // Episode 2, minutes later: same pattern, and it MUST speak again.
        let t1 = t0 + Duration::from_secs(500);
        assert!(!st.sustained(d, t1));
        assert!(
            st.sustained(d, t1 + tick) && st.should_speak(Severity::Caution, t1 + tick),
            "a fresh episode past the cooldown must speak even after the sustain wait"
        );
    }

    #[test]
    fn the_coarse_phase_is_the_flight_models() {
        use crate::flight::Phase as F;
        assert_eq!(coarse_phase(F::TaxiOut, 0.0), Phase::Ground);
        assert_eq!(coarse_phase(F::TakeoffRoll, 0.0), Phase::Takeoff);
        assert_eq!(coarse_phase(F::InitialClimb, 400.0), Phase::Climb);
        assert_eq!(coarse_phase(F::Descent, 9000.0), Phase::Cruise, "high: no approach checks");
        assert_eq!(coarse_phase(F::Descent, 2500.0), Phase::Approach);
        assert_eq!(coarse_phase(F::Approach, 2000.0), Phase::Approach);
        assert_eq!(coarse_phase(F::Approach, 900.0), Phase::Landing);
        assert_eq!(coarse_phase(F::TaxiIn, 0.0), Phase::Ground);
    }
