
    /// EGSS 22, 2026-09-22 (issue #65): "Down and stopped. Brakes on, your
    /// aircraft." with the 737's `parking_brake_ratio` at 0.03, the rollout's
    /// squeeze expired. The stop's line is chosen from the parking brake's
    /// read-back: the brake reading 0.03 through every drive names that it did
    /// not set, and only a brake that reads set is "Brakes on".
    #[tokio::test]
    async fn the_stop_says_brakes_on_only_when_the_parking_brake_reads_set() {
        use crate::xplane::FakeXPlane;
        const PARK: &str = "sim/cockpit2/controls/parking_brake_ratio";
        const SET: &str = "sim/flight_controls/park_brake_set";
        let egss = FakeXPlane::with_reads(&[(PARK, 0.03)]);
        let r = crate::tools::controls::set_park_brake_confirmed(&egss, None, 3).await;
        let line = super::stopped_line(r.held);
        assert!(!line.contains("Brakes on"), "{line}");
        assert!(line.contains("did not set"), "{line}");

        let set = FakeXPlane::with_reads(&[(PARK, 0.03)]);
        set.step_on_command(SET, PARK, 0.97);
        let r = crate::tools::controls::set_park_brake_confirmed(&set, None, 3).await;
        assert_eq!(
            super::stopped_line(r.held),
            "Down and stopped. Brakes on, your aircraft."
        );
        assert!(
            !set.wrote(PARK),
            "the command set it; no raw ratio write needed"
        );
    }

    /// The flare flies the ELEVATOR, and it has to arrest a real sink in the
    /// three seconds it has. Flown against a plant with no autopilot in it —
    /// elevator to pitch rate to sink — because removing the autopilot's
    /// 1.5 s of dead time is the entire point of the change.
    #[test]
    fn the_flare_arrests_a_jets_sink_on_the_elevator() {
        // A 737-ish short-period response: elevator commands a pitch rate,
        // pitch changes the sink, both first order and quick. Deliberately
        // NOT the VS-mode plant, which is what we stopped flying through.
        struct Plant {
            vvi: f64,
            rate: f64,
        }
        impl Plant {
            fn step(&mut self, elev: f64, dt: f64) {
                // Elevator to vertical acceleration, with a short lag.
                let want = elev * 1400.0; // fpm/s at full commanded travel
                self.rate += (want - self.rate) * (dt / 0.35).min(1.0);
                self.vvi += self.rate * dt;
            }
        }
        let mut law = FlarePitch::default();
        let mut p = Plant {
            vvi: -900.0,
            rate: 0.0,
        };
        let target = -175.0; // the 737's flare target
        let dt = 0.1; // the flare tick rate
        let mut prev = p.vvi;
        let mut worst_up: f64 = 0.0;
        for _ in 0..30 {
            // 3 seconds
            let trend = (p.vvi - prev) / dt;
            prev = p.vvi;
            let e = law.step(target, p.vvi, trend, dt, false);
            worst_up = worst_up.max(e);
            p.step(e, dt);
        }
        assert!(
            p.vvi > target - 120.0,
            "three seconds must arrest 900 fpm to near {target}: got {:.0}",
            p.vvi
        );
        assert!(
            p.vvi < 0.0,
            "and must not balloon: the arrest settles ON the target, it does \
             not fly through it: got {:.0}",
            p.vvi
        );
        assert!(
            (p.vvi - target).abs() < 60.0,
            "settled on {target}, not merely near it: got {:.0}",
            p.vvi
        );
        assert!(
            worst_up <= FLARE_PITCH_MAX + 1e-9,
            "never past the clamp: {worst_up}"
        );
    }

    /// EGLF 06, 2026-09-22: the arrest worked and then gave the runway back.
    /// The law beat its target at thirteen feet, relaxed the column from 0.35
    /// to 0.185 because the error had gone negative, and [`FlarePower`] -
    /// unlocked by that very arrest - took the lever from 0.29 to idle over
    /// the last 3.4 s. Touchdown at about 360 fpm and still worsening.
    ///
    /// The plant is the flare plant above with the thrust leaving underneath
    /// it: a sinking bias that ramps in over three seconds, which is what
    /// losing a third of a lever's travel at 140 kt feels like. Without the
    /// ratchet the law hands it back; with it the column stays where the
    /// arrest put it and the wheels arrive softly.
    #[test]
    fn the_flare_does_not_hand_back_the_arrest_when_the_thrust_leaves() {
        struct Plant {
            vvi: f64,
            rate: f64,
            /// Sink the departing thrust adds, fpm/s at full effect.
            thrust_loss: f64,
        }
        impl Plant {
            fn step(&mut self, elev: f64, dt: f64, retard: f64) {
                let want = elev * 1400.0 - self.thrust_loss * retard;
                self.rate += (want - self.rate) * (dt / 0.35).min(1.0);
                self.vvi += self.rate * dt;
            }
        }

        /// Fly the last four seconds and return the touchdown sink.
        fn fly(ratchet: bool) -> f64 {
            let mut law = FlarePitch::default();
            let mut p = Plant {
                vvi: -510.0, // where the EGLF arrest had got to at 21 ft
                rate: 0.0,
                thrust_loss: 260.0,
            };
            let (target, dt) = (-197.0, 0.1);
            let mut prev = p.vvi;
            for tick in 0..40 {
                // The lever walks off over the last 3.4 s, as it did live.
                let retard = (tick as f64 * dt / 3.4).clamp(0.0, 1.0);
                let trend = (p.vvi - prev) / dt;
                prev = p.vvi;
                let e = law.step(target, p.vvi, trend, dt, ratchet);
                p.step(e, dt, retard);
            }
            p.vvi
        }

        let handed_back = fly(false);
        assert!(
            handed_back < -300.0,
            "the tracker must still reproduce the firm landing, or this test \
             is not about EGLF any more: got {handed_back:.0}"
        );
        let held = fly(true);
        assert!(
            held > -300.0,
            "the column stays where the arrest put it, so the wheels arrive \
             under the instructor's firm threshold: got {held:.0}"
        );
        assert!(held < 0.0, "and the aircraft still lands: got {held:.0}");
    }

    /// EDDV 27L, 2026-09-22, the Citation: the flare at 28 ft held its pull
    /// through a climb (+545 fpm, up to 58 ft), and the phase flipped back to
    /// final, which pushed. The plant needs a STANDING pull to hold the flare
    /// attitude (`E_TRIM`), so neutral elevator sinks and anything held above
    /// it climbs: that is what makes a held pull a balloon. The old ratchet
    /// is exactly the running maximum of the free law (the integrator never
    /// reads `held`), so the test flies both.
    #[test]
    fn a_balloon_eases_the_pull_and_settles_onto_the_runway() {
        const E_TRIM: f64 = 0.10;
        struct Plant {
            h: f64,
            vvi: f64,
            rate: f64,
        }
        impl Plant {
            fn step(&mut self, elev: f64, dt: f64) {
                let want = (elev - E_TRIM) * 1400.0;
                self.rate += (want - self.rate) * (dt / 0.35).min(1.0);
                self.vvi += self.rate * dt;
                self.h += self.vvi / 60.0 * dt;
            }
        }
        // (peak height, touchdown sink) from the flare gate at 140 kt.
        fn fly(old_ratchet: bool) -> (f64, f64) {
            let mut law = FlarePitch::default();
            let mut p = Plant {
                h: 37.0,
                vvi: -480.0,
                rate: 0.0,
            };
            let (_, target) = arrest_targets_fpm(-(140.0 / 60.0) * GLIDEPATH_FT_PER_NM);
            let dt = 0.1;
            let (mut prev, mut peak, mut held) = (p.vvi, p.h, f64::MIN);
            for _ in 0..300 {
                let trend = (p.vvi - prev) / dt;
                prev = p.vvi;
                let e = if old_ratchet {
                    held = held.max(law.step(target, p.vvi, trend, dt, false));
                    held
                } else {
                    law.step(target, p.vvi, trend, dt, true)
                };
                p.step(e, dt);
                peak = peak.max(p.h);
                if p.h <= 0.0 {
                    return (peak, p.vvi);
                }
            }
            (peak, p.vvi)
        }
        let (old_peak, _) = fly(true);
        assert!(
            old_peak > 55.0,
            "the old ratchet must still balloon, or this test is not about \
             EDDV any more: peaked at {old_peak:.0} ft"
        );
        let (peak, sink) = fly(false);
        assert!(
            peak < old_peak - 10.0,
            "eased: {peak:.0} ft against {old_peak:.0}"
        );
        let (_, band) = arrest_gates_ft(25.0, 140.0);
        assert!(
            peak < band,
            "never high enough to balk: {peak:.0} against {band:.0}"
        );
        assert!(
            sink > -400.0 && sink < 0.0,
            "and it lands softly: {sink:.0} fpm"
        );
    }

    /// Once flaring, a balloon is still the flare; above the roundout band it
    /// is a balked landing, never "final" again with its push.
    #[test]
    fn a_balloon_stays_in_the_flare_and_a_big_one_goes_around() {
        let rwy = rwy_north();
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 0.05);
        let (flare_ft, band_ft) = arrest_gates_ft(25.0, 140.0);
        let at =
            |radio: f64| guidance(lat, lon, radio, radio, 140.0, 140.0, &rwy, 132.0, 25.0, 0.0);

        let g = at(58.0);
        assert!(58.0 > flare_ft && 58.0 < band_ft, "{flare_ft} {band_ft}");
        assert_eq!(g.phase, ApproachPhase::Final, "the tick that pushed");
        let held = hold_flare(g, true, 58.0, 140.0, 25.0);
        assert_eq!(held.phase, ApproachPhase::Flare);
        assert!(held.arresting);
        let (_, flare_vs) = arrest_targets_fpm(-(140.0 / 60.0) * GLIDEPATH_FT_PER_NM);
        assert_eq!(held.target_vs_fpm, flare_vs);
        // Never flared: nothing changes.
        assert_eq!(
            hold_flare(g, false, 58.0, 140.0, 25.0).phase,
            ApproachPhase::Final
        );

        let high = band_ft + 10.0;
        let balked = hold_flare(at(high), true, high, 140.0, 25.0);
        assert_eq!(balked.phase, ApproachPhase::Intercept);
        assert_eq!(balked.balk, Some("balloon"));
    }

    /// ETND 26, 2026-09-23: 1,244 m of runway, the SF50 flaring at 32 ft and
    /// 344 m past the threshold, 900 m ahead, a 525 m landing distance. The
    /// overflight balk stood and the go-around hit the runway at -740 fpm.
    #[test]
    fn a_low_flare_with_room_to_stop_is_committed() {
        let g = Guidance {
            target_heading_deg: 265.0,
            target_vs_fpm: 1500.0,
            target_ias_kts: 87.0,
            dist_nm: 0.186,
            glidepath_alt_ft: 0.0,
            phase: ApproachPhase::Intercept,
            balk: Some("overflew_the_runway"),
            arresting: false,
        };
        let sf50_m = Some(1721.0 * 0.3048);
        let c = commit_flare(g, true, 32.0, 77.0, 25.0, 900.0, sf50_m, 265.0);
        assert_eq!(c.phase, ApproachPhase::Flare);
        assert_eq!(c.balk, None);
        assert!(c.arresting && c.target_vs_fpm < 0.0);
        assert_eq!(c.target_heading_deg, 265.0, "the runway's heading, not the FAP's");
        // Not flaring (crossed the numbers high and fast): the balk stands.
        assert_eq!(commit_flare(g, false, 32.0, 77.0, 25.0, 900.0, sf50_m, 265.0).balk, g.balk);
        // Above the band: the float's own geometry judges it.
        let (_, band) = arrest_gates_ft(25.0, 77.0);
        assert_eq!(commit_flare(g, true, band + 5.0, 77.0, 25.0, 900.0, sf50_m, 265.0).balk, g.balk);
        // No room to stop: still a go-around. At 20 ft the SF50 has flown
        // 175 m of its book's 525, so it asks 350 m (400 m now commits).
        assert_eq!(commit_flare(g, true, 20.0, 77.0, 25.0, 300.0, sf50_m, 265.0).balk, g.balk);
        // No declared distance: the 900 m floor.
        assert_eq!(commit_flare(g, true, 20.0, 77.0, 25.0, 850.0, None, 265.0).balk, g.balk);
        // A balloon (EDLP 24: 52 ft, 2,180 m of runway, 1,900 ahead) is
        // landed below the commit height and balked above it.
        let balloon = Guidance {
            balk: Some("balloon"),
            ..g
        };
        let c = commit_flare(balloon, true, 52.0, 81.0, 25.0, 1900.0, sf50_m, 240.0);
        assert_eq!((c.phase, c.balk), (ApproachPhase::Flare, None));
        let (roundout_vs, flare_vs) = arrest_targets_fpm(-(81.0 / 60.0) * GLIDEPATH_FT_PER_NM);
        assert_eq!(c.target_vs_fpm, roundout_vs, "down to the band at the roundout's sink");
        assert!(roundout_vs < flare_vs);
        assert_eq!(
            commit_flare(balloon, true, BALLOON_COMMIT_FT + 1.0, 81.0, 25.0, 1900.0, sf50_m, 240.0)
                .balk,
            Some("balloon")
        );
        assert_eq!(
            commit_flare(balloon, true, 52.0, 81.0, 25.0, 300.0, sf50_m, 240.0).balk,
            Some("balloon"),
            "no room to stop"
        );
        // Any other balk is not ours to overrule.
        let off = Guidance {
            balk: Some("off_centreline_at_flare"),
            ..g
        };
        assert_eq!(commit_flare(off, true, 20.0, 77.0, 25.0, 900.0, sf50_m, 265.0).balk, off.balk);
    }

    /// LFBD 11, 2026-09-27, the stock 737 (POM 5,360 ft): the flare floated
    /// to 7.9 ft radio at 145 kt over the ground, 796 m past the threshold
    /// with 1,622 m of the 2,418 ahead, and the overflight balk stood by
    /// 12 m against the whole book figure. It landed anyway, 1.3 s later,
    /// and stopped with about 700 m to spare.
    #[test]
    fn a_flare_is_credited_the_air_it_has_already_flown() {
        let b738_m = Some(5360.0 * 0.3048);
        let g = Guidance {
            target_heading_deg: 293.0,
            target_vs_fpm: 1500.0,
            target_ias_kts: 150.0,
            dist_nm: 0.43,
            glidepath_alt_ft: 0.0,
            phase: ApproachPhase::Intercept,
            balk: Some("overflew_the_runway"),
            arresting: false,
        };
        let c = commit_flare(g, true, 7.9, 145.0, 25.0, 1622.0, b738_m, 110.0);
        assert_eq!((c.phase, c.balk), (ApproachPhase::Flare, None), "the recorded landing");
        assert_eq!(c.target_heading_deg, 110.0, "the runway's heading");
        // The need at 7.9 ft: 1,634 less 42.1 ft of 3 degree path (245 m).
        let need = flare_pavement_need_m(7.9, b738_m);
        assert!((need - 1389.0).abs() < 2.0, "{need}");
        // The whole 291 m only at the ground, none at or above 50 ft.
        assert!((SCREEN_AIR_M - 290.8).abs() < 0.5, "{SCREEN_AIR_M}");
        let at_wheels = flare_pavement_need_m(0.0, b738_m);
        assert!((at_wheels - (5360.0 * 0.3048 - SCREEN_AIR_M)).abs() < 1e-9);
        assert_eq!(flare_pavement_need_m(50.0, b738_m), 5360.0 * 0.3048);
        assert_eq!(flare_pavement_need_m(80.0, b738_m), 5360.0 * 0.3048);
        // Genuinely too little runway still balks: 1,300 m at 7.9 ft.
        assert_eq!(
            commit_flare(g, true, 7.9, 145.0, 25.0, 1300.0, b738_m, 110.0).balk,
            Some("overflew_the_runway")
        );
        // A light single keeps half its book: the 172's 407 m asks 203 at
        // the ground, never the geometric 116.
        let c172_m = 1335.0 * 0.3048;
        assert_eq!(flare_pavement_need_m(0.0, Some(c172_m)), 0.5 * c172_m);
        // No declared figure: the Baron's 900 m, a rollout already, whole.
        assert_eq!(flare_pavement_need_m(5.0, None), FLOAT_MIN_REMAINING_M);
    }

    #[test]
    fn a_balloon_holds_the_approach_thrust_and_the_ratchet_lets_go_ahead_of_it() {
        assert_eq!(balloon_power(Some(300.0), Some(0.32), false, false), Some(0.32));
        assert_eq!(
            balloon_power(Some(157.0), Some(0.52), false, true),
            None,
            "LFOS: a progressive retard never restores approach power"
        );
        assert_eq!(
            balloon_power(Some(-200.0), Some(0.32), false, false),
            None,
            "sinking in the band: the flare's law"
        );
        assert_eq!(
            balloon_power(Some(-200.0), Some(0.32), true, false),
            Some(0.32),
            "coming back down to the band on power"
        );
        assert_eq!(balloon_power(Some(300.0), None, false, false), None, "nothing measured");
        // EDLP 24: 0.3 held, vvi -192 easing at +480 fpm/s is a balloon in
        // 0.4 s. The ratchet lets go now instead of at the climb.
        let mut p = FlarePitch::default();
        let held = p.step(-107.0, -451.0, -200.0, 0.2, true);
        assert!(held > 0.2, "{held}");
        let next = p.step(-107.0, -192.0, 480.0, 0.2, true);
        assert!(next < held, "released ahead of the balloon: {held} -> {next}");
        assert!(next >= 0.0, "never a push");
    }

    /// The ratchet is the flare's, not the roundout's, and it never outlives
    /// its approach: a balk and the next arrest starts from nothing.
    #[test]
    fn the_ratchet_is_the_flares_alone_and_resets_with_the_approach() {
        let mut law = FlarePitch::default();
        // Sinking hard: the law pulls, and the ratchet remembers it.
        let pulled = law.step(-197.0, -600.0, 0.0, 0.1, true);
        assert!(pulled > 0.0, "a sink that fast is a pull: {pulled}");
        // Now beating the target by a mile. The tracker wants the column
        // forward; in the flare it does not get it.
        let held = law.step(-197.0, 0.0, 0.0, 0.1, true);
        assert!(
            held >= pulled,
            "the flare's column only comes back: {pulled} then {held}"
        );
        // The same tick in the ROUNDOUT is ordinary flying.
        let mut loose = FlarePitch::default();
        loose.step(-197.0, -600.0, 0.0, 0.1, false);
        let relaxed = loose.step(-197.0, 0.0, 0.0, 0.1, false);
        assert!(
            relaxed < held,
            "the roundout is free both ways: {relaxed} against {held}"
        );
        law.reset();
        let fresh = law.step(-197.0, 0.0, 0.0, 0.1, true);
        assert!(
            fresh < held,
            "a new approach inherits no standing pull: {fresh} against {held}"
        );
    }

    /// The old VS path could not do it, which is why the change exists: the
    /// same three seconds through the 737's declared lag and dead time.
    #[test]
    fn the_vs_dial_could_not_have_arrested_it() {
        // 1.5 s of dead time plus a 1.5 s lag, the 737's ArrestLeadProfile.
        let (dt, target) = (0.6f64, -175.0f64); // and the OLD tick rate
        let mut vvi = -900.0f64;
        let mut queue = std::collections::VecDeque::from(vec![vvi; 3]);
        for _ in 0..5 {
            // 3 seconds at the old rate is five commands
            let commanded = target;
            queue.push_back(commanded);
            let effective = queue.pop_front().unwrap_or(commanded);
            vvi += (effective - vvi) * (dt / 1.5).min(1.0);
        }
        // Three seconds of dead time and lag leave it at about -436 against a
        // -175 target: a quarter of a G of unarrested sink still to absorb in
        // the undercarriage. The elevator law above settles ON -175 in the
        // same three seconds, which is the whole argument for the change.
        assert!(
            vvi < target - 200.0,
            "the dial path is still {:.0} fpm fast at touchdown: {vvi:.0}",
            target - vvi
        );
    }

    /// The tick rate is the bandwidth the flare has to fly with.
    ///
    /// 400 ms everywhere measured 0.59 s end to end on the EGPH landing
    /// (2026-09-20) — 1.7 Hz. A flare lasts about three seconds and the
    /// 737 declares 1.5 s of dead time, so the arrest got roughly two
    /// commands the aircraft could still act on: it touched down at 977 fpm
    /// against a 175 fpm target, and the yoke stepped visibly between
    /// updates. No gain fixes a bandwidth problem.
    #[test]
    fn the_flare_ticks_fast_enough_to_fly_an_arrest() {
        // Periods since 2026-09-30 (paced, not slept): the MacBook's measured
        // ticks, which the 100/200/400 sleeps plus their rounds made.
        assert_eq!(tick_ms(None, false), 540, "nothing to fly yet");
        assert_eq!(
            tick_ms(Some(ApproachPhase::Intercept), false),
            540,
            "miles out"
        );
        assert!(tick_ms(Some(ApproachPhase::Final), false) <= 330);
        let flare = tick_ms(Some(ApproachPhase::Flare), false);
        // The roundout is the arrest's first half, flown on the same
        // channels with the servos off: it ticks at the flare's rate.
        assert_eq!(tick_ms(Some(ApproachPhase::Final), true), flare);
        assert!(flare <= 200, "the flare needs feet, not seconds: {flare}");
        // The 737's arrest lead is 1.5 s of dead time; a three second flare
        // must carry enough commands to matter through it (the MacBook's
        // 0.20 s flare tick, the one the arrest was tuned on: fifteen).
        let commands_in_flare = 3000 / flare;
        assert!(
            commands_in_flare >= 15,
            "only {commands_in_flare} commands in a flare"
        );
        // ...and it must never get SLOWER as the aircraft gets closer.
        assert!(
            tick_ms(Some(ApproachPhase::Flare), false)
                <= tick_ms(Some(ApproachPhase::Final), false)
                && tick_ms(Some(ApproachPhase::Final), false)
                    <= tick_ms(Some(ApproachPhase::Intercept), false),
            "the tick rate must rise monotonically toward the ground"
        );
    }

    /// The tick's I/O budget: ONE read round and ONE write round, on the
    /// plugin's frame and on an older plugin's split reads alike. Every round
    /// is a sim frame (iMac 2026-09-30: one GET 46 ms at 20 fps, eight
    /// concurrent 44 to 51, six sequential 312), and the Final tick waited
    /// on about seven (`power` dt median 0.56 s against its 200 ms sleep).
    /// At 20 fps two rounds make the Flare 0.2 s and Final 0.3 s; at 50 fps
    /// 0.14 and 0.24.
    #[tokio::test(start_paused = true)]
    async fn a_tick_reads_in_one_round_and_writes_in_one() {
        use sim::state::{self, Want, Writes};
        use sim::xplane::{FakeXPlane, FramedXPlane};
        let pos = sim::position::Position {
            lat: 36.66,
            lon: -4.48,
            elev_m: 80.0,
            psi_true_deg: 131.0,
            gs_ms: 70.0,
            vs_fpm: -700.0,
            agl_m: 60.0,
            frame: Some(7),
            sim_time_s: Some(1.0),
            source: sim::position::Source::Plugin,
        };
        let fake = FakeXPlane::new();
        let frame = state::State::from_parts(
            Ok(pos),
            &[(F::Paused, 0.0), (F::Replay, 0.0), (F::Ias, 130.0), (F::Bug, 131.0)],
            state::Source::Plugin,
        );
        fake.set_read_bytes(state::STATE, Some(&state::encode(&frame)));
        // The profile's own lever and approach status ride beside the frame.
        fake.set_read("test/lever", 0.45);
        fake.set_read("test/appr", 0.0);
        // The split: every field the tick wants, and the position.
        for f in AUTOLAND_WANT_ILS {
            fake.set_read(f.name(), 0.0);
        }
        for (k, v) in [
            (crate::datarefs::LATITUDE, 36.66),
            (crate::datarefs::LONGITUDE, -4.48),
            (crate::datarefs::ELEVATION_M, 80.0),
            (PSI_TRUE, 131.0),
            (GS_MS, 70.0),
            ("sim/flightmodel/position/vh_ind_fpm", -700.0),
            (crate::datarefs::AGL_METERS, 18.0),
            (crate::datarefs::PAUSED, 0.0),
            (crate::datarefs::IS_IN_REPLAY, 0.0),
        ] {
            fake.set_read(k, v);
        }
        let xp = FramedXPlane::new(fake, std::time::Duration::from_millis(50));
        for plugin in [true, false] {
            if !plugin {
                xp.inner.set_read_bytes(state::STATE, None);
            }
            let (rounds, _) = xp
                .rounds(|| async {
                    let (st, extra) =
                        state::read_with(&xp, Want(AUTOLAND_WANT_ILS), &["test/lever", "test/appr"])
                            .await;
                    assert!(!st.frozen());
                    assert!(st.position().is_ok() && extra.iter().all(|r| r.is_ok()));
                    let mut w = Writes::new();
                    w.set(VS_DIAL, -650.0)
                        .set(FLARE_PITCH, 0.12)
                        .set(FLARE_ROLL, -0.02)
                        .set(HDG_BUG, 131.2)
                        .set("test/lever", 0.4);
                    assert_eq!(w.flush(&xp).await.ok("test/lever"), Some(true));
                })
                .await;
            assert_eq!(rounds, 2, "plugin frame: {plugin}");
        }
    }
    // LEMG 13, 2026-09-12: the raw heading-minus-track fed straight back to
    // the heading bug hunted the final (crab sd 0.5° on the intercept, 4.0°
    // at the numbers, a ±9° swing with a ~13 s period) into the lateral
    // balk at 74 ft. The filter must pass the wind angle and eat the swing.
    /// LEMG 13, 2026-09-12, replayed through the real lateral law: a 172
    /// whose autopilot settles 2° left of its bug, flown from 3 nm on the
    /// centreline with the loop's own crab filter and tick. Second-order
    /// heading response, the tick's transport delay, the glidepath flown
    /// exactly so only the lateral answer is under test. Returns the
    /// cross-track (nm, right-positive) at 100 ft radio or at the balk,
    /// the guidance there and the trim carried.
    fn fly_final_with_heading_bias(bias_deg: f64, trim_on: bool) -> (f64, Guidance, f64) {
        let rwy = rwy_north();
        let gs_kts = 60.0;
        let dt = 0.6;
        // 3 nm south of the threshold, on the centreline, on the path.
        let (mut lat, mut lon) = project_latlon(0.0, 0.0, 180.0, 3.0);
        let mut psi: f64 = 0.0;
        let mut psi_dot: f64 = 0.0;
        let mut crab = super::CrabFilter::new();
        let mut trim = super::TrackTrim::new();
        let mut bug = 0.0;
        let mut trim_deg = 0.0;
        let mut g = guidance(
            lat, lon, 1000.0, 1000.0, gs_kts, 65.0, &rwy, 65.0, 25.0, 0.0,
        );
        for _ in 0..800 {
            let alt = g.glidepath_alt_ft;
            g = guidance(
                lat,
                lon,
                alt,
                alt - rwy.elev_ft,
                gs_kts,
                65.0,
                &rwy,
                65.0,
                25.0,
                0.0,
            );
            if alt - rwy.elev_ft < 100.0 || g.phase == ApproachPhase::Intercept {
                break;
            }
            let track = psi; // no wind: the bias is the only thing between bug and track
            let c = crab.update(super::CrabFilter::raw(Some(psi), Some(track)), dt);
            trim_deg = if trim_on {
                trim.update(g.phase, g.target_heading_deg, Some(track), dt)
            } else {
                0.0
            };
            bug = (g.target_heading_deg + c + trim_deg).rem_euclid(360.0);
            // The autopilot: second-order toward (bug + bias), ωn 0.5 rad/s, ζ 0.8.
            let err = crate::angles::signed_diff_deg(bug + bias_deg, psi);
            psi_dot += (0.25 * err - 2.0 * 0.8 * 0.5 * psi_dot) * dt;
            psi = (psi + psi_dot * dt).rem_euclid(360.0);
            let (nlat, nlon) = project_latlon(lat, lon, track, gs_kts * dt / 3600.0);
            lat = nlat;
            lon = nlon;
        }
        let _ = bug;
        (cross_track_nm(lat, lon, &rwy), g, trim_deg)
    }

    fn baron_flare_power() -> crate::aircraft::FlarePowerProfile {
        let profile: crate::aircraft::AircraftProfile =
            serde_yaml::from_str(include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/profiles/beechcraft_baron_58.yaml"))).unwrap();
        profile.flare_power.unwrap()
    }

    #[test]
    fn baron_recorded_flare_walks_the_power_off_and_never_cuts_it() {
        // LEBL 20, 2026-09-16. Replay measurements, NOT a flight-model
        // prediction: the old rule commanded zero throughout these sink rates.
        // Issue #76: the retard is timed, not held for the sink the arrest is
        // removing and let go in the float it made: the lever walks off at
        // the airframe's own rate from the flare gate, never a step.
        let mut power = FlarePower::default();
        let cfg = baron_flare_power();
        assert_eq!(cfg.retard_below_ft, None, "the Baron retards from the flare gate");
        let samples = [
            -393.65, -278.33, -148.51, -49.71, -24.33, -84.22, -211.0, -360.38, -515.28,
        ];
        let mut throttle = 0.30;
        let mut commands = Vec::new();
        for vs in samples {
            throttle = power
                .step(cfg, throttle, vs, 0.67, 10.0, Some(112.0), Some(110.0))
                .unwrap();
            commands.push(throttle);
        }
        assert_eq!(commands[0], 0.30, "no step to idle at flare entry");
        for w in commands.windows(2) {
            assert!(w[1] < w[0], "walking off whatever the sink: {commands:?}");
            assert!(
                w[0] - w[1] <= cfg.reduction_per_second * 0.67 + 1e-9,
                "never faster than the airframe's rate: {commands:?}"
            );
        }
        assert!(commands[8] > 0.0, "not cut: {commands:?}");
        // Slow (8 kt under the approach speed) and still sinking steadily:
        // the thrust stays, the sink it would cost is the wheels'.
        let mut power = FlarePower::default();
        for _ in 0..5 {
            let t = power
                .step(cfg, 0.30, -300.0, 0.67, 10.0, Some(102.0), Some(110.0))
                .unwrap();
            assert_eq!(t, 0.30);
        }
        assert_eq!(power.last_key, Some(RetardKey::Slow));
        // The recorded flare slow: it FLOATS (-393 to -278 in 0.67 s is a
        // float 1.5 s ahead, then -50 and -24), and slow never holds power
        // in a float (issue #146): the lever walks off and never comes back.
        let mut power = FlarePower::default();
        let mut throttle = 0.30;
        let mut slow = Vec::new();
        for vs in samples {
            throttle = power
                .step(cfg, throttle, vs, 0.67, 10.0, Some(102.0), Some(110.0))
                .unwrap();
            slow.push(throttle);
        }
        assert_eq!(slow[0], 0.30);
        for w in slow.windows(2) {
            assert!(w[1] < w[0], "{slow:?}");
        }
        assert!(slow[8] > 0.0, "not cut: {slow:?}");
        assert_ne!(power.last_key, Some(RetardKey::Slow));
    }

    #[test]
    fn flare_power_never_restarts_and_resets_for_a_new_approach() {
        let cfg = baron_flare_power();
        let mut power = FlarePower::default();
        let (ias, app) = (Some(110.0), Some(110.0));
        assert_eq!(power.step(cfg, 0.3, -50.0, 0.5, 10.0, ias, app), Some(0.3));
        assert_eq!(power.last_key, Some(RetardKey::Entry));
        assert!(power.step(cfg, 0.3, -150.0, 0.5, 10.0, ias, app).unwrap() < 0.3);
        // A balloon: power coming off, not held.
        assert!(power.step(cfg, 0.3, 100.0, 0.5, 10.0, ias, app).unwrap() < 0.3);
        // A lagging throttle read must not restart/increase the command.
        assert!(power.step(cfg, 0.4, -100.0, 0.5, 10.0, ias, app).unwrap() < 0.3);
        power.reset();
        assert_eq!(power.step(cfg, 0.45, -300.0, 0.5, 10.0, ias, app), Some(0.45));
        // An unread airspeed is no speed key: the retard goes on by time.
        assert!(power.step(cfg, 0.45, -300.0, 0.5, 10.0, None, app).unwrap() < 0.45);
    }

    #[test]
    fn flare_power_reaches_idle_smoothly_and_rejects_missing_measurements() {
        let cfg = baron_flare_power();
        let mut power = FlarePower::default();
        let (ias, app) = (Some(110.0), Some(110.0));
        let mut throttle = power.step(cfg, 0.3, -100.0, 0.5, 10.0, ias, app).unwrap();
        for _ in 0..30 {
            let next = power.step(cfg, throttle, -100.0, 0.5, 10.0, ias, app).unwrap();
            assert!((0.0..=throttle).contains(&next));
            assert!(throttle - next <= cfg.reduction_per_second * 0.5 + 1e-9);
            throttle = next;
        }
        assert_eq!(throttle, 0.0);
        assert_eq!(power.step(cfg, f64::NAN, -100.0, 0.5, 10.0, ias, app), None);
        assert_eq!(power.step(cfg, 0.3, f64::NAN, 0.5, 10.0, ias, app), None);
        assert_eq!(power.step(cfg, 0.3, -100.0, 0.0, 10.0, ias, app), None);
        assert_eq!(power.step(cfg, 0.3, -100.0, 0.5, f64::NAN, ias, app), None);
        let invalid = crate::aircraft::FlarePowerProfile {
            reduction_per_second: -1.0,
            ..cfg
        };
        assert_eq!(power.step(invalid, 0.3, -100.0, 0.5, 10.0, ias, app), None);
    }
    /// LEMG 13 (2026-09-26, 737): the arrest levelled at 20 ft, the sink-keyed
    /// retard took the lever to idle IN the float, and the sink came back to
    /// -859 fpm. A jet retards by height: nothing above it, and below it a
    /// ramp that reaches idle as the wheels arrive.
    /// LEMG 13, the approach test (2026-09-26): -300 by 29 ft, then the sink
    /// built back to -587 against a target still steeper than the aircraft.
    /// Held to the best sink reached, the law pulls as soon as it grows.
    #[test]
    fn the_flare_never_lets_the_sink_grow_again() {
        let mut law = FlarePitch::default();
        law.step(-450.0, -700.0, 0.0, 0.2, false);
        // Flare: reaches -300 while the target is -636.
        let a = law.step(-636.0, -300.0, 0.0, 0.2, true);
        // The sink grows to -400 with the target still -580: pull more.
        let b = law.step(-580.0, -400.0, -200.0, 0.2, true);
        assert!(b > a, "pulled for the growing sink: {a} -> {b}");
    }

    /// LEMG circuit (2026-09-26): the roundout's pull is where the flare
    /// starts. The flare's first ask (a path target the aircraft is already
    /// shallower than) must not take the column forward at the hand-over.
    #[test]
    fn the_flare_starts_from_the_roundouts_pull() {
        let mut law = FlarePitch::default();
        // Roundout: sinking 860 against 460 wanted, the pull builds.
        let mut pull = 0.0;
        for _ in 0..10 {
            pull = law.step(-460.0, -860.0, 0.0, 0.2, false);
        }
        assert!(pull > 0.2, "{pull}");
        // Flare: the target is the path (-759) and the sink already -690.
        let first = law.step(-759.0, -690.0, 200.0, 0.2, true);
        assert!(first >= pull - 1e-9, "the column held: {first} after {pull}");
    }

    /// EGLF 24, 2026-10-03, sortie 1791029430: the stock C172 arrested the
    /// sink at 21 ft, dropped 0.183 of elevator to neutral and cut 0.497 of
    /// approach power to idle. Energy and lift then vanished together and it
    /// touched at 436 fpm. Its airframe laws remove both continuously.
    #[test]
    fn the_c172_flare_does_not_drop_its_energy_or_column() {
        let profile = crate::aircraft::load("cessna_172sp").expect("C172 profile");
        let power_cfg = profile.flare_power.expect("continuous flare power");
        let elevator_cfg = profile.flare_elevator.expect("continuous elevator easing");

        let mut power = FlarePower::default();
        let mut lever = power
            .step(power_cfg, 0.497, -17.0, 0.2, 21.0, Some(65.0), Some(65.0))
            .unwrap();
        assert_eq!(lever, 0.497, "entry preserves the measured approach power");
        for _ in 0..10 {
            lever = power
                .step(power_cfg, lever, -20.0, 0.2, 20.0, Some(65.0), Some(65.0))
                .unwrap();
        }
        assert!((lever - 0.297).abs() < 1e-9, "two seconds removes 0.20, not all power: {lever}");

        let mut pitch = FlarePitch::with_gains(FlareGains::from_profile(Some(elevator_cfg)));
        let mut pull = 0.0;
        for _ in 0..8 {
            pull = pitch.step(-150.0, -300.0, 0.0, 0.2, true);
        }
        let eased = pitch.step(-150.0, -5.0, 300.0, 0.2, true);
        assert!(pull - eased <= 0.030_000_001, "one tick eased {pull} -> {eased}");
        assert!(eased > 0.0, "a predicted balloon must not drop the column to neutral");
    }

    /// LEMG 13 (2026-09-26): the 737 floated from 36 to 19 ft on a flat
    /// -175. Exponential with 4 s, it is the path at the gate, -475 at 20 ft,
    /// -325 at 10, and the touchdown sink at the wheels; undeclared, flat.
    #[test]
    fn the_737s_flare_is_exponential_and_the_rest_stay_flat() {
        let (flare_vs, nominal) = (-175.0, -700.0);
        let t = |h: f64| super::flare_target_with(h, flare_vs, nominal, Some(4.0));
        assert_eq!(t(35.0), -700.0, "the path at the gate, never steeper");
        assert!((t(20.0) + 475.0).abs() < 1.0, "{}", t(20.0));
        assert!((t(10.0) + 325.0).abs() < 1.0, "{}", t(10.0));
        assert!((t(0.0) + 175.0).abs() < 1.0, "{}", t(0.0));
        assert_eq!(super::flare_target_with(20.0, flare_vs, nominal, None), flare_vs);
        // Time from 35 ft: the float is gone (about 5.5 s, not 12).
        let (mut h, mut secs) = (35.0_f64, 0.0);
        while h > 0.5 {
            h += t(h) / 60.0 * 0.1;
            secs += 0.1;
        }
        assert!(secs < 7.0, "{secs:.1} s from 35 ft");
    }

    #[test]
    fn a_jet_retards_by_height_timed_to_the_touchdown() {
        let cfg = crate::aircraft::FlarePowerProfile {
            reduction_per_second: 0.08,
            retard_below_ft: Some(20.0),
            retard_max_per_s: None,
            retard_slow_kts: None,
            retard_at_s: None,
            retard_fast_kts: None,
            hold_from_s: None,
            retard_float_per_s: None,
        };
        let (ias, app) = (Some(145.0), Some(145.0));
        let mut power = FlarePower::default();
        // First tick keeps the approach power; at 36 ft in a float, still held.
        assert_eq!(power.step(cfg, 0.35, -170.0, 0.2, 36.0, ias, app), Some(0.35));
        assert_eq!(power.step(cfg, 0.35, -170.0, 0.2, 30.0, ias, app), Some(0.35));
        assert_eq!(power.last_key, Some(RetardKey::Height));
        // Below 20 ft at 170 fpm: about 7 s to the wheels, about 0.05 a second.
        let next = power.step(cfg, 0.35, -170.0, 1.0, 20.0, ias, app).unwrap();
        assert!((0.29..0.31).contains(&next), "{next}");
        assert_eq!(power.last_key, Some(RetardKey::Timed));
        // A normal sink closer in: a quicker retard, never past the rate.
        let mut quick = FlarePower::default();
        quick.step(cfg, 0.3, -300.0, 0.2, 10.0, ias, app);
        let next = quick.step(cfg, 0.3, -300.0, 1.0, 10.0, ias, app).unwrap();
        assert!((0.3 - 0.08 - 1e-9..0.3).contains(&next), "{next}");
        // Sinking 600 at 12 ft is the SAME retard as 300: the sink the arrest
        // is removing never holds the lever (issue #76), it only shortens
        // the time left.
        let mut hard = FlarePower::default();
        hard.step(cfg, 0.3, -600.0, 0.2, 12.0, ias, app);
        assert!(hard.step(cfg, 0.3, -600.0, 0.2, 12.0, ias, app).unwrap() < 0.3);
        // Slow: 6 kt under the approach speed keeps the thrust at any height.
        let mut slow = FlarePower::default();
        slow.step(cfg, 0.3, -200.0, 0.2, 10.0, Some(139.0), app);
        assert_eq!(slow.step(cfg, 0.3, -200.0, 0.2, 10.0, Some(139.0), app), Some(0.3));
        assert_eq!(slow.last_key, Some(RetardKey::Slow));
        // ...and the speed back, the retard resumes from where it stood.
        assert!(slow.step(cfg, 0.3, -200.0, 0.2, 9.0, Some(142.0), app).unwrap() < 0.3);
    }

    #[test]
    fn a_heading_bias_on_final_parks_beside_the_centreline_without_trim() {
        // The flight, in the plant: 2° left of the bug settles some 40 m
        // left and never closes, and a lateral gate balks it — the go-around
        // the pilot watched twice.
        let (xtk, g, _) = fly_final_with_heading_bias(-2.0, false);
        assert_eq!(
            g.phase,
            ApproachPhase::Intercept,
            "{:.0} m off",
            xtk * super::M_PER_NM
        );
        assert!(
            matches!(
                g.balk,
                Some("off_centreline_low" | "unstabilised_below_floor")
            ),
            "{:?}",
            g.balk
        );
        assert!(
            xtk < -0.015 && xtk > -0.035,
            "expected the standing offset the flight showed, got {:.0} m",
            xtk * super::M_PER_NM
        );
    }

    #[test]
    fn the_track_trim_takes_a_biased_final_to_the_centreline() {
        for bias in [-2.0, 2.0, -1.0] {
            let (xtk, g, trim) = fly_final_with_heading_bias(bias, true);
            assert_eq!(g.phase, ApproachPhase::Final, "bias {bias}: {:?}", g.balk);
            assert!(
                xtk.abs() < 0.005,
                "bias {bias}: still {:.0} m off at 100 ft",
                xtk * super::M_PER_NM
            );
            assert!(
                (trim + bias).abs() < 0.6,
                "bias {bias}: the trim should carry the bias, carries {trim:.2}"
            );
        }
        // No bias: nothing to trim, and the trim invents nothing.
        let (xtk, _, trim) = fly_final_with_heading_bias(0.0, true);
        assert!(
            xtk.abs() < 0.003 && trim.abs() < 0.3,
            "xtk {xtk} trim {trim}"
        );
    }

    #[test]
    fn the_track_trim_is_final_only_and_never_a_turn() {
        let mut t = super::TrackTrim::new();
        // A 30° track error is a turn in progress: not integrated.
        assert_eq!(t.update(ApproachPhase::Final, 90.0, Some(60.0), 1.0), 0.0);
        // The first read of a track measures no rate yet, and two steady
        // seconds are not yet settled: nothing learnt.
        let mut t = super::TrackTrim::new();
        assert_eq!(t.update(ApproachPhase::Final, 90.0, Some(88.0), 3.0), 0.0);
        assert_eq!(t.update(ApproachPhase::Final, 90.0, Some(88.0), 2.0), 0.0);
        // Settled, a steady 2° error integrates at 2/30 per second.
        let v = t.update(ApproachPhase::Final, 90.0, Some(88.0), 3.0);
        assert!((v - 0.2).abs() < 1e-9, "{v}");
        // Held through the flare, whatever the track does.
        assert_eq!(t.update(ApproachPhase::Flare, 90.0, Some(80.0), 5.0), v);
        // A missing track read holds the value.
        assert_eq!(t.update(ApproachPhase::Final, 90.0, None, 5.0), v);
        // Wraps: 359 wanted, 1 flown is -2, not +358.
        let mut w = super::TrackTrim::new();
        for _ in 0..2 {
            w.update(ApproachPhase::Final, 359.0, Some(1.0), 3.0);
        }
        assert!(w.update(ApproachPhase::Final, 359.0, Some(1.0), 3.0) < 0.0);
        // A 5 degree error is a roll-out still converging, not a bias.
        let before = t.value();
        assert_eq!(
            t.update(ApproachPhase::Final, 90.0, Some(85.0), 1.0),
            before
        );
        // Capped, and reset by the intercept (a go-around starts clean).
        for _ in 0..10_000 {
            t.update(ApproachPhase::Final, 90.0, Some(87.5), 1.0);
        }
        assert_eq!(t.value(), super::TRACK_TRIM_MAX_DEG);
        assert_eq!(
            t.update(ApproachPhase::Intercept, 90.0, Some(85.0), 1.0),
            0.0
        );
    }

    /// Issue #189: LEMG 30 A330, sortie 1790710769-full_auto (2026-09-29),
    /// autoland's first 20 s after the circuit's handoff (`autoland` ticks:
    /// seconds, ground track, the guidance's wanted track). Calm air, the
    /// aircraft on its bug: the roll-out off the circuit's intercept taught
    /// the old trim -0.51 degrees in ten seconds (recorded), which held the
    /// final 17 m left and went around at 99 ft. A turning track teaches it
    /// nothing now; the steady error after the roll-out is still learnt.
    #[test]
    fn the_track_trim_learns_no_roll_out() {
        let ticks = [
            (2.0, 301.33, 298.52), (3.3, 301.27, 298.36), (3.8, 301.17, 298.30),
            (4.4, 301.01, 298.24), (5.0, 300.78, 298.17), (5.5, 300.60, 298.13),
            (6.0, 300.37, 298.09), (6.4, 300.10, 298.04), (7.0, 299.80, 298.00),
            (7.6, 299.46, 297.96), (8.0, 299.20, 297.93), (8.4, 298.97, 297.91),
            (8.8, 298.71, 297.89), (9.1, 298.60, 297.89), (9.6, 298.31, 297.87),
            (10.0, 298.10, 297.86), (10.6, 297.87, 297.86), (11.2, 297.66, 297.86),
            (11.6, 297.55, 297.86), (12.1, 297.43, 297.86), (12.5, 297.36, 297.86),
            (13.0, 297.30, 297.87), (13.4, 297.27, 297.87), (13.9, 297.25, 297.88),
            (14.3, 297.25, 297.88), (14.7, 297.26, 297.89), (15.3, 297.28, 297.89),
            (15.6, 297.31, 297.90), (16.1, 297.34, 297.90), (16.5, 297.37, 297.90),
            (17.1, 297.40, 297.91), (17.5, 297.41, 297.91), (17.9, 297.42, 297.91),
            (18.4, 297.43, 297.92), (18.9, 297.42, 297.92), (19.4, 297.40, 297.92),
            (19.8, 297.38, 297.93), (20.3, 297.35, 297.93),
        ];
        let mut t = super::TrackTrim::new();
        // The old rule, the error cap alone, for the record.
        let mut old = 0.0_f64;
        let mut prev = 0.0;
        let mut at_10 = (0.0, 0.0);
        for (i, (s, trk, want)) in ticks.iter().enumerate() {
            let dt = if i == 0 { 0.45 } else { s - prev };
            prev = *s;
            let v = t.update(ApproachPhase::Final, *want, Some(*trk), dt);
            let err = crate::angles::signed_diff_deg(*want, *trk);
            if err.abs() <= super::TRACK_TRIM_ERR_CAP_DEG {
                old += err * dt / super::TRACK_TRIM_TAU_S;
            }
            if (*s - 10.0).abs() < 0.05 {
                at_10 = (v, old);
            }
        }
        assert!(at_10.1 < -0.3, "the old rule learns the roll-out: {at_10:?}");
        assert!(at_10.0.abs() < 0.05, "a turning track teaches nothing: {at_10:?}");
        // Rolled out 0.5 to 0.6 degrees left of what it wants: learnt RIGHT.
        assert!(t.value() > 0.02, "{}", t.value());
    }

    #[test]
    fn crab_filter_settles_on_a_steady_crosswind() {
        let mut f = super::CrabFilter::new();
        let mut out = 0.0;
        for _ in 0..90 {
            out = f.update(Some(6.0), 0.5);
        }
        assert!((out - 6.0).abs() < 0.01, "45 s in, got {out}");
        // A fresh filter takes its first read whole — no 15 s ramp from zero
        // on the tick the final begins.
        let mut g = super::CrabFilter::new();
        assert_eq!(g.update(Some(6.0), 0.5), 6.0);
    }

    #[test]
    fn crab_filter_eats_the_lemg_13_hunt() {
        let mut f = super::CrabFilter::new();
        let mut worst = 0.0_f64;
        for i in 0..600 {
            let t = i as f64 * 0.5;
            let raw = 2.0 + 9.0 * (std::f64::consts::TAU * t / 13.0).sin();
            let out = f.update(Some(raw), 0.5);
            if t > 60.0 {
                worst = worst.max((out - 2.0).abs());
            }
        }
        assert!(worst < 2.0, "the swing came through at {worst:.1} degrees");
        assert!(
            worst > 0.5,
            "a filter that passes nothing is a constant, got {worst:.2}"
        );
    }

    #[test]
    fn crab_filter_holds_through_a_missed_read() {
        let mut f = super::CrabFilter::new();
        for _ in 0..90 {
            f.update(Some(6.0), 0.5);
        }
        assert!(
            (f.update(None, 0.5) - 6.0).abs() < 0.01,
            "a missed read is not a zero crab"
        );
        assert_eq!(
            super::CrabFilter::new().update(None, 0.5),
            0.0,
            "never read = zero"
        );
        // Wrap-safe: 359 vs 3 is a 4 degree crab, not 356.
        assert!((super::CrabFilter::raw(Some(359.0), Some(3.0)).unwrap() + 4.0).abs() < 1e-9);
        assert_eq!(super::CrabFilter::raw(Some(90.0), None), None);
    }

    #[test]
    fn a_clearance_for_another_end_differs_and_blanks_do_not() {
        assert!(super::clearance_differs("12", "13"));
        assert!(!super::clearance_differs("12", " 12 "));
        assert!(!super::clearance_differs("09R", "09r"));
        assert!(!super::clearance_differs("12", ""));
    }

    /// LXGB 2026-09-14: "final runway two seven" earned "negative, runway
    /// zero niner ... cleared to land" five seconds after the arm-time pick,
    /// and nothing looked again.
    #[test]
    fn a_landing_clearance_for_another_end_moves_the_approach_once() {
        use crate::tower::State::*;
        let sw = |f, st, c, a| super::clearance_switch(f, st, c, a, false, None);
        assert_eq!(
            sw("27", Some(ClearedToLand), Some("09"), None),
            Some("09".into())
        );
        // EGTE 2026-09-21: the go-around's own state change published the
        // flipped runway in use. A missed approach is never re-aimed.
        assert_eq!(
            super::clearance_switch("08", Some(Inbound), Some("26"), None, true, None),
            None
        );
        // Told to join the other end on the inbound call: same switch, earlier.
        assert_eq!(sw("27", Some(Inbound), Some("09"), None), Some("09".into()));
        // Looked up once: the apt.dat pass is not repeated every tick.
        assert_eq!(sw("27", Some(ClearedToLand), Some("09"), Some("09")), None);
        assert_eq!(sw("27", Some(ClearedToLand), Some("09"), Some("09 ")), None);
        // Our own end, a blank, or no clearance at all: nothing to do.
        assert_eq!(sw("27", Some(ClearedToLand), Some("27"), None), None);
        assert_eq!(sw("27", Some(ClearedToLand), Some(""), None), None);
        assert_eq!(sw("27", Some(ClearedToLand), None, None), None);
        // A departure's runway is the departure's, whatever end it names.
        assert_eq!(sw("27", Some(TakeoffCleared), Some("09"), None), None);
        assert_eq!(sw("27", Some(TaxiCleared), Some("09"), None), None);
        assert_eq!(sw("27", None, Some("09"), None), None);
    }

    /// #182, LEMG 2026-09-29 (sortie 1790668215-full_auto): at t+649.4 the
    /// tower sent the 737 round off 30 ("go around, traffic on the runway
    /// ... expect runway one three", atc `resequenced` 30 to 13, oncoming),
    /// and the FSM sync published 13 with `Inbound`. The missed approach
    /// read no switch; the reposition flew a right-hand pattern for 30 from
    /// t+661.3 and found 13 only at t+984.6, once `reversal_done` ended the
    /// missed approach. The order's own runway is a re-sequence: taken once
    /// the climb away is flown, and only when the clearance agrees with it.
    #[test]
    fn a_go_around_order_that_resequences_moves_the_approach_after_the_climb() {
        use crate::tower::State::{ClearedToLand, Inbound};
        // Past the climb (the caller passes the order's runway): 13 now.
        assert_eq!(
            super::clearance_switch("30", Some(Inbound), Some("13"), None, true, Some("13")),
            Some("13".into())
        );
        // In the climb, or with an order that named none (EGTE's runway in
        // use flipping under a go-around): the missed approach is flown.
        assert_eq!(
            super::clearance_switch("30", Some(Inbound), Some("13"), None, true, None),
            None
        );
        // The clearance must agree with the order: a runway in use that has
        // moved on again is not what the tower told us to expect.
        assert_eq!(
            super::clearance_switch("30", Some(Inbound), Some("12"), None, true, Some("13")),
            None
        );
        // Looked up once, and never onto a departure's runway.
        assert_eq!(
            super::clearance_switch(
                "30",
                Some(Inbound),
                Some("13"),
                Some("13"),
                true,
                Some("13")
            ),
            None
        );
        assert_eq!(
            super::clearance_switch(
                "30",
                Some(crate::tower::State::TakeoffCleared),
                Some("13"),
                None,
                true,
                Some("13")
            ),
            None
        );
        // Already flying the expected end: nothing to do.
        assert_eq!(
            super::clearance_switch(
                "13",
                Some(ClearedToLand),
                Some("13"),
                None,
                true,
                Some("13")
            ),
            None
        );
        // The latch says when the climb is flown: straight ahead to 800 ft
        // radio, then the reposition (the recorded 748 ft order, 856 ft at
        // the pattern's start).
        let mut latch = super::GoAroundLatch::new();
        assert!(latch.order(false));
        assert!(latch.climbing());
        latch.update(
            Some(ApproachPhase::Final),
            ApproachPhase::Intercept,
            748.0,
            1.53,
            1.53,
            5.75,
            0.008,
        );
        assert!(latch.climbing(), "still climbing away at 748 ft");
        latch.update(
            Some(ApproachPhase::Intercept),
            ApproachPhase::Intercept,
            856.0,
            1.0,
            1.0,
            5.75,
            0.009,
        );
        assert!(
            !latch.climbing() && latch.active(),
            "repositioning: the re-sequence is taken"
        );
    }

    #[test]
    fn only_the_landing_echo_is_the_approachs_to_make() {
        assert!(super::landing_readback(
            "Cleared to land runway 09, November one seven two Sierra Papa."
        ));
        assert!(super::landing_readback(
            "Cleared touch and go runway 09, N172SP."
        ));
        assert!(!super::landing_readback(
            "Cleared for takeoff runway 09, N172SP."
        ));
        assert!(!super::landing_readback("Holding position, N172SP."));
        assert!(!super::landing_readback("Lining up runway 09, N172SP."));
    }

    use super::*;

    #[test]
    fn rollout_requires_weight_on_wheels() {
        // A normal flare can be below five feet and slightly below target;
        // neither airspeed nor radio altitude is proof of touchdown.
        assert!(!should_start_rollout(false, false, 3.0, 70.0, 89.0, 90.0, None));
        assert!(!should_start_rollout(false, false, 3.0, 40.0, 89.0, 90.0, None));
        // The same committed state starts rollout once the wheels are down,
        // including a tailwind landing whose groundspeed remains high.
        assert!(should_start_rollout(false, true, 3.0, 70.0, 89.0, 90.0, None));
        assert!(should_start_rollout(false, true, 3.0, 40.0, 95.0, 90.0, None));
        // Once latched, a noisy WOW sample cannot return control to approach.
        assert!(should_start_rollout(true, false, 10.0, 70.0, 95.0, 90.0, None));
    }

    /// LEMG 12 (2026-09-26, 17:52Z, a 737, approach speed 145): the guidance
    /// ticks from the last airborne one to the balk, as recorded (t from
    /// 1790448340.0, the sim's on_ground, radio ft, IAS; 146 kt over the
    /// ground throughout). The speed rule never latched the rollout, the
    /// flare ran 2.8 s on the wheels, and at 3.3 s `overflew_the_runway`
    /// balked from 0.33 ft into a full-power go-around and a "going around"
    /// call to the tower.
    #[test]
    fn the_wheels_are_a_landing_and_nothing_balks_from_the_pavement() {
        const TICKS: &[(f64, bool, f64, f64)] = &[
            (0.0, false, 3.06, 147.42),
            (0.3, true, 0.97, 147.34),
            (0.5, true, -0.01, 147.28),
            (0.7, true, 0.33, 147.21),
            (1.0, true, 0.90, 147.08),
            (1.2, true, 0.94, 146.99),
            (1.4, true, 0.58, 146.89),
            (1.7, true, 0.28, 146.73),
            (1.9, true, 0.38, 146.60),
            (2.2, true, 0.37, 146.45),
            (2.4, true, 0.30, 146.29),
            (2.6, true, 0.32, 146.12),
            (2.8, true, 0.35, 145.95),
            (3.1, true, 0.32, 145.65),
            (3.3, true, 0.33, 145.46),
        ];
        let (gs, approach) = (146.0, 145.0);
        // (1) The old rule: not one tick latched.
        assert!(TICKS.iter().all(|&(_, g, r, ias)| !should_start_rollout(false, g, r, gs, ias, approach, None)));
        // The wheels: latched 0.4 s after first contact (the third tick on
        // the ground at this 0.2 s tick).
        let mut n = None;
        let mut last_t: Option<f64> = None;
        let latched = TICKS.iter().find(|&&(t, g, r, ias)| {
            let dt = last_t.map_or(0.0, |t0| t - t0);
            last_t = Some(t);
            n = wheels_on_time(n, g, r, dt);
            should_start_rollout(false, g, r, gs, ias, approach, n)
        });
        assert_eq!(latched.map(|t| t.0), Some(0.7), "the rollout latches at the wheels");
        // A bounce breaks the clock.
        assert_eq!(wheels_on_time(Some(0.2), false, 4.0, 0.2), None);
        assert_eq!(wheels_on_time(Some(0.2), true, 6.0, 0.2), None, "a wheel brushing at 6 ft is not down");
        // The same 0.4 s on a slower tick: 0.39 s ticks latch on the
        // second read after contact, not the third (0.78 s).
        let mut s = wheels_on_time(None, true, 0.5, 0.39);
        assert!(!should_start_rollout(false, true, 0.5, gs, 147.0, approach, s));
        s = wheels_on_time(s, true, 0.5, 0.39);
        assert!(!should_start_rollout(false, true, 0.5, gs, 147.0, approach, s));
        s = wheels_on_time(s, true, 0.5, 0.39);
        assert!(should_start_rollout(false, true, 0.5, gs, 147.0, approach, s));

        // (2) The balk tick as the guidance produced it.
        let balked = Guidance {
            target_heading_deg: 117.71,
            target_vs_fpm: 2000.0,
            target_ias_kts: 150.0,
            dist_nm: 0.61,
            glidepath_alt_ft: 292.3,
            phase: ApproachPhase::Intercept,
            balk: Some("overflew_the_runway"),
            arresting: false,
        };
        let (held, refused) = wheels_hold_the_landing(balked, true, true, 117.3);
        assert_eq!(refused, Some("overflew_the_runway"));
        assert_eq!((held.phase, held.balk), (ApproachPhase::Flare, None));
        assert!((held.target_heading_deg - 117.3).abs() < 1e-9, "down the runway's course");
        // The go-around latch never arms off it; off the raw balk it did.
        let mut latch = GoAroundLatch::new();
        assert!(latch
            .update(Some(ApproachPhase::Flare), held.phase, 0.33, 0.61, -0.61, FAP_NM, 0.0)
            .is_none());
        let mut latch = GoAroundLatch::new();
        assert_eq!(
            latch.update(Some(ApproachPhase::Flare), balked.phase, 0.33, 0.61, -0.61, FAP_NM, 0.0),
            Some(MissedStage::Climb),
            "the recorded go-around"
        );
        // Airborne, the balk stands (an overflight is still a go-around).
        let (air, none) = wheels_hold_the_landing(balked, false, true, 117.3);
        assert_eq!((air.phase, air.balk, none), (ApproachPhase::Intercept, Some("overflew_the_runway"), None));
        // A flare that stays a flare on the ground is untouched.
        let flare = Guidance { phase: ApproachPhase::Flare, balk: None, arresting: true, ..balked };
        assert_eq!(wheels_hold_the_landing(flare, true, true, 117.3).1, None);

        // (4) No "going around" call with the wheels on the runway.
        assert!(!go_around_call_due(true, false, true, None, 0.3, None));
        assert!(go_around_call_due(true, false, false, None, 300.0, None));
        assert!(!go_around_call_due(true, true, false, None, 300.0, None), "once");
        assert!(!go_around_call_due(false, false, false, None, 300.0, None));
    }

    /// LFBD 11, 2026-09-27: the 737 balked `overflew_the_runway` at 7.9 ft
    /// radio sinking 184 fpm, "going around" went out on that tick and the
    /// tower dropped the landing clearance, and the wheels touched 1.3 s
    /// later. The call now waits for the climb; a balk the wheels take never
    /// calls. The first approach's `off_centreline_low` at 96 ft still calls
    /// as soon as it climbs.
    /// EGKK-EGBB 15, sortie 1790601822-full_auto (2026-09-28, #155): the
    /// final report at +3314.2 got "continue approach, expect late landing
    /// clearance" and the tower never came back; the book stayed Inbound down
    /// the whole final (the recorded autoland ticks' radio heights below,
    /// +3313.6 to +3452.0) and the wheels touched uncleared at +3472.2. The
    /// crew now asks at 1,000 ft (the +3390.8 tick, 990 ft) and goes around at
    /// 300 (the +3435.7 tick, 296 ft), once each; cleared, or with no tower
    /// working us, it says nothing; a missed approach asks again next time.
    #[test]
    fn an_uncleared_final_asks_then_goes_around() {
        let radio = [
            2103.0, 2097.0, 1991.0, 1886.0, 1796.0, 1692.0, 1593.0, 1499.0, 1394.0, 1298.0,
            1200.0, 1096.0, 990.0, 893.0, 795.0, 694.0, 596.0, 499.0, 396.0, 296.0, 199.0, 95.0,
        ];
        let run = |working: bool| {
            let mut w = UnclearedWatch::default();
            radio
                .iter()
                .filter_map(|&r| w.step(working, r, false, false).map(|s| (r, s)))
                .collect::<Vec<_>>()
        };
        let steps = run(true);
        assert_eq!(steps.first(), Some(&(990.0, UnclearedStep::Ask)));
        assert_eq!(steps.get(1), Some(&(296.0, UnclearedStep::GoAround)));
        assert_eq!(
            steps.iter().filter(|s| s.1 == UnclearedStep::Ask).count(),
            1,
            "asked once: {steps:?}"
        );
        // Cleared (or no tower working us): silent all the way down.
        assert!(run(false).is_empty());
        // Never on the wheels.
        let mut w = UnclearedWatch::default();
        assert_eq!(w.step(true, 0.0, true, false), None);
        // A missed approach resets the question for the next approach.
        let mut w = UnclearedWatch::default();
        assert_eq!(w.step(true, 900.0, false, false), Some(UnclearedStep::Ask));
        assert_eq!(w.step(true, 900.0, false, false), None);
        assert_eq!(w.step(true, 250.0, false, true), None);
        assert_eq!(w.step(true, 900.0, false, false), Some(UnclearedStep::Ask));
        // An unread height is no measurement.
        assert_eq!(UnclearedWatch::default().step(true, f64::NAN, false, false), None);
    }

    #[test]
    fn a_low_balk_is_called_on_the_radio_only_once_it_climbs() {
        let due = |w: &BalkWatch, on_ground, radio, vvi| {
            go_around_call_due(true, false, on_ground, Some(w), radio, vvi)
        };
        // The recorded balk: 7.9 ft, sinking, spooling.
        let mut w = BalkWatch::new(Some("overflew_the_runway"), 7.9);
        assert!(!due(&w, false, 7.9, Some(-184.0)), "not on the balk tick");
        w.note(4.0, false);
        assert!(!due(&w, false, 4.0, Some(-150.0)));
        // The wheels take it: never called, whatever follows under 200 ft.
        w.note(0.4, true);
        assert!(!due(&w, true, 0.4, Some(-60.0)), "on the runway");
        w.note(15.0, false);
        assert!(!due(&w, false, 15.0, Some(300.0)), "a bounce is not a go-around");
        // A touch that really leaves is called once clear of 200 ft.
        w.note(GO_AROUND_CALL_AGL_FT, false);
        assert!(due(&w, false, GO_AROUND_CALL_AGL_FT, Some(900.0)));

        // The same balk flown away without touching: called on the climb.
        let mut w = BalkWatch::new(Some("overflew_the_runway"), 7.9);
        w.note(3.0, false);
        assert!(!due(&w, false, 3.0 + GO_AROUND_RISE_FT - 1.0, Some(200.0)), "not yet risen");
        w.note(3.0 + GO_AROUND_RISE_FT, false);
        assert!(due(&w, false, 3.0 + GO_AROUND_RISE_FT, Some(400.0)));
        assert!(!due(&w, false, 3.0 + GO_AROUND_RISE_FT, Some(-50.0)), "the VSI says sinking");
        assert!(due(&w, false, 3.0 + GO_AROUND_RISE_FT, None), "no VSI: the radio's rise stands");
        assert!(!go_around_call_due(true, true, false, Some(&w), 40.0, Some(400.0)), "once");

        // The first LFBD approach: off_centreline_low at 96 ft, settling to
        // 88 before the power bites, called as soon as it climbs.
        let mut w = BalkWatch::new(Some("off_centreline_low"), 96.0);
        w.note(88.0, false);
        assert!(!due(&w, false, 88.0, Some(-300.0)));
        w.note(99.0, false);
        assert!(due(&w, false, 99.0, Some(500.0)));
        // A balk above the call height is called on its own tick.
        let w = BalkWatch::new(Some("unstabilised_at_gate"), 1000.0);
        assert!(due(&w, false, 1000.0, Some(-700.0)));
    }

    /// The rollout brakes by wanted deceleration, not a fixed clamp. A flat
    /// 0.45 measured ~3.3 m/s² from 79 kt to a dead stop — "brakes really
    /// really hard" from the right seat (EGHI, 2026-08-02) — on a runway with
    /// over a kilometre to spare.
    #[test]
    fn rollout_brakes_for_comfort_unless_the_runway_disagrees() {
        // Plenty of pavement: the comfort rate, well under the old 0.45.
        let easy = rollout_brake_ratio(79.0, 0.0, 1300.0);
        assert!((0.2..0.3).contains(&easy), "{easy}");
        // Short remaining: physics outranks comfort.
        let tight = rollout_brake_ratio(79.0, 0.0, 300.0);
        assert!(tight > 0.5, "{tight}");
        // ...and the cap holds even when the sums say more.
        assert_eq!(rollout_brake_ratio(120.0, 0.0, 80.0), 0.8);
        // Slowing to a vacate handoff needs less than slowing to a stop.
        assert!(
            rollout_brake_ratio(40.0, VACATE_HANDOFF_KTS, 500.0)
                <= rollout_brake_ratio(40.0, 0.0, 500.0)
        );
        // Near the target the floor still holds a gentle squeeze on.
        assert!(rollout_brake_ratio(12.0, 10.0, 800.0) >= 0.2);
    }

    // Gibraltar 2026-09-11: the only lateral balk was the flare gate under
    // 100 ft, so a final that never converged flew to 95 ft still 306 m off
    // the centreline, went around with no height in hand, and hit the North
    // Front. Below the terrain floor, off the course is a go-around.
    #[test]
    fn an_unstabilised_final_balks_with_height_in_hand() {
        let rwy = rwy_north();
        // The crash geometry at the floor: 1.5 nm out, 480 ft, 0.9 nm right.
        let (clat, clon) = project_latlon(0.0, 0.0, 180.0, 1.5);
        let (lat, lon) = project_latlon(clat, clon, 90.0, 0.9);
        let g = guidance(lat, lon, 480.0, 480.0, 90.0, 100.0, &rwy, 100.0, 25.0, 0.0);
        assert_eq!(
            g.phase,
            ApproachPhase::Intercept,
            "balk at the floor, not at the flare gate"
        );
        // A normal final 40 m off at the same point: still on final.
        let (lat, lon) = project_latlon(clat, clon, 90.0, 0.02);
        let g = guidance(lat, lon, 480.0, 480.0, 90.0, 100.0, &rwy, 100.0, 25.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Final);
        // Above the floor the same geometry is not a balk but a REPOSITION:
        // steer out to the join point instead of calling it final. The RV-10
        // "Land the aircraft" at 1.5 nm abeam, 660 ft (2026-09-11).
        let (clat, clon) = project_latlon(0.0, 0.0, 180.0, 1.06);
        let (lat, lon) = project_latlon(clat, clon, 90.0, 1.06);
        let g = guidance(lat, lon, 660.0, 660.0, 120.0, 120.0, &rwy, 80.0, 25.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Intercept, "not a final from there");
        // The join point lies down the extended centreline, south of us and
        // west: a heading in the south-west quadrant, never the runway's.
        assert!(
            (160.0..=260.0).contains(&g.target_heading_deg),
            "{}",
            g.target_heading_deg
        );

        // The rule itself: the offset the steepest cut can still close with
        // margin in the distance left. No opinion outside the FAP (an
        // intercept is allowed to be wide out there).
        assert!(!stabilised_xtk(0.9, 1.5), "the Gibraltar handoff");
        assert!(stabilised_xtk(0.10, 1.5));
        assert!(stabilised_xtk(0.2, 0.8), "the pursuit flies this one out");
        assert!(!stabilised_xtk(0.2, 0.3));
    }

    /// The flare is entered from a roundout, never from a level float. Past
    /// the threshold the unsigned glidepath distance grows again, alt_err
    /// goes negative, and the level-hold branch commanded 0 fpm at 27 ft over
    /// the runway: the aircraft floated, bled the speed the flare needed, and
    /// dropped on at -380 fpm one knot above the clean stall (EGHI,
    /// 2026-08-02).
    #[test]
    fn the_flare_is_entered_from_a_roundout_not_a_float() {
        let mut rwy = rwy_north();
        // Short final, over the pavement environment, just above the flare
        // gate (radio 40 ft, flare 25, band 50): a gentle continued descent.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 0.2);
        let g = guidance(lat, lon, 40.0, 40.0, 95.0, 95.0, &rwy, 95.0, 25.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Final);
        let (roundout, _) = arrest_targets_fpm(-(95.0 / 60.0) * GLIDEPATH_FT_PER_NM);
        assert_eq!(g.target_vs_fpm, roundout);

        // The live float case: just past the threshold (inside the overflight
        // margin), 30 ft radio — alt_err is negative here and the old law
        // held LEVEL. It must keep descending.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 0.05);
        let g = guidance(lat, lon, 30.0, 30.0, 93.0, 93.0, &rwy, 95.0, 25.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Final);
        // The roundout's target scales with the path sink (a 95 kt approach
        // sinks 500 fpm, so its roundout is 300, not the 172's 250).
        let (roundout, _) = arrest_targets_fpm(-(93.0 / 60.0) * GLIDEPATH_FT_PER_NM);
        assert!(
            roundout < ROUNDOUT_VS_FPM && roundout > -350.0,
            "{roundout}"
        );
        assert_eq!(
            g.target_vs_fpm, roundout,
            "a level float at 30 ft is how the mush happened"
        );

        // Below the flare gate the flare still owns the touchdown.
        let g = guidance(lat, lon, 20.0, 20.0, 92.0, 92.0, &rwy, 95.0, 25.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Flare);
        let (_, flare) = arrest_targets_fpm(-(92.0 / 60.0) * GLIDEPATH_FT_PER_NM);
        assert!(flare < FLARE_VS_FPM && flare > -150.0, "{flare}");
        assert_eq!(g.target_vs_fpm, flare);

        // Dragged LOW short of the threshold the roundout must NOT own it:
        // from 30 ft at 0.3 nm its -250 grounds ~190 m before the pavement.
        // The level-hold holds, and the path comes down to us — the guard is
        // the physical reach test, not the distance band (audit, 2026-08-02).
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 0.3);
        let g = guidance(lat, lon, 30.0, 30.0, 100.0, 100.0, &rwy, 95.0, 25.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Final);
        assert_eq!(
            g.target_vs_fpm, 0.0,
            "a commanded descent here lands in the approach lights"
        );

        // And a genuine overflight go-around is untouched by the roundout:
        // 150 ft over the pavement 0.3 nm in, where a 3 degree descent
        // reaches the band 550 m further on. On this 3,000 m test runway
        // that still leaves the room a landing needs (the rule is
        // geometric now), so the strip is 1,800 m here: the band would be
        // reached 1,100 m in with 700 m left, and that IS an overflight.
        rwy.length_m = 1800.0;
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 0.3);
        let g = guidance(lat, lon, 150.0, 150.0, 95.0, 95.0, &rwy, 95.0, 25.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Intercept);
        assert_eq!(g.balk, Some("overflew_the_runway"));
    }

    /// LXGB 27, 2026-09-14, replayed through the vertical law against the
    /// autopilot it was talking to: a VS mode that follows its dial as a
    /// first-order lag of 4.5 s behind one tick of dead time, and the flare's
    /// throttle cut as a sink disturbance of 250 fpm arriving over 1.5 s that
    /// the same lag has to reject. Both fitted to the tick log: -250 dialled
    /// at 49 ft, still -386 four seconds on; -100 dialled at 23 ft with the
    /// throttle closed, the sink eased to -257 and then GREW back to -309 at
    /// the wheels. A pure lag without the disturbance lands this at -140 and
    /// would have passed the old law. Flies from 60 ft radio just short of
    /// the numbers at the 172's approach sink, the dial either the bare
    /// target (the live sortie) or led through [`vs_dial_fpm`]. Returns the
    /// sink rate at the wheels, the shallowest rate seen (a balloon would be
    /// positive) and the seconds it took.
    fn fly_arrest_through_a_lagging_autopilot(lead: bool) -> (f64, f64, f64) {
        let rwy = rwy_north();
        let dt = 0.6;
        let tau_s = 4.5;
        let idle_sink_fpm = -250.0;
        let idle_tau_s = 1.5;
        let gs_kts = 75.0;
        let mut radio = 60.0;
        let mut vs = -400.0;
        let mut pending_dial = vs; // one tick of dead time on the dial
        let mut idle_sink = 0.0;
        let mut along_nm = -0.12; // short of the threshold, on the centreline
        let mut shallowest: f64 = vs;
        let mut t = 0.0;
        let mut saw_roundout = false;
        let mut saw_flare = false;
        loop {
            let (lat, lon) = if along_nm < 0.0 {
                project_latlon(0.0, 0.0, 180.0, -along_nm)
            } else {
                project_latlon(0.0, 0.0, 0.0, along_nm)
            };
            let g = guidance(lat, lon, radio, radio, gs_kts, 65.0, &rwy, 65.0, 25.0, 0.0);
            assert_ne!(
                g.phase,
                ApproachPhase::Intercept,
                "balked at {radio:.0} ft: {:?}",
                g.balk
            );
            saw_roundout |= g.phase == ApproachPhase::Final && g.arresting;
            saw_flare |= g.phase == ApproachPhase::Flare;
            let dial = if lead {
                vs_dial_fpm(g.target_vs_fpm, vs, 0.0, g.arresting, Default::default())
            } else {
                g.target_vs_fpm
            };
            let applied = pending_dial;
            pending_dial = dial;
            // The throttle closes at the flare (the loop's own rule) and the
            // extra sink enters the state; the autopilot only sees it as
            // vertical-speed error and works it off through its lag.
            let want_sink = if g.phase == ApproachPhase::Flare {
                idle_sink_fpm
            } else {
                0.0
            };
            let d_sink = (want_sink - idle_sink) * (dt / idle_tau_s);
            idle_sink += d_sink;
            vs += (applied - vs) * (dt / tau_s) + d_sink;
            shallowest = shallowest.max(vs);
            radio += vs / 60.0 * dt;
            along_nm += gs_kts * dt / 3600.0;
            t += dt;
            if radio <= 0.0 {
                assert!(
                    saw_roundout && saw_flare,
                    "the arrest must run roundout then flare"
                );
                return (vs, shallowest, t);
            }
            assert!(t < 60.0, "never touched down");
        }
    }

    /// The flare must arrive at the wheels, not at the dial. The bare target
    /// reproduces the live arrival (three firm 172 landings in three days,
    /// -309 fpm at LXGB); the led dial arrests it inside the same height,
    /// without a balloon and without floating the runway away.
    #[test]
    fn the_landing_arrest_leads_the_autopilots_lag() {
        let (bare_vs, _, bare_t) = fly_arrest_through_a_lagging_autopilot(false);
        assert!(
            bare_vs < -250.0,
            "the bare dial should reproduce the firm arrival, got {bare_vs:.0}"
        );

        let (led_vs, shallowest, led_t) = fly_arrest_through_a_lagging_autopilot(true);
        assert!(
            led_vs > -170.0,
            "led dial still firm: {led_vs:.0} fpm at the wheels"
        );
        assert!(
            led_vs < -40.0,
            "the arrest must still land, got {led_vs:.0}"
        );
        assert!(shallowest < 0.0, "ballooned to {shallowest:.0} fpm");
        assert!(
            led_t < bare_t + 8.0,
            "floated: {led_t:.1} s vs {bare_t:.1} s"
        );

        // Outside an arrest the dial is the target, exactly as before.
        let lead = crate::aircraft::ArrestLeadProfile::default();
        assert_eq!(
            (lead.gain, lead.max_climb_fpm, lead.trend_seconds),
            (VS_LEAD_GAIN, VS_LEAD_MAX_CLIMB_FPM, 0.0)
        );
        assert_eq!(vs_dial_fpm(-380.0, -600.0, 0.0, false, lead), -380.0);
        // Inside one it leads by the outstanding error, capped at a gentle climb.
        assert_eq!(vs_dial_fpm(-100.0, -300.0, 0.0, true, lead), 300.0);
        assert_eq!(vs_dial_fpm(-100.0, -150.0, 0.0, true, lead), 0.0);
        assert_eq!(vs_dial_fpm(-100.0, -100.0, 0.0, true, lead), -100.0);
        assert_eq!(vs_dial_fpm(-100.0, 200.0, 0.0, true, lead), -700.0);
        // The 172 has no trend term: a trend changes nothing for it.
        assert_eq!(vs_dial_fpm(-100.0, -300.0, 150.0, true, lead), 300.0);
        // A jet's lead predicts the sink `trend_seconds` ahead and caps low:
        // sinking 300 but easing at 100 fpm/s reads as 150 in 1.5 s.
        let jet = crate::aircraft::ArrestLeadProfile {
            gain: 1.5,
            max_climb_fpm: 100.0,
            trend_seconds: 1.5,
        };
        assert_eq!(vs_dial_fpm(-100.0, -300.0, 100.0, true, jet), -25.0);
        assert_eq!(vs_dial_fpm(-100.0, -700.0, 0.0, true, jet), 100.0);
        assert_eq!(
            vs_dial_fpm(-100.0, -300.0, f64::NAN, true, jet),
            200.0_f64.min(100.0)
        );
    }

    /// The gates and targets are seconds of sink: a 172 keeps its feet and
    /// fpm exactly, a jet's open higher and aim firmer.
    #[test]
    fn arrest_gates_and_targets_scale_with_the_path_sink() {
        // 172 at 65-78 kt over the ground: 25 / 50 ft, -250 / -100 fpm.
        for gs in [55.0, 65.0, 75.0] {
            assert_eq!(arrest_gates_ft(25.0, gs), (25.0, 50.0), "gs {gs}");
            let (r, f) = arrest_targets_fpm(-(gs / 60.0) * GLIDEPATH_FT_PER_NM);
            assert_eq!((r, f), (ROUNDOUT_VS_FPM, FLARE_VS_FPM), "gs {gs}");
        }
        // The 737 on EGCC 23L, 132 kt over the ground: 700 fpm on the path.
        let nominal = -(132.0 / 60.0) * GLIDEPATH_FT_PER_NM;
        let (flare, band) = arrest_gates_ft(25.0, 132.0);
        assert!((flare - 35.0).abs() < 1.0, "{flare}");
        assert!((band - 76.0).abs() < 1.0, "{band}");
        let (r, f) = arrest_targets_fpm(nominal);
        assert!((r - -420.0).abs() < 2.0, "{r}");
        assert!((f - -175.0).abs() < 2.0, "{f}");
        // A failed (zero) groundspeed read is the fixed gates, never zero.
        assert_eq!(arrest_gates_ft(25.0, 0.0), (25.0, 50.0));
    }

    /// EGCC 23L, 2026-09-19, replayed: the 737 crossed the numbers at 100 ft
    /// sinking 730 fpm, the level-hold branch dialled ZERO at 60 ft over
    /// the pavement, and the first approach balked "overflew the runway" at
    /// 57 ft and 0.174 nm with 2,700 m of runway ahead.
    #[test]
    fn a_jet_over_the_pavement_keeps_its_sink_and_is_not_an_overflight() {
        let mut rwy = rwy_north();
        rwy.length_m = 3040.0;
        let gs = 132.0;
        let nominal = -(gs / 60.0) * GLIDEPATH_FT_PER_NM;
        // 60 ft over the runway, 150 m in, above the 76 ft band? No: in it.
        // 90 ft, 100 m in: above the band, alt_err negative, the old law
        // held level. Now: the path's own sink, and still Final.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 100.0 / M_PER_NM);
        let g = guidance(lat, lon, 90.0, 90.0, gs, 145.0, &rwy, 145.0, 25.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Final);
        assert_eq!(
            g.target_vs_fpm, nominal,
            "a jet over the pavement keeps descending"
        );
        assert!(!g.arresting);
        // The first balk: 57 ft, 0.174 nm past the threshold, in the band.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 0.174);
        let g = guidance(lat, lon, 57.0, 57.0, gs, 145.0, &rwy, 145.0, 25.0, 0.0);
        assert_eq!(
            g.phase,
            ApproachPhase::Final,
            "this was a landing: {:?}",
            g.balk
        );
        assert!(g.arresting);
        let (roundout, _) = arrest_targets_fpm(nominal);
        assert_eq!(g.target_vs_fpm, roundout);
        // The second: 11 ft, 0.83 nm in, IS past what 3,040 m allows a
        // landing that still needs 900 m (the GA figure; the profile's
        // 1,634 m is stricter still): the overflight verdict stands there.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 0.83);
        let g = guidance(lat, lon, 11.0, 11.0, gs, 155.0, &rwy, 145.0, 25.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Intercept);
        assert_eq!(g.balk, Some("overflew_the_runway"));
    }

    /// The 737's vertical-speed mode, fitted to the LFPG 09R tick log
    /// (2026-09-22, the dial stepped from -788 to +30 at the roundout gate
    /// and to +100 at the flare's, the sink -801 to -699 in six seconds):
    /// two first-order lags of 5 s in series and three seconds of dead time
    /// (four ticks). The EGCC fit before it (1.5 s lags, 2 s dead) was
    /// three times too quick, which is how the dial-path tests passed
    /// while the aircraft arrived at -672. Flown from 100 ft over the
    /// numbers at the path sink, gs 132, with the flare's thrust reduction
    /// as the same sink disturbance the 172 plant carries. Returns the sink
    /// at the wheels, the shallowest rate seen (a balloon is positive), the
    /// metres of runway used and whether the flare gate was reached.
    fn fly_jet_arrest(lead: crate::aircraft::ArrestLeadProfile) -> (f64, f64, f64, bool) {
        let mut rwy = rwy_north();
        rwy.length_m = 3040.0;
        let dt = 0.7;
        let (t1, t2) = (5.0, 5.0);
        let dead_ticks = 4;
        let gs_kts = 132.0;
        let nominal = -(gs_kts / 60.0) * GLIDEPATH_FT_PER_NM;
        let mut radio = 100.0;
        let mut vs = nominal;
        let mut vs_rate = 0.0;
        let mut dials = std::collections::VecDeque::from(vec![vs; dead_ticks]);
        let mut idle_sink = 0.0;
        let mut along_nm = 0.0;
        let mut shallowest: f64 = vs;
        let mut saw_flare = false;
        let mut prev_vs = vs;
        let mut t = 0.0;
        loop {
            let (lat, lon) = project_latlon(0.0, 0.0, 0.0, along_nm);
            let g = guidance(
                lat, lon, radio, radio, gs_kts, 145.0, &rwy, 145.0, 25.0, 0.0,
            );
            assert_ne!(
                g.phase,
                ApproachPhase::Intercept,
                "balked at {radio:.0} ft: {:?}",
                g.balk
            );
            saw_flare |= g.phase == ApproachPhase::Flare;
            let trend = (vs - prev_vs) / dt;
            prev_vs = vs;
            dials.push_back(vs_dial_fpm(g.target_vs_fpm, vs, trend, g.arresting, lead));
            let applied = dials.pop_front().unwrap();
            let want_sink = if g.phase == ApproachPhase::Flare {
                -250.0
            } else {
                0.0
            };
            let sub = 7;
            for _ in 0..sub {
                let h = dt / sub as f64;
                let acc = (applied - vs - (t1 + t2) * vs_rate) / (t1 * t2);
                vs_rate += acc * h;
                vs += vs_rate * h;
                let d_sink = (want_sink - idle_sink) * (h / 1.5);
                idle_sink += d_sink;
                vs += d_sink;
                radio += vs / 60.0 * h;
                along_nm += gs_kts * h / 3600.0;
                if radio <= 0.0 {
                    break;
                }
            }
            shallowest = shallowest.max(vs);
            t += dt;
            if radio <= 0.0 {
                return (vs, shallowest, along_nm * M_PER_NM, saw_flare);
            }
            assert!(t < 60.0, "never touched down");
        }
    }

    /// Through the servo as logged, the dial path arrives hard whatever
    /// the lead: the 172's, the 737's own, or a dial pinned at the cap from
    /// the gate. Six seconds of arrest against three of dead time and ten
    /// of lag is arithmetic, not tuning, and it is why the arrest takes the
    /// servos and flies the elevator (`arrest_hands_step`).
    #[test]
    fn the_dial_path_cannot_arrest_the_737_through_its_logged_servo() {
        let jet = crate::aircraft::ArrestLeadProfile {
            gain: 1.5,
            max_climb_fpm: 100.0,
            trend_seconds: 1.5,
        };
        let pinned = crate::aircraft::ArrestLeadProfile {
            gain: 5.0,
            max_climb_fpm: 1000.0,
            trend_seconds: 0.0,
        };
        for (name, lead) in [
            (
                "the 172's lead",
                crate::aircraft::ArrestLeadProfile::default(),
            ),
            ("the 737's lead", jet),
            ("a dial pinned at +1000", pinned),
        ] {
            let (td, _, _, flared) = fly_jet_arrest(lead);
            assert!(flared, "{name}: never reached the flare gate");
            assert!(
                td < -450.0,
                "{name}: the logged servo cannot have arrested it, yet the plant landed at {td:.0} fpm"
            );
        }
    }

    /// The King Air's 110 approach over a landing floor of 91 (1.2 x 76):
    /// the retard's reference is 1.3 Vs, 99; an SR22-like 75 over 71 keeps 75.
    #[test]
    fn the_retard_protects_the_stall_margin_not_the_approach_target() {
        assert!((retard_reference_kts(110.0, Some(91.0)) - 98.583).abs() < 0.01);
        assert_eq!(retard_reference_kts(75.0, Some(71.0)), 75.0);
        assert_eq!(retard_reference_kts(110.0, None), 110.0);
    }

    /// SR22 1790792031: an unstabilised go-around at 914 ft over the runway
    /// and 800 ft radio over the hills climbs on to 1,200 ft over the runway
    /// before the reposition, where it had released the next tick.
    #[test]
    fn an_unstabilised_go_around_climbs_over_its_gate() {
        let mut latch = GoAroundLatch::new();
        assert!(latch.order_over_gate(false, 1000.0));
        latch.note_height_aal(914.0);
        let s = latch.update(Some(ApproachPhase::Final), ApproachPhase::Final, 820.0, 4.1, 4.1, 3.3, 0.0);
        assert_eq!(s, Some(MissedStage::Climb));
        latch.note_height_aal(1210.0);
        let s = latch.update(Some(ApproachPhase::Final), ApproachPhase::Final, 1110.0, 3.9, 3.9, 4.4, 0.0);
        assert_eq!(s, Some(MissedStage::Reposition));
        // A tower's order keeps the old test: 800 ft radio.
        let mut latch = GoAroundLatch::new();
        assert!(latch.order(false));
        latch.note_height_aal(914.0);
        let s = latch.update(Some(ApproachPhase::Final), ApproachPhase::Final, 820.0, 4.1, 4.1, 3.3, 0.0);
        assert_eq!(s, Some(MissedStage::Reposition));
    }

    /// King Air 1790787298: the arrest's level-off at 36 ft on approach
    /// thrust. Sinking on the approach, the speed law keeps the lever; the
    /// sink trending through level retards it, under the approach floor; a
    /// climb walks it to idle and no further.
    #[test]
    fn a_floating_roundout_takes_the_thrust_off() {
        assert_eq!(roundout_float_lever(0.612, -557.0, 40.0, 0.33, 0.1), None);
        let lead = roundout_float_lever(0.612, -182.0, 140.0, 0.33, 0.1).expect("level on the trend");
        assert!((lead - 0.579).abs() < 1e-9, "{lead}");
        assert!(roundout_float_lever(0.612, 91.0, 130.0, 0.33, 0.1).is_some());
        assert_eq!(roundout_float_lever(0.11, 300.0, 0.0, 0.5, 0.1), Some(IDLE_THROTTLE));
    }

    /// The decision itself: taken at the roundout gate, kept through the
    /// flare, returned the tick the arrest ends without wheels on.
    #[test]
    fn the_arrest_takes_the_servos_once_and_returns_them_on_a_balk() {
        assert_eq!(arrest_hands_step(false, false), ArrestHands::Off);
        assert_eq!(arrest_hands_step(false, true), ArrestHands::Take);
        assert_eq!(arrest_hands_step(true, true), ArrestHands::Keep);
        assert_eq!(arrest_hands_step(true, false), ArrestHands::Return);
    }

    /// The servos-out arrest over the runway, flown from the gate of the LEMG
    /// 13 landing at 17:43Z (2026-09-26, a 737): 79 ft radio, 3.9 m right of
    /// the centreline and crossing left at 2.3 m/s, 1.4 degrees of left bank,
    /// 144 kt, 8.5 s to the wheels. It touched down 15.3 m left and the
    /// rollout scored 16.8 m.
    ///
    /// The plant is second order in roll (aileron to roll rate with a 0.5 s
    /// lag, 20 deg/s per unit of aileron: the recorded 1 deg/s at 0.035 to
    /// 0.08 of aileron), bank to sideways acceleration, and the arrest's own
    /// tick of 0.25 s with one tick of transport delay. The number asserted is
    /// the offset at the wheels, where the scorer's rollout begins, and the
    /// worst overshoot past the centreline on the way down.
    #[test]
    fn the_arrest_closes_the_centreline_by_the_track_over_the_runway() {
        use crate::autotakeoff::{climb_bank_target, roll_target_heading, WingsLevel, XtkRate};
        let fly = |track_law: bool, x0: f64, v0: f64, phi0: f64| -> (f64, f64) {
            let (gs_kts, t_end, dt) = (144.0, 8.5, 0.25);
            let v_air = gs_kts * 0.514_444;
            let (mut x, mut v, mut phi, mut p) = (x0, v0, phi0, 0.0_f64);
            let mut wings = if track_law { arrest_wings() } else { WingsLevel::default() };
            let mut rate = XtkRate::default();
            let mut prev_x: Option<f64> = None;
            let mut queued = 0.0_f64;
            let (mut t, mut worst) = (0.0, 0.0_f64);
            while t < t_end {
                let radio = 79.0 * (1.0 - t / t_end);
                let rate_now = rate.update(prev_x.map_or(0.0, |px| (x - px) / dt), dt);
                prev_x = Some(x);
                // Coordinated: the heading is the track (no wind, no crab).
                let track_deg = (v / v_air).asin().to_degrees();
                let bank_tgt = if track_law {
                    let side = side_velocity_m_s(Some(track_deg), 0.0, gs_kts).unwrap();
                    arrest_bank_target(x, side, phi, radio)
                } else {
                    let hdg = roll_target_heading(0.0, x, rate_now, gs_kts);
                    let err = crate::angles::signed_diff_deg(hdg, track_deg);
                    climb_bank_target(err)
                };
                let ail = wings.step(phi, bank_tgt, dt);
                let applied = std::mem::replace(&mut queued, ail);
                for _ in 0..10 {
                    let h = dt / 10.0;
                    p += (20.0 * applied - p) / 0.5 * h;
                    phi += p * h;
                    v += 9.81 * phi.to_radians().tan() * h;
                    x += v * h;
                }
                worst = worst.max(-x * x0.signum());
                t += dt;
            }
            (x, worst)
        };
        let (old_wheels, _) = fly(false, 3.9, -2.3, -1.4);
        assert!(old_wheels.abs() > 5.0, "the heading law lands {old_wheels:.1} m off");
        let (wheels, worst) = fly(true, 3.9, -2.3, -1.4);
        assert!(wheels.abs() < 2.0, "the track law lands {wheels:.1} m off");
        assert!(worst < 5.0, "and never overshoots {worst:.1} m past the line");
        // The 17:25Z landing: 5.9 m right at 100 ft, crossing at 2.5 m/s.
        let (old_wheels, _) = fly(false, 5.9, -2.5, 0.0);
        assert!(old_wheels.abs() > 5.0, "{old_wheels:.1}");
        let (wheels, worst) = fly(true, 5.9, -2.5, 0.0);
        assert!(wheels.abs() < 2.0 && worst < 5.0, "{wheels:.1} / {worst:.1}");
    }

    /// LEMG 13, 2026-09-26 (20:17Z, a 737): the arrest gate at 79 ft, 11.89 m
    /// right, drifting 0.13 m/s left, 2.08 degrees of left bank, 10.4 s to
    /// the wheels. Recorded: the drift built to 2.65 m/s, the bank target
    /// reversed at 42 ft but the bank crossed wings level only at 26 ft, it
    /// crossed the centreline at 22 ft still at 2.5 m/s and touched down
    /// 6.5 m left.
    ///
    /// The roll plant is FITTED to those ticks, not the faster one above:
    /// 13 deg/s of roll rate per unit of aileron (0.15 to 0.18 of aileron took
    /// the bank from -3.8 to -0.5 degrees in 1.7 s) through a 0.5 s lag, and
    /// a slow return toward wings level (3.1 to 2.2 degrees in 2 s at zero
    /// aileron), with the real `WingsLevel`, a 0.25 s tick and one tick of
    /// delay. Asserted: the offset at the wheels and the worst overshoot.
    #[test]
    fn the_arrest_stops_its_drift_by_the_centreline() {
        use crate::autotakeoff::WingsLevel;
        fn old_law(x: f64, side: f64, radio: f64) -> f64 {
            let want = (-x / 4.0).clamp(-2.5, 2.5);
            let bank = ((want - side) / (9.81 * 2.0)).atan().to_degrees();
            let cap = (2.0 + radio.max(0.0) / 10.0).min(5.0);
            bank.clamp(-cap, cap)
        }
        let fly_k = |new: bool, x0: f64, v0: f64, phi0: f64, t_end: f64, k: f64| -> (f64, f64) {
            let (gate_ft, dt) = (79.0, 0.25);
            let (mut x, mut v, mut phi, mut p) = (x0, v0, phi0, 0.0_f64);
            let mut wings = if new { arrest_wings() } else { WingsLevel::default() };
            let mut queued = 0.0_f64;
            let (mut t, mut worst) = (0.0, 0.0_f64);
            while t < t_end {
                let radio = gate_ft * (1.0 - t / t_end);
                let tgt = if new {
                    arrest_bank_target(x, v, phi, radio)
                } else {
                    old_law(x, v, radio)
                };
                let ail = wings.step(phi, tgt, dt);
                let applied = std::mem::replace(&mut queued, ail);
                for _ in 0..10 {
                    let h = dt / 10.0;
                    p += (k * applied - phi / 7.5 - p) / 0.5 * h;
                    phi += p * h;
                    v += 9.81 * phi.to_radians().tan() * h;
                    x += v * h;
                }
                worst = worst.max(-x * x0.signum());
                t += dt;
            }
            (x, worst)
        };
        let fly = |new: bool, x0: f64, v0: f64, phi0: f64, t_end: f64| fly_k(new, x0, v0, phi0, t_end, 13.0);
        let (old_wheels, old_worst) = fly(false, 11.89, -0.13, -2.08, 10.4);
        let (wheels, worst) = fly(true, 11.89, -0.13, -2.08, 10.4);
        let (small, small_worst) = fly(true, 3.0, 0.0, 0.0, 10.4);
        assert!(old_wheels < -4.0 && old_worst > 4.0, "the plant reproduces the overshoot: {old_wheels:.1} m");
        assert!(wheels.abs() < 2.0, "lands {wheels:.1} m off");
        assert!(worst < 2.0, "overshoots {worst:.1} m past the line");
        assert!(small.abs() < 1.0 && small_worst < 1.0, "from 3 m: {small:.1} / {small_worst:.1}");
        // Not tuned to one roll rate: the gate lands inside 2 m on a plant
        // that rolls a third slower or half again faster.
        for k in [9.0, 20.0] {
            let (w, o) = fly_k(true, 11.89, -0.13, -2.08, 10.4, k);
            assert!(w.abs() < 2.0 && o < 2.0, "at {k} deg/s per unit: {w:.1} / {o:.1}");
        }
        // Drifting AWAY at the gate still comes in, and a gate already
        // crossing fast (17:43Z's 3.9 m at 2.3 m/s) lands nearer than it did.
        let (w, o) = fly(true, 8.0, 1.0, 0.0, 10.4);
        assert!(w.abs() < 1.0 && o < 1.0, "drifting away: {w:.1} / {o:.1}");
        let (w_old, _) = fly(false, 3.9, -2.3, -1.4, 8.5);
        let (w, _) = fly(true, 3.9, -2.3, -1.4, 8.5);
        assert!(w.abs() < 2.0 && w.abs() < w_old.abs(), "crossing fast: {w:.1} against {w_old:.1}");
    }

    /// LEMG 13, 2026-09-26 (20:17Z, a 737), the last 7.5 s of the final: on
    /// the centreline at 172 ft, 8.9 m right at 88 ft and 9 to 12 m right at
    /// the 79 ft gate, with the bug within half a degree of the runway
    /// (131.68 true) all the way. Replayed from the recorded inputs: the
    /// crab plus track trim the loop added (0.59 falling to 0.27 degrees),
    /// the recorded drift of the track off the heading (the wind), and an
    /// autopilot that holds heading 0.6 degrees right of its bug through a
    /// 3 s lag (the recorded heading ran 132.1 to 132.6 against a bug of
    /// 131.6 to 132.3). Asserted: the offset at the gate.
    #[test]
    fn the_short_final_holds_the_centreline_to_the_gate() {
        let course = 131.68_f64;
        let gs = 144.0_f64;
        let v_air = gs * 0.514_444;
        let crab_trim = |t: f64| if t < 4.7 { 0.59 - (0.59 - 0.27) * t / 4.7 } else { 0.28 };
        // The recorded track minus heading, at the state samples.
        let drift_pts = [(0.0, 0.9), (0.9, 0.3), (2.2, 0.2), (3.6, 0.7), (5.0, 0.2), (6.3, -0.4), (7.6, -0.4)];
        let drift = |t: f64| {
            let mut d = drift_pts[0].1;
            for w in drift_pts.windows(2) {
                if t >= w[0].0 && t <= w[1].0 {
                    d = w[0].1 + (w[1].1 - w[0].1) * (t - w[0].0) / (w[1].0 - w[0].0);
                }
            }
            if t > drift_pts[drift_pts.len() - 1].0 {
                d = drift_pts[drift_pts.len() - 1].1;
            }
            d
        };
        let fly_with = |new: bool, ap_bias: f64, ap_tau: f64, t_end: f64| -> (f64, f64) {
            let dt = 0.33;
            let (mut x, mut hdg, mut t, mut worst) = (0.0_f64, 131.8_f64, 0.0_f64, 0.0_f64);
            let mut queued = 132.27_f64;
            let mut sf = ShortFinal::new(132.27 - course, hdg + drift(0.0) - course);
            while t < t_end {
                let track = hdg + drift(t);
                let bug = if new {
                    course + sf.step(x, track - course, gs, dt, None)
                } else {
                    course + (-x / M_PER_NM * 80.0).clamp(-8.0, 8.0) + crab_trim(t)
                };
                let applied = std::mem::replace(&mut queued, bug);
                for _ in 0..10 {
                    let h = dt / 10.0;
                    hdg += (applied + ap_bias - hdg) / ap_tau * h;
                    x += v_air * (hdg + drift(t) - course).to_radians().sin() * h;
                }
                worst = worst.max(x.abs());
                t += dt;
            }
            (x, worst)
        };
        let fly = |new: bool| fly_with(new, 0.6, 3.0, 7.5);
        let (old_gate, _) = fly(false);
        let (gate, worst) = fly(true);
        assert!(old_gate > 6.0, "the replay reproduces the drift to the gate: {old_gate:.1} m");
        assert!(gate.abs() < 2.0 && worst < 3.0, "the gate at {gate:.1} m, worst {worst:.1}");
        // Not tuned to one autopilot: no bias, a larger one, a slower and a
        // quicker heading mode.
        for (b, tau) in [(0.0, 3.0), (1.2, 3.0), (0.6, 5.0), (0.6, 1.5)] {
            let (g, w) = fly_with(true, b, tau, 7.5);
            assert!(g.abs() < 2.5 && w < 3.5, "bias {b}, lag {tau} s: gate {g:.1}, worst {w:.1}");
            // ...and a slow airframe's 20 s of short final settles, not rings.
            let (g, w) = fly_with(true, b, tau, 20.0);
            assert!(g.abs() < 1.5 && w < 4.0, "20 s, bias {b}, lag {tau} s: {g:.1} / {w:.1}");
        }
        // Small and bounded: a metre with no drift asks a tenth of a degree,
        // and nothing asks more than the cap.
        assert!(short_final_cut_deg(1.0, 0.0, 144.0).abs() < 0.2);
        assert_eq!(short_final_cut_deg(200.0, 3.0, 144.0), -SHORT_FINAL_MAX_CUT_DEG);
        assert_eq!(short_final_cut_deg(f64::NAN, 0.0, 144.0), 0.0);
        assert!(short_final_cut_deg(0.0, 1.3, 144.0) < -0.5, "a drift is answered before it is metres");
    }

    /// The bank is shallow near the wheels and nothing without a reading.
    #[test]
    fn the_arrest_bank_tapers_to_the_wheels() {
        assert_eq!(arrest_bank_target(-30.0, 0.0, 0.0, 80.0), ARREST_BANK_MAX_DEG);
        assert_eq!(arrest_bank_target(-30.0, 0.0, 0.0, 0.0), ARREST_BANK_WHEELS_DEG);
        assert!(arrest_bank_target(30.0, 0.0, 0.0, 80.0) < 0.0, "right of the line banks left");
        assert_eq!(arrest_bank_target(f64::NAN, 0.0, 0.0, 50.0), 0.0);
        assert_eq!(arrest_bank_target(0.0, 0.0, 0.0, 50.0), 0.0);
        // Crossing left at 2.3 m/s over the centreline: bank right now.
        assert!(arrest_bank_target(0.0, -2.3, 0.0, 50.0) > 3.0);
        assert!(side_velocity_m_s(None, 130.0, 140.0).is_none());
        let v = side_velocity_m_s(Some(129.0), 130.0, 140.0).unwrap();
        assert!((v + 1.26).abs() < 0.05, "1 degree left at 140 kt: {v}");
    }

    /// The overflight rule must know a landing float from an overflight. At
    /// EGHI it balked at 26.3 ft radio — ONE FOOT above the flare gate — at
    /// 0.165 nm past the threshold with 1,400 m of runway still ahead, then
    /// the missed approach timed out mid-reposition and the give-up handed
    /// the pilot a dirty aircraft at half throttle (2026-08-02).
    #[test]
    fn a_low_float_over_a_long_runway_is_a_landing_not_an_overflight() {
        // EGHI-sized runway: 1,723 m.
        let mut rwy = rwy_north();
        rwy.length_m = 1723.0;
        // Low in the float: the allowance is runway-aware — 823 m here.
        let low = overflight_allowance_nm(1723.0, 26.0, 50.0, None);
        assert!((low - 823.0 / M_PER_NM).abs() < 0.01, "{low}");
        // High past the threshold keeps the old hair-trigger: from 160 ft a
        // 3 degree descent runs 640 m to the band, more than the float has.
        assert_eq!(overflight_allowance_nm(1723.0, 160.0, 50.0, None), 0.15);
        // A short strip keeps the old rule even in the float, and a long one
        // is capped at half its length.
        assert_eq!(overflight_allowance_nm(700.0, 20.0, 50.0, None), 0.15);
        assert!(
            (overflight_allowance_nm(3900.0, 20.0, 50.0, None) - 1950.0 / M_PER_NM).abs() < 0.01
        );
        // EGCC 23L, 2026-09-19: the 737 at 57 ft radio, 322 m past the
        // numbers and sinking, balked as an overflight with 2,700 m ahead.
        // Its band is 76 ft, so it is IN the float, and the pavement a
        // landing needs is its own 1,634 m: 1,406 m of float, not 278.
        let jet = overflight_allowance_nm(3040.0, 57.0, 76.0, Some(1634.0));
        assert!(jet > 0.174, "{jet}");
        assert!((jet - 1406.0 / M_PER_NM).abs() < 0.01, "{jet}");
        // Above its band the same geometry applies: 100 ft is 24 ft above
        // the band, 140 m of run, still 1,266 m of float.
        let high = overflight_allowance_nm(3040.0, 100.0, 76.0, Some(1634.0));
        assert!((high - 1266.0 / M_PER_NM).abs() < 0.02, "{high}");
        // A recited landing distance never LOOSENS the GA figure.
        assert_eq!(
            overflight_allowance_nm(1723.0, 26.0, 50.0, Some(300.0)),
            overflight_allowance_nm(1723.0, 26.0, 50.0, None)
        );

        // The live balk, replayed: 0.165 nm past the threshold, 26 ft radio.
        // Must be a roundout on Final, not a go-around.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 0.165);
        let g = guidance(lat, lon, 26.0, 26.0, 94.0, 97.0, &rwy, 95.0, 25.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Final, "this was a landing");
        let (roundout, _) = arrest_targets_fpm(-(94.0 / 60.0) * GLIDEPATH_FT_PER_NM);
        assert_eq!(g.target_vs_fpm, roundout);

        // Deeper down the float, below the flare gate — dist past the old
        // 0.35 nm flare-zone bound — the flare still owns the touchdown.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 0.40);
        let g = guidance(lat, lon, 18.0, 18.0, 92.0, 92.0, &rwy, 95.0, 25.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Flare);

        // Past the allowance the overflight verdict returns.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 0.50);
        let g = guidance(lat, lon, 20.0, 20.0, 92.0, 92.0, &rwy, 95.0, 25.0, 0.0);
        assert_eq!(
            g.phase,
            ApproachPhase::Intercept,
            "not down by 823 m: go around"
        );

        // And HIGH past the threshold goes around at the old margin, float or
        // no float — the 167 ft overflight stays caught.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 0.2);
        let g = guidance(lat, lon, 160.0, 160.0, 95.0, 95.0, &rwy, 95.0, 25.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Intercept);
    }

    fn rwy_north() -> Runway {
        // A runway at the equator, landing due north (course 0), threshold at 0,0.
        Runway {
            icao: "TEST".into(),
            id: "36".into(),
            thr_lat: 0.0,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 3000.0,
            surface: 1,
            dist_nm: 0.0,
            elev_ft: 0.0,
        }
    }

    /// EGTE 08, 2026-09-21: 1,880 m off at 3.3 nm and 160 kt, on the path at
    /// 1,000 ft. Position called it interceptable; it arrived 207 m off at
    /// 94 ft. The gate balks it with the height still in hand.
    #[test]
    fn a_jet_final_that_cannot_converge_balks_at_a_thousand_feet() {
        let at = |along_nm: f64, xtk_m: f64, radio: f64, gs: f64| {
            let (lat, lon) = project_latlon(0.0, 0.0, 180.0, along_nm);
            let (lat, lon) = project_latlon(lat, lon, 90.0, xtk_m / M_PER_NM);
            guidance(
                lat,
                lon,
                radio,
                radio,
                gs,
                gs,
                &rwy_north(),
                150.0,
                25.0,
                0.0,
            )
        };
        let g = at(3.0, 1880.0, 990.0, 160.0);
        assert_eq!(g.balk, Some("unstabilised_at_gate"));
        assert_eq!(g.phase, ApproachPhase::Intercept);
        // Above the gate the same offset is still an intercept in progress.
        let g = at(3.3, 1880.0, 1100.0, 160.0);
        assert_eq!(g.balk, None);
        // An ordinary jet final through the gate, and a light single's.
        assert_eq!(at(3.0, 150.0, 990.0, 150.0).balk, None);
        assert_eq!(at(2.0, 250.0, 700.0, 70.0).balk, None);
    }

    /// Flap 40 out at 161 kt under a 162 placard, back to 10 at 162, out again
    /// at 900 ft (EGTE, 2026-09-21).
    #[test]
    fn flaps_do_not_cycle_on_a_placard() {
        let limits = [
            (0.0, None),
            (0.5, Some(210.0)),
            (0.875, Some(175.0)),
            (1.0, Some(162.0)),
        ];
        let held = |w, ias, cur| flap_held_within_limits(w, ias, &limits, cur);
        // Not extended with a knot of room...
        assert_eq!(held(1.0, 161.0, Some(0.875)), 0.875);
        // ...extended with five...
        assert_eq!(held(1.0, 156.0, Some(0.875)), 1.0);
        // ...held through a touch of the limit...
        assert_eq!(held(1.0, 163.0, Some(1.0)), 1.0);
        // ...and brought in when properly over it.
        assert_eq!(held(1.0, 172.0, Some(1.0)), 0.875);
        // The go-around's half is obeyed at once.
        assert_eq!(held(0.5, 150.0, Some(1.0)), 0.5);
        // No flap profile: the schedule stands.
        assert_eq!(flap_held_within_limits(1.0, 300.0, &[], Some(0.0)), 1.0);
    }

    /// LEMG 13, 2026-09-28 (sortie 1790584266-approach, and every LEMG setup
    /// of the run): the sim handed autoland the 737 at flaps 2 (0.25), the
    /// schedule wanted clean at 9.9 nm, and the first tick took the flaps up
    /// at a 58 kt airspeed read, the next at 146, under the 139 kt clean
    /// stall. An approach keeps the flap the wing flies on until the speed
    /// flies the cleaner configuration (the 737's "up" is declared at 210).
    #[test]
    fn the_approach_keeps_the_flap_the_wing_is_flying_on() {
        let p = crate::aircraft::load("boeing_737_800").unwrap();
        let sched = crate::aircraft::flap_schedule(p.flaps.as_ref());
        let retract = |r: f64| {
            sched
                .iter()
                .find(|(x, _)| (x - r).abs() < 1e-3)
                .map(|(_, k)| *k)
        };
        let limits = [
            (0.0, None),
            (0.25, Some(250.0)),
            (0.5, Some(210.0)),
            (0.875, Some(175.0)),
            (1.0, Some(162.0)),
        ];
        let wanted = flap_schedule(9.88, 2379.2 - 3240.9);
        assert_eq!(wanted, 0.0, "the distance schedule wants clean at 9.9 nm");
        // The old rule took them away on the recorded ticks...
        assert_eq!(flap_held_within_limits(wanted, 58.2, &limits, Some(0.25)), 0.0);
        assert_eq!(flap_held_within_limits(wanted, 146.7, &limits, Some(0.25)), 0.0);
        // ...the approach keeps them, every tick of the dive.
        for ias in [58.2, 146.7, 163.6, 166.0, 172.2, 203.5] {
            let held = flap_held_within_limits(wanted, ias, &limits, Some(0.25));
            assert_eq!(
                flap_kept_flying(held, Some(0.25), ias, &limits, false, retract),
                0.25,
                "{ias} kt"
            );
        }
        // At the manoeuvring speed they come in.
        assert_eq!(flap_kept_flying(0.0, Some(0.25), 212.2, &limits, false, retract), 0.0);
        // No declared speed: the configuration's floor stands in.
        assert_eq!(flap_kept_flying(0.0, Some(0.25), 160.0, &limits, false, |_| Some(167.0)), 0.25);
        assert_eq!(flap_kept_flying(0.0, Some(0.25), 170.0, &limits, false, |_| Some(167.0)), 0.0);
        // Nothing known: the schedule stands, as before.
        assert_eq!(flap_kept_flying(0.0, Some(0.25), 150.0, &limits, false, |_| None), 0.0);
        // A placard still wins, and so does the go-around's half.
        assert_eq!(flap_kept_flying(0.875, Some(1.0), 172.0, &limits, false, retract), 0.875);
        assert_eq!(flap_kept_flying(0.5, Some(1.0), 145.0, &limits, true, retract), 0.5);
        // Extending is never held back, and an unread flap changes nothing.
        assert_eq!(flap_kept_flying(0.5, Some(0.25), 180.0, &limits, false, retract), 0.5);
        assert_eq!(flap_kept_flying(0.0, None, 58.2, &limits, false, retract), 0.0);
    }

    /// The A330's placards and per-notch floors, straight from its profile.
    fn a330_flaps() -> (Vec<(f64, Option<f64>)>, impl Fn(f64) -> Option<f64>) {
        let p = crate::aircraft::load("airbus_a330_300").unwrap();
        let f = p.flaps.clone().unwrap();
        let limits: Vec<_> = f.detents.iter().map(|d| (d.ratio, d.vfe_kts)).collect();
        let clean = p.vs_clean_kts;
        let floors: Vec<_> = f
            .detents
            .iter()
            .map(|d| (d.ratio, if d.ratio == 0.0 { clean } else { d.vs_kts }))
            .collect();
        let floor = move |r: f64| {
            floors
                .iter()
                .find(|(x, _)| (x - r).abs() < 1e-3)
                .and_then(|(_, vs)| vs.map(|v| (1.2 * v).round()))
        };
        (limits, floor)
    }

    /// LEMG 13, sortie 1790622093-approach (2026-09-28, #171): an A330 set up
    /// on a 10 nm final at 206 kt with CONF 1 out. At 7.9 nm (211 kt) the
    /// distance schedule's "clean" took CONF 1 away at the clean retract
    /// speed, the clean floor (210) then held the lever up, and after the
    /// pattern the inbound leg chased the clean 223 kt with CONF 2 out: the
    /// placard took both flaps and the final ran at 200 to 241 kt.
    #[test]
    fn the_a330_final_keeps_its_flap_and_flies_under_the_placard() {
        let (limits, floor) = a330_flaps();
        assert_eq!(flap_placard_kts(0.0, &limits), None);
        assert_eq!(flap_placard_kts(0.25, &limits), Some(215.0));
        assert_eq!(flap_placard_kts(0.5, &limits), Some(196.0));
        assert_eq!(floor(0.0), Some(210.0), "the clean floor, 1.2 x 175");
        assert_eq!(floor(0.25), Some(176.0));

        // t 36.9: 7.88 nm, 47 ft under the path, 211.3 kt, CONF 1 out.
        let sched = flap_schedule(7.877, 2557.3 - 2604.4);
        assert_eq!(sched, 0.0, "the distance schedule wants clean");
        // The old ask: CONF 1 came up at the clean retract speed (210).
        let old = flap_held_within_limits(sched, 211.3, &limits, Some(0.25));
        let retract = |_: f64| Some(210.0);
        assert_eq!(flap_kept_flying(old, Some(0.25), 211.3, &limits, false, retract), 0.0);
        // The approach never asks for less than is out.
        let wanted = approach_flap_wanted(sched, Some(0.25));
        assert_eq!(wanted, 0.25);
        let held = flap_held_within_limits(wanted, 211.3, &limits, Some(0.25));
        assert_eq!(flap_kept_flying(held, Some(0.25), 211.3, &limits, false, retract), 0.25);
        // Inside 6 nm it still asks for more, and a placard still takes it.
        assert_eq!(approach_flap_wanted(flap_schedule(5.4, 400.0), Some(0.25)), 0.5);
        assert_eq!(flap_held_within_limits(0.25, 236.4, &limits, Some(0.25)), 0.0);
        // Nothing read is nothing kept.
        assert_eq!(approach_flap_wanted(0.0, None), 0.0);

        // The speed: t 2.1, 9.9 nm, CONF 1. The schedule's clean speed is
        // 1.45 x 154 = 223, over CONF 1's 215 placard.
        let at_arm = scheduled_ias(9.916, 154.0);
        assert!(at_arm > 215.0, "{at_arm}");
        assert_eq!(approach_target_kts(at_arm, 0.25, &limits, floor(0.25)), 205.0);
        // t 553.1, the pattern's inbound leg at 12.6 nm with CONF 2 out:
        // the governor went to full power for 223 and reached 241.
        let inbound = scheduled_ias(12.635, 154.0);
        assert!((inbound - 223.3).abs() < 0.1, "{inbound}");
        assert_eq!(approach_target_kts(inbound, 0.5, &limits, floor(0.5)), 186.0);
        // The schedule under the placard stands, and so does the floor.
        assert_eq!(approach_target_kts(154.0, 0.25, &limits, floor(0.25)), 176.0);
        assert_eq!(approach_target_kts(154.0, 0.0, &limits, floor(0.0)), 210.0);
        assert_eq!(approach_target_kts(180.0, 1.0, &limits, None), 170.0);
    }

    /// Same sortie, t 679.0: Final at 5.28 nm, 239.4 kt over the ground,
    /// 491.7 ft above the path, on a -1,200 dial. The 3 degree path at that
    /// groundspeed is 1,269 fpm: the flat cap could not even hold it, and a
    /// second later the aircraft was 505 ft high and the guidance aimed at a
    /// join point behind it.
    #[test]
    fn the_glidepath_law_can_always_hold_the_path_it_is_on() {
        let nominal = -(239.4 / 60.0) * GLIDEPATH_FT_PER_NM;
        assert!(nominal < -1200.0, "{nominal}");
        let cap = path_descent_cap_fpm(nominal);
        assert!(cap <= nominal - 299.0, "{cap}");
        let law = (nominal - 491.7 * glidepath_gain_fpm_per_ft(239.4)).clamp(cap, 0.0);
        assert!(law < nominal, "it catches up: {law}");
        // A slower final keeps the old limit.
        assert_eq!(path_descent_cap_fpm(-(150.0 / 60.0) * GLIDEPATH_FT_PER_NM), -1200.0);
        assert_eq!(path_descent_cap_fpm(-(70.0 / 60.0) * GLIDEPATH_FT_PER_NM), -1200.0);
    }

    /// Same sortie, t 680.4: the second `reversal` (missed false, final not
    /// called) at 5.2 nm on the centreline, tracking the runway, 505 ft
    /// above the path. Over 500 ft the guidance aims at the join point for
    /// its altitude, 8 nm out, behind it: `reversal::needed` read that as a
    /// pattern. The copilot then said "Overflew the runway" at 5.2 nm.
    #[test]
    fn a_pattern_begun_short_of_the_field_says_why_and_never_overflew() {
        let rwy = Runway {
            icao: "LEMG".into(),
            id: "13".into(),
            thr_lat: 36.6860,
            thr_lon: -4.5100,
            course_deg: 131.5,
            length_m: 2750.0,
            surface: 1,
            dist_nm: 0.0,
            elev_ft: 48.0,
        };
        let (lat, lon) = crate::nav::project_latlon(
            rwy.thr_lat,
            rwy.thr_lon,
            (rwy.course_deg + 180.0) % 360.0,
            5.2,
        );
        let aim = join_point_nm(2250.0, rwy.elev_ft);
        assert!(aim > 5.2, "the join point is behind: {aim}");
        assert!(crate::reversal::needed(&rwy, lat, lon, 132.2, aim, false));
        assert_eq!(crate::reversal::why(false, false, 505.0), "join_point_behind");
        assert_eq!(crate::reversal::why(true, false, -40.0), "missed_approach");
        assert_eq!(crate::reversal::why(false, true, 100.0), "final_pointing_away");
        assert_eq!(crate::reversal::why(false, false, 100.0), "fap_behind");
        // The phase change on that tick: the pattern's own line says it.
        assert_eq!(dropped_final_line(5.07, true, 516.0), None);
        let high = dropped_final_line(5.07, false, 516.0).unwrap();
        assert!(!high.contains("Overflew"), "{high}");
        // Only with the runway behind.
        assert!(dropped_final_line(-0.3, false, 800.0).unwrap().starts_with("Overflew"));
        assert!(dropped_final_line(-0.3, true, 800.0).unwrap().starts_with("Overflew"));
    }

    /// Same sortie: +100 to +2,000 fpm on the dial under the reposition
    /// floor while the speed law held the lever at 0.10 (the dive's rising
    /// airspeed read as "fast"). A climb is flown on thrust.
    #[test]
    fn a_commanded_climb_is_never_flown_at_idle() {
        use ApproachPhase::*;
        assert_eq!(climb_power_floor(Intercept, 100.0, Some(0.3)), 0.3);
        assert_eq!(climb_power_floor(Intercept, 2000.0, Some(0.3)), 0.3);
        assert_eq!(climb_power_floor(Intercept, 2000.0, None), 0.1);
        assert_eq!(climb_power_floor(Intercept, -700.0, Some(0.3)), 0.1);
        assert_eq!(climb_power_floor(Intercept, 0.0, Some(0.3)), 0.1);
        assert_eq!(climb_power_floor(Final, 500.0, Some(0.3)), 0.1);
        assert_eq!(climb_power_floor(Flare, 500.0, Some(0.3)), 0.1);
    }

    /// Sortie 1790584266-approach, replayed: autoland armed on 12 from the
    /// stale clearance, 3.6 km beside its centreline. The guidance never
    /// descended or flared there: every recorded tick is an Intercept, no
    /// arrest, and from 1,470 ft radio down to 16 ft the law asked to CLIMB
    /// (the reposition floor). The dive was the aircraft's: flaps taken away
    /// under the clean stall at idle (see the two tests above). Pinned so a
    /// later gate cannot flare or descend off a centreline this far away.
    #[test]
    fn the_lemg_crash_ticks_were_never_a_descent_or_a_flare() {
        let rwy = Runway {
            icao: "LEMG".into(),
            id: "12".into(),
            // apt.dat's LEMG 12 threshold and its course to 30's.
            thr_lat: 36.6910753,
            thr_lon: -4.5078270,
            course_deg: crate::nav::bearing_deg(36.6910753, -4.5078270, 36.6795648, -4.4805125),
            length_m: 2750.0,
            surface: 1,
            dist_nm: 0.0,
            elev_ft: 48.0,
        };
        // (dist nm, xtk m, alt ft, radio ft, ias kt), as recorded.
        let ticks = [
            (9.72, -3547.73, 2302.61, 1469.68, 166.04),
            (9.55, -3491.5, 2198.82, 1079.73, 163.1),
            (9.47, -3481.21, 2152.65, 909.03, 163.04),
            (9.37, -3482.11, 2078.31, 864.14, 170.6),
            (9.27, -3495.28, 1944.93, 728.23, 173.39),
            (9.16, -3519.9, 1821.9, 467.69, 175.85),
            (9.06, -3546.88, 1697.67, 274.78, 185.54),
            (8.96, -3575.45, 1560.96, 131.37, 192.15),
            (8.84, -3607.9, 1436.3, 72.21, 203.51),
            (8.81, -3619.45, 1401.01, 46.95, 207.8),
            (8.77, -3631.42, 1370.29, 16.36, 212.19),
        ];
        for (dist, xtk_m, alt, radio, ias) in ticks {
            let (dist, ias): (f64, f64) = (dist, ias);
            let lateral = xtk_m / M_PER_NM;
            let along = (dist * dist - lateral * lateral).sqrt();
            let (lat, lon) = crate::reversal::pattern_point(&rwy, along, lateral);
            let g = guidance(lat, lon, alt, radio, ias.max(174.0), ias, &rwy, 145.0, 25.0, 0.0);
            assert_eq!(g.phase, ApproachPhase::Intercept, "{dist} nm {radio} ft");
            assert!(!g.arresting, "{dist} nm {radio} ft");
            assert!(g.target_vs_fpm > 0.0, "{dist} nm {radio} ft: {}", g.target_vs_fpm);
        }
    }

    /// #131, sortie 1790583663-approach: re-armed from a 10 nm final for 13
    /// onto 12 (courses 131 and 117), 3.5 km beside 12's centreline, and at
    /// 7.6 nm 1,566 m left, 513 ft over the path, tracking 151 at 204 kt,
    /// a reversal began ("Repositioning for runway one two") that was still
    /// flying ten minutes later. From both recorded positions the final is
    /// re-intercepted: the guidance steers in toward the FAP, never out to a
    /// join point behind, and no pattern is needed. Too high to lose on the
    /// way in, or too far off to intercept, it is still a join point.
    #[test]
    fn a_retarget_to_the_near_parallel_reintercepts_its_final() {
        let rwy = Runway {
            icao: "LEMG".into(),
            id: "12".into(),
            thr_lat: 36.6910753,
            thr_lon: -4.5078270,
            course_deg: crate::nav::bearing_deg(36.6910753, -4.5078270, 36.6795648, -4.4805125),
            length_m: 2750.0,
            surface: 1,
            dist_nm: 0.0,
            elev_ft: 48.0,
        };
        let at = |dist: f64, xtk_m: f64| {
            let lateral = xtk_m / M_PER_NM;
            let along = (dist * dist - lateral * lateral).sqrt();
            let (lat, lon) = crate::reversal::pattern_point(&rwy, along, lateral);
            (lat, lon, along, lateral)
        };
        // (dist nm, xtk m, alt ft, radio ft, ias, gs, track): the retarget's
        // first tick on 12, and the tick the reversal began.
        for (dist, xtk_m, alt, radio, ias, gs, track) in [
            (9.67, -3525.0, 2274.0, 1373.0, 168.0, 177.0, 132.0),
            (7.60, -1566.0, 3030.0, 2124.0, 194.0, 204.0, 151.0),
        ] {
            let (lat, lon, along, lateral) = at(dist, xtk_m);
            let g = guidance(lat, lon, alt, radio, gs, ias, &rwy, 145.0, 25.0, 0.0);
            let above = alt - g.glidepath_alt_ft;
            assert!(!aims_at_join_point(above, along, lateral, gs), "{dist} nm {above:.0} ft over");
            let toward = crate::angles::abs_diff_deg(g.target_heading_deg, rwy.course_deg);
            assert!(toward.abs() < 45.0, "{dist} nm: steering {toward:.0} off the course");
            let aim = FAP_NM;
            assert!(!crate::reversal::needed(&rwy, lat, lon, track, aim, false), "{dist} nm");
        }
        // The rule as it was: the join point 10.8 nm out, behind at 7.6 nm.
        let (lat, lon, _, _) = at(7.60, -1566.0);
        let behind = join_point_nm(3030.0, rwy.elev_ft);
        assert!(behind > 10.0 && crate::reversal::needed(&rwy, lat, lon, 151.0, behind, false));
        // Past what the way in can lose, or beside a final no cut reaches.
        assert!(aims_at_join_point(1_200.0, 7.5, -0.85, 204.0));
        assert!(aims_at_join_point(600.0, 3.5, 0.0, 204.0), "inside the gate: no room");
        assert!(aims_at_join_point(600.0, 7.5, -5.0, 204.0), "not interceptable");
        assert!(!aims_at_join_point(450.0, 3.5, 0.0, 204.0), "within 500 ft: as before");
    }

    /// EGTE 2026-09-21: handed back at 152 kt with half flap and a flat 0.75
    /// lever; 346 kt followed.
    /// LEMG 13 (2026-09-26): armed at 2.6 nm, 800 ft high, lined up: a
    /// go-around, not a turn onto the downwind. Far out or pointing the
    /// wrong way, the pattern stands.
    #[test]
    fn a_late_high_approach_goes_around_before_it_repositions() {
        assert!(super::late_go_around(2.6, 7.0));
        assert!(!super::late_go_around(2.6, 120.0), "pointing away: the pattern");
        assert!(!super::late_go_around(super::FAP_NM + 2.0, 5.0), "outside the FAP: the pattern");
        assert!(!super::late_go_around(-0.5, 5.0), "past the threshold");
    }

    #[test]
    fn a_give_up_hands_back_a_speed_the_configuration_allows() {
        let limits = [
            (0.0, None),
            (0.5, Some(210.0)),
            (0.875, Some(175.0)),
            (1.0, Some(162.0)),
        ];
        let half = hand_back_speed_kts(148.0, 0.5, &limits);
        assert!((160.0..=195.0).contains(&half), "{half}");
        // Full flap still out: inside ITS placard, never under the approach speed.
        let full = hand_back_speed_kts(148.0, 1.0, &limits);
        assert!((148.0..=150.0).contains(&full), "{full}");
        // Clean: the clean approach speed.
        assert_eq!(
            hand_back_speed_kts(148.0, 0.0, &limits),
            clean_approach_ias(148.0)
        );
    }

    #[test]
    fn the_downwind_is_flown_faster_than_the_final_and_inside_half_flap() {
        let limits = [(0.0, None), (0.5, Some(210.0)), (1.0, Some(162.0))];
        let v = pattern_ias(148.0, &limits);
        assert!(v > 160.0 && v <= 195.0, "{v}");
        // A low half-flap placard caps it; never under the approach speed.
        let v = pattern_ias(65.0, &[(0.0, None), (0.5, Some(85.0)), (1.0, Some(85.0))]);
        assert!((65.0..=70.0).contains(&v), "{v}");
    }

    /// The pattern's downwind passes abeam the join point outbound: the old
    /// release test would let go there on a narrow pattern.
    #[test]
    fn a_held_reposition_ends_when_the_pattern_does() {
        let mut latch = GoAroundLatch::new();
        assert!(latch.order(false));
        let stage = latch.update(None, ApproachPhase::Intercept, 900.0, 1.0, -1.0, 5.75, 0.0);
        assert_eq!(stage, Some(MissedStage::Reposition));
        latch.hold_reposition();
        let stage = latch.update(None, ApproachPhase::Intercept, 1600.0, 5.8, 5.7, 5.75, 0.4);
        assert_eq!(
            stage,
            Some(MissedStage::Reposition),
            "held abeam the join point"
        );
        latch.finish_reposition();
        assert!(!latch.active());
        // Unheld, the old rule still releases there.
        let mut latch = GoAroundLatch::new();
        latch.order(false);
        latch.update(None, ApproachPhase::Intercept, 900.0, 1.0, -1.0, 5.75, 0.0);
        let stage = latch.update(None, ApproachPhase::Intercept, 1600.0, 5.8, 5.7, 5.75, 0.4);
        assert_eq!(stage, None);
    }

    #[test]
    fn true_to_mag_adds_xplane_variation() {
        // The live KPDK final (2026-07-14): variation dataref +5.65 (positive-
        // west), psi 191.3 true read back as magpsi 196.8. The bug must be dialed
        // ABOVE the true command here; the old subtraction flew 11 degrees left
        // of every commanded heading and crossed the threshold 0.4 nm off.
        assert!((true_to_mag(191.3, 5.65) - 196.95).abs() < 0.01);
        // Runway course 20.4 true at KPDK -> 26.05 mag, never 14.75.
        assert!((true_to_mag(20.4, 5.65) - 26.05).abs() < 0.01);
        // Wraps: east-variation field (negative dataref) just below north.
        assert!((true_to_mag(2.0, -5.0) - 357.0).abs() < 0.01);
    }

    #[test]
    fn mag_to_true_undoes_true_to_mag() {
        assert!((mag_to_true(196.95, 5.65) - 191.3).abs() < 0.01);
        assert!((mag_to_true(357.0, -5.0) - 2.0).abs() < 0.01);
        for var in [-20.0, -5.65, 0.0, 5.65, 20.0] {
            for t in [0.0, 1.0, 90.0, 179.5, 359.0] {
                let back = mag_to_true(true_to_mag(t, var), var);
                assert!((back - t).abs() < 1e-9, "{t} at {var}: {back}");
            }
        }
    }

    #[test]
    fn intercept_steers_to_the_fap_not_away_from_the_field() {
        // The live flee (XLE000N): outside the FAP the old law tracked the bare
        // runway course, which flies AWAY when the aircraft is past the threshold.
        // Place it 8 nm NORTH of a north-landing runway (past the field, departure
        // side): the command must steer SOUTH toward the FAP, not north on course.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 8.0); // 8 nm north, past the threshold
        let g = guidance(
            lat,
            lon,
            2000.0,
            2000.0,
            90.0,
            90.0,
            &rwy_north(),
            90.0,
            20.0,
            0.0,
        );
        assert_eq!(g.phase, ApproachPhase::Intercept);
        // FAP is 5 nm SOUTH of the threshold; from 8 nm north that's ~due south.
        assert!(
            crate::angles::abs_diff_deg(g.target_heading_deg, 180.0) < 20.0,
            "must steer toward the FAP/field, got hdg {}",
            g.target_heading_deg
        );
    }

    #[test]
    fn speed_protection_lowers_the_nose_before_a_stall() {
        // The live failure (XLE000N): stuck below the glidepath 13 nm out, holding
        // level, IAS decaying to 53 kt until the AP stalled off. Below the path AND
        // slow, the guidance must now command a DESCENT to regain speed, not a level
        // hold. 5 nm south, well below the path, 55 kt against a 90 kt approach.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 5.0);
        let g = guidance(
            lat,
            lon,
            200.0,
            200.0,
            55.0,
            55.0,
            &rwy_north(),
            90.0,
            20.0,
            0.0,
        );
        assert!(
            g.target_vs_fpm < -100.0,
            "slow + below path must descend, got {}",
            g.target_vs_fpm
        );
        // On speed at the same spot, it holds level to wait for the path (no dive).
        let g_ok = guidance(
            lat,
            lon,
            200.0,
            200.0,
            90.0,
            90.0,
            &rwy_north(),
            90.0,
            20.0,
            0.0,
        );
        assert_eq!(g_ok.target_vs_fpm, 0.0, "on speed below path → level hold");
        // ...and protection must never be SHALLOWER than the path the aircraft
        // is already flying. On a fast airframe the fixed 40 fpm/kt rate is
        // less than the path itself, so it used to REDUCE the descent — which
        // both loses the path and loses speed, the opposite of the intent. The
        // Luton MD-82 (2026-07-28) was 9 kt slow at 1.5 nm on a 3° path: the
        // path wanted ~740 fpm down, protection commanded 360, and it arrived
        // over the threshold 51 ft high and 12 kt fast for a go-around.
        let (jlat, jlon) = project_latlon(0.0, 0.0, 180.0, 1.5);
        let jet = guidance(
            jlat,
            jlon,
            520.0 + 1.5 * GLIDEPATH_FT_PER_NM,
            1.5 * GLIDEPATH_FT_PER_NM,
            140.0,
            131.0,
            &rwy_north(),
            140.0,
            25.0,
            0.0,
        );
        let path_vs = -(140.0 / 60.0) * GLIDEPATH_FT_PER_NM;
        assert!(
            jet.target_vs_fpm <= path_vs,
            "protection must not be shallower than the {path_vs:.0} fpm path, got {}",
            jet.target_vs_fpm
        );
        // Speed protection never fights the flare (we WANT to slow down landing).
        let g_flare = guidance(
            0.001,
            0.0,
            30.0,
            15.0,
            40.0,
            40.0,
            &rwy_north(),
            90.0,
            20.0,
            0.0,
        );
        assert_eq!(g_flare.phase, ApproachPhase::Flare);
        assert!(
            g_flare.target_vs_fpm > -200.0,
            "flare not overridden by speed floor"
        );
    }

    #[test]
    fn overflight_past_the_threshold_goes_around_to_the_fap() {
        // The live failure (2026-07-06): overflew the numbers, then tracked the
        // runway course straight out the departure end to 5.4 nm, never turning
        // back. Place the aircraft 0.5 nm PAST a north-landing threshold (0.5 nm
        // north, departure side) at 200 ft, on speed — inside the old "Final"
        // distance so the bug applied. It must now go around: phase Intercept,
        // steering SOUTH back toward the FAP, not north on the runway course.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 0.5); // 0.5 nm north, past the threshold
        let g = guidance(
            lat,
            lon,
            200.0,
            200.0,
            90.0,
            90.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert_eq!(
            g.phase,
            ApproachPhase::Intercept,
            "past the threshold → go around"
        );
        assert!(
            crate::angles::abs_diff_deg(g.target_heading_deg, 180.0) < 25.0,
            "must turn back toward the FAP, got hdg {}",
            g.target_heading_deg
        );
        // Sanity: a mirror point 0.5 nm SHORT of the threshold (approach side) is a
        // normal final, not a go-around.
        let (slat, slon) = project_latlon(0.0, 0.0, 180.0, 0.5);
        let g_final = guidance(
            slat,
            slon,
            200.0,
            200.0,
            90.0,
            90.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert_ne!(
            g_final.phase,
            ApproachPhase::Intercept,
            "short of the threshold is not a go-around"
        );
    }

    #[test]
    fn climbs_off_the_terrain_floor_only_with_energy() {
        // The live failure (2026-07-06): level at ~50 ft AGL 5 nm out over rising
        // terrain, no climb anywhere in the loop. 3 nm out, 200 ft AGL (below the
        // 500 ft floor), on the approach side.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 3.0);
        // FAST (approach + 10): surplus energy → bounded climb away from terrain.
        let g_fast = guidance(
            lat,
            lon,
            200.0,
            200.0,
            100.0,
            100.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert!(
            g_fast.target_vs_fpm > 0.0,
            "below the floor + fast → climb, got {}",
            g_fast.target_vs_fpm
        );
        assert!(
            g_fast.target_vs_fpm <= MAX_CLIMB_FPM,
            "climb is bounded, got {}",
            g_fast.target_vs_fpm
        );
        // And it asks the throttle for extra speed to sustain the climb.
        assert!(
            g_fast.target_ias_kts > 90.0,
            "climb raises the speed target, got {}",
            g_fast.target_ias_kts
        );
        // ON SPEED (no surplus): must NOT climb — hold level and let the governor
        // rebuild speed first (climbing slow would stall).
        let g_slow = guidance(
            lat,
            lon,
            200.0,
            200.0,
            90.0,
            90.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert_eq!(
            g_slow.target_vs_fpm, 0.0,
            "below the floor but no surplus → level, got {}",
            g_slow.target_vs_fpm
        );
    }

    #[test]
    fn terrain_floor_is_not_enforced_on_short_final() {
        // Low AGL close in is the approach, not terrain — don't climb away from it.
        // 0.5 nm short of the threshold, 200 ft AGL, fast: no climb (inside the
        // floor-enforcement range).
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 0.5);
        let g = guidance(
            lat,
            lon,
            200.0,
            200.0,
            100.0,
            100.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert!(
            g.target_vs_fpm <= 0.0,
            "no climb on short final, got {}",
            g.target_vs_fpm
        );
        assert_eq!(
            g.target_ias_kts, 90.0,
            "no climb-energy bonus on short final"
        );
    }

    #[test]
    fn floor_never_captures_an_on_path_final_at_a_flat_field() {
        // The LXGB 09 overflight (2026-07-22): a 3° path crosses 500 AGL at
        // ~1.6 nm, and the floor held a perfectly on-path approach level (and
        // hot, via the climb-energy bonus) from there to 1.0 nm — 216 ft high
        // at the numbers, guaranteed go-around at every flat field. On path
        // with low radio IS the approach: keep descending, no speed bonus.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 1.3);
        // On the TDZ-aimed path at 1.3 nm (~465 ft) — still below the 500 floor.
        let on_path = (1.3 + TOUCHDOWN_AIM_NM) * GLIDEPATH_FT_PER_NM;
        let g = guidance(
            lat,
            lon,
            on_path,
            on_path,
            90.0,
            90.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert_eq!(g.phase, ApproachPhase::Final);
        assert!(
            g.target_vs_fpm < -300.0,
            "on path must keep descending, got {}",
            g.target_vs_fpm
        );
        assert_eq!(
            g.target_ias_kts, 90.0,
            "no climb-energy bonus on an on-path final"
        );
        // Deeply below the path at the same distance: the floor still stands.
        let g_low = guidance(
            lat,
            lon,
            200.0,
            200.0,
            90.0,
            90.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert_eq!(
            g_low.target_vs_fpm, 0.0,
            "dragged-in-low keeps the floor, got {}",
            g_low.target_vs_fpm
        );
    }

    #[test]
    fn flare_balks_off_the_centreline_and_keeps_steering_on_it() {
        // The LXGB 09 crash (2026-07-22): the flare froze the bare runway
        // course with 0.10 nm of standing cross-track — touchdown 185 m right
        // of the centreline, wrecked in the grass. Off the pavement at flare
        // height must be a balked landing (Intercept → the latch climbs
        // straight ahead), never a touchdown.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 0.15);
        let off = lon + 0.10 / 60.0; // 0.10 nm right of the centreline
        let g = guidance(
            lat,
            off,
            20.0,
            20.0,
            80.0,
            80.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert_eq!(
            g.phase,
            ApproachPhase::Intercept,
            "off the pavement at 20 ft = balk"
        );
        // ...and the balk fires EARLY, from final at 80 ft — the 22 ft balk
        // still skimmed the ground at 91 kt (2026-07-22).
        let (hlat, hlon) = project_latlon(0.0, 0.0, 180.0, 0.4);
        let hoff = hlon + 0.05 / 60.0;
        let g_early = guidance(
            hlat,
            hoff,
            80.0,
            80.0,
            85.0,
            85.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert_eq!(
            g_early.phase,
            ApproachPhase::Intercept,
            "off-centreline at 80 ft = balk"
        );
        // On the centreline the flare still fires, and a small offset keeps a
        // gentle correction instead of locking the bare course.
        let near = lon + 0.01 / 60.0;
        let g_ok = guidance(
            lat,
            near,
            20.0,
            20.0,
            80.0,
            80.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert_eq!(g_ok.phase, ApproachPhase::Flare);
        let cut = crate::angles::signed_diff_deg(rwy_north().course_deg, g_ok.target_heading_deg);
        assert!(
            cut > 0.1 && cut <= 8.0,
            "flare keeps a bounded correction, got {cut}"
        );
    }

    #[test]
    fn intercept_holds_the_floor_even_close_to_the_threshold() {
        // The KPDK 16 rejoin (2026-07-14): armed 0.75 nm out on the WRONG side
        // of the threshold at ~300 AGL. Distance is symmetric, so the glidepath
        // read "on path at 0.7 nm" and the vertical law descended a repositioning
        // aircraft into the ground 0.35 nm short. In Intercept the floor must
        // hold at ANY distance: climb with surplus energy, level without — never
        // a glidepath descent.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 0.7); // 0.7 nm PAST a north-landing threshold
        let g_fast = guidance(
            lat,
            lon,
            300.0,
            300.0,
            100.0,
            100.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert_eq!(g_fast.phase, ApproachPhase::Intercept);
        assert!(
            g_fast.target_vs_fpm > 0.0,
            "repositioning low + fast → climb, got {}",
            g_fast.target_vs_fpm
        );
        let g_slow = guidance(
            lat,
            lon,
            300.0,
            300.0,
            90.0,
            90.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert_eq!(g_slow.phase, ApproachPhase::Intercept);
        assert_eq!(
            g_slow.target_vs_fpm, 0.0,
            "repositioning low + no surplus → level, got {}",
            g_slow.target_vs_fpm
        );
    }

    #[test]
    fn low_go_around_latches_straight_ahead_until_the_floor() {
        // The LXGB 09 crash (2026-07-14): overflew the numbers at ~57 ft, the
        // intercept guidance flipped the heading 178° toward the FAP, the flare
        // re-fired past the threshold and pulled the power, and the aircraft
        // hit 200 m south of the strip. Low, the latch must own the go-around.
        use ApproachPhase::*;
        use MissedStage::*;
        let mut latch = GoAroundLatch::new();
        // Just past the threshold, so along-track is negative throughout.
        let up = |l: &mut GoAroundLatch, p, n, r| l.update(Some(p), n, r, 0.2, -0.2, FAP_NM, 0.0);
        // Final ticking along at 60 ft: not a go-around.
        assert_eq!(up(&mut latch, Final, Final, 60.0), None);
        // Overflight: Final degrades to Intercept at 57 ft → latch, climbing.
        assert_eq!(up(&mut latch, Final, Intercept, 57.0), Some(Climb));
        // The raw guidance flickering (Intercept → Flare past the threshold →
        // Intercept) must not shake it loose while still low.
        assert_eq!(up(&mut latch, Intercept, Flare, 22.0), Some(Climb));
        assert_eq!(up(&mut latch, Flare, Intercept, 35.0), Some(Climb));
        // Above the old 500 ft floor but below the turn height: still climbing
        // straight ahead — turning back at 520 ft is how the Rock CFIT
        // happened (2026-07-22).
        assert_eq!(up(&mut latch, Intercept, Intercept, 520.0), Some(Climb));
        // At the turn height: start repositioning, but do NOT hand back yet —
        // 800 ft over the far end with the field behind us is not an approach.
        assert_eq!(
            up(&mut latch, Intercept, Intercept, 850.0),
            Some(Reposition)
        );
        // A HIGH overflight never latches: the immediate turn back is fine.
        let mut high = GoAroundLatch::new();
        assert_eq!(up(&mut high, Final, Intercept, 800.0), None);
    }

    /// The missed approach is a procedure, not a moment: the reposition stage
    /// runs until the aircraft is back where an approach starts. Anything less
    /// and the stateless guidance re-decides every tick — the Luton orbit,
    /// 1.1 to 4.1 nm for three and a half minutes (2026-07-28).
    #[test]
    fn the_reposition_runs_until_established_outside_the_fap() {
        use ApproachPhase::*;
        use MissedStage::*;
        let mut l = GoAroundLatch::new();
        // Platform height puts the join point ~5.75 nm out.
        let join = join_point_nm(1600.0, 0.0);
        assert_eq!(
            l.update(Some(Final), Intercept, 50.0, 0.2, -0.2, join, 0.0),
            Some(Climb)
        );
        assert_eq!(
            l.update(Some(Intercept), Intercept, 900.0, 1.0, -1.0, join, 0.0),
            Some(Reposition)
        );
        // Departure side, miles out: still repositioning, however far.
        assert_eq!(
            l.update(Some(Intercept), Intercept, 1600.0, 6.0, -6.0, join, 0.0),
            Some(Reposition)
        );
        // Approach side but short of the join point: an approach can't start
        // here, and handing back mid-outbound-leg means a 180 on final.
        assert_eq!(
            l.update(Some(Intercept), Intercept, 1600.0, 5.2, 5.2, join, 0.0),
            Some(Reposition)
        );
        // Abeam the join point but 2.7 nm beside the centreline (the 737's
        // turn-back, LFLL 2026-09-17): not an approach yet, keep homing.
        assert_eq!(
            l.update(
                Some(Intercept),
                Intercept,
                1600.0,
                join + 1.3,
                join + 1.0,
                join,
                -2.7
            ),
            Some(Reposition)
        );
        // Out at the join point on the approach side: hand back.
        assert_eq!(
            l.update(Some(Intercept), Intercept, 1600.0, join, join, join, 0.3),
            None
        );
    }

    /// A controller's order starts a missed approach from wherever we are,
    /// and it counts like any other.
    ///
    /// Malaga 2026-08-05: "cleared to land" and, one second later, "GO
    /// AROUND, traffic on the runway". Nothing consumed the order — the
    /// approach flew on for another seventy seconds through a 0.5 nm
    /// same-altitude conflict and only balked because it happened to be
    /// high.
    #[test]
    fn an_ordered_go_around_is_flown_from_wherever_the_approach_is() {
        use ApproachPhase::*;
        let mut l = GoAroundLatch::new();
        // Established on final, stable, nothing wrong with the approach: the
        // aircraft would never have called this one itself.
        assert_eq!(
            l.update(Some(Final), Final, 600.0, 2.0, 2.0, FAP_NM, 0.0),
            None
        );
        assert!(l.order(false), "the order starts one");
        assert_eq!(l.stage, Some(MissedStage::Climb));
        assert_eq!(l.attempts(), 1, "an ordered go-around is still a go-around");
        // Idempotent while one is already being flown — a controller
        // repeating the order must not restart the climb or burn an attempt.
        assert!(!l.order(false));
        assert_eq!(l.attempts(), 1);
        // And it flies on as usual from there: climb clear of the ground,
        // then reposition. (The raw guidance under a climb-away reads
        // Intercept — it is already turning back toward the FAP, which is
        // exactly what the latch exists to override until we are clear.)
        assert_eq!(
            l.update(Some(Final), Intercept, 900.0, 0.5, -0.5, 6.0, 0.0),
            Some(MissedStage::Reposition),
            "clear of the ground, so reposition"
        );

        // On the ground it is too late: a landing that has already happened
        // is not one to abandon, and the rollout is the safer place to be.
        let mut rolled = GoAroundLatch::new();
        assert!(!rolled.order(true));
        assert_eq!(rolled.attempts(), 0);
    }

    #[test]
    fn go_around_latch_counts_attempts_for_the_runaway_backstop() {
        use ApproachPhase::*;
        let mut l = GoAroundLatch::new();
        assert_eq!(l.attempts(), 0);
        // Each Final->Intercept low is one go-around; the latch releases once
        // the aircraft is repositioned outside the FAP, ready to try again.
        for n in 1..=3 {
            assert!(
                l.update(Some(Final), Intercept, 50.0, 0.2, -0.2, FAP_NM, 0.0)
                    .is_some(),
                "latched low"
            );
            assert_eq!(l.attempts(), n, "counted");
            assert!(
                l.update(Some(Intercept), Intercept, 50.0, 0.3, -0.3, FAP_NM, 0.0)
                    .is_some(),
                "stays latched climbing"
            );
            assert!(
                l.update(Some(Intercept), Intercept, 900.0, 2.0, -2.0, FAP_NM, 0.0)
                    .is_some(),
                "and stays latched repositioning"
            );
            assert!(
                l.update(Some(Intercept), Intercept, 1600.0, 5.5, 5.5, FAP_NM, 0.0)
                    .is_none(),
                "released once established at the join point"
            );
        }
        // The loop gives up once attempts EXCEED the cap.
        assert!(l.attempts() > MAX_GO_AROUNDS);
    }

    #[test]
    fn go_around_guidance_climbs_straight_ahead_on_speed() {
        let rwy = rwy_north();
        let climb = |ias| {
            missed_approach_guidance(
                &rwy,
                MissedStage::Climb,
                -0.005,
                0.0,
                200.0,
                150.0,
                0.3,
                100.0,
                ias,
                90.0,
                0.0,
                Some(70.0),
            )
        };
        // On speed: full climb, straight down the runway heading.
        let g = climb(92.0);
        assert_eq!(g.phase, ApproachPhase::Intercept);
        assert!(
            crate::angles::abs_diff_deg(g.target_heading_deg, rwy.course_deg) < 0.1,
            "straight ahead"
        );
        assert!(g.target_vs_fpm > 0.0 && g.target_vs_fpm <= MAX_CLIMB_FPM);
        // Slow (deficit beyond the floor margin): level acceleration first —
        // climbing slow at full flap is how a go-around stalls.
        let g_slow = climb(75.0);
        assert_eq!(g_slow.target_vs_fpm, 0.0, "build speed before climbing");
        assert!(crate::angles::abs_diff_deg(g_slow.target_heading_deg, rwy.course_deg) < 0.1);
    }

    /// The tower is owed the call, so the words have to be words it knows: the
    /// FSM sat in ClearedToLand through a four-minute missed approach because
    /// nothing was ever transmitted (Luton, 2026-07-28). This pins the phrase
    /// the loop sends against the parser that has to understand it.
    #[test]
    fn the_going_around_call_reads_as_a_go_around_to_the_tower() {
        let call = crate::circuit::radio_call(
            &Some("November five five two Alpha Alpha".to_string()),
            "going around",
        );
        assert_eq!(
            crate::tower::parse_intent(&call.to_ascii_lowercase()),
            crate::tower::Intent::GoAround,
            "{call}"
        );
        // ...and without a readable tail number it still parses.
        let bare = crate::circuit::radio_call(&None, "going around");
        assert_eq!(
            crate::tower::parse_intent(&bare),
            crate::tower::Intent::GoAround
        );
    }

    /// Both calls the vectored arrival makes have to be words the tower knows,
    /// for the same reason the go-around does. Asking for vectors and then
    /// reporting established is the sequence that earns the approach clearance
    /// and, after it, the landing clearance — a phrasing the parser shrugs at
    /// leaves the automation complying with a controller who never engaged.
    #[test]
    fn the_vectored_arrival_calls_read_as_the_tower_expects() {
        let cs = Some("November five five two Alpha Alpha".to_string());

        let ask = crate::circuit::radio_call(&cs, "request vectors");
        assert_eq!(
            crate::tower::parse_intent(&ask.to_ascii_lowercase()),
            crate::tower::Intent::RequestVectors,
            "{ask}"
        );
        // ...and with no readable tail number.
        assert_eq!(
            crate::tower::parse_intent(&crate::circuit::radio_call(&None, "request vectors")),
            crate::tower::Intent::RequestVectors
        );

        // "established" is the report the ILS clearance asks for, and it must
        // reach the tower as the FINAL report that earns the landing
        // clearance — not as an unknown.
        let est = crate::circuit::radio_call(&cs, "established");
        assert_ne!(
            crate::tower::parse_intent(&est.to_ascii_lowercase()),
            crate::tower::Intent::Unknown,
            "the established report must parse: {est}"
        );

        // The guard that makes "request vectors" safe: a ground call carrying
        // the same word is NOT a vectoring request (a real 2026-07-09 call).
        assert_ne!(
            crate::tower::parse_intent("request vectors for taxi to parking"),
            crate::tower::Intent::RequestVectors
        );
    }

    /// The reposition leg flies OUTBOUND to the join point and levels at
    /// platform height — it never dives at the glidepath under it, which is
    /// the path it is flying away from.
    #[test]
    fn the_reposition_leg_flies_outbound_and_holds_platform_height() {
        let rwy = rwy_north(); // course ~000, threshold at 0,0
                               // Half a mile past the far end at 900 ft, on speed: the join point is
                               // SOUTH (approach side), so the aircraft must turn around.
        let (lat, lon) = project_latlon(0.0, 0.0, 0.0, 0.5);
        let g = missed_approach_guidance(
            &rwy,
            MissedStage::Reposition,
            lat,
            lon,
            900.0,
            900.0,
            0.5,
            200.0,
            95.0,
            90.0,
            0.0,
            Some(70.0),
        );
        assert!(
            crate::angles::abs_diff_deg(g.target_heading_deg, 180.0) < 20.0,
            "outbound to the join point, got {}",
            g.target_heading_deg
        );
        assert!(g.target_vs_fpm > 0.0, "still below platform: climb");
        // At platform height it levels off rather than chasing the path down.
        let level = missed_approach_guidance(
            &rwy,
            MissedStage::Reposition,
            lat,
            lon,
            REPOSITION_MIN_AGL_FT + 100.0,
            REPOSITION_MIN_AGL_FT + 100.0,
            0.5,
            200.0,
            95.0,
            90.0,
            0.0,
            Some(70.0),
        );
        assert_eq!(level.target_vs_fpm, 0.0, "hold platform, never dive at it");
    }

    #[test]
    fn final_cuts_hard_enough_to_capture_the_centreline() {
        // The LXGB final (2026-07-14) entered 0.45 nm off the centreline and
        // the old 8°/nm gain commanded 3.6° of cut — it crossed the threshold
        // still ~120 m off ("coming in at an angle"). A meaningful offset must
        // earn a real intercept angle, decaying near the centreline.
        let rwy = rwy_north();
        // 3 nm south (on final), 0.45 nm EAST (right) of the centreline.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 3.0);
        let east = lon + 0.45 / 60.0;
        let g = guidance(lat, east, 900.0, 900.0, 90.0, 90.0, &rwy, 90.0, 20.0, 0.0);
        assert_eq!(g.phase, ApproachPhase::Final);
        // Right of course → cut LEFT (west of north): 0.45 × 40 clamps to 30.
        let cut = crate::angles::signed_diff_deg(rwy.course_deg, g.target_heading_deg);
        assert!(cut > 15.0, "0.45 nm off must earn a real cut, got {cut}");
        // Nearly on the centreline: gentle.
        let near = lon + 0.03 / 60.0;
        let g2 = guidance(lat, near, 900.0, 900.0, 90.0, 90.0, &rwy, 90.0, 20.0, 0.0);
        let cut2 = crate::angles::signed_diff_deg(rwy.course_deg, g2.target_heading_deg);
        assert!(cut2.abs() < 5.0, "small offset stays gentle, got {cut2}");
    }

    #[test]
    fn glidepath_descends_toward_the_threshold() {
        // 5 nm south of the threshold at 2000 ft, 120 kt groundspeed, tracking north.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 5.0); // 5 nm south
        let g = guidance(
            lat,
            lon,
            2000.0,
            2000.0,
            120.0,
            90.0,
            &rwy_north(),
            90.0,
            20.0,
            0.0,
        );
        assert!((g.dist_nm - 5.0).abs() < 0.2, "5 nm out, got {}", g.dist_nm);
        // Glidepath alt at 5 nm ≈ 1590 ft; we're above it → commanded to descend.
        assert!(g.glidepath_alt_ft > 1400.0 && g.glidepath_alt_ft < 1700.0);
        assert!(
            g.target_vs_fpm < 0.0,
            "should descend, got {}",
            g.target_vs_fpm
        );
        // Roughly on the centreline → heading close to the runway course (north).
        assert!(
            crate::angles::abs_diff_deg(g.target_heading_deg, 0.0) < 10.0,
            "hdg {}",
            g.target_heading_deg
        );
    }

    #[test]
    fn armed_high_overhead_flies_outbound_to_a_join_point() {
        // The live failure: 7,000 ft only 6 nm out (path altitude ~1,900) — the
        // old guidance aimed at the FAP and could never capture. Now it must fly
        // AWAY from the runway (toward the join point down the extended
        // centreline) and descend.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 6.0); // 6 nm south
        let g = guidance(
            lat,
            lon,
            7000.0,
            7000.0,
            135.0,
            110.0,
            &rwy_north(),
            110.0,
            20.0,
            0.0,
        );
        assert_eq!(g.phase, ApproachPhase::Intercept);
        // Runway is north (bearing ~0); the join point is further SOUTH — so the
        // commanded heading must be southbound, not at the runway.
        assert!(
            crate::angles::abs_diff_deg(g.target_heading_deg, 180.0) < 30.0,
            "should fly outbound, got hdg {}",
            g.target_heading_deg
        );
        assert!(
            g.target_vs_fpm <= -1000.0,
            "and descend hard, got {}",
            g.target_vs_fpm
        );
    }

    #[test]
    fn flare_near_the_ground_arrests_descent_and_tracks_runway() {
        let g = guidance(
            0.001,
            0.0,
            30.0,
            15.0,
            100.0,
            90.0,
            &rwy_north(),
            90.0,
            20.0,
            0.0,
        );
        assert_eq!(g.phase, ApproachPhase::Flare);
        assert!(
            g.target_vs_fpm > -200.0,
            "flare eases the descent, got {}",
            g.target_vs_fpm
        );
        assert!(
            crate::angles::abs_diff_deg(g.target_heading_deg, 0.0) < 1.0,
            "flare tracks the runway"
        );
    }

    #[test]
    fn low_radio_far_from_the_runway_is_terrain_not_a_flare() {
        // The LEMI failure: 7 ft AGL over rising terrain 4.1 nm out — the old
        // radio-only trigger "flared" (power to idle, vs -100) into open ground.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 4.1); // 4.1 nm south
        let g = guidance(
            lat,
            lon,
            900.0,
            7.0,
            130.0,
            90.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert_ne!(
            g.phase,
            ApproachPhase::Flare,
            "no flare 4 nm from the threshold"
        );
        // And the same low radio just short of the threshold IS the flare.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 0.1);
        let g = guidance(
            lat,
            lon,
            25.0,
            15.0,
            90.0,
            90.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert_eq!(g.phase, ApproachPhase::Flare);
    }

    /// The user-visible property the whole final law exists for: from a real
    /// offset at the top of final, the aircraft must be ESTABLISHED — cut
    /// under 3° — well before short final, not rolling out in the flare.
    /// March the kinematics through the guidance and check where alignment
    /// actually arrives.
    #[test]
    fn the_final_is_established_miles_before_the_threshold() {
        let rwy = rwy_north();
        let gs_kts = 120.0;
        // Top of final: 5 nm out, 0.4 nm right of the centreline.
        let mut d_nm = 5.0f64;
        let mut off_nm = 0.4f64;
        let mut established_at = None;
        for _ in 0..600 {
            if d_nm < 0.4 {
                break;
            }
            let (lat, lon) = project_latlon(0.0, 0.0, 180.0, d_nm);
            let lon_off = lon + off_nm / 60.0;
            let on_path = (d_nm + TOUCHDOWN_AIM_NM) * GLIDEPATH_FT_PER_NM;
            let g = guidance(
                lat, lon_off, on_path, on_path, gs_kts, 100.0, &rwy, 90.0, 25.0, 0.0,
            );
            let cut = crate::angles::wrap180(g.target_heading_deg);
            if established_at.is_none() && cut.abs() < 3.0 && off_nm.abs() < 0.05 {
                established_at = Some(d_nm);
            }
            // Fly one second along the commanded heading (no wind, perfect AP).
            let dt_h = 1.0 / 3600.0;
            let trk = g.target_heading_deg.to_radians();
            off_nm += gs_kts * dt_h * trk.sin();
            d_nm -= gs_kts * dt_h * trk.cos().abs();
        }
        let est = established_at.expect("the approach must become established");
        assert!(
            est > 2.0,
            "established by {est:.2} nm — short final is too late (the EGSS \
             approach aligned at 0.13 nm and it felt exactly as unsafe as it was)"
        );
    }

    #[test]
    fn final_pursuit_cut_scales_with_distance_remaining() {
        // 0.2 nm RIGHT of the centreline. The cut toward the line must be
        // DISTANCE-aware: modest 3 nm out, aggressive at 0.8 nm — the fixed
        // per-nm gain commanded 7° at 150 ft with 145 m to close and the
        // aircraft crossed short final diagonal (2026-07-24).
        let cut = |g: &Guidance| crate::angles::signed_diff_deg(g.target_heading_deg, 0.0);
        let at = |d_nm: f64| {
            let (lat, lon) = project_latlon(0.0, 0.0, 180.0, d_nm);
            let east = (lat, lon + 0.2 / 60.0);
            let on_path = (d_nm + TOUCHDOWN_AIM_NM) * GLIDEPATH_FT_PER_NM;
            guidance(
                east.0,
                east.1,
                on_path,
                on_path,
                90.0,
                90.0,
                &rwy_north(),
                90.0,
                25.0,
                0.0,
            )
        };
        let far = at(3.0);
        let close = at(0.8);
        // Offset RIGHT of a north course → cut LEFT (negative), and a REAL
        // intercept at BOTH distances. The fixed-lead pursuit replaced the
        // half-remaining aim precisely because the far-out cut was too shy:
        // with the aim at d/2 the offset decayed as d² and the aircraft rode
        // the whole final at an angle, aligning at 0.13 nm — over the numbers
        // (EGSS 22, 2026-07-28). Same offset now earns roughly the same cut
        // at any distance, which is what makes the convergence exponential
        // in time and the last miles wings-level.
        assert!(
            cut(&far) < -12.0,
            "far cut is a real intercept: {:.1}",
            cut(&far)
        );
        assert!(cut(&close) < -12.0, "close cut too: {:.1}", cut(&close));
        assert!(
            cut(&close) <= cut(&far) + 1.0,
            "never softer close-in: far {:.1} vs close {:.1}",
            cut(&far),
            cut(&close)
        );
        // And clamped — never a base-turn-sized swing on final.
        assert!(cut(&close) >= -35.5);
        // On the centreline: no cut either distance.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 1.5);
        let on_path = (1.5 + TOUCHDOWN_AIM_NM) * GLIDEPATH_FT_PER_NM;
        let g = guidance(
            lat,
            lon,
            on_path,
            on_path,
            90.0,
            90.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert!(cut(&g).abs() < 1.5, "centred: {:.1}", cut(&g));
        // SHORT final (inside the 0.3 nm pursuit floor), small offset (a big
        // one is the balk gate's problem): tiny corrections only — the
        // halfway aim fell BEHIND the aircraft at 0.06 nm and the command
        // thrashed ±35° at 100 ft over the numbers (2026-07-24).
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 0.06);
        let near = (lat, lon + 0.015 / 60.0); // ~28 m right
        let on_path = (0.06 + TOUCHDOWN_AIM_NM) * GLIDEPATH_FT_PER_NM;
        let over = guidance(
            near.0,
            near.1,
            on_path,
            on_path,
            90.0,
            90.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        assert!(
            cut(&over).abs() <= 8.5,
            "no thrash over the numbers: {:.1}",
            cut(&over)
        );
        assert!(
            cut(&over) < -0.5,
            "still corrects toward the centreline: {:.1}",
            cut(&over)
        );
    }

    #[test]
    fn glidepath_aims_at_the_touchdown_zone_not_the_threshold_edge() {
        // On the extended centreline 3 nm out, established: the commanded
        // glidepath altitude must be the 3° path aimed ~300 m PAST the threshold,
        // so it crosses the numbers at ~50 ft, not zero.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 3.0); // 3 nm south of thr
        let g = guidance(
            lat,
            lon,
            1000.0,
            1000.0,
            90.0,
            90.0,
            &rwy_north(),
            90.0,
            25.0,
            0.0,
        );
        let expected = (3.0 + TOUCHDOWN_AIM_NM) * GLIDEPATH_FT_PER_NM;
        assert!(
            (g.glidepath_alt_ft - expected).abs() < 1.0,
            "gp {} vs {expected}",
            g.glidepath_alt_ft
        );
        // Threshold-crossing height ~50 ft (the aim offset × slope), not 0.
        let tch = TOUCHDOWN_AIM_NM * GLIDEPATH_FT_PER_NM;
        assert!((45.0..=55.0).contains(&tch), "TCH ~50 ft, was {tch}");
    }

    #[test]
    fn full_flaps_wait_for_the_glidepath() {
        // Established on the path inside 3 nm: full flaps.
        assert_eq!(flap_schedule(2.0, 0.0), 1.0);
        assert_eq!(flap_schedule(2.0, 100.0), 1.0);
        // Dragging level BELOW the path (the LERL energy trap): stay at half.
        assert_eq!(flap_schedule(2.0, -300.0), 0.5);
        // Approach segment and clean segment unchanged.
        assert_eq!(flap_schedule(4.5, -300.0), 0.5);
        assert_eq!(flap_schedule(8.0, 0.0), 0.0);
    }

    #[test]
    fn a_distant_outbound_reposition_stays_clean() {
        // #211: `outbound` described a long visual reversal as well as a
        // close downwind, putting the 172's first notch out many miles early.
        assert_eq!(approach_leg_flap_schedule(34.5, -6_000.0, true), 0.0);
        assert_eq!(approach_leg_flap_schedule(23.0, -5_000.0, true), 0.0);
        // A real close downwind keeps the existing half-flap configuration.
        assert_eq!(approach_leg_flap_schedule(5.9, 400.0, true), 0.5);
        // Inbound legs still use the ordinary distance/vertical schedule.
        assert_eq!(approach_leg_flap_schedule(8.0, 0.0, false), 0.0);
        assert_eq!(approach_leg_flap_schedule(4.5, -300.0, false), 0.5);
        assert_eq!(approach_leg_flap_schedule(2.0, 0.0, false), 1.0);
    }

    #[test]
    fn the_landing_flap_is_selected_a_mile_before_the_gate() {
        // #199: at 3 nm the landing flap went out AT the 1,000 ft gate, so
        // the gate could never find the landing configuration. Now 3.98 nm
        // on a 3 degree path: EGNX 09's second approach (2026-09-30) at
        // 3.26 nm and 150 kt would have had it selected.
        assert!((landing_flap_nm() - 3.98).abs() < 0.02, "{}", landing_flap_nm());
        assert_eq!(flap_schedule(3.9, 0.0), 1.0);
        assert_eq!(flap_schedule(3.26, -10.0), 1.0);
        assert_eq!(flap_schedule(4.1, 0.0), 0.5);
        let gate_nm = 1000.0 / GLIDEPATH_FT_PER_NM - TOUCHDOWN_AIM_NM;
        assert!(landing_flap_nm() - gate_nm >= 1.0 - 1e-9);
    }

    #[test]
    fn cross_track_sign_is_right_of_course() {
        // Aircraft to the EAST of a north-landing runway's extended centreline (which
        // runs south of the threshold) → right of the inbound course → positive.
        let (lat, lon) = project_latlon(0.0, 0.0, 180.0, 3.0); // 3 nm south, on centreline
        assert!(
            cross_track_nm(lat, lon, &rwy_north()).abs() < 0.05,
            "on centreline ~0"
        );
        let east = (lat, lon + 0.02);
        assert!(
            cross_track_nm(east.0, east.1, &rwy_north()) > 0.0,
            "east = right = positive"
        );
    }

    #[test]
    fn bearing_cardinals() {
        // Due north / east / south / west from the equator.
        assert!((bearing_deg(0.0, 0.0, 1.0, 0.0) - 0.0).abs() < 0.5, "north");
        assert!((bearing_deg(0.0, 0.0, 0.0, 1.0) - 90.0).abs() < 0.5, "east");
        assert!(
            (bearing_deg(0.0, 0.0, -1.0, 0.0) - 180.0).abs() < 0.5,
            "south"
        );
        assert!(
            (bearing_deg(0.0, 0.0, 0.0, -1.0) - 270.0).abs() < 0.5,
            "west"
        );
    }

    #[test]
    fn heading_diff_wraps() {
        assert_eq!(crate::angles::abs_diff_deg(10.0, 350.0), 20.0);
        assert_eq!(crate::angles::abs_diff_deg(350.0, 10.0), 20.0);
        assert!((crate::angles::abs_diff_deg(90.0, 270.0) - 180.0).abs() < 1e-9);
        assert_eq!(crate::angles::abs_diff_deg(45.0, 45.0), 0.0);
    }

    #[test]
    fn parses_a_real_type_100_line() {
        // A real apt.dat land-runway record (ends 14/32).
        let line = "100 7.62 3 0 0.25 0 0 0 14 63.1361123 -142.5229183 0 0 1 0 0 0 32 63.1315349 -142.5209494 0 0 1 0 0 0";
        let (e1, e2, length_m, surface) = parse_runway_100(line).expect("parses");
        assert_eq!(e1.id, "14");
        assert_eq!(e2.id, "32");
        assert_eq!(surface, 3);
        // The two ends are the same runway → reciprocal courses (~180 apart).
        assert!((crate::angles::abs_diff_deg(e1.course, e2.course) - 180.0).abs() < 2.0);
        // ~510 m strip; sanity-bound the length.
        assert!(
            length_m > 300.0 && length_m < 800.0,
            "length was {length_m}"
        );
    }

    #[test]
    fn rejects_malformed_and_non_runway_lines() {
        assert!(parse_runway_100("1 100 0 0 KJFK").is_none());
        assert!(parse_runway_100("100 7.62 3").is_none()); // too few fields
    }

    // Real-data smoke test against the actual Global Airports apt.dat. Ignored by
    // default (363 MB, slow, machine-specific path); run with:
    //   cargo test autoland::tests::real_apt_dat -- --ignored --nocapture
    #[test]
    #[ignore]
    fn real_apt_dat_finds_a_known_airport() {
        let apt = format!(
            "{}/X-Plane 12/Global Scenery/Global Airports/Earth nav data/apt.dat",
            std::env::var("HOME").unwrap()
        );
        // Near KSEA (Seattle-Tacoma). Track roughly south to favour the 16s.
        let best = best_landing_runway(&apt, 47.449, -122.309, 160.0, 30.0, 1500.0)
            .expect("read apt.dat")
            .expect("a suitable runway near KSEA");
        println!(
            "selected: {} rwy {} @ {:.0} nm, {:.0} m, course {:.0}",
            best.icao, best.id, best.dist_nm, best.length_m, best.course_deg
        );
        assert_eq!(best.icao, "KSEA");
        assert!(best.length_m > 2000.0, "KSEA runways are long");
        assert!(best.dist_nm < 5.0, "should be right at the field");
    }

    // runway_by_id honours the arrival's already-chosen end: a bare ident is
    // paved-only, while an exact field may deliberately name its turf runway.
    #[test]
    fn runway_by_id_matches_the_named_hard_end() {
        let dir = std::env::temp_dir().join(format!("xcopilot_rwyid_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("apt.dat");
        // A hard runway 09/27 and a grass 05/23, both at ~test coords.
        std::fs::write(
            &path,
            "1 20 0 0 TEST Test Field\n\
             100 30 1 0 0.25 0 0 0 09 36.150 -5.360 0 0 1 0 0 0 27 36.150 -5.330 0 0 1 0 0 0\n\
             100 30 3 0 0.25 0 0 0 05 36.140 -5.350 0 0 1 0 0 0 23 36.160 -5.350 0 0 1 0 0 0\n",
        )
        .unwrap();
        let p = path.to_str().unwrap();
        // The into-wind end the arrival chose (27) resolves to the hard runway.
        let r = runway_by_id(p, 36.15, -5.34, "27", 40.0, None)
            .unwrap()
            .expect("27 is at the field");
        assert_eq!(r.id, "27");
        assert!(r.hard(), "27 is asphalt");
        // Its reciprocal is equally resolvable.
        assert_eq!(
            runway_by_id(p, 36.15, -5.34, "09", 40.0, None)
                .unwrap()
                .unwrap()
                .id,
            "09"
        );
        // A bare grass end is filtered out of emergency/autoland self-picks.
        assert!(
            runway_by_id(p, 36.15, -5.34, "05", 40.0, None)
                .unwrap()
                .is_none(),
            "grass is skipped"
        );
        // A plan explicitly naming this airport and end owns that choice.
        let grass = runway_by_id(p, 36.15, -5.34, "05", 40.0, Some("TEST"))
            .unwrap()
            .expect("the named turf runway resolves");
        assert_eq!((grass.icao.as_str(), grass.id.as_str(), grass.surface), ("TEST", "05", 3));
        // An ident that isn't here → None (caller falls back to track pick).
        assert!(
            runway_by_id(p, 36.15, -5.34, "18", 40.0, None)
                .unwrap()
                .is_none(),
            "no 18 here"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    // The circuit handoff must FORCE its own runway, because the track-based
    // search is free to prefer a different AIRFIELD entirely.
    //
    // Southampton, 2026-07-30: the circuit turned final at EGHI and handed to
    // autoland with `forced_rwy: None`. The search chose EGHL 09 sixteen miles
    // north, and the aircraft flew away from the field it had just circled for
    // five minutes, pinned in Intercept below a glidepath it could never
    // capture. This pins the DIFFERENCE between the two picks: on downwind
    // (heading away from the landing end), the track search does not return
    // the near field, and forcing the ident does.
    /// A runway ident is NOT unique, and the old lookup resolved it by
    /// proximity alone. EGPH-bound over the Forth, the nearest "24" was EGPJ
    /// (Fife) nine miles away, and the ILS was flown to a different airport
    /// (2026-09-20). The previous test could never catch this: its two fields
    /// had different idents, so the ambiguity never arose.
    #[test]
    fn a_planned_arrival_lands_at_the_airport_the_plan_named() {
        let dir = std::env::temp_dir().join(format!("xvl_two_24s_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("apt.dat");
        // Both fields have a 24. EGPJ is NEARER the aircraft; EGPH is the
        // destination. Geometry taken from the real pair across the Forth.
        std::fs::write(
            &path,
            "1 395 0 0 EGPJ Fife\n\
             100 45 1 0 0.25 0 0 0 06 56.180 -3.420 0 0 1 0 0 0 24 56.190 -3.390 0 0 1 0 0 0\n\
             1 136 0 0 EGPH Edinburgh\n\
             100 45 1 0 0.25 0 0 0 06 55.940 -3.400 0 0 1 0 0 0 24 55.960 -3.340 0 0 1 0 0 0\n",
        )
        .unwrap();
        let p = path.to_str().unwrap();
        // North of Edinburgh, closer to Fife — where the aircraft actually was.
        let (lat, lon) = (56.166, -3.286);

        // The bug: an ident with no airport behind it goes to the near field.
        let bare = runway_by_id(p, lat, lon, "24", 40.0, None)
            .unwrap()
            .expect("some 24 is in range");
        assert_eq!(bare.icao, "EGPJ", "proximity picks the wrong field");

        // The fix: the plan names its destination, so there is nothing to pick.
        let planned = runway_by_id(p, lat, lon, "24", 40.0, Some("EGPH"))
            .unwrap()
            .expect("EGPH has a 24");
        assert_eq!(planned.icao, "EGPH");
        assert!(
            planned.dist_nm > bare.dist_nm,
            "and it is the FURTHER one, which is the whole point"
        );

        // An ident the named field does not have is None — the caller falls
        // back to the track search rather than silently landing elsewhere.
        assert!(runway_by_id(p, lat, lon, "13", 40.0, Some("EGPH"))
            .unwrap()
            .is_none());
        // Case and whitespace in the ICAO must not send us to the wrong place.
        assert_eq!(
            runway_by_id(p, lat, lon, "24", 40.0, Some(" egph "))
                .unwrap()
                .map(|r| r.icao),
            Some("EGPH".to_string())
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_forced_ident_beats_the_track_search_at_the_wrong_field() {
        let dir = std::env::temp_dir().join(format!("xvl_circuit_rwy_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("apt.dat");
        // NEAR field: runway 02/20, the one we flew the pattern around.
        // FAR field ~16 nm north: runway 09/27, aligned nothing like our track.
        std::fs::write(
            &path,
            "1 20 0 0 NEAR Near Field\n\
             100 45 1 0 0.25 0 0 0 02 50.950 -1.360 0 0 1 0 0 0 20 50.965 -1.350 0 0 1 0 0 0\n\
             1 620 0 0 FAR Far Field\n\
             100 45 1 0 0.25 0 0 0 09 51.190 -1.040 0 0 1 0 0 0 27 51.190 -1.010 0 0 1 0 0 0\n",
        )
        .unwrap();
        let p = path.to_str().unwrap();
        // Downwind for 20: abeam the near field, tracking roughly 020 — i.e.
        // the RECIPROCAL of the landing direction, which is what made the
        // track search look elsewhere in the first place.
        let (lat, lon, hdg) = (50.958, -1.330, 20.0);

        // Forcing the pattern's ident lands us at the near field, every time.
        let forced = runway_by_id(p, lat, lon, "20", 40.0, None)
            .unwrap()
            .expect("20 is the pattern runway, right here");
        assert_eq!(forced.id, "20");
        assert!(
            forced.dist_nm < 5.0,
            "forced pick must be the field we are circling, got {:.1} nm",
            forced.dist_nm
        );

        // The bare track search is NOT required to agree, and that is the whole
        // point: whatever it returns, it is not authoritative for a circuit.
        // If it ever wanders to the far field, the forced pick above is what
        // keeps the aircraft home.
        if let Ok(Some(searched)) = best_landing_runway(p, lat, lon, hdg, 40.0, 1200.0) {
            assert!(
                forced.dist_nm <= searched.dist_nm,
                "forcing the ident must never be farther than the search: \
                 forced {} at {:.1} nm vs searched {} at {:.1} nm",
                forced.id,
                forced.dist_nm,
                searched.id,
                searched.dist_nm
            );
        }
        let _ = std::fs::remove_dir_all(&dir);
    }

    // X-Plane 12 numbers paved surfaces 20-38 and 50-57, not just the legacy
    // asphalt=1 / concrete=2. Accepting only [1, 2] hid 36% of the world's
    // usable runways from the emergency autoland, Heathrow among them.
    #[test]
    fn xp12_paved_surface_codes_are_landable() {
        // Legacy pair, still valid.
        assert!(is_hard_surface(1), "asphalt");
        assert!(is_hard_surface(2), "concrete");

        // The one that started it: EGHI 20 is surface 22, and it sat 2.16 nm
        // away while the search took Lasham at 15.81.
        assert!(is_hard_surface(22), "EGHI, KCLT, LTFJ");
        // A spread of real fields across the XP12 material range.
        for (code, who) in [
            (23, "ENGM / PANC"),
            (25, "EGLL"),
            (26, "EKCH / ENBR"),
            (27, "EFHK"),
            (29, "EGLL / EBCI"),
            (30, "EGLC / BIRK"),
            (33, "EGAA / LTFM"),
            (34, "ESSA / EKCH"),
            (35, "EBBR"),
            (37, "OMDB / LTAC"),
            (50, "CYYC / UKBB"),
            (51, "KSEA"),
            (55, "ULLI / KPDX"),
            (57, "KGEG"),
        ] {
            assert!(is_hard_surface(code), "surface {code} is paved: {who}");
        }

        // The soft ones must NEVER become a runway — this is why the check
        // stays a whitelist.
        for (code, what) in [
            (3, "turf"),
            (4, "dirt"),
            (5, "gravel"),
            (12, "dry lakebed"),
            (13, "water"),
            (14, "snow"),
            (15, "transparent"),
        ] {
            assert!(
                !is_hard_surface(code),
                "surface {code} ({what}) is not landable"
            );
        }

        // Unused/unknown codes are rejected rather than assumed paved.
        assert!(!is_hard_surface(0));
        assert!(!is_hard_surface(45), "39-49 unused in the global apt.dat");
        assert!(!is_hard_surface(99));
    }

    // A forced ident must survive the surface gate. `runway_by_id` filters on
    // hard() too, so before the XP12 codes were accepted, the circuit forcing
    // its own runway at Southampton would STILL have been rejected here and
    // fallen back to the track search that chose the wrong airfield.
    #[test]
    fn a_forced_ident_resolves_on_an_xp12_paved_runway() {
        let dir = std::env::temp_dir().join(format!("xvl_surf22_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("apt.dat");
        // EGHI-shaped: runway 02/20, surface 22, 1885 m.
        std::fs::write(
            &path,
            "1 44 0 0 EGHI Southampton\n\
             100 45 22 0 0.25 0 0 0 02 50.9425 -1.3660 0 0 1 0 0 0 20 50.9585 -1.3520 0 0 1 0 0 0\n",
        )
        .unwrap();
        let p = path.to_str().unwrap();
        let r = runway_by_id(p, 50.984, -1.311, "20", 40.0, None)
            .unwrap()
            .expect("surface 22 is a paved runway and must resolve");
        assert_eq!(r.id, "20");
        assert_eq!(r.surface, 22);
        assert!(r.dist_nm < 5.0, "it is right there: {:.1} nm", r.dist_nm);
        let _ = std::fs::remove_dir_all(&dir);
    }

    // Gibraltar 2026-09-11: the straight-in handed off at 195 kt and the gear
    // and flaps went out at once, 40 kt over the Baron's placards. The config
    // now waits for the speed the profile allows and goes out as it decays.
    #[test]
    fn gear_and_flaps_wait_for_their_placard_speeds() {
        // Baron 58: VLO 152, approach flap Vfe 152, full flap Vfe 122.
        const BARON: [(f64, Option<f64>); 3] =
            [(0.0, None), (0.5, Some(152.0)), (1.0, Some(122.0))];
        assert!(
            !gear_may_extend(195.0, Some(152.0)),
            "no gear at 195 on a 152 placard"
        );
        assert!(gear_may_extend(150.0, Some(152.0)));
        assert!(
            gear_may_extend(195.0, None),
            "no figure, no gate (the old behaviour)"
        );

        assert_eq!(
            flap_within_limits(0.5, 195.0, &BARON),
            0.0,
            "held clean at 195"
        );
        assert_eq!(
            flap_within_limits(0.5, 145.0, &BARON),
            0.5,
            "approach flap once under 152"
        );
        assert_eq!(
            flap_within_limits(1.0, 130.0, &BARON),
            0.5,
            "full wanted, 130 kt: only approach"
        );
        assert_eq!(
            flap_within_limits(1.0, 100.0, &BARON),
            1.0,
            "full flap under 122"
        );
        assert_eq!(
            flap_within_limits(0.0, 195.0, &BARON),
            0.0,
            "retraction never gated"
        );
        assert_eq!(
            flap_within_limits(1.0, 195.0, &[]),
            1.0,
            "no flap profile: schedule stands"
        );
    }

    // The approach speed is what you cross the threshold at, NOT what you fly
    // the intercept at. Commanding it from the moment autoland arms flew an
    // SF50 (approach 87 kt) at 92 kt nine miles out with the gear up, and the
    // airframe's own gear warning called it (Southampton, 2026-07-30).
    #[test]
    fn the_approach_speed_is_flown_at_the_faf_not_at_range() {
        const VAPP: f64 = 87.0; // SF50, profiles/cirrus_vision_sf50.yaml

        // Far out: clean manoeuvring speed, near what it was actually flying
        // when the circuit handed off (125 kt).
        let far = scheduled_ias(16.0, VAPP);
        assert!(
            (120.0..=132.0).contains(&far),
            "clean intercept speed should be ~126 kt, got {far:.0}"
        );
        assert_eq!(far, scheduled_ias(40.0, VAPP), "flat beyond the ramp");

        // The failure datapoint: 8.8 nm out it was doing 92. It should be well
        // above the approach speed there.
        let at_bug = scheduled_ias(8.8, VAPP);
        assert!(
            at_bug > VAPP + 20.0,
            "at 8.8 nm we must still be clean-ish, got {at_bug:.0}"
        );

        // Gear comes down at 6 nm — comfortably below the SF50's 160 kt Vlo.
        assert!(scheduled_ias(6.0, VAPP) < 160.0, "gear speed must be legal");

        // Stabilised at the approach speed by the FAP and all the way in.
        assert_eq!(scheduled_ias(FAP_NM, VAPP), VAPP);
        assert_eq!(scheduled_ias(2.0, VAPP), VAPP);
        assert_eq!(scheduled_ias(0.0, VAPP), VAPP);

        // Monotonic: never speeds up as it gets closer, never below Vapp.
        let mut prev = f64::MAX;
        let mut d = 20.0;
        while d >= 0.0 {
            let v = scheduled_ias(d, VAPP);
            assert!(
                v <= prev + 1e-9,
                "speed must not rise on the way in at {d} nm"
            );
            assert!(v >= VAPP - 1e-9, "never below the approach speed at {d} nm");
            prev = v;
            d -= 0.25;
        }

        // Scales with the airframe, so a 172 is not asked to fly a jet profile.
        assert!(scheduled_ias(16.0, 65.0) < 100.0, "172 stays a 172");
    }

    /// The pursuit lead must stay FIXED all the way to the threshold. It did
    /// not: flooring the aim at the threshold let the effective lead shrink
    /// inside one lead distance, which is the d/2 law the fixed lead was
    /// introduced to replace, back on short final where being lined up
    /// matters most. Southampton: the cut sat at 16 degrees from 0.87 nm to
    /// 0.32, then snapped to the runway course over the numbers with the
    /// aircraft still 6 degrees off and still turning (2026-08-01).
    #[test]
    fn the_pursuit_lead_does_not_collapse_on_short_final() {
        let (gs, len) = (105.0, 1723.0); // the Baron at EGHI 02
        let lead = |along: f64| along - pursuit_aim_nm(along, gs, len);

        // The same lead at every distance, far out and short final alike.
        let far = lead(1.68);
        for along in [1.2, 0.9, 0.7, 0.5, 0.35] {
            assert!(
                (lead(along) - far).abs() < 1e-9,
                "lead collapsed to {:.3} at {along} nm (was {far:.3})",
                lead(along)
            );
        }

        // Which means the aim walks PAST the threshold, onto the runway. That
        // is the point: the extended centreline and the runway centreline are
        // the same line.
        assert!(pursuit_aim_nm(0.35, gs, len) < 0.0);

        // ...but never past the far end of it.
        let past = -pursuit_aim_nm(0.0, gs, len);
        assert!(
            past <= len / M_PER_NM,
            "aim {past:.2} nm past the threshold on a {:.2} nm runway",
            len / M_PER_NM
        );

        // The cut therefore keeps shrinking with the offset instead of
        // holding: half the offset at the same range is half the cut.
        let cut = |xtk: f64, along: f64| (xtk / (along - pursuit_aim_nm(along, gs, len))).atan();
        assert!(cut(0.25, 0.7) > cut(0.125, 0.7));
        // ...and closing in at a FIXED offset no longer grows it.
        assert!((cut(0.1, 1.2) - cut(0.1, 0.4)).abs() < 1e-9);
    }
    #[test]
    fn the_self_pick_knows_the_airframe_the_wind_and_the_longer_parallel() {
        // The A330 lands in 4,750 ft (1,448 m, a factored field length):
        // nothing under it. A 172, or no figure on file, keeps the old 1,200.
        assert!((self_pick_min_length_m(Some(4750.0 * 0.3048)) - 1447.8).abs() < 1.0);
        assert_eq!(self_pick_min_length_m(Some(407.0)), 1200.0);
        assert_eq!(self_pick_min_length_m(None), 1200.0);
        assert_eq!(self_pick_min_length_m(Some(f64::NAN)), 1200.0);

        // Opposite ends of one runway, the downwind end nearer and dead
        // ahead: 12 kt on the tail loses to the into-wind end 3 nm further
        // on with a turn (LXGB 27 into 090 at 21 is the case on file).
        let wind = Some((240.0, 12.0));
        let downwind_end = landing_score(5.0, 0.0, 60.0, wind, 2500.0, 1200.0);
        let into_wind_end = landing_score(8.0, 40.0, 240.0, wind, 2500.0, 1200.0);
        assert!(
            into_wind_end < downwind_end,
            "{into_wind_end} vs {downwind_end}"
        );
        // With no wind known the nearer end ahead still wins, as before.
        assert!(
            landing_score(5.0, 0.0, 60.0, None, 2500.0, 1200.0)
                < landing_score(8.0, 40.0, 240.0, None, 2500.0, 1200.0)
        );
        // A crosswind is nobody's tailwind.
        assert_eq!(
            landing_score(5.0, 0.0, 60.0, Some((150.0, 20.0)), 1200.0, 1200.0),
            5.0
        );
        // Parallels: the threshold 0.2 nm nearer loses to 1,000 m more runway.
        assert!(
            landing_score(6.2, 5.0, 130.0, None, 3200.0, 1200.0)
                < landing_score(6.0, 5.0, 130.0, None, 2200.0, 1200.0)
        );
        // ...but length never outbids a field that is really nearer.
        assert!(
            landing_score(6.0, 5.0, 130.0, None, 1500.0, 1200.0)
                < landing_score(7.0, 5.0, 130.0, None, 4000.0, 1200.0)
        );
    }

    /// LIPK (2026-09-25): "land the aircraft" 6.4 nm from both Forli 30 and
    /// Cervia 11, with LIPE-LIPK on the shelf, went to Cervia. The plan's
    /// destination is the pick whenever it has a suitable runway in range.
    #[test]
    fn the_self_pick_prefers_the_plans_destination() {
        let apt = std::env::temp_dir().join(format!("xvl_selfpick_{}.dat", std::process::id()));
        std::fs::write(
            &apt,
            "1     97 0 0 LIPK Forli\n\
             100 45.00 23 1 0.25 0 3 0  12 44.2001695 12.0562203 0 0 3 3 0 1 30 44.1897225 12.0848141 147 8 3 1 0 0\n\
             1     18 0 0 LIPC Cervia Pisignano\n\
             100 45.00 1 701 0.25 0 3 1  11 44.2303638 12.2901111 119 0 3 1 0 0 29 44.2182383 12.3243840 116 0 3 7 0 0\n",
        )
        .unwrap();
        let path = apt.to_str().unwrap();
        let pick = |dest: Option<&str>| {
            best_landing_runway_in_wind(path, 44.1447, 12.2021, 8.0, 40.0, 1200.0, &|_| None, dest)
                .unwrap()
                .map(|r| format!("{} {}", r.icao.trim(), r.id))
        };
        assert_eq!(pick(None).as_deref(), Some("LIPC 11"), "the old pick, no plan");
        assert_eq!(pick(Some("LIPK")).as_deref(), Some("LIPK 30"));
        assert_eq!(pick(Some("lipk")).as_deref(), Some("LIPK 30"));
        // A destination out of range changes nothing.
        assert_eq!(pick(Some("EGLL")).as_deref(), Some("LIPC 11"));
        let _ = std::fs::remove_file(&apt);
    }

    /// LEMG 737, 2026-10-05 12:52:22: "land the aircraft" at 8,000 ft, 25 nm
    /// south-west of Malaga and 26 nm from Gibraltar, tracking 178. The 8 nm
    /// bonus lost to the turn: Gibraltar 09 was picked, twice that day. The
    /// destination is now the answer whenever it has a runway in reach.
    #[test]
    fn the_self_pick_keeps_to_the_destination_over_a_nearer_field() {
        let apt = std::env::temp_dir().join(format!("xvl_selfpick_lemg_{}.dat", std::process::id()));
        std::fs::write(
            &apt,
            "1     11 0 0 LXGB Gibraltar\n\
             100 45.11 1 0 0.20 0 3 0  09 36.1508156 -5.3596312 136 0 5 1 0 3 27 36.1514944 -5.3400082 98 0 5 7 0 3\n\
             1     48 0 0 LEMG Malaga\n\
             100 45.11 1 1 0.00 1 3 0  12 36.6910753 -4.5078270 0 190 3 1 1 1 30 36.6795648 -4.4805125 0 489 3 0 0 1\n\
             100 45.11 27 27 0.00 1 3 0  13 36.6845573 -4.5126253 0 75 3 1 0 2 31 36.6654027 -4.4858140 0 75 3 1 0 3\n",
        )
        .unwrap();
        let path = apt.to_str().unwrap();
        let wind = |icao: &str| match icao {
            "LXGB" => Some((90.0, 5.0)),
            _ => Some((130.0, 8.0)),
        };
        let pick = |dest: Option<&str>| {
            best_landing_runway_in_wind(path, 36.3438, -4.8796, 178.5, 40.0, 1633.7, &wind, dest)
                .unwrap()
                .map(|r| format!("{} {}", r.icao.trim(), r.id))
        };
        assert_eq!(pick(None).as_deref(), Some("LXGB 09"), "the day's pick, no destination");
        let at_malaga = pick(Some("LEMG")).unwrap();
        assert!(at_malaga.starts_with("LEMG "), "{at_malaga}");
        assert_eq!(at_malaga, "LEMG 13", "into the 8 kt from 130");
        let _ = std::fs::remove_file(&apt);
    }

    #[test]
    fn the_vs_watchdog_excuses_a_slow_autopilot_and_names_what_is_commanded() {
        // LEMI: cmd 0, diving 2,300 and not recovering. Counts, every tick.
        // A 0.33 s tick, the MacBook's Final (the rate is per second).
        let t = 0.33;
        assert!(vs_not_held(-2300.0, None, t));
        assert!(vs_not_held(-2300.0, Some(-2300.0), t));
        assert!(vs_not_held(-2400.0, Some(-2300.0), t));
        // The A330 at the arm: a -1,200 step from level, closing at 40 fpm a
        // tick. Far from the command, and being flown toward it.
        assert!(!vs_not_held(1100.0, Some(1140.0), t));
        // Closing too slowly to be the autopilot's doing.
        assert!(vs_not_held(1100.0, Some(1105.0), t));
        // Inside the band is held, whatever the trend.
        assert!(!vs_not_held(650.0, Some(100.0), t));
        // The same closing RATE judged alike on a slower tick: 40 fpm in
        // 0.33 s is 121 fpm/s, held; at 0.56 s 40 fpm is 71 fpm/s, still
        // held; 5 fpm in 0.56 s (9 fpm/s) is not.
        assert!(!vs_not_held(1100.0, Some(1140.0), 0.56));
        assert!(vs_not_held(1100.0, Some(1105.0), 0.56));
        assert!(vs_not_held_line(-1200.0).contains("descent"));
        assert!(vs_not_held_line(150.0).contains("climb"));
        assert!(vs_not_held_line(0.0).contains("level"));
    }

    #[test]
    fn the_altitude_dial_is_parked_above_the_arm() {
        // LEAX 2026-09-15: the plan's 2,600 ft dialled, the arm at 2,733 ft
        // indicated, the capture already arresting the descent. Parked 1,000
        // up, rounded to the hundred; VS re-selected by the caller.
        assert_eq!(parked_altitude_dial_ft(2600.0, 2733.0), Some(3800.0));
        // Dial at the aircraft's own altitude (an ALT hold at the arm): parked.
        assert_eq!(parked_altitude_dial_ft(2700.0, 2700.0), Some(3700.0));
        // Cruise altitude still dialled far above: nothing to do.
        assert_eq!(parked_altitude_dial_ft(8000.0, 2733.0), None);
        // Just inside the clear band counts as in the way.
        assert_eq!(parked_altitude_dial_ft(3100.0, 2733.0), Some(3800.0));
        assert_eq!(parked_altitude_dial_ft(3300.0, 2733.0), None);
    }

    #[test]
    fn the_glidepath_aims_at_the_probed_threshold() {
        // LEAX: header 160 ft, the 30 threshold probed at 117, the aim point
        // 300 m in at 121 - the higher of the two is the aim.
        assert_eq!(
            aim_elevation_ft(160.0, Some(&[117.0, 121.0])),
            (121.0, Some(121.0))
        );
        // Mesh and header agree within the trust band: the header stays.
        assert_eq!(
            aim_elevation_ft(160.0, Some(&[150.0, 158.0])),
            (160.0, Some(158.0))
        );
        // No probe (old plugin, paused sim): the header, and nothing seen.
        assert_eq!(aim_elevation_ft(160.0, None), (160.0, None));
        assert_eq!(aim_elevation_ft(160.0, Some(&[])), (160.0, None));
        // An absurd probe (a point in the sea beside the strip) is reported,
        // never flown.
        assert_eq!(
            aim_elevation_ft(160.0, Some(&[-500.0, -480.0])),
            (160.0, Some(-480.0))
        );
        // The LEAX approach: aimed at 121 the threshold crossing is ~57 ft
        // over the pavement; aimed at 160 it was 99 ft (radio) and balked.
        let tch = |aim: f64| aim + TOUCHDOWN_AIM_NM * GLIDEPATH_FT_PER_NM - 117.0;
        assert!(tch(121.0) < 65.0 && tch(121.0) > 45.0, "{}", tch(121.0));
        assert!(tch(160.0) > 90.0);
    }
