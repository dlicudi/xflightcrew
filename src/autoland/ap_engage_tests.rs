    use super::*;

    /// An arm over an autopilot already flying (the circuit's, full auto's):
    /// confirmed on the second live read, never held, never re-pressed.
    #[test]
    fn an_engaged_autopilot_is_confirmed_and_never_held() {
        let mut e = ApEngage::default();
        assert!(e.arm(0.0).fire);
        let s = e.step(0.6, Some(1.0), 150.0);
        assert!(!s.fire && !s.hold && !s.confirmed_now);
        let s = e.step(1.2, Some(1.0), 150.0);
        assert!(s.confirmed_now && !s.hold);
        assert_eq!(e.tries(), 1);
        assert_eq!(
            e.step(1.8, Some(0.0), 150.0),
            EngageStep::default(),
            "confirmed is sticky"
        );
    }

    /// The lost press: re-pressed every tick while the servos read off,
    /// held level meanwhile, confirmed only on live reads; a read on while
    /// the flight is still starting (IAS 5) is not a confirmation, and the
    /// flight start dropping it is pressed again.
    #[test]
    fn a_lost_press_is_pressed_again_until_the_servos_read_on() {
        let mut e = ApEngage::default();
        e.arm(0.0);
        let s = e.step(0.05, Some(0.0), 12.0);
        assert!(!s.fire && s.hold, "too soon to press again, but held");
        let s = e.step(0.65, Some(0.0), 85.0);
        assert!(s.fire && s.hold && s.try_n == 2);
        let s = e.step(1.25, Some(1.0), 5.0);
        assert!(
            !s.fire && !s.hold && !s.confirmed_now,
            "not live: no confirmation"
        );
        let s = e.step(1.85, Some(0.0), 132.0);
        assert!(s.fire && s.hold && s.try_n == 3, "the start dropped it");
        let s = e.step(2.45, None, 150.0);
        assert!(
            !s.confirmed_now && s.hold && s.fire && s.try_n == 4,
            "a failed read is no measurement: still pressed, still held"
        );
        let s = e.step(3.05, Some(1.0), 160.0);
        assert!(!s.confirmed_now && !s.fire && !s.hold);
        let s = e.step(3.65, None, 160.0);
        assert!(!s.confirmed_now, "a failed read does not confirm");
        let s = e.step(4.25, Some(1.0), 160.0);
        assert!(s.confirmed_now && s.try_n == 4);
    }

    /// An autopilot that never comes in: said and handed over after
    /// ENGAGE_GIVE_UP_S of live flight, never before, never while starting.
    #[test]
    fn an_autopilot_that_never_engages_is_handed_over() {
        let mut e = ApEngage::default();
        e.arm(0.0);
        let mut t = 0.6;
        while t < 5.0 {
            assert!(!e.step(t, Some(0.0), 0.0).give_up, "not live yet at {t}");
            t += 0.6;
        }
        let live_from = t;
        let mut gave = None;
        while t < 30.0 {
            if e.step(t, Some(0.0), 170.0).give_up {
                gave = Some(t);
                break;
            }
            t += 0.6;
        }
        let gave = gave.expect("gave up");
        assert!(
            gave - live_from >= ENGAGE_GIVE_UP_S - 1e-9
                && gave - live_from < ENGAGE_GIVE_UP_S + 0.7,
            "{gave} from {live_from}"
        );
        assert!(
            !ENGAGE_GIVE_UP_LINE.contains('\u{2014}') && !ENGAGE_GIVE_UP_LINE.contains('\u{2013}')
        );
    }

    /// The 737's pitch plant, fitted to the seven LEMG flares of 2026-09-26
    /// (4,970 fpm per unit of elevator through lags of 0.84 and 3.1 s after
    /// 1.35 s of dead time), with the new-flight start of LEMG 13 (sortie
    /// 1790587073-approach, 2026-09-28) as its disturbance: the sim starts
    /// the flight at 1.3 s descending 900 fpm with 0.54 of nose-down
    /// mistrim, read through a 1 s VSI (fitted to the recorded ticks, 112
    /// fpm rms). The servos: the flight start drops any engage made before
    /// it; a press after it lands at the next frame.
    struct NewFlight {
        t: f64,
        hist: std::collections::VecDeque<f64>,
        x1: f64,
        x2: f64,
        vsi: f64,
        alt_ft: f64,
        servos: bool,
        pressed: bool,
    }

    const DT: f64 = 0.05;
    const START_S: f64 = 1.3;
    const START_FPM: f64 = -900.0;
    const MISTRIM: f64 = -0.54;
    const FPM_PER_ELEV: f64 = 4970.0;
    const DEAD_S: f64 = 1.35;
    const ALT0_FT: f64 = 2318.2;

    impl NewFlight {
        fn new() -> Self {
            let n = (DEAD_S / DT).round() as usize;
            Self {
                t: 0.0,
                hist: std::iter::repeat_n(0.0, n).collect(),
                x1: 0.0,
                x2: 0.0,
                vsi: 0.0,
                alt_ft: ALT0_FT,
                servos: false,
                pressed: false,
            }
        }
        fn press(&mut self) {
            self.pressed = true;
        }
        /// Fly to `until` with `elev` on the channel (the loop's
        /// sample-and-hold: one write per tick).
        fn run(&mut self, until: f64, elev: f64) {
            while self.t < until - 1e-9 {
                let live = self.t >= START_S;
                if !live {
                    // The flight start wipes whatever was engaged before it.
                    self.servos = false;
                    self.pressed = false;
                } else if self.pressed {
                    self.servos = true;
                    self.pressed = false;
                }
                let e_in = if live { MISTRIM + elev } else { 0.0 };
                self.hist.push_back(e_in);
                let e = self.hist.pop_front().unwrap_or(0.0);
                self.x1 += (FPM_PER_ELEV * e - self.x1) * DT / 0.84;
                self.x2 += (self.x1 - self.x2) * DT / 3.1;
                let truth = if live { START_FPM + self.x2 } else { 0.0 };
                self.vsi += (truth - self.vsi) * DT / 1.0;
                self.alt_ft += truth / 60.0 * DT;
                self.t += DT;
            }
        }
    }

    /// The autoland ticks of 1790587073 (seconds from the setup's result,
    /// IAS kt, the VSI read fpm) until the AP came in at +10.9 s.
    const TICKS: [(f64, f64, f64); 18] = [
        (0.9, 12.4, -5.0),
        (1.5, 85.3, -207.0),
        (2.1, 132.1, -533.0),
        (2.7, 153.8, -819.0),
        (3.3, 162.4, -1017.0),
        (3.9, 165.9, -1167.0),
        (4.5, 167.7, -1311.0),
        (5.1, 168.9, -1502.0),
        (5.7, 169.6, -1720.0),
        (6.3, 170.2, -1965.0),
        (6.9, 170.9, -2208.0),
        (7.5, 171.8, -2428.0),
        (8.1, 173.1, -2642.0),
        (8.7, 174.4, -2836.0),
        (9.3, 175.6, -3027.0),
        (9.9, 176.9, -3193.0),
        (10.5, 178.1, -3337.0),
        (11.0, 179.4, -3450.0),
    ];
    /// The arm's one press, just before the first tick (threshold_elev +0.8).
    const ARM_S: f64 = 0.85;

    /// The old arm: one press, never read back. Nobody flies until the
    /// watchdog's re-press at +9.5 s. The plant reproduces the tape.
    #[test]
    fn the_old_arm_lets_the_new_flight_sink_until_the_watchdog() {
        let mut p = NewFlight::new();
        p.run(ARM_S, 0.0);
        p.press();
        let mut sq = 0.0;
        for &(t, _, vvi) in &TICKS {
            p.run(t, 0.0);
            assert!(!p.servos, "the arm's press was lost at {t}");
            sq += (p.vsi - vvi).powi(2);
        }
        let rms = (sq / TICKS.len() as f64).sqrt();
        assert!(
            rms < 150.0,
            "the plant is the recorded flight: {rms:.0} fpm rms"
        );
        assert!(
            p.vsi < -3000.0,
            "the sink handed to the late AP: {:.0}",
            p.vsi
        );
        assert!(
            ALT0_FT - p.alt_ft > 250.0,
            "lost {:.0} ft before anyone flew",
            ALT0_FT - p.alt_ft
        );
    }

    /// The new arm on the same flight: pressed until the servos READ on,
    /// and held level on the elevator until then. The AP is in within two
    /// seconds of the flight starting and inherits a sink it can hold.
    #[test]
    fn a_setup_approach_is_flown_from_the_first_tick() {
        let mut p = NewFlight::new();
        p.run(ARM_S, 0.0);
        let mut e = ApEngage::default();
        assert!(e.arm(0.0).fire);
        p.press();
        let mut law = LevelHold::with_gains(engage_hold_gains());
        let mut prev: Option<(f64, f64)> = None;
        let mut elev = 0.0;
        let mut confirmed_at = None;
        let mut held_ticks = 0;
        for &(t, ias, _) in &TICKS {
            p.run(t, elev);
            let servos = Some(if p.servos { 1.0 } else { 0.0 });
            let s = e.step(t - ARM_S, servos, ias);
            if s.fire {
                p.press();
            }
            if s.confirmed_now {
                confirmed_at = Some((t, p.vsi, p.alt_ft));
                break;
            }
            let trend = prev.map_or(0.0, |(v, t0)| (p.vsi - v) / (t - t0));
            let dt = prev.map_or(0.6, |(_, t0)| t - t0);
            prev = Some((p.vsi, t));
            elev = if s.hold {
                held_ticks += 1;
                law.step(0.0, p.vsi, trend, dt)
            } else {
                0.0
            };
        }
        let (t, vsi, alt) = confirmed_at.expect("the autopilot is confirmed");
        assert!(t <= 3.5, "confirmed at +{t} s, the old arm at +10.9");
        assert!(held_ticks >= 1, "held from the first tick");
        assert!(vsi > -1000.0, "the AP inherits {vsi:.0} fpm, not -3,200");
        assert!(
            ALT0_FT - alt < 60.0,
            "lost {:.0} ft before the AP was in",
            ALT0_FT - alt
        );
    }

    /// The hold alone, with an autopilot that never comes in: the new
    /// flight's sink is arrested on the elevator and the approach handed
    /// over at the bound with the aircraft level, not at -3,450 fpm.
    #[test]
    fn with_no_autopilot_the_hold_keeps_the_new_flight_level_until_the_hand_over() {
        let mut p = NewFlight::new();
        p.run(ARM_S, 0.0);
        let mut e = ApEngage::default();
        e.arm(0.0);
        let mut law = LevelHold::with_gains(engage_hold_gains());
        let mut prev: Option<(f64, f64)> = None;
        let mut elev = 0.0;
        let mut worst_vsi: f64 = 0.0;
        let mut lowest = p.alt_ft;
        let mut t = 0.9;
        let mut gave = false;
        while t < 30.0 {
            p.run(t, elev);
            lowest = lowest.min(p.alt_ft);
            if t > 4.0 {
                worst_vsi = worst_vsi.min(p.vsi);
            }
            let ias = if t < 1.5 { 12.0 } else { 170.0 };
            let s = e.step(t - ARM_S, Some(0.0), ias);
            if s.give_up {
                gave = true;
                break;
            }
            let trend = prev.map_or(0.0, |(v, t0)| (p.vsi - v) / (t - t0));
            let dt = prev.map_or(0.6, |(_, t0)| t - t0);
            prev = Some((p.vsi, t));
            assert!(s.hold);
            elev = law.step(0.0, p.vsi, trend, dt);
            assert!(elev <= engage_hold_gains().max + 1e-9);
            t += 0.6;
        }
        assert!(gave, "handed over");
        // The flight starts at -900 and the elevator arrives 1.35 s late:
        // the peak is that start, never the -3,450 nobody arrested.
        assert!(worst_vsi > -1200.0, "the sink never builds: {worst_vsi:.0}");
        assert!(p.vsi.abs() < 400.0, "level at the hand-over: {:.0}", p.vsi);
        assert!(ALT0_FT - lowest < 150.0, "lost {:.0} ft", ALT0_FT - lowest);
    }
