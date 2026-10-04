    use super::*;
    use crate::xplane::FakeXPlane;
    use std::sync::Mutex;
    use tokio::time::Instant;

    const AT_MODE: &str = crate::aircraft::TakeoffConfigProfile::AT_MODE;
    const CMD_AT_OFF: &str = crate::aircraft::TakeoffConfigProfile::CMD_AT_OFF;
    const GS: &str = "sim/flightmodel/position/groundspeed";
    const PARK: &str = "sim/cockpit2/controls/parking_brake_ratio";
    const SET: &str = "sim/flight_controls/park_brake_set";

    /// Takeoff thrust's acceleration at 60 to 70 kt (the roll records:
    /// 60.4 to 66.7 kt in 1.2 s) and its decay once the levers close (a
    /// CFM56 spooling down from TOGA, fitted so one second of regular
    /// brakes peaks where the sortie did, 73 kt).
    const THRUST_KTS_S: f64 = 5.0;
    const SPOOL_TAU_S: f64 = 1.5;
    /// Coasting at idle, nobody braking: 73.3 kt at +76.8 to 52.7 at +116.
    const COAST_KTS_S: f64 = 0.5;
    /// Maximum braking, from the EGKK rejects (1790524628, 1790525414,
    /// 1790525500): about 0.3 s before the brakes bite, a second to build,
    /// then 8 to 15 kt/s tick to tick, 10.5 on the average.
    const MAX_BRAKE_KTS_S: f64 = 10.5;
    const REGULAR_BRAKE_KTS_S: f64 = 3.5;
    const BRAKE_DEAD_S: f64 = 0.3;
    const BRAKE_BUILD_S: f64 = 1.0;

    #[derive(Default)]
    struct State {
        last: Option<Instant>,
        gs_kts: f64,
        idle_at: Option<Instant>,
        /// The held brake command: effort, since (continuous), until.
        brake: Option<(f64, Instant, Instant)>,
        max_kts: f64,
        gs_at_park: Option<f64>,
        thrust_back: bool,
    }

    /// The 737 as the reject found it: rolling at `kts` on takeoff thrust,
    /// the autothrottle `at_mode`.
    struct RejectPlant {
        sim: FakeXPlane,
        st: Mutex<State>,
    }

    impl RejectPlant {
        fn rolling_at_kts(kts: f64, at_mode: f64) -> Self {
            let sim = FakeXPlane::with_reads(&[
                (crate::datarefs::ON_GROUND_ANY, 1.0),
                (crate::autotaxi::TUG_ATTACHED, 0.0),
                (PARK, 0.0),
                (AT_MODE, at_mode),
                (sim::datarefs::LATITUDE, 0.0),
                (sim::datarefs::LONGITUDE, 0.0),
                (sim::datarefs::ELEVATION_M, 15.0),
                ("sim/flightmodel/position/psi", 0.0),
                ("sim/flightmodel/position/vh_ind_fpm", 0.0),
                (sim::datarefs::AGL_METERS, 0.0),
                (YAW_RATE, 0.0),
            ]);
            sim.step_on_command(SET, PARK, 1.0);
            sim.set_on_command(CMD_AT_OFF, AT_MODE, 0.0);
            let st = State { gs_kts: kts, max_kts: kts, ..Default::default() };
            RejectPlant { sim, st: Mutex::new(st) }
        }

        /// Integrate to now in 10 ms steps.
        fn advance(&self) {
            let now = Instant::now();
            let at_engaged = self.sim.read_value(AT_MODE).is_some_and(|m| m > 0.5);
            let parked = self.sim.read_value(PARK).is_some_and(|p| p > 0.5);
            let mut s = self.st.lock().unwrap();
            let Some(mut t) = s.last else {
                s.last = Some(now);
                return;
            };
            let step = std::time::Duration::from_millis(10);
            while t + step <= now {
                t += step;
                let thrust = match s.idle_at {
                    _ if at_engaged => THRUST_KTS_S,
                    None => THRUST_KTS_S,
                    Some(i) => {
                        THRUST_KTS_S * (-(t - i).as_secs_f64() / SPOOL_TAU_S).exp()
                    }
                };
                let brake = match s.brake {
                    _ if parked => MAX_BRAKE_KTS_S,
                    Some((effort, since, until)) if t <= until => {
                        let on = (t - since).as_secs_f64() - BRAKE_DEAD_S;
                        effort * (on / BRAKE_BUILD_S).clamp(0.0, 1.0)
                    }
                    _ => 0.0,
                };
                let coast = if s.gs_kts > 0.0 { COAST_KTS_S } else { 0.0 };
                s.gs_kts = (s.gs_kts + (thrust - brake - coast) * 0.01).max(0.0);
                s.max_kts = s.max_kts.max(s.gs_kts);
            }
            s.last = Some(t);
        }

        fn gs_kts(&self) -> f64 {
            self.advance();
            self.st.lock().unwrap().gs_kts
        }
    }

    #[async_trait::async_trait]
    impl crate::xplane::XPlane for RejectPlant {
        async fn get_float(&self, name: &str) -> anyhow::Result<f64> {
            if name == GS {
                return Ok(self.gs_kts() / MS_TO_KTS);
            }
            self.advance();
            self.sim.get_float(name).await
        }
        async fn set_float(&self, name: &str, value: f64) -> anyhow::Result<()> {
            self.advance();
            if name.contains("throttle") {
                let mut s = self.st.lock().unwrap();
                if value > 0.05 {
                    s.thrust_back = s.idle_at.is_some();
                    s.idle_at = None;
                } else if s.idle_at.is_none() {
                    s.idle_at = Some(Instant::now());
                }
            }
            self.sim.set_float(name, value).await
        }
        async fn set_int(&self, name: &str, value: i64) -> anyhow::Result<()> {
            self.sim.set_int(name, value).await
        }
        async fn run_command(&self, name: &str) -> anyhow::Result<()> {
            self.advance();
            if name == SET {
                let gs = self.st.lock().unwrap().gs_kts;
                self.st.lock().unwrap().gs_at_park.get_or_insert(gs);
            }
            self.sim.run_command(name).await
        }
        async fn run_command_for(&self, name: &str, secs: f64) -> anyhow::Result<()> {
            self.advance();
            let effort = match name {
                BRAKES_MAX => MAX_BRAKE_KTS_S,
                crate::tools::BRAKES_REGULAR => REGULAR_BRAKE_KTS_S,
                _ => 0.0,
            };
            if effort > 0.0 {
                let now = Instant::now();
                let until = now + std::time::Duration::from_secs_f64(secs);
                let mut s = self.st.lock().unwrap();
                let since = match s.brake {
                    Some((_, since, prev_until)) if now <= prev_until => since,
                    _ => now,
                };
                s.brake = Some((effort, since, until));
            }
            self.sim.run_command_for(name, secs).await
        }
        async fn get_float_array(&self, name: &str) -> anyhow::Result<Vec<f64>> {
            self.sim.get_float_array(name).await
        }
        async fn set_int_array_elem(&self, name: &str, i: usize, v: i64) -> anyhow::Result<()> {
            self.sim.set_int_array_elem(name, i, v).await
        }
        async fn set_float_array_elem(&self, name: &str, i: usize, v: f64) -> anyhow::Result<()> {
            self.sim.set_float_array_elem(name, i, v).await
        }
    }

    fn rwy_north() -> Runway {
        Runway {
            icao: "LEMG".into(),
            id: "13".into(),
            thr_lat: -0.001,
            thr_lon: 0.0,
            course_deg: 0.0,
            length_m: 3200.0,
            surface: 1,
            dist_nm: 0.0,
            elev_ft: 52.0,
        }
    }

    /// The plant is the sortie: the old reject (idle and one second of the
    /// regular brakes, then nobody) pushed the 737 from 69 to about 73 kt
    /// on the spool-down and left it above 50 kt forty seconds on.
    #[tokio::test(start_paused = true)]
    async fn the_plant_reproduces_the_lemg_reject() {
        let jet = RejectPlant::rolling_at_kts(69.2, 0.0);
        jet.set_float(&throttle_dref(), 0.0).await.unwrap();
        jet.run_command_for(crate::tools::BRAKES_REGULAR, 1.0).await.unwrap();
        tokio::time::sleep(std::time::Duration::from_secs(6)).await;
        jet.gs_kts();
        let peak = jet.st.lock().unwrap().max_kts;
        assert!((72.0..75.0).contains(&peak), "peak {peak:.1} kt, the sortie's 73.3");
        tokio::time::sleep(std::time::Duration::from_secs(34)).await;
        let gs = jet.gs_kts();
        assert!(gs > 50.0, "{gs:.1} kt forty seconds on, the sortie's 52.7");
    }

    /// The fix: the reject owns the levers (idle every tick, an ENGAGED
    /// autothrottle taken out and read back), brakes at maximum inside the
    /// held command's expiry to a FULL stop on the centreline, and sets the
    /// parking brake only once stopped (#112's caution never sees its own
    /// brake rolling), read back, before "your aircraft".
    #[tokio::test(start_paused = true)]
    async fn a_rejected_takeoff_is_flown_to_a_stop_and_parked() {
        let jet = RejectPlant::rolling_at_kts(69.2, 1.0);
        let at_off = reject_hands(&jet).await;
        assert_eq!(at_off, Some(true), "the engaged autothrottle is out, read back");
        let mut law = RollSteer::for_airframe();
        let stop = stop_on_centreline(&jet, &rwy_north(), &mut law).await;
        let held = crate::full_auto::hand_back_hold(&jet, Some(1.0), super::REJECT_HOLD).await;
        let s = jet.st.lock().unwrap();
        assert!(stop.stopped, "{stop:?}");
        assert!(!s.thrust_back, "the levers never came back up");
        assert!(
            jet.sim.writes_to(&throttle_dref()).iter().all(|v| *v == 0.0),
            "idle held: {:?}",
            jet.sim.writes_to(&throttle_dref())
        );
        assert!(s.max_kts < 71.0, "peak {:.1} kt, never the sortie's 73.3", s.max_kts);
        assert!(stop.secs < 12.0, "stopped in {:.1} s", stop.secs);
        assert!(stop.brake_pulses >= 8, "maximum braking renewed: {stop:?}");
        assert_eq!(held, Some(true), "the parking brake reads set");
        let at_park = s.gs_at_park.expect("the parking brake was set");
        assert!(at_park < 1.0, "parked at {at_park:.1} kt, not rolling");
        assert!(s.gs_kts < 0.1, "held at {:.2} kt", s.gs_kts);
        assert_eq!(
            reject_end_line(&stop, held),
            "Stopped. Parking brake set. Your aircraft."
        );
    }

    /// Nothing claimed that did not read back.
    #[test]
    fn the_reject_says_only_what_holds_the_aircraft() {
        let stopped = RejectStop { stopped: true, ..Default::default() };
        let rolling = RejectStop::default();
        assert!(reject_end_line(&stopped, Some(false)).contains("didn't set"));
        assert!(!reject_end_line(&rolling, Some(false)).starts_with("Stopped"));
        assert!(reject_end_line(&rolling, None).contains("couldn't stop"));
    }

    /// An engaged autothrottle moves the levers; armed and hard off do not.
    #[test]
    fn only_an_engaged_autothrottle_is_taken_out() {
        use crate::takeoff_config::autothrottle_engaged;
        assert!(!autothrottle_engaged(-1.0) && !autothrottle_engaged(0.0));
        assert!(autothrottle_engaged(1.0) && autothrottle_engaged(2.0));
        assert!(!autothrottle_engaged(f64::NAN));
    }
