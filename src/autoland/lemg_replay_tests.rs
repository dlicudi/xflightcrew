    use super::*;

    /// LEMG's 13/31 row from the installed apt.dat (Global Airports), 45.11 m
    /// wide.
    pub const LEMG_13_31: &str = "100 45.11 1 1 0.00 1 3 0  13 36.6845573 -4.5126253 0 75 3 1 0 2 31 36.6654027 -4.4858140 0 75 3 1 0 3";
    pub const LEMG_WIDTH_M: f64 = 45.11;
    /// ...and its 12/30 row, the same width.
    pub const LEMG_12_30: &str = "100 45.11 1 1 0.00 1 3 0  12 36.6910753 -4.5078270 0 190 3 1 1 1 30 36.6795648 -4.4805125 0 489 3 0 0 1";

    /// One end of LEMG 13/31 or 12/30, parsed from the row the app reads.
    pub fn lemg(id: &str) -> Runway {
        let row = if id == "12" || id == "30" { LEMG_12_30 } else { LEMG_13_31 };
        let (e1, e2, length_m, surface) = parse_runway_100(row).expect("the LEMG row parses");
        let e = if e1.id == id { e1 } else { e2 };
        Runway {
            icao: "LEMG".into(),
            id: e.id,
            thr_lat: e.lat,
            thr_lon: e.lon,
            course_deg: e.course,
            length_m,
            surface,
            dist_nm: 0.0,
            elev_ft: 48.0,
        }
    }

    /// The point `along_m` short of the threshold (positive on the approach
    /// side) and `xtk_m` right of the inbound course.
    pub fn at(rwy: &Runway, along_m: f64, xtk_m: f64) -> (f64, f64) {
        let recip = (rwy.course_deg + 180.0).rem_euclid(360.0);
        let (lat, lon) = project_latlon(rwy.thr_lat, rwy.thr_lon, recip, along_m / M_PER_NM);
        let side = if xtk_m >= 0.0 {
            rwy.course_deg + 90.0
        } else {
            rwy.course_deg - 90.0
        };
        project_latlon(lat, lon, side.rem_euclid(360.0), xtk_m.abs() / M_PER_NM)
    }

    /// A 737 in heading mode, fitted to the recorded LEMG ticks: the
    /// autopilot banks 2 degrees per degree of heading error up to 25 (the
    /// base and final turns held 23 to 27), rolls toward it through a 1 s lag
    /// at no more than 7 degrees a second (0 to 16 degrees in 3.6 s on the
    /// short final of sortie 1790591256), turns coordinated, and flies the
    /// bug of the tick before (the loop's transport delay). The wind blows
    /// from 106 degrees at 6 m/s: a 2 degree crab on 131 and 3.7 on 158, as
    /// recorded.
    pub struct Jet {
        pub lat: f64,
        pub lon: f64,
        pub psi: f64,
        pub phi: f64,
        pub tas_mps: f64,
        pub wind_to_deg: f64,
        pub wind_mps: f64,
        /// How its autopilot answers the bug.
        pub fit: HeadingFit,
        /// A heading window that takes whole steps (the Zibo's MCP: 1
        /// degree, magnetic), the bug rounded to it; `None` flies the bug
        /// as written.
        pub window_step: Option<f64>,
        /// True less magnetic, degrees, for the window's rounding.
        pub var_deg: f64,
        /// The nose's yaw about the flight path, degrees and seconds
        /// (amplitude, period): what the heading mode and the crab filter
        /// read on top of the heading flown. `None`: none.
        pub yaw_wobble: Option<(f64, f64)>,
        /// The heading mode settling off its window, degrees, seconds and
        /// radians (amplitude, period, phase): `None`, on it.
        pub ap_offset: Option<(f64, f64, f64)>,
        t: f64,
        queued: std::collections::VecDeque<f64>,
    }

    /// A heading mode, fitted: bank `gain` degrees per degree of heading
    /// error up to `bank_max_deg`, rolled toward through a `lag_s` lag at no
    /// more than `roll_dps`, flying the bug `delay_ticks` ticks late.
    #[derive(Debug, Clone, Copy)]
    pub struct HeadingFit {
        pub gain: f64,
        pub bank_max_deg: f64,
        pub roll_dps: f64,
        pub lag_s: f64,
        pub delay_ticks: usize,
    }

    /// The 737 above.
    pub const B737_HEADING: HeadingFit =
        HeadingFit { gain: 2.0, bank_max_deg: 25.0, roll_dps: 7.0, lag_s: 1.0, delay_ticks: 1 };
    /// The A330's, fitted to the circuit ticks from the turn onto base to
    /// the handoff of the LEMG 30 circuits of 2026-09-29 (sortie
    /// 1790710769-full_auto 1.0 degrees rms, 1790698132 1.4; the 737's fit
    /// flies them 4.0 and 5.1 wrong): the same 2 per degree to 25, but
    /// rolled at 3 degrees a second through 2 s, two ticks late. The state
    /// records agree: 8 degrees of bank 4 s after the bug moved 59, 22 at 8.
    pub const A330_HEADING: HeadingFit =
        HeadingFit { gain: 2.0, bank_max_deg: 25.0, roll_dps: 3.0, lag_s: 2.0, delay_ticks: 2 };
    /// The Zibo 737's MCP heading select, fitted to the LEMG 30 ticks of
    /// 2026-10-07 (`bench/approach/lemg30_zibo_2026-10-07.jsonl`, the
    /// recorded window replayed in 20 s pieces from each piece's recorded
    /// heading and bank): 1.5 degrees of bank per degree to 25, rolled at 3
    /// degrees a second through a 4 s lag, the window answered 2 s after it
    /// was read back (three ticks of the 0.6 s final). The finals fly it 0.8
    /// degrees rms in heading and in bank, the pattern's turns 1.1 and 0.7;
    /// the stock 737's fit flies the same finals 0.9 and 1.1, the turns 2.8
    /// and 2.9 (`the_zibo_heading_fit_flies_the_recorded_windows`). Its
    /// window takes whole degrees: [`Jet::window_step`].
    pub const ZIBO_HEADING: HeadingFit =
        HeadingFit { gain: 1.5, bank_max_deg: 25.0, roll_dps: 3.0, lag_s: 4.0, delay_ticks: 3 };

    impl HeadingFit {
        /// The bank's rate toward what the heading error asks, deg/s.
        pub fn roll_rate(&self, heading_err_deg: f64, phi: f64) -> f64 {
            let want = (self.gain * heading_err_deg).clamp(-self.bank_max_deg, self.bank_max_deg);
            ((want - phi) / self.lag_s).clamp(-self.roll_dps, self.roll_dps)
        }
    }

    impl Jet {
        pub fn new(lat: f64, lon: f64, psi: f64, tas_kts: f64) -> Self {
            Self {
                lat,
                lon,
                psi,
                phi: 0.0,
                tas_mps: tas_kts * 0.514_444,
                wind_to_deg: 286.0,
                wind_mps: 6.0,
                fit: B737_HEADING,
                window_step: None,
                var_deg: 0.0,
                yaw_wobble: None,
                ap_offset: None,
                t: 0.0,
                queued: std::collections::VecDeque::new(),
            }
        }

        /// The heading read (true): the heading flown plus the yaw wobble.
        pub fn nose(&self) -> f64 {
            let w = self.yaw_wobble.map_or(0.0, |(a, p)| {
                a * (2.0 * std::f64::consts::PI * self.t / p).sin()
            });
            (self.psi + w).rem_euclid(360.0)
        }

        /// The bug (true) as the window takes it.
        pub fn windowed(&self, bug: f64) -> f64 {
            match self.window_step.filter(|s| *s > 0.0) {
                Some(s) => {
                    (((bug - self.var_deg) / s).round() * s + self.var_deg).rem_euclid(360.0)
                }
                None => bug,
            }
        }

        fn offset(&self) -> f64 {
            self.ap_offset.map_or(0.0, |(a, p, ph)| {
                a * (2.0 * std::f64::consts::PI * self.t / p + ph).sin()
            })
        }

        fn ground(&self) -> (f64, f64) {
            let (s, c) = self.psi.to_radians().sin_cos();
            let (ws, wc) = self.wind_to_deg.to_radians().sin_cos();
            (
                self.tas_mps * s + self.wind_mps * ws,
                self.tas_mps * c + self.wind_mps * wc,
            )
        }

        /// The ground track, degrees true.
        pub fn track(&self) -> f64 {
            let (e, n) = self.ground();
            e.atan2(n).to_degrees().rem_euclid(360.0)
        }

        pub fn gs_kts(&self) -> f64 {
            let (e, n) = self.ground();
            e.hypot(n) / 0.514_444
        }

        /// One loop tick of `dt` seconds with this tick's `bug` (true).
        pub fn step(&mut self, bug: f64, dt: f64) {
            let f = self.fit;
            self.queued.push_back(self.windowed(bug));
            let applied = if self.queued.len() > f.delay_ticks {
                self.queued.pop_front().unwrap_or(bug)
            } else {
                *self.queued.front().unwrap_or(&bug)
            };
            let n = 20;
            let h = dt / n as f64;
            for _ in 0..n {
                let err = crate::angles::signed_diff_deg(applied, self.nose()) + self.offset();
                self.phi += f.roll_rate(err, self.phi) * h;
                self.t += h;
                let rate = (9.81 * self.phi.to_radians().tan() / self.tas_mps).to_degrees();
                self.psi = (self.psi + rate * h).rem_euclid(360.0);
                let (e, nn) = self.ground();
                let (lat, lon) = project_latlon(
                    self.lat,
                    self.lon,
                    e.atan2(nn).to_degrees().rem_euclid(360.0),
                    e.hypot(nn) * h / M_PER_NM,
                );
                self.lat = lat;
                self.lon = lon;
            }
        }
    }

    /// The autoland loop's lateral answer, composed as the loop composes it:
    /// guidance on the runway's width, the crab filter, the final's one law
    /// (`FinalLateral`) on the response time measured over the final's first
    /// 90 s (`ResponseTime`). The vertical is flown exactly on the path.
    pub struct FinalLoop {
        pub crab: CrabFilter,
        pub lateral: FinalLateral,
        pub response: ResponseTime,
        pub width_m: Option<f64>,
        /// The knob as the loop sets it (`bug_knob`), when held; the knob
        /// the aircraft flies, how many times it moved (0.2 degrees, the
        /// least `drive_heading_bug_step` writes) and the loop's clock.
        pub knob: Option<crate::heading_bug::BugHold>,
        pub knob_at: Option<f64>,
        /// A deadband over the loop's own (`bug_knob`), for comparisons.
        pub knob_db: Option<crate::heading_bug::BugDeadband>,
        pub moves: u32,
        pub t: f64,
        final_t0: Option<f64>,
    }

    impl FinalLoop {
        pub fn new(width_m: Option<f64>) -> Self {
            Self {
                crab: CrabFilter::new(),
                lateral: FinalLateral::default(),
                response: ResponseTime::default(),
                width_m,
                knob: None,
                knob_at: None,
                knob_db: None,
                moves: 0,
                t: 0.0,
                final_t0: None,
            }
        }

        /// ...with the knob set as today's loop sets it.
        pub fn held(mut self) -> Self {
            self.knob = Some(crate::heading_bug::BugHold::default());
            self
        }

        /// One tick: the bug (true) and the guidance behind it.
        pub fn step(&mut self, jet: &Jet, rwy: &Runway, dt: f64) -> (f64, Guidance) {
            let (psi, trk, gs) = (jet.nose(), jet.track(), jet.gs_kts());
            let crab = self.crab.update(CrabFilter::raw(Some(psi), Some(trk)), dt);
            let dist = haversine_nm(jet.lat, jet.lon, rwy.thr_lat, rwy.thr_lon);
            let alt = rwy.elev_ft + (dist + TOUCHDOWN_AIM_NM) * GLIDEPATH_FT_PER_NM;
            let g = guidance_on(
                jet.lat,
                jet.lon,
                alt,
                alt - rwy.elev_ft,
                gs,
                145.0,
                rwy,
                145.0,
                25.0,
                0.0,
                self.width_m,
            );
            let in_final = g.phase == ApproachPhase::Final && !g.arresting;
            let bug = match g.phase {
                ApproachPhase::Final if in_final => {
                    let xtk_m = cross_track_nm(jet.lat, jet.lon, rwy) * M_PER_NM;
                    let side = side_velocity_m_s(Some(trk), rwy.course_deg, gs).unwrap_or(0.0);
                    self.final_t0.get_or_insert(self.t);
                    self.lateral
                        .bug_true(xtk_m, side, gs, rwy.course_deg, crab, self.response.t_r(), dt)
                }
                ApproachPhase::Intercept => {
                    self.lateral.reset();
                    g.target_heading_deg
                }
                _ => g.target_heading_deg + crab,
            };
            let mut bug = bug.rem_euclid(360.0);
            if let Some(k) = self.knob.as_mut() {
                match bug_knob(g.phase, g.arresting, None, None) {
                    Some((key, db)) => {
                        let wrote = k.decide(key, bug, self.t, self.knob_db.unwrap_or(db));
                        // A window in whole steps reads back the write
                        // rounded, and the knob remembers what it reads
                        // (`BugHold::took`), as the loop's does.
                        if let (Some(w), Some(_)) = (wrote, jet.window_step) {
                            k.took(Some(jet.windowed(w)));
                        }
                        bug = k.set().unwrap_or(bug);
                    }
                    None => k.reset(),
                }
            }
            let evidence = in_final
                && self.final_t0.is_some_and(|t0| self.t - t0 < TR_FINAL_WINDOW_S)
                && alt - rwy.elev_ft > TR_MIN_RADIO_FT;
            self.response.feed(Some(jet.windowed(bug)), Some(psi), Some(trk), dt, evidence);
            self.t += dt;
            if self
                .knob_at
                .is_none_or(|a| crate::angles::abs_diff_deg(bug, a) >= 0.2)
            {
                self.moves += 1;
                self.knob_at = Some(bug);
            }
            (bug, g)
        }
    }

    /// What a flown final came to: the offset at the roundout gate (the last
    /// tick above 80 ft on the path), the worst offset below 500 ft, the
    /// worst bank on the last 0.3 nm, and the first balk (why, radio, xtk).
    #[derive(Debug, Default)]
    pub struct Flown {
        pub gate_xtk_m: f64,
        pub worst_low_xtk_m: f64,
        pub worst_short_bank_deg: f64,
        pub balk: Option<(&'static str, f64, f64)>,
    }

    /// Fly `jet` down the final with `lp` to the roundout gate (80 ft on the
    /// path) or the first balk.
    pub fn fly_final(jet: &mut Jet, lp: &mut FinalLoop, rwy: &Runway, dt: f64) -> Flown {
        let mut out = Flown::default();
        for _ in 0..3000 {
            let (bug, g) = lp.step(jet, rwy, dt);
            let xtk_m = cross_track_nm(jet.lat, jet.lon, rwy) * M_PER_NM;
            let radio = g.glidepath_alt_ft - rwy.elev_ft;
            if let Some(why) = g.balk {
                out.balk = Some((why, radio, xtk_m));
                return out;
            }
            if radio < 80.0 {
                return out;
            }
            out.gate_xtk_m = xtk_m;
            if radio < 500.0 {
                out.worst_low_xtk_m = out.worst_low_xtk_m.max(xtk_m.abs());
            }
            if g.dist_nm <= 0.3 {
                out.worst_short_bank_deg = out.worst_short_bank_deg.max(jet.phi.abs());
            }
            jet.step(bug, dt);
        }
        out
    }
