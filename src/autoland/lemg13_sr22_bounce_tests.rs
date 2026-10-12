    use super::*;

    const FLOWN: [&str; 3] = [
        include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/landings/sr22_lemg13_flare_1790701546.jsonl")),
        include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/landings/sr22_lemg13_flare_1790702352.jsonl")),
        include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/landings/sr22_lemg13_flare_1790702623.jsonl")),
    ];
    /// The recorded first hops over the runway (ft): the peak between the
    /// first contact and the second, and between the second and the third.
    const FLOWN_HOPS: [(f64, f64); 3] = [(4.9, 5.6), (4.9, 6.0), (4.5, 4.9)];

    /// The loop's tick on the iMac.
    const TICK_S: f64 = 0.45;
    /// The SR22's landing-configuration stall (POH `vs0_kts`): the plant
    /// sinks past it.
    const VSO_KTS: f64 = 60.0;
    /// The SR22 at rest reads this on its radio altimeter, ft.
    const RADIO_AT_REST_FT: f64 = 0.15;
    /// On the wheels the rollout's brakes and the rolling drag, kt/s (the
    /// recorded rollouts: 3 to 5).
    const ROLL_DECEL_KTS_S: f64 = 3.0;
    /// The gear's travel, ft.
    const GEAR_TRAVEL_FT: f64 = 0.4;

    /// One recorded flare: the arrest's ticks from the servos taken (s,
    /// radio ft, vvi fpm, IAS or the rollout's groundspeed kt), the elevator
    /// as written (s, ratio) and the lever as commanded (s, ratio).
    struct Flown {
        ticks: Vec<(f64, f64, Option<f64>, f64, bool)>,
        writes: Vec<(f64, f64)>,
        levers: Vec<(f64, f64)>,
    }

    fn flown(src: &str) -> Flown {
        let rows: Vec<serde_json::Value> = src
            .lines()
            .filter(|l| !l.trim().is_empty())
            .map(|l| serde_json::from_str(l).expect("a record"))
            .collect();
        let t0 = rows
            .iter()
            .find(|r| r["event"] == "arrest_hands")
            .and_then(|r| r["t"].as_f64())
            .expect("the arrest took the servos");
        let mut out = Flown {
            ticks: vec![],
            writes: vec![],
            levers: vec![],
        };
        for r in &rows {
            let t = r["t"].as_f64().unwrap() - t0;
            match r["event"].as_str() {
                None => {
                    out.ticks.push((
                        t,
                        r["radio"].as_f64().unwrap(),
                        r["vvi"].as_f64(),
                        r["ias"].as_f64().unwrap(),
                        false,
                    ));
                    if let Some(e) = r["flare_elev"].as_f64() {
                        out.writes.push((t, e));
                    }
                }
                Some("power") => {
                    if let Some(l) = r["throttle_command"].as_f64() {
                        out.levers.push((t, l));
                    }
                }
                Some("rollout") => {
                    out.ticks.push((
                        t,
                        r["radio"].as_f64().unwrap(),
                        None,
                        r["gs"].as_f64().unwrap(),
                        true,
                    ));
                    out.levers.push((t, 0.0));
                }
                _ => {}
            }
        }
        out
    }

    /// The plant's numbers.
    #[derive(Debug, Clone, Copy)]
    struct Fit {
        /// Standing sink per unit of elevator (fpm), its two equal lags (s)
        /// and the dead time (s).
        k: f64,
        lag: f64,
        dead: f64,
        /// The tail's ground effect: sink at the runway (fpm), growing from
        /// `tail_ft`.
        tail_fpm: f64,
        tail_ft: f64,
        /// fpm of sink per knot lost from the gate.
        per_kt: f64,
        /// The vertical speed's own lag toward the standing sink (s).
        path_s: f64,
        /// The wing's ground cushion: climb at the runway (fpm) fading by
        /// this height (ft).
        wing_fpm: f64,
        wing_ft: f64,
        /// The gear: spring (ft/s^2 per ft) and damper (per s).
        spring: f64,
        damper: f64,
        /// The mains' contact: climb tendency per fpm of sink met, and its
        /// decay (s).
        kick: f64,
        kick_s: f64,
        /// ...met up to this sink (fpm), and scaled by the dynamic pressure
        /// against 80 kt: the nose the gear lifts is worth what the wing
        /// makes of it.
        kick_cap: f64,
        /// The vertical-speed gauge the laws read lags the flight path by
        /// this (s); 0 reads the path itself.
        gauge_s: f64,
        /// The column's neutral as the plant feels it, added to what is
        /// written (the trim the servos left).
        off: f64,
        /// Sink per unit of lever (fpm) and the lever's spool lag (s).
        thr_fpm: f64,
        thr_lag: f64,
        /// The speed model, kt/s: -drag0 - drag_w x climb (per 1,000 fpm)
        /// + thrust x lever; and the stall (kt) the plant sinks past.
        drag0: f64,
        drag_w: f64,
        thrust: f64,
        vso: f64,
    }

    /// The best fit (the three flares' sink before contact, 39 to 63 fpm
    /// rms, and their first six hops): a soft gear and a strong nose-up
    /// kick. Replayed, the first hops are 5.8, 5.0 and 6.0 ft against 4.9,
    /// 4.9 and 4.5; closed loop, the law as flown bounces 5.9 ft on it.
    const FIT: Fit = Fit {
        k: 6170.0,
        lag: 1.42,
        dead: 1.19,
        tail_fpm: 380.0,
        tail_ft: 40.0,
        per_kt: 122.0,
        path_s: 0.34,
        wing_fpm: 85.0,
        wing_ft: 4.9,
        spring: 9.3,
        damper: 2.7,
        kick: 4.95,
        kick_s: 1.38,
        kick_cap: 369.0,
        gauge_s: 0.0,
        off: 0.0,
        thr_fpm: 316.0,
        thr_lag: 2.7,
        drag0: 2.70,
        drag_w: 5.1,
        thrust: 2.74,
        vso: VSO_KTS,
    };
    /// The plants that replay the records (`the_plant_replays_the_80_kt_bounces`):
    /// the best fit, its dead time 0.2 s shorter (3.8, 6.5, 3.6 ft) and its
    /// kick a fifth weaker (3.9, 3.1, 4.1). A law that lands on all of them
    /// does not depend on one number being right. The elevator 15 % weaker
    /// or stronger, the tail's ground effect 40 % either way, the speed's
    /// sink 30 % either way, or another fit (a heavy vertical and a stiff
    /// gear) do not replay the three flares (first hops of 1.1 to 36 ft)
    /// and are not plants of this aircraft.
    fn plants() -> Vec<Fit> {
        vec![
            FIT,
            Fit {
                dead: FIT.dead - 0.2,
                ..FIT
            },
            Fit {
                kick: FIT.kick * 0.8,
                ..FIT
            },
        ]
    }

    pub(super) struct Plant {
        fit: Fit,
        t: f64,
        h: f64,
        /// Vertical speed, fpm.
        w: f64,
        /// The gauge's vertical speed, fpm ([`Fit::gauge_s`]).
        g: f64,
        ias: f64,
        ias0: f64,
        base: f64,
        b1: f64,
        b2: f64,
        thr: f64,
        thr0: f64,
        thr_cmd: f64,
        kick: f64,
        stall: f64,
        /// The elevator as written, with the plugin's 1 s expiry.
        written: Vec<(f64, f64)>,
        /// Recorded IAS to follow (a replay) instead of the speed model.
        ias_track: Option<Vec<(f64, f64)>>,
        on_wheels: bool,
        /// Every contact: (time, sink met fpm, IAS).
        contacts: Vec<(f64, f64, f64)>,
        /// Highest point after the first contact.
        bounce_ft: f64,
        /// The gauge at the first contact (the scorer's sink), fpm, and the
        /// ground covered from the gate to it, m.
        first_gauge: Option<f64>,
        along_m: f64,
    }

    impl Plant {
        /// At the arrest gate, sinking steadily with `pipe` on the column
        /// and the lever holding the speed.
        fn at(fit: Fit, h: f64, vvi: f64, ias: f64, pipe: f64) -> Self {
            let b = fit.k * (pipe + fit.off);
            // The lever the speed model holds this sink on.
            let thr0 = ((fit.drag0 + fit.drag_w * vvi / 1000.0) / fit.thrust).clamp(0.0, 1.0);
            let mut p = Plant {
                fit,
                t: 0.0,
                h,
                w: vvi,
                g: vvi,
                ias,
                ias0: ias,
                base: 0.0,
                b1: b,
                b2: b,
                thr: thr0,
                thr0,
                thr_cmd: thr0,
                kick: 0.0,
                stall: 0.0,
                written: vec![(-10.0, pipe)],
                ias_track: None,
                on_wheels: false,
                contacts: vec![],
                bounce_ft: 0.0,
                first_gauge: None,
                along_m: 0.0,
            };
            p.base = vvi - b - p.ground(h);
            p
        }

        fn ground(&self, h: f64) -> f64 {
            let h = h.max(0.0);
            -self.fit.tail_fpm * (1.0 - h / self.fit.tail_ft).max(0.0)
                + self.fit.wing_fpm * (1.0 - h / self.fit.wing_ft).max(0.0)
        }

        fn write(&mut self, e: f64) {
            self.written.push((self.t, e));
        }

        fn arriving(&self) -> f64 {
            let due = self.t - self.fit.dead;
            self.fit.off
                + match self.written.iter().take_while(|(t, _)| *t <= due + 1e-9).last() {
                    // The autopilot's column at the gate never expires.
                    Some((tw, e)) if due - tw <= 1.0 || *tw < -1.0 => *e,
                    _ => 0.0,
                }
        }

        fn standing(&self) -> f64 {
            self.base
                + self.b2
                + self.fit.thr_fpm * (self.thr - self.thr0)
                + self.fit.per_kt * (self.ias - self.ias0)
                + self.ground(self.h)
                + self.kick
                - self.stall
        }

        fn step(&mut self, dt: f64) {
            let f = self.fit;
            let e = self.arriving();
            self.b1 += (f.k * e - self.b1) * dt / f.lag;
            self.b2 += (self.b1 - self.b2) * dt / f.lag;
            self.thr += (self.thr_cmd - self.thr) * dt / f.thr_lag;
            self.kick -= self.kick * dt / f.kick_s;
            match &self.ias_track {
                Some(tr) => {
                    self.ias = tr
                        .iter()
                        .take_while(|(t, _)| *t <= self.t)
                        .last()
                        .map_or(self.ias0, |x| x.1)
                }
                None => {
                    self.ias += if self.on_wheels {
                        -ROLL_DECEL_KTS_S
                    } else {
                        -f.drag0 - f.drag_w * self.w / 1000.0 + f.thrust * self.thr
                    } * dt
                }
            }
            if self.ias < f.vso && !self.on_wheels {
                self.stall += 800.0 * dt;
            }
            let mut a = (self.standing() - self.w) / f.path_s;
            if self.h < 0.0 {
                let g = (f.spring * -self.h - f.damper * self.w / 60.0) * 60.0;
                if g > 0.0 {
                    a += g;
                }
            }
            self.w += a * dt;
            self.g = if f.gauge_s > 0.0 {
                self.g + (self.w - self.g) * dt / f.gauge_s
            } else {
                self.w
            };
            let was = self.h;
            self.h += self.w / 60.0 * dt;
            // The gear bottoms out: the soft spring the hops were fitted
            // with is only the first few inches of it.
            if self.h < -GEAR_TRAVEL_FT {
                self.h = -GEAR_TRAVEL_FT;
                self.w = self.w.max(0.0);
            }
            if was >= 0.0 && self.h < 0.0 {
                self.kick +=
                    f.kick * (-self.w).max(0.0).min(f.kick_cap) * (self.ias / 80.0).powi(2);
                self.contacts.push((self.t, self.w, self.ias));
                self.first_gauge.get_or_insert(self.g);
            }
            self.on_wheels = self.h <= 0.0;
            if !self.contacts.is_empty() {
                self.bounce_ft = self.bounce_ft.max(self.h);
            }
            if self.contacts.is_empty() {
                self.along_m += self.ias / 1.943_844 * dt;
            }
            self.t += dt;
        }

        fn radio(&self) -> f64 {
            self.h.max(0.0) + RADIO_AT_REST_FT
        }

        /// The vertical speed the laws read: the gauge.
        fn vvi(&self) -> f64 {
            self.g
        }
    }

    /// The hops of a height series: the peak between each contact and the
    /// next (a contact is the radio under 0.8 ft).
    fn hops(series: &[(f64, f64)]) -> Vec<f64> {
        let mut out = vec![];
        let mut i = 0;
        while i < series.len() {
            if series[i].1 < 0.8 {
                while i < series.len() && series[i].1 < 0.8 {
                    i += 1;
                }
                let mut peak = 0.0_f64;
                let start = i;
                while i < series.len() && series[i].1 >= 0.8 {
                    peak = peak.max(series[i].1);
                    i += 1;
                }
                if i > start && i < series.len() {
                    out.push(peak);
                }
            } else {
                i += 1;
            }
        }
        out
    }

    /// The recorded column and lever, open loop, from the recorded gate: the
    /// plant's radio at every recorded tick.
    fn replay(f: &Flown, fit: Fit) -> Vec<(f64, f64)> {
        let (_, h, v, ias, _) = f.ticks[0];
        let mut p = Plant::at(fit, h, v.unwrap(), ias, f.writes[0].1);
        p.ias_track = Some(f.ticks.iter().map(|x| (x.0, x.3)).collect());
        p.thr0 = f.levers.first().map_or(p.thr0, |l| l.1);
        p.thr = p.thr0;
        p.written = vec![(-10.0, f.writes[0].1)];
        let mut out = vec![];
        let (mut wi, mut li) = (0, 0);
        for &(t, ..) in &f.ticks {
            while p.t < t - 1e-9 {
                while wi < f.writes.len() && f.writes[wi].0 <= p.t + 1e-9 {
                    let e = f.writes[wi].1;
                    p.write(e);
                    wi += 1;
                }
                while li < f.levers.len() && f.levers[li].0 <= p.t + 1e-9 {
                    p.thr_cmd = f.levers[li].1;
                    li += 1;
                }
                p.step(0.01);
            }
            out.push((t, p.radio()));
        }
        out
    }

    /// The loop's laws over the plant, from a gate to the wheels and on to
    /// `until` s. `hold_off`: today's SR22 (its profile); else the flare as
    /// flown at 80 kt (no hold-off, the lever to a second from the wheels,
    /// the rollout letting the channel go).
    #[derive(Debug)]
    struct Landing {
        first_fpm: f64,
        #[allow(dead_code)] // the record's, read only through Debug
        contacts: usize,
        bounce_ft: f64,
        min_ias: f64,
        #[allow(dead_code)] // the record's, read only through Debug
        secs_to_wheels: f64,
    }

    fn land(
        p: &mut Plant,
        model: &mut FlareModel,
        power_cfg: crate::aircraft::FlarePowerProfile,
        approach_ias: f64,
        rough: bool,
        until: f64,
    ) -> Landing {
        let holds_off = model.holds_off();
        let mut power = FlarePower::default();
        let mut touch = TouchHold::default();
        let mut last_elev: Option<f64> = None;
        let (mut next, mut flaring, mut rolling, mut wheels_on) = (0.0, false, false, None::<f64>);
        let mut min_ias = p.ias;
        let approach_thr = p.thr0;
        while p.t < until {
            if p.t + 1e-9 >= next {
                next += TICK_S;
                let gs = p.ias;
                let radio = p.radio();
                let on_ground = p.on_wheels;
                wheels_on = wheels_on_time(wheels_on, on_ground, radio, TICK_S);
                if should_start_rollout(rolling, on_ground, radio, gs, p.ias, approach_ias, wheels_on) {
                    if !rolling && holds_off {
                        touch.note(true, last_elev);
                    }
                    rolling = true;
                }
                if rolling {
                    p.thr_cmd = 0.0;
                    if let Some(e) = touch.elev(gs, on_ground, TICK_S) {
                        p.write(e);
                    }
                } else {
                    let (flare_ft, band_ft) = arrest_gates_ft(25.0, gs);
                    flaring = radio <= flare_ft || (flaring && radio <= band_ft);
                    if holds_off && flaring {
                        touch.note(on_ground || radio < TOUCH_RADIO_FT, last_elev);
                    }
                    model.note_speed(p.ias, TICK_S);
                    let o = model.step(radio, gs, p.vvi(), TICK_S, flaring, rough);
                    let e = if touch.touched() {
                        touch.elev(gs, on_ground, TICK_S)
                    } else {
                        Some(o.elev)
                    };
                    if let Some(e) = e {
                        p.write(e);
                        last_elev = Some(e);
                    }
                    p.thr_cmd = if touch.touched() {
                        0.0
                    } else if !flaring {
                        power.reset();
                        approach_thr
                    } else if let Some(t) =
                        balloon_power(Some(p.vvi()), Some(approach_thr), radio > band_ft, true)
                    {
                        t
                    } else {
                        power
                            .step(power_cfg, p.thr_cmd, p.vvi(), TICK_S, radio, Some(p.ias), Some(approach_ias))
                            .unwrap_or(p.thr_cmd)
                    };
                }
            }
            p.step(0.01);
            if p.contacts.is_empty() {
                min_ias = min_ias.min(p.ias);
            }
        }
        let first = p.contacts.first().copied().unwrap_or((f64::NAN, f64::NAN, f64::NAN));
        Landing {
            first_fpm: -first.1,
            contacts: p.contacts.len(),
            bounce_ft: p.bounce_ft,
            min_ias,
            secs_to_wheels: first.0,
        }
    }

    /// The plant is the recording: the three flares' column and lever as
    /// written, open loop, give back their first hop within 2 ft on every
    /// plant (4.9, 4.9 and 4.5 recorded).
    #[test]
    fn the_plant_replays_the_80_kt_bounces() {
        for fit in plants() {
            for (src, want) in FLOWN.iter().zip(FLOWN_HOPS) {
                let f = flown(src);
                let got = hops(&replay(&f, fit));
                assert!(!got.is_empty(), "{fit:?}: no hop");
                assert!((got[0] - want.0).abs() < 2.0, "{fit:?}: {got:?} against {want:?}");
            }
        }
    }

    /// Held on: the pull it touched with, never more and never less, the
    /// lever closed; let go only on the wheels below the release speed.
    #[test]
    fn touch_hold_keeps_the_pull_it_touched_with() {
        let mut t = TouchHold::default();
        assert!(!t.note(false, Some(0.3)), "no contact");
        assert_eq!(t.elev(70.0, false, 0.45), None, "nothing held before a contact");
        assert!(t.note(true, Some(0.31)));
        assert!(!t.note(true, Some(0.5)), "the first contact's pull, once");
        assert_eq!(t.elev(66.0, true, 0.45), Some(0.31));
        assert_eq!(t.elev(64.0, false, 0.45), Some(0.31), "a hop keeps it");
        let e = t.elev(40.0, true, 1.0).unwrap();
        assert!((e - (0.31 - TOUCH_HOLD_EASE_PER_S)).abs() < 1e-9, "{e}");
        for _ in 0..20 {
            t.elev(30.0, true, 1.0);
        }
        assert_eq!(t.elev(30.0, true, 1.0), None, "let go at neutral");
        let mut push = TouchHold::default();
        push.note(true, Some(-0.04));
        assert_eq!(push.elev(70.0, true, 0.45), Some(0.0), "never a push");
    }

    /// A recorded flare's column and lever, open loop, from its gate on a
    /// plant: (t, radio, gauge) at every recorded tick before the wheels.
    /// `pipe` is the column the autopilot left at the gate and `bias` a
    /// standing sink the plant does not explain (the trim the autopilot
    /// left, the phugoid it was in), both fitted with the plant.
    fn replay_open(f: &Flown, fit: Fit, pipe: f64, bias: f64) -> Vec<(f64, f64, f64)> {
        let (_, h, v, ias, _) = f.ticks[0];
        let mut p = Plant::at(fit, h, v.unwrap(), ias, pipe);
        p.base += bias;
        p.ias_track = Some(f.ticks.iter().map(|x| (x.0, x.3)).collect());
        p.thr0 = f.levers.first().map_or(p.thr0, |l| l.1);
        p.thr = p.thr0;
        p.thr_cmd = p.thr0;
        let mut out = vec![];
        let (mut wi, mut li) = (0, 0);
        for &(t, _, _, _, rolling) in &f.ticks {
            if rolling {
                break;
            }
            while p.t < t - 1e-9 {
                while wi < f.writes.len() && f.writes[wi].0 <= p.t + 1e-9 {
                    let e = f.writes[wi].1;
                    p.write(e);
                    wi += 1;
                }
                while li < f.levers.len() && f.levers[li].0 <= p.t + 1e-9 {
                    p.thr_cmd = f.levers[li].1;
                    li += 1;
                }
                p.step(0.01);
            }
            out.push((t, p.radio(), p.vvi()));
        }
        out
    }

    /// How well a plant replays a recorded flare: the radio's rms (ft), the
    /// gauge's rms (fpm), and the gauge's rms over the last 2 s before the
    /// wheels (fpm), where the flare is won or lost.
    fn replay_rms(f: &Flown, fit: Fit, pipe: f64, bias: f64) -> (f64, f64, f64) {
        let rec: Vec<_> = f.ticks.iter().filter(|x| !x.4).collect();
        let got = replay_open(f, fit, pipe, bias);
        let n = got.len().min(rec.len());
        let t_end = rec[n - 1].0;
        let (mut dh, mut dv, mut dl, mut nl) = (0.0, 0.0, 0.0, 0.0_f64);
        for (g, r) in got.iter().zip(&rec) {
            let e = g.2 - r.2.unwrap();
            dh += (g.1 - r.1).powi(2);
            dv += e * e;
            if g.0 > t_end - 2.0 {
                dl += e * e;
                nl += 1.0;
            }
        }
        let n = n as f64;
        ((dh / n).sqrt(), (dv / n).sqrt(), (dl / nl.max(1.0)).sqrt())
    }

    /// A model flare from a gate on a plant, calm or rough: the landing, the
    /// gauge at the first contact (fpm, positive) and the ground covered
    /// from the gate to it (m).
    fn land_model(
        fit: Fit,
        gate: (f64, f64, f64, f64, f64),
        fe: crate::aircraft::FlareElevatorProfile,
        power: crate::aircraft::FlarePowerProfile,
        approach_ias: f64,
        rough: bool,
    ) -> (Landing, f64, f64) {
        let (h, v, ias, pipe, bias) = gate;
        let mut p = Plant::at(fit, h, v, ias, pipe);
        p.base += bias;
        let mut m = FlareModel::from_profile(Some(fe)).expect("a model");
        let l = land(&mut p, &mut m, power, approach_ias, rough, 45.0);
        (l, -p.first_gauge.unwrap_or(f64::NAN), p.along_m)
    }

    /// The Lancair Evolution's flares at LILB 03 (iMac 2026-09-30). The first
    /// (sortie 1790740292-full_auto,
    /// `bench/landings/evolution_lilb03_flare_1790740292.jsonl`) on the jets'
    /// default tracker (no `flare_elevator:`): the gauge the laws read
    /// (`vvi_fpm_pilot`) lags the flight path, at 9 ft the height had been
    /// level for a second while the gauge still read -386 to -82, the
    /// tracker's balloon lead let the whole 0.35 of pull go at once, and the
    /// aircraft dropped 9 ft onto the runway (392 fpm on the gauge). Then
    /// two on the model flare (#196, `evolution_lilb_model_flare_*.jsonl`):
    /// 1790781114-full_auto pushed from 52 ft at 361 fpm, the sink built to
    /// 576 and it landed at 570; 1790793076-full_auto, with the push gone
    /// (3974df3), came into the model at 339 fpm and 53 ft against a target
    /// of 500, held the column at neutral to 28 ft while the sink built to
    /// 454, pulled 0.14 to 0.18 from 24 ft, and the sink still grew to 595
    /// at 13 ft: 584 at the wheels.
    mod lilb03_evolution {
        use super::*;

        const SRC: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/landings/evolution_lilb03_flare_1790740292.jsonl"));
        const MODEL_SRC: [&str; 2] = [
            include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/landings/evolution_lilb_model_flare_1790793076.jsonl")),
            include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/landings/evolution_lilb_model_flare_1790781114.jsonl")),
        ];

        /// The first fit, to the tracker's flare alone (radio 0.66 ft rms,
        /// gauge 43 fpm rms): 6,450 fpm per unit through 0.29 s lags after
        /// 0.23 s, the path's own 1.5 s, the gauge 0.97 s behind it, 152 fpm
        /// per knot, a wing cushion of 350 fpm under 3.6 ft, NO sink near the
        /// ground, the column's neutral 0.084 of pull (the trim), the lever
        /// worth nothing but its speed. It does not replay the model flares:
        /// it cannot follow 1790781114 at all (4.8 ft rms at any column), and
        /// closed loop it lands the model flare as flown at under 100 fpm
        /// where the sim landed it at 584 (`the_first_fit_passed_the_flare_the_sim_failed`).
        pub(super) const EVO_FIRST: Fit = Fit {
            k: 6453.0,
            lag: 0.29,
            dead: 0.23,
            tail_fpm: 0.0,
            per_kt: 152.0,
            path_s: 1.5,
            wing_fpm: 351.0,
            wing_ft: 3.65,
            gauge_s: 0.965,
            off: 0.084,
            thr_fpm: 1.6,
            thr_lag: 0.6,
            drag0: 2.0,
            drag_w: 1.53,
            thrust: 3.3,
            vso: 61.0,
            ..FIT
        };

        /// Refitted to all three flares from the servos taken to the wheels,
        /// each with its own column and standing bias at the gate: 6,790 fpm
        /// per unit through two 0.49 s lags after 0.35 s, the path 1.06 s
        /// (held at the wing's own: 2 W/S over g rho V CL_alpha at 90 kt),
        /// the gauge 0.61 s behind it, 105 fpm per knot, and a sink the
        /// ground adds, 920 fpm at the wheels growing from 22 ft: radio 1.2 to
        /// 1.9 ft rms, gauge 40 to 55 fpm rms. Without that sink the best
        /// replay is 3.4 to 7.3 ft and 111 to 223 fpm; the path left free
        /// fits as well from 0.7 to 1.9 s with 440 to 880 fpm of it, so the
        /// path is held at the physics. The gear's kick is 2.5: past the
        /// recorded contacts it hops 0.35 ft after 1790793076 (none flown) and
        /// 3.2 ft after 1790781114 (3.9 flown); the SR22's 4.95 hops 8.5 and 24.
        pub(super) const EVO: Fit = Fit {
            k: 6794.0,
            lag: 0.493,
            dead: 0.352,
            path_s: 1.06,
            gauge_s: 0.612,
            per_kt: 104.8,
            tail_fpm: 920.0,
            tail_ft: 21.9,
            wing_fpm: 0.0,
            kick: 2.5,
            ..EVO_FIRST
        };

        /// Each recorded flare with the column (less the neutral) and the
        /// standing bias the refit found at its gate.
        fn flares() -> Vec<(Flown, f64, f64)> {
            vec![
                (flown(SRC), -0.0148, -2.5),
                (flown(MODEL_SRC[0]), 0.029, -84.4),
                (flown(MODEL_SRC[1]), 0.0145, 1.7),
            ]
        }

        /// The fit and its neighbours, every one of which still replays the
        /// three flares (4 ft, 95 fpm): the dead time 0.1 s either way, the
        /// path and the gauge 20 % either way, the ground's sink 15 % either
        /// way (30 % off, the last two seconds replay 110 fpm rms and worse).
        fn evo_plants() -> Vec<Fit> {
            vec![
                EVO,
                Fit { dead: EVO.dead + 0.1, ..EVO },
                Fit { dead: EVO.dead - 0.1, ..EVO },
                Fit { path_s: EVO.path_s * 1.2, ..EVO },
                Fit { path_s: EVO.path_s * 0.8, ..EVO },
                Fit { gauge_s: EVO.gauge_s * 1.2, ..EVO },
                Fit { gauge_s: EVO.gauge_s * 0.8, ..EVO },
                Fit { tail_fpm: EVO.tail_fpm * 1.15, ..EVO },
                Fit { tail_fpm: EVO.tail_fpm * 0.85, ..EVO },
            ]
        }

        /// The plant is the recording: each flare's column and lever as
        /// written, open loop, give back its radio and gauge to the wheels on
        /// every plant kept, and the fit follows 1790781114's pushed flare
        /// that the first fit cannot.
        #[test]
        fn the_evolution_plant_replays_its_flares() {
            for fit in evo_plants() {
                for (f, pipe, bias) in flares() {
                    let (dh, dv, _) = replay_rms(&f, fit, pipe, bias);
                    eprintln!("replay dead {:.2} path {:.2} gauge {:.2} ground {:.0}: {dh:.2} ft, {dv:.0} fpm rms", fit.dead, fit.path_s, fit.gauge_s, fit.tail_fpm);
                    assert!(dh < 4.0 && dv < 95.0, "{fit:?}: {dh:.2} ft, {dv:.0} fpm");
                }
            }
            let pushed = flown(MODEL_SRC[1]);
            let (fit_ft, ..) = replay_rms(&pushed, EVO, 0.0145, 1.7);
            let first_ft = [-0.1, -0.05, 0.0, 0.05, 0.1, 0.2]
                .iter()
                .map(|&pipe| replay_rms(&pushed, EVO_FIRST, pipe, 0.0).0)
                .fold(f64::INFINITY, f64::min);
            assert!(fit_ft < 1.5 && first_ft > 4.0, "the pushed flare: {fit_ft:.2} ft against {first_ft:.2}");
        }

        /// The model flare as flown in 1790793076 (`flare_elevator` before
        /// this fix: 6,450 fpm, 1.3 s lags, 0.67 s dead, no ground, 150 fpm
        /// a knot, to 200 fpm on a 3 s constant).
        fn flare_as_flown() -> crate::aircraft::FlareElevatorProfile {
            let mut fe = evolution().flare_elevator.expect("the Evolution declares its flare");
            fe.tau_s = Some(3.0);
            let mut m = fe.model.expect("a model");
            m.fpm_per_elevator = 6450.0;
            m.lag_s = 1.3;
            m.lag2_s = None;
            m.dead_s = 0.67;
            m.ground_fpm = 0.0;
            m.ground_ft = 40.0;
            m.touchdown_fpm = Some(200.0);
            m.fpm_per_kt = Some(150.0);
            fe.model = Some(m);
            fe
        }

        /// The recorded gates of the model flares with their columns and
        /// biases.
        const MODEL_GATES: [(f64, f64, f64, f64, f64); 2] =
            [(52.9, -339.0, 91.4, 0.029, -84.4), (52.6, -361.0, 90.3, 0.0145, 1.7)];

        /// Closed loop from the recorded gates, the model flare as flown
        /// meets the refit plants as it met the runway (584 fpm, the gauge
        /// the tick before it 356), and the first fit landed it softly: the
        /// tests passed the flare the sim failed.
        #[test]
        fn the_first_fit_passed_the_flare_the_sim_failed() {
            let power = evolution().flare_power.expect("the Evolution declares its retard");
            let mut refit_soft = f64::INFINITY;
            for fit in evo_plants() {
                for gate in MODEL_GATES {
                    let (l, gauge, _) = land_model(fit, gate, flare_as_flown(), power, 90.0, true);
                    refit_soft = refit_soft.min(l.first_fpm.min(gauge));
                }
            }
            let (h, v, ias, _, _) = MODEL_GATES[0];
            let (first, _, _) = land_model(EVO_FIRST, (h, v, ias, 0.071 - 0.084, 0.0), flare_as_flown(), power, 90.0, true);
            eprintln!("as flown: refit plants no softer than {refit_soft:.0} fpm, the first fit {:.0}", first.first_fpm);
            assert!(refit_soft > 400.0, "the refit plants: {refit_soft:.0}");
            assert!(first.first_fpm < 150.0, "the first fit: {:.0}", first.first_fpm);
        }

        /// The time constant is not the fault: the model as flown on a 3, 5
        /// or 8 s constant meets the refit plant hard from both recorded
        /// model gates. What it lacks is the sink near the ground.
        #[test]
        fn a_flatter_exponential_alone_lands_as_hard() {
            let power = evolution().flare_power.expect("the Evolution declares its retard");
            let mut soft = f64::INFINITY;
            for tau in [3.0, 5.0, 8.0] {
                let mut fe = flare_as_flown();
                fe.tau_s = Some(tau);
                for gate in MODEL_GATES {
                    for rough in [false, true] {
                        let (l, _, _) = land_model(EVO, gate, fe, power, 90.0, rough);
                        soft = soft.min(l.first_fpm);
                    }
                }
            }
            eprintln!("Evolution as flown on 3 to 8 s: no softer than {soft:.0} fpm");
            assert!(soft > 400.0, "{soft:.0}");
        }

        /// The loop's default tracker as it flew the Evolution: the arrest's
        /// target above the flare gate, the flare's below it, the ratchet in
        /// the flare, the lever idle there but the approach's own on a
        /// balloon, the channel let go on the wheels.
        pub(super) fn land_tracker(p: &mut Plant, gains: FlareGains, until: f64) -> Landing {
            let mut law = FlarePitch::with_gains(gains);
            let (mut next, mut flaring, mut rolling, mut wheels_on) = (0.0, false, false, None::<f64>);
            let mut prev: Option<f64> = None;
            let mut min_ias = p.ias;
            let approach_thr = p.thr0;
            while p.t < until {
                if p.t + 1e-9 >= next {
                    next += TICK_S;
                    let (gs, radio, vvi) = (p.ias, p.radio(), p.vvi());
                    wheels_on = wheels_on_time(wheels_on, p.on_wheels, radio, TICK_S);
                    if should_start_rollout(rolling, p.on_wheels, radio, gs, p.ias, 90.0, wheels_on) {
                        rolling = true;
                    }
                    let trend = prev.map_or(0.0, |v| (vvi - v) / TICK_S);
                    prev = Some(vvi);
                    if rolling {
                        p.thr_cmd = 0.0;
                    } else {
                        let (flare_ft, band_ft) = arrest_gates_ft(25.0, gs);
                        flaring = radio <= flare_ft || (flaring && radio <= band_ft);
                        let nominal = -(gs / 60.0) * gp_ft_per_nm();
                        let (roundout, flare_vs) = arrest_targets_fpm(nominal);
                        let target = if flaring { flare_vs } else { roundout };
                        let e = law.step(target, vvi, trend, TICK_S, flaring);
                        p.write(e);
                        p.thr_cmd = if !flaring {
                            approach_thr
                        } else {
                            balloon_power(Some(vvi), Some(approach_thr), radio > band_ft, false)
                                .unwrap_or(0.0)
                        };
                    }
                }
                p.step(0.01);
                if p.contacts.is_empty() {
                    min_ias = min_ias.min(p.ias);
                }
            }
            let first = p.contacts.first().copied().unwrap_or((f64::NAN, f64::NAN, f64::NAN));
            Landing {
                first_fpm: -first.1,
                contacts: p.contacts.len(),
                bounce_ft: p.bounce_ft,
                min_ias,
                secs_to_wheels: first.0,
            }
        }

        fn evolution() -> crate::aircraft::AircraftProfile {
            crate::aircraft::load("lancair_evolution").expect("the Evolution profile loads")
        }

        /// The three recorded gates with their columns and biases, and the
        /// tracker's gate's neighbours: steeper and shallower, faster and
        /// slower, lower.
        const GATES: [(f64, f64, f64, f64, f64); 7] = [
            (50.2, -404.0, 90.1, -0.0148, -2.5),
            (52.9, -339.0, 91.4, 0.029, -84.4),
            (52.6, -361.0, 90.3, 0.0145, 1.7),
            (50.0, -470.0, 90.0, -0.0148, -2.5),
            (50.0, -340.0, 90.0, -0.0148, -2.5),
            (50.0, -404.0, 94.0, -0.0148, -2.5),
            (46.0, -404.0, 87.0, -0.0148, -2.5),
        ];

        /// The tracker as flown drops in on the plant from its recorded
        /// gate, as it did (the gauge's 392 at the wheels, the height
        /// falling at 800 or more); the declared model flare
        /// (`flare_elevator` model, `flare_power`) from every gate on every
        /// plant that replays, calm and rough: one contact at 60 to 250 fpm
        /// on the flight path and on the gauge, under 2 ft after it, never
        /// slower than the stall, and no more than 700 m past the gate (the
        /// tracker's contact came about 580 m past it on a 601 m strip, so a
        /// float to the stall from the POM's 90 kt final is no answer). The
        /// hop was held under the SR22's 1.5 ft; the refit's gear is the
        /// Evolution's own (EVO's kick, from its two model contacts: 0.35 ft
        /// replayed where none was flown, 3.2 where 3.9 was), and on the
        /// ground's 15 % heavier neighbour it hops 1.5 to 1.7 ft off 231 to
        /// 242 fpm, the size of the 0.35 ft it overstates plus the hop a
        /// 240 fpm contact makes on it.
        #[test]
        fn the_evolution_flares_where_the_tracker_dropped_it_in() {
            let mut flown_worst = 0.0_f64;
            for fit in evo_plants() {
                let (h, v, ias, pipe, bias) = GATES[0];
                let mut p = Plant::at(fit, h, v, ias, pipe);
                p.base += bias;
                let l = land_tracker(&mut p, FlareGains::default(), 30.0);
                flown_worst = flown_worst.max(l.first_fpm);
            }
            assert!(flown_worst > 400.0, "the tracker as flown: {flown_worst:.0} fpm");
            let prof = evolution();
            let fe = prof.flare_elevator.expect("the Evolution declares its flare");
            let power = prof.flare_power.expect("the Evolution declares its retard");
            assert!(!FlareModel::from_profile(Some(fe)).expect("a model").holds_off());
            let mut worst = (f64::INFINITY, 0.0_f64, 0.0_f64, 0.0_f64);
            let mut bad: Vec<String> = vec![];
            for fit in evo_plants() {
                for gate in GATES {
                    for rough in [false, true] {
                        let (l, gauge, along) = land_model(fit, gate, fe, power, 90.0, rough);
                        let why = format!(
                            "dead {:.2} path {:.2} gauge {:.2} ground {:.0}, {gate:?} rough {rough}: {l:?}, gauge {gauge:.0}, {along:.0} m",
                            fit.dead, fit.path_s, fit.gauge_s, fit.tail_fpm
                        );
                        if !((60.0..=250.0).contains(&l.first_fpm)
                            && (60.0..=250.0).contains(&gauge)
                            && l.bounce_ft < 2.0
                            && l.min_ias > fit.vso
                            && along < 700.0)
                        {
                            bad.push(why);
                        }
                        worst = (
                            worst.0.min(l.first_fpm.min(gauge)),
                            worst.1.max(l.first_fpm.max(gauge)),
                            worst.2.max(l.bounce_ft),
                            worst.3.max(along),
                        );
                    }
                }
            }
            eprintln!(
                "Evolution flare: {:.0} to {:.0} fpm, {:.2} ft after, {:.0} m past the gate",
                worst.0, worst.1, worst.2, worst.3
            );
            assert!(bad.is_empty(), "{} landings out of limits:\n{}", bad.len(), bad.join("\n"));
        }
    }
