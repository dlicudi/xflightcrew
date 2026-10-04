    const LEMG13: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/takeoffs/lemg13_circuit_climb_outs.jsonl"));

    /// One climb-out as the blackbox drew it: `autotakeoff` `climb_manual`
    /// (t, agl, ias), the `ap_engaged` tick (t, agl, ias, cmd_vs), every VS
    /// command after it (`climb`, then the circuit's `leg`, as (t, cmd_vs)),
    /// the handoff time, and every flap handle move (t, ratio), the
    /// one-shot `flaps_handoff` as 0.
    pub struct Recorded {
        pub sortie: String,
        pub manual: Vec<(f64, f64, f64)>,
        pub engage: (f64, f64, f64, f64),
        pub commands: Vec<(f64, f64)>,
        pub handoff_t: f64,
        pub flaps: Vec<(f64, f64)>,
    }

    pub fn load() -> Vec<Recorded> {
        let recs: Vec<serde_json::Value> = LEMG13
            .lines()
            .filter_map(|l| serde_json::from_str(l).ok())
            .collect();
        let mut sorties: Vec<String> = recs
            .iter()
            .filter_map(|r| r["sortie"].as_str().map(str::to_string))
            .collect();
        sorties.dedup();
        let f = |r: &serde_json::Value, k: &str| r[k].as_f64().unwrap_or(f64::NAN);
        sorties
            .into_iter()
            .map(|sortie| {
                let mine: Vec<&serde_json::Value> = recs
                    .iter()
                    .filter(|r| r["sortie"] == sortie.as_str())
                    .collect();
                let ev =
                    |r: &serde_json::Value, k: &str, e: &str| r["kind"] == k && r["event"] == e;
                let manual = mine
                    .iter()
                    .filter(|r| ev(r, "autotakeoff", "climb_manual"))
                    .map(|r| (f(r, "t"), f(r, "agl"), f(r, "ias")))
                    .collect();
                let e = mine
                    .iter()
                    .find(|r| ev(r, "autotakeoff", "ap_engaged"))
                    .expect("an engage");
                let commands = mine
                    .iter()
                    .filter(|r| ev(r, "autotakeoff", "climb") || ev(r, "circuit", "leg"))
                    .map(|r| (f(r, "t"), f(r, "cmd_vs")))
                    .collect();
                let handoff_t = mine
                    .iter()
                    .find(|r| ev(r, "autotakeoff", "handoff"))
                    .map(|r| f(r, "t"))
                    .expect("a handoff");
                let flaps = mine
                    .iter()
                    .filter_map(|r| {
                        if ev(r, "autotakeoff", "flap_step") {
                            Some((f(r, "t"), f(r, "to")))
                        } else if ev(r, "autotakeoff", "flaps_handoff") && r["up"] == true {
                            Some((f(r, "t"), 0.0))
                        } else {
                            None
                        }
                    })
                    .collect();
                Recorded {
                    sortie,
                    manual,
                    engage: (f(e, "t"), f(e, "agl"), f(e, "ias"), f(e, "cmd_vs")),
                    commands,
                    handoff_t,
                    flaps,
                }
            })
            .collect()
    }

    /// The climb the pitch was flying as the autopilot took it: the slope
    /// of the last second of hand-flown heights, fpm (over the runway the
    /// height above it is the altitude less a constant).
    pub fn flown_fpm(manual: &[(f64, f64, f64)]) -> f64 {
        let t_end = manual.last().expect("hand-flown ticks").0;
        let last: Vec<&(f64, f64, f64)> = manual.iter().filter(|m| m.0 >= t_end - 1.0).collect();
        let n = last.len() as f64;
        let (mt, mh) = (
            last.iter().map(|m| m.0).sum::<f64>() / n,
            last.iter().map(|m| m.1).sum::<f64>() / n,
        );
        let num: f64 = last.iter().map(|m| (m.0 - mt) * (m.1 - mh)).sum();
        let den: f64 = last.iter().map(|m| (m.0 - mt).powi(2)).sum();
        num / den * 60.0
    }

    /// The 737's VS mode as EGPH 24 drew it (2026-09-21): 2 s of dead time,
    /// then two 1.5 s lags. The LEMG 13 engages agree: the dial read 1,500
    /// and the aircraft held 2,150 to 2,250 for the four seconds to 500 ft.
    pub struct VsMode {
        delay: std::collections::VecDeque<f64>,
        vs1: f64,
        pub vs: f64,
    }

    impl VsMode {
        pub fn new(vs: f64, dt: f64) -> Self {
            let n = (2.0 / dt).round() as usize;
            VsMode {
                delay: std::collections::VecDeque::from(vec![vs; n]),
                vs1: vs,
                vs,
            }
        }
        pub fn step(&mut self, cmd: f64, dt: f64) -> f64 {
            self.delay.push_back(cmd);
            let seen = self.delay.pop_front().unwrap_or(cmd);
            self.vs1 += (seen - self.vs1) * dt / 1.5;
            self.vs += (self.vs1 - self.vs) * dt / 1.5;
            self.vs
        }
    }

    /// Knots a second at `ias` climbing `vs_fpm` with a thrust-to-weight
    /// excess `tw` (#78's fit to LEMG 13: 0.185).
    pub fn accel_kts_s(tw: f64, ias: f64, vs_fpm: f64) -> f64 {
        let sin_gamma = (vs_fpm / (ias * 101.27)).clamp(-1.0, 1.0);
        (tw - sin_gamma) * 9.81 / 0.514_444
    }

    /// The climb the #78 pitch hold flies at V2 + 15 on that fit: every bit of
    /// the excess into the path, about 3,090 fpm at 165 kt.
    pub fn held_climb_fpm(tw: f64, ias: f64) -> f64 {
        ias * 101.27 * tw
    }
