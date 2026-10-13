    /// Every autoland tick's bug (true) from four LEMG 13 approaches of
    /// 2026-09-28, the arm to the wheels (two 5 nm finals, one with an
    /// intercept, and a full-auto circuit's final).
    const LEMG_13_FINAL_KNOBS: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/landings/lemg13_final_knobs.jsonl"));

    /// Issue #93: those approaches' bugs replayed through the knob autoland
    /// now sets (`bug_knob`): the knob moved (0.2 degrees or more, the least
    /// `drive_heading_bug_step` writes) less often than the law's bug in
    /// every approach and two thirds as often in all, down to the arrest
    /// (the arrest and the flare still steer every tick), and always within
    /// a turn's steering of the law's own bug. On the final's quarter degree
    /// a second (2026-10-12; the last 0.3 nm no longer steered every tick):
    /// 47 moves against 93, where half a degree every two seconds outside
    /// the short final made 21 against 76.
    #[test]
    fn the_recorded_finals_move_the_knob_less() {
        let moved = |a: Option<f64>, b: f64| {
            a.is_none_or(|a| crate::angles::abs_diff_deg(b, a) >= 0.2)
        };
        let mut by_sortie: Vec<(String, Vec<serde_json::Value>)> = Vec::new();
        for line in LEMG_13_FINAL_KNOBS.lines() {
            let v: serde_json::Value = serde_json::from_str(line).expect("a bench line");
            let s = v["sortie"].as_str().unwrap().to_string();
            match by_sortie.last_mut() {
                Some((id, ticks)) if *id == s => ticks.push(v),
                _ => by_sortie.push((s, vec![v])),
            }
        }
        assert_eq!(by_sortie.len(), 4);
        let (mut free_all, mut held_all) = (0u32, 0u32);
        for (s, ticks) in &by_sortie {
            let mut hold = crate::heading_bug::BugHold::default();
            let (mut free, mut held) = (0u32, 0u32);
            let (mut free_at, mut held_at) = (None::<f64>, None::<f64>);
            let t0 = ticks[0]["t"].as_f64().unwrap();
            for k in ticks {
                let b = k["bug_true"].as_f64().unwrap();
                let phase = match k["phase"].as_str() {
                    Some("Flare") => super::ApproachPhase::Flare,
                    Some("Final") => super::ApproachPhase::Final,
                    _ => super::ApproachPhase::Intercept,
                };
                let short = k["arresting"] == true;
                let leg = match k["leg"].as_str() {
                    Some("outbound") => Some(crate::reversal::Leg::Outbound),
                    Some("base") => Some(crate::reversal::Leg::Base),
                    Some("inbound") => Some(crate::reversal::Leg::Inbound),
                    _ => None,
                };
                let knob = match super::bug_knob(phase, short, leg, None) {
                    Some((key, db)) => {
                        hold.decide(key, b, k["t"].as_f64().unwrap() - t0, db);
                        let h = hold.set().unwrap_or(b);
                        let off = crate::angles::abs_diff_deg(h, b);
                        assert!(
                            off < crate::heading_bug::BUG_STEER_DEG,
                            "{s}: the knob {h:.1} against the law's {b:.1}"
                        );
                        h
                    }
                    None => {
                        hold.reset();
                        free_at = None;
                        held_at = None;
                        continue;
                    }
                };
                if moved(free_at, b) {
                    free += 1;
                    free_at = Some(b);
                }
                if moved(held_at, knob) {
                    held += 1;
                    held_at = Some(knob);
                }
            }
            assert!(held < free, "{s}: {held} moves against {free}");
            free_all += free;
            held_all += held;
        }
        assert!(held_all * 3 <= free_all * 2, "{held_all} moves against {free_all}");
    }
