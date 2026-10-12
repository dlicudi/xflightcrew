use super::*;

/// The six arrests autoland flew on the Zibo at LEMG 30 on 2026-10-07 (the
/// final from 400 ft, the arrest to the wheels, the rollout's first tick
/// and the flight model's touchdown): sorties 1791363565 (450 fpm, the
/// fixed 145 kt), 1791371247 (203), the "land here" at 1791383430 (209),
/// sortie 1791385153 (259), the reposition at 1791388663 (326) and sortie
/// 1791392327 (475).
const BENCH: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/bench/landings/zibo_lemg30_flares.jsonl"
));

#[derive(Debug, Clone, Copy)]
struct Tick {
    t: f64,
    radio: f64,
    ias: f64,
    tgt_ias: f64,
    elev: Option<f64>,
    arresting: bool,
    ground: bool,
    dist: f64,
}

struct Flare {
    id: String,
    ticks: Vec<Tick>,
}

fn flares() -> Vec<Flare> {
    let mut out: Vec<Flare> = Vec::new();
    for line in BENCH.lines() {
        let v: serde_json::Value = serde_json::from_str(line).expect("a bench line");
        let id = v["flare"].as_str().expect("an id").to_string();
        if out.last().is_none_or(|f| f.id != id) {
            out.push(Flare {
                id: id.clone(),
                ticks: Vec::new(),
            });
        }
        let f = out.last_mut().expect("pushed");
        let n = |k: &str| v[k].as_f64().unwrap_or(f64::NAN);
        if v["kind"].as_str() == Some("tick") {
            f.ticks.push(Tick {
                t: n("t"),
                radio: n("radio"),
                ias: n("ias"),
                tgt_ias: n("tgt_ias"),
                elev: v["flare_elev"].as_f64(),
                arresting: v["arresting"] == true,
                ground: v["ground"] == true,
                dist: n("dist"),
            });
        }
    }
    assert_eq!(out.len(), 6);
    out
}

impl Flare {
    /// The arrest: the servos out and our column written, to the first tick
    /// on the wheels.
    fn arrest(&self) -> Vec<Tick> {
        let mut a = Vec::new();
        for k in self
            .ticks
            .iter()
            .filter(|k| k.arresting && k.elev.is_some())
        {
            a.push(*k);
            if k.ground || k.radio < 1.5 {
                break;
            }
        }
        a
    }
    /// The groundspeed through the arrest, kt (the threshold distance's
    /// travel, which is unsigned either side of the threshold).
    fn gs_kts(&self) -> f64 {
        let a = self.arrest();
        let nm: f64 = a.windows(2).map(|w| (w[1].dist - w[0].dist).abs()).sum();
        nm / (a[a.len() - 1].t - a[0].t) * 3600.0
    }
    /// The gate's distance short of the threshold, metres.
    fn gate_short_m(&self) -> f64 {
        self.arrest()[0].dist * M_PER_NM
    }
}

/// The go/no-go at the roundout gate (`landing_call`) on the attitude
/// flare: LAND from each recorded gate at LEMG 30, the touchdown predicted
/// 844 to 1,129 m past the threshold, 1,620 m or more left against the
/// roll's 792 (the old law's 6.5 s exponential predicted 557 to 926).
#[test]
fn the_landing_call_still_lands_every_recorded_gate() {
    let p = crate::aircraft::load("zibo_738").expect("the Zibo profile loads");
    let need = wheels_need_m(
        p.reference
            .as_ref()
            .and_then(|r| r.landing_distance_ft)
            .map(|f| f * M_PER_FT),
        p.reference
            .as_ref()
            .and_then(|r| r.landing_roll_ft)
            .map(|f| f * M_PER_FT),
    );
    for f in flares() {
        let g = f.arrest()[0];
        let call = landing_call(
            f.gate_short_m(),
            2751.46,
            g.radio,
            f.gs_kts(),
            Some(g.ias),
            g.tgt_ias,
            need,
        )
        .expect("a call");
        assert!(call.land && call.touchdown_m < 1200.0, "{}: {call:?}", f.id);
    }
}
