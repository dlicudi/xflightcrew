//! The sortie scorecard: blackbox ticks in, one `stage_result` per stage out.
//!
//! Unattended testing needs a verdict per stage, not a pilot watching (asked
//! 2026-09-25). Pure: [`Scorer::feed`] takes each blackbox record in order and
//! returns the records it concludes, so the SAME code runs live (a blackbox tap,
//! `stage_result` written beside the ticks) and offline (`xvl debrief` over any
//! file). Every pass line is a number with a limit, because the complaints that
//! started it ("jerky braking", "lined up at an angle", "it wobbles on the
//! centreline") were seen and never measured. The limits are first guesses,
//! named here so they can be argued with.

use serde_json::{json, Value};

/// Taxi: most off the centreline.
pub const TAXI_MAX_LAT_M: f64 = 3.0;
/// Taxi: most INSIDE a corner of the route, while turning through it. The
/// route is a polyline and its corners are vertices no wheel can follow: an
/// aircraft flying a turn of radius R through a vertex of deflection D is
/// R (1 - cos D/2) inside it at the bisector however well it is flown, and
/// the projection onto the polyline measures exactly that. EGBB 33
/// (1790534902, 2026-09-27): the 737 held 0.1 to 0.5 m down every straight
/// and scored "3.2 m off the line" at a 56 degree vertex flown at 21 m
/// (geometry alone: 2.5 m) and "4.2 m" on the line-up's 92 degree vertex at
/// 13 m (3.9 m), each peak at the tick the projection jumped 3 to 7 m
/// along the route across the vertex's inside. 14 of the taxi and line-up
/// failures in the 22 sorties on file (3.1 to 4.3 m) were this shape. The
/// allowance needs that jump: inside a drawn CURVE (no jump) the law's own
/// bend cap holds the cut to 1.5 m, so a deeper one there is judged as the
/// line (LIME's 8.5 m inside a gentle bend, 2026-09-25). 5 m covers the
/// 737's 90 degree vertex at its full-lock radius; a swing OUTSIDE a corner
/// (LEMG Alpha's 5.8 m wide) is judged against [`TAXI_MAX_LAT_M`].
pub const TAXI_MAX_CUT_M: f64 = 5.0;
/// The route's own turn rate (the drive's `turn_dps`) at or above which a
/// corner is open, and the yaw rate a tick must carry the same way to be
/// turning through it.
pub const CORNER_TURN_DPS: f64 = 2.0;
pub const CORNER_YAW_DPS: f64 = 2.0;
/// A corner is closed this long after the route last asked for the turn.
pub const CORNER_TAIL_S: f64 = 10.0;
/// A tick's `remaining_m` falling by more than the ground covered plus this
/// much (and this many seconds of groundspeed, for the ticks' own timing
/// jitter: straights at 18 kt read up to 1.3 m over) is the projection
/// crossing a vertex. EGBB's four vertices read 1.9 to 6.5 m over.
const CORNER_JUMP_M: f64 = 1.0;
const CORNER_JUMP_S: f64 = 0.15;
const KTS_TO_MS: f64 = 0.514_444;
/// Taxi: after a JOIN (the drive engaged off its route and turned onto it)
/// the line is judged from the first tick inside [`TAXI_MAX_LAT_M`]; getting
/// there may take this long from the `joined` record. The join's own record
/// fires at 15 m and 45 degrees, so every judged join opened with up to 15 m
/// "off the line" that the law was still closing (1790441293: 14.5 m at the
/// `joined` tick, 6.7 s and 23 m to be inside 3 m). Measured over 211 joins
/// in the logs: median 6 s, 90th percentile 12 s.
pub const TAXI_CAPTURE_MAX_S: f64 = 30.0;
/// Taxi: a drive tick PAST the planned stop (`remaining_m` below zero; the
/// stop is `STOP_SHORT_M`, 5 m, before the route's last point) is not a
/// lateral sample. The drive is only stopping there (`stop_straighten` has
/// faded the steering to its floor from 1 m before the stop), and the projection runs onto the route's last stub or clamps to its end
/// point, so the "offset" is the distance to that point, overrun included:
/// 1790426786 read 2.8 to 3.8 m at err 0 creeping onto the hold at
/// `remaining_m` -5, and 1790414806 3.0 to 4.4 m against a final stub 106
/// degrees off the nose at -2 to -4.
const TAXI_PAST_END_M: f64 = 0.0;
/// A `yoke_under_ap` within this of the crew's own yoke-channel write is the
/// loop's output read back, never a hand (the monitor's own window).
const CREW_YOKE_FRESH_S: f64 = 1.5;
/// A `brake_yield` within this of the crew's own brake command is the
/// plugin latching on OUR command, never a hand: the held command's 1 s
/// plus the 1 s decay tail of `tools::brake_cmd_quiet`, the monitor's own
/// window. EGKK (1790524628, 1790525414, 1790525500): 0.4 to 0.9 s after
/// each reject.
const CREW_BRAKE_FRESH_S: f64 = 2.0;
/// Taxi: fastest the brake command may move (ratio per second) outside a stop.
pub const TAXI_MAX_BRAKE_RATE: f64 = 1.5;
/// A takeoff starts aligned or it is a line-up failure.
pub const LINEUP_MAX_ERR_DEG: f64 = 3.0;
pub const LINEUP_MAX_XTK_M: f64 = 2.0;
/// Takeoff roll: most off the centreline.
pub const ROLL_MAX_XTK_M: f64 = 5.0;
/// Touchdown: firmer than this fails; crabbed more than this fails.
pub const TOUCHDOWN_MAX_SINK_FPM: f64 = 400.0;
pub const TOUCHDOWN_MAX_DRIFT_DEG: f64 = 5.0;
/// The first contact is judged, not the one the state record happens to
/// catch. The 1 Hz `state` sample never saw the wheels on LEMG 13
/// (1790441293): the 737 met the runway at 17:04:34Z at -794 fpm (the
/// autoland flare tick, radio -0.5 ft), bounced to 11.8 ft, and the stage
/// scored the SECOND contact's 278 fpm, a pass on sink. An autoland tick
/// with the wheels on (`ground`, logged since 2026-09-26) or, in older
/// logs, the radio height at or under this is a contact.
const CONTACT_RADIO_FT: f64 = 1.0;
/// Back up past this after a contact is a bounce (rapport's own 2 ft).
const BOUNCE_FT: f64 = 2.0;
/// A contact this long before the state record's touchdown belongs to it.
const CONTACT_WITHIN_S: f64 = 30.0;
/// The flight model's `touchdown` event this close to the state record that
/// saw the wheels is the same contact (it is recorded just before it, from
/// the same monitor tick).
const FLIGHT_TD_WITHIN_S: f64 = 2.0;
/// Rollout: most off the centreline.
pub const ROLLOUT_MAX_XTK_M: f64 = 5.0;
/// A stream quiet this long has ended.
const GAP_S: f64 = 5.0;

fn f(r: &Value, k: &str) -> Option<f64> {
    r.get(k).and_then(Value::as_f64)
}

fn s<'a>(r: &'a Value, k: &str) -> &'a str {
    r.get(k).and_then(Value::as_str).unwrap_or("")
}

fn r1(v: f64) -> f64 {
    (v * 10.0).round() / 10.0
}

fn r2(v: f64) -> f64 {
    (v * 100.0).round() / 100.0
}

/// One taxi (or line-up) drive, from `autotaxi` `drive` ticks.
#[derive(Debug, Default, Clone)]
struct Taxi {
    line_up: bool,
    t0: f64,
    t_last: f64,
    ticks: usize,
    max_gs: f64,
    /// Off the line and inside its vertices.
    line: LineJudge,
    brake_rate: f64,
    brake_reversals: usize,
    yaw_swings: usize,
    last: Option<(f64, f64, f64, f64)>, // t, bl, br, yaw
    last_err: f64,
    last_lat: f64,
    ended_by: String,
    /// Not yet on the line: joining, or joined and still closing.
    capture: Capture,
    /// Closest to the line while not yet judged (for the never-captured line).
    closest_uncaptured: Option<f64>,
    /// Seconds from `joined` to the first tick on the line.
    capture_secs: Option<f64>,
    past_end_ticks: usize,
    sortie: Option<String>,
}

/// A corner of the route the drive is turning through.
#[derive(Debug, Clone, Copy)]
struct Corner {
    /// +1 a right turn, -1 a left.
    side: f64,
    /// The last time the route asked for it (`turn_dps`).
    asked_t: f64,
    /// Most inside it so far, judged when it closes.
    inside: f64,
    /// The projection jumped across its inside: a vertex, not a curve.
    vertex: bool,
}

/// The centreline, judged: most off the line ([`TAXI_MAX_LAT_M`]) and most
/// inside a route VERTEX while turning through it ([`TAXI_MAX_CUT_M`]). One
/// measure for the sortie's taxi drive and the instructor's taxi lesson
/// (`instructor::Task::Taxi`), so a pilot and the crew are held to the same
/// line. Pure: fed per tick with the signed offset (positive right of the
/// route), the route left, the side the route asks a turn and the yaw.
#[derive(Debug, Default, Clone)]
pub struct LineJudge {
    /// Most off the line, metres (a drawn curve's inside included).
    pub max_lat: f64,
    /// Most inside a route vertex while turning through it, metres.
    pub max_cut: f64,
    /// The corner being turned through.
    corner: Option<Corner>,
    /// The last tick's time and `remaining_m`, for the projection's jump.
    prev_rem: Option<(f64, f64)>,
}

impl LineJudge {
    /// One tick's geometry: `asked` is the side (+1 right, -1 left) the route
    /// asks a turn this tick, if it does. A corner opens where the route asks
    /// for a turn and stays open while the aircraft is still turning that way
    /// (the projection has passed the vertex by the time the offset peaks,
    /// and the route's turn reads zero there). Returns whether `lat_m` is on
    /// the open corner's inside: a CUT, judged when the corner closes.
    pub fn corner(
        &mut self,
        t: f64,
        gs_kts: f64,
        lat_m: f64,
        remaining_m: Option<f64>,
        asked: Option<f64>,
        yaw_dps: f64,
    ) -> bool {
        // The projection leaping along the route faster than the aircraft
        // moves is its foot crossing from one leg of a vertex to the next:
        // 2 |lat| tan(D/2) at once, 3 to 7 m at EGBB and EGKK.
        let jumped = match (self.prev_rem, remaining_m) {
            (Some((t1, r1)), Some(r2)) => {
                let v = gs_kts * KTS_TO_MS;
                (r1 - r2) - v * (t - t1).max(0.0) > CORNER_JUMP_M + CORNER_JUMP_S * v
            }
            _ => false,
        };
        self.prev_rem = remaining_m.map(|m| (t, m));
        if let Some(side) = asked {
            match self.corner.as_mut() {
                Some(c) if c.side == side => c.asked_t = t,
                _ => {
                    self.flush();
                    self.corner = Some(Corner {
                        side,
                        asked_t: t,
                        inside: 0.0,
                        vertex: false,
                    });
                }
            }
        }
        if self
            .corner
            .is_some_and(|c| yaw_dps * c.side < CORNER_YAW_DPS || t - c.asked_t > CORNER_TAIL_S)
        {
            self.flush();
        }
        if let Some(c) = self.corner.as_mut() {
            c.vertex |= jumped;
        }
        self.corner.is_some_and(|c| lat_m * c.side > 0.0)
    }

    /// The corner being turned through, if one is open: whether the
    /// projection has jumped across it yet (a vertex, not a drawn curve).
    pub fn open_vertex(&self) -> Option<bool> {
        self.corner.map(|c| c.vertex)
    }

    /// Judge one tick's offset, `cutting` as [`LineJudge::corner`] said.
    pub fn judge(&mut self, lat_m: f64, cutting: bool) {
        match self.corner.as_mut() {
            Some(c) if cutting => c.inside = c.inside.max(lat_m.abs()),
            _ => self.max_lat = self.max_lat.max(lat_m.abs()),
        }
    }

    /// Close the open corner: inside a vertex it is a cut, inside a drawn
    /// curve it is off the line like any other tick (the law's own bend cap
    /// keeps a curve's cut to 1.5 m, so one past the limit is real).
    pub fn flush(&mut self) {
        if let Some(c) = self.corner.take() {
            if c.vertex {
                self.max_cut = self.max_cut.max(c.inside);
            } else {
                self.max_lat = self.max_lat.max(c.inside);
            }
        }
    }
}

/// Where a taxi drive stands against its line.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
enum Capture {
    /// On the line: every tick is judged.
    #[default]
    Tracking,
    /// Turning onto the route (`join` .. `joined`): not judged.
    Joining,
    /// `joined` at this time, still closing on the line: not judged.
    Closing(f64),
}

/// One takeoff roll, from `autotakeoff` `roll` ticks.
#[derive(Debug, Default, Clone)]
struct Roll {
    t0: f64,
    start_err: Option<f64>,
    start_xtk: Option<f64>,
    max_xtk: f64,
    max_err_fast: f64,
    max_brake: f64,
    last_xtk: f64,
    last_ias: f64,
    ticks: usize,
    t_last: f64,
    sortie: Option<String>,
}

/// The landing, from `state` (touchdown) and `autoland` `rollout` ticks.
#[derive(Debug, Default, Clone)]
struct Landing {
    t_td: f64,
    sink: f64,
    drift: f64,
    max_xtk: Option<f64>,
    t_last: f64,
    sortie: Option<String>,
    /// Off the runway again after the first contact: the height reached
    /// and the sink at the contact the state record caught.
    bounce: Option<(f64, f64)>,
}

/// The first time the wheels met the runway, from the autoland ticks.
#[derive(Debug, Clone, Copy)]
struct Contact {
    t: f64,
    /// Steepest sink in the [`instructor::TOUCHDOWN_WINDOW_S`] before it.
    sink: f64,
    /// Highest radio height since (a bounce past [`BOUNCE_FT`]).
    up_ft: f64,
}

#[derive(Debug, Default)]
pub struct Scorer {
    taxi: Option<Taxi>,
    line_up_next: bool,
    roll: Option<Roll>,
    landing: Option<Landing>,
    /// Recent airborne state samples (t, vsi, hdg, track) for the touchdown.
    air: Vec<(f64, f64, f64, Option<f64>)>,
    airborne_since: Option<f64>,
    interventions: usize,
    /// `yoke_under_ap` records that were the crew's own channel write.
    crew_yoke_discounted: usize,
    /// `brake_yield` records that were the crew's own brake command.
    crew_brake_discounted: usize,
    /// The pilot's commands to the crew during the sortie (`tool` records).
    commands: usize,
    results: Vec<(String, bool)>,
    /// The aircraft struck the ground or wrecked inside the sortie (a
    /// `flight` `impact` or `Crashed`, a `state` `crashed`, a `rapport`
    /// touchdown with `impact`): the sortie fails whatever its stages say.
    crashed: bool,
    /// A `join` (or `joined`) arrived before the drive's first tick: the
    /// drive logs both before its first `drive` record when the engage is
    /// already inside the join-done gate.
    capture_pending: Option<Capture>,
    /// The sortie the last record belonged to (untagged: none).
    sortie: Option<String>,
    /// Last time a loop of ours wrote the elevator/aileron channel.
    crew_yoke_t: Option<f64>,
    /// Last time a loop of ours fired the sim's brake commands: a reject
    /// (`brakes_regular`, then `brakes_max` renewed through its rollout),
    /// the taxi's arrest when its hold vanished, or a state record whose
    /// `in.crew_brake` says a crew brake was in flight.
    crew_brake_t: Option<f64>,
    /// The takeoff has rotated: its roll ticks now write the elevator.
    rotated: bool,
    /// Recent autoland (t, vvi) for the first contact's sink.
    flare_vvi: Vec<(f64, f64)>,
    contact: Option<Contact>,
    /// The flight model's last `touchdown` (t, sink): the landing's sink
    /// when it belongs to the state record's touchdown (#142).
    flight_td: Option<(f64, f64)>,
}

/// The taxi events that mean the drive STOPPED short, not arrived. Each used
/// to end the stream only by the 5 s gap, and the drive was judged on its
/// offset alone: 1790408535 stood `stagnant` 32 m off its route (the join
/// never done) and scored "taxi pass".
const TAXI_FAILURES: &[&str] = &[
    "stagnant",
    "off_route",
    "pedal_yield",
    "hold_gone",
    "overspeed_stop",
    "tug_never_released",
    "stopped_from_outside",
    // The route's end reached off its line (`autotaxi::arrived_on_line`).
    "stopped_off_line",
];

/// The takeoff events that mean the roll was STOPPED, not finished.
const ROLL_FAILURES: &[&str] = &[
    "reject",
    "wrong_runway",
    "no_runway_fit",
    "pedal_yield",
    "throttle_yield",
    "released",
];

fn angle(a: f64) -> f64 {
    (a + 540.0).rem_euclid(360.0) - 180.0
}

impl Scorer {
    pub fn new() -> Self {
        Self::default()
    }

    /// One record in, the conclusions it completes out (`stage_result`,
    /// `sortie_result`). Never fed its own output.
    pub fn feed(&mut self, r: &Value) -> Vec<(String, Value)> {
        let mut out = Vec::new();
        let t = f(r, "t").unwrap_or(0.0);
        self.sortie = r.get("sortie").and_then(Value::as_str).map(str::to_string);
        // A stream that went quiet has ended, whatever record says so.
        if self.taxi.as_ref().is_some_and(|x| t - x.t_last > GAP_S) {
            self.close_taxi("gap", &mut out);
        }
        if self.landing.as_ref().is_some_and(|x| t - x.t_last > GAP_S) {
            self.close_landing(&mut out);
        }
        let (kind, event) = (s(r, "kind"), s(r, "event"));
        let crew_brake = matches!(
            (kind, event),
            ("autotakeoff", "reject" | "reject_rollout" | "reject_stopped")
                | ("autotaxi", "hold_gone")
        ) || (kind == "state"
            && r.pointer("/in/crew_brake").and_then(Value::as_bool) == Some(true));
        if crew_brake {
            self.crew_brake_t = Some(t);
        }
        match (kind, event) {
            // The backtrack's turn is the line-up's second leg.
            ("autotaxi", "line_up_path" | "backtrack_turn") => self.line_up_next = true,
            ("autotaxi", "drive") => self.taxi_tick(r, t),
            ("autotaxi", "holding_point") => self.close_taxi(event, &mut out),
            ("autotaxi", e) if TAXI_FAILURES.contains(&e) => {
                // A drive replaced by the next one was not stopped short.
                let superseded = r.get("superseded").and_then(Value::as_bool) == Some(true);
                self.close_taxi_as(e, !superseded, &mut out)
            }
            // A new drive is a new stream, whatever the old one was doing.
            ("autotaxi", "engaged") => self.close_taxi("reengaged", &mut out),
            ("autotakeoff", "roll") => {
                if self.rotated {
                    self.crew_yoke_t = Some(t);
                }
                self.roll_tick(r, t)
            }
            ("autotakeoff", "rotate") => self.rotated = true,
            ("autotakeoff", "climb_manual") => {
                if r.get("pitch").is_some_and(|v| !v.is_null()) {
                    self.crew_yoke_t = Some(t);
                }
                self.close_roll(None, &mut out)
            }
            // The roll ends in the air: liftoff (or the climb, for a stream
            // that never logged one) is the pass. Only a roll that was STOPPED
            // is a failure; every other milestone (rotate, trim, flap steps)
            // is the roll going as it should. Any unknown event used to close
            // it as failed, so "rotate" failed every takeoff and the roll
            // ticks after it opened a second one (LIPK 2026-09-26).
            ("autotakeoff", "liftoff" | "climb") => self.close_roll(None, &mut out),
            ("autotakeoff", e) if self.roll.is_some() && ROLL_FAILURES.contains(&e) => {
                self.close_roll(Some(e), &mut out)
            }
            // A join onto the route is not "off the line": the lateral
            // offset is judged once the drive has joined (LIPK 2026-09-26:
            // a 69 m join from the push scored the taxi as failed).
            ("autotaxi", "join" | "joined") => {
                let c = if event == "join" { Capture::Joining } else { Capture::Closing(t) };
                match self.taxi.as_mut() {
                    Some(x) => x.capture = c,
                    None => self.capture_pending = Some(c),
                }
            }
            ("autoland", "rollout") => {
                if let Some(l) = self.landing.as_mut() {
                    if let Some(x) = f(r, "xtk_m") {
                        l.max_xtk = Some(l.max_xtk.unwrap_or(0.0).max(x.abs()));
                    }
                    l.t_last = t;
                }
            }
            ("autoland", "") => {
                // The arrest and the flare write the elevator and aileron
                // channels.
                if ["flare_elev", "flare_ail"]
                    .iter()
                    .any(|k| r.get(*k).is_some_and(|v| !v.is_null()))
                {
                    self.crew_yoke_t = Some(t);
                }
                self.contact_tick(r, t);
            }
            ("state", _) => self.state_tick(r, t, &mut out),
            // The circuit gave the aircraft back as no pattern (#160): far
            // above its height, a leg without end, the servos out.
            ("circuit", "pattern_guard") => {
                let why = match s(r, "why") {
                    "above_pattern" => format!(
                        "{:.0} ft above the pattern on the {}",
                        f(r, "above_ft").unwrap_or(0.0),
                        s(r, "leg").to_lowercase()
                    ),
                    "leg_overdue" => format!(
                        "{} for {:.0} s against {:.0}",
                        s(r, "leg").to_lowercase(),
                        f(r, "secs").unwrap_or(0.0),
                        f(r, "limit_s").unwrap_or(0.0)
                    ),
                    "servos_off" => "the autopilot's servos went out".to_string(),
                    // #181: the climb watch took it.
                    "climb_not_held" => format!(
                        "the autopilot did not hold the climb ({}, {:.0} kt, {:.0} fpm)",
                        s(r, "watch"),
                        f(r, "ias").unwrap_or(0.0),
                        f(r, "vs").unwrap_or(0.0)
                    ),
                    other => format!("gave up: {other}"),
                };
                let sortie = self.sortie.clone();
                self.push(
                    "circuit",
                    sortie,
                    vec![why],
                    json!({ "leg": s(r, "leg"), "alt_msl": f(r, "alt_msl"),
                            "pattern_ft": f(r, "pattern_ft") }),
                    &mut out,
                );
            }
            ("full_auto", _) if s(r, "stage") == "give_up" => {
                if self.roll.is_some() {
                    self.close_roll(Some("give_up"), &mut out);
                }
                self.close_taxi("give_up", &mut out);
            }
            // A yoke read back while our own channel holds it is the loop's
            // output, not a hand: the live monitor no longer records these
            // (`monitor::interventions_now`), and older sorties are re-judged
            // here (1790441293: four, all the takeoff's and the flare's pull).
            ("intervention", _)
                if s(r, "what") == "yoke_under_ap"
                    && self.crew_yoke_t.is_some_and(|c| t - c <= CREW_YOKE_FRESH_S) =>
            {
                self.crew_yoke_discounted += 1
            }
            // The plugin's brake latch rising on our own brake command (a
            // reject's stop, EGKK 1790524628): the live monitor no longer
            // records these (`monitor::CrewBrakeLatch`); older sorties are
            // re-judged here.
            ("intervention", _)
                if s(r, "what") == "brake_yield"
                    && self.crew_brake_t.is_some_and(|c| t - c <= CREW_BRAKE_FRESH_S) =>
            {
                self.crew_brake_discounted += 1
            }
            ("intervention", _) => self.interventions += 1,
            ("tool", _) => self.commands += 1,
            ("flight", _)
                if s(r, "ev") == "impact"
                    || (s(r, "ev") == "abnormal" && s(r, "to") == "Crashed") =>
            {
                self.crashed = true
            }
            // A reposition (the flight model saw the aircraft jump: a new
            // flight, a setup, a reload) or another airframe is not a
            // landing: the air track before it belonged to another flight.
            // The runner's swap from the SR22 still on final to 30 at
            // 1,270 ft to the SF50 on 13's threshold (1790607309.3,
            // 2026-09-28) scored "touched down at 549 fpm", the SR22's
            // approach sink met by the SF50's wheels on the runway (#158).
            ("flight", _) if s(r, "ev") == "reposition" => self.new_flight(&mut out),
            ("flight_meta", _) => self.new_flight(&mut out),
            ("flight", _) if s(r, "ev") == "touchdown" => {
                self.flight_td = f(r, "sink").map(|sink| (t, sink.min(0.0).abs()))
            }
            ("rapport", _) if r.get("impact").and_then(Value::as_bool) == Some(true) => {
                self.crashed = true
            }
            ("sortie", "begin") => {
                self.results.clear();
                self.crashed = false;
                self.interventions = 0;
                self.crew_yoke_discounted = 0;
                self.crew_brake_discounted = 0;
                self.crew_brake_t = None;
                self.commands = 0;
                self.flight_td = None;
            }
            ("sortie", "end") => {
                self.close_taxi("sortie_end", &mut out);
                self.close_landing(&mut out);
                let failed: Vec<&str> =
                    self.results.iter().filter(|(_, ok)| !ok).map(|(n, _)| n.as_str()).collect();
                // Unattended means nobody's hands AND nobody's words acted
                // on the aircraft: a pilot who said "set power 100%" mid
                // mission (1790366657) flew part of it, and the flag said
                // unattended because only yield latches were counted.
                let unattended = self.interventions == 0 && self.commands == 0;
                // A sortie that scored nothing proved nothing: 1790584266
                // flew a 737 into the ground 8.8 nm short of LEMG 12, the
                // runner's "my controls" ended the sortie 0.1 s before the
                // impact record, and it was written `pass: true, stages: 0`.
                // A mission that ends in the air never finished: the SF50
                // circuit 1790607325 climbed away from its downwind to
                // 21,900 ft, the deadline ended it there, and it was written
                // PASS on its alignment and takeoff (#160). Every mission
                // this scores (full auto, the approach, the helicopter route)
                // ends on the ground.
                let ended_airborne = self.airborne_since.is_some();
                let pass = failed.is_empty()
                    && unattended
                    && s(r, "outcome") != "gave_up"
                    && !self.crashed
                    && !ended_airborne
                    && !self.results.is_empty();
                let mut v = json!({
                    "outcome": s(r, "outcome"),
                    "stages": self.results.len(),
                    "failed": failed,
                    "interventions": self.interventions,
                    "crew_yoke_discounted": self.crew_yoke_discounted,
                    "crew_brake_discounted": self.crew_brake_discounted,
                    "commands": self.commands,
                    "unattended": unattended,
                    "crashed": self.crashed,
                    "ended_airborne": ended_airborne,
                    "pass": pass,
                });
                if let (Some(o), Some(id)) = (v.as_object_mut(), self.sortie.clone()) {
                    o.insert("sortie".into(), id.into());
                }
                out.push(("sortie_result".into(), v));
            }
            _ => {}
        }
        out
    }

    /// A new flight: whatever was airborne before it never lands. A landing
    /// already open (its wheels met the runway before the jump) is judged.
    fn new_flight(&mut self, out: &mut Vec<(String, Value)>) {
        self.close_landing(out);
        self.airborne_since = None;
        self.air.clear();
        self.contact = None;
        self.flare_vvi.clear();
        self.flight_td = None;
    }

    fn taxi_tick(&mut self, r: &Value, t: f64) {
        let line_up = std::mem::take(&mut self.line_up_next);
        let capture = self.capture_pending.take().unwrap_or_default();
        let sortie = self.sortie.clone();
        let x = self.taxi.get_or_insert_with(|| Taxi {
            t0: t,
            capture,
            sortie,
            ..Taxi::default()
        });
        x.line_up |= line_up;
        x.ticks += 1;
        x.t_last = t;
        let (gs, lat) = (f(r, "gs").unwrap_or(0.0), f(r, "lat_m").unwrap_or(0.0));
        let (bl, br, yaw) = (
            f(r, "bl").unwrap_or(0.0),
            f(r, "br").unwrap_or(0.0),
            f(r, "yaw_dps").unwrap_or(0.0),
        );
        x.max_gs = x.max_gs.max(gs);
        let turn = f(r, "turn_dps").unwrap_or(0.0);
        let asked = (turn.abs() >= CORNER_TURN_DPS).then(|| turn.signum());
        let cutting = x
            .line
            .corner(t, gs, lat, f(r, "remaining_m"), asked, yaw);
        let past_end = f(r, "remaining_m").is_some_and(|m| m < TAXI_PAST_END_M);
        if past_end {
            x.past_end_ticks += 1;
        } else {
            if let Capture::Closing(tj) = x.capture {
                if lat.abs() <= TAXI_MAX_LAT_M {
                    x.capture = Capture::Tracking;
                    x.capture_secs = Some(t - tj);
                }
            }
            if x.capture == Capture::Tracking {
                x.line.judge(lat, cutting);
            } else {
                x.closest_uncaptured =
                    Some(x.closest_uncaptured.map_or(lat.abs(), |c: f64| c.min(lat.abs())));
            }
        }
        x.last_err = f(r, "err").unwrap_or(0.0);
        x.last_lat = lat;
        if let Some((t1, bl1, br1, yaw1)) = x.last {
            let dt = t - t1;
            // A stop is allowed to squeeze; judged only while still moving.
            if dt > 0.05 && gs > 2.0 {
                x.brake_rate = x.brake_rate.max(((bl - bl1).abs().max((br - br1).abs())) / dt);
            }
            let (d1, d) = (bl1 - br1, bl - br);
            if d1.abs() > 0.05 && d.abs() > 0.05 && d1.signum() != d.signum() {
                x.brake_reversals += 1;
            }
            if yaw1.abs() > 2.0 && yaw.abs() > 2.0 && yaw1.signum() != yaw.signum() {
                x.yaw_swings += 1;
            }
        }
        x.last = Some((t, bl, br, yaw));
    }

    fn close_taxi(&mut self, by: &str, out: &mut Vec<(String, Value)>) {
        // Only an arrival passes: a give-up or a sortie that ends while a
        // drive is still going is a drive that did not arrive.
        let stopped = matches!(by, "give_up" | "sortie_end");
        self.close_taxi_as(by, stopped, out)
    }

    /// Close the drive, `stopped` when it ended short of its end point.
    fn close_taxi_as(&mut self, by: &str, stopped: bool, out: &mut Vec<(String, Value)>) {
        self.capture_pending = None;
        let Some(mut x) = self.taxi.take() else { return };
        if x.ticks < 5 {
            return;
        }
        x.line.flush();
        x.ended_by = by.to_string();
        let mut fails = Vec::new();
        if stopped {
            fails.push(format!("ended by {by}"));
        }
        match x.capture {
            Capture::Tracking => {}
            Capture::Joining => fails.push(format!(
                "never joined the line (closest {:.1} m)",
                x.closest_uncaptured.unwrap_or(0.0)
            )),
            Capture::Closing(_) => fails.push(format!(
                "never within {TAXI_MAX_LAT_M:.0} m of the line (closest {:.1} m)",
                x.closest_uncaptured.unwrap_or(0.0)
            )),
        }
        if let Some(c) = x.capture_secs.filter(|c| *c > TAXI_CAPTURE_MAX_S) {
            fails.push(format!("{c:.0} s to reach the line after the join"));
        }
        if x.line.max_lat > TAXI_MAX_LAT_M {
            fails.push(format!("{:.1} m off the line", x.line.max_lat));
        }
        if x.line.max_cut > TAXI_MAX_CUT_M {
            fails.push(format!("{:.1} m inside a corner", x.line.max_cut));
        }
        if x.brake_rate > TAXI_MAX_BRAKE_RATE {
            fails.push(format!("brake moved {:.1}/s", x.brake_rate));
        }
        let stage = if x.line_up { "line_up" } else { "taxi" };
        let sortie = x.sortie.clone();
        self.push(stage, sortie, fails, json!({
            "secs": r1(x.t_last - x.t0), "max_gs": r1(x.max_gs), "max_lat_m": r1(x.line.max_lat),
            "max_cut_m": r1(x.line.max_cut),
            "brake_rate": r2(x.brake_rate), "brake_reversals": x.brake_reversals,
            "yaw_swings": x.yaw_swings, "end_err_deg": r1(x.last_err),
            "end_lat_m": r1(x.last_lat), "ended_by": x.ended_by,
            "capture_secs": x.capture_secs.map(r1), "past_end_ticks": x.past_end_ticks,
        }), out);
    }

    fn roll_tick(&mut self, r: &Value, t: f64) {
        let sortie = self.sortie.clone();
        let x = self.roll.get_or_insert_with(|| Roll { t0: t, sortie, ..Roll::default() });
        x.ticks += 1;
        x.t_last = t;
        let (err, xtk, gs) = (f(r, "err"), f(r, "xtk_m"), f(r, "gs").unwrap_or(0.0));
        if x.start_err.is_none() {
            x.start_err = err;
            x.start_xtk = xtk;
        }
        if let Some(v) = xtk {
            x.max_xtk = x.max_xtk.max(v.abs());
            x.last_xtk = v;
        }
        if gs > 20.0 {
            x.max_err_fast = x.max_err_fast.max(err.unwrap_or(0.0).abs());
        }
        x.max_brake = x.max_brake.max(f(r, "bl").unwrap_or(0.0)).max(f(r, "br").unwrap_or(0.0));
        x.last_ias = f(r, "ias").unwrap_or(0.0);
    }

    fn close_roll(&mut self, ended: Option<&str>, out: &mut Vec<(String, Value)>) {
        self.rotated = false;
        let Some(x) = self.roll.take() else { return };
        let (e0, x0) = (x.start_err.unwrap_or(0.0).abs(), x.start_xtk.unwrap_or(0.0).abs());
        // The line-up is judged where it hands over: the roll's first tick.
        let mut lu = Vec::new();
        if e0 > LINEUP_MAX_ERR_DEG {
            lu.push(format!("{e0:.0} degrees off the runway"));
        }
        if x0 > LINEUP_MAX_XTK_M {
            lu.push(format!("{x0:.1} m off the centreline"));
        }
        self.push("alignment", x.sortie.clone(), lu, json!({ "err_deg": r1(e0), "xtk_m": r1(x0) }), out);
        let mut fails = Vec::new();
        if let Some(e) = ended {
            fails.push(format!("ended by {e}"));
        }
        if x.max_xtk > ROLL_MAX_XTK_M {
            fails.push(format!("{:.1} m off the centreline", x.max_xtk));
        }
        self.push("takeoff", x.sortie.clone(), fails, json!({
            "ticks": x.ticks, "max_xtk_m": r1(x.max_xtk), "liftoff_xtk_m": r1(x.last_xtk),
            "max_err_fast_deg": r1(x.max_err_fast), "max_brake": r2(x.max_brake),
            "last_ias": r1(x.last_ias), "ended_by": ended.unwrap_or("climb"),
            "secs": r1(x.t_last - x.t0),
        }), out);
    }

    /// One autoland guidance tick: the first contact and any bounce after it.
    fn contact_tick(&mut self, r: &Value, t: f64) {
        let (Some(radio), Some(vvi)) = (f(r, "radio"), f(r, "vvi")) else { return };
        let window = crate::instructor::TOUCHDOWN_WINDOW_S;
        self.flare_vvi.push((t, vvi));
        self.flare_vvi.retain(|(tv, _)| t - tv <= window);
        if self.contact.is_some_and(|c| t - c.t > CONTACT_WITHIN_S) {
            self.contact = None;
        }
        let wheels = match r.get("ground").and_then(Value::as_bool) {
            Some(g) => g,
            None => radio <= CONTACT_RADIO_FT,
        };
        match self.contact.as_mut() {
            None if wheels => {
                let recent: Vec<(f64, f64)> = self.flare_vvi.clone();
                self.contact = Some(Contact {
                    t,
                    sink: crate::instructor::touchdown_sink_fpm(&recent, t, vvi).abs(),
                    up_ft: 0.0,
                })
            }
            Some(c) => c.up_ft = c.up_ft.max(radio),
            None => {}
        }
    }

    fn state_tick(&mut self, r: &Value, t: f64, out: &mut Vec<(String, Value)>) {
        let ground = r.get("ground").and_then(Value::as_bool).unwrap_or(true);
        let crashed = r.get("crashed").and_then(Value::as_bool).unwrap_or(false);
        if crashed {
            self.crashed = true;
        }
        if !ground {
            self.airborne_since.get_or_insert(t);
            self.air.push((
                t,
                f(r, "vsi").unwrap_or(0.0),
                f(r, "hdg").unwrap_or(0.0),
                f(r, "track"),
            ));
            self.air.retain(|(ta, ..)| t - ta <= 3.0);
            return;
        }
        let Some(since) = self.airborne_since.take() else { return };
        // A hop is not a flight: 30 s airborne before a touchdown counts.
        if t - since < 30.0 {
            self.air.clear();
            return;
        }
        // The state samples' own sink, the flight model's window: it ENDS at
        // this ground sample and is truly TOUCHDOWN_WINDOW_S long (the 3 s
        // kept behind the last AIRBORNE sample reached ~4.4 s back and
        // scored the pre-flare descent, 1790585651: 461 fpm on a 130).
        let recent: Vec<(f64, f64)> = self.air.iter().map(|(ta, v, ..)| (*ta, *v)).collect();
        let sink = crate::instructor::touchdown_sink_fpm(&recent, t, f(r, "vsi").unwrap_or(0.0))
            .min(0.0)
            .abs();
        let model = self
            .flight_td
            .take()
            .filter(|(tf, _)| *tf >= since && (t - tf).abs() <= FLIGHT_TD_WITHIN_S)
            .map(|(_, s)| s);
        let drift = self
            .air
            .last()
            .and_then(|(_, _, h, tr)| tr.map(|tr| angle(h - tr).abs()))
            .unwrap_or(0.0);
        self.air.clear();
        self.close_landing(out);
        // The first contact, when the autoland ticks saw one before this
        // sample, is the touchdown; this sample is then the second contact.
        let first = self.contact.take().filter(|c| t - c.t <= CONTACT_WITHIN_S && c.t >= since);
        // The first contact's sink is the flight model's touchdown when it
        // logged one; else the autoland window ending AT the contact; else
        // the state samples'. After a bounce the model's touchdown (its
        // first ground SAMPLE) is the second contact, so the first stays the
        // autoland window's. Never the max of two windows (#142: that max
        // was the stretched state window's 461 fpm on 1790585651).
        let (t_td, sink, bounce) = match first {
            Some(c) if c.up_ft > BOUNCE_FT => {
                (c.t, c.sink, Some((c.up_ft, model.unwrap_or(sink))))
            }
            Some(c) => (c.t, model.unwrap_or(c.sink), None),
            None => (t, model.unwrap_or(sink), None),
        };
        self.flare_vvi.clear();
        self.landing = Some(Landing {
            t_td,
            sink,
            drift,
            max_xtk: None,
            t_last: t,
            sortie: self.sortie.clone(),
            bounce,
        });
        if crashed {
            self.close_landing(out);
        }
    }

    fn close_landing(&mut self, out: &mut Vec<(String, Value)>) {
        let Some(l) = self.landing.take() else { return };
        let mut fails = Vec::new();
        if l.sink > TOUCHDOWN_MAX_SINK_FPM {
            fails.push(format!("touched down at {:.0} fpm", l.sink));
        }
        if let Some((up, _)) = l.bounce {
            fails.push(format!("bounced to {up:.0} ft"));
        }
        if l.drift > TOUCHDOWN_MAX_DRIFT_DEG {
            fails.push(format!("{:.0} degrees of drift", l.drift));
        }
        if l.max_xtk.is_some_and(|x| x > ROLLOUT_MAX_XTK_M) {
            fails.push(format!("{:.1} m off the centreline rolling out", l.max_xtk.unwrap_or(0.0)));
        }
        self.push("landing", l.sortie.clone(), fails, json!({
            "sink_fpm": l.sink.round(), "drift_deg": r1(l.drift),
            "rollout_max_xtk_m": l.max_xtk.map(r1), "t_touchdown": l.t_td,
            "bounce_ft": l.bounce.map(|(up, _)| r1(up)),
            "second_contact_sink_fpm": l.bounce.map(|(_, s2)| s2.round()),
        }), out);
    }

    /// One stage's verdict, tagged with the sortie it was flown in (none:
    /// a stand-alone tool, and the key is left out).
    fn push(
        &mut self,
        stage: &str,
        sortie: Option<String>,
        fails: Vec<String>,
        mut metrics: Value,
        out: &mut Vec<(String, Value)>,
    ) {
        let ok = fails.is_empty();
        if let Some(o) = metrics.as_object_mut() {
            o.insert("stage".into(), stage.into());
            o.insert("pass".into(), ok.into());
            o.insert("why".into(), fails.into());
            if let Some(id) = sortie {
                o.insert("sortie".into(), id.into());
            }
        }
        self.results.push((stage.to_string(), ok));
        out.push(("stage_result".into(), metrics));
    }
}

static LIVE: std::sync::Mutex<Option<Scorer>> = std::sync::Mutex::new(None);

/// The blackbox tap: every record through the live scorer.
pub fn tap(r: &Value) -> Vec<(String, Value)> {
    let mut g = LIVE.lock().unwrap_or_else(|e| e.into_inner());
    g.get_or_insert_with(Scorer::new).feed(r)
}

/// Score a recorded stream: every conclusion with its time, in order.
pub fn score_all<'a>(records: impl Iterator<Item = &'a Value>) -> Vec<Value> {
    let mut sc = Scorer::new();
    let mut out = Vec::new();
    for r in records {
        // Recorded conclusions are re-derived, never trusted twice.
        if matches!(s(r, "kind"), "stage_result" | "sortie_result") {
            continue;
        }
        for (k, mut v) in sc.feed(r) {
            if let Some(o) = v.as_object_mut() {
                o.insert("kind".into(), k.into());
                o.insert("t".into(), r.get("t").cloned().unwrap_or(Value::Null));
            }
            out.push(v);
        }
    }
    out
}

/// The Scorecard block: conclusions grouped by the mission they were flown
/// in, in order, each group headed. A stage outside any mission (a spoken
/// "land the aircraft", a stand-alone taxi) is labelled as such: they used
/// to sit between the missions' verdicts with nothing to tell a mission's
/// landing from a stand-alone autoland.
pub fn block(card: &[Value]) -> String {
    let mut out = String::new();
    let mut group: Option<Option<&str>> = None;
    for v in card {
        let key = v.get("sortie").and_then(Value::as_str);
        if group != Some(key) {
            match key {
                Some(id) => out.push_str(&format!("  Mission {id}:\n")),
                None => out.push_str("  Outside a mission (stand-alone tools):\n"),
            }
            group = Some(key);
        }
        out.push_str(&format!("    {}\n", line(v)));
    }
    out
}

/// One line per conclusion for the debrief.
pub fn line(v: &Value) -> String {
    let why: Vec<&str> = v
        .get("why")
        .and_then(Value::as_array)
        .map(|a| a.iter().filter_map(Value::as_str).collect())
        .unwrap_or_default();
    if s(v, "kind") == "sortie_result" {
        // The stage NAMES, never the JSON value's debug form (it printed
        // `failed Array [String("taxi")]`).
        let failed: Vec<&str> = v
            .get("failed")
            .and_then(Value::as_array)
            .map(|a| a.iter().filter_map(Value::as_str).collect())
            .unwrap_or_default();
        let n = |k: &str| v.get(k).and_then(Value::as_u64).unwrap_or(0);
        let discounted: String = [
            ("crew_yoke_discounted", "crew yoke"),
            ("crew_brake_discounted", "crew brake"),
        ]
        .iter()
        .filter_map(|(k, what)| match n(k) {
            0 => None,
            d => Some(format!(" ({d} {what} not counted)")),
        })
        .collect();
        return format!(
            "SORTIE {}: {} stages, {}, {} interventions{}, {} commands, {}, outcome {}{}{}",
            if v.get("pass").and_then(Value::as_bool) == Some(true) { "PASS" } else { "FAIL" },
            n("stages"),
            if failed.is_empty() {
                "none failed".to_string()
            } else {
                format!("failed {}", failed.join(", "))
            },
            n("interventions"),
            discounted,
            n("commands"),
            if v.get("unattended").and_then(Value::as_bool) == Some(true) {
                "unattended"
            } else {
                "attended"
            },
            s(v, "outcome"),
            if v.get("crashed").and_then(Value::as_bool) == Some(true) { ", crashed" } else { "" },
            if v.get("ended_airborne").and_then(Value::as_bool) == Some(true) {
                ", ended airborne"
            } else {
                ""
            },
        );
    }
    format!(
        "{:<9} {}{}",
        s(v, "stage"),
        if v.get("pass").and_then(Value::as_bool) == Some(true) { "pass" } else { "FAIL" },
        if why.is_empty() { String::new() } else { format!(": {}", why.join(", ")) }
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn drive(t: f64, gs: f64, lat: f64, bl: f64, br: f64, yaw: f64, err: f64) -> Value {
        json!({"t": t, "kind": "autotaxi", "event": "drive", "gs": gs, "lat_m": lat,
               "bl": bl, "br": br, "yaw_dps": yaw, "err": err})
    }

    fn roll(t: f64, gs: f64, err: f64, xtk: f64) -> Value {
        json!({"t": t, "kind": "autotakeoff", "event": "roll", "gs": gs, "err": err,
               "xtk_m": xtk, "bl": 0.0, "br": 0.0, "ias": gs})
    }

    /// LIME 28, 2026-09-25: lined up 21 degrees off, the roll swung further,
    /// the pilot's pedals stood it down. Every piece of that is a FAIL.
    #[test]
    fn the_lime_departure_fails_where_it_failed() {
        let mut recs = vec![json!({"t": 0.0, "kind": "sortie", "event": "begin"})];
        recs.push(json!({"t": 1.0, "kind": "autotaxi", "event": "line_up_path"}));
        for i in 0..20 {
            let t = 2.0 + i as f64 * 0.25;
            recs.push(drive(t, 6.0, -0.1 * i as f64, 0.0, 0.01, -2.0 - 0.1 * i as f64, i as f64));
        }
        recs.push(json!({"t": 8.0, "kind": "autotaxi", "event": "holding_point"}));
        for i in 0..16 {
            recs.push(roll(15.0 + i as f64 * 0.25, 1.0 + i as f64 * 0.3, 21.0 + 0.5 * i as f64, -1.4));
        }
        recs.push(json!({"t": 19.5, "kind": "autotakeoff", "event": "pedal_yield"}));
        recs.push(json!({"t": 19.6, "kind": "intervention", "what": "pedal_yield"}));
        recs.push(json!({"t": 20.0, "kind": "full_auto", "stage": "give_up"}));
        recs.push(json!({"t": 21.0, "kind": "sortie", "event": "end", "outcome": "gave_up"}));
        let out = score_all(recs.iter());
        let lines: Vec<String> = out.iter().map(line).collect();
        println!("{}", lines.join("\n"));
        let stage = |n: &str| out.iter().find(|v| s(v, "stage") == n).cloned().unwrap();
        assert_eq!(stage("line_up")["pass"], true, "the drive itself stayed on its line");
        assert_eq!(stage("alignment")["pass"], false);
        assert_eq!(stage("takeoff")["pass"], false);
        assert_eq!(stage("takeoff")["ended_by"], "pedal_yield");
        let sortie = out.last().unwrap();
        assert_eq!(sortie["pass"], false);
        assert_eq!(sortie["interventions"], 1);
    }

    #[test]
    fn jerky_brakes_and_a_wander_fail_the_taxi() {
        let mut recs = Vec::new();
        for i in 0..20 {
            let b = if i % 2 == 0 { 0.0 } else { 0.6 };
            recs.push(drive(i as f64 * 0.2, 8.0, if i == 10 { 4.0 } else { 0.5 }, b, 0.0, 0.0, 0.0));
        }
        recs.push(json!({"t": 10.0, "kind": "autotaxi", "event": "holding_point"}));
        let out = score_all(recs.iter());
        assert_eq!(out[0]["stage"], "taxi");
        assert_eq!(out[0]["pass"], false);
        assert_eq!(out[0]["why"].as_array().unwrap().len(), 2, "{}", out[0]);
    }

    #[test]
    fn a_touchdown_is_judged_on_its_sink_and_drift() {
        let mut recs = Vec::new();
        for i in 0..40 {
            recs.push(json!({"t": i as f64, "kind": "state", "ground": false,
                "vsi": if i > 37 { -650.0 } else { -500.0 }, "hdg": 90.0, "track": 88.0}));
        }
        recs.push(json!({"t": 40.0, "kind": "state", "ground": true, "vsi": -100.0, "hdg": 90.0, "track": 90.0}));
        recs.push(json!({"t": 41.0, "kind": "autoland", "event": "rollout", "xtk_m": 1.2}));
        recs.push(json!({"t": 50.0, "kind": "state", "ground": true, "vsi": 0.0, "hdg": 90.0, "track": 90.0}));
        let out = score_all(recs.iter());
        let l = out.iter().find(|v| v["stage"] == "landing").unwrap();
        assert_eq!(l["pass"], false, "{l}");
        assert_eq!(l["sink_fpm"], 650.0);
        assert_eq!(l["rollout_max_xtk_m"], 1.2);
    }

    /// The 737's LEMG sortie (1790441293), trimmed to 518 of its 3,871
    /// records (every tenth taxi tick and every one far off the line, every
    /// third roll tick, the touchdown's state and flare ticks whole, the crew's yoke
    /// writes before each intervention). Pinned so any change to a verdict is
    /// a changed line here; `xvl debrief` prints the same block from the same
    /// file (asserted below), so the two agree by construction.
    const LEMG_737: &str = include_str!("../bench/sorties/1790441293-full_auto.trimmed.jsonl");

    fn records(text: &str) -> Vec<Value> {
        text.lines().filter_map(|l| serde_json::from_str(l).ok()).collect()
    }

    /// The 737's two LEMG 31 landings of 2026-09-28 (1790584715, 1790585651),
    /// trimmed to their landings: every `state` from 45 s before the flight
    /// model's touchdown to 40 s after, the autoland guidance and rollout
    /// ticks from 5 s before the first contact, the `flight` touchdown and
    /// the sortie's begin and end. Live the stage scored 397 and 461 fpm
    /// (the second a FAIL): the state samples' window was kept 3 s behind
    /// the last AIRBORNE sample, ~4.4 s before the ground one, and reached
    /// the pre-flare descent (-461 at 1790586565.1, 2.8 s before contact).
    /// The contact's vvi was -180 and -166; the flight model said 105 and
    /// 130 (#142).
    const LEMG_31_LANDINGS: [(&str, f64, f64, f64); 2] = [
        (
            include_str!("../bench/sorties/1790584715-landing.trimmed.jsonl"),
            105.0,
            5.9,
            1790585602.0,
        ),
        (
            include_str!("../bench/sorties/1790585651-landing.trimmed.jsonl"),
            130.0,
            9.7,
            1790586567.9,
        ),
    ];

    #[test]
    fn the_landing_sink_is_the_first_contacts_not_the_descent_before_it() {
        for (text, model_sink, xtk, contact_t) in LEMG_31_LANDINGS {
            let recs = records(text);
            let landing = |recs: &[Value]| {
                score_all(recs.iter()).into_iter().find(|v| s(v, "stage") == "landing").unwrap()
            };
            let l = landing(&recs);
            println!("{}", line(&l));
            assert_eq!(l["sink_fpm"], model_sink, "the flight model's touchdown: {l}");
            assert_eq!(l["t_touchdown"], contact_t, "the first contact's time: {l}");
            assert_eq!(l["rollout_max_xtk_m"], xtk, "{l}");
            let why: Vec<&str> =
                l["why"].as_array().unwrap().iter().filter_map(Value::as_str).collect();
            assert_eq!(why, vec![format!("{xtk:.1} m off the centreline rolling out")], "{l}");
            // With no flight model record (older logs), the autoland window
            // that ENDS at the contact: never the descent seconds before it.
            let no_model: Vec<Value> =
                recs.iter().filter(|r| s(r, "kind") != "flight").cloned().collect();
            let l = landing(&no_model);
            println!("without the model: {} {}", line(&l), l["sink_fpm"]);
            let sink = l["sink_fpm"].as_f64().unwrap();
            assert!(sink < TOUCHDOWN_MAX_SINK_FPM && sink >= 166.0, "{l}");
            // And with neither the model nor autoland ticks, the state
            // samples' own window, TOUCHDOWN_WINDOW_S before the ground one.
            let states: Vec<Value> = recs
                .iter()
                .filter(|r| matches!(s(r, "kind"), "state" | "sortie"))
                .cloned()
                .collect();
            let l = landing(&states);
            println!("state only: {} {}", line(&l), l["sink_fpm"]);
            assert!(l["sink_fpm"].as_f64().unwrap() < TOUCHDOWN_MAX_SINK_FPM, "{l}");
        }
    }

    #[test]
    fn the_recorded_lemg_sortie_scores_as_pinned() {
        let recs = records(LEMG_737);
        let card = score_all(recs.iter());
        let text = block(&card);
        print!("{text}");
        assert_eq!(
            text,
            "  Mission 1790441293-full_auto:\n\
             \x20   taxi      FAIL: 5.8 m off the line\n\
             \x20   line_up   FAIL: 6.0 m off the line\n\
             \x20   alignment pass\n\
             \x20   takeoff   FAIL: 6.9 m off the centreline\n\
             \x20   landing   FAIL: touched down at 794 fpm, bounced to 12 ft, 12.0 m off the centreline rolling out\n\
             \x20   SORTIE FAIL: 5 stages, failed taxi, line_up, takeoff, landing, \
             0 interventions (4 crew yoke not counted), 0 commands, unattended, outcome ended\n"
        );
        let stage = |n: &str| card.iter().find(|v| s(v, "stage") == n).cloned().unwrap();
        // The join closed at 14.5 m; the line is judged from its first tick
        // inside 3 m (6.7 s in the whole file, the next kept tick here).
        assert_eq!(stage("taxi")["capture_secs"], 8.2);
        // The first contact at 17:04:34Z, not the 278 fpm of the second.
        let landing = stage("landing");
        assert_eq!(landing["sink_fpm"], 794.0);
        assert_eq!(landing["t_touchdown"], 1790442274.5);
        assert_eq!(landing["bounce_ft"], 11.8);
        assert_eq!(landing["second_contact_sink_fpm"], 278.0);
        let sortie = card.last().unwrap();
        assert_eq!(sortie["interventions"], 0, "the flare's own pull is not a hand");
        assert_eq!(sortie["crew_yoke_discounted"], 4);
        // The recorded (live, older) verdicts are never trusted twice.
        assert!(recs.iter().all(|r| s(r, "kind") != "stage_result"));
    }

    /// The 737's first EGKK 26L sortie (1790524628, 2026-09-27), trimmed to
    /// 678 of its 2,576 records (every tenth taxi tick and every one over
    /// 2.5 m off, every third roll tick, every reject and state record
    /// whole; api_load, taxi_guide, speedhold, duty, atc and the live
    /// verdicts dropped). The reject at 87 kt latched the plugin's
    /// `brake_yield` 0.5 s later on the crew's own brake command, logged as
    /// an intervention: the takeoff still fails, the sortie has no hand in it.
    const EGKK_737_REJECT: &str =
        include_str!("../bench/sorties/1790524628-full_auto.trimmed.jsonl");

    #[test]
    fn the_recorded_egkk_reject_scores_as_pinned() {
        let recs = records(EGKK_737_REJECT);
        let card = score_all(recs.iter());
        let text = block(&card);
        print!("{text}");
        assert_eq!(
            text,
            "  Mission 1790524628-full_auto:\n\
             \x20   taxi      pass\n\
             \x20   line_up   pass\n\
             \x20   alignment pass\n\
             \x20   takeoff   FAIL: ended by reject, 9.4 m off the centreline\n\
             \x20   SORTIE FAIL: 4 stages, failed takeoff, \
             0 interventions (1 crew brake not counted), 0 commands, unattended, outcome gave_up\n"
        );
        // The taxi's "4.2 m off the line" was the inside of a vertex (the
        // projection jumped 7 m along the route across it): a cut, in limit.
        let taxi = card.iter().find(|v| s(v, "stage") == "taxi").cloned().unwrap();
        assert_eq!(taxi["max_cut_m"], 4.2, "{taxi}");
        assert!(taxi["max_lat_m"].as_f64().unwrap() <= TAXI_MAX_LAT_M, "{taxi}");
        let sortie = card.last().unwrap();
        assert_eq!(sortie["interventions"], 0, "the reject's own brake is not a hand");
        assert_eq!(sortie["crew_brake_discounted"], 1);
        assert_eq!(sortie["unattended"], true);
        assert_eq!(sortie["pass"], false, "a reject is still a failed sortie");
        // The same latch with no crew brake near it is still a hand.
        let no_reject: Vec<Value> = recs
            .iter()
            .filter(|r| {
                !(s(r, "kind") == "autotakeoff"
                    && matches!(s(r, "event"), "reject" | "reject_rollout" | "reject_stopped"))
            })
            .cloned()
            .collect();
        let sortie = score_all(no_reject.iter()).last().cloned().unwrap();
        assert_eq!(sortie["interventions"], 1, "{sortie}");
        assert!(recs.iter().all(|r| s(r, "kind") != "stage_result"));
    }

    /// The 737's EGBB 33 sortie (1790534902, 2026-09-27), trimmed to 1,021
    /// of its 8,246 records (every tenth taxi tick and every one over 2 m
    /// off, turning or in a route turn; every autotakeoff record; every
    /// tenth state and all of the minute around the touchdown with its
    /// autoland ticks; speech, ATC, procedure, api_load and the live
    /// verdicts dropped). Live it scored taxi 3.2 m and line-up 4.2 m off
    /// the line: both the inside of a vertex, every straight held to 0.5 m.
    /// The takeoff is a real failure: the heading held within 0.3 degrees
    /// of its target while a 0.65 degree drift carried the wheels 5.7 m
    /// right by liftoff (`autotakeoff::DriftFilter`).
    const EGBB_737: &str = include_str!("../bench/sorties/1790534902-full_auto.trimmed.jsonl");

    #[test]
    fn the_recorded_egbb_sortie_scores_as_pinned() {
        let recs = records(EGBB_737);
        let card = score_all(recs.iter());
        let text = block(&card);
        print!("{text}");
        assert_eq!(
            text,
            "  Mission 1790534902-full_auto:\n\
             \x20   taxi      pass\n\
             \x20   line_up   pass\n\
             \x20   alignment pass\n\
             \x20   takeoff   FAIL: 5.7 m off the centreline\n\
             \x20   landing   pass\n\
             \x20   SORTIE FAIL: 5 stages, failed takeoff, \
             0 interventions, 0 commands, unattended, outcome ended\n"
        );
        let stage = |n: &str| card.iter().find(|v| s(v, "stage") == n).cloned().unwrap();
        let (taxi, line_up) = (stage("taxi"), stage("line_up"));
        println!("{taxi}\n{line_up}");
        assert_eq!(taxi["max_cut_m"], 3.2, "the 56 degree vertex onto Sierra");
        assert_eq!(line_up["max_cut_m"], 4.2, "the line-up's 92 degree vertex");
        // The rest: 0.5 m down the straights; the taxi's 2.0 m is the bend
        // onto Echo at 9.6 kt, whose projection moved too little to prove a
        // vertex, so it is judged as the line.
        assert_eq!(taxi["max_lat_m"], 2.0);
        assert_eq!(line_up["max_lat_m"], 0.8);
        assert_eq!(stage("takeoff")["liftoff_xtk_m"], 5.7);
        assert!(recs.iter().all(|r| s(r, "kind") != "stage_result"));
    }

    /// A route corner is a vertex no wheel can follow: inside it while
    /// turning through it is a cut, judged at [`TAXI_MAX_CUT_M`], and only
    /// where the projection proved a vertex by jumping across it. A drawn
    /// curve cut as deep, a swing outside the corner, or a cut past the
    /// limit all still fail.
    #[test]
    fn a_cut_inside_a_vertex_is_not_off_the_line() {
        // A left corner at 6.5 kt: the route asks the turn, the aircraft
        // yaws left and runs up to `peak` inside (left, negative); `jump`
        // metres of projection leap at the vertex.
        let corner = |peak: f64, jump: f64, side: f64| {
            let mut recs = Vec::new();
            let mut rem = 500.0;
            for i in 0..40 {
                let t = i as f64 * 0.2;
                let turning = (10..30).contains(&i);
                let lat = if turning {
                    let k = if i < 20 { (i - 10) as f64 / 10.0 } else { (30 - i) as f64 / 10.0 };
                    side * peak * k
                } else {
                    0.2
                };
                rem -= 0.67;
                if i == 20 {
                    rem -= jump;
                }
                let mut d = drive(t, 6.5, lat, 0.0, 0.0, if turning { -9.0 } else { 0.0 }, 0.0);
                d["turn_dps"] = json!(if (8..20).contains(&i) { -10.0 } else { 0.0 });
                d["remaining_m"] = json!(rem);
                recs.push(d);
            }
            recs.push(json!({"t": 9.0, "kind": "autotaxi", "event": "holding_point"}));
            score_all(recs.iter()).remove(0)
        };
        let cut = corner(3.2, 3.9, -1.0);
        assert_eq!(cut["pass"], true, "{cut}");
        assert_eq!(cut["max_cut_m"], 3.2);
        assert_eq!(cut["max_lat_m"], 0.2);
        let curve = corner(3.2, 0.0, -1.0);
        assert_eq!(curve["pass"], false, "no vertex, a drawn curve: {curve}");
        assert_eq!(curve["why"][0], "3.2 m off the line");
        let wide = corner(3.2, 3.9, 1.0);
        assert_eq!(wide["pass"], false, "outside the turn: {wide}");
        let deep = corner(6.0, 5.0, -1.0);
        assert_eq!(deep["pass"], false, "{deep}");
        assert_eq!(deep["why"][0], "6.0 m inside a corner");
    }

    #[test]
    fn the_debrief_prints_the_same_scorecard() {
        let recs = records(LEMG_737);
        let text = crate::debrief::report(LEMG_737.lines());
        assert!(text.contains(&block(&score_all(recs.iter()))), "{text}");
        assert!(!text.contains("Array"), "{text}");
    }

    /// A join is judged from the first tick ON the line, not from the
    /// `joined` record (15 m, 45 degrees); one that never gets there fails.
    #[test]
    fn a_join_is_judged_once_it_reaches_the_line() {
        let mut recs = vec![json!({"t": 0.0, "kind": "autotaxi", "event": "join"})];
        let lat = [30.0, 25.0, 20.0, 14.5, 12.0, 9.0, 6.0, 4.0, 2.9, 2.0, 1.5, 1.0];
        for (i, l) in lat.iter().enumerate() {
            let t = 1.0 + i as f64;
            if *l == 14.5 {
                recs.push(json!({"t": t - 0.01, "kind": "autotaxi", "event": "joined"}));
            }
            recs.push(drive(t, 6.0, *l, 0.0, 0.0, 0.0, 0.0));
        }
        recs.push(json!({"t": 20.0, "kind": "autotaxi", "event": "holding_point"}));
        let out = score_all(recs.iter());
        assert_eq!(out[0]["pass"], true, "{}", out[0]);
        assert_eq!(out[0]["max_lat_m"], 2.9);
        assert_eq!(out[0]["capture_secs"], 5.0);

        // Joined, and never within the limit: that is the failure.
        let mut recs = vec![json!({"t": 0.0, "kind": "autotaxi", "event": "join"})];
        recs.push(json!({"t": 0.5, "kind": "autotaxi", "event": "joined"}));
        for i in 0..10 {
            recs.push(drive(1.0 + i as f64, 6.0, 8.0, 0.0, 0.0, 0.0, 0.0));
        }
        recs.push(json!({"t": 20.0, "kind": "autotaxi", "event": "holding_point"}));
        let out = score_all(recs.iter());
        assert_eq!(out[0]["pass"], false);
        assert_eq!(out[0]["why"][0], "never within 3 m of the line (closest 8.0 m)");
    }

    /// Past the planned stop the projection measures the distance to the
    /// route's end point, overrun included (1790426786: 2.8 to 3.8 m at err
    /// 0, `remaining_m` -5): not a lateral sample.
    #[test]
    fn creeping_past_the_stop_is_not_off_the_line() {
        let mut recs = Vec::new();
        for i in 0..10 {
            let mut d = drive(i as f64 * 0.2, 8.0, 0.5, 0.0, 0.0, 0.0, 0.0);
            d["remaining_m"] = json!(100.0 - i as f64);
            recs.push(d);
        }
        for (i, l) in [2.8, 3.0, 3.4, 3.8].iter().enumerate() {
            let mut d = drive(2.0 + i as f64 * 0.2, 2.1, *l, 0.0, 0.0, 0.0, 0.0);
            d["remaining_m"] = json!(-5.0);
            recs.push(d);
        }
        recs.push(json!({"t": 3.0, "kind": "autotaxi", "event": "holding_point"}));
        let out = score_all(recs.iter());
        assert_eq!(out[0]["pass"], true, "{}", out[0]);
        assert_eq!(out[0]["past_end_ticks"], 4);
    }

    /// 1790408535: the drive stood `stagnant` 32 m off its route and the
    /// taxi scored a pass, judged on an offset it never reached.
    #[test]
    fn a_drive_that_stops_short_fails() {
        let mut recs = vec![json!({"t": 0.0, "kind": "autotaxi", "event": "join"})];
        for i in 0..20 {
            recs.push(drive(i as f64, 0.1, 32.6, 0.0, 0.0, 0.0, -79.0));
        }
        recs.push(json!({"t": 20.0, "kind": "autotaxi", "event": "stagnant"}));
        let out = score_all(recs.iter());
        assert_eq!(out[0]["pass"], false);
        assert_eq!(out[0]["ended_by"], "stagnant");
        assert_eq!(out[0]["why"][0], "ended by stagnant");
        // A drive superseded by the next one was not stopped short.
        let mut recs = Vec::new();
        for i in 0..10 {
            recs.push(drive(i as f64 * 0.2, 6.0, 0.5, 0.0, 0.0, 0.0, 0.0));
        }
        recs.push(json!({"t": 2.1, "kind": "autotaxi", "event": "stopped_from_outside",
                         "superseded": true}));
        assert_eq!(score_all(recs.iter())[0]["pass"], true);
    }

    /// Stages outside a mission are labelled and never mixed into the
    /// mission's verdict; a pilot's command makes a sortie attended.
    #[test]
    fn stages_group_by_mission_and_commands_attend_it() {
        let taxi = |t0: f64, sortie: Option<&str>| {
            let mut v: Vec<Value> = (0..10)
                .map(|i| drive(t0 + i as f64 * 0.2, 6.0, 0.5, 0.0, 0.0, 0.0, 0.0))
                .collect();
            v.push(json!({"t": t0 + 2.1, "kind": "autotaxi", "event": "holding_point"}));
            if let Some(id) = sortie {
                for r in &mut v {
                    r["sortie"] = json!(id);
                }
            }
            v
        };
        let mut recs = taxi(0.0, None);
        recs.push(json!({"t": 10.0, "kind": "sortie", "event": "begin", "sortie": "9-full_auto"}));
        recs.extend(taxi(11.0, Some("9-full_auto")));
        recs.push(json!({"t": 14.0, "kind": "tool", "name": "set_throttle", "sortie": "9-full_auto"}));
        recs.push(json!({"t": 15.0, "kind": "sortie", "event": "end", "outcome": "ended",
                         "sortie": "9-full_auto"}));
        recs.extend(taxi(20.0, None));
        let card = score_all(recs.iter());
        let text = block(&card);
        print!("{text}");
        assert_eq!(
            text,
            "  Outside a mission (stand-alone tools):\n\
             \x20   taxi      pass\n\
             \x20 Mission 9-full_auto:\n\
             \x20   taxi      pass\n\
             \x20   SORTIE FAIL: 1 stages, none failed, 0 interventions, 1 commands, attended, outcome ended\n\
             \x20 Outside a mission (stand-alone tools):\n\
             \x20   taxi      pass\n"
        );
        let sortie = card.iter().find(|v| s(v, "kind") == "sortie_result").unwrap();
        assert_eq!(sortie["unattended"], false);
        assert_eq!(sortie["pass"], false, "a flown-by-voice sortie is not an unattended pass");
    }

    /// LEMG 13, 17:04:34Z (1790441293): the autoland flare ticks as recorded
    /// (t, radio ft, vvi fpm) and the 1 Hz state samples around them. The
    /// state record never had the wheels on until the second contact, and
    /// the stage scored that one's 278 fpm; the first contact was -794.
    #[test]
    fn a_bounced_landing_is_judged_on_its_first_contact() {
        let flare = [
            (1790442273.6, 11.8, -598.0),
            (1790442273.9, 8.1, -664.0),
            (1790442274.1, 5.1, -724.0),
            (1790442274.3, 1.7, -773.0),
            (1790442274.5, -0.5, -794.0),
            (1790442274.7, 0.5, -697.0),
            (1790442274.9, 1.8, -512.0),
            (1790442275.1, 3.0, -303.0),
            (1790442275.5, 4.8, -68.0),
            (1790442276.1, 7.3, 99.0),
            (1790442276.8, 11.2, 213.0),
            (1790442277.2, 11.8, 191.0),
            (1790442277.7, 10.4, 78.0),
            (1790442278.2, 5.4, -162.0),
        ];
        let state = [
            (1790442240.0, false, -700.0),
            (1790442272.5, false, -398.8),
            (1790442273.5, false, -413.4),
            (1790442274.6, false, -663.8),
            (1790442275.7, false, -385.9),
            (1790442276.9, false, 117.9),
            (1790442277.9, false, 191.2),
            (1790442279.2, false, -277.9),
            (1790442280.4, false, -42.7),
            (1790442281.7, true, -36.6),
            (1790442283.1, true, 1.0),
        ];
        let mut recs: Vec<Value> = flare
            .iter()
            .map(|(t, radio, vvi)| {
                json!({"t": t, "kind": "autoland", "radio": radio, "vvi": vvi, "phase": "Flare"})
            })
            .chain(state.iter().map(|(t, g, vsi)| {
                json!({"t": t, "kind": "state", "ground": g, "vsi": vsi, "hdg": 131.0, "track": 131.0})
            }))
            .collect();
        recs.sort_by(|a, b| f(a, "t").partial_cmp(&f(b, "t")).unwrap());
        recs.push(json!({"t": 1790442290.0, "kind": "sortie", "event": "end", "outcome": "ended"}));
        let out = score_all(recs.iter());
        let l = out.iter().find(|v| v["stage"] == "landing").unwrap();
        assert_eq!(l["sink_fpm"], 794.0, "{l}");
        assert_eq!(l["t_touchdown"], 1790442274.5);
        assert_eq!(l["bounce_ft"], 11.8);
        assert_eq!(l["second_contact_sink_fpm"], 278.0);
        assert_eq!(l["pass"], false);
        assert_eq!(l["why"][1], "bounced to 12 ft");

        // A clean arrival that latched the rollout at 2 ft has no contact in
        // the flare ticks: judged on the state sample as before.
        let recs = [
            json!({"t": 0.0, "kind": "state", "ground": false, "vsi": -300.0}),
            json!({"t": 39.0, "kind": "autoland", "radio": 2.1, "vvi": -250.0}),
            json!({"t": 39.5, "kind": "state", "ground": false, "vsi": -240.0}),
            json!({"t": 40.0, "kind": "state", "ground": true, "vsi": -200.0}),
            json!({"t": 50.0, "kind": "sortie", "event": "end"}),
        ];
        let out = score_all(recs.iter());
        let l = out.iter().find(|v| v["stage"] == "landing").unwrap();
        assert_eq!(l["sink_fpm"], 240.0, "{l}");
        assert!(l["bounce_ft"].is_null());
    }

    /// The offline twin of the monitor's rule: a `yoke_under_ap` inside the
    /// crew's own write window is not counted; one with no write of ours is.
    #[test]
    fn a_yoke_the_crew_is_writing_is_not_an_intervention() {
        let recs = vec![
            json!({"t": 0.0, "kind": "sortie", "event": "begin"}),
            json!({"t": 10.0, "kind": "autoland", "flare_elev": 0.3, "flare_ail": 0.0}),
            json!({"t": 11.0, "kind": "intervention", "what": "yoke_under_ap", "yp": 0.3}),
            json!({"t": 30.0, "kind": "autoland", "flare_elev": null, "flare_ail": null}),
            json!({"t": 30.5, "kind": "intervention", "what": "yoke_under_ap", "yp": -0.4}),
            json!({"t": 40.0, "kind": "sortie", "event": "end", "outcome": "ended"}),
        ];
        let out = score_all(recs.iter());
        let sortie = out.last().unwrap();
        assert_eq!(sortie["interventions"], 1, "{sortie}");
        assert_eq!(sortie["crew_yoke_discounted"], 1);
    }

    /// LEMG 1790584266-approach (2026-09-28), as recorded: the 737 flew into
    /// the ground 8.8 nm short of runway 12 at 3,353 fpm, the runner's "my
    /// controls" ended the sortie at 290.1 and the impact was logged at
    /// 290.2, outside it. The sortie scored nothing and was written
    /// `pass: true, stages: 0`.
    #[test]
    fn a_sortie_that_scored_nothing_or_crashed_is_no_pass() {
        let id = "1790584266-approach";
        let recs = vec![
            json!({"t": 1790584266.3, "kind": "sortie", "event": "begin", "sortie": id}),
            json!({"t": 1790584288.9, "kind": "state", "ground": false, "crashed": false,
                   "agl": 46.9, "vsi": -3353.2, "hdg": 107.7, "track": 108.8, "sortie": id}),
            json!({"t": 1790584290.1, "kind": "sortie", "event": "end", "outcome": "ended",
                   "sortie": id}),
            json!({"t": 1790584290.2, "kind": "flight", "ev": "impact", "gs": 13.0,
                   "sink": -3353.0}),
        ];
        let card = score_all(recs.iter());
        let sortie = card.iter().find(|v| s(v, "kind") == "sortie_result").unwrap();
        assert_eq!(sortie["stages"], 0);
        assert_eq!(sortie["pass"], false, "nothing scored, nothing proved: {sortie}");
        // Its last state was 47 ft up: it ended in the air, too (#160).
        assert_eq!(
            line(sortie),
            "SORTIE FAIL: 0 stages, none failed, 0 interventions, 0 commands, unattended, \
             outcome ended, ended airborne"
        );

        let taxi_sortie = |before: Option<Value>, during: Option<Value>| {
            let mut recs: Vec<Value> = before.into_iter().collect();
            recs.push(json!({"t": 0.0, "kind": "sortie", "event": "begin", "sortie": "9-full_auto"}));
            for i in 0..10 {
                let mut d = drive(1.0 + i as f64 * 0.2, 6.0, 0.5, 0.0, 0.0, 0.0, 0.0);
                d["sortie"] = json!("9-full_auto");
                recs.push(d);
            }
            recs.push(json!({"t": 3.1, "kind": "autotaxi", "event": "holding_point",
                             "sortie": "9-full_auto"}));
            recs.extend(during);
            recs.push(json!({"t": 30.0, "kind": "sortie", "event": "end", "outcome": "ended",
                             "sortie": "9-full_auto"}));
            score_all(recs.iter())
                .into_iter()
                .find(|v| s(v, "kind") == "sortie_result")
                .unwrap()
        };
        assert_eq!(taxi_sortie(None, None)["pass"], true, "the clean one passes");
        // A crash inside the sortie fails it whatever its stages said.
        for crash in [
            json!({"t": 20.0, "kind": "flight", "ev": "impact", "sink": -3353.0}),
            json!({"t": 20.0, "kind": "flight", "ev": "abnormal", "from": "Normal", "to": "Crashed"}),
            json!({"t": 20.0, "kind": "rapport", "event": "touchdown", "impact": true}),
            json!({"t": 20.0, "kind": "state", "ground": true, "crashed": true}),
        ] {
            let sortie = taxi_sortie(None, Some(crash.clone()));
            assert_eq!(sortie["pass"], false, "{crash}: {sortie}");
            assert_eq!(sortie["crashed"], true);
            assert!(line(&sortie).ends_with("outcome ended, crashed"), "{}", line(&sortie));
        }
        // The last case's wreck, before this sortie began, is not this one's.
        let before = json!({"t": -5.0, "kind": "flight", "ev": "impact", "sink": -3353.0});
        let sortie = taxi_sortie(Some(before), None);
        assert_eq!(sortie["pass"], true, "{sortie}");
    }

    /// #158: the runner's swap from the SR22 still on final to LEMG 30
    /// (1,270 ft, 549 fpm down) to the SF50 on 13's threshold was scored as a
    /// landing: "touched down at 549 fpm", `t_touchdown` 1790607309.3, the
    /// flight model's `reposition` itself. The records around it as recorded
    /// (the SR22's descent before 1790607300.4 at its recorded sink).
    #[test]
    fn a_reposition_is_never_a_touchdown() {
        let air = |t: f64, agl: f64, vsi: f64| {
            json!({"t": t, "kind": "state", "ground": false, "agl": agl, "vsi": vsi,
                   "hdg": 299.3, "track": 298.2, "crashed": false})
        };
        let ground = |t: f64, vsi: f64, track: f64| {
            json!({"t": t, "kind": "state", "ground": true, "agl": 0.0, "vsi": vsi,
                   "hdg": 132.0, "track": track, "crashed": false})
        };
        let mut recs = Vec::new();
        for i in 0..36 {
            let t = 1790607250.0 + i as f64 * 1.4;
            recs.push(air(t, 1700.0 - i as f64 * 12.0, -549.0));
        }
        recs.push(air(1790607300.4, 1271.1, -548.1));
        recs.push(json!({"t": 1790607300.7, "kind": "flight", "ev": "reposition", "to": "Approach"}));
        for t in [1790607300.7, 1790607301.1, 1790607301.7] {
            recs.push(air(t, -193.9, -549.3));
        }
        recs.push(json!({"t": 1790607302.1, "kind": "flight_meta", "aircraft": "cirrus_vision_sf50"}));
        for t in [1790607302.1, 1790607303.5, 1790607308.7] {
            recs.push(air(t, -193.9, -549.3));
        }
        recs.push(json!({"t": 1790607309.3, "kind": "flight", "ev": "reposition", "to": "LinedUp"}));
        for t in [1790607309.3, 1790607309.6, 1790607310.0] {
            recs.push(ground(t, 0.0, 0.0));
        }
        recs.push(json!({"t": 1790607309.9, "kind": "autoland", "radio": 1073.5, "vvi": 0.0,
                         "ground": false}));
        for i in 0..10 {
            recs.push(ground(1790607311.8 + i as f64 * 0.35, 1.6, 139.8));
        }
        recs.push(ground(1790607330.0, 0.0, 139.8));
        let landings = |recs: &[Value]| -> Vec<Value> {
            score_all(recs.iter())
                .into_iter()
                .filter(|v| s(v, "stage") == "landing")
                .collect()
        };
        assert!(landings(&recs).is_empty(), "{:?}", landings(&recs));
        // The same stream with no word of the new flight is the recorded
        // verdict: the scorer path the markers now close.
        let blind: Vec<Value> = recs
            .iter()
            .filter(|r| s(r, "kind") != "flight_meta" && s(r, "ev") != "reposition")
            .cloned()
            .collect();
        let old = landings(&blind);
        assert_eq!(old.len(), 1, "{old:?}");
        assert_eq!(old[0]["sink_fpm"], 549.0);
        assert_eq!(old[0]["t_touchdown"], 1790607309.3);
        // Either marker alone is a new flight.
        for keep in ["flight_meta", "reposition"] {
            let one: Vec<Value> = recs
                .iter()
                .filter(|r| match keep {
                    "flight_meta" => s(r, "ev") != "reposition",
                    _ => s(r, "kind") != "flight_meta"
                        && !(s(r, "ev") == "reposition" && s(r, "to") == "Approach"),
                })
                .cloned()
                .collect();
            assert!(landings(&one).is_empty(), "{keep}: {:?}", landings(&one));
        }
    }

    /// A landing whose wheels met the runway BEFORE a reposition is still
    /// judged: the new flight closes it, it does not erase it.
    #[test]
    fn a_landing_before_a_reposition_is_still_judged() {
        let mut recs = Vec::new();
        for i in 0..40 {
            recs.push(json!({"t": i as f64, "kind": "state", "ground": false, "agl": 500.0,
                             "vsi": -180.0, "hdg": 130.0, "track": 130.0, "crashed": false}));
        }
        recs.push(json!({"t": 40.0, "kind": "state", "ground": true, "agl": 0.0, "vsi": -180.0,
                         "hdg": 130.0, "track": 130.0, "crashed": false}));
        recs.push(json!({"t": 42.0, "kind": "flight", "ev": "reposition", "to": "LinedUp"}));
        let card = score_all(recs.iter());
        let l: Vec<&Value> = card.iter().filter(|v| s(v, "stage") == "landing").collect();
        assert_eq!(l.len(), 1, "{card:?}");
        assert_eq!(l[0]["t_touchdown"], 40.0);
        assert_eq!(l[0]["pass"], true);
    }

    /// The SF50's LEMG 13 circuit (1790607325, 2026-09-28, #160), trimmed
    /// to 1,279 of its 2,252 records (every circuit tick to +320 s and every
    /// fifth after, the state records cut to what the scorer reads, api_load
    /// and duty dropped). Nothing flew the aircraft from 300 ft: it climbed
    /// in circles to 21,882 ft and the deadline ended the sortie there. Live
    /// it was written PASS on its alignment and takeoff.
    const LEMG_SF50_CIRCUIT: &str =
        include_str!("../bench/sorties/1790607325-full_auto.trimmed.jsonl");

    #[test]
    fn a_mission_that_ends_in_the_air_never_passes() {
        let recs = records(LEMG_SF50_CIRCUIT);
        let live = recs
            .iter()
            .find(|r| s(r, "kind") == "sortie_result")
            .unwrap();
        assert_eq!(live["pass"], true, "the verdict that hid it");
        let card = score_all(recs.iter());
        let text = block(&card);
        print!("{text}");
        assert_eq!(
            text,
            "  Mission 1790607325-full_auto:\n\
             \x20   alignment pass\n\
             \x20   takeoff   pass\n\
             \x20   SORTIE FAIL: 2 stages, none failed, \
             0 interventions, 0 commands, unattended, outcome ended, ended airborne\n"
        );
        let sortie = card.last().unwrap();
        assert_eq!(sortie["ended_airborne"], true);
        let top = recs
            .iter()
            .filter(|r| s(r, "kind") == "state")
            .filter_map(|r| f(r, "agl"))
            .fold(0.0, f64::max);
        assert!(top > 21_800.0, "{top}");
        // With the circuit's own guard it gives the aircraft back at the
        // pattern's margin, and says so as a failed stage.
        let body: Vec<Value> = recs
            .iter()
            .filter(|r| !matches!(s(r, "kind"), "sortie" | "sortie_result" | "stage_result"))
            .filter(|r| f(r, "t").is_some_and(|t| t < 1790607590.0))
            .cloned()
            .collect();
        let mut guarded = vec![
            json!({"t": 1790607325.1, "kind": "sortie", "event": "begin",
                                      "label": "full_auto", "sortie": "1790607325-full_auto"}),
        ];
        guarded.extend(body);
        guarded.push(
            json!({"t": 1790607593.6, "kind": "circuit", "event": "pattern_guard",
                            "why": "above_pattern", "leg": "Crosswind", "above_ft": 1695.2,
                            "alt_msl": 3301.5, "pattern_ft": 1606.3,
                            "sortie": "1790607325-full_auto"}),
        );
        let end = json!({"t": 1790607600.0, "kind": "sortie", "event": "end",
                         "outcome": "ended", "sortie": "1790607325-full_auto"});
        let mut given_back = guarded.clone();
        given_back.push(end.clone());
        let card = score_all(given_back.iter());
        let circuit = card.iter().find(|v| s(v, "stage") == "circuit").unwrap();
        assert_eq!(circuit["pass"], false);
        assert_eq!(
            circuit["why"][0],
            "1695 ft above the pattern on the crosswind"
        );
        assert_eq!(card.last().unwrap()["pass"], false);
        assert_eq!(card.last().unwrap()["ended_airborne"], true);
        // The ground clears it: a mission that ends on its wheels is judged
        // on its stages as before.
        let mut landed = guarded;
        landed.push(
            json!({"t": 1790607595.0, "kind": "state", "ground": true, "agl": 0.0,
                           "vsi": 0.0, "hdg": 130.0, "track": 130.0, "crashed": false}),
        );
        landed.push(end);
        assert_eq!(
            score_all(landed.iter()).last().unwrap()["ended_airborne"],
            false
        );
    }
}
