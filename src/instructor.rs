//! The flight instructor: the third persona on the intercom.
//!
//! An instructor teaches, watches and grades; it never touches a control.
//! That is a MODE, not a manner: in [`crate::mode::Mode::Instructor`] every
//! curated tool is refused at the one door they all walk through (the main
//! loop's tool dispatch), the monitor's fix actions and the cockpit duties
//! stand down, and a checklist runs only in its Instruct flavour. What is
//! left is exactly what a right-seat instructor does with folded arms: the
//! monitor's callouts, the ATC coach, the checklist read-and-watch, and this
//! module's lessons.
//!
//! A lesson is one ACS task (straight and level, a climb, a descent, steep
//! turns, slow flight, the traffic pattern, a landing). The instructor
//! ASSIGNS the numbers at the brief (the altitude, the heading, the speed the
//! POH gives for the task) so the scorer never has to guess what the pilot
//! meant, then grades against the Airman Certification Standards tolerances
//! for the chosen level, narrating a sustained excursion as it happens
//! ("Altitude, 120 low.") and reading the grade back at the end. Everything
//! is blackboxed under kind `instructor`, and the same scorer runs OFFLINE
//! over the blackbox's `state` records in `xvl debrief` (`grade_flight`), so
//! a hand-flown sortie with no lesson armed still gets its numbers.
//!
//! Pure state machine plus text, tested as plain `cargo test`: the monitor
//! feeds samples through [`tick`], the main loop feeds words through
//! [`request`] and [`handle`]. No model, no guessing: a word outside the
//! closed vocabulary is not a lesson command.

use std::sync::Mutex;

use serde_json::{json, Value};

use crate::aircraft::AircraftProfile;

/// An excursion must persist this long before the instructor speaks it. A
/// gust is not a deviation; a pilot who has not noticed after three seconds
/// needs telling.
pub const NARRATE_SUSTAIN_S: f64 = 3.0;
/// And an excursion still standing is re-spoken no sooner than this.
pub const RENOTIFY_S: f64 = 20.0;
/// Momentary excursions an examiner lets pass, in total judged seconds. Past
/// this the axis fails however small the deviation.
pub const EXCURSION_ALLOW_S: f64 = 3.0;
/// A deviation past this multiple of the tolerance fails the axis outright,
/// however brief.
pub const GROSS_MULT: f64 = 2.0;
/// The taxi lesson's centreline (#88), metres off the cleared route: the
/// sortie scorecard's own line at private (`scorecard::TAXI_MAX_LAT_M`, the
/// crew's taxi is held to it), tighter at commercial. Inside a route VERTEX
/// while turning through it the offset is a cut, judged at
/// `scorecard::TAXI_MAX_CUT_M` by the same `scorecard::LineJudge`: no wheel
/// follows a polyline's corner.
pub const TAXI_LINE_PRIVATE_M: f64 = crate::scorecard::TAXI_MAX_LAT_M;
pub const TAXI_LINE_COMMERCIAL_M: f64 = 2.0;
/// A route turn this close ahead (the turn card's next decision point) asks
/// for the turn: the corner opens, as the drive's `turn_dps` opens it for
/// the scorecard.
pub const LINE_CORNER_AHEAD_M: f64 = 15.0;
/// The line is not judged this close to the route's end: the stop at the
/// hold short, where the projection runs onto the last stub.
pub const LINE_END_M: f64 = 5.0;
/// Level flight for the level-off judgement: this little vertical speed for
/// [`LEVEL_OFF_HOLD_S`].
pub const LEVEL_OFF_VSI_FPM: f64 = 150.0;
pub const LEVEL_OFF_HOLD_S: f64 = 5.0;
/// A climb or descent is judged only while it is one: past this rate.
pub const CLIMB_VSI_FPM: f64 = 200.0;
/// Straight-and-level and slow flight run this long before they are complete.
pub const STRAIGHT_LEVEL_S: f64 = 120.0;
pub const SLOW_FLIGHT_S: f64 = 90.0;
/// Steep turns: in the turn past this bank, rolled out below this.
pub const TURN_IN_BANK_DEG: f64 = 30.0;
pub const TURN_OUT_BANK_DEG: f64 = 10.0;
/// The final approach is judged below this height, descending.
pub const FINAL_AGL_FT: f64 = 800.0;
/// A touchdown sinking faster than this is "firm"; drift past this is drift.
pub const FIRM_TOUCHDOWN_FPM: f64 = 300.0;
/// The touchdown's sink is the steepest seen this long before the ground
/// flag, not the last sample: at the monitor's 1 Hz the last "airborne"
/// sample can already be on the oleos (LFPG 09R, 2026-09-22: -690 at 12 ft,
/// -221 one sample later with the flag still clear, -672 by the sim's own
/// report, and the debrief said 221).
pub const TOUCHDOWN_WINDOW_S: f64 = 2.5;

/// The sink at the wheels from `recent` airborne samples `(t_s, vsi_fpm)`
/// and the first ground sample: the steepest inside [`TOUCHDOWN_WINDOW_S`]
/// of it, the ground sample included (the oleos have eaten part of it by
/// then; LXGB 27, 2026-09-14: -309 in the air, -156 on the ground).
pub fn touchdown_sink_fpm(recent: &[(f64, f64)], t_ground_s: f64, ground_vsi_fpm: f64) -> f64 {
    recent
        .iter()
        .filter(|(t, _)| t_ground_s - *t <= TOUCHDOWN_WINDOW_S)
        .map(|(_, v)| *v)
        .fold(ground_vsi_fpm, f64::min)
}
pub const DRIFT_DEG: f64 = 5.0;
/// Pattern altitude is judged this close to it (the upwind and the descent
/// on base are not the pattern altitude).
pub const PATTERN_BAND_FT: f64 = 300.0;
/// The taxi lesson's straight-leg pace when the profile names none, and
/// how far over it is a fault.
pub const DEFAULT_TAXI_KTS: f64 = 15.0;
pub const TAXI_OVER_KTS: f64 = 3.0;
/// The taxi is judged as moving above this, complete after this long
/// stopped (the hold-short) once it has covered this far.
pub const TAXI_MOVING_KTS: f64 = 2.0;
pub const TAXI_STOP_S: f64 = 10.0;
pub const TAXI_MIN_M: f64 = 150.0;
/// A takeoff roll starts past this groundspeed; the centreline heading is
/// judged from here; the climb speed from this height to the top.
pub const ROLL_KTS: f64 = 15.0;
pub const ROLL_JUDGE_KTS: f64 = 25.0;
pub const CLIMB_JUDGE_AGL_FT: f64 = 200.0;
pub const TAKEOFF_TOP_AGL_FT: f64 = 1000.0;
/// A rotation this far under Vr is early (a fail); this far over is late
/// (a note).
pub const ROTATE_EARLY_KTS: f64 = 10.0;
pub const ROTATE_LATE_KTS: f64 = 15.0;
/// The full-flight syllabus: the landing arms below this height descending
/// after the flight has been above the takeoff top; the shutdown list
/// after this long stopped.
pub const SYLLABUS_STOP_S: f64 = 20.0;
pub const ENGINE_NAG_S: f64 = 60.0;
/// Offline segmentation: a level run this long is scored as straight and
/// level; a climb or descent this long is scored as one.
pub const OFFLINE_LEVEL_MIN_S: f64 = 60.0;
pub const OFFLINE_CLIMB_MIN_S: f64 = 30.0;
/// Most segments the offline grade prints.
pub const OFFLINE_MAX_SEGMENTS: usize = 12;
/// Offline: a heading this far from the run's reference ends the straight
/// sub-run; the turn is skipped while consecutive samples still swing
/// faster than this.
pub const OFFLINE_TURN_DEG: f64 = 20.0;
pub const OFFLINE_TURN_RATE_DPS: f64 = 1.0;

/// Offline: a smaller heading change is still a TURN, not a wander, when the
/// aircraft settles on the new heading: past the private heading tolerance
/// and then steady (within `OFFLINE_SETTLED_DEG`) for `OFFLINE_SETTLED_S`.
/// A route's 10 to 20 degree turns at its fixes were graded as "heading 19
/// degrees left (FAIL)" on six of nine segments of a followed plan
/// (EGPH-EGTE, 2026-09-21); a wander comes back and is still graded.
pub const OFFLINE_COURSE_CHANGE_DEG: f64 = 10.0;
pub const OFFLINE_SETTLED_DEG: f64 = 4.0;
pub const OFFLINE_SETTLED_S: f64 = 30.0;

/// Do the samples from here on hold a NEW heading, off `ref_hdg` by more than
/// the tolerance, for the settled window? `ahead[0]` is the sample in question.
pub fn course_change_at(ahead: &[Sample], ref_hdg: f64) -> bool {
    let Some(first) = ahead.first() else {
        return false;
    };
    if crate::angles::wrap180(first.hdg_deg - ref_hdg).abs() <= OFFLINE_COURSE_CHANGE_DEG {
        return false;
    }
    let window: Vec<&Sample> = ahead
        .iter()
        .take_while(|s| s.t_s - first.t_s <= OFFLINE_SETTLED_S)
        .collect();
    let spans = window
        .last()
        .is_some_and(|l| l.t_s - first.t_s >= OFFLINE_SETTLED_S * 0.8);
    spans
        && window
            .iter()
            .all(|s| crate::angles::wrap180(s.hdg_deg - first.hdg_deg).abs() <= OFFLINE_SETTLED_DEG)
}

/// The certificate whose standards apply.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Level {
    Private,
    Commercial,
}

impl Level {
    pub fn name(self) -> &'static str {
        match self {
            Level::Private => "private",
            Level::Commercial => "commercial",
        }
    }
}

/// One ACS task the instructor can teach and grade.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Task {
    Taxi,
    Takeoff,
    StraightAndLevel,
    Climb,
    Descent,
    SteepTurns,
    SlowFlight,
    Pattern,
    Landing,
}

/// The tolerances for a task at a level. `hdg_deg` and `bank` are `None`
/// where the ACS names no standard for that axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Standards {
    pub alt_ft: Option<f64>,
    pub hdg_deg: Option<f64>,
    pub ias_plus: f64,
    pub ias_minus: f64,
    /// (target bank, tolerance) where a bank is the task.
    pub bank: Option<(f64, f64)>,
    /// Metres off the cleared route (the taxi's centreline).
    pub line_m: Option<f64>,
}

impl Task {
    pub const ALL: [Task; 9] = [
        Task::Taxi,
        Task::Takeoff,
        Task::StraightAndLevel,
        Task::Climb,
        Task::Descent,
        Task::SteepTurns,
        Task::SlowFlight,
        Task::Pattern,
        Task::Landing,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Task::Taxi => "taxiing",
            Task::Takeoff => "a normal takeoff",
            Task::StraightAndLevel => "straight and level",
            Task::Climb => "a constant airspeed climb",
            Task::Descent => "a constant airspeed descent",
            Task::SteepTurns => "steep turns",
            Task::SlowFlight => "slow flight",
            Task::Pattern => "the traffic pattern",
            Task::Landing => "a normal approach and landing",
        }
    }

    /// The blackbox and panel key.
    pub fn key(self) -> &'static str {
        match self {
            Task::Taxi => "taxi",
            Task::Takeoff => "takeoff",
            Task::StraightAndLevel => "straight_and_level",
            Task::Climb => "climb",
            Task::Descent => "descent",
            Task::SteepTurns => "steep_turns",
            Task::SlowFlight => "slow_flight",
            Task::Pattern => "pattern",
            Task::Landing => "landing",
        }
    }

    /// The ACS tolerances. Private: altitude 100 ft, heading 10 degrees,
    /// airspeed 10 kt (climbs and descents +10/-5, slow flight +10/-0),
    /// steep turns 45 degrees of bank within 5. Commercial: 50 ft, 5
    /// degrees, 5 kt (slow flight +5/-0), steep turns 50 degrees of bank,
    /// roll-out within 10.
    pub fn standards(self, level: Level) -> Standards {
        let (alt, hdg, ias) = match level {
            Level::Private => (100.0, 10.0, 10.0),
            Level::Commercial => (50.0, 5.0, 5.0),
        };
        // The taxi's centreline: the sortie scorecard's line at private,
        // tighter at commercial (the ACS names no figure).
        let line = match level {
            Level::Private => TAXI_LINE_PRIVATE_M,
            Level::Commercial => TAXI_LINE_COMMERCIAL_M,
        };
        let steep_bank = match level {
            Level::Private => 45.0,
            Level::Commercial => 50.0,
        };
        match self {
            // The taxi's speed axis is the PACE over the ground: over the
            // airframe's straight-leg pace by more than this is the fault;
            // slow is never one.
            Task::Taxi => Standards {
                alt_ft: None,
                hdg_deg: None,
                ias_plus: TAXI_OVER_KTS,
                ias_minus: 100.0,
                bank: None,
                line_m: Some(line),
            },
            // Centreline as the roll heading; Vy +10/-5 in the initial climb.
            Task::Takeoff => Standards {
                alt_ft: None,
                hdg_deg: Some(hdg),
                ias_plus: 10.0,
                ias_minus: 5.0,
                bank: None,
                line_m: None,
            },
            Task::StraightAndLevel => Standards {
                alt_ft: Some(alt),
                hdg_deg: Some(hdg),
                ias_plus: ias,
                ias_minus: ias,
                bank: None,
                line_m: None,
            },
            Task::Climb | Task::Descent => Standards {
                alt_ft: Some(alt),
                hdg_deg: Some(hdg),
                ias_plus: 10.0,
                ias_minus: 5.0,
                bank: None,
                line_m: None,
            },
            Task::SteepTurns => Standards {
                alt_ft: Some(alt),
                hdg_deg: Some(10.0),
                ias_plus: ias,
                ias_minus: ias,
                bank: Some((steep_bank, 5.0)),
                line_m: None,
            },
            Task::SlowFlight => Standards {
                alt_ft: Some(alt),
                hdg_deg: Some(hdg),
                ias_plus: ias,
                ias_minus: 0.0,
                bank: Some((0.0, 10.0)),
                line_m: None,
            },
            Task::Pattern => Standards {
                alt_ft: Some(alt),
                hdg_deg: None,
                ias_plus: ias,
                ias_minus: ias,
                bank: None,
                line_m: None,
            },
            Task::Landing => Standards {
                alt_ft: None,
                hdg_deg: None,
                ias_plus: 10.0,
                ias_minus: 5.0,
                bank: None,
                line_m: None,
            },
        }
    }

    /// Air work needs height and a wing in the air; the pattern and a
    /// landing can start anywhere (the pattern from the ground).
    fn entry_ok(self, s: &Sample) -> Result<(), &'static str> {
        match self {
            Task::Taxi if !s.on_ground => Err("that's a ground lesson"),
            Task::Taxi if !s.engine_running => Err("start the engine first"),
            Task::Taxi => Ok(()),
            Task::Takeoff if !s.on_ground => Err("that's a ground lesson"),
            Task::Takeoff => Ok(()),
            Task::Pattern => Ok(()),
            Task::Landing if !s.on_ground => Ok(()),
            Task::Landing => Err("get airborne first"),
            _ if s.on_ground => Err("get airborne first"),
            _ if s.agl_ft < 1500.0 => Err("climb above fifteen hundred feet first"),
            _ => Ok(()),
        }
    }
}

/// The airframe's numbers the tasks are flown to, from the profile.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Refs {
    pub vy_kts: Option<f64>,
    pub vr_kts: Option<f64>,
    /// Straight-leg taxi pace: the profile's, else [`DEFAULT_TAXI_KTS`].
    pub taxi_kts: f64,
    pub approach_kts: Option<f64>,
    pub cruise_ias_kts: Option<f64>,
    pub va_kts: Option<f64>,
    /// The slow-flight speed: the airframe's speed floor (1.2 x clean Vs),
    /// else 1.3 x Vso.
    pub slow_kts: Option<f64>,
    pub pattern_agl_ft: f64,
    pub pattern_kts: Option<f64>,
}

impl Refs {
    pub fn from_profile(p: &AircraftProfile) -> Refs {
        let r = p.reference.as_ref();
        let vs = p.vs_clean_kts;
        // The approach autoland and the circuit fly: one rule
        // (`aircraft::approach_kts`), so the brief, the scorer and the
        // pattern size agree with the aircraft.
        let approach = crate::aircraft::approach_kts(p);
        let vy = r.and_then(|r| r.vy_kts);
        let slow = vs
            .map(|v| (v * crate::aircraft::SPEED_FLOOR_RATIO).round())
            .or_else(|| {
                r.and_then(|r| r.vs0_kts)
                    .map(|v| (v * crate::aircraft::APPROACH_VSO_RATIO).round())
            });
        let pattern_agl = crate::tools::circuit::pattern_alt_agl_ft(approach.unwrap_or(65.0));
        let pattern_kts = match (vy, approach) {
            (Some(vy), Some(app)) => Some(crate::circuit::pattern_speed_kts(vy, app, vs)),
            _ => None,
        };
        Refs {
            vy_kts: vy,
            vr_kts: r.and_then(|r| r.vr_kts),
            taxi_kts: p
                .taxi
                .as_ref()
                .and_then(|t| t.kts)
                .unwrap_or(DEFAULT_TAXI_KTS),
            approach_kts: approach,
            cruise_ias_kts: r.and_then(|r| r.cruise_ias_kts),
            va_kts: r.and_then(|r| r.va_kts),
            slow_kts: slow,
            pattern_agl_ft: pattern_agl,
            pattern_kts,
        }
    }
}

/// One tick of the aircraft, as the monitor sees it. `t_s` is any monotonic
/// clock in seconds; the scorer only differences it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    pub t_s: f64,
    pub alt_ft: f64,
    pub agl_ft: f64,
    pub hdg_deg: f64,
    pub ias_kts: f64,
    pub bank_deg: f64,
    pub vsi_fpm: f64,
    pub on_ground: bool,
    /// Ground track, for the touchdown drift; `None` when unread.
    pub track_deg: Option<f64>,
    /// Groundspeed, knots (the taxi's pace, the takeoff roll).
    pub gs_kts: f64,
    pub engine_running: bool,
    /// Where the aircraft stands against the cleared taxi route, while one
    /// is published and the aircraft is on the ground; `None` otherwise.
    pub taxi: Option<TaxiLine>,
}

/// The aircraft against the cleared taxi route, from the taxi guide's turn
/// card (`taxi_guide::TurnCard`, the same projection the panel's arrow
/// draws).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TaxiLine {
    /// Signed cross-track, metres, positive right of the route.
    pub xtk_m: f64,
    /// Route left to its end, metres.
    pub remaining_m: f64,
    /// The side (+1 right, -1 left) of a route turn inside
    /// [`LINE_CORNER_AHEAD_M`], if there is one.
    pub turn: Option<f64>,
    /// Past the guide's off-route line: the controller's conversation.
    pub off_route: bool,
}

impl TaxiLine {
    pub fn from_card(c: &crate::taxi_guide::TurnCard) -> TaxiLine {
        TaxiLine {
            xtk_m: c.xtk_m,
            remaining_m: c.remaining_m,
            turn: (c.kind == 'T' && c.dist_m <= LINE_CORNER_AHEAD_M && c.turn_deg != 0.0)
                .then(|| c.turn_deg.signum()),
            off_route: c.kind == 'X',
        }
    }

    /// The blackbox `state` record's `taxi` field: `[xtk, remaining, turn,
    /// off]`, turn 0 for none.
    pub fn json(&self) -> Value {
        json!([
            (self.xtk_m * 10.0).round() / 10.0,
            self.remaining_m.round(),
            self.turn.unwrap_or(0.0),
            u8::from(self.off_route)
        ])
    }

    pub fn from_json(v: &Value) -> Option<TaxiLine> {
        let a = v.as_array()?;
        let turn = a.get(2)?.as_f64()?;
        Some(TaxiLine {
            xtk_m: a.first()?.as_f64()?,
            remaining_m: a.get(1)?.as_f64()?,
            turn: (turn != 0.0).then_some(turn.signum()),
            off_route: a.get(3)?.as_f64()? > 0.5,
        })
    }
}

/// What the instructor assigned for the task.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Targets {
    pub alt_ft: Option<f64>,
    pub hdg_deg: Option<f64>,
    pub ias_kts: Option<f64>,
    pub bank_deg: Option<f64>,
    /// The taxi's pace over the ground.
    pub pace_kts: Option<f64>,
    /// The takeoff's rotation speed.
    pub vr_kts: Option<f64>,
}

const KTS_TO_MPS: f64 = 0.514_444;

fn round_to(v: f64, step: f64) -> f64 {
    (v / step).round() * step
}

impl Task {
    /// The instructor's assignment at the brief: numbers the pilot can hear
    /// and fly, from the airframe's book where the task has a book figure
    /// and from the aircraft's own state, rounded, where it does not.
    pub fn assign(self, entry: &Sample, refs: &Refs, level: Level) -> Targets {
        let hdg = Some(round_to(crate::angles::wrap360(entry.hdg_deg), 10.0) % 360.0);
        let here = round_to(entry.alt_ft, 100.0);
        match self {
            Task::Taxi => Targets {
                pace_kts: Some(refs.taxi_kts),
                ..Targets::default()
            },
            // The heading is a placeholder until the roll starts (the
            // aircraft at the hold line points anywhere); the scorer
            // retargets it at ROLL_KTS.
            Task::Takeoff => Targets {
                hdg_deg: hdg,
                ias_kts: refs.vy_kts,
                vr_kts: refs.vr_kts,
                ..Targets::default()
            },
            Task::StraightAndLevel => Targets {
                alt_ft: Some(here),
                hdg_deg: hdg,
                ias_kts: Some(refs.cruise_ias_kts.unwrap_or(round_to(entry.ias_kts, 5.0))),
                bank_deg: None,
                ..Targets::default()
            },
            Task::Climb => Targets {
                alt_ft: Some(((entry.alt_ft + 1000.0) / 500.0).ceil() * 500.0),
                hdg_deg: hdg,
                ias_kts: refs.vy_kts.or(Some(round_to(entry.ias_kts, 5.0))),
                bank_deg: None,
                ..Targets::default()
            },
            Task::Descent => {
                let ground = entry.alt_ft - entry.agl_ft;
                let floor = ground + 1500.0;
                let want = ((entry.alt_ft - 1000.0) / 500.0).floor() * 500.0;
                Targets {
                    alt_ft: Some(want.max(round_to(floor, 100.0))),
                    hdg_deg: hdg,
                    ias_kts: Some(round_to(entry.ias_kts, 5.0)),
                    bank_deg: None,
                    ..Targets::default()
                }
            }
            Task::SteepTurns => {
                let ias = match refs.va_kts {
                    Some(va) if entry.ias_kts > va => (va / 5.0).floor() * 5.0,
                    _ => round_to(entry.ias_kts, 5.0),
                };
                Targets {
                    alt_ft: Some(here),
                    hdg_deg: hdg,
                    ias_kts: Some(ias),
                    bank_deg: self.standards(level).bank.map(|(b, _)| b),
                    ..Targets::default()
                }
            }
            Task::SlowFlight => Targets {
                alt_ft: Some(here),
                hdg_deg: hdg,
                ias_kts: refs.slow_kts,
                bank_deg: Some(0.0),
                ..Targets::default()
            },
            Task::Pattern => {
                let field = entry.alt_ft - entry.agl_ft;
                Targets {
                    alt_ft: Some(round_to(field + refs.pattern_agl_ft, 100.0)),
                    hdg_deg: None,
                    ias_kts: refs.pattern_kts.map(|k| k.round()),
                    bank_deg: None,
                    ..Targets::default()
                }
            }
            Task::Landing => Targets {
                alt_ft: None,
                hdg_deg: None,
                ias_kts: refs.approach_kts.map(|k| k.round()),
                bank_deg: None,
                ..Targets::default()
            },
        }
    }

    /// The brief: the task, the numbers, the standards, and how to start.
    pub fn brief(self, t: &Targets, level: Level) -> String {
        let s = self.standards(level);
        let mut parts: Vec<String> = Vec::new();
        match self {
            Task::Taxi => parts.push(format!(
                "Taxi. Get your clearance and taxi to the runway; I'll call the turns. Keep it under {} on the straights, walking pace in the turns, and stop short of the hold line.",
                t.pace_kts.map(knots).unwrap_or_else(|| "taxi pace".into()),
            )),
            Task::Takeoff => parts.push(format!(
                "Normal takeoff. Line up, full power, track the centreline, rotate at {} and climb at {} to a thousand feet.",
                t.vr_kts.map(knots).unwrap_or_else(|| "the book speed".into()),
                t.ias_kts.map(knots).unwrap_or_else(|| "climb speed".into()),
            )),
            Task::StraightAndLevel => parts.push(format!(
                "Straight and level. Hold {}, heading {}, {}, for two minutes.",
                t.alt_ft.map(feet).unwrap_or_default(),
                t.hdg_deg.map(hdg).unwrap_or_default(),
                t.ias_kts.map(knots).unwrap_or_else(|| "your speed".into()),
            )),
            Task::Climb => parts.push(format!(
                "Constant airspeed climb. Climb to {} at {}, heading {}, and level off there.",
                t.alt_ft.map(feet).unwrap_or_default(),
                t.ias_kts.map(knots).unwrap_or_else(|| "climb speed".into()),
                t.hdg_deg.map(hdg).unwrap_or_default(),
            )),
            Task::Descent => parts.push(format!(
                "Constant airspeed descent. Descend to {} at {}, heading {}, and level off there.",
                t.alt_ft.map(feet).unwrap_or_default(),
                t.ias_kts.map(knots).unwrap_or_else(|| "your speed".into()),
                t.hdg_deg.map(hdg).unwrap_or_default(),
            )),
            Task::SteepTurns => parts.push(format!(
                "Steep turns. {} of bank, a full turn left then a full turn right, holding {} and {}. Roll out on heading {}.",
                t.bank_deg.map(|b| format!("{b:.0} degrees")).unwrap_or_default(),
                t.alt_ft.map(feet).unwrap_or_default(),
                t.ias_kts.map(knots).unwrap_or_else(|| "your speed".into()),
                t.hdg_deg.map(hdg).unwrap_or_default(),
            )),
            Task::SlowFlight => parts.push(format!(
                "Slow flight. Slow to {} and hold {}, heading {}, wings level, for ninety seconds.",
                t.ias_kts.map(knots).unwrap_or_else(|| "just above the stall".into()),
                t.alt_ft.map(feet).unwrap_or_default(),
                t.hdg_deg.map(hdg).unwrap_or_default(),
            )),
            Task::Pattern => parts.push(format!(
                "Traffic pattern. Pattern altitude {}, {} on the downwind, then a normal landing.",
                t.alt_ft.map(feet).unwrap_or_default(),
                t.ias_kts.map(knots).unwrap_or_else(|| "pattern speed".into()),
            )),
            Task::Landing => parts.push(format!(
                "Normal approach and landing. {} on final, no drift, nose on the centreline.",
                t.ias_kts.map(knots).unwrap_or_else(|| "Approach speed".into()),
            )),
        }
        let mut std: Vec<String> = Vec::new();
        if self == Task::Taxi {
            std.push(format!(
                "no more than {:.0} knots over the pace",
                s.ias_plus
            ));
            if let Some(l) = s.line_m {
                std.push(format!("centreline within {l:.0} metres"));
            }
        }
        if let Some(a) = s.alt_ft {
            std.push(format!("altitude within {a:.0} feet"));
        }
        if let Some(h) = s.hdg_deg {
            std.push(format!("heading within {h:.0} degrees"));
        }
        if self == Task::Taxi {
            // said above, as pace
        } else if s.ias_plus == s.ias_minus {
            std.push(format!("airspeed within {:.0} knots", s.ias_plus));
        } else {
            std.push(format!(
                "airspeed plus {:.0} minus {:.0}",
                s.ias_plus, s.ias_minus
            ));
        }
        if let Some((b, tol)) = s.bank {
            if b > 0.0 {
                std.push(format!("bank within {tol:.0} degrees"));
            }
        }
        parts.push(format!(
            "{} standards: {}.",
            cap_first(level.name()),
            std.join(", ")
        ));
        parts.push("Say begin when you're ready.".into());
        parts.join(" ")
    }
}

fn feet(v: f64) -> String {
    let n = v.round() as i64;
    let s = n.abs().to_string();
    let mut out = String::new();
    for (i, c) in s.chars().enumerate() {
        if i > 0 && (s.len() - i).is_multiple_of(3) {
            out.push(',');
        }
        out.push(c);
    }
    format!("{}{out} feet", if n < 0 { "-" } else { "" })
}

fn hdg(v: f64) -> String {
    let n = crate::angles::wrap360(v).round() as i64;
    format!("{:03}", if n == 0 { 360 } else { n })
}

fn knots(v: f64) -> String {
    format!("{:.0} knots", v)
}

fn cap_first(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
        None => String::new(),
    }
}

/// Which axis a narration or a grade line is about.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Axis {
    Altitude,
    Heading,
    Airspeed,
    Bank,
    Pace,
    /// The taxi's cross-track to the cleared route.
    Centreline,
}

impl Axis {
    fn name(self) -> &'static str {
        match self {
            Axis::Altitude => "altitude",
            Axis::Heading => "heading",
            Axis::Airspeed => "airspeed",
            Axis::Bank => "bank",
            Axis::Pace => "pace",
            Axis::Centreline => "centreline",
        }
    }
}

/// One axis under judgement.
#[derive(Clone, Debug)]
struct AxisScore {
    axis: Axis,
    target: f64,
    tol_plus: f64,
    tol_minus: f64,
    /// The deviation of largest magnitude seen (signed: positive is high,
    /// right, fast, steep).
    worst: f64,
    out_s: f64,
    judged_s: f64,
    out_since: Option<f64>,
    spoken_at: Option<f64>,
    hinted: bool,
}

impl AxisScore {
    fn new(axis: Axis, target: f64, tol_plus: f64, tol_minus: f64) -> Self {
        AxisScore {
            axis,
            target,
            tol_plus,
            tol_minus,
            worst: 0.0,
            out_s: 0.0,
            judged_s: 0.0,
            out_since: None,
            spoken_at: None,
            hinted: false,
        }
    }

    fn outside(&self, dev: f64) -> bool {
        dev > self.tol_plus || dev < -self.tol_minus
    }

    /// Feed one judged sample. Returns a narration line when an excursion
    /// has stood for the sustain, or when it has just cleared after being
    /// spoken.
    fn feed(&mut self, dev: f64, dt: f64, t: f64) -> Option<String> {
        self.judged_s += dt;
        if dev.abs() > self.worst.abs() {
            self.worst = dev;
        }
        if self.outside(dev) {
            self.out_s += dt;
            let since = *self.out_since.get_or_insert(t);
            let stood = t - since >= NARRATE_SUSTAIN_S;
            let due = self.spoken_at.is_none_or(|s| t - s >= RENOTIFY_S);
            if stood && due {
                self.spoken_at = Some(t);
                let hint = if self.hinted { "" } else { self.hint(dev) };
                self.hinted = true;
                return Some(format!("{}{hint}", self.excursion_line(dev)));
            }
            None
        } else {
            let was_spoken = self.out_since.is_some() && self.spoken_at.is_some();
            self.out_since = None;
            if was_spoken {
                self.spoken_at = None;
                return Some(format!("Good, back on {}.", self.axis.name()));
            }
            None
        }
    }

    fn excursion_line(&self, dev: f64) -> String {
        let mag = dev.abs();
        match self.axis {
            Axis::Altitude => format!(
                "Altitude, {:.0} {}.",
                round_to(mag, 10.0),
                if dev > 0.0 { "high" } else { "low" }
            ),
            Axis::Heading => format!(
                "Heading, {:.0} {}.",
                mag.round(),
                if dev > 0.0 { "right" } else { "left" }
            ),
            Axis::Airspeed => format!(
                "Airspeed, {:.0} {}.",
                mag.round(),
                if dev > 0.0 { "fast" } else { "slow" }
            ),
            Axis::Bank => format!(
                "Bank, {:.0} {}.",
                mag.round(),
                if dev > 0.0 { "steep" } else { "shallow" }
            ),
            Axis::Pace => format!("Pace, {:.0} over.", mag.round()),
            Axis::Centreline => format!(
                "Centreline, {:.0} {} {}.",
                mag.round(),
                if mag.round() == 1.0 {
                    "metre"
                } else {
                    "metres"
                },
                if dev > 0.0 { "right" } else { "left" }
            ),
        }
    }

    /// The correction, said with the first excursion only: an instructor
    /// says how once and then lets the pilot find it.
    fn hint(&self, dev: f64) -> &'static str {
        match (self.axis, dev > 0.0) {
            (Axis::Altitude, true) => " Ease the nose down.",
            (Axis::Altitude, false) => " Nose up a touch.",
            (Axis::Heading, true) => " Come left.",
            (Axis::Heading, false) => " Come right.",
            (Axis::Airspeed, true) => " Ease the power back.",
            (Axis::Airspeed, false) => " Lower the nose.",
            (Axis::Bank, true) => " Shallow it a little.",
            (Axis::Bank, false) => " A little more bank.",
            (Axis::Pace, _) => " Ease the power back and hold it with the brakes.",
            (Axis::Centreline, true) => " A touch of left rudder, onto the line.",
            (Axis::Centreline, false) => " A touch of right rudder, onto the line.",
        }
    }

    fn grade(&self) -> AxisGrade {
        let tol = if self.worst >= 0.0 {
            self.tol_plus
        } else {
            self.tol_minus
        };
        let gross = tol > 0.0 && self.worst.abs() > tol * GROSS_MULT;
        let pass = self.judged_s > 0.0 && !gross && self.out_s <= EXCURSION_ALLOW_S;
        AxisGrade {
            axis: self.axis,
            target: self.target,
            worst: self.worst,
            tol_plus: self.tol_plus,
            tol_minus: self.tol_minus,
            out_s: self.out_s,
            judged_s: self.judged_s,
            pass,
        }
    }
}

/// The grade for one axis.
#[derive(Clone, Debug, PartialEq)]
pub struct AxisGrade {
    pub axis: Axis,
    pub target: f64,
    pub worst: f64,
    pub tol_plus: f64,
    pub tol_minus: f64,
    pub out_s: f64,
    pub judged_s: f64,
    pub pass: bool,
}

impl AxisGrade {
    fn unit(&self) -> &'static str {
        match self.axis {
            Axis::Altitude => "feet",
            Axis::Heading | Axis::Bank => "degrees",
            Axis::Airspeed | Axis::Pace => "knots",
            Axis::Centreline => "metres",
        }
    }

    fn worst_words(&self) -> String {
        let mag = self.worst.abs();
        if mag.round() == 0.0 {
            return "held on the number".to_string();
        }
        let dir = match (self.axis, self.worst > 0.0) {
            (Axis::Altitude, true) => "high",
            (Axis::Altitude, false) => "low",
            (Axis::Heading, true) => "right",
            (Axis::Heading, false) => "left",
            (Axis::Airspeed, true) => "fast",
            (Axis::Airspeed, false) => "slow",
            (Axis::Bank, true) => "steep",
            (Axis::Bank, false) => "shallow",
            (Axis::Pace, true) => "over",
            (Axis::Pace, false) => "under",
            (Axis::Centreline, true) => "right",
            (Axis::Centreline, false) => "left",
        };
        let unit = if mag.round() == 1.0 {
            self.unit().trim_end_matches('s')
        } else {
            self.unit()
        };
        format!("{:.0} {unit} {dir} at worst", mag.round())
    }

    pub fn spoken(&self) -> String {
        if self.judged_s <= 0.0 {
            return format!("{}: not judged.", cap_first(self.axis.name()));
        }
        if self.pass {
            format!(
                "{}: {}, within standards.",
                cap_first(self.axis.name()),
                self.worst_words()
            )
        } else {
            format!(
                "{}: {}, outside for {:.0} seconds.",
                cap_first(self.axis.name()),
                self.worst_words(),
                self.out_s.round()
            )
        }
    }

    fn json(&self) -> Value {
        json!({
            "axis": self.axis.name(),
            "target": self.target,
            "worst": (self.worst * 10.0).round() / 10.0,
            "out_s": (self.out_s * 10.0).round() / 10.0,
            "judged_s": self.judged_s.round(),
            "pass": self.pass,
        })
    }
}

/// The grade for a task.
#[derive(Clone, Debug, PartialEq)]
pub struct Grade {
    pub task: Task,
    pub level: Level,
    pub axes: Vec<AxisGrade>,
    /// Task-specific findings (the level-off, the roll-outs, the touchdown).
    pub notes: Vec<String>,
    pub complete: bool,
    pub pass: bool,
}

impl Grade {
    pub fn spoken(&self) -> String {
        let judged: Vec<&AxisGrade> = self.axes.iter().filter(|a| a.judged_s > 0.0).collect();
        if judged.is_empty() && self.notes.is_empty() {
            return format!("Nothing to grade yet for {}.", self.task.name());
        }
        let mut parts: Vec<String> = judged.iter().map(|a| a.spoken()).collect();
        parts.extend(self.notes.iter().cloned());
        let verdict = if !self.complete {
            "Task not completed."
        } else if self.pass {
            "That meets the standard."
        } else {
            "Not to standard yet. Again when you're ready."
        };
        format!("{} {verdict}", parts.join(" "))
    }

    /// One line per axis for the written debrief.
    pub fn written(&self) -> String {
        let parts: Vec<String> = self
            .axes
            .iter()
            .filter(|a| a.judged_s > 0.0)
            .map(|a| {
                format!(
                    "{} {}{}",
                    a.axis.name(),
                    a.worst_words().replace(" at worst", ""),
                    if a.pass { "" } else { " (FAIL)" }
                )
            })
            .collect();
        let mut text = parts.join(", ");
        if !self.notes.is_empty() {
            let notes: Vec<&str> = self.notes.iter().map(|n| n.trim_end_matches('.')).collect();
            if !text.is_empty() {
                text.push_str("; ");
            }
            text.push_str(&notes.join("; "));
        }
        format!(
            "{}{}",
            text,
            if self.complete {
                if self.pass {
                    format!(". Within {} standards.", self.level.name())
                } else {
                    format!(". Below {} standards.", self.level.name())
                }
            } else {
                ". Not completed.".to_string()
            }
        )
    }

    pub fn json(&self) -> Value {
        json!({
            "task": self.task.key(),
            "level": self.level.name(),
            "complete": self.complete,
            "pass": self.pass,
            "axes": self.axes.iter().map(AxisGrade::json).collect::<Vec<_>>(),
            "notes": self.notes,
        })
    }
}

/// Judges one task from samples. Pure: no clock, no sim, no voice.
#[derive(Clone, Debug)]
pub struct Scorer {
    task: Task,
    level: Level,
    targets: Targets,
    alt: Option<AxisScore>,
    hdg: Option<AxisScore>,
    ias: Option<AxisScore>,
    bank: Option<AxisScore>,
    pace: Option<AxisScore>,
    /// The taxi's centreline: the narrated axis and the scorecard's judge
    /// behind its grade (vertex cuts are cuts).
    line: Option<AxisScore>,
    line_judge: crate::scorecard::LineJudge,
    /// On the line once: judged from the first tick inside the tolerance
    /// (the stand is off the route; the scorecard's capture).
    line_captured: bool,
    /// Left the cleared route (the guide's off-route line).
    left_route: bool,
    /// The open corner's state last tick (`LineJudge::open_vertex`), the
    /// seconds past the tolerance inside it, and the seconds inside corners
    /// that closed as drawn CURVES: those are time off the line after all.
    corner_state: Option<bool>,
    corner_out_s: f64,
    curve_out_s: f64,
    started_t: Option<f64>,
    last: Option<Sample>,
    /// Seconds the aircraft has been level (climb/descent level-off watch).
    level_s: f64,
    /// Seconds level away from the assigned altitude (a level-off elsewhere).
    level_wrong_s: f64,
    in_turn: bool,
    turns_done: u8,
    /// Whether pattern altitude was reached (the pattern's judged window).
    reached_pattern: bool,
    /// Taxi: metres covered and seconds stopped since.
    moved_m: f64,
    stopped_s: f64,
    /// Takeoff: the roll has started, the wheels have left.
    roll_started: bool,
    lifted: bool,
    /// The approach speed the pattern's final is judged against.
    landing_kts: Option<f64>,
    notes: Vec<String>,
    complete: bool,
    /// The task itself went wrong (levelled at the wrong altitude, a firm
    /// or drifting touchdown): a fail whatever the axes say.
    task_failed: bool,
    /// Airborne `(t_s, vsi_fpm)` inside the touchdown window, for
    /// [`touchdown_sink_fpm`].
    sink_window: Vec<(f64, f64)>,
}

impl Scorer {
    pub fn new(task: Task, level: Level, targets: Targets) -> Self {
        let s = task.standards(level);
        let axis = |a: Axis, target: Option<f64>, plus: f64, minus: f64| {
            target.map(|t| AxisScore::new(a, t, plus, minus))
        };
        let alt = s
            .alt_ft
            .and_then(|tol| axis(Axis::Altitude, targets.alt_ft, tol, tol));
        let hdg = s
            .hdg_deg
            .and_then(|tol| axis(Axis::Heading, targets.hdg_deg, tol, tol));
        let ias = axis(Axis::Airspeed, targets.ias_kts, s.ias_plus, s.ias_minus);
        let bank = s
            .bank
            .and_then(|(_, tol)| axis(Axis::Bank, targets.bank_deg, tol, tol));
        let pace = if task == Task::Taxi {
            axis(Axis::Pace, targets.pace_kts, s.ias_plus, s.ias_minus)
        } else {
            None
        };
        let ias = if task == Task::Taxi { None } else { ias };
        let line = s
            .line_m
            .map(|tol| AxisScore::new(Axis::Centreline, 0.0, tol, tol));
        Scorer {
            task,
            level,
            targets,
            alt,
            hdg,
            ias,
            bank,
            pace,
            line,
            line_judge: crate::scorecard::LineJudge::default(),
            line_captured: false,
            left_route: false,
            corner_state: None,
            corner_out_s: 0.0,
            curve_out_s: 0.0,
            started_t: None,
            last: None,
            level_s: 0.0,
            level_wrong_s: 0.0,
            in_turn: false,
            turns_done: 0,
            reached_pattern: false,
            moved_m: 0.0,
            stopped_s: 0.0,
            roll_started: false,
            lifted: false,
            landing_kts: None,
            notes: Vec::new(),
            complete: false,
            task_failed: false,
            sink_window: Vec::new(),
        }
    }

    /// The speed the final is flown at, for the pattern (the landing task
    /// carries it as its own airspeed target).
    pub fn with_landing_speed(mut self, kts: Option<f64>) -> Self {
        self.landing_kts = kts.map(f64::round);
        self
    }

    pub fn task(&self) -> Task {
        self.task
    }

    pub fn targets(&self) -> Targets {
        self.targets
    }

    pub fn complete(&self) -> bool {
        self.complete
    }

    /// Retarget one axis mid-lesson (the pilot said "make it four thousand").
    pub fn retarget(&mut self, axis: Axis, target: f64) {
        let slot = match axis {
            Axis::Altitude => &mut self.alt,
            Axis::Heading => &mut self.hdg,
            Axis::Airspeed => &mut self.ias,
            Axis::Bank => &mut self.bank,
            Axis::Pace => &mut self.pace,
            // The line is the route's: nothing to retarget.
            Axis::Centreline => return,
        };
        if let Some(a) = slot.as_mut() {
            a.target = target;
        }
        match axis {
            Axis::Altitude => self.targets.alt_ft = Some(target),
            Axis::Heading => self.targets.hdg_deg = Some(target),
            Axis::Airspeed => self.targets.ias_kts = Some(target),
            Axis::Bank => self.targets.bank_deg = Some(target),
            Axis::Pace => self.targets.pace_kts = Some(target),
            Axis::Centreline => {}
        }
    }

    /// Seconds since the first sample.
    pub fn elapsed_s(&self) -> f64 {
        match (self.started_t, self.last) {
            (Some(s), Some(l)) => l.t_s - s,
            _ => 0.0,
        }
    }

    /// Feed one sample. Returns narration lines (excursions, recoveries and
    /// the task's own milestones); the caller speaks them.
    pub fn tick(&mut self, s: &Sample) -> Vec<String> {
        let mut lines = Vec::new();
        if self.complete {
            return lines;
        }
        let Some(prev) = self.last else {
            self.started_t = Some(s.t_s);
            self.last = Some(*s);
            return lines;
        };
        let dt = (s.t_s - prev.t_s).clamp(0.0, 5.0);
        self.last = Some(*s);
        let t = s.t_s;
        let climbing = s.vsi_fpm > CLIMB_VSI_FPM;
        let descending = s.vsi_fpm < -CLIMB_VSI_FPM;
        let level = s.vsi_fpm.abs() < LEVEL_OFF_VSI_FPM;
        let judge = |axis: &mut Option<AxisScore>, dev: f64, lines: &mut Vec<String>| {
            if let Some(a) = axis.as_mut() {
                if let Some(l) = a.feed(dev, dt, t) {
                    lines.push(l);
                }
            }
        };
        let hdg_dev = |target: Option<f64>| target.map(|h| crate::angles::wrap180(s.hdg_deg - h)).unwrap_or(0.0);
        match self.task {
            Task::Taxi => {
                let gs = s.gs_kts.abs();
                if gs > TAXI_MOVING_KTS {
                    judge(
                        &mut self.pace,
                        gs - self.targets.pace_kts.unwrap_or(gs),
                        &mut lines,
                    );
                }
                if let Some(l) = self.line_tick(s, &prev, dt) {
                    lines.push(l);
                }
                self.moved_m += gs * KTS_TO_MPS * dt;
                if s.on_ground && gs > ROLL_KTS * 2.0 {
                    // Straight onto the runway and rolling: no hold-short.
                    self.complete = true;
                    self.task_failed = true;
                    self.notes
                        .push("Rolled without stopping at the hold line.".into());
                    lines.push("You never stopped at the hold line.".into());
                } else if self.moved_m >= TAXI_MIN_M && sim::ground::stopped_kts(gs) {
                    self.stopped_s += dt;
                    if self.stopped_s >= TAXI_STOP_S {
                        self.complete = true;
                        self.notes
                            .push(format!("Taxied {:.0} metres and held.", self.moved_m));
                        lines.push("Stopped. Holding short, I take it.".into());
                    }
                } else {
                    self.stopped_s = 0.0;
                }
            }
            Task::Takeoff => {
                let gs = s.gs_kts.abs();
                if !self.roll_started {
                    if s.on_ground && gs > ROLL_KTS {
                        self.roll_started = true;
                        self.retarget(Axis::Heading, crate::angles::wrap360(s.hdg_deg));
                        lines.push("Rolling.".into());
                    }
                    return lines;
                }
                let judging_hdg = (s.on_ground && gs > ROLL_JUDGE_KTS)
                    || (!s.on_ground && s.agl_ft < TAKEOFF_TOP_AGL_FT);
                if judging_hdg {
                    judge(&mut self.hdg, hdg_dev(self.targets.hdg_deg), &mut lines);
                }
                if !self.lifted && !s.on_ground && prev.on_ground {
                    self.lifted = true;
                    let mut note = format!("Rotated at {:.0} knots", prev.ias_kts.round());
                    if let Some(vr) = self.targets.vr_kts {
                        note.push_str(&format!(", Vr is {vr:.0}"));
                        if prev.ias_kts < vr - ROTATE_EARLY_KTS {
                            note.push_str(". Early");
                            self.task_failed = true;
                        } else if prev.ias_kts > vr + ROTATE_LATE_KTS {
                            note.push_str(". Late");
                        }
                    }
                    note.push('.');
                    self.notes.push(note);
                    lines.push("Airborne.".into());
                }
                if self.lifted && s.agl_ft > CLIMB_JUDGE_AGL_FT {
                    judge(
                        &mut self.ias,
                        s.ias_kts - self.targets.ias_kts.unwrap_or(s.ias_kts),
                        &mut lines,
                    );
                }
                if self.lifted && s.agl_ft >= TAKEOFF_TOP_AGL_FT {
                    self.complete = true;
                    lines.push("A thousand feet. That completes the takeoff.".into());
                }
            }
            Task::StraightAndLevel | Task::SlowFlight => {
                judge(
                    &mut self.alt,
                    s.alt_ft - self.targets.alt_ft.unwrap_or(s.alt_ft),
                    &mut lines,
                );
                judge(&mut self.hdg, hdg_dev(self.targets.hdg_deg), &mut lines);
                judge(
                    &mut self.ias,
                    s.ias_kts - self.targets.ias_kts.unwrap_or(s.ias_kts),
                    &mut lines,
                );
                if self.task == Task::SlowFlight {
                    judge(&mut self.bank, s.bank_deg.abs(), &mut lines);
                }
                let want = if self.task == Task::SlowFlight {
                    SLOW_FLIGHT_S
                } else {
                    STRAIGHT_LEVEL_S
                };
                if self.elapsed_s() >= want {
                    self.complete = true;
                    lines.push("That's time.".into());
                }
            }
            Task::Climb | Task::Descent => {
                judge(&mut self.hdg, hdg_dev(self.targets.hdg_deg), &mut lines);
                let moving = if self.task == Task::Climb {
                    climbing
                } else {
                    descending
                };
                if moving {
                    self.level_s = 0.0;
                    self.level_wrong_s = 0.0;
                    judge(
                        &mut self.ias,
                        s.ias_kts - self.targets.ias_kts.unwrap_or(s.ias_kts),
                        &mut lines,
                    );
                } else if level && self.elapsed_s() > LEVEL_OFF_HOLD_S {
                    let target = self.targets.alt_ft.unwrap_or(s.alt_ft);
                    let dev = s.alt_ft - target;
                    if dev.abs() <= 300.0 {
                        self.level_s += dt;
                        judge(&mut self.alt, dev, &mut lines);
                        if self.level_s >= LEVEL_OFF_HOLD_S {
                            self.complete = true;
                            self.notes.push(format!(
                                "Levelled at {}, {:.0} {} the assigned altitude.",
                                feet(round_to(s.alt_ft, 10.0)),
                                round_to(dev.abs(), 10.0),
                                if dev >= 0.0 { "above" } else { "below" }
                            ));
                            lines.push("Level. That completes the task.".into());
                        }
                    } else {
                        self.level_wrong_s += dt;
                        if self.level_wrong_s >= 4.0 * LEVEL_OFF_HOLD_S {
                            self.complete = true;
                            self.task_failed = true;
                            self.notes.push(format!(
                                "Levelled at {}, assigned {}.",
                                feet(round_to(s.alt_ft, 10.0)),
                                feet(target)
                            ));
                            lines.push(format!(
                                "You've levelled at {}. The assignment was {}.",
                                feet(round_to(s.alt_ft, 100.0)),
                                feet(target)
                            ));
                        }
                    }
                }
            }
            Task::SteepTurns => {
                judge(
                    &mut self.alt,
                    s.alt_ft - self.targets.alt_ft.unwrap_or(s.alt_ft),
                    &mut lines,
                );
                judge(
                    &mut self.ias,
                    s.ias_kts - self.targets.ias_kts.unwrap_or(s.ias_kts),
                    &mut lines,
                );
                let bank = s.bank_deg.abs();
                if self.in_turn {
                    // The bank is judged in the turn; the roll-out itself
                    // is judged by its heading, not as a shallow bank.
                    if bank >= TURN_OUT_BANK_DEG {
                        judge(
                            &mut self.bank,
                            bank - self.targets.bank_deg.unwrap_or(bank),
                            &mut lines,
                        );
                    }
                    if bank < TURN_OUT_BANK_DEG {
                        self.in_turn = false;
                        self.turns_done += 1;
                        let dev = hdg_dev(self.targets.hdg_deg);
                        if let Some(h) = self.hdg.as_mut() {
                            // The roll-out heading is judged once per turn:
                            // one second of weight so the grade reads the
                            // roll-outs, not the turn itself.
                            if let Some(l) = h.feed(dev, 1.0, t) {
                                lines.push(l);
                            }
                        }
                        self.notes.push(format!(
                            "Turn {} rolled out {:.0} degrees {} of heading.",
                            self.turns_done,
                            dev.abs().round(),
                            if dev >= 0.0 { "right" } else { "left" }
                        ));
                        if self.turns_done >= 2 {
                            self.complete = true;
                            lines.push("And that's both turns.".into());
                        } else {
                            lines.push("One. Now the other way.".into());
                        }
                    }
                } else if bank > TURN_IN_BANK_DEG {
                    self.in_turn = true;
                }
            }
            Task::Pattern => {
                let tpa = self.targets.alt_ft.unwrap_or(s.alt_ft);
                if !s.on_ground && (s.alt_ft - tpa).abs() < PATTERN_BAND_FT {
                    if !self.reached_pattern {
                        self.reached_pattern = true;
                        lines.push("Pattern altitude.".into());
                    }
                    if level {
                        judge(&mut self.alt, s.alt_ft - tpa, &mut lines);
                        judge(
                            &mut self.ias,
                            s.ias_kts - self.targets.ias_kts.unwrap_or(s.ias_kts),
                            &mut lines,
                        );
                    }
                }
                self.landing_tick(s, &prev, &mut lines);
            }
            Task::Landing => {
                self.landing_tick(s, &prev, &mut lines);
            }
        }
        lines
    }

    /// The final and the touchdown, shared by the pattern and the landing.
    fn landing_tick(&mut self, s: &Sample, prev: &Sample, lines: &mut Vec<String>) {
        let dt = (s.t_s - prev.t_s).clamp(0.0, 5.0);
        if !s.on_ground {
            self.sink_window
                .retain(|(t, _)| s.t_s - *t <= TOUCHDOWN_WINDOW_S);
            self.sink_window.push((s.t_s, s.vsi_fpm));
        }
        let on_final = !s.on_ground && s.agl_ft < FINAL_AGL_FT && s.vsi_fpm < -100.0;
        if on_final {
            // The pattern's speed axis is the pattern speed; on final the
            // approach speed applies. Same slot, retargeted once.
            if self.task == Task::Pattern {
                if let Some(app) = self.landing_kts {
                    if let Some(a) = self.ias.as_mut() {
                        if a.target != app {
                            a.target = app;
                            a.tol_minus = 5.0;
                        }
                    }
                }
            }
            if let Some(a) = self.ias.as_mut() {
                if let Some(l) = a.feed(s.ias_kts - a.target, dt, s.t_s) {
                    lines.push(l);
                }
            }
        }
        if s.on_ground && !prev.on_ground {
            let sink = touchdown_sink_fpm(&self.sink_window, s.t_s, s.vsi_fpm).abs();
            let drift = s.track_deg.map(|tr| crate::angles::wrap180(s.hdg_deg - tr).abs());
            let mut note = format!(
                "Touchdown at {:.0} feet per minute, {:.0} knots",
                sink.round(),
                prev.ias_kts.round()
            );
            if let Some(d) = drift {
                if d > DRIFT_DEG {
                    note.push_str(&format!(", {d:.0} degrees of drift"));
                } else {
                    note.push_str(", no drift");
                }
            }
            note.push('.');
            if sink > FIRM_TOUCHDOWN_FPM {
                note.push_str(" Firm.");
                self.task_failed = true;
            }
            if drift.is_some_and(|d| d > DRIFT_DEG) {
                self.task_failed = true;
            }
            self.notes.push(note);
            self.complete = true;
            lines.push("Down. That completes the task.".into());
        }
    }

    pub fn grade(&self) -> Grade {
        let mut axes: Vec<AxisGrade> = [&self.alt, &self.hdg, &self.ias, &self.bank, &self.pace]
            .into_iter()
            .flatten()
            .map(AxisScore::grade)
            .collect();
        let mut notes = self.notes.clone();
        let mut task_failed = self.task_failed;
        if let Some(a) = &self.line {
            let (g, cut) = self.line_grade(a);
            if cut > crate::scorecard::TAXI_MAX_CUT_M {
                notes.push(format!("Cut a corner by {cut:.0} metres."));
                task_failed = true;
            }
            if self.left_route {
                notes.push("Left the cleared route.".into());
                task_failed = true;
            }
            axes.push(g);
        }
        let pass = self.complete
            && !task_failed
            && axes.iter().filter(|a| a.judged_s > 0.0).all(|a| a.pass);
        Grade {
            task: self.task,
            level: self.level,
            axes,
            notes,
            complete: self.complete,
            pass,
        }
    }

    /// The taxi's centreline for one tick: judged while moving on the
    /// cleared route, from the first tick inside the tolerance, short of the
    /// route's end. The offset goes to the scorecard's `LineJudge` (the
    /// grade's numbers); the narration hears it too, except inside a corner
    /// being turned through within `TAXI_MAX_CUT_M` (a cut is not an
    /// excursion) and off the route altogether (the guide and the tower own
    /// that; the grade fails it).
    fn line_tick(&mut self, s: &Sample, prev: &Sample, dt: f64) -> Option<String> {
        let tl = s.taxi?;
        let axis = self.line.as_mut()?;
        let gs = s.gs_kts.abs();
        let yaw = if dt > 0.0 {
            crate::angles::wrap180(s.hdg_deg - prev.hdg_deg) / dt
        } else {
            0.0
        };
        let cutting =
            self.line_judge
                .corner(s.t_s, gs, tl.xtk_m, Some(tl.remaining_m), tl.turn, yaw);
        let now = self.line_judge.open_vertex();
        if let (Some(was_vertex), None) = (self.corner_state, now) {
            if !was_vertex {
                self.curve_out_s += self.corner_out_s;
            }
            self.corner_out_s = 0.0;
        }
        self.corner_state = now;
        if !s.on_ground || gs <= TAXI_MOVING_KTS || tl.remaining_m < LINE_END_M {
            return None;
        }
        if tl.off_route {
            if self.line_captured {
                self.left_route = true;
            }
            return None;
        }
        if !self.line_captured {
            if tl.xtk_m.abs() > axis.tol_plus {
                return None;
            }
            self.line_captured = true;
        }
        self.line_judge.judge(tl.xtk_m, cutting);
        if cutting && tl.xtk_m.abs() > axis.tol_plus {
            self.corner_out_s += dt;
        }
        let dev = if cutting && tl.xtk_m.abs() <= crate::scorecard::TAXI_MAX_CUT_M {
            0.0
        } else {
            tl.xtk_m
        };
        axis.feed(dev, dt, s.t_s)
    }

    /// The centreline's grade: the narrated axis's time outside, the
    /// judge's worst offset (a drawn curve's inside included) with the side
    /// the axis saw, and the deepest vertex cut.
    fn line_grade(&self, a: &AxisScore) -> (AxisGrade, f64) {
        let mut judge = self.line_judge.clone();
        judge.flush();
        let mut g = a.grade();
        let side = if g.worst < 0.0 { -1.0 } else { 1.0 };
        let worst = g.worst.abs().max(judge.max_lat);
        g.worst = side * worst;
        let open_curve = self.line_judge.open_vertex() == Some(false);
        g.out_s += self.curve_out_s + if open_curve { self.corner_out_s } else { 0.0 };
        g.pass =
            g.judged_s > 0.0 && worst <= g.tol_plus * GROSS_MULT && g.out_s <= EXCURSION_ALLOW_S;
        (g, judge.max_cut)
    }
}

// ---------------------------------------------------------------- the lesson

/// Where a lesson stands.
#[derive(Debug)]
enum Stage {
    Idle,
    /// Briefed, waiting for "begin".
    Briefed {
        task: Task,
    },
    Flying {
        scorer: Box<Scorer>,
    },
    /// The full flight from the ramp, step by step.
    Syllabus(Box<Syllabus>),
    /// "Show me": the crew's automation flies the task, the instructor
    /// narrates, then hands back to the brief.
    Demonstrating(Box<Demo>),
}

struct Lesson {
    stage: Stage,
    level: Level,
    refs: Refs,
    last_grade: Option<Grade>,
    /// The latest sample the monitor handed over, for the brief's numbers.
    latest: Option<Sample>,
    /// The words a "show me" hands the main loop to route through the tool
    /// door (the task's own request phrase), taken once.
    pending_command: Option<&'static str>,
}

static LESSON: Mutex<Lesson> = Mutex::new(Lesson {
    stage: Stage::Idle,
    level: Level::Private,
    refs: Refs {
        vy_kts: None,
        vr_kts: None,
        taxi_kts: DEFAULT_TAXI_KTS,
        approach_kts: None,
        cruise_ias_kts: None,
        va_kts: None,
        slow_kts: None,
        pattern_agl_ft: 1000.0,
        pattern_kts: None,
    },
    last_grade: None,
    latest: None,
    pending_command: None,
});

fn lesson() -> std::sync::MutexGuard<'static, Lesson> {
    LESSON.lock().unwrap_or_else(|e| e.into_inner())
}

/// Republish the airframe's numbers on every swap, and end any lesson: a
/// lesson's targets belong to the airframe that was briefed.
pub fn install(profile: &AircraftProfile) {
    let refs = Refs::from_profile(profile);
    abandon("airframe_swap");
    let mut l = lesson();
    l.refs = refs;
    l.latest = None;
}

/// What the pilot asked the instructor for.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Request {
    /// "Teach me steep turns."
    Start(Task),
    /// "Begin." (only while briefed)
    Begin,
    /// "End the lesson." / "Stop the lesson."
    End,
    /// "How am I doing?"
    Interim,
    /// "Say the standards again."
    Standards,
    /// "Commercial standards." / "Private standards."
    SetLevel(Level),
    /// "What can you teach?"
    Menu,
    /// "Teach me a full flight" / "a lesson from cold and dark".
    FullFlight,
    /// "Show me" (the briefed or flying task) / "show me a takeoff".
    ShowMe(Option<Task>),
    /// "My controls" while the instructor demonstrates.
    TakeBack,
}

fn words(text: &str) -> Vec<String> {
    text.to_ascii_lowercase()
        .split(|c: char| !c.is_ascii_alphanumeric() && c != '\'')
        .filter(|w| !w.is_empty())
        .map(|w| w.replace('\'', ""))
        .collect()
}

fn task_named(t: &str) -> Option<Task> {
    let t = t.to_ascii_lowercase();
    let has = |p: &str| t.contains(p);
    if has("straight and level") || has("straight-and-level") || has("level flight") {
        return Some(Task::StraightAndLevel);
    }
    if has("steep turn") {
        return Some(Task::SteepTurns);
    }
    if has("slow flight") {
        return Some(Task::SlowFlight);
    }
    if has("traffic pattern") || has("the pattern") || has("a pattern") || has("circuit") {
        return Some(Task::Pattern);
    }
    if has("landing") || has("approach") {
        return Some(Task::Landing);
    }
    if has("climb") {
        return Some(Task::Climb);
    }
    if has("descent") || has("descend") {
        return Some(Task::Descent);
    }
    None
}

/// Classify an utterance as a lesson command. Closed vocabulary: a lesson
/// starts only from a teaching verb plus a task name ("teach me steep
/// turns", "let's practice slow flight", "lesson on the pattern"); "begin"
/// counts only while a brief is standing; everything else is not ours and
/// routes on ("climb to four thousand" is a command, not a lesson).
pub fn request(text: &str) -> Option<Request> {
    let w = words(text);
    if w.is_empty() || w.len() > 12 {
        return None;
    }
    let joined = w.join(" ");
    let has = |p: &str| joined.contains(p);
    let has_word = |x: &str| w.iter().any(|y| y == x);
    let teaching = has("teach me")
        || has("lesson")
        || has("practice")
        || has("practise")
        || has("instruct me")
        || has("coach me")
        || has("drill")
        || has("exercise");
    if teaching && (has("what can you teach") || has("which lessons") || has("what lessons")) {
        return Some(Request::Menu);
    }
    if has("what can you teach") || has("list the lessons") {
        return Some(Request::Menu);
    }
    if has_word("commercial") && (has("standard") || teaching) {
        return Some(Request::SetLevel(Level::Commercial));
    }
    if has_word("private") && (has("standard") || teaching) {
        return Some(Request::SetLevel(Level::Private));
    }
    if has("end the lesson")
        || has("stop the lesson")
        || has("end lesson")
        || has("stop lesson")
        || has("thats enough")
        || has("that's enough")
        || has("debrief me")
        || has("give me the debrief")
        || has("cancel the lesson")
    {
        return Some(Request::End);
    }
    // The demonstration's words. A take-back answers only a demonstration
    // in progress (elsewhere "my controls" is the crew's release path); a
    // "show me" only a lesson standing, or, naming a task, instructor mode
    // ("show me the weather" in any mode is not ours).
    let stage_kind = lesson_stage_kind();
    if stage_kind == StageKind::Demonstrating && take_back_words(&joined) {
        return Some(Request::TakeBack);
    }
    if show_me_words(&joined, &w) {
        let stands = stage_kind != StageKind::Idle;
        match demo_task_named(&joined) {
            Some(task) if stands || crate::mode::instructing() => {
                return Some(Request::ShowMe(Some(task)));
            }
            None if stands && w.iter().all(|x| SHOW_ME_BARE.contains(&x.as_str())) => {
                return Some(Request::ShowMe(None));
            }
            _ => {}
        }
    }
    if has("how am i doing")
        || has("how was that")
        || has("how did i do")
        || has("grade me")
        || has("am i within standards")
        || has("am i in standards")
        || has("how is that looking")
    {
        return Some(Request::Interim);
    }
    if (has("standards") || has("the numbers") || has("the brief"))
        && (has("again") || has("repeat") || has("say") || has("what are"))
    {
        return Some(Request::Standards);
    }
    if teaching {
        if has("full flight")
            || has("whole flight")
            || has("cold and dark")
            || has("cold dark")
            || has("from the ramp")
            || has("gate to gate")
            || has("complete flight")
        {
            return Some(Request::FullFlight);
        }
        if let Some(task) = task_named(&joined) {
            return Some(Request::Start(task));
        }
        return Some(Request::Menu);
    }
    // "Begin" and friends only answer a standing brief, so a bare "begin"
    // said to nobody in particular stays unrecognised.
    let briefed = matches!(lesson().stage, Stage::Briefed { .. });
    if briefed {
        const BEGIN: &[&str] = &[
            "begin",
            "beginning",
            "ready",
            "go",
            "ahead",
            "starting",
            "start",
            "now",
            "lets",
            "ok",
            "okay",
            "im",
            "i",
            "am",
            "the",
            "lesson",
            "task",
            "yes",
            "roger",
        ];
        let any_begin = has_word("begin")
            || has_word("beginning")
            || has_word("ready")
            || has_word("starting")
            || has_word("start")
            || has("go ahead")
            || has("lets go");
        if any_begin && w.iter().all(|x| BEGIN.contains(&x.as_str())) {
            return Some(Request::Begin);
        }
    }
    None
}

// ------------------------------------------------------- the demonstration

/// A demonstration whose automation has not started this long after "show
/// me" did not start (the tool refused, and said why): the lesson resumes.
pub const DEMO_START_S: f64 = 30.0;
/// The automation's loop has been off this long: the demonstration is over.
pub const DEMO_SETTLE_S: f64 = 2.0;
/// The final and flare cues of a demonstrated landing (feet AGL).
pub const DEMO_SHORT_FINAL_AGL_FT: f64 = 500.0;
pub const DEMO_FLARE_AGL_FT: f64 = 30.0;

/// The words a bare "show me" may carry and still be one (a lesson
/// standing): anything else is some other question.
const SHOW_ME_BARE: &[&str] = &[
    "show",
    "me",
    "how",
    "demonstrate",
    "demonstration",
    "a",
    "the",
    "it",
    "that",
    "this",
    "please",
    "can",
    "could",
    "would",
    "you",
    "do",
    "one",
    "first",
    "give",
    "again",
    "instructor",
    "let",
    "see",
    "watch",
    "its",
    "done",
    "to",
    "fly",
    "i",
    "want",
    "like",
    "lets",
    "have",
];

fn show_me_words(joined: &str, w: &[String]) -> bool {
    joined.contains("show me")
        || w.iter().any(|x| x == "demonstrate" || x == "demonstration")
        || joined.contains("let me see you")
        || joined.contains("let me watch you")
}

fn take_back_words(joined: &str) -> bool {
    [
        "my controls",
        "my control",
        "i have control",
        "i have the controls",
        "i have the aircraft",
        "my aircraft",
        "stop the demonstration",
        "stop the demo",
        "end the demonstration",
        "let me fly",
        "ill take it",
        "i will take it",
    ]
    .iter()
    .any(|p| joined.contains(p))
}

/// A task named in a "show me": the two ground tasks by their own words,
/// then the lesson names.
fn demo_task_named(t: &str) -> Option<Task> {
    if t.contains("takeoff") || t.contains("take off") {
        return Some(Task::Takeoff);
    }
    if t.contains("taxi") {
        return Some(Task::Taxi);
    }
    task_named(t)
}

impl Task {
    /// The crew automation that demonstrates the task: its tool and the
    /// panel's request phrase for it ([`crate::auto_status::REQUEST_PHRASES`],
    /// grammar-pinned), so a demonstration walks the same door a button
    /// does. `None`: nothing of the crew's flies it (air work is hands).
    pub fn demonstration(self) -> Option<(&'static str, &'static str)> {
        let (tool, code) = match self {
            Task::Taxi => ("auto_taxi", 7),
            Task::Takeoff => ("auto_takeoff", 9),
            Task::Pattern => ("full_auto", 10),
            Task::Landing => ("autoland", 2),
            _ => return None,
        };
        crate::auto_status::REQUEST_PHRASES
            .iter()
            .find(|(c, _)| *c == code)
            .map(|(_, phrase)| (tool, *phrase))
    }

    /// Where the demonstration can start: the lesson's own entry, and the
    /// pattern from the ground (the crew's circuit is full auto, which
    /// never arms airborne).
    fn demo_entry_ok(self, s: &Sample) -> Result<(), &'static str> {
        match self {
            Task::Pattern if !s.on_ground => {
                Err("I fly that one from the ground. Ask me to show you a landing instead")
            }
            _ => self.entry_ok(s),
        }
    }
}

/// The lesson's stage, by kind (for the word classifier).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum StageKind {
    Idle,
    Briefed,
    Flying,
    Syllabus,
    Demonstrating,
}

fn lesson_stage_kind() -> StageKind {
    match lesson().stage {
        Stage::Idle => StageKind::Idle,
        Stage::Briefed { .. } => StageKind::Briefed,
        Stage::Flying { .. } => StageKind::Flying,
        Stage::Syllabus(_) => StageKind::Syllabus,
        Stage::Demonstrating(_) => StageKind::Demonstrating,
    }
}

/// One demonstration: the automation flies, the instructor narrates, and
/// the task's own scorer grades it as the INSTRUCTOR's (blackboxed
/// `demo_end` with `by: instructor`, never the student's `last_grade`).
#[derive(Debug)]
struct Demo {
    task: Task,
    tool: &'static str,
    targets: Targets,
    started_t: f64,
    last_t: Option<f64>,
    /// The automation's loop has been seen running.
    seen: bool,
    /// Seconds the loop has been off since it was seen.
    quiet_s: f64,
    /// Airborne at some point of the demonstration (the touchdown cue).
    airborne_seen: bool,
    said: Vec<&'static str>,
    scorer: Scorer,
}

/// How a demonstration ended by itself.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum DemoEnd {
    /// The automation finished the task.
    Complete,
    /// The automation never started (its tool refused, and said why).
    NotStarted,
}

impl DemoEnd {
    fn key(self) -> &'static str {
        match self {
            DemoEnd::Complete => "complete",
            DemoEnd::NotStarted => "not_started",
        }
    }
}

impl Demo {
    fn new(
        task: Task,
        tool: &'static str,
        entry: &Sample,
        refs: &Refs,
        level: Level,
        now_s: f64,
    ) -> Demo {
        let targets = task.assign(entry, refs, level);
        Demo {
            task,
            tool,
            targets,
            started_t: now_s,
            last_t: None,
            seen: false,
            quiet_s: 0.0,
            airborne_seen: !entry.on_ground,
            said: Vec::new(),
            scorer: Scorer::new(task, level, targets).with_landing_speed(refs.approach_kts),
        }
    }

    /// One tick: `running` is the automation's loop flag. Returns the
    /// narration to speak and, once, how the demonstration ended.
    fn tick(&mut self, s: &Sample, running: bool) -> (Vec<String>, Option<DemoEnd>) {
        let dt = self
            .last_t
            .map(|t| (s.t_s - t).clamp(0.0, 5.0))
            .unwrap_or(0.0);
        self.last_t = Some(s.t_s);
        let mut lines = Vec::new();
        if running {
            self.seen = true;
            self.quiet_s = 0.0;
        } else if self.seen {
            self.quiet_s += dt;
        }
        if !self.seen {
            if s.t_s - self.started_t >= DEMO_START_S {
                return (lines, Some(DemoEnd::NotStarted));
            }
            return (lines, None);
        }
        // The instructor's own scorer: its excursion lines are for a
        // student, so they stay in the grade, never on the intercom.
        let _ = self.scorer.tick(s);
        if !s.on_ground {
            self.airborne_seen = true;
        }
        if running {
            if let Some(line) = self.cue(s) {
                lines.push(line);
            }
        }
        if self.quiet_s >= DEMO_SETTLE_S {
            return (lines, Some(DemoEnd::Complete));
        }
        (lines, None)
    }

    /// The next teaching point the aircraft has reached, each said once.
    fn cue(&mut self, s: &Sample) -> Option<String> {
        let t = self.targets;
        let gs = s.gs_kts.abs();
        let air = !s.on_ground;
        let touchdown = s.on_ground && self.airborne_seen;
        let flare = air && self.airborne_seen && s.agl_ft < DEMO_FLARE_AGL_FT && s.vsi_fpm < 0.0;
        let knots_or = |k: Option<f64>, what: &str| match k {
            Some(k) => knots(k),
            None => what.to_string(),
        };
        let said = |k: &str| self.said.iter().any(|x| *x == k);
        let cues: Vec<(&'static str, bool, String)> = match self.task {
            Task::Taxi => vec![
                (
                    "taxi_start",
                    true,
                    format!(
                        "A little power to get moving, then back to idle. A steady {} on the straights, the nosewheel on the yellow line.",
                        knots_or(t.pace_kts, "walking pace")
                    ),
                ),
                (
                    "taxi_brakes",
                    gs > TAXI_MOVING_KTS,
                    "Moving. Brakes checked, first thing.".into(),
                ),
                (
                    "taxi_turn",
                    s.taxi.and_then(|l| l.turn).is_some(),
                    "Turn coming. Slow first, then steer as the nosewheel reaches the line, not before."
                        .into(),
                ),
                (
                    "taxi_hold",
                    s.taxi.is_some_and(|l| l.remaining_m < 60.0) && gs > TAXI_MOVING_KTS,
                    "Hold short line ahead. Power off early and let it roll to the line.".into(),
                ),
            ],
            Task::Takeoff => vec![
                (
                    "takeoff_start",
                    true,
                    match t.vr_kts {
                        Some(vr) => format!(
                            "On the centreline, heels on the floor, full power smoothly. Rotation at {}.",
                            knots(vr)
                        ),
                        None => "On the centreline, heels on the floor, full power smoothly."
                            .into(),
                    },
                ),
                (
                    "takeoff_roll",
                    s.on_ground && gs > ROLL_KTS,
                    "Power set, gauges in the green, airspeed alive. Rudder holds the centreline."
                        .into(),
                ),
                (
                    "takeoff_rotate",
                    s.on_ground && t.vr_kts.is_some_and(|vr| s.ias_kts >= vr - 3.0),
                    "Rotate. Gentle back pressure, and let it fly off.".into(),
                ),
                (
                    "takeoff_climb",
                    air && s.agl_ft >= CLIMB_JUDGE_AGL_FT,
                    format!(
                        "Positive rate. Pitch for {}, runway heading, and trim.",
                        knots_or(t.ias_kts, "the climb speed")
                    ),
                ),
            ],
            Task::Pattern => vec![
                (
                    "pattern_start",
                    true,
                    "The whole pattern: takeoff, pattern altitude, then power and pitch on every leg."
                        .into(),
                ),
                (
                    "pattern_lift",
                    air,
                    "Airborne. Climbing out on the runway heading.".into(),
                ),
                (
                    "pattern_alt",
                    air && t.alt_ft.is_some_and(|a| s.alt_ft >= a - 150.0),
                    format!(
                        "Pattern altitude{}. Power back, {} on the downwind.",
                        t.alt_ft.map(|a| format!(", {}", feet(a))).unwrap_or_default(),
                        knots_or(t.ias_kts, "pattern speed")
                    ),
                ),
                (
                    "pattern_descent",
                    air && said("pattern_alt") && s.vsi_fpm < -300.0,
                    "Power back abeam the numbers, and down we come. Base when the runway is over my shoulder."
                        .into(),
                ),
                (
                    "short_final",
                    air && said("pattern_descent") && s.agl_ft < DEMO_SHORT_FINAL_AGL_FT,
                    "Final. The aim point stays still in the windscreen, the speed on its number."
                        .into(),
                ),
                (
                    "flare",
                    flare,
                    "Power off. Hold it off, hold it off.".into(),
                ),
                (
                    "touchdown",
                    touchdown,
                    "Touchdown, main wheels first. Straight down the centreline.".into(),
                ),
            ],
            Task::Landing => vec![
                (
                    "landing_start",
                    true,
                    format!(
                        "A stabilised approach: the aim point fixed, {}, small corrections early.",
                        knots_or(t.ias_kts, "the approach speed")
                    ),
                ),
                (
                    "short_final",
                    air && s.agl_ft < DEMO_SHORT_FINAL_AGL_FT,
                    "Short final. Still on the speed, still on the aim point.".into(),
                ),
                (
                    "flare",
                    flare,
                    "Power off. Hold it off, hold it off.".into(),
                ),
                (
                    "touchdown",
                    touchdown,
                    "Touchdown, main wheels first. Straight down the centreline, brakes gently."
                        .into(),
                ),
            ],
            _ => Vec::new(),
        };
        let (key, _, line) = cues.into_iter().find(|(k, due, _)| *due && !said(k))?;
        self.said.push(key);
        Some(line)
    }
}

/// Is the demonstrated task's automation flying?
#[cfg(not(test))]
fn demo_loop_running(task: Task) -> bool {
    // owners-by-name: each task demonstrates its own loop.
    use std::sync::atomic::Ordering::Relaxed;
    match task {
        Task::Taxi => crate::autotaxi::ACTIVE.load(Relaxed),
        Task::Takeoff => crate::autotakeoff::running() || crate::autotakeoff::ACTIVE.load(Relaxed),
        Task::Pattern => {
            crate::full_auto::running()
                || crate::full_auto::ACTIVE.load(Relaxed)
                || crate::circuit::ACTIVE.load(Relaxed)
        }
        Task::Landing => crate::autoland::ACTIVE.load(Relaxed),
        _ => false,
    }
}

/// The tests' loop flag: the real ones belong to the loops' own tests.
#[cfg(test)]
static TEST_DEMO_RUNNING: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
#[cfg(test)]
static TEST_DEMO_STOPPED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

#[cfg(test)]
fn demo_loop_running(_: Task) -> bool {
    TEST_DEMO_RUNNING.load(std::sync::atomic::Ordering::SeqCst)
}

/// The pilot takes the aircraft back mid-demonstration: the loop stands
/// down the way the crew's release path stands it down ("my controls"): a
/// takeoff hands over with the power set, a taxi lets go of the speed, a
/// circuit or a landing gives back a flyable aircraft.
#[cfg(not(test))]
fn stop_demo_loop(task: Task) {
    use std::sync::atomic::Ordering::SeqCst;
    match task {
        Task::Taxi => {
            crate::autotaxi::ACTIVE.store(false, SeqCst);
            crate::tools::speedhold::ACTIVE.store(false, SeqCst);
        }
        Task::Takeoff => crate::autotakeoff::cancel(),
        Task::Pattern => {
            crate::full_auto::abort();
            crate::circuit::abort();
        }
        Task::Landing => {
            crate::autoland::abort();
        }
        _ => {}
    }
    crate::flight::note_pilot_takeover("my controls");
}

#[cfg(test)]
fn stop_demo_loop(_: Task) {
    TEST_DEMO_STOPPED.store(true, std::sync::atomic::Ordering::SeqCst);
}

/// The words a "show me" hands the main loop, once: route them on to the
/// tool door, where [`crate::mode::tool_may_actuate`] lets exactly this
/// demonstration's tool through.
pub fn take_demo_command() -> Option<&'static str> {
    lesson().pending_command.take()
}

/// The hand-back line: the controls, and how the lesson goes on.
fn hand_back_line(task: Task, s: Option<&Sample>, lead: &str) -> String {
    let next = match s.map(|s| task.entry_ok(s)) {
        Some(Ok(())) => "Say begin when you're ready, and fly it the same way.",
        _ => "Set up for it again and say begin.",
    };
    format!("{lead}Your controls. {next}")
}

/// Close a demonstration: the permit shuts, the instructor's grade is
/// blackboxed as the instructor's, and the lesson stands briefed on the
/// same task (`resume`) or ends.
fn end_demo(l: &mut Lesson, demo: Box<Demo>, reason: &str, resume: bool) {
    crate::mode::close_demonstration();
    l.pending_command = None;
    let grade = demo.seen.then(|| demo.scorer.grade().json());
    crate::blackbox::record(
        "instructor",
        json!({
            "event": "demo_end", "task": demo.task.key(), "tool": demo.tool,
            "reason": reason, "by": "instructor", "flown": demo.seen,
            "cues": demo.said, "grade": grade,
        }),
    );
    l.stage = if resume {
        Stage::Briefed { task: demo.task }
    } else {
        Stage::Idle
    };
}

/// "Show me": hand the aircraft to the crew's automation for the task.
fn show_me(l: &mut Lesson, task: Option<Task>, now_s: f64) -> String {
    let task = match (&l.stage, task) {
        (Stage::Demonstrating(d), _) => {
            return format!("I'm showing you {} now. Watch.", d.task.name());
        }
        (Stage::Syllabus(_), _) => {
            return "Not in the full flight. Say end the lesson first, then show me, and a task."
                .into();
        }
        (_, Some(t)) => t,
        (Stage::Briefed { task }, None) => *task,
        (Stage::Flying { scorer }, None) => scorer.task(),
        (Stage::Idle, None) => return "Show you what? Say show me, and a task.".into(),
    };
    let Some((tool, phrase)) = task.demonstration() else {
        return format!(
            "I can't demonstrate {}: nothing of the crew's flies it, it's all hands. I can show you a taxi, a takeoff, the pattern or a landing.",
            task.name()
        );
    };
    if !crate::mode::instructing() {
        return "Demonstrations run in instructor mode. Flip the mode in Settings and ask again."
            .into();
    }
    let Some(s) = l.latest else {
        return "I can't see the aircraft yet. Give me a moment and ask again.".into();
    };
    if let Err(why) = task.demo_entry_ok(&s) {
        return format!("To show you {}, {why}.", task.name());
    }
    let from = match &l.stage {
        Stage::Idle => "idle",
        Stage::Briefed { .. } => "briefed",
        _ => "flying",
    };
    // A student's attempt in progress is closed as the student's, on the
    // numbers so far: the demonstration is never graded to them.
    if let Stage::Flying { scorer } = &l.stage {
        let grade = scorer.grade();
        crate::blackbox::record(
            "instructor",
            json!({ "event": "end", "reason": "demonstration", "grade": grade.json() }),
        );
        l.last_grade = Some(grade);
    }
    let demo = Demo::new(task, tool, &s, &l.refs, l.level, now_s);
    crate::blackbox::record(
        "instructor",
        json!({
            "event": "demo_start", "task": task.key(), "tool": tool, "phrase": phrase,
            "from": from, "by": "instructor", "t_s": now_s,
            "alt": demo.targets.alt_ft, "ias": demo.targets.ias_kts,
            "vr": demo.targets.vr_kts, "pace": demo.targets.pace_kts,
        }),
    );
    l.stage = Stage::Demonstrating(Box::new(demo));
    l.pending_command = Some(phrase);
    crate::mode::open_demonstration(tool);
    format!(
        "My controls. I'll show you {}. Watch, and say my controls to take it back.",
        task.name()
    )
}

fn menu() -> String {
    format!(
        "I can teach {}, or a full flight from cold and dark. Say teach me, and the one you want.",
        Task::ALL
            .iter()
            .map(|t| t.name())
            .collect::<Vec<_>>()
            .join(", ")
    )
}

/// Answer a lesson request. `now_s` is the monitor's clock (seconds), the
/// same one [`tick`] is fed with. Returns the line to speak.
/// A lesson word that needs the aircraft back first.
const DEMO_BUSY: &str = "I'm demonstrating. Say my controls first.";

pub fn handle(req: Request, now_s: f64) -> String {
    let mut l = lesson();
    match req {
        Request::Menu => menu(),
        Request::ShowMe(task) => show_me(&mut l, task, now_s),
        Request::TakeBack => {
            let stage = std::mem::replace(&mut l.stage, Stage::Idle);
            let Stage::Demonstrating(demo) = stage else {
                l.stage = stage;
                return "You have it.".into();
            };
            let task = demo.task;
            stop_demo_loop(task);
            end_demo(&mut l, demo, "pilot", true);
            hand_back_line(task, l.latest.as_ref(), "")
        }
        Request::SetLevel(level) => {
            l.level = level;
            crate::blackbox::record(
                "instructor",
                json!({ "event": "level", "level": level.name() }),
            );
            format!("{} standards from here on.", cap_first(level.name()))
        }
        Request::FullFlight => {
            match &l.stage {
                Stage::Flying { scorer } => {
                    return format!(
                        "We're in the middle of {}. Say end the lesson first.",
                        scorer.task().name()
                    );
                }
                Stage::Syllabus(_) => return "We're already on the full flight.".into(),
                Stage::Demonstrating(_) => return DEMO_BUSY.into(),
                _ => {}
            }
            let Some(s) = l.latest else {
                return "I can't see the aircraft yet. Give me a moment and ask again.".into();
            };
            if !s.on_ground {
                return "The full flight starts on the ramp. Land first, or pick an air task."
                    .into();
            }
            let level = l.level;
            let syllabus = Syllabus::new(&s, l.refs.clone(), level);
            let opener = syllabus.opener();
            l.stage = Stage::Syllabus(Box::new(syllabus));
            l.last_grade = None;
            crate::blackbox::record(
                "instructor",
                json!({ "event": "syllabus", "level": level.name(), "engine_running": s.engine_running }),
            );
            opener
        }
        Request::Start(task) => {
            match &l.stage {
                Stage::Flying { scorer } => {
                    return format!(
                        "We're in the middle of {}. Say end the lesson first.",
                        scorer.task().name()
                    );
                }
                Stage::Syllabus(_) => {
                    return "We're on the full flight. Say end the lesson to leave it.".into();
                }
                Stage::Demonstrating(_) => return DEMO_BUSY.into(),
                _ => {}
            }
            let Some(s) = l.latest else {
                return "I can't see the aircraft yet. Give me a moment and ask again.".into();
            };
            if let Err(why) = task.entry_ok(&s) {
                return format!("For {}, {why}.", task.name());
            }
            let targets = task.assign(&s, &l.refs, l.level);
            if targets.ias_kts.is_none()
                && matches!(
                    task,
                    Task::Climb | Task::SlowFlight | Task::Landing | Task::Pattern
                )
            {
                return format!(
                    "I have no book speed for {} on this airframe, so I can't grade it. Pick another task.",
                    task.name()
                );
            }
            let level = l.level;
            l.stage = Stage::Briefed { task };
            l.last_grade = None;
            crate::blackbox::record(
                "instructor",
                json!({
                    "event": "briefed", "task": task.key(), "level": level.name(),
                    "alt": targets.alt_ft, "hdg": targets.hdg_deg, "ias": targets.ias_kts, "bank": targets.bank_deg,
                }),
            );
            task.brief(&targets, level)
        }
        Request::Begin => {
            let Stage::Briefed { task } = l.stage else {
                return "Nothing briefed. Say teach me, and a task.".into();
            };
            let Some(s) = l.latest else {
                return "I can't see the aircraft yet.".into();
            };
            if let Err(why) = task.entry_ok(&s) {
                return format!("Not yet: {why}.");
            }
            // Re-assigned at begin, from where the aircraft IS now: a brief
            // given at 3,000 ft and begun at 3,500 would grade the wrong
            // altitude for the whole task.
            let level = l.level;
            let targets = task.assign(&s, &l.refs, level);
            let mut scorer =
                Scorer::new(task, level, targets).with_landing_speed(l.refs.approach_kts);
            let mut first = s;
            first.t_s = now_s;
            scorer.tick(&first);
            l.stage = Stage::Flying {
                scorer: Box::new(scorer),
            };
            crate::blackbox::record(
                "instructor",
                json!({
                    "event": "begin", "task": task.key(),
                    "alt": targets.alt_ft, "hdg": targets.hdg_deg, "ias": targets.ias_kts, "bank": targets.bank_deg,
                }),
            );
            let mut line = String::from("Your aircraft. Go.");
            if let Some(a) = targets.alt_ft {
                if task == Task::Climb || task == Task::Descent {
                    line = format!(
                        "Your aircraft. {} to {}. Go.",
                        if task == Task::Climb {
                            "Climb"
                        } else {
                            "Descend"
                        },
                        feet(a)
                    );
                }
            }
            line
        }
        Request::Interim => match &l.stage {
            Stage::Flying { scorer } => scorer.grade().spoken(),
            Stage::Syllabus(sy) => sy.interim(),
            Stage::Demonstrating(_) => "I'm flying this one. Watch, and you'll fly it next.".into(),
            _ => match &l.last_grade {
                Some(g) => g.spoken(),
                None => "No lesson running. Say teach me, and a task.".into(),
            },
        },
        Request::Standards => match &l.stage {
            Stage::Briefed { task } => {
                let Some(s) = l.latest else {
                    return "I can't see the aircraft yet.".into();
                };
                task.brief(&task.assign(&s, &l.refs, l.level), l.level)
            }
            Stage::Flying { scorer } => scorer.task().brief(&scorer.targets(), l.level),
            Stage::Syllabus(sy) => sy.standards(),
            Stage::Demonstrating(d) => d.task.brief(&d.targets, l.level),
            Stage::Idle => "No lesson running. Say teach me, and a task.".into(),
        },
        Request::End => {
            let stage = std::mem::replace(&mut l.stage, Stage::Idle);
            match stage {
                Stage::Idle => "No lesson running.".into(),
                Stage::Briefed { task } => {
                    crate::blackbox::record(
                        "instructor",
                        json!({ "event": "end", "task": task.key(), "reason": "cancelled" }),
                    );
                    "Lesson cancelled.".into()
                }
                Stage::Flying { scorer } => {
                    let grade = scorer.grade();
                    crate::blackbox::record(
                        "instructor",
                        json!({ "event": "end", "reason": "pilot", "grade": grade.json() }),
                    );
                    let line = format!("Lesson over. {}", grade.spoken());
                    l.last_grade = Some(grade);
                    line
                }
                Stage::Syllabus(mut sy) => {
                    if let Some(g) = sy.close_current() {
                        crate::blackbox::record(
                            "instructor",
                            json!({ "event": "end", "reason": "pilot", "grade": g.json() }),
                        );
                    }
                    crate::blackbox::record(
                        "instructor",
                        json!({ "event": "syllabus_end", "reason": "pilot" }),
                    );
                    format!("Lesson over. {}", sy.summary())
                }
                Stage::Demonstrating(demo) => {
                    stop_demo_loop(demo.task);
                    end_demo(&mut l, demo, "lesson_ended", false);
                    "Lesson over. Your controls.".into()
                }
            }
        }
    }
}

/// What the full-flight syllabus needs from the checklist flow: the seam
/// is a trait so the syllabus tests run with a fake and the monitor passes
/// [`FlowIo`].
pub trait SyllabusIo {
    /// The installed list for a section key, if the airframe has one.
    fn find_list(&self, key: &str) -> Option<usize>;
    /// Start that list in the Instruct flavour; `false` if one is running.
    fn start_list(&mut self, idx: usize) -> bool;
    /// Is a list running right now?
    fn list_running(&self) -> bool;
}

/// The real flow.
pub struct FlowIo<'a> {
    pub config: &'a crate::config::Config,
    pub voice: &'a crate::voice::Voice,
}

impl SyllabusIo for FlowIo<'_> {
    fn find_list(&self, key: &str) -> Option<usize> {
        crate::flow::find_section(key)
    }
    fn start_list(&mut self, idx: usize) -> bool {
        crate::flow::start(
            idx,
            crate::flow::Flavour::Instruct,
            self.config.clone(),
            self.voice.clone(),
        )
    }
    fn list_running(&self) -> bool {
        crate::flow::ACTIVE.load(std::sync::atomic::Ordering::SeqCst)
    }
}

/// No flow at all (tests of the single-task lessons).
#[cfg(test)]
struct NoIo;

#[cfg(test)]
impl SyllabusIo for NoIo {
    fn find_list(&self, _: &str) -> Option<usize> {
        None
    }
    fn start_list(&mut self, _: usize) -> bool {
        false
    }
    fn list_running(&self) -> bool {
        false
    }
}

/// [`tick_with`] with no checklist flow (the single-task lesson tests).
#[cfg(test)]
pub fn tick(s: Sample) -> Vec<String> {
    tick_with(s, &mut NoIo)
}

/// The monitor's tick: feed the sample, get the lines to speak. A task that
/// completes debriefs itself here and the lesson returns to idle; the
/// syllabus drives the checklist flow through `io`.
pub fn tick_with(s: Sample, io: &mut dyn SyllabusIo) -> Vec<String> {
    let mut l = lesson();
    l.latest = Some(s);
    if let Stage::Demonstrating(demo) = &mut l.stage {
        let running = demo_loop_running(demo.task);
        let (mut lines, end) = demo.tick(&s, running);
        for line in &lines {
            crate::blackbox::record(
                "instructor",
                json!({ "event": "demo_narration", "task": demo.task.key(), "line": line, "t_s": s.t_s, "by": "instructor" }),
            );
        }
        if let Some(end) = end {
            let Stage::Demonstrating(demo) = std::mem::replace(&mut l.stage, Stage::Idle) else {
                unreachable!("matched above");
            };
            let task = demo.task;
            // A loop the pilot cancelled at the tool door ended in their
            // hands, not as a finished demonstration.
            let by_pilot = crate::mode::demonstration_stopped();
            let reason = if by_pilot { "pilot" } else { end.key() };
            end_demo(&mut l, demo, reason, true);
            let lead = match end {
                _ if by_pilot => "",
                DemoEnd::Complete => "That's the demonstration. ",
                DemoEnd::NotStarted => "The demonstration didn't start. ",
            };
            lines.push(hand_back_line(task, Some(&s), lead));
        }
        return lines;
    }
    if let Stage::Syllabus(sy) = &mut l.stage {
        let (lines, done) = sy.tick(&s, io);
        if done {
            crate::blackbox::record(
                "instructor",
                json!({ "event": "syllabus_end", "reason": "complete" }),
            );
            l.stage = Stage::Idle;
        }
        return lines;
    }
    let Stage::Flying { scorer } = &mut l.stage else {
        return Vec::new();
    };
    let mut lines = scorer.tick(&s);
    for line in &lines {
        crate::blackbox::record(
            "instructor",
            json!({ "event": "narration", "task": scorer.task().key(), "line": line, "t_s": s.t_s }),
        );
    }
    if scorer.complete() {
        let grade = scorer.grade();
        crate::blackbox::record(
            "instructor",
            json!({ "event": "end", "reason": "complete", "grade": grade.json() }),
        );
        lines.push(grade.spoken());
        l.last_grade = Some(grade);
        l.stage = Stage::Idle;
    }
    lines
}

/// The lesson clock, seconds since the process started: the monitor's
/// ticks and the main loop's "begin" must share one clock.
pub fn clock_s() -> f64 {
    static START: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
    START
        .get_or_init(std::time::Instant::now)
        .elapsed()
        .as_secs_f64()
}

/// End whatever lesson stands, without a debrief (the mode changed under
/// it, the airframe swapped). Returns whether one was running.
pub fn abandon(reason: &str) -> bool {
    let mut l = lesson();
    if matches!(l.stage, Stage::Idle) {
        return false;
    }
    // A demonstration's permit dies with the lesson; the loop it armed is
    // the caller's (a swap stands every loop down; out of instructor mode
    // the crew owns what flies).
    if let Stage::Demonstrating(d) = &l.stage {
        crate::blackbox::record(
            "instructor",
            json!({ "event": "demo_end", "task": d.task.key(), "tool": d.tool, "reason": reason, "by": "instructor", "flown": d.seen }),
        );
        crate::mode::close_demonstration();
        l.pending_command = None;
    }
    l.stage = Stage::Idle;
    crate::blackbox::record("instructor", json!({ "event": "end", "reason": reason }));
    true
}

/// Is a lesson briefed or flying?
pub fn active() -> bool {
    !matches!(lesson().stage, Stage::Idle)
}

/// Is the full-flight syllabus running? The flow's phase boundaries then
/// run their list in the Instruct flavour at once instead of proposing.
pub fn syllabus_active() -> bool {
    matches!(lesson().stage, Stage::Syllabus(_))
}

/// The panel's manager line.
pub fn status() -> Option<String> {
    let l = lesson();
    match &l.stage {
        Stage::Idle => None,
        Stage::Briefed { task } => {
            Some(format!("lesson: {}, briefed", task.key().replace('_', " ")))
        }
        Stage::Flying { scorer } => Some(format!(
            "lesson: {}, {:.0} s",
            scorer.task().key().replace('_', " "),
            scorer.elapsed_s()
        )),
        Stage::Syllabus(sy) => Some(format!("lesson: full flight, {}", sy.step_name())),
        Stage::Demonstrating(d) => Some(format!(
            "lesson: demonstrating {}",
            d.task.key().replace('_', " ")
        )),
    }
}

/// A refused autopilot-shaped command during a lesson becomes the lesson's
/// target instead ("heading two seven zero" with a lesson flying retargets
/// the heading axis). Returns the line to speak, or `None` when the call is
/// not a target for the running task.
pub fn take_target(tool: &str, args: &Value) -> Option<String> {
    let mut l = lesson();
    let scorer = match &mut l.stage {
        Stage::Flying { scorer } => scorer,
        Stage::Syllabus(sy) => sy.scorer.as_mut()?,
        _ => return None,
    };
    let (axis, value, said) = match tool {
        "set_heading" => {
            let h = args.get("heading")?.as_f64()?;
            (Axis::Heading, crate::angles::wrap360(h), format!("Heading {}.", hdg(h)))
        }
        "set_altitude" | "fly_to" => {
            let a = args.get("altitude")?.as_f64()?;
            (Axis::Altitude, a, format!("{} it is.", cap_first(&feet(a))))
        }
        "hold_speed" => {
            let k = args.get("knots")?.as_f64()?;
            if k <= 0.0 {
                return None;
            }
            (
                Axis::Airspeed,
                k,
                format!("{} it is.", cap_first(&knots(k))),
            )
        }
        _ => return None,
    };
    if scorer.targets().alt_ft.is_none() && axis == Axis::Altitude {
        return None;
    }
    scorer.retarget(axis, value);
    crate::blackbox::record(
        "instructor",
        json!({ "event": "retarget", "axis": axis.name(), "value": value }),
    );
    Some(format!("{said} Your hands."))
}

/// The instructor's answer to a command that would actuate. One honest
/// line naming the action; the pilot does it.
pub fn refusal(label: &str, first_time: bool) -> String {
    let action = label.to_ascii_lowercase().replace(" > ", " ");
    if first_time {
        format!(
            "Instructor mode: I don't touch the controls. {}: that's your hands. Flip the mode in Settings if you want the copilot back.",
            cap_first(&action)
        )
    } else {
        format!("{}: your hands.", cap_first(&action))
    }
}

// ------------------------------------------------------------ the syllabus

/// One step of the full flight.
#[derive(Clone, Debug, PartialEq)]
enum Step {
    /// A checklist in the Instruct flavour, the first key the file has.
    List {
        name: &'static str,
        keys: &'static [&'static str],
    },
    /// Wait for the engine to be running after the start list.
    WaitEngine,
    /// A graded task.
    Fly(Task),
    /// The pilot's own flight; the lists ride the phase boundaries and the
    /// landing is armed on final.
    Enroute,
    /// Stopped after the landing: the shutdown list follows.
    WaitStopped,
}

impl Step {
    fn name(&self) -> String {
        match self {
            Step::List { name, .. } => format!("{name} checklist"),
            Step::WaitEngine => "engine start".into(),
            Step::Fly(t) => t.key().replace('_', " "),
            Step::Enroute => "enroute".into(),
            Step::WaitStopped => "after landing".into(),
        }
    }
}

const STEPS: &[Step] = &[
    Step::List {
        name: "preflight",
        keys: &["preflight", "cockpit preparation", "cold and dark"],
    },
    Step::List {
        name: "before start",
        keys: &[
            "before starting engine",
            "before engine start",
            "before start",
        ],
    },
    Step::List {
        name: "engine start",
        keys: &["starting engine", "engine start"],
    },
    Step::WaitEngine,
    Step::List {
        name: "before taxi",
        keys: &["before taxi", "before taxiing"],
    },
    Step::Fly(Task::Taxi),
    Step::List {
        name: "before takeoff",
        keys: &["before takeoff", "holding point"],
    },
    Step::Fly(Task::Takeoff),
    Step::Enroute,
    Step::Fly(Task::Landing),
    Step::WaitStopped,
    Step::List {
        name: "shutdown",
        keys: &["engine shutdown", "shutdown", "parking", "securing"],
    },
];

/// The full flight from the ramp: the ground lists read and watched, the
/// taxi and takeoff graded, the pilot's own flight with the lists at their
/// phases, the landing graded, the shutdown list. Never writes: every list
/// is Instruct, every task a scorer.
#[derive(Debug)]
pub struct Syllabus {
    i: usize,
    /// The current step has been entered (its line said, its list asked for).
    entered: bool,
    list_idx: Option<usize>,
    list_started: bool,
    /// Ticks seen with the list running, so a list that starts and ends
    /// between two ticks still counts as done once running was observed.
    list_seen_running: bool,
    scorer: Option<Box<Scorer>>,
    grades: Vec<Grade>,
    lists_done: Vec<String>,
    level: Level,
    refs: Refs,
    wait_s: f64,
    nag_s: f64,
    /// The flight has been above the takeoff top (the landing arms only
    /// after that, never on the takeoff's own climb-out).
    was_high: bool,
    last_t: Option<f64>,
}

impl Syllabus {
    fn new(s: &Sample, refs: Refs, level: Level) -> Self {
        // Engine already running: the ramp lists are behind us.
        let i = if s.engine_running {
            STEPS
                .iter()
                .position(|st| {
                    matches!(
                        st,
                        Step::List {
                            name: "before taxi",
                            ..
                        }
                    )
                })
                .unwrap_or(0)
        } else {
            0
        };
        Syllabus {
            i,
            entered: false,
            list_idx: None,
            list_started: false,
            list_seen_running: false,
            scorer: None,
            grades: Vec::new(),
            lists_done: Vec::new(),
            level,
            refs,
            wait_s: 0.0,
            nag_s: 0.0,
            was_high: false,
            last_t: None,
        }
    }

    fn opener(&self) -> String {
        if self.i == 0 {
            "Full flight from cold and dark. I read the lists, you do the items; I call the taxi and grade the taxi, the takeoff and the landing. Your aircraft throughout. Preflight first.".into()
        } else {
            "Full flight from here. Engine's running, so the before taxi list is first, then I grade the taxi, the takeoff and the landing. Your aircraft throughout.".into()
        }
    }

    fn step(&self) -> Option<&'static Step> {
        STEPS.get(self.i)
    }

    fn step_name(&self) -> String {
        self.step().map(Step::name).unwrap_or_else(|| "done".into())
    }

    fn interim(&self) -> String {
        match (self.step(), &self.scorer) {
            (Some(Step::Fly(_)), Some(sc)) => sc.grade().spoken(),
            (Some(Step::List { name, .. }), _) => {
                format!("We're on the {name} checklist. {}", self.summary())
            }
            (Some(Step::Enroute), _) => {
                format!("Your flight; the landing's next. {}", self.summary())
            }
            (Some(Step::WaitEngine), _) => "Waiting for the engine.".into(),
            (Some(Step::WaitStopped), _) => "Taxi in and stop; the shutdown list is next.".into(),
            _ => self.summary(),
        }
    }

    fn standards(&self) -> String {
        match (self.step(), &self.scorer) {
            (Some(Step::Fly(t)), Some(sc)) => t.brief(&sc.targets(), self.level),
            _ => self.opener(),
        }
    }

    /// The grades so far, one verdict each.
    fn summary(&self) -> String {
        if self.grades.is_empty() && self.lists_done.is_empty() {
            return "Nothing graded yet.".into();
        }
        let mut parts: Vec<String> = self
            .grades
            .iter()
            .map(|g| {
                format!(
                    "{}: {}.",
                    cap_first(&g.task.key().replace('_', " ")),
                    if !g.complete {
                        "not completed"
                    } else if g.pass {
                        "to standard"
                    } else {
                        "not to standard"
                    }
                )
            })
            .collect();
        if !self.lists_done.is_empty() {
            parts.push(format!("Lists done: {}.", self.lists_done.join(", ")));
        }
        parts.join(" ")
    }

    /// End the running task's scorer as not completed, for the summary.
    fn close_current(&mut self) -> Option<Grade> {
        let sc = self.scorer.take()?;
        let g = sc.grade();
        self.grades.push(g.clone());
        Some(g)
    }

    fn advance(&mut self) {
        self.i += 1;
        self.entered = false;
        self.list_idx = None;
        self.list_started = false;
        self.list_seen_running = false;
        self.scorer = None;
        self.wait_s = 0.0;
        self.nag_s = 0.0;
    }

    /// One tick. Lines to speak, and whether the flight is complete.
    fn tick(&mut self, s: &Sample, io: &mut dyn SyllabusIo) -> (Vec<String>, bool) {
        let mut lines = Vec::new();
        let dt = self
            .last_t
            .map(|t| (s.t_s - t).clamp(0.0, 5.0))
            .unwrap_or(0.0);
        self.last_t = Some(s.t_s);
        if s.agl_ft > TAKEOFF_TOP_AGL_FT && !s.on_ground {
            self.was_high = true;
        }
        // Enter steps until one is waiting on the aircraft (a missing list
        // is skipped in the same tick).
        for _ in 0..STEPS.len() {
            let Some(step) = self.step() else {
                lines.push(format!("That's the flight. {}", self.summary()));
                return (lines, true);
            };
            if self.entered {
                break;
            }
            self.entered = true;
            match step {
                Step::List { name, keys } => {
                    self.list_idx = keys.iter().find_map(|k| io.find_list(k));
                    match self.list_idx {
                        Some(_) => {
                            lines.push(format!("{} checklist. I'll read, you do.", cap_first(name)))
                        }
                        None => {
                            lines.push(format!(
                                "No {name} list on this airframe. Do it from memory and carry on."
                            ));
                            self.advance();
                            continue;
                        }
                    }
                }
                Step::WaitEngine => {}
                Step::Fly(task) => {
                    let targets = task.assign(s, &self.refs, self.level);
                    let mut sc = Scorer::new(*task, self.level, targets)
                        .with_landing_speed(self.refs.approach_kts);
                    sc.tick(s);
                    crate::blackbox::record(
                        "instructor",
                        json!({
                            "event": "begin", "task": task.key(), "syllabus": true,
                            "alt": targets.alt_ft, "hdg": targets.hdg_deg, "ias": targets.ias_kts, "pace": targets.pace_kts,
                        }),
                    );
                    lines.push(
                        task.brief(&targets, self.level)
                            .replace(" Say begin when you're ready.", ""),
                    );
                    self.scorer = Some(Box::new(sc));
                }
                Step::Enroute => {
                    lines.push("Your flight. I'll run the lists at their phases and take the landing when you're on final.".into());
                }
                Step::WaitStopped => {}
            }
            break;
        }
        let Some(step) = self.step() else {
            return (lines, true);
        };
        match step {
            Step::List { name, .. } => {
                let Some(idx) = self.list_idx else {
                    return (lines, false);
                };
                if !self.list_started {
                    // A list the tracker started may still be running: wait
                    // for the flow, then take it.
                    if io.start_list(idx) {
                        self.list_started = true;
                    }
                    return (lines, false);
                }
                if io.list_running() {
                    self.list_seen_running = true;
                    return (lines, false);
                }
                if self.list_seen_running {
                    self.lists_done.push(name.to_string());
                    crate::blackbox::record(
                        "instructor",
                        json!({ "event": "list_done", "name": name }),
                    );
                    self.advance();
                }
            }
            Step::WaitEngine => {
                if s.engine_running {
                    lines.push("Engine's running.".into());
                    self.advance();
                } else {
                    self.wait_s += dt;
                    self.nag_s += dt;
                    if self.wait_s >= ENGINE_NAG_S && self.nag_s >= ENGINE_NAG_S {
                        self.nag_s = 0.0;
                        lines.push("Engine's not running yet. Start it and I'll carry on.".into());
                    }
                }
            }
            Step::Fly(task) => {
                let task = *task;
                let Some(sc) = self.scorer.as_mut() else {
                    return (lines, false);
                };
                lines.extend(sc.tick(s));
                if sc.complete() {
                    let g = sc.grade();
                    crate::blackbox::record(
                        "instructor",
                        json!({ "event": "end", "reason": "complete", "syllabus": true, "grade": g.json() }),
                    );
                    lines.push(g.spoken());
                    self.grades.push(g);
                    let _ = task;
                    self.advance();
                }
            }
            Step::Enroute => {
                let on_final = !s.on_ground && s.agl_ft < FINAL_AGL_FT && s.vsi_fpm < -100.0;
                if self.was_high && on_final {
                    lines.push("On final. I have the landing.".into());
                    self.advance();
                }
            }
            Step::WaitStopped => {
                if s.on_ground && sim::ground::stopped_kts(s.gs_kts) {
                    self.wait_s += dt;
                    if self.wait_s >= SYLLABUS_STOP_S {
                        self.advance();
                    }
                } else {
                    self.wait_s = 0.0;
                }
            }
        }
        (lines, false)
    }
}

// ------------------------------------------------------ the offline scorer

/// Grade a hand-flown flight from the blackbox's `state` records: level
/// runs as straight and level, sustained climbs and descents, steep turns
/// where the record carries bank, and the final approach with its
/// touchdown. Targets are captured at each segment's entry (rounded the
/// way the brief would round them); the airframe's book speeds come from
/// the profile when one is named. Lines for the debrief.
pub fn grade_flight(
    states: &[&Value],
    profile: Option<&AircraftProfile>,
    level: Level,
) -> Vec<String> {
    let refs = profile.map(Refs::from_profile).unwrap_or_default();
    let f = |v: &Value, k: &str| v.get(k).and_then(Value::as_f64);
    let samples: Vec<Sample> = states
        .iter()
        .filter_map(|e| {
            Some(Sample {
                t_s: f(e, "t")?,
                alt_ft: f(e, "alt")?,
                agl_ft: f(e, "agl")?,
                hdg_deg: f(e, "hdg")?,
                ias_kts: f(e, "ias")?,
                bank_deg: f(e, "bank").unwrap_or(0.0),
                // The flight model's vertical speed, else the gauge (#198).
                vsi_fpm: f(e, "vs").or(f(e, "vsi"))?,
                on_ground: e.get("ground").and_then(Value::as_bool).unwrap_or(true),
                track_deg: f(e, "track"),
                gs_kts: f(e, "gs").unwrap_or(0.0),
                engine_running: e.get("eng").and_then(Value::as_bool).unwrap_or(true),
                taxi: e.get("taxi").and_then(TaxiLine::from_json),
            })
        })
        .collect();
    let has_bank = states.iter().any(|e| e.get("bank").is_some());
    let mut out = Vec::new();
    if samples.len() < 2 {
        return out;
    }
    #[derive(Clone, Copy, PartialEq, Eq, Debug)]
    enum Seg {
        Ground,
        Climb,
        Descent,
        Level,
        Turn,
    }
    let classify = |s: &Sample| {
        if s.on_ground {
            Seg::Ground
        } else if has_bank && s.bank_deg.abs() > TURN_IN_BANK_DEG {
            Seg::Turn
        } else if s.vsi_fpm > CLIMB_VSI_FPM {
            Seg::Climb
        } else if s.vsi_fpm < -CLIMB_VSI_FPM {
            Seg::Descent
        } else {
            Seg::Level
        }
    };
    // Segments: runs of one class, with brief flickers (under 8 s) absorbed
    // into the run around them.
    let mut runs: Vec<(Seg, usize, usize)> = Vec::new();
    let mut start = 0;
    for i in 1..=samples.len() {
        if i == samples.len() || classify(&samples[i]) != classify(&samples[start]) {
            runs.push((classify(&samples[start]), start, i));
            start = i;
        }
    }
    let mut merged: Vec<(Seg, usize, usize)> = Vec::new();
    for (seg, a, b) in runs {
        let dur = samples[b - 1].t_s - samples[a].t_s;
        match merged.last_mut() {
            Some((pseg, _, pb)) if dur < 8.0 && seg != Seg::Ground && *pseg != Seg::Ground => {
                *pb = b;
            }
            Some((pseg, _, pb)) if *pseg == seg => *pb = b,
            _ => merged.push((seg, a, b)),
        }
    }
    // A cross-country leg is not straight and level: every airborne run is
    // cut at its turns into straight sub-runs (the turn itself dropped), so
    // "level at 7,500 for 18 min" never carries a 180 degree heading
    // change through the heading axis.
    let mut straight: Vec<(Seg, usize, usize)> = Vec::new();
    for (seg, a, b) in merged {
        if matches!(seg, Seg::Ground | Seg::Turn) {
            straight.push((seg, a, b));
            continue;
        }
        let mut start = a;
        let mut ref_hdg = samples[start].hdg_deg;
        let mut i = start + 1;
        while i < b {
            let settled_turn = course_change_at(&samples[i..b], ref_hdg);
            if crate::angles::wrap180(samples[i].hdg_deg - ref_hdg).abs() > OFFLINE_TURN_DEG || settled_turn {
                // A turn is recognised partway round it (twenty degrees in,
                // or after it has settled), so the straight run ends where
                // the heading last sat on the old one: the first twenty
                // degrees of every turn were being graded as the run's
                // heading error, "20 degrees left (FAIL)" on a followed plan.
                let end = (start..i)
                    .rev()
                    .find(|&k| crate::angles::wrap180(samples[k].hdg_deg - ref_hdg).abs() <= OFFLINE_SETTLED_DEG)
                    .map_or(i, |k| k + 1);
                straight.push((seg, start, end));
                // Skip the turn: consecutive samples still swinging.
                let mut j = i;
                while j + 1 < b {
                    let dt = (samples[j + 1].t_s - samples[j].t_s).max(0.1);
                    if crate::angles::wrap180(samples[j + 1].hdg_deg - samples[j].hdg_deg).abs() / dt
                        < OFFLINE_TURN_RATE_DPS
                    {
                        break;
                    }
                    j += 1;
                }
                start = j;
                ref_hdg = samples[start].hdg_deg;
                i = start + 1;
                continue;
            }
            i += 1;
        }
        if start < b {
            straight.push((seg, start, b));
        }
    }
    let mut segments = 0;
    for (seg, a, b) in &straight {
        if segments >= OFFLINE_MAX_SEGMENTS {
            break;
        }
        let mut slice = &samples[*a..*b];
        if *seg == Seg::Descent {
            // The final approach is the landing's to grade.
            let cut = slice
                .iter()
                .position(|s| s.agl_ft < FINAL_AGL_FT)
                .unwrap_or(slice.len());
            slice = &slice[..cut];
        }
        if slice.len() < 2 {
            continue;
        }
        let dur = slice[slice.len() - 1].t_s - slice[0].t_s;
        let entry = slice[0];
        let task = match seg {
            Seg::Level if dur >= OFFLINE_LEVEL_MIN_S => Task::StraightAndLevel,
            Seg::Climb if dur >= OFFLINE_CLIMB_MIN_S => Task::Climb,
            Seg::Descent if dur >= OFFLINE_CLIMB_MIN_S => Task::Descent,
            Seg::Turn if dur >= 10.0 => Task::SteepTurns,
            _ => continue,
        };
        let mut targets = task.assign(&entry, &refs, level);
        // Offline the assignment is the pilot's own: the altitude they
        // levelled at, the heading and speed they SETTLED on (the median,
        // not the entry: a level-off's first seconds still carry the climb
        // speed and a roll-out's the turn), except the climb speed, which
        // is the book's if the book has one.
        let held_hdg = round_to(circular_mean_deg(slice.iter().map(|s| s.hdg_deg)), 5.0) % 360.0;
        let held_ias = round_to(median(slice.iter().map(|s| s.ias_kts)), 5.0);
        targets.hdg_deg = Some(held_hdg);
        match task {
            Task::StraightAndLevel => {
                targets.ias_kts = Some(held_ias);
            }
            Task::Climb | Task::Descent => {
                // Judged against the level-off the pilot actually made.
                targets.alt_ft = Some(round_to(slice[slice.len() - 1].alt_ft, 100.0));
                if task == Task::Descent || refs.vy_kts.is_none() {
                    targets.ias_kts = Some(held_ias);
                }
            }
            Task::SteepTurns => {
                targets.alt_ft = Some(round_to(entry.alt_ft, 100.0));
                targets.ias_kts = Some(held_ias);
            }
            _ => {}
        }
        let mut scorer = Scorer::new(task, level, targets);
        // The offline scorer judges the whole run: level-off logic and the
        // two-minute clock are the live lesson's; here every sample counts
        // on the axes the task names.
        for s in slice {
            scorer.tick_offline(s);
        }
        let g = scorer.grade_offline();
        segments += 1;
        out.push(format!(
            "{}Z {} {:.0} ft for {}: {}",
            hms(entry.t_s),
            match task {
                Task::StraightAndLevel => "level at".to_string(),
                Task::Climb => "climb from".to_string(),
                Task::Descent => "descent from".to_string(),
                Task::SteepTurns => "steep turn at".to_string(),
                _ => task.key().to_string(),
            },
            entry.alt_ft,
            mins(dur),
            g.written()
        ));
    }
    // Taxis on a cleared route (state records carrying `taxi`): pace and
    // the centreline, one line each, the same scorer as the lesson.
    let mut i = 0;
    while i < samples.len() {
        if samples[i].taxi.is_none() || !samples[i].on_ground {
            i += 1;
            continue;
        }
        let a = i;
        while i < samples.len() && samples[i].taxi.is_some() && samples[i].on_ground {
            i += 1;
        }
        let run = &samples[a..i];
        let moved = run.iter().any(|s| s.gs_kts.abs() > TAXI_MOVING_KTS);
        if run.len() < 5 || !moved {
            continue;
        }
        // No profile named: the default straight-leg pace, as the lesson.
        let mut taxi_refs = refs.clone();
        if taxi_refs.taxi_kts <= 0.0 {
            taxi_refs.taxi_kts = DEFAULT_TAXI_KTS;
        }
        let targets = Task::Taxi.assign(&run[0], &taxi_refs, level);
        let mut scorer = Scorer::new(Task::Taxi, level, targets);
        for s in run {
            scorer.tick_offline(s);
        }
        let g = scorer.grade_offline();
        out.push(format!(
            "{}Z taxi for {}: {}",
            hms(run[0].t_s),
            mins(run[run.len() - 1].t_s - run[0].t_s),
            g.written()
        ));
    }
    // The final approach and touchdown: the last airborne run before a
    // touchdown, judged below FINAL_AGL_FT.
    if let Some(app) = refs.approach_kts {
        for i in 1..samples.len() {
            if samples[i].on_ground && !samples[i - 1].on_ground {
                let mut scorer = Scorer::new(
                    Task::Landing,
                    level,
                    Targets {
                        ias_kts: Some(app.round()),
                        ..Targets::default()
                    },
                );
                let from = (0..i)
                    .rev()
                    .find(|&j| samples[j].on_ground || samples[j].agl_ft > FINAL_AGL_FT)
                    .map(|j| j + 1)
                    .unwrap_or(0);
                for s in &samples[from..=i] {
                    scorer.tick(s);
                }
                let g = scorer.grade();
                out.push(format!(
                    "{}Z final and landing: {}",
                    hms(samples[i].t_s),
                    g.written()
                ));
            }
        }
    }
    out
}

impl Scorer {
    /// Offline: judge every sample on the task's axes, without the live
    /// lesson's completion logic.
    fn tick_offline(&mut self, s: &Sample) {
        let Some(prev) = self.last else {
            self.started_t = Some(s.t_s);
            self.last = Some(*s);
            return;
        };
        let dt = (s.t_s - prev.t_s).clamp(0.0, 5.0);
        self.last = Some(*s);
        let t = s.t_s;
        let mut sink = Vec::new();
        let mut judge = |axis: &mut Option<AxisScore>, dev: f64| {
            if let Some(a) = axis.as_mut() {
                if let Some(l) = a.feed(dev, dt, t) {
                    sink.push(l);
                }
            }
        };
        match self.task {
            Task::StraightAndLevel | Task::SteepTurns => {
                judge(
                    &mut self.alt,
                    s.alt_ft - self.targets.alt_ft.unwrap_or(s.alt_ft),
                );
                judge(
                    &mut self.ias,
                    s.ias_kts - self.targets.ias_kts.unwrap_or(s.ias_kts),
                );
                if self.task == Task::StraightAndLevel {
                    judge(
                        &mut self.hdg,
                        self.targets
                            .hdg_deg
                            .map(|h| crate::angles::wrap180(s.hdg_deg - h))
                            .unwrap_or(0.0),
                    );
                } else {
                    judge(
                        &mut self.bank,
                        s.bank_deg.abs() - self.targets.bank_deg.unwrap_or(45.0),
                    );
                }
            }
            Task::Climb | Task::Descent => {
                judge(
                    &mut self.hdg,
                    self.targets
                        .hdg_deg
                        .map(|h| crate::angles::wrap180(s.hdg_deg - h))
                        .unwrap_or(0.0),
                );
                judge(
                    &mut self.ias,
                    s.ias_kts - self.targets.ias_kts.unwrap_or(s.ias_kts),
                );
            }
            Task::Taxi => {
                let gs = s.gs_kts.abs();
                if gs > TAXI_MOVING_KTS {
                    judge(&mut self.pace, gs - self.targets.pace_kts.unwrap_or(gs));
                }
                self.line_tick(s, &prev, dt);
            }
            _ => {}
        }
    }

    fn grade_offline(&self) -> Grade {
        let mut g = self.grade();
        g.complete = true;
        g.pass = g.axes.iter().filter(|a| a.judged_s > 0.0).all(|a| a.pass);
        g
    }
}

/// The circular mean of headings, degrees 0..360.
fn circular_mean_deg(it: impl Iterator<Item = f64>) -> f64 {
    let (mut x, mut y, mut n) = (0.0, 0.0, 0usize);
    for h in it {
        x += h.to_radians().cos();
        y += h.to_radians().sin();
        n += 1;
    }
    if n == 0 {
        return 0.0;
    }
    crate::angles::wrap360(y.atan2(x).to_degrees())
}

fn median(it: impl Iterator<Item = f64>) -> f64 {
    let mut v: Vec<f64> = it.collect();
    if v.is_empty() {
        return 0.0;
    }
    v.sort_by(|a, b| a.total_cmp(b));
    v[v.len() / 2]
}

fn hms(t: f64) -> String {
    let secs = t.max(0.0) as u64 % 86_400;
    format!(
        "{:02}:{:02}:{:02}",
        secs / 3600,
        (secs / 60) % 60,
        secs % 60
    )
}

fn mins(s: f64) -> String {
    if s < 90.0 {
        format!("{:.0} s", s)
    } else {
        format!("{:.0} min", s / 60.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn refs172() -> Refs {
        Refs {
            vy_kts: Some(74.0),
            vr_kts: Some(55.0),
            taxi_kts: 15.0,
            approach_kts: Some(65.0),
            cruise_ias_kts: Some(108.0),
            va_kts: Some(99.0),
            slow_kts: Some(58.0),
            pattern_agl_ft: 1000.0,
            pattern_kts: Some(74.0),
        }
    }

    /// The instructor briefs and scores the approach autoland and the
    /// circuit fly (`aircraft::approach_kts`): an airframe declaring none is
    /// max(1.3 Vso, 1.2 Vs), not the 1.3 x CLEAN stall it still briefed
    /// (refactor audit 2026-10-04, finding #2).
    #[test]
    fn the_brief_approach_is_the_aircrafts_rule() {
        let mut p = crate::aircraft::load("cirrus_sr22").unwrap();
        assert_eq!(Refs::from_profile(&p).approach_kts, Some(75.0), "declared wins");
        p.reference.as_mut().unwrap().approach_kts = None;
        // SR22: Vso 60, Vs 70: max(78, 84) = 84, not 1.3 x 70 = 91.
        assert_eq!(Refs::from_profile(&p).approach_kts, Some(84.0));
    }

    fn air(t: f64, alt: f64, hdg: f64, ias: f64) -> Sample {
        Sample {
            t_s: t,
            alt_ft: alt,
            agl_ft: alt - 500.0,
            hdg_deg: hdg,
            ias_kts: ias,
            bank_deg: 0.0,
            vsi_fpm: 0.0,
            on_ground: false,
            track_deg: Some(hdg),
            gs_kts: ias,
            engine_running: true,
            taxi: None,
        }
    }

    #[test]
    fn the_brief_assigns_rounded_numbers_from_the_book() {
        let entry = air(0.0, 3470.0, 268.0, 104.0);
        let t = Task::StraightAndLevel.assign(&entry, &refs172(), Level::Private);
        assert_eq!(t.alt_ft, Some(3500.0));
        assert_eq!(t.hdg_deg, Some(270.0));
        assert_eq!(t.ias_kts, Some(108.0), "cruise IAS from the book");
        let c = Task::Climb.assign(&entry, &refs172(), Level::Private);
        assert_eq!(c.alt_ft, Some(4500.0), "next 500 at least 1,000 up");
        assert_eq!(c.ias_kts, Some(74.0), "Vy");
        let d = Task::Descent.assign(&entry, &refs172(), Level::Private);
        assert_eq!(
            d.alt_ft,
            Some(2000.0),
            "1,000 down to a 500, and ground plus 1,500 is 2,000 too"
        );
        assert_eq!(d.ias_kts, Some(105.0), "the entry speed, rounded");
        let brief = Task::Climb.brief(&c, Level::Private);
        assert!(brief.contains("4,500 feet"), "{brief}");
        assert!(brief.contains("74 knots"), "{brief}");
        assert!(brief.contains("heading 270"), "{brief}");
        assert!(brief.contains("plus 10 minus 5"), "{brief}");
        assert!(brief.ends_with("Say begin when you're ready."), "{brief}");
        assert!(!brief.contains('\u{2014}') && !brief.contains('\u{2013}'));
    }

    #[test]
    fn a_descent_is_never_assigned_into_the_ground() {
        let mut entry = air(0.0, 2400.0, 90.0, 100.0);
        entry.agl_ft = 1600.0; // ground at 800
        let d = Task::Descent.assign(&entry, &refs172(), Level::Private);
        // 1,400 wanted, floored to ground + 1,500 = 2,300.
        assert_eq!(d.alt_ft, Some(2300.0));
    }

    #[test]
    fn a_steady_two_minutes_passes_straight_and_level() {
        let t = Targets {
            alt_ft: Some(3500.0),
            hdg_deg: Some(270.0),
            ias_kts: Some(108.0),
            bank_deg: None,
            ..Targets::default()
        };
        let mut sc = Scorer::new(Task::StraightAndLevel, Level::Private, t);
        let mut lines = Vec::new();
        for i in 0..=125 {
            let s = air(
                i as f64,
                3500.0 + (i % 5) as f64 * 8.0,
                270.0 + ((i % 3) as f64 - 1.0) * 2.0,
                108.0 + (i % 2) as f64 * 3.0,
            );
            lines.extend(sc.tick(&s));
        }
        assert!(sc.complete());
        assert_eq!(
            lines,
            vec!["That's time.".to_string()],
            "no excursion narrated"
        );
        let g = sc.grade();
        assert!(g.pass, "{}", g.spoken());
        assert!(
            g.spoken().ends_with("That meets the standard."),
            "{}",
            g.spoken()
        );
    }

    #[test]
    fn a_sustained_excursion_is_narrated_once_then_the_recovery() {
        let t = Targets {
            alt_ft: Some(3500.0),
            hdg_deg: Some(270.0),
            ias_kts: Some(108.0),
            bank_deg: None,
            ..Targets::default()
        };
        let mut sc = Scorer::new(Task::StraightAndLevel, Level::Private, t);
        let mut lines = Vec::new();
        for i in 0..40 {
            // 130 ft low from t=10 to t=25, then back.
            let alt = if (10..25).contains(&i) {
                3370.0
            } else {
                3500.0
            };
            lines.extend(sc.tick(&air(i as f64, alt, 270.0, 108.0)));
        }
        assert_eq!(
            lines,
            vec![
                "Altitude, 130 low. Nose up a touch.".to_string(),
                "Good, back on altitude.".to_string()
            ]
        );
        let g = sc.grade();
        let alt = g.axes.iter().find(|a| a.axis == Axis::Altitude).unwrap();
        assert!(!alt.pass, "15 s outside is not momentary");
        assert_eq!(alt.worst, -130.0);
        assert!(!g.pass);
        assert!(!g.complete, "the clock has not run");
        assert!(g.spoken().contains("Task not completed."));
    }

    #[test]
    fn a_momentary_bump_is_forgiven_but_a_gross_one_is_not() {
        let t = Targets {
            alt_ft: Some(3500.0),
            hdg_deg: Some(270.0),
            ias_kts: Some(108.0),
            bank_deg: None,
            ..Targets::default()
        };
        let mut sc = Scorer::new(Task::StraightAndLevel, Level::Private, t);
        for i in 0..30 {
            let alt = if i == 10 || i == 11 { 3610.0 } else { 3500.0 };
            sc.tick(&air(i as f64, alt, 270.0, 108.0));
        }
        let alt = sc
            .grade()
            .axes
            .into_iter()
            .find(|a| a.axis == Axis::Altitude)
            .unwrap();
        assert!(alt.pass, "2 s outside, corrected promptly");
        let mut sc = Scorer::new(Task::StraightAndLevel, Level::Private, t);
        for i in 0..30 {
            let alt = if i == 10 { 3750.0 } else { 3500.0 };
            sc.tick(&air(i as f64, alt, 270.0, 108.0));
        }
        let alt = sc
            .grade()
            .axes
            .into_iter()
            .find(|a| a.axis == Axis::Altitude)
            .unwrap();
        assert!(!alt.pass, "250 ft is gross however brief");
    }

    #[test]
    fn heading_deviation_wraps_through_north() {
        let t = Targets {
            alt_ft: Some(3500.0),
            hdg_deg: Some(5.0),
            ias_kts: Some(108.0),
            bank_deg: None,
            ..Targets::default()
        };
        let mut sc = Scorer::new(Task::StraightAndLevel, Level::Private, t);
        for i in 0..10 {
            sc.tick(&air(i as f64, 3500.0, 358.0, 108.0));
        }
        let h = sc
            .grade()
            .axes
            .into_iter()
            .find(|a| a.axis == Axis::Heading)
            .unwrap();
        assert_eq!(h.worst, -7.0, "358 is 7 left of 005, not 353 right");
        assert!(h.pass);
    }

    #[test]
    fn a_climb_is_judged_on_speed_then_on_the_level_off() {
        let t = Targets {
            alt_ft: Some(4500.0),
            hdg_deg: Some(270.0),
            ias_kts: Some(74.0),
            bank_deg: None,
            ..Targets::default()
        };
        let mut sc = Scorer::new(Task::Climb, Level::Private, t);
        let mut lines = Vec::new();
        let mut alt = 3500.0;
        let mut t_s = 0.0;
        // 90 s at 700 fpm, 71 kt (3 slow: within +10/-5), then level at 4,460.
        while alt < 4460.0 {
            let mut s = air(t_s, alt, 270.0, 71.0);
            s.vsi_fpm = 700.0;
            lines.extend(sc.tick(&s));
            alt += 700.0 / 60.0;
            t_s += 1.0;
        }
        for _ in 0..12 {
            lines.extend(sc.tick(&air(t_s, 4460.0, 270.0, 90.0)));
            t_s += 1.0;
        }
        assert!(sc.complete(), "{lines:?}");
        assert!(
            lines.iter().any(|l| l == "Level. That completes the task."),
            "{lines:?}"
        );
        let g = sc.grade();
        assert!(g.pass, "{}", g.spoken());
        let ias = g.axes.iter().find(|a| a.axis == Axis::Airspeed).unwrap();
        assert!(
            ias.judged_s > 60.0 && ias.judged_s < 100.0,
            "speed judged in the climb only: {}",
            ias.judged_s
        );
        assert!(
            g.notes[0].starts_with("Levelled at 4,460 feet, 40 below"),
            "{:?}",
            g.notes
        );
    }

    #[test]
    fn a_climb_levelled_elsewhere_names_the_assignment() {
        let t = Targets {
            alt_ft: Some(4500.0),
            hdg_deg: Some(270.0),
            ias_kts: Some(74.0),
            bank_deg: None,
            ..Targets::default()
        };
        let mut sc = Scorer::new(Task::Climb, Level::Private, t);
        let mut lines = Vec::new();
        for i in 0..10 {
            let mut s = air(i as f64, 3500.0 + i as f64 * 10.0, 270.0, 74.0);
            s.vsi_fpm = 600.0;
            lines.extend(sc.tick(&s));
        }
        for i in 10..40 {
            lines.extend(sc.tick(&air(i as f64, 4000.0, 270.0, 100.0)));
        }
        assert!(sc.complete());
        assert!(
            lines
                .iter()
                .any(|l| l.contains("levelled at 4,000 feet. The assignment was 4,500 feet")),
            "{lines:?}"
        );
        assert!(!sc.grade().pass);
    }

    #[test]
    fn steep_turns_count_roll_outs_and_judge_the_bank() {
        let t = Targets {
            alt_ft: Some(3500.0),
            hdg_deg: Some(90.0),
            ias_kts: Some(95.0),
            bank_deg: Some(45.0),
            ..Targets::default()
        };
        let mut sc = Scorer::new(Task::SteepTurns, Level::Private, t);
        let mut lines = Vec::new();
        let mut t_s = 0.0;
        let turn = |sc: &mut Scorer,
                    lines: &mut Vec<String>,
                    t_s: &mut f64,
                    sign: f64,
                    rollout_hdg: f64| {
            for i in 0..30 {
                let mut s = air(*t_s, 3500.0, 90.0 + sign * i as f64 * 12.0, 95.0);
                s.bank_deg = sign * 45.0;
                lines.extend(sc.tick(&s));
                *t_s += 1.0;
            }
            for _ in 0..5 {
                lines.extend(sc.tick(&air(*t_s, 3500.0, rollout_hdg, 95.0)));
                *t_s += 1.0;
            }
        };
        turn(&mut sc, &mut lines, &mut t_s, 1.0, 94.0);
        assert!(
            lines.iter().any(|l| l == "One. Now the other way."),
            "{lines:?}"
        );
        turn(&mut sc, &mut lines, &mut t_s, -1.0, 82.0);
        assert!(sc.complete());
        let g = sc.grade();
        assert_eq!(g.notes.len(), 2);
        assert!(g.notes[0].contains("4 degrees right"), "{:?}", g.notes);
        assert!(g.notes[1].contains("8 degrees left"), "{:?}", g.notes);
        assert!(g.pass, "{}", g.spoken());
    }

    /// LFPG 09R, 2026-09-22, at the monitor's rate: -690 at 12 ft, then a
    /// sample on the oleos at -221 with the flag still clear, then ground.
    /// The wheels met the runway at 690, and the grade says so.
    #[test]
    fn the_touchdown_sink_is_the_steepest_in_the_window_not_the_last_sample() {
        let recent = [(0.0, -772.0), (1.1, -750.0), (2.1, -690.0), (3.2, -221.0)];
        assert_eq!(touchdown_sink_fpm(&recent, 4.3, -38.0), -690.0);
        // Only the window counts: a steep rate five seconds up is the approach.
        let recent = [(0.0, -900.0), (2.5, -300.0), (3.5, -250.0)];
        assert_eq!(touchdown_sink_fpm(&recent, 4.5, -156.0), -300.0);
        // The ground sample itself when it is the steepest on record.
        assert_eq!(touchdown_sink_fpm(&[(4.0, -100.0)], 4.5, -180.0), -180.0);
        assert_eq!(touchdown_sink_fpm(&[], 4.5, -180.0), -180.0);
    }

    #[test]
    fn a_landing_reports_the_touchdown_and_a_firm_one_fails() {
        let t = Targets {
            ias_kts: Some(65.0),
            ..Targets::default()
        };
        let mut sc = Scorer::new(Task::Landing, Level::Private, t);
        let mut lines = Vec::new();
        let mut agl = 600.0;
        let mut t_s = 0.0;
        while agl > 0.0 {
            let mut s = air(t_s, 1000.0 + agl, 90.0, 66.0);
            s.agl_ft = agl;
            s.vsi_fpm = -500.0;
            lines.extend(sc.tick(&s));
            agl -= 500.0 / 60.0 * 2.0;
            t_s += 2.0;
        }
        let mut down = air(t_s, 1000.0, 92.0, 55.0);
        down.agl_ft = 0.0;
        down.on_ground = true;
        lines.extend(sc.tick(&down));
        assert!(sc.complete());
        let g = sc.grade();
        assert!(
            g.notes[0].starts_with("Touchdown at 500 feet per minute, 66 knots, no drift. Firm."),
            "{:?}",
            g.notes
        );
        assert!(!g.pass, "a firm arrival is not to standard");
        let ias = g.axes.iter().find(|a| a.axis == Axis::Airspeed).unwrap();
        assert!(ias.pass, "1 kt fast on final");
    }

    #[test]
    fn requests_are_a_closed_vocabulary() {
        let _g = crate::mode::TEST_LOCK.lock().unwrap();
        lesson().stage = Stage::Idle;
        assert_eq!(
            request("teach me steep turns"),
            Some(Request::Start(Task::SteepTurns))
        );
        assert_eq!(
            request("let's practice slow flight"),
            Some(Request::Start(Task::SlowFlight))
        );
        assert_eq!(
            request("lesson on the traffic pattern"),
            Some(Request::Start(Task::Pattern))
        );
        assert_eq!(
            request("coach me through a normal landing"),
            Some(Request::Start(Task::Landing))
        );
        assert_eq!(
            request("practice a constant airspeed climb"),
            Some(Request::Start(Task::Climb))
        );
        assert_eq!(
            request("teach me straight and level"),
            Some(Request::Start(Task::StraightAndLevel))
        );
        assert_eq!(request("teach me something"), Some(Request::Menu));
        assert_eq!(request("what can you teach"), Some(Request::Menu));
        assert_eq!(request("end the lesson"), Some(Request::End));
        assert_eq!(request("how am I doing?"), Some(Request::Interim));
        assert_eq!(
            request("commercial standards please"),
            Some(Request::SetLevel(Level::Commercial))
        );
        assert_eq!(request("say the standards again"), Some(Request::Standards));
        // Commands and chatter are not lessons.
        assert_eq!(request("climb to four thousand"), None);
        assert_eq!(request("flaps ten"), None);
        assert_eq!(request("descend and maintain three thousand"), None);
        assert_eq!(request("begin"), None, "nothing briefed");
        assert_eq!(request("talk me through the descent checklist"), None);
    }

    #[test]
    fn the_lesson_runs_brief_begin_tick_debrief() {
        let _g = crate::mode::TEST_LOCK.lock().unwrap();
        // Reset the process-wide lesson.
        {
            let mut l = lesson();
            l.stage = Stage::Idle;
            l.refs = refs172();
            l.level = Level::Private;
            l.latest = None;
        }
        assert!(
            handle(Request::Start(Task::StraightAndLevel), 0.0).contains("can't see the aircraft")
        );
        tick(air(0.0, 3480.0, 271.0, 106.0));
        let brief = handle(Request::Start(Task::StraightAndLevel), 0.0);
        assert!(
            brief.starts_with("Straight and level. Hold 3,500 feet, heading 270, 108 knots"),
            "{brief}"
        );
        assert_eq!(
            request("begin"),
            Some(Request::Begin),
            "a brief is standing"
        );
        assert_eq!(request("okay ready"), Some(Request::Begin));
        assert_eq!(
            status().as_deref(),
            Some("lesson: straight and level, briefed")
        );
        assert_eq!(handle(Request::Begin, 1.0), "Your aircraft. Go.");
        assert!(active());
        // A refused AP command becomes the target.
        assert_eq!(
            take_target("set_heading", &json!({ "heading": 300 })).as_deref(),
            Some("Heading 300. Your hands.")
        );
        assert_eq!(take_target("set_flaps", &json!({ "notch": 1 })), None);
        let mut said = Vec::new();
        for i in 2..130 {
            said.extend(tick(air(i as f64, 3500.0, 300.0, 108.0)));
        }
        assert!(!active(), "complete: back to idle");
        assert_eq!(said.first().map(String::as_str), Some("That's time."));
        assert!(
            said.last().unwrap().ends_with("That meets the standard."),
            "{said:?}"
        );
        assert!(handle(Request::Interim, 200.0).ends_with("That meets the standard."));
        assert_eq!(handle(Request::End, 200.0), "No lesson running.");
        assert_eq!(request("begin"), None, "nothing briefed any more");
    }

    #[test]
    fn a_lesson_needs_height_and_a_wing() {
        let _g = crate::mode::TEST_LOCK.lock().unwrap();
        {
            let mut l = lesson();
            l.stage = Stage::Idle;
            l.refs = refs172();
            l.level = Level::Private;
        }
        let mut ground = air(0.0, 100.0, 90.0, 0.0);
        ground.on_ground = true;
        ground.agl_ft = 0.0;
        tick(ground);
        assert_eq!(
            handle(Request::Start(Task::SteepTurns), 0.0),
            "For steep turns, get airborne first."
        );
        let brief = handle(Request::Start(Task::Pattern), 0.0);
        assert!(
            brief.starts_with(
                "Traffic pattern. Pattern altitude 1,100 feet, 74 knots on the downwind"
            ),
            "{brief}"
        );
        assert_eq!(handle(Request::End, 0.0), "Lesson cancelled.");
        let mut low = air(1.0, 1200.0, 90.0, 100.0);
        low.agl_ft = 900.0;
        tick(low);
        assert_eq!(
            handle(Request::Start(Task::SlowFlight), 1.0),
            "For slow flight, climb above fifteen hundred feet first."
        );
    }

    #[test]
    fn the_offline_grader_finds_the_level_runs_and_the_landing() {
        let mk = |t: f64, alt: f64, agl: f64, hdg: f64, ias: f64, vsi: f64, ground: bool| json!({ "t": t, "alt": alt, "agl": agl, "hdg": hdg, "ias": ias, "vsi": vsi, "ground": ground, "track": hdg, "bank": 0.0 });
        let mut states = Vec::new();
        let mut t = 1000.0;
        // 90 s level at 3,500, wandering 40 ft.
        for i in 0..90 {
            states.push(mk(
                t,
                3500.0 + (i % 4) as f64 * 10.0,
                3000.0,
                180.0,
                105.0,
                0.0,
                false,
            ));
            t += 1.0;
        }
        // 60 s descent at 500 fpm, 100 kt.
        let mut alt = 3500.0;
        for _ in 0..60 {
            states.push(mk(t, alt, alt - 500.0, 180.0, 100.0, -500.0, false));
            alt -= 500.0 / 60.0;
            t += 1.0;
        }
        // 120 s level at 3,000 but 160 ft low for 30 s.
        for i in 0..120 {
            let a = if (40..70).contains(&i) {
                2840.0
            } else {
                3000.0
            };
            states.push(mk(t, a, a - 500.0, 180.0, 105.0, 0.0, false));
            t += 1.0;
        }
        // Final at 68 kt and a soft touchdown.
        let mut agl = 700.0;
        while agl > 0.0 {
            states.push(mk(t, 500.0 + agl, agl, 180.0, 68.0, -400.0, false));
            agl -= 400.0 / 60.0;
            t += 1.0;
        }
        states.push(mk(t, 500.0, 0.0, 180.0, 60.0, -150.0, true));
        let refs: Vec<&Value> = states.iter().collect();
        let profile = crate::aircraft::load("cessna_172sp").expect("172 profile");
        let lines = grade_flight(&refs, Some(&profile), Level::Private);
        assert_eq!(lines.len(), 4, "{lines:#?}");
        assert!(
            lines[0].contains("level at 3500 ft for 89 s")
                && lines[0].contains("Within private standards"),
            "{}",
            lines[0]
        );
        assert!(
            lines[1].contains("descent from 3500 ft for 59 s") && lines[1].contains("Within"),
            "{}",
            lines[1]
        );
        assert!(
            lines[2].contains("level at 3000 ft for 2 min")
                && lines[2].contains("altitude 160 feet low (FAIL)"),
            "{}",
            lines[2]
        );
        assert!(
            lines[3].contains("final and landing")
                && lines[3].contains(
                    "; Touchdown at 400 feet per minute, 68 knots, no drift. Firm. Below"
                ),
            "{}",
            lines[3]
        );
    }

    fn gnd(t: f64, gs: f64, hdg: f64, eng: bool) -> Sample {
        Sample {
            t_s: t,
            alt_ft: 100.0,
            agl_ft: 0.0,
            hdg_deg: hdg,
            ias_kts: gs,
            bank_deg: 0.0,
            vsi_fpm: 0.0,
            on_ground: true,
            track_deg: Some(hdg),
            gs_kts: gs,
            engine_running: eng,
            taxi: None,
        }
    }

    #[test]
    fn a_taxi_is_graded_on_pace_and_completes_at_the_hold_short() {
        let t = Targets {
            pace_kts: Some(15.0),
            ..Targets::default()
        };
        let mut sc = Scorer::new(Task::Taxi, Level::Private, t);
        let mut lines = Vec::new();
        let mut t_s = 0.0;
        for _ in 0..60 {
            lines.extend(sc.tick(&gnd(t_s, 12.0, 90.0, true)));
            t_s += 1.0;
        }
        assert!(lines.is_empty(), "12 kt is under the pace: {lines:?}");
        for _ in 0..6 {
            lines.extend(sc.tick(&gnd(t_s, 20.0, 90.0, true)));
            t_s += 1.0;
        }
        assert_eq!(
            lines,
            vec!["Pace, 5 over. Ease the power back and hold it with the brakes.".to_string()]
        );
        for _ in 0..10 {
            lines.extend(sc.tick(&gnd(t_s, 12.0, 90.0, true)));
            t_s += 1.0;
        }
        assert_eq!(
            lines.last().map(String::as_str),
            Some("Good, back on pace.")
        );
        for _ in 0..12 {
            lines.extend(sc.tick(&gnd(t_s, 0.0, 90.0, true)));
            t_s += 1.0;
        }
        assert!(sc.complete(), "{lines:?}");
        assert_eq!(
            lines.last().map(String::as_str),
            Some("Stopped. Holding short, I take it.")
        );
        let g = sc.grade();
        assert!(!g.pass, "6 s over the pace: {}", g.spoken());
        assert!(g.notes[0].starts_with("Taxied 4"), "{:?}", g.notes);
        // A taxi that rolls straight onto the runway never held short.
        let mut sc = Scorer::new(Task::Taxi, Level::Private, t);
        for i in 0..40 {
            sc.tick(&gnd(i as f64, if i < 30 { 12.0 } else { 40.0 }, 90.0, true));
        }
        assert!(sc.complete() && !sc.grade().pass);
        assert!(sc.grade().notes[0].contains("without stopping"));
    }

    /// A taxi on a cleared route at 12 kt: `off(i)` is the cross-track at
    /// tick `i` (1 s ticks), a right corner flown from tick 30 to 38 (the
    /// route asks for it from 28; the heading swings 90 to 180), the
    /// projection jumping `jump` metres along the route at tick 34 (a
    /// vertex; 0 is a drawn curve), then a stop. Returns the lines said and
    /// the samples fed.
    fn taxi_on_route(
        level: Level,
        off: impl Fn(usize) -> f64,
        jump: f64,
    ) -> (Scorer, Vec<String>, Vec<Sample>) {
        let t = Targets {
            pace_kts: Some(15.0),
            ..Targets::default()
        };
        let mut sc = Scorer::new(Task::Taxi, level, t);
        let (mut lines, mut fed) = (Vec::new(), Vec::new());
        let mut rem = 800.0;
        for i in 0..90usize {
            let moving = i < 78;
            let gs = if moving { 12.0 } else { 0.0 };
            let hdg = match i {
                0..=30 => 90.0,
                31..=38 => 90.0 + (i - 30) as f64 * 11.25,
                _ => 180.0,
            };
            if moving {
                rem -= 12.0 * KTS_TO_MPS + if i == 34 { jump } else { 0.0 };
            }
            let s = Sample {
                taxi: Some(TaxiLine {
                    xtk_m: off(i),
                    remaining_m: rem,
                    turn: (28..=34).contains(&i).then_some(1.0),
                    off_route: false,
                }),
                ..gnd(i as f64, gs, hdg, true)
            };
            lines.extend(sc.tick(&s));
            fed.push(s);
        }
        (sc, lines, fed)
    }

    fn centreline(g: &Grade) -> &AxisGrade {
        g.axes
            .iter()
            .find(|a| a.axis == Axis::Centreline)
            .expect("a centreline axis")
    }

    #[test]
    fn the_taxi_is_judged_on_the_centreline_and_a_vertex_cut_is_a_cut() {
        // Off the stand 20 m from the line, joined by tick 5, 0.3 m down the
        // straights, 3.8 m inside the right corner while turning through it.
        let off = |i: usize| match i {
            0..=4 => 20.0 - i as f64 * 4.9,
            32..=36 => 3.8,
            _ => 0.3,
        };
        let (sc, lines, _) = taxi_on_route(Level::Private, off, 5.0);
        assert!(sc.complete());
        assert_eq!(
            lines,
            vec!["Stopped. Holding short, I take it.".to_string()],
            "a cut inside a vertex is no excursion, the join is not judged"
        );
        let g = sc.grade();
        assert!(g.pass, "{}", g.spoken());
        assert_eq!(centreline(&g).worst.abs().round(), 0.0, "{:?}", g.axes);
        assert!(
            g.notes.iter().all(|n| !n.contains("corner")),
            "{:?}",
            g.notes
        );
        // The same offset inside a drawn CURVE (no projection jump) is off
        // the line, as the scorecard reads it: 3.8 m for 5 s, its time
        // outside counted once the corner closes as a curve.
        let (sc, lines, _) = taxi_on_route(Level::Private, off, 0.0);
        let g = sc.grade();
        assert_eq!((centreline(&g).worst * 10.0).round() / 10.0, 3.8);
        assert_eq!(centreline(&g).out_s, 5.0);
        assert!(!centreline(&g).pass);
        assert_eq!(lines.len(), 1, "never narrated in the corner: {lines:?}");
        // Commercial holds the line to 2 m: the same curve fails it.
        let (sc, _, _) = taxi_on_route(Level::Commercial, off, 0.0);
        assert!(!centreline(&sc.grade()).pass);
        // A vertex cut deeper than TAXI_MAX_CUT_M fails the task.
        let deep = |i: usize| if (33..=34).contains(&i) { 6.0 } else { 0.3 };
        let (sc, _, _) = taxi_on_route(Level::Private, deep, 5.0);
        let g = sc.grade();
        assert!(!g.pass);
        assert!(
            g.notes.contains(&"Cut a corner by 6 metres.".to_string()),
            "{:?}",
            g.notes
        );
    }

    #[test]
    fn a_wander_off_the_line_is_narrated_once_with_its_correction() {
        let off = |i: usize| if (10..18).contains(&i) { 4.2 } else { 0.3 };
        let (sc, lines, _) = taxi_on_route(Level::Private, off, 5.0);
        assert_eq!(
            lines,
            vec![
                "Centreline, 4 metres right. A touch of left rudder, onto the line.".to_string(),
                "Good, back on centreline.".to_string(),
                "Stopped. Holding short, I take it.".to_string(),
            ]
        );
        let g = sc.grade();
        assert!(!g.pass);
        assert_eq!(
            centreline(&g).spoken(),
            "Centreline: 4 metres right at worst, outside for 8 seconds."
        );
        // The brief says the standard.
        let brief = Task::Taxi.brief(
            &Task::Taxi.assign(&gnd(0.0, 0.0, 90.0, true), &Refs::default(), Level::Private),
            Level::Private,
        );
        assert!(brief.contains("centreline within 3 metres"), "{brief}");
    }

    #[test]
    fn the_debrief_grades_a_taxi_from_its_state_records() {
        let off = |i: usize| if (10..18).contains(&i) { 4.2 } else { 0.3 };
        let (_, _, fed) = taxi_on_route(Level::Private, off, 5.0);
        let states: Vec<Value> = fed
            .iter()
            .map(|s| {
                json!({"kind": "state", "t": 1_790_000_000.0 + s.t_s, "alt": s.alt_ft,
                    "agl": 0.0, "hdg": s.hdg_deg, "ias": s.ias_kts, "vsi": 0.0,
                    "ground": true, "gs": s.gs_kts, "eng": true,
                    "taxi": s.taxi.map(|l| l.json())})
            })
            .collect();
        let refs: Vec<&Value> = states.iter().collect();
        let lines = grade_flight(&refs, None, Level::Private);
        assert_eq!(lines.len(), 1, "{lines:?}");
        assert!(
            lines[0]
                .contains("taxi for 89 s: pace 3 knots under, centreline 4 metres right (FAIL)"),
            "{lines:?}"
        );
        // A state record without the field grades no taxi.
        let bare: Vec<Value> = states
            .iter()
            .map(|v| {
                let mut v = v.clone();
                v.as_object_mut().unwrap().remove("taxi");
                v
            })
            .collect();
        let refs: Vec<&Value> = bare.iter().collect();
        assert!(grade_flight(&refs, None, Level::Private).is_empty());
    }

    #[test]
    fn a_takeoff_captures_the_roll_heading_and_judges_the_rotation_and_climb() {
        let t = Targets {
            hdg_deg: Some(180.0), // the hold-short heading: a placeholder
            ias_kts: Some(74.0),
            vr_kts: Some(55.0),
            ..Targets::default()
        };
        let mut sc = Scorer::new(Task::Takeoff, Level::Private, t);
        let mut lines = Vec::new();
        let mut t_s = 0.0;
        // Lined up on 092, the roll.
        let mut gs = 0.0;
        while gs < 56.0 {
            lines.extend(sc.tick(&gnd(t_s, gs, 92.0, true)));
            gs += 3.0;
            t_s += 1.0;
        }
        assert_eq!(lines, vec!["Rolling.".to_string()]);
        // Airborne at 56, climbing at 72 on 094.
        let mut agl = 10.0;
        while agl < 1100.0 {
            let mut s = air(t_s, 100.0 + agl, 94.0, 72.0);
            s.agl_ft = agl;
            s.vsi_fpm = 700.0;
            lines.extend(sc.tick(&s));
            agl += 700.0 / 60.0;
            t_s += 1.0;
        }
        assert!(sc.complete(), "{lines:?}");
        assert!(lines.contains(&"Airborne.".to_string()));
        assert_eq!(
            lines.last().map(String::as_str),
            Some("A thousand feet. That completes the takeoff.")
        );
        let g = sc.grade();
        assert_eq!(
            g.notes[0], "Rotated at 54 knots, Vr is 55.",
            "the last sample on the wheels"
        );
        let h = g.axes.iter().find(|a| a.axis == Axis::Heading).unwrap();
        assert_eq!(h.target, 92.0, "the roll heading, not the hold-short's");
        assert_eq!(h.worst, 2.0);
        assert!(g.pass, "{}", g.spoken());
        // An early rotation fails the task.
        let mut sc = Scorer::new(Task::Takeoff, Level::Private, t);
        for i in 0..15 {
            sc.tick(&gnd(i as f64, i as f64 * 3.0, 92.0, true));
        }
        let mut s = air(15.0, 150.0, 92.0, 43.0);
        s.agl_ft = 5.0;
        sc.tick(&s);
        let g = sc.grade();
        assert_eq!(g.notes[0], "Rotated at 42 knots, Vr is 55. Early.");
        assert!(!g.pass);
    }

    /// A checklist flow that has some lists and runs each for two ticks.
    struct FakeIo {
        have: Vec<&'static str>,
        running_left: u32,
        started: Vec<usize>,
    }

    impl SyllabusIo for FakeIo {
        fn find_list(&self, key: &str) -> Option<usize> {
            self.have.iter().position(|k| *k == key)
        }
        fn start_list(&mut self, idx: usize) -> bool {
            if self.running_left > 0 {
                return false;
            }
            self.started.push(idx);
            self.running_left = 2;
            true
        }
        fn list_running(&self) -> bool {
            self.running_left > 0
        }
    }

    impl FakeIo {
        fn tick(&mut self) {
            self.running_left = self.running_left.saturating_sub(1);
        }
    }

    #[test]
    fn the_full_flight_runs_from_cold_and_dark_to_shutdown() {
        let _g = crate::mode::TEST_LOCK.lock().unwrap();
        {
            let mut l = lesson();
            l.stage = Stage::Idle;
            l.refs = refs172();
            l.level = Level::Private;
            l.latest = None;
        }
        let mut io = FakeIo {
            have: vec!["before taxi", "before takeoff", "engine shutdown"],
            running_left: 0,
            started: Vec::new(),
        };
        let mut t_s = 0.0;
        let mut said: Vec<String> = Vec::new();
        let run = |s: Sample, io: &mut FakeIo, said: &mut Vec<String>| {
            said.extend(tick_with(s, io));
            io.tick();
        };
        // Cold and dark on the ramp.
        run(gnd(t_s, 0.0, 180.0, false), &mut io, &mut said);
        assert!(handle(Request::Start(Task::SteepTurns), t_s).contains("get airborne first"));
        let opener = handle(Request::FullFlight, t_s);
        assert!(
            opener.starts_with("Full flight from cold and dark."),
            "{opener}"
        );
        assert_eq!(
            status().as_deref(),
            Some("lesson: full flight, preflight checklist")
        );
        assert!(syllabus_active());
        assert!(handle(Request::FullFlight, t_s).contains("already on the full flight"));
        // The three ramp lists this airframe lacks are skipped in one tick,
        // then the engine is waited for.
        t_s += 1.0;
        run(gnd(t_s, 0.0, 180.0, false), &mut io, &mut said);
        assert_eq!(
            said.iter().filter(|l| l.starts_with("No ")).count(),
            3,
            "{said:?}"
        );
        assert_eq!(
            status().as_deref(),
            Some("lesson: full flight, engine start")
        );
        // Engine comes alive: the before taxi list runs for two ticks.
        for _ in 0..5 {
            t_s += 1.0;
            run(gnd(t_s, 0.0, 180.0, true), &mut io, &mut said);
        }
        assert!(said.contains(&"Engine's running.".to_string()));
        assert!(said.contains(&"Before taxi checklist. I'll read, you do.".to_string()));
        assert_eq!(io.started, vec![0]);
        assert_eq!(
            status().as_deref(),
            Some("lesson: full flight, taxi"),
            "{said:?}"
        );
        assert!(
            said.iter()
                .any(|l| l.starts_with("Taxi. Get your clearance")),
            "{said:?}"
        );
        // A pace command during the taxi retargets the pace, never writes.
        assert_eq!(
            take_target("hold_speed", &json!({ "knots": 12 })).as_deref(),
            Some("12 knots it is. Your hands.")
        );
        // Taxi 60 s at 10 kt, stop 11 s.
        for _ in 0..60 {
            t_s += 1.0;
            run(gnd(t_s, 10.0, 180.0, true), &mut io, &mut said);
        }
        for _ in 0..11 {
            t_s += 1.0;
            run(gnd(t_s, 0.0, 180.0, true), &mut io, &mut said);
        }
        assert!(
            said.contains(&"Stopped. Holding short, I take it.".to_string()),
            "{said:?}"
        );
        // Before takeoff list, then the takeoff.
        for _ in 0..4 {
            t_s += 1.0;
            run(gnd(t_s, 0.0, 180.0, true), &mut io, &mut said);
        }
        assert_eq!(io.started, vec![0, 1]);
        assert_eq!(
            status().as_deref(),
            Some("lesson: full flight, takeoff"),
            "{said:?}"
        );
        let mut gs = 0.0;
        while gs < 58.0 {
            t_s += 1.0;
            run(gnd(t_s, gs, 92.0, true), &mut io, &mut said);
            gs += 3.0;
        }
        let mut agl = 10.0;
        while agl < 1100.0 {
            t_s += 1.0;
            let mut s = air(t_s, 100.0 + agl, 92.0, 73.0);
            s.agl_ft = agl;
            s.vsi_fpm = 700.0;
            run(s, &mut io, &mut said);
            agl += 700.0 / 60.0;
        }
        assert!(
            said.contains(&"A thousand feet. That completes the takeoff.".to_string()),
            "{said:?}"
        );
        assert!(
            said.iter().any(|l| l.starts_with("Your flight.")),
            "{said:?}"
        );
        assert_eq!(status().as_deref(), Some("lesson: full flight, enroute"));
        // Cruise, then a final: the landing is taken.
        for _ in 0..30 {
            t_s += 1.0;
            run(air(t_s, 3000.0, 92.0, 105.0), &mut io, &mut said);
        }
        let mut agl = 700.0;
        while agl > 0.0 {
            t_s += 1.0;
            let mut s = air(t_s, 100.0 + agl, 92.0, 66.0);
            s.agl_ft = agl;
            s.vsi_fpm = -250.0;
            run(s, &mut io, &mut said);
            agl -= 250.0 / 60.0;
        }
        assert!(
            said.contains(&"On final. I have the landing.".to_string()),
            "{said:?}"
        );
        t_s += 1.0;
        run(gnd(t_s, 55.0, 92.0, true), &mut io, &mut said);
        assert!(said.iter().any(|l| l.starts_with("Down.")), "{said:?}");
        // Roll out, taxi in, stop: the shutdown list, then the flight is over.
        for _ in 0..25 {
            t_s += 1.0;
            run(gnd(t_s, 0.0, 92.0, true), &mut io, &mut said);
        }
        for _ in 0..4 {
            t_s += 1.0;
            run(gnd(t_s, 0.0, 92.0, false), &mut io, &mut said);
        }
        assert_eq!(io.started, vec![0, 1, 2], "{said:?}");
        let last = said.last().unwrap();
        assert!(last.starts_with("That's the flight. Taxi: to standard. Takeoff: to standard. Landing: to standard. Lists done: before taxi, before takeoff, shutdown."), "{last}");
        assert!(!active(), "back to idle");
    }

    #[test]
    fn the_full_flight_starts_at_before_taxi_with_the_engine_running() {
        let _g = crate::mode::TEST_LOCK.lock().unwrap();
        {
            let mut l = lesson();
            l.stage = Stage::Idle;
            l.refs = refs172();
            l.level = Level::Private;
        }
        tick(gnd(0.0, 0.0, 180.0, true));
        let opener = handle(Request::FullFlight, 0.0);
        assert!(
            opener.starts_with("Full flight from here. Engine's running"),
            "{opener}"
        );
        assert_eq!(
            status().as_deref(),
            Some("lesson: full flight, before taxi checklist")
        );
        assert_eq!(request("teach me a full flight"), Some(Request::FullFlight));
        assert_eq!(
            request("lesson from cold and dark"),
            Some(Request::FullFlight)
        );
        assert!(handle(Request::Start(Task::Taxi), 0.0).contains("on the full flight"));
        let end = handle(Request::End, 0.0);
        assert!(end.starts_with("Lesson over. Nothing graded yet."), "{end}");
        assert!(!active());
        // Airborne, the full flight is refused.
        tick(air(1.0, 3000.0, 90.0, 100.0));
        assert!(handle(Request::FullFlight, 1.0).contains("starts on the ramp"));
    }

    #[test]
    fn the_refusal_names_the_action_and_the_way_back() {
        let first = refusal("FLAPS > 1", true);
        assert!(
            first.starts_with(
                "Instructor mode: I don't touch the controls. Flaps 1: that's your hands."
            ),
            "{first}"
        );
        assert!(first.contains("Settings"));
        assert_eq!(refusal("HEADING > 270", false), "Heading 270: your hands.");
    }

    #[test]
    fn every_task_briefs_without_dashes_for_every_level() {
        let entry = air(0.0, 3500.0, 90.0, 100.0);
        for task in Task::ALL {
            for level in [Level::Private, Level::Commercial] {
                let t = task.assign(&entry, &refs172(), level);
                let b = task.brief(&t, level);
                assert!(
                    !b.contains('\u{2014}') && !b.contains('\u{2013}') && !b.contains(" - "),
                    "{b}"
                );
                assert!(b.contains("standards:"), "{b}");
            }
        }
        let c = Task::SteepTurns.assign(&entry, &refs172(), Level::Commercial);
        assert_eq!(c.bank_deg, Some(50.0));
        assert_eq!(c.ias_kts, Some(95.0), "at or below Va, rounded down to 5");
    }

    // ---------------------------------------------------- the demonstration

    /// A clean slate for a demonstration test: instructor mode, no lesson,
    /// the loop flags down. The caller holds `mode::TEST_LOCK`.
    fn demo_reset() {
        use std::sync::atomic::Ordering::SeqCst;
        crate::mode::set(crate::mode::Mode::Instructor);
        crate::mode::close_demonstration();
        TEST_DEMO_RUNNING.store(false, SeqCst);
        TEST_DEMO_STOPPED.store(false, SeqCst);
        let mut l = lesson();
        l.stage = Stage::Idle;
        l.refs = refs172();
        l.level = Level::Private;
        l.latest = None;
        l.last_grade = None;
        l.pending_command = None;
    }

    fn demo_done() {
        use std::sync::atomic::Ordering::SeqCst;
        TEST_DEMO_RUNNING.store(false, SeqCst);
        lesson().stage = Stage::Idle;
        crate::mode::set(crate::mode::Mode::Both);
    }

    #[test]
    fn show_me_is_a_closed_vocabulary() {
        let _g = crate::mode::TEST_LOCK.lock().unwrap();
        demo_reset();
        crate::mode::set(crate::mode::Mode::Both);
        assert_eq!(request("show me the weather"), None);
        assert_eq!(
            request("show me a takeoff"),
            None,
            "a task named outside instructor mode is not a lesson word"
        );
        crate::mode::set(crate::mode::Mode::Instructor);
        assert_eq!(
            request("show me a takeoff"),
            Some(Request::ShowMe(Some(Task::Takeoff)))
        );
        assert_eq!(
            request("can you demonstrate the landing"),
            Some(Request::ShowMe(Some(Task::Landing)))
        );
        assert_eq!(
            request("show me how to taxi"),
            Some(Request::ShowMe(Some(Task::Taxi)))
        );
        assert_eq!(
            request("show me the traffic pattern"),
            Some(Request::ShowMe(Some(Task::Pattern)))
        );
        assert_eq!(request("show me"), None, "no lesson standing, no task");
        assert_eq!(request("show me the weather"), None);
        assert_eq!(request("my controls"), None, "no demonstration running");
        lesson().stage = Stage::Briefed {
            task: Task::Landing,
        };
        assert_eq!(request("show me"), Some(Request::ShowMe(None)));
        assert_eq!(
            request("could you demonstrate it first please"),
            Some(Request::ShowMe(None))
        );
        assert_eq!(
            request("show me how it's done"),
            Some(Request::ShowMe(None))
        );
        assert_eq!(
            request("show me the fuel"),
            None,
            "a bare show-me carries nothing else"
        );
        assert_eq!(request("my controls"), None, "briefed is not demonstrating");
        demo_done();
    }

    /// Every demonstration walks the panel button's own words through the
    /// tool door: the spoken gate passes them and the grammar picks the
    /// very tool the permit opens.
    #[test]
    fn every_demonstration_is_the_panel_request_for_its_tool() {
        let profile = crate::aircraft::load("cessna_172sp").expect("cessna profile");
        let reg = crate::tools::Registry::for_aircraft(&profile);
        let mut shown = 0;
        for task in Task::ALL {
            let Some((tool, phrase)) = task.demonstration() else {
                continue;
            };
            shown += 1;
            assert!(crate::router::looks_like_command(phrase), "{phrase}");
            let calls = crate::grammar::try_parse(phrase, &reg, &profile)
                .unwrap_or_else(|| panic!("{phrase} does not parse"));
            assert_eq!(calls.len(), 1, "{phrase}");
            assert_eq!(calls[0].name, tool, "{phrase}");
        }
        assert_eq!(shown, 4, "taxi, takeoff, pattern, landing");
        assert_eq!(Task::SteepTurns.demonstration(), None, "air work is hands");
    }

    #[test]
    fn a_takeoff_demonstration_flies_narrates_and_hands_back_to_the_brief() {
        use std::sync::atomic::Ordering::SeqCst;
        let _g = crate::mode::TEST_LOCK.lock().unwrap();
        demo_reset();
        tick(gnd(0.0, 0.0, 90.0, true));
        assert!(handle(Request::Start(Task::Takeoff), 0.5).starts_with("Normal takeoff"));
        assert_eq!(request("show me"), Some(Request::ShowMe(None)));
        let line = handle(Request::ShowMe(None), 1.0);
        assert!(
            line.starts_with("My controls. I'll show you a normal takeoff."),
            "{line}"
        );
        // The main loop routes the button's words, once.
        assert_eq!(take_demo_command(), Some("take off for me"));
        assert_eq!(take_demo_command(), None);
        // The hands gate: that tool once, nothing else, the duties still off.
        assert!(!crate::mode::tool_may_actuate("set_flaps", &json!({})));
        assert!(crate::mode::tool_may_actuate("auto_takeoff", &json!({})));
        assert!(!crate::mode::tool_may_actuate("auto_takeoff", &json!({})));
        assert!(!crate::mode::crew_actuates());
        assert_eq!(status().as_deref(), Some("lesson: demonstrating takeoff"));
        assert_eq!(
            request("begin"),
            None,
            "nothing briefed while demonstrating"
        );
        assert_eq!(
            handle(Request::Interim, 1.5),
            "I'm flying this one. Watch, and you'll fly it next."
        );
        // The automation flies: the roll, the rotation at 55, the climb.
        TEST_DEMO_RUNNING.store(true, SeqCst);
        let mut said = Vec::new();
        let mut t = 2.0;
        let mut gs = 0.0;
        while gs < 58.0 {
            gs += 3.0;
            let mut s = gnd(t, gs, 90.0, true);
            s.ias_kts = gs;
            said.extend(tick(s));
            t += 1.0;
        }
        let mut agl = 0.0;
        while agl < 400.0 {
            agl += 40.0;
            let mut s = air(t, 100.0 + agl, 90.0, 70.0);
            s.agl_ft = agl;
            s.vsi_fpm = 700.0;
            said.extend(tick(s));
            t += 1.0;
        }
        assert_eq!(said.len(), 4, "one line per teaching point: {said:?}");
        assert!(said[0].ends_with("Rotation at 55 knots."), "{said:?}");
        assert!(said[1].starts_with("Power set"), "{said:?}");
        assert!(said[2].starts_with("Rotate."), "{said:?}");
        assert_eq!(
            said[3],
            "Positive rate. Pitch for 74 knots, runway heading, and trim."
        );
        // The loop hands over: two quiet seconds end the demonstration.
        TEST_DEMO_RUNNING.store(false, SeqCst);
        let mut end = Vec::new();
        for _ in 0..4 {
            let mut s = air(t, 520.0, 90.0, 74.0);
            s.agl_ft = 420.0;
            end.extend(tick(s));
            t += 1.0;
        }
        assert_eq!(
            end,
            vec![
                "That's the demonstration. Your controls. Set up for it again and say begin."
                    .to_string()
            ]
        );
        // The lesson stands briefed on the same task, the permit shut, and
        // nothing of the demonstration graded to the student.
        assert!(matches!(
            lesson().stage,
            Stage::Briefed {
                task: Task::Takeoff
            }
        ));
        assert!(lesson().last_grade.is_none());
        assert_eq!(crate::mode::demonstration_tool(), None);
        assert!(!crate::mode::tool_may_actuate(
            "auto_takeoff",
            &json!({ "off": true })
        ));
        demo_done();
    }

    #[test]
    fn the_pilot_takes_a_demonstration_back_and_one_that_never_starts_resumes() {
        use std::sync::atomic::Ordering::SeqCst;
        let _g = crate::mode::TEST_LOCK.lock().unwrap();
        demo_reset();
        tick(air(0.0, 2500.0, 270.0, 90.0));
        // Straight from idle, naming the task.
        let line = handle(Request::ShowMe(Some(Task::Landing)), 1.0);
        assert!(line.contains("a normal approach and landing"), "{line}");
        assert_eq!(take_demo_command(), Some("land the aircraft"));
        TEST_DEMO_RUNNING.store(true, SeqCst);
        let said = tick(air(2.0, 2400.0, 270.0, 65.0));
        assert_eq!(
            said,
            vec![
                "A stabilised approach: the aim point fixed, 65 knots, small corrections early."
                    .to_string()
            ]
        );
        assert_eq!(request("my controls"), Some(Request::TakeBack));
        assert_eq!(request("I have control"), Some(Request::TakeBack));
        assert_eq!(
            handle(Request::TakeBack, 3.0),
            "Your controls. Say begin when you're ready, and fly it the same way."
        );
        assert!(TEST_DEMO_STOPPED.load(SeqCst), "the loop was stood down");
        assert_eq!(crate::mode::demonstration_tool(), None);
        assert!(matches!(
            lesson().stage,
            Stage::Briefed {
                task: Task::Landing
            }
        ));
        // A demonstration whose tool refused: the loop never runs.
        TEST_DEMO_RUNNING.store(false, SeqCst);
        handle(Request::ShowMe(None), 10.0);
        let mut said = Vec::new();
        let mut t = 11.0;
        while t < 10.0 + DEMO_START_S + 2.0 {
            said.extend(tick(air(t, 2400.0, 270.0, 90.0)));
            t += 1.0;
        }
        assert_eq!(
            said,
            vec!["The demonstration didn't start. Your controls. Say begin when you're ready, and fly it the same way."
                .to_string()]
        );
        // The pilot's cancel at the tool door ends it in their hands.
        handle(Request::ShowMe(None), 50.0);
        TEST_DEMO_RUNNING.store(true, SeqCst);
        tick(air(51.0, 2400.0, 270.0, 65.0));
        assert!(crate::mode::tool_may_actuate(
            "autoland",
            &json!({ "cancel": true })
        ));
        TEST_DEMO_RUNNING.store(false, SeqCst);
        let mut said = Vec::new();
        for i in 0..4 {
            said.extend(tick(air(52.0 + i as f64, 2400.0, 270.0, 65.0)));
        }
        assert_eq!(
            said,
            vec![
                "Your controls. Say begin when you're ready, and fly it the same way.".to_string()
            ]
        );
        // "End the lesson" mid-demonstration stands the loop down too.
        handle(Request::ShowMe(None), 60.0);
        TEST_DEMO_STOPPED.store(false, SeqCst);
        assert_eq!(handle(Request::End, 61.0), "Lesson over. Your controls.");
        assert!(TEST_DEMO_STOPPED.load(SeqCst));
        assert!(!active());
        assert_eq!(crate::mode::demonstration_tool(), None);
        demo_done();
    }

    #[test]
    fn only_the_crews_tasks_are_demonstrated_and_a_students_attempt_stays_theirs() {
        let _g = crate::mode::TEST_LOCK.lock().unwrap();
        demo_reset();
        tick(air(0.0, 3500.0, 90.0, 100.0));
        let no = handle(Request::ShowMe(Some(Task::SteepTurns)), 0.5);
        assert!(no.starts_with("I can't demonstrate steep turns"), "{no}");
        assert_eq!(take_demo_command(), None, "nothing routed");
        assert_eq!(crate::mode::demonstration_tool(), None, "nothing opened");
        let air_pattern = handle(Request::ShowMe(Some(Task::Pattern)), 0.6);
        assert!(
            air_pattern
                .starts_with("To show you the traffic pattern, I fly that one from the ground"),
            "{air_pattern}"
        );
        // A student flying the landing asks to see it: their attempt is
        // closed as theirs, and the demonstration opens.
        handle(Request::Start(Task::Landing), 1.0);
        handle(Request::Begin, 1.0);
        for i in 2..6 {
            tick(air(i as f64, 3400.0, 90.0, 66.0));
        }
        assert!(lesson().last_grade.is_none());
        assert!(handle(Request::ShowMe(None), 6.0).starts_with("My controls."));
        assert!(
            lesson().last_grade.is_some(),
            "the attempt so far is the student's"
        );
        assert_eq!(crate::mode::demonstration_tool(), Some("autoland"));
        assert_eq!(handle(Request::Start(Task::Climb), 6.5), DEMO_BUSY);
        // The mode changing under it retires the permit with the lesson.
        assert!(abandon("mode_change"));
        assert_eq!(crate::mode::demonstration_tool(), None);
        assert_eq!(take_demo_command(), None);
        // Outside instructor mode nothing is demonstrated.
        crate::mode::set(crate::mode::Mode::Both);
        let off = handle(Request::ShowMe(Some(Task::Takeoff)), 7.0);
        assert!(
            off.starts_with("Demonstrations run in instructor mode"),
            "{off}"
        );
        demo_done();
    }
}
