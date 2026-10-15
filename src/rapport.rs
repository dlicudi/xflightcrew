//! The copilot as a person who was there: remembers the flight, says a word
//! about the landing, offers what the pilot is about to want, and keeps
//! anything that can wait until the flight deck is quiet.
//!
//! One state machine ([`Rapport`]) fed a [`View`] by the monitor tick, like
//! the cabin's. It SEES every tick (a landing under autoland is still a
//! landing) and only SPEAKS through [`Rapport::tick`]'s return, which the
//! monitor voices. Every rule is deterministic and every line is built from
//! a measured fact; nothing here touches a control.
//!
//! - **Memory**: takeoffs, landings, go-arounds and the field, per flight and
//!   per session. "That's our second go-around." "Back where we started."
//!   The wheels are the FLIGHT MODEL's (`flight::Flight`): its touchdown,
//!   bounce, touch and go and time aloft, never a second detector here that
//!   could disagree with it (#85).
//! - **The landing word**: the sink at the wheels, by the instructor's own
//!   scorer (`instructor::touchdown_sink_fpm`, through the model's
//!   `Touchdown`), said once the roll is done
//!   and only when it is worth saying. Praise goes to the pilot, never to an
//!   automation; a hard landing is said at every chattiness level.
//! - **Offers**: the next flap as the speed comes inside its placard, the
//!   gear under its limit, the arrival brief thirty miles out. With the
//!   pilot flying only; an offer is a sentence, never an action (the
//!   `cockpit_auto` rule): the pilot's "flaps ten" is an ordinary command.
//! - **Asides**: a line that can wait (a remark, a memory) is held while the
//!   workload is high (`tempo::busy_now`) and said once it has been calm for
//!   [`CALM_S`], several joined into one ("By the way, ...").

use std::sync::Mutex;
use std::time::{Duration, Instant};

use crate::persona::{Chatter, Dialect, Persona};

/// A remark held longer than this is stale and dropped.
pub const ASIDE_TTL_S: f64 = 300.0;
/// Held asides go out after the workload has been low this long.
pub const CALM_S: f64 = 8.0;
/// An aside held longer than this gets "By the way" in front: it is no
/// longer part of the moment it was about.
pub const BY_THE_WAY_S: f64 = 20.0;
/// The landing word waits for the roll to be done.
pub const REMARK_KTS: f64 = 30.0;
/// A touchdown this close to where the flight took off is the same field.
pub const SAME_FIELD_M: f64 = 5_500.0;
/// A same-field flight shorter than this is a circuit, not a trip.
pub const CIRCUIT_S: f64 = 12.0 * 60.0;
/// Sink bands at the wheels, fpm (positive).
pub const GREASER_FPM: f64 = 120.0;
pub const NICE_FPM: f64 = 250.0;
pub const FIRM_FPM: f64 = 400.0;
pub const HARD_FPM: f64 = 600.0;
/// The arrival brief is offered inside this band of distance to go, nm.
pub const BRIEF_OFFER_NM: (f64, f64) = (20.0, 40.0);
/// How long an offer stands waiting for "go ahead" / "not now".
pub const OFFER_TTL: Duration = Duration::from_secs(90);

/// What the monitor tick sees, one sample.
#[derive(Clone, Copy, Debug, Default)]
pub struct View {
    pub t_s: f64,
    pub on_ground: bool,
    pub groundspeed_kts: f64,
    pub agl_ft: f64,
    pub vsi_fpm: f64,
    pub ias_kts: f64,
    pub lat: f64,
    pub lon: f64,
    /// The flap HANDLE, 0..1 (what a detent's ratio is written to).
    pub flap_ratio: f64,
    pub gear_down: bool,
    pub automation_flying: bool,
    /// The pilot's hand on the yoke under the automation lately
    /// (`monitor::inputs::pilot_hands_recent`): a landing then is theirs.
    pub pilot_hands: bool,
    pub dest_dist_nm: Option<f64>,
    pub crashed: bool,
    /// Instructor mode: the instructor grades the landing, and a copilot
    /// offering flaps in a lesson would be flying it for the student.
    pub instructing: bool,
    /// `tempo::busy_now`.
    pub busy: bool,
    /// The monitor's quiet gate (a recovery, autoland, the panel's mute).
    pub quiet: bool,
}

/// One flap detent, as the offer needs it.
#[derive(Clone, Debug, PartialEq)]
pub struct Detent {
    pub spoken: String,
    pub ratio: f64,
    pub vfe_kts: Option<f64>,
}

/// The airframe facts the offers need, rebuilt per profile swap.
#[derive(Clone, Debug, Default)]
pub struct Airframe {
    pub detents: Vec<Detent>,
    pub retractable_gear: bool,
    pub vlo_kts: Option<f64>,
    pub rotorcraft: bool,
}

impl Airframe {
    pub fn from_profile(p: &crate::aircraft::AircraftProfile) -> Self {
        let mut detents: Vec<Detent> = p
            .flaps
            .iter()
            .flat_map(|f| f.detents.iter())
            .map(|d| Detent {
                spoken: d.spoken.clone(),
                ratio: d.ratio,
                vfe_kts: d.vfe_kts,
            })
            .collect();
        detents.sort_by(|a, b| a.ratio.total_cmp(&b.ratio));
        Airframe {
            detents,
            retractable_gear: p.retractable_gear,
            vlo_kts: p.vlo_kts,
            rotorcraft: p.rotorcraft,
        }
    }
}

/// A standing question the pilot can answer "go ahead" / "not now".
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Offer {
    ArrivalBrief,
}

/// What a tick decided to say.
#[derive(Clone, Debug, PartialEq)]
pub enum Line {
    /// Say it now.
    Now(String),
    /// Say it now, and it is a question (`offer()` stands for [`OFFER_TTL`]).
    Offer(Offer, String),
}

impl Line {
    pub fn text(&self) -> &str {
        match self {
            Line::Now(t) | Line::Offer(_, t) => t,
        }
    }
}

/// One touchdown, as the remark and the memory need it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Landing {
    pub t_s: f64,
    pub sink_fpm: f64,
    pub by_automation: bool,
    pub airborne_s: f64,
    pub same_field: bool,
    /// Landings at this field this session, this one included.
    pub landings_here: u32,
    /// Times the wheels left the runway again and came back between the
    /// touchdown and the remark (the flight model's `Bounce`).
    pub bounces: u32,
}

#[derive(Default)]
pub struct Rapport {
    /// The landing booked at the model's touchdown, waiting for the roll.
    pending: Option<Landing>,
    /// Session memory: the field of the last landing, and how many there.
    field: Option<(f64, f64)>,
    landings_here: u32,
    go_arounds: u32,
    /// Descending low (an approach): the lowest height seen.
    approach_low: Option<f64>,
    /// Detents the speed has been above the placard of, this arrival.
    above_vfe: Vec<String>,
    offered: Vec<String>,
    gear_offered: bool,
    brief_offered: bool,
    named_once: bool,
    asides: Vec<(String, f64)>,
    calm_since: Option<f64>,
}

impl Rapport {
    pub fn new() -> Self {
        Self::default()
    }

    /// One monitor tick: `v` and the flight model `f` after this tick's
    /// sample, with the `events` that tick produced (`v.t_s` on the model's
    /// clock). Returns what to say now; everything deferrable that arrives
    /// while the flight deck is busy is held as an aside.
    pub fn tick(
        &mut self,
        v: &View,
        f: &crate::flight::Flight,
        events: &[crate::flight::Event],
        a: &Airframe,
        p: &Persona,
    ) -> Vec<Line> {
        use crate::flight::Event;
        let mut out = Vec::new();
        let mut asides: Vec<String> = Vec::new();
        let airborne = f.phase().airborne();

        // The wheels, as the flight model saw them: one answer to when the
        // aircraft landed, bounced or went flying again.
        for e in events {
            match e {
                // A touch and go (or a new flight before the word was
                // said): the landing word rides the climb-out.
                Event::TouchAndGo | Event::Takeoff => {
                    if let Some(l) = self.pending.take() {
                        asides.extend(landing_lines(&l, p, &mut self.named_once));
                    }
                }
                // Counted at the second contact, so a touch and go (which
                // never comes back) is not one.
                Event::Bounce => {
                    if let Some(l) = self.pending.as_mut() {
                        l.bounces += 1;
                    }
                }
                Event::Touchdown(td) => self.touch_down(v, f.memory(), td, false),
                Event::Impact(td) => self.touch_down(v, f.memory(), td, true),
                _ => {}
            }
        }

        if airborne {
            // A go-around: low and descending, then climbing away without
            // touching down. Pilot or automation, the count is the flight's.
            if !v.on_ground && v.agl_ft < 400.0 && v.vsi_fpm < -200.0 {
                let low = self.approach_low.map_or(v.agl_ft, |l| l.min(v.agl_ft));
                self.approach_low = Some(low);
            }
            if let Some(low) = self.approach_low {
                if !v.on_ground && v.vsi_fpm > 300.0 && v.agl_ft > low + 200.0 {
                    self.approach_low = None;
                    self.go_arounds += 1;
                    self.reset_arrival();
                    if let Some(l) = go_around_line(self.go_arounds, v.automation_flying, p) {
                        asides.push(l);
                    }
                }
            }
        }

        // The landing word, once the roll is done.
        if let Some(l) = self.pending {
            if v.crashed || v.instructing {
                self.pending = None;
            } else if v.on_ground && v.groundspeed_kts < REMARK_KTS && v.t_s - l.t_s >= 3.0 {
                self.pending = None;
                let lines = landing_lines(&l, p, &mut self.named_once);
                if v.busy || v.quiet {
                    asides.extend(lines);
                } else {
                    out.extend(lines.into_iter().map(Line::Now));
                }
            }
        }

        // Offers: pilot flying, airborne, not in a lesson, not in a hurry.
        let may_offer = airborne
            && !v.automation_flying
            && !v.instructing
            && !a.rotorcraft
            && !v.crashed
            && p.chatter >= Chatter::Normal;
        if airborne {
            self.track_placards(v, a);
        }
        if may_offer && !(v.busy || v.quiet) {
            for line in self.offers(v, a, p) {
                if let Line::Offer(kind, _) = &line {
                    *lock(&OFFER) = Some((*kind, Instant::now()));
                }
                out.push(line);
            }
        }

        // Asides: held while busy, said together once it has been calm.
        let now = v.t_s;
        self.asides.extend(asides.into_iter().map(|s| (s, now)));
        self.asides.retain(|(_, t)| now - *t < ASIDE_TTL_S);
        if v.busy || v.quiet || v.crashed {
            self.calm_since = None;
        } else {
            let calm = *self.calm_since.get_or_insert(now);
            if now - calm >= CALM_S && !self.asides.is_empty() {
                let held: Vec<(String, f64)> = std::mem::take(&mut self.asides);
                out.push(Line::Now(join_asides(&held, now)));
            }
        }
        out
    }

    /// Book the model's touchdown. The sink is the model's (the
    /// instructor's scorer at its first contact); the time aloft runs from
    /// its takeoff, touch and go or join (`Memory::aloft_t`); the same field
    /// is where this flight took off (`Memory::origin`; a flight joined in
    /// the air has none). An `impact` (climbing into the ground, or past
    /// `flight::IMPACT_SINK_FPM`) is booked but never counted as a landing
    /// at the field, and gets a word only when it came down hard: a strike
    /// climbing into the ground is no "barely felt that".
    fn touch_down(
        &mut self,
        v: &View,
        m: &crate::flight::Memory,
        td: &crate::flight::Touchdown,
        impact: bool,
    ) {
        let sink = (-td.sink_fpm).max(0.0);
        let here = td.lat.zip(td.lon).unwrap_or((v.lat, v.lon));
        let near = |(lat, lon): (f64, f64)| {
            crate::nav::haversine_nm(lat, lon, here.0, here.1) * 1852.0 < SAME_FIELD_M
        };
        let same_field = m.origin.is_some_and(near);
        if !impact {
            match self.field {
                Some(f) if near(f) => self.landings_here += 1,
                _ => {
                    self.field = Some(here);
                    self.landings_here = 1;
                }
            }
        }
        let l = Landing {
            t_s: td.t,
            sink_fpm: sink,
            by_automation: v.automation_flying && !v.pilot_hands,
            airborne_s: td.t - m.aloft_t.unwrap_or(td.t),
            same_field,
            landings_here: self.landings_here,
            bounces: 0,
        };
        *lock(&LAST_LANDING_BY_AUTOMATION) = Some(l.by_automation);
        crate::blackbox::record(
            "rapport",
            serde_json::json!({
                "event": "touchdown", "sink_fpm": l.sink_fpm.round(),
                "by_automation": l.by_automation, "airborne_s": l.airborne_s.round(),
                "same_field": l.same_field, "landings_here": l.landings_here,
                "go_arounds": self.go_arounds, "impact": impact,
            }),
        );
        self.go_arounds = 0;
        self.approach_low = None;
        self.reset_arrival();
        self.brief_offered = false;
        let worth_a_word = !impact || sink >= HARD_FPM;
        self.pending = (!v.crashed && !v.instructing && worth_a_word).then_some(l);
    }

    fn reset_arrival(&mut self) {
        self.above_vfe.clear();
        self.offered.clear();
        self.gear_offered = false;
    }

    /// Which placards the speed has been above this arrival: a flap is only
    /// offered when the speed comes DOWN through its limit, never because a
    /// slow aircraft has always been inside it.
    fn track_placards(&mut self, v: &View, a: &Airframe) {
        for d in &a.detents {
            if let Some(vfe) = d.vfe_kts {
                if v.ias_kts > vfe + 2.0 && !self.above_vfe.contains(&d.spoken) {
                    self.above_vfe.push(d.spoken.clone());
                }
            }
        }
    }

    fn offers(&mut self, v: &View, a: &Airframe, p: &Persona) -> Vec<Line> {
        let mut out = Vec::new();
        // Never up high: a descent at the cruise is no arrival, and no flap
        // goes out above the airframes' 20,000 ft limit ("Speed's good for
        // flaps one" at FL360, the 737 descending to recover its speed,
        // 2026-10-06).
        let arriving = v.agl_ft < FLAP_OFFER_MAX_AGL_FT
            && (v.vsi_fpm < -250.0
                || v.dest_dist_nm.is_some_and(|d| d < 12.0)
                || (v.agl_ft < 3000.0 && v.flap_ratio > 0.01));

        // The next detent, once the speed has come down through its placard.
        let next = a.detents.iter().find(|d| d.ratio > v.flap_ratio + 0.02);
        if let Some(d) = next {
            if let Some(vfe) = d.vfe_kts {
                let crossed = self.above_vfe.contains(&d.spoken);
                if arriving
                    && crossed
                    && v.ias_kts <= vfe - 3.0
                    && !self.offered.contains(&d.spoken)
                {
                    self.offered.push(d.spoken.clone());
                    out.push(Line::Now(flap_offer_line(&d.spoken, p.dialect)));
                }
            }
        }

        // The gear, under its limit, low and coming down.
        if a.retractable_gear && !v.gear_down {
            if let Some(vlo) = a.vlo_kts {
                let close = v.dest_dist_nm.is_none_or(|d| d < 12.0);
                if !self.gear_offered
                    && arriving
                    && close
                    && v.agl_ft < 2500.0
                    && v.vsi_fpm < -200.0
                    && v.ias_kts < vlo - 5.0
                {
                    self.gear_offered = true;
                    out.push(Line::Now(gear_offer_line(p.dialect)));
                }
            }
        }

        // The arrival brief, once a flight, thirty-odd miles out.
        if let Some(d) = v.dest_dist_nm {
            if !self.brief_offered
                && d >= BRIEF_OFFER_NM.0
                && d <= BRIEF_OFFER_NM.1
                && v.agl_ft > 1500.0
            {
                self.brief_offered = true;
                let n = ((d / 5.0).round() * 5.0) as u32;
                let line = format!(
                    "About {} miles to run. Want the arrival brief?",
                    words_for(n)
                );
                out.push(Line::Offer(Offer::ArrivalBrief, line));
            }
        }
        out
    }
}

fn lock<T>(m: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

static LAST_LANDING_BY_AUTOMATION: Mutex<Option<bool>> = Mutex::new(None);
static OFFER: Mutex<Option<(Offer, Instant)>> = Mutex::new(None);

/// The flight's facts the social replies need.
pub fn social_facts() -> crate::persona::Social {
    crate::persona::Social {
        last_landing_by_automation: *lock(&LAST_LANDING_BY_AUTOMATION),
    }
}

/// The unexpired offer, if any.
pub fn offer() -> Option<Offer> {
    let mut o = lock(&OFFER);
    match *o {
        Some((kind, at)) if at.elapsed() < OFFER_TTL => Some(kind),
        _ => {
            *o = None;
            None
        }
    }
}

pub fn clear_offer() {
    *lock(&OFFER) = None;
}

#[cfg(test)]
pub fn set_offer_for_test() {
    *lock(&OFFER) = Some((Offer::ArrivalBrief, Instant::now()));
}

/// Several held asides as one line. Pure.
pub fn join_asides(held: &[(String, f64)], now: f64) -> String {
    let joined = held
        .iter()
        .map(|(s, _)| s.as_str())
        .collect::<Vec<_>>()
        .join(" ");
    let oldest = held.iter().map(|(_, t)| *t).fold(now, f64::min);
    if now - oldest >= BY_THE_WAY_S {
        let mut c = joined.chars();
        match c.next() {
            Some(f) => format!("By the way, {}{}", f.to_lowercase(), c.as_str()),
            None => joined,
        }
    } else {
        joined
    }
}

fn round10(fpm: f64) -> u32 {
    ((fpm / 10.0).round() * 10.0).max(0.0) as u32
}

/// The word about the landing, then what the flight's memory adds. Pure but
/// for the name: the pilot is called by name on the first praise of the
/// session only (`named_once`), since every time is a salesman.
pub fn landing_lines(l: &Landing, p: &Persona, named_once: &mut bool) -> Vec<String> {
    let mut out = Vec::new();
    let name = if *named_once {
        None
    } else {
        p.pilot_name.as_deref()
    };
    if let Some((line, praised)) = landing_remark(l, p.chatter, p.dialect, name) {
        if praised && name.is_some() {
            *named_once = true;
        }
        out.push(line);
    }
    if let Some(m) = memory_line(l, p.chatter) {
        out.push(m);
    }
    out
}

/// The remark alone: `(line, used_the_name)`, or `None` when the landing is
/// not worth a word at this chattiness. Pure.
pub fn landing_remark(
    l: &Landing,
    chatter: Chatter,
    d: Dialect,
    name: Option<&str>,
) -> Option<(String, bool)> {
    let r = round10(l.sink_fpm);
    let brit = d == Dialect::British;
    // A bounce is what the pilot felt, whatever the first contact measured.
    if l.bounces > 0 && l.sink_fpm < HARD_FPM && chatter > Chatter::Quiet {
        let line = match (l.by_automation, l.bounces, brit) {
            (true, 1, _) => "Sorry about that bounce.",
            (true, _, _) => "Sorry, we bounced that one.",
            (false, 1, true) => "Bit of a bounce, but we're down.",
            (false, 1, false) => "Little bounce, but we're down.",
            (false, _, _) => "A couple of bounces there, but we're down.",
        };
        return Some((line.to_string(), false));
    }
    if l.sink_fpm >= HARD_FPM {
        return Some((
            format!("That was a hard one, about {r} feet a minute. Worth a look at the gear."),
            false,
        ));
    }
    if chatter == Chatter::Quiet {
        return None;
    }
    if l.by_automation {
        // Nobody compliments their own landing.
        return match l.sink_fpm {
            s if s >= FIRM_FPM => Some((
                format!("Firmer than I'd like, about {r} feet a minute."),
                false,
            )),
            s if s < NICE_FPM && chatter == Chatter::Chatty => {
                // A contact that rounds to nothing (t=1790530411: the vario
                // read +48 at the wheels) has no number worth saying.
                let line = if r < 10 {
                    "Smooth one, barely a sink.".to_string()
                } else {
                    format!("Smooth one, about {r} feet a minute.")
                };
                Some((line, false))
            }
            _ => None,
        };
    }
    match l.sink_fpm {
        s if s < GREASER_FPM => {
            let (word, rest) = if brit {
                ("Lovely", "Barely felt that.")
            } else {
                ("Smooth", "Barely felt it.")
            };
            Some(match name {
                Some(n) => (format!("{word}, {n}. {rest}"), true),
                None => (format!("{word}. {rest}"), false),
            })
        }
        s if s < NICE_FPM => Some(match name {
            Some(n) => (format!("Nice one, {n}. About {r} feet a minute."), true),
            None => (format!("Nice one, about {r} feet a minute."), false),
        }),
        s if s < FIRM_FPM => (chatter == Chatter::Chatty)
            .then(|| (format!("Decent one, about {r} feet a minute."), false)),
        _ => Some((
            if brit {
                format!("Bit firm, that. About {r} feet a minute.")
            } else {
                format!("Little firm. About {r} feet a minute.")
            },
            false,
        )),
    }
}

/// What the flight's memory adds at a landing, or `None`. Pure.
pub fn memory_line(l: &Landing, chatter: Chatter) -> Option<String> {
    if chatter == Chatter::Quiet {
        return None;
    }
    if l.same_field && l.airborne_s >= CIRCUIT_S {
        return Some("Back where we started.".to_string());
    }
    if chatter < Chatter::Chatty {
        return None;
    }
    if l.same_field && l.landings_here >= 2 {
        return Some(format!("That's {} landings.", words_for(l.landings_here)));
    }
    if !l.same_field && l.airborne_s >= 20.0 * 60.0 {
        return Some(format!("{} in the air.", duration_words(l.airborne_s)));
    }
    None
}

/// The go-around remark: the pilot's first earns a word, every flight's
/// second and later are counted. Pure.
pub fn go_around_line(n: u32, by_automation: bool, p: &Persona) -> Option<String> {
    if p.chatter == Chatter::Quiet {
        return None;
    }
    match n {
        0 => None,
        1 if by_automation => None,
        1 => Some("Good call on the go-around.".to_string()),
        n => Some(format!("That's our {} go-around.", ordinal(n))),
    }
}

/// No flap is offered above this height.
const FLAP_OFFER_MAX_AGL_FT: f64 = 20_000.0;

pub fn flap_offer_line(spoken: &str, d: Dialect) -> String {
    match d {
        Dialect::British => format!("Speed's good for flaps {spoken}."),
        Dialect::American => format!("We're good for flaps {spoken}."),
    }
}

pub fn gear_offer_line(d: Dialect) -> String {
    match d {
        Dialect::British => "Below gear speed, whenever you're ready.".to_string(),
        Dialect::American => "We're under gear speed, whenever you want it.".to_string(),
    }
}

fn ordinal(n: u32) -> String {
    match n {
        2 => "second".into(),
        3 => "third".into(),
        4 => "fourth".into(),
        5 => "fifth".into(),
        n => format!("{n}th"),
    }
}

/// Small numbers as words (the TTS reads digits too, but a count is a word).
pub fn words_for(n: u32) -> String {
    const W: [&str; 21] = [
        "zero",
        "one",
        "two",
        "three",
        "four",
        "five",
        "six",
        "seven",
        "eight",
        "nine",
        "ten",
        "eleven",
        "twelve",
        "thirteen",
        "fourteen",
        "fifteen",
        "sixteen",
        "seventeen",
        "eighteen",
        "nineteen",
        "twenty",
    ];
    match n {
        0..=20 => W[n as usize].to_string(),
        25 => "twenty five".into(),
        30 => "thirty".into(),
        35 => "thirty five".into(),
        40 => "forty".into(),
        45 => "forty five".into(),
        50 => "fifty".into(),
        n => n.to_string(),
    }
}

/// A flight time as people say it, to five minutes: "Forty minutes", "An
/// hour and ten", "Two hours". Pure.
pub fn duration_words(secs: f64) -> String {
    let mins = ((secs / 60.0 / 5.0).round() * 5.0) as u32;
    let (h, m) = (mins / 60, mins % 60);
    let hours = match h {
        0 => String::new(),
        1 => "An hour".to_string(),
        h => format!("{} hours", cap(&words_for(h))),
    };
    match (h, m) {
        (0, m) => format!("{} minutes", cap(&words_for(m))),
        (_, 0) => hours,
        (_, m) => format!("{hours} and {}", words_for(m)),
    }
}

fn cap(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
        None => String::new(),
    }
}

/// "Brief the arrival", "arrival brief", "approach brief", "brief me on the
/// approach". Closed: brief/briefing plus arrival/approach, nothing that
/// commands.
pub fn arrival_brief_request(text: &str) -> bool {
    let t = text.to_lowercase();
    let words: Vec<&str> = t
        .split(|c: char| !c.is_ascii_alphanumeric())
        .filter(|w| !w.is_empty())
        .collect();
    let brief = words.iter().any(|w| matches!(*w, "brief" | "briefing"));
    let what = words.iter().any(|w| matches!(*w, "arrival" | "approach"));
    const OK: &[&str] = &[
        "brief", "briefing", "arrival", "approach", "the", "me", "on", "us", "an", "a", "give",
        "can", "you", "could", "please", "lets", "have", "do", "go", "ahead", "with", "our", "for",
        "run", "through",
    ];
    brief && what && words.len() <= 9 && words.iter().all(|w| OK.contains(w))
}

/// The arrival brief for the loaded plan's destination: the weather on file,
/// the runway into the wind, what approaches the field offers, the field's
/// elevation. Blocking (an apt.dat pass): run it off the command loop's
/// async thread.
pub fn arrival_brief(apt_dat: &str, cifp: &str, fixes: &str, xplane_root: &str) -> String {
    let Some(fp) = crate::simbrief::loaded_plan() else {
        return "No flight plan loaded, so I have no arrival to brief.".to_string();
    };
    let dest = fp.destination.clone();
    let airport = match crate::nav::find_airport(apt_dat, &dest) {
        Ok(Some(a)) => a,
        _ => return format!("I can't find {dest} in the airport data to brief it."),
    };
    let name = crate::party_line::spoken_airport(&airport.name).to_string();
    let mut parts = vec![format!("Arrival brief for {name}.")];
    match crate::weather::briefing(xplane_root, &dest, &name) {
        Ok(w) | Err(w) => parts.push(w),
    }
    // The crew's own pick (`full_auto::wind_runway`): its calm, and only a
    // runway the field would assign (never the grass beside the main strip).
    let wind = crate::weather::surface_wind(xplane_root, &dest);
    if let Some(end) = crate::full_auto::wind_runway(&airport.runways, wind) {
        parts.push(format!(
            "Expect runway {}, into the wind.",
            crate::crew_radio::spell_runway(&end)
        ));
    }
    if let Ok(line) = crate::full_auto::brief_arrival(apt_dat, cifp, fixes, &dest, None) {
        parts.push(line);
    }
    parts.push(format!("Field elevation {:.0} feet.", airport.elev_ft));
    parts.join(" ")
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::flight::{Flight, Sample};

    /// The rapport as the monitor runs it: the flight model ticked first on
    /// the same sample, its events handed over.
    struct Rig {
        r: Rapport,
        f: Flight,
    }

    impl Rig {
        fn new() -> Rig {
            Rig {
                r: Rapport::new(),
                f: Flight::new(),
            }
        }
        fn tick(&mut self, v: &View, a: &Airframe, p: &Persona) -> Vec<Line> {
            self.tick_sample(&sample(v), v, a, p)
        }
        fn tick_sample(&mut self, s: &Sample, v: &View, a: &Airframe, p: &Persona) -> Vec<Line> {
            let ev = self.f.tick(s);
            self.r.tick(v, &self.f, &ev, a, p)
        }
    }

    impl std::ops::Deref for Rig {
        type Target = Rapport;
        fn deref(&self) -> &Rapport {
            &self.r
        }
    }

    /// The flight model's sample for a synthetic view: engines running, the
    /// field at sea level.
    fn sample(v: &View) -> Sample {
        Sample {
            t: v.t_s,
            on_ground: v.on_ground,
            gs_kts: v.groundspeed_kts,
            ias_kts: v.ias_kts,
            vsi_fpm: v.vsi_fpm,
            agl_ft: v.agl_ft,
            alt_ft: v.agl_ft,
            engine_running: true,
            lat: Some(v.lat),
            lon: Some(v.lon),
            crashed: Some(v.crashed),
            ..Sample::default()
        }
    }

    fn persona(chatter: Chatter) -> Persona {
        Persona {
            chatter,
            dialect: Dialect::British,
            pilot_name: None,
        }
    }

    fn c172() -> Airframe {
        Airframe {
            detents: vec![
                Detent {
                    spoken: "up".into(),
                    ratio: 0.0,
                    vfe_kts: None,
                },
                Detent {
                    spoken: "ten".into(),
                    ratio: 0.333,
                    vfe_kts: Some(110.0),
                },
                Detent {
                    spoken: "twenty".into(),
                    ratio: 0.666,
                    vfe_kts: Some(85.0),
                },
            ],
            retractable_gear: true,
            vlo_kts: Some(140.0),
            rotorcraft: false,
        }
    }

    /// Fly a whole sortie through the tracker at 3 Hz: a takeoff, a climb,
    /// an arrival, a touchdown at `sink`, a roll to a stop. Returns every
    /// line said, in order, with the time it was said.
    fn fly(r: &mut Rig, p: &Persona, sink: f64, automation: bool) -> Vec<(f64, Line)> {
        fly_hands(r, p, sink, automation, false)
    }

    /// [`fly`] with the pilot's hand on the yoke under the automation.
    fn fly_hands(
        r: &mut Rig,
        p: &Persona,
        sink: f64,
        automation: bool,
        pilot_hands: bool,
    ) -> Vec<(f64, Line)> {
        let a = c172();
        let mut said = Vec::new();
        let mut t = 0.0;
        let step = |r: &mut Rig, v: View, said: &mut Vec<(f64, Line)>| {
            for l in r.tick(&v, &a, p) {
                said.push((v.t_s, l));
            }
        };
        let base = View {
            automation_flying: automation,
            pilot_hands,
            lat: 36.15,
            lon: -5.35,
            ..View::default()
        };
        // Roll and lift off.
        for i in 0..30 {
            t += 0.33;
            let gs = 20.0 + i as f64 * 2.0;
            let v = View {
                t_s: t,
                on_ground: true,
                groundspeed_kts: gs,
                ias_kts: gs,
                busy: gs > 30.0,
                ..base
            };
            step(r, v, &mut said);
        }
        // Climb to 3,000 ft and cruise fast.
        let mut agl = 0.0;
        while agl < 3000.0 {
            t += 0.33;
            agl += 10.0;
            let v = View {
                t_s: t,
                agl_ft: agl,
                vsi_fpm: 1800.0,
                ias_kts: 120.0,
                groundspeed_kts: 120.0,
                busy: agl < 1000.0,
                ..base
            };
            step(r, v, &mut said);
        }
        // Down to the ground, slowing, gear down at 1,500 ft.
        while agl > 0.0 {
            t += 0.33;
            agl = (agl - 4.0).max(0.0);
            let ias = (70.0 + agl / 30.0).min(130.0);
            let vsi = if agl < 60.0 { -sink } else { -700.0 };
            let v = View {
                t_s: t,
                agl_ft: agl,
                vsi_fpm: vsi,
                ias_kts: ias,
                groundspeed_kts: ias,
                gear_down: agl < 1500.0,
                dest_dist_nm: Some(agl / 300.0),
                busy: agl < 1000.0,
                on_ground: agl <= 0.0,
                ..base
            };
            step(r, v, &mut said);
        }
        // Roll out and stop, then sit quietly.
        for i in 0..80 {
            t += 0.33;
            let gs = (65.0 - i as f64 * 2.0).max(0.0);
            let v = View {
                t_s: t,
                on_ground: true,
                groundspeed_kts: gs,
                ias_kts: gs,
                busy: gs > 30.0,
                ..base
            };
            step(r, v, &mut said);
        }
        said
    }

    #[test]
    fn a_nice_pilot_landing_gets_one_word_after_the_roll() {
        let mut r = Rig::new();
        let said = fly(&mut r, &persona(Chatter::Normal), 180.0, false);
        let words: Vec<&str> = said.iter().map(|(_, l)| l.text()).collect();
        let remark: Vec<&&str> = words
            .iter()
            .filter(|w| w.contains("feet a minute"))
            .collect();
        assert_eq!(remark.len(), 1, "{words:?}");
        assert!(remark[0].starts_with("Nice one, about 180"), "{words:?}");
        // The offers came on the arrival, before the landing: the flaps at
        // their placard, the gear under its limit.
        assert!(words.iter().any(|w| w.contains("flaps ten")), "{words:?}");
        assert!(words.iter().any(|w| w.contains("gear speed")), "{words:?}");
        // Only the NEXT detent is ever offered: nobody jumps to twenty.
        assert!(!words.iter().any(|w| w.contains("twenty")), "{words:?}");
    }

    #[test]
    fn an_automation_is_never_complimented_and_a_hard_landing_always_said() {
        let mut r = Rig::new();
        let said = fly(&mut r, &persona(Chatter::Normal), 180.0, true);
        assert!(
            said.iter()
                .all(|(_, l)| !l.text().contains("feet a minute")),
            "{said:?}"
        );
        // Offers are for the pilot flying.
        assert!(
            said.iter().all(|(_, l)| !l.text().contains("flaps")),
            "{said:?}"
        );
        let mut r = Rig::new();
        let said = fly(&mut r, &persona(Chatter::Quiet), 700.0, true);
        assert!(
            said.iter()
                .any(|(_, l)| l.text().contains("hard one, about 700")),
            "{said:?}"
        );
    }

    /// 2026-10-15: the pilot held the yoke through the autoland's flare and
    /// the landing was booked `by_automation`. A hand on the yoke makes it
    /// the pilot's landing.
    #[test]
    fn a_landing_with_the_pilots_hand_on_the_yoke_is_the_pilots() {
        let mut r = Rig::new();
        let said = fly_hands(&mut r, &persona(Chatter::Normal), 180.0, true, true);
        assert!(
            said.iter()
                .any(|(_, l)| l.text().starts_with("Nice one, about 180")),
            "{said:?}"
        );
    }

    #[test]
    fn quiet_volunteers_nothing_that_is_not_owed() {
        let mut r = Rig::new();
        let said = fly(&mut r, &persona(Chatter::Quiet), 180.0, false);
        assert!(said.is_empty(), "{said:?}");
    }

    #[test]
    fn asides_wait_for_the_calm_and_go_out_together() {
        let held = vec![
            ("That's our second go-around.".to_string(), 0.0),
            ("Back where we started.".to_string(), 5.0),
        ];
        assert_eq!(
            join_asides(&held, 30.0),
            "By the way, that's our second go-around. Back where we started."
        );
        assert_eq!(join_asides(&held[1..], 10.0), "Back where we started.");

        // Through the tracker: a go-around below a thousand feet is held
        // until it has been calm for CALM_S.
        let a = c172();
        let p = persona(Chatter::Normal);
        let mut r = Rig::new();
        let mut t = 0.0;
        let mut said = Vec::new();
        let mut tick =
            |r: &mut Rig, agl: f64, vsi: f64, busy: bool, said: &mut Vec<(f64, String)>| {
                t += 1.0;
                let v = View {
                    t_s: t,
                    agl_ft: agl,
                    vsi_fpm: vsi,
                    ias_kts: 90.0,
                    groundspeed_kts: 90.0,
                    gear_down: true,
                    busy,
                    automation_flying: true,
                    ..View::default()
                };
                for l in r.tick(&v, &a, &p) {
                    said.push((t, l.text().to_string()));
                }
            };
        for _ in 0..5 {
            tick(&mut r, 2000.0, 0.0, false, &mut said);
        }
        for g in 0..2 {
            // Down to 150 ft, then away.
            for agl in (150..400).rev().step_by(50) {
                tick(&mut r, agl as f64, -600.0, true, &mut said);
            }
            for agl in (150..1000).step_by(100) {
                tick(&mut r, agl as f64, 900.0, true, &mut said);
            }
            assert_eq!(r.go_arounds, g + 1);
        }
        assert!(said.is_empty(), "held while busy: {said:?}");
        for _ in 0..(CALM_S as usize + 2) {
            tick(&mut r, 2000.0, 0.0, false, &mut said);
        }
        assert_eq!(said.len(), 1, "{said:?}");
        assert!(said[0].1.contains("second go-around"), "{said:?}");
    }

    #[test]
    fn a_bounce_is_not_a_takeoff() {
        let a = c172();
        let p = persona(Chatter::Normal);
        let mut r = Rig::new();
        // An arrival joined at 300 ft, down to the wheels.
        let mut t = 80.0;
        for i in 0..60 {
            t += 0.33;
            let v = View {
                t_s: t,
                agl_ft: 300.0 - i as f64 * 5.0,
                vsi_fpm: -400.0,
                groundspeed_kts: 60.0,
                ias_kts: 60.0,
                ..View::default()
            };
            r.tick(&v, &a, &p);
        }
        t += 0.33;
        let mut v = View {
            t_s: t,
            on_ground: true,
            groundspeed_kts: 60.0,
            ..View::default()
        };
        r.tick(&v, &a, &p);
        assert!(r.pending.is_some());
        // Two seconds in the air at 35 ft: a bounce.
        for _ in 0..6 {
            t += 0.33;
            v = View {
                t_s: t,
                agl_ft: 35.0,
                groundspeed_kts: 58.0,
                ..View::default()
            };
            r.tick(&v, &a, &p);
        }
        assert!(!r.f.phase().airborne(), "{:?}", r.f.phase());
        assert!(r.pending.is_some(), "the landing word survives a bounce");
        v = View {
            t_s: t + 0.33,
            on_ground: true,
            groundspeed_kts: 55.0,
            ..View::default()
        };
        r.tick(&v, &a, &p);
        assert_eq!(
            r.pending.map(|l| l.bounces),
            Some(1),
            "counted at the second contact"
        );
    }

    /// Down to the wheels at `vsi` and roll to a stop, pilot flying; every
    /// line said.
    fn meet_the_ground(vsi: f64) -> (Rig, Vec<String>) {
        let a = c172();
        let p = persona(Chatter::Normal);
        let mut r = Rig::new();
        let mut said = Vec::new();
        let mut t = 50.0;
        for i in 0..30 {
            t += 0.33;
            let v = View {
                t_s: t,
                agl_ft: 200.0 - i as f64 * 6.0,
                vsi_fpm: vsi,
                groundspeed_kts: 70.0,
                ias_kts: 70.0,
                gear_down: true,
                ..View::default()
            };
            said.extend(r.tick(&v, &a, &p).into_iter().map(|l| l.text().to_string()));
        }
        for i in 0..60 {
            t += 0.33;
            let gs = (68.0 - i as f64 * 2.0).max(0.0);
            let v = View {
                t_s: t,
                on_ground: true,
                vsi_fpm: if i == 0 { vsi } else { 0.0 },
                groundspeed_kts: gs,
                ias_kts: gs,
                gear_down: true,
                ..View::default()
            };
            said.extend(r.tick(&v, &a, &p).into_iter().map(|l| l.text().to_string()));
        }
        (r, said)
    }

    /// 2026-10-06: the 737 at FL360 slowed through flaps one's 250 kt
    /// placard and descended to recover its speed; "Speed's good for flaps
    /// one" was offered at the cruise. Only below 20,000 ft.
    #[test]
    fn no_flap_is_offered_at_the_cruise() {
        let a = c172();
        let p = persona(Chatter::Normal);
        let fast = |agl: f64| View { agl_ft: agl, ias_kts: 150.0, vsi_fpm: -1100.0, ..View::default() };
        let slow = |agl: f64| View { agl_ft: agl, ias_kts: 80.0, vsi_fpm: -1100.0, ..View::default() };
        let flaps = |r: &mut Rapport, agl: f64| {
            r.track_placards(&fast(agl), &a);
            r.offers(&slow(agl), &a, &p)
                .iter()
                .any(|l| l.text().contains("flaps"))
        };
        assert!(!flaps(&mut Rapport::new(), 35_700.0), "at the cruise");
        assert!(flaps(&mut Rapport::new(), 3_000.0), "on the arrival");
    }

    #[test]
    fn the_ground_met_climbing_is_no_landing_to_praise() {
        // The flight model reads a contact climbing at 500 fpm as an
        // IMPACT: the rapport books it, counts no landing at the field and
        // never calls it smooth (its own detector read it as a 0 fpm
        // greaser).
        let (r, said) = meet_the_ground(500.0);
        assert_eq!(r.f.memory().impacts, 1);
        assert!(said.is_empty(), "{said:?}");
        assert_eq!(r.landings_here, 0);
        // A strike past the impact sink is still said, as the hard landing
        // it is.
        let (r, said) = meet_the_ground(-1_600.0);
        assert_eq!(r.f.memory().impacts, 1);
        assert_eq!(
            said,
            vec!["That was a hard one, about 1600 feet a minute. Worth a look at the gear."]
        );
        // And an ordinary one is the model's touchdown, at the model's sink.
        let (r, said) = meet_the_ground(-200.0);
        let td = r.f.memory().last_touchdown.unwrap();
        assert_eq!(td.sink_fpm, -200.0);
        assert_eq!(said, vec!["Nice one, about 200 feet a minute."]);
    }

    #[test]
    fn the_remark_bands() {
        let l = |sink: f64| Landing {
            t_s: 0.0,
            sink_fpm: sink,
            by_automation: false,
            airborne_s: 600.0,
            same_field: false,
            landings_here: 1,
            bounces: 0,
        };
        let say = |sink: f64, c: Chatter| {
            landing_remark(&l(sink), c, Dialect::American, None).map(|(s, _)| s)
        };
        assert_eq!(
            say(90.0, Chatter::Normal).as_deref(),
            Some("Smooth. Barely felt it.")
        );
        assert_eq!(
            say(200.0, Chatter::Normal).as_deref(),
            Some("Nice one, about 200 feet a minute.")
        );
        assert_eq!(say(320.0, Chatter::Normal), None, "unremarkable");
        assert!(say(320.0, Chatter::Chatty).unwrap().starts_with("Decent"));
        assert!(say(480.0, Chatter::Normal).unwrap().contains("firm"));
        assert!(say(650.0, Chatter::Quiet).unwrap().contains("hard one"));
        // The name, on the first praise only.
        let p = Persona {
            chatter: Chatter::Normal,
            dialect: Dialect::British,
            pilot_name: Some("Sam".into()),
        };
        let mut named = false;
        let first = landing_lines(&l(200.0), &p, &mut named);
        assert_eq!(first, vec!["Nice one, Sam. About 200 feet a minute."]);
        let second = landing_lines(&l(100.0), &p, &mut named);
        assert_eq!(second, vec!["Lovely. Barely felt that."]);
        let mut fresh = false;
        assert_eq!(
            landing_lines(&l(100.0), &p, &mut fresh),
            vec!["Lovely, Sam. Barely felt that."]
        );
        let bounced = Landing {
            bounces: 2,
            ..l(150.0)
        };
        assert_eq!(
            say_bounce(&bounced),
            Some("A couple of bounces there, but we're down.".to_string())
        );
        fn say_bounce(l: &Landing) -> Option<String> {
            landing_remark(l, Chatter::Normal, Dialect::American, None).map(|(s, _)| s)
        }
    }

    #[test]
    fn the_memory_says_what_the_flight_was() {
        let l = Landing {
            t_s: 0.0,
            sink_fpm: 300.0,
            by_automation: false,
            airborne_s: 45.0 * 60.0,
            same_field: true,
            landings_here: 1,
            bounces: 0,
        };
        assert_eq!(
            memory_line(&l, Chatter::Normal).as_deref(),
            Some("Back where we started.")
        );
        let circuit = Landing {
            airborne_s: 6.0 * 60.0,
            landings_here: 3,
            ..l
        };
        assert_eq!(memory_line(&circuit, Chatter::Normal), None);
        assert_eq!(
            memory_line(&circuit, Chatter::Chatty).as_deref(),
            Some("That's three landings.")
        );
        let trip = Landing {
            same_field: false,
            airborne_s: 70.0 * 60.0,
            ..l
        };
        assert_eq!(
            memory_line(&trip, Chatter::Chatty).as_deref(),
            Some("An hour and ten in the air.")
        );
        assert_eq!(duration_words(40.0 * 60.0), "Forty minutes");
        assert_eq!(duration_words(120.0 * 60.0), "Two hours");
        assert_eq!(words_for(35), "thirty five");
    }

    #[test]
    fn the_brief_request_is_closed_and_the_runway_is_into_the_wind() {
        assert!(arrival_brief_request("brief the arrival"));
        assert!(arrival_brief_request(
            "can you give me the approach brief please"
        ));
        assert!(arrival_brief_request("arrival briefing"));
        assert!(!arrival_brief_request("brief the flight plan"));
        assert!(!arrival_brief_request("arm the approach"));
        assert!(!arrival_brief_request(
            "brief the approach and set flaps ten"
        ));
    }

    /// The same-field test reads the shared great circle (`nav::haversine_nm`)
    /// where it had its own flat-earth copy: at a field's scale they agree to
    /// a metre, so no landing changes field.
    #[test]
    fn the_same_field_distance_is_the_shared_great_circle() {
        let old = |a: (f64, f64), b: (f64, f64)| {
            let (la1, lo1) = (a.0.to_radians(), a.1.to_radians());
            let (la2, lo2) = (b.0.to_radians(), b.1.to_radians());
            let x = (lo2 - lo1) * ((la1 + la2) / 2.0).cos();
            let y = la2 - la1;
            (x * x + y * y).sqrt() * 6_371_000.0
        };
        for (a, b) in [
            ((36.675, -4.499), (36.675, -4.437)),
            ((36.675, -4.499), (36.724, -4.499)),
            ((51.148, -0.190), (51.180, -0.150)),
            ((-33.94, 151.17), (-33.97, 151.21)),
        ] {
            let new = crate::nav::haversine_nm(a.0, a.1, b.0, b.1) * 1852.0;
            assert!(
                (new - old(a, b)).abs() < 1.0,
                "{a:?} {b:?}: {new} vs {}",
                old(a, b)
            );
        }
    }

    /// The brief names the runway the crew will fly (`full_auto::wind_runway`),
    /// not a pick of its own: 3.5 kt is the crew's calm (its own 3 kt line
    /// named an end the arrival would not take), and a grass strip beside a
    /// hard one is never the field's runway however the wind lies.
    #[test]
    fn the_brief_names_the_crews_runway() {
        let rwy = |a: &str, b: &str, lat_a: f64, lon_a: f64, lat_b: f64, lon_b: f64, surface| {
            crate::nav::Runway {
                ident_a: a.into(),
                ident_b: b.into(),
                lat_a,
                lon_a,
                lat_b,
                lon_b,
                width_m: 30.0,
                surface,
            }
        };
        let field = vec![
            rwy("09", "27", 50.0, -1.02, 50.0, -0.98, 1),
            rwy("18", "36", 50.01, -1.0, 49.99, -1.0, 3),
        ];
        let pick = |w| crate::full_auto::wind_runway(&field, Some(w));
        assert_eq!(pick((250.0, 12.0)).as_deref(), Some("27"));
        assert_eq!(pick((100.0, 8.0)).as_deref(), Some("09"));
        assert_eq!(pick((250.0, 3.5)), None, "under the crew's calm");
        assert_eq!(
            pick((200.0, 12.0)).as_deref(),
            Some("27"),
            "the grass 18 is not assignable"
        );
        assert_eq!(crate::full_auto::wind_runway(&field, None), None);
        let src = crate::test_support::module_source("rapport");
        let brief = &src[src.find("pub fn arrival_brief(").unwrap()..];
        assert!(brief.contains("crate::full_auto::wind_runway("));
    }

    #[test]
    fn the_arrival_brief_is_offered_once_a_flight() {
        let a = c172();
        let p = persona(Chatter::Normal);
        let mut r = Rig::new();
        let mut offers = 0;
        for i in 0..200 {
            let v = View {
                t_s: i as f64,
                agl_ft: 8000.0,
                ias_kts: 120.0,
                groundspeed_kts: 120.0,
                dest_dist_nm: Some(45.0 - i as f64 * 0.2),
                ..View::default()
            };
            for l in r.tick(&v, &a, &p) {
                if let Line::Offer(Offer::ArrivalBrief, s) = l {
                    offers += 1;
                    assert_eq!(s, "About forty miles to run. Want the arrival brief?");
                }
            }
        }
        assert_eq!(offers, 1);
        clear_offer();
    }

    /// Five recorded landings (bench/landings, slim `state` records): a
    /// pilot's, two full-auto circuits (one with a go-around), a hard one on
    /// a flight joined in the air, and a contact the vario read climbing.
    const LANDINGS: &str = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/bench/landings/rapport_landings.jsonl"));

    /// Replay the recorded landings through the rapport, chatty, and write
    /// down every touchdown it booked and every line it said. Each window
    /// is a fresh session.
    fn replay_landings() -> Vec<String> {
        let a = Airframe::default();
        let p = persona(Chatter::Chatty);
        let mut out = Vec::new();
        let mut r = Rig::new();
        let mut t0 = 0.0;
        let mut booked: Option<Landing> = None;
        for line in LANDINGS.lines() {
            let v: serde_json::Value = serde_json::from_str(line).unwrap();
            if v["kind"] == "window" {
                out.push(format!("window {}", v["name"].as_str().unwrap()));
                r = Rig::new();
                t0 = f64::NAN;
                booked = None;
                continue;
            }
            let f = |k: &str| v[k].as_f64().unwrap_or(0.0);
            if t0.is_nan() {
                t0 = f("t");
            }
            let view = View {
                t_s: f("t"),
                on_ground: v["ground"].as_bool().unwrap_or(false),
                groundspeed_kts: f("gs"),
                agl_ft: f("agl"),
                vsi_fpm: if v["vs"].is_number() { f("vs") } else { f("vsi") },
                ias_kts: f("ias"),
                lat: f("lat"),
                lon: f("lon"),
                flap_ratio: f("flap"),
                gear_down: true,
                automation_flying: v["ctl"]
                    .as_str()
                    .is_some_and(|c| c.starts_with("Automation")),
                crashed: v["crashed"].as_bool().unwrap_or(false),
                ..View::default()
            };
            let s = Sample::from_state_record(&v).unwrap();
            let said = r.tick_sample(&s, &view, &a, &p);
            if let Some(l) = r.pending {
                if booked.map(|b| b.t_s) != Some(l.t_s) {
                    out.push(format!(
                        "touchdown +{:.0}s sink {:.0} auto {} airborne {:.0}s same_field {} here {}",
                        l.t_s - t0,
                        l.sink_fpm,
                        l.by_automation,
                        (l.airborne_s / 10.0).round() * 10.0,
                        l.same_field,
                        l.landings_here
                    ));
                }
                booked = Some(l);
            }
            for s in said {
                out.push(format!("said +{:.0}s {}", view.t_s - t0, s.text()));
            }
        }
        out
    }

    #[test]
    fn the_recorded_landings_say_what_they_said() {
        let got = replay_landings();
        let want = vec![
            "window pilot_130",
            "touchdown +3378s sink 126 auto false airborne 3360s same_field false here 1",
            "said +3392s Nice one, about 130 feet a minute.",
            "said +3392s 55 minutes in the air.",
            "window circuit_43",
            "touchdown +381s sink 278 auto true airborne 360s same_field true here 1",
            "window joined_hard_769",
            "touchdown +198s sink 769 auto true airborne 180s same_field false here 1",
            "said +219s That was a hard one, about 770 feet a minute. Worth a look at the gear.",
            "window circuit_ga_883",
            "touchdown +908s sink 244 auto true airborne 880s same_field true here 1",
            "said +926s Smooth one, about 240 feet a minute.",
            "said +926s Back where we started.",
            "window contact_plus48",
            "touchdown +2392s sink 0 auto true airborne 2370s same_field false here 1",
            "said +2416s Smooth one, barely a sink.",
            "said +2416s Forty minutes in the air.",
        ];
        assert_eq!(got, want, "{got:#?}");
    }

    #[test]
    fn no_line_carries_a_dash() {
        for d in [Dialect::British, Dialect::American] {
            assert!(!flap_offer_line("ten", d).contains('\u{2014}'));
            assert!(!gear_offer_line(d).contains('\u{2014}'));
        }
        for n in 1..6 {
            if let Some(l) = go_around_line(n, false, &persona(Chatter::Normal)) {
                assert!(!l.contains('\u{2014}'), "{l}");
            }
        }
    }
}
