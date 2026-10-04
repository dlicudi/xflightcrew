//! The command loop's word-only branches ([`question_shortcut`]), moved
//! out of `main.rs` verbatim (refactor audit 2026-10-04, phase 2). The
//! crate root re-exports them, so every caller still names `crate::...`.

use super::*;

/// A branch of the command loop that answers (or acts on) a line from its
/// words alone, before the grammar ever sees it. The loop routes through
/// [`question_shortcut`], in this order, and `xvl sortie --check` asks the
/// same function whether a scripted line is taken here instead of reaching
/// its tool: LEMG (2026-09-28) "put us enroute 30 miles before top of
/// descent" was answered "Top of descent about 893 miles ahead" and the
/// setup never ran.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum Shortcut {
    Fuel,
    ReminderCancel,
    ReminderList,
    PlanCheck,
    SystemsCheck,
    PowerLoss,
    ReferenceLimits,
    SystemsSurvey,
    AircraftInfo,
    PlanFlight,
    TopOfDescent,
    RemoteWeather,
    Eta,
    DestinationElevation,
    TakeoffRunway,
    RunwayAdvice,
    LandingReadiness,
    ConfigAdvice,
    Traffic,
    OnCourse,
    NamedFix,
    NavProgress,
    LoadRoute,
    WrongPlan,
    PlanBrief,
}

impl Shortcut {
    /// Every shortcut, in the loop's order.
    #[cfg(test)]
    pub(crate) const ALL: [Shortcut; 25] = [
        Shortcut::Fuel,
        Shortcut::ReminderCancel,
        Shortcut::ReminderList,
        Shortcut::PlanCheck,
        Shortcut::SystemsCheck,
        Shortcut::PowerLoss,
        Shortcut::ReferenceLimits,
        Shortcut::SystemsSurvey,
        Shortcut::AircraftInfo,
        Shortcut::PlanFlight,
        Shortcut::TopOfDescent,
        Shortcut::RemoteWeather,
        Shortcut::Eta,
        Shortcut::DestinationElevation,
        Shortcut::TakeoffRunway,
        Shortcut::RunwayAdvice,
        Shortcut::LandingReadiness,
        Shortcut::ConfigAdvice,
        Shortcut::Traffic,
        Shortcut::OnCourse,
        Shortcut::NamedFix,
        Shortcut::NavProgress,
        Shortcut::LoadRoute,
        Shortcut::WrongPlan,
        Shortcut::PlanBrief,
    ];

    /// The branch's name in a check's table; the local planner's is the
    /// `plan_flight` a spec step names.
    pub(crate) fn name(self) -> &'static str {
        match self {
            Shortcut::Fuel => "fuel question",
            Shortcut::ReminderCancel => "reminder cancel",
            Shortcut::ReminderList => "reminder list",
            Shortcut::PlanCheck => "plan check",
            Shortcut::SystemsCheck => "systems check",
            Shortcut::PowerLoss => "power loss question",
            Shortcut::ReferenceLimits => "reference limits",
            Shortcut::SystemsSurvey => "systems survey",
            Shortcut::AircraftInfo => "aircraft info",
            Shortcut::PlanFlight => "plan_flight",
            Shortcut::TopOfDescent => "top of descent question",
            Shortcut::RemoteWeather => "weather question",
            Shortcut::Eta => "eta question",
            Shortcut::DestinationElevation => "destination elevation question",
            Shortcut::TakeoffRunway => "takeoff runway question",
            Shortcut::RunwayAdvice => "runway advice",
            Shortcut::LandingReadiness => "landing readiness question",
            Shortcut::ConfigAdvice => "config advice",
            Shortcut::Traffic => "traffic question",
            Shortcut::OnCourse => "on course question",
            Shortcut::NamedFix => "named fix question",
            Shortcut::NavProgress => "next waypoint question",
            Shortcut::LoadRoute => "load the route",
            Shortcut::WrongPlan => "wrong plan",
            Shortcut::PlanBrief => "flight plan brief",
        }
    }
}

/// The command loop's pre-grammar shortcut for this line, if one takes it:
/// the loop's decisions without their speech. `plan` is the flight plan on
/// the shelf (`None`: the fix and "load it" readings cannot match). Pure.
pub(crate) fn question_shortcut(
    command: &str,
    registry: &Registry,
    profile: &aircraft::AircraftProfile,
    plan: Option<&simbrief::FlightPlan>,
) -> Option<Shortcut> {
    use Shortcut::*;
    let tl = command.to_ascii_lowercase();
    // A new flight's setup is never a question: "put us enroute 30 miles
    // before top of descent" was answered as one at LEMG (2026-09-28), and
    // "enroute ... before the destination" carries "route" and "the", the
    // flight plan brief's words (`xvl sortie --check`, 2026-09-29).
    if grammar::enroute_setup_request(command) || grammar::approach_setup_request(&tl) {
        return None;
    }
    if is_fuel_query(command) && grammar::try_parse(command, registry, profile).is_none() {
        return Some(Fuel);
    }
    let has = |w: &str| tl.contains(w);
    let any = |ws: &[&str]| ws.iter().any(|w| tl.contains(w));
    // Reminder memory: cancel or list what's armed. "Cancel" wins over
    // arming; a mention naming a condition falls through to arm it.
    if has("reminder") {
        let cancel = [
            "cancel", "clear", "forget", "remove", "delete", "scrap", "never mind", "nevermind",
        ];
        if any(&cancel) {
            return Some(ReminderCancel);
        }
        let arming = ["wake", "descent", "feet", "altitude", "minute", "waypoint", "reach"];
        if !any(&arming) {
            return Some(ReminderList);
        }
    }
    // Before the systems check, whose "check ... aircraft" these words can have.
    if grammar::plan_check_request(&tl) {
        return Some(PlanCheck);
    }
    let systems_check = any(&[
        "all systems",
        "systems check",
        "safe parameter",
        "in the green",
        "status check",
        "status update",
    ])
        // "Anything to check?" / "anything I should check?"
        || (has("anything") && has("check"))
        // "aircraft health check" / "check the aircraft"
        || (has("check") && (has("aircraft") || has("health")))
        // "How's the aircraft health?" / "How's the aircraft doing?"
        || (has("aircraft") && (has("health") || has("doing")));
    if systems_check {
        return Some(SystemsCheck);
    }
    if is_power_loss_query(&tl) {
        return Some(PowerLoss);
    }
    if reference_limits(profile, &tl).is_some() {
        return Some(ReferenceLimits);
    }
    if systems::is_systems_query(&tl) {
        return Some(SystemsSurvey);
    }
    if aircraft_info(profile, &tl).is_some() {
        return Some(AircraftInfo);
    }
    // The local planner ("plan a flight to Malaga", "divert"); its short
    // answer to "to where?" is the loop's, on its own clock.
    if flightplan::parse_request(&tl).is_some() {
        return Some(PlanFlight);
    }
    // "wake me / remind me / let me know / tell me when ..." is a reminder
    // to arm, not a status query to answer now.
    let is_reminder =
        any(&["wake", "remind", "notify", "let me know", "tell me when", "heads up"]);
    if !is_reminder {
        // "top of descent" is reliably misheard, so top+descent loosely.
        let tod = (has("top") && has("descent"))
            || has("t o d")
            || tl.split(|c: char| !c.is_ascii_alphanumeric()).any(|w| w == "tod");
        if tod {
            return Some(TopOfDescent);
        }
        if weather_words(&tl) && remote_weather_target(&tl, plan).is_some() {
            return Some(RemoteWeather);
        }
        if eta_question(&tl) {
            return Some(Eta);
        }
        if (has("destination") || has("arrival"))
            && any(&["altitude", "elevation", "height", "how high"])
        {
            return Some(DestinationElevation);
        }
        if answers::takeoff_runway_query(&tl) {
            return Some(TakeoffRunway);
        }
        if has("runway")
            && any(&["which", "what", "should", "use", "best", "active", "recommend"])
        {
            return Some(RunwayAdvice);
        }
        // Phrased as a question so "set flaps for landing" never lands here.
        if any(&["good for landing", "ready for landing", "ready to land", "good to land"])
            || (has("landing") && has("are we"))
        {
            return Some(LandingReadiness);
        }
        // Guarded so light COMMANDS ("nav lights on") never match.
        if (has("light") || has("configur"))
            && any(&["need", "should", "do we", "are we", "anything"])
        {
            return Some(ConfigAdvice);
        }
        if has("traffic") {
            return Some(Traffic);
        }
        if any(&["on course", "off course", "on track"]) {
            return Some(OnCourse);
        }
        // Only a plan already on the shelf is consulted.
        if plan.is_some_and(|fp| fix_query(&tl, &fp.waypoints).is_some()) {
            return Some(NamedFix);
        }
        if any(&[
            "next waypoint",
            "next fix",
            "eta",
            "how far",
            "time to",
            "distance to",
            "how much left",
            "how much further",
            "how much longer",
        ]) {
            return Some(NavProgress);
        }
    }
    // "route" is reliably misheard as "root"/"road".
    let routeish = any(&["route", "root", "road"]);
    // "load it" / "send that" right after a local plan was briefed.
    let load_followup =
        plan.is_some() && tl.split_whitespace().any(|w| w == "it" || w == "that");
    // Not the load SHEET ("load the fuel from the flight plan").
    let wants_load = !grammar::load_aircraft_request(&tl)
        && any(&["load", "program", "send"])
        && (routeish || load_followup || any(&["plan", "gps", "fms", "flight"]));
    if wants_load {
        return Some(LoadRoute);
    }
    if wrong_plan_complaint(&tl) {
        return Some(WrongPlan);
    }
    // Not when the intent is to FLY it (the navigate tool, full auto).
    let nav_intent = grammar::flight_plan_request(&tl)
        || grammar::resume_plan_request(&tl)
        || any(&["navigate", "follow", "track"])
        || tl
            .split(|c: char| !c.is_ascii_alphanumeric())
            .any(|w| w == "fly" || w == "execute");
    let is_plan = !nav_intent
        && (any(&[
            "flight plan",
            "simbrief",
            "fetch the plan",
            "where are we going",
            "where are we headed",
        ]) || (routeish && any(&["our", "the", "what"])));
    if is_plan {
        return Some(PlanBrief);
    }
    None
}

/// Standard Levenshtein edit distance.
pub(crate) fn levenshtein(a: &str, b: &str) -> usize {
    let a: Vec<char> = a.chars().collect();
    let b: Vec<char> = b.chars().collect();
    let mut prev: Vec<usize> = (0..=b.len()).collect();
    let mut cur = vec![0usize; b.len() + 1];
    for i in 1..=a.len() {
        cur[0] = i;
        for j in 1..=b.len() {
            let cost = if a[i - 1] == b[j - 1] { 0 } else { 1 };
            cur[j] = (prev[j] + 1).min(cur[j - 1] + 1).min(prev[j - 1] + cost);
        }
        std::mem::swap(&mut prev, &mut cur);
    }
    prev[b.len()]
}
