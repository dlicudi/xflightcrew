//! The crew's ground automations: takeoff, line up, backtrack, taxi.

use super::*;

/// The crew drives on the ground: takeoff, line up, backtrack, taxi to parking, auto taxi.
pub(super) fn ground_drive(line: &Line) -> Option<ToolCall> {
    let t = line.t;
    let num = line.num;

    // Auto takeoff, the copilot flies the roll and initial climb. Only
    // first/second-person shapes: a READBACK ("cleared for takeoff runway
    // two seven") and a call to the tower ("ready for takeoff") carry none
    // of these, so neither can ever arm it, the same trap the auto-taxi
    // rules dodge. Abort shapes first, so "abort the takeoff" never re-arms.
    {
        let takeoff_word =
            t.contains("takeoff") || t.contains("take off") || t.contains("take us off");
        if takeoff_word
            && (has(t, "abort") || has(t, "reject") || has(t, "cancel") || has(t, "stop"))
        {
            return Some(call("auto_takeoff", json!({ "off": true })));
        }
        if t == "abort" || t == "abort abort" || t == "abort abort abort" {
            return Some(call("auto_takeoff", json!({ "off": true })));
        }
        // The normalizer peels trailing politeness, so "take off for me"
        // arrives as the bare words, and like bare "taxi", nobody says
        // them to a copilot meaning anything else (a readback always
        // carries "cleared for" plus the runway, so it stays longer).
        if t == "takeoff" || t == "take off"
            || t.contains("auto takeoff") || t.contains("auto take off")
            || t.contains("take off for me") || t.contains("takeoff for me")
            || t.contains("take off for us") || t.contains("takeoff for us")
            || t.contains("take us off") || t.contains("take off when ready")
            || t.contains("you have the takeoff") || t.contains("you have the take off")
            || t.contains("your takeoff") || t.contains("your take off")
            || t.contains("you fly the takeoff") || t.contains("fly the takeoff")
            // Whisper's renderings of the shapes above (live 2026-07-20):
            // "you have the takeoff" → "you have to take off", "your
            // takeoff" → "you'll take off" (apostrophe already stripped),
            // "you handle the takeoff" → "use handle to take off". All
            // still second-person; no readback or tower call carries them.
            || t.contains("you have to take off") || t.contains("youll take off")
            || t.contains("you handle the takeoff") || t.contains("handle the takeoff")
            || t.contains("handle to take off")
        {
            return Some(call("auto_takeoff", json!({})));
        }
    }
    // Line up, the copilot drives from the hold-short position onto the
    // runway centreline. The pilot's READBACK "line up and wait ..." always
    // carries "wait" (and "lining up" is the short echo), so both are
    // excluded; the command shapes are first/second-person. Trailing "for
    // me" peels, so bare "line up" as the whole utterance is the command
    // ("can you line me up to the runway" went unanswered live, 2026-07-20).
    if !has(t, "wait")
        && !t.contains("lining")
        && (t == "line up" || t == "line up to the runway" || t == "line up on the runway"
            || t.contains("line us up") || t.contains("line me up")
            || t.contains("get us lined up") || t.contains("take us onto the runway")
            // "let's line up with the runway" peels its "lets" and arrives
            // as this shape (three phrasings went unanswered live,
            // 2026-07-20). The wait/lining guards above still exclude the
            // LUAW readback and the short echo.
            || (t.starts_with("line up") && has(t, "runway")))
    {
        return Some(call("line_up", json!({})));
    }
    // Backtrack, the full-length line-up (drive to the threshold, turn
    // around). Whole-word "backtrack" so the -ing echo never matches, and
    // the "wait" guard mirrors the LUAW readback trap. The normalizer folds
    // whisper's split "back track".
    if has(t, "backtrack")
        && !has(t, "wait")
        && !t.contains("lining")
        && (t == "backtrack"
            || t.contains("backtrack for me")
            || t.contains("backtrack for us")
            || t.contains("backtrack us")
            || t.contains("backtrack the runway")
            || t.contains("backtrack runway"))
    {
        return Some(call("line_up", json!({ "backtrack": true })));
    }
    // Taxi to PARKING after landing, the copilot requests the taxi-in from ATC
    // and drives to the nearest stand. BEFORE auto_taxi (which is the departure
    // taxi to a holding point). Generic/request-shaped forms only: a specific
    // "taxi to stand 12 via ..." is a taxi READBACK to the tower, not a request
    // to the copilot (the same trap auto_taxi dodges).
    if t.contains("taxi to parking")
        // Whisper, twice live: "request parking to taxi", "talk to taxi".
        || t.contains("parking to taxi")
        || t == "talk to taxi"
        || t.contains("taxi to the stand")
        || t.contains("taxi to the gate")
        || t.contains("taxi to the ramp")
        || t.contains("taxi to the apron")
        || t.contains("taxi us in")
        || t == "taxi in"
        || t.contains("take us to parking")
        || t.contains("take us to the stand")
        || t.contains("take us to the gate")
        || t.contains("take us to the ramp")
        || t.contains("park us")
        || t.contains("park the aircraft")
        || t.contains("park the plane")
    {
        return Some(call("taxi_to_parking", json!({})));
    }
    // Auto taxi, the copilot steers the cleared route to the holding point.
    // Before every speed-hold rule, so "taxi for me at 12 knots" doesn't fall
    // into plain hold_speed. Deliberately NOT matching "taxi to holding point
    // ...", that's the pilot's taxi READBACK to the tower, never a request
    // to drive (the same trap is_taxi_query dodges by being question-shaped).
    if t.contains("auto taxi") || t.contains("autotaxi") {
        return Some(if has(t, "stop") || has(t, "cancel") || has(t, "off") {
            call("auto_taxi", json!({ "off": true }))
        } else if let Some(n) = num {
            call("auto_taxi", json!({ "knots": n }))
        } else {
            call("auto_taxi", json!({}))
        });
    }
    // Bare "taxi" as the whole utterance lands here too: the normalizer
    // peels trailing politeness, so "taxi for me" arrives as just "taxi",
    // and nobody says the bare word to a copilot meaning anything else.
    // The exact "taxi to (the) holding point" with NO specifics is a
    // request too: "let's taxi to holding point" peels its leading "lets"
    // and arrived here as exactly that, then fell into the readback trap
    // and went unanswered (live, 2026-07-20). A real readback always
    // carries the specifics being echoed (the point name, runway, or
    // "via ..."), so anything longer still never arms the drive.
    if t == "taxi"
        || t == "taxi to holding point"
        || t == "taxi to the holding point"
        // The runway named as the destination and nothing else ("Taxi to
        // runway.", Twin Otter, 2026-09-22, followed by three more tries and
        // a give-up). Exact like the holding-point shapes above and for the
        // same reason: a taxi READBACK always carries the specifics being
        // echoed (the runway number, the route, "via ..."), so it is longer
        // than this and never lands here.
        || t == "taxi to runway"
        || t == "taxi to the runway"
        || t == "taxi to the active"
        || t.contains("taxi for me")
        || t.contains("taxi for us")
        || t.contains("you have the taxi")
        || t.contains("you take the taxi")
        || t.contains("you taxi")
        || t.contains("take us to the holding")
        || t.contains("take us to holding")
        || t.contains("taxi us")
    {
        return Some(match num {
            Some(n) => call("auto_taxi", json!({ "knots": n })),
            None => call("auto_taxi", json!({})),
        });
    }

    None
}
