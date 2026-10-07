//! The altitude the PILOT hands the follower: the one seam between the
//! copilot's altitude commands (the autopilot tool, the monitor's cabin
//! descent) and the procedure follower that flies them. The copilot side
//! calls [`take`]; the follower reads [`held`] and clears it on its exits.
//! Neither names the other (pillars audit 2026-10-07, knot 1).
//!
//! The follower is the ONE writer of the altitude dial and rewrites it every
//! tick. So a spoken "climb to four thousand" was obeyed by the autopilot tool
//! and undone within 500 ms, twice, while the aircraft oscillated at 220 ft
//! over rising ground (2026-09-13): the pilot's own override was the thing the
//! automation was strongest against. It is now not a fight over a dataref: the
//! tool hands the number here and the follower flies it as its target until the
//! pilot says another one, keeping every leg's lateral steering.

static PILOT_ALT: std::sync::Mutex<Option<f64>> = std::sync::Mutex::new(None);

/// Is the follower flying (its loop flag, through the owners' one page)?
fn follower_flying() -> bool {
    crate::flight_state::Owner::Procedure.active()
}

/// Hand the follower an altitude the pilot asked for. `false` (and nothing
/// taken) when no procedure is flying, so the caller drives the autopilot
/// itself the way it always has.
pub fn take(ft: f64) -> bool {
    if !follower_flying() {
        return false;
    }
    *PILOT_ALT.lock().unwrap_or_else(|e| e.into_inner()) = Some(ft);
    crate::blackbox::record(
        "procedure",
        serde_json::json!({"event": "pilot_altitude", "ft": ft}),
    );
    true
}

/// The pilot's commanded altitude, if they have taken the vertical.
pub fn held() -> Option<f64> {
    if !follower_flying() {
        return None;
    }
    *PILOT_ALT.lock().unwrap_or_else(|e| e.into_inner())
}

/// Forget the pilot's altitude: the follower's exits and a new run.
pub(crate) fn clear() {
    *PILOT_ALT.lock().unwrap_or_else(|e| e.into_inner()) = None;
}

/// The rate the follower's vertical law will actually fly, fpm (magnitude):
/// what a readback must promise once [`take`] has handed the altitude over,
/// whatever rate the pilot asked for (`procedure::vs_clamp_fpm`).
pub fn follower_rate_fpm() -> f64 {
    crate::procedure::vs_clamp_fpm()
}
