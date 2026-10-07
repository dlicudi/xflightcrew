//! The instruments: the gyro and the altimeter.

use super::*;

/// The gyro.
pub(super) fn gyro(line: &Line) -> Option<ToolCall> {
    let t = line.t;

    if has(t, "gyro") {
        // align/sync/set gyro, also catches mishears like "think/zinc gyro".
        return Some(call("align_gyro", json!({})));
    }

    None
}

/// The altimeter setting.
pub(super) fn altimeter(line: &Line) -> Option<ToolCall> {
    let t = line.t;

    // altimeter / barometer
    if has(t, "altimeter") || has(t, "barometer") || has(t, "baro") || has(t, "qnh") {
        if let Some(inhg) = altimeter_inhg(t) {
            return Some(call("set_barometer", json!({ "inhg": inhg })));
        }
        // No setting spoken ("set the altimeter", "altimeter to field") → dial in
        // the current local QNH. But a number that just didn't parse as a valid
        // setting ("altimeter 50") falls to the LLM rather than silently setting it.
        if !t.chars().any(|c| c.is_ascii_digit()) {
            return Some(call("set_altimeter_current", json!({})));
        }
    }

    None
}
