//! Shared instrument and confirmed-command guards for flight loops.
/// A missing/non-finite instrument is never a control-law input.
pub(crate) fn measurements<const N: usize>(
    kind: &str,
    values: [anyhow::Result<f64>; N],
) -> Option<[f64; N]> {
    let values = values.map(|v| v.ok().filter(|v| v.is_finite()));
    if values.iter().any(Option::is_none) {
        crate::blackbox::record(
            kind,
            serde_json::json!({
                "event": "missing_measurement", "values": values.as_slice(),
            }),
        );
        return None;
    }
    Some(values.map(Option::unwrap))
}

/// Only suppress a later write once the sim has confirmed this intent.
pub(crate) async fn set_confirmed(
    kind: &str,
    xp: &dyn crate::xplane::XPlane,
    name: &str,
    want: f64,
    last: &mut Option<f64>,
    change: f64,
) -> bool {
    if last.is_some_and(|v| (v - want).abs() <= change) {
        return true;
    }
    let result = async {
        xp.set_float(name, want).await?;
        xp.get_float(name).await
    }
    .await;
    let actual = result.ok().filter(|v| v.is_finite());
    if actual.is_some_and(|v| (v - want).abs() <= 1.0) {
        *last = Some(want);
        true
    } else {
        *last = None;
        crate::blackbox::record(
            kind,
            serde_json::json!({
                "event": "write_unconfirmed", "dataref": name, "want": want, "actual": actual,
            }),
        );
        false
    }
}
