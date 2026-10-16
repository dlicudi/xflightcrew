//! "Direct LESTA": the pilot monitoring does the DIR TO on the FMS and
//! presses LNAV, the way it was done by hand on the stock 737 at FL380
//! (2026-10-16): clear the scratchpad, DIR INTC, pick the fix from the
//! page's own list, EXEC only on a MOD page, LNAV at once (the leg starts
//! at present position; on HDG the aircraft drifts off it within a minute
//! and LNAV only arms). Read back on the GPS's active ident and the LNAV
//! lamp, never assumed.
//!
//! The FMS is the one source: the fix must be listed on the DIRECT-TO page
//! (`plan::best_ident` over its rows), so a plan that lives only in the FMS
//! (xfc restarted mid-flight) still works and nothing outside it is typed.
//! Only airborne, only with CMD in, only where the airframe declares its
//! LNAV (`lnav_command`, the 737 today). LNAV pressed ONCE through the
//! panel owner (`press_lnav`): an ARMED LNAV reads lamp 0 and gpss 1, and a
//! second press disarms it. Blackbox `direct_to`.

use std::time::Duration;

use anyhow::Result;
use async_trait::async_trait;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::autoflight::names;
use crate::datarefs::fms_cdu1_line;
use crate::xplane::XPlane;

const KEY_CLEAR: &str = "sim/FMS/key_clear";
const DIR_INTC: &str = "sim/FMS/dir_intc";
const NEXT_PAGE: &str = "sim/FMS/next";
const EXEC: &str = "sim/FMS/exec";
/// The scratchpad's text line.
const SCRATCHPAD: usize = 13;
/// The DIRECT-TO page's fix rows: LSK 2L to 6L (LSK 1L is the entry box).
const ROW_LSKS: std::ops::RangeInclusive<usize> = 2..=6;
/// Pages of the list looked at, the first included.
const MAX_PAGES: usize = 3;
/// CLR presses before a scratchpad that will not blank is refused.
const MAX_CLEARS: usize = 30;
/// Between CLR presses: the FMS takes them in its flight loop.
const KEY_MS: u64 = 150;
/// After a page change (DIR INTC, NEXT, LSK, EXEC) before it is read.
const PAGE_MS: u64 = 400;

/// A CDU line's bytes as text, the NUL padding and edges trimmed. Pure.
pub fn cdu_text(bytes: &[u8]) -> String {
    String::from_utf8_lossy(bytes)
        .trim_matches('\0')
        .trim()
        .to_string()
}

/// The scratchpad shows nothing: blanks, with or without its brackets. Pure.
pub fn scratchpad_blank(line: &str) -> bool {
    line.trim()
        .trim_start_matches('[')
        .trim_end_matches(']')
        .trim()
        .is_empty()
}

/// The title line says a change waits for EXEC. Pure.
pub fn is_mod(line0: &str) -> bool {
    line0.trim_start().starts_with("MOD")
}

/// The fix a DIRECT-TO row lists, `<LESTA            FL374>`: the word
/// after the `<`, letters and digits only (`<-----` is no fix). Pure.
pub fn row_ident(line: &str) -> Option<&str> {
    let id = line.trim().strip_prefix('<')?.split_whitespace().next()?;
    id.chars().all(|c| c.is_ascii_alphanumeric()).then_some(id)
}

/// What LNAV reads after the press.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
pub enum Lnav {
    Engaged,
    Armed,
    Out,
    Unread,
}

/// From the lamp-or-bitfield read and `gpss_status` (live on the 737:
/// armed is lamp 0 with gpss 1, engaged lamp 1 with gpss 2). Pure.
pub fn lnav_state(engaged: Option<bool>, gpss: Option<f64>) -> Lnav {
    match (engaged, gpss.map(f64::round)) {
        (_, Some(2.0)) => Lnav::Engaged,
        (_, Some(1.0)) => Lnav::Armed,
        (Some(true), _) => Lnav::Engaged,
        (Some(false), _) => Lnav::Out,
        (None, _) => Lnav::Unread,
    }
}

/// The spoken readback. Pure.
pub fn readback(ident: &str, on_fix: bool, lnav: Lnav) -> String {
    if !on_fix {
        return format!(
            "Executed direct {ident}, but the FMS isn't showing it as the active waypoint."
        );
    }
    match lnav {
        Lnav::Engaged => format!("Direct {ident}, LNAV."),
        Lnav::Armed => format!("Direct {ident}, LNAV armed."),
        Lnav::Out => format!("Direct {ident}, but LNAV didn't engage. Check the heading."),
        Lnav::Unread => format!("Direct {ident}, but I can't read LNAV."),
    }
}

async fn read_line(xp: &dyn XPlane, n: usize) -> Option<String> {
    xp.get_bytes(&fms_cdu1_line(n))
        .await
        .ok()
        .map(|b| cdu_text(&b))
}

async fn press(xp: &dyn XPlane, cmd: &str, ms: u64) {
    let _ = xp.run_command(cmd).await;
    tokio::time::sleep(Duration::from_millis(ms)).await;
}

/// CLR until the scratchpad reads blank, at most [`MAX_CLEARS`] presses.
/// `None` unread.
async fn clear_scratchpad(xp: &dyn XPlane) -> Option<bool> {
    for _ in 0..MAX_CLEARS {
        if scratchpad_blank(&read_line(xp, SCRATCHPAD).await?) {
            return Some(true);
        }
        press(xp, KEY_CLEAR, KEY_MS).await;
    }
    Some(scratchpad_blank(&read_line(xp, SCRATCHPAD).await?))
}

/// LNAV and gpss read together.
async fn read_lnav(xp: &dyn XPlane) -> Lnav {
    let (on, gpss) = tokio::join!(
        crate::autoflight::read_lnav(xp),
        xp.get_float(names::GPSS_STATUS)
    );
    lnav_state(on, gpss.ok())
}

/// The row's LSK for `fix`: as heard, the row it most plausibly names
/// (`plan::best_ident`); `exact`, only the row spelt as the FMS spells it
/// (the follower's own leg, never a near namesake). Pure.
pub fn pick_row(fix: &str, rows: &[(usize, String)], exact: bool) -> Option<usize> {
    let rows = rows.iter().map(|(l, id)| (*l, id.as_str()));
    if exact {
        let want = crate::simbrief::fms_ident(fix);
        rows.into_iter()
            .find(|(_, id)| id.eq_ignore_ascii_case(&want))
            .map(|(l, _)| l)
    } else {
        crate::plan::best_ident(fix, rows)
    }
}

/// The listed fix [`pick_row`] names, page by page: its LSK and ident.
/// The rows seen go on the tape either way.
async fn find_row(
    xp: &dyn XPlane,
    fix: &str,
    exact: bool,
    seen: &mut Vec<String>,
) -> Option<(usize, String)> {
    for page in 0..MAX_PAGES {
        if page > 0 {
            press(xp, NEXT_PAGE, PAGE_MS).await;
        }
        let mut rows = Vec::new();
        for lsk in ROW_LSKS {
            if let Some(id) = read_line(xp, 2 * lsk).await.as_deref().and_then(row_ident) {
                rows.push((lsk, id.to_string()));
            }
        }
        seen.extend(rows.iter().map(|(_, id)| id.clone()));
        if let Some(lsk) = pick_row(fix, &rows, exact) {
            return rows.into_iter().find(|(l, _)| *l == lsk);
        }
    }
    None
}

/// The FMS half of a DIR TO, no gates and no LNAV: clear the scratchpad,
/// DIR INTC, the row [`pick_row`] names, EXEC only on a MOD page. `Ok` the
/// ident executed, `Err` the spoken reason nothing was. Blackbox
/// `direct_to` with `by` (the pilot's tool, or the follower's leg).
pub async fn execute(xp: &dyn XPlane, fix: &str, exact: bool, by: &str) -> Result<String, String> {
    // A leftover in the scratchpad would be entered by the row's LSK.
    match clear_scratchpad(xp).await {
        Some(true) => {}
        Some(false) => return Err("The FMS scratchpad won't clear.".to_string()),
        None => return Err("I can't read the FMS.".to_string()),
    }
    press(xp, DIR_INTC, PAGE_MS).await;
    let mut seen = Vec::new();
    let Some((lsk, ident)) = find_row(xp, fix, exact, &mut seen).await else {
        let _ = clear_scratchpad(xp).await;
        crate::blackbox::record(
            "direct_to",
            json!({"by": by, "heard": fix, "listed": seen, "executed": false}),
        );
        return Err(format!(
            "{} isn't in the FMS route. Nothing executed.",
            fix.to_ascii_uppercase()
        ));
    };
    press(xp, &format!("sim/FMS/ls_{lsk}l"), PAGE_MS).await;
    let line0 = read_line(xp, 0).await;
    if !line0.as_deref().is_some_and(is_mod) {
        let _ = clear_scratchpad(xp).await;
        crate::blackbox::record(
            "direct_to",
            json!({"by": by, "heard": fix, "ident": ident, "line0": line0, "executed": false}),
        );
        return Err(format!(
            "The FMS didn't take direct {ident}. Nothing executed."
        ));
    }
    press(xp, EXEC, PAGE_MS).await;
    crate::blackbox::record(
        "direct_to",
        json!({"by": by, "heard": fix, "ident": ident, "line0": line0, "executed": true}),
    );
    Ok(ident)
}

/// The whole DIR TO, the gates first. `fix` is as heard; it must be listed
/// on the FMS's DIRECT-TO page.
pub async fn run(xp: &dyn XPlane, fix: &str) -> String {
    let heard = fix.trim();
    if heard.is_empty() {
        return "Direct to which fix?".to_string();
    }
    if !crate::autoflight::panel().lnav_declared() {
        return "Direct to a fix isn't supported on this aircraft yet.".to_string();
    }
    // The follower sequences the FMS itself (`xcopilot/fms/goto`): two
    // writers of one FMS fight.
    if let Some(owner) = crate::flight_state::crew_flying() {
        return format!(
            "The {} is flying the route. Say my controls first.",
            owner.name()
        );
    }
    match xp.get_float(crate::datarefs::ON_GROUND_ANY).await {
        Ok(g) if !sim::ground::on_ground(g) => {}
        Ok(_) => return "We're on the ground. Direct to is for the air.".to_string(),
        Err(_) => return "I can't tell if we're airborne.".to_string(),
    }
    match crate::autoflight::read_status(xp, names::SERVOS_ON).await {
        Ok(s) if s >= 0.5 => {}
        Ok(_) => return "The autopilot isn't engaged, so no LNAV. Engage CMD first.".to_string(),
        Err(_) => return "I can't read the autopilot.".to_string(),
    }
    let ident = match execute(xp, heard, false, "pilot").await {
        Ok(ident) => ident,
        Err(line) => return line,
    };
    // Pressed once, never again: a second press disarms an armed LNAV.
    let before = read_lnav(xp).await;
    let pressed = matches!(before, Lnav::Out);
    if pressed {
        crate::autoflight::press_lnav(xp, "direct_to").await;
        tokio::time::sleep(Duration::from_secs_f64(crate::autoflight::LNAV_SETTLE_S)).await;
    }
    let (lnav, nav_id) = tokio::join!(read_lnav(xp), xp.get_bytes(crate::datarefs::GPS_NAV_ID));
    let nav_id = nav_id.ok().map(|b| cdu_text(&b));
    let on_fix = nav_id
        .as_deref()
        .is_some_and(|n| crate::lnav::same_ident(n, &ident));
    crate::blackbox::record(
        "direct_to",
        json!({"by": "pilot", "ident": ident, "readback": true,
               "gps_nav_id": nav_id, "lnav_before": before, "pressed": pressed, "lnav": lnav}),
    );
    readback(&ident, on_fix, lnav)
}

pub struct DirectTo;

#[async_trait]
impl Tool for DirectTo {
    fn name(&self) -> &str {
        "direct_to"
    }

    fn description(&self) -> &str {
        "'Direct LESTA', 'proceed direct LESTA', 'go direct to LESTA': DIR TO a \
         fix of the FMS route, executed, and LNAV engaged. Read back on the \
         active waypoint and the LNAV lamp."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": { "fix": {"type": "string"} },
            "required": ["fix"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let fix = args.get("fix").and_then(Value::as_str).unwrap_or_default();
        Ok(run(ctx.xplane, fix).await)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xplane::FakeXPlane;

    /// The scratchpad as the 737's CDU sends it blank, and with leftovers.
    #[test]
    fn the_scratchpad_is_blank_only_when_it_shows_nothing() {
        assert!(scratchpad_blank("[                      ]"));
        assert!(scratchpad_blank(&cdu_text(b"[                      ]\0\0")));
        assert!(scratchpad_blank(""));
        assert!(!scratchpad_blank("[VNAV UNAVAILABLE      ]"));
    }

    /// EXEC only on a MOD page: the DIR INTC page itself is ACT.
    #[test]
    fn exec_waits_for_a_mod_page() {
        assert!(is_mod(" MOD RTE LEGS      1/3"));
        assert!(!is_mod(" ACT DIRECT-TO     1/1"));
        assert!(!is_mod(""));
    }

    /// The DIRECT-TO rows as seen live, and the heard word matched to one.
    #[test]
    fn the_page_rows_name_their_fixes() {
        assert_eq!(
            row_ident(&cdu_text(b"<LESTA            FL374>\0")),
            Some("LESTA")
        );
        assert_eq!(row_ident("<EMBOR            FL380>"), Some("EMBOR"));
        assert_eq!(row_ident("<-----     NEAREST APTS>"), None);
        assert_eq!(row_ident(""), None);
        let rows = [(2, "LESTA"), (3, "EMBOR")];
        assert_eq!(crate::plan::best_ident("lesta", rows), Some(2));
        assert_eq!(crate::plan::best_ident("embor", rows), Some(3));
        assert_eq!(crate::plan::best_ident("pimos", rows), None);
    }

    /// As heard, the nearest spelling; the follower's own leg, only its
    /// exact ident (never DIR TO a near namesake of the leg).
    #[test]
    fn the_follower_picks_only_its_exact_row() {
        let rows = vec![(2, "INPIP".to_string()), (3, "INREV".to_string())];
        assert_eq!(pick_row("inpip", &rows, false), Some(2));
        assert_eq!(pick_row("INPIP", &rows, true), Some(2));
        assert_eq!(pick_row("INREV", &rows, true), Some(3));
        assert_eq!(pick_row("inpit", &rows, false), Some(2));
        assert_eq!(pick_row("INPIT", &rows, true), None);
        assert_eq!(pick_row("EGPH arrival", &rows, true), None);
    }

    /// Armed and engaged as the 737 reads them, and what is said for each.
    /// Never "Roger" on a failed readback: what did not take is named.
    #[test]
    fn lnav_armed_is_told_from_engaged_and_said() {
        assert_eq!(lnav_state(Some(false), Some(1.0)), Lnav::Armed);
        assert_eq!(lnav_state(Some(true), Some(2.0)), Lnav::Engaged);
        assert_eq!(lnav_state(Some(false), Some(0.0)), Lnav::Out);
        assert_eq!(lnav_state(None, None), Lnav::Unread);
        assert_eq!(
            readback("LESTA", true, Lnav::Engaged),
            "Direct LESTA, LNAV."
        );
        assert_eq!(
            readback("LESTA", true, Lnav::Armed),
            "Direct LESTA, LNAV armed."
        );
        assert!(readback("LESTA", true, Lnav::Out).contains("LNAV didn't engage"));
        assert!(readback("LESTA", false, Lnav::Engaged).contains("isn't showing it"));
    }

    /// An airframe without a declared LNAV is refused before anything is
    /// pressed (no live profile in a test: the stock panel declares none).
    #[tokio::test]
    async fn an_airframe_without_lnav_is_refused_and_nothing_pressed() {
        let fake = FakeXPlane::new();
        assert_eq!(
            run(&fake, "lesta").await,
            "Direct to a fix isn't supported on this aircraft yet."
        );
        assert_eq!(run(&fake, "").await, "Direct to which fix?");
        assert!(fake.commands().is_empty());
    }
}
