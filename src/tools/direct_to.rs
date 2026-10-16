//! "Direct LESTA": the pilot monitoring does the DIR TO on the FMS and
//! presses LNAV, the way it was done by hand on the stock 737 at FL380
//! (2026-10-16): clear the scratchpad, DIR INTC, type the ident, LSK 1L,
//! EXEC only on a MOD page, LNAV at once (the leg starts at present
//! position; on HDG the aircraft drifts off it within a minute and LNAV
//! only arms). Read back on the GPS's active ident and the LNAV lamp,
//! never assumed.
//!
//! Only a fix in the LOADED plan (`plan::match_ident`), only airborne,
//! only with CMD in, only where the airframe declares its LNAV
//! (`lnav_command`, the 737 today). LNAV through the panel owner
//! (`autoflight::LnavEngage`, `press_lnav`). Blackbox `direct_to`.

use std::time::{Duration, Instant};

use anyhow::Result;
use async_trait::async_trait;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::autoflight::{names, LnavEngage, LnavStep};
use crate::xplane::XPlane;

const KEY_CLEAR: &str = "sim/FMS/key_clear";
const DIR_INTC: &str = "sim/FMS/dir_intc";
const LSK_1L: &str = "sim/FMS/ls_1l";
const EXEC: &str = "sim/FMS/exec";
/// A key's command is this plus the letter or digit.
const KEY_PREFIX: &str = "sim/FMS/key_";
/// CLR presses before a scratchpad that will not blank is refused.
const MAX_CLEARS: usize = 30;
/// Between key presses: the FMS takes them in its flight loop.
const KEY_MS: u64 = 150;
/// After a page change (DIR INTC, LSK, EXEC) before it is read.
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

/// The key commands that type `ident`, `None` if it holds anything but
/// letters and digits. Pure.
pub fn key_commands(ident: &str) -> Option<Vec<String>> {
    if ident.is_empty() {
        return None;
    }
    ident
        .chars()
        .map(|c| {
            c.is_ascii_alphanumeric()
                .then(|| format!("{KEY_PREFIX}{}", c.to_ascii_uppercase()))
        })
        .collect()
}

/// The spoken readback. Pure.
pub fn readback(ident: &str, on_fix: bool, lnav: Option<bool>, armed: bool) -> String {
    if !on_fix {
        return format!(
            "Executed direct {ident}, but the FMS isn't showing it as the active waypoint."
        );
    }
    match (lnav, armed) {
        (Some(true), false) => format!("Direct {ident}, LNAV."),
        (_, true) => format!("Direct {ident}, LNAV armed."),
        (Some(false), _) => format!("Direct {ident}, but LNAV didn't engage. Check the heading."),
        (None, _) => format!("Direct {ident}, but I can't read LNAV."),
    }
}

async fn read_line(xp: &dyn XPlane, name: &str) -> Option<String> {
    xp.get_bytes(name).await.ok().map(|b| cdu_text(&b))
}

async fn press(xp: &dyn XPlane, cmd: &str, ms: u64) -> bool {
    let ok = xp.run_command(cmd).await.is_ok();
    tokio::time::sleep(Duration::from_millis(ms)).await;
    ok
}

/// CLR until the scratchpad reads blank, at most [`MAX_CLEARS`] presses.
/// `None` unread.
async fn clear_scratchpad(xp: &dyn XPlane) -> Option<bool> {
    for _ in 0..MAX_CLEARS {
        if scratchpad_blank(&read_line(xp, crate::datarefs::FMS_CDU1_SCRATCHPAD).await?) {
            return Some(true);
        }
        press(xp, KEY_CLEAR, KEY_MS).await;
    }
    Some(scratchpad_blank(
        &read_line(xp, crate::datarefs::FMS_CDU1_SCRATCHPAD).await?,
    ))
}

/// LNAV in through the panel owner: read before every press, a press only
/// on a settled read that says out. `None` unread.
async fn engage_lnav(xp: &dyn XPlane) -> Option<bool> {
    let mut eng = LnavEngage::default();
    let t0 = Instant::now();
    loop {
        match eng.step(
            t0.elapsed().as_secs_f64(),
            crate::autoflight::read_lnav(xp).await,
        ) {
            LnavStep::Engaged => return Some(true),
            LnavStep::Failed => return Some(false),
            LnavStep::Unread => return None,
            LnavStep::Press => {
                crate::autoflight::press_lnav(xp, "direct_to").await;
            }
            LnavStep::Wait => {}
        }
        tokio::time::sleep(Duration::from_millis(250)).await;
    }
}

/// The whole DIR TO, the gates first. `fix` is as heard; it must name a
/// fix of the loaded plan.
pub async fn run(xp: &dyn XPlane, fix: &str) -> String {
    if fix.trim().is_empty() {
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
    let Some(plan) = crate::simbrief::loaded_plan() else {
        return "There's no flight plan loaded.".to_string();
    };
    let Some((_, wp)) = crate::plan::match_ident(fix.trim(), &plan.waypoints) else {
        return format!(
            "I can't find {} in the flight plan.",
            fix.trim().to_ascii_uppercase()
        );
    };
    let ident = crate::simbrief::fms_ident(&wp.ident);
    let Some(keys) = key_commands(&ident) else {
        return format!("I can't type {ident} into the FMS.");
    };
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
    match clear_scratchpad(xp).await {
        Some(true) => {}
        Some(false) => return "The FMS scratchpad won't clear.".to_string(),
        None => return "I can't read the FMS.".to_string(),
    }
    press(xp, DIR_INTC, PAGE_MS).await;
    for k in &keys {
        press(xp, k, KEY_MS).await;
    }
    press(xp, LSK_1L, PAGE_MS).await;
    let line0 = read_line(xp, crate::datarefs::FMS_CDU1_LINE0).await;
    let modded = line0.as_deref().is_some_and(is_mod);
    if !modded {
        let _ = clear_scratchpad(xp).await;
        crate::blackbox::record(
            "direct_to",
            json!({"ident": ident, "line0": line0, "executed": false}),
        );
        return format!("The FMS didn't take direct {ident}. Nothing executed.");
    }
    press(xp, EXEC, PAGE_MS).await;
    let lnav = engage_lnav(xp).await;
    let (nav_id, gpss) = tokio::join!(
        xp.get_bytes(crate::datarefs::GPS_NAV_ID),
        xp.get_float(names::GPSS_STATUS)
    );
    let nav_id = nav_id.ok().map(|b| cdu_text(&b));
    let gpss = gpss.ok();
    let on_fix = nav_id
        .as_deref()
        .is_some_and(|n| crate::lnav::same_ident(n, &ident));
    let armed = gpss.is_some_and(|g| g.round() == 1.0);
    crate::blackbox::record(
        "direct_to",
        json!({"ident": ident, "line0": line0, "executed": true, "gps_nav_id": nav_id,
               "lnav": lnav, "gpss_status": gpss}),
    );
    readback(&ident, on_fix, lnav, armed)
}

pub struct DirectTo;

#[async_trait]
impl Tool for DirectTo {
    fn name(&self) -> &str {
        "direct_to"
    }

    fn description(&self) -> &str {
        "'Direct LESTA', 'proceed direct LESTA', 'go direct to LESTA': DIR TO a \
         fix of the loaded plan on the FMS, executed, and LNAV engaged. Read back \
         on the active waypoint and the LNAV lamp."
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
        assert!(!scratchpad_blank("[LES                   ]"));
    }

    /// EXEC only on a MOD page: the DIR INTC page itself is ACT.
    #[test]
    fn exec_waits_for_a_mod_page() {
        assert!(is_mod(" MOD RTE LEGS      1/3"));
        assert!(!is_mod(" ACT DIRECT-TO     1/1"));
        assert!(!is_mod(""));
    }

    /// The ident decodes from the NUL-padded bytes, and types key by key.
    #[test]
    fn the_ident_decodes_and_types() {
        assert_eq!(cdu_text(b"LESTA\0\0\0\0"), "LESTA");
        assert_eq!(
            key_commands("LE5"),
            Some(vec![
                "sim/FMS/key_L".into(),
                "sim/FMS/key_E".into(),
                "sim/FMS/key_5".into()
            ])
        );
        assert_eq!(key_commands("LE-S"), None);
        assert_eq!(key_commands(""), None);
    }

    /// Never "Roger" on a failed readback: what did not take is named.
    #[test]
    fn the_readback_names_what_did_not_take() {
        assert_eq!(
            readback("LESTA", true, Some(true), false),
            "Direct LESTA, LNAV."
        );
        assert_eq!(
            readback("LESTA", true, Some(true), true),
            "Direct LESTA, LNAV armed."
        );
        assert!(readback("LESTA", true, Some(false), false).contains("LNAV didn't engage"));
        assert!(readback("LESTA", false, Some(true), false).contains("isn't showing it"));
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
