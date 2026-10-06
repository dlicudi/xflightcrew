//! The crew's preflight on the airframe's own switches, from a live battery
//! to ready for the start: what full auto does between "Battery's on" and
//! the ramp items, where the profile declares it (`preflight:`,
//! [`crate::aircraft::PreflightProfile`]).
//!
//! The Zibo 737 is the airframe that asked for it. Full auto went from the
//! battery straight to the ramp and the start: its lamps are AC, so on the
//! battery alone "Nav lights did not come on"; the start refused on 0 psi
//! of duct pressure; and nothing it needed to fly (hydraulics, the IRS, the
//! yaw damper, the fuel pumps) was ever switched (EGHL, 2026-10-06). The
//! sequence is the aircraft's own checklist's (`clist.txt` v9.0, PRELIMINARY
//! FLIGHTDECK and PREFLIGHT PREPARATION), every switch read before it is
//! fired and read back after, at a crew's pace (`jet_start::run_steps`).
//!
//! The IRS alignment it begins is waited on before the taxi ([`wait_aligned`]):
//! the platform is the attitude the autopilot flies on.

use std::time::Duration;

use serde_json::json;

use crate::aircraft::{AlignWait, PositionEntry, PreflightProfile};
use crate::xplane::XPlane;

/// How the preflight ended: the switches that did not take, by short name,
/// or the hand-back line when the APU it needs never came up.
pub type Outcome = Result<Vec<String>, String>;

/// Run the preflight: the APU and its generators first where declared
/// (through the start's own APU steps, `apu`), then the steps, then the
/// present position for the alignment they began. `say` is the voice
/// (milestones only), `panel` the transcript (every switch).
pub async fn run(
    xp: &dyn XPlane,
    p: &PreflightProfile,
    apu: Option<&crate::tools::jet_start::BoeingStart>,
    say: &mut (dyn FnMut(&str) + Send),
    panel: &mut (dyn FnMut(&str) + Send),
) -> Outcome {
    let mut not_taken = Vec::new();
    if p.apu_first {
        if let Some(start) = apu {
            start.apu_up(xp, say, panel, &mut not_taken).await?;
        }
    }
    not_taken.extend(crate::tools::jet_start::run_steps(xp, &p.steps, say, panel).await);
    if let Some(entry) = p.align.as_ref().and_then(|a| a.position_entry.as_ref()) {
        let entered = enter_position(xp, entry).await;
        if entered == Some(false) {
            not_taken.push("I.R.S. position".into());
        }
    }
    crate::blackbox::record(
        "preflight",
        json!({"event": "done", "not_taken": not_taken}),
    );
    Ok(not_taken)
}

/// A position as the 737's POS INIT takes it: hemisphere, whole degrees,
/// minutes to a tenth (`N5111.1W00102.5`, the shape `B738.LATLON.lua`'s
/// `dist_to_latlon` parses: latitude in characters 1 to 7, longitude in 8
/// to 15). Pure.
pub fn pos_init_string(lat: f64, lon: f64) -> String {
    fn part(v: f64, pos: char, neg: char, deg_width: usize) -> String {
        let h = if v >= 0.0 { pos } else { neg };
        let a = v.abs();
        let mut deg = a.trunc() as u32;
        let mut tenths = ((a - a.trunc()) * 600.0).round() as u32;
        if tenths >= 600 {
            deg += 1;
            tenths = 0;
        }
        format!("{h}{deg:0deg_width$}{:02}.{}", tenths / 10, tenths % 10)
    }
    format!("{}{}", part(lat, 'N', 'S', 2), part(lon, 'E', 'W', 3))
}

/// Type the present position in and hand it to the IRS. `Some(true)` when
/// the entry field read back what was written, `None` when the position
/// itself did not read (nothing was written: no measurement is no entry).
pub async fn enter_position(xp: &dyn XPlane, entry: &PositionEntry) -> Option<bool> {
    let Ok((lat, lon, _)) = sim::position::read_lat_lon(xp).await else {
        crate::blackbox::record("preflight", json!({"event": "irs_position", "read": null}));
        return None;
    };
    let text = pos_init_string(lat, lon);
    let wrote = xp.set_bytes(&entry.dataref, text.as_bytes()).await.is_ok();
    let back = xp.get_bytes(&entry.dataref).await.ok().map(|b| {
        String::from_utf8_lossy(&b)
            .trim_end_matches('\0')
            .trim()
            .to_string()
    });
    let took = wrote && back.as_deref() == Some(text.as_str());
    if took {
        let _ = xp.run_command(&entry.command).await;
    }
    crate::blackbox::record(
        "preflight",
        json!({"event": "irs_position", "entered": text, "read_back": back, "ok": took}),
    );
    Some(took)
}

/// Whether every system reads its aligned mode: `None` when any did not
/// read (no measurement), never a guess. Pure.
pub fn aligned(modes: &[Option<f64>], want: f64) -> Option<bool> {
    let mut all = true;
    for m in modes {
        let v = (*m)?;
        all &= (v - want).abs() < 0.5;
    }
    Some(all && !modes.is_empty())
}

/// The spoken wait: minutes left when the airframe says how long. Pure.
pub fn wait_line(remaining_s: Option<f64>) -> String {
    match remaining_s.filter(|s| *s > 0.0) {
        Some(s) => {
            let min = (s / 60.0).ceil().max(1.0) as u32;
            if min == 1 {
                "Waiting for the I.R.S. to align, about a minute.".to_string()
            } else {
                format!("Waiting for the I.R.S. to align, about {min} minutes.")
            }
        }
        None => "Waiting for the I.R.S. to align.".to_string(),
    }
}

/// How a wait on the alignment ended.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AlignVerdict {
    /// Every system in its aligned mode (at once, or after a wait).
    Aligned { waited: bool },
    /// Not aligned at the deadline, with the seconds the airframe said
    /// were left, when it said.
    TimedOut { remaining_s: Option<f64> },
    /// The caller's stop came first.
    Stopped,
    /// Every system reads OFF: nothing to wait for.
    Off,
}

async fn read_modes(xp: &dyn XPlane, a: &AlignWait) -> Vec<Option<f64>> {
    let mut out = Vec::with_capacity(a.modes.len());
    for m in &a.modes {
        out.push(xp.get_float(m).await.ok().filter(|v| v.is_finite()));
    }
    out
}

async fn read_remaining(xp: &dyn XPlane, a: &AlignWait) -> Option<f64> {
    match &a.remaining {
        Some(r) => xp.get_float(r).await.ok().filter(|v| v.is_finite()),
        None => None,
    }
}

/// Wait for the alignment, bounded by the airframe's `timeout_secs`; one
/// line said when there is a wait at all. `stop` is polled every second.
pub async fn wait_aligned(
    xp: &dyn XPlane,
    a: &AlignWait,
    stop: &(dyn Fn() -> bool + Send + Sync),
    say: &mut (dyn FnMut(&str) + Send),
) -> AlignVerdict {
    let first = read_modes(xp, a).await;
    if aligned(&first, a.want) == Some(true) {
        return AlignVerdict::Aligned { waited: false };
    }
    // Every system OFF: nothing is aligning, and no wait will change that
    // (an engine started by hand on a dark overhead, then full auto).
    if !first.is_empty() && first.iter().all(|m| m.is_some_and(|v| v.abs() < 0.5)) {
        crate::blackbox::record("preflight", json!({"event": "align_off"}));
        return AlignVerdict::Off;
    }
    let remaining = read_remaining(xp, a).await;
    let line = wait_line(remaining);
    say(&line);
    crate::blackbox::record(
        "preflight",
        json!({"event": "align_wait", "remaining_s": remaining, "timeout_s": a.timeout_secs}),
    );
    let deadline = tokio::time::Instant::now() + Duration::from_secs_f64(a.timeout_secs);
    loop {
        tokio::time::sleep(Duration::from_secs(1)).await;
        if stop() {
            return AlignVerdict::Stopped;
        }
        if aligned(&read_modes(xp, a).await, a.want) == Some(true) {
            crate::blackbox::record("preflight", json!({"event": "aligned"}));
            return AlignVerdict::Aligned { waited: true };
        }
        if tokio::time::Instant::now() >= deadline {
            let remaining_s = read_remaining(xp, a).await;
            crate::blackbox::record(
                "preflight",
                json!({"event": "align_timeout", "remaining_s": remaining_s}),
            );
            return AlignVerdict::TimedOut { remaining_s };
        }
    }
}

/// The hand-back line for an alignment that did not finish. Pure.
pub fn align_timeout_line(remaining_s: Option<f64>) -> String {
    match remaining_s.filter(|s| *s > 0.0) {
        Some(s) => format!(
            "The I.R.S. still isn't aligned, {:.0} seconds to go. Your aircraft.",
            s
        ),
        None => "The I.R.S. hasn't aligned. Check the position entry. Your aircraft.".to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aircraft::StartStep;
    use crate::xplane::FakeXPlane;

    #[test]
    fn the_position_is_typed_as_pos_init_takes_it() {
        // EGHL, probed 2026-10-07 and accepted by the Zibo's IRS.
        assert_eq!(
            pos_init_string(51.18508594769988, -1.0408489190568715),
            "N5111.1W00102.5"
        );
        assert_eq!(pos_init_string(-33.9461, 151.1772), "S3356.8E15110.6");
        // A tenth that rounds up to sixty minutes carries into the degree.
        assert_eq!(pos_init_string(45.99999, 7.0), "N4600.0E00700.0");
        assert_eq!(pos_init_string(0.0, 0.0).len(), 15);
    }

    #[test]
    fn aligned_needs_every_system_read_and_in_nav() {
        assert_eq!(aligned(&[Some(2.0), Some(2.0)], 2.0), Some(true));
        assert_eq!(aligned(&[Some(2.0), Some(1.0)], 2.0), Some(false));
        assert_eq!(aligned(&[Some(2.0), None], 2.0), None, "no measurement");
        assert_eq!(aligned(&[], 2.0), Some(false));
    }

    #[test]
    fn the_wait_is_said_in_minutes_when_the_airframe_says_how_long() {
        assert_eq!(
            wait_line(Some(433.0)),
            "Waiting for the I.R.S. to align, about 8 minutes."
        );
        assert_eq!(
            wait_line(Some(20.0)),
            "Waiting for the I.R.S. to align, about a minute."
        );
        assert_eq!(wait_line(None), "Waiting for the I.R.S. to align.");
        assert_eq!(wait_line(Some(0.0)), "Waiting for the I.R.S. to align.");
    }

    fn align(entry: bool) -> AlignWait {
        AlignWait {
            modes: vec!["irs_mode".into(), "irs2_mode".into()],
            want: 2.0,
            remaining: Some("remain".into()),
            timeout_secs: 60.0,
            position_entry: entry.then(|| PositionEntry {
                dataref: "pos_fmc_set".into(),
                command: "fmc_to_irs".into(),
            }),
        }
    }

    fn knob(cmd: &str, down: &str, pos: &str, want: f64) -> StartStep {
        StartStep {
            command: cmd.into(),
            position: Some(pos.into()),
            want,
            held_secs: None,
            down_command: Some(down.into()),
            say: None,
            max_presses: Some(4),
            when_above: None,
        }
    }

    #[tokio::test(start_paused = true)]
    async fn a_knob_is_stepped_detent_by_detent_to_nav_and_the_position_entered() {
        let xp = FakeXPlane::new();
        xp.set_read("irs_left", 0.0);
        xp.step_on_command("irs_L_right", "irs_left", 1.0);
        xp.step_on_command("irs_L_left", "irs_left", -1.0);
        xp.set_read("sim/flightmodel/position/latitude", 51.18508594769988);
        xp.set_read("sim/flightmodel/position/longitude", -1.0408489190568715);
        let p = PreflightProfile {
            apu_first: false,
            steps: vec![knob("irs_L_right", "irs_L_left", "irs_left", 2.0)],
            align: Some(align(true)),
        };
        let out = run(&xp, &p, None, &mut |_| {}, &mut |_| {}).await;
        assert_eq!(out, Ok(vec![]));
        assert_eq!(xp.get_float("irs_left").await.unwrap(), 2.0);
        assert_eq!(
            xp.commands(),
            vec!["irs_L_right", "irs_L_right", "fmc_to_irs"],
            "two detents from OFF to NAV, then the position handed over"
        );
        assert_eq!(
            xp.get_bytes("pos_fmc_set").await.unwrap(),
            b"N5111.1W00102.5".to_vec()
        );
    }

    #[tokio::test(start_paused = true)]
    async fn a_knob_past_its_detent_steps_back_and_one_that_never_moves_is_named() {
        // ATT (3) back to NAV (2): one press the other way.
        let xp = FakeXPlane::new();
        xp.set_read("irs_left", 3.0);
        xp.step_on_command("irs_L_right", "irs_left", 1.0);
        xp.step_on_command("irs_L_left", "irs_left", -1.0);
        let p = PreflightProfile {
            apu_first: false,
            steps: vec![knob("irs_L_right", "irs_L_left", "irs_left", 2.0)],
            align: None,
        };
        assert_eq!(
            run(&xp, &p, None, &mut |_| {}, &mut |_| {}).await,
            Ok(vec![])
        );
        assert_eq!(xp.commands(), vec!["irs_L_left"]);
        // A knob whose command moves nothing: one press, named, never four.
        let dead = FakeXPlane::new();
        dead.set_read("irs_left", 0.0);
        let out = run(&dead, &p, None, &mut |_| {}, &mut |_| {}).await;
        assert_eq!(out, Ok(vec!["irs left".to_string()]));
        assert_eq!(dead.commands(), vec!["irs_L_right"]);
    }

    #[tokio::test(start_paused = true)]
    async fn a_conditional_step_runs_only_above_its_reading() {
        let step = StartStep {
            command: "fuel_pump_ctr1".into(),
            position: Some("ctr1_pos".into()),
            want: 1.0,
            held_secs: None,
            down_command: None,
            say: None,
            max_presses: None,
            when_above: Some(crate::aircraft::StepCondition {
                dataref: "m_fuel[1]".into(),
                value: 453.0,
            }),
        };
        let p = PreflightProfile {
            apu_first: false,
            steps: vec![step],
            align: None,
        };
        for (centre, fired) in [(0.0, false), (2000.0, true)] {
            let xp = FakeXPlane::new();
            xp.set_read_array("m_fuel", &[3700.0, centre, 3700.0]);
            xp.set_read("ctr1_pos", 0.0);
            xp.set_on_command("fuel_pump_ctr1", "ctr1_pos", 1.0);
            let out = run(&xp, &p, None, &mut |_| {}, &mut |_| {}).await;
            assert_eq!(out, Ok(vec![]));
            assert_eq!(!xp.commands().is_empty(), fired, "centre {centre} kg");
        }
        // An unread tank is no reason to switch.
        let xp = FakeXPlane::new();
        xp.set_read("ctr1_pos", 0.0);
        run(&xp, &p, None, &mut |_| {}, &mut |_| {}).await.unwrap();
        assert!(xp.commands().is_empty());
    }

    /// The Zibo's declared preflight flown against a cockpit cold as probed
    /// at EGHL (2026-10-07): every switch read first, the ones already set
    /// passed over, the knobs stepped detent by detent, nothing left
    /// untaken, the centre pumps left off with an empty centre tank.
    #[tokio::test(start_paused = true)]
    async fn the_zibo_preflight_flies_its_overhead_from_cold() {
        let p = crate::aircraft::load("zibo_738").expect("zibo profile");
        let pf = p.preflight.clone().expect("preflight");
        let xp = FakeXPlane::new();
        // Cold, as read 2026-10-07 (battery on, APU off).
        let cold = [
            ("laminar/B738/toggle_switch/hydro_pumps1_pos", 1.0),
            ("laminar/B738/toggle_switch/hydro_pumps2_pos", 1.0),
            ("laminar/B738/toggle_switch/cab_util_pos", 1.0),
            ("laminar/B738/toggle_switch/ife_pass_seat_pos", 1.0),
            ("laminar/B738/air/l_recirc_fan_pos", 1.0),
            ("laminar/B738/air/r_recirc_fan_pos", 1.0),
            ("laminar/B738/air/isolation_valve_pos", 2.0),
            ("laminar/B738/toggle_switch/bleed_air_1_pos", 1.0),
            ("laminar/B738/toggle_switch/bleed_air_2_pos", 1.0),
            ("laminar/B738/autobrake/autobrake_pos", 1.0),
        ];
        for s in &pf.steps {
            let pos = s
                .position
                .as_deref()
                .expect("every preflight step reads a position");
            let v = cold
                .iter()
                .find(|(n, _)| *n == pos)
                .map_or(0.0, |(_, v)| *v);
            xp.set_read(pos, v);
            match (&s.max_presses, &s.down_command) {
                (Some(_), Some(down)) => {
                    // The knobs as probed: `command` steps up, `down_command`
                    // down.
                    xp.step_on_command(&s.command, pos, 1.0);
                    xp.step_on_command(down, pos, -1.0);
                }
                _ => xp.set_on_command(&s.command, pos, s.want),
            }
        }
        xp.set_read_array("sim/flightmodel/weight/m_fuel", &[3697.9, 0.0, 3697.9]);
        xp.set_read("sim/flightmodel/position/latitude", 51.18508594769988);
        xp.set_read("sim/flightmodel/position/longitude", -1.0408489190568715);
        let mut said = Vec::new();
        let out = run(
            &xp,
            &pf,
            None,
            &mut |s| said.push(s.to_string()),
            &mut |_| {},
        )
        .await;
        assert_eq!(out, Ok(vec![]));
        for s in &pf.steps {
            let pos = s.position.as_deref().unwrap();
            let centre = s.when_above.is_some();
            let v = xp.get_float(pos).await.unwrap();
            if centre {
                assert_eq!(v, 0.0, "{pos}: the centre pumps stay off when it is empty");
            } else {
                assert_eq!(v, s.want, "{pos}");
            }
        }
        let ran = xp.commands();
        for passed_over in [
            "laminar/B738/toggle_switch/hydro_pumps1",
            "laminar/B738/autopilot/cab_util_toggle",
            "laminar/B738/toggle_switch/l_recirc_fan",
            "laminar/B738/toggle_switch/bleed_air_1",
            "laminar/B738/toggle_switch/iso_valve_dn",
            "laminar/B738/toggle_switch/fuel_pump_ctr1",
        ] {
            assert!(!ran.iter().any(|c| c == passed_over), "{passed_over} fired");
        }
        assert_eq!(
            xp.ran_count("laminar/B738/toggle_switch/irs_L_right"),
            2,
            "OFF to NAV"
        );
        assert_eq!(
            xp.ran_count("laminar/B738/knob/ac_power_up"),
            3,
            "to APU GEN"
        );
        assert_eq!(
            xp.ran_count("laminar/B738/knob/autobrake_dn"),
            1,
            "OFF to RTO"
        );
        assert_eq!(
            ran.first().map(String::as_str),
            Some("laminar/B738/toggle_switch/irs_L_right")
        );
        assert_eq!(
            ran.last().map(String::as_str),
            Some("laminar/B738/push_button/fmc_to_irs_pos")
        );
        assert!(said.contains(&"Hydraulics on.".to_string()), "{said:?}");
        assert!(!said.contains(&"Centre pumps on.".to_string()), "{said:?}");
    }

    /// The APU comes first, through the start's own APU steps; one that
    /// never comes up is the preflight's hand-back, before a switch of the
    /// overhead is touched.
    #[tokio::test(start_paused = true)]
    async fn an_apu_that_never_comes_up_ends_the_preflight_before_the_overhead() {
        let p = crate::aircraft::load("zibo_738").expect("zibo profile");
        let pf = p.preflight.clone().expect("preflight");
        let start = crate::tools::jet_start::BoeingStart::new(p.boeing_start.clone().unwrap());
        let xp = FakeXPlane::new();
        xp.set_read("sim/cockpit2/electrical/APU_N1_percent", 0.0);
        let out = run(&xp, &pf, Some(&start), &mut |_| {}, &mut |_| {}).await;
        assert_eq!(
            out,
            Err("The A.P.U. hasn't come up, so there's no start air. Your aircraft.".to_string())
        );
        assert!(!xp.ran("laminar/B738/toggle_switch/irs_L_right"));
        assert!(
            xp.ran("laminar/B738/switch/battery_dn"),
            "the APU steps ran"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn the_alignment_is_waited_on_said_once_and_bounded() {
        let a = align(false);
        // Aligned already: no wait, nothing said.
        let xp = FakeXPlane::new();
        xp.set_read("irs_mode", 2.0);
        xp.set_read("irs2_mode", 2.0);
        let mut said = Vec::new();
        let v = wait_aligned(&xp, &a, &|| false, &mut |s| said.push(s.to_string())).await;
        assert_eq!(v, AlignVerdict::Aligned { waited: false });
        assert!(said.is_empty());
        // Still aligning at the deadline: timed out, with the seconds left.
        let xp = FakeXPlane::new();
        xp.set_read("irs_mode", 1.0);
        xp.set_read("irs2_mode", 2.0);
        xp.set_read("remain", 300.0);
        let mut said = Vec::new();
        let v = wait_aligned(&xp, &a, &|| false, &mut |s| said.push(s.to_string())).await;
        assert_eq!(
            v,
            AlignVerdict::TimedOut {
                remaining_s: Some(300.0)
            }
        );
        assert_eq!(
            said,
            vec!["Waiting for the I.R.S. to align, about 5 minutes."]
        );
        assert_eq!(
            align_timeout_line(Some(300.0)),
            "The I.R.S. still isn't aligned, 300 seconds to go. Your aircraft."
        );
        // A stop ends the wait.
        let v = wait_aligned(&xp, &a, &|| true, &mut |_| {}).await;
        assert_eq!(v, AlignVerdict::Stopped);
        // Both OFF: no wait at all.
        let xp = FakeXPlane::new();
        xp.set_read("irs_mode", 0.0);
        xp.set_read("irs2_mode", 0.0);
        let mut said = Vec::new();
        let v = wait_aligned(&xp, &a, &|| false, &mut |s| said.push(s.to_string())).await;
        assert_eq!(v, AlignVerdict::Off);
        assert!(said.is_empty());
    }
}
