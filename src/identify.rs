//! `xvl identify`: the measurement flight that teaches the pilot controller
//! an airframe (2026-10-13; the Rust of the 737's ident.py of 2026-10-12).
//!
//! The aircraft is started in the air at a safe height on its approach
//! speed and flap, settled on its own autopilot, then for each axis the
//! autopilot comes off and the control is moved by a known input through the
//! plugin's channels (`xcopilot/pitch`, `roll`, `taxi_steer`: the elevator a
//! 3-2-1-1, the ailerons and the rudder a doublet, each sign twice), the
//! answer read at the sim's pace by one parallel round a tick, then the
//! autopilot back on to settle. A power step in altitude hold measures what
//! the lever buys. The fits (`pilot::ident`) are stored per `.acf`
//! (`aircraft::learned`), where the detector loads them on the next airframe
//! swap or start of xvl.
//!
//! **Safety.** xvl must be stopped (one writer per channel). A paused or
//! replaying sim is refused. Every input aborts on a bank past
//! [`BANK_ABORT_DEG`], a pitch past [`PITCH_ABORT_DEG`] or a height lost past
//! [`HEIGHT_LOSS_ABORT_FT`]; an abort hands the aircraft back to its own
//! autopilot at once and ends the flight there. The sim is paused at the end.

use crate::aircraft::learned::{fingerprint, Conditions, Fit, Learned, VERSION};
use crate::pilot::ident::{self, Axis, EnergySample, Sample};
use crate::xplane::XPlaneClient;

const PITCH_CH: &str = "xcopilot/pitch";
const ROLL_CH: &str = "xcopilot/roll";
const YAW_CH: &str = "xcopilot/taxi_steer";
const INIT_JSON: &str = "xcopilot/sim/init_flight";
const ROLL_RATE: &str = "sim/flightmodel/position/P";

/// Abort an input on a bank past this, degrees.
pub const BANK_ABORT_DEG: f64 = 20.0;
/// ...a pitch this far from where the input began, degrees.
pub const PITCH_ABORT_DEG: f64 = 10.0;
/// ...or this much height lost from where it began, ft.
pub const HEIGHT_LOSS_ABORT_FT: f64 = 800.0;
/// Each input's amplitude, unit control: small enough to stay linear, big
/// enough to read over the air (the 737's 0.08 elevator, 0.15 aileron and
/// 0.2 rudder rolled it to 15 degrees, 2026-10-12).
fn amplitude(axis: Axis) -> f64 {
    match axis {
        Axis::Pitch => 0.06,
        Axis::Roll => 0.12,
        Axis::Yaw => 0.15,
    }
}

const USAGE: &str = "usage: xvl identify [--alt FT] [--ias KT] [--flap RATIO] [--here]\n\
  Air-starts the loaded aircraft at FT (default 5000) over its present position\n\
  at the profile's approach speed and approach flap, flies the measurement\n\
  inputs and stores the fits in profiles/learned/. --here skips the air start\n\
  (the aircraft must already be flying, level, on its autopilot). xvl must be stopped.";

/// One tick's reads, in [`NAMES`] order.
const NAMES: [&str; 11] = [
    crate::datarefs::THETA_DEG,
    crate::datarefs::PHI_DEG,
    crate::datarefs::ALPHA_DEG,
    crate::datarefs::SIDESLIP_DEG,
    ROLL_RATE,
    crate::datarefs::PITCH_RATE_DEG_S,
    crate::datarefs::YAW_RATE_DEG_S,
    crate::datarefs::INDICATED_AIRSPEED_KTS,
    crate::datarefs::TRUE_AIRSPEED_MS,
    crate::datarefs::LOCAL_VY_MS,
    crate::datarefs::ELEVATION_M,
];

/// One tick, read in one parallel round.
#[derive(Debug, Clone, Copy)]
struct Tick {
    t: f64,
    theta: f64,
    phi: f64,
    alpha: f64,
    beta: f64,
    p: f64,
    q: f64,
    r: f64,
    ias: f64,
    tas_kts: f64,
    vy_fpm: f64,
    alt_ft: f64,
}

async fn read_tick(xp: &XPlaneClient, t0: std::time::Instant) -> Option<Tick> {
    let reads = futures::future::join_all(NAMES.iter().map(|n| xp.get_float(n))).await;
    let v: Vec<f64> = reads.into_iter().map(|r| r.ok().filter(|v| v.is_finite())).collect::<Option<_>>()?;
    Some(Tick {
        t: t0.elapsed().as_secs_f64(),
        theta: v[0],
        phi: v[1],
        alpha: v[2],
        beta: v[3],
        p: v[4],
        q: v[5],
        r: v[6],
        ias: v[7],
        tas_kts: v[8] * 1.943_844,
        vy_fpm: v[9] * 196.850_394,
        alt_ft: v[10] / 0.3048,
    })
}

/// The sample an axis is fitted on, from a tick and the command held.
/// Pure.
fn sample(axis: Axis, k: &Tick, u: f64) -> Sample {
    let (rate, angle, push) = match axis {
        Axis::Pitch => (k.q, k.alpha, 0.0),
        Axis::Roll => (k.p, 0.0, 0.0),
        Axis::Yaw => {
            let push = -(9.81 / (k.tas_kts * 0.514_444).max(15.0)
                * k.phi.to_radians().sin()
                * k.theta.to_radians().cos())
            .to_degrees();
            (k.r, k.beta, push)
        }
    };
    Sample { t: k.t, u, rate, angle, angle_push: push, ias_kts: k.ias, tas_kts: k.tas_kts }
}

/// The roll on the rudder's slip (the dihedral), sampled beside the yaw.
fn roll_on_slip(k: &Tick) -> Sample {
    Sample { t: k.t, u: 0.0, rate: k.p, angle: k.beta, angle_push: 0.0, ias_kts: k.ias, tas_kts: k.tas_kts }
}

/// Why an input stopped early. Pure.
fn abort_reason(k: &Tick, start: &Tick) -> Option<&'static str> {
    if k.phi.abs() > BANK_ABORT_DEG {
        Some("bank")
    } else if (k.theta - start.theta).abs() > PITCH_ABORT_DEG {
        Some("pitch")
    } else if start.alt_ft - k.alt_ft > HEIGHT_LOSS_ABORT_FT {
        Some("height")
    } else {
        None
    }
}

/// The input at `t` seconds from its start, signed: 0 outside it. Pure.
pub fn input_at(axis: Axis, t: f64) -> f64 {
    let mut at = 0.0;
    for &(len, sign) in ident::input_for(axis) {
        if (at..at + len).contains(&t) {
            return sign;
        }
        at += len;
    }
    0.0
}

async fn ap_on(xp: &XPlaneClient) {
    use crate::autoflight::Button;
    for b in [Button::FlightDirectorOn, Button::Engage, Button::Heading, Button::AltitudeHold] {
        let _ = crate::autoflight::press(xp, b).await;
    }
}

async fn ap_off(xp: &XPlaneClient) {
    let _ = crate::autoflight::press(xp, crate::autoflight::Button::DisconnectFd).await;
}

/// The lever held near `ias`: a hand on the throttle while the autopilot
/// holds the height (ident.py's).
async fn speed_hand(xp: &XPlaneClient, lever: &str, k: &Tick, ias: f64) {
    if let Ok(t) = xp.get_float(lever).await {
        let _ = xp.set_float(lever, (t + 0.004 * (ias - k.ias)).clamp(0.0, 1.0)).await;
    }
}

/// Settle on the autopilot for `secs`, the speed held.
async fn settle(xp: &XPlaneClient, lever: &str, t0: std::time::Instant, ias: f64, secs: f64) -> anyhow::Result<()> {
    let end = t0.elapsed().as_secs_f64() + secs;
    while t0.elapsed().as_secs_f64() < end {
        if crate::xplane::sim_frozen(xp).await {
            anyhow::bail!("the sim is paused or replaying: refused");
        }
        if let Some(k) = read_tick(xp, t0).await {
            speed_hand(xp, lever, &k, ias).await;
        }
        tokio::time::sleep(std::time::Duration::from_millis(100)).await;
    }
    Ok(())
}

/// One input on `axis` with `sign`: the samples (and the roll on the slip
/// for the yaw), or the abort's reason.
async fn fly_input(
    xp: &XPlaneClient,
    axis: Axis,
    sign: f64,
    t0: std::time::Instant,
) -> anyhow::Result<Result<(Vec<Sample>, Vec<Sample>), &'static str>> {
    let ch = match axis {
        Axis::Pitch => PITCH_CH,
        Axis::Roll => ROLL_CH,
        Axis::Yaw => YAW_CH,
    };
    // Every input is flown about the TRIMMED controls, as flight test flies
    // them: the yoke the autopilot holds at the disconnect (its servos
    // write the yoke). From centred controls the 172's nose moved 10 deg
    // before the input began and the flight measured nothing (LEMG,
    // 2026-10-13).
    let held = |n: &'static str| async move { xp.get_float(n).await.unwrap_or(0.0) };
    let (base_p, base_r, base_y) = tokio::join!(
        held("sim/cockpit2/controls/yoke_pitch_ratio"),
        held("sim/cockpit2/controls/yoke_roll_ratio"),
        held("sim/cockpit2/controls/yoke_heading_ratio"),
    );
    ap_off(xp).await;
    let start = loop {
        if let Some(k) = read_tick(xp, t0).await {
            break k;
        }
    };
    let begin = start.t;
    let (mut out, mut slip) = (Vec::new(), Vec::new());
    let mut why = None;
    // 1 s neutral, the input, then hands off to 6 s past its end.
    let span: f64 = ident::input_for(axis).iter().map(|s| s.0).sum::<f64>() + 7.0;
    loop {
        if crate::xplane::sim_frozen(xp).await {
            ap_on(xp).await;
            anyhow::bail!("the sim paused during an input: refused");
        }
        let now = t0.elapsed().as_secs_f64() - begin;
        if now > span {
            break;
        }
        let u = amplitude(axis) * sign * input_at(axis, now - 1.0);
        // Every channel every tick: a 1 s heartbeat, the others held.
        let (a, b, c) = tokio::join!(
            xp.set_float(PITCH_CH, base_p + if ch == PITCH_CH { u } else { 0.0 }),
            xp.set_float(ROLL_CH, base_r + if ch == ROLL_CH { u } else { 0.0 }),
            xp.set_float(YAW_CH, base_y + if ch == YAW_CH { u } else { 0.0 }),
        );
        if a.is_err() || b.is_err() || c.is_err() {
            why = Some("a channel write failed (is the plugin loaded?)");
            break;
        }
        let Some(k) = read_tick(xp, t0).await else { continue };
        // The tape: every tick of every input, so a fit or an abort can be
        // read afterwards (the first runs aborted with nothing to read).
        crate::blackbox::record(
            "identify",
            serde_json::json!({"event": "tick", "axis": format!("{axis:?}"), "sign": sign, "u": u,
                "base": [base_p, base_r, base_y], "theta": k.theta, "phi": k.phi, "alpha": k.alpha,
                "beta": k.beta, "p": k.p, "q": k.q, "r": k.r, "ias": k.ias, "vy": k.vy_fpm, "alt": k.alt_ft, "t": k.t}),
        );
        if let Some(w) = abort_reason(&k, &start) {
            why = Some(w);
            break;
        }
        out.push(sample(axis, &k, u));
        if axis == Axis::Yaw {
            slip.push(roll_on_slip(&k));
        }
    }
    // The channels stop: they expire within a second; the autopilot back.
    ap_on(xp).await;
    // An input stopped by a limit keeps what it measured before the limit:
    // the rudder's slip rolls a light aircraft past 20 deg in the hands-off
    // seconds, and throwing the whole input away left the 172's yaw
    // unmeasured on two runs (LEMG, 2026-10-13). Too short to fit, it is
    // dropped.
    const MIN_KEPT_S: f64 = 4.0;
    let kept_s = match (out.first(), out.last()) {
        (Some(a), Some(b)) => b.t - a.t,
        _ => 0.0,
    };
    Ok(match why {
        Some(_) if kept_s >= MIN_KEPT_S => Ok((out, slip)),
        Some(w) => Err(w),
        None => Ok((out, slip)),
    })
}

/// The power step: the lever 0.1 up for 12 s, 0.1 down for 12 s, the
/// autopilot holding the height.
async fn power_step(xp: &XPlaneClient, lever: &str, t0: std::time::Instant) -> Vec<EnergySample> {
    let base = xp.get_float(lever).await.unwrap_or(0.5);
    let begin = t0.elapsed().as_secs_f64();
    let (mut out, mut prev): (Vec<EnergySample>, Option<Tick>) = (Vec::new(), None);
    loop {
        let now = t0.elapsed().as_secs_f64() - begin;
        if now > 30.0 || crate::xplane::sim_frozen(xp).await {
            break;
        }
        let thr = if (3.0..15.0).contains(&now) {
            base + 0.1
        } else if (15.0..27.0).contains(&now) {
            base - 0.1
        } else {
            base
        }
        .clamp(0.0, 1.0);
        let _ = xp.set_float(lever, thr).await;
        let Some(k) = read_tick(xp, t0).await else { continue };
        crate::blackbox::record(
            "identify",
            serde_json::json!({"event": "power", "thr": thr, "ias": k.ias, "vy": k.vy_fpm, "alt": k.alt_ft, "theta": k.theta, "t": k.t}),
        );
        if let Some(p) = prev {
            let dt = (k.t - p.t).max(0.05);
            out.push(EnergySample { t: k.t, thr, ias_kts: k.ias, accel_kts_s: (k.ias - p.ias) / dt, vs_fpm: k.vy_fpm });
        }
        prev = Some(k);
    }
    let _ = xp.set_float(lever, base).await;
    out
}

/// `xvl identify`: `Ok(true)` when something was measured and stored.
pub async fn main_cmd(args: &[String], base_url: &str) -> anyhow::Result<bool> {
    let (mut alt_ft, mut ias, mut flap, mut here) = (5_000.0, None, None, false);
    let mut it = args.iter();
    while let Some(a) = it.next() {
        let mut num = |what: &str| -> anyhow::Result<f64> {
            it.next()
                .and_then(|v| v.parse().ok())
                .ok_or_else(|| anyhow::anyhow!("{what} needs a number\n{USAGE}"))
        };
        match a.as_str() {
            "--alt" => alt_ft = num("--alt")?,
            "--ias" => ias = Some(num("--ias")?),
            "--flap" => flap = Some(num("--flap")?),
            "--here" => here = true,
            "-h" | "--help" => {
                println!("{USAGE}");
                return Ok(true);
            }
            other => anyhow::bail!("unknown argument '{other}'\n{USAGE}"),
        }
    }
    let xp = XPlaneClient::new(base_url.to_string());
    let who = crate::detect::read_identity(&xp).await?;
    let picked = crate::detect::pick_profile(&who);
    let p = picked
        .profile
        .or_else(|| crate::aircraft::load(&picked.id))
        .ok_or_else(|| anyhow::anyhow!("no profile for {}", picked.id))?;
    crate::flight_state::install_for_check(&p);
    let acf_rel = who.relative_path.trim().to_string();
    let acf_text = std::fs::read_to_string(format!("{}/{acf_rel}", crate::config::xplane_root()))
        .map_err(|e| anyhow::anyhow!("the .acf {acf_rel} is unreadable ({e}): nothing to key a measurement to"))?;
    let ias = ias.or_else(|| crate::aircraft::approach_kts(&p)).unwrap_or(80.0);
    // The approach flap: the last detent short of full.
    let flap = flap.unwrap_or_else(|| {
        let d: Vec<f64> = p.flaps.as_ref().map(|f| f.detents.iter().map(|d| d.ratio).collect()).unwrap_or_default();
        d.iter().rev().nth(1).copied().unwrap_or(0.0)
    });
    println!("# Identification: {} ({acf_rel})\n", p.display_name);
    // Measuring the flight as it is needs it flying; an air start reloads
    // the flight (and is unpaused after the load), so a pause before it,
    // such as the one this tool leaves, is no reason to refuse.
    if here && crate::xplane::sim_frozen(&xp).await {
        println!("Refused: the sim is paused or replaying.");
        return Ok(false);
    }
    if !here {
        let (lat, lon, psi) = tokio::join!(
            xp.get_float("sim/flightmodel/position/latitude"),
            xp.get_float("sim/flightmodel/position/longitude"),
            xp.get_float(crate::datarefs::TRUE_HEADING_DEG),
        );
        let tas = crate::tools::setup_enroute::tas_kts(ias, alt_ft);
        let init = serde_json::json!({
            "aircraft": { "path": acf_rel },
            "lle_air_start": {
                "latitude": lat?, "longitude": lon?,
                "elevation_in_meters": alt_ft * 0.3048,
                "heading_true": psi.unwrap_or(0.0),
                "speed_in_meters_per_second": tas * 0.514_444,
            },
            "engine_status": { "all_engines": { "running": true } },
        });
        xp.set_data(INIT_JSON, init.to_string().as_bytes()).await?;
        println!("Air start at {alt_ft:.0} ft, {ias:.0} kt; silent while it loads.");
        // Nothing asks a loading sim (an init polled through hung X-Plane,
        // 2026-10-11): write once, wait, then the load gate holds the rest.
        tokio::time::sleep(std::time::Duration::from_secs(25)).await;
        let _ = xp.run_command("sim/operation/pause_off").await;
        tokio::time::sleep(std::time::Duration::from_secs(2)).await;
    }
    if crate::xplane::sim_frozen(&xp).await {
        println!("Refused: the sim is paused or replaying after the load.");
        return Ok(false);
    }
    let lever = crate::aircraft::throttle_dataref();
    let _ = crate::tools::flaps::set_handle(&xp, flap).await;
    let _ = crate::autoflight::set_dial(&xp, crate::autoflight::Dial::Altitude, (alt_ft / 100.0).round() * 100.0).await;
    if let Ok(psi) = xp.get_float(crate::datarefs::MAG_PSI).await {
        let _ = crate::autoflight::set_dial(&xp, crate::autoflight::Dial::HeadingBug, psi).await;
    }
    ap_on(&xp).await;
    let t0 = std::time::Instant::now();
    settle(&xp, &lever, t0, ias, 40.0).await?;
    let k0 = read_tick(&xp, t0).await.ok_or_else(|| anyhow::anyhow!("the sim does not read"))?;
    let conditions = Conditions { ias_kts: k0.ias, alt_ft: k0.alt_ft, flap };
    let mut fits: [Option<Fit>; 3] = [None, None, None];
    let mut slips = Vec::new();
    let mut aborted = None;
    'axes: for (n, axis) in [Axis::Pitch, Axis::Roll, Axis::Yaw].into_iter().enumerate() {
        let mut runs = Vec::new();
        for sign in [1.0, -1.0] {
            match fly_input(&xp, axis, sign, t0).await? {
                Ok((s, slip)) => {
                    runs.push(s);
                    slips.push(slip);
                }
                Err(why) => {
                    // One input aborted skips its axis, not the flight: the
                    // others are still measured. Only a height lost ends it.
                    println!("{axis:?} {sign:+.0}: ABORTED on {why}; the autopilot has it.");
                    settle(&xp, &lever, t0, ias, 20.0).await?;
                    if why.contains("height") {
                        aborted = Some(why);
                        break 'axes;
                    }
                    break;
                }
            }
            settle(&xp, &lever, t0, ias, 20.0).await?;
        }
        fits[n] = ident::fit_axis(&runs, axis.has_angle());
        match &fits[n] {
            Some(f) => println!(
                "{axis:?}: b {:.1} deg/s2 a unit, damping {:.2} /s, stiffness {:.2}, path {:.2} /s, delay {:.2} s, rate {:.1} deg/s a unit (R2 {:.2}, {} inputs)",
                f.response.b, f.response.damping, f.response.stiffness, f.response.path, f.response.delay_s,
                f.response.rate_gain(), f.r2, f.segments
            ),
            None => println!("{axis:?}: no input explained the answer; not stored."),
        }
    }
    if let (Some(roll), false) = (fits[1].as_mut(), slips.iter().all(|s| s.is_empty())) {
        roll.response.cross = ident::fit_cross(&slips, &roll.response).unwrap_or(0.0);
        println!("Roll on the slip: {:.2} deg/s2 a degree.", roll.response.cross);
    }
    let energy = if aborted.is_none() {
        let e = ident::fit_energy(&power_step(&xp, &lever, t0).await);
        match &e {
            Some(e) => println!("Energy: {:.0} fpm a unit of lever, the engine {:.1} s behind (R2 {:.2}).", e.fpm_per_unit, e.spool_s, e.r2),
            None => println!("Energy: the power step explained nothing; not stored."),
        }
        e
    } else {
        None
    };
    let _ = xp.run_command("sim/operation/pause_on").await;
    println!("\nPaused.");
    let [pitch, roll, yaw] = fits;
    if pitch.is_none() && roll.is_none() && yaw.is_none() && energy.is_none() {
        println!("Nothing measured: nothing stored.");
        return Ok(false);
    }
    let learned = Learned {
        version: VERSION,
        acf: acf_rel,
        acf_fingerprint: fingerprint(&acf_text),
        measured_on: today(),
        conditions,
        pitch,
        roll,
        yaw,
        energy,
    };
    let path = learned.save()?;
    crate::blackbox::record("identify", serde_json::json!({"acf": learned.acf, "stored": path.display().to_string(),
        "aborted": aborted, "learned": learned}));
    println!("Stored: {} (loaded at the next airframe detection).", path.display());
    Ok(true)
}

/// Today's date, UTC, YYYY-MM-DD (civil from days, Hinnant's algorithm).
fn today() -> String {
    let days = (std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs()) / 86_400) as i64;
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!("{y:04}-{m:02}-{d:02}")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tick(theta: f64, phi: f64, alt_ft: f64) -> Tick {
        Tick { t: 0.0, theta, phi, alpha: 3.0, beta: 1.0, p: 0.5, q: 0.2, r: 0.1, ias: 100.0, tas_kts: 110.0, vy_fpm: 0.0, alt_ft }
    }

    #[test]
    fn an_input_aborts_on_bank_pitch_and_height() {
        let start = tick(2.0, 0.0, 5_000.0);
        assert_eq!(abort_reason(&tick(3.0, 5.0, 4_990.0), &start), None);
        assert_eq!(abort_reason(&tick(3.0, -21.0, 5_000.0), &start), Some("bank"));
        assert_eq!(abort_reason(&tick(13.0, 0.0, 5_000.0), &start), Some("pitch"));
        assert_eq!(abort_reason(&tick(2.0, 0.0, 4_100.0), &start), Some("height"));
    }

    #[test]
    fn each_axis_samples_its_own_rate_and_angle() {
        let k = tick(2.0, 10.0, 5_000.0);
        assert_eq!(sample(Axis::Pitch, &k, 0.1).rate, 0.2);
        assert_eq!(sample(Axis::Pitch, &k, 0.1).angle, 3.0);
        assert_eq!(sample(Axis::Roll, &k, 0.1).angle, 0.0);
        let y = sample(Axis::Yaw, &k, 0.1);
        assert_eq!((y.rate, y.angle), (0.1, 1.0));
        assert!(y.angle_push < 0.0, "a right bank pushes the slip negative");
        assert_eq!(input_at(Axis::Roll, 0.5), 1.0);
        assert_eq!(input_at(Axis::Roll, 2.0), -1.0);
        assert_eq!(input_at(Axis::Roll, 3.5), 0.0);
        assert_eq!(today().len(), 10);
    }
}
