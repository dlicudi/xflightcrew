//! Instrument settings: `set_barometer` (altimeter setting).

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};

const BARO: &str = "sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot";
/// Pascals per inch of mercury (1013.25 hPa ≈ 29.92 inHg).
const PA_PER_INHG: f64 = 3386.389;
const DG_SYNC: &str = "sim/instruments/DG_sync_mag";
const DG_DRIFT: &str = "sim/cockpit/gyros/dg_drift_vac_deg";
// Low-vacuum annunciator: the DG is vacuum-driven, so a sync won't hold if the
// vacuum's failed — worth saying so rather than confirming a sync that'll drift.
const LOW_VACUUM: &str = "sim/cockpit/warnings/annunciators/low_vacuum";
const PITOT_HEAT: &str = "sim/cockpit2/ice/ice_pitot_heat_on_pilot";
// Command-driven so the panel switch animates (a raw write may not); PITOT_HEAT
// is read back to confirm.
const CMD_PITOT_HEAT_ON: &str = "sim/ice/pitot_heat0_on";
const CMD_PITOT_HEAT_OFF: &str = "sim/ice/pitot_heat0_off";

pub struct SetBarometer;

#[derive(Deserialize)]
struct Args {
    /// Altimeter setting in inches of mercury (e.g. 29.92).
    inhg: f64,
}

#[async_trait]
impl Tool for SetBarometer {
    fn name(&self) -> &str {
        "set_barometer"
    }

    fn description(&self) -> &str {
        "Set the altimeter (barometric pressure) in inches of mercury. Use for \
         'set altimeter two niner niner two' (29.92), 'altimeter 30.15', \
         'set baro 29.80', 'set the Kollsman to 29.92', 'altimeter setting \
         30.02', 'set QNH three zero one zero'. (inHg only — convert a hPa/QNH \
         value before calling.)"
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "inhg": {
                    "type": "number",
                    "minimum": 28.0,
                    "maximum": 31.5,
                    "description": "Altimeter setting in inches of mercury, e.g. 29.92."
                }
            },
            "required": ["inhg"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let args: Args = serde_json::from_value(args.clone())?;
        let inhg = args.inhg.clamp(28.0, 31.5);
        ctx.xplane.set_float(BARO, inhg).await?;
        let actual = ctx.xplane.get_float(BARO).await?;
        Ok(format!("Roger, altimeter {actual:.2}."))
    }
}

/// Standard pressure (inHg) — what the altimeter is set to above the transition
/// altitude (1013.25 hPa).
const STANDARD_INHG: f64 = 29.92;

/// An altimeter error worth a word or a hand: 0.03 inHg is about 30 ft. The
/// sim's QNH drifts a hundredth at a time under real weather, so anything
/// tighter would have the copilot re-touching the knob all flight.
pub const ALTIMETER_TOLERANCE_INHG: f64 = 0.03;

/// The setting the altimeter SHOULD carry: local QNH below the transition
/// altitude, standard at or above it. Pure; the one rule behind the spoken
/// tool, full auto's departure set, the follower's arm and the monitor's
/// cross-check, so they can never disagree about what "correct" is.
pub fn correct_setting_inhg(
    indicated_alt_ft: f64,
    transition_altitude_ft: f64,
    qnh_pa: f64,
) -> f64 {
    if indicated_alt_ft >= transition_altitude_ft {
        STANDARD_INHG
    } else {
        (qnh_pa / PA_PER_INHG).clamp(28.0, 31.5)
    }
}

/// "three zero one zero" for 30.10: the way a setting is read on the air.
pub fn spoken_setting(inhg: f64) -> String {
    crate::atis::spoken_digits(&format!("{inhg:.2}").replace('.', ""))
}

/// What a [`sync_altimeter`] did: the knob before and after.
#[derive(Debug, Clone, Copy)]
pub struct AltimeterSync {
    pub was: f64,
    pub now: f64,
}

impl AltimeterSync {
    /// The knob actually moved (by more than the tolerance).
    pub fn changed(&self) -> bool {
        (self.now - self.was).abs() >= ALTIMETER_TOLERANCE_INHG
    }
}

/// Set the altimeter to the correct setting for where the aircraft is now.
/// `None` when the sim gave no pressure (nothing written). Read back from the
/// knob, so `now` is what the pilot sees, not what was asked.
pub async fn sync_altimeter(
    xp: &crate::xplane::XPlaneClient,
    transition_altitude_ft: f64,
) -> Option<AltimeterSync> {
    let pa = xp.get_float(crate::datarefs::QNH_PASCALS).await.ok()?;
    if pa <= 0.0 {
        return None;
    }
    let alt = xp
        .get_float(crate::datarefs::ALTITUDE_FT)
        .await
        .unwrap_or(0.0);
    let was = xp.get_float(BARO).await.unwrap_or(0.0);
    let want = correct_setting_inhg(alt, transition_altitude_ft, pa);
    xp.set_float(BARO, want).await.ok()?;
    let now = xp.get_float(BARO).await.unwrap_or(want);
    Some(AltimeterSync { was, now })
}

pub struct SetAltimeterCurrent {
    /// Transition altitude (ft MSL): at or above it the "correct" setting is
    /// standard pressure, not local QNH.
    pub transition_altitude_ft: f64,
}

#[async_trait]
impl Tool for SetAltimeterCurrent {
    fn name(&self) -> &str {
        "set_altimeter_current"
    }

    fn description(&self) -> &str {
        "Set the altimeter to the CORRECT setting automatically — no number needed. \
         Below the transition altitude that's the local pressure (QNH), read from \
         the environment; at or above it that's standard pressure (29.92). Use when \
         the pilot wants it right without stating a value: 'set the altimeter', 'set \
         altimeter to current', 'altimeter to field', 'set the local QNH', 'set \
         standard', 'altimeter standard', 'update the altimeter', 'correct the \
         altimeter'. For an explicit value ('altimeter two niner niner two') use \
         set_barometer instead."
    }

    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }

    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        // Above the transition altitude everyone flies on standard (29.92), so the
        // "correct" setting there is standard, not QNH — dialling QNH in the flight
        // levels would be wrong. Below it, the sim's ambient sea-level pressure
        // (QNH) is correct: with it set the altimeter reads true MSL / field
        // elevation. Use indicated altitude to decide (it's what the pilot sees).
        let alt = ctx
            .xplane
            .get_float(crate::datarefs::ALTITUDE_FT)
            .await
            .unwrap_or(0.0);
        let pa = ctx.xplane.get_float(crate::datarefs::QNH_PASCALS).await?;
        let inhg = correct_setting_inhg(alt, self.transition_altitude_ft, pa);
        ctx.xplane.set_float(BARO, inhg).await?;
        let actual = ctx.xplane.get_float(BARO).await?;
        if alt >= self.transition_altitude_ft {
            let fl = (self.transition_altitude_ft / 100.0).round() as i64;
            return Ok(format!(
                "Roger, above the transition altitude — altimeter standard, {actual:.2}. \
                 (Local QNH below flight level {fl}.)"
            ));
        }
        Ok(format!("Roger, altimeter set to the local {actual:.2}."))
    }
}

pub struct SetPitotHeat {
    /// Airframe-specific switch when the stock pitot-heat command is a no-op
    /// (the profile's `pitot_heat_dataref`). The Laminar MD-82's ice script
    /// drives all pitot/stall heat from its overhead heat-selector knob
    /// (`laminar/md82/ice/heatknob`, 0 = off, 1–9 = on) and overwrites the
    /// stock datarefs every frame, so only the knob works. The stock mirror
    /// dataref is still used for the readback — the script keeps it honest.
    pub override_dataref: Option<String>,
}

#[derive(Deserialize)]
struct PitotArgs {
    on: bool,
}

#[async_trait]
impl Tool for SetPitotHeat {
    fn name(&self) -> &str {
        "set_pitot_heat"
    }

    fn description(&self) -> &str {
        "Turn pitot heat on or off (anti-ice for the airspeed system). Use for \
         'pitot heat on', 'pitot heat off', 'turn on pitot heat', 'pitot anti-ice \
         on', 'pitot heat', 'anti-ice on'."
    }

    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": { "on": {"type": "boolean", "description": "true = on, false = off"} },
            "required": ["on"]
        })
    }

    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: PitotArgs = serde_json::from_value(args.clone())?;
        match &self.override_dataref {
            Some(dref) => {
                ctx.xplane
                    .set_float(dref, if a.on { 1.0 } else { 0.0 })
                    .await?;
            }
            None => {
                ctx.xplane
                    .run_command(if a.on {
                        CMD_PITOT_HEAT_ON
                    } else {
                        CMD_PITOT_HEAT_OFF
                    })
                    .await?;
            }
        }
        // The mirror dataref updates a frame behind an overridden switch (the
        // airframe's script copies switch → dataref in after_physics).
        tokio::time::sleep(std::time::Duration::from_millis(300)).await;
        let on = ctx.xplane.get_float(PITOT_HEAT).await? > 0.5;
        if on != a.on {
            return Ok(format!(
                "Tried, but pitot heat reads {}. Check the switch.",
                if on { "on" } else { "off" }
            ));
        }
        Ok(format!(
            "Roger, pitot heat {}.",
            if on { "on" } else { "off" }
        ))
    }
}

pub struct AlignGyro;

#[async_trait]
impl Tool for AlignGyro {
    fn name(&self) -> &str {
        "align_gyro"
    }

    fn description(&self) -> &str {
        "Align the directional gyro (heading indicator) to the magnetic compass. \
         This is the fix when the gyro/heading indicator has drifted or is wrong. \
         Use for 'align the gyro', 'sync the heading indicator', 'set the DG', \
         'sync the directional gyro', 'fix the gyro drift', 'the gyro is drifting', \
         'correct the heading indicator', 'the DG is off'."
    }

    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }

    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        ctx.xplane.run_command(DG_SYNC).await?;
        // Confirm the drift is gone (best-effort). dg_drift_vac_deg reads near
        // 360 for a small *negative* drift, so fold it into -180..180 first —
        // otherwise a -4° drift would read back as "356 degrees".
        let raw = ctx.xplane.get_float(DG_DRIFT).await.unwrap_or(0.0);
        let drift = (raw + 180.0).rem_euclid(360.0) - 180.0;
        // If the vacuum's low, the gyro itself is failing — a sync won't hold, so
        // flag that instead of confirming a sync that'll just drift back off.
        if ctx.xplane.get_float(LOW_VACUUM).await.unwrap_or(0.0) > 0.5 {
            return Ok("Synced the gyro, but vacuum's low — it won't hold. \
                       Use the compass and cross-check the attitude indicator."
                .to_string());
        }
        // A couple of degrees is a good sync (the compass itself isn't perfect).
        if drift.abs() < 2.0 {
            Ok("Roger, directional gyro synced to the compass.".to_string())
        } else {
            Ok(format!(
                "Gyro synced; still {:.0} degrees of drift.",
                drift.abs()
            ))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn correct_setting_is_qnh_below_and_standard_above_transition() {
        // 101,946 Pa is the LINTO QNH of 2026-09-14: 30.10, not the 30.15 flown.
        let q = correct_setting_inhg(7_000.0, 18_000.0, 101_946.0);
        assert!((q - 30.10).abs() < 0.005, "{q}");
        assert_eq!(correct_setting_inhg(18_000.0, 18_000.0, 101_946.0), 29.92);
        assert_eq!(correct_setting_inhg(25_000.0, 6_000.0, 98_000.0), 29.92);
        // A silly pressure is clamped, never written raw.
        assert_eq!(correct_setting_inhg(0.0, 18_000.0, 0.0), 28.0);
    }

    #[test]
    fn a_setting_is_spoken_digit_by_digit() {
        assert_eq!(spoken_setting(30.10), "three zero one zero");
        assert_eq!(spoken_setting(29.92), "two niner niner two");
    }

    #[test]
    fn a_sync_within_tolerance_did_not_change_anything() {
        assert!(!AltimeterSync {
            was: 30.15,
            now: 30.16
        }
        .changed());
        assert!(AltimeterSync {
            was: 30.15,
            now: 30.10
        }
        .changed());
    }
}
