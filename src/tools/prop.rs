//! Constant-speed propeller control: `set_prop`.
//!
//! The blue prop levers command a *governed RPM*, not a raw position. X-Plane's
//! actuator dataref carries that as radians/second (all engines); the pilot and
//! the tach both speak RPM, so this tool works in RPM and converts. Named detents
//! (full/cruise/low) come from the airframe's [`PropProfile`]; an explicit RPM is
//! clamped to the governing range. Writing the actuator dataref moves the lever —
//! the same mechanism `set_throttle`/`set_mixture` use (no command needed).

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};
use crate::aircraft::PropProfile;

/// rad/sec → RPM (RPM = rad/s × 60 / 2π). Divide RPM by this to get rad/sec.
const RAD_PER_SEC_TO_RPM: f64 = 9.549_296_585_513_72;

pub struct SetProp {
    prop: PropProfile,
}

#[derive(Deserialize)]
struct PropArg {
    /// Named detent: full/cruise/low (+ synonyms). Ignored if `rpm` is given.
    #[serde(default)]
    setting: Option<String>,
    /// Explicit governed RPM, clamped to the airframe's range.
    #[serde(default)]
    rpm: Option<f64>,
}

impl SetProp {
    pub fn new(prop: PropProfile) -> Self {
        Self { prop }
    }

    /// Resolve the request to a target RPM within the governing range, or `None`
    /// if neither a usable `rpm` nor a recognized `setting` was given.
    fn target_rpm(&self, arg: &PropArg) -> Option<f64> {
        if let Some(rpm) = arg.rpm {
            return Some(rpm.clamp(self.prop.min_rpm, self.prop.max_rpm));
        }
        match arg.setting.as_deref()?.to_lowercase().as_str() {
            "full" | "high" | "max" | "maximum" | "takeoff" | "forward" | "fine" => {
                Some(self.prop.max_rpm)
            }
            "cruise" | "climb" | "mid" => Some(self.prop.cruise_rpm),
            "low" | "min" | "minimum" | "back" | "coarse" | "aft" => Some(self.prop.min_rpm),
            _ => None,
        }
    }
}

#[async_trait]
impl Tool for SetProp {
    fn name(&self) -> &str {
        "set_prop"
    }
    fn description(&self) -> &str {
        "Set the propeller RPM via the prop levers (constant-speed props): 'full' \
         (takeoff), 'cruise', or 'low', or an explicit RPM. Use for 'props full \
         forward', 'props up', 'set the props to cruise', 'prop RPM 1900', 'props \
         back to 1600', 'coarse pitch', 'props to 2000'."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "setting": {"type": "string", "enum": ["full", "cruise", "low"],
                            "description": "Named prop detent: full (takeoff), cruise, or low."},
                "rpm": {"type": "integer",
                        "description": "Explicit governed RPM (clamped to the airframe's range)."}
            }
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let arg: PropArg = serde_json::from_value(args.clone())?;
        let Some(rpm) = self.target_rpm(&arg) else {
            return Ok(format!(
                "Say a prop setting: full, cruise, low, or an RPM between {:.0} and {:.0}.",
                self.prop.min_rpm, self.prop.max_rpm
            ));
        };
        // Write the governed RPM as rad/sec, then read the actuator back and report
        // the actual commanded RPM — confirm the action, not the intent.
        ctx.xplane
            .set_float(&self.prop.dataref, rpm / RAD_PER_SEC_TO_RPM)
            .await?;
        let actual = ctx.xplane.get_float(&self.prop.dataref).await? * RAD_PER_SEC_TO_RPM;
        Ok(format!("Roger, props {} RPM.", round_to(actual, 50.0)))
    }
}

/// Round to the nearest `step` RPM — the tach and the pilot think in 50s/100s, not
/// the exact float that falls out of the rad/sec round-trip.
fn round_to(v: f64, step: f64) -> i64 {
    ((v / step).round() * step) as i64
}

#[cfg(test)]
mod tests {
    use super::*;

    fn c90_prop() -> PropProfile {
        PropProfile {
            dataref: "sim/cockpit2/engine/actuators/prop_rotation_speed_rad_sec_all".into(),
            max_rpm: 2200.0,
            cruise_rpm: 1900.0,
            min_rpm: 1600.0,
        }
    }

    fn arg(setting: Option<&str>, rpm: Option<f64>) -> PropArg {
        PropArg { setting: setting.map(|s| s.into()), rpm }
    }

    #[test]
    fn named_detents_map_to_profile_rpm() {
        let p = SetProp::new(c90_prop());
        assert_eq!(p.target_rpm(&arg(Some("full"), None)), Some(2200.0));
        assert_eq!(p.target_rpm(&arg(Some("forward"), None)), Some(2200.0));
        assert_eq!(p.target_rpm(&arg(Some("cruise"), None)), Some(1900.0));
        assert_eq!(p.target_rpm(&arg(Some("low"), None)), Some(1600.0));
        assert_eq!(p.target_rpm(&arg(Some("coarse"), None)), Some(1600.0));
        assert_eq!(p.target_rpm(&arg(Some("gibberish"), None)), None);
        assert_eq!(p.target_rpm(&arg(None, None)), None);
    }

    #[test]
    fn explicit_rpm_is_clamped_and_wins_over_setting() {
        let p = SetProp::new(c90_prop());
        assert_eq!(p.target_rpm(&arg(None, Some(1900.0))), Some(1900.0));
        assert_eq!(p.target_rpm(&arg(None, Some(9000.0))), Some(2200.0)); // over max
        assert_eq!(p.target_rpm(&arg(None, Some(500.0))), Some(1600.0)); // under min
        // rpm takes precedence over a named setting.
        assert_eq!(p.target_rpm(&arg(Some("low"), Some(2000.0))), Some(2000.0));
    }

    #[test]
    fn rpm_rad_sec_round_trip_is_stable() {
        // 1900 RPM → rad/sec → RPM must land back on 1900 after rounding.
        let rad = 1900.0 / RAD_PER_SEC_TO_RPM;
        assert_eq!(round_to(rad * RAD_PER_SEC_TO_RPM, 50.0), 1900);
    }
}
