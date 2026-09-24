//! Bleed air and crew oxygen: the one writer for the cabin's air.
//!
//! Declared per airframe (`cabin_air:` in the profile, [`CabinAirProfile`]),
//! republished on every swap. Every control is READ first and left alone in
//! state, fired only when off, and read back: the SF50's oxygen is a toggle,
//! so firing it blind turns a working supply OFF (it did, once, during the
//! rescue that found this: EDLP-LOWI, 2026-09-24).
//!
//! Callers: full auto with the takeoff config (every departure, including one
//! that starts with the engine already running, which is how the bleed was
//! missed), and the monitor's cabin-altitude callout.

use crate::aircraft::CabinAirProfile;
use crate::xplane::XPlane;

static CABIN_AIR: std::sync::Mutex<Option<CabinAirProfile>> = std::sync::Mutex::new(None);

/// Publish the airframe's cabin air. Called on EVERY profile swap; `None`
/// retires the previous airframe's.
pub fn publish(p: Option<CabinAirProfile>) {
    *CABIN_AIR.lock().unwrap_or_else(|e| e.into_inner()) = p;
}

fn declared() -> Option<CabinAirProfile> {
    CABIN_AIR.lock().unwrap_or_else(|e| e.into_inner()).clone()
}

/// What one control ended up as: `None` not declared (or unreadable before
/// and after), `Some(true)` on, `Some(false)` asked and did not take.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CabinAir {
    pub bleed: Option<bool>,
    pub oxygen: Option<bool>,
    /// Switched by this call (a line is owed for each).
    pub bleed_switched: bool,
    pub oxygen_switched: bool,
}

impl CabinAir {
    /// The milestone line: `None` when nothing was switched and nothing
    /// failed (a crew does not announce a switch already in place).
    pub fn line(&self) -> Option<String> {
        let mut failed = Vec::new();
        if self.bleed == Some(false) {
            failed.push("the bleed air");
        }
        if self.oxygen == Some(false) {
            failed.push("the oxygen");
        }
        if !failed.is_empty() {
            return Some(format!(
                "I couldn't switch on {}. Check it on the overhead.",
                failed.join(" or ")
            ));
        }
        match (self.bleed_switched, self.oxygen_switched) {
            (true, true) => Some("Bleed air and oxygen on."),
            (true, false) => Some("Bleed air on."),
            (false, true) => Some("Oxygen on."),
            (false, false) => None,
        }
        .map(str::to_string)
    }

    /// Anything switched by this call.
    pub fn changed(&self) -> bool {
        self.bleed_switched || self.oxygen_switched
    }
}

const SETTLE_MS: u64 = 400;

async fn read(xp: &dyn XPlane, name: &str) -> Option<f64> {
    xp.get_float(name).await.ok().filter(|v| v.is_finite())
}

async fn settle() {
    tokio::time::sleep(std::time::Duration::from_millis(SETTLE_MS)).await;
}

/// Bleed on: read, fired only when off, read back.
async fn bleed_on(xp: &dyn XPlane, p: &CabinAirProfile, changed: &mut bool) -> Option<bool> {
    if read(xp, &p.bleed_read).await.is_some_and(|v| v >= 0.5) {
        return Some(true);
    }
    xp.run_command(&p.bleed_on_command).await.ok()?;
    *changed = true;
    settle().await;
    read(xp, &p.bleed_read).await.map(|v| v >= 0.5)
}

/// Oxygen on: the button (a toggle, so only ever fired when it reads off),
/// then the valve behind it.
async fn oxygen_on(xp: &dyn XPlane, p: &CabinAirProfile, changed: &mut bool) -> Option<bool> {
    let mut on = None;
    if let (Some(toggle), Some(r)) = (&p.oxygen_toggle, &p.oxygen_read) {
        let now = read(xp, r).await?;
        on = Some(if now >= 1.0 {
            true
        } else {
            xp.run_command(toggle).await.ok()?;
            *changed = true;
            settle().await;
            read(xp, r).await.is_some_and(|v| v >= 1.0)
        });
    }
    if let Some(valve) = &p.oxygen_valve {
        let open = if read(xp, valve).await.is_some_and(|v| v >= 0.5) {
            true
        } else {
            let _ = xp.set_float(valve, 1.0).await;
            *changed = true;
            settle().await;
            read(xp, valve).await.is_some_and(|v| v >= 0.5)
        };
        on = Some(on.unwrap_or(true) && open);
    }
    on
}

/// Bleed air and crew oxygen on, where the airframe declares them.
pub async fn ensure_on(xp: &dyn XPlane) -> CabinAir {
    let Some(p) = declared() else {
        return CabinAir::default();
    };
    let (mut bleed_switched, mut oxygen_switched) = (false, false);
    let bleed = bleed_on(xp, &p, &mut bleed_switched).await;
    let oxygen = oxygen_on(xp, &p, &mut oxygen_switched).await;
    let out = CabinAir {
        bleed,
        oxygen,
        bleed_switched,
        oxygen_switched,
    };
    crate::blackbox::record(
        "cabin_air",
        serde_json::json!({ "bleed": bleed, "oxygen": oxygen,
            "bleed_switched": bleed_switched, "oxygen_switched": oxygen_switched }),
    );
    out
}

/// The altitude a hypoxic cabin descends to, handed to a flying follower
/// (whose terrain floor still outranks it).
pub const HYPOXIA_DESCENT_FT: f64 = 10_000.0;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xplane::FakeXPlane;

    fn sf50() -> CabinAirProfile {
        CabinAirProfile {
            bleed_on_command: "laminar/SF50/fresh_air_on".into(),
            bleed_read: "sim/cockpit2/bleedair/actuators/pack_center".into(),
            oxygen_toggle: Some("sim/operation/slider_20".into()),
            oxygen_read: Some("sim/cockpit2/oxygen/actuators/demand_flow_setting".into()),
            oxygen_valve: Some("sim/cockpit2/oxygen/actuators/o2_valve_on".into()),
        }
    }

    static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    /// Runs against the process-wide selector, so one test owns it.
    #[tokio::test]
    async fn switches_only_what_is_off_and_never_toggles_a_working_supply_off() {
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let p = sf50();
        publish(Some(p.clone()));

        // EDLP-LOWI as found: bleed off, the button on (2), the valve shut.
        let xp = FakeXPlane::with_reads(&[
            (p.bleed_read.as_str(), 0.0),
            (p.oxygen_read.as_deref().unwrap(), 2.0),
            (p.oxygen_valve.as_deref().unwrap(), 0.0),
        ]);
        xp.step_on_command(&p.bleed_on_command, &p.bleed_read, 1.0);
        let got = ensure_on(&xp).await;
        assert!(xp.ran(&p.bleed_on_command));
        assert!(
            !xp.ran(p.oxygen_toggle.as_deref().unwrap()),
            "the button read on: toggling it would have shut the oxygen off"
        );
        assert_eq!(xp.last_write(p.oxygen_valve.as_deref().unwrap()), Some(1.0));
        assert_eq!(got.bleed, Some(true));
        assert!(got.bleed_switched && got.oxygen_switched, "{got:?}");

        // All already on: nothing fired, nothing said.
        let xp = FakeXPlane::with_reads(&[
            (p.bleed_read.as_str(), 1.0),
            (p.oxygen_read.as_deref().unwrap(), 2.0),
            (p.oxygen_valve.as_deref().unwrap(), 1.0),
        ]);
        let got = ensure_on(&xp).await;
        assert!(!xp.ran(&p.bleed_on_command) && !xp.wrote(p.oxygen_valve.as_deref().unwrap()));
        assert_eq!(got.line(), None);

        // The button off: fired once.
        let xp = FakeXPlane::with_reads(&[
            (p.bleed_read.as_str(), 1.0),
            (p.oxygen_read.as_deref().unwrap(), 0.0),
            (p.oxygen_valve.as_deref().unwrap(), 1.0),
        ]);
        xp.step_on_command(
            p.oxygen_toggle.as_deref().unwrap(),
            p.oxygen_read.as_deref().unwrap(),
            2.0,
        );
        let got = ensure_on(&xp).await;
        assert_eq!(got.oxygen, Some(true));
        assert_eq!(got.line().as_deref(), Some("Oxygen on."));

        // An undeclared airframe touches nothing.
        publish(None);
        let xp = FakeXPlane::new();
        assert_eq!(ensure_on(&xp).await, CabinAir::default());
    }

    #[test]
    fn a_switch_that_did_not_take_is_named() {
        let c = CabinAir {
            bleed: Some(false),
            oxygen: Some(true),
            bleed_switched: true,
            oxygen_switched: false,
        };
        assert!(c.line().unwrap().contains("the bleed air"));
    }
}
