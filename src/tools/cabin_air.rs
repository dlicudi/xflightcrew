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
//! missed), and the monitor's cabin-altitude callout: bleed and the oxygen
//! SUPPLY. The masks are the monitor's `masks_tick` alone, worn only in the
//! cabin-altitude warning band (#159).

use crate::aircraft::CabinAirProfile;
use crate::xplane::XPlane;

/// Publish the airframe's cabin air alone (the snapshot's `cabin_air`,
/// republished on every swap by `flight_state::publish_airframe`), and
/// forget the masks as a swap does.
#[cfg(test)]
pub fn publish(p: Option<CabinAirProfile>) {
    crate::aircraft::update_airframe_facts(|a| a.cabin_air = p);
    forget_masks();
}

/// A claim on the masks never outlives its airframe: called on EVERY
/// profile swap.
pub fn forget_masks() {
    *MASKS.lock().unwrap_or_else(|e| e.into_inner()) = (MaskWatch::default(), None);
}

/// The live airframe's cabin air; `None` retires the previous airframe's.
fn declared() -> Option<CabinAirProfile> {
    crate::aircraft::airframe_facts().cabin_air.clone()
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

/// The oxygen SUPPLY on: the master button (a toggle, so only ever fired
/// when it reads off). Never the masks: those are worn, not switched on
/// (`MaskWatch`).
async fn oxygen_on(xp: &dyn XPlane, p: &CabinAirProfile, changed: &mut bool) -> Option<bool> {
    let (toggle, r) = (p.oxygen_toggle.as_ref()?, p.oxygen_read.as_ref()?);
    let now = read(xp, r).await?;
    if now >= 1.0 {
        return Some(true);
    }
    xp.run_command(toggle).await.ok()?;
    *changed = true;
    settle().await;
    Some(read(xp, r).await.is_some_and(|v| v >= 1.0))
}

/// Bleed air and the crew oxygen SUPPLY on, where the airframe declares
/// them. The masks stay where they are: a mission wants oxygen available,
/// not a crew breathing it on the ground (#159).
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

/// Cabin altitude at which the masks go on: the monitor's warning band
/// (`cabin_alt_warning`, the regulatory oxygen line).
pub const MASKS_ON_CABIN_FT: f64 = 12_500.0;
/// And come off again, a thousand feet under it so a cabin hovering on the
/// line does not don and doff them in turn.
pub const MASKS_OFF_CABIN_FT: f64 = 11_500.0;
/// Seconds between mask watch ticks.
const MASK_TICK_S: f64 = 2.0;

/// What the mask watch does this tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MaskStep {
    Hold,
    /// Entering the warning band with the masks off: put them on.
    Don,
    /// Below the band again with the masks WE put on still on: off.
    Doff,
    /// The pilot took our masks off inside the band: the claim is dropped
    /// and nothing puts them back this spell.
    Released,
}

/// The crew's masks, worn only while the cabin warrants it. On the SF50 the
/// masks are `o2_valve_on` (clicking the cockpit masks, `sf50_oxygen.lua`:
/// "opening o2 valve (wearing masks) will play breathing sound regardless if
/// there is an actual oxy flow"), and the master button in front of them is
/// the supply. Both were once written on for every mission and a
/// `valve_sync` re-opened the masks two seconds after the pilot took them
/// off, so the breathing played the whole flight (#159).
///
/// One-shot per spell in the band (the `cockpit_auto` shape): donned on the
/// way in only when they read off (a pilot's own masks are not claimed),
/// taken off on the way out only when WE put them on, and never re-asserted
/// over a pilot who took them off.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct MaskWatch {
    /// Inside the band, from the entry until the cabin is back under
    /// `MASKS_OFF_CABIN_FT`.
    spell: bool,
    /// The masks on are ours to take off.
    claimed: bool,
}

impl MaskWatch {
    pub fn step(&mut self, cabin_ft: f64, airborne: bool, donned: bool) -> MaskStep {
        if !self.spell {
            if airborne && cabin_ft >= MASKS_ON_CABIN_FT {
                self.spell = true;
                if !donned {
                    self.claimed = true;
                    return MaskStep::Don;
                }
            }
            return MaskStep::Hold;
        }
        if cabin_ft < MASKS_OFF_CABIN_FT {
            self.spell = false;
            let ours = std::mem::take(&mut self.claimed);
            return if ours && donned {
                MaskStep::Doff
            } else {
                MaskStep::Hold
            };
        }
        if self.claimed && !donned {
            self.claimed = false;
            return MaskStep::Released;
        }
        MaskStep::Hold
    }

    /// A don that did not read back is not a claim.
    fn drop_claim(&mut self) {
        self.claimed = false;
    }
}

static MASKS: std::sync::Mutex<(MaskWatch, Option<std::time::Instant>)> = std::sync::Mutex::new((
    MaskWatch {
        spell: false,
        claimed: false,
    },
    None,
));

const CABIN_ALT: &str = "sim/cockpit2/pressurization/indicators/cabin_altitude_ft";

/// The monitor's mask tick (crew hands only): rate-limited, reads the cabin
/// and the masks itself (a failed read is no measurement: the tick is
/// skipped, never a cabin at 0 ft that takes the masks off), and acts on
/// the watch. Donning turns the supply on first (a mask with no flow is the
/// EDLP-LOWI blackout). Returns the line owed, if any, and whether it is a
/// warning (the don) or an advisory (the doff).
pub async fn masks_tick(xp: &dyn XPlane, on_ground: bool) -> Option<(bool, String)> {
    let p = declared()?;
    let masks = p.oxygen_masks.clone()?;
    {
        let mut g = MASKS.lock().unwrap_or_else(|e| e.into_inner());
        if g.1.is_some_and(|t| t.elapsed().as_secs_f64() < MASK_TICK_S) {
            return None;
        }
        g.1 = Some(std::time::Instant::now());
    }
    let cabin = read(xp, CABIN_ALT).await?;
    let donned = read(xp, &masks).await? >= 0.5;
    let step = MASKS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .0
        .step(cabin, !on_ground, donned);
    match step {
        MaskStep::Hold => None,
        MaskStep::Released => {
            crate::blackbox::record(
                "cabin_air",
                serde_json::json!({ "event": "masks", "action": "released", "cabin_ft": cabin.round() }),
            );
            None
        }
        MaskStep::Don => {
            let mut switched = false;
            let supply = oxygen_on(xp, &p, &mut switched).await;
            let _ = xp.set_float(&masks, 1.0).await;
            settle().await;
            let taken = read(xp, &masks).await.is_some_and(|v| v >= 0.5);
            if !taken {
                MASKS
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .0
                    .drop_claim();
            }
            crate::blackbox::record(
                "cabin_air",
                serde_json::json!({ "event": "masks", "action": "don", "cabin_ft": cabin.round(),
                    "taken": taken, "supply": supply, "supply_switched": switched }),
            );
            Some((
                true,
                match (taken, supply) {
                    (true, Some(false)) => {
                        "Masks on, but the oxygen master did not come on. Check it now."
                    }
                    (true, _) => "Oxygen masks on.",
                    (false, _) => "I couldn't get the oxygen masks on. Put yours on now.",
                }
                .to_string(),
            ))
        }
        MaskStep::Doff => {
            let _ = xp.set_float(&masks, 0.0).await;
            settle().await;
            let taken = read(xp, &masks).await.is_some_and(|v| v < 0.5);
            crate::blackbox::record(
                "cabin_air",
                serde_json::json!({ "event": "masks", "action": "doff", "cabin_ft": cabin.round(),
                    "taken": taken }),
            );
            taken.then(|| (false, "Cabin's comfortable again, masks off.".to_string()))
        }
    }
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
            oxygen_masks: Some("sim/cockpit2/oxygen/actuators/o2_valve_on".into()),
        }
    }

    /// The SF50's real profile, not a copy: the masks are `o2_valve_on`.
    #[test]
    fn the_sf50_profile_names_supply_and_masks_apart() {
        let text = std::fs::read_to_string(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("profiles/cirrus_vision_sf50.yaml"),
        )
        .unwrap();
        let doc: serde_yaml::Value = serde_yaml::from_str(&text).unwrap();
        let air: CabinAirProfile = serde_yaml::from_value(doc["cabin_air"].clone()).unwrap();
        assert_eq!(air, sf50());
    }

    static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    fn rearm_tick() {
        MASKS.lock().unwrap_or_else(|e| e.into_inner()).1 = None;
    }

    /// Runs against the process-wide selector, so one test owns it.
    #[tokio::test]
    async fn switches_only_what_is_off_and_never_toggles_a_working_supply_off() {
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let p = sf50();
        let masks = p.oxygen_masks.as_deref().unwrap();
        publish(Some(p.clone()));

        // EDLP-LOWI as found: bleed off, the button on (2). The masks are
        // not the mission's: nothing puts them on at the start (#159).
        let xp = FakeXPlane::with_reads(&[
            (p.bleed_read.as_str(), 0.0),
            (p.oxygen_read.as_deref().unwrap(), 2.0),
            (masks, 0.0),
        ]);
        xp.step_on_command(&p.bleed_on_command, &p.bleed_read, 1.0);
        let got = ensure_on(&xp).await;
        assert!(xp.ran(&p.bleed_on_command));
        assert!(
            !xp.ran(p.oxygen_toggle.as_deref().unwrap()),
            "the button read on: toggling it would have shut the oxygen off"
        );
        assert!(!xp.wrote(masks), "a mission never puts the masks on");
        assert_eq!(got.bleed, Some(true));
        assert_eq!(got.oxygen, Some(true));
        assert!(got.bleed_switched && !got.oxygen_switched, "{got:?}");
        assert_eq!(got.line().as_deref(), Some("Bleed air on."));

        // All already on: nothing fired, nothing said, and masks a pilot
        // wears stay on.
        let xp = FakeXPlane::with_reads(&[
            (p.bleed_read.as_str(), 1.0),
            (p.oxygen_read.as_deref().unwrap(), 2.0),
            (masks, 1.0),
        ]);
        let got = ensure_on(&xp).await;
        assert!(!xp.ran(&p.bleed_on_command) && !xp.wrote(masks));
        assert_eq!(got.line(), None);

        // The button off: fired once, the supply only.
        let xp = FakeXPlane::with_reads(&[
            (p.bleed_read.as_str(), 1.0),
            (p.oxygen_read.as_deref().unwrap(), 0.0),
            (masks, 0.0),
        ]);
        xp.step_on_command(
            p.oxygen_toggle.as_deref().unwrap(),
            p.oxygen_read.as_deref().unwrap(),
            2.0,
        );
        let got = ensure_on(&xp).await;
        assert_eq!(got.oxygen, Some(true));
        assert_eq!(got.line().as_deref(), Some("Oxygen on."));
        assert!(!xp.wrote(masks));

        // 2026-09-28: the supply on at a comfortable cabin, the pilot has
        // the masks off. Nothing puts them back (the old valve_sync read
        // them back 1 seconds after the pilot's write of 0).
        publish(Some(p.clone()));
        let xp = FakeXPlane::with_reads(&[
            (CABIN_ALT, 7_800.0),
            (p.oxygen_read.as_deref().unwrap(), 2.0),
            (masks, 0.0),
        ]);
        for _ in 0..3 {
            rearm_tick();
            assert_eq!(masks_tick(&xp, false).await, None);
        }
        assert!(!xp.wrote(masks));

        // The hypoxic climb: into the warning band with the supply OFF and
        // the masks off. The supply comes on first, then the masks.
        let xp = FakeXPlane::with_reads(&[
            (CABIN_ALT, 13_200.0),
            (p.oxygen_read.as_deref().unwrap(), 0.0),
            (masks, 0.0),
        ]);
        xp.step_on_command(
            p.oxygen_toggle.as_deref().unwrap(),
            p.oxygen_read.as_deref().unwrap(),
            2.0,
        );
        rearm_tick();
        let line = masks_tick(&xp, false).await;
        assert!(xp.ran(p.oxygen_toggle.as_deref().unwrap()));
        assert_eq!(xp.last_write(masks), Some(1.0));
        // The fake does not apply writes, so the don read back off: said.
        assert_eq!(
            line,
            Some((
                true,
                "I couldn't get the oxygen masks on. Put yours on now.".to_string()
            ))
        );

        // A failed cabin read is no measurement: nothing happens.
        publish(Some(p.clone()));
        let xp = FakeXPlane::with_reads(&[(masks, 1.0)]);
        rearm_tick();
        assert_eq!(masks_tick(&xp, false).await, None);
        assert!(!xp.wrote(masks));

        // An undeclared airframe touches nothing.
        publish(None);
        let xp = FakeXPlane::new();
        assert_eq!(ensure_on(&xp).await, CabinAir::default());
        rearm_tick();
        assert_eq!(masks_tick(&xp, false).await, None);
    }

    #[test]
    fn masks_are_worn_only_in_the_warning_band() {
        let mut w = MaskWatch::default();
        // A comfortable cabin and the ground: nothing, whatever the masks.
        assert_eq!(w.step(8_000.0, true, false), MaskStep::Hold);
        assert_eq!(w.step(8_000.0, true, true), MaskStep::Hold);
        assert_eq!(
            w.step(13_000.0, false, false),
            MaskStep::Hold,
            "on the ground"
        );
        assert_eq!(
            w.step(11_000.0, true, false),
            MaskStep::Hold,
            "the caution band"
        );
        // Into the band: on, once.
        assert_eq!(w.step(12_600.0, true, false), MaskStep::Don);
        assert_eq!(w.step(14_000.0, true, true), MaskStep::Hold);
        // Hovering on the line: still on (hysteresis).
        assert_eq!(w.step(12_000.0, true, true), MaskStep::Hold);
        assert_eq!(w.step(12_700.0, true, true), MaskStep::Hold);
        // Well under it: off, once.
        assert_eq!(w.step(11_400.0, true, true), MaskStep::Doff);
        assert_eq!(w.step(9_000.0, true, false), MaskStep::Hold);
        // A second climb is a second spell.
        assert_eq!(w.step(13_000.0, true, false), MaskStep::Don);
    }

    #[test]
    fn masks_are_never_reasserted_over_the_pilot() {
        // The pilot takes ours off in the band: released, never put back.
        let mut w = MaskWatch::default();
        assert_eq!(w.step(13_000.0, true, false), MaskStep::Don);
        assert_eq!(w.step(13_500.0, true, false), MaskStep::Released);
        assert_eq!(w.step(14_000.0, true, false), MaskStep::Hold);
        // Puts them on again himself: his, not taken off at the exit.
        assert_eq!(w.step(14_000.0, true, true), MaskStep::Hold);
        assert_eq!(w.step(10_000.0, true, true), MaskStep::Hold);

        // Masks the pilot already wore at the entry are his too.
        let mut w = MaskWatch::default();
        assert_eq!(w.step(13_000.0, true, true), MaskStep::Hold);
        assert_eq!(w.step(10_000.0, true, true), MaskStep::Hold);

        // A don that did not take is no claim.
        let mut w = MaskWatch::default();
        assert_eq!(w.step(13_000.0, true, false), MaskStep::Don);
        w.drop_claim();
        assert_eq!(w.step(13_000.0, true, false), MaskStep::Hold);
        assert_eq!(w.step(10_000.0, true, true), MaskStep::Hold);
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
