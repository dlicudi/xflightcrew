//! Ramp / ground-ops — the crew's coordination with the ground: chocks, ground
//! power, pushback, and ground services. These aren't airframe systems; they're
//! X-Plane ground-crew actions (`sim/ground_ops/*` and the chock toggle), so
//! they're available on any aircraft, not gated by the profile.
//!
//! The `toggle_*` commands are toggles, so the on/off tools read the current
//! state first and only fire the command when it differs — the same "confirm the
//! action, not the intent" discipline as the rest of the surface.

use anyhow::Result;
use async_trait::async_trait;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};

// --- chocks ---
const CMD_TOGGLE_CHOCKS: &str = "sim/flight_controls/toggle_chocks";
/// Per-wheel chock state (array, 1 = chocked).
const IS_CHOCKED: &str = "sim/flightmodel2/gear/is_chocked";
// --- ground power ---
/// Summons or dismisses the ground power CART. It drives up, plugs in, and
/// starts generating — and that is all it does.
const CMD_TOGGLE_GPU: &str = "sim/ground_ops/toggle_gpu_request";
/// What the cart is generating. Above zero means a cart is there and live; it
/// says NOTHING about whether the aircraft is drawing from it.
const GPU_VOLTS: &str = "sim/cockpit2/electrical/GPU_generator_volts";
/// The aircraft's external-power contactor: 1 = the cart is actually feeding
/// the bus.
///
/// This is the half that was missing, and the failure is silent in the worst
/// way (Baron 58, 2026-08-01). The cart arrived and generated 28 volts, so a
/// check on `GPU_VOLTS` alone said "ground power's already connected" — twice,
/// to a pilot who had been told by full auto that the battery was at 21 volts
/// and would not start. The bus was still on the battery at 20.7. On airframes
/// with an EXT PWR switch the pilot closes this themselves; the Baron models no
/// such switch, so nobody was ever going to close it and the cart sat there
/// humming to itself.
pub(crate) const GPU_ON: &str = "sim/cockpit/electrical/gpu_on";
/// Main bus volts — what the aircraft actually has to start with, and the only
/// honest readback for "is there power".
const BUS_VOLTS: &str = crate::datarefs::BUS_VOLTS;
// --- pushback ---
const CMD_PUSH_STRAIGHT: &str = "sim/ground_ops/pushback_straight";
const CMD_PUSH_LEFT: &str = "sim/ground_ops/pushback_left";
const CMD_PUSH_RIGHT: &str = "sim/ground_ops/pushback_right";
pub(crate) const CMD_PUSH_STOP: &str = "sim/ground_ops/pushback_stop";
// --- services ---
const CMD_SERVICE: &str = "sim/ground_ops/service_plane";

#[derive(Deserialize)]
struct OnArg {
    on: bool,
}

/// True when the chocks are in.
async fn chocked(ctx: &ToolCtx<'_>) -> bool {
    ctx.xplane
        .get_float_array(IS_CHOCKED)
        .await
        .ok()
        .and_then(|a| a.first().copied())
        .unwrap_or(0.0)
        > 0.5
}

/// True when a ground power cart is present and generating.
async fn cart_present(ctx: &ToolCtx<'_>) -> bool {
    ctx.xplane.get_float(GPU_VOLTS).await.unwrap_or(0.0) > 1.0
}

/// True when the aircraft is actually being fed by it — cart present AND the
/// contactor closed. Both halves, because either alone is a lie.
async fn gpu_connected(ctx: &ToolCtx<'_>) -> bool {
    cart_present(ctx).await && ctx.xplane.get_float(GPU_ON).await.unwrap_or(0.0) > 0.5
}

/// Main bus volts, for a readback that reports power rather than intent.
async fn bus_volts(ctx: &ToolCtx<'_>) -> Option<f64> {
    ctx.xplane
        .get_float_array(BUS_VOLTS)
        .await
        .ok()
        .and_then(|v| v.first().copied())
}

fn on_off_schema() -> Value {
    json!({
        "type": "object",
        "properties": { "on": { "type": "boolean", "description": "true = on/in, false = off/out" } },
        "required": ["on"]
    })
}

pub struct SetChocks;

#[async_trait]
impl Tool for SetChocks {
    fn name(&self) -> &str {
        "set_chocks"
    }
    fn description(&self) -> &str {
        "Place or remove the wheel chocks. Use for 'chocks in', 'chocks out', \
         'set the chocks', 'remove the chocks', 'pull the chocks', 'chocks away'."
    }
    fn parameters(&self) -> Value {
        on_off_schema()
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: OnArg = serde_json::from_value(args.clone())?;
        // The command is a toggle — only fire it if the state isn't already what
        // was asked. is_chocked updates immediately, so the readback is honest.
        if chocked(ctx).await != a.on {
            ctx.xplane.run_command(CMD_TOGGLE_CHOCKS).await?;
        }
        Ok(if chocked(ctx).await {
            "Roger, chocks in.".to_string()
        } else {
            "Roger, chocks out.".to_string()
        })
    }
}

pub struct SetGroundPower;

/// When WE last summoned a cart that has not yet arrived (no volts). The
/// request command is a toggle and a cart in transit generates nothing, so
/// volts alone cannot tell "no cart asked for" from "cart still driving up" —
/// and "ground power off" during the drive-up used to early-out as "already
/// disconnected" and let the cart arrive anyway. Cleared the moment the cart
/// is seen live or the request is cancelled, and EXPIRES: a cart the sim
/// dismissed on its own (engine start, taxi) with no GPU call in between
/// would latch this for the rest of the process, and "ground power off" at
/// the destination hours later then fired the toggle against nothing and
/// summoned a fresh cart while answering "disconnected" — the
/// process-wide-fact-outliving-its-context trap again. Carts drive up in
/// under a minute; a request older than the TTL is stale evidence, and doing
/// nothing is the safe read. In-process only: a cart summoned through the
/// sim's own menu is invisible here, exactly as before.
static CART_REQUESTED: std::sync::Mutex<Option<std::time::Instant>> = std::sync::Mutex::new(None);

/// How long a summoned-but-unseen cart stays "pending" before the latch is
/// stale. Generous against the sim's actual drive-up time.
const CART_REQUEST_TTL_SECS: u64 = 180;

/// The pending-cart latch is process-wide, so its tests take turns.
#[cfg(test)]
pub(crate) static GPU_TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

/// A cart summoned for the ended flight is not this flight's (the
/// new-flight door, `flight_state::forget_flight`).
pub(crate) fn forget_cart_request() {
    *CART_REQUESTED.lock().unwrap_or_else(|e| e.into_inner()) = None;
}

#[cfg(test)]
pub(crate) fn note_cart_requested_for_test() {
    *CART_REQUESTED.lock().unwrap_or_else(|e| e.into_inner()) = Some(std::time::Instant::now());
}

#[cfg(test)]
pub(crate) fn cart_pending_for_test() -> bool {
    cart_pending()
}

fn cart_pending() -> bool {
    CART_REQUESTED
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .is_some_and(|t| t.elapsed().as_secs() < CART_REQUEST_TTL_SECS)
}

#[async_trait]
impl Tool for SetGroundPower {
    fn name(&self) -> &str {
        "set_ground_power"
    }
    fn description(&self) -> &str {
        "Request or disconnect external ground power (GPU). Use for 'ground power \
         on', 'connect ground power', 'request ground power', 'ground power off', \
         'disconnect ground power', 'remove ground power'."
    }
    fn parameters(&self) -> Value {
        on_off_schema()
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: OnArg = serde_json::from_value(args.clone())?;
        // TWO steps, because they are two different things: the cart has to be
        // there, and the aircraft has to be drawing from it. The request
        // command is a TOGGLE, so it only ever fires when the cart's presence
        // is not already what we want — firing it blind sends away the cart we
        // just asked for.
        let cart = cart_present(ctx).await;
        if cart {
            // A live cart makes the pending-request latch moot either way.
            *CART_REQUESTED.lock().unwrap_or_else(|e| e.into_inner()) = None;
        }
        if a.on {
            if gpu_connected(ctx).await {
                return Ok(match bus_volts(ctx).await {
                    Some(v) => format!("Ground power's already connected, bus at {v:.0} volts."),
                    None => "Ground power's already connected.".to_string(),
                });
            }
            if !cart {
                ctx.xplane.run_command(CMD_TOGGLE_GPU).await?;
                *CART_REQUESTED.lock().unwrap_or_else(|e| e.into_inner()) =
                    Some(std::time::Instant::now());
            }
            ctx.xplane.set_int(GPU_ON, 1).await?;
            // Report the BUS, not the request: "requesting ground power" while
            // the battery sits at 21 volts is the answer that started this. A
            // cart still driving up reads as no power yet, and says so.
            return Ok(match bus_volts(ctx).await {
                Some(v) if v > 24.0 => format!("Roger, ground power on, bus at {v:.0} volts."),
                Some(v) => {
                    format!(
                        "Ground power requested — the cart's still coming, bus at {v:.0} volts."
                    )
                }
                None => "Roger, requesting ground power.".to_string(),
            });
        }
        // Disconnect: check the halves INDEPENDENTLY. The old shortcut asked
        // "connected?" (cart AND contactor), so a summoned-but-not-arrived
        // cart, or a closed contactor with no cart, answered "already
        // disconnected" and left both standing — the cart then arrived and fed
        // a bus the pilot had been told was off external power.
        let contactor = ctx.xplane.get_float(GPU_ON).await.unwrap_or(0.0) > 0.5;
        let pending = cart_pending();
        if !contactor && !cart && !pending {
            return Ok("Ground power's already disconnected.".to_string());
        }
        // Contactor first, then release the cart — the bus is never dropped by
        // a cart driving away from a still-closed contactor. A cart still in
        // transit (our own pending request) is cancelled by the same toggle.
        ctx.xplane.set_int(GPU_ON, 0).await?;
        if cart || pending {
            ctx.xplane.run_command(CMD_TOGGLE_GPU).await?;
        }
        *CART_REQUESTED.lock().unwrap_or_else(|e| e.into_inner()) = None;
        Ok("Roger, ground power disconnected.".to_string())
    }
}

pub struct Pushback;

fn pushback_continues(action: &str) -> bool {
    !matches!(action, "stop" | "halt")
}

#[derive(Deserialize)]
struct PushArg {
    /// "straight" (start), "left", "right", or "stop".
    action: String,
}

#[async_trait]
impl Tool for Pushback {
    fn name(&self) -> &str {
        "pushback"
    }
    fn description(&self) -> &str {
        "Start, steer, or stop a pushback from the gate/parking. Use for 'push \
         back', 'pushback', 'pushback straight', 'pushback left', 'pushback \
         right', 'stop pushback', 'pushback stop', 'hold the pushback'."
    }
    fn parameters(&self) -> Value {
        json!({
            "type": "object",
            "properties": {
                "action": {
                    "type": "string",
                    "enum": ["straight", "left", "right", "stop"],
                    "description": "straight = start pushing; left/right steer; stop ends it"
                }
            },
            "required": ["action"]
        })
    }
    async fn execute(&self, args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        let a: PushArg = serde_json::from_value(args.clone())?;
        // Full auto's steered push owns the tug: "skip / cancel the pushback"
        // ends the push (its loop stops the tug and sets the brakes) and the
        // mission turns the aircraft round under its own steam. The bare
        // "stop" word is routed earlier and aborts the whole mission; this is
        // the pilot naming the push, not the flight.
        if !pushback_continues(&a.action) && crate::pushback::steered() {
            crate::pushback::skip();
            return Ok("Roger, skipping the push.".to_string());
        }
        let (cmd, say) = match a.action.as_str() {
            "stop" | "halt" => (CMD_PUSH_STOP, "Roger, pushback stopped."),
            "left" => (CMD_PUSH_LEFT, "Roger, pushing back, turning left."),
            "right" => (CMD_PUSH_RIGHT, "Roger, pushing back, turning right."),
            "straight" | "start" => (CMD_PUSH_STRAIGHT, "Roger, pushing back."),
            other => {
                return Ok(format!(
                    "Say pushback straight, left, right, or stop — not '{other}'."
                ))
            }
        };
        ctx.xplane.run_command(cmd).await?;
        // Native pushback commands start a manoeuvre that continues after this
        // one-shot tool returns. Publish that ownership so the Flight tab keeps
        // CANCEL visible; STOP clears it after the command has landed.
        crate::pushback::native_push(pushback_continues(&a.action));
        Ok(say.to_string())
    }
}

pub struct RequestService;

#[async_trait]
impl Tool for RequestService {
    fn name(&self) -> &str {
        "request_service"
    }
    fn description(&self) -> &str {
        "Call ground services to the aircraft (fuel and ramp services). Use for \
         'request ground services', 'service the aircraft', 'call the trucks', \
         'refuel', 'request fuel', 'top off the tanks', 'get us fuelled'."
    }
    fn parameters(&self) -> Value {
        json!({ "type": "object", "properties": {}, "additionalProperties": false })
    }
    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        ctx.xplane.run_command(CMD_SERVICE).await?;
        Ok("Roger, requesting ground services.".to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xplane::FakeXPlane;

    fn ctx<'a>(fake: &'a FakeXPlane, cfg: &'a crate::config::Config) -> ToolCtx<'a> {
        ToolCtx {
            xplane: fake,
            config: cfg,
            voice: None,
            status: None,
            reminders: None,
        }
    }


    /// No cart yet: summon one AND close the contactor. Closing it is the half
    /// that was missing — the cart generated 28 volts into a bus that stayed on
    /// the battery at 20.7, and the tool called that "already connected".
    #[tokio::test]
    async fn connecting_summons_the_cart_and_closes_the_contactor() {
        let _g = GPU_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *CART_REQUESTED.lock().unwrap_or_else(|e| e.into_inner()) = None;
        let cfg = crate::config::Config::default();
        let fake = FakeXPlane::with_reads(&[(GPU_VOLTS, 0.0), (GPU_ON, 0.0)]);
        let out = SetGroundPower
            .execute(&serde_json::json!({"on": true}), &ctx(&fake, &cfg))
            .await
            .expect("connect");
        assert!(fake.ran(CMD_TOGGLE_GPU), "no cart present, so summon one");
        assert_eq!(fake.last_write(GPU_ON), Some(1.0), "{out}");
    }

    /// A cart already standing there must NOT be re-requested: the request is a
    /// toggle, so firing it blind sends away the cart we are trying to use.
    /// Only the contactor moves.
    #[tokio::test]
    async fn a_cart_already_there_is_not_toggled_away() {
        let _g = GPU_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *CART_REQUESTED.lock().unwrap_or_else(|e| e.into_inner()) = None;
        let cfg = crate::config::Config::default();
        let fake = FakeXPlane::with_reads(&[(GPU_VOLTS, 28.0), (GPU_ON, 0.0)]);
        SetGroundPower
            .execute(&serde_json::json!({"on": true}), &ctx(&fake, &cfg))
            .await
            .expect("connect");
        assert!(
            !fake.ran(CMD_TOGGLE_GPU),
            "the cart is already here; toggling would dismiss it"
        );
        assert_eq!(fake.last_write(GPU_ON), Some(1.0));
    }

    /// "Already connected" needs BOTH halves. A live cart with an open
    /// contactor is not power, and saying so is what left a pilot with a flat
    /// battery and a cheerful readback.
    #[tokio::test]
    async fn a_live_cart_alone_is_not_connected() {
        let cfg = crate::config::Config::default();
        let live_cart_open_contactor = FakeXPlane::with_reads(&[(GPU_VOLTS, 28.0), (GPU_ON, 0.0)]);
        assert!(
            !gpu_connected(&ctx(&live_cart_open_contactor, &cfg)).await,
            "cart present, contactor open: the bus has nothing"
        );
        let closed_but_no_cart = FakeXPlane::with_reads(&[(GPU_VOLTS, 0.0), (GPU_ON, 1.0)]);
        assert!(
            !gpu_connected(&ctx(&closed_but_no_cart, &cfg)).await,
            "contactor closed onto nothing"
        );
        let both = FakeXPlane::with_reads(&[(GPU_VOLTS, 28.0), (GPU_ON, 1.0)]);
        assert!(gpu_connected(&ctx(&both, &cfg)).await);
    }

    /// Disconnecting opens the contactor first, then releases the cart — in
    /// that order, so the bus is never dropped by a cart driving away from a
    /// still-closed contactor.
    #[tokio::test]
    async fn disconnecting_opens_the_contactor_then_releases_the_cart() {
        let _g = GPU_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *CART_REQUESTED.lock().unwrap_or_else(|e| e.into_inner()) = None;
        let cfg = crate::config::Config::default();
        let fake = FakeXPlane::with_reads(&[(GPU_VOLTS, 28.0), (GPU_ON, 1.0)]);
        SetGroundPower
            .execute(&serde_json::json!({"on": false}), &ctx(&fake, &cfg))
            .await
            .expect("disconnect");
        assert_eq!(fake.last_write(GPU_ON), Some(0.0));
        assert!(fake.ran(CMD_TOGGLE_GPU), "send the cart away");
    }

    /// A cart still driving up (our own fresh request, no volts yet) IS
    /// cancelled by "ground power off" — the case the latch exists for.
    #[tokio::test]
    async fn a_cart_still_driving_up_is_cancelled_by_disconnect() {
        let _g = GPU_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *CART_REQUESTED.lock().unwrap_or_else(|e| e.into_inner()) = Some(std::time::Instant::now());
        let cfg = crate::config::Config::default();
        let fake = FakeXPlane::with_reads(&[(GPU_VOLTS, 0.0), (GPU_ON, 0.0)]);
        let out = SetGroundPower
            .execute(&serde_json::json!({"on": false}), &ctx(&fake, &cfg))
            .await
            .expect("disconnect");
        assert!(
            fake.ran(CMD_TOGGLE_GPU),
            "the in-transit cart must be cancelled: {out}"
        );
        assert!(
            CART_REQUESTED.lock().unwrap().is_none(),
            "the latch dies with the cancellation"
        );
    }

    /// A request the sim outlived (the cart came and went with no GPU call in
    /// between, or never came at all) must NOT fire the toggle at the
    /// destination hours later — that SUMMONS a cart while saying
    /// "disconnected". Past the TTL the latch is stale evidence, not a fact.
    #[tokio::test]
    async fn a_stale_cart_request_never_summons_a_cart() {
        let _g = GPU_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        *CART_REQUESTED.lock().unwrap_or_else(|e| e.into_inner()) = Some(
            std::time::Instant::now() - std::time::Duration::from_secs(CART_REQUEST_TTL_SECS + 60),
        );
        let cfg = crate::config::Config::default();
        let fake = FakeXPlane::with_reads(&[(GPU_VOLTS, 0.0), (GPU_ON, 0.0)]);
        let out = SetGroundPower
            .execute(&serde_json::json!({"on": false}), &ctx(&fake, &cfg))
            .await
            .expect("disconnect");
        assert!(
            !fake.ran(CMD_TOGGLE_GPU),
            "a stale latch fired the toggle against nothing: {out}"
        );
        assert!(out.contains("already disconnected"), "{out}");
        *CART_REQUESTED.lock().unwrap_or_else(|e| e.into_inner()) = None;
    }

    #[test]
    fn native_pushback_stays_cancellable_until_stop() {
        for action in ["straight", "start", "left", "right"] {
            assert!(pushback_continues(action), "{action}");
        }
        for action in ["stop", "halt"] {
            assert!(!pushback_continues(action), "{action}");
        }
    }
}
