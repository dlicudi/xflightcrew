//! `go_around`: the pilot orders the approach flown round (#77, #102).
//!
//! Only a loop flying an approach can go around: autoland, or the circuit on
//! base or final. The order goes on the crew's own channel
//! ([`crate::go_around`]), which those loops watch beside the tower's, so a
//! pilot's go-around is flown exactly as the tower's is. The readback is the
//! loop's answer, never the request: "Going around." only once a loop is
//! flying it. Gated: nothing flying an approach, or the wheels down, is a
//! spoken refusal and no order.

use anyhow::Result;
use async_trait::async_trait;
use serde_json::{json, Value};

use super::{Tool, ToolCtx};

pub struct GoAround;

#[async_trait]
impl Tool for GoAround {
    fn name(&self) -> &str {
        "go_around"
    }
    fn description(&self) -> &str {
        "Go around: the approach autoland or the circuit is flying is abandoned \
         and flown as a missed approach (full power, climb, clean up, the pattern \
         round). Use for 'go around', 'going around', 'take it around', 'missed \
         approach'. Refused with nothing flying an approach, or on the runway."
    }
    fn parameters(&self) -> Value {
        json!({"type": "object", "properties": {}})
    }
    async fn execute(&self, _args: &Value, ctx: &ToolCtx<'_>) -> Result<String> {
        use crate::flight_state::Owner;
        let autoland = Owner::Autoland.active();
        let circuit = Owner::Circuit.active();
        // An unread ground flag is not a landing: the loop's own latch
        // refuses an order on the wheels anyway.
        let on_ground = ctx
            .xplane
            .get_float(crate::datarefs::ON_GROUND_ANY)
            .await
            .is_ok_and(|v| v > 0.5);
        if let Some(why) = crate::go_around::refusal(autoland, circuit, on_ground) {
            crate::blackbox::record(
                "go_around",
                json!({"event": "refused", "why": why, "autoland": autoland,
                       "circuit": circuit, "on_ground": on_ground}),
            );
            return Ok(why.to_string());
        }
        let n = crate::go_around::order();
        let a = crate::go_around::wait_answer(n).await;
        crate::blackbox::record(
            "go_around",
            json!({"event": "answered", "n": n, "answer": a.map(|a| format!("{a:?}"))}),
        );
        Ok(crate::go_around::readback(a).to_string())
    }
}
