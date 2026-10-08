//! The world's scheduler surface: the ticks the crew's monitor drives
//! (`src/world_tick.rs`) and the types they trade. Part of the one crew/world
//! seam with [`crate::party_line`]: the station itself is crate-private, so
//! this page and the party line are everything the crew can reach of it.
//!
//! A tick reads the sim through the read-only `SimView` and RETURNS what the
//! world says; the crew speaks it. A tick may update world state (the
//! station's own watches), never the crew's.

/// The runway watch: taxi compliance, the go-around judgement, the landing
/// hold's release, the traffic on our strip.
pub use crate::station::{
    compliance_tick_facing, go_around_tick, inbound_traffic_for_strip, landing_hold_tick,
    on_active_runway_facing,
};

/// The ATIS broadcast on COM1.
pub use crate::station::{atis_due, atis_retune, atis_tick, AtisWx};

/// The tower's after-takeoff handoff, the radar unit's squawk watch, the
/// enroute region and vectors to final.
pub use crate::station::{
    handoff_tick, region_tick, squawk_tick, squawk_watch_active, vector_tick,
};

/// The takeoff hold, the wake book and the party-line chatter.
pub use crate::station::{
    chatter_tick, player_blocked_strip, takeoff_hold_tick_from, void_takeoff_hold_by_roll,
    wake_note_targets,
};

/// Our own AI traffic (stage D): where each traffic flight is, by its
/// stable number; the calls it keys, as text, through an inbox the command
/// loop drains (`party_line::Station::answer_traffic` answers each).
pub use crate::station::{
    note_traffic_state, post_traffic_call, take_traffic_calls, TrafficCall, TrafficReply,
    TrafficTarget,
};

/// Traffic: the TCAS read, the near-miss and the traffic advisory.
pub use crate::station::{
    advisory_line, is_near_miss, near_miss_line, nearest_traffic_factor, read_tcas_targets,
    traffic_worth_recalling, TcasTarget, TrafficFactor,
};
