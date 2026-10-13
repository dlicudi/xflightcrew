//! The packed sink (stage D increment 10, the app half): our own traffic to
//! the plugin, which draws it. The roster goes out on change, the frames at
//! most every 250 ms, both packed by `traffic::wire` (the layout the
//! plugin's `traffic_wire.h` decodes) into `xcopilot/traffic/roster` and
//! `xcopilot/traffic/frames`, and `xcopilot/traffic/enable` says whether
//! our traffic is on.
//!
//! The monitor never waits on it: [`TrafficSink::publish`] hands the latest
//! packet to a detached writer task over a watch channel (latest only: a
//! frame the writer had no time for is replaced, never queued), which makes
//! at most one round of writes per [`WRITE_EVERY`]. No per-aircraft requests,
//! no reads.
//!
//! THE DRAW CULL (multi-field traffic, stage 1, 2026-10-13): the plugin's
//! cost is per DRAWN aircraft (an XPMP2 plane each, a terrain probe per
//! ground one per frame), the fleet's is Rust-side and small. So only the
//! aircraft within [`DRAW_RANGE_NM`] of us are sent ([`DrawCull`]), at most
//! [`DRAW_MAX`], the rest simulated but not drawn; TCAS sees what is drawn.
//! The cull is a roster change (the plugin destroys the plane), never
//! `FLAG_HIDDEN` (a hidden plane still costs its update).

use std::collections::BTreeSet;
use std::time::Duration;

use traffic::wire::{self, RosterEntry};

const ROSTER: &str = "xcopilot/traffic/roster";
const FRAMES: &str = "xcopilot/traffic/frames";
const ENABLE: &str = "xcopilot/traffic/enable";

/// One round of writes at most this often (4 Hz).
pub(crate) const WRITE_EVERY: Duration = Duration::from_millis(250);

/// After a write fails (a plugin without the traffic datarefs, a sim gone)
/// the writer waits this long before it tries again.
const BACKOFF: Duration = Duration::from_secs(10);

/// The enable flag and the roster are written again this often even when
/// nothing changed: the plugin can lose them behind the writer's back (a
/// plugin reload starts at enable 0; a field change withdrew while the next
/// link loaded and the plugin sat disabled with frames still arriving, LEMG
/// 2026-10-08). Two small writes per period.
const REASSERT_EVERY: Duration = Duration::from_secs(5);

/// Aircraft within this of us are drawn, nm: the whole simulated area (the
/// pilot's ask, 2026-10-13, first 40 nm for TCAS range, then all of the
/// 100 nm world; at LEMG with LXGB beside it the frame rate held, see the
/// history). The cap below still bounds the cost.
pub(crate) const DRAW_RANGE_NM: f64 = 100.0;

/// A drawn aircraft stays drawn until this far, nm: one flying along the
/// ring does not flicker in and out (the plugin destroys and recreates
/// the plane on every roster change).
pub(crate) const DRAW_OUT_NM: f64 = 110.0;

/// At most this many aircraft drawn, nearest first (the design's budget:
/// LEMG's 10:00 start draws about 40 moving plus up to 80 parked,
/// `schedule::PARKED_MAX`). Frame cost past 60 drawn is unmeasured: raise
/// it only on `api_load` and the fps dataref.
pub(crate) const DRAW_MAX: usize = 120;

/// A parked (dormant) aircraft farther than this, nm, is the first to go
/// when [`DRAW_MAX`] binds: a dark aircraft on a stand 15 nm away is
/// invisible, a moving one is TCAS traffic.
pub(crate) const PARKED_FAR_NM: f64 = 15.0;

/// Which simulated aircraft are drawn (module docs): within
/// [`DRAW_RANGE_NM`] of us, kept to [`DRAW_OUT_NM`] once drawn, at most
/// [`DRAW_MAX`] (far parked ones dropped first, then the farthest). Pure
/// but for its memory of who was drawn.
#[derive(Debug, Default)]
pub(crate) struct DrawCull {
    drawn: BTreeSet<u32>,
}

impl DrawCull {
    /// The aircraft to draw of `fleet`, seen from `own` (lat, lon), in id
    /// order (a stable roster: the same set is the same roster). With no
    /// position of ours, the first [`DRAW_MAX`] by id.
    pub(crate) fn select<'a>(
        &mut self,
        own: Option<(f64, f64)>,
        fleet: &[&'a traffic::TrafficAircraft],
    ) -> Vec<&'a traffic::TrafficAircraft> {
        let mut near: Vec<(bool, f64, &'a traffic::TrafficAircraft)> = fleet
            .iter()
            .filter_map(|a| {
                let d = own.map_or(0.0, |o| traffic::places::dist_nm(o, a.kin.pos()));
                let reach = if self.drawn.contains(&a.id) {
                    DRAW_OUT_NM
                } else {
                    DRAW_RANGE_NM
                };
                (d <= reach).then_some((a.dormant() && d > PARKED_FAR_NM, d, *a))
            })
            .collect();
        near.sort_by(|x, y| {
            x.0.cmp(&y.0)
                .then(x.1.total_cmp(&y.1))
                .then(x.2.id.cmp(&y.2.id))
        });
        near.truncate(DRAW_MAX);
        let mut out: Vec<&'a traffic::TrafficAircraft> = near.into_iter().map(|n| n.2).collect();
        out.sort_by_key(|a| a.id);
        self.drawn = out.iter().map(|a| a.id).collect();
        out
    }

    /// Nobody drawn (our traffic withdrawn).
    pub(crate) fn clear(&mut self) {
        self.drawn.clear();
    }
}

/// What the writer is to make true, the latest only.
#[derive(Debug, Clone, Default, PartialEq)]
pub(crate) struct Packet {
    /// Our traffic is on (`xcopilot/traffic/enable`).
    pub(crate) enabled: bool,
    /// The roster and its sequence: written when the sequence moves.
    pub(crate) roster: Option<(u32, Vec<u8>)>,
    /// The latest frames.
    pub(crate) frames: Option<Vec<u8>>,
}

/// The monitor's half: packs the fleet and hands it to the writer.
pub(crate) struct TrafficSink {
    tx: tokio::sync::watch::Sender<Packet>,
    roster: Vec<RosterEntry>,
    roster_seq: u32,
    frame_seq: u32,
}

impl TrafficSink {
    /// Start the writer on the current runtime, against the sim at
    /// `base_url`. `None` outside a runtime (no writer, nothing sent).
    pub(crate) fn start(base_url: String) -> Option<TrafficSink> {
        let handle = tokio::runtime::Handle::try_current().ok()?;
        let (tx, rx) = tokio::sync::watch::channel(Packet::default());
        handle.spawn(write_loop(base_url, rx));
        Some(TrafficSink::with_sender(tx))
    }

    fn with_sender(tx: tokio::sync::watch::Sender<Packet>) -> TrafficSink {
        TrafficSink {
            tx,
            roster: Vec::new(),
            roster_seq: 0,
            frame_seq: 0,
        }
    }

    /// The aircraft drawn now (after [`DrawCull`]), taken at the sim's
    /// running time `sim_time_s`: a new roster when who is drawn changed,
    /// the frames every time.
    pub(crate) fn publish(&mut self, fleet: &[&traffic::TrafficAircraft], sim_time_s: f64) {
        let roster: Vec<RosterEntry> = fleet.iter().map(|a| wire::roster_entry(a)).collect();
        if roster != self.roster {
            self.roster = roster;
            self.roster_seq = self.roster_seq.wrapping_add(1);
        }
        self.frame_seq = self.frame_seq.wrapping_add(1);
        let frames: Vec<wire::FrameEntry> = fleet.iter().map(|a| wire::frame_entry(a)).collect();
        let packet = Packet {
            enabled: true,
            roster: Some((
                self.roster_seq,
                wire::encode_roster(self.roster_seq, &self.roster),
            )),
            frames: Some(wire::encode_frames(self.frame_seq, sim_time_s, &frames)),
        };
        self.tx.send_replace(packet);
    }

    /// Our traffic is off: an empty roster, no frames, disabled.
    pub(crate) fn withdraw(&mut self) {
        if !self.roster.is_empty() {
            self.roster.clear();
            self.roster_seq = self.roster_seq.wrapping_add(1);
        }
        self.tx.send_replace(Packet {
            enabled: false,
            roster: Some((self.roster_seq, wire::encode_roster(self.roster_seq, &[]))),
            frames: None,
        });
    }
}

/// The writer: wait for a packet, make the sim agree with it (enable, then
/// the roster if its sequence moved, then the frames), then rest out the
/// rest of [`WRITE_EVERY`]. A failed write is tried again with the next
/// packet after [`BACKOFF`].
async fn write_loop(base_url: String, mut rx: tokio::sync::watch::Receiver<Packet>) {
    let xp = crate::xplane::XPlaneClient::new(base_url);
    let mut wrote = Written::default();
    while rx.changed().await.is_ok() {
        let packet = rx.borrow_and_update().clone();
        let ok = wrote.apply(&packet, &xp).await;
        tokio::time::sleep(if ok { WRITE_EVERY } else { BACKOFF }).await;
    }
}

/// What the writer last made true.
#[derive(Default)]
struct Written {
    enabled: Option<bool>,
    roster_seq: Option<u32>,
    /// When enable and the roster were last written whatever the cache said.
    reasserted: Option<std::time::Instant>,
}

impl Written {
    /// The writes `packet` still needs, in order. False on a failed write.
    async fn apply(&mut self, packet: &Packet, xp: &crate::xplane::XPlaneClient) -> bool {
        if self.reasserted.is_none_or(|t| t.elapsed() >= REASSERT_EVERY) {
            self.enabled = None;
            self.roster_seq = None;
            self.reasserted = Some(std::time::Instant::now());
        }
        if self.enabled != Some(packet.enabled) {
            if xp.set_int(ENABLE, i64::from(packet.enabled)).await.is_err() {
                return false;
            }
            self.enabled = Some(packet.enabled);
        }
        if let Some((seq, bytes)) = &packet.roster {
            if self.roster_seq != Some(*seq) {
                if xp.set_data(ROSTER, bytes).await.is_err() {
                    return false;
                }
                self.roster_seq = Some(*seq);
            }
        }
        if let Some(bytes) = &packet.frames {
            if xp.set_data(FRAMES, bytes).await.is_err() {
                return false;
            }
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fleet() -> (traffic::FieldModel, Vec<traffic::TrafficAircraft>) {
        let apt = format!(
            "{}/crates/traffic/fixtures/xtst.dat",
            env!("CARGO_MANIFEST_DIR")
        );
        let field = traffic::FieldModel::load(&apt, "XTST").unwrap().unwrap();
        let ac = |id: u32, stand: usize| {
            traffic::TrafficAircraft::at_stand(
                id,
                &format!("G-ABC{id}"),
                "Cessna",
                "C172",
                traffic::Plan::Circuit {
                    runway: "09".into(),
                    laps: 1,
                    left: true,
                },
                &field,
                &field.net.stands[stand].name,
                u64::from(id),
            )
            .unwrap()
        };
        let acs = vec![ac(1, 0), ac(2, 1)];
        (field, acs)
    }

    /// LATEST ONLY, ROSTER ON CHANGE: every publish replaces the packet (a
    /// frame the writer had no time for is never queued), the roster's
    /// sequence moves only when who is flying changes, the frames' every
    /// time, and both decode as the plugin decodes them.
    #[test]
    fn the_sink_hands_the_writer_the_latest_packet() {
        let (tx, rx) = tokio::sync::watch::channel(Packet::default());
        let mut sink = TrafficSink::with_sender(tx);
        let (_field, mut acs) = fleet();
        sink.publish(&acs.iter().collect::<Vec<_>>(), 100.0);
        sink.publish(&acs.iter().collect::<Vec<_>>(), 100.25);
        let p = rx.borrow().clone();
        assert!(p.enabled);
        let (seq, roster) = p.roster.clone().unwrap();
        assert_eq!(seq, 1, "the same fleet is the same roster");
        let (rseq, entries) = wire::decode_roster(&roster).unwrap();
        assert_eq!((rseq, entries.len()), (1, 2));
        assert_eq!(entries[0].callsign, "G-ABC1");
        let (fseq, t, frames) = wire::decode_frames(p.frames.as_ref().unwrap()).unwrap();
        assert_eq!(
            (fseq, t, frames.len()),
            (2, 100.25, 2),
            "the latest frame only"
        );
        acs.pop();
        sink.publish(&acs.iter().collect::<Vec<_>>(), 100.5);
        assert_eq!(
            rx.borrow().roster.as_ref().unwrap().0,
            2,
            "a despawn is a new roster"
        );
        sink.withdraw();
        let p = rx.borrow().clone();
        assert!(!p.enabled && p.frames.is_none());
        let (seq, bytes) = p.roster.unwrap();
        assert_eq!(seq, 3);
        assert!(
            wire::decode_roster(&bytes).unwrap().1.is_empty(),
            "nobody drawn"
        );
    }

    /// Sixty aircraft fit the plugin's 16 KB in both buffers, as the spec
    /// sized it (about 3.3 KB base64 of frames).
    #[test]
    fn sixty_aircraft_fit_the_wire() {
        let (_field, one) = fleet();
        let many: Vec<traffic::TrafficAircraft> = (0..60)
            .map(|i| {
                let mut a = one[0].clone();
                a.id = i + 1;
                a
            })
            .collect();
        let (tx, rx) = tokio::sync::watch::channel(Packet::default());
        let mut sink = TrafficSink::with_sender(tx);
        sink.publish(&many.iter().collect::<Vec<_>>(), 1.0);
        let p = rx.borrow().clone();
        let frames = p.frames.unwrap();
        assert_eq!(frames.len(), wire::FRAME_HEADER + 60 * wire::FRAME_ENTRY);
        assert!(frames.len() * 4 / 3 < 3_400, "{} bytes", frames.len());
        assert!(p.roster.unwrap().1.len() <= wire::BUF_BYTES);
    }

    /// `one` as aircraft `id`, `nm` north of `at`: parked (dormant) or
    /// flying a circuit.
    fn north_of(
        one: &traffic::TrafficAircraft,
        id: u32,
        at: (f64, f64),
        nm: f64,
        parked: bool,
    ) -> traffic::TrafficAircraft {
        let mut a = one.clone();
        a.id = id;
        a.kin.lat = at.0 + nm / 60.0;
        a.kin.lon = at.1;
        a.plan = if parked {
            traffic::Plan::Parked
        } else {
            traffic::Plan::Circuit {
                runway: "09".into(),
                laps: 1,
                left: true,
            }
        };
        a
    }

    fn ids(v: &[&traffic::TrafficAircraft]) -> Vec<u32> {
        v.iter().map(|a| a.id).collect()
    }

    /// THE RING (multi-field traffic stage 1, 2026-10-13): an aircraft is
    /// drawn from inside 40 nm, stays drawn out to 44 nm (one flying along
    /// the ring does not flicker), goes past it, and is not drawn again
    /// before it is back inside 40.
    #[test]
    fn the_draw_ring_has_hysteresis() {
        let (_field, one) = fleet();
        let own = (36.0, -4.5);
        let mut cull = DrawCull::default();
        let mut at = |nm: f64| {
            let a = [north_of(&one[0], 7, own, nm, false)];
            ids(&cull.select(Some(own), &a.iter().collect::<Vec<_>>()))
        };
        let mid = (DRAW_RANGE_NM + DRAW_OUT_NM) / 2.0;
        assert!(at(mid).is_empty(), "between the rings coming in");
        assert_eq!(at(DRAW_RANGE_NM - 1.0), [7]);
        assert_eq!(at(DRAW_OUT_NM - 0.5), [7], "kept on the ring");
        assert!(at(DRAW_OUT_NM + 0.5).is_empty(), "gone past the outer ring");
        assert!(at(mid).is_empty(), "not back before the inner ring");
    }

    /// THE CAP (stage 1): past [`DRAW_MAX`] the parked aircraft beyond
    /// 15 nm go first, then the farthest; the roster is in id order
    /// whatever the distances.
    #[test]
    fn the_draw_cap_keeps_the_nearest_and_drops_far_parked_first() {
        let (_field, one) = fleet();
        let own = (36.0, -4.5);
        let mut acs = Vec::new();
        // 100 moving within 30 nm, 30 parked at 20 nm, 10 parked at 1 nm.
        for i in 0..100u32 {
            let nm = 30.0 * f64::from(i) / 100.0;
            acs.push(north_of(&one[0], 1000 - i, own, nm, false));
        }
        for i in 0..30u32 {
            acs.push(north_of(&one[0], 2000 + i, own, 20.0, true));
        }
        for i in 0..10u32 {
            acs.push(north_of(&one[0], 3000 + i, own, 1.0, true));
        }
        let mut cull = DrawCull::default();
        let drawn = cull.select(Some(own), &acs.iter().collect::<Vec<_>>());
        assert_eq!(drawn.len(), DRAW_MAX);
        let far_parked = drawn
            .iter()
            .filter(|a| (2000..3000).contains(&a.id))
            .count();
        assert_eq!(far_parked, DRAW_MAX - 110, "far parked fill what is left");
        assert_eq!(
            drawn.iter().filter(|a| a.id >= 3000).count(),
            10,
            "near parked stay"
        );
        assert!(drawn.windows(2).all(|w| w[0].id < w[1].id), "id order");
        // Over the cap with moving aircraft alone: the nearest win.
        let moving: Vec<traffic::TrafficAircraft> = (0..130u32)
            .map(|i| north_of(&one[0], i + 1, own, 0.2 * f64::from(i), false))
            .collect();
        let drawn = DrawCull::default().select(Some(own), &moving.iter().collect::<Vec<_>>());
        assert_eq!(drawn.len(), DRAW_MAX);
        assert!(
            drawn.iter().all(|a| a.id as usize <= DRAW_MAX),
            "the nearest 120"
        );
    }
}
