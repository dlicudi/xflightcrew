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

    /// The fleet now, taken at the sim's running time `sim_time_s`: a new
    /// roster when who is flying changed, the frames every time.
    pub(crate) fn publish(&mut self, fleet: &[traffic::TrafficAircraft], sim_time_s: f64) {
        let roster: Vec<RosterEntry> = fleet.iter().map(wire::roster_entry).collect();
        if roster != self.roster {
            self.roster = roster;
            self.roster_seq = self.roster_seq.wrapping_add(1);
        }
        self.frame_seq = self.frame_seq.wrapping_add(1);
        let frames: Vec<wire::FrameEntry> = fleet.iter().map(wire::frame_entry).collect();
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
}

impl Written {
    /// The writes `packet` still needs, in order. False on a failed write.
    async fn apply(&mut self, packet: &Packet, xp: &crate::xplane::XPlaneClient) -> bool {
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
        sink.publish(&acs, 100.0);
        sink.publish(&acs, 100.25);
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
        sink.publish(&acs, 100.5);
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
        sink.publish(&many, 1.0);
        let p = rx.borrow().clone();
        let frames = p.frames.unwrap();
        assert_eq!(frames.len(), wire::FRAME_HEADER + 60 * wire::FRAME_ENTRY);
        assert!(frames.len() * 4 / 3 < 3_400, "{} bytes", frames.len());
        assert!(p.roster.unwrap().1.len() <= wire::BUF_BYTES);
    }
}
