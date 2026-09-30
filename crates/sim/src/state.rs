//! Everything a control tick reads, as ONE frame saw it, in ONE request.
//!
//! Every Web API request is served inside X-Plane's own frame, so a tick
//! costs one frame per ROUND of requests it waits on, however many requests
//! the round holds. Measured on the iMac at 20 fps on 2026-09-30 (a 50 ms
//! frame): one GET 46 ms, two to eight concurrent GETs 44 to 51 ms, six
//! sequential GETs 312 ms. The loops ticked at their sleep plus six to
//! eight rounds: autoland's `power` dt median 0.56 s on Final (sleep 200 ms)
//! and 0.39 s in the Flare (100 ms), the rollout 0.48 s (300 ms), the takeoff
//! roll 0.34 s (150 ms), where the faster MacBook ran about 0.3 s, and the
//! laws tuned there rang or lagged here (#185, the castoring-nose patches).
//!
//! The plugin fills `xcopilot/state` (`plugin/src/drefs.cpp`) AT THE READ:
//! the Web API calls its accessor on the sim's own thread, so every field is
//! the same frame's, a pause included (a paused sim runs no flight loop, so
//! a per-frame copy would hold the pause flag at 0). [`read`] takes it in one
//! request, joined with any reads it cannot carry (a profile's own throttle
//! dataref), so a tick's reads are one round. An older `.xpl` lacks it: then
//! the fields are read separately, all in one `join`, which is still one
//! round, and the reading says so ([`Source::Split`]).

use anyhow::{anyhow, Result};

use crate::position::{self, Position};
use crate::xplane::XPlane;

/// The plugin's state frame: native little-endian doubles.
pub const STATE: &str = "xcopilot/state";
/// The layout [`parse`] understands (element 0 of the buffer).
pub const LAYOUT_VERSION: f64 = 1.0;
/// The position prefix: the same ten doubles as `xcopilot/position`.
const POSITION_FIELDS: usize = 10;

/// A field of the frame past the position prefix, by its slot in layout 1.
/// The plugin writes them in this order (`plugin/src/drefs.cpp`
/// `kStateSources`); a test pins the two lists together.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum F {
    /// Indicated airspeed, kt (`position/indicated_airspeed`).
    Ias = 10,
    /// True track, deg (`position/hpath`).
    Hpath,
    /// True bank, deg (`position/true_phi`).
    Phi,
    /// True pitch, deg (`position/true_theta`).
    Theta,
    /// Pitch rate, deg/s (`position/Q`).
    Q,
    /// Yaw rate, deg/s (`position/R`).
    R,
    /// Any wheel on the ground (`failures/onground_any`).
    OnGround,
    /// Magnetic variation, deg (`position/magnetic_variation`).
    MagVar,
    /// The pilot's VSI, fpm (`vvi_fpm_pilot`).
    Vvi,
    Paused,
    Replay,
    /// Autopilot servos engaged.
    Servos,
    HdgStatus,
    VsStatus,
    /// The pilot's heading bug, deg magnetic.
    Bug,
    Hdef,
    Vdef,
    LocValid,
    GsValid,
    ApprStatus,
    GsStatus,
    /// The plugin's yield latches.
    SteerYield,
    PitchYield,
    RollYield,
    BrakeYield,
    /// A tug on the bar (`overflow/pushback_attached`).
    Tug,
    /// Engine 1's torque, Nm (`ENGN_TRQ[0]`).
    Trq0,
    /// `throttle_ratio_all`, the stock lever (a profile's own throttle is
    /// read beside the frame).
    ThrottleAll,
}

/// Every field, in slot order.
pub const FIELDS: &[F] = &[
    F::Ias,
    F::Hpath,
    F::Phi,
    F::Theta,
    F::Q,
    F::R,
    F::OnGround,
    F::MagVar,
    F::Vvi,
    F::Paused,
    F::Replay,
    F::Servos,
    F::HdgStatus,
    F::VsStatus,
    F::Bug,
    F::Hdef,
    F::Vdef,
    F::LocValid,
    F::GsValid,
    F::ApprStatus,
    F::GsStatus,
    F::SteerYield,
    F::PitchYield,
    F::RollYield,
    F::BrakeYield,
    F::Tug,
    F::Trq0,
    F::ThrottleAll,
];

/// Doubles in layout 1.
pub const LEN: usize = POSITION_FIELDS + 28;

impl F {
    /// The dataref the field copies (the name a split read asks for).
    pub const fn name(self) -> &'static str {
        match self {
            F::Ias => "sim/flightmodel/position/indicated_airspeed",
            F::Hpath => "sim/flightmodel/position/hpath",
            F::Phi => "sim/flightmodel/position/true_phi",
            F::Theta => "sim/flightmodel/position/true_theta",
            F::Q => "sim/flightmodel/position/Q",
            F::R => "sim/flightmodel/position/R",
            F::OnGround => crate::datarefs::ON_GROUND_ANY,
            F::MagVar => "sim/flightmodel/position/magnetic_variation",
            F::Vvi => crate::datarefs::VVI_FPM,
            F::Paused => crate::datarefs::PAUSED,
            F::Replay => crate::datarefs::IS_IN_REPLAY,
            F::Servos => crate::datarefs::SERVOS_ON,
            F::HdgStatus => crate::datarefs::HEADING_STATUS,
            F::VsStatus => "sim/cockpit2/autopilot/vvi_status",
            F::Bug => "sim/cockpit2/autopilot/heading_dial_deg_mag_pilot",
            F::Hdef => "sim/cockpit2/radios/indicators/nav1_hdef_dots_pilot",
            F::Vdef => "sim/cockpit2/radios/indicators/nav1_vdef_dots_pilot",
            F::LocValid => "sim/cockpit2/radios/indicators/nav1_display_horizontal",
            F::GsValid => "sim/cockpit2/radios/indicators/nav1_display_vertical",
            F::ApprStatus => crate::datarefs::APPROACH_STATUS,
            F::GsStatus => "sim/cockpit2/autopilot/glideslope_status",
            F::SteerYield => "xcopilot/steer_yield",
            F::PitchYield => "xcopilot/pitch_yield",
            F::RollYield => "xcopilot/roll_yield",
            F::BrakeYield => "xcopilot/brake_yield",
            F::Tug => "sim/aircraft/overflow/pushback_attached",
            F::Trq0 => "sim/flightmodel/engine/ENGN_TRQ",
            F::ThrottleAll => "sim/cockpit2/engine/actuators/throttle_ratio_all",
        }
    }

    const fn slot(self) -> usize {
        self as usize
    }

    /// The field that copies `name`, if the frame carries it (a caller with
    /// a profile's own dataref name asks whether the frame has it).
    pub fn for_name(name: &str) -> Option<F> {
        FIELDS.iter().copied().find(|f| f.name() == name)
    }
}

/// Which read a [`State`] came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Source {
    /// The plugin's frame, one request.
    Plugin,
    /// Separate requests in one round (an older plugin).
    Split,
}

impl Source {
    /// The blackbox word.
    pub fn as_str(self) -> &'static str {
        match self {
            Source::Plugin => "plugin",
            Source::Split => "split",
        }
    }
}

/// One tick's reading. A field not read, or read as no number, is `NaN`
/// and every accessor answers it as a failed read: a failed read is no
/// measurement.
#[derive(Debug, Clone)]
pub struct State {
    /// Where the aircraft is (`Err` when unread: the caller's `lat_lon`
    /// and `field` helpers take it as they took `position::read`).
    pos: std::result::Result<Position, String>,
    vals: [f64; LEN],
    pub source: Source,
}

/// Two readings are the same when every field is: an unread field (NaN)
/// equals an unread field, which `f64`'s own equality would deny.
impl PartialEq for State {
    fn eq(&self, other: &Self) -> bool {
        self.pos == other.pos
            && self.source == other.source
            && self
                .vals
                .iter()
                .zip(other.vals.iter())
                .all(|(a, b)| a.to_bits() == b.to_bits() || (a.is_nan() && b.is_nan()))
    }
}

impl State {
    /// The position as `position::read` returned it.
    pub fn position(&self) -> Result<Position> {
        self.pos
            .clone()
            .map_err(|e| anyhow!("position unread: {e}"))
    }

    /// A field as a read result.
    pub fn get(&self, f: F) -> Result<f64> {
        let v = self.vals[f.slot()];
        if v.is_finite() {
            Ok(v)
        } else {
            Err(anyhow!("{} unread", f.name()))
        }
    }

    /// A field, `None` when unread.
    pub fn opt(&self, f: F) -> Option<f64> {
        self.get(f).ok()
    }

    /// Paused or in a replay (`xplane::sim_frozen`'s rule: an unread flag
    /// is live, fail open).
    pub fn frozen(&self) -> bool {
        self.opt(F::Paused).is_some_and(|v| v > 0.5) || self.opt(F::Replay).is_some_and(|v| v > 0.5)
    }

    /// The sim frame the plugin filled (`None` on a split read).
    pub fn frame(&self) -> Option<u64> {
        self.pos.as_ref().ok().and_then(|p| p.frame)
    }

    /// A reading from a position and field values, for tests and replays.
    pub fn from_parts(pos: Result<Position>, fields: &[(F, f64)], source: Source) -> Self {
        let mut vals = [f64::NAN; LEN];
        if let Ok(p) = &pos {
            fill_position(&mut vals, p);
        }
        for (f, v) in fields {
            vals[f.slot()] = *v;
        }
        Self {
            pos: pos.map_err(|e| format!("{e:#}")),
            vals,
            source,
        }
    }
}

fn fill_position(vals: &mut [f64; LEN], p: &Position) {
    vals[0] = LAYOUT_VERSION;
    vals[1] = p.frame.unwrap_or(0) as f64;
    vals[2] = p.sim_time_s.unwrap_or(0.0);
    vals[3] = p.lat;
    vals[4] = p.lon;
    vals[5] = p.elev_m;
    vals[6] = p.psi_true_deg;
    vals[7] = p.gs_ms;
    vals[8] = p.vs_fpm;
    vals[9] = p.agl_m;
}

/// The plugin buffer as a [`State`], `None` unless it is a whole layout-1
/// buffer whose position prefix parses (`position::parse`'s rules: version
/// 1, every prefix field a number, on the earth). A field past the prefix
/// that the plugin could not find is NaN, and only that field is unread.
pub fn parse(bytes: &[u8]) -> Option<State> {
    if bytes.len() < LEN * 8 {
        return None;
    }
    let pos = position::parse(&bytes[..POSITION_FIELDS * 8])?;
    let mut vals = [f64::NAN; LEN];
    for (i, slot) in vals.iter_mut().enumerate() {
        let mut b = [0u8; 8];
        b.copy_from_slice(&bytes[i * 8..i * 8 + 8]);
        *slot = f64::from_le_bytes(b);
    }
    if vals[0] != LAYOUT_VERSION {
        return None;
    }
    Some(State {
        pos: Ok(pos),
        vals,
        source: Source::Plugin,
    })
}

/// A buffer in the plugin's layout, for tests and replays.
pub fn encode(s: &State) -> Vec<u8> {
    let mut vals = s.vals;
    if let Ok(p) = &s.pos {
        fill_position(&mut vals, p);
    }
    vals.iter().flat_map(|x| x.to_le_bytes()).collect()
}

/// The fields a split read asks for: the frame always carries all of them,
/// so this only bounds what an older plugin costs in requests (never in
/// rounds: they go out together).
#[derive(Debug, Clone, Copy)]
pub struct Want<'a>(pub &'a [F]);

async fn split_field<X: XPlane + ?Sized>(xp: &X, f: F) -> f64 {
    let v = if f == F::Trq0 {
        xp.get_float_array(f.name())
            .await
            .ok()
            .and_then(|a| a.first().copied())
    } else {
        xp.get_float(f.name()).await.ok()
    };
    v.unwrap_or(f64::NAN)
}

/// The tick's reading: the plugin's frame in one request, else the wanted
/// fields and the position read separately, all in ONE round. The freeze
/// flags ride every split read (they are cached half a second by the
/// client, `SLOW_DATAREFS`), so [`State::frozen`] answers on both paths.
pub async fn read<X: XPlane + ?Sized>(xp: &X, want: Want<'_>) -> State {
    if let Ok(bytes) = xp.get_bytes(STATE).await {
        if let Some(s) = parse(&bytes) {
            return s;
        }
    }
    split(xp, want).await
}

async fn split<X: XPlane + ?Sized>(xp: &X, want: Want<'_>) -> State {
    let mut fields: Vec<F> = want.0.to_vec();
    for f in [F::Paused, F::Replay] {
        if !fields.contains(&f) {
            fields.push(f);
        }
    }
    let reads = futures::future::join_all(fields.iter().map(|&f| split_field(xp, f)));
    let (pos, vals) = tokio::join!(position::read(xp), reads);
    let mut s = State::from_parts(pos, &[], Source::Split);
    for (f, v) in fields.iter().zip(vals) {
        s.vals[f.slot()] = v;
    }
    s
}

/// The reading and `extra` scalar reads (datarefs the frame does not
/// carry, e.g. a profile's own throttle), in ONE round. An extra the frame
/// does carry is answered from it, no request.
pub async fn read_with<X: XPlane + ?Sized>(
    xp: &X,
    want: Want<'_>,
    extra: &[&str],
) -> (State, Vec<Result<f64>>) {
    let own: Vec<Option<F>> = extra.iter().map(|n| F::for_name(n)).collect();
    // The frame carries it only if the plugin sent one: on a split read
    // the carried fields join the split round instead.
    let extras = futures::future::join_all(
        extra
            .iter()
            .zip(&own)
            .filter(|(_, f)| f.is_none())
            .map(|(n, _)| xp.get_float(n)),
    );
    let mut want_all: Vec<F> = want.0.to_vec();
    for f in own.iter().flatten() {
        if !want_all.contains(f) {
            want_all.push(*f);
        }
    }
    let (s, mut uncarried) = tokio::join!(read(xp, Want(&want_all)), extras);
    let mut out = Vec::with_capacity(extra.len());
    let mut it = uncarried.drain(..);
    for f in own {
        out.push(match f {
            Some(f) => s.get(f),
            None => it.next().unwrap_or_else(|| Err(anyhow!("unread"))),
        });
    }
    (s, out)
}

/// A tick's writes, sent together: ONE round, however many (the reads'
/// rule for the other half of a tick). A dataref set twice keeps the last
/// value, as two writes in a row would have left it. Writes that must land
/// in an ORDER (a mode engaged, then its dial) are not for this: they stay
/// awaited one after the other where they are made.
#[derive(Debug, Default, Clone)]
pub struct Writes(Vec<(String, f64)>);

/// What a [`Writes::flush`] got back, per dataref.
#[derive(Debug, Default)]
pub struct Written(Vec<(String, bool)>);

impl Writes {
    pub fn new() -> Self {
        Self::default()
    }

    /// Queue `name = value` for the flush.
    pub fn set(&mut self, name: &str, value: f64) -> &mut Self {
        match self.0.iter_mut().find(|(n, _)| n == name) {
            Some(slot) => slot.1 = value,
            None => self.0.push((name.to_string(), value)),
        }
        self
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    /// The value queued for `name`, if any.
    pub fn queued(&self, name: &str) -> Option<f64> {
        self.0.iter().find(|(n, _)| n == name).map(|(_, v)| *v)
    }

    /// Send every queued write at once and empty the batch.
    pub async fn flush<X: XPlane + ?Sized>(&mut self, xp: &X) -> Written {
        let items = std::mem::take(&mut self.0);
        let results =
            futures::future::join_all(items.iter().map(|(n, v)| xp.set_float(n, *v))).await;
        Written(
            items
                .into_iter()
                .zip(results)
                .map(|((n, _), r)| (n, r.is_ok()))
                .collect(),
        )
    }
}

impl Written {
    /// Whether the write to `name` went through (`None`: not in the batch).
    pub fn ok(&self, name: &str) -> Option<bool> {
        self.0.iter().find(|(n, _)| n == name).map(|(_, ok)| *ok)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xplane::{FakeXPlane, FramedXPlane};

    fn pos() -> Position {
        Position {
            lat: 36.674_914_321_987_65,
            lon: -4.499_123_456_789_01,
            elev_m: 16.25,
            psi_true_deg: 128.5,
            gs_ms: 66.9,
            vs_fpm: -3.0,
            agl_m: 2.1,
            frame: Some(123_456),
            sim_time_s: Some(4_321.75),
            source: position::Source::Plugin,
        }
    }

    fn sample() -> State {
        let fields: Vec<(F, f64)> = FIELDS
            .iter()
            .enumerate()
            .map(|(i, &f)| (f, 100.0 + i as f64))
            .collect();
        State::from_parts(Ok(pos()), &fields, Source::Plugin)
    }

    const WANT: Want = Want(&[F::Ias, F::OnGround, F::Phi]);

    #[test]
    fn the_frame_parses_to_the_state_it_holds() {
        let s = sample();
        let bytes = encode(&s);
        assert_eq!(bytes.len(), LEN * 8);
        assert_eq!(LEN, POSITION_FIELDS + FIELDS.len());
        let back = parse(&bytes).unwrap();
        assert_eq!(back, s);
        assert_eq!(back.position().unwrap(), pos());
        assert_eq!(back.get(F::Ias).unwrap(), 100.0);
        assert_eq!(back.get(F::ThrottleAll).unwrap(), 127.0);
        assert_eq!(back.frame(), Some(123_456));
        // A later layout adding fields still reads.
        let mut longer = bytes.clone();
        longer.extend_from_slice(&7.0_f64.to_le_bytes());
        assert_eq!(parse(&longer).unwrap(), s);
    }

    #[test]
    fn slots_follow_the_layout() {
        for (i, f) in FIELDS.iter().enumerate() {
            assert_eq!(f.slot(), POSITION_FIELDS + i, "{f:?}");
            assert_eq!(F::for_name(f.name()), Some(*f));
        }
    }

    /// The plugin lists its sources in the same order as [`FIELDS`]: the
    /// two ends of the wire pinned together.
    #[test]
    fn the_plugin_fills_the_same_slots() {
        let src = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../plugin/src/drefs.cpp"
        ))
        .unwrap();
        let start = src.find("kStateSources[] = {").expect("kStateSources");
        let body = &src[start..start + src[start..].find("};").unwrap()];
        let names: Vec<&str> = body.split('"').skip(1).step_by(2).collect();
        let want: Vec<&str> = FIELDS.iter().map(|f| f.name()).collect();
        assert_eq!(names, want);
        assert!(src.contains(&format!("\"{STATE}\"")), "registered");
    }

    #[test]
    fn a_broken_frame_is_no_frame_and_a_missing_field_is_one_unread_field() {
        let s = sample();
        assert!(parse(&encode(&s)[..LEN * 8 - 8]).is_none(), "short");
        let mut unready = encode(&s);
        unready[..8].copy_from_slice(&0.0_f64.to_le_bytes());
        assert!(parse(&unready).is_none(), "version 0");
        let mut nan_lat = encode(&s);
        nan_lat[3 * 8..4 * 8].copy_from_slice(&f64::NAN.to_le_bytes());
        assert!(parse(&nan_lat).is_none(), "the position must be whole");
        let mut nan_ias = encode(&s);
        let at = F::Ias.slot() * 8;
        nan_ias[at..at + 8].copy_from_slice(&f64::NAN.to_le_bytes());
        let p = parse(&nan_ias).unwrap();
        assert!(
            p.get(F::Ias).is_err(),
            "a dataref the plugin could not find"
        );
        assert!(p.get(F::Phi).is_ok());
    }

    #[test]
    fn frozen_is_the_freeze_flags_failing_open() {
        let live = State::from_parts(Ok(pos()), &[], Source::Split);
        assert!(!live.frozen(), "unread flags are live");
        let paused = State::from_parts(Ok(pos()), &[(F::Paused, 1.0)], Source::Split);
        assert!(paused.frozen());
        let replay = State::from_parts(
            Ok(pos()),
            &[(F::Paused, 0.0), (F::Replay, 1.0)],
            Source::Split,
        );
        assert!(replay.frozen());
    }

    fn split_fake() -> FakeXPlane {
        let fake = FakeXPlane::new();
        for (k, v) in [
            (crate::datarefs::LATITUDE, 51.15),
            (crate::datarefs::LONGITUDE, -0.19),
            (crate::datarefs::ELEVATION_M, 60.0),
            ("sim/flightmodel/position/psi", 77.5),
            ("sim/flightmodel/position/groundspeed", 40.0),
            ("sim/flightmodel/position/vh_ind_fpm", 0.0),
            (crate::datarefs::AGL_METERS, 1.9),
            (F::Ias.name(), 88.0),
            (F::OnGround.name(), 1.0),
            (F::Phi.name(), -1.5),
            (F::Paused.name(), 0.0),
            (F::Replay.name(), 0.0),
            ("test/throttle", 0.4),
        ] {
            fake.set_read(k, v);
        }
        fake
    }

    #[tokio::test]
    async fn the_frame_is_read_when_the_plugin_has_it() {
        let fake = split_fake();
        fake.set_read_bytes(STATE, Some(&encode(&sample())));
        let s = read(&fake, WANT).await;
        assert_eq!(s.source, Source::Plugin);
        assert_eq!(s, sample());
    }

    /// An older plugin: the wanted fields and the position, read as they
    /// always were, and the reading says it is split.
    #[tokio::test]
    async fn an_older_plugin_falls_back_to_the_separate_reads() {
        let fake = split_fake();
        let s = read(&fake, WANT).await;
        assert_eq!(s.source, Source::Split);
        assert_eq!(s.position().unwrap().source, position::Source::Split);
        assert_eq!(s.position().unwrap().lat, 51.15);
        assert_eq!(s.get(F::Ias).unwrap(), 88.0);
        assert_eq!(s.get(F::Phi).unwrap(), -1.5);
        assert!(!s.frozen());
        assert!(s.get(F::Theta).is_err(), "not wanted, not read");
        // A plugin with the position snapshot but no state frame: the
        // position is still one frame's.
        fake.set_read_bytes(position::POSITION, Some(&position::encode(&pos())));
        let s = read(&fake, WANT).await;
        assert_eq!(s.source, Source::Split);
        assert_eq!(s.position().unwrap(), pos());
    }

    #[tokio::test]
    async fn a_failed_split_field_is_one_failed_read() {
        let fake = split_fake();
        let s = read(&fake, Want(&[F::Ias, F::Theta])).await;
        assert!(s.get(F::Theta).is_err());
        assert!(s.get(F::Ias).is_ok());
        let bare = FakeXPlane::with_reads(&[(F::Ias.name(), 90.0)]);
        let s = read(&bare, Want(&[F::Ias])).await;
        assert!(s.position().is_err(), "no position, no measurement");
        assert_eq!(s.get(F::Ias).unwrap(), 90.0);
    }

    #[tokio::test]
    async fn extras_ride_the_same_read() {
        let fake = split_fake();
        fake.set_read_bytes(STATE, Some(&encode(&sample())));
        let (s, extra) = read_with(&fake, WANT, &["test/throttle", F::ThrottleAll.name()]).await;
        assert_eq!(s.source, Source::Plugin);
        assert_eq!(extra[0].as_ref().unwrap(), &0.4);
        assert_eq!(extra[1].as_ref().unwrap(), &127.0, "the frame's own lever");
        // Split: the carried one joins the split round.
        fake.set_read_bytes(STATE, None);
        fake.set_read(F::ThrottleAll.name(), 0.7);
        let (_, extra) = read_with(&fake, WANT, &["test/throttle", F::ThrottleAll.name()]).await;
        assert_eq!(extra[0].as_ref().unwrap(), &0.4);
        assert_eq!(extra[1].as_ref().unwrap(), &0.7);
    }

    #[tokio::test(start_paused = true)]
    async fn a_tick_of_writes_is_one_round_and_the_last_value_stands() {
        let xp = FramedXPlane::new(FakeXPlane::new(), std::time::Duration::from_millis(50));
        let mut w = Writes::new();
        w.set("xcopilot/taxi_steer", 0.0)
            .set("xcopilot/taxi_brake_l", 0.2)
            .set("xcopilot/taxi_brake_r", 0.0)
            .set("xcopilot/taxi_steer", 0.3);
        assert_eq!(w.queued("xcopilot/taxi_steer"), Some(0.3));
        let mut done = Written::default();
        let (rounds, requests) = xp.rounds(|| async { done = w.flush(&xp).await }).await;
        assert_eq!((rounds, requests), (1, 3));
        assert!(w.is_empty());
        assert_eq!(done.ok("xcopilot/taxi_steer"), Some(true));
        assert_eq!(done.ok("xcopilot/pitch"), None);
        assert_eq!(xp.inner.writes_to("xcopilot/taxi_steer"), vec![0.3]);
    }

    /// The point of it all: one round a tick, the frame or the split.
    #[tokio::test(start_paused = true)]
    async fn a_reading_is_one_round_either_way() {
        let fake = split_fake();
        fake.set_read_bytes(STATE, Some(&encode(&sample())));
        let xp = FramedXPlane::new(fake, std::time::Duration::from_millis(50));
        let r = xp.rounds(|| async {
            let _ = read_with(&xp, WANT, &["test/throttle"]).await;
        });
        let (rounds, requests) = r.await;
        assert_eq!(
            (rounds, requests),
            (1, 2),
            "the frame and the profile's lever"
        );
        xp.inner.set_read_bytes(STATE, None);
        let (rounds, requests) = xp
            .rounds(|| async {
                let _ = read_with(&xp, WANT, &["test/throttle"]).await;
            })
            .await;
        // The split: 7 position reads + 3 wanted + 2 freeze flags + the lever,
        // all at once (the absent frames cost no request, the client's
        // absent memory).
        assert_eq!((rounds, requests), (1, 13));
    }
}
