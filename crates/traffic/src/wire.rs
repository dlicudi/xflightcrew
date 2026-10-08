//! The packed wire to the plugin (stage D increment 10): the bytes the bin
//! writes into `xcopilot/traffic/roster` (on change) and
//! `xcopilot/traffic/frames` (at most 4 Hz), and the plugin's
//! `plugin/src/traffic_wire.h` decodes. That header is the one reference;
//! this module matches it byte for byte (pinned by the golden test below,
//! whose bytes the plugin's own test decodes).
//!
//! Every integer and float is LITTLE-ENDIAN, floats IEEE 754. Both buffers
//! open with a `u16` version ([`VERSION`]) and a `u16` count, and fit in
//! [`BUF_BYTES`].
//!
//! Roster: header 8 bytes (`u16 version`, `u16 count`, `u32 seq`), then 48
//! bytes per aircraft: `u32 id` @0, `u32 mode_s` @4 (24 bit, 0 = the plugin
//! picks), `u16 squawk` @8 (the four octal figures read as a decimal
//! number), `u16` reserved @10, `char[4]` ICAO type @12, `char[4]` airline
//! @16, `char[16]` livery @20, `char[12]` callsign @36, NUL-padded.
//!
//! Frames: header 16 bytes (`u16 version`, `u16 count`, `u32 seq`, `f64` sim
//! time @8), then 40 bytes per aircraft: `u32 id` @0, `f32` altitude ft MSL
//! @4, `f64` lat @8, `f64` lon @16, `u16` true heading @24, `i16` pitch @26
//! and `i16` roll @28 in centidegrees, `u16` groundspeed @30 in tenths of a
//! knot, `i16` vertical speed @32 in ft/min, `u8` gear @34, flaps @35,
//! spoiler @36, reverser @37 (0..255 = 0..1), `u8` lights @38, `u8` flags @39.
//!
//! Pure: no I/O. The decoders are for tests and tools; the plugin is the one
//! that reads these bytes.

use crate::aircraft::TrafficAircraft;
use crate::phase::{Phase, Plan};

/// The wire's version, in both buffers.
pub const VERSION: u16 = 1;
/// Each buffer's size in the plugin.
pub const BUF_BYTES: usize = 16_384;
pub const ROSTER_HEADER: usize = 8;
pub const ROSTER_ENTRY: usize = 48;
pub const FRAME_HEADER: usize = 16;
pub const FRAME_ENTRY: usize = 40;
/// The most aircraft one buffer carries (the rest are left out).
pub const MAX_ROSTER: usize = (BUF_BYTES - ROSTER_HEADER) / ROSTER_ENTRY;
pub const MAX_FRAMES: usize = (BUF_BYTES - FRAME_HEADER) / FRAME_ENTRY;

pub const LIGHT_NAV: u8 = 1 << 0;
pub const LIGHT_BEACON: u8 = 1 << 1;
pub const LIGHT_STROBE: u8 = 1 << 2;
pub const LIGHT_LANDING: u8 = 1 << 3;
pub const LIGHT_TAXI: u8 = 1 << 4;

pub const FLAG_ON_GROUND: u8 = 1 << 0;
pub const FLAG_ENGINES: u8 = 1 << 1;
pub const FLAG_HIDDEN: u8 = 1 << 2;

/// One aircraft in the roster: who it is and what it looks like.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct RosterEntry {
    pub id: u32,
    /// 24 bit; 0 = let the plugin pick.
    pub mode_s: u32,
    /// The four octal figures read as a decimal number (7000 = 7000).
    pub squawk: u16,
    pub icao_type: String,
    /// The ICAO airline ("EZY"); empty for general aviation.
    pub airline: String,
    /// Empty = any.
    pub livery: String,
    /// The label ("G-ABCD", "EZY12AB").
    pub callsign: String,
}

/// One aircraft in one frame, in engineering units.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct FrameEntry {
    pub id: u32,
    pub lat: f64,
    pub lon: f64,
    pub alt_ft: f64,
    /// Degrees true.
    pub hdg_deg: f64,
    pub pitch_deg: f64,
    pub roll_deg: f64,
    pub gs_kts: f64,
    pub vs_fpm: f64,
    /// Each 0..1.
    pub gear: f64,
    pub flaps: f64,
    pub spoiler: f64,
    pub reverser: f64,
    pub lights: u8,
    pub flags: u8,
}

/// A buffer the decoders refuse, as the plugin does.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WireError {
    Short,
    Version,
    Value,
}

/// The roster: header and up to [`MAX_ROSTER`] entries.
pub fn encode_roster(seq: u32, entries: &[RosterEntry]) -> Vec<u8> {
    let entries = &entries[..entries.len().min(MAX_ROSTER)];
    let mut b = Vec::with_capacity(ROSTER_HEADER + entries.len() * ROSTER_ENTRY);
    b.extend(VERSION.to_le_bytes());
    b.extend((entries.len() as u16).to_le_bytes());
    b.extend(seq.to_le_bytes());
    for e in entries {
        b.extend(e.id.to_le_bytes());
        b.extend((e.mode_s & 0x00ff_ffff).to_le_bytes());
        b.extend(e.squawk.to_le_bytes());
        b.extend(0u16.to_le_bytes());
        put_str(&mut b, &e.icao_type, 4);
        put_str(&mut b, &e.airline, 4);
        put_str(&mut b, &e.livery, 16);
        put_str(&mut b, &e.callsign, 12);
    }
    b
}

/// The frames: header (with the sim time they were taken at) and up to
/// [`MAX_FRAMES`] entries. A non-finite value goes out as 0 (the plugin
/// refuses a buffer with a non-finite position whole).
pub fn encode_frames(seq: u32, sim_time_s: f64, entries: &[FrameEntry]) -> Vec<u8> {
    let entries = &entries[..entries.len().min(MAX_FRAMES)];
    let mut b = Vec::with_capacity(FRAME_HEADER + entries.len() * FRAME_ENTRY);
    b.extend(VERSION.to_le_bytes());
    b.extend((entries.len() as u16).to_le_bytes());
    b.extend(seq.to_le_bytes());
    b.extend(finite(sim_time_s).to_le_bytes());
    for e in entries {
        b.extend(e.id.to_le_bytes());
        b.extend((finite(e.alt_ft) as f32).to_le_bytes());
        b.extend(finite(e.lat).clamp(-90.0, 90.0).to_le_bytes());
        b.extend(finite(e.lon).clamp(-180.0, 180.0).to_le_bytes());
        let hdg = (finite(e.hdg_deg).rem_euclid(360.0) * 100.0).round() as u32 % 36_000;
        b.extend((hdg as u16).to_le_bytes());
        b.extend(centideg(e.pitch_deg).to_le_bytes());
        b.extend(centideg(e.roll_deg).to_le_bytes());
        let gs = (finite(e.gs_kts) * 10.0)
            .round()
            .clamp(0.0, f64::from(u16::MAX)) as u16;
        b.extend(gs.to_le_bytes());
        let vs = finite(e.vs_fpm)
            .round()
            .clamp(f64::from(i16::MIN), f64::from(i16::MAX)) as i16;
        b.extend(vs.to_le_bytes());
        for r in [e.gear, e.flaps, e.spoiler, e.reverser] {
            b.push((finite(r).clamp(0.0, 1.0) * 255.0).round() as u8);
        }
        b.push(e.lights);
        b.push(e.flags);
    }
    b
}

/// The roster entry for one traffic aircraft: an airline's three letters
/// where its callsign is one ("EZY4069"), none for a registration; a squawk
/// of 7000 in the circuit, else a discrete code from its id.
pub fn roster_entry(ac: &TrafficAircraft) -> RosterEntry {
    let cs = ac.callsign.as_bytes();
    let airline =
        (cs.len() > 3 && cs[..3].iter().all(u8::is_ascii_uppercase) && cs[3].is_ascii_digit())
            .then(|| ac.callsign[..3].to_string())
            .unwrap_or_default();
    let squawk = match ac.plan {
        Plan::Circuit { .. } | Plan::Parked => 7000,
        _ => {
            // Four octal figures: 2 then the id's three.
            let n = ac.id % 512;
            (2000 + (n / 64) * 100 + (n / 8 % 8) * 10 + n % 8) as u16
        }
    };
    RosterEntry {
        id: ac.id,
        mode_s: 0,
        squawk,
        icao_type: ac.icao_type.clone(),
        airline,
        livery: String::new(),
        callsign: ac.callsign.clone(),
    }
}

/// The frame entry for one traffic aircraft as it is now. Engines run from
/// the beacon (lit for the taxi) until it parks; no spoiler or reverser in
/// v1.
pub fn frame_entry(ac: &TrafficAircraft) -> FrameEntry {
    let k = &ac.kin;
    let mut lights = 0;
    for (on, bit) in [
        (k.lights.nav, LIGHT_NAV),
        (k.lights.beacon, LIGHT_BEACON),
        (k.lights.strobe, LIGHT_STROBE),
        (k.lights.landing, LIGHT_LANDING),
        (k.lights.taxi, LIGHT_TAXI),
    ] {
        if on {
            lights |= bit;
        }
    }
    let engines = k.lights.beacon || !matches!(ac.phase, Phase::Parked | Phase::Gone);
    let mut flags = 0;
    if k.on_ground {
        flags |= FLAG_ON_GROUND;
    }
    if engines {
        flags |= FLAG_ENGINES;
    }
    FrameEntry {
        id: ac.id,
        lat: k.lat,
        lon: k.lon,
        alt_ft: k.alt_ft,
        hdg_deg: k.hdg_deg,
        pitch_deg: k.pitch,
        roll_deg: k.roll,
        gs_kts: k.gs_kts,
        vs_fpm: k.vs_fpm,
        gear: k.gear,
        flaps: k.flaps,
        spoiler: 0.0,
        reverser: 0.0,
        lights,
        flags,
    }
}

/// Decode a roster as the plugin does: `(seq, entries)`.
pub fn decode_roster(b: &[u8]) -> Result<(u32, Vec<RosterEntry>), WireError> {
    if b.len() < ROSTER_HEADER {
        return Err(WireError::Short);
    }
    if u16_at(b, 0) != VERSION {
        return Err(WireError::Version);
    }
    let count = usize::from(u16_at(b, 2));
    if b.len() < ROSTER_HEADER + count * ROSTER_ENTRY {
        return Err(WireError::Short);
    }
    let entries = (0..count)
        .map(|i| {
            let e = &b[ROSTER_HEADER + i * ROSTER_ENTRY..];
            RosterEntry {
                id: u32_at(e, 0),
                mode_s: u32_at(e, 4),
                squawk: u16_at(e, 8),
                icao_type: str_at(e, 12, 4),
                airline: str_at(e, 16, 4),
                livery: str_at(e, 20, 16),
                callsign: str_at(e, 36, 12),
            }
        })
        .collect();
    Ok((u32_at(b, 4), entries))
}

/// Decode frames as the plugin does: `(seq, sim time, entries)`, in the
/// wire's resolution.
pub fn decode_frames(b: &[u8]) -> Result<(u32, f64, Vec<FrameEntry>), WireError> {
    if b.len() < FRAME_HEADER {
        return Err(WireError::Short);
    }
    if u16_at(b, 0) != VERSION {
        return Err(WireError::Version);
    }
    let count = usize::from(u16_at(b, 2));
    if b.len() < FRAME_HEADER + count * FRAME_ENTRY {
        return Err(WireError::Short);
    }
    let sim_time_s = f64_at(b, 8);
    if !sim_time_s.is_finite() {
        return Err(WireError::Value);
    }
    let mut entries = Vec::with_capacity(count);
    for i in 0..count {
        let e = &b[FRAME_HEADER + i * FRAME_ENTRY..];
        let (lat, lon) = (f64_at(e, 8), f64_at(e, 16));
        let alt_ft = f64::from(f32::from_le_bytes([e[4], e[5], e[6], e[7]]));
        if !lat.is_finite() || !lon.is_finite() || !alt_ft.is_finite() {
            return Err(WireError::Value);
        }
        if lat.abs() > 90.0 || lon.abs() > 180.0 {
            return Err(WireError::Value);
        }
        let ratio = |x: u8| f64::from(x) / 255.0;
        entries.push(FrameEntry {
            id: u32_at(e, 0),
            lat,
            lon,
            alt_ft,
            hdg_deg: f64::from(u16_at(e, 24) % 36_000) / 100.0,
            pitch_deg: f64::from(u16_at(e, 26) as i16) / 100.0,
            roll_deg: f64::from(u16_at(e, 28) as i16) / 100.0,
            gs_kts: f64::from(u16_at(e, 30)) / 10.0,
            vs_fpm: f64::from(u16_at(e, 32) as i16),
            gear: ratio(e[34]),
            flaps: ratio(e[35]),
            spoiler: ratio(e[36]),
            reverser: ratio(e[37]),
            lights: e[38],
            flags: e[39],
        });
    }
    Ok((u32_at(b, 4), sim_time_s, entries))
}

fn finite(v: f64) -> f64 {
    if v.is_finite() {
        v
    } else {
        0.0
    }
}

fn centideg(deg: f64) -> i16 {
    (finite(deg) * 100.0)
        .round()
        .clamp(f64::from(i16::MIN), f64::from(i16::MAX)) as i16
}

/// `s` in `n` bytes, NUL-padded, cut (not terminated) when full; ASCII only
/// (anything else is a `?`).
fn put_str(b: &mut Vec<u8>, s: &str, n: usize) {
    let mut field: Vec<u8> = s
        .chars()
        .map(|c| if c.is_ascii() { c as u8 } else { b'?' })
        .take(n)
        .collect();
    field.resize(n, 0);
    b.extend(field);
}

fn u16_at(b: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([b[at], b[at + 1]])
}

fn u32_at(b: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([b[at], b[at + 1], b[at + 2], b[at + 3]])
}

fn f64_at(b: &[u8], at: usize) -> f64 {
    let mut x = [0u8; 8];
    x.copy_from_slice(&b[at..at + 8]);
    f64::from_le_bytes(x)
}

fn str_at(b: &[u8], at: usize, n: usize) -> String {
    let f = &b[at..at + n];
    let len = f.iter().position(|c| *c == 0).unwrap_or(n);
    String::from_utf8_lossy(&f[..len]).into_owned()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(b: &[u8]) -> String {
        b.iter().map(|x| format!("{x:02x}")).collect()
    }

    /// The fixed roster the plugin's test decodes too.
    fn golden_roster() -> Vec<RosterEntry> {
        vec![
            RosterEntry {
                id: 1,
                mode_s: 0,
                squawk: 7000,
                icao_type: "C172".into(),
                airline: String::new(),
                livery: String::new(),
                callsign: "G-ABCD".into(),
            },
            RosterEntry {
                id: 258,
                mode_s: 0x00ab_cdef,
                squawk: 2341,
                icao_type: "A320".into(),
                airline: "EZY".into(),
                livery: "EZY".into(),
                callsign: "EZY12AB".into(),
            },
        ]
    }

    /// The fixed frames the plugin's test decodes too: every field at a
    /// value that shows its scale and sign.
    fn golden_frames() -> Vec<FrameEntry> {
        vec![
            FrameEntry {
                id: 1,
                lat: 36.5,
                lon: -4.25,
                alt_ft: 1000.0,
                hdg_deg: 90.0,
                pitch_deg: 2.5,
                roll_deg: -10.0,
                gs_kts: 85.5,
                vs_fpm: -500.0,
                gear: 1.0,
                flaps: 0.5,
                spoiler: 0.0,
                reverser: 0.0,
                lights: LIGHT_NAV | LIGHT_BEACON | LIGHT_STROBE | LIGHT_LANDING,
                flags: FLAG_ENGINES,
            },
            FrameEntry {
                id: 258,
                lat: -33.75,
                lon: 151.125,
                alt_ft: 25.0,
                hdg_deg: 359.99,
                pitch_deg: 0.0,
                roll_deg: 0.0,
                gs_kts: 0.0,
                vs_fpm: 0.0,
                gear: 1.0,
                flaps: 0.0,
                spoiler: 1.0,
                reverser: 1.0,
                lights: LIGHT_NAV | LIGHT_TAXI,
                flags: FLAG_ON_GROUND | FLAG_ENGINES,
            },
        ]
    }

    /// THE GOLDEN BYTES: fixed input, exact hex. The plugin's decoder
    /// (`plugin/test/traffic_wire_test.cpp`) is checked against these same
    /// bytes (the spec's "Wire (as built)"); a change here is a wire change.
    #[test]
    fn the_golden_bytes() {
        let roster = encode_roster(7, &golden_roster());
        assert_eq!(roster.len(), ROSTER_HEADER + 2 * ROSTER_ENTRY);
        assert_eq!(hex(&roster), GOLDEN_ROSTER);
        let frames = encode_frames(42, 1234.5, &golden_frames());
        assert_eq!(frames.len(), FRAME_HEADER + 2 * FRAME_ENTRY);
        assert_eq!(hex(&frames), GOLDEN_FRAMES);
    }

    /// `encode_roster(7, golden_roster())`, field by field: header
    /// (version 1, count 2, seq 7); then id 1, mode S 0, squawk 7000,
    /// reserved, "C172", no airline, no livery, "G-ABCD"; then id 258, mode S
    /// abcdef, squawk 2341, reserved, "A320", "EZY", "EZY", "EZY12AB".
    #[rustfmt::skip]
    const GOLDEN_ROSTER: &str = concat!(
        "0100", "0200", "07000000",
        "01000000", "00000000", "581b", "0000", "43313732", "00000000",
        "00000000000000000000000000000000", "472d41424344000000000000",
        "02010000", "efcdab00", "2509", "0000", "41333230", "455a5900",
        "455a5900000000000000000000000000", "455a59313241420000000000",
    );

    /// `encode_frames(42, 1234.5, golden_frames())`: header (version 1,
    /// count 2, seq 42, sim time 1234.5); then id 1 at 1,000 ft, 36.5 N
    /// 4.25 W, heading 90, pitch 2.5, roll -10, 85.5 kt, -500 fpm, gear 1,
    /// flaps 0.5, nav beacon strobe landing, engines; then id 258 at 25 ft,
    /// 33.75 S 151.125 E, heading 359.99, gear 1, spoiler 1, reverser 1,
    /// nav taxi, on the ground with engines.
    #[rustfmt::skip]
    const GOLDEN_FRAMES: &str = concat!(
        "0100", "0200", "2a000000", "00000000004a9340",
        "01000000", "00007a44", "0000000000404240", "00000000000011c0",
        "2823", "fa00", "18fc", "5703", "0cfe", "ff", "80", "00", "00", "0f", "02",
        "02010000", "0000c841", "0000000000e040c0", "0000000000e46240",
        "9f8c", "0000", "0000", "0000", "0000", "ff", "00", "ff", "ff", "11", "03",
    );

    /// What goes out comes back, in the wire's resolution.
    #[test]
    fn the_layout_round_trips() {
        let (seq, r) = decode_roster(&encode_roster(7, &golden_roster())).unwrap();
        assert_eq!(seq, 7);
        assert_eq!(r, golden_roster());
        let (seq, t, f) = decode_frames(&encode_frames(42, 1234.5, &golden_frames())).unwrap();
        assert_eq!((seq, t), (42, 1234.5));
        let want = golden_frames();
        for (a, b) in f.iter().zip(&want) {
            assert_eq!(
                (a.id, a.lat, a.lon, a.lights, a.flags),
                (b.id, b.lat, b.lon, b.lights, b.flags)
            );
            assert!((a.alt_ft - b.alt_ft).abs() < 1e-3);
            assert!(
                (a.hdg_deg - b.hdg_deg).abs() < 0.006,
                "{} {}",
                a.hdg_deg,
                b.hdg_deg
            );
            assert!((a.pitch_deg - b.pitch_deg).abs() < 0.006);
            assert!((a.roll_deg - b.roll_deg).abs() < 0.006);
            assert!((a.gs_kts - b.gs_kts).abs() < 0.051);
            assert!((a.vs_fpm - b.vs_fpm).abs() < 0.51);
            assert!((a.flaps - b.flaps).abs() < 0.003, "{}", a.flaps);
            assert!((a.gear - b.gear).abs() < 0.003);
        }
    }

    /// The edges: a full buffer stops at the plugin's 16 KB, strings are cut
    /// not overrun, a heading of 360 is 0, out-of-range numbers are clamped,
    /// a non-finite one goes out as 0, and the decoder refuses what the
    /// plugin refuses.
    #[test]
    fn the_edges_hold() {
        let many = vec![FrameEntry::default(); 1000];
        let b = encode_frames(1, 0.0, &many);
        assert!(b.len() <= BUF_BYTES, "{}", b.len());
        assert_eq!(decode_frames(&b).unwrap().2.len(), MAX_FRAMES);
        let many = vec![RosterEntry::default(); 1000];
        assert!(encode_roster(1, &many).len() <= BUF_BYTES);

        let long = RosterEntry {
            callsign: "ABCDEFGHIJKLMNOP".into(),
            icao_type: "B7378".into(),
            ..Default::default()
        };
        let (_, r) = decode_roster(&encode_roster(1, &[long])).unwrap();
        assert_eq!(r[0].callsign, "ABCDEFGHIJKL", "cut at 12");
        assert_eq!(r[0].icao_type, "B737", "cut at 4");

        let odd = FrameEntry {
            hdg_deg: 360.0,
            vs_fpm: 99_999.0,
            gs_kts: -5.0,
            alt_ft: f64::NAN,
            flaps: 3.0,
            ..Default::default()
        };
        let (_, _, f) = decode_frames(&encode_frames(1, 0.0, &[odd])).unwrap();
        assert_eq!(f[0].hdg_deg, 0.0);
        assert_eq!(f[0].vs_fpm, 32_767.0);
        assert_eq!(f[0].gs_kts, 0.0);
        assert_eq!(f[0].alt_ft, 0.0);
        assert_eq!(f[0].flaps, 1.0);

        let mut b = encode_frames(1, 0.0, &golden_frames());
        assert_eq!(decode_frames(&b[..30]), Err(WireError::Short));
        b[0] = 2;
        assert_eq!(decode_frames(&b), Err(WireError::Version));
    }

    /// A traffic aircraft on the wire: its type, an airline's letters or
    /// none, the circuit's 7000 or a discrete octal code, its lights and
    /// flags.
    #[test]
    fn an_aircraft_goes_on_the_wire() {
        let field = crate::field::fixtures::xtst();
        let mut ac = TrafficAircraft::at_stand(
            5,
            "EZY4069",
            "Easy four zero six niner",
            "AT76",
            Plan::Departure {
                runway: "09".into(),
                sid: None,
                exit_bearing: 90.0,
            },
            &field,
            &field.net.stands[0].name,
            1,
        )
        .unwrap();
        let r = roster_entry(&ac);
        assert_eq!((r.airline.as_str(), r.icao_type.as_str()), ("EZY", "AT76"));
        assert_eq!(r.squawk, 2005, "2 then 005 in octal");
        assert!(
            r.squawk.to_string().chars().all(|c| c < '8'),
            "octal figures"
        );
        let f = frame_entry(&ac);
        assert_eq!(f.flags, FLAG_ON_GROUND, "parked, beacon off: engines off");
        ac.kin.lights.beacon = true;
        assert_eq!(frame_entry(&ac).flags, FLAG_ON_GROUND | FLAG_ENGINES);
        assert_eq!(frame_entry(&ac).lights, LIGHT_BEACON);
        ac.callsign = "G-ABCD".into();
        ac.plan = Plan::Circuit {
            runway: "09".into(),
            laps: 1,
            left: true,
        };
        let r = roster_entry(&ac);
        assert_eq!((r.airline.as_str(), r.squawk), ("", 7000));
    }
}
