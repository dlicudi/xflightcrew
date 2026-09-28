//! Where the aircraft is, as ONE frame saw it.
//!
//! Latitude and longitude are separate Web API requests, and the API serves
//! each from whichever flight-loop frame it lands in, so one reading can pair
//! two frames. On LEMG's diagonal 13/31 that was up to 2.9 m of cross-track
//! for a single tick above 120 kt, 0.7 m on EGKK's east-west strip (#157,
//! found with #123 on 2026-09-28); every lateral law and the scorer read it.
//!
//! The plugin copies the fields into one buffer after the flight model each
//! frame (`xcopilot/position`, `plugin/src/drefs.cpp`), and [`read`] takes the
//! whole buffer in one request. An older `.xpl` lacks the dataref: then the
//! stock datarefs are read separately, as before, and the reading says so
//! ([`Source::Split`]) so the blackbox shows which one a law flew on. The
//! livestate WebSocket is no substitute: its cache is keyed per dataref and a
//! reader takes each under its own lock, so a pair can still straddle two
//! pushes, and the laws read over REST.

use anyhow::{anyhow, Result};

use crate::xplane::XPlane;

/// The plugin's packed position: native little-endian doubles, see
/// [`parse`] for the layout.
pub const POSITION: &str = "xcopilot/position";
/// The layout [`parse`] understands (element 0 of the buffer).
pub const LAYOUT_VERSION: f64 = 1.0;
/// Doubles in layout 1.
const FIELDS: usize = 10;

const LAT: &str = crate::datarefs::LATITUDE;
const LON: &str = crate::datarefs::LONGITUDE;
const ELEV_M: &str = crate::datarefs::ELEVATION_M;
const PSI_TRUE: &str = "sim/flightmodel/position/psi";
const GS_MS: &str = "sim/flightmodel/position/groundspeed";
const VS_FPM: &str = "sim/flightmodel/position/vh_ind_fpm";
const AGL_M: &str = crate::datarefs::AGL_METERS;

/// Which read a [`Position`] came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Source {
    /// The plugin's one-frame snapshot.
    Plugin,
    /// Separate stock-dataref requests (an older plugin): the fields may
    /// straddle frames.
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

/// One reading of where the aircraft is and how it moves.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Position {
    pub lat: f64,
    pub lon: f64,
    /// True altitude MSL, metres.
    pub elev_m: f64,
    /// True heading (`psi`), degrees.
    pub psi_true_deg: f64,
    /// Groundspeed, m/s.
    pub gs_ms: f64,
    /// Vertical speed (`vh_ind_fpm`), feet a minute.
    pub vs_fpm: f64,
    /// Height above the ground below (`y_agl`), metres.
    pub agl_m: f64,
    /// The sim frame the plugin copied (`None` on a split read).
    pub frame: Option<u64>,
    /// Sim time of that frame, seconds (`None` on a split read).
    pub sim_time_s: Option<f64>,
    pub source: Source,
}

/// The plugin buffer as a [`Position`], `None` unless it is a whole layout-1
/// buffer with every field a number and the position on the earth. Layout:
/// `[version, frame, sim time s, lat, lon, elevation m, psi deg, groundspeed
/// m/s, vertical speed fpm, y_agl m]`. Version 0 is a plugin that has not
/// copied a frame yet; a NaN field is a sim dataref the plugin could not find.
pub fn parse(bytes: &[u8]) -> Option<Position> {
    if bytes.len() < FIELDS * 8 {
        return None;
    }
    let mut v = [0.0_f64; FIELDS];
    for (i, slot) in v.iter_mut().enumerate() {
        let mut b = [0u8; 8];
        b.copy_from_slice(&bytes[i * 8..i * 8 + 8]);
        *slot = f64::from_le_bytes(b);
    }
    if v[0] != LAYOUT_VERSION || v.iter().any(|x| !x.is_finite()) {
        return None;
    }
    let [_, frame, t, lat, lon, elev_m, psi, gs_ms, vs_fpm, agl_m] = v;
    if lat.abs() > 90.0 || lon.abs() > 180.0 || frame < 0.0 {
        return None;
    }
    Some(Position {
        lat,
        lon,
        elev_m,
        psi_true_deg: psi,
        gs_ms,
        vs_fpm,
        agl_m,
        frame: Some(frame as u64),
        sim_time_s: Some(t),
        source: Source::Plugin,
    })
}

/// A buffer in the plugin's layout, for tests and replays.
pub fn encode(p: &Position) -> Vec<u8> {
    let v = [
        LAYOUT_VERSION,
        p.frame.unwrap_or(0) as f64,
        p.sim_time_s.unwrap_or(0.0),
        p.lat,
        p.lon,
        p.elev_m,
        p.psi_true_deg,
        p.gs_ms,
        p.vs_fpm,
        p.agl_m,
    ];
    v.iter().flat_map(|x| x.to_le_bytes()).collect()
}

/// The plugin's snapshot, `None` when the plugin lacks it or it does not
/// parse (the caller falls back).
async fn snapshot<X: XPlane + ?Sized>(xp: &X) -> Option<Position> {
    parse(&xp.get_bytes(POSITION).await.ok()?)
}

/// Where the aircraft is: the plugin's one-frame snapshot in one request,
/// else every field read separately (an older plugin). A failed field on the
/// split read fails the whole reading: a failed read is no measurement.
pub async fn read<X: XPlane + ?Sized>(xp: &X) -> Result<Position> {
    if let Some(p) = snapshot(xp).await {
        return Ok(p);
    }
    let (lat, lon, elev_m, psi, gs_ms, vs_fpm, agl_m) = tokio::join!(
        xp.get_float(LAT),
        xp.get_float(LON),
        xp.get_float(ELEV_M),
        xp.get_float(PSI_TRUE),
        xp.get_float(GS_MS),
        xp.get_float(VS_FPM),
        xp.get_float(AGL_M),
    );
    Ok(Position {
        lat: lat?,
        lon: lon?,
        elev_m: elev_m?,
        psi_true_deg: psi?,
        gs_ms: gs_ms?,
        vs_fpm: vs_fpm?,
        agl_m: agl_m?,
        frame: None,
        sim_time_s: None,
        source: Source::Split,
    })
}

/// Latitude and longitude only: the snapshot, else the two stock reads (so an
/// older plugin costs what it always did).
pub async fn read_lat_lon<X: XPlane + ?Sized>(xp: &X) -> Result<(f64, f64, Source)> {
    if let Some(p) = snapshot(xp).await {
        return Ok((p.lat, p.lon, Source::Plugin));
    }
    let (lat, lon) = tokio::join!(xp.get_float(LAT), xp.get_float(LON));
    Ok((lat?, lon?, Source::Split))
}

/// A reading's latitude and longitude as two read results, for a tick that
/// destructures its reads field by field.
pub fn lat_lon(r: &Result<Position>) -> (Result<f64>, Result<f64>) {
    match r {
        Ok(p) => (Ok(p.lat), Ok(p.lon)),
        Err(e) => (
            Err(anyhow!("position unread: {e:#}")),
            Err(anyhow!("position unread: {e:#}")),
        ),
    }
}

/// One field of a reading as a read result.
pub fn field(r: &Result<Position>, f: impl Fn(&Position) -> f64) -> Result<f64> {
    match r {
        Ok(p) => Ok(f(p)),
        Err(e) => Err(anyhow!("position unread: {e:#}")),
    }
}

/// The blackbox word for a reading's source (`null` when unread).
pub fn source_word(r: &Result<Position>) -> Option<&'static str> {
    r.as_ref().ok().map(|p| p.source.as_str())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xplane::FakeXPlane;

    fn sample() -> Position {
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
            source: Source::Plugin,
        }
    }

    fn split_reads(fake: &FakeXPlane) {
        for (k, v) in [
            (LAT, 51.15),
            (LON, -0.19),
            (ELEV_M, 60.0),
            (PSI_TRUE, 77.5),
            (GS_MS, 40.0),
            (VS_FPM, 0.0),
            (AGL_M, 1.9),
        ] {
            fake.set_read(k, v);
        }
    }

    /// The buffer round-trips bit for bit: doubles, so the latitude keeps the
    /// millimetres a float array would round to 0.4 m.
    #[test]
    fn the_plugin_buffer_parses_to_the_frame_it_holds() {
        let p = sample();
        let bytes = encode(&p);
        assert_eq!(bytes.len(), 80);
        assert_eq!(parse(&bytes), Some(p));
        // A longer buffer (a later layout adding fields) still reads.
        let mut longer = bytes.clone();
        longer.extend_from_slice(&7.0_f64.to_le_bytes());
        assert_eq!(parse(&longer), Some(p));
    }

    #[test]
    fn an_unready_short_or_broken_buffer_is_no_measurement() {
        let p = sample();
        let mut unready = encode(&p);
        unready[..8].copy_from_slice(&0.0_f64.to_le_bytes());
        assert_eq!(parse(&unready), None, "version 0: no frame copied yet");
        assert_eq!(parse(&encode(&p)[..72]), None, "short");
        assert_eq!(parse(&[]), None);
        let mut nan = encode(&p);
        nan[4 * 8..5 * 8].copy_from_slice(&f64::NAN.to_le_bytes());
        assert_eq!(parse(&nan), None, "a field the plugin could not find");
        let mut other = encode(&p);
        other[..8].copy_from_slice(&2.0_f64.to_le_bytes());
        assert_eq!(parse(&other), None, "a layout this build does not know");
        let mut off_earth = encode(&p);
        off_earth[3 * 8..4 * 8].copy_from_slice(&91.0_f64.to_le_bytes());
        assert_eq!(parse(&off_earth), None);
    }

    #[tokio::test]
    async fn the_snapshot_is_read_when_the_plugin_has_it() {
        let fake = FakeXPlane::new();
        split_reads(&fake);
        fake.set_read_bytes(POSITION, Some(&encode(&sample())));
        let p = read(&fake).await.unwrap();
        assert_eq!(p, sample());
        let (lat, lon, src) = read_lat_lon(&fake).await.unwrap();
        assert_eq!((lat, lon, src), (sample().lat, sample().lon, Source::Plugin));
    }

    /// An older plugin: the stock datarefs, read as they always were, and the
    /// reading says it is split.
    #[tokio::test]
    async fn an_older_plugin_falls_back_to_the_separate_reads() {
        let fake = FakeXPlane::new();
        split_reads(&fake);
        let p = read(&fake).await.unwrap();
        assert_eq!(p.source, Source::Split);
        assert_eq!((p.lat, p.lon, p.psi_true_deg, p.gs_ms), (51.15, -0.19, 77.5, 40.0));
        assert_eq!((p.elev_m, p.vs_fpm, p.agl_m), (60.0, 0.0, 1.9));
        assert_eq!(p.frame, None);
        assert_eq!(read_lat_lon(&fake).await.unwrap(), (51.15, -0.19, Source::Split));
        // A plugin that has not copied a frame yet is the same fallback.
        let mut unready = encode(&sample());
        unready[..8].copy_from_slice(&0.0_f64.to_le_bytes());
        fake.set_read_bytes(POSITION, Some(&unready));
        assert_eq!(read(&fake).await.unwrap().source, Source::Split);
    }

    #[tokio::test]
    async fn a_failed_split_field_is_no_measurement() {
        let fake = FakeXPlane::with_reads(&[(LAT, 51.15)]);
        assert!(read(&fake).await.is_err());
        assert!(read_lat_lon(&fake).await.is_err());
        let r = read(&fake).await;
        let (lat, lon) = lat_lon(&r);
        assert!(lat.is_err() && lon.is_err());
        assert_eq!(source_word(&r), None);
    }
}
