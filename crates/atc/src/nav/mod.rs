//! Navigation-data lookup from X-Plane's `earth_nav.dat`.
//!
//! Parses ILS localizer records (row code 4) to resolve "the ILS for runway N"
//! into a frequency + magnetic course, choosing the one nearest the aircraft.
//!
//! Field encoding (decoded empirically against KSEA, then verified): a row-4
//! line is `4 lat lon elev FREQ range BEARING ident AIRPORT region RUNWAY name`,
//! where FREQ is in 10 kHz units (11075 = 110.75 MHz) and the localizer
//! magnetic course is BEARING / 360 (123840.337 → 344°).

use std::io::{BufRead, BufReader};

use anyhow::{Context, Result};

mod airports;
mod approach;
mod geo;
mod ils;
mod minimums;
mod paint;
mod procedures;
mod runways;
mod sid_star;
mod taxi;

pub use airports::*;
pub use approach::*;
pub use geo::*;
pub use ils::*;
pub use minimums::*;
pub use paint::*;
pub use procedures::*;
pub use runways::*;
pub use sid_star::*;
pub use taxi::*;

/// A path under the real X-Plane install for the ignored real-install
/// tests: `XCOPILOT_XPLANE_ROOT`, else `$HOME/X-Plane 12` (the crew's
/// `config::xplane_root`, which this crate cannot depend on).
#[cfg(test)]
pub(crate) fn test_xplane_path(rel: &str) -> String {
    let root = std::env::var("XCOPILOT_XPLANE_ROOT").unwrap_or_else(|_| {
        format!("{}/X-Plane 12", std::env::var("HOME").unwrap_or_default())
    });
    format!("{root}/{rel}")
}

#[cfg(test)]
mod tests;
