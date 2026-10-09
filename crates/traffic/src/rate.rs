//! How busy a field is, and when: the generated schedule's movement rate
//! ([`peak_per_h`]) from what apt.dat says about the field, shaped through
//! the day by the field's LOCAL SOLAR time ([`day_factor`]) from the sim's
//! zulu time, which the bin passes in ([`crate::SpawnCtx::zulu_s`]); this
//! crate never reads the sim.
//!
//! Until 2026-10-09 every field flew one departure and one arrival every
//! ten minutes, 12 movements an hour at Gatwick, Heathrow and Malaga alike,
//! and 03:00 looked like noon. Real peaks: Gatwick 50 and more on one
//! runway, Heathrow about 80 on two, Malaga about 25, a small regional a
//! handful, and the night is quiet.

use crate::field::{stand_fits, FieldModel};
use crate::schedule::{REGIONAL, SMALL_REGIONAL};

/// Airline movements an hour at a towered field before its stands are
/// counted, and at an untowered one: a regional's few rotations.
pub const BASE_TOWERED_PER_H: f64 = 5.0;
pub const BASE_UNTOWERED_PER_H: f64 = 1.0;

/// Airline movements an hour each airline stand ([`airline_stands`]) adds
/// at the peak. Fitted on the install's apt.dat (2026-10-09): EGKK's 189
/// give 47 (real 50 to 55 on one runway), EGLL's 282 give 67 (real about
/// 80 on two; the one runway here caps it lower), LEMG's 82 give 23 (real
/// about 25), EGHI's 11 give 7 (Southampton, a few rotations an hour).
pub const PER_AIRLINE_STAND_PER_H: f64 = 0.22;

/// The runway in use sequences one arrival in this many seconds at best,
/// on average, and one departure between each pair: every movement here
/// shares ONE runway (the schedule never splits the field's runways), so
/// one runway's mixed mode is the ceiling whatever the stands say. 100 s is
/// inside the 90 to 120 s a real single runway lands at.
pub const ARRIVAL_GAP_S: f64 = 100.0;
pub const DEPARTURE_GAP_S: f64 = 100.0;

/// The closest two movements of one kind are ever scheduled, seconds: the
/// jitter on the gap never brings two arrivals closer than a runway lands.
pub const MIN_GAP_S: f64 = 90.0;

/// The most of each kind an hour: [`ARRIVAL_GAP_S`], [`DEPARTURE_GAP_S`].
pub fn arrivals_cap_per_h() -> f64 {
    3600.0 / ARRIVAL_GAP_S
}
pub fn departures_cap_per_h() -> f64 {
    3600.0 / DEPARTURE_GAP_S
}

/// The field's airline stands: those an A320 or the ATR fits
/// ([`stand_fits`]), else, where none does, those the small regional fits:
/// a light aircraft's code B stand that also names turboprops is an
/// airline stand only where nothing bigger parks (UNSS, ULKK).
pub fn airline_stands(field: &FieldModel) -> usize {
    let count = |types: &[&str]| {
        field
            .net
            .stands
            .iter()
            .filter(|s| types.iter().any(|t| stand_fits(s, t)))
            .count()
    };
    match count(&["A320", REGIONAL]) {
        0 => count(&[SMALL_REGIONAL]),
        n => n,
    }
}

/// Airline movements (departures and arrivals together) an hour at the
/// field's peak, before the density and the time of day: the base for a
/// towered or an untowered field plus [`PER_AIRLINE_STAND_PER_H`] an
/// airline stand; none where no airliner has a stand. The runway caps it
/// later ([`arrivals_cap_per_h`]).
pub fn peak_per_h(field: &FieldModel) -> f64 {
    let stands = airline_stands(field);
    if stands == 0 {
        return 0.0;
    }
    let base = if field.airport.staffed_tower().is_some() {
        BASE_TOWERED_PER_H
    } else {
        BASE_UNTOWERED_PER_H
    };
    base + PER_AIRLINE_STAND_PER_H * stands as f64
}

/// The field's longitude, degrees east: its reference point's, else 0
/// (Greenwich, whose solar time is zulu).
pub fn field_lon(field: &FieldModel) -> f64 {
    field.reference().map_or(0.0, |p| p.1)
}

/// Local solar time, hours 0 to 24, at longitude `lon` at `zulu_s` seconds
/// after midnight zulu (any number of days on).
pub fn local_solar_h(zulu_s: f64, lon: f64) -> f64 {
    (zulu_s / 3600.0 + lon / 15.0).rem_euclid(24.0)
}

/// The zulu seconds at which it is `local_h` solar time at `lon`.
pub fn zulu_for_local(local_h: f64, lon: f64) -> f64 {
    ((local_h - lon / 15.0) * 3600.0).rem_euclid(86_400.0)
}

/// The day's shape, local solar hour by hour (the value at the top of each
/// hour, straight lines between): near nothing from midnight to five, the
/// morning bank at the peak from six to nine, the midday plateau at 70%,
/// the evening bank from five to nine, falling away after ten.
const DAY: [f64; 24] = [
    0.03, 0.02, 0.02, 0.02, 0.03, 0.20, 0.85, 1.00, 1.00, 0.90, 0.75, 0.70, //
    0.70, 0.70, 0.70, 0.70, 0.80, 0.95, 1.00, 1.00, 0.95, 0.85, 0.55, 0.20,
];

/// The share of the peak flown at local solar hour `h` ([`DAY`]).
pub fn day_factor(h: f64) -> f64 {
    let h = h.rem_euclid(24.0);
    let i = h.floor() as usize % 24;
    let f = h - h.floor();
    DAY[i] + (DAY[(i + 1) % 24] - DAY[i]) * f
}

/// The share of the hour's movements that are departures at local solar
/// hour `h`: the first wave goes out in the morning (the aircraft night
/// stopped at their base), the last comes home late in the evening.
pub fn departure_share(h: f64) -> f64 {
    let h = h.rem_euclid(24.0);
    if (5.0..10.0).contains(&h) {
        0.65
    } else if h >= 21.0 || h < 2.0 {
        0.35
    } else {
        0.5
    }
}

/// Circuits are flown in daylight only: a school's day, roughly.
pub fn circuit_hours(h: f64) -> bool {
    (7.0..20.0).contains(&h.rem_euclid(24.0))
}

/// Departures and arrivals an hour at `field` at `t_s` seconds after a
/// schedule starting at `zulu_s` (`None`: no clock, the field's peak, a
/// 50/50 split), times `scale` (the density), each capped by the runway.
pub fn rates_at(field: &FieldModel, scale: f64, zulu_s: Option<f64>, t_s: f64) -> (f64, f64) {
    let peak = peak_per_h(field) * scale;
    let (factor, dep) = match zulu_s {
        Some(z) => {
            let h = local_solar_h(z + t_s, field_lon(field));
            (day_factor(h), departure_share(h))
        }
        None => (1.0, 0.5),
    };
    let total = peak * factor;
    (
        (total * dep).min(departures_cap_per_h()),
        (total * (1.0 - dep)).min(arrivals_cap_per_h()),
    )
}

/// Times in `0..horizon_s` for a stream whose rate an hour is `rate(t)`:
/// the expected count rises with the rate's integral and a movement falls
/// each time it passes the next threshold (each 0.8 to 1.2 of one, drawn
/// from `draw`), never within [`MIN_GAP_S`] of the last, and none before
/// `start_s`. `phase` (0 to 1) is how much of the first interval had passed
/// at `start_s`. A rate that falls to nothing (the night) simply schedules
/// nothing.
pub fn stream_times(
    horizon_s: f64,
    start_s: f64,
    phase: f64,
    mut rate: impl FnMut(f64) -> f64,
    mut draw: impl FnMut() -> f64,
) -> Vec<f64> {
    const STEP_S: f64 = 5.0;
    let mut out: Vec<f64> = Vec::new();
    let mut need = draw();
    let mut have = phase.clamp(0.0, 1.0) * need;
    let mut t = start_s;
    while t < horizon_s {
        have += rate(t).max(0.0) / 3600.0 * STEP_S;
        t += STEP_S;
        if have >= need && t < horizon_s {
            let at = out.last().map_or(t, |l: &f64| t.max(l + MIN_GAP_S));
            if at >= horizon_s {
                break;
            }
            out.push(at);
            t = at;
            have -= need;
            need = draw();
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_day_is_quiet_at_night_and_banks_morning_and_evening() {
        assert!(day_factor(3.0) < 0.05);
        assert_eq!(day_factor(7.5), 1.0);
        assert_eq!(day_factor(13.0), 0.7);
        assert_eq!(day_factor(18.5), 1.0);
        assert!(day_factor(23.5) < 0.2);
        assert!((day_factor(5.5) - 0.525).abs() < 1e-9, "between the hours");
        assert!(departure_share(7.0) > 0.5 && departure_share(22.0) < 0.5);
    }

    #[test]
    fn local_solar_time_is_zulu_plus_the_longitude() {
        assert_eq!(local_solar_h(12.0 * 3600.0, 0.0), 12.0);
        assert!((local_solar_h(12.0 * 3600.0, -73.77) - 7.082).abs() < 0.01);
        assert!((local_solar_h(23.0 * 3600.0, 30.0) - 1.0).abs() < 1e-9);
        let z = zulu_for_local(8.0, -4.51);
        assert!((local_solar_h(z, -4.51) - 8.0).abs() < 1e-9);
    }

    #[test]
    fn a_stream_flies_its_rate_and_never_closer_than_the_runway() {
        let mut k = 0u32;
        let mut draw = || {
            k += 1;
            0.8 + 0.4 * f64::from(k % 5) / 4.0
        };
        let ts = stream_times(3600.0, 0.0, 0.5, |_| 30.0, &mut draw);
        assert!((28..=32).contains(&ts.len()), "{}", ts.len());
        assert!(ts.windows(2).all(|w| w[1] - w[0] >= MIN_GAP_S - 1e-9));
        let fast = stream_times(3600.0, 0.0, 0.5, |_| 200.0, || 1.0);
        assert!(fast.len() <= 41, "the gap holds: {}", fast.len());
        assert!(stream_times(3600.0, 0.0, 0.5, |_| 0.05, || 1.0).is_empty());
    }
}
