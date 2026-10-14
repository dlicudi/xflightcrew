//! Who speaks with what voice, for the CURRENT configuration: the copilot,
//! the cabin, and every station position at the home field or at any other
//! field. The casting code the radio thread runs is the truth
//! ([`super::station_model`] over the installed pool, then
//! [`super::loadable_station_model`]); this module calls it and says, per
//! row, where the value came from. Pure over its inputs ([`cast`]); the
//! live view is [`cast_now`].

use crate::config::Config;

use super::StationSpeaker;

/// Where a role's voice came from.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Source {
    /// An environment variable (or a `.env` line) pins it.
    Env(&'static str),
    /// The pilot's saved pick (prefs.json).
    Pref,
    /// The built-in voice.
    Default,
    /// Another role's voice, as designed (the ATIS reads in the tower's,
    /// ground and delivery sit in the tower's building).
    Inherit(&'static str),
    /// Drawn from the away-field pool by the field's seed.
    Pool,
    /// The country's own accent (`voice::local_accents_for`).
    Accent,
}

impl Source {
    pub fn describe(&self) -> String {
        match self {
            Source::Env(e) => format!("env {e}"),
            Source::Pref => "your pick".into(),
            Source::Default => "built-in".into(),
            Source::Inherit(who) => format!("the {who}'s"),
            Source::Pool => "away-field pool".into(),
            Source::Accent => "local accent".into(),
        }
    }
}

/// Which field the controllers are cast at.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Field<'a> {
    /// The configured home field: the curated per-position voices.
    Home,
    /// Any other field, by ICAO and its apt.dat name (the seed is the
    /// name's, as `station_work` casts it).
    Away { icao: &'a str, name: &'a str },
}

/// One role's voice.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Row {
    /// "copilot", "cabin", "tower", "ground", "delivery", "approach",
    /// "departure", "ATIS".
    pub role: &'static str,
    /// The registry setting that picks it, where one does (`None` for an
    /// away field's controller: the pool or the accent decides).
    pub setting: Option<&'static str>,
    /// The voice the cast picks (`path.onnx[#N]`; empty = the `say`
    /// fallback).
    pub spec: String,
    pub installed: bool,
    /// What actually loads when the pick is not installed (the tower's
    /// voice, then the copilot's, then any on disk), or `None`.
    pub instead: Option<String>,
    pub source: Source,
}

/// The station positions, in the order a cast is shown.
pub const POSITIONS: [(StationSpeaker, &str); 6] = [
    (StationSpeaker::Tower, "tower"),
    (StationSpeaker::Ground, "ground"),
    (StationSpeaker::Delivery, "delivery"),
    (StationSpeaker::Approach, "approach"),
    (StationSpeaker::Departure, "departure"),
    (StationSpeaker::Atis, "ATIS"),
];

/// The cast, pure: `source_of` says where a slot's variable got its value
/// (env, pref or default), `installed` whether a spec is on disk, `locals`
/// the field's accent voices on disk, `on_disk` every voice there (asked
/// only for a missing pick's stand-in).
pub fn cast(
    config: &Config,
    field: Field,
    source_of: impl Fn(&'static str) -> Source,
    installed: impl Fn(&str) -> bool + Copy,
    locals: &[String],
    on_disk: impl Fn() -> Vec<String>,
) -> Vec<Row> {
    let mut rows = Vec::new();
    let persona = |role, setting, env, spec: &str| Row {
        role,
        setting: Some(setting),
        spec: spec.to_string(),
        installed: installed(spec),
        instead: None,
        source: source_of(env),
    };
    rows.push(persona(
        "copilot",
        "copilot_voice",
        "PIPER_VOICE",
        &config.piper_voice,
    ));
    rows.push(persona(
        "cabin",
        "cabin_voice",
        "XCOPILOT_CABIN_VOICE",
        &config.cabin_piper_voice,
    ));
    // The radio thread's own view of the config: the pool drawn from what
    // is installed (`radio_loop`).
    let mut station_cfg = config.clone();
    station_cfg.atc_voice_pool = super::installed_pool(&config.atc_voice_pool, installed);
    let (seed, locals) = match field {
        Field::Home => (0, &[][..]),
        Field::Away { name, .. } => (super::station_voice_seed(name), locals),
    };
    let pool_drawn = seed != 0 && !station_cfg.atc_voice_pool.is_empty();
    for (who, role) in POSITIONS {
        let spec = super::station_model(who, &station_cfg, seed, locals);
        let mut said = std::collections::HashSet::new();
        let loads =
            super::loadable_station_model(&spec, &station_cfg, installed, &on_disk, &mut said);
        let accent = seed != 0
            && !locals.is_empty()
            && matches!(who, StationSpeaker::Tower | StationSpeaker::Atis);
        let (setting, source) = if accent {
            (None, Source::Accent)
        } else if pool_drawn {
            (None, Source::Pool)
        } else {
            home_source(who, config, &source_of)
        };
        rows.push(Row {
            role,
            setting: if seed == 0 { setting } else { None },
            installed: spec.is_empty() || installed(&spec),
            instead: (loads != spec).then_some(loads),
            spec,
            source,
        });
    }
    rows
}

/// A home position's setting and where its voice came from: its own slot,
/// or the tower's when it inherits (ground and delivery always, approach,
/// departure and the ATIS when their slot is blank).
fn home_source(
    who: StationSpeaker,
    config: &Config,
    source_of: &impl Fn(&'static str) -> Source,
) -> (Option<&'static str>, Source) {
    let own = |key, env, value: &str| {
        if value.is_empty() {
            (Some(key), Source::Inherit("tower"))
        } else {
            (Some(key), source_of(env))
        }
    };
    match who {
        StationSpeaker::Approach => own(
            "atc_approach_voice",
            "XCOPILOT_ATC_APPROACH_PIPER_VOICE",
            &config.atc_approach_piper_voice,
        ),
        StationSpeaker::Departure => own(
            "atc_departure_voice",
            "XCOPILOT_ATC_DEPARTURE_PIPER_VOICE",
            &config.atc_departure_piper_voice,
        ),
        StationSpeaker::Atis => own(
            "atc_atis_voice",
            "XCOPILOT_ATC_ATIS_PIPER_VOICE",
            &config.atc_atis_piper_voice,
        ),
        StationSpeaker::Ground | StationSpeaker::Delivery => {
            (Some("atc_tower_voice"), Source::Inherit("tower"))
        }
        _ => (
            Some("atc_tower_voice"),
            source_of("XCOPILOT_ATC_PIPER_VOICE"),
        ),
    }
}

/// Where a voice slot's value comes from in this process: its variable
/// (pinning, or for the copilot's and cabin's only a default their pick
/// beats), its saved pref, or the built-in.
pub fn source_now(env: &'static str) -> Source {
    let picked = match env {
        "PIPER_VOICE" => crate::prefs::pilot_voice().is_some(),
        "XCOPILOT_CABIN_VOICE" => crate::prefs::cabin_voice().is_some(),
        _ => false,
    };
    if picked {
        return Source::Pref;
    }
    if std::env::var(env).is_ok() {
        return Source::Env(env);
    }
    let saved = crate::config::BOOT_KNOBS
        .iter()
        .find(|(e, _)| *e == env)
        .and_then(|(_, key)| crate::prefs::setting(key))
        .is_some();
    if saved {
        Source::Pref
    } else {
        Source::Default
    }
}

/// The cast for the live configuration (`settings::live`), at the home
/// field or at `icao` (its name from apt.dat, so the seed is the one the
/// radio uses; a field apt.dat does not know is cast by its ICAO).
pub fn cast_now(at: Option<&str>) -> Vec<Row> {
    let config = crate::settings::live();
    match at {
        None => cast(
            &config,
            Field::Home,
            source_now,
            super::voice_installed,
            &[],
            super::pilot_voice_choices,
        ),
        Some(icao) => {
            let icao = icao.trim().to_ascii_uppercase();
            let apt = if config.apt_data_path.is_empty() {
                crate::config::apt_dat_path()
            } else {
                config.apt_data_path.clone()
            };
            let name = crate::nav::find_airport(&apt, &icao)
                .ok()
                .flatten()
                .map(|a| a.name)
                .unwrap_or_else(|| icao.clone());
            let locals = super::local_voices_on_disk(&icao);
            cast(
                &config,
                Field::Away {
                    icao: &icao,
                    name: &name,
                },
                source_now,
                super::voice_installed,
                &locals,
                super::pilot_voice_choices,
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> Config {
        Config {
            piper_voice: "m/copilot.onnx".into(),
            cabin_piper_voice: "m/cabin.onnx".into(),
            atc_piper_voice: "m/tower.onnx".into(),
            atc_approach_piper_voice: "m/approach.onnx".into(),
            atc_departure_piper_voice: String::new(),
            atc_atis_piper_voice: String::new(),
            atc_voice_pool: "m/p1.onnx,m/p2.onnx,m/gone.onnx,m/p3.onnx,m/p4.onnx".into(),
            ..Config::default()
        }
    }

    fn installed(spec: &str) -> bool {
        !spec.contains("gone") && !spec.contains("missing")
    }

    fn row<'a>(rows: &'a [Row], role: &str) -> &'a Row {
        rows.iter().find(|r| r.role == role).expect(role)
    }

    /// At home every position wears its curated slot or inherits the
    /// tower's, said as such, and names the setting that picks it.
    #[test]
    fn the_home_cast_is_the_curated_slots() {
        let src = |e: &'static str| {
            if e == "XCOPILOT_ATC_PIPER_VOICE" {
                Source::Env(e)
            } else {
                Source::Default
            }
        };
        let rows = cast(&cfg(), Field::Home, src, installed, &[], Vec::new);
        assert_eq!(rows.len(), 8);
        let twr = row(&rows, "tower");
        assert_eq!(twr.spec, "m/tower.onnx");
        assert_eq!(twr.source, Source::Env("XCOPILOT_ATC_PIPER_VOICE"));
        assert_eq!(twr.setting, Some("atc_tower_voice"));
        assert_eq!(row(&rows, "ground").source, Source::Inherit("tower"));
        assert_eq!(row(&rows, "approach").spec, "m/approach.onnx");
        let dep = row(&rows, "departure");
        assert_eq!(
            (dep.spec.as_str(), &dep.source),
            ("m/tower.onnx", &Source::Inherit("tower"))
        );
        assert_eq!(dep.setting, Some("atc_departure_voice"));
        assert_eq!(row(&rows, "ATIS").spec, "m/tower.onnx");
        assert_eq!(row(&rows, "copilot").setting, Some("copilot_voice"));
    }

    /// Away, the cast is the radio's own: the installed pool by the field's
    /// seed (a missing pool voice is never drawn), the accent at the tower
    /// and ATIS where the country has one; no setting picks those rows.
    #[test]
    fn an_away_field_is_cast_as_the_radio_casts_it() {
        let away = Field::Away {
            icao: "EGHI",
            name: "Southampton",
        };
        let rows = cast(&cfg(), away, |_| Source::Default, installed, &[], Vec::new);
        for (who, role) in POSITIONS {
            let r = row(&rows, role);
            assert!(!r.spec.contains("gone"), "{role}: {}", r.spec);
            assert_eq!(r.source, Source::Pool, "{role}");
            assert_eq!(r.setting, None, "{role}");
            let mut c = cfg();
            c.atc_voice_pool = "m/p1.onnx,m/p2.onnx,m/p3.onnx,m/p4.onnx".into();
            let seed = super::super::station_voice_seed("Southampton");
            assert_eq!(
                r.spec,
                super::super::station_model(who, &c, seed, &[]),
                "{role}"
            );
        }
        let locals = vec!["m/es_ES-davefx-medium.onnx".to_string()];
        let rows = cast(
            &cfg(),
            Field::Away {
                icao: "LEMG",
                name: "Malaga",
            },
            |_| Source::Default,
            installed,
            &locals,
            Vec::new,
        );
        assert_eq!(row(&rows, "tower").source, Source::Accent);
        assert_eq!(row(&rows, "tower").spec, locals[0]);
        assert_eq!(row(&rows, "approach").source, Source::Pool);
    }

    /// A pick that is not installed is shown as missing, with the voice
    /// that loads instead (the tower's, by `loadable_station_model`).
    #[test]
    fn a_missing_voice_is_marked_with_what_loads_instead() {
        let mut c = cfg();
        c.atc_approach_piper_voice = "m/missing.onnx".into();
        c.piper_voice = "m/missing-copilot.onnx".into();
        let rows = cast(
            &c,
            Field::Home,
            |_| Source::Default,
            installed,
            &[],
            Vec::new,
        );
        let app = row(&rows, "approach");
        assert!(!app.installed);
        assert_eq!(app.instead.as_deref(), Some("m/tower.onnx"));
        assert!(!row(&rows, "copilot").installed);
        assert!(row(&rows, "tower").installed && row(&rows, "tower").instead.is_none());
    }
}
