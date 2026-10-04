//! Tuning, the frequency briefing and spoken wind.

use super::*;

/// Which controller COM1 is on, if any — the voice that narrates traffic.
pub fn tuned_controller(com1_khz: i64) -> Option<StationSpeaker> {
    CONTROLLER_FREQS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .iter()
        .find(|(k, _)| (com1_khz - k).abs() <= 5)
        .map(|(_, s)| *s)
}

/// Which station COM1 hears — controllers OR the ATIS loop. Unlike
/// [`tuned_controller`] this resolves the ATIS frequency too, so the speech
/// arbiter can tell "on the ATIS broadcast" from "off frequency" and cut
/// live station audio the instant the pilot tunes away.
pub fn tuned_station(com1_khz: i64) -> Option<StationSpeaker> {
    tuned_controller(com1_khz).or_else(|| {
        ATIS.lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_ref()
            .filter(|a| (com1_khz - a.khz).abs() <= 5)
            .map(|_| StationSpeaker::Atis)
    })
}

/// The information letter in a call ("with information Alpha", "we have
/// information bravo", or just "information B").
pub fn heard_information(t: &str) -> Option<char> {
    let mut toks = t
        .split(|c: char| !c.is_alphanumeric())
        .filter(|w| !w.is_empty());
    while let Some(tok) = toks.next() {
        if tok != "information" {
            continue;
        }
        let next = toks.next()?;
        // A NATO word ("alpha") or a bare letter ("b").
        if next.len() == 1 {
            let c = next.chars().next()?.to_ascii_uppercase();
            if c.is_ascii_uppercase() {
                return Some(c);
            }
        }
        for letter in 'A'..='Z' {
            if atis::nato(letter).to_ascii_lowercase() == next {
                return Some(letter);
            }
        }
        return None;
    }
    None
}

/// Outcome of a named-station tuning request ("tune the tower").
pub enum Tune {
    /// Set COM1 to this kHz; `label` is spoken in the readback.
    Found { label: String, khz: i64 },
    /// The airport has no such frequency — decline by name, don't guess.
    Missing(String),
}

/// Every comm service a field can publish, in briefing order (the order a
/// pilot works them: information first, then the ground-to-air chain).
pub(super) const SERVICES: &[(&str, u32)] = &[
    ("atis", nav::FREQ_ATIS),
    ("clearance", nav::FREQ_CLEARANCE),
    ("ground", nav::FREQ_GROUND),
    ("tower", nav::FREQ_TOWER),
    ("approach", nav::FREQ_APPROACH),
    ("departure", nav::FREQ_DEPARTURE),
    ("unicom", nav::FREQ_UNICOM),
];

/// The field's published services in briefing order, for the dashboard's
/// radio panel: (service name, frequency in kHz). The same rows the spoken
/// briefing reads, so screen and voice can never disagree.
pub fn services(apt: &nav::AirportInfo) -> Vec<(String, i64)> {
    SERVICES
        .iter()
        .filter_map(|(word, kind)| apt.freq(*kind).map(|f| (word.to_string(), f.khz)))
        .collect()
}

/// Is this a "what frequencies are here" / "who can I talk to" question?
/// Crew knowledge, not a radio call — checked after the tuning verbs so
/// "tune the tower" still tunes. Deterministic on purpose: the answer is a
/// list read from apt.dat, exactly the kind of question the LLM must never
/// improvise.
pub fn freq_query(t: &str) -> bool {
    let interrogative = t.contains("what")
        || t.contains("which")
        || t.contains("list")
        || t.contains("any")
        || t.contains("available");
    let subject = t.contains("frequenc") || t.contains("station");
    let talk =
        t.contains("who") && (t.contains("talk") || t.contains("call") || t.contains("contact"));
    (interrogative && subject) || talk
}

impl Station {
    /// The spoken frequency briefing: every service this field publishes, plus
    /// what COM1 is on right now (named when it matches a service). Pure —
    /// the caller reads COM1 (see [`Station::freq_briefing`]) so this tests
    /// without a simulator.
    pub fn briefing_text(&self, com1_khz: Option<i64>) -> String {
        let parts: Vec<String> = SERVICES
            .iter()
            .filter_map(|(word, kind)| {
                self.apt
                    .freq(*kind)
                    .map(|f| format!("{word} {}", atis::spoken_mhz(f.khz)))
            })
            .collect();
        let mut line = if parts.is_empty() {
            format!(
                "{} has no published frequencies.",
                spoken_airport(&self.apt.name)
            )
        } else {
            format!("{}: {}.", spoken_airport(&self.apt.name), parts.join(", "))
        };
        if let Some(com1) = com1_khz {
            // Same 5 kHz tolerance as tuned_service: 8.33 channel names can
            // sit 5 kHz off the carrier. Some fields publish ground and tower
            // on the SAME frequency (8 in the 2026-07 apt.dat, military
            // mostly) — name the service that would actually answer, in
            // tuned_service's precedence, not the first row in briefing order.
            let matches: Vec<&str> = SERVICES
                .iter()
                .filter_map(|(word, kind)| {
                    self.apt
                        .freq(*kind)
                        .filter(|f| (f.khz - com1).abs() <= 5)
                        .map(|_| *word)
                })
                .collect();
            let tuned = matches
                .iter()
                .find(|w| matches!(**w, "tower" | "approach" | "departure"))
                .or_else(|| matches.first())
                .copied();
            line.push_str(&match tuned {
                Some(w) => format!(" Com one is on the {w}."),
                None => format!(" Com one is on {}.", atis::spoken_mhz(com1)),
            });
        }
        line
    }

    /// [`Station::briefing_text`] with COM1 read live. A failed read just
    /// drops the "com one is on" sentence — the list still answers.
    pub async fn freq_briefing(&self, xp: SimView<'_>) -> String {
        let com1 = xp
            .get_float(datarefs::COM1_FREQ_KHZ)
            .await
            .ok()
            .map(|v| v.round() as i64);
        self.briefing_text(com1)
    }

    /// The crew's note after a keyed call nobody answered while this station
    /// is in range. The full directory ([`Station::briefing_text`]) answers
    /// the QUESTION "what frequencies are there" — after dead air the only
    /// useful fact is who WILL answer (all six services recited after one
    /// void call on ground read as noise, 2026-07-20). Only tower, approach
    /// and departure are ever manned here; a COM1 sitting on a published but
    /// unmanned row (ground, clearance, ATIS, unicom) gets told so by name.
    /// Pure — the caller reads COM1 and the air/ground state.
    pub fn void_note(&self, com1_khz: Option<i64>, airborne: bool) -> String {
        let (word, khz) = match self.approach_khz.filter(|_| airborne) {
            Some(k) => ("approach", k),
            None => ("tower", self.tower_khz),
        };
        let apt = spoken_airport(&self.apt.name);
        let tuned = com1_khz.and_then(|c| {
            SERVICES.iter().find_map(|(w, kind)| {
                self.apt
                    .freq(*kind)
                    .filter(|f| (f.khz - c).abs() <= 5)
                    .map(|_| *w)
            })
        });
        match tuned {
            Some("atis") => format!(
                "That's the ATIS, it only broadcasts. Call {apt} {word}, {}.",
                atis::spoken_mhz(khz)
            ),
            Some(w) if w != word => format!(
                "Nobody's manning {w} here. Call {apt} {word}, {}.",
                atis::spoken_mhz(khz)
            ),
            _ => format!("Try {apt} {word} on {}.", atis::spoken_mhz(khz)),
        }
    }

    /// Deterministic named-station tuning: "tune/select/switch to/contact the
    /// tower|atis|ground|approach|departure" (or generic "atc"/"station" =
    /// tower) resolved against this airport's apt.dat frequencies. Requires a
    /// tuning verb, so radio chatter that merely mentions a station never
    /// re-tunes the radio. `None` = not a tuning request at all.
    pub fn tune_request(&self, t: &str) -> Option<Tune> {
        // STT warts for station names (all live 2026-07-20): ATIS spelled
        // letter-by-letter or heard as "atheist", tower heard as "tau".
        // Nobody asks for any of these otherwise.
        let t = &t
            .replace("a t i s", "atis")
            .replace("atheist", "atis")
            .replace("tau frequency", "tower frequency");
        // Flight-parameter talk that happens to name a station ("change to
        // approach speed", "select approach flaps") is never a tuning request.
        if t.contains("speed")
            || t.contains("knots")
            || t.contains("flaps")
            || t.contains("heading")
            || t.contains("altitude")
        {
            return None;
        }
        let verb = t.contains("tune")
            || t.contains("switch to")
            || t.contains("select")
            || t.contains("contact")
            || t.contains("change to")
            || (t.contains("set") && t.contains("com"))
            // "change/switch frequency to the tower" — but only when the
            // sentence is about the radio: bare "change"/"switch" are too
            // promiscuous ("switch the landing light").
            || ((t.contains("change") || t.contains("switch"))
                && (t.contains("freq") || t.contains("com") || t.contains("radio")));
        // A short bare "<station> frequency" is a tuning request even with
        // the verb clipped by STT ("To tower frequency." lost its "change",
        // live 2026-07-20). Short only: a real readback quoting a frequency
        // carries digits and runs longer.
        let bare = t.contains("frequency") && t.split_whitespace().count() <= 4;
        if !verb && !bare {
            return None;
        }
        let named = SERVICES.iter().find(|(w, _)| t.contains(w));
        // Generic "tune atc" / "tune the station" means the tower.
        let (word, kind) = match named {
            Some((w, k)) => (*w, *k),
            None if t.contains("atc") || t.contains("station") => ("tower", nav::FREQ_TOWER),
            None => return None,
        };
        Some(match self.apt.freq(kind) {
            Some(f) => Tune::Found {
                label: format!("{} {}", spoken_airport(&self.apt.name), word),
                khz: f.khz,
            },
            None => Tune::Missing(format!(
                "{} has no {} frequency.",
                spoken_airport(&self.apt.name),
                word
            )),
        })
    }
}

/// Wind spoken tower-style ("zero niner zero degrees, three knots" / "calm").
pub(super) fn spoken_wind(from_deg: f64, kts: f64) -> String {
    if kts < 3.0 {
        return "calm".to_string();
    }
    let dir = ((from_deg / 10.0).round() as i64 * 10).rem_euclid(360);
    let dir = if dir == 0 { 360 } else { dir };
    format!(
        "{} degrees, {} knots",
        atis::spoken_digits(&format!("{dir:03}")),
        atis::spoken_digits(&format!("{kts:.0}"))
    )
}
