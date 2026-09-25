//! What this airframe's systems ARE, surveyed once per airframe at detection:
//! the autopilot's type (and so which modes it can fly), the avionics class,
//! pressurisation and bleed. The copilot reads this before it reaches for a
//! facility, instead of assuming every aircraft is a 172 with a heading bug.
//!
//! The pilot's rule (2026-09-25): the copilot should understand the
//! aircraft's systems, above all when it ASSISTS a pilot who is flying ("set
//! up the ILS"). The spoken ILS setup had fired APPR blind on every airframe:
//! a toggle, so a second ask disarmed it, with no readback, on autopilots
//! that fly the localiser only.
//!
//! Two sources, both the sim's own:
//! - the `.acf` (`from_acf`): avionics the author declared (glass HSI, TCAS,
//!   auto pressurisation, APU bleed, radio-altimeter callouts);
//! - the live `sim/aircraft/autopilot/preconfigured_ap_type`: the autopilot
//!   model, whose enum names which modes exist.
//!
//! A survey is a set of FACTS for the tools to consult, never a licence: a
//! facility is still read back when used (an X-Plane "custom" autopilot is
//! an unknown, and the readback is the answer). Published per airframe,
//! reset to empty on every swap (`flight_state::publish_airframe`).

use serde_json::json;

/// The live dataref naming the autopilot model.
pub const AP_TYPE: &str = "sim/aircraft/autopilot/preconfigured_ap_type";

/// X-Plane's preconfigured autopilot models (the dataref's enum).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ApType {
    /// Plane Maker "custom": modes set per aircraft, unknown until used.
    Custom,
    Airliner,
    Gfc700,
    Stec55,
    Stec55AltPreselect,
    Kap140SingleAxis,
    Kap140DualAxis,
    Kap140DualAxisAltPreselect,
    PiperAutocontrol,
    CollinsFcs65,
}

impl ApType {
    pub fn from_code(code: i64) -> Option<Self> {
        Some(match code {
            0 => Self::Custom,
            1 => Self::Airliner,
            2 => Self::Gfc700,
            3 => Self::Stec55,
            4 => Self::Stec55AltPreselect,
            5 => Self::Kap140SingleAxis,
            6 => Self::Kap140DualAxis,
            7 => Self::Kap140DualAxisAltPreselect,
            8 => Self::PiperAutocontrol,
            9 => Self::CollinsFcs65,
            _ => return None,
        })
    }

    /// Spoken name.
    pub fn name(self) -> &'static str {
        match self {
            Self::Custom => "a custom autopilot",
            Self::Airliner => "an airliner autopilot",
            Self::Gfc700 => "a Garmin GFC 700",
            Self::Stec55 => "an S-Tec 55",
            Self::Stec55AltPreselect => "an S-Tec 55 with altitude preselect",
            Self::Kap140SingleAxis => "a single-axis KAP 140",
            Self::Kap140DualAxis => "a two-axis KAP 140",
            Self::Kap140DualAxisAltPreselect => "a two-axis KAP 140 with altitude preselect",
            Self::PiperAutocontrol => "a Piper Autocontrol",
            Self::CollinsFcs65 => "a Collins FCS 65",
        }
    }

    /// Can this autopilot track a localiser? `None` = unknown (custom).
    pub fn tracks_localizer(self) -> Option<bool> {
        match self {
            Self::Custom => None,
            // A wing leveller with a heading knob.
            Self::PiperAutocontrol => Some(false),
            _ => Some(true),
        }
    }

    /// Can this autopilot fly a glideslope? A single-axis unit has no pitch
    /// servo at all. `None` = unknown (custom).
    pub fn flies_glideslope(self) -> Option<bool> {
        match self {
            Self::Custom => None,
            Self::Kap140SingleAxis | Self::PiperAutocontrol => Some(false),
            _ => Some(true),
        }
    }
}

/// One airframe's systems.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Systems {
    /// A survey has been taken for the airframe loaded (false = nothing known).
    pub surveyed: bool,
    pub ap_type: Option<ApType>,
    /// The raw `.acf` `_autopilot_logic`, for the log only (its enum is not
    /// the dataref's: the 737 reads 1, the A330 12, the 172 3).
    pub acf_autopilot_logic: Option<i64>,
    pub glass_hsi: bool,
    pub tcas: bool,
    pub auto_pressurization: bool,
    pub apu_bleed: bool,
    pub radio_alt_callouts: bool,
}

impl Systems {
    /// The `.acf` half of the survey. Pure.
    pub fn from_acf(text: &str) -> Self {
        let mut s = Self {
            surveyed: true,
            ..Self::default()
        };
        for l in text.lines() {
            let mut it = l.split_whitespace();
            if it.next() != Some("P") {
                continue;
            }
            let (Some(k), Some(v)) = (it.next(), it.next()) else {
                continue;
            };
            let on = v.parse::<f64>().is_ok_and(|x| x > 0.5);
            match k {
                "acf/_autopilot_logic" => {
                    s.acf_autopilot_logic = v.parse::<f64>().ok().map(|x| x as i64)
                }
                "acf/_has_GLASS_hsi" => s.glass_hsi = on,
                "acf/_has_TCAS" => s.tcas = on,
                "acf/_auto_pressurization" => s.auto_pressurization = on,
                "acf/_has_APU_bleed_air" => s.apu_bleed = on,
                "acf/_has_radio_alt_callout" => s.radio_alt_callouts = on,
                _ => {}
            }
        }
        s
    }

    /// Transport-category avionics: the panel has a radio altimeter with a
    /// decision-height bug worth setting. An airliner autopilot, or TCAS in
    /// a pressurised cabin: the 172 G1000's `.acf` declares TCAS and radio
    /// altimeter callouts (LIPE, 2026-09-25) and has no radio altimeter.
    pub fn transport_avionics(&self) -> bool {
        self.ap_type == Some(ApType::Airliner) || (self.tcas && self.auto_pressurization)
    }

    /// The spoken answer to "what systems / autopilot have we got".
    pub fn line(&self) -> String {
        if !self.surveyed {
            return "I haven't surveyed this airframe's systems yet.".to_string();
        }
        let mut parts: Vec<String> = Vec::new();
        match self.ap_type {
            Some(ap) => {
                let mut s = format!("We've got {}", ap.name());
                match (ap.tracks_localizer(), ap.flies_glideslope()) {
                    (Some(true), Some(true)) => s.push_str(", it'll fly a coupled ILS"),
                    (Some(true), Some(false)) => {
                        s.push_str(", localiser only, the glideslope is hand flown")
                    }
                    (Some(false), _) => s.push_str(", heading only, no approach mode"),
                    _ => s.push_str(", I'll learn its modes as I use them"),
                }
                parts.push(s);
            }
            None => parts.push("I can't tell what autopilot this is".to_string()),
        }
        let mut kit: Vec<&str> = Vec::new();
        if self.glass_hsi {
            kit.push("glass instruments");
        }
        if self.tcas {
            kit.push("TCAS");
        }
        if self.auto_pressurization {
            kit.push("automatic pressurisation");
        }
        if self.apu_bleed {
            kit.push("an APU with bleed air");
        }
        if !kit.is_empty() {
            parts.push(format!("plus {}", join_and(&kit)));
        }
        format!("{}.", parts.join(". "))
    }

    pub fn blackbox(&self) -> serde_json::Value {
        json!({
            "ap_type": self.ap_type.map(|a| format!("{a:?}")),
            "acf_autopilot_logic": self.acf_autopilot_logic,
            "glass_hsi": self.glass_hsi,
            "tcas": self.tcas,
            "auto_pressurization": self.auto_pressurization,
            "apu_bleed": self.apu_bleed,
            "radio_alt_callouts": self.radio_alt_callouts,
            "transport_avionics": self.transport_avionics(),
        })
    }
}

fn join_and(items: &[&str]) -> String {
    match items {
        [] => String::new(),
        [a] => a.to_string(),
        [rest @ .., last] => format!("{} and {last}", rest.join(", ")),
    }
}

static CURRENT: std::sync::Mutex<Option<Systems>> = std::sync::Mutex::new(None);

/// Publish the survey for the airframe loaded; `None` clears it (every swap).
pub fn publish(s: Option<Systems>) {
    *CURRENT.lock().unwrap_or_else(|e| e.into_inner()) = s;
}

/// The survey for the airframe loaded (empty, `surveyed: false`, before one).
pub fn current() -> Systems {
    CURRENT
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone()
        .unwrap_or_default()
}

/// Is the utterance asking what the aircraft's systems or autopilot are?
pub fn is_systems_query(t: &str) -> bool {
    let asks = t.contains("what autopilot")
        || t.contains("which autopilot")
        || t.contains("what kind of autopilot")
        || t.contains("what type of autopilot")
        || t.contains("what systems")
        || t.contains("which systems")
        || t.contains("what avionics")
        || t.contains("tell me about the autopilot")
        || t.contains("tell me about the systems")
        || t.contains("tell me about the avionics");
    // "can the autopilot fly the ILS / an approach / a coupled approach"
    let can = (t.contains("can the autopilot")
        || t.contains("can it fly")
        || t.contains("will the autopilot"))
        && (t.contains("ils")
            || t.contains("approach")
            || t.contains("glideslope")
            || t.contains("coupled"));
    asks || can
}

#[cfg(test)]
mod tests {
    use super::*;

    const B738: &str = "P acf/_autopilot_logic 1\nP acf/_has_GLASS_hsi 1\nP acf/_has_TCAS 1\n\
P acf/_auto_pressurization 1\nP acf/_has_APU_bleed_air 1\nP acf/_has_radio_alt_callout 1\n";
    const C172: &str = "P acf/_autopilot_logic 3\nP acf/_has_GLASS_hsi 0\nP acf/_has_TCAS 0\n\
P acf/_auto_pressurization 0\nP acf/_has_APU_bleed_air 0\nP acf/_has_radio_alt_callout 0\n";

    #[test]
    fn the_acf_says_what_avionics_the_author_fitted() {
        let b = Systems::from_acf(B738);
        assert!(b.surveyed && b.glass_hsi && b.tcas && b.auto_pressurization && b.apu_bleed);
        assert_eq!(b.acf_autopilot_logic, Some(1));
        assert!(b.transport_avionics());
        let c = Systems::from_acf(C172);
        assert!(
            !c.transport_avionics(),
            "a 172 has no radio altimeter to set"
        );
        assert!(!c.tcas && !c.apu_bleed);
        // The 172 G1000 as flown at LIPE: TCAS and RA callouts declared, no
        // radio altimeter to set.
        let mut g = Systems::from_acf(C172);
        g.tcas = true;
        g.radio_alt_callouts = true;
        g.ap_type = Some(ApType::Gfc700);
        assert!(!g.transport_avionics());
    }

    #[test]
    fn the_autopilot_type_names_which_modes_exist() {
        assert_eq!(ApType::from_code(5), Some(ApType::Kap140SingleAxis));
        assert_eq!(ApType::Kap140SingleAxis.flies_glideslope(), Some(false));
        assert_eq!(ApType::Kap140SingleAxis.tracks_localizer(), Some(true));
        assert_eq!(ApType::PiperAutocontrol.tracks_localizer(), Some(false));
        assert_eq!(ApType::Airliner.flies_glideslope(), Some(true));
        assert_eq!(
            ApType::Custom.flies_glideslope(),
            None,
            "custom is learned by readback"
        );
        assert_eq!(ApType::from_code(42), None);
    }

    #[test]
    fn the_spoken_survey_says_what_the_autopilot_can_fly() {
        let mut s = Systems::from_acf(B738);
        s.ap_type = Some(ApType::Airliner);
        let l = s.line();
        assert!(l.contains("coupled ILS"), "{l}");
        assert!(l.contains("TCAS"), "{l}");
        let mut c = Systems::from_acf(C172);
        c.ap_type = Some(ApType::Kap140SingleAxis);
        assert!(c.line().contains("localiser only"), "{}", c.line());
        assert!(Systems::default().line().contains("haven't surveyed"));
        assert!(
            !l.contains('\u{2014}') && !l.contains('\u{2013}'),
            "no dashes in speech"
        );
    }

    #[test]
    fn systems_questions_are_recognised_and_commands_are_not() {
        assert!(is_systems_query("what autopilot have we got"));
        assert!(is_systems_query("can the autopilot fly the ils"));
        assert!(is_systems_query("what systems does this aircraft have"));
        assert!(!is_systems_query("autopilot on"));
        assert!(!is_systems_query("set up the ils runway 24"));
    }
}
