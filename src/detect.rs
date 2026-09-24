//! The aircraft on the other side of the API: which profile an ICAO type gets,
//! the live profile handle the loops read, the swap-time stand-down, and the
//! generic profile an unknown airframe is built from its own .acf. Split out
//! of main.rs on 2026-09-16.

use super::*;

/// The active aircraft profile, shared between the command loop, the monitor, and
/// the detection poller. The poller swaps it when the sim's aircraft changes;
/// consumers notice via the generation counter and rebuild off the new profile.
pub(crate) struct LiveProfile {
    inner: std::sync::RwLock<aircraft::AircraftProfile>,
    generation: std::sync::atomic::AtomicU64,
}

impl LiveProfile {
    pub(crate) fn new(p: aircraft::AircraftProfile) -> std::sync::Arc<Self> {
        crate::flight_state::publish_airframe(&p);
        std::sync::Arc::new(Self {
            inner: std::sync::RwLock::new(p),
            generation: std::sync::atomic::AtomicU64::new(0),
        })
    }
    /// A clone of the current profile.
    pub(crate) fn get(&self) -> aircraft::AircraftProfile {
        self.inner.read().unwrap().clone()
    }
    pub(crate) fn id(&self) -> String {
        self.inner.read().unwrap().id.clone()
    }
    /// Bumps every time the profile is swapped — consumers compare to know to rebuild.
    pub(crate) fn generation(&self) -> u64 {
        self.generation.load(std::sync::atomic::Ordering::Acquire)
    }
    pub(crate) fn swap(&self, p: aircraft::AircraftProfile) {
        // A spawned controller owns parameters from the profile that armed it.
        // Stop every owner before changing the global actuator selectors;
        // otherwise an old takeoff/autoland loop can start driving the newly
        // loaded aircraft with the previous airframe's speeds and limits.
        crate::flight_state::stand_down();
        // Republished on EVERY swap, including to None: a selector that
        // outlives its airframe aims the next aircraft's mode writes at a knob
        // it doesn't have (the same shape as the radio-power flag that latched
        // across a 172 -> Twin Otter change, 2026-07-27).
        crate::flight_state::publish_airframe(&p);
        *self.inner.write().unwrap() = p;
        self.generation
            .fetch_add(1, std::sync::atomic::Ordering::Release);
    }
}

/// Release every continuous manager before an aircraft profile changes.
///

/// Map an X-Plane ICAO type code to a profile id. Unknown types fall back to
/// the Cessna so the app stays usable; add rows as profiles are added.
/// The tests' view of the map: a known type's profile, else the 172.
#[cfg(test)]
pub(crate) fn icao_to_profile(icao: &str) -> String {
    known_profile(icao).unwrap_or_else(|| "cessna_172sp".to_string())
}

/// The curated profile for an ICAO type, if one exists. `None` is an
/// airframe nobody has written a profile for — and that is NOT the 172: the
/// RV-10 flew on the 172's numbers for lack of this distinction (approach
/// speed equal to its own clean stall, flap limit 11 kt under its placard,
/// a healthy 12 volt battery called low; 2026-09-11). An unknown type gets a
/// profile built from its own `.acf` placards instead (`generic_profile`).
pub(crate) fn known_profile(icao: &str) -> Option<String> {
    Some(
        match icao.to_uppercase().as_str() {
            "C172" => "cessna_172sp",
            "SR22" => "cirrus_sr22",
            "MD82" | "MD80" | "MD83" | "MD88" => "md82",
            "C750" | "C75B" => "citation_x",
            "SF50" => "cirrus_vision_sf50",
            "DC3" | "DC30" => "dc3",
            "BE9L" | "BE9T" | "C90" | "C90B" => "king_air_c90b",
            "BE58" | "BE55" => "beechcraft_baron_58",
            "EVOT" | "EVOP" => "lancair_evolution",
            "DHC6" => "dhc6_twin_otter",
            "RV10" => "vans_rv10",
            "B738" | "B737" | "B739" => "boeing_737_800",
            "A333" | "A332" | "A330" => "airbus_a330_300",
            "B744" | "B748" | "B74F" => "boeing_747_400",
            "S76" | "S76C" | "S76D" => "sikorsky_s76c",
            _ => return None,
        }
        .to_string(),
    )
}

/// The names a profile relies on that the running sim does not have: two
/// index fetches (every dataref, every command), one membership test each.
pub(crate) async fn missing_profile_names(
    xp: &XPlaneClient,
    names: &[String],
) -> Result<Vec<String>> {
    let datarefs: std::collections::HashSet<String> = xp
        .dataref_index()
        .await?
        .into_iter()
        .map(|(n, _)| n)
        .collect();
    let commands: std::collections::HashSet<String> =
        xp.command_index().await?.into_iter().collect();
    Ok(aircraft::missing_references(names, &datarefs, &commands))
}

/// The profile id an unknown type flies under, so the detector's "changed?"
/// comparison is stable across ticks.
pub(crate) fn generic_profile_id(icao: &str) -> String {
    format!("generic_{}", icao.to_lowercase())
}

/// The loaded airframe's `.acf` text: the sim names the file
/// (`acf_relative_path`, relative to the install). `None` when unreadable.
pub(crate) async fn acf_text(xp: &XPlaneClient) -> Option<String> {
    let rel = xp
        .get_string("sim/aircraft/view/acf_relative_path")
        .await
        .ok()
        .filter(|r| !r.is_empty())?;
    std::fs::read_to_string(format!("{}/{}", tools::autoland::xplane_root(), rel)).ok()
}

/// A profile for an unknown airframe from its own `.acf`: the sim names the
/// file (`acf_relative_path`, relative to the install), the placards come
/// out of it. `None` when the file can't be found or read — the caller then
/// falls back to the 172's numbers and SAYS so.
pub(crate) async fn generic_profile(
    xp: &XPlaneClient,
    icao: &str,
) -> Option<aircraft::AircraftProfile> {
    let rel = xp
        .get_string("sim/aircraft/view/acf_relative_path")
        .await
        .ok()
        .filter(|r| !r.is_empty())?;
    let path = format!("{}/{}", tools::autoland::xplane_root(), rel);
    match std::fs::read_to_string(&path) {
        Ok(text) => {
            let mut p = aircraft::generic_from_acf(icao, &text);
            p.id = generic_profile_id(icao);
            tracing::warn!(icao, path = %path, "no profile for this airframe; built one from its .acf placards");
            Some(p)
        }
        Err(e) => {
            tracing::warn!(icao, path = %path, "no profile for this airframe and its .acf is unreadable: {e}");
            None
        }
    }
}

/// The aircraft-change poller: re-reads `acf_ICAO`, swaps the live profile
/// when it changes (tools, limits, monitor follow), stands every loop down
/// first, and probes each new profile's names plus the plugin contract
/// against the sim. Its own thread and runtime; auto-detect only.
pub(crate) fn spawn_aircraft_detector(
    live: std::sync::Arc<LiveProfile>,
    base: String,
    det_voice: Voice,
    det_status: Option<crate::plugin_status::StatusSink>,
) {
    std::thread::spawn(move || {
        let Ok(rt) = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
        else {
            return;
        };
        rt.block_on(async move {
            let xp = XPlaneClient::new(base);
            // The last ICAO shown on the panel, so it's published once and on change.
            let mut shown = String::new();
            // The profile id already probed against this sim, so each
            // airframe is asked about once.
            let mut probed = String::new();
            let mut last_plugin_probe = std::time::Instant::now();
            loop {
                if let Ok(icao) = xp.aircraft_icao().await {
                    let icao = icao.trim().to_string();
                    if !icao.is_empty() {
                        // Mirror type + callsign to the panel chip
                        // ("C172 N172SP") — the callsign is what ATC
                        // addresses, so the pilot should see it too.
                        let tail = xp
                            .get_string(datarefs::TAILNUM)
                            .await
                            .ok()
                            .map(|t| t.trim().to_uppercase())
                            .filter(|t| !t.is_empty());
                        let chip = match &tail {
                            Some(t) => format!("{icao} {t}"),
                            None => icao.clone(),
                        };
                        if chip != shown {
                            if let Some(s) = det_status.as_ref() {
                                s.aircraft(&chip);
                            }
                            shown = chip;
                        }
                        let known = known_profile(&icao);
                        let id = known.clone().unwrap_or_else(|| generic_profile_id(&icao));
                        if id != live.id() {
                            // A known type loads its profile. An unknown one
                            // gets its own placards, or the 172's numbers —
                            // and either way the pilot HEARS which, because
                            // a silent fallback is how the RV-10 was flown
                            // to a stall on a Cessna's approach speed.
                            let (loaded, line) = match known {
                                Some(_) => (aircraft::load(&id), None),
                                None => match generic_profile(&xp, &icao).await {
                                    Some(p) => (Some(p), Some("No profile for this airframe. I'm flying it on the placards from its own aircraft file. Treat every automation as untested.".to_string())),
                                    None => (
                                        aircraft::load("cessna_172sp").map(|mut p| {
                                            p.id = id.clone();
                                            p
                                        }),
                                        Some(format!("No profile for {icao} and I can't read its aircraft file. Falling back to Cessna 172 numbers, which are wrong for it. No automation until there's a profile.")),
                                    ),
                                },
                            };
                            if let Some(p) = loaded {
                                let name = p.display_name.clone();
                                tracing::info!(icao = %icao, profile = %id, "aircraft changed; switching profile");
                                // A manual/native push continues inside
                                // X-Plane after its one-shot command. Stop
                                // it before clearing its published state
                                // and swapping actuator selectors.
                                if pushback::ACTIVE.load(
                                    std::sync::atomic::Ordering::SeqCst,
                                ) {
                                    let _ = xp
                                        .run_command(tools::ground::CMD_PUSH_STOP)
                                        .await;
                                }
                                live.swap(p);
                                // Mirror OR action, never both: voice.alert already
                                // posts the spoken line to the panel transcript, so a
                                // separate s.alert here double-wrote it (blue + green).
                                let line = line.unwrap_or_else(|| format!("Now set up for the {name}."));
                                det_voice.alert(voice::Priority::Advisory, line);
                            }
                        }
                        // A profile line is a hypothesis until asked of the
                        // sim. Once per airframe: every dataref and command
                        // the live profile relies on, looked up by name; the
                        // pilot hears what is missing before a tool fails
                        // on it (the RV-10's xlua selector, 2026-09-11).
                        // A stale plugin is re-probed once a minute: the
                        // pilot reloads it mid-session (2026-09-16, one
                        // second after the app's first probe) and the
                        // verdict must follow without an airframe swap.
                        if probed == id
                            && health::plugin_missing().is_some_and(|m| !m.is_empty())
                            && last_plugin_probe.elapsed().as_secs() >= 60
                        {
                            probed.clear();
                        }
                        if id == live.id() && probed != id {
                            last_plugin_probe = std::time::Instant::now();
                            // The gear-warning envelope is the airframe's own
                            // (its .acf), when the profile does not declare
                            // one: the takeoff raises the gear only once the
                            // horn will not sound (SF50, 2026-09-23).
                            let prof = live.get();
                            if prof.retractable_gear && prof.gear_warning.is_none() {
                                if let Some(text) = acf_text(&xp).await {
                                    let w = aircraft::gear_warning_from_acf(&text);
                                    tracing::info!(?w, "gear warning envelope from the .acf");
                                    crate::autotakeoff::publish_gear_warning(w);
                                }
                            }
                            let mut names = aircraft::profile_references(&live.get());
                            let profile_total = names.len();
                            // The plugin's side of the wire contract rides
                            // the same index fetch: a stale .xpl is said
                            // here, not found as a "timeout" hours later.
                            names.extend(health::PLUGIN_DATAREFS.iter().map(|s| s.to_string()));
                            match missing_profile_names(&xp, &names).await {
                                Ok(missing) => {
                                    probed = id.clone();
                                    let (plugin_missing, missing): (Vec<String>, Vec<String>) =
                                        missing.into_iter().partition(|n| n.starts_with("xcopilot/"));
                                    tracing::info!(profile = %id, total = profile_total, missing = ?missing, "profile probe");
                                    blackbox::record(
                                        "profile_probe",
                                        serde_json::json!({ "profile": id, "total": profile_total, "missing": missing }),
                                    );
                                    if let Some(line) = aircraft::probe_summary(&missing, profile_total) {
                                        det_voice.alert(voice::Priority::Caution, line);
                                    }
                                    tracing::info!(total = health::PLUGIN_DATAREFS.len(), missing = ?plugin_missing, "plugin probe");
                                    blackbox::record(
                                        "plugin_probe",
                                        serde_json::json!({ "total": health::PLUGIN_DATAREFS.len(), "missing": plugin_missing }),
                                    );
                                    // Said once per process; the Health
                                    // screen keeps showing it.
                                    let already_said = health::plugin_missing().is_some_and(|m| !m.is_empty());
                                    if let Some(line) = health::plugin_summary(&plugin_missing) {
                                        if !already_said {
                                            det_voice.alert(voice::Priority::Caution, line);
                                        }
                                    }
                                    health::set_plugin_missing(plugin_missing);
                                }
                                // Sim unreachable: ask again next tick.
                                Err(e) => tracing::debug!("profile probe deferred: {e:#}"),
                            }
                        }
                    }
                }
                tokio::time::sleep(std::time::Duration::from_secs(3)).await;
            }
        });
    });
}
