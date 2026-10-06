//! The aircraft on the other side of the API: which profile a loaded MODEL gets
//! (its ICAO and .acf, resolved against every profile's `match:` block),
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
        crate::flight_state::airframe_begins(&p, "boot");
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
        // A spawned controller owns parameters from the profile that armed it:
        // the door stops every owner BEFORE changing the global actuator
        // selectors (else an old takeoff/autoland loop drives the newly loaded
        // aircraft with the previous airframe's speeds and limits), then
        // republishes every selector, including to None (a selector that
        // outlives its airframe aims the next aircraft's mode writes at a knob
        // it doesn't have, the radio-power flag that latched across a 172 ->
        // Twin Otter change, 2026-07-27), then forgets the old flight.
        crate::flight_state::airframe_begins(&p, "swap");
        *self.inner.write().unwrap() = p;
        self.generation
            .fetch_add(1, std::sync::atomic::Ordering::Release);
    }
}

/// Who the loaded aircraft is: the strings detection resolves a profile from.
/// The ICAO names a TYPE; the path (and author, description) name the MODEL.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(crate) struct AcfIdentity {
    /// `sim/aircraft/view/acf_ICAO`.
    pub icao: String,
    /// `sim/aircraft/view/acf_relative_path`, relative to the install.
    pub relative_path: String,
    /// `sim/aircraft/view/acf_author`.
    pub author: String,
    /// `sim/aircraft/view/acf_descrip`.
    pub descrip: String,
}

/// Case-folded, trimmed, backslashes as slashes: how every matcher string and
/// every identity string is compared.
fn normalise(s: &str) -> String {
    s.trim().replace('\\', "/").to_lowercase()
}

/// The detector's change key: the model, not the type. Two .acf files with
/// one ICAO (the stock 737 and the Zibo) are two keys; a livery change is
/// not a change (the livery path is never read).
pub(crate) fn model_key(icao: &str, relative_path: &str) -> (String, String) {
    (icao.trim().to_uppercase(), normalise(relative_path))
}

impl AcfIdentity {
    pub(crate) fn key(&self) -> (String, String) {
        model_key(&self.icao, &self.relative_path)
    }
}

/// The identity, read in one join. Any failed read is no identity: the
/// caller asks again (a missing author must not look like "no author").
pub(crate) async fn read_identity(xp: &XPlaneClient) -> Result<AcfIdentity> {
    let (icao, rel, author, descrip) = tokio::join!(
        xp.aircraft_icao(),
        xp.get_string(datarefs::ACF_RELATIVE_PATH),
        xp.get_string(datarefs::ACF_AUTHOR),
        xp.get_string(datarefs::ACF_DESCRIP),
    );
    Ok(AcfIdentity {
        icao: icao?.trim().to_string(),
        relative_path: rel?.trim().to_string(),
        author: author?.trim().to_string(),
        descrip: descrip?.trim().to_string(),
    })
}

/// What detection made of an identity.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum Resolution {
    /// A profile claims this model. `keys` are the match keys that hit (all
    /// it declared); `tied_with` names a second profile that matched as
    /// specifically, a profile-authoring error.
    Profile {
        id: String,
        keys: Vec<&'static str>,
        tied_with: Option<String>,
    },
    /// A type we have a profile for, but not this model (the Zibo on the
    /// stock 737's ICAO). Never the other model's profile.
    UnknownModel,
    /// A type no profile claims.
    UnknownType,
}

/// The one decision from identity to profile, for the startup probe and the
/// swap poller alike. Pure. Candidates are the profiles whose `match.icao`
/// holds the ICAO; a candidate matches only when every key it declares hits;
/// the most specific (most keys) wins, a tie goes to the first id and is
/// warned. An ICAO hit with no model match is `UnknownModel`.
pub(crate) fn resolve(
    who: &AcfIdentity,
    profiles: &[(String, aircraft::ProfileMatch)],
) -> Resolution {
    let path = normalise(&who.relative_path);
    let author = normalise(&who.author);
    let descrip = normalise(&who.descrip);
    // A key hits when any of its (non-empty) entries is a substring; an
    // empty list, or one of empty strings, is a key not declared.
    let key = |entries: &[String], hay: &str| -> Option<bool> {
        let entries: Vec<String> = entries
            .iter()
            .map(|e| normalise(e))
            .filter(|e| !e.is_empty())
            .collect();
        (!entries.is_empty()).then(|| entries.iter().any(|e| hay.contains(e.as_str())))
    };
    let mut claimed = false;
    let mut hits: Vec<(&str, Vec<&'static str>)> = Vec::new();
    for (id, m) in profiles {
        if !m.claims_type(&who.icao) {
            continue;
        }
        claimed = true;
        let mut keys = vec!["icao"];
        let mut all = true;
        for (name, entries, hay) in [
            ("acf", &m.acf, path.as_str()),
            ("author", &m.author, author.as_str()),
            ("description", &m.description, descrip.as_str()),
        ] {
            match key(entries, hay) {
                Some(true) => keys.push(name),
                Some(false) => all = false,
                None => {}
            }
        }
        if all {
            hits.push((id.as_str(), keys));
        }
    }
    // Most keys first, then by id, so a tie is deterministic.
    hits.sort_by(|a, b| b.1.len().cmp(&a.1.len()).then(a.0.cmp(b.0)));
    let mut it = hits.into_iter();
    match it.next() {
        Some((id, keys)) => {
            let tied_with = it
                .next()
                .filter(|(_, k)| k.len() == keys.len())
                .map(|(o, _)| o.to_string());
            if let Some(other) = &tied_with {
                tracing::warn!(icao = %who.icao, path = %who.relative_path, chosen = id, other = %other, "two profiles match this aircraft equally; picked the first by id");
            }
            Resolution::Profile {
                id: id.to_string(),
                keys,
                tied_with,
            }
        }
        None if claimed => Resolution::UnknownModel,
        None => Resolution::UnknownType,
    }
}

/// The profile id an unmatched airframe flies under: the type AND the model's
/// file, so two unknown models of one type are two ids (and the detector's
/// "changed?" comparison is stable across ticks).
pub(crate) fn generic_profile_id(who: &AcfIdentity) -> String {
    let clean = |s: &str| -> String {
        let mut out = String::new();
        for c in s.to_lowercase().chars() {
            let c = if c.is_ascii_alphanumeric() { c } else { '_' };
            if !(c == '_' && out.ends_with('_')) {
                out.push(c);
            }
        }
        out.trim_matches('_').to_string()
    };
    let stem = std::path::Path::new(&normalise(&who.relative_path))
        .file_stem()
        .and_then(|s| s.to_str())
        .map(clean)
        .filter(|s| !s.is_empty());
    let icao = clean(&who.icao);
    match stem {
        Some(stem) => format!("generic_{icao}_{stem}"),
        None => format!("generic_{icao}"),
    }
}

/// "a B738", "an A333": the article as the letters are spoken.
fn a_type(icao: &str) -> String {
    let an = icao
        .chars()
        .next()
        .is_some_and(|c| "AEFHILMNORSX".contains(c.to_ascii_uppercase()));
    format!("{} {icao}", if an { "an" } else { "a" })
}

/// What the startup probe and the poller load for an identity, and the line
/// the pilot hears when it is not a curated profile.
pub(crate) struct Picked {
    pub id: String,
    pub profile: Option<aircraft::AircraftProfile>,
    pub line: Option<String>,
    pub resolution: Resolution,
    pub who: AcfIdentity,
}

impl Picked {
    /// Why this profile: the identity and the keys that picked it, for the
    /// swap's `profile_probe` record.
    pub(crate) fn blackbox(&self) -> serde_json::Value {
        let (resolution, keys, tied_with) = match &self.resolution {
            Resolution::Profile {
                keys, tied_with, ..
            } => ("profile", keys.clone(), tied_with.clone()),
            Resolution::UnknownModel => ("unknown_model", Vec::new(), None),
            Resolution::UnknownType => ("unknown_type", Vec::new(), None),
        };
        serde_json::json!({
            "icao": self.who.icao,
            "relative_path": self.who.relative_path,
            "author": self.who.author,
            "descrip": self.who.descrip,
            "resolution": resolution,
            "keys": keys,
            "tied_with": tied_with,
        })
    }
}

/// Resolve an identity against every profile's `match:` block and load what
/// it names. A curated profile loads by id. Anything else, including a model
/// of a type we DO have a profile for, flies its own `.acf` placards and SAYS
/// so; an unreadable file gets the 172's numbers, said too. A silent fallback
/// is how the RV-10 was flown to a stall on a Cessna's approach speed
/// (2026-09-11), and how a Zibo would have flown the stock 737's datarefs.
pub(crate) fn pick_profile(who: &AcfIdentity) -> Picked {
    let resolution = resolve(who, &aircraft::profile_matchers());
    let icao = who.icao.trim().to_uppercase();
    let (id, profile, line) = match &resolution {
        Resolution::Profile { id, .. } => (id.clone(), aircraft::load(id), None),
        other => {
            let id = generic_profile_id(who);
            let model = matches!(other, Resolution::UnknownModel);
            match generic_profile(who, &id) {
                Some(p) => {
                    let line = if model {
                        format!("This is {} but not one I have a profile for. I'm flying it on the placards from its own aircraft file. Treat every automation as untested.", a_type(&icao))
                    } else {
                        "No profile for this airframe. I'm flying it on the placards from its own aircraft file. Treat every automation as untested.".to_string()
                    };
                    (id, Some(p), Some(line))
                }
                None => {
                    let p = aircraft::load("cessna_172sp").map(|mut p| {
                        p.id = id.clone();
                        p
                    });
                    let line = if model {
                        format!("This is {} but not one I have a profile for, and I can't read its aircraft file. Falling back to Cessna 172 numbers, which are wrong for it. No automation until there's a profile.", a_type(&icao))
                    } else {
                        format!("No profile for {icao} and I can't read its aircraft file. Falling back to Cessna 172 numbers, which are wrong for it. No automation until there's a profile.")
                    };
                    (id, p, Some(line))
                }
            }
        }
    };
    tracing::info!(icao = %who.icao, path = %who.relative_path, author = %who.author, profile = %id, resolution = ?resolution, "aircraft resolved");
    Picked {
        id,
        profile,
        line,
        resolution,
        who: who.clone(),
    }
}

/// The names a profile relies on that the running sim does not have, from
/// the session's cached indexes ([`ProbeIndex`]).
pub(crate) async fn missing_profile_names(
    xp: &XPlaneClient,
    names: &[String],
) -> Result<Vec<String>> {
    // Taken out for the call and put back after: one probe at a time (the
    // detector's own thread), and a failed one leaves the cache as it was.
    let mut index = PROBE_INDEX.lock().unwrap_or_else(|e| e.into_inner()).take();
    let result = missing_with_cache(xp, &mut index, names, std::time::Instant::now()).await;
    *PROBE_INDEX.lock().unwrap_or_else(|e| e.into_inner()) = index;
    result
}

// ---- The probe's indexes, once per session ---------------------------------
//
// The probe used to fetch the WHOLE dataref index and the WHOLE command index
// (thousands of entries each) on every airframe swap, right as the new flight
// loaded: frame time the sim pays while X-Plane 12.4.4-b2 hung twice after a
// flight init (2026-09-28). Stock names survive a swap, so the indexes are
// fetched once per session and kept; a name the cache lacks is asked by name
// (one small lookup), and only a name that is no dataref asks the command
// index again, at most every [`COMMAND_INDEX_REFRESH`]. A name kept from an
// airframe that has gone (its own plugin's datarefs) may read present: a
// false "all there" for that one name, never a false missing.

/// Above this many unknown names, the dataref index is fetched whole again.
const PER_NAME_MAX: usize = 25;
/// The command index is fetched again no sooner than this.
const COMMAND_INDEX_REFRESH: std::time::Duration = std::time::Duration::from_secs(300);
/// A probe waits this long after a new model is first seen, and for the
/// load's quiet window to close.
pub(crate) const PROBE_SETTLE: std::time::Duration = std::time::Duration::from_secs(10);

/// What the sim has, as the probe last learnt it.
#[derive(Debug, Default)]
pub(crate) struct ProbeIndex {
    datarefs: std::collections::HashSet<String>,
    commands: std::collections::HashSet<String>,
    commands_at: Option<std::time::Instant>,
}

static PROBE_INDEX: std::sync::Mutex<Option<ProbeIndex>> = std::sync::Mutex::new(None);

/// The sim's answers the probe asks for.
#[async_trait::async_trait]
pub(crate) trait IndexSource: Send + Sync {
    async fn dataref_names(&self) -> Result<Vec<String>>;
    async fn command_names(&self) -> Result<Vec<String>>;
    async fn dataref_exists(&self, name: &str) -> Result<bool>;
    async fn command_exists(&self, name: &str) -> Result<bool>;
}

#[async_trait::async_trait]
impl IndexSource for XPlaneClient {
    async fn dataref_names(&self) -> Result<Vec<String>> {
        Ok(self.dataref_index().await?.into_iter().map(|(n, _)| n).collect())
    }
    async fn command_names(&self) -> Result<Vec<String>> {
        self.command_index().await
    }
    async fn dataref_exists(&self, name: &str) -> Result<bool> {
        XPlaneClient::dataref_exists(self, name).await
    }
    async fn command_exists(&self, name: &str) -> Result<bool> {
        XPlaneClient::command_exists(self, name).await
    }
}

/// The missing names, asking the sim only what `cache` cannot answer. The
/// first call fetches both indexes; later ones ask by name. Fills `cache`.
pub(crate) async fn missing_with_cache(
    src: &dyn IndexSource,
    cache: &mut Option<ProbeIndex>,
    names: &[String],
    now: std::time::Instant,
) -> Result<Vec<String>> {
    let Some(idx) = cache.as_mut() else {
        let datarefs = src.dataref_names().await?.into_iter().collect();
        let commands = src.command_names().await?.into_iter().collect();
        let idx =
            cache.insert(ProbeIndex { datarefs, commands, commands_at: Some(now) });
        return Ok(aircraft::missing_references(names, &idx.datarefs, &idx.commands));
    };
    let unknown = aircraft::missing_references(names, &idx.datarefs, &idx.commands);
    if unknown.is_empty() {
        return Ok(unknown);
    }
    if unknown.len() > PER_NAME_MAX {
        idx.datarefs = src.dataref_names().await?.into_iter().collect();
    } else {
        for n in &unknown {
            if src.dataref_exists(n).await? {
                idx.datarefs.insert(n.clone());
            }
        }
    }
    let still = aircraft::missing_references(names, &idx.datarefs, &idx.commands);
    // A name that is no dataref may be a command the new airframe's plugin
    // registered after the session's index was fetched: asked by name, as
    // the datarefs are. The stock 737 at LFRG (2026-10-05) reported its own
    // app_press, guard_cover00 and both std_press missing, swapped to a few
    // minutes after boot, inside the index's refresh window.
    if !still.is_empty() && still.len() <= PER_NAME_MAX {
        for n in &still {
            if src.command_exists(n).await? {
                idx.commands.insert(n.clone());
            }
        }
    }
    let still = aircraft::missing_references(names, &idx.datarefs, &idx.commands);
    let stale = idx
        .commands_at
        .is_none_or(|t| now.saturating_duration_since(t) >= COMMAND_INDEX_REFRESH);
    if !still.is_empty() && stale {
        idx.commands = src.command_names().await?.into_iter().collect();
        idx.commands_at = Some(now);
    }
    Ok(aircraft::missing_references(names, &idx.datarefs, &idx.commands))
}

/// May the probe run: the model has been the same for [`PROBE_SETTLE`] and
/// no flight is loading. Pure.
pub(crate) fn probe_settled(
    model_seen_at: Option<std::time::Instant>,
    now: std::time::Instant,
    loading: bool,
) -> bool {
    !loading
        && model_seen_at.is_none_or(|t| now.saturating_duration_since(t) >= PROBE_SETTLE)
}

/// The loaded airframe's `.acf` text: the sim names the file
/// (`acf_relative_path`, relative to the install). `None` when unreadable.
pub(crate) async fn acf_text(xp: &XPlaneClient) -> Option<String> {
    let rel = xp
        .get_string(datarefs::ACF_RELATIVE_PATH)
        .await
        .ok()
        .filter(|r| !r.is_empty())?;
    std::fs::read_to_string(format!("{}/{}", tools::autoland::xplane_root(), rel)).ok()
}

/// A profile for an unmatched airframe from its own `.acf`: the identity names
/// the file (relative to the install), the placards come out of it. `None`
/// when the file can't be found or read; the caller then falls back to the
/// 172's numbers and SAYS so.
pub(crate) fn generic_profile(who: &AcfIdentity, id: &str) -> Option<aircraft::AircraftProfile> {
    let rel = who.relative_path.trim();
    if rel.is_empty() {
        tracing::warn!(icao = %who.icao, "no profile for this airframe and the sim names no .acf");
        return None;
    }
    let path = format!("{}/{}", tools::autoland::xplane_root(), rel);
    match std::fs::read_to_string(&path) {
        Ok(text) => {
            let mut p = aircraft::generic_from_acf(&who.icao, &text);
            p.id = id.to_string();
            tracing::warn!(icao = %who.icao, path = %path, "no profile for this airframe; built one from its .acf placards");
            Some(p)
        }
        Err(e) => {
            tracing::warn!(icao = %who.icao, path = %path, "no profile for this airframe and its .acf is unreadable: {e}");
            None
        }
    }
}

/// The aircraft-change poller: re-reads `acf_ICAO` and the .acf path, swaps
/// the live profile when the MODEL changes (tools, limits, monitor follow), stands every loop down
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
            // The model last resolved (icao, normalised .acf path), the
            // profile id it resolved to, and why, for the probe record.
            // Keyed on the MODEL: the stock 737 and the Zibo share B738.
            let mut last_key: Option<(String, String)> = None;
            let mut id = String::new();
            let mut identity = serde_json::Value::Null;
            // When the model last resolved: the probe waits for the swap's
            // load to settle before its index requests.
            let mut model_seen_at: Option<std::time::Instant> = None;
            loop {
                let (icao, rel) = tokio::join!(
                    xp.aircraft_icao(),
                    xp.get_string(datarefs::ACF_RELATIVE_PATH)
                );
                if let (Ok(icao), Ok(rel)) = (icao, rel) {
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
                        let key = model_key(&icao, &rel);
                        // A new model is resolved once: the author and
                        // description are read only then. A failed read
                        // leaves the key unset, so the next tick asks again.
                        if last_key.as_ref() != Some(&key) {
                            if let Ok(who) = read_identity(&xp).await {
                                let first = last_key.is_none();
                                last_key = Some(who.key());
                                model_seen_at = Some(std::time::Instant::now());
                                let picked = pick_profile(&who);
                                identity = picked.blackbox();
                                id = picked.id.clone();
                                if id != live.id() {
                                    if let Some(p) = picked.profile {
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
                                        let line = picked.line.unwrap_or_else(|| format!("Now set up for the {name}."));
                                        det_voice.alert(voice::Priority::Advisory, line);
                                    }
                                } else if first {
                                    // The startup probe already loaded it; an
                                    // uncurated airframe is still SAID, once.
                                    if let Some(line) = picked.line {
                                        det_voice.alert(voice::Priority::Advisory, line);
                                    }
                                }
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
                        let settled = probe_settled(
                            model_seen_at,
                            std::time::Instant::now(),
                            crate::xplane::sim_loading(),
                        );
                        if id == live.id() && probed != id && settled {
                            last_plugin_probe = std::time::Instant::now();
                            // The gear-warning envelope is the airframe's own
                            // (its .acf), when the profile does not declare
                            // one: the takeoff raises the gear only once the
                            // horn will not sound (SF50, 2026-09-23).
                            let prof = live.get();
                            let acf = acf_text(&xp).await;
                            // What this airframe's systems ARE: the copilot
                            // consults the survey before it reaches for a
                            // facility (the ILS setup first, 2026-09-25).
                            {
                                let mut sys = acf
                                    .as_deref()
                                    .map(crate::systems::Systems::from_acf)
                                    .unwrap_or_else(|| crate::systems::Systems { surveyed: true, ..Default::default() });
                                sys.ap_type = xp
                                    .get_float(crate::systems::AP_TYPE)
                                    .await
                                    .ok()
                                    .and_then(|c| crate::systems::ApType::from_code(c.round() as i64));
                                tracing::info!(profile = %id, systems = ?sys, "systems survey");
                                let mut rec = sys.blackbox();
                                rec["profile"] = serde_json::json!(id);
                                blackbox::record("systems", rec);
                                crate::systems::publish(Some(sys));
                            }
                            if let Some(text) = acf {
                                if prof.retractable_gear && prof.gear_warning.is_none() {
                                    let w = aircraft::gear_warning_from_acf(&text);
                                    tracing::info!(?w, "gear warning envelope from the .acf");
                                    crate::autotakeoff::publish_gear_warning(w);
                                }
                                // A castoring nose steers on the brakes,
                                // declared or not.
                                let castors = aircraft::nose_castors_from_acf(&text);
                                aircraft::publish_nose_castors(castors);
                                aircraft::publish_nose_free(aircraft::nose_free_from_acf(&text));
                                let declared = prof.taxi.as_ref().is_some_and(|t| t.brake_steer);
                                tracing::info!(castors, declared, "nose gear from the .acf");
                                if castors && !declared {
                                    blackbox::record(
                                        "profile_probe",
                                        serde_json::json!({ "profile": id, "nose_castors": true, "brake_steer_declared": false }),
                                    );
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
                                        serde_json::json!({ "profile": id, "total": profile_total, "missing": missing, "identity": identity }),
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
                                    // Fixed by a rebuild and reload: the
                                    // EFB's caution about it goes too.
                                    if already_said && plugin_missing.is_empty() {
                                        crate::efb::feed::withdraw_alerts("Plugin check:");
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

/// The resolver for an ICAO and .acf path alone (no author or
/// description): the curated profile they resolve to, `None` when generic.
/// The tests' view, and `xvl sortie --check`'s.
pub(crate) fn resolved_id(icao: &str, relative_path: &str) -> Option<String> {
    let who = AcfIdentity {
        icao: icao.into(),
        relative_path: relative_path.into(),
        ..Default::default()
    };
    match resolve(&who, &aircraft::profile_matchers()) {
        Resolution::Profile { id, .. } => Some(id),
        _ => None,
    }
}

#[cfg(test)]
mod probe_cache_tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering::SeqCst};
    use std::time::{Duration, Instant};

    #[derive(Default)]
    struct Sim {
        datarefs: Vec<&'static str>,
        commands: Vec<&'static str>,
        dref_dumps: AtomicUsize,
        cmd_dumps: AtomicUsize,
        lookups: AtomicUsize,
        cmd_lookups: AtomicUsize,
        /// Registered by an airframe's plugin after the index was fetched.
        late_commands: Vec<&'static str>,
    }

    #[async_trait::async_trait]
    impl IndexSource for Sim {
        async fn dataref_names(&self) -> Result<Vec<String>> {
            self.dref_dumps.fetch_add(1, SeqCst);
            Ok(self.datarefs.iter().map(|s| s.to_string()).collect())
        }
        async fn command_names(&self) -> Result<Vec<String>> {
            self.cmd_dumps.fetch_add(1, SeqCst);
            Ok(self.commands.iter().map(|s| s.to_string()).collect())
        }
        async fn dataref_exists(&self, name: &str) -> Result<bool> {
            self.lookups.fetch_add(1, SeqCst);
            Ok(self.datarefs.contains(&name))
        }
        async fn command_exists(&self, name: &str) -> Result<bool> {
            self.cmd_lookups.fetch_add(1, SeqCst);
            Ok(self.commands.contains(&name) || self.late_commands.contains(&name))
        }
    }

    fn names(v: &[&str]) -> Vec<String> {
        v.iter().map(|s| s.to_string()).collect()
    }

    /// The whole indexes are fetched once per session: a swap to an
    /// airframe whose names are all known asks the sim nothing, a new
    /// dataref is one lookup by name, and a missing name asks the command
    /// index again only once per refresh window (an older plugin's missing
    /// flag is re-probed every minute).
    #[tokio::test]
    async fn the_probe_fetches_the_indexes_once_per_session() {
        let sim = Sim {
            datarefs: vec![crate::datarefs::LATITUDE, crate::datarefs::LONGITUDE, "xvl/own"],
            commands: vec!["sim/operation/pause_toggle"],
            ..Default::default()
        };
        let t = Instant::now();
        let mut cache = None;
        let first = names(&[crate::datarefs::LATITUDE, "sim/operation/pause_toggle"]);
        assert!(missing_with_cache(&sim, &mut cache, &first, t).await.unwrap().is_empty());
        assert_eq!((sim.dref_dumps.load(SeqCst), sim.cmd_dumps.load(SeqCst)), (1, 1));

        // The next airframe: known names cost nothing.
        let known = names(&[crate::datarefs::LONGITUDE, "sim/operation/pause_toggle"]);
        assert!(missing_with_cache(&sim, &mut cache, &known, t).await.unwrap().is_empty());
        assert_eq!((sim.dref_dumps.load(SeqCst), sim.cmd_dumps.load(SeqCst)), (1, 1));
        assert_eq!(sim.lookups.load(SeqCst), 0);

        // A name the cache lacks: one lookup (the index said it was there
        // all along, so it is found without a dump).
        let own = names(&["xvl/own"]);
        let mut fresh = Some(ProbeIndex { commands_at: Some(t), ..Default::default() });
        assert!(missing_with_cache(&sim, &mut fresh, &own, t).await.unwrap().is_empty());
        assert_eq!(sim.lookups.load(SeqCst), 1);
        assert_eq!(sim.dref_dumps.load(SeqCst), 1);

        // A name nobody has: said missing; the command index is asked again
        // only once the refresh window has passed.
        let gone = names(&["xcopilot/sim/loading"]);
        let later = t + Duration::from_secs(60);
        assert_eq!(missing_with_cache(&sim, &mut cache, &gone, later).await.unwrap(), gone);
        assert_eq!(sim.cmd_dumps.load(SeqCst), 1, "inside the window");
        let much_later = t + COMMAND_INDEX_REFRESH;
        assert_eq!(missing_with_cache(&sim, &mut cache, &gone, much_later).await.unwrap(), gone);
        assert_eq!(sim.cmd_dumps.load(SeqCst), 2);
        assert_eq!(sim.dref_dumps.load(SeqCst), 1, "never the whole dataref index again");
    }

    /// An airframe's own command, registered by its plugin after the
    /// session's index was fetched, is found by name inside the refresh
    /// window (the stock 737's app_press at LFRG, 2026-10-05).
    #[tokio::test]
    async fn a_command_registered_after_the_index_is_asked_by_name() {
        let sim = Sim {
            commands: vec!["sim/operation/pause_toggle"],
            late_commands: vec!["laminar/B738/autopilot/app_press"],
            ..Default::default()
        };
        let t = Instant::now();
        let mut cache = Some(ProbeIndex {
            commands: ["sim/operation/pause_toggle".to_string()].into(),
            commands_at: Some(t),
            ..Default::default()
        });
        let own = names(&["laminar/B738/autopilot/app_press"]);
        let soon = t + Duration::from_secs(60);
        assert!(missing_with_cache(&sim, &mut cache, &own, soon).await.unwrap().is_empty());
        assert_eq!(sim.cmd_lookups.load(SeqCst), 1);
        assert_eq!(sim.cmd_dumps.load(SeqCst), 0, "no whole index inside the window");
        // Known now: the next probe asks nothing.
        assert!(missing_with_cache(&sim, &mut cache, &own, soon).await.unwrap().is_empty());
        assert_eq!(sim.cmd_lookups.load(SeqCst), 1);
    }

    #[test]
    fn the_probe_waits_for_the_swap_to_settle() {
        let t = Instant::now();
        assert!(probe_settled(None, t, false));
        assert!(!probe_settled(Some(t), t + Duration::from_secs(3), false));
        assert!(probe_settled(Some(t), t + PROBE_SETTLE, false));
        assert!(!probe_settled(Some(t), t + PROBE_SETTLE, true), "a load is loading");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SPARKY_AUTHOR: &str = "Mohammad Gazzawi / Sergio Santagada / Javier Rollon / Jim Gregory / Michael M. / YesAviation / PresentMonkey / Wolfair / Notchmaster / Reddeviln / Totedemac/ Ghostfire / Mark Parker / Others (see github)";

    fn who(icao: &str, path: &str, author: &str) -> AcfIdentity {
        AcfIdentity {
            icao: icao.into(),
            relative_path: path.into(),
            author: author.into(),
            descrip: String::new(),
        }
    }

    /// Every installed .acf (ICAO and author from its own `P acf/_ICAO` and
    /// `P acf/_author` lines, 2026-09-27) and the profile it must fly.
    /// `None` is generic, said aloud.
    fn installed() -> Vec<(AcfIdentity, Option<&'static str>)> {
        let lr = "Laminar Research - dmax3d.com";
        let awx = "Johan van Wyk, Fred Stegmann, Manfred Jahn, Jan Visser and Team";
        vec![
            (
                who(
                    "C172",
                    "Aircraft/Laminar Research/Cessna 172 SP/Cessna_172SP.acf",
                    lr,
                ),
                Some("cessna_172sp"),
            ),
            (
                who(
                    "C172",
                    "Aircraft/Laminar Research/Cessna 172 SP/Cessna_172SP_G1000.acf",
                    lr,
                ),
                Some("cessna_172sp"),
            ),
            (
                who(
                    "C172",
                    "Aircraft/Laminar Research/Cessna 172 SP/Cessna_172SP_seaplane.acf",
                    lr,
                ),
                Some("cessna_172sp"),
            ),
            (
                who(
                    "SR22",
                    "Aircraft/Laminar Research/Cirrus SR22/Cirrus SR22.acf",
                    "Rodrigo Fernandez P., Alex Unruh, Austin Meyer",
                ),
                Some("cirrus_sr22"),
            ),
            (
                who(
                    "MD82",
                    "Aircraft/Laminar Research/McDonnell Douglas MD-82/MD80.acf",
                    "Sergio Santagada",
                ),
                Some("md82"),
            ),
            (
                who(
                    "C750",
                    "Aircraft/Laminar Research/Cessna Citation X/Cessna_CitationX.acf",
                    "Massimo Durando",
                ),
                Some("citation_x"),
            ),
            (
                who(
                    "SF50",
                    "Aircraft/Laminar Research/Cirrus Vision SF50/CirrusSF50.acf",
                    lr,
                ),
                Some("cirrus_vision_sf50"),
            ),
            (who("DC3", "Aircraft/AWXDC3/awxdc3.acf", awx), Some("dc3")),
            (
                who("DC3", "Aircraft/AWXDC3/awxdc3GNS.acf", awx),
                Some("dc3"),
            ),
            (
                who("DC3", "Aircraft/AWXDC3/awxdc3_Pas.acf", awx),
                Some("dc3"),
            ),
            (
                who("DC3", "Aircraft/AWXDC3/AWXDC3Float.acf", "Aeroworx"),
                Some("dc3"),
            ),
            (
                who(
                    "BE9L",
                    "Aircraft/Laminar Research/Beechcraft King Air C90B/C90B.acf",
                    "Tom Kyler/Jim Gregory",
                ),
                Some("king_air_c90b"),
            ),
            (
                who(
                    "BE58",
                    "Aircraft/Laminar Research/Beechcraft Baron 58/Baron_58.acf",
                    "Tom Kyler/Jim Gregory",
                ),
                Some("beechcraft_baron_58"),
            ),
            (
                who(
                    "EVOT",
                    "Aircraft/Laminar Research/Lancair Evolution/N844X.acf",
                    "Austin Meyer, Rodrigo Fernandez, Alex Unruh",
                ),
                Some("lancair_evolution"),
            ),
            (
                who(
                    "DHC6",
                    "Aircraft/DHC6-300 Twin Otter V2/DHC6.acf",
                    "Rob Wilson",
                ),
                Some("dhc6_twin_otter"),
            ),
            (
                who(
                    "DHC6",
                    "Aircraft/DHC6-300 Twin Otter V2/DHC6S.acf",
                    "Rob Wilson",
                ),
                Some("dhc6_twin_otter"),
            ),
            (
                who(
                    "DHC6",
                    "Aircraft/DHC6-300 Twin Otter V2/DHC6T.acf",
                    "Rob Wilson",
                ),
                Some("dhc6_twin_otter"),
            ),
            (
                who(
                    "RV10",
                    "Aircraft/Laminar Research/Van's Aircraft RV-10/RV-10.acf",
                    "Rodrigo Fernandez",
                ),
                Some("vans_rv10"),
            ),
            (
                who(
                    "B738",
                    "Aircraft/Laminar Research/Boeing 737-800/b738.acf",
                    "Alex Unruh",
                ),
                Some("boeing_737_800"),
            ),
            (
                who(
                    "A333",
                    "Aircraft/Laminar Research/Airbus A330-300/A330.acf",
                    "Alex Unruh",
                ),
                Some("airbus_a330_300"),
            ),
            (
                who(
                    "A333",
                    "Aircraft/Laminar Research/Airbus A330-300/A330_AI.acf",
                    "Alex Unruh",
                ),
                Some("airbus_a330_300"),
            ),
            (
                who("B744", "Aircraft/Sparky/747-400/747-400.acf", SPARKY_AUTHOR),
                Some("boeing_747_400"),
            ),
            (
                who(
                    "B744",
                    "Aircraft/Sparky/747-400/747-400_XP11.acf",
                    SPARKY_AUTHOR,
                ),
                Some("boeing_747_400"),
            ),
            (
                who(
                    "S76",
                    "Aircraft/Laminar Research/Sikorsky S-76/S-76C.acf",
                    "Brett Sumpter",
                ),
                Some("sikorsky_s76c"),
            ),
            // Installed, no profile: the ToLiss A319 and the Laminar types
            // nobody has written one for.
            (
                who(
                    "A319",
                    "Aircraft/ToLissA319_V1p12p1/a319.acf",
                    "Gliding Kiwi",
                ),
                None,
            ),
            (
                who(
                    "A319",
                    "Aircraft/ToLissA319_V1p12p1/a319_XP11_StdDef.acf",
                    "Gliding Kiwi",
                ),
                None,
            ),
            (
                who(
                    "PA18",
                    "Aircraft/Laminar Research/Piper PA-18 Super Cub/PA-18-150.acf",
                    lr,
                ),
                None,
            ),
            (
                who(
                    "R22",
                    "Aircraft/Laminar Research/Robinson R22 Beta II/Robinson R22 Beta II.acf",
                    "Marcel Felde",
                ),
                None,
            ),
            // The Zibo says B738 too, and is NOT the stock 737.
            (
                who("B738", "Aircraft/Zibo/B737-800X/b738_4k.acf", "Zibo"),
                None,
            ),
            // Windows reports backslashes.
            (
                who(
                    "B738",
                    "Aircraft\\Laminar Research\\Boeing 737-800\\b738.acf",
                    "Alex Unruh",
                ),
                Some("boeing_737_800"),
            ),
            (
                who(
                    "b738",
                    "AIRCRAFT/LAMINAR RESEARCH/BOEING 737-800/B738.ACF",
                    "Alex Unruh",
                ),
                Some("boeing_737_800"),
            ),
        ]
    }

    #[test]
    fn every_installed_model_resolves_to_its_own_profile_or_generic() {
        let profiles = aircraft::profile_matchers();
        for (w, want) in installed() {
            let got = resolve(&w, &profiles);
            match (want, &got) {
                (Some(want), Resolution::Profile { id, tied_with, .. }) => {
                    assert_eq!(id, want, "{w:?}");
                    assert_eq!(tied_with, &None, "two shipped profiles tie on {w:?}");
                }
                (None, Resolution::UnknownModel | Resolution::UnknownType) => {}
                _ => panic!("{w:?} resolved to {got:?}, wanted {want:?}"),
            }
        }
    }

    /// The Zibo on the stock 737's ICAO: an unknown MODEL of a known type,
    /// flown generic and said so, never the stock profile.
    #[test]
    fn a_zibo_is_not_the_stock_737() {
        let zibo = who("B738", "Aircraft/Zibo/B737-800X/b738_4k.acf", "Zibo");
        assert_eq!(
            resolve(&zibo, &aircraft::profile_matchers()),
            Resolution::UnknownModel
        );
        let tolis = who(
            "A319",
            "Aircraft/ToLissA319_V1p12p1/a319.acf",
            "Gliding Kiwi",
        );
        assert_eq!(
            resolve(&tolis, &aircraft::profile_matchers()),
            Resolution::UnknownType
        );
        // The file does not exist, so the 172's numbers: said, and under the
        // model's own generic id, never boeing_737_800.
        let picked = pick_profile(&zibo);
        assert_eq!(picked.id, "generic_b738_b738_4k");
        let line = picked
            .line
            .clone()
            .expect("an uncurated airframe is said aloud");
        assert!(
            line.starts_with("This is a B738 but not one I have a profile for"),
            "{line}"
        );
        let p = picked.profile.clone().expect("a flyable fallback");
        assert_eq!(p.id, "generic_b738_b738_4k");
        let rec = picked.blackbox();
        assert_eq!(rec["resolution"], "unknown_model");
        assert_eq!(rec["relative_path"], "Aircraft/Zibo/B737-800X/b738_4k.acf");
    }

    #[test]
    fn the_probe_record_names_the_identity_and_the_keys() {
        let sparky = who("B744", "Aircraft/Sparky/747-400/747-400.acf", SPARKY_AUTHOR);
        let picked = pick_profile(&sparky);
        assert_eq!(picked.id, "boeing_747_400");
        assert!(picked.line.is_none());
        let rec = picked.blackbox();
        assert_eq!(rec["resolution"], "profile");
        assert_eq!(rec["keys"], serde_json::json!(["icao", "acf", "author"]));
        assert_eq!(rec["icao"], "B744");
        assert_eq!(rec["author"], SPARKY_AUTHOR);
        // Every declared key must hit: the Sparky folder with another
        // author, or a renamed folder, is not the Sparky.
        let other = who(
            "B744",
            "Aircraft/Sparky/747-400/747-400.acf",
            "Laminar Research",
        );
        assert_eq!(
            resolve(&other, &aircraft::profile_matchers()),
            Resolution::UnknownModel
        );
        let moved = who("B744", "Aircraft/My 747/747-400.acf", SPARKY_AUTHOR);
        assert_eq!(
            resolve(&moved, &aircraft::profile_matchers()),
            Resolution::UnknownModel
        );
    }

    #[test]
    fn every_shipped_profile_declares_its_model() {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("profiles");
        let mut n = 0;
        for e in std::fs::read_dir(dir).unwrap().flatten() {
            let path = e.path();
            if path.extension().is_none_or(|x| x != "yaml") {
                continue;
            }
            let text = std::fs::read_to_string(&path).unwrap();
            let p: aircraft::AircraftProfile =
                serde_yaml::from_str(&text).unwrap_or_else(|e| panic!("{}: {e:#}", path.display()));
            let m = p
                .matcher
                .unwrap_or_else(|| panic!("{} has no match: block", p.id));
            assert!(!m.icao.is_empty(), "{} match: needs icao", p.id);
            let model = [&m.acf, &m.author, &m.description]
                .iter()
                .any(|l| l.iter().any(|e| !e.trim().is_empty()));
            assert!(model, "{}: an ICAO alone is a type, not a model", p.id);
            n += 1;
        }
        assert!(n >= 15, "only {n} profiles walked");
        // The built-ins (the YAML-missing fallback) declare theirs too.
        for id in ["cessna_172sp", "cirrus_sr22"] {
            let m = aircraft::builtin(id)
                .and_then(|p| p.matcher)
                .expect("built-in match");
            assert!(!m.acf.is_empty(), "{id}");
        }
    }

    #[test]
    fn specificity_wins_and_a_tie_is_picked_by_id() {
        let m = |icao: &str, acf: &[&str], author: &[&str]| aircraft::ProfileMatch {
            icao: vec![icao.into()],
            acf: acf.iter().map(|s| s.to_string()).collect(),
            author: author.iter().map(|s| s.to_string()).collect(),
            description: Vec::new(),
        };
        let profiles = vec![
            ("a_catch_all".to_string(), m("B738", &[], &[])),
            ("b_folder".to_string(), m("B738", &["Zibo/"], &[])),
            (
                "c_folder_and_author".to_string(),
                m("B738", &["zibo/"], &["zibo"]),
            ),
            ("d_folder_too".to_string(), m("B738", &["B737-800X"], &[])),
        ];
        let z = who("B738", "Aircraft/Zibo/B737-800X/b738_4k.acf", "Zibo team");
        assert_eq!(
            resolve(&z, &profiles),
            Resolution::Profile {
                id: "c_folder_and_author".into(),
                keys: vec!["icao", "acf", "author"],
                tied_with: None
            }
        );
        let z2 = who("B738", "Aircraft/Zibo/B737-800X/b738_4k.acf", "someone");
        assert_eq!(
            resolve(&z2, &profiles),
            Resolution::Profile {
                id: "b_folder".into(),
                keys: vec!["icao", "acf"],
                tied_with: Some("d_folder_too".into())
            }
        );
        // Only the catch-all is left for another model of the type.
        let s = who(
            "B738",
            "Aircraft/Laminar Research/Boeing 737-800/b738.acf",
            "",
        );
        assert_eq!(
            resolve(&s, &profiles),
            Resolution::Profile {
                id: "a_catch_all".into(),
                keys: vec!["icao"],
                tied_with: None
            }
        );
        // A description key.
        let d = vec![(
            "desc".to_string(),
            aircraft::ProfileMatch {
                icao: vec!["X1".into()],
                description: vec!["Special".into()],
                ..Default::default()
            },
        )];
        let mut w = who("X1", "Aircraft/x/x.acf", "");
        assert_eq!(resolve(&w, &d), Resolution::UnknownModel);
        w.descrip = "A special build".into();
        assert!(matches!(resolve(&w, &d), Resolution::Profile { .. }));
    }

    /// The poller re-resolves on the MODEL: same ICAO, another .acf, is a
    /// change; the same file spelled another way is not.
    #[test]
    fn the_detector_key_is_the_model_not_the_type() {
        let stock = model_key("B738", "Aircraft/Laminar Research/Boeing 737-800/b738.acf");
        let zibo = model_key("B738", "Aircraft/Zibo/B737-800X/b738_4k.acf");
        assert_ne!(stock, zibo);
        assert_eq!(
            stock,
            model_key(
                " b738 ",
                "Aircraft\\Laminar Research\\Boeing 737-800\\B738.acf"
            )
        );
        assert_eq!(
            stock,
            who(
                "B738",
                "Aircraft/Laminar Research/Boeing 737-800/b738.acf",
                "any"
            )
            .key()
        );
        // Two unknown models of one type are two generic ids.
        let a = generic_profile_id(&who("B738", "Aircraft/Zibo/B737-800X/b738_4k.acf", ""));
        let b = generic_profile_id(&who("B738", "Aircraft/Other/737/b738_other.acf", ""));
        assert_ne!(a, b);
        assert_eq!(generic_profile_id(&who("A319", "", "")), "generic_a319");
        assert_eq!(
            generic_profile_id(&who(
                "A319",
                "Aircraft/ToLissA319_V1p12p1/a319_XP11_StdDef.acf",
                ""
            )),
            "generic_a319_a319_xp11_stddef"
        );
    }

    #[test]
    fn a_plan_type_finds_a_profile_of_that_type() {
        assert_eq!(
            aircraft::profile_for_type("B738").map(|p| p.id).as_deref(),
            Some("boeing_737_800")
        );
        assert_eq!(
            aircraft::profile_for_type("b744").map(|p| p.id).as_deref(),
            Some("boeing_747_400")
        );
        assert!(aircraft::profile_for_type("S76").is_some_and(|p| p.rotorcraft));
        assert!(aircraft::profile_for_type("A319").is_none());
        assert!(aircraft::profile_for_type("").is_none());
    }

    #[test]
    fn the_article_follows_the_spoken_letter() {
        assert_eq!(a_type("B738"), "a B738");
        assert_eq!(a_type("A333"), "an A333");
        assert_eq!(a_type("MD82"), "an MD82");
    }
}
