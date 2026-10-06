//! The EFB: the copilot's knowledge as a web page on the LAN (an iPad, an
//! iPhone, a second screen), plus buttons that SPEAK for the pilot.
//!
//! Two rules make it safe to bolt onto a real-time loop (CLAUDE.md "EFB"):
//!
//! - **No new actuation path.** A button or the typed box POSTs pilot English
//!   (`/say`) that goes onto the command channel exactly like the panel's
//!   typed line (`pollers::queue_command`, blackbox `typed` `taken`, source
//!   `efb`): the grammar picks the tool, its gates and readback included. A
//!   button can never do what the voice cannot ([`state::PHRASES`], pinned
//!   to the routing the loop does).
//! - **No new sim reads.** The page is built from what the app already holds
//!   (`gather`: the flight model, the owners, the shelf's plan, the follower,
//!   the party line, the voice's last controller line, the monitor's tick as
//!   it already read it, the METAR file). A fact nobody holds is a dash.
//!
//! Off by default (`XCOPILOT_EFB`); every request carries the token
//! (`XCOPILOT_EFB_TOKEN`, generated and saved at the first enable). Its own
//! thread and runtime: the command loop blocks on the mic and would starve
//! anything run on its runtime, and nothing here holds a lock across an await.

pub(crate) mod airport;
pub(crate) mod feed;
pub(crate) mod flight_tab;
pub(crate) mod ptt;
pub(crate) mod route_map;
pub(crate) mod state;
pub(crate) mod transcript;

use std::net::SocketAddr;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use axum::extract::{ConnectInfo, Query, State};
use axum::http::{header, HeaderMap, HeaderValue, StatusCode};
use axum::response::sse::{Event, KeepAlive, Sse};
use axum::response::{Html, IntoResponse, Response};
use axum::routing::{get, post};
use axum::{Json, Router};

use crate::voice::Voice;

/// The page, one self-contained file (vanilla JS, no build step).
const PAGE: &str = include_str!("page.html");

/// The Airport page's drawing code (vanilla JS, no library, no tiles).
const DIAGRAM_JS: &str = include_str!("diagram.js");

/// The cookie that carries the token after the first `?t=` load.
const COOKIE: &str = "xvl_efb";

/// One line per this long, at most: a stuck finger or a double tap never
/// queues a command twice.
pub(crate) const SAY_MIN_GAP: Duration = Duration::from_millis(500);

/// The longest line accepted (the longest command a pilot says is far shorter).
const SAY_MAX_CHARS: usize = 200;

/// One setting change per this long, at most (a double tap on a toggle).
pub(crate) const SETTING_MIN_GAP: Duration = Duration::from_millis(300);

/// The push to talk's heartbeat is every 250 ms; faster than this is a
/// stuck loop on the page, refused without effect (a release never is).
const PTT_MIN_GAP: Duration = Duration::from_millis(100);

/// The SSE push period: change-gated, so a quiet flight sends little.
const PUSH_EVERY: Duration = Duration::from_secs(1);

/// The METAR file is re-read this often at most (a disk read, never the sim).
const METAR_EVERY: Duration = Duration::from_secs(60);

/// A token we accept: 8 to 64 letters, digits, '-' or '_'.
pub(crate) fn token_ok(t: &str) -> bool {
    (8..=64).contains(&t.len())
        && t.chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_')
}

/// A fresh 128-bit token, hex. `RandomState` is keyed from the OS's
/// randomness; two of them, mixed with the clock and the pid, are plenty
/// for a key on a home LAN (no crypto crate for one string).
pub(crate) fn new_token() -> String {
    use std::hash::{BuildHasher, Hasher};
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0);
    (0..2u8)
        .map(|i| {
            let mut h = std::collections::hash_map::RandomState::new().build_hasher();
            h.write_u128(nanos);
            h.write_u32(std::process::id());
            h.write_u8(i);
            format!("{:016x}", h.finish())
        })
        .collect()
}

/// Equal without an early exit on the first differing byte.
fn same(a: &str, b: &str) -> bool {
    a.len() == b.len()
        && a.bytes()
            .zip(b.bytes())
            .fold(0u8, |acc, (x, y)| acc | (x ^ y))
            == 0
}

/// The token a request carries: `?t=`, the `X-EFB-Token` header, or the cookie.
fn presented<'a>(headers: &'a HeaderMap, query: Option<&'a str>) -> Option<&'a str> {
    if let Some(q) = query.filter(|q| !q.is_empty()) {
        return Some(q);
    }
    if let Some(h) = headers.get("x-efb-token").and_then(|v| v.to_str().ok()) {
        return Some(h);
    }
    headers
        .get_all(header::COOKIE)
        .iter()
        .filter_map(|v| v.to_str().ok())
        .flat_map(|v| v.split(';'))
        .filter_map(|kv| kv.trim().split_once('='))
        .find(|(k, _)| *k == COOKIE)
        .map(|(_, v)| v)
}

/// Does the request carry the live token? Never with no token set.
pub(crate) fn authorised(token: &str, headers: &HeaderMap, query: Option<&str>) -> bool {
    !token.is_empty() && presented(headers, query).is_some_and(|t| same(t, token))
}

/// A pilot's line from the page, cleaned: one line, no control characters,
/// no em or en dash, bounded. `None` for nothing to say.
pub(crate) fn clean_line(raw: &str) -> Option<String> {
    let one: String = raw
        .chars()
        .map(|c| if c.is_control() { ' ' } else { c })
        .collect();
    let t =
        crate::phrase::scrub_dashes(one.split_whitespace().collect::<Vec<_>>().join(" ").trim());
    if t.is_empty() {
        return None;
    }
    Some(t.chars().take(SAY_MAX_CHARS).collect())
}

/// The last METAR read, kept so the file is not re-read every second.
#[derive(Default)]
struct MetarCache {
    read_at: Option<Instant>,
    /// (icao, raw line or None) for the fields asked last time.
    lines: Vec<(String, Option<String>)>,
    modified: Option<SystemTime>,
}

/// The server's own state: the voice (read only: the controller's last
/// line), the command channel, the rate limit, the last line sent.
pub(crate) struct Server {
    voice: Option<Voice>,
    tx: Mutex<std::sync::mpsc::Sender<String>>,
    last_say: Mutex<Option<Instant>>,
    last_setting: Mutex<Option<Instant>>,
    last_ptt: Mutex<Option<Instant>>,
    sent: Mutex<Option<(f64, String)>>,
    metar: Mutex<MetarCache>,
}

impl Server {
    pub(crate) fn new(voice: Option<Voice>, tx: std::sync::mpsc::Sender<String>) -> Arc<Server> {
        Arc::new(Server {
            voice,
            tx: Mutex::new(tx),
            last_say: Mutex::new(None),
            last_setting: Mutex::new(None),
            last_ptt: Mutex::new(None),
            sent: Mutex::new(None),
            metar: Mutex::new(MetarCache::default()),
        })
    }
}

fn now_unix() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

/// The fields whose weather the page shows: the plan's, else none.
fn metar_fields(plan: Option<&state::PlanFacts>) -> Vec<(&'static str, String)> {
    let Some(p) = plan else { return Vec::new() };
    [
        ("Destination", &p.destination),
        ("Origin", &p.origin),
        ("Alternate", &p.alternate),
    ]
    .into_iter()
    .filter(|(_, i)| !i.trim().is_empty())
    .map(|(r, i)| (r, i.trim().to_ascii_uppercase()))
    .collect()
}

/// Refresh the METAR cache when stale or the fields changed: a file read
/// off the runtime's thread, never a sim read.
async fn refresh_metars(server: &Arc<Server>, fields: Vec<String>) {
    let due = {
        let c = server.metar.lock().unwrap_or_else(|e| e.into_inner());
        let same_fields = c.lines.iter().map(|(i, _)| i).eq(fields.iter());
        !same_fields || c.read_at.is_none_or(|t| t.elapsed() >= METAR_EVERY)
    };
    if !due {
        return;
    }
    let read = tokio::task::spawn_blocking(move || {
        let path = crate::weather::latest_file(&crate::config::xplane_root());
        let modified = path
            .as_ref()
            .and_then(|p| std::fs::metadata(p).ok())
            .and_then(|m| m.modified().ok());
        let text = path
            .and_then(|p| std::fs::read_to_string(p).ok())
            .unwrap_or_default();
        let lines = fields
            .iter()
            .map(|i| {
                (
                    i.clone(),
                    crate::weather::line_for(&text, i).map(str::to_string),
                )
            })
            .collect::<Vec<_>>();
        (lines, modified)
    })
    .await;
    if let Ok((lines, modified)) = read {
        let mut c = server.metar.lock().unwrap_or_else(|e| e.into_inner());
        c.lines = lines;
        c.modified = modified;
        c.read_at = Some(Instant::now());
    }
}

/// The plan on the shelf, reduced to what the pages show.
fn plan_facts() -> Option<state::PlanFacts> {
    let fp = crate::simbrief::loaded_plan()?;
    let stage = |s: crate::simbrief::Stage| match s {
        crate::simbrief::Stage::Climb => "climb",
        crate::simbrief::Stage::Cruise => "cruise",
        crate::simbrief::Stage::Descent => "descent",
    };
    Some(state::PlanFacts {
        source: match fp.source {
            crate::simbrief::PlanSource::SimBrief => "SimBrief",
            crate::simbrief::PlanSource::Local => "Local",
        },
        origin: fp.origin.clone(),
        destination: fp.destination.clone(),
        alternate: fp.alternate.clone(),
        aircraft: fp.aircraft.clone(),
        route: fp.route.clone(),
        cruise_alt_ft: fp.cruise_alt_ft,
        distance_nm: fp.distance_nm,
        ete_min: fp.ete_min,
        tod_to_dest_nm: fp.tod_to_dest_nm,
        block_fuel_kg: fp.block_fuel_kg,
        trip_fuel_kg: fp.trip_fuel_kg,
        reserve_kg: fp.reserve_kg,
        min_takeoff_fuel_kg: fp.min_takeoff_fuel_kg,
        sid: fp.procedures.sid.clone(),
        star: fp.procedures.star.clone(),
        origin_rwy: fp.procedures.origin_rwy.clone(),
        dest_rwy: fp.procedures.dest_rwy.clone(),
        waypoints: fp
            .waypoints
            .iter()
            .map(|w| (w.ident.clone(), w.lat, w.lon, w.alt))
            .collect(),
        stages: fp
            .schedule
            .fixes
            .iter()
            .map(|f| (f.ident.clone(), stage(f.stage)))
            .collect(),
        weights: fp.weights.as_ref().map(|w| state::WeightFacts {
            pax: w.pax,
            cargo_kg: w.cargo_kg,
            payload_kg: w.payload_kg,
            est_tow_kg: w.est_tow_kg,
            max_tow_kg: w.max_tow_kg,
            est_ldw_kg: w.est_ldw_kg,
            max_ldw_kg: w.max_ldw_kg,
        }),
        shelved_t: crate::simbrief::plan_shelved_at()
            .and_then(|t| t.duration_since(UNIX_EPOCH).ok())
            .map(|d| d.as_secs_f64()),
    })
}

/// One registry row as the page reads it: its live value, how the terminal
/// renders it, and the variable that pins it.
fn setting_facts(s: &crate::settings::Setting) -> state::SettingFacts {
    use crate::settings::{Kind, Value};
    let v = s.value();
    let value = match (&s.kind, &v) {
        (_, Value::Bool(b)) => if *b { "on" } else { "off" }.to_string(),
        (Kind::Num { decimals, .. }, Value::Num(n)) => format!("{n:.*}", *decimals),
        (_, Value::Num(n)) => n.to_string(),
        (_, Value::Choice(c)) => c.clone(),
        (_, Value::Text(t)) => t.clone(),
    };
    state::SettingFacts {
        key: s.key,
        label: s.label,
        help: s.help,
        kind: match s.kind {
            Kind::Bool => state::SettingKind::Bool,
            Kind::Num {
                min,
                max,
                step,
                unit,
                ..
            } => state::SettingKind::Num {
                min,
                max,
                step,
                unit,
            },
            Kind::Choice(o) => state::SettingKind::Choice(o),
            Kind::Text { .. } => state::SettingKind::Text,
        },
        value,
        shown: s.kind.render(&v),
        pinned_env: s.env_pinned().then_some(s.env),
    }
}

/// Every fact the page reads, from the app's own accessors. Locks only,
/// each taken and dropped in turn; no sim read, no await.
fn gather(server: &Server) -> state::Facts {
    let tick = feed::tick();
    let plan = plan_facts();
    let profile = crate::aircraft::live_profile();
    let com1_khz = feed::com1();
    let com1_tag = com1_khz
        .and_then(crate::station::tuned_station)
        .map(|s| s.tag());
    let station = tick.as_ref().and_then(|t| {
        crate::station::active_station_in_range(t.lat, t.lon, t.alt_ft)
            .map(|(field, services)| state::StationFacts { field, services })
    });
    let controller = server
        .voice
        .as_ref()
        .and_then(|v| v.last_controller_line())
        .map(|c| state::ControllerFacts {
            text: c.text,
            tag: c.speaker.tag(),
            khz: c.khz,
            age_s: c.at.elapsed().as_secs_f64(),
        });
    let metar = server.metar.lock().unwrap_or_else(|e| e.into_inner());
    let metar_age_s = metar
        .modified
        .and_then(|m| SystemTime::now().duration_since(m).ok())
        .map(|d| d.as_secs_f64());
    let arrival = crate::full_auto::arrival_runway().map(|r| (r, "full auto's arrival"));
    let departure = crate::party_line::clearance_runway().map(|r| (r, "cleared"));
    let metars = metar_fields(plan.as_ref())
        .into_iter()
        .map(|(role, icao)| {
            let raw = metar
                .lines
                .iter()
                .find(|(i, _)| *i == icao)
                .and_then(|(_, l)| l.clone());
            let runway = match role {
                "Destination" => arrival.clone().or_else(|| {
                    plan.as_ref()
                        .and_then(|p| p.dest_rwy.clone())
                        .filter(|r| !r.is_empty())
                        .map(|r| (r, "the plan's"))
                }),
                "Origin" => departure.clone().or_else(|| {
                    plan.as_ref()
                        .and_then(|p| p.origin_rwy.clone())
                        .filter(|r| !r.is_empty())
                        .map(|r| (r, "the plan's"))
                }),
                _ => None,
            };
            state::MetarFacts {
                role,
                icao,
                raw,
                runway,
            }
        })
        .collect();
    drop(metar);
    let callsign = feed::callsign();
    let callsign_written = callsign.as_deref().and_then(|spoken| {
        let i = crate::callsign::inputs();
        let ids: Vec<String> = i
            .override_id
            .into_iter()
            .chain(i.plan.map(|p| p.id))
            .collect();
        transcript::written_callsign(spoken, &ids)
    });
    // The one takeoff rule at the plan's takeoff weight, and the one landing
    // rule's book figure: arithmetic over the profile, never a read.
    let reference = profile.as_ref().and_then(|p| p.reference.clone());
    let takeoff_need_m = profile.as_ref().and_then(|p| {
        let (roll, mtow) = crate::aircraft::profile_takeoff_roll(p);
        roll?;
        let tow = plan
            .as_ref()
            .and_then(|p| p.weights.as_ref())
            .map(|w| w.est_tow_kg)
            .filter(|w| *w > 0.0);
        Some(crate::aircraft::takeoff_needed_m(roll, tow, mtow, 500.0))
    });
    let landing_need_m = reference.as_ref().and_then(|r| {
        crate::aircraft::planned_landing_needed_ft(r.landing_distance_ft, r.takeoff_distance_ft)
            .map(|ft| ft * 0.3048)
    });
    state::Facts {
        now_t: now_unix(),
        flight: crate::flight::published(),
        owner: crate::flight_state::crew_has_aircraft().map(|o| o.name()),
        mission: crate::flight_state::crew_mission().map(|o| o.name()),
        auto: crate::auto_status::compose(),
        tick,
        callsign,
        callsign_written,
        lateral: feed::lateral(),
        takeoff_need_m,
        landing_need_m,
        aircraft_type: profile
            .as_ref()
            .and_then(|p| p.matcher.as_ref())
            .and_then(|m| m.icao.first().cloned()),
        aircraft_name: profile.as_ref().map(|p| p.display_name.clone()),
        plan,
        active_fix: crate::procedure::active_leg()
            .map(|l| (l.fix, crate::procedure::fix_distance_nm())),
        tod: crate::procedure::planned_tod().map(|t| state::TodFacts {
            fix: t.fix,
            to_tod_nm: t.to_tod_nm,
            target_ft: t.target_ft,
            started: t.started,
            held_ft: t.held_ft,
        }),
        com1_khz,
        com1_tag,
        station,
        ifr: crate::party_line::assigned_clearance().map(|c| state::IfrFacts {
            dest: c.dest,
            sid: c.sid,
            runway: c.runway,
            initial_alt_ft: c.initial_alt_ft,
            squawk: c.squawk,
        }),
        readback_owed: crate::party_line::readback_owed(),
        controller,
        lines: feed::lines(),
        metars,
        metar_age_s,
        audit: feed::AUDIT.get(),
        preflight: feed::PREFLIGHT.get(),
        last_sent: server
            .sent
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone(),
        settings: crate::settings::ALL.iter().map(setting_facts).collect(),
        alerts: feed::alerts(),
        // A background line (a callout, the cabin) speaks without the loop's
        // own phase: the voice knows it is talking.
        activity: if server.voice.as_ref().is_some_and(|v| v.is_speaking()) {
            Some("speaking")
        } else {
            feed::activity()
        },
        ptt_available: ptt::available(),
        ptt_held: ptt::showing_held(),
        checklist: crate::flow::sheet(),
        proposal: crate::flow::proposal_title(),
        flow_waiting: crate::flow::waiting_for(),
        airport: airport_facts(),
    }
}

/// The Airport page's facts: the station's field, the taxi route the party
/// line holds, the guide's card, the runways. The diagram itself is filled
/// in by [`current`] once the wanted field is known.
fn airport_facts() -> airport::AirportFacts {
    let route = crate::party_line::with_taxi_route(|w| {
        w.map(|w| airport::RouteFacts {
            source: if crate::party_line::clearance_state().is_some() {
                "cleared"
            } else {
                "crew"
            },
            path: w.path.clone(),
            seg: w.seg.clone(),
            via: w.via.clone(),
            holding_point: w.holding_point.clone(),
            to_stand: crate::party_line::taxi_to_stand(),
        })
    });
    airport::AirportFacts {
        station: crate::station::active_icao(),
        diagram: None,
        unavailable: false,
        route,
        card: feed::turn_card(),
        cleared_runway: crate::party_line::clearance_runway(),
        arrival_runway: crate::full_auto::arrival_runway(),
    }
}

/// The page state now, the METAR cache refreshed first when due.
async fn current(server: &Arc<Server>) -> state::EfbState {
    let fields = metar_fields(plan_facts().as_ref())
        .into_iter()
        .map(|(_, i)| i)
        .collect();
    refresh_metars(server, fields).await;
    let mut f = gather(server);
    // The field the Airport page wants: built once, off this thread.
    let want = airport::wanted_field(
        &f.airport,
        f.tick.as_ref(),
        f.plan.as_ref(),
        state::airborne(&f),
    );
    if let Some(w) = want {
        airport::ensure(&w.icao);
        f.airport.diagram = airport::lookup(&w.icao).map(|r| r.reference.clone());
        f.airport.unavailable = airport::unavailable(&w.icao);
    }
    state::build(&f)
}

#[derive(serde::Deserialize, Default)]
struct TokenQuery {
    t: Option<String>,
}

#[derive(serde::Deserialize)]
struct SayBody {
    text: String,
}

#[derive(serde::Deserialize)]
struct SettingBody {
    key: String,
    value: String,
    /// The page asked its question and the pilot said yes.
    #[serde(default)]
    confirm: bool,
}

#[derive(serde::Deserialize)]
struct PttBody {
    /// "press", "beat" or "release".
    action: String,
}

fn live_token() -> String {
    crate::settings::live().efb_token.clone()
}

/// Refuse, blackboxed: who knocked and on which door (never the token tried).
fn refuse(peer: SocketAddr, path: &str) -> Response {
    tracing::warn!(%peer, path, "EFB: refused, wrong or missing token");
    crate::blackbox::record(
        "efb",
        serde_json::json!({ "event": "auth_failed", "peer": peer.ip().to_string(), "path": path }),
    );
    (StatusCode::FORBIDDEN, "forbidden").into_response()
}

/// The page with no token: no data, only a script that retries with the
/// token this device saved (a home-screen launch, a cleared cookie).
const NO_TOKEN_PAGE: &str = r#"<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>XVL EFB</title><style>body{margin:0;background:#0D1117;color:#E6EDF3;font:16px system-ui,sans-serif;padding:32px 20px}</style></head>
<body><p>This EFB needs its link with the token. Open the URL xvl printed at boot (or the EFB settings).</p>
<script>try{var t=localStorage.getItem('xvl_efb_t');if(t&&location.search.indexOf('t=')<0){location.replace('/?t='+encodeURIComponent(t));}}catch(e){}</script>
</body></html>"#;

async fn page(
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    Query(q): Query<TokenQuery>,
    headers: HeaderMap,
) -> Response {
    let token = live_token();
    if !authorised(&token, &headers, q.t.as_deref()) {
        refuse(peer, "/");
        return (StatusCode::FORBIDDEN, Html(NO_TOKEN_PAGE)).into_response();
    }
    let mut r = Html(PAGE).into_response();
    if let Ok(v) = HeaderValue::from_str(&format!(
        "{COOKIE}={token}; Path=/; Max-Age=31536000; SameSite=Strict; HttpOnly"
    )) {
        r.headers_mut().insert(header::SET_COOKIE, v);
    }
    r.headers_mut()
        .insert(header::CACHE_CONTROL, HeaderValue::from_static("no-store"));
    r
}

/// The web app manifest: "Add to Home Screen" opens full screen, with the
/// token in its start URL (a home-screen app has its own cookie jar).
pub(crate) fn manifest(token: &str) -> serde_json::Value {
    serde_json::json!({
        "name": "XVoiceLayer EFB",
        "short_name": "XVL EFB",
        "start_url": format!("/?t={token}"),
        "scope": "/",
        "display": "standalone",
        "orientation": "any",
        "background_color": "#0D1117",
        "theme_color": "#11161D",
        "icons": [
            { "src": "/icon-192.png", "sizes": "192x192", "type": "image/png", "purpose": "any" },
            { "src": "/icon-512.png", "sizes": "512x512", "type": "image/png", "purpose": "any" },
            { "src": "/icon.svg", "sizes": "any", "type": "image/svg+xml", "purpose": "any" }
        ]
    })
}

async fn manifest_route(
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    Query(q): Query<TokenQuery>,
    headers: HeaderMap,
) -> Response {
    let token = live_token();
    if !authorised(&token, &headers, q.t.as_deref()) {
        return refuse(peer, "/manifest.webmanifest");
    }
    (
        [(header::CONTENT_TYPE, "application/manifest+json")],
        manifest(&token).to_string(),
    )
        .into_response()
}

/// The home-screen icon: no data in it, so it needs no token.
const ICON: &str = r##"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 180 180"><rect width="180" height="180" rx="36" fill="#0D1117"/><rect x="18" y="18" width="144" height="144" rx="26" fill="none" stroke="#2E6E8E" stroke-width="6"/><text x="90" y="112" font-family="Helvetica,Arial,sans-serif" font-size="58" font-weight="700" fill="#7FD3F7" text-anchor="middle">XVL</text></svg>"##;

async fn icon() -> Response {
    ([(header::CONTENT_TYPE, "image/svg+xml")], ICON).into_response()
}

/// The home-screen PNGs (`tools/efb_icon.py` draws them): iOS fetches the
/// touch icon without the page's cookie, so like the SVG they need no token.
const ICON_180: &[u8] = include_bytes!("icons/icon-180.png");
const ICON_192: &[u8] = include_bytes!("icons/icon-192.png");
const ICON_512: &[u8] = include_bytes!("icons/icon-512.png");

fn png(bytes: &'static [u8]) -> Response {
    (
        [
            (header::CONTENT_TYPE, "image/png"),
            (header::CACHE_CONTROL, "public, max-age=86400"),
        ],
        bytes,
    )
        .into_response()
}

async fn state_route(
    State(server): State<Arc<Server>>,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    Query(q): Query<TokenQuery>,
    headers: HeaderMap,
) -> Response {
    if !authorised(&live_token(), &headers, q.t.as_deref()) {
        return refuse(peer, "/state");
    }
    let s = current(&server).await;
    ([(header::CACHE_CONTROL, "no-store")], Json(s)).into_response()
}

#[derive(serde::Deserialize, Default)]
struct AirportQuery {
    t: Option<String>,
    icao: Option<String>,
}

/// The diagram of a field the page state named (`airport.icao`), with its
/// version as the ETag: the page fetches it once per field. 404 while it is
/// still being built, or for a field nobody asked for.
async fn airport_route(
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    Query(q): Query<AirportQuery>,
    headers: HeaderMap,
) -> Response {
    if !authorised(&live_token(), &headers, q.t.as_deref()) {
        return refuse(peer, "/airport");
    }
    let icao = q.icao.unwrap_or_default().trim().to_ascii_uppercase();
    let Some(ready) = airport::lookup(&icao) else {
        return (StatusCode::NOT_FOUND, "no diagram for that field yet").into_response();
    };
    let etag = format!("\"{}\"", ready.reference.version);
    let fresh = headers
        .get(header::IF_NONE_MATCH)
        .and_then(|v| v.to_str().ok())
        .is_some_and(|v| v.split(',').any(|t| t.trim() == etag));
    let mut r = if fresh {
        StatusCode::NOT_MODIFIED.into_response()
    } else {
        (
            [(header::CONTENT_TYPE, "application/json")],
            ready.json.clone(),
        )
            .into_response()
    };
    if let Ok(v) = HeaderValue::from_str(&etag) {
        r.headers_mut().insert(header::ETAG, v);
    }
    r.headers_mut()
        .insert(header::CACHE_CONTROL, HeaderValue::from_static("no-cache"));
    r
}

/// The Airport page's script: code, no data, so like the icon it needs no
/// token (a `<script src>` carries no header, and a home-screen app's first
/// load must not depend on the cookie having landed).
async fn diagram_js() -> Response {
    (
        [
            (header::CONTENT_TYPE, "text/javascript; charset=utf-8"),
            (header::CACHE_CONTROL, "no-store"),
        ],
        DIAGRAM_JS,
    )
        .into_response()
}

async fn events(
    State(server): State<Arc<Server>>,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    Query(q): Query<TokenQuery>,
    headers: HeaderMap,
) -> Response {
    if !authorised(&live_token(), &headers, q.t.as_deref()) {
        return refuse(peer, "/events");
    }
    // Change-gated: the state is sent when its JSON differs from the last
    // one sent (the clock field excluded), else nothing but keep-alives.
    let stream = futures::stream::unfold(
        (server, None::<String>, true),
        |(server, last, mut first)| async move {
            loop {
                if !first {
                    tokio::time::sleep(PUSH_EVERY).await;
                }
                first = false;
                let s = current(&server).await;
                let mut v = serde_json::to_value(&s).unwrap_or_default();
                let body = v.to_string();
                if let Some(o) = v.as_object_mut() {
                    o.remove("t");
                    // Ages tick every second; they ride along when anything else moves.
                    if let Some(r) = o.get_mut("radio").and_then(|r| r.as_object_mut()) {
                        if let Some(c) = r.get_mut("last_call").and_then(|c| c.as_object_mut()) {
                            c.remove("age_s");
                        }
                    }
                    if let Some(w) = o.get_mut("weather").and_then(|w| w.as_object_mut()) {
                        w.remove("file_age_s");
                    }
                }
                let key = v.to_string();
                if last.as_deref() != Some(key.as_str()) {
                    let ev: Result<Event, std::convert::Infallible> =
                        Ok(Event::default().data(body));
                    return Some((ev, (server, Some(key), false)));
                }
                // Unchanged: nothing sent (the keep-alive covers the quiet).
            }
        },
    );
    Sse::new(stream)
        .keep_alive(KeepAlive::new().interval(Duration::from_secs(15)))
        .into_response()
}

async fn say(
    State(server): State<Arc<Server>>,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    Query(q): Query<TokenQuery>,
    headers: HeaderMap,
    body: Option<Json<SayBody>>,
) -> Response {
    if !authorised(&live_token(), &headers, q.t.as_deref()) {
        return refuse(peer, "/say");
    }
    let Some(Json(body)) = body else {
        return (StatusCode::BAD_REQUEST, "expected {\"text\": ...}").into_response();
    };
    let Some(text) = clean_line(&body.text) else {
        return (StatusCode::BAD_REQUEST, "nothing to say").into_response();
    };
    {
        let mut last = server.last_say.lock().unwrap_or_else(|e| e.into_inner());
        if last.is_some_and(|t| t.elapsed() < SAY_MIN_GAP) {
            crate::blackbox::record(
                "efb",
                serde_json::json!({ "event": "say_limited", "text": text }),
            );
            return (StatusCode::TOO_MANY_REQUESTS, "one line at a time").into_response();
        }
        *last = Some(Instant::now());
    }
    // The panel's typed line, exactly: stamped on the channel, blackboxed
    // `typed` `taken` (the command loop then logs `consumed` and `routed`).
    tracing::info!(typed = %text, "EFB line");
    crate::blackbox::record("efb", serde_json::json!({ "event": "say", "text": text }));
    crate::blackbox::record(
        "typed",
        serde_json::json!({ "event": "taken", "text": text, "source": "efb" }),
    );
    let sent = {
        let tx = server.tx.lock().unwrap_or_else(|e| e.into_inner());
        crate::pollers::queue_command(&tx, text.clone())
    };
    if !sent {
        return (StatusCode::SERVICE_UNAVAILABLE, "the command loop is gone").into_response();
    }
    *server.sent.lock().unwrap_or_else(|e| e.into_inner()) = Some((now_unix(), text.clone()));
    Json(serde_json::json!({ "sent": text })).into_response()
}

/// A setting change from the page, judged as the registry would: `Ok` with
/// the value as now shown, else the status and the line the page says. The
/// EFB's own port and token, and a pinned knob, are refused here as on the
/// page; the mode and turning the EFB off need the page's confirm.
pub(crate) fn setting_change(
    key: &str,
    raw: &str,
    confirmed: bool,
) -> Result<String, (StatusCode, String)> {
    let s = crate::settings::ALL
        .iter()
        .find(|s| s.key == key)
        .ok_or((StatusCode::NOT_FOUND, format!("no such setting: {key}")))?;
    if let Some(why) = state::setting_locked(key, s.env_pinned().then_some(s.env)) {
        return Err((StatusCode::FORBIDDEN, why));
    }
    let v = s.kind.parse(raw).ok_or((
        StatusCode::BAD_REQUEST,
        format!("{} cannot be {}", s.label, raw.trim()),
    ))?;
    if state::needs_confirm(key, &s.kind.render(&v)) && !confirmed {
        return Err((StatusCode::CONFLICT, "confirm first".into()));
    }
    crate::settings::set(key, &v).map_err(|e| (StatusCode::BAD_REQUEST, e))?;
    Ok(s.rendered())
}

async fn setting(
    State(server): State<Arc<Server>>,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    Query(q): Query<TokenQuery>,
    headers: HeaderMap,
    body: Option<Json<SettingBody>>,
) -> Response {
    if !authorised(&live_token(), &headers, q.t.as_deref()) {
        return refuse(peer, "/setting");
    }
    let Some(Json(body)) = body else {
        return (StatusCode::BAD_REQUEST, "expected {\"key\": ..., \"value\": ...}").into_response();
    };
    {
        let mut last = server
            .last_setting
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        if last.is_some_and(|t| t.elapsed() < SETTING_MIN_GAP) {
            return (StatusCode::TOO_MANY_REQUESTS, "one change at a time").into_response();
        }
        *last = Some(Instant::now());
    }
    let (key, value) = (body.key.clone(), body.value.clone());
    // The registry's setter persists to prefs.json: a file write, off the
    // runtime's thread.
    let out = tokio::task::spawn_blocking(move || setting_change(&key, &value, body.confirm))
        .await
        .unwrap_or_else(|_| Err((StatusCode::INTERNAL_SERVER_ERROR, "setter failed".into())));
    // The token is never written to the log, whatever was sent.
    let logged = if body.key == "efb_token" {
        String::new()
    } else {
        body.value.clone()
    };
    match out {
        Ok(shown) => {
            tracing::info!(key = %body.key, value = %shown, "EFB setting");
            crate::blackbox::record(
                "efb",
                serde_json::json!({ "event": "setting", "key": body.key, "value": logged }),
            );
            Json(serde_json::json!({ "key": body.key, "shown": shown })).into_response()
        }
        Err((code, why)) => {
            crate::blackbox::record(
                "efb",
                serde_json::json!({ "event": "setting_refused", "key": body.key, "value": logged, "why": why }),
            );
            (code, why).into_response()
        }
    }
}

async fn ptt_route(
    State(server): State<Arc<Server>>,
    ConnectInfo(peer): ConnectInfo<SocketAddr>,
    Query(q): Query<TokenQuery>,
    headers: HeaderMap,
    body: Option<Json<PttBody>>,
) -> Response {
    if !authorised(&live_token(), &headers, q.t.as_deref()) {
        return refuse(peer, "/ptt");
    }
    let Some(Json(body)) = body else {
        return (StatusCode::BAD_REQUEST, "expected {\"action\": ...}").into_response();
    };
    let action = body.action.as_str();
    if action != "release" {
        let mut last = server.last_ptt.lock().unwrap_or_else(|e| e.into_inner());
        if last.is_some_and(|t| t.elapsed() < PTT_MIN_GAP) {
            return (StatusCode::TOO_MANY_REQUESTS, "too fast").into_response();
        }
        *last = Some(Instant::now());
    }
    let held = match action {
        "press" => {
            if !ptt::press() {
                return (
                    StatusCode::CONFLICT,
                    "xvl is not listening on push to talk in this run",
                )
                    .into_response();
            }
            true
        }
        "beat" => ptt::beat(),
        "release" => {
            ptt::release();
            false
        }
        _ => return (StatusCode::BAD_REQUEST, "press, beat or release").into_response(),
    };
    Json(serde_json::json!({ "held": held })).into_response()
}

/// The routes, on a server.
pub(crate) fn router(server: Arc<Server>) -> Router {
    Router::new()
        .route("/", get(page))
        .route("/state", get(state_route))
        .route("/events", get(events))
        .route("/say", post(say))
        .route("/setting", post(setting))
        .route("/ptt", post(ptt_route))
        .route("/airport", get(airport_route))
        .route("/diagram.js", get(diagram_js))
        .route("/manifest.webmanifest", get(manifest_route))
        .route("/icon.svg", get(icon))
        .route("/apple-touch-icon.png", get(|| async { png(ICON_180) }))
        .route("/icon-180.png", get(|| async { png(ICON_180) }))
        .route("/icon-192.png", get(|| async { png(ICON_192) }))
        .route("/icon-512.png", get(|| async { png(ICON_512) }))
        .with_state(server)
}

/// This machine's LAN address, for the URL in the log: the source address
/// a UDP socket would use toward a public address (nothing is sent).
fn lan_ip() -> String {
    std::net::UdpSocket::bind("0.0.0.0:0")
        .and_then(|s| {
            s.connect("192.0.2.1:80")?;
            s.local_addr()
        })
        .map(|a| a.ip().to_string())
        .unwrap_or_else(|_| "127.0.0.1".to_string())
}

/// The token in force, generated and saved at the first enable when none is
/// set (a pinned empty env var gets one for this run only).
fn ensure_token() -> String {
    let have = live_token();
    if token_ok(&have) {
        return have;
    }
    let fresh = new_token();
    match crate::settings::set("efb_token", &crate::settings::Value::Text(fresh.clone())) {
        Ok(()) => {}
        Err(e) => {
            tracing::warn!("EFB: token not saved ({e}); using one for this run");
            crate::settings::update(|c| c.efb_token = fresh.clone());
        }
    }
    fresh
}

/// Start the EFB's supervisor: its own thread and runtime, idle while the
/// setting is off, serving while it is on, rebinding when the port changes.
/// `tx` is the command channel the panel's typed line uses.
pub(crate) fn spawn(voice: Voice, tx: std::sync::mpsc::Sender<String>, print_url: bool) {
    std::thread::Builder::new()
        .name("efb".into())
        .spawn(move || {
            let Ok(rt) = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
            else {
                return;
            };
            rt.block_on(supervise(Server::new(Some(voice), tx), print_url));
        })
        .ok();
}

async fn supervise(server: Arc<Server>, print_url: bool) {
    let mut announced = false;
    loop {
        let cfg = crate::settings::live();
        if !cfg.efb {
            tokio::time::sleep(Duration::from_secs(2)).await;
            continue;
        }
        let port = if cfg.efb_port >= 1024 {
            cfg.efb_port
        } else {
            8787
        };
        drop(cfg);
        let token = ensure_token();
        let listener = match tokio::net::TcpListener::bind(("0.0.0.0", port)).await {
            Ok(l) => l,
            Err(e) => {
                tracing::warn!(port, "EFB: cannot listen: {e}");
                crate::blackbox::record(
                    "efb",
                    serde_json::json!({ "event": "bind_failed", "port": port, "error": e.to_string() }),
                );
                tokio::time::sleep(Duration::from_secs(10)).await;
                continue;
            }
        };
        let url = format!("http://{}:{port}/?t={token}", lan_ip());
        if !announced && print_url {
            // Once per process at the first bind: the bookmark for the iPad
            // (the terminal dashboard owns the screen: there, the log only).
            println!("\n  EFB: {url}\n");
        }
        announced = true;
        *SERVING.lock().unwrap_or_else(|e| e.into_inner()) = Some(url.clone());
        tracing::info!(%url, "EFB serving");
        crate::blackbox::record(
            "efb",
            serde_json::json!({ "event": "start", "port": port, "host": lan_ip() }),
        );
        // Serve until the setting turns off or the port moves.
        let stop = async move {
            loop {
                tokio::time::sleep(Duration::from_secs(2)).await;
                let c = crate::settings::live();
                if !c.efb || (c.efb_port >= 1024 && c.efb_port != port) {
                    break;
                }
            }
        };
        let app = router(server.clone()).into_make_service_with_connect_info::<SocketAddr>();
        if let Err(e) = axum::serve(listener, app)
            .with_graceful_shutdown(stop)
            .await
        {
            tracing::warn!("EFB server ended: {e}");
        }
        *SERVING.lock().unwrap_or_else(|e| e.into_inner()) = None;
        crate::blackbox::record("efb", serde_json::json!({ "event": "stop", "port": port }));
    }
}

/// The link the server is answering on, while it is: the terminal shows it
/// (the dashboard owns the screen, so the boot print is plain mode only).
static SERVING: std::sync::Mutex<Option<String>> = std::sync::Mutex::new(None);

/// The EFB link now, `None` while it is off or not yet listening.
pub fn serving_url() -> Option<String> {
    SERVING.lock().unwrap_or_else(|e| e.into_inner()).clone()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tokens_are_fresh_and_well_formed() {
        let a = new_token();
        let b = new_token();
        assert_eq!(a.len(), 32);
        assert!(token_ok(&a));
        assert_ne!(a, b);
        assert!(!token_ok("short"));
        assert!(!token_ok("has space in it"));
        assert!(!token_ok(""));
    }

    #[test]
    fn a_request_is_authorised_by_query_header_or_cookie_only_with_the_token() {
        let tok = "abcdef0123456789";
        let mut h = HeaderMap::new();
        assert!(!authorised(tok, &h, None));
        assert!(authorised(tok, &h, Some(tok)));
        assert!(!authorised(tok, &h, Some("abcdef012345678x")));
        assert!(
            !authorised("", &h, Some("")),
            "no token set, nothing passes"
        );
        h.insert(
            header::COOKIE,
            HeaderValue::from_static("a=b; xvl_efb=abcdef0123456789"),
        );
        assert!(authorised(tok, &h, None));
        let mut h = HeaderMap::new();
        h.insert("x-efb-token", HeaderValue::from_static("abcdef0123456789"));
        assert!(authorised(tok, &h, None));
    }

    #[test]
    fn a_line_is_one_clean_line() {
        assert_eq!(
            clean_line("  flaps\nup \u{2014} now "),
            Some("flaps up, now".into())
        );
        assert_eq!(clean_line(" \t "), None);
        assert_eq!(
            clean_line(&"a".repeat(500)).map(|s| s.len()),
            Some(SAY_MAX_CHARS)
        );
    }

    #[test]
    fn the_manifest_opens_full_screen_with_the_token() {
        let m = manifest("abcdef0123456789");
        assert_eq!(m["display"], "standalone");
        assert_eq!(m["start_url"], "/?t=abcdef0123456789");
        let sizes: Vec<&str> = m["icons"]
            .as_array()
            .unwrap()
            .iter()
            .map(|i| i["sizes"].as_str().unwrap())
            .collect();
        assert_eq!(sizes, ["192x192", "512x512", "any"]);
    }

    /// The checked-in icons are the PNGs their names say (`tools/efb_icon.py`).
    #[test]
    fn the_icons_are_pngs_of_their_size() {
        for (bytes, size) in [(ICON_180, 180u32), (ICON_192, 192), (ICON_512, 512)] {
            assert_eq!(&bytes[..8], b"\x89PNG\r\n\x1a\n");
            assert_eq!(&bytes[12..16], b"IHDR");
            let w = u32::from_be_bytes(bytes[16..20].try_into().unwrap());
            let h = u32::from_be_bytes(bytes[20..24].try_into().unwrap());
            assert_eq!((w, h), (size, size));
        }
    }

    /// The page cannot change its own link, a pinned knob, or the mode
    /// without its confirm; nothing here reaches the setter (which persists).
    #[test]
    fn a_setting_from_the_page_is_judged_before_the_setter() {
        let _g = crate::settings::TEST_LOCK
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        let code = |k: &str, v: &str, c: bool| setting_change(k, v, c).map_err(|(s, _)| s);
        assert_eq!(code("efb_token", "abcdefgh1234", true), Err(StatusCode::FORBIDDEN));
        assert_eq!(code("efb_port", "9000", true), Err(StatusCode::FORBIDDEN));
        assert_eq!(code("nonsense", "1", true), Err(StatusCode::NOT_FOUND));
        if std::env::var("XCOPILOT_MODE").is_err() {
            assert_eq!(code("mode", "instructor", false), Err(StatusCode::CONFLICT));
            assert_eq!(code("mode", "sideways", true), Err(StatusCode::BAD_REQUEST));
        }
        if std::env::var("XCOPILOT_EFB").is_err() {
            assert_eq!(code("efb", "false", false), Err(StatusCode::CONFLICT), "off needs the confirm");
        }
        if std::env::var("XCOPILOT_VOICE_VOL").is_err() {
            assert_eq!(code("voice_volume", "9", true), Err(StatusCode::BAD_REQUEST));
        }
    }

    /// The page ships in the binary and has no dash a font chokes on.
    #[test]
    fn the_page_is_embedded_and_dash_free() {
        assert!(PAGE.contains("<title>"));
        assert!(PAGE.contains("apple-mobile-web-app-capable"));
        assert!(PAGE.contains("viewport-fit=cover"));
        assert!(!PAGE.contains('\u{2014}') && !PAGE.contains('\u{2013}'));
        assert!(PAGE.contains("<script src=\"/diagram.js\"></script>"));
        for tab in ["flight", "map", "plan", "radio"] {
            assert!(
                PAGE.contains(&format!("data-tab=\"{tab}\"")),
                "the {tab} tab"
            );
        }
        assert!(
            PAGE.contains("id=\"t-settings\""),
            "settings behind the gear"
        );
        assert!(!PAGE.contains("more-sheet"), "four tabs, no More sheet");
        assert!(DIAGRAM_JS.contains("window.XvlDiagram"));
        assert!(!DIAGRAM_JS.contains('\u{2014}') && !DIAGRAM_JS.contains('\u{2013}'));
        assert!(!DIAGRAM_JS.contains("http"), "no tile, no CDN: this server only");
    }

    /// The server against a fixture-free app: the page, the state and a
    /// line, over real HTTP on a loopback port. The command channel is a
    /// plain mpsc here; the line lands on it exactly as the panel's would.
    #[test]
    fn the_server_answers_with_the_token_and_queues_a_line() {
        let _g = crate::settings::TEST_LOCK
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        let _q = crate::pollers::QUEUE_TEST_LOCK
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        crate::settings::update(|c| c.efb_token = "testtoken0123456".into());
        let (tx, rx) = std::sync::mpsc::channel::<String>();
        let server = Server::new(None, tx);
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        rt.block_on(async move {
            let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
            let addr = listener.local_addr().unwrap();
            let app = router(server).into_make_service_with_connect_info::<SocketAddr>();
            tokio::spawn(async move {
                let _ = axum::serve(listener, app).await;
            });
            let http = reqwest::Client::builder().no_proxy().build().unwrap();
            let base = format!("http://{addr}");
            let r = http.get(format!("{base}/state")).send().await.unwrap();
            assert_eq!(r.status(), 403);
            let r = http
                .get(format!("{base}/?t=wrongtoken000000"))
                .send()
                .await
                .unwrap();
            assert_eq!(r.status(), 403);
            let r = http
                .get(format!("{base}/?t=testtoken0123456"))
                .send()
                .await
                .unwrap();
            assert_eq!(r.status(), 200);
            assert!(r.headers().get("set-cookie").is_some());
            let r = http
                .get(format!("{base}/state"))
                .header("x-efb-token", "testtoken0123456")
                .send()
                .await
                .unwrap();
            assert_eq!(r.status(), 200);
            let v: serde_json::Value = r.json().await.unwrap();
            assert_eq!(v["my_controls"]["say"], "my controls");
            assert_eq!(
                v["flight"]["layout"], "parked",
                "no flight model: the ramp's layout"
            );
            assert!(v["transcript"].is_array());
            assert!(v["map"]["mode"].is_string());
            let say = |text: &str| {
                http.post(format!("{base}/say?t=testtoken0123456"))
                    .json(&serde_json::json!({ "text": text }))
                    .send()
            };
            assert_eq!(say("my controls").await.unwrap().status(), 200);
            assert_eq!(
                say("my controls").await.unwrap().status(),
                429,
                "rate limited"
            );
            tokio::time::sleep(SAY_MIN_GAP).await;
            assert_eq!(say("   ").await.unwrap().status(), 400);
            // The icons load with no token (iOS fetches them cookie-less).
            let r = http
                .get(format!("{base}/apple-touch-icon.png"))
                .send()
                .await
                .unwrap();
            assert_eq!(r.status(), 200);
            assert_eq!(r.headers()["content-type"], "image/png");
            // A setting the page may not change, refused with the token.
            let r = http
                .post(format!("{base}/setting?t=testtoken0123456"))
                .json(&serde_json::json!({ "key": "efb_token", "value": "zzzzzzzz" }))
                .send()
                .await
                .unwrap();
            assert_eq!(r.status(), 403);
            let r = http
                .post(format!("{base}/setting"))
                .json(&serde_json::json!({ "key": "mode", "value": "both" }))
                .send()
                .await
                .unwrap();
            assert_eq!(r.status(), 403, "no token, no setting");
            // Push to talk: refused while this run does not capture on it,
            // a release always taken.
            let ptt_post = |action: &str| {
                http.post(format!("{base}/ptt?t=testtoken0123456"))
                    .json(&serde_json::json!({ "action": action }))
                    .send()
            };
            let _p = ptt::TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
            ptt::set_available(false);
            assert_eq!(ptt_post("press").await.unwrap().status(), 409);
            assert_eq!(ptt_post("release").await.unwrap().status(), 200);
            ptt::set_available(true);
            tokio::time::sleep(PTT_MIN_GAP).await;
            let r = ptt_post("press").await.unwrap();
            assert_eq!(r.status(), 200);
            assert!(ptt::held());
            assert_eq!(ptt_post("release").await.unwrap().status(), 200);
            assert!(!ptt::held());
            ptt::set_available(false);
        });
        assert_eq!(rx.try_recv().as_deref(), Ok("my controls"));
        assert!(rx.try_recv().is_err(), "the limited line never queued");
        // Taken off the channel's stamp queue, as the command loop would.
        assert!(crate::pollers::took_command().is_some());
        crate::settings::update(|c| c.efb_token = String::new());
    }
}
