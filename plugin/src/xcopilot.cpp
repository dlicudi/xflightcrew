// x-copilot X-Plane plugin — a virtual push-to-talk plus a slick in-sim status
// panel (Dear ImGui) for the x-copilot voice assistant.
//
// PTT: command `xcopilot/ptt` (bind to a button/key) drives the readable dataref
// `xcopilot/ptt_active` (1 while held). The app captures the mic only while it's
// 1 — no radio keying, no macOS permission.
//
// Panel: writable datarefs the app pushes into —
//   xcopilot/status/phase  (int: 0 idle, 1 listening, 2 thinking, 3 speaking)
//   xcopilot/status/heard  (text: last transcript)
//   xcopilot/status/action (text: last readback)
//   xcopilot/status/radio  (text: comm plan + live COM1, '>' = tuned service)
//   xcopilot/status/atc    (text: in-range stations, "NAME,nm,SVC khz,...|...")
// rendered as a toolbar of status chips over tabs: Log (scrolling transmission
// record), Route (the pushed flight plan + a Send-to-FMS re-push), ATC (every
// station in VHF range, click a frequency to tune COM1), and Settings (live
// switches, e.g. the LLM-fallback dataref `xcopilot/llm_enabled`, -1 auto
// / 0 off / 1 on, which the app polls each utterance).
// Toggle via Plugins → x-copilot.
//
// Build/install: see plugin/README.md.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <memory>
#include <string>
#include <vector>

#include "XPLMDataAccess.h"
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMMenus.h"
#include "XPLMNavigation.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"

// Project version. Single version for the whole product (app + plugin ship and
// must be compatible as a pair); keep in sync with Cargo.toml.
#define XCOPILOT_VERSION "0.1.0"
// Git build id, injected by the Makefile (-DXCOPILOT_BUILD=...). Lets us confirm
// from the log/panel exactly which build is loaded after a reload. "dev" if built
// outside the Makefile.
#ifndef XCOPILOT_BUILD
#define XCOPILOT_BUILD "dev"
#endif

#include "imgui.h"
#include "ImgWindow.h"
#include "ImgFontAtlas.h"

// ---- colours ----------------------------------------------------------------

static const ImU32 COL_GREEN = IM_COL32(120, 225, 140, 255);
static const ImU32 COL_DIM   = IM_COL32(150, 155, 165, 255);
static const ImU32 COL_AMBER = IM_COL32(255, 185, 40, 255);
static const ImU32 COL_CYAN  = IM_COL32(90, 200, 255, 255);
static const ImU32 COL_WHITE = IM_COL32(235, 235, 240, 255);
static const ImU32 COL_RED   = IM_COL32(240, 90, 80, 255);

// ---- PTT --------------------------------------------------------------------

static int g_ptt_active = 0;
static XPLMCommandRef g_ptt_cmd = nullptr;
static XPLMDataRef g_ptt_dref = nullptr;

static int PttCommandHandler(XPLMCommandRef, XPLMCommandPhase phase, void*) {
    if (phase == xplm_CommandBegin) g_ptt_active = 1;
    else if (phase == xplm_CommandEnd) g_ptt_active = 0;
    return 1;
}
static int ReadPttActive(void*) { return g_ptt_active; }

// Radio PTT: the transmit switch, as distinct from the intercom above. While
// held, the app routes the utterance to whatever ATC station COM1 is tuned to
// (never to the copilot). Command `xcopilot/radio_ptt` drives the readable
// dataref `xcopilot/radio_ptt_active`, mirroring the intercom pair exactly.
static int g_radio_ptt_active = 0;
static XPLMCommandRef g_radio_ptt_cmd = nullptr;
static XPLMDataRef g_radio_ptt_dref = nullptr;

static int RadioPttCommandHandler(XPLMCommandRef, XPLMCommandPhase phase, void*) {
    if (phase == xplm_CommandBegin) g_radio_ptt_active = 1;
    else if (phase == xplm_CommandEnd) g_radio_ptt_active = 0;
    return 1;
}
static int ReadRadioPttActive(void*) { return g_radio_ptt_active; }

// ---- LLM fallback toggle ----------------------------------------------------

// A runtime switch the panel flips and the app reads live (each utterance), so
// the answer-only LLM fallback can be turned on/off mid-flight without a restart.
//   -1 = untouched → the app follows its own XCOPILOT_LLM startup default
//    0 = forced off
//    1 = forced on
static int g_llm_enabled = -1;
static XPLMDataRef g_llm_dref = nullptr;
static int ReadLlmEnabled(void*) { return g_llm_enabled; }
static void WriteLlmEnabled(void*, int v) { g_llm_enabled = v; }

// ---- panel requests (plugin → app) -------------------------------------------

// The emergency buttons write a request code the app polls and consumes (same
// direction as PTT): 0 = none, 1 = recover, 2 = autoland, 3 = my controls. The
// app runs the matching voice command and writes 0 back.
static int g_request = 0;
static XPLMDataRef g_request_dref = nullptr;
static int ReadRequest(void*) { return g_request; }
static void WriteRequest(void*, int v) { g_request = v; }

// ---- panel options (plugin → app) --------------------------------------------

// Settings-popup switches the app polls live. Ints so a missing plugin (dataref
// unresolved) is distinguishable from "off"; defaults match the app's defaults.
static int g_opt_camera_snap = 0;   // snap-to-panel camera on actions — OFF by
                                    // default (moving the pilot's view proved
                                    // disorienting; the callout toast carries the
                                    // information). Re-enable via Settings.
static int g_opt_negative_tone = 1; // negative tone on unrecognized command
static int g_opt_monitor_mute = 0;  // silence the monitor's callouts
static XPLMDataRef g_opt_snap_dref = nullptr;
static XPLMDataRef g_opt_tone_dref = nullptr;
static XPLMDataRef g_opt_mute_dref = nullptr;
static int ReadOptSnap(void*) { return g_opt_camera_snap; }
static void WriteOptSnap(void*, int v) { g_opt_camera_snap = v; }
static int ReadOptTone(void*) { return g_opt_negative_tone; }
static void WriteOptTone(void*, int v) { g_opt_negative_tone = v; }
static int ReadOptMute(void*) { return g_opt_monitor_mute; }
static void WriteOptMute(void*, int v) { g_opt_monitor_mute = v; }

// ---- status datarefs --------------------------------------------------------

struct TextRef { char buf[512]; int len; };

static int g_phase = 0;
static TextRef g_heard = {{0}, 0};
static TextRef g_action = {{0}, 0};
static TextRef g_alert = {{0}, 0};
static TextRef g_aircraft = {{0}, 0};
// Live flight strip — one compact status line the app refreshes (emergency
// phase/runway, cabin altitude, fuel endurance). Empty = hidden.
static TextRef g_strip = {{0}, 0};
// On-screen callout — WHAT the copilot is changing ("FLAPS > 10"), flashed in a
// floating toast over the sim for a couple of seconds after each action.
static TextRef g_callout = {{0}, 0};
static float g_callout_time = -1000.0f;
// Radio row — the ATC field's comm plan plus live COM1, refreshed by the app
// ("GIBRALTAR | >TWR 131.200 | APP 122.800"). '>' marks the tuned service.
static TextRef g_radio = {{0}, 0};
// ATC tab — every station in VHF range, nearest first, refreshed by the app
// as "NAME,dist_nm,SVC khz,...|NAME,..." ('*' prefix = the active station).
static TextRef g_atc = {{0}, 0};
// Calls tab — the radio coach's suggested next call for the current tower
// state, refreshed continuously by the app. Empty = no station.
static TextRef g_coach = {{0}, 0};

static XPLMDataRef g_phase_dref = nullptr;
static XPLMDataRef g_heard_dref = nullptr;
static XPLMDataRef g_action_dref = nullptr;
static XPLMDataRef g_alert_dref = nullptr;
static XPLMDataRef g_aircraft_dref = nullptr;
static XPLMDataRef g_strip_dref = nullptr;
static XPLMDataRef g_callout_dref = nullptr;
static XPLMDataRef g_radio_dref = nullptr;
static XPLMDataRef g_atc_dref = nullptr;
static XPLMDataRef g_coach_dref = nullptr;
// COM1 active frequency (integer kHz despite the sim's `hz` name) — read for
// the ATC tab's tuned highlight, written by its click-to-tune.
static XPLMDataRef g_com1_dref = nullptr;

// Transmission log shown in the panel, each entry stamped with wall-clock time
// so a test flight can be reviewed afterwards.
struct LogEntry { std::string time; std::string tag; ImU32 color; std::string text; };
static std::vector<LogEntry> g_log;
static bool g_log_scroll = false;
static bool g_log_autoscroll = true;
// New entries arrived since the Log tab was last shown — drives an unread dot
// on the tab so activity isn't missed while on Route/Settings.
static bool g_log_unseen = false;

static void AppendLog(const char* tag, ImU32 color, const char* text) {
    if (text == nullptr || text[0] == '\0') return;
    char ts[16] = {0};
    time_t now = time(nullptr);
    struct tm lt;
    localtime_r(&now, &lt);
    strftime(ts, sizeof ts, "%H:%M:%S", &lt);
    g_log.push_back({ts, tag, color, text});
    if (g_log.size() > 200) g_log.erase(g_log.begin());
    if (g_log_autoscroll) g_log_scroll = true;
    g_log_unseen = true;
}

// Last time the app wrote any status (sim seconds). The app heartbeats the phase
// ~1/s, so a stale value means it isn't running → NOT CONNECTED. Starts stale.
static float g_last_status_time = -1000.0f;

static int ReadPhase(void*) { return g_phase; }
static void WritePhase(void*, int v) {
    g_phase = v;
    g_last_status_time = XPLMGetElapsedTime();
}

static int ReadText(void* ref, void* out, int offset, int max) {
    TextRef* t = static_cast<TextRef*>(ref);
    if (out == nullptr) return t->len;
    if (offset >= t->len) return 0;
    int n = t->len - offset;
    if (n > max) n = max;
    std::memcpy(out, t->buf + offset, static_cast<size_t>(n));
    return n;
}

// Store the whole value (the app PATCHes at offset 0), NUL-terminated. Any text
// write is proof the app is alive, so it refreshes the connection heartbeat too
// (not just phase writes — a busy strip stream must not read as disconnected).
static void StoreText(TextRef* t, void* in, int offset, int len) {
    g_last_status_time = XPLMGetElapsedTime();
    if (offset != 0) return;
    if (len > 511) len = 511;
    if (len < 0) len = 0;
    std::memset(t->buf, 0, sizeof t->buf);
    std::memcpy(t->buf, in, static_cast<size_t>(len));
    t->len = len;
}

static void WriteHeard(void*, void* in, int offset, int len) {
    StoreText(&g_heard, in, offset, len);
    // The pilot's own words, named as the speaker they are: the log reads as a
    // three-voice transcript (PILOT / COPILOT / ATC), white because it's the
    // pilot's half of the dialogue.
    AppendLog("PILOT", COL_WHITE, g_heard.buf);
}
static void WriteAction(void*, void* in, int offset, int len) {
    StoreText(&g_action, in, offset, len);
    // A station prefix ("TWR: " / "APP: " / "DEP: " / "ATIS: ") is attribution,
    // not content: strip it and tag the row with the SPECIFIC position, so the
    // transcript names which controller spoke. Everything else is the crew.
    static const struct { const char* prefix; const char* tag; } kStations[] = {
        {"TWR: ", "TOWER"}, {"APP: ", "APP"}, {"DEP: ", "DEP"}, {"ATIS: ", "ATIS"},
        // Party-line traffic: the ghost pilots' side of the frequency. Spelled
        // out on the panel (every other tag is a plain speaker name).
        {"TFC: ", "TRAFFIC"},
    };
    for (const auto& s : kStations) {
        int n = static_cast<int>(std::strlen(s.prefix));
        if (g_action.len > n && std::strncmp(g_action.buf, s.prefix, n) == 0) {
            std::memmove(g_action.buf, g_action.buf + n, static_cast<size_t>(g_action.len - n + 1));
            g_action.len -= n;
            AppendLog(s.tag, COL_AMBER, g_action.buf);
            return;
        }
    }
    AppendLog("COPILOT", COL_GREEN, g_action.buf);
}
// Active aircraft ICAO — shown beside the connection chip, not logged.
static void WriteAircraft(void*, void* in, int offset, int len) {
    StoreText(&g_aircraft, in, offset, len);
}
// Live flight strip — a status row the app refreshes continuously, not logged.
static void WriteStrip(void*, void* in, int offset, int len) {
    StoreText(&g_strip, in, offset, len);
}
// Radio row — refreshed continuously like the strip, not logged.
static void WriteRadio(void*, void* in, int offset, int len) {
    StoreText(&g_radio, in, offset, len);
}

// One in-range station for the ATC tab, parsed from the app's payload.
struct AtcService { std::string label; int khz; };
struct AtcField {
    std::string name;
    bool active;    // the station that answers radio calls ('*' prefix)
    int dist_nm;
    std::vector<AtcService> freqs;
};
static std::vector<AtcField> g_atc_fields;

// Parse "NAME,dist,SVC khz,...|NAME,..." into fields — cheap enough to run on
// every write; the ATC tab renders straight from this.
static std::vector<AtcField> parse_atc() {
    char buf[sizeof g_atc.buf];
    std::memcpy(buf, g_atc.buf, sizeof buf);

    std::vector<AtcField> fields;
    char* save1 = nullptr;
    for (char* tok = strtok_r(buf, "|", &save1); tok; tok = strtok_r(nullptr, "|", &save1)) {
        char* save2 = nullptr;
        char* name = strtok_r(tok, ",", &save2);
        char* dist = strtok_r(nullptr, ",", &save2);
        if (!name || !dist) continue;
        AtcField f;
        f.active = (name[0] == '*');
        f.name = f.active ? name + 1 : name;
        f.dist_nm = std::atoi(dist);
        for (char* svc = strtok_r(nullptr, ",", &save2); svc; svc = strtok_r(nullptr, ",", &save2)) {
            char* sp = std::strchr(svc, ' ');
            if (!sp) continue;
            *sp = '\0';
            f.freqs.push_back({svc, std::atoi(sp + 1)});
        }
        fields.push_back(std::move(f));
    }
    return fields;
}

static void WriteAtc(void*, void* in, int offset, int len) {
    StoreText(&g_atc, in, offset, len);
    g_atc_fields = parse_atc();
}

// Coach line — refreshed continuously like the radio row, not logged.
static void WriteCoach(void*, void* in, int offset, int len) {
    StoreText(&g_coach, in, offset, len);
}
// Callout — stamp the arrival time; the toast shows for a short window after.
// An optional "nx,ny|" prefix (fractions of the half-screen from centre) aims
// the toast toward the control the camera glanced at.
static float g_callout_nx = 0.0f, g_callout_ny = 0.0f;
static void WriteCallout(void*, void* in, int offset, int len) {
    StoreText(&g_callout, in, offset, len);
    g_callout_nx = 0.0f;
    g_callout_ny = 0.0f;
    char* bar = std::strchr(g_callout.buf, '|');
    float nx = 0.0f, ny = 0.0f;
    if (bar && std::sscanf(g_callout.buf, "%f,%f|", &nx, &ny) == 2) {
        g_callout_nx = nx;
        g_callout_ny = ny;
        int rest = g_callout.len - static_cast<int>(bar + 1 - g_callout.buf);
        std::memmove(g_callout.buf, bar + 1, static_cast<size_t>(rest) + 1);
        g_callout.len = rest;
    }
    g_callout_time = XPLMGetElapsedTime();
}
static void WriteAlert(void*, void* in, int offset, int len) {
    StoreText(&g_alert, in, offset, len);
    // First char is a severity marker ('1' advisory, '2' caution, '3' warning).
    // The tag names the SPEAKER — every alert is the copilot's voice, so the
    // transcript reads as who said it; severity survives as the row colour.
    const char* text = g_alert.buf;
    ImU32 col = COL_AMBER;
    if (g_alert.len >= 1) {
        switch (g_alert.buf[0]) {
            case '1': col = COL_CYAN;  text++; break;
            case '2': col = COL_AMBER; text++; break;
            case '3': col = COL_RED;   text++; break;
            default: break;  // no marker (legacy) — amber
        }
    }
    AppendLog("COPILOT", col, text);
}

// ---- FMS route loading ------------------------------------------------------
// The app writes the SimBrief route into xcopilot/fms/route as
// "IDENT,lat,lon,alt|IDENT,lat,lon,alt|...", then writes 1 to xcopilot/fms/load
// to build it in the GPS/FMS (only the plugin can, via XPLMNavigation).

static TextRef g_fms_route = {{0}, 0};
static int g_fms_load = 0;
static XPLMDataRef g_fms_route_dref = nullptr;
static XPLMDataRef g_fms_load_dref = nullptr;

// One parsed route point from the app's "IDENT,lat,lon,alt|..." payload, with the
// navaid it resolved to (if any) so we can pick the right .fms row type.
struct RoutePoint {
    std::string ident;
    float lat, lon;
    int alt;
    XPLMNavRef ref;     // XPLM_NAV_NOT_FOUND for a bare lat/lon point
    XPLMNavType type;   // the navaid's type (only meaningful when ref is set)
};

// Parse the app's payload into points, without navaid resolution — cheap enough
// to run on every route write; the Route tab renders straight from this.
static std::vector<RoutePoint> parse_route() {
    char buf[sizeof g_fms_route.buf];
    std::strncpy(buf, g_fms_route.buf, sizeof buf);
    buf[sizeof buf - 1] = '\0';

    std::vector<RoutePoint> pts;
    char* save1 = nullptr;
    for (char* tok = strtok_r(buf, "|", &save1); tok; tok = strtok_r(nullptr, "|", &save1)) {
        char* save2 = nullptr;
        char* ident = strtok_r(tok, ",", &save2);
        char* slat = strtok_r(nullptr, ",", &save2);
        char* slon = strtok_r(nullptr, ",", &save2);
        char* salt = strtok_r(nullptr, ",", &save2);
        if (!slat || !slon) continue;
        RoutePoint p;
        p.ident = ident ? ident : "";
        p.lat = static_cast<float>(std::atof(slat));
        p.lon = static_cast<float>(std::atof(slon));
        p.alt = salt ? std::atoi(salt) : 0;
        p.ref = XPLM_NAV_NOT_FOUND;
        p.type = 0;
        pts.push_back(p);
    }
    return pts;
}

// Display copy for the panel's Route tab, refreshed whenever the app writes.
static std::vector<RoutePoint> g_route_pts;

static void WriteFmsRoute(void*, void* in, int offset, int len) {
    StoreText(&g_fms_route, in, offset, len);
    g_route_pts = parse_route();
}

// X-Plane's loaded AIRAC cycle (e.g. "2406"), read once from the nav-data header
// so the flight plans we generate carry the *matching* cycle — a mismatch pops
// X-Plane's "different AIRAC cycle" warning on every load. Empty if undetermined.
static std::string g_airac_cycle;

static void read_airac_cycle() {
    char base[1024] = {0};
    XPLMGetSystemPath(base);
    std::string root = base;
    // XPLMGetSystemPath usually ends in a separator, but don't rely on it.
    if (!root.empty() && root.back() != '/' && root.back() != '\\') {
        root += '/';
    }
    // Custom nav data (Navigraph etc.) overrides the default when present, so try
    // it first; the cycle is in the file's header line ("... data cycle 2406 ...").
    const char* rels[] = {"Custom Data/earth_nav.dat", "Resources/default data/earth_nav.dat"};
    for (const char* rel : rels) {
        std::string path = root + rel;
        FILE* f = std::fopen(path.c_str(), "r");
        if (!f) continue; // e.g. no Custom Data installed — fall through to default
        char ln[512];
        for (int i = 0; i < 4 && std::fgets(ln, sizeof ln, f); ++i) {
            const char* m = std::strstr(ln, "data cycle ");
            if (m) {
                int c = std::atoi(m + std::strlen("data cycle "));
                if (c > 0) {
                    char cy[16];
                    std::snprintf(cy, sizeof cy, "%d", c);
                    g_airac_cycle = cy;
                }
                break;
            }
        }
        std::fclose(f);
        if (!g_airac_cycle.empty()) return;
    }
}

// Map an X-Plane navaid type to the X-Plane .fms row code.
static int fms_row_type(const RoutePoint& p) {
    if (p.ref == XPLM_NAV_NOT_FOUND) return 28; // lat/lon
    if (p.type & xplm_Nav_Airport) return 1;
    if (p.type & xplm_Nav_NDB) return 2;
    if (p.type & xplm_Nav_VOR) return 3;
    return 11; // fix / intersection / DME
}

static void load_fms_route() {
    const XPLMNavType kTypes = xplm_Nav_Fix | xplm_Nav_VOR | xplm_Nav_NDB |
                               xplm_Nav_DME | xplm_Nav_Airport;

    // Parse the payload into points, resolving each to a navaid near its position.
    std::vector<RoutePoint> pts = parse_route();
    for (RoutePoint& p : pts) {
        if (p.ident.empty()) continue;
        float la = p.lat, lo = p.lon;
        p.ref = XPLMFindNavAid(nullptr, p.ident.c_str(), &la, &lo, nullptr, kTypes);
        if (p.ref != XPLM_NAV_NOT_FOUND) {
            XPLMGetNavAidInfo(p.ref, &p.type, nullptr, nullptr, nullptr, nullptr, nullptr,
                              nullptr, nullptr, nullptr);
        }
    }

    // The CDU's ORIGIN/DEST airport fields are only set by *loading* a flight plan
    // (X-Plane parses the ADEP/ADES lines) — the per-entry API just adds waypoints
    // and leaves ORIGIN/DEST blank (confirmed: entry 0 was LEBL type-1 yet the page
    // stayed empty). So build an X-Plane 11 .fms with explicit ADEP/ADES and load
    // it; that drives both the GA GPS and the native FMS on device 0.
    if (pts.size() >= 2) {
        const RoutePoint& dep = pts.front();
        const RoutePoint& des = pts.back();
        std::string fms = "I\n1100 Version\n";
        // Stamp X-Plane's own AIRAC cycle so the loader doesn't warn about a cycle
        // mismatch on every load. Omit the line if we couldn't read it.
        if (!g_airac_cycle.empty()) {
            fms += "CYCLE " + g_airac_cycle + "\n";
        }
        fms += "ADEP " + dep.ident + "\n";
        fms += "ADES " + des.ident + "\n";
        char line[256];
        std::snprintf(line, sizeof line, "NUMENR %d\n", static_cast<int>(pts.size()));
        fms += line;
        for (size_t i = 0; i < pts.size(); ++i) {
            const RoutePoint& p = pts[i];
            const char* via = (i == 0) ? "ADEP" : (i + 1 == pts.size()) ? "ADES" : "DRCT";
            int rowtype = (i == 0 || i + 1 == pts.size()) ? 1 : fms_row_type(p);
            std::snprintf(line, sizeof line, "%d %s %s %d %.6f %.6f\n", rowtype,
                          p.ident.c_str(), via, p.alt, p.lat, p.lon);
            fms += line;
        }
        XPLMLoadFMSFlightPlan(0, fms.c_str(), static_cast<unsigned int>(fms.size()));
    }

    // Active leg = the first fix *after* the origin (entry 1) so we fly forward up
    // the route, not back to the departure airport at entry 0. Set it on both the
    // legacy GPS plan and the native FMS plan.
    int idx = static_cast<int>(pts.size());
    if (idx > 1) {
        XPLMSetDestinationFMSEntry(1);
        XPLMSetDestinationFMSFlightPlanEntry(xplm_Fpl_Pilot_Primary, 1);
    } else if (idx == 1) {
        XPLMSetDestinationFMSEntry(0);
        XPLMSetDestinationFMSFlightPlanEntry(xplm_Fpl_Pilot_Primary, 0);
    }
    char msg[96];
    std::snprintf(msg, sizeof msg, "x-copilot plugin: loaded %d FMS waypoints\n", idx);
    XPLMDebugString(msg);
}

static int ReadFmsLoad(void*) { return g_fms_load; }
static void WriteFmsLoad(void*, int v) {
    if (v != 0) {
        load_fms_route();
    }
    g_fms_load = 0; // edge-triggered; reset so a re-load needs a fresh write
}

// ---- panel ------------------------------------------------------------------

// While the app is connected: 0/1 = resting ("CONNECTED" — with push-to-talk the
// mic is shut until you press, and the green [PTT] chip shows actual capture),
// 2 = working on a command, 3 = reading back. A stale heartbeat (handled in
// DrawPanel) shows NOT CONNECTED instead.
static const char* PhaseLabel(int p) {
    switch (p) {
        case 1: return "LISTENING";
        case 2: return "THINKING";
        case 3: return "SPEAKING";
        default: return "CONNECTED";
    }
}
static ImU32 PhaseColor(int p) {
    switch (p) {
        case 1: return COL_WHITE; // mic open — the brightest state on the strip
        case 2: return COL_AMBER;
        case 3: return COL_CYAN;
        default: return COL_GREEN;
    }
}

// A small outlined "chip" (like the mockup's RAMP tags), drawn in `col` at the
// current cursor; advances the cursor by its size so SameLine() text follows.
static void Chip(const char* label, ImU32 col) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 p = ImGui::GetCursorScreenPos();
    ImVec2 ts = ImGui::CalcTextSize(label);
    const float padx = 6.0f, pady = 2.0f;
    ImVec2 size(ts.x + padx * 2, ts.y + pady * 2);
    dl->AddRect(p, ImVec2(p.x + size.x, p.y + size.y), col, 3.0f, 0, 1.2f);
    dl->AddText(ImVec2(p.x + padx, p.y + pady), col, label);
    ImGui::Dummy(size);
}

// Log speaker tag: a fixed-width filled pill so every row's text starts at the
// same column — the log reads as a transcript, not a ragged debug feed. Fill
// is the tag's own hue at low alpha; the label carries the color.
static void TagChip(const char* label, ImU32 col, float width) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 p = ImGui::GetCursorScreenPos();
    ImVec2 ts = ImGui::CalcTextSize(label);
    const float pady = 2.0f;
    ImVec2 size(width, ts.y + pady * 2);
    ImU32 fill = (col & 0x00FFFFFFu) | 0x2E000000u;
    dl->AddRectFilled(p, ImVec2(p.x + size.x, p.y + size.y), fill, 3.0f);
    dl->AddText(ImVec2(p.x + (width - ts.x) * 0.5f, p.y + pady), col, label);
    ImGui::Dummy(size);
}

class XCopilotPanel : public ImgWindow {
public:
    XCopilotPanel(int l, int t, int r, int b)
        : ImgWindow(l, t, r, b, xplm_WindowDecorationRoundRectangle,
                    xplm_WindowLayerFloatingWindows) {
        // Keep the title clean; version + build id go to Log.txt on startup instead.
        SetWindowTitle("X-COPILOT");
        ImGuiIO& io = ImGui::GetIO();
        io.IniFilename = nullptr;
        // One visual system: dark slate instrument, grey controls, color used
        // ONLY as signal (green ok, amber ATC/caution, cyan info, red danger).
        // Everything blue that ImGui ships by default is restyled — a cockpit
        // panel shouldn't look like a debug tool.
        ImGuiStyle& s = ImGui::GetStyle();
        s.WindowRounding = 8.0f;
        s.ChildRounding = 6.0f;
        s.FrameRounding = 4.0f;
        s.TabRounding = 4.0f;
        s.GrabRounding = 4.0f;
        s.ItemSpacing = ImVec2(8, 6);
        s.FramePadding = ImVec2(8, 3);
        const ImVec4 slate0(0.09f, 0.10f, 0.12f, 0.96f);  // window
        const ImVec4 slate1(0.12f, 0.13f, 0.15f, 1.00f);  // wells
        const ImVec4 slate2(0.17f, 0.19f, 0.22f, 1.00f);  // controls
        const ImVec4 slate3(0.23f, 0.25f, 0.29f, 1.00f);  // hover
        const ImVec4 slate4(0.28f, 0.31f, 0.36f, 1.00f);  // active
        s.Colors[ImGuiCol_WindowBg]         = slate0;
        s.Colors[ImGuiCol_ChildBg]          = slate1;
        s.Colors[ImGuiCol_Border]           = ImVec4(0.26f, 0.28f, 0.32f, 0.55f);
        s.Colors[ImGuiCol_FrameBg]          = slate2;
        s.Colors[ImGuiCol_FrameBgHovered]   = slate3;
        s.Colors[ImGuiCol_FrameBgActive]    = slate4;
        s.Colors[ImGuiCol_Button]           = slate2;
        s.Colors[ImGuiCol_ButtonHovered]    = slate3;
        s.Colors[ImGuiCol_ButtonActive]     = slate4;
        s.Colors[ImGuiCol_Tab]              = slate1;
        s.Colors[ImGuiCol_TabHovered]       = slate3;
        s.Colors[ImGuiCol_TabActive]        = slate2;
        s.Colors[ImGuiCol_TabUnfocused]     = slate1;
        s.Colors[ImGuiCol_TabUnfocusedActive] = slate2;
        s.Colors[ImGuiCol_CheckMark]        = ImVec4(0.47f, 0.88f, 0.55f, 1.0f);
        s.Colors[ImGuiCol_ScrollbarBg]      = slate1;
        s.Colors[ImGuiCol_ScrollbarGrab]    = slate2;
        s.Colors[ImGuiCol_ScrollbarGrabHovered] = slate3;
        s.Colors[ImGuiCol_ScrollbarGrabActive]  = slate4;
        s.Colors[ImGuiCol_TitleBg]          = slate0;
        s.Colors[ImGuiCol_TitleBgActive]    = slate0;
        s.Colors[ImGuiCol_Separator]        = ImVec4(0.26f, 0.28f, 0.32f, 0.55f);
        s.Colors[ImGuiCol_HeaderHovered]    = slate3;
        s.Colors[ImGuiCol_HeaderActive]     = slate4;
        s.Colors[ImGuiCol_TableRowBg]       = slate1;
        s.Colors[ImGuiCol_TableRowBgAlt]    = ImVec4(0.14f, 0.15f, 0.18f, 1.0f);
        // Allow shrinking down to the toolbar row (collapsed view).
        SetWindowResizingLimits(260, kCollapsedH, 1400, 1400);
    }

private:
    // Collapse to just the toolbar row (status chips under the title bar), so the
    // panel can tuck out of the way without closing. Remembers the expanded size.
    static constexpr int kCollapsedH = 46;
    bool mCollapsed = false;
    // Selected tab (0 = Log, 1 = Route, 2 = ATC, 3 = Calls, 4 = Settings) — content renders after
    // the shared context rows, so the tab bar itself sits at the very top.
    int mTab = 0;
    int mExpW = 420, mExpH = 300;

    void toggleCollapse() {
        int l, t, r, b;
        GetWindowGeometry(l, t, r, b);
        if (!mCollapsed) {
            mExpW = r - l;
            mExpH = t - b;
            SetWindowGeometry(l, t, r, t - kCollapsedH); // keep position, shrink height
            mCollapsed = true;
        } else {
            SetWindowGeometry(l, t, l + mExpW, t - mExpH); // restore size at current spot
            mCollapsed = false;
        }
    }

public:
    // Launch state: a slim status strip, not a window over the scenery. The
    // panel earns its full size when the pilot asks for it ("+"), instead of
    // greeting every flight as a splash screen.
    void startCollapsed() {
        if (!mCollapsed) toggleCollapse();
    }

protected:
    // The status strip: crew state, aircraft, and the two talk lights, with
    // the collapse/expand control right-aligned. Lives at the BOTTOM of the
    // expanded panel (instrument-style: controls above, annunciators below),
    // and is the only row shown when collapsed.
    void drawStatusBar(bool connected) {
        if (connected) {
            Chip(PhaseLabel(g_phase), PhaseColor(g_phase));
            if (g_aircraft.len > 0) {
                ImGui::SameLine();
                Chip(g_aircraft.buf, COL_CYAN);
            }
        } else {
            Chip("NOT CONNECTED", COL_RED);
        }
        ImGui::SameLine();
        Chip("PTT", g_ptt_active ? COL_GREEN : COL_DIM);
        ImGui::SameLine();
        // TX = the radio (ATC) push-to-talk, distinct from the intercom PTT.
        Chip("TX", g_radio_ptt_active ? COL_AMBER : COL_DIM);
        const char* collapseLbl = mCollapsed ? "+" : "-";
        float collapseW = ImGui::CalcTextSize(collapseLbl).x
            + ImGui::GetStyle().FramePadding.x * 2.0f;
        ImGui::SameLine(ImGui::GetContentRegionMax().x - collapseW);
        if (ImGui::SmallButton(collapseLbl)) {
            toggleCollapse();
        }
    }

    void buildInterface() override {
        // The app heartbeats the phase ~1/s; a stale timestamp = not running.
        bool connected = (XPLMGetElapsedTime() - g_last_status_time) < 3.0f;
        // Collapsed: just the status strip under the title bar.
        if (mCollapsed) {
            drawStatusBar(connected);
            return;
        }

        // Tabs at the very top: what you're looking AT comes first; state
        // annunciators live in the bottom status bar. Content renders after
        // the shared context rows, so those stay visible on every tab.
        if (ImGui::BeginTabBar("xcp_tabs")) {
            // Unread dot on the Log tab while entries arrive unseen — cleared
            // the moment the tab is shown, so it only lights up from others.
            ImGuiTabItemFlags logFlags =
                g_log_unseen ? ImGuiTabItemFlags_UnsavedDocument : 0;
            if (ImGui::BeginTabItem("Log", nullptr, logFlags)) {
                g_log_unseen = false;
                mTab = 0;
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Route")) {
                mTab = 1;
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("ATC")) {
                mTab = 2;
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Calls")) {
                mTab = 3;
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Settings")) {
                mTab = 4;
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }

        // Live flight strip — the app refreshes this with what the copilot sees
        // (emergency phase/runway, cabin altitude, endurance). Hidden when empty.
        if (connected && g_strip.len > 0) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(COL_CYAN));
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextUnformatted(g_strip.buf);
            ImGui::PopTextWrapPos();
            ImGui::PopStyleColor();
        }

        // Radio row — the field's comm plan as chips, the tuned one lit amber
        // ('>' prefix from the app). An instrument row, not a log line: it
        // reads like the radio stack, at a glance. Hidden when empty.
        if (connected && g_radio.len > 0) {
            char row[512];
            std::memcpy(row, g_radio.buf, sizeof row);
            char* seg = row;
            bool first = true;
            while (seg != nullptr) {
                char* bar = std::strstr(seg, " | ");
                if (bar != nullptr) *bar = '\0';
                bool lit = (seg[0] == '>');
                if (lit) seg++;
                if (!first) ImGui::SameLine();
                Chip(seg, lit ? COL_AMBER : COL_DIM);
                first = false;
                seg = (bar != nullptr) ? bar + 3 : nullptr;
            }
        }

        // Emergency buttons — the mouse fallback when voice fails mid-emergency.
        // Guarded-switch styling: the two that take the aircraft are filled dark
        // red, the one that gives it back dark green. Each writes a request code
        // the app polls and runs as the matching voice command.
        if (connected) {
            const ImVec4 redBg(0.42f, 0.13f, 0.11f, 1.0f), redHov(0.55f, 0.17f, 0.14f, 1.0f);
            const ImVec4 greenBg(0.11f, 0.32f, 0.16f, 1.0f), greenHov(0.14f, 0.42f, 0.21f, 1.0f);
            ImGui::PushStyleColor(ImGuiCol_Button, redBg);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, redHov);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, redHov);
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(COL_WHITE));
            if (ImGui::SmallButton("RECOVER")) g_request = 1;
            ImGui::SameLine();
            if (ImGui::SmallButton("AUTOLAND")) g_request = 2;
            ImGui::PopStyleColor(4);
            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_Button, greenBg);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, greenHov);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, greenHov);
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(COL_WHITE));
            if (ImGui::SmallButton("MY CONTROLS")) g_request = 3;
            ImGui::PopStyleColor(4);
            if (g_request != 0) {
                ImGui::SameLine();
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "sent...");
            }
            ImGui::Separator();
        }

        // Body: the selected tab's content. Log = the transmission record;
        // Route = what the app pushed for the FMS; Settings = the runtime
        // switches.
        switch (mTab) {
            case 0: buildLogTab(connected); break;
            case 1: buildRouteTab(); break;
            case 2: buildAtcTab(connected); break;
            case 3: buildCallsTab(connected); break;
            default: buildSettingsTab(); break;
        }

        // Bottom status bar — annunciators live under the content, like a
        // panel's lamp row.
        ImGui::Separator();
        drawStatusBar(connected);
    }

    // Transmission log — recent heard/action lines with coloured tags, each
    // stamped so a test can be reviewed afterwards.
    void buildLogTab(bool connected) {
        // Two reserved rows below the well: the log controls and the bottom
        // status bar (drawn by buildInterface after the tab content).
        const float footer = ImGui::GetFrameHeightWithSpacing() * 2.0f
            + ImGui::GetStyle().ItemSpacing.y;
        ImGui::BeginChild("log", ImVec2(0, -footer), true);
        // Columns measured from the actual font (time | speaker | text) so
        // nothing ever clips and every message starts at the same x — the log
        // reads as a transcript, not a ragged debug feed.
        const float pad = 8.0f;
        const float kTimeW = ImGui::CalcTextSize("88:88:88").x + pad;
        const float kTagW = ImGui::CalcTextSize("COPILOT").x + 14.0f; // widest tag
        if (g_log.empty()) {
            // Empty state, centred in the well — not a footer mumble.
            const char* hint = connected ? "Hold PTT and speak." : "Start x-copilot to connect.";
            ImVec2 ts = ImGui::CalcTextSize(hint);
            ImGui::SetCursorPos(ImVec2((ImGui::GetWindowWidth() - ts.x) * 0.5f,
                                       (ImGui::GetWindowHeight() - ts.y) * 0.5f));
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "%s", hint);
        }
        for (const LogEntry& e : g_log) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "%s", e.time.c_str());
            ImGui::SameLine(kTimeW + pad);
            TagChip(e.tag.c_str(), e.color, kTagW);
            ImGui::SameLine(kTimeW + kTagW + pad * 2);
            // Wrap long readbacks at the panel edge instead of running off it.
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(e.color));
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextUnformatted(e.text.c_str());
            ImGui::PopTextWrapPos();
            ImGui::PopStyleColor();
        }
        if (g_log_scroll) { ImGui::SetScrollHereY(1.0f); g_log_scroll = false; }
        ImGui::EndChild();

        // Footer: just the log controls — state lives in the chips, version in
        // Settings. An empty left cell keeps the controls on this row.
        ImGui::Dummy(ImVec2(1, 1));
        // Right-aligned: auto-scroll toggle + clear.
        const char* clearLbl = "Clear";
        float ctlW = ImGui::CalcTextSize("Clear").x + ImGui::CalcTextSize("follow").x
            + ImGui::GetFrameHeight() + ImGui::GetStyle().FramePadding.x * 6.0f
            + ImGui::GetStyle().ItemSpacing.x * 2.0f;
        ImGui::SameLine(ImGui::GetContentRegionMax().x - ctlW);
        ImGui::Checkbox("follow", &g_log_autoscroll);
        ImGui::SameLine();
        if (ImGui::SmallButton(clearLbl)) {
            g_log.clear();
        }
    }

    // The route the app pushed (SimBrief import), visible before/after it lands
    // in the FMS — plus a manual re-push for after an aircraft or plan reload.
    void buildRouteTab() {
        if (g_route_pts.empty()) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                               "No route loaded.");
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                               "Say \"load the flight plan\" to import from SimBrief.");
            return;
        }
        const RoutePoint& dep = g_route_pts.front();
        const RoutePoint& des = g_route_pts.back();
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_WHITE),
                           "%s  >  %s   (%d waypoints)", dep.ident.c_str(),
                           des.ident.c_str(), static_cast<int>(g_route_pts.size()));
        ImGui::SameLine();
        // Re-push the plan into the GPS/FMS, e.g. after switching aircraft.
        if (ImGui::SmallButton("Send to FMS")) {
            load_fms_route();
            AppendLog("COPILOT", COL_GREEN, "Flight plan sent to the FMS.");
        }
        if (ImGui::BeginTable("route", 4,
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY |
                              ImGuiTableFlags_BordersInnerV)) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed, 28.0f);
            ImGui::TableSetupColumn("IDENT");
            ImGui::TableSetupColumn("ALT", ImGuiTableColumnFlags_WidthFixed, 60.0f);
            ImGui::TableSetupColumn("POSITION");
            ImGui::TableHeadersRow();
            for (size_t i = 0; i < g_route_pts.size(); ++i) {
                const RoutePoint& p = g_route_pts[i];
                bool endpoint = (i == 0 || i + 1 == g_route_pts.size());
                ImU32 col = endpoint ? COL_CYAN : COL_WHITE;
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "%d",
                                   static_cast<int>(i + 1));
                ImGui::TableNextColumn();
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(col), "%s",
                                   p.ident.empty() ? "(pos)" : p.ident.c_str());
                ImGui::TableNextColumn();
                if (p.alt > 0) {
                    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(col), "%d", p.alt);
                }
                ImGui::TableNextColumn();
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                                   "%.4f, %.4f", p.lat, p.lon);
            }
            ImGui::EndTable();
        }
    }

    // Every station in VHF range, nearest first, pushed by the app. The tab is
    // an instrument, not a listing: each frequency is a button — click to tune
    // COM1 — and the tuned one is lit amber. The active station (the one that
    // answers radio calls) is named in green.
    void buildAtcTab(bool connected) {
        if (!connected) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                               "Waiting for the app.");
            return;
        }
        if (g_atc_fields.empty()) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                               "No stations in VHF range.");
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                               "Range grows with altitude.");
            return;
        }
        int com1 = g_com1_dref ? XPLMGetDatai(g_com1_dref) : 0;
        // Two reserved rows below the well: the tune hint and the bottom
        // status bar (drawn by buildInterface after the tab content).
        const float footer = ImGui::GetFrameHeightWithSpacing() * 2.0f
            + ImGui::GetStyle().ItemSpacing.y;
        if (ImGui::BeginChild("atc_well", ImVec2(0, -footer))) {
            for (size_t i = 0; i < g_atc_fields.size(); ++i) {
                const AtcField& f = g_atc_fields[i];
                ImGui::PushID(static_cast<int>(i));
                ImU32 nameCol = f.active ? COL_GREEN : COL_WHITE;
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(nameCol), "%s",
                                   f.name.c_str());
                ImGui::SameLine();
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "%d nm%s",
                                   f.dist_nm, f.active ? "  (active)" : "");
                // Frequency chips flow left-to-right and WRAP at the panel
                // edge — Heathrow publishes six services, and clipped
                // buttons can't be clicked.
                const float wrap_x =
                    ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x;
                for (size_t j = 0; j < f.freqs.size(); ++j) {
                    const AtcService& s = f.freqs[j];
                    bool tuned = std::abs(com1 - s.khz) <= 5;
                    char label[32];
                    std::snprintf(label, sizeof label, "%s %.3f", s.label.c_str(),
                                  s.khz / 1000.0);
                    if (j > 0) {
                        const ImGuiStyle& st = ImGui::GetStyle();
                        float next_w =
                            ImGui::CalcTextSize(label).x + st.FramePadding.x * 2.0f;
                        if (ImGui::GetItemRectMax().x + st.ItemSpacing.x + next_w
                            < wrap_x) {
                            ImGui::SameLine();
                        }
                    }
                    ImGui::PushID(static_cast<int>(j));
                    ImU32 col = tuned ? COL_AMBER : COL_DIM;
                    ImGui::PushStyleColor(ImGuiCol_Text,
                                          ImGui::ColorConvertU32ToFloat4(col));
                    if (ImGui::SmallButton(label) && g_com1_dref) {
                        XPLMSetDatai(g_com1_dref, s.khz);
                    }
                    ImGui::PopStyleColor();
                    ImGui::PopID();
                }
                if (i + 1 < g_atc_fields.size()) ImGui::Separator();
                ImGui::PopID();
            }
        }
        ImGui::EndChild();
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                           "Click a frequency to tune COM1.");
    }

    // The radio coach: what to SAY NOW (live, from the tower's state machine,
    // pushed by the app) over a phraseology reference — the learning tool.
    // Ask the copilot "what do I say?" for the same line by voice.
    void buildCallsTab(bool connected) {
        if (connected && g_coach.len > 0) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "SAY NOW");
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(COL_CYAN));
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextUnformatted(g_coach.buf);
            ImGui::PopTextWrapPos();
            ImGui::PopStyleColor();
            ImGui::Separator();
        }
        // One reserved row below the well: the bottom status bar.
        const float footer = ImGui::GetFrameHeightWithSpacing()
            + ImGui::GetStyle().ItemSpacing.y;
        if (ImGui::BeginChild("calls_well", ImVec2(0, -footer))) {
            struct Phrase { const char* say; const char* gets; };
            struct Group { const char* title; const Phrase* items; int n; };
            static const Phrase dep[] = {
                {"<Station> Tower, <callsign>, with information <letter>, request departure clearance", "zone clearance + squawk"},
                {"Request taxi", "taxi route via named taxiways"},
                {"Holding point <name>, ready for departure", "takeoff clearance, or line up and wait"},
                {"Lining up runway <number>, <callsign>", "readback of line up and wait"},
                {"Cleared for takeoff runway <number>, <callsign>", "readback: runway clearances must be read back"},
            };
            static const Phrase arr[] = {
                {"<Station> Tower, <callsign>, 10 miles out, inbound with information <letter>", "join instruction"},
                {"Request touch and go", "circuit join, option to land"},
                {"Crosswind / Downwind / Turning base / Final", "sequenced around the circuit"},
                {"Cleared to land runway <number>, <callsign>", "readback of the landing clearance"},
                {"Going around", "climb-out and rejoin"},
                {"Runway vacated, request taxi to parking", "taxi in"},
            };
            static const Phrase misc[] = {
                {"Radio check", "reading you five"},
                {"Say wind / Say QNH", "current numbers"},
                {"What's my squawk", "your assigned code"},
                {"Request flight following", "radar advisories"},
                {"Request vectors", "headings to final"},
                {"Request frequency change", "change approved"},
            };
            static const Phrase emer[] = {
                {"Mayday, mayday, mayday, <callsign>, <problem>", "priority + cleared any runway"},
                {"Pan pan, pan pan, pan pan, <callsign>, <problem>", "urgency handling"},
            };
            static const Group groups[] = {
                {"DEPARTURE", dep, 5}, {"ARRIVAL AND CIRCUIT", arr, 6},
                {"ANY TIME", misc, 6}, {"EMERGENCY", emer, 2},
            };
            for (const Group& g : groups) {
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "%s", g.title);
                for (int i = 0; i < g.n; ++i) {
                    ImGui::Bullet();
                    ImGui::SameLine();
                    ImGui::PushTextWrapPos(0.0f);
                    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_WHITE), "%s", g.items[i].say);
                    ImGui::PopTextWrapPos();
                    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(COL_DIM));
                    ImGui::Indent(18.0f);
                    ImGui::PushTextWrapPos(0.0f);
                    ImGui::TextUnformatted(g.items[i].gets);
                    ImGui::PopTextWrapPos();
                    ImGui::Unindent(18.0f);
                    ImGui::PopStyleColor();
                }
                ImGui::Spacing();
            }
        }
        ImGui::EndChild();
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                           "Or ask the copilot: what do I say?");
    }

    // Live runtime switches. Each checkbox drives a dataref the app polls live;
    // changes apply on the next utterance/tick, no restart.
    void buildSettingsTab() {
        bool llm_on = (g_llm_enabled == 1);
        if (ImGui::Checkbox("LLM fallback (answer questions)", &llm_on)) {
            g_llm_enabled = llm_on ? 1 : 0;
        }
        bool snap = g_opt_camera_snap != 0;
        if (ImGui::Checkbox("Camera snap-to-panel on actions", &snap)) {
            g_opt_camera_snap = snap ? 1 : 0;
        }
        bool tone = g_opt_negative_tone != 0;
        if (ImGui::Checkbox("Negative tone on unrecognized", &tone)) {
            g_opt_negative_tone = tone ? 1 : 0;
        }
        bool mute = g_opt_monitor_mute != 0;
        if (ImGui::Checkbox("Mute monitor callouts", &mute)) {
            g_opt_monitor_mute = mute ? 1 : 0;
        }
        ImGui::Spacing();
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                           "Changes apply live; no restart needed.");
        ImGui::Separator();
        // Dev corner: the plugin reload (bounces ALL plugins — the SDK can't
        // reload just one) and the build identity, out of the pilot's way.
        if (ImGui::SmallButton("Reload plugins")) {
            XPLMReloadPlugins();
        }
        ImGui::SameLine();
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                           "after a rebuild · v" XCOPILOT_VERSION " · " XCOPILOT_BUILD);
    }
};

// The floating callout toast is gone: a notification splashed over the scenery
// after every action proved to be noise — the voice readback confirms the
// action, and the panel log keeps the written record. The callout dataref
// stays registered below so the app's writes keep succeeding; the plugin just
// no longer draws them.

static std::unique_ptr<XCopilotPanel> g_panel;
static XPLMMenuID g_menu = nullptr;

static void MenuHandler(void*, void*) {
    if (g_panel) g_panel->SetVisible(!g_panel->GetVisible());
}

// Bindable panel toggle (xcopilot/toggle_panel) — a joystick button or key
// beats reaching for the plugins menu mid-flight.
static XPLMCommandRef g_toggle_cmd = nullptr;
static int TogglePanelHandler(XPLMCommandRef, XPLMCommandPhase phase, void*) {
    if (phase == xplm_CommandBegin && g_panel) {
        g_panel->SetVisible(!g_panel->GetVisible());
    }
    return 0;
}

// ---- lifecycle --------------------------------------------------------------

PLUGIN_API int XPluginStart(char* name, char* sig, char* desc) {
    std::strcpy(name, "X-COPILOT");
    std::strcpy(sig, "com.dlicudi.xcopilot");
    std::strcpy(desc, "X-COPILOT " XCOPILOT_VERSION ", push-to-talk and status panel for the voice copilot.");

    // Get POSIX paths from the SDK, not legacy Mac HFS ("Macintosh HD:Users:...");
    // otherwise fopen can't open the nav-data file we read the AIRAC cycle from.
    XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);

    g_ptt_cmd = XPLMCreateCommand("xcopilot/ptt", "x-copilot: hold to talk");
    XPLMRegisterCommandHandler(g_ptt_cmd, PttCommandHandler, 1, nullptr);

    g_ptt_dref = XPLMRegisterDataAccessor(
        "xcopilot/ptt_active", xplmType_Int, 0,
        ReadPttActive, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);

    g_radio_ptt_cmd =
        XPLMCreateCommand("xcopilot/radio_ptt", "x-copilot: hold to transmit (radio/ATC)");
    XPLMRegisterCommandHandler(g_radio_ptt_cmd, RadioPttCommandHandler, 1, nullptr);

    g_radio_ptt_dref = XPLMRegisterDataAccessor(
        "xcopilot/radio_ptt_active", xplmType_Int, 0,
        ReadRadioPttActive, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_phase_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/phase", xplmType_Int, 1,
        ReadPhase, WritePhase, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_heard_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/heard", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteHeard, &g_heard, &g_heard);
    g_action_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/action", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteAction, &g_action, &g_action);
    g_alert_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/alert", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteAlert, &g_alert, &g_alert);
    g_aircraft_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/aircraft", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteAircraft, &g_aircraft, &g_aircraft);
    g_fms_route_dref = XPLMRegisterDataAccessor(
        "xcopilot/fms/route", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteFmsRoute, &g_fms_route, &g_fms_route);
    g_fms_load_dref = XPLMRegisterDataAccessor(
        "xcopilot/fms/load", xplmType_Int, 1,
        ReadFmsLoad, WriteFmsLoad, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_llm_dref = XPLMRegisterDataAccessor(
        "xcopilot/llm_enabled", xplmType_Int, 1,
        ReadLlmEnabled, WriteLlmEnabled, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_request_dref = XPLMRegisterDataAccessor(
        "xcopilot/request", xplmType_Int, 1,
        ReadRequest, WriteRequest, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_strip_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/strip", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteStrip, &g_strip, &g_strip);
    g_opt_snap_dref = XPLMRegisterDataAccessor(
        "xcopilot/opt/camera_snap", xplmType_Int, 1,
        ReadOptSnap, WriteOptSnap, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_opt_tone_dref = XPLMRegisterDataAccessor(
        "xcopilot/opt/negative_tone", xplmType_Int, 1,
        ReadOptTone, WriteOptTone, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_opt_mute_dref = XPLMRegisterDataAccessor(
        "xcopilot/opt/monitor_mute", xplmType_Int, 1,
        ReadOptMute, WriteOptMute, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_callout_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/callout", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteCallout, &g_callout, &g_callout);
    g_radio_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/radio", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteRadio, &g_radio, &g_radio);
    g_atc_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/atc", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteAtc, &g_atc, &g_atc);
    g_coach_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/coach", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteCoach, &g_coach, &g_coach);
    g_com1_dref = XPLMFindDataRef("sim/cockpit2/radios/actuators/com1_frequency_hz_833");

    // One shared font atlas for all ImgWindows (uses ImGui's default font).
    ImgWindow::sFontAtlas = std::make_shared<ImgFontAtlas>();

    int sl, st, sr, sb;
    XPLMGetScreenBoundsGlobal(&sl, &st, &sr, &sb);
    // Start well below the top edge so the panel clears the X-Plane menu bar.
    int wl = sl + 50, wt = st - 150, wr = wl + 420, wb = wt - 300;
    g_panel = std::make_unique<XCopilotPanel>(wl, wt, wr, wb);
    // Launch as a slim status strip (chips only) — expand is one click away.
    g_panel->startCollapsed();
    g_toggle_cmd = XPLMCreateCommand("xcopilot/toggle_panel", "x-copilot: toggle the status panel");
    XPLMRegisterCommandHandler(g_toggle_cmd, TogglePanelHandler, 1, nullptr);

    int idx = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "X-COPILOT", nullptr, 0);
    g_menu = XPLMCreateMenu("X-COPILOT", XPLMFindPluginsMenu(), idx, MenuHandler, nullptr);
    XPLMAppendMenuItem(g_menu, "Toggle status panel", (void*)"toggle", 0);

    read_airac_cycle(); // for stamping generated flight plans with the matching cycle

    char started[160];
    std::snprintf(started, sizeof started,
                  "x-copilot plugin: started, build %s, AIRAC %s (ptt + ImGui status panel)\n",
                  XCOPILOT_BUILD, g_airac_cycle.empty() ? "unknown" : g_airac_cycle.c_str());
    XPLMDebugString(started);
    return 1;
}

PLUGIN_API void XPluginStop(void) {
    if (g_toggle_cmd) { XPLMUnregisterCommandHandler(g_toggle_cmd, TogglePanelHandler, 1, nullptr); g_toggle_cmd = nullptr; }
    if (g_coach_dref) { XPLMUnregisterDataAccessor(g_coach_dref); g_coach_dref = nullptr; }
    if (g_atc_dref) { XPLMUnregisterDataAccessor(g_atc_dref); g_atc_dref = nullptr; }
    if (g_radio_dref) { XPLMUnregisterDataAccessor(g_radio_dref); g_radio_dref = nullptr; }
    if (g_callout_dref) { XPLMUnregisterDataAccessor(g_callout_dref); g_callout_dref = nullptr; }
    g_panel.reset();
    ImgWindow::sFontAtlas.reset();
    if (g_menu) { XPLMDestroyMenu(g_menu); g_menu = nullptr; }
    if (g_opt_mute_dref) { XPLMUnregisterDataAccessor(g_opt_mute_dref); g_opt_mute_dref = nullptr; }
    if (g_opt_tone_dref) { XPLMUnregisterDataAccessor(g_opt_tone_dref); g_opt_tone_dref = nullptr; }
    if (g_opt_snap_dref) { XPLMUnregisterDataAccessor(g_opt_snap_dref); g_opt_snap_dref = nullptr; }
    if (g_strip_dref) { XPLMUnregisterDataAccessor(g_strip_dref); g_strip_dref = nullptr; }
    if (g_request_dref) { XPLMUnregisterDataAccessor(g_request_dref); g_request_dref = nullptr; }
    if (g_llm_dref) { XPLMUnregisterDataAccessor(g_llm_dref); g_llm_dref = nullptr; }
    if (g_fms_load_dref) { XPLMUnregisterDataAccessor(g_fms_load_dref); g_fms_load_dref = nullptr; }
    if (g_fms_route_dref) { XPLMUnregisterDataAccessor(g_fms_route_dref); g_fms_route_dref = nullptr; }
    if (g_aircraft_dref) { XPLMUnregisterDataAccessor(g_aircraft_dref); g_aircraft_dref = nullptr; }
    if (g_alert_dref) { XPLMUnregisterDataAccessor(g_alert_dref); g_alert_dref = nullptr; }
    if (g_action_dref) { XPLMUnregisterDataAccessor(g_action_dref); g_action_dref = nullptr; }
    if (g_heard_dref) { XPLMUnregisterDataAccessor(g_heard_dref); g_heard_dref = nullptr; }
    if (g_phase_dref) { XPLMUnregisterDataAccessor(g_phase_dref); g_phase_dref = nullptr; }
    if (g_ptt_dref) { XPLMUnregisterDataAccessor(g_ptt_dref); g_ptt_dref = nullptr; }
    if (g_ptt_cmd) { XPLMUnregisterCommandHandler(g_ptt_cmd, PttCommandHandler, 1, nullptr); g_ptt_cmd = nullptr; }
    if (g_radio_ptt_dref) { XPLMUnregisterDataAccessor(g_radio_ptt_dref); g_radio_ptt_dref = nullptr; }
    if (g_radio_ptt_cmd) { XPLMUnregisterCommandHandler(g_radio_ptt_cmd, RadioPttCommandHandler, 1, nullptr); g_radio_ptt_cmd = nullptr; }
}

PLUGIN_API int XPluginEnable(void) { return 1; }
PLUGIN_API void XPluginDisable(void) {}
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID, int, void*) {}
