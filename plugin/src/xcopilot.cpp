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
// rendered as a toolbar of status chips + a scrolling transmission log.
// The toolbar also carries an LLM-fallback toggle button driving the readable
// dataref `xcopilot/llm_enabled` (-1 auto / 0 off / 1 on), which the app polls
// each utterance to enable/disable its answer-only LLM fallback live.
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

static XPLMDataRef g_phase_dref = nullptr;
static XPLMDataRef g_heard_dref = nullptr;
static XPLMDataRef g_action_dref = nullptr;
static XPLMDataRef g_alert_dref = nullptr;
static XPLMDataRef g_aircraft_dref = nullptr;
static XPLMDataRef g_strip_dref = nullptr;
static XPLMDataRef g_callout_dref = nullptr;

// Transmission log shown in the panel, each entry stamped with wall-clock time
// so a test flight can be reviewed afterwards.
struct LogEntry { std::string time; std::string tag; ImU32 color; std::string text; };
static std::vector<LogEntry> g_log;
static bool g_log_scroll = false;
static bool g_log_autoscroll = true;

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
    AppendLog("HEARD", COL_DIM, g_heard.buf);
}
static void WriteAction(void*, void* in, int offset, int len) {
    StoreText(&g_action, in, offset, len);
    AppendLog("ROGER", COL_GREEN, g_action.buf);
}
// Active aircraft ICAO — shown beside the connection chip, not logged.
static void WriteAircraft(void*, void* in, int offset, int len) {
    StoreText(&g_aircraft, in, offset, len);
}
// Live flight strip — a status row the app refreshes continuously, not logged.
static void WriteStrip(void*, void* in, int offset, int len) {
    StoreText(&g_strip, in, offset, len);
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
    // First char is a severity marker ('1' advisory, '2' caution, '3' warning) —
    // the standard annunciator hierarchy. Strip it and pick the chip accordingly.
    const char* text = g_alert.buf;
    const char* tag = "CAUTION";
    ImU32 col = COL_AMBER;
    if (g_alert.len >= 1) {
        switch (g_alert.buf[0]) {
            case '1': tag = "ADVISORY"; col = COL_CYAN; text++; break;
            case '2': tag = "CAUTION";  col = COL_AMBER; text++; break;
            case '3': tag = "WARNING";  col = COL_RED;  text++; break;
            default: break;  // no marker (legacy) — treat as caution
        }
    }
    AppendLog(tag, col, text);
}

// ---- FMS route loading ------------------------------------------------------
// The app writes the SimBrief route into xcopilot/fms/route as
// "IDENT,lat,lon,alt|IDENT,lat,lon,alt|...", then writes 1 to xcopilot/fms/load
// to build it in the GPS/FMS (only the plugin can, via XPLMNavigation).

static TextRef g_fms_route = {{0}, 0};
static int g_fms_load = 0;
static XPLMDataRef g_fms_route_dref = nullptr;
static XPLMDataRef g_fms_load_dref = nullptr;

static void WriteFmsRoute(void*, void* in, int offset, int len) {
    StoreText(&g_fms_route, in, offset, len);
}

// One parsed route point from the app's "IDENT,lat,lon,alt|..." payload, with the
// navaid it resolved to (if any) so we can pick the right .fms row type.
struct RoutePoint {
    std::string ident;
    float lat, lon;
    int alt;
    XPLMNavRef ref;     // XPLM_NAV_NOT_FOUND for a bare lat/lon point
    XPLMNavType type;   // the navaid's type (only meaningful when ref is set)
};

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
    char buf[sizeof g_fms_route.buf];
    std::strncpy(buf, g_fms_route.buf, sizeof buf);
    buf[sizeof buf - 1] = '\0';

    const XPLMNavType kTypes = xplm_Nav_Fix | xplm_Nav_VOR | xplm_Nav_NDB |
                               xplm_Nav_DME | xplm_Nav_Airport;

    // Parse the payload into points, resolving each to a navaid near its position.
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
        if (!p.ident.empty()) {
            float la = p.lat, lo = p.lon;
            p.ref = XPLMFindNavAid(nullptr, p.ident.c_str(), &la, &lo, nullptr, kTypes);
            if (p.ref != XPLM_NAV_NOT_FOUND) {
                XPLMGetNavAidInfo(p.ref, &p.type, nullptr, nullptr, nullptr, nullptr, nullptr,
                                  nullptr, nullptr, nullptr);
            }
        }
        pts.push_back(p);
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
        case 2: return "THINKING";
        case 3: return "SPEAKING";
        default: return "CONNECTED";
    }
}
static ImU32 PhaseColor(int p) {
    switch (p) {
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

class XCopilotPanel : public ImgWindow {
public:
    XCopilotPanel(int l, int t, int r, int b)
        : ImgWindow(l, t, r, b, xplm_WindowDecorationRoundRectangle,
                    xplm_WindowLayerFloatingWindows) {
        SetWindowTitle("X-COPILOT PLUGIN v" XCOPILOT_VERSION " (" XCOPILOT_BUILD ")");
        ImGuiIO& io = ImGui::GetIO();
        io.IniFilename = nullptr;
        ImGuiStyle& s = ImGui::GetStyle();
        s.WindowRounding = 8.0f;
        s.ChildRounding = 6.0f;
        s.FrameRounding = 4.0f;
        s.ItemSpacing = ImVec2(8, 6);
        s.Colors[ImGuiCol_WindowBg] = ImVec4(0.13f, 0.14f, 0.16f, 0.94f);
        s.Colors[ImGuiCol_ChildBg]  = ImVec4(0.10f, 0.11f, 0.13f, 0.85f);
        s.Colors[ImGuiCol_Border]   = ImVec4(0.30f, 0.32f, 0.36f, 0.60f);
        // Allow shrinking down to the toolbar row (collapsed view).
        SetWindowResizingLimits(260, kCollapsedH, 1400, 1400);
    }

private:
    // Collapse to just the toolbar row (status chips under the title bar), so the
    // panel can tuck out of the way without closing. Remembers the expanded size.
    static constexpr int kCollapsedH = 46;
    bool mCollapsed = false;
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

protected:
    void buildInterface() override {
        // Toolbar: PTT light + connection/phase, as chips. The app heartbeats the
        // phase ~1/s, so a stale timestamp means it isn't running.
        bool connected = (XPLMGetElapsedTime() - g_last_status_time) < 3.0f;
        Chip("PTT", g_ptt_active ? COL_GREEN : COL_DIM);
        ImGui::SameLine();
        if (connected) {
            Chip(PhaseLabel(g_phase), PhaseColor(g_phase));
            // Detected aircraft profile (ICAO), beside the connection chip.
            if (g_aircraft.len > 0) {
                ImGui::SameLine();
                Chip(g_aircraft.buf, COL_CYAN);
            }
        } else {
            Chip("NOT CONNECTED", COL_RED);
        }
        // Settings popup — all the runtime switches in one place, so the toolbar
        // stays chips-only as options grow. Each checkbox drives a dataref the app
        // polls live; changes apply on the next utterance/tick, no restart.
        ImGui::SameLine();
        if (ImGui::SmallButton("Settings")) {
            ImGui::OpenPopup("xcp_settings");
        }
        if (ImGui::BeginPopup("xcp_settings")) {
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
            ImGui::EndPopup();
        }
        // Collapse / expand to just this toolbar row, to tuck the panel away.
        ImGui::SameLine();
        if (ImGui::SmallButton(mCollapsed ? "+" : "_")) {
            toggleCollapse();
        }
        // One-click reload after a rebuild — right-aligned and set apart from the
        // other buttons so it can't be fat-fingered (it bounces ALL plugins; the
        // SDK can't reload just one).
        const char* reloadLbl = "Reload";
        float reloadW = ImGui::CalcTextSize(reloadLbl).x + ImGui::GetStyle().FramePadding.x * 2.0f;
        ImGui::SameLine(ImGui::GetContentRegionMax().x - reloadW);
        if (ImGui::SmallButton(reloadLbl)) {
            XPLMReloadPlugins();
        }
        ImGui::Separator();
        // Collapsed: stop here — just the toolbar shows under the title bar.
        if (mCollapsed) {
            return;
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

        // Emergency buttons — the mouse fallback when voice fails mid-emergency.
        // Each writes a request code the app polls and runs as the matching voice
        // command ("take control" / "land the aircraft" / "my controls").
        if (connected) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(COL_RED));
            if (ImGui::SmallButton("RECOVER")) g_request = 1;
            ImGui::SameLine();
            if (ImGui::SmallButton("AUTOLAND")) g_request = 2;
            ImGui::PopStyleColor();
            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(COL_GREEN));
            if (ImGui::SmallButton("MY CONTROLS")) g_request = 3;
            ImGui::PopStyleColor();
            if (g_request != 0) {
                ImGui::SameLine();
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "sent...");
            }
            ImGui::Separator();
        }

        // Transmission log — recent heard/action lines with coloured tags, each
        // stamped so a test can be reviewed afterwards.
        const float footer = ImGui::GetFrameHeightWithSpacing();
        ImGui::BeginChild("log", ImVec2(0, -footer), true);
        for (const LogEntry& e : g_log) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "%s", e.time.c_str());
            ImGui::SameLine();
            Chip(e.tag.c_str(), e.color);
            ImGui::SameLine();
            // Wrap long readbacks at the panel edge instead of running off it.
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(e.color));
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextUnformatted(e.text.c_str());
            ImGui::PopTextWrapPos();
            ImGui::PopStyleColor();
        }
        if (g_log_scroll) { ImGui::SetScrollHereY(1.0f); g_log_scroll = false; }
        ImGui::EndChild();

        // Footer: connection state + version on the left; log controls on the right.
        if (!connected) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "Not connected. Start x-copilot.");
        } else if (g_log.empty()) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_GREEN), "Connected to x-copilot v" XCOPILOT_VERSION ". Hold your PTT button and speak.");
        } else {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_GREEN), "Connected to x-copilot v" XCOPILOT_VERSION);
        }
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
};

// Floating callout toast: a borderless window upper-centre of the screen that
// flashes the action label ("FLAPS > 10") while the camera glances at the panel
// — the "what am I changing" graphic. Shown/hidden by a slow flight loop.
class CalloutToast : public ImgWindow {
public:
    CalloutToast(int l, int t, int r, int b)
        : ImgWindow(l, t, r, b, xplm_WindowDecorationNone, xplm_WindowLayerFloatingWindows) {
        SetWindowTitle("x-copilot callout");
        ImGuiIO& io = ImGui::GetIO();
        io.IniFilename = nullptr;
    }
protected:
    void buildInterface() override {
        ImGui::SetWindowFontScale(1.8f);
        ImVec2 ts = ImGui::CalcTextSize(g_callout.buf);
        ImGui::SetCursorPos(ImVec2((ImGui::GetWindowWidth() - ts.x) * 0.5f,
                                   (ImGui::GetWindowHeight() - ts.y) * 0.5f));
        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(COL_AMBER));
        ImGui::TextUnformatted(g_callout.buf);
        ImGui::PopStyleColor();
        ImGui::SetWindowFontScale(1.0f);
    }
};
static std::unique_ptr<CalloutToast> g_toast;

// Show the toast for a beat after each callout write, then hide it again —
// positioned toward the control (the "nx,ny|" offset) when the callout came
// with a camera glance, centre-screen otherwise.
static float ToastLoop(float, float, int, void*) {
    if (g_toast) {
        bool show = g_callout.len > 0 && (XPLMGetElapsedTime() - g_callout_time) < 2.5f;
        if (show && !g_toast->GetVisible()) {
            int sl, st, sr, sb;
            XPLMGetScreenBoundsGlobal(&sl, &st, &sr, &sb);
            const int cw = 400, chh = 84;
            int cx = (sl + sr) / 2 + static_cast<int>(g_callout_nx * (sr - sl) * 0.35f);
            int cy = (st + sb) / 2 + static_cast<int>(g_callout_ny * (st - sb) * 0.35f);
            g_toast->SetWindowGeometry(cx - cw / 2, cy + chh / 2, cx + cw / 2, cy - chh / 2);
        }
        if (g_toast->GetVisible() != show) g_toast->SetVisible(show);
    }
    return 0.2f;
}

static std::unique_ptr<XCopilotPanel> g_panel;
static XPLMMenuID g_menu = nullptr;

static void MenuHandler(void*, void*) {
    if (g_panel) g_panel->SetVisible(!g_panel->GetVisible());
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

    // One shared font atlas for all ImgWindows (uses ImGui's default font).
    ImgWindow::sFontAtlas = std::make_shared<ImgFontAtlas>();

    int sl, st, sr, sb;
    XPLMGetScreenBoundsGlobal(&sl, &st, &sr, &sb);
    int wl = sl + 50, wt = st - 50, wr = wl + 420, wb = wt - 300;
    g_panel = std::make_unique<XCopilotPanel>(wl, wt, wr, wb);
    // Callout toast: upper-centre, borderless, hidden until a callout arrives.
    int cw = 400, chh = 84;
    int cl2 = (sl + sr) / 2 - cw / 2;
    int ct2 = st - (st - sb) / 5;
    g_toast = std::make_unique<CalloutToast>(cl2, ct2, cl2 + cw, ct2 - chh);
    g_toast->SetVisible(false);
    XPLMRegisterFlightLoopCallback(ToastLoop, 1.0f, nullptr);

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
    XPLMUnregisterFlightLoopCallback(ToastLoop, nullptr);
    g_toast.reset();
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
}

PLUGIN_API int XPluginEnable(void) { return 1; }
PLUGIN_API void XPluginDisable(void) {}
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID, int, void*) {}
