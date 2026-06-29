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
// Toggle via Plugins → x-copilot.
//
// Build/install: see plugin/README.md.

#include <cstdio>
#include <cstdlib>
#include <cstring>
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

// ---- status datarefs --------------------------------------------------------

struct TextRef { char buf[512]; int len; };

static int g_phase = 0;
static TextRef g_heard = {{0}, 0};
static TextRef g_action = {{0}, 0};
static TextRef g_alert = {{0}, 0};
static TextRef g_aircraft = {{0}, 0};

static XPLMDataRef g_phase_dref = nullptr;
static XPLMDataRef g_heard_dref = nullptr;
static XPLMDataRef g_action_dref = nullptr;
static XPLMDataRef g_alert_dref = nullptr;
static XPLMDataRef g_aircraft_dref = nullptr;

// Transmission log shown in the panel.
struct LogEntry { std::string tag; ImU32 color; std::string text; };
static std::vector<LogEntry> g_log;
static bool g_log_scroll = false;

static void AppendLog(const char* tag, ImU32 color, const char* text) {
    if (text == nullptr || text[0] == '\0') return;
    g_log.push_back({tag, color, text});
    if (g_log.size() > 60) g_log.erase(g_log.begin());
    g_log_scroll = true;
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

// Store the whole value (the app PATCHes at offset 0), NUL-terminated.
static void StoreText(TextRef* t, void* in, int offset, int len) {
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

    // Diagnostic: read back the native primary plan's first (ORIGIN) and last
    // (DEST) entries so we can see whether the airports actually landed in it and
    // resolved as airport navaids (type 1) — the CDU's ORIGIN/DEST want airports.
    {
        int n = XPLMCountFMSFlightPlanEntries(xplm_Fpl_Pilot_Primary);
        char dbg[256];
        std::snprintf(dbg, sizeof dbg, "x-copilot plugin: native plan has %d entries\n", n);
        XPLMDebugString(dbg);
        int probes[2] = {0, n - 1};
        for (int p = 0; p < 2; ++p) {
            int i = probes[p];
            if (i < 0) continue;
            XPLMNavType ty = 0;
            char id[256];
            id[0] = '\0';
            XPLMNavRef ref = XPLM_NAV_NOT_FOUND;
            int al = 0;
            float la = 0.f, lo = 0.f;
            XPLMGetFMSFlightPlanEntryInfo(xplm_Fpl_Pilot_Primary, i, &ty, id, &ref, &al, &la, &lo);
            std::snprintf(dbg, sizeof dbg,
                          "x-copilot plugin: entry %d type=%d id=%s ref=%d lat=%.3f lon=%.3f\n",
                          i, static_cast<int>(ty), id, static_cast<int>(ref), la, lo);
            XPLMDebugString(dbg);
        }
    }
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

        // Transmission log — recent heard/action lines with coloured tags.
        const float footer = ImGui::GetFrameHeightWithSpacing();
        ImGui::BeginChild("log", ImVec2(0, -footer), true);
        for (const LogEntry& e : g_log) {
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

        // Footer: connection state + version.
        if (!connected) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "Not connected. Start x-copilot.");
        } else if (g_log.empty()) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_GREEN), "Connected to x-copilot v" XCOPILOT_VERSION ". Hold your PTT button and speak.");
        } else {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_GREEN), "Connected to x-copilot v" XCOPILOT_VERSION);
        }
    }
};

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

    // One shared font atlas for all ImgWindows (uses ImGui's default font).
    ImgWindow::sFontAtlas = std::make_shared<ImgFontAtlas>();

    int sl, st, sr, sb;
    XPLMGetScreenBoundsGlobal(&sl, &st, &sr, &sb);
    int wl = sl + 50, wt = st - 50, wr = wl + 420, wb = wt - 300;
    g_panel = std::make_unique<XCopilotPanel>(wl, wt, wr, wb);

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
    g_panel.reset();
    ImgWindow::sFontAtlas.reset();
    if (g_menu) { XPLMDestroyMenu(g_menu); g_menu = nullptr; }
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
