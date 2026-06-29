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

// Project version. Single version for the whole product (app + plugin ship and
// must be compatible as a pair); keep in sync with Cargo.toml.
#define XCOPILOT_VERSION "0.1.0"

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

static void load_fms_route() {
    // Clear the existing flight plan (high to low so indices stay valid).
    for (int i = XPLMCountFMSEntries() - 1; i >= 0; --i) {
        XPLMClearFMSEntry(i);
    }
    char buf[sizeof g_fms_route.buf];
    std::strncpy(buf, g_fms_route.buf, sizeof buf);
    buf[sizeof buf - 1] = '\0';

    const XPLMNavType kTypes = xplm_Nav_Fix | xplm_Nav_VOR | xplm_Nav_NDB |
                               xplm_Nav_DME | xplm_Nav_Airport;
    int idx = 0;
    char* save1 = nullptr;
    for (char* tok = strtok_r(buf, "|", &save1); tok; tok = strtok_r(nullptr, "|", &save1)) {
        char* save2 = nullptr;
        char* ident = strtok_r(tok, ",", &save2);
        char* slat = strtok_r(nullptr, ",", &save2);
        char* slon = strtok_r(nullptr, ",", &save2);
        char* salt = strtok_r(nullptr, ",", &save2);
        if (!slat || !slon) continue;
        float lat = static_cast<float>(std::atof(slat));
        float lon = static_cast<float>(std::atof(slon));
        int alt = salt ? std::atoi(salt) : 0;
        // Prefer the named navaid near this position; fall back to a lat/lon point.
        XPLMNavRef ref = XPLM_NAV_NOT_FOUND;
        if (ident && ident[0]) {
            ref = XPLMFindNavAid(nullptr, ident, &lat, &lon, nullptr, kTypes);
        }
        if (ref != XPLM_NAV_NOT_FOUND) {
            XPLMSetFMSEntryInfo(idx, ref, alt);
        } else {
            XPLMSetFMSEntryLatLon(idx, lat, lon, alt);
        }
        idx++;
    }
    // Active leg = the first fix *after* the origin (entry 1), so we fly forward up
    // the route rather than back to the departure airport at entry 0. The GPS then
    // sequences onward by itself. (With a single entry, just target it.)
    if (idx > 1) {
        XPLMSetDestinationFMSEntry(1);
    } else if (idx == 1) {
        XPLMSetDestinationFMSEntry(0);
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

// ---- jet engine start -------------------------------------------------------
// The app writes 1 to xcopilot/cmd/start_engines; the plugin runs the turbine
// start IN-PROCESS. This is the whole point of doing it here: XPLMCommandOnce
// reaches laminar's own command handlers (APU bleed, ignition select), which the
// X-Plane Web/REST API can't activate from outside the sim. A flight-loop state
// machine steps the sequence, watching N2 like a pilot holding the starter.
//
// MD-82-specific datarefs/commands for now (laminar/md82...); null-checked, so on
// other aircraft the steps that don't apply are simply skipped.

static int g_start_cmd = 0;
static XPLMDataRef g_start_cmd_dref = nullptr;

enum StartState { ST_IDLE, ST_PREP, ST_CRANK, ST_LIGHT, ST_RUN_WAIT, ST_FINISH };
static StartState g_start_state = ST_IDLE;
static int g_start_pos = 0;       // index into the start order
static float g_start_timer = 0.f; // seconds elapsed in the current wait

// Cached refs (resolved lazily once the MD-82 — and its plugin — are loaded).
static XPLMDataRef dr_n2 = nullptr, dr_mixt = nullptr, dr_fuelpump = nullptr,
                   dr_apu_on = nullptr, dr_ign = nullptr, dr_xfl = nullptr, dr_xfr = nullptr;
static XPLMCommandRef cmd_apu_up = nullptr, cmd_ign_up = nullptr, cmd_ign_dwn = nullptr;
static bool g_start_refs_ready = false;

// Start order: right engine (index 1) first, then left (index 0) — one starter on
// the bleed at a time, as the checklist runs it.
static const int kStartOrder[2] = {1, 0};

static void init_start_refs() {
    if (g_start_refs_ready) return;
    dr_n2       = XPLMFindDataRef("sim/cockpit2/engine/indicators/N2_percent");
    dr_mixt     = XPLMFindDataRef("sim/flightmodel/engine/ENGN_mixt");
    dr_fuelpump = XPLMFindDataRef("sim/cockpit2/fuel/fuel_tank_pump_on");
    dr_apu_on   = XPLMFindDataRef("laminar/md82/bleedair/APU_on");
    dr_ign      = XPLMFindDataRef("laminar/md82/ignition_sys");
    dr_xfl      = XPLMFindDataRef("laminar/md82/bleedair/engineL_xfeed_lever");
    dr_xfr      = XPLMFindDataRef("laminar/md82/bleedair/engineR_xfeed_lever");
    cmd_apu_up  = XPLMFindCommand("laminar/md82cmd/bleedair/APU_up");
    cmd_ign_up  = XPLMFindCommand("laminar/md82cmd/ignition_sys_up");
    cmd_ign_dwn = XPLMFindCommand("laminar/md82cmd/ignition_sys_dwn");
    g_start_refs_ready = true;
}

static float N2_of(int eng) {
    if (!dr_n2) return 0.f;
    float v[8] = {0};
    XPLMGetDatavf(dr_n2, v, 0, 8);
    return (eng >= 0 && eng < 8) ? v[eng] : 0.f;
}

// Stepped by the flight loop every ~0.5s. Returns seconds to next call (0 = stop).
static float StartFlightLoop(float, float, int, void*) {
    switch (g_start_state) {
    case ST_IDLE:
        return 0.f; // disabled until armed again
    case ST_PREP: {
        // Open the APU bleed valve (command — the dataref is read-only), engine
        // bleed cross-feeds (writable), fuel boost pumps, and select ignition A.
        if (dr_apu_on && XPLMGetDataf(dr_apu_on) < 0.5f && cmd_apu_up) XPLMCommandOnce(cmd_apu_up);
        if (dr_xfl) XPLMSetDataf(dr_xfl, 1.0f);
        if (dr_xfr) XPLMSetDataf(dr_xfr, 1.0f);
        if (dr_fuelpump) { int p[3] = {1, 1, 1}; XPLMSetDatavi(dr_fuelpump, p, 0, 3); }
        if (dr_ign && XPLMGetDataf(dr_ign) < 0.5f && cmd_ign_up) XPLMCommandOnce(cmd_ign_up);
        g_start_pos = 0;
        g_start_state = ST_CRANK;
        return 1.0f; // give the bleed a moment
    }
    case ST_CRANK: {
        int eng = kStartOrder[g_start_pos];
        XPLMCommandRef st = XPLMFindCommand(
            eng == 0 ? "sim/starters/engage_starter_1" : "sim/starters/engage_starter_2");
        if (st) XPLMCommandOnce(st);
        g_start_timer = 0.f;
        g_start_state = ST_LIGHT;
        return 0.5f;
    }
    case ST_LIGHT: { // wait for the starter to motor N2 up, then introduce fuel
        int eng = kStartOrder[g_start_pos];
        g_start_timer += 0.5f;
        if (N2_of(eng) > 20.0f) {
            if (dr_mixt) { float one = 1.0f; XPLMSetDatavf(dr_mixt, &one, eng, 1); }
            g_start_timer = 0.f;
            g_start_state = ST_RUN_WAIT;
        } else if (g_start_timer > 45.0f) {
            AppendLog("STARTER", COL_AMBER, "Engine didn't spool, check bleed air. Start aborted.");
            g_start_state = ST_FINISH;
        }
        return 0.5f;
    }
    case ST_RUN_WAIT: { // wait for it to reach idle, then the next engine
        int eng = kStartOrder[g_start_pos];
        g_start_timer += 0.5f;
        if (N2_of(eng) > 50.0f || g_start_timer > 60.0f) {
            g_start_pos++;
            g_start_state = (g_start_pos < 2) ? ST_CRANK : ST_FINISH;
        }
        return 0.5f;
    }
    case ST_FINISH: {
        if (dr_ign && cmd_ign_dwn) {
            for (int i = 0; i < 4 && XPLMGetDataf(dr_ign) > 0.5f; ++i) XPLMCommandOnce(cmd_ign_dwn);
        }
        AppendLog("STARTER", COL_GREEN, "Engine start complete.");
        g_start_state = ST_IDLE;
        return 0.f;
    }
    }
    return 0.f;
}

static int ReadStartCmd(void*) { return g_start_cmd; }
static void WriteStartCmd(void*, int v) {
    if (v != 0 && g_start_state == ST_IDLE) {
        init_start_refs();
        g_start_state = ST_PREP;
        AppendLog("STARTER", COL_CYAN, "Starting engines, stand by.");
        XPLMSetFlightLoopCallbackInterval(StartFlightLoop, 0.5f, 1, nullptr);
    }
    g_start_cmd = 0; // edge-triggered
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
        SetWindowTitle("X-COPILOT PLUGIN v" XCOPILOT_VERSION);
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
        ImGui::Separator();

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
    g_start_cmd_dref = XPLMRegisterDataAccessor(
        "xcopilot/cmd/start_engines", xplmType_Int, 1,
        ReadStartCmd, WriteStartCmd, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    // Engine-start state machine, registered idle (interval 0); WriteStartCmd arms it.
    XPLMRegisterFlightLoopCallback(StartFlightLoop, 0.0f, nullptr);

    // One shared font atlas for all ImgWindows (uses ImGui's default font).
    ImgWindow::sFontAtlas = std::make_shared<ImgFontAtlas>();

    int sl, st, sr, sb;
    XPLMGetScreenBoundsGlobal(&sl, &st, &sr, &sb);
    int wl = sl + 50, wt = st - 50, wr = wl + 420, wb = wt - 300;
    g_panel = std::make_unique<XCopilotPanel>(wl, wt, wr, wb);

    int idx = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "X-COPILOT", nullptr, 0);
    g_menu = XPLMCreateMenu("X-COPILOT", XPLMFindPluginsMenu(), idx, MenuHandler, nullptr);
    XPLMAppendMenuItem(g_menu, "Toggle status panel", (void*)"toggle", 0);

    XPLMDebugString("x-copilot plugin: started (ptt + ImGui status panel)\n");
    return 1;
}

PLUGIN_API void XPluginStop(void) {
    g_panel.reset();
    ImgWindow::sFontAtlas.reset();
    if (g_menu) { XPLMDestroyMenu(g_menu); g_menu = nullptr; }
    if (g_fms_load_dref) { XPLMUnregisterDataAccessor(g_fms_load_dref); g_fms_load_dref = nullptr; }
    if (g_fms_route_dref) { XPLMUnregisterDataAccessor(g_fms_route_dref); g_fms_route_dref = nullptr; }
    XPLMUnregisterFlightLoopCallback(StartFlightLoop, nullptr);
    if (g_start_cmd_dref) { XPLMUnregisterDataAccessor(g_start_cmd_dref); g_start_cmd_dref = nullptr; }
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
