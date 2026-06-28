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
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "XPLMDataAccess.h"
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMMenus.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"

// Project version. Single version for the whole product (app + plugin ship and
// must be compatible as a pair); keep in sync with Cargo.toml.
#define NINER_VERSION "0.1.0"

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

static XPLMDataRef g_phase_dref = nullptr;
static XPLMDataRef g_heard_dref = nullptr;
static XPLMDataRef g_action_dref = nullptr;
static XPLMDataRef g_alert_dref = nullptr;

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
        SetWindowTitle("Niner");
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

        // Footer hint.
        if (!connected) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "Waiting for the copilot to connect.");
        } else if (g_log.empty()) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "Niner " NINER_VERSION ". Hold your PTT button and speak.");
        } else {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "%s", PhaseLabel(g_phase));
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
    std::strcpy(name, "Niner");
    std::strcpy(sig, "com.dlicudi.xcopilot");
    std::strcpy(desc, "Niner " NINER_VERSION ", push-to-talk and status panel for the voice copilot.");

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

    // One shared font atlas for all ImgWindows (uses ImGui's default font).
    ImgWindow::sFontAtlas = std::make_shared<ImgFontAtlas>();

    int sl, st, sr, sb;
    XPLMGetScreenBoundsGlobal(&sl, &st, &sr, &sb);
    int wl = sl + 50, wt = st - 50, wr = wl + 420, wb = wt - 300;
    g_panel = std::make_unique<XCopilotPanel>(wl, wt, wr, wb);

    int idx = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "Niner", nullptr, 0);
    g_menu = XPLMCreateMenu("Niner", XPLMFindPluginsMenu(), idx, MenuHandler, nullptr);
    XPLMAppendMenuItem(g_menu, "Toggle status panel", (void*)"toggle", 0);

    XPLMDebugString("x-copilot plugin: started (ptt + ImGui status panel)\n");
    return 1;
}

PLUGIN_API void XPluginStop(void) {
    g_panel.reset();
    ImgWindow::sFontAtlas.reset();
    if (g_menu) { XPLMDestroyMenu(g_menu); g_menu = nullptr; }
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
