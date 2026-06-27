// x-copilot X-Plane plugin — a virtual push-to-talk and a compact in-sim status
// panel for the x-copilot voice assistant.
//
// PTT: registers a command `xcopilot/ptt` (bind it to a joystick button or key in
// X-Plane → Settings → Keyboard/Joystick) and a readable dataref
// `xcopilot/ptt_active` that is 1 while the command is held. The x-copilot app
// reads that dataref over the Web API and captures the mic only while it's 1 — a
// dedicated PTT that doesn't key the radio and needs no macOS permission.
//
// Status panel: registers writable datarefs the app pushes into —
//   xcopilot/status/phase  (int: 0 idle, 1 listening, 2 thinking, 3 speaking)
//   xcopilot/status/heard  (text: last transcript)
//   xcopilot/status/action (text: last readback)
// and draws them in a small floating window (toggle via Plugins → x-copilot).
//
// Build/install: see plugin/README.md.

#include <cstdio>
#include <cstring>

#include "XPLMDataAccess.h"
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMMenus.h"
#include "XPLMPlugin.h"
#include "XPLMUtilities.h"

// ---- PTT --------------------------------------------------------------------

static int g_ptt_active = 0;
static XPLMCommandRef g_ptt_cmd = nullptr;
static XPLMDataRef g_ptt_dref = nullptr;

static int PttCommandHandler(XPLMCommandRef /*cmd*/, XPLMCommandPhase phase, void* /*ref*/) {
    if (phase == xplm_CommandBegin) {
        g_ptt_active = 1;
    } else if (phase == xplm_CommandEnd) {
        g_ptt_active = 0;
    }
    return 1;  // not consumed; there's no default behaviour for a custom command
}

static int ReadPttActive(void* /*ref*/) { return g_ptt_active; }

// ---- Status datarefs --------------------------------------------------------

// A small fixed text buffer exposed as a writable byte-array dataref.
struct TextRef {
    char buf[128];
    int len;
};

static int g_phase = 0;
static TextRef g_heard = {{0}, 0};
static TextRef g_action = {{0}, 0};

static XPLMDataRef g_phase_dref = nullptr;
static XPLMDataRef g_heard_dref = nullptr;
static XPLMDataRef g_action_dref = nullptr;

static int ReadPhase(void* /*ref*/) { return g_phase; }
static void WritePhase(void* /*ref*/, int v) { g_phase = v; }

// Byte-array read: NULL out = size query; else copy [offset, offset+max).
static int ReadText(void* ref, void* out, int offset, int max) {
    TextRef* t = static_cast<TextRef*>(ref);
    if (out == nullptr) return t->len;
    if (offset >= t->len) return 0;
    int n = t->len - offset;
    if (n > max) n = max;
    std::memcpy(out, t->buf + offset, static_cast<size_t>(n));
    return n;
}

// Byte-array write: the app PATCHes the whole value at offset 0; we keep it
// NUL-terminated so the draw code can treat it as a C string.
static void WriteText(void* ref, void* in, int offset, int len) {
    TextRef* t = static_cast<TextRef*>(ref);
    if (offset != 0) return;
    if (len > 127) len = 127;
    if (len < 0) len = 0;
    std::memset(t->buf, 0, sizeof t->buf);
    std::memcpy(t->buf, in, static_cast<size_t>(len));
    t->len = len;
}

// ---- Status panel -----------------------------------------------------------

static XPLMWindowID g_win = nullptr;
static XPLMMenuID g_menu = nullptr;

static const char* PhaseLabel(int p) {
    switch (p) {
        case 1: return "Listening";
        case 2: return "Thinking";
        case 3: return "Speaking";
        default: return "Idle";
    }
}

static void DrawPanel(XPLMWindowID win, void* /*ref*/) {
    int l, t, r, b;
    XPLMGetWindowGeometry(win, &l, &t, &r, &b);

    float white[] = {0.92f, 0.92f, 0.92f};
    float green[] = {0.25f, 0.9f, 0.35f};
    float dim[]   = {0.45f, 0.45f, 0.45f};
    float amber[] = {1.0f, 0.72f, 0.15f};
    float cyan[]  = {0.35f, 0.8f, 1.0f};

    int x = l + 8;
    int y = t - 16;

    XPLMDrawString(white, x, y, const_cast<char*>("x-copilot"), nullptr, xplmFont_Proportional);
    y -= 18;

    // PTT light + current phase, side by side.
    float* ptt_col = g_ptt_active ? green : dim;
    XPLMDrawString(ptt_col, x, y, const_cast<char*>("[PTT]"), nullptr, xplmFont_Proportional);
    float* ph_col = g_phase == 1 ? green : g_phase == 2 ? amber : g_phase == 3 ? cyan : dim;
    XPLMDrawString(ph_col, x + 46, y, const_cast<char*>(PhaseLabel(g_phase)), nullptr,
                   xplmFont_Proportional);
    y -= 16;

    if (g_heard.len > 0) {
        char line[160];
        std::snprintf(line, sizeof line, "> %s", g_heard.buf);
        XPLMDrawString(white, x, y, line, nullptr, xplmFont_Proportional);
    }
    y -= 16;
    if (g_action.len > 0) {
        char line[160];
        std::snprintf(line, sizeof line, "= %s", g_action.buf);
        XPLMDrawString(green, x, y, line, nullptr, xplmFont_Proportional);
    }
}

static int NoMouse(XPLMWindowID, int, int, XPLMMouseStatus, void*) { return 1; }
static XPLMCursorStatus NoCursor(XPLMWindowID, int, int, void*) { return xplm_CursorDefault; }
static int NoWheel(XPLMWindowID, int, int, int, int, void*) { return 1; }
static void NoKey(XPLMWindowID, char, XPLMKeyFlags, char, void*, int) {}

static void CreatePanel() {
    int sl, st, sr, sb;
    XPLMGetScreenBoundsGlobal(&sl, &st, &sr, &sb);

    XPLMCreateWindow_t p;
    std::memset(&p, 0, sizeof p);
    p.structSize = sizeof p;
    p.left = sl + 50;
    p.top = st - 50;
    p.right = p.left + 240;
    p.bottom = p.top - 92;
    p.visible = 1;
    p.drawWindowFunc = DrawPanel;
    p.handleMouseClickFunc = NoMouse;
    p.handleRightClickFunc = NoMouse;
    p.handleMouseWheelFunc = NoWheel;
    p.handleKeyFunc = NoKey;
    p.handleCursorFunc = NoCursor;
    p.refcon = nullptr;
    p.layer = xplm_WindowLayerFloatingWindows;
    p.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;
    g_win = XPLMCreateWindowEx(&p);
}

static void MenuHandler(void* /*menu*/, void* /*item*/) {
    if (g_win) {
        XPLMSetWindowIsVisible(g_win, !XPLMGetWindowIsVisible(g_win));
    }
}

// ---- Plugin lifecycle -------------------------------------------------------

PLUGIN_API int XPluginStart(char* name, char* sig, char* desc) {
    std::strcpy(name, "x-copilot");
    std::strcpy(sig, "com.dlicudi.xcopilot");
    std::strcpy(desc, "Push-to-talk and status panel for the x-copilot voice assistant.");

    g_ptt_cmd = XPLMCreateCommand("xcopilot/ptt", "x-copilot: hold to talk");
    XPLMRegisterCommandHandler(g_ptt_cmd, PttCommandHandler, 1 /*before*/, nullptr);

    g_ptt_dref = XPLMRegisterDataAccessor(
        "xcopilot/ptt_active", xplmType_Int, 0 /*not writable*/,
        ReadPttActive, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);

    g_phase_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/phase", xplmType_Int, 1 /*writable*/,
        ReadPhase, WritePhase, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);

    g_heard_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/heard", xplmType_Data, 1 /*writable*/,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteText, &g_heard, &g_heard);

    g_action_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/action", xplmType_Data, 1 /*writable*/,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteText, &g_action, &g_action);

    CreatePanel();

    int idx = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "x-copilot", nullptr, 0);
    g_menu = XPLMCreateMenu("x-copilot", XPLMFindPluginsMenu(), idx, MenuHandler, nullptr);
    XPLMAppendMenuItem(g_menu, "Toggle status panel", (void*)"toggle", 0);

    XPLMDebugString("x-copilot plugin: started (ptt + status panel)\n");
    return 1;
}

PLUGIN_API void XPluginStop(void) {
    if (g_win) { XPLMDestroyWindow(g_win); g_win = nullptr; }
    if (g_menu) { XPLMDestroyMenu(g_menu); g_menu = nullptr; }
    if (g_action_dref) { XPLMUnregisterDataAccessor(g_action_dref); g_action_dref = nullptr; }
    if (g_heard_dref) { XPLMUnregisterDataAccessor(g_heard_dref); g_heard_dref = nullptr; }
    if (g_phase_dref) { XPLMUnregisterDataAccessor(g_phase_dref); g_phase_dref = nullptr; }
    if (g_ptt_dref) { XPLMUnregisterDataAccessor(g_ptt_dref); g_ptt_dref = nullptr; }
    if (g_ptt_cmd) {
        XPLMUnregisterCommandHandler(g_ptt_cmd, PttCommandHandler, 1, nullptr);
        g_ptt_cmd = nullptr;
    }
}

PLUGIN_API int XPluginEnable(void) { return 1; }
PLUGIN_API void XPluginDisable(void) {}
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID /*from*/, int /*msg*/, void* /*param*/) {}
