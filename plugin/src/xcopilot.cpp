// x-copilot X-Plane plugin — a virtual push-to-talk (and, later, a status display)
// for the x-copilot voice assistant.
//
// PTT: registers a command `xcopilot/ptt` (bind it to a joystick button or key in
// X-Plane → Settings → Joystick/Keyboard) and a readable dataref
// `xcopilot/ptt_active` that is 1 while the command is held. The x-copilot app
// reads that dataref over the X-Plane Web API and captures the mic only while it's
// 1 — a dedicated PTT that, unlike `contact_atc_ptt`, does NOT key the radio and
// needs no macOS permission.
//
// Build/install: see plugin/README.md.

#include <cstring>

#include "XPLMDataAccess.h"
#include "XPLMPlugin.h"
#include "XPLMUtilities.h"

static int g_ptt_active = 0;
static XPLMCommandRef g_ptt_cmd = nullptr;
static XPLMDataRef g_ptt_dref = nullptr;

// 1 while the PTT command is held, 0 otherwise — drives `xcopilot/ptt_active`.
static int PttCommandHandler(XPLMCommandRef /*cmd*/, XPLMCommandPhase phase, void* /*ref*/) {
    if (phase == xplm_CommandBegin) {
        g_ptt_active = 1;
    } else if (phase == xplm_CommandEnd) {
        g_ptt_active = 0;
    }
    return 1;  // not consumed; there's no default behaviour for a custom command
}

static int ReadPttActive(void* /*ref*/) { return g_ptt_active; }

PLUGIN_API int XPluginStart(char* name, char* sig, char* desc) {
    std::strcpy(name, "x-copilot");
    std::strcpy(sig, "com.dlicudi.xcopilot");
    std::strcpy(desc, "Push-to-talk (and status) for the x-copilot voice assistant.");

    g_ptt_cmd = XPLMCreateCommand("xcopilot/ptt", "x-copilot: hold to talk");
    XPLMRegisterCommandHandler(g_ptt_cmd, PttCommandHandler, 1 /*before*/, nullptr);

    // A read-only int dataref the x-copilot app polls over the Web API.
    g_ptt_dref = XPLMRegisterDataAccessor(
        "xcopilot/ptt_active", xplmType_Int, 0 /*not writable*/,
        ReadPttActive, nullptr,  // int
        nullptr, nullptr,        // float
        nullptr, nullptr,        // double
        nullptr, nullptr,        // int array
        nullptr, nullptr,        // float array
        nullptr, nullptr,        // raw data
        nullptr, nullptr);       // read/write refcons

    XPLMDebugString("x-copilot plugin: started (xcopilot/ptt + xcopilot/ptt_active)\n");
    return 1;
}

PLUGIN_API void XPluginStop(void) {
    if (g_ptt_dref) {
        XPLMUnregisterDataAccessor(g_ptt_dref);
        g_ptt_dref = nullptr;
    }
    if (g_ptt_cmd) {
        XPLMUnregisterCommandHandler(g_ptt_cmd, PttCommandHandler, 1, nullptr);
        g_ptt_cmd = nullptr;
    }
}

PLUGIN_API int XPluginEnable(void) { return 1; }
PLUGIN_API void XPluginDisable(void) {}
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID /*from*/, int /*msg*/, void* /*param*/) {}
