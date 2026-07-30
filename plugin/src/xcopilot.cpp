// XVoiceLayer X-Plane plugin — a virtual push-to-talk plus a slick in-sim status
// panel (Dear ImGui) for the XVoiceLayer voice assistant.
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
// Toggle via Plugins → XVoiceLayer.
//
// Layout: this file is only the plugin lifecycle. The dataref/command surface
// lives in drefs.cpp, the ImGui panel in panel.cpp, and the FMS mailbox plus
// flight-plan composer in fms.cpp.
//
// Build/install: see plugin/README.md.

#include <cstdio>
#include <cstring>

#include "XPLMPlugin.h"
#include "XPLMUtilities.h"

#include "common.h"
#include "drefs.h"
#include "fms.h"
#include "panel.h"

// ---- lifecycle --------------------------------------------------------------

PLUGIN_API int XPluginStart(char* name, char* sig, char* desc) {
    std::strcpy(name, "XVoiceLayer");
    // The SIGNATURE is an identity contract, not branding, so it keeps the old
    // name for the same reason the xcopilot/* datarefs do: X-Plane and any
    // other plugin identify us by this string. Nothing looks it up today, but
    // changing it buys nothing and can only break a future lookup.
    std::strcpy(sig, "com.dlicudi.xcopilot");
    std::strcpy(desc, "XVoiceLayer " XCOPILOT_VERSION ", push-to-talk and status panel for the voice layer.");

    // Get POSIX paths from the SDK, not legacy Mac HFS ("Macintosh HD:Users:...");
    // otherwise fopen can't open the nav-data file we read the AIRAC cycle from.
    XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);

    drefs::start();
    panel::create();

    fms::read_airac_cycle(); // for stamping generated flight plans with the matching cycle

    char started[160];
    std::snprintf(started, sizeof started,
                  "XVoiceLayer plugin: started, build %s, AIRAC %s (ptt + ImGui status panel)\n",
                  XCOPILOT_BUILD,
                  fms::airac_cycle().empty() ? "unknown" : fms::airac_cycle().c_str());
    XPLMDebugString(started);
    return 1;
}

PLUGIN_API void XPluginStop(void) {
    // Teardown mirrors the original single-file order: the flight loop and the
    // panel-feed datarefs first, then the panel itself, then the rest.
    drefs::stop_first();
    panel::destroy();
    drefs::stop_rest();
}

PLUGIN_API int XPluginEnable(void) { return 1; }
PLUGIN_API void XPluginDisable(void) {}
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID, int, void*) {}
