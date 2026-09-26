// ---- Flight initialisation ---------------------------------------------------
// The app writes a flight-initialisation JSON (developer.x-plane.com/article/
// flight-initialization-api) into xcopilot/sim/init_flight: the aircraft, a
// ramp or runway start, cold and dark or running. Only a plugin can start a
// flight, and only from the main thread, so the write just stores it and the
// next flight loop runs XPLMInitFlight. xcopilot/sim/init_result is -1 while
// one is pending, else the XPLMInitResult of the last (0 = success), and
// xcopilot/sim/init_seq counts completed calls so the app can wait on it.
//
// What it is for: "set up the flight" (the SimBrief plan's aircraft at a
// suitable stand at its origin) and the unattended sortie runner, which loads
// an aircraft and a field, commands a mission and reads the scorecard.

#include "flightinit.h"

#include <cstdio>
#include <cstring>

#include "XPLMDataAccess.h"
#include "XPLMPlanes.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"

#include "common.h"
#include "drefs.h"

static TextRef g_init_json = {{0}, 0};
static bool g_pending = false;
static int g_result = 0;
static int g_seq = 0;
static XPLMDataRef g_json_dref = nullptr;
static XPLMDataRef g_result_dref = nullptr;
static XPLMDataRef g_seq_dref = nullptr;

static void WriteInitJson(void*, void* in, int offset, int n) {
    StoreText(&g_init_json, in, offset, n);
    g_pending = g_init_json.len > 0;
}

static int ReadResult(void*) { return g_pending ? -1 : g_result; }
static int ReadSeq(void*) { return g_seq; }

// Runs XPLMInitFlight outside any dataref callback: the call ends the flight
// and reloads the aircraft, which is no thing to do in the middle of the Web
// API dispatching a write.
static float InitLoop(float, float, int, void*) {
    if (!g_pending) return 0.25f;
    g_pending = false;
    char json[sizeof g_init_json.buf + 1];
    const int n = g_init_json.len < static_cast<int>(sizeof g_init_json.buf)
                      ? g_init_json.len
                      : static_cast<int>(sizeof g_init_json.buf);
    std::memcpy(json, g_init_json.buf, n);
    json[n] = '\0';
    g_result = XPLMInitFlight(json);
    ++g_seq;
    char msg[160];
    std::snprintf(msg, sizeof msg, "XVoiceLayer plugin: init_flight result %d (seq %d)\n", g_result, g_seq);
    XPLMDebugString(msg);
    return 0.25f;
}

namespace flightinit {

void start() {
    g_json_dref = XPLMRegisterDataAccessor(
        "xcopilot/sim/init_flight", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteInitJson, &g_init_json, &g_init_json);
    g_result_dref = XPLMRegisterDataAccessor(
        "xcopilot/sim/init_result", xplmType_Int, 0,
        ReadResult, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_seq_dref = XPLMRegisterDataAccessor(
        "xcopilot/sim/init_seq", xplmType_Int, 0,
        ReadSeq, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    XPLMRegisterFlightLoopCallback(InitLoop, 0.25f, nullptr);
}

void stop() {
    XPLMUnregisterFlightLoopCallback(InitLoop, nullptr);
    if (g_seq_dref) { XPLMUnregisterDataAccessor(g_seq_dref); g_seq_dref = nullptr; }
    if (g_result_dref) { XPLMUnregisterDataAccessor(g_result_dref); g_result_dref = nullptr; }
    if (g_json_dref) { XPLMUnregisterDataAccessor(g_json_dref); g_json_dref = nullptr; }
}

} // namespace flightinit
