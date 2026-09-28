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
//
// The load quiet window: xcopilot/sim/loading (read-only) is 1 from the
// mailbox write until 3 s of sim frames after XPLMInitFlight returns, and the
// app holds its non-essential Web API traffic while it reads 1. X-Plane
// 12.4.4-b2 hung twice (2026-09-28) 1 to 2 s after an init that unloaded the
// SR22 (XLua 1.1.0r1), with our requests still arriving through the load; the
// cause is unproven, so the load is also bracketed in Log.txt: a line right
// before XPLMInitFlight and one at the first flight-loop tick after it.

#include "flightinit.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>

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
static XPLMDataRef g_loading_dref = nullptr;

// The quiet window. Sim seconds after the call are summed per frame, each
// frame's share capped so the load's own long first frame does not count.
static const float kQuietSimS = 3.0f;
static const float kFrameCapS = 0.1f;
// A paused sim runs no flight loop: past this wall time after the call the
// window reads closed anyway (the app fails open on its side too).
static const double kQuietWallMaxS = 15.0;
static bool g_loading = false;
static bool g_after_call = false;  // the call returned, the window is counting
static bool g_first_tick = false;  // the next loop tick is the first after it
static float g_quiet_s = 0.0f;
static int g_quiet_frames = 0;
static std::chrono::steady_clock::time_point g_returned_at;

static double wall_since(std::chrono::steady_clock::time_point t) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count();
}

// The start type ("ramp_start", "runway_start", ...) and the aircraft's path,
// for the Log.txt bracket. A scan, not a parser: the app writes the JSON.
static std::string start_type(const char* json) {
    const char* p = std::strstr(json, "_start\"");
    if (!p) return "?";
    const char* q = p;
    while (q > json && q[-1] != '"') --q;
    return std::string(q, p + 6);
}

static std::string aircraft_path(const char* json) {
    const char* a = std::strstr(json, "\"aircraft\"");
    const char* p = a ? std::strstr(a, "\"path\"") : nullptr;
    if (!p) return "?";
    p = std::strchr(p + 6, ':');
    if (!p) return "?";
    p = std::strchr(p, '"');
    if (!p) return "?";
    const char* e = std::strchr(p + 1, '"');
    if (!e) return "?";
    return std::string(p + 1, e);
}

static void WriteInitJson(void*, void* in, int offset, int n) {
    StoreText(&g_init_json, in, offset, n);
    g_pending = g_init_json.len > 0;
    if (g_pending) {
        g_loading = true;
        g_after_call = false;
    }
}

static int ReadResult(void*) { return g_pending ? -1 : g_result; }
static int ReadSeq(void*) { return g_seq; }

static int ReadLoading(void*) {
    if (!g_loading) return 0;
    if (g_after_call && wall_since(g_returned_at) > kQuietWallMaxS) return 0;
    return 1;
}

// Counts the quiet window after a call, one frame at a time.
static float QuietTick(float since_last_loop) {
    if (g_first_tick) {
        g_first_tick = false;
        char msg[192];
        std::snprintf(msg, sizeof msg,
                      "XVoiceLayer plugin: init_flight seq %d: first flight loop after the call "
                      "(%.2f s wall, %.2f s sim)\n",
                      g_seq, wall_since(g_returned_at), since_last_loop);
        XPLMDebugString(msg);
    }
    g_quiet_s += since_last_loop < kFrameCapS ? since_last_loop : kFrameCapS;
    ++g_quiet_frames;
    if (g_quiet_s < kQuietSimS && wall_since(g_returned_at) <= kQuietWallMaxS) {
        return -1.0f;  // every frame until the window closes
    }
    g_after_call = false;
    g_loading = false;
    char msg[160];
    std::snprintf(msg, sizeof msg,
                  "XVoiceLayer plugin: init_flight seq %d: load quiet window closed "
                  "(%d frames, %.1f s wall)\n",
                  g_seq, g_quiet_frames, wall_since(g_returned_at));
    XPLMDebugString(msg);
    return 0.25f;
}

// Runs XPLMInitFlight outside any dataref callback: the call ends the flight
// and reloads the aircraft, which is no thing to do in the middle of the Web
// API dispatching a write.
static float InitLoop(float, float since_last_loop, int, void*) {
    if (!g_pending) {
        return g_after_call ? QuietTick(since_last_loop) : 0.25f;
    }
    g_pending = false;
    char json[sizeof g_init_json.buf + 1];
    const int n = g_init_json.len < static_cast<int>(sizeof g_init_json.buf)
                      ? g_init_json.len
                      : static_cast<int>(sizeof g_init_json.buf);
    std::memcpy(json, g_init_json.buf, n);
    json[n] = '\0';
    {
        char msg[512];
        std::snprintf(msg, sizeof msg,
                      "XVoiceLayer plugin: init_flight seq %d: calling XPLMInitFlight (%s, %s)\n",
                      g_seq + 1, start_type(json).c_str(), aircraft_path(json).c_str());
        XPLMDebugString(msg);
    }
    const auto called_at = std::chrono::steady_clock::now();
    g_result = XPLMInitFlight(json);
    ++g_seq;
    g_returned_at = std::chrono::steady_clock::now();
    g_after_call = true;
    g_first_tick = true;
    g_quiet_s = 0.0f;
    g_quiet_frames = 0;
    char msg[160];
    std::snprintf(msg, sizeof msg,
                  "XVoiceLayer plugin: init_flight result %d (seq %d, %.2f s in the call)\n",
                  g_result, g_seq, wall_since(called_at));
    XPLMDebugString(msg);
    return -1.0f;
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
    g_loading_dref = XPLMRegisterDataAccessor(
        "xcopilot/sim/loading", xplmType_Int, 0,
        ReadLoading, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    XPLMRegisterFlightLoopCallback(InitLoop, 0.25f, nullptr);
}

void stop() {
    XPLMUnregisterFlightLoopCallback(InitLoop, nullptr);
    if (g_loading_dref) { XPLMUnregisterDataAccessor(g_loading_dref); g_loading_dref = nullptr; }
    if (g_seq_dref) { XPLMUnregisterDataAccessor(g_seq_dref); g_seq_dref = nullptr; }
    if (g_result_dref) { XPLMUnregisterDataAccessor(g_result_dref); g_result_dref = nullptr; }
    if (g_json_dref) { XPLMUnregisterDataAccessor(g_json_dref); g_json_dref = nullptr; }
}

bool loading() { return ReadLoading(nullptr) != 0; }

} // namespace flightinit
