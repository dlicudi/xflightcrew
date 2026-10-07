// The nosewheel tiller channel. See tiller.h for the contract.
//
// Why it exists: the pedal channel (xcopilot/taxi_steer, yoke_heading_ratio)
// is the RUDDER pedals, and an airliner's pedals steer the nose only a few
// degrees. Probed on the Zibo 737 parked with hydraulics up (2026-10-07), 0.5
// on the pedals gave 11 degrees of rudder and 3.5 of nosewheel
// (tire_steer_command_deg), so auto taxi held full steer for many seconds at
// 8 kt with 152 degrees to turn and rolled 90 m off its route. Real turns use
// the tiller (78 degrees on the 737, acf_nw_steerdeg1; the pedals' 7 is
// acf_nw_steerdeg2).
//
// The Zibo's own plugin (zibomod) owns the nose: it holds override_wheel_steer
// and writes tire_steer_command_deg itself every frame from its inputs. Its
// NOSEWHEEL AXIS option (laminar/B738/effects/nosewheel, persisted as
// "NOSEWHEEL AXIS" in b738x.cfg: 0 ON = the hardware tiller axis, 1 YAW, 2
// ROLL, 3 VR) picks the input, and in VR the input is
// laminar/B738/nosewheel_steer_ratio (Zibo README 3.26g). Over the Web API a
// write of that ratio read back 0 at once with the option at ON: zibomod
// writes it from the tiller axis every frame. Which input actually turns the
// nose had to be found in the sim, and a plugin reload is the expensive step,
// so every candidate is here behind xcopilot/tiller_method and one reload
// tests them all:
//   1 ZIBO_VR       option held at VR while applied, the ratio written
//   2 ZIBO_YAW      option held at YAW while applied, the value ridden on the
//                   pedal channel (tiller::yaw_override, one writer)
//   3 ZIBO_OVERRIDE laminar/B738/nosewheel_steer_override = 1 and the ratio
//   4 STOCK         override_wheel_steer = 1 and tire_steer_command_deg[0] =
//                   value x xcopilot/tiller_max_deg
// The option is the PILOT's preference: whatever it read at engage is put
// back at release. And the writes go out in BOTH flight-loop phases by
// default (xcopilot/tiller_phases, bit 1 before the flight model, bit 2 after),
// because zibomod's place in the frame is unknown: an input written after its
// read is a frame late, one written before its reset is gone.
// xcopilot/tiller_echo is the target as found at the top of our before-phase
// write (did anybody move it since we last wrote?), xcopilot/tiller_applied
// the method actually held (0 = nothing).

#include "tiller.h"

#include <cmath>
#include <initializer_list>

#include "XPLMDataAccess.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"

#include "drefs.h"

namespace tiller {
namespace {

enum Method { kNone = 0, kZiboVr = 1, kZiboYaw = 2, kZiboOverride = 3, kStock = 4 };

// Same freshness as every heartbeat channel.
const float kFreshSecs = 1.0f;
// joy_use_tiller in sim/joystick/joystick_axis_assignments (the order of the
// joy_use_* names in the X-Plane 12 binary: none 0, ptch 1, roll 2, hdng 3,
// ... gear 36, tiller 37; the brake and yaw codes drefs.cpp uses agree).
const int kAxisTiller = 37;
const float kTillerYieldDelta = 0.25f;
const int kArmFrames = 30;
const int kHoldFrames = 4;
const float kZiboVrOption = 3.0f;
const float kZiboYawOption = 1.0f;

XPLMDataRef d_tiller = nullptr;
XPLMDataRef d_method = nullptr;
XPLMDataRef d_max_deg = nullptr;
XPLMDataRef d_phases = nullptr;
XPLMDataRef d_echo = nullptr;
XPLMDataRef d_applied = nullptr;
XPLMFlightLoopID loop_before = nullptr;
XPLMFlightLoopID loop_after = nullptr;

float g_value = 0.0f;
float g_stamp = -1.0f;
int g_method = kNone;
float g_max_deg = 0.0f;
int g_phases = 3;
float g_echo = 0.0f;

// What is held right now (kNone = nothing) and what to put back.
int g_applied = kNone;
XPLMDataRef r_option = nullptr;
XPLMDataRef r_ratio = nullptr;
XPLMDataRef r_override = nullptr;
XPLMDataRef r_wheel_override = nullptr;
XPLMDataRef r_tire_cmd = nullptr;
float g_option_saved = 0.0f;
bool g_option_saved_valid = false;
int g_wheel_override_saved = 0;

// The pilot's own tiller, when one is assigned: the same baseline-and-hold
// yield as the pedals.
XPLMDataRef r_axis_values = nullptr;
XPLMDataRef r_axis_assign = nullptr;
int g_axis = -1;
float g_axis_base = 0.0f;
int g_engage_frames = 0;
int g_deflect_frames = 0;
bool g_yield = false;

bool good(XPLMDataRef r) { return r && XPLMIsDataRefGood(r); }

float axis_value() {
    if (g_axis < 0 || !r_axis_values) return -1.0f;
    float v = 0.0f;
    XPLMGetDatavf(r_axis_values, &v, g_axis, 1);
    return v;
}

// The airframe datarefs are looked up at each engage: they exist only while
// the airframe that registers them is loaded.
bool engage(int method) {
    r_option = r_ratio = r_override = r_wheel_override = r_tire_cmd = nullptr;
    g_option_saved_valid = false;
    switch (method) {
    case kZiboVr:
    case kZiboYaw:
        r_option = XPLMFindDataRef("laminar/B738/effects/nosewheel");
        if (!good(r_option)) return false;
        g_option_saved = XPLMGetDataf(r_option);
        g_option_saved_valid = true;
        if (method == kZiboVr) {
            r_ratio = XPLMFindDataRef("laminar/B738/nosewheel_steer_ratio");
            if (!good(r_ratio)) return false;
        }
        break;
    case kZiboOverride:
        r_override = XPLMFindDataRef("laminar/B738/nosewheel_steer_override");
        r_ratio = XPLMFindDataRef("laminar/B738/nosewheel_steer_ratio");
        if (!good(r_override) || !good(r_ratio)) return false;
        break;
    case kStock:
        r_wheel_override = XPLMFindDataRef("sim/operation/override/override_wheel_steer");
        r_tire_cmd = XPLMFindDataRef("sim/flightmodel2/gear/tire_steer_command_deg");
        if (!good(r_wheel_override) || !good(r_tire_cmd) || !(g_max_deg > 0.0f)) return false;
        g_wheel_override_saved = XPLMGetDatai(r_wheel_override);
        break;
    default:
        return false;
    }
    g_axis = -1;
    if (r_axis_assign) {
        int assign[500];
        const int n = XPLMGetDatavi(r_axis_assign, assign, 0, 500);
        for (int i = 0; i < n; ++i) {
            if (assign[i] == kAxisTiller) { g_axis = i; break; }
        }
    }
    g_engage_frames = 0;
    g_deflect_frames = 0;
    g_applied = method;
    return true;
}

// Everything held is let go, the airframe's own state put back. Writes are
// guarded: an airframe swapped out from under us leaves its refs dead.
void release() {
    switch (g_applied) {
    case kZiboVr:
        if (good(r_ratio)) XPLMSetDataf(r_ratio, 0.0f);
        if (good(r_option) && g_option_saved_valid) XPLMSetDataf(r_option, g_option_saved);
        break;
    case kZiboYaw:
        if (good(r_option) && g_option_saved_valid) XPLMSetDataf(r_option, g_option_saved);
        break;
    case kZiboOverride:
        if (good(r_ratio)) XPLMSetDataf(r_ratio, 0.0f);
        if (good(r_override)) XPLMSetDataf(r_override, 0.0f);
        break;
    case kStock: {
        if (good(r_tire_cmd)) {
            float zero = 0.0f;
            XPLMSetDatavf(r_tire_cmd, &zero, 0, 1);
        }
        if (good(r_wheel_override)) XPLMSetDatai(r_wheel_override, g_wheel_override_saved);
        break;
    }
    default:
        break;
    }
    g_applied = kNone;
    g_option_saved_valid = false;
}

void write(bool before) {
    switch (g_applied) {
    case kZiboVr:
        if (before && good(r_ratio)) g_echo = XPLMGetDataf(r_ratio);
        if (good(r_option) && XPLMGetDataf(r_option) != kZiboVrOption) {
            XPLMSetDataf(r_option, kZiboVrOption);
        }
        if (good(r_ratio)) XPLMSetDataf(r_ratio, g_value);
        break;
    case kZiboYaw:
        if (good(r_option) && XPLMGetDataf(r_option) != kZiboYawOption) {
            XPLMSetDataf(r_option, kZiboYawOption);
        }
        break;
    case kZiboOverride:
        if (before && good(r_ratio)) g_echo = XPLMGetDataf(r_ratio);
        if (good(r_override)) XPLMSetDataf(r_override, 1.0f);
        if (good(r_ratio)) XPLMSetDataf(r_ratio, g_value);
        break;
    case kStock: {
        if (before && good(r_tire_cmd)) XPLMGetDatavf(r_tire_cmd, &g_echo, 0, 1);
        if (good(r_wheel_override)) XPLMSetDatai(r_wheel_override, 1);
        float deg = g_value * g_max_deg;
        if (good(r_tire_cmd)) XPLMSetDatavf(r_tire_cmd, &deg, 0, 1);
        break;
    }
    default:
        break;
    }
}

void tick(bool before) {
    const bool fresh = g_stamp >= 0.0f && XPLMGetElapsedTime() - g_stamp <= kFreshSecs;
    // The pilot's feet on the pedals are the pilot steering: the tiller goes
    // with the pedals' yield.
    if (g_applied != kNone && drefs::steer_yielded()) g_yield = true;
    if (!fresh || g_yield || g_method == kNone || g_applied != g_method) {
        if (g_applied != kNone) release();
    }
    if (!fresh) {
        g_yield = false;  // writes stopped: re-arm for next time
        return;
    }
    if (g_yield || g_method == kNone) return;
    if (g_applied == kNone && !engage(g_method)) return;
    const float ax = axis_value();
    if (g_engage_frames < kArmFrames) {
        ++g_engage_frames;
        g_axis_base = ax;
    } else if (g_axis >= 0 && ax >= 0.0f && std::fabs(ax - g_axis_base) > kTillerYieldDelta) {
        if (++g_deflect_frames >= kHoldFrames) {
            // The pilot's hand on the tiller: let go THIS frame, and latch
            // the steer yield the app already polls so the drive stands down.
            g_yield = true;
            drefs::yield_steering();
            release();
            return;
        }
    } else {
        g_deflect_frames = 0;
    }
    if (g_phases & (before ? 1 : 2)) write(before);
}

float LoopBefore(float, float, int, void*) {
    tick(true);
    return -1.0f;
}
float LoopAfter(float, float, int, void*) {
    tick(false);
    return -1.0f;
}

float ReadTiller(void*) { return g_value; }
void WriteTiller(void*, float v) {
    g_value = v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v);
    g_stamp = XPLMGetElapsedTime();
}
int ReadMethod(void*) { return g_method; }
void WriteMethod(void*, int v) { g_method = (v >= kNone && v <= kStock) ? v : kNone; }
float ReadMaxDeg(void*) { return g_max_deg; }
void WriteMaxDeg(void*, float v) { g_max_deg = v > 0.0f && v <= 120.0f ? v : 0.0f; }
int ReadPhases(void*) { return g_phases; }
void WritePhases(void*, int v) { g_phases = (v >= 1 && v <= 3) ? v : 3; }
float ReadEcho(void*) { return g_echo; }
int ReadApplied(void*) { return g_applied; }

XPLMFlightLoopID make_loop(XPLMFlightLoopPhaseType phase, XPLMFlightLoop_f f) {
    XPLMCreateFlightLoop_t p;
    p.structSize = sizeof p;
    p.phase = phase;
    p.callbackFunc = f;
    p.refcon = nullptr;
    XPLMFlightLoopID id = XPLMCreateFlightLoop(&p);
    XPLMScheduleFlightLoop(id, -1.0f, 1);
    return id;
}

}  // namespace

bool yaw_override(float* v) {
    if (g_applied != kZiboYaw || g_yield) return false;
    // A tiller at 0 (a release, or the app's last write before letting go)
    // never blanks the pedal command on the yaw axis: at speed it is the
    // rudder (LEMG 30 rollout, 2026-10-07: 0.65 asked, 0 sent, 94 m off).
    if (g_value == 0.0f) return false;
    *v = g_value;
    return true;
}

void start() {
    d_tiller = XPLMRegisterDataAccessor(
        "xcopilot/tiller", xplmType_Float, 1,
        nullptr, nullptr, ReadTiller, WriteTiller, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    d_method = XPLMRegisterDataAccessor(
        "xcopilot/tiller_method", xplmType_Int, 1,
        ReadMethod, WriteMethod, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    d_max_deg = XPLMRegisterDataAccessor(
        "xcopilot/tiller_max_deg", xplmType_Float, 1,
        nullptr, nullptr, ReadMaxDeg, WriteMaxDeg, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    d_phases = XPLMRegisterDataAccessor(
        "xcopilot/tiller_phases", xplmType_Int, 1,
        ReadPhases, WritePhases, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    d_echo = XPLMRegisterDataAccessor(
        "xcopilot/tiller_echo", xplmType_Float, 0,
        nullptr, nullptr, ReadEcho, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    d_applied = XPLMRegisterDataAccessor(
        "xcopilot/tiller_applied", xplmType_Int, 0,
        ReadApplied, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    r_axis_values = XPLMFindDataRef("sim/joystick/joystick_axis_values");
    r_axis_assign = XPLMFindDataRef("sim/joystick/joystick_axis_assignments");
    loop_before = make_loop(xplm_FlightLoop_Phase_BeforeFlightModel, LoopBefore);
    loop_after = make_loop(xplm_FlightLoop_Phase_AfterFlightModel, LoopAfter);
}

void stop() {
    if (loop_before) { XPLMDestroyFlightLoop(loop_before); loop_before = nullptr; }
    if (loop_after) { XPLMDestroyFlightLoop(loop_after); loop_after = nullptr; }
    if (g_applied != kNone) release();
    g_stamp = -1.0f;
    for (XPLMDataRef* d : {&d_tiller, &d_method, &d_max_deg, &d_phases, &d_echo, &d_applied}) {
        if (*d) { XPLMUnregisterDataAccessor(*d); *d = nullptr; }
    }
}

}  // namespace tiller
