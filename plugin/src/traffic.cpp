// ---- Our AI traffic, drawn through XPMP2 -----------------------------------
// The app's fleet (crates/traffic, the bin's traffic_link) is the truth; this
// module only DRAWS it. The wire (traffic_wire.h):
//   xcopilot/traffic/roster  data rw  who exists (id, type, airline, livery,
//                                      callsign, mode S, squawk), on change
//   xcopilot/traffic/frames  data rw  where they are, 4 Hz, sim-time stamped
//   xcopilot/traffic/enable  int  rw  1 draws and takes the TCAS slots, 0
//                                      destroys every aircraft and frees them
//   xcopilot/traffic/labels  int  rw  1 shows the callsign labels
//   xcopilot/traffic/status  data r   "models=N|tcas=OWNER|drawn=N|
//                                      roster_seq=N|frame_seq=N|errors=N|
//                                      last_error=KIND|create_errors=N|
//                                      csl=ok|enabled=0/1"
//
// Setters only copy the bytes and raise a flag (CLAUDE.md: plugin work never
// runs in a Web API set handler); the flight loop (BeforeFlightModel, every
// frame) decodes, creates and destroys the aircraft, and moves the render
// clock. Nothing runs while the sim is loading (flightinit::loading()); a
// paused sim runs no flight loop, so everything freezes with it.
//
// Rendering: each id keeps its last two frames and is drawn ONE FRAME BEHIND
// on the app's sim-time stamps (sim/time/total_running_time_sec on both
// sides): Hermite position (tangents from groundspeed along the heading,
// reversed when the aircraft backs, as on a push), shortest-way heading,
// linear pitch, roll and ratios. Past the newest frame it extrapolates at
// most 1.5 s, then freezes. An aircraft on the ground sits on the mesh
// (XPLMProbeTerrainXYZ), not the app's apt.dat elevation; XPMP2's
// clamp_all_to_ground keeps the rest above it.
//
// CSL: XPMP2 needs a resource folder (Doc8643.txt, related.txt, relOp.txt,
// MapIcons.png, Obj8DataRefs.txt) and CSL packages, both user-installed and
// read-only for us: XCOPILOT_TRAFFIC_CSL (in X-Plane's environment) names the
// CSL folder, else LiveTraffic's Resources/CSL, in plugins/LiveTraffic or, for a
// LiveTraffic set aside, plugins/Disabled/LiveTraffic; the resources are its
// parent folder (LiveTraffic's layout). Without them the status says "no CSL"
// and nothing draws, and switching traffic on tries again (models installed
// after the sim started need no restart).

#include "traffic.h"

#include <sys/stat.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <map>
#include <memory>
#include <string>

#include "XPLMDataAccess.h"
#include "XPLMPlanes.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMScenery.h"
#include "XPLMUtilities.h"

#include "XPMPAircraft.h"
#include "XPMPMultiplayer.h"

#include "common.h"
#include "drefs.h"
#include "flightinit.h"
#include "traffic_wire.h"

namespace {

using traffic_wire::FrameEntry;
using traffic_wire::RosterEntry;

// ---- the mailboxes ----------------------------------------------------------

struct WireBuf {
    uint8_t buf[traffic_wire::kBufBytes];
    int len;
};

WireBuf g_roster_in = {{0}, 0};
WireBuf g_frames_in = {{0}, 0};
bool g_roster_dirty = false;
bool g_frames_dirty = false;
int g_enable_req = 0;
int g_labels_req = 0;
TextRef g_status = {{0}, 0};

XPLMDataRef g_roster_dref = nullptr;
XPLMDataRef g_frames_dref = nullptr;
XPLMDataRef g_enable_dref = nullptr;
XPLMDataRef g_labels_dref = nullptr;
XPLMDataRef g_status_dref = nullptr;

XPLMDataRef g_sim_time = nullptr;  // sim/time/total_running_time_sec
XPLMDataRef g_paused = nullptr;    // sim/time/paused
XPLMFlightLoopID g_loop = nullptr;

// ---- state the flight loop owns ---------------------------------------------

bool g_inited = false;        // XPMPMultiplayerInit succeeded
std::string g_csl_state = "not started";
int g_models = 0;
bool g_enabled = false;       // applied value of g_enable_req
bool g_labels = false;        // applied value of g_labels_req
bool g_tcas_retry = false;    // XPLMPlanesAvailable_f fired: try again
std::string g_tcas_owner = "off";
double g_owner_checked_at = -1.0;
uint32_t g_roster_seq = 0;
uint32_t g_frame_seq = 0;
int g_decode_errors = 0;
int g_create_errors = 0;
const char* g_last_error = "none";
double g_status_at = -1.0;

// The render clock: local sim time minus the transport offset (smoothed)
// minus one frame interval (smoothed), so the clock sits one frame behind
// the newest stamp.
bool g_have_offset = false;
double g_offset_s = 0.0;
double g_interval_s = 0.25;
double g_last_stamp = -1.0;
double g_render_t = 0.0;

constexpr double kExtrapolateMaxS = 1.5;
constexpr double kKtToMs = 0.514444;
constexpr double kEarthR = 6371008.8;
constexpr double kDeg = M_PI / 180.0;
constexpr float kWheelRadiusM = 0.35f;

struct Sample {
    double t = 0.0;
    FrameEntry f;
};

class TrafficAc;

struct Track {
    RosterEntry meta;
    Sample a, b;       // a older, b newer
    int samples = 0;   // 0, 1 or 2 held
    double dir = 1.0;  // +1 moving along the heading, -1 backing (a push)
    std::unique_ptr<TrafficAc> ac;
    bool failed = false;  // creation threw; retried on the next roster change
    bool seen = false;    // present in the newest roster
};

// std::map: node addresses are stable, so an aircraft can hold its Track*.
std::map<uint32_t, Track> g_tracks;

// ---- interpolation ----------------------------------------------------------

struct Pose {
    double lat, lon, alt_ft;
    float hdg, pitch, roll, gs_kts;
    float gear, flaps, spoiler, reverser;
    uint8_t lights, flags;
};

double wrap180(double d) {
    while (d > 180.0) d -= 360.0;
    while (d < -180.0) d += 360.0;
    return d;
}

Pose pose_of(const FrameEntry& f) {
    return Pose{f.lat, f.lon, f.alt_ft, f.hdg_deg, f.pitch_deg, f.roll_deg, f.gs_kts,
                f.gear, f.flaps, f.spoiler, f.reverser, f.lights, f.flags};
}

// Ground velocity (east, north) in m/s along the heading, signed by `dir`.
void velocity(const FrameEntry& f, double dir, double& ve, double& vn) {
    const double v = dir * f.gs_kts * kKtToMs;
    ve = v * std::sin(f.hdg_deg * kDeg);
    vn = v * std::cos(f.hdg_deg * kDeg);
}

// The pose at render time `t`; false when the track has no frame yet.
bool pose_at(const Track& tr, double t, Pose& out) {
    if (tr.samples == 0) return false;
    if (tr.samples == 1 || t <= tr.a.t) {
        out = pose_of(tr.samples == 1 ? tr.b.f : tr.a.f);
        return true;
    }
    const FrameEntry& fa = tr.a.f;
    const FrameEntry& fb = tr.b.f;
    const double m_lat = kEarthR * kDeg;
    const double m_lon = m_lat * std::cos(fa.lat * kDeg);
    const bool both_ground = (fa.flags & traffic_wire::kFlagOnGround) &&
                             (fb.flags & traffic_wire::kFlagOnGround);

    if (t > tr.b.t) {
        // Past the newest frame: straight on, at most kExtrapolateMaxS, then frozen.
        const double dt = std::fmin(t - tr.b.t, kExtrapolateMaxS);
        double ve, vn;
        velocity(fb, tr.dir, ve, vn);
        out = pose_of(fb);
        out.lat = fb.lat + vn * dt / m_lat;
        out.lon = fb.lon + (m_lon > 1.0 ? ve * dt / m_lon : 0.0);
        if (!(fb.flags & traffic_wire::kFlagOnGround)) out.alt_ft = fb.alt_ft + fb.vs_fpm / 60.0 * dt;
        return true;
    }

    const double T = tr.b.t - tr.a.t;
    const double s = (t - tr.a.t) / T;
    const double s2 = s * s, s3 = s2 * s;
    const double h00 = 2 * s3 - 3 * s2 + 1, h10 = s3 - 2 * s2 + s;
    const double h01 = -2 * s3 + 3 * s2, h11 = s3 - s2;

    // Chord from a to b in metres (east, north).
    const double ce = m_lon > 1.0 ? wrap180(fb.lon - fa.lon) * m_lon : 0.0;
    const double cn = (fb.lat - fa.lat) * m_lat;
    double ve0, vn0, ve1, vn1;
    velocity(fa, tr.dir, ve0, vn0);
    velocity(fb, tr.dir, ve1, vn1);
    const double chord = std::hypot(ce, cn);
    const double reach = std::fmax(fa.gs_kts, fb.gs_kts) * kKtToMs * T;
    double e, n;
    if (chord > 2.0 * reach + 30.0) {
        // A jump the speeds cannot explain (a respawn, a reposition): linear.
        e = s * ce;
        n = s * cn;
    } else {
        e = h10 * T * ve0 + h01 * ce + h11 * T * ve1;
        n = h10 * T * vn0 + h01 * cn + h11 * T * vn1;
    }

    out.lat = fa.lat + n / m_lat;
    out.lon = fa.lon + (m_lon > 1.0 ? e / m_lon : 0.0);
    if (both_ground) {
        out.alt_ft = fa.alt_ft + s * (fb.alt_ft - fa.alt_ft);
    } else {
        out.alt_ft = h00 * fa.alt_ft + h10 * T * fa.vs_fpm / 60.0 + h01 * fb.alt_ft +
                     h11 * T * fb.vs_fpm / 60.0;
    }
    double hdg = fa.hdg_deg + s * wrap180(fb.hdg_deg - fa.hdg_deg);
    if (hdg < 0.0) hdg += 360.0;
    if (hdg >= 360.0) hdg -= 360.0;
    const float sf = static_cast<float>(s);
    auto lerp = [sf](float x, float y) { return x + sf * (y - x); };
    out.hdg = static_cast<float>(hdg);
    out.pitch = lerp(fa.pitch_deg, fb.pitch_deg);
    out.roll = lerp(fa.roll_deg, fb.roll_deg);
    out.gs_kts = lerp(fa.gs_kts, fb.gs_kts);
    out.gear = lerp(fa.gear, fb.gear);
    out.flaps = lerp(fa.flaps, fb.flaps);
    out.spoiler = lerp(fa.spoiler, fb.spoiler);
    out.reverser = lerp(fa.reverser, fb.reverser);
    out.lights = s < 0.5 ? fa.lights : fb.lights;
    out.flags = s < 0.5 ? fa.flags : fb.flags;
    return true;
}

// ---- the aircraft -------------------------------------------------------------

class TrafficAc : public XPMP2::Aircraft {
public:
    explicit TrafficAc(Track* tr)
        : XPMP2::Aircraft(tr->meta.type, tr->meta.airline, tr->meta.livery,
                          tr->meta.mode_s & 0xFFFFFFu, "", tr->meta.callsign),
          track_(tr) {
        label = tr->meta.callsign;
        acRadar.code = tr->meta.squawk;
        acRadar.mode = xpmpTransponderMode_ModeC;
        SetVisible(false);  // until its first frame
    }
    ~TrafficAc() override {
        if (probe_) XPLMDestroyProbe(probe_);
    }

    void UpdatePosition(float elapsed, int) override {
        if (flightinit::loading()) return;  // hold the last pose through a load
        Pose p;
        if (!pose_at(*track_, g_render_t, p) || (p.flags & traffic_wire::kFlagHidden)) {
            if (IsVisible()) SetVisible(false);
            return;
        }
        if (!IsVisible()) SetVisible(true);

        const bool on_ground = (p.flags & traffic_wire::kFlagOnGround) != 0;
        SetLocation(p.lat, p.lon, p.alt_ft, on_ground);
        if (on_ground) sit_on_mesh();
        SetHeading(p.hdg);
        SetPitch(p.pitch);
        SetRoll(p.roll);

        SetGearRatio(p.gear);
        SetFlapRatio(p.flaps);
        SetSlatRatio(p.flaps);
        SetSpoilerRatio(p.spoiler);
        SetSpeedbrakeRatio(p.spoiler);
        SetReversDeployRatio(p.reverser);
        SetThrustReversRatio(p.reverser);
        SetLightsNav(p.lights & traffic_wire::kLightNav);
        SetLightsBeacon(p.lights & traffic_wire::kLightBeacon);
        SetLightsStrobe(p.lights & traffic_wire::kLightStrobe);
        SetLightsLanding(p.lights & traffic_wire::kLightLanding);
        SetLightsTaxi(p.lights & traffic_wire::kLightTaxi);

        // Wheels turn with the groundspeed on the ground; engines idle on the
        // ground, rising with the speed, and run at cruise in the air.
        const float ms = p.gs_kts * static_cast<float>(kKtToMs);
        const float tyre_rpm = on_ground ? ms / (2.0f * static_cast<float>(M_PI) * kWheelRadiusM) * 60.0f : 0.0f;
        float eng_rpm = 0.0f;
        if (p.flags & traffic_wire::kFlagEngines) {
            eng_rpm = on_ground ? 1000.0f + 15.0f * p.gs_kts : 2300.0f;
            if (eng_rpm > 2300.0f) eng_rpm = 2300.0f;
        }
        SetThrustRatio(on_ground ? std::fmin(1.0f, p.gs_kts / 140.0f) : 0.7f);
        SetTireRotRpm(tyre_rpm);
        SetEngineRotRpm(eng_rpm);
        SetPropRotRpm(eng_rpm);
        const float dt = elapsed > 0.0f && elapsed < 1.0f ? elapsed : 0.0f;
        tyre_deg_ = std::fmod(tyre_deg_ + tyre_rpm * 6.0f * dt, 360.0f);
        eng_deg_ = std::fmod(eng_deg_ + eng_rpm * 6.0f * dt, 360.0f);
        SetTireRotAngle(tyre_deg_);
        SetEngineRotAngle(eng_deg_);
        SetPropRotAngle(eng_deg_);
    }

private:
    // On the ground the aircraft sits on the mesh, not on the app's apt.dat
    // elevation (they differ by metres at some fields).
    void sit_on_mesh() {
        if (!probe_) probe_ = XPLMCreateProbe(xplm_ProbeY);
        if (!probe_) return;
        XPLMProbeInfo_t info;
        std::memset(&info, 0, sizeof info);
        info.structSize = sizeof info;
        if (XPLMProbeTerrainXYZ(probe_, drawInfo.x, drawInfo.y, drawInfo.z, &info) ==
            xplm_ProbeHitTerrain) {
            drawInfo.y = info.locationY + GetVertOfs();
        }
    }

    Track* track_;
    XPLMProbeRef probe_ = nullptr;
    float tyre_deg_ = 0.0f;
    float eng_deg_ = 0.0f;
};

// ---- XPMP2 configuration -----------------------------------------------------

// The CSL folders are the pilot's and read-only for us: XPMP2 writes no
// .xpmp2.obj copies (replace_texture and replace_datarefs off), so a CSL that
// names a livery texture shows its base texture. No network (remote off), no
// sound in v1.
int Prefs(const char* section, const char* key, int dflt) {
    (void)section;
    if (!std::strcmp(key, XPMP_CFG_ITM_REPLDATAREFS)) return 0;
    if (!std::strcmp(key, XPMP_CFG_ITM_REPLTEXTURE)) return 0;
    if (!std::strcmp(key, XPMP_CFG_ITM_CLAMPALL)) return 1;
    if (!std::strcmp(key, XPMP_CFG_ITM_HANDLE_DUP_ID)) return 1;
    if (!std::strcmp(key, XPMP_CFG_ITM_SUPPORT_REMOTE)) return -1;
    if (!std::strcmp(key, XPMP_CFG_ITM_ACTIVATE_SOUND)) return 0;
    if (!std::strcmp(key, XPMP_CFG_ITM_LOGLEVEL)) return 2;
    if (!std::strcmp(key, XPMP_CFG_ITM_MODELMATCHING)) return 0;
    return dflt;
}

bool is_dir(const std::string& p) {
    struct stat st;
    return stat(p.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

bool is_file(const std::string& p) {
    struct stat st;
    return stat(p.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

std::string parent_of(std::string p) {
    while (!p.empty() && p.back() == '/') p.pop_back();
    const size_t slash = p.rfind('/');
    return slash == std::string::npos ? std::string() : p.substr(0, slash);
}

void log(const std::string& line) {
    XPLMDebugString(("XVoiceLayer plugin: traffic: " + line + "\n").c_str());
}

void init_xpmp2() {
    char sys[1024] = {0};
    XPLMGetSystemPath(sys);
    std::string root(sys);
    if (!root.empty() && root.back() != '/') root += '/';
    std::string lt_res = root + "Resources/plugins/LiveTraffic/Resources";
    if (!is_dir(lt_res)) {
        const std::string aside = root + "Resources/plugins/Disabled/LiveTraffic/Resources";
        if (is_dir(aside)) lt_res = aside;
    }
    const char* env = std::getenv("XCOPILOT_TRAFFIC_CSL");
    const std::string csl = env && *env ? std::string(env) : lt_res + "/CSL";
    std::string res = parent_of(csl);
    if (!is_file(res + "/Doc8643.txt")) res = lt_res;

    if (!is_dir(csl) || !is_file(res + "/Doc8643.txt")) {
        g_csl_state = "no CSL";
        log("no CSL (looked for " + csl + " and Doc8643.txt in " + res + "); nothing will draw");
        return;
    }
    const char* err = XPMPMultiplayerInit("XVoiceLayer", res.c_str(), Prefs, "C172", "XVL");
    if (err && *err) {
        g_csl_state = std::string("init failed: ") + err;
        log(g_csl_state);
        XPMPMultiplayerCleanup();
        return;
    }
    g_inited = true;
    const double t0 = XPLMGetElapsedTime();
    err = XPMPLoadCSLPackage(csl.c_str());
    g_models = XPMPGetNumberOfInstalledModels();
    if (g_models <= 0) {
        g_csl_state = "no CSL";
    } else {
        g_csl_state = "ok";
    }
    XPMPEnableAircraftLabels(false);
    char msg[1400];
    std::snprintf(msg, sizeof msg, "%d CSL models from %s (resources %s) in %.1f s%s%s", g_models,
                  csl.c_str(), res.c_str(), XPLMGetElapsedTime() - t0,
                  err && *err ? ", warning: " : "", err && *err ? err : "");
    log(msg);
}

// ---- TCAS ------------------------------------------------------------------------

void PlanesAvailable(void*) { g_tcas_retry = true; }

std::string controller_name() {
    if (XPMPHasControlOfAIAircraft()) return "ours";
    int total = 0, active = 0;
    XPLMPluginID who = XPLM_NO_PLUGIN_ID;
    XPLMCountAircraft(&total, &active, &who);
    if (who == XPLM_NO_PLUGIN_ID) return "none";
    char name[256] = {0};
    XPLMGetPluginInfo(who, name, nullptr, nullptr, nullptr);
    return name[0] ? std::string(name) : "plugin " + std::to_string(who);
}

void take_tcas() {
    const char* err = XPMPMultiplayerEnable(PlanesAvailable, nullptr);
    g_tcas_owner = controller_name();
    if (err && *err) log(std::string("TCAS slots held by ") + g_tcas_owner + " (" + err + ")");
    else log("TCAS slots ours");
}

// ---- the flight loop ---------------------------------------------------------------

bool want_aircraft() { return g_inited && g_enabled && g_models > 0; }

void reconcile() {
    for (auto it = g_tracks.begin(); it != g_tracks.end();) {
        Track& tr = it->second;
        if (!tr.seen) {
            it = g_tracks.erase(it);  // destroys its aircraft
            continue;
        }
        if (!want_aircraft()) {
            tr.ac.reset();
        } else if (!tr.ac && !tr.failed) {
            try {
                tr.ac = std::make_unique<TrafficAc>(&tr);
            } catch (const std::exception& e) {
                tr.failed = true;
                ++g_create_errors;
                log("could not create " + tr.meta.callsign + " (" + tr.meta.type + "): " + e.what());
            }
        }
        ++it;
    }
}

void apply_roster() {
    traffic_wire::Roster r;
    const auto e = traffic_wire::decode_roster(g_roster_in.buf, static_cast<size_t>(g_roster_in.len), r);
    if (e != traffic_wire::Error::None) {
        ++g_decode_errors;
        g_last_error = traffic_wire::error_name(e);
        return;
    }
    g_roster_seq = r.seq;
    for (auto& kv : g_tracks) kv.second.seen = false;
    for (const RosterEntry& re : r.entries) {
        Track& tr = g_tracks[re.id];
        const bool changed = tr.meta.type != re.type || tr.meta.airline != re.airline ||
                             tr.meta.livery != re.livery || tr.meta.callsign != re.callsign ||
                             tr.meta.mode_s != re.mode_s;
        if (changed) {
            tr.ac.reset();  // re-matched with the new identity
        } else if (tr.ac) {
            tr.ac->acRadar.code = re.squawk;
        }
        tr.meta = re;
        tr.failed = false;  // a roster change is the retry
        tr.seen = true;
    }
    reconcile();
}

void apply_frames(double now) {
    traffic_wire::Frames f;
    const auto e = traffic_wire::decode_frames(g_frames_in.buf, static_cast<size_t>(g_frames_in.len), f);
    if (e != traffic_wire::Error::None) {
        ++g_decode_errors;
        g_last_error = traffic_wire::error_name(e);
        return;
    }
    g_frame_seq = f.seq;
    const double stamp = f.sim_time_s;
    // The transport offset, smoothed; snapped on a jump (a new flight resets
    // the sim's running time).
    const double off = now - stamp;
    if (!g_have_offset || std::fabs(off - g_offset_s) > 2.0) {
        g_offset_s = off;
        g_have_offset = true;
    } else {
        g_offset_s += 0.1 * (off - g_offset_s);
    }
    if (g_last_stamp >= 0.0) {
        const double d = stamp - g_last_stamp;
        if (d > 0.05 && d < 2.0) g_interval_s += 0.2 * (d - g_interval_s);
    }
    g_last_stamp = stamp;

    for (const FrameEntry& fe : f.entries) {
        auto it = g_tracks.find(fe.id);
        if (it == g_tracks.end()) continue;  // not on the roster (yet)
        Track& tr = it->second;
        if (tr.samples > 0 && stamp <= tr.b.t) {
            if (stamp < tr.b.t - 2.0) {
                tr.samples = 0;  // the clock went back: start over
            } else {
                continue;  // a repeat or an old frame
            }
        }
        if (tr.samples == 0) {
            tr.a = tr.b = Sample{stamp, fe};
            tr.samples = 1;
            continue;
        }
        tr.a = tr.b;
        tr.b = Sample{stamp, fe};
        tr.samples = 2;
        // Backing (a push) when the chord runs against the heading.
        const double ce = wrap180(tr.b.f.lon - tr.a.f.lon) * std::cos(tr.a.f.lat * kDeg);
        const double cn = tr.b.f.lat - tr.a.f.lat;
        const double he = std::sin(tr.a.f.hdg_deg * kDeg), hn = std::cos(tr.a.f.hdg_deg * kDeg);
        if (std::hypot(ce, cn) * kEarthR * kDeg > 0.3) tr.dir = (ce * he + cn * hn) < 0.0 ? -1.0 : 1.0;
    }
}

void publish_status(double wall) {
    if (g_enabled && g_inited && wall - g_owner_checked_at > 2.0) {
        g_owner_checked_at = wall;
        g_tcas_owner = controller_name();
    }
    int drawn = 0;
    for (const auto& kv : g_tracks) {
        if (kv.second.ac && kv.second.ac->IsVisible()) ++drawn;
    }
    char s[512];
    const int n = std::snprintf(
        s, sizeof s,
        "models=%d|tcas=%s|drawn=%d|roster_seq=%u|frame_seq=%u|errors=%d|last_error=%s|"
        "create_errors=%d|csl=%s|enabled=%d",
        g_models, g_enabled ? g_tcas_owner.c_str() : "off", drawn, g_roster_seq, g_frame_seq,
        g_decode_errors, g_last_error, g_create_errors, g_csl_state.c_str(), g_enabled ? 1 : 0);
    const int len = n < 0 ? 0 : (n < static_cast<int>(sizeof s) ? n : static_cast<int>(sizeof s) - 1);
    std::memset(g_status.buf, 0, sizeof g_status.buf);
    std::memcpy(g_status.buf, s, static_cast<size_t>(len));
    g_status.len = len;
}

float TrafficLoop(float, float, int, void*) {
    const double wall = XPLMGetElapsedTime();
    if (wall - g_status_at > 0.25) {
        g_status_at = wall;
        publish_status(wall);
    }
    if (flightinit::loading()) return -1.0f;  // nothing asks a loading sim
    if (g_paused && XPLMGetDatai(g_paused)) return -1.0f;
    const double now = g_sim_time ? XPLMGetDataf(g_sim_time) : wall;

    const bool want_enable = g_enable_req != 0;
    if (want_enable != g_enabled) {
        g_enabled = want_enable;
        // No CSL at start: look again now, so the models need no sim restart.
        if (g_enabled && !g_inited) init_xpmp2();
        if (g_inited) {
            if (g_enabled) {
                take_tcas();
            } else {
                XPMPMultiplayerDisable();
                g_tcas_owner = "off";
            }
        }
        reconcile();
    }
    if (g_enabled && g_inited && g_tcas_retry) {
        g_tcas_retry = false;
        take_tcas();
    }
    const bool want_labels = g_labels_req != 0;
    if (want_labels != g_labels) {
        g_labels = want_labels;
        if (g_inited) XPMPEnableAircraftLabels(g_labels);
    }
    if (g_roster_dirty) {
        g_roster_dirty = false;
        apply_roster();
    }
    if (g_frames_dirty) {
        g_frames_dirty = false;
        apply_frames(now);
    }
    if (g_have_offset) g_render_t = now - g_offset_s - g_interval_s;
    return -1.0f;
}

// ---- dataref accessors (copy and flag only) ----------------------------------------

int ReadWire(void* ref, void* out, int offset, int max) {
    const WireBuf* w = static_cast<const WireBuf*>(ref);
    if (out == nullptr) return w->len;
    if (offset < 0 || offset >= w->len || max <= 0) return 0;
    int n = w->len - offset;
    if (n > max) n = max;
    std::memcpy(out, w->buf + offset, static_cast<size_t>(n));
    return n;
}

// The app writes the whole buffer at offset 0; anything else is refused and
// counted, never half-applied.
void StoreWire(WireBuf* w, bool* dirty, const void* in, int offset, int len) {
    if (offset != 0 || len < 0 || len > static_cast<int>(sizeof w->buf) || (len > 0 && !in)) {
        ++g_decode_errors;
        g_last_error = offset != 0 ? "offset" : "oversize";
        return;
    }
    if (len > 0) std::memcpy(w->buf, in, static_cast<size_t>(len));
    w->len = len;
    *dirty = true;
}

void WriteRoster(void*, void* in, int offset, int len) {
    StoreWire(&g_roster_in, &g_roster_dirty, in, offset, len);
}
void WriteFrames(void*, void* in, int offset, int len) {
    StoreWire(&g_frames_in, &g_frames_dirty, in, offset, len);
}
int ReadEnable(void*) { return g_enable_req; }
void WriteEnable(void*, int v) { g_enable_req = v ? 1 : 0; }
int ReadLabels(void*) { return g_labels_req; }
void WriteLabels(void*, int v) { g_labels_req = v ? 1 : 0; }

XPLMDataRef register_data(const char* name, XPLMGetDatab_f rd, XPLMSetDatab_f wr, void* ref) {
    return XPLMRegisterDataAccessor(name, xplmType_Data, wr ? 1 : 0, nullptr, nullptr, nullptr,
                                    nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
                                    rd, wr, ref, ref);
}

XPLMDataRef register_int(const char* name, XPLMGetDatai_f rd, XPLMSetDatai_f wr) {
    return XPLMRegisterDataAccessor(name, xplmType_Int, 1, rd, wr, nullptr, nullptr, nullptr,
                                    nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
                                    nullptr, nullptr);
}

void unregister(XPLMDataRef& d) {
    if (d) XPLMUnregisterDataAccessor(d);
    d = nullptr;
}

} // namespace

namespace traffic {

void start() {
    g_roster_dref = register_data("xcopilot/traffic/roster", ReadWire, WriteRoster, &g_roster_in);
    g_frames_dref = register_data("xcopilot/traffic/frames", ReadWire, WriteFrames, &g_frames_in);
    g_enable_dref = register_int("xcopilot/traffic/enable", ReadEnable, WriteEnable);
    g_labels_dref = register_int("xcopilot/traffic/labels", ReadLabels, WriteLabels);
    g_status_dref = register_data("xcopilot/traffic/status", ReadText, nullptr, &g_status);
    g_sim_time = XPLMFindDataRef("sim/time/total_running_time_sec");
    g_paused = XPLMFindDataRef("sim/time/paused");

    init_xpmp2();
    publish_status(XPLMGetElapsedTime());

    XPLMCreateFlightLoop_t fl;
    std::memset(&fl, 0, sizeof fl);
    fl.structSize = sizeof fl;
    fl.phase = xplm_FlightLoop_Phase_BeforeFlightModel;
    fl.callbackFunc = TrafficLoop;
    fl.refcon = nullptr;
    g_loop = XPLMCreateFlightLoop(&fl);
    XPLMScheduleFlightLoop(g_loop, -1.0f, 1);
}

void stop() {
    if (g_loop) {
        XPLMDestroyFlightLoop(g_loop);
        g_loop = nullptr;
    }
    g_tracks.clear();  // every aircraft destroyed before XPMP2 goes
    if (g_inited) {
        XPMPMultiplayerCleanup();
        g_inited = false;
    }
    unregister(g_status_dref);
    unregister(g_labels_dref);
    unregister(g_enable_dref);
    unregister(g_frames_dref);
    unregister(g_roster_dref);
}

} // namespace traffic
