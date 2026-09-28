// ---- FMS route loading ------------------------------------------------------
// The app writes the SimBrief route into xcopilot/fms/route as
// "IDENT,lat,lon,alt|IDENT,lat,lon,alt|...", then writes 1 to xcopilot/fms/load
// to build it in the GPS/FMS (only the plugin can, via XPLMNavigation). In
// flight it writes "IDENT,lat,lon" to xcopilot/fms/goto to sequence the plan.

#include "fms.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "XPLMDataAccess.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"

#include "drefs.h"
#include "flightinit.h"

static TextRef g_fms_route = {{0}, 0};
static int g_fms_load = 0;
static TextRef g_fms_goto = {{0}, 0};
static XPLMDataRef g_fms_route_dref = nullptr;
static XPLMDataRef g_fms_load_dref = nullptr;
static XPLMDataRef g_fms_goto_dref = nullptr;

// Parse the app's payload into points, without navaid resolution — cheap enough
// to run on every route write; the Route tab renders straight from this.
static std::vector<RoutePoint> parse_route() {
    char buf[sizeof g_fms_route.buf];
    std::strncpy(buf, g_fms_route.buf, sizeof buf);
    buf[sizeof buf - 1] = '\0';

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
        pts.push_back(p);
    }
    return pts;
}

// Display copy for the panel's Route tab, refreshed whenever the app writes.
static std::vector<RoutePoint> g_route_pts;

// The follower's last goto named a leg the loaded plan lacks (an approach
// leg): the FMS destination is then stale and the goto itself is the truth.
static bool g_goto_off_plan = false;
static std::string g_goto_ident;
static double g_goto_lat = 0.0, g_goto_lon = 0.0;

static void WriteFmsRoute(void*, void* in, int offset, int len) {
    StoreText(&g_fms_route, in, offset, len);
    g_route_pts = parse_route();
}

// X-Plane's loaded AIRAC cycle (e.g. "2406"), read once from the nav-data header
// so the flight plans we generate carry the *matching* cycle — a mismatch pops
// X-Plane's "different AIRAC cycle" warning on every load. Empty if undetermined.
static std::string g_airac_cycle;

static void read_airac_cycle_impl() {
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
    g_goto_off_plan = false; // a new plan: the old flight's approach leg is history
    const XPLMNavType kTypes = xplm_Nav_Fix | xplm_Nav_VOR | xplm_Nav_NDB |
                               xplm_Nav_DME | xplm_Nav_Airport;

    // Parse the payload into points, resolving each to a navaid near its position.
    std::vector<RoutePoint> pts = parse_route();
    for (RoutePoint& p : pts) {
        if (p.ident.empty()) continue;
        float la = p.lat, lo = p.lon;
        p.ref = XPLMFindNavAid(nullptr, p.ident.c_str(), &la, &lo, nullptr, kTypes);
        if (p.ref != XPLM_NAV_NOT_FOUND) {
            XPLMGetNavAidInfo(p.ref, &p.type, nullptr, nullptr, nullptr, nullptr, nullptr,
                              nullptr, nullptr, nullptr);
        }
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
    std::snprintf(msg, sizeof msg, "XVoiceLayer plugin: loaded %d FMS waypoints\n", idx);
    XPLMDebugString(msg);
}

// ---- Sequencing -------------------------------------------------------------
// The app's follower steers on the heading bug and never flies GPS mode, so the
// sim's own FMS sat on entry 1 for a whole flight while the aircraft was five
// fixes on (LXGB-LEMG, 2026-09-12): the GPS page, the G1000 readouts and every
// gps_* dataref described a fix long behind. The follower now writes
// "IDENT,lat,lon" into xcopilot/fms/goto each time it captures a fix; the entry
// is found among the LOADED plan's entries (whoever loaded it), by ident first,
// nearest to the position among namesakes, else the nearest entry within a mile
// (a bare lat/lon point has no ident), and becomes the destination on both the
// legacy GPS plan and the native FMS plan. No match changes nothing: approach
// legs are not in the enroute plan and must not drag the destination around.

static double approx_nm(double lat1, double lon1, double lat2, double lon2) {
    const double dlat = (lat2 - lat1) * 60.0;
    const double dlon = (lon2 - lon1) * 60.0 * std::cos((lat1 + lat2) * 0.5 * M_PI / 180.0);
    return std::sqrt(dlat * dlat + dlon * dlon);
}

static bool ident_eq(const char* a, const char* b) {
    for (;; ++a, ++b) {
        const int ca = std::toupper(static_cast<unsigned char>(*a));
        const int cb = std::toupper(static_cast<unsigned char>(*b));
        if (ca != cb) return false;
        if (ca == 0) return true;
    }
}

// The entry the payload names, or -1. Pure over the entry table so it can be
// reasoned about without the sim: ident match wins, distance breaks ties, and a
// nameless payload or an unmatched ident falls back to proximity only.
static int find_fms_entry(const char* ident, double lat, double lon) {
    const int n = XPLMCountFMSEntries();
    int best = -1;
    double best_nm = 1e9;
    bool best_named = false;
    for (int i = 0; i < n; ++i) {
        XPLMNavType type = xplm_Nav_Unknown;
        char id[256] = {0};
        XPLMNavRef ref = XPLM_NAV_NOT_FOUND;
        int alt = 0;
        float la = 0.0f, lo = 0.0f;
        XPLMGetFMSEntryInfo(i, &type, id, &ref, &alt, &la, &lo);
        const double d = approx_nm(lat, lon, la, lo);
        const bool named = ident[0] != '\0' && ident_eq(id, ident);
        if (named && !best_named) {
            best = i; best_nm = d; best_named = true;
        } else if (named == best_named && d < best_nm) {
            best = i; best_nm = d;
        }
    }
    if (best < 0) return -1;
    if (!best_named && best_nm > 1.0) return -1;
    return best;
}

// ---- Deferred to the flight loop --------------------------------------------
// The Web API runs a write's accessor inside its own request dispatch, so the
// accessors only store and flag: the navaid searches, the plan load and the
// destination changes run in FmsLoop, as flightinit.cpp runs XPLMInitFlight
// (2026-09-28: X-Plane hung twice after an init with our traffic still
// arriving; nothing heavy stays in a set handler). A load waits out the
// flight-init quiet window.
static bool g_load_pending = false;
static bool g_goto_pending = false;

static void sequence_goto() {
    char buf[sizeof g_fms_goto.buf];
    std::strncpy(buf, g_fms_goto.buf, sizeof buf);
    buf[sizeof buf - 1] = '\0';
    char* save = nullptr;
    char* ident = strtok_r(buf, ",", &save);
    char* slat = strtok_r(nullptr, ",", &save);
    char* slon = strtok_r(nullptr, ",", &save);
    if (!ident || !slat || !slon) return;
    const int i = find_fms_entry(ident, std::atof(slat), std::atof(slon));
    g_goto_off_plan = i < 0;
    g_goto_ident = ident;
    g_goto_lat = std::atof(slat);
    g_goto_lon = std::atof(slon);
    char msg[160];
    if (i < 0) {
        std::snprintf(msg, sizeof msg, "XVoiceLayer plugin: fms/goto %s: no such entry in the loaded plan\n", ident);
        XPLMDebugString(msg);
        return;
    }
    XPLMSetDestinationFMSEntry(i);
    XPLMSetDestinationFMSFlightPlanEntry(xplm_Fpl_Pilot_Primary, i);
    std::snprintf(msg, sizeof msg, "XVoiceLayer plugin: fms/goto %s -> entry %d\n", ident, i);
    XPLMDebugString(msg);
}

static void WriteFmsGoto(void*, void* in, int offset, int len) {
    StoreText(&g_fms_goto, in, offset, len);
    g_goto_pending = true;
}

static int ReadFmsLoad(void*) { return g_fms_load; }
static void WriteFmsLoad(void*, int v) {
    if (v != 0) {
        g_load_pending = true;
    }
    g_fms_load = 0; // edge-triggered; reset so a re-load needs a fresh write
}

static float FmsLoop(float, float, int, void*) {
    if (flightinit::loading()) return 0.25f;
    if (g_load_pending) {
        g_load_pending = false;
        load_fms_route();
    }
    if (g_goto_pending) {
        g_goto_pending = false;
        sequence_goto();
    }
    return 0.1f;
}

namespace fms {

void register_datarefs() {
    g_fms_route_dref = XPLMRegisterDataAccessor(
        "xcopilot/fms/route", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteFmsRoute, &g_fms_route, &g_fms_route);
    g_fms_load_dref = XPLMRegisterDataAccessor(
        "xcopilot/fms/load", xplmType_Int, 1,
        ReadFmsLoad, WriteFmsLoad, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_fms_goto_dref = XPLMRegisterDataAccessor(
        "xcopilot/fms/goto", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteFmsGoto, &g_fms_goto, &g_fms_goto);
    XPLMRegisterFlightLoopCallback(FmsLoop, 0.1f, nullptr);
}

void unregister_datarefs() {
    XPLMUnregisterFlightLoopCallback(FmsLoop, nullptr);
    if (g_fms_goto_dref) { XPLMUnregisterDataAccessor(g_fms_goto_dref); g_fms_goto_dref = nullptr; }
    if (g_fms_load_dref) { XPLMUnregisterDataAccessor(g_fms_load_dref); g_fms_load_dref = nullptr; }
    if (g_fms_route_dref) { XPLMUnregisterDataAccessor(g_fms_route_dref); g_fms_route_dref = nullptr; }
}

void load_route() { g_load_pending = true; }

void read_airac_cycle() { read_airac_cycle_impl(); }

const std::string& airac_cycle() { return g_airac_cycle; }

const std::vector<RoutePoint>& route_points() { return g_route_pts; }

double distance_nm(double lat1, double lon1, double lat2, double lon2) {
    return approx_nm(lat1, lon1, lat2, lon2);
}

double bearing_true_deg(double lat1, double lon1, double lat2, double lon2) {
    const double dlat = lat2 - lat1;
    const double dlon = (lon2 - lon1) * std::cos((lat1 + lat2) * 0.5 * M_PI / 180.0);
    double b = std::atan2(dlon, dlat) * 180.0 / M_PI;
    if (b < 0.0) b += 360.0;
    return b;
}

// The row of the display route this fix is: ident first, nearest among
// namesakes, else the nearest nameless point within a mile.
static int route_row(const std::string& ident, double lat, double lon) {
    int best = -1;
    double best_nm = 1e9;
    bool best_named = false;
    for (size_t i = 0; i < g_route_pts.size(); ++i) {
        const RoutePoint& p = g_route_pts[i];
        const double d = approx_nm(lat, lon, p.lat, p.lon);
        const bool named = !ident.empty() && ident_eq(p.ident.c_str(), ident.c_str());
        if (named && !best_named) {
            best = static_cast<int>(i); best_nm = d; best_named = true;
        } else if (named == best_named && d < best_nm) {
            best = static_cast<int>(i); best_nm = d;
        }
    }
    if (best >= 0 && !best_named && best_nm > 1.0) return -1;
    return best;
}

ActiveFix active_fix() {
    ActiveFix a = {false, "", 0.0, 0.0, -1};
    if (g_goto_off_plan) {
        a.valid = true;
        a.ident = g_goto_ident;
        a.lat = g_goto_lat;
        a.lon = g_goto_lon;
        return a;
    }
    const int n = XPLMCountFMSEntries();
    const int i = XPLMGetDestinationFMSEntry();
    if (n <= 0 || i < 0 || i >= n) return a;
    XPLMNavType type = xplm_Nav_Unknown;
    char id[256] = {0};
    XPLMNavRef ref = XPLM_NAV_NOT_FOUND;
    int alt = 0;
    float la = 0.0f, lo = 0.0f;
    XPLMGetFMSEntryInfo(i, &type, id, &ref, &alt, &la, &lo);
    if (la == 0.0f && lo == 0.0f) return a;
    a.valid = true;
    a.ident = id;
    a.lat = la;
    a.lon = lo;
    a.row = route_row(a.ident, a.lat, a.lon);
    return a;
}

} // namespace fms
