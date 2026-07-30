// ---- FMS route loading ------------------------------------------------------
// The app writes the SimBrief route into xcopilot/fms/route as
// "IDENT,lat,lon,alt|IDENT,lat,lon,alt|...", then writes 1 to xcopilot/fms/load
// to build it in the GPS/FMS (only the plugin can, via XPLMNavigation).

#include "fms.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"

#include "drefs.h"

static TextRef g_fms_route = {{0}, 0};
static int g_fms_load = 0;
static XPLMDataRef g_fms_route_dref = nullptr;
static XPLMDataRef g_fms_load_dref = nullptr;

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

static int ReadFmsLoad(void*) { return g_fms_load; }
static void WriteFmsLoad(void*, int v) {
    if (v != 0) {
        load_fms_route();
    }
    g_fms_load = 0; // edge-triggered; reset so a re-load needs a fresh write
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
}

void unregister_datarefs() {
    if (g_fms_load_dref) { XPLMUnregisterDataAccessor(g_fms_load_dref); g_fms_load_dref = nullptr; }
    if (g_fms_route_dref) { XPLMUnregisterDataAccessor(g_fms_route_dref); g_fms_route_dref = nullptr; }
}

void load_route() { load_fms_route(); }

void read_airac_cycle() { read_airac_cycle_impl(); }

const std::string& airac_cycle() { return g_airac_cycle; }

const std::vector<RoutePoint>& route_points() { return g_route_pts; }

} // namespace fms
