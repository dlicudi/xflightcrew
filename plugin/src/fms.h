// FMS route loading: the app-to-plugin mailbox datarefs, navaid resolution,
// flight-plan composition, and the XPLMLoadFMSFlightPlan hand-off. Only the
// plugin can reach the GPS/FMS, so this is the capability half of the
// REST-for-transport, plugin-for-capability architecture.

#pragma once

#include <string>
#include <vector>

#include "XPLMNavigation.h"

// One parsed route point from the app's "IDENT,lat,lon,alt|..." payload, with the
// navaid it resolved to (if any) so we can pick the right .fms row type.
struct RoutePoint {
    std::string ident;
    float lat, lon;
    int alt;
    XPLMNavRef ref;     // XPLM_NAV_NOT_FOUND for a bare lat/lon point
    XPLMNavType type;   // the navaid's type (only meaningful when ref is set)
};

namespace fms {

// Register/unregister xcopilot/fms/route, xcopilot/fms/load and
// xcopilot/fms/goto (the follower sequencing the loaded plan). Called by
// drefs::start / drefs::stop_rest at the mailboxes' original slot in the
// registration sequence.
void register_datarefs();
void unregister_datarefs();

// Compose the plan from the last pushed route and load it into the GPS/FMS.
// Also triggered by the panel's Send to FMS button.
void load_route();

// Read X-Plane's loaded AIRAC cycle once at startup; empty if undetermined.
void read_airac_cycle();
const std::string& airac_cycle();

// Display copy for the panel's Route tab, refreshed whenever the app writes.
const std::vector<RoutePoint>& route_points();

} // namespace fms
