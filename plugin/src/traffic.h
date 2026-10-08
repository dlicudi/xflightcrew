// Our AI traffic, drawn through XPMP2 (stage D, docs/tasks/pillars-audit/
// stage-d-traffic.md). The app's fleet writes xcopilot/traffic/roster and
// xcopilot/traffic/frames (layout in traffic_wire.h); a flight loop decodes
// them and keeps one XPMP2 aircraft per roster id, drawn one frame behind.

#pragma once

namespace traffic {

// Register the xcopilot/traffic/* datarefs, initialise XPMP2 with the CSL
// folder (XCOPILOT_TRAFFIC_CSL, else LiveTraffic's), load its models and
// start the flight loop. Call after drefs::start().
void start();
// Destroy every aircraft, XPMPMultiplayerCleanup, unregister. Call first in
// XPluginStop, before the rest of the teardown.
void stop();

} // namespace traffic
