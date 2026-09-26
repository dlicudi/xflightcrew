// Flight initialisation from the app: xcopilot/sim/init_flight (JSON in),
// xcopilot/sim/init_result and xcopilot/sim/init_seq (out). See flightinit.cpp.

#pragma once

namespace flightinit {

// Register the mailboxes and the flight loop that runs XPLMInitFlight.
void start();
void stop();

} // namespace flightinit
