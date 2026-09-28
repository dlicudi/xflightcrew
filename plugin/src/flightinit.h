// Flight initialisation from the app: xcopilot/sim/init_flight (JSON in),
// xcopilot/sim/init_result, xcopilot/sim/init_seq and xcopilot/sim/loading
// (out). See flightinit.cpp.

#pragma once

namespace flightinit {

// Register the mailboxes and the flight loop that runs XPLMInitFlight.
void start();
void stop();

// Inside the load quiet window (xcopilot/sim/loading reads 1): from the
// mailbox write until 3 s of sim frames after XPLMInitFlight returns.
bool loading();

} // namespace flightinit
