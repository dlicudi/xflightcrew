// The nosewheel TILLER channel: xcopilot/tiller (-1..1, right positive), the
// large-angle ground steering an airliner's pedals do not give. Same heartbeat
// contract as xcopilot/taxi_steer (written every tick, expires within a second
// of writes stopping, yields to the pilot), but WHAT it drives is chosen by
// xcopilot/tiller_method, which the app sets from the airframe's profile. With
// method 0 (the default, and every airframe that declares none) the channel is
// inert: nothing is found, nothing is written.

#pragma once

namespace tiller {

// Register the xcopilot/tiller* datarefs and the two flight loops (before and
// after the flight model).
void start();
// Release anything held (the airframe's own state restored), destroy the
// loops, unregister the datarefs.
void stop();

// Method 2 (the Zibo's YAW nosewheel option) rides the pedal channel: while it
// is applied the steer channel writes the tiller value into the yaw ratio in
// place of its own. True (and *v set) only then.
bool yaw_override(float* v);

} // namespace tiller
