// Voices through the sim's own audio: the app writes each utterance to a WAV
// and posts "channel|volume|path" into xcopilot/audio/play; the plugin
// plays it on X-Plane's FMOD buses (COM1 for the station, the copilot bus for
// the intercom) so it sits in the cockpit mix, follows the sim's volumes and
// headset simulation, and mutes when the sim mutes. xcopilot/audio/cut stops a
// channel (barge-in). Opt-in from the Settings tab (xcopilot/opt/audio_bus).

#pragma once

namespace audio {

void register_datarefs();
void unregister_datarefs();

// Flight-loop tick: play whatever the app posted (XPLMPlayPCMOnBus is
// main-thread only, so the accessor only flags it).
void tick();

int opt_audio_bus();
void set_opt_audio_bus(int v);

} // namespace audio
