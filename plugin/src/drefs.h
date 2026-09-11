// The plugin's dataref and command surface: registration, storage, and the
// narrow accessors the panel renders from. Everything the app pushes over the
// Web API lands here; the two FMS mailboxes are owned by fms.cpp but are
// registered/unregistered at their original slot in this module's sequence.

#pragma once

#include <string>
#include <vector>

#include "common.h"

// Generic accessors for a TextRef-backed data dataref, shared with the FMS
// mailbox registration in fms.cpp.
int ReadText(void* ref, void* out, int offset, int max);
// Store the whole value (the app PATCHes at offset 0), NUL-terminated. Any text
// write is proof the app is alive, so it refreshes the connection heartbeat too
// (not just phase writes, a busy strip stream must not read as disconnected).
void StoreText(TextRef* t, void* in, int offset, int len);

// One in-range station for the ATC tab, parsed from the app's payload.
struct AtcService { std::string label; int khz; };
struct AtcField {
    std::string name;
    bool active;    // the station that answers radio calls ('*' prefix)
    int dist_nm;
    std::vector<AtcService> freqs;
};

namespace drefs {

// Register the commands, every xcopilot/* dataref (including the FMS mailboxes,
// via fms::register_datarefs at their original slot), the sim dataref lookups,
// and the taxi-brake flight loop. Original registration order preserved.
void start();
// Teardown is split so the caller can destroy the panel in between, matching
// the original XPluginStop order: stop_first() drops the taxi-brake loop (and
// releases any held brake) plus the panel-feed datarefs; stop_rest() takes
// down everything else, including the FMS mailboxes.
void stop_first();
void stop_rest();

// The app heartbeats the phase ~1/s; a stale timestamp = not running.
bool app_connected();

int phase();
bool ptt_active();
bool radio_ptt_active();
const TextRef& aircraft();
const TextRef& strip();
const TextRef& radio_row();
const TextRef& coach();
bool coach_urgent();
char coach_group();
// The Flight tab's automation feed (marker byte already stripped): one line
// per active manager, primary first as "NAME|stage". auto_active() is the
// marker — true while something the CANCEL button can stand down is running.
const TextRef& auto_status();
bool auto_active();
const std::vector<AtcField>& atc_fields();

// COM1, for the ATC tab: khz read (0 if the sim dataref is missing) and the
// click-to-tune write (a no-op if missing).
int com1_khz();
void tune_com1(int khz);

// Panel request codes (plugin to app), see the g_request comment in drefs.cpp.
int request();
void set_request(int v);

// Settings-tab switches; each drives a dataref the app polls live.
int llm_enabled();
void set_llm_enabled(int v);
int opt_camera_snap();
void set_opt_camera_snap(int v);
int opt_negative_tone();
void set_opt_negative_tone(int v);
int opt_monitor_mute();
void set_opt_monitor_mute(int v);
int opt_mode();
void set_opt_mode(int v);
int opt_cabin();
void set_opt_cabin(int v);
// UI scale index (0 = 100% .. 4 = 200% in quarter steps; -1 = app hasn't published).
int opt_ui_scale();
void set_opt_ui_scale(int v);
// Hand a typed command to the app (xcopilot/typed; consumed by the app).
void set_typed(const char* text);
int opt_voice();
void set_opt_voice(int v);
const char* voice_list();
int voice_list_len();

} // namespace drefs
