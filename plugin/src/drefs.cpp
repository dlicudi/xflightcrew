// Dataref/command storage and registration. The accessor callbacks live here;
// rows that belong in the panel's transmission log are forwarded to
// panel::append_log as they arrive.

#include "drefs.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "XPLMDataAccess.h"
#include "XPLMGraphics.h"
#include "XPLMProcessing.h"
#include "XPLMScenery.h"
#include "XPLMUtilities.h"

#include "fms.h"
#include "flightinit.h"
#include "audio.h"
#include "panel.h"
#include "tiller.h"

// ---- PTT --------------------------------------------------------------------

static int g_ptt_active = 0;
static XPLMCommandRef g_ptt_cmd = nullptr;
static XPLMDataRef g_ptt_dref = nullptr;

static int PttCommandHandler(XPLMCommandRef, XPLMCommandPhase phase, void*) {
    if (phase == xplm_CommandBegin) g_ptt_active = 1;
    else if (phase == xplm_CommandEnd) g_ptt_active = 0;
    return 1;
}
static int ReadPttActive(void*) { return g_ptt_active; }

// Radio PTT: the transmit switch, as distinct from the intercom above. While
// held, the app routes the utterance to whatever ATC station COM1 is tuned to
// (never to the copilot). Command `xcopilot/radio_ptt` drives the readable
// dataref `xcopilot/radio_ptt_active`, mirroring the intercom pair exactly.
static int g_radio_ptt_active = 0;
static XPLMCommandRef g_radio_ptt_cmd = nullptr;
static XPLMDataRef g_radio_ptt_dref = nullptr;

static int RadioPttCommandHandler(XPLMCommandRef, XPLMCommandPhase phase, void*) {
    if (phase == xplm_CommandBegin) g_radio_ptt_active = 1;
    else if (phase == xplm_CommandEnd) g_radio_ptt_active = 0;
    return 1;
}
static int ReadRadioPttActive(void*) { return g_radio_ptt_active; }

// ---- panel requests (plugin → app) -------------------------------------------

// The emergency buttons write a request code the app polls and consumes (same
// direction as PTT): 0 = none, 1 = recover, 2 = autoland, 3 = my controls. The
// app runs the matching voice command and writes 0 back.
static int g_request = 0;
static XPLMDataRef g_request_dref = nullptr;
static int ReadRequest(void*) { return g_request; }
static void WriteRequest(void*, int v) { g_request = v; }

// Typed command (plugin → app). The Transcript tab's text line writes the
// pilot's words here; the app polls, injects them on the typed-command queue
// (the same one the terminal and the buttons ride), and consumes by writing
// a single NUL back, which reads as empty here. Never buffered: a second
// line typed before the app polled replaces the first.
static TextRef g_typed = {{0}, 0};
static XPLMDataRef g_typed_dref = nullptr;
// When the typed line was last set, so the app can place how long a line sat
// before it was taken (issue #90: "about 15 s" at LEMG, 2026-09-26, that the
// app's own log could not place). xcopilot/typed_age reads the seconds since
// (a steady wall-clock timer, running through a sim pause), -1 with no line.
static std::chrono::steady_clock::time_point g_typed_at{};
static XPLMDataRef g_typed_age_dref = nullptr;
static void StampTyped() {
    if (g_typed.len > 0) g_typed_at = std::chrono::steady_clock::now();
}
static float ReadTypedAge(void*) {
    if (g_typed.len <= 0 || g_typed.buf[0] == '\0') return -1.0f;
    return std::chrono::duration<float>(std::chrono::steady_clock::now() - g_typed_at).count();
}
static void WriteTyped(void* ref, void* in, int offset, int len) {
    StoreText(static_cast<TextRef*>(ref), in, offset, len);
    if (g_typed.buf[0] == '\0') g_typed.len = 0;
    StampTyped();
}

// ---- panel options (plugin → app) --------------------------------------------

// Settings-popup switches the app polls live. Ints so a missing plugin (dataref
// unresolved) is distinguishable from "off"; defaults match the app's defaults.
static int g_opt_camera_snap = 0;   // snap-to-panel camera on actions — OFF by
                                    // default (moving the pilot's view proved
                                    // disorienting; the callout toast carries the
                                    // information). Re-enable via Settings.
static int g_opt_negative_tone = 1; // negative tone on unrecognized command
static int g_opt_monitor_mute = 0;  // silence the monitor's callouts
static XPLMDataRef g_opt_snap_dref = nullptr;
static XPLMDataRef g_opt_tone_dref = nullptr;
static XPLMDataRef g_opt_mute_dref = nullptr;
static int ReadOptSnap(void*) { return g_opt_camera_snap; }
static void WriteOptSnap(void*, int v) { g_opt_camera_snap = v; }
static int ReadOptTone(void*) { return g_opt_negative_tone; }
static void WriteOptTone(void*, int v) { g_opt_negative_tone = v; }
static int ReadOptMute(void*) { return g_opt_monitor_mute; }
static void WriteOptMute(void*, int v) { g_opt_monitor_mute = v; }

// App mode (Both / Copilot only / ATC only / Instructor). The app publishes
// the live mode (0..3, mode.rs panel_index) and polls for the pilot's pick; a pick applies
// within ~2 s and persists to prefs.json as the next boot's default. -1 =
// untouched, which is also how the app detects a plugin reload wiped this
// state and re-publishes.
static int g_opt_mode = -1;
static XPLMDataRef g_opt_mode_dref = nullptr;
static int ReadOptMode(void*) { return g_opt_mode; }
static void WriteOptMode(void*, int v) { g_opt_mode = v; }

// Cabin-crew staffing (Auto / Aboard / Off). Same protocol as the mode: the
// app publishes the live pick (0/1/2, cabin.rs panel_index), polls for the
// pilot's change, applies it live and persists it. -1 = untouched/wiped.
static int g_opt_cabin = -1;
static XPLMDataRef g_opt_cabin_dref = nullptr;
static int ReadOptCabin(void*) { return g_opt_cabin; }
static void WriteOptCabin(void*, int v) { g_opt_cabin = v; }

// UI scale (100% / 125% / 150%, index 0/1/2). Same publish/pick protocol as
// the mode: the app publishes the saved pref, the Settings radios write the
// pilot's pick back, the app persists it. The panel applies whatever is here
// each frame, so a pick takes effect immediately. -1 = untouched/wiped.
static int g_opt_ui_scale = -1;
static XPLMDataRef g_opt_ui_scale_dref = nullptr;
static int ReadOptUiScale(void*) { return g_opt_ui_scale; }
static void WriteOptUiScale(void*, int v) { g_opt_ui_scale = v; }

// Copilot voice picker. The app publishes the roster of piper models on disk
// (newline-separated display names) into `voice_list` and the active voice's
// index into `voice`; the Settings combo writes the pilot's pick back, the app
// polls it and swaps the voice live. -1 = untouched (follow the app's voice),
// which is also how the app detects a plugin reload wiped this state and
// re-publishes. Own buffer type: the roster outgrows TextRef's 512 as voices
// are added.
static int g_opt_voice = -1;
static XPLMDataRef g_opt_voice_dref = nullptr;
static int ReadOptVoice(void*) { return g_opt_voice; }
static void WriteOptVoice(void*, int v) { g_opt_voice = v; }
// The cabin crew's pick from the same roster.
static int g_opt_cabin_voice = -1;
static XPLMDataRef g_opt_cabin_voice_dref = nullptr;
static int ReadOptCabinVoice(void*) { return g_opt_cabin_voice; }
static void WriteOptCabinVoice(void*, int v) { g_opt_cabin_voice = v; }
static struct { char buf[4096]; int len; } g_voice_list = {{0}, 0};
static XPLMDataRef g_voice_list_dref = nullptr;
static int ReadVoiceList(void*, void* out, int offset, int max) {
    if (out == nullptr) return g_voice_list.len;
    if (offset >= g_voice_list.len) return 0;
    int n = g_voice_list.len - offset;
    if (n > max) n = max;
    std::memcpy(out, g_voice_list.buf + offset, static_cast<size_t>(n));
    return n;
}
static void WriteVoiceList(void*, void* in, int offset, int len) {
    if (offset != 0) return;
    if (len > static_cast<int>(sizeof g_voice_list.buf) - 1) len = sizeof g_voice_list.buf - 1;
    if (len < 0) len = 0;
    std::memset(g_voice_list.buf, 0, sizeof g_voice_list.buf);
    std::memcpy(g_voice_list.buf, in, static_cast<size_t>(len));
    g_voice_list.len = len;
}

// Text settings (the registry's free-text knobs: callsign, airline
// telephony). App to plugin, `xcopilot/opt/text`: one row per knob,
// "key|label|pinned_env|value|notice|help", '\n'-separated, republished every
// few seconds (a reload zeroes it). Plugin to app, `xcopilot/opt/text_edit`:
// "key=value" lines written on Enter or focus loss, consumed by the app with
// a single NUL (the typed line's protocol); an edit made before the app
// polled is appended, not dropped.
static TextRef g_opt_text = {{0}, 0};
static XPLMDataRef g_opt_text_dref = nullptr;
static void WriteOptText(void* ref, void* in, int offset, int len) {
    StoreText(static_cast<TextRef*>(ref), in, offset, len);
}
static TextRef g_opt_text_edit = {{0}, 0};
static XPLMDataRef g_opt_text_edit_dref = nullptr;
static void WriteOptTextEdit(void* ref, void* in, int offset, int len) {
    StoreText(static_cast<TextRef*>(ref), in, offset, len);
    if (g_opt_text_edit.buf[0] == '\0') g_opt_text_edit.len = 0;
}

// ---- status datarefs --------------------------------------------------------

static int g_phase = 0;
static TextRef g_heard = {{0}, 0};
static TextRef g_action = {{0}, 0};
static TextRef g_alert = {{0}, 0};
static TextRef g_aircraft = {{0}, 0};
// Live flight strip — one compact status line the app refreshes (emergency
// phase/runway, cabin altitude, fuel endurance). Empty = hidden.
static TextRef g_strip = {{0}, 0};
// Taxi turn card, GPS style: "kind|dist_m|label|aim_true_deg|xtk_m|turn_deg|
// remaining_m" (empty = hide), refreshed like the strip.
static TextRef g_taxi = {{0}, 0};
// On-screen callout — WHAT the copilot is changing ("FLAPS > 10"), flashed in a
// floating toast over the sim for a couple of seconds after each action.
static TextRef g_callout = {{0}, 0};
static float g_callout_time = -1000.0f;
// Radio row — the ATC field's comm plan plus live COM1, refreshed by the app
// ("GIBRALTAR | >TWR 131.200 | APP 122.800"). '>' marks the tuned service.
static TextRef g_radio = {{0}, 0};
// ATC tab — every station in VHF range, nearest first, refreshed by the app
// as "NAME,dist_nm,SVC khz,...|NAME,..." ('*' prefix = the active station).
static TextRef g_atc = {{0}, 0};
// The radio coach's suggested next call for the current tower state,
// refreshed continuously by the app. Empty = no station. Rendered twice:
// the Calls tab's SAY NOW header, and the Transcript tab's pinned footer strip.
static TextRef g_coach = {{0}, 0};
// Automation feed — the Flight tab's live view of what the copilot is flying
// (full auto stage, autoland phase, taxi governor...). Marker byte + one line
// per active manager, primary first as "NAME|stage".
static TextRef g_auto = {{0}, 0};
// The plan's side of the load sheet, for the Route tab:
// "ORIGIN|DEST|block_kg|payload_kg|max_tow_kg|pax|unit". Empty = no plan.
static TextRef g_loadsheet = {{0}, 0};
// The Route tab's mission brief and the State tab's lines (the app's flight
// model and duty engine), written by the app, read by the panel.
static TextRef g_brief = {{0}, 0};
static TextRef g_state = {{0}, 0};
// The Settings tab's resolved callsign under its field: "spoken|source"
// (source override, simbrief or tail), refreshed by the app.
static TextRef g_callsign = {{0}, 0};
static XPLMDataRef g_callsign_dref = nullptr;
static void WriteCallsign(void* ref, void* in, int offset, int len) {
    StoreText(static_cast<TextRef*>(ref), in, offset, len);
}

static XPLMDataRef g_phase_dref = nullptr;
static XPLMDataRef g_heard_dref = nullptr;
static XPLMDataRef g_action_dref = nullptr;
static XPLMDataRef g_alert_dref = nullptr;
static XPLMDataRef g_aircraft_dref = nullptr;
static XPLMDataRef g_strip_dref = nullptr;
static XPLMDataRef g_taxi_dref = nullptr;
static XPLMDataRef g_callout_dref = nullptr;
static XPLMDataRef g_radio_dref = nullptr;
static XPLMDataRef g_atc_dref = nullptr;
static XPLMDataRef g_coach_dref = nullptr;
static XPLMDataRef g_auto_dref = nullptr;
static XPLMDataRef g_loadsheet_dref = nullptr;
static XPLMDataRef g_brief_dref = nullptr;
static XPLMDataRef g_state_dref = nullptr;
// The Board tab's flight information board (xcopilot/status/board,
// 2026-10-13): "H|field|HH:MM" then "D" or "A" rows, written every few
// seconds by an app running our own traffic at a timetable field. Its
// write time is kept: a board not refreshed for 30 s (the app stopped, the
// traffic stood down, an app older than the board never writing it) is
// stale and the tab says so.
static TextRef g_board = {{0}, 0};
static double g_board_at = -1.0;
static XPLMDataRef g_board_dref = nullptr;
// The Board tab's field pick (xcopilot/opt/board_field, 2026-10-13): the
// ICAO the pilot picked in its selector, plugin to app, read by the app as
// it publishes the board (empty = the field the traffic runs at). The
// choices ride the board's header ("H|field|HH:MM|ICAO,ICAO"); an app
// before the selector sends none and the tab shows no selector.
static TextRef g_board_field = {{0}, 0};
static XPLMDataRef g_board_field_dref = nullptr;
// COM1 active frequency (integer kHz despite the sim's `hz` name) — read for
// the ATC tab's tuned highlight, written by its click-to-tune.
static XPLMDataRef g_com1_dref = nullptr;
// TRUE heading, for the taxi card's arrow (its aim bearing is true too).
static XPLMDataRef g_psi_dref = nullptr;
// Proportional taxi brake. The app writes xcopilot/taxi_brake (0..1) and the
// flight loop drives the toe-brake ratios from it every frame, holding
// override_toe_brakes only while a fresh value wants brake; writes stopping
// for any reason (release, crash, kill -9) expire it within a second, ratios
// zeroed and the override dropped — the dataref equivalent of a held
// command, a boot that can't be left on. Per-frame is the part the app
// process can't provide: REST-rate ratio writes lose the re-drive fight
// (and l_brake_add, the no-override channel, is stomped by XP12 every
// frame — tested live 2026-07-14, writes read back -0.0 immediately).
static XPLMDataRef g_taxi_brake_dref = nullptr;
static XPLMDataRef g_l_brake_ratio = nullptr;
static XPLMDataRef g_r_brake_ratio = nullptr;
static XPLMDataRef g_override_toe_brakes = nullptr;
static float g_taxi_brake = 0.0f;
static float g_taxi_brake_stamp = -1.0f;
static bool g_taxi_brake_applied = false;
static const float kTaxiBrakeFreshSecs = 1.0f;
// The sim's OWN toe brakes against OUR rudder. For a pilot with no toe-brake
// axes assigned, X-Plane applies automatic differential toe braking whenever
// the rudder input is deflected hard at taxi speed ("auto-toe brakes",
// 11.30+, per the aircraft's Plane Maker "brake power to help with steering"
// parameter; the 172's is on). The steer channel below IS rudder input, so
// every hard nosewheel command the copilot made got the sim's brake stab on
// the inside wheel on top: the 172 fell from 6 kt to under 1 in four seconds
// in a corner with no brake commanded, and sat sawing 1.5 -> 0.9 kt every
// 1.2 s at the join as the speed-tapered steer command crossed the sim's
// threshold back and forth (Gibraltar, 2026-09-12, "jerky braking when
// steering"). The rule that switches the auto-brake off is "a plugin is
// controlling the toe brakes", i.e. this override, so it is held for the
// WHOLE time the steer channel is applied, ratios zero unless a channel
// wants brake, not only while braking.
//
// Which would lock the pilot out of their own brakes for the length of a
// steered taxi, so the yield below mirrors the steer channel's: a brake
// COMMAND (the key or a button, the only brake a pedal-less pilot has), or a
// toe-brake AXIS moving off its engage baseline, while the override is held
// for steering alone, latches xcopilot/brake_yield and drops the override
// that frame, so the sim's own brake handling runs and the ratios show the
// press, which is exactly what the governor's takeback detector reads. A
// channel that WANTS brake keeps the override: the pilot's press changes
// nothing about an aircraft already being braked, and a governor holding a
// stop must never lose its hold to a tap on the key. Re-armed when the steer
// channel goes stale.
static XPLMDataRef g_brake_yield_dref = nullptr;
static int g_brake_yield = 0;
static bool g_brake_hold_for_steer = false;
static XPLMCommandRef g_brake_cmds[4] = {nullptr, nullptr, nullptr, nullptr};
static const char* const kBrakeCmdNames[4] = {
    "sim/flight_controls/brakes_regular",
    "sim/flight_controls/brakes_max",
    "sim/flight_controls/brakes_toggle_regular",
    "sim/flight_controls/brakes_toggle_max",
};
static int g_brake_axis_l = -1;
static int g_brake_axis_r = -1;
static float g_brake_axis_baseline_l = 0.0f;
static float g_brake_axis_baseline_r = 0.0f;
static int g_brake_deflect_frames = 0;

// The DIFFERENTIAL half of the same pedals: xcopilot/taxi_brake_l and
// xcopilot/taxi_brake_r, one side at a time, the steering assist for a
// castoring-nosewheel airframe where yoke_heading has nothing to work with.
//
// It lives here and not in the app for the reason written above the symmetric
// channel, which the assist had to learn twice: the app wrote the ratios
// straight over REST and XP12 re-drove them from the pilot's assigned toe-brake
// axes the same frame. Probed live at a Southampton stand (2026-07-30), a
// PATCH of 0.25 read back 0.0000 within 400 ms with no override up and held
// 0.2500 indefinitely with one — so every brake-steer tuning note in
// autotaxi.rs from the 2026-07-28 and -29 sorties was tuning a number that
// never reached the aircraft, and the SF50 wandered its whole taxi with its
// only real control surface disconnected.
//
// One stamp for the pair: the app writes both sides together, so either write
// refreshes the channel and a silent writer expires both.
static XPLMDataRef g_steer_brake_l_dref = nullptr;
static XPLMDataRef g_steer_brake_r_dref = nullptr;
static float g_steer_brake_l = 0.0f;
static float g_steer_brake_r = 0.0f;
static float g_steer_brake_stamp = -1.0f;

// Proportional taxi steering, same shape as the brake channel: the app writes
// xcopilot/taxi_steer (-1..1) and the flight loop drives yoke_heading_ratio
// from it every frame under override_joystick_heading, expiring within a
// second of writes stopping — override dropped, ratio zeroed, the pedals the
// pilot's again whatever killed the writer.
//
// The pilot's feet always win, in-plugin, zero latency: while steering we
// watch the hardware yaw axis (found by scanning the axis assignments;
// baseline captured at engage so an off-centre resting axis can't false-
// trigger), and any real deflection latches xcopilot/steer_yield, drops the
// override THAT frame, and ignores further steer writes until the channel
// goes stale. The app polls the latch and stands its steering loop down.
static XPLMDataRef g_taxi_steer_dref = nullptr;
static XPLMDataRef g_steer_yield_dref = nullptr;
static XPLMDataRef g_yoke_heading = nullptr;
static XPLMDataRef g_override_heading = nullptr;
static XPLMDataRef g_joy_axis_values = nullptr;
static XPLMDataRef g_joy_axis_assignments = nullptr;
static float g_taxi_steer = 0.0f;
static float g_taxi_steer_stamp = -1.0f;
static bool g_taxi_steer_applied = false;
static int g_steer_yield = 0;
static int g_yaw_axis = -1;
static float g_yaw_axis_baseline = 0.5f;
static int g_steer_engage_frames = 0;
static int g_yaw_deflect_frames = 0;
// Rotation pitch, the takeoff's elevator — same shape as the steer channel:
// the app writes xcopilot/pitch (-1..1, positive = aft yoke, nose up) and the
// flight loop drives yoke_pitch_ratio from it every frame under
// override_joystick_pitch, expiring within a second of writes stopping. The
// AP's VS mode syncs to the CURRENT vertical speed at engage — zero on the
// runway — so no dataref preset can make it rotate; rotation is elevator,
// and only a plugin can hold the elevator against the joystick re-drive.
// The pilot's HANDS win like the feet do: real pitch-axis deflection from
// the engage baseline latches xcopilot/pitch_yield and drops the override
// that frame.
static XPLMDataRef g_pitch_dref = nullptr;
static XPLMDataRef g_pitch_yield_dref = nullptr;
static XPLMDataRef g_yoke_pitch = nullptr;
static XPLMDataRef g_override_pitch = nullptr;
static float g_pitch = 0.0f;
static float g_pitch_stamp = -1.0f;
static bool g_pitch_applied = false;
static int g_pitch_yield = 0;
static int g_pitch_axis = -1;
static float g_pitch_axis_baseline = 0.5f;
static int g_pitch_engage_frames = 0;
static int g_pitch_deflect_frames = 0;

// Terrain probe mailbox — the app's terrain eyes, and the generic answer to
// "how does the copilot know where the Rock is": it asks the sim. The app
// writes "lat,lon|lat,lon|..." to xcopilot/terrain/query; the next flight
// loop probes each point (XPLMProbeTerrainXYZ) and publishes ground
// elevations, meters MSL, in xcopilot/terrain/elev_m, bumping
// xcopilot/terrain/gen so the app knows the answer is fresh. Query capacity
// is one TextRef (511 bytes ≈ 24 points) — enough for a full pattern
// polyline per query. Only the app writes the query; probing runs in the
// flight loop, never in the accessor (the Web API thread must not touch
// scenery).
static XPLMDataRef g_terrain_query_dref = nullptr;
static XPLMDataRef g_terrain_elev_dref = nullptr;
static XPLMDataRef g_terrain_gen_dref = nullptr;
static XPLMDataRef g_terrain_valid_dref = nullptr;
static bool g_terrain_valid = false;
static TextRef g_terrain_query;
static const int kTerrainMaxPts = 32;
static float g_terrain_elev[kTerrainMaxPts];
static int g_terrain_n = 0;
static int g_terrain_gen = 0;
static bool g_terrain_pending = false;
static XPLMProbeRef g_terrain_probe = nullptr;

// And the ailerons (xcopilot/roll → yoke_roll_ratio under
// override_joystick_roll): between liftoff and the AP taking the climb,
// nobody else holds the wings against full-power torque.
static XPLMDataRef g_roll_dref = nullptr;
static XPLMDataRef g_roll_yield_dref = nullptr;
static XPLMDataRef g_yoke_roll = nullptr;
static XPLMDataRef g_override_roll = nullptr;
static float g_roll = 0.0f;
static float g_roll_stamp = -1.0f;
static bool g_roll_applied = false;
static int g_roll_yield = 0;
static int g_roll_axis = -1;
static float g_roll_axis_baseline = 0.5f;
static int g_roll_engage_frames = 0;
static int g_roll_deflect_frames = 0;

// The yoke ROUTE (xcopilot/yoke_method, set by the app from the airframe's
// profile `yoke_input:` on every swap). 0, the default, is the stock path
// above: yoke_*_ratio under override_joystick_*. 1 is the Zibo 737's own
// external input: zibomod holds the stock overrides itself and rewrites the
// yoke ratios every frame (probed parked 2026-10-07: yoke_pitch_ratio 0.5
// read back 0, the elevator unmoved; the LXGB takeoff of 2026-10-07 held
// xcopilot/pitch at -0.5 for 3 s while the nose rose 10 to 22 degrees), but
// with laminar/B738/flt_ctrls/joy_override = 1 it takes joy_pitch, joy_roll
// and joy_heading as the yoke (joy_pitch 0.5 put the elevator at -8.4
// degrees, joy_roll 0.5 the ailerons at 7.5/-10, the ratios following with
// the stock sense). On that route the stock overrides are never touched:
// dropping one zibomod holds hands the axis to the raw joystick. The route
// is latched per engage, so a swap mid-hold releases what was held.
static XPLMDataRef g_yoke_method_dref = nullptr;
static int g_yoke_method = 0;
static const int kYokeStock = 0;
static const int kYokeZiboJoy = 1;
static XPLMDataRef g_joy_override = nullptr;
static XPLMDataRef g_joy_pitch = nullptr;
static XPLMDataRef g_joy_roll = nullptr;
static XPLMDataRef g_joy_heading = nullptr;
// Which route each axis was engaged on (valid while *_applied).
static int g_pitch_route = kYokeStock;
static int g_roll_route = kYokeStock;
// The yaw the steer channel wrote this frame (0 when it holds nothing):
// joy_override also takes the yaw from joy_heading, so while the route holds
// the yoke it carries the steer channel's value, never a stale centre.
static float g_yaw_written = 0.0f;
static int ReadYokeMethod(void*) { return g_yoke_method; }
static void WriteYokeMethod(void*, int v) {
    g_yoke_method = (v == kYokeZiboJoy) ? kYokeZiboJoy : kYokeStock;
}
static bool joy_good(XPLMDataRef r) { return r && XPLMIsDataRefGood(r); }
// The route a fresh engage takes: the Zibo's only when all its names exist.
// Found at engage, never at start: zibomod registers them after we load,
// and an aircraft unload invalidates them.
static int yoke_route_now() {
    if (g_yoke_method != kYokeZiboJoy) return kYokeStock;
    g_joy_override = XPLMFindDataRef("laminar/B738/flt_ctrls/joy_override");
    g_joy_pitch = XPLMFindDataRef("laminar/B738/flt_ctrls/joy_pitch");
    g_joy_roll = XPLMFindDataRef("laminar/B738/flt_ctrls/joy_roll");
    g_joy_heading = XPLMFindDataRef("laminar/B738/flt_ctrls/joy_heading");
    if (joy_good(g_joy_override) && joy_good(g_joy_pitch)
        && joy_good(g_joy_roll) && joy_good(g_joy_heading)) {
        return kYokeZiboJoy;
    }
    return kYokeStock;
}
// The autopilot owns the yoke on the Zibo route. Its tablet's AP YOKE
// NEUTRAL TOL. (REAL) and YOKE MOVEMENT (CWS) mean a yoke off neutral keeps
// CMD A from engaging and drops an engaged one to CWS. Autoland held the
// aircraft level on this channel until the autopilot read engaged, so twenty
// CMD A presses met a held yoke and none took ("The autopilot will not
// engage", LEMG 30, 2026-10-07). A crew lets go of the yoke to engage: the
// channels let go while CMD A or B reads lit, and for kEngageReleaseS after
// any CMD press, so the press meets a neutral yoke.
static const float kEngageReleaseS = 1.5f;
static float g_engage_release_until = -1.0f;
static XPLMDataRef g_cmd_a_status = nullptr;
static XPLMDataRef g_cmd_b_status = nullptr;
static XPLMCommandRef g_cmd_a_press = nullptr;
static XPLMCommandRef g_cmd_b_press = nullptr;
static int EngagePressHandler(XPLMCommandRef, XPLMCommandPhase phase, void*) {
    if (phase == xplm_CommandBegin) {
        g_engage_release_until = XPLMGetElapsedTime() + kEngageReleaseS;
    }
    return 1;
}
// Found and hooked once the airframe has them (zibomod registers them after
// we load), on the Zibo route only.
static void hook_engage_once() {
    if (!g_cmd_a_status) g_cmd_a_status = XPLMFindDataRef("laminar/B738/autopilot/cmd_a_status");
    if (!g_cmd_b_status) g_cmd_b_status = XPLMFindDataRef("laminar/B738/autopilot/cmd_b_status");
    if (!g_cmd_a_press) {
        g_cmd_a_press = XPLMFindCommand("laminar/B738/autopilot/cmd_a_press");
        if (g_cmd_a_press) XPLMRegisterCommandHandler(g_cmd_a_press, EngagePressHandler, 1, nullptr);
    }
    if (!g_cmd_b_press) {
        g_cmd_b_press = XPLMFindCommand("laminar/B738/autopilot/cmd_b_press");
        if (g_cmd_b_press) XPLMRegisterCommandHandler(g_cmd_b_press, EngagePressHandler, 1, nullptr);
    }
}
static bool ap_owns_yoke() {
    if (g_yoke_method != kYokeZiboJoy) return false;
    hook_engage_once();
    if (XPLMGetElapsedTime() < g_engage_release_until) return true;
    const auto lit = [](XPLMDataRef r) {
        return r && XPLMIsDataRefGood(r) && XPLMGetDataf(r) > 0.5f;
    };
    return lit(g_cmd_a_status) || lit(g_cmd_b_status);
}

static bool joy_route_held() {
    return (g_pitch_applied && g_pitch_route == kYokeZiboJoy)
        || (g_roll_applied && g_roll_route == kYokeZiboJoy);
}
// Hold one yoke axis on the Zibo route: the override, the axis, the yaw.
static void joy_hold(XPLMDataRef axis, float v) {
    if (!joy_good(g_joy_override) || !joy_good(axis)) return;
    XPLMSetDataf(g_joy_override, 1.0f);
    XPLMSetDataf(axis, v);
    if (joy_good(g_joy_heading)) XPLMSetDataf(g_joy_heading, g_yaw_written);
}
// Frames the override is still held, axes centred, before it is dropped.
// Centring and dropping in one frame left the yoke where it was: zibomod
// never read the centred joy_roll and yoke_roll_ratio stayed at 0.5, the
// ailerons at 7.5 degrees (probed parked 2026-10-07); a centre held a
// second came back to 0.
static const int kJoyDropFrames = 30;
static int g_joy_drop_frames = 0;
static void joy_drop_now() {
    if (joy_good(g_joy_pitch)) XPLMSetDataf(g_joy_pitch, 0.0f);
    if (joy_good(g_joy_roll)) XPLMSetDataf(g_joy_roll, 0.0f);
    if (joy_good(g_joy_heading)) XPLMSetDataf(g_joy_heading, 0.0f);
    if (joy_good(g_joy_override)) XPLMSetDataf(g_joy_override, 0.0f);
    g_joy_drop_frames = 0;
}
// Centre one axis; once neither axis holds the route, the override is kept
// centred for kJoyDropFrames and then dropped (call after clearing that
// axis's *_applied).
static void joy_release(XPLMDataRef axis) {
    if (joy_good(axis)) XPLMSetDataf(axis, 0.0f);
    if (!joy_route_held()) g_joy_drop_frames = kJoyDropFrames;
}
// Every frame: the centred hold counting down to the drop.
static void joy_drop_tick() {
    if (g_joy_drop_frames <= 0) return;
    if (joy_route_held()) {
        g_joy_drop_frames = 0;
        return;
    }
    if (--g_joy_drop_frames > 0) {
        if (joy_good(g_joy_override)) XPLMSetDataf(g_joy_override, 1.0f);
        if (joy_good(g_joy_pitch)) XPLMSetDataf(g_joy_pitch, 0.0f);
        if (joy_good(g_joy_roll)) XPLMSetDataf(g_joy_roll, 0.0f);
        if (joy_good(g_joy_heading)) XPLMSetDataf(g_joy_heading, g_yaw_written);
    } else {
        joy_drop_now();
    }
}

// Axis-assignment codes in sim/joystick/joystick_axis_assignments.
static const int kAxisPitch = 1;
static const int kAxisRoll = 2;
static const int kAxisYaw = 3;
static const int kAxisBrakeL = 6;
static const int kAxisBrakeR = 7;
// Movement of a toe-brake axis from its engage baseline that reads as a foot
// on the pedal (direction-agnostic: calibrations invert).
static const float kBrakeYieldDelta = 0.25f;
// Deflection from the baseline that reads as the pilot's hands on the yoke.
static const float kPitchYieldDelta = 0.25f;
// Deflection from the baseline that reads as the pilot's feet.
static const float kYawYieldDelta = 0.25f;
// The yield only ARMS this many frames (~0.5 s) after engage, with the
// baseline following the axis until then: the app probes the channel while
// the pilot may still be centring the pedals from hand-steering, and a
// baseline frozen at that instant read the feet coming OFF as a press —
// the first Gatwick engage yielded 0.6 s in with nobody on the pedals
// (2026-07-20).
static const int kYieldArmFrames = 30;
// And it needs this many CONSECUTIVE deflected frames — a press, not one
// jittery read.
static const int kYieldHoldFrames = 4;

// Last time the app wrote any status (sim seconds). The app heartbeats the phase
// ~1/s, so a stale value means it isn't running → NOT CONNECTED. Starts stale.
static float g_last_status_time = -1000.0f;

static int ReadPhase(void*) { return g_phase; }
static void WritePhase(void*, int v) {
    g_phase = v;
    g_last_status_time = XPLMGetElapsedTime();
}

int ReadText(void* ref, void* out, int offset, int max) {
    TextRef* t = static_cast<TextRef*>(ref);
    if (out == nullptr) return t->len;
    if (offset >= t->len) return 0;
    int n = t->len - offset;
    if (n > max) n = max;
    std::memcpy(out, t->buf + offset, static_cast<size_t>(n));
    return n;
}

// Store the whole value (the app PATCHes at offset 0), NUL-terminated. Any text
// write is proof the app is alive, so it refreshes the connection heartbeat too
// (not just phase writes — a busy strip stream must not read as disconnected).
void StoreText(TextRef* t, void* in, int offset, int len) {
    g_last_status_time = XPLMGetElapsedTime();
    if (offset != 0) return;
    if (len > static_cast<int>(sizeof t->buf) - 1) len = static_cast<int>(sizeof t->buf) - 1;
    if (len < 0) len = 0;
    std::memset(t->buf, 0, sizeof t->buf);
    std::memcpy(t->buf, in, static_cast<size_t>(len));
    t->len = len;
}

// The next transcript row's "FROM > TO" label: the app writes it just
// before the heard or action line it labels (the same writer thread, in
// order). Taken by that row, never by a later one: a label older than a
// couple of seconds belongs to nobody. An older app never writes it, and the
// rows keep their tags.
static TextRef g_fromto = {{0}, 0};
static double g_fromto_at = -1.0;
static XPLMDataRef g_fromto_dref = nullptr;
static void WriteFromTo(void* ref, void* in, int offset, int len) {
    StoreText(static_cast<TextRef*>(ref), in, offset, len);
    g_fromto_at = XPLMGetElapsedTime();
}
static std::string TakeFromTo() {
    std::string out;
    if (g_fromto.len > 0 && g_fromto_at >= 0.0 && XPLMGetElapsedTime() - g_fromto_at < 2.0) {
        out.assign(g_fromto.buf, static_cast<size_t>(g_fromto.len));
    }
    g_fromto.len = 0;
    g_fromto.buf[0] = '\0';
    g_fromto_at = -1.0;
    return out;
}

static void WriteHeard(void*, void* in, int offset, int len) {
    StoreText(&g_heard, in, offset, len);
    // The pilot's own words, named as the speaker they are: the log reads as a
    // three-voice transcript (PILOT / COPILOT / ATC), white because it's the
    // pilot's half of the dialogue.
    const std::string route = TakeFromTo();
    panel::append_log("PILOT", COL_WHITE, g_heard.buf, false, route.c_str());
}
static void WriteAction(void*, void* in, int offset, int len) {
    StoreText(&g_action, in, offset, len);
    const std::string route = TakeFromTo();
    // A leading '~' marks AMBIENCE — the party line's overheard exchanges,
    // not a call addressed to this aircraft. Strip it, remember it: the row
    // keeps its speaker tag but renders dim.
    bool ambient = false;
    if (g_action.len > 0 && g_action.buf[0] == '~') {
        std::memmove(g_action.buf, g_action.buf + 1, static_cast<size_t>(g_action.len));
        g_action.len -= 1;
        ambient = true;
    }
    // A station prefix ("TWR: " / "GND: " / "APP: " / "ATIS: "...) is attribution,
    // not content: strip it and tag the row with the SPECIFIC position, so the
    // transcript names which controller spoke. Everything else is the crew.
    static const struct { const char* prefix; const char* tag; } kStations[] = {
        {"TWR: ", "TOWER"}, {"APP: ", "APP"}, {"DEP: ", "DEP"}, {"ATIS: ", "ATIS"},
        // The ground positions (apt.dat 1052/1053): their own chips.
        {"GND: ", "GROUND"}, {"DEL: ", "DELIVERY"},
        // Party-line traffic: the ghost pilots' side of the frequency. Spelled
        // out on the panel (every other tag is a plain speaker name).
        {"TFC: ", "TRAFFIC"},
    };
    for (const auto& s : kStations) {
        int n = static_cast<int>(std::strlen(s.prefix));
        if (g_action.len > n && std::strncmp(g_action.buf, s.prefix, n) == 0) {
            std::memmove(g_action.buf, g_action.buf + n, static_cast<size_t>(g_action.len - n + 1));
            g_action.len -= n;
            panel::append_log(s.tag, COL_AMBER, g_action.buf, ambient, route.c_str());
            return;
        }
    }
    panel::append_log("COPILOT", COL_GREEN, g_action.buf, ambient, route.c_str());
}
// Active aircraft ICAO — shown beside the connection chip, not logged.
static void WriteAircraft(void*, void* in, int offset, int len) {
    StoreText(&g_aircraft, in, offset, len);
}
// The taxi-speed governor renews this every braking tick; the stamp is what
// makes the value expire instead of stick.
static float ReadTaxiBrake(void*) { return g_taxi_brake; }
static void WriteTaxiBrake(void*, float v) {
    g_taxi_brake = v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
    g_taxi_brake_stamp = XPLMGetElapsedTime();
}
static float ReadSteerBrakeL(void*) { return g_steer_brake_l; }
static void WriteSteerBrakeL(void*, float v) {
    g_steer_brake_l = v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
    g_steer_brake_stamp = XPLMGetElapsedTime();
}
static float ReadSteerBrakeR(void*) { return g_steer_brake_r; }
static void WriteSteerBrakeR(void*, float v) {
    g_steer_brake_r = v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
    g_steer_brake_stamp = XPLMGetElapsedTime();
}
static float ReadTaxiSteer(void*) { return g_taxi_steer; }
static void WriteTaxiSteer(void*, float v) {
    g_taxi_steer = v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v);
    g_taxi_steer_stamp = XPLMGetElapsedTime();
}
static int ReadSteerYield(void*) { return g_steer_yield; }
static int ReadBrakeYield(void*) { return g_brake_yield; }
// Terrain mailbox accessors: the write only stores + flags; the probing
// itself happens in the flight loop.
static void WriteTerrainQuery(void*, void* in, int offset, int len) {
    StoreText(&g_terrain_query, in, offset, len);
    g_terrain_pending = g_terrain_query.len > 0;
}
static int ReadTerrainElev(void*, float* out, int offset, int max) {
    if (!out) return g_terrain_n;
    if (offset >= g_terrain_n) return 0;
    int n = g_terrain_n - offset;
    if (n > max) n = max;
    std::memcpy(out, g_terrain_elev + offset, static_cast<size_t>(n) * sizeof(float));
    return n;
}
static int ReadTerrainGen(void*) { return g_terrain_gen; }
static int ReadTerrainValid(void*) { return g_terrain_valid ? 1 : 0; }
static float ReadPitch(void*) { return g_pitch; }
static void WritePitch(void*, float v) {
    g_pitch = v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v);
    g_pitch_stamp = XPLMGetElapsedTime();
}
static int ReadPitchYield(void*) { return g_pitch_yield; }
static float ReadRoll(void*) { return g_roll; }
static void WriteRoll(void*, float v) {
    g_roll = v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v);
    g_roll_stamp = XPLMGetElapsedTime();
}
static int ReadRollYield(void*) { return g_roll_yield; }

// The hardware yaw axis's current value, or -1 if none is assigned.
static float yaw_axis_value() {
    if (g_yaw_axis < 0 || !g_joy_axis_values) return -1.0f;
    float v = 0.0f;
    XPLMGetDatavf(g_joy_axis_values, &v, g_yaw_axis, 1);
    return v;
}

// A hardware toe-brake axis's current value, or -1 if none is assigned.
static float brake_axis_value(int axis) {
    if (axis < 0 || !g_joy_axis_values) return -1.0f;
    float v = 0.0f;
    XPLMGetDatavf(g_joy_axis_values, &v, axis, 1);
    return v;
}
// Before-handler on the sim's brake commands, never consuming: a press while
// the toe-brake override is up for steering alone is the pilot asking for
// the brakes, and the answer is to hand them over, not to swallow the press.
static int BrakeCommandHandler(XPLMCommandRef, XPLMCommandPhase phase, void*) {
    if (phase == xplm_CommandBegin && g_brake_hold_for_steer) g_brake_yield = 1;
    return 1;
}

// The hardware pitch axis's current value, or -1 if none is assigned.
static float pitch_axis_value() {
    if (g_pitch_axis < 0 || !g_joy_axis_values) return -1.0f;
    float v = 0.0f;
    XPLMGetDatavf(g_joy_axis_values, &v, g_pitch_axis, 1);
    return v;
}

// The hardware roll axis's current value, or -1 if none is assigned.
static float roll_axis_value() {
    if (g_roll_axis < 0 || !g_joy_axis_values) return -1.0f;
    float v = 0.0f;
    XPLMGetDatavf(g_joy_axis_values, &v, g_roll_axis, 1);
    return v;
}

static float TaxiAssistLoop(float, float, int, void*) {
    audio::tick(); // main-thread only: whatever the app posted since last frame
    // A pending terrain query: probe every point this frame and publish.
    // Scenery access is flight-loop-only, which is why the accessor merely
    // flagged it. Points a pattern-radius from the aircraft are always inside
    // the loaded scenery region. A miss invalidates the batch: zero must
    // never impersonate measured sea level on a longer local flight.
    if (g_terrain_pending) {
        g_terrain_pending = false;
        if (g_terrain_probe == nullptr) g_terrain_probe = XPLMCreateProbe(xplm_ProbeY);
        char buf[sizeof g_terrain_query.buf];
        std::memcpy(buf, g_terrain_query.buf, sizeof buf);
        int n = 0;
        g_terrain_valid = true;
        char* save = nullptr;
        for (char* tok = strtok_r(buf, "|", &save); tok && n < kTerrainMaxPts;
             tok = strtok_r(nullptr, "|", &save)) {
            double lat = 0.0, lon = 0.0;
            if (std::sscanf(tok, "%lf,%lf", &lat, &lon) != 2) {
                g_terrain_valid = false;
                continue;
            }
            double x = 0.0, y = 0.0, z = 0.0;
            XPLMWorldToLocal(lat, lon, 0.0, &x, &y, &z);
            XPLMProbeInfo_t info;
            info.structSize = sizeof info;
            float elev_m = 0.0f;
            if (g_terrain_probe
                && XPLMProbeTerrainXYZ(g_terrain_probe, static_cast<float>(x),
                                       static_cast<float>(y), static_cast<float>(z),
                                       &info) == xplm_ProbeHitTerrain) {
                double plat = 0.0, plon = 0.0, palt = 0.0;
                XPLMLocalToWorld(info.locationX, info.locationY, info.locationZ,
                                 &plat, &plon, &palt);
                elev_m = static_cast<float>(palt);
                if (!std::isfinite(elev_m)) g_terrain_valid = false;
            } else {
                g_terrain_valid = false;
            }
            g_terrain_elev[n++] = elev_m;
        }
        g_terrain_n = n;
        g_terrain_valid = g_terrain_valid && n > 0;
        g_terrain_gen++;
    }

    // Both brake channels land on the same two ratios, so they are arbitrated
    // HERE, per wheel, in the one place that owns the override — two writers
    // racing for one dataref is the bug this channel exists to end. The
    // stopping brake is symmetric and the steering assist is one-sided; the
    // wheel takes whichever wants it harder, so a stop mid-turn still stops
    // and never cancels the turn it was called in.
    const bool fresh = g_taxi_brake_stamp >= 0.0f
        && XPLMGetElapsedTime() - g_taxi_brake_stamp <= kTaxiBrakeFreshSecs;
    const bool sb_fresh = g_steer_brake_stamp >= 0.0f
        && XPLMGetElapsedTime() - g_steer_brake_stamp <= kTaxiBrakeFreshSecs;
    float want_l = fresh ? g_taxi_brake : 0.0f;
    float want_r = want_l;
    if (sb_fresh) {
        if (g_steer_brake_l > want_l) want_l = g_steer_brake_l;
        if (g_steer_brake_r > want_r) want_r = g_steer_brake_r;
    }
    const bool wanted = want_l > 0.0f || want_r > 0.0f;
    // Held while braking, and while STEERING: an override up is what keeps
    // the sim's auto-toe brakes off our rudder (see g_brake_yield). Last
    // frame's steer flag, one frame of lag either way is nothing. A pilot
    // who has claimed the brakes keeps them until the steer channel is
    // stale, unless a channel actually wants brake.
    const bool steer_hold = g_taxi_steer_applied && !g_brake_yield;
    g_brake_hold_for_steer = steer_hold && !wanted;
    if (wanted || steer_hold) {
        if (g_override_toe_brakes) XPLMSetDatai(g_override_toe_brakes, 1);
        if (g_l_brake_ratio) XPLMSetDataf(g_l_brake_ratio, want_l);
        if (g_r_brake_ratio) XPLMSetDataf(g_r_brake_ratio, want_r);
        g_taxi_brake_applied = true;
    } else if (g_taxi_brake_applied) {
        if (g_l_brake_ratio) XPLMSetDataf(g_l_brake_ratio, 0.0f);
        if (g_r_brake_ratio) XPLMSetDataf(g_r_brake_ratio, 0.0f);
        // Dropped last: the sim re-drives the ratios from the pilot's
        // hardware the very next frame.
        if (g_override_toe_brakes) XPLMSetDatai(g_override_toe_brakes, 0);
        g_taxi_brake_applied = false;
    }

    const bool steer_fresh = g_taxi_steer_stamp >= 0.0f
        && XPLMGetElapsedTime() - g_taxi_steer_stamp <= kTaxiBrakeFreshSecs;
    if (steer_fresh && !g_steer_yield) {
        if (!g_taxi_steer_applied) {
            // Engage edge: find the yaw axis so the yield can watch the
            // pilot's actual hardware.
            g_yaw_axis = -1;
            g_brake_axis_l = -1;
            g_brake_axis_r = -1;
            if (g_joy_axis_assignments) {
                int assign[500];
                const int n = XPLMGetDatavi(g_joy_axis_assignments, assign, 0, 500);
                for (int i = 0; i < n; ++i) {
                    if (assign[i] == kAxisYaw && g_yaw_axis < 0) g_yaw_axis = i;
                    if (assign[i] == kAxisBrakeL && g_brake_axis_l < 0) g_brake_axis_l = i;
                    if (assign[i] == kAxisBrakeR && g_brake_axis_r < 0) g_brake_axis_r = i;
                }
            }
            g_steer_engage_frames = 0;
            g_yaw_deflect_frames = 0;
            g_brake_deflect_frames = 0;
            g_taxi_steer_applied = true;
        }
        const float yaw = yaw_axis_value();
        const float brk_l = brake_axis_value(g_brake_axis_l);
        const float brk_r = brake_axis_value(g_brake_axis_r);
        if (g_steer_engage_frames < kYieldArmFrames) {
            // Not armed yet: the baseline keeps following the axis, so the
            // pedals settling after engage become the neutral, not a press.
            ++g_steer_engage_frames;
            g_yaw_axis_baseline = yaw;
            g_brake_axis_baseline_l = brk_l;
            g_brake_axis_baseline_r = brk_r;
        } else {
            if (g_yaw_axis >= 0 && yaw >= 0.0f
                && std::fabs(yaw - g_yaw_axis_baseline) > kYawYieldDelta) {
                if (++g_yaw_deflect_frames >= kYieldHoldFrames) g_steer_yield = 1;
            } else {
                g_yaw_deflect_frames = 0;
            }
            // Toe-brake pedals, for the pilot who has them: the same
            // baseline-and-hold shape, yielding the BRAKES, not the steering.
            const bool brk_moved =
                (g_brake_axis_l >= 0 && brk_l >= 0.0f
                 && std::fabs(brk_l - g_brake_axis_baseline_l) > kBrakeYieldDelta)
                || (g_brake_axis_r >= 0 && brk_r >= 0.0f
                    && std::fabs(brk_r - g_brake_axis_baseline_r) > kBrakeYieldDelta);
            if (brk_moved && g_brake_hold_for_steer) {
                if (++g_brake_deflect_frames >= kYieldHoldFrames) g_brake_yield = 1;
            } else {
                g_brake_deflect_frames = 0;
            }
        }
        if (!g_steer_yield) {
            // The tiller riding the yaw axis (the Zibo's YAW nosewheel
            // option) takes the ratio in place of the pedal value: one
            // writer for yoke_heading_ratio.
            float yaw_out = g_taxi_steer;
            tiller::yaw_override(&yaw_out);
            if (g_override_heading) XPLMSetDatai(g_override_heading, 1);
            if (g_yoke_heading) XPLMSetDataf(g_yoke_heading, yaw_out);
            g_yaw_written = yaw_out;
        }
    }
    if (!steer_fresh || g_steer_yield) g_yaw_written = 0.0f;
    if ((!steer_fresh || g_steer_yield) && g_taxi_steer_applied) {
        if (g_yoke_heading) XPLMSetDataf(g_yoke_heading, 0.0f);
        // Dropped last, like the brake: the pilot's yaw hardware re-drives
        // the ratio the very next frame.
        if (g_override_heading) XPLMSetDatai(g_override_heading, 0);
        g_taxi_steer_applied = false;
    }
    // Writes stopped (the app saw the latch, or died): re-arm for next time.
    if (!steer_fresh) {
        g_steer_yield = 0;
        g_brake_yield = 0;
    }

    // The pitch channel: same freshness, same yield discipline, the yoke
    // instead of the pedals.
    const bool pitch_fresh = g_pitch_stamp >= 0.0f
        && XPLMGetElapsedTime() - g_pitch_stamp <= kTaxiBrakeFreshSecs;
    const bool ap_owns = ap_owns_yoke();
    if (pitch_fresh && !g_pitch_yield && !ap_owns) {
        if (!g_pitch_applied) {
            g_pitch_axis = -1;
            if (g_joy_axis_assignments) {
                int assign[500];
                const int n = XPLMGetDatavi(g_joy_axis_assignments, assign, 0, 500);
                for (int i = 0; i < n; ++i) {
                    if (assign[i] == kAxisPitch) { g_pitch_axis = i; break; }
                }
            }
            g_pitch_engage_frames = 0;
            g_pitch_deflect_frames = 0;
            g_pitch_route = yoke_route_now();
            g_pitch_applied = true;
        }
        const float ax = pitch_axis_value();
        if (g_pitch_engage_frames < kYieldArmFrames) {
            ++g_pitch_engage_frames;
            g_pitch_axis_baseline = ax;
        } else if (g_pitch_axis >= 0 && ax >= 0.0f
                   && std::fabs(ax - g_pitch_axis_baseline) > kPitchYieldDelta) {
            if (++g_pitch_deflect_frames >= kYieldHoldFrames) g_pitch_yield = 1;
        } else {
            g_pitch_deflect_frames = 0;
        }
        if (!g_pitch_yield) {
            if (g_pitch_route == kYokeZiboJoy) {
                joy_hold(g_joy_pitch, g_pitch);
            } else {
                if (g_override_pitch) XPLMSetDatai(g_override_pitch, 1);
                if (g_yoke_pitch) XPLMSetDataf(g_yoke_pitch, g_pitch);
            }
        }
    }
    if ((!pitch_fresh || g_pitch_yield || ap_owns) && g_pitch_applied) {
        g_pitch_applied = false;
        if (g_pitch_route == kYokeZiboJoy) {
            joy_release(g_joy_pitch);
        } else {
            if (g_yoke_pitch) XPLMSetDataf(g_yoke_pitch, 0.0f);
            // Dropped last: the pilot's pitch hardware (or the engaged AP
            // servo) owns the elevator the very next frame.
            if (g_override_pitch) XPLMSetDatai(g_override_pitch, 0);
        }
    }
    if (!pitch_fresh) g_pitch_yield = 0;

    // The roll channel: identical discipline, the other yoke axis.
    const bool roll_fresh = g_roll_stamp >= 0.0f
        && XPLMGetElapsedTime() - g_roll_stamp <= kTaxiBrakeFreshSecs;
    if (roll_fresh && !g_roll_yield && !ap_owns) {
        if (!g_roll_applied) {
            g_roll_axis = -1;
            if (g_joy_axis_assignments) {
                int assign[500];
                const int n = XPLMGetDatavi(g_joy_axis_assignments, assign, 0, 500);
                for (int i = 0; i < n; ++i) {
                    if (assign[i] == kAxisRoll) { g_roll_axis = i; break; }
                }
            }
            g_roll_engage_frames = 0;
            g_roll_deflect_frames = 0;
            g_roll_route = yoke_route_now();
            g_roll_applied = true;
        }
        const float ax = roll_axis_value();
        if (g_roll_engage_frames < kYieldArmFrames) {
            ++g_roll_engage_frames;
            g_roll_axis_baseline = ax;
        } else if (g_roll_axis >= 0 && ax >= 0.0f
                   && std::fabs(ax - g_roll_axis_baseline) > kPitchYieldDelta) {
            if (++g_roll_deflect_frames >= kYieldHoldFrames) g_roll_yield = 1;
        } else {
            g_roll_deflect_frames = 0;
        }
        if (!g_roll_yield) {
            if (g_roll_route == kYokeZiboJoy) {
                joy_hold(g_joy_roll, g_roll);
            } else {
                if (g_override_roll) XPLMSetDatai(g_override_roll, 1);
                if (g_yoke_roll) XPLMSetDataf(g_yoke_roll, g_roll);
            }
        }
    }
    if ((!roll_fresh || g_roll_yield || ap_owns) && g_roll_applied) {
        g_roll_applied = false;
        if (g_roll_route == kYokeZiboJoy) {
            joy_release(g_joy_roll);
        } else {
            if (g_yoke_roll) XPLMSetDataf(g_yoke_roll, 0.0f);
            if (g_override_roll) XPLMSetDatai(g_override_roll, 0);
        }
    }
    if (!roll_fresh) g_roll_yield = 0;
    joy_drop_tick();
    return -1.0f; // every frame: braking rides the physics rate, not REST rate
}
// Live flight strip — a status row the app refreshes continuously, not logged.
static void WriteStrip(void*, void* in, int offset, int len) {
    StoreText(&g_strip, in, offset, len);
}
// The taxi turn card, refreshed like the strip, not logged.
static void WriteTaxi(void*, void* in, int offset, int len) {
    StoreText(&g_taxi, in, offset, len);
}
// The load sheet feed — refreshed like the strip, not logged.
static void WriteLoadSheet(void*, void* in, int offset, int len) {
    StoreText(&g_loadsheet, in, offset, len);
}
// The mission brief and the state lines — refreshed, not logged.
static void WriteBrief(void*, void* in, int offset, int len) {
    StoreText(&g_brief, in, offset, len);
}
static void WriteState(void*, void* in, int offset, int len) {
    StoreText(&g_state, in, offset, len);
}
static void WriteBoard(void*, void* in, int offset, int len) {
    StoreText(&g_board, in, offset, len);
    g_board_at = XPLMGetElapsedTime();
}
// Radio row — refreshed continuously like the strip, not logged.
static void WriteRadio(void*, void* in, int offset, int len) {
    StoreText(&g_radio, in, offset, len);
}

static std::vector<AtcField> g_atc_fields;

// Parse "NAME,dist,SVC khz,...|NAME,..." into fields — cheap enough to run on
// every write; the ATC tab renders straight from this.
static std::vector<AtcField> parse_atc() {
    char buf[sizeof g_atc.buf];
    std::memcpy(buf, g_atc.buf, sizeof buf);

    std::vector<AtcField> fields;
    char* save1 = nullptr;
    for (char* tok = strtok_r(buf, "|", &save1); tok; tok = strtok_r(nullptr, "|", &save1)) {
        char* save2 = nullptr;
        char* name = strtok_r(tok, ",", &save2);
        char* dist = strtok_r(nullptr, ",", &save2);
        if (!name || !dist) continue;
        AtcField f;
        f.active = (name[0] == '*');
        f.name = f.active ? name + 1 : name;
        f.dist_nm = std::atoi(dist);
        for (char* svc = strtok_r(nullptr, ",", &save2); svc; svc = strtok_r(nullptr, ",", &save2)) {
            char* sp = std::strchr(svc, ' ');
            if (!sp) continue;
            *sp = '\0';
            f.freqs.push_back({svc, std::atoi(sp + 1)});
        }
        fields.push_back(std::move(f));
    }
    return fields;
}

static void WriteAtc(void*, void* in, int offset, int len) {
    StoreText(&g_atc, in, offset, len);
    g_atc_fields = parse_atc();
}

// Coach line — refreshed continuously like the radio row, not logged. The app
// leads the payload with two marker bytes (the alert severity scheme):
// urgency ('1' hint, '2' an OWED readback — amber text + a dot on the Calls
// tab) and phase group ('D' departure / 'A' arrival — highlights the matching
// phraseology reference group). Parsed off here; markerless text renders as a
// plain hint (legacy).
static bool g_coach_urgent = false;
static char g_coach_group = 0;
static void WriteCoach(void*, void* in, int offset, int len) {
    StoreText(&g_coach, in, offset, len);
    g_coach_urgent = false;
    g_coach_group = 0;
    if (g_coach.len >= 2 && (g_coach.buf[0] == '1' || g_coach.buf[0] == '2')
        && (g_coach.buf[1] == 'D' || g_coach.buf[1] == 'A')) {
        g_coach_urgent = (g_coach.buf[0] == '2');
        g_coach_group = g_coach.buf[1];
        std::memmove(g_coach.buf, g_coach.buf + 2,
                     static_cast<size_t>(g_coach.len - 2) + 1);
        g_coach.len -= 2;
    }
}
// Automation feed — the coach's marker-byte scheme: byte 0 is '1' while a
// manager is actually flying/driving (the AUTO annunciator; CANCEL renders
// unconditionally, since the app's pick falls back to the universal "my
// controls"), '0' when idle, stripped off here so the panel renders clean
// lines. Markerless text (an older app) renders as-is and reads as idle.
static bool g_auto_active = false;
static void WriteAuto(void*, void* in, int offset, int len) {
    StoreText(&g_auto, in, offset, len);
    g_auto_active = false;
    if (g_auto.len >= 1 && (g_auto.buf[0] == '0' || g_auto.buf[0] == '1')) {
        g_auto_active = (g_auto.buf[0] == '1');
        std::memmove(g_auto.buf, g_auto.buf + 1,
                     static_cast<size_t>(g_auto.len - 1) + 1);
        g_auto.len -= 1;
    }
}

// Callout — stamp the arrival time; the toast shows for a short window after.
// An optional "nx,ny|" prefix (fractions of the half-screen from centre) aims
// the toast toward the control the camera glanced at.
static float g_callout_nx = 0.0f, g_callout_ny = 0.0f;
static void WriteCallout(void*, void* in, int offset, int len) {
    StoreText(&g_callout, in, offset, len);
    g_callout_nx = 0.0f;
    g_callout_ny = 0.0f;
    char* bar = std::strchr(g_callout.buf, '|');
    float nx = 0.0f, ny = 0.0f;
    if (bar && std::sscanf(g_callout.buf, "%f,%f|", &nx, &ny) == 2) {
        g_callout_nx = nx;
        g_callout_ny = ny;
        int rest = g_callout.len - static_cast<int>(bar + 1 - g_callout.buf);
        std::memmove(g_callout.buf, bar + 1, static_cast<size_t>(rest) + 1);
        g_callout.len = rest;
    }
    g_callout_time = XPLMGetElapsedTime();
}
static void WriteAlert(void*, void* in, int offset, int len) {
    StoreText(&g_alert, in, offset, len);
    // First char is a severity marker ('1' advisory, '2' caution, '3' warning).
    // The tag names the SPEAKER — every alert is the copilot's voice, so the
    // transcript reads as who said it; severity survives as the row colour.
    const char* text = g_alert.buf;
    ImU32 col = COL_AMBER;
    if (g_alert.len >= 1) {
        switch (g_alert.buf[0]) {
            case '1': col = COL_CYAN;  text++; break;
            case '2': col = COL_AMBER; text++; break;
            case '3': col = COL_RED;   text++; break;
            default: break;  // no marker (legacy) — amber
        }
    }
    panel::append_log("COPILOT", col, text);
}

// ---- position snapshot (plugin -> app) ----------------------------------------

// xcopilot/position: where the aircraft is, as ONE frame saw it. Latitude
// and longitude used to arrive in separate Web API requests, so one reading
// could pair two frames: on LEMG's diagonal 13/31 up to 2.9 m of cross-track
// for a single tick above 120 kt (issue #157, 2026-09-28). A flight loop
// after the flight model copies every field into one buffer each frame; the
// app reads the buffer in one request. A data dataref of native (little-
// endian) doubles, since a float array would round latitude to ~0.4 m:
//   [0] layout version (1; 0 until the first frame is copied)
//   [1] frame (XPLMGetCycleNumber)   [2] sim time, s (total_running_time_sec)
//   [3] latitude, deg   [4] longitude, deg   [5] elevation MSL, m
//   [6] true heading (psi), deg   [7] groundspeed, m/s
//   [8] vertical speed, fpm (vh_ind_fpm)   [9] height above ground (y_agl), m
// A field whose sim dataref is missing is NaN: the app reads no measurement.
static const int kPositionFields = 10;
static double g_position[kPositionFields] = {0};
static XPLMDataRef g_position_dref = nullptr;
static XPLMFlightLoopID g_position_loop = nullptr;
static XPLMDataRef g_pos_lat = nullptr;
static XPLMDataRef g_pos_lon = nullptr;
static XPLMDataRef g_pos_elev = nullptr;
static XPLMDataRef g_pos_psi = nullptr;
static XPLMDataRef g_pos_gs = nullptr;
static XPLMDataRef g_pos_vs = nullptr;
static XPLMDataRef g_pos_agl = nullptr;
static XPLMDataRef g_pos_time = nullptr;

static double PosD(XPLMDataRef r) { return r ? XPLMGetDatad(r) : std::nan(""); }
static double PosF(XPLMDataRef r) {
    return r ? static_cast<double>(XPLMGetDataf(r)) : std::nan("");
}

static float PositionLoop(float, float, int, void*) {
    g_position[1] = static_cast<double>(XPLMGetCycleNumber());
    g_position[2] = PosF(g_pos_time);
    g_position[3] = PosD(g_pos_lat);
    g_position[4] = PosD(g_pos_lon);
    g_position[5] = PosD(g_pos_elev);
    g_position[6] = PosF(g_pos_psi);
    g_position[7] = PosF(g_pos_gs);
    g_position[8] = PosF(g_pos_vs);
    g_position[9] = PosF(g_pos_agl);
    g_position[0] = 1.0;
    return -1.0f;  // every frame
}

static int ReadPosition(void*, void* out, int offset, int max) {
    const int size = static_cast<int>(sizeof g_position);
    if (!out) return size;
    if (offset < 0 || offset >= size || max <= 0) return 0;
    int n = size - offset;
    if (n > max) n = max;
    std::memcpy(out, reinterpret_cast<const char*>(g_position) + offset,
                static_cast<size_t>(n));
    return n;
}

static void StartPosition() {
    g_pos_lat = XPLMFindDataRef("sim/flightmodel/position/latitude");
    g_pos_lon = XPLMFindDataRef("sim/flightmodel/position/longitude");
    g_pos_elev = XPLMFindDataRef("sim/flightmodel/position/elevation");
    g_pos_psi = XPLMFindDataRef("sim/flightmodel/position/psi");
    g_pos_gs = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
    g_pos_vs = XPLMFindDataRef("sim/flightmodel/position/vh_ind_fpm");
    g_pos_agl = XPLMFindDataRef("sim/flightmodel/position/y_agl");
    g_pos_time = XPLMFindDataRef("sim/time/total_running_time_sec");
    g_position_dref = XPLMRegisterDataAccessor(
        "xcopilot/position", xplmType_Data, 0,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadPosition, nullptr, nullptr, nullptr);
    XPLMCreateFlightLoop_t params;
    params.structSize = sizeof params;
    params.phase = xplm_FlightLoop_Phase_AfterFlightModel;
    params.callbackFunc = PositionLoop;
    params.refcon = nullptr;
    g_position_loop = XPLMCreateFlightLoop(&params);
    XPLMScheduleFlightLoop(g_position_loop, -1.0f, 1);
}

static void StopPosition() {
    if (g_position_loop) { XPLMDestroyFlightLoop(g_position_loop); g_position_loop = nullptr; }
    if (g_position_dref) { XPLMUnregisterDataAccessor(g_position_dref); g_position_dref = nullptr; }
    g_position[0] = 0.0;
}

// ---- state frame (plugin -> app) ---------------------------------------------

// xcopilot/state: everything a control tick reads, as ONE frame saw it, in
// ONE request. Every Web API request is served inside the sim's frame, so a
// tick costs a frame per ROUND of requests it waits on: measured on the iMac
// at 20 fps (2026-09-30) one GET took 46 ms, eight concurrent GETs 44 to 51,
// six sequential 312. The loops waited on six to eight rounds a tick
// (autoland's Final 0.56 s against a 200 ms sleep). The app reads this
// buffer, and any dataref it cannot carry, in one round (crates/sim/src/
// state.rs), falling back to separate reads without it.
//
// Filled AT THE READ, not per frame: the Web API calls the accessor on the
// sim's own thread, so every field is the same frame's, and a paused sim
// (which runs no flight loop) still reports its pause flag.
// Native little-endian doubles:
//   [0] layout version (1)   [1] frame (XPLMGetCycleNumber)   [2] sim time, s
//   [3..9] as xcopilot/position: lat, lon, elevation m, psi, groundspeed
//          m/s, vh_ind_fpm, y_agl m
//   [10..] kStateSources, in order (the app's `state::FIELDS`, test-pinned)
// A field whose dataref is missing is NaN: the app reads that field as no
// measurement, the rest of the frame stands.
static const char* const kStateSources[] = {
    "sim/flightmodel/position/indicated_airspeed",
    "sim/flightmodel/position/hpath",
    "sim/flightmodel/position/true_phi",
    "sim/flightmodel/position/true_theta",
    "sim/flightmodel/position/Q",
    "sim/flightmodel/position/R",
    "sim/flightmodel/failures/onground_any",
    "sim/flightmodel/position/magnetic_variation",
    "sim/cockpit2/gauges/indicators/vvi_fpm_pilot",
    "sim/time/paused",
    "sim/time/is_in_replay",
    "sim/cockpit2/autopilot/servos_on",
    "sim/cockpit2/autopilot/heading_status",
    "sim/cockpit2/autopilot/vvi_status",
    "sim/cockpit2/autopilot/heading_dial_deg_mag_pilot",
    "sim/cockpit2/radios/indicators/nav1_hdef_dots_pilot",
    "sim/cockpit2/radios/indicators/nav1_vdef_dots_pilot",
    "sim/cockpit2/radios/indicators/nav1_display_horizontal",
    "sim/cockpit2/radios/indicators/nav1_display_vertical",
    "sim/cockpit2/autopilot/approach_status",
    "sim/cockpit2/autopilot/glideslope_status",
    "xcopilot/steer_yield",
    "xcopilot/pitch_yield",
    "xcopilot/roll_yield",
    "xcopilot/brake_yield",
    "sim/aircraft/overflow/pushback_attached",
    "sim/flightmodel/engine/ENGN_TRQ",
    "sim/cockpit2/engine/actuators/throttle_ratio_all",
};
static const int kStateSourceCount =
    static_cast<int>(sizeof kStateSources / sizeof kStateSources[0]);
static const int kStateFields = kPositionFields + kStateSourceCount;
static double g_frame[kStateFields] = {0};
static XPLMDataRef g_frame_dref = nullptr;
static XPLMDataRef g_frame_refs[kStateSourceCount] = {nullptr};
static XPLMDataTypeID g_frame_types[kStateSourceCount] = {0};

// Any dataref as a number: a double, float or int scalar, else element 0
// of an array (ENGN_TRQ is engine 1's).
static double StateNum(XPLMDataRef r, XPLMDataTypeID t) {
    if (!r) return std::nan("");
    if (t & xplmType_Double) return XPLMGetDatad(r);
    if (t & xplmType_Float) return static_cast<double>(XPLMGetDataf(r));
    if (t & xplmType_Int) return static_cast<double>(XPLMGetDatai(r));
    if (t & xplmType_FloatArray) {
        float v = 0.0f;
        return XPLMGetDatavf(r, &v, 0, 1) == 1 ? static_cast<double>(v) : std::nan("");
    }
    if (t & xplmType_IntArray) {
        int v = 0;
        return XPLMGetDatavi(r, &v, 0, 1) == 1 ? static_cast<double>(v) : std::nan("");
    }
    return std::nan("");
}

static void FillState() {
    g_frame[1] = static_cast<double>(XPLMGetCycleNumber());
    g_frame[2] = PosF(g_pos_time);
    g_frame[3] = PosD(g_pos_lat);
    g_frame[4] = PosD(g_pos_lon);
    g_frame[5] = PosD(g_pos_elev);
    g_frame[6] = PosF(g_pos_psi);
    g_frame[7] = PosF(g_pos_gs);
    g_frame[8] = PosF(g_pos_vs);
    g_frame[9] = PosF(g_pos_agl);
    for (int i = 0; i < kStateSourceCount; ++i) {
        // Found late: an add-on's or our own dataref registered after start.
        if (!g_frame_refs[i]) {
            g_frame_refs[i] = XPLMFindDataRef(kStateSources[i]);
            if (g_frame_refs[i]) g_frame_types[i] = XPLMGetDataRefTypes(g_frame_refs[i]);
        }
        g_frame[kPositionFields + i] = StateNum(g_frame_refs[i], g_frame_types[i]);
    }
    g_frame[0] = 1.0;
}

static int ReadState(void*, void* out, int offset, int max) {
    const int size = static_cast<int>(sizeof g_frame);
    if (!out) return size;
    if (offset < 0 || offset >= size || max <= 0) return 0;
    // One fill per read: a chunked read's later pieces are the same frame.
    if (offset == 0) FillState();
    int n = size - offset;
    if (n > max) n = max;
    std::memcpy(out, reinterpret_cast<const char*>(g_frame) + offset,
                static_cast<size_t>(n));
    return n;
}

static void StartState() {
    for (int i = 0; i < kStateSourceCount; ++i) {
        g_frame_refs[i] = XPLMFindDataRef(kStateSources[i]);
        g_frame_types[i] = g_frame_refs[i] ? XPLMGetDataRefTypes(g_frame_refs[i]) : 0;
    }
    g_frame_dref = XPLMRegisterDataAccessor(
        "xcopilot/state", xplmType_Data, 0,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadState, nullptr, nullptr, nullptr);
}

static void StopState() {
    if (g_frame_dref) { XPLMUnregisterDataAccessor(g_frame_dref); g_frame_dref = nullptr; }
    g_frame[0] = 0.0;
}

// ---- registration and accessors ----------------------------------------------

namespace drefs {

void start() {
    g_ptt_cmd = XPLMCreateCommand("xcopilot/ptt", "XFlightCrew: hold to talk");
    XPLMRegisterCommandHandler(g_ptt_cmd, PttCommandHandler, 1, nullptr);

    g_ptt_dref = XPLMRegisterDataAccessor(
        "xcopilot/ptt_active", xplmType_Int, 0,
        ReadPttActive, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);

    g_radio_ptt_cmd =
        XPLMCreateCommand("xcopilot/radio_ptt", "XFlightCrew: hold to transmit (radio/ATC)");
    XPLMRegisterCommandHandler(g_radio_ptt_cmd, RadioPttCommandHandler, 1, nullptr);

    g_radio_ptt_dref = XPLMRegisterDataAccessor(
        "xcopilot/radio_ptt_active", xplmType_Int, 0,
        ReadRadioPttActive, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_phase_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/phase", xplmType_Int, 1,
        ReadPhase, WritePhase, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_heard_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/heard", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteHeard, &g_heard, &g_heard);
    g_action_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/action", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteAction, &g_action, &g_action);
    g_alert_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/alert", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteAlert, &g_alert, &g_alert);
    g_aircraft_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/aircraft", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteAircraft, &g_aircraft, &g_aircraft);
    fms::register_datarefs();
    flightinit::start();
    audio::register_datarefs();
    g_request_dref = XPLMRegisterDataAccessor(
        "xcopilot/request", xplmType_Int, 1,
        ReadRequest, WriteRequest, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_strip_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/strip", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteStrip, &g_strip, &g_strip);
    g_taxi_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/taxi", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteTaxi, &g_taxi, &g_taxi);
    g_opt_snap_dref = XPLMRegisterDataAccessor(
        "xcopilot/opt/camera_snap", xplmType_Int, 1,
        ReadOptSnap, WriteOptSnap, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_opt_tone_dref = XPLMRegisterDataAccessor(
        "xcopilot/opt/negative_tone", xplmType_Int, 1,
        ReadOptTone, WriteOptTone, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_opt_mute_dref = XPLMRegisterDataAccessor(
        "xcopilot/opt/monitor_mute", xplmType_Int, 1,
        ReadOptMute, WriteOptMute, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_opt_voice_dref = XPLMRegisterDataAccessor(
        "xcopilot/opt/voice", xplmType_Int, 1,
        ReadOptVoice, WriteOptVoice, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_opt_cabin_voice_dref = XPLMRegisterDataAccessor(
        "xcopilot/opt/cabin_voice", xplmType_Int, 1,
        ReadOptCabinVoice, WriteOptCabinVoice, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_opt_mode_dref = XPLMRegisterDataAccessor(
        "xcopilot/opt/mode", xplmType_Int, 1,
        ReadOptMode, WriteOptMode, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_opt_cabin_dref = XPLMRegisterDataAccessor(
        "xcopilot/opt/cabin_crew", xplmType_Int, 1,
        ReadOptCabin, WriteOptCabin, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_voice_list_dref = XPLMRegisterDataAccessor(
        "xcopilot/opt/voice_list", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadVoiceList, WriteVoiceList, nullptr, nullptr);
    g_callout_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/callout", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteCallout, &g_callout, &g_callout);
    g_radio_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/radio", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteRadio, &g_radio, &g_radio);
    g_atc_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/atc", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteAtc, &g_atc, &g_atc);
    g_coach_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/coach", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteCoach, &g_coach, &g_coach);
    g_auto_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/auto", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteAuto, &g_auto, &g_auto);
    g_loadsheet_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/loadsheet", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteLoadSheet, &g_loadsheet, &g_loadsheet);
    g_brief_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/brief", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteBrief, &g_brief, &g_brief);
    g_state_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/state", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteState, &g_state, &g_state);
    g_board_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/board", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteBoard, &g_board, &g_board);
    g_board_field_dref = XPLMRegisterDataAccessor(
        "xcopilot/opt/board_field", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteOptText,
        &g_board_field, &g_board_field);
    g_opt_ui_scale_dref = XPLMRegisterDataAccessor(
        "xcopilot/opt/ui_scale", xplmType_Int, 1,
        ReadOptUiScale, WriteOptUiScale, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_typed_dref = XPLMRegisterDataAccessor(
        "xcopilot/typed", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteTyped, &g_typed, &g_typed);
    g_typed_age_dref = XPLMRegisterDataAccessor(
        "xcopilot/typed_age", xplmType_Float, 0,
        nullptr, nullptr, ReadTypedAge, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_opt_text_dref = XPLMRegisterDataAccessor(
        "xcopilot/opt/text", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteOptText, &g_opt_text, &g_opt_text);
    g_opt_text_edit_dref = XPLMRegisterDataAccessor(
        "xcopilot/opt/text_edit", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteOptTextEdit,
        &g_opt_text_edit, &g_opt_text_edit);
    g_callsign_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/callsign", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteCallsign, &g_callsign, &g_callsign);
    g_fromto_dref = XPLMRegisterDataAccessor(
        "xcopilot/status/fromto", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteFromTo, &g_fromto, &g_fromto);
    g_com1_dref = XPLMFindDataRef("sim/cockpit2/radios/actuators/com1_frequency_hz_833");
    g_psi_dref = XPLMFindDataRef("sim/flightmodel/position/psi");
    g_taxi_brake_dref = XPLMRegisterDataAccessor(
        "xcopilot/taxi_brake", xplmType_Float, 1,
        nullptr, nullptr, ReadTaxiBrake, WriteTaxiBrake, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_steer_brake_l_dref = XPLMRegisterDataAccessor(
        "xcopilot/taxi_brake_l", xplmType_Float, 1,
        nullptr, nullptr, ReadSteerBrakeL, WriteSteerBrakeL, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_steer_brake_r_dref = XPLMRegisterDataAccessor(
        "xcopilot/taxi_brake_r", xplmType_Float, 1,
        nullptr, nullptr, ReadSteerBrakeR, WriteSteerBrakeR, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_l_brake_ratio = XPLMFindDataRef("sim/cockpit2/controls/left_brake_ratio");
    g_r_brake_ratio = XPLMFindDataRef("sim/cockpit2/controls/right_brake_ratio");
    g_override_toe_brakes = XPLMFindDataRef("sim/operation/override/override_toe_brakes");
    g_taxi_steer_dref = XPLMRegisterDataAccessor(
        "xcopilot/taxi_steer", xplmType_Float, 1,
        nullptr, nullptr, ReadTaxiSteer, WriteTaxiSteer, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_steer_yield_dref = XPLMRegisterDataAccessor(
        "xcopilot/steer_yield", xplmType_Int, 0,
        ReadSteerYield, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_brake_yield_dref = XPLMRegisterDataAccessor(
        "xcopilot/brake_yield", xplmType_Int, 0,
        ReadBrakeYield, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    for (int i = 0; i < 4; ++i) {
        g_brake_cmds[i] = XPLMFindCommand(kBrakeCmdNames[i]);
        if (g_brake_cmds[i]) {
            XPLMRegisterCommandHandler(g_brake_cmds[i], BrakeCommandHandler, 1, nullptr);
        }
    }
    g_pitch_dref = XPLMRegisterDataAccessor(
        "xcopilot/pitch", xplmType_Float, 1,
        nullptr, nullptr, ReadPitch, WritePitch, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_pitch_yield_dref = XPLMRegisterDataAccessor(
        "xcopilot/pitch_yield", xplmType_Int, 0,
        ReadPitchYield, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_yoke_pitch = XPLMFindDataRef("sim/cockpit2/controls/yoke_pitch_ratio");
    g_override_pitch = XPLMFindDataRef("sim/operation/override/override_joystick_pitch");
    g_terrain_query_dref = XPLMRegisterDataAccessor(
        "xcopilot/terrain/query", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteTerrainQuery,
        &g_terrain_query, &g_terrain_query);
    g_terrain_elev_dref = XPLMRegisterDataAccessor(
        "xcopilot/terrain/elev_m", xplmType_FloatArray, 0,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, ReadTerrainElev, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_terrain_valid_dref = XPLMRegisterDataAccessor(
        "xcopilot/terrain/valid", xplmType_Int, 0,
        ReadTerrainValid, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_terrain_gen_dref = XPLMRegisterDataAccessor(
        "xcopilot/terrain/gen", xplmType_Int, 0,
        ReadTerrainGen, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_roll_dref = XPLMRegisterDataAccessor(
        "xcopilot/roll", xplmType_Float, 1,
        nullptr, nullptr, ReadRoll, WriteRoll, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_roll_yield_dref = XPLMRegisterDataAccessor(
        "xcopilot/roll_yield", xplmType_Int, 0,
        ReadRollYield, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_yoke_method_dref = XPLMRegisterDataAccessor(
        "xcopilot/yoke_method", xplmType_Int, 1,
        ReadYokeMethod, WriteYokeMethod, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_yoke_roll = XPLMFindDataRef("sim/cockpit2/controls/yoke_roll_ratio");
    g_override_roll = XPLMFindDataRef("sim/operation/override/override_joystick_roll");
    g_yoke_heading = XPLMFindDataRef("sim/cockpit2/controls/yoke_heading_ratio");
    g_override_heading = XPLMFindDataRef("sim/operation/override/override_joystick_heading");
    g_joy_axis_values = XPLMFindDataRef("sim/joystick/joystick_axis_values");
    g_joy_axis_assignments = XPLMFindDataRef("sim/joystick/joystick_axis_assignments");
    XPLMRegisterFlightLoopCallback(TaxiAssistLoop, -1.0f, nullptr);
    tiller::start();
    StartPosition();
    StartState();
}

void stop_first() {
    StopState();
    StopPosition();
    XPLMUnregisterFlightLoopCallback(TaxiAssistLoop, nullptr);
    // The tiller lets go first: it may be holding an airframe option.
    tiller::stop();
    // Leave no brake behind: if the loop was mid-apply, release the ratios
    // and drop the override ourselves — nobody else will after the callback
    // is gone.
    if (g_taxi_brake_applied) {
        if (g_l_brake_ratio) XPLMSetDataf(g_l_brake_ratio, 0.0f);
        if (g_r_brake_ratio) XPLMSetDataf(g_r_brake_ratio, 0.0f);
        if (g_override_toe_brakes) XPLMSetDatai(g_override_toe_brakes, 0);
        g_taxi_brake_applied = false;
    }
    // And no steering: same cleanup, same reason.
    if (g_taxi_steer_applied) {
        if (g_yoke_heading) XPLMSetDataf(g_yoke_heading, 0.0f);
        if (g_override_heading) XPLMSetDatai(g_override_heading, 0);
        g_taxi_steer_applied = false;
    }
    // And no elevator or aileron, each released on the route it was held.
    if (g_pitch_applied) {
        g_pitch_applied = false;
        if (g_pitch_route == kYokeZiboJoy) {
            joy_release(g_joy_pitch);
        } else {
            if (g_yoke_pitch) XPLMSetDataf(g_yoke_pitch, 0.0f);
            if (g_override_pitch) XPLMSetDatai(g_override_pitch, 0);
        }
    }
    if (g_roll_applied) {
        g_roll_applied = false;
        if (g_roll_route == kYokeZiboJoy) {
            joy_release(g_joy_roll);
        } else {
            if (g_yoke_roll) XPLMSetDataf(g_yoke_roll, 0.0f);
            if (g_override_roll) XPLMSetDatai(g_override_roll, 0);
        }
    }
    if (g_cmd_a_press) { XPLMUnregisterCommandHandler(g_cmd_a_press, EngagePressHandler, 1, nullptr); g_cmd_a_press = nullptr; }
    if (g_cmd_b_press) { XPLMUnregisterCommandHandler(g_cmd_b_press, EngagePressHandler, 1, nullptr); g_cmd_b_press = nullptr; }
    // The loop is gone: no centred hold can count down, so drop it now.
    if (g_joy_drop_frames > 0) joy_drop_now();
    if (g_taxi_steer_dref) { XPLMUnregisterDataAccessor(g_taxi_steer_dref); g_taxi_steer_dref = nullptr; }
    if (g_steer_yield_dref) { XPLMUnregisterDataAccessor(g_steer_yield_dref); g_steer_yield_dref = nullptr; }
    if (g_brake_yield_dref) { XPLMUnregisterDataAccessor(g_brake_yield_dref); g_brake_yield_dref = nullptr; }
    for (int i = 0; i < 4; ++i) {
        if (g_brake_cmds[i]) {
            XPLMUnregisterCommandHandler(g_brake_cmds[i], BrakeCommandHandler, 1, nullptr);
            g_brake_cmds[i] = nullptr;
        }
    }
    if (g_pitch_dref) { XPLMUnregisterDataAccessor(g_pitch_dref); g_pitch_dref = nullptr; }
    if (g_pitch_yield_dref) { XPLMUnregisterDataAccessor(g_pitch_yield_dref); g_pitch_yield_dref = nullptr; }
    if (g_terrain_query_dref) { XPLMUnregisterDataAccessor(g_terrain_query_dref); g_terrain_query_dref = nullptr; }
    if (g_terrain_elev_dref) { XPLMUnregisterDataAccessor(g_terrain_elev_dref); g_terrain_elev_dref = nullptr; }
    if (g_terrain_valid_dref) { XPLMUnregisterDataAccessor(g_terrain_valid_dref); g_terrain_valid_dref = nullptr; }
    if (g_terrain_gen_dref) { XPLMUnregisterDataAccessor(g_terrain_gen_dref); g_terrain_gen_dref = nullptr; }
    if (g_terrain_probe) { XPLMDestroyProbe(g_terrain_probe); g_terrain_probe = nullptr; }
    if (g_roll_dref) { XPLMUnregisterDataAccessor(g_roll_dref); g_roll_dref = nullptr; }
    if (g_roll_yield_dref) { XPLMUnregisterDataAccessor(g_roll_yield_dref); g_roll_yield_dref = nullptr; }
    if (g_yoke_method_dref) { XPLMUnregisterDataAccessor(g_yoke_method_dref); g_yoke_method_dref = nullptr; }
    if (g_taxi_brake_dref) { XPLMUnregisterDataAccessor(g_taxi_brake_dref); g_taxi_brake_dref = nullptr; }
    if (g_steer_brake_l_dref) { XPLMUnregisterDataAccessor(g_steer_brake_l_dref); g_steer_brake_l_dref = nullptr; }
    if (g_steer_brake_r_dref) { XPLMUnregisterDataAccessor(g_steer_brake_r_dref); g_steer_brake_r_dref = nullptr; }
    if (g_typed_dref) { XPLMUnregisterDataAccessor(g_typed_dref); g_typed_dref = nullptr; }
    if (g_typed_age_dref) { XPLMUnregisterDataAccessor(g_typed_age_dref); g_typed_age_dref = nullptr; }
    if (g_opt_text_dref) { XPLMUnregisterDataAccessor(g_opt_text_dref); g_opt_text_dref = nullptr; }
    if (g_opt_text_edit_dref) { XPLMUnregisterDataAccessor(g_opt_text_edit_dref); g_opt_text_edit_dref = nullptr; }
    if (g_callsign_dref) { XPLMUnregisterDataAccessor(g_callsign_dref); g_callsign_dref = nullptr; }
    if (g_fromto_dref) { XPLMUnregisterDataAccessor(g_fromto_dref); g_fromto_dref = nullptr; }
    if (g_opt_ui_scale_dref) { XPLMUnregisterDataAccessor(g_opt_ui_scale_dref); g_opt_ui_scale_dref = nullptr; }
    if (g_loadsheet_dref) { XPLMUnregisterDataAccessor(g_loadsheet_dref); g_loadsheet_dref = nullptr; }
    if (g_brief_dref) { XPLMUnregisterDataAccessor(g_brief_dref); g_brief_dref = nullptr; }
    if (g_state_dref) { XPLMUnregisterDataAccessor(g_state_dref); g_state_dref = nullptr; }
    if (g_board_dref) { XPLMUnregisterDataAccessor(g_board_dref); g_board_dref = nullptr; }
    if (g_board_field_dref) { XPLMUnregisterDataAccessor(g_board_field_dref); g_board_field_dref = nullptr; }
    if (g_auto_dref) { XPLMUnregisterDataAccessor(g_auto_dref); g_auto_dref = nullptr; }
    if (g_coach_dref) { XPLMUnregisterDataAccessor(g_coach_dref); g_coach_dref = nullptr; }
    if (g_atc_dref) { XPLMUnregisterDataAccessor(g_atc_dref); g_atc_dref = nullptr; }
    if (g_radio_dref) { XPLMUnregisterDataAccessor(g_radio_dref); g_radio_dref = nullptr; }
    if (g_callout_dref) { XPLMUnregisterDataAccessor(g_callout_dref); g_callout_dref = nullptr; }
}

void stop_rest() {
    if (g_voice_list_dref) { XPLMUnregisterDataAccessor(g_voice_list_dref); g_voice_list_dref = nullptr; }
    if (g_opt_cabin_dref) { XPLMUnregisterDataAccessor(g_opt_cabin_dref); g_opt_cabin_dref = nullptr; }
    if (g_opt_mode_dref) { XPLMUnregisterDataAccessor(g_opt_mode_dref); g_opt_mode_dref = nullptr; }
    if (g_opt_voice_dref) { XPLMUnregisterDataAccessor(g_opt_voice_dref); g_opt_voice_dref = nullptr; }
    if (g_opt_cabin_voice_dref) { XPLMUnregisterDataAccessor(g_opt_cabin_voice_dref); g_opt_cabin_voice_dref = nullptr; }
    if (g_opt_mute_dref) { XPLMUnregisterDataAccessor(g_opt_mute_dref); g_opt_mute_dref = nullptr; }
    if (g_opt_tone_dref) { XPLMUnregisterDataAccessor(g_opt_tone_dref); g_opt_tone_dref = nullptr; }
    if (g_opt_snap_dref) { XPLMUnregisterDataAccessor(g_opt_snap_dref); g_opt_snap_dref = nullptr; }
    if (g_taxi_dref) { XPLMUnregisterDataAccessor(g_taxi_dref); g_taxi_dref = nullptr; }
    if (g_strip_dref) { XPLMUnregisterDataAccessor(g_strip_dref); g_strip_dref = nullptr; }
    if (g_request_dref) { XPLMUnregisterDataAccessor(g_request_dref); g_request_dref = nullptr; }
    audio::unregister_datarefs();
    flightinit::stop();
    fms::unregister_datarefs();
    if (g_aircraft_dref) { XPLMUnregisterDataAccessor(g_aircraft_dref); g_aircraft_dref = nullptr; }
    if (g_alert_dref) { XPLMUnregisterDataAccessor(g_alert_dref); g_alert_dref = nullptr; }
    if (g_action_dref) { XPLMUnregisterDataAccessor(g_action_dref); g_action_dref = nullptr; }
    if (g_heard_dref) { XPLMUnregisterDataAccessor(g_heard_dref); g_heard_dref = nullptr; }
    if (g_phase_dref) { XPLMUnregisterDataAccessor(g_phase_dref); g_phase_dref = nullptr; }
    if (g_ptt_dref) { XPLMUnregisterDataAccessor(g_ptt_dref); g_ptt_dref = nullptr; }
    if (g_ptt_cmd) { XPLMUnregisterCommandHandler(g_ptt_cmd, PttCommandHandler, 1, nullptr); g_ptt_cmd = nullptr; }
    if (g_radio_ptt_dref) { XPLMUnregisterDataAccessor(g_radio_ptt_dref); g_radio_ptt_dref = nullptr; }
    if (g_radio_ptt_cmd) { XPLMUnregisterCommandHandler(g_radio_ptt_cmd, RadioPttCommandHandler, 1, nullptr); g_radio_ptt_cmd = nullptr; }
}

bool steer_yielded() { return g_steer_yield != 0; }
void yield_steering() { g_steer_yield = 1; }

bool app_connected() {
    // The app heartbeats the phase ~1/s; a stale timestamp = not running.
    return (XPLMGetElapsedTime() - g_last_status_time) < 3.0f;
}

int phase() { return g_phase; }
bool ptt_active() { return g_ptt_active != 0; }
bool radio_ptt_active() { return g_radio_ptt_active != 0; }
const TextRef& aircraft() { return g_aircraft; }
const TextRef& strip() { return g_strip; }
const TextRef& taxi_card() { return g_taxi; }
float heading_true() { return g_psi_dref ? XPLMGetDataf(g_psi_dref) : 0.0f; }
const TextRef& radio_row() { return g_radio; }
const TextRef& coach() { return g_coach; }
bool coach_urgent() { return g_coach_urgent; }
char coach_group() { return g_coach_group; }
const TextRef& auto_status() { return g_auto; }
const TextRef& load_sheet() { return g_loadsheet; }
const TextRef& brief() { return g_brief; }
const TextRef& state_lines() { return g_state; }
const TextRef& board() { return g_board; }
void set_board_field(const char* icao) {
    size_t n = std::strlen(icao);
    if (n > sizeof g_board_field.buf - 1) n = sizeof g_board_field.buf - 1;
    std::memcpy(g_board_field.buf, icao, n);
    g_board_field.buf[n] = '\0';
    g_board_field.len = static_cast<int>(n);
}
bool board_fresh() {
    return g_board_at >= 0.0 && XPLMGetElapsedTime() - g_board_at < 30.0;
}
bool auto_active() { return g_auto_active; }
const std::vector<AtcField>& atc_fields() { return g_atc_fields; }

int com1_khz() { return g_com1_dref ? XPLMGetDatai(g_com1_dref) : 0; }
void tune_com1(int khz) { if (g_com1_dref) XPLMSetDatai(g_com1_dref, khz); }

int request() { return g_request; }
void set_request(int v) { g_request = v; }

int opt_camera_snap() { return g_opt_camera_snap; }
void set_opt_camera_snap(int v) { g_opt_camera_snap = v; }
int opt_negative_tone() { return g_opt_negative_tone; }
void set_opt_negative_tone(int v) { g_opt_negative_tone = v; }
int opt_monitor_mute() { return g_opt_monitor_mute; }
void set_opt_monitor_mute(int v) { g_opt_monitor_mute = v; }
int opt_mode() { return g_opt_mode; }
void set_opt_mode(int v) { g_opt_mode = v; }
int opt_cabin() { return g_opt_cabin; }
void set_opt_cabin(int v) { g_opt_cabin = v; }
int opt_ui_scale() { return g_opt_ui_scale; }
void set_opt_ui_scale(int v) { g_opt_ui_scale = v; }
void set_typed(const char* text) {
    std::memset(g_typed.buf, 0, sizeof g_typed.buf);
    std::strncpy(g_typed.buf, text, sizeof g_typed.buf - 1);
    g_typed.len = static_cast<int>(std::strlen(g_typed.buf));
    StampTyped();
}
const TextRef& opt_text() { return g_opt_text; }
const TextRef& callsign_status() { return g_callsign; }
bool send_text_edit(const char* key, const char* value) {
    char line[sizeof g_opt_text_edit.buf];
    int n = std::snprintf(line, sizeof line, "%s=%s", key, value);
    if (n < 0 || n >= static_cast<int>(sizeof line)) return false;
    // Append behind an edit the app has not consumed yet; one that does not
    // fit is refused rather than truncated.
    int at = g_opt_text_edit.len;
    int need = at + (at > 0 ? 1 : 0) + n;
    if (need >= static_cast<int>(sizeof g_opt_text_edit.buf)) return false;
    if (at > 0) g_opt_text_edit.buf[at++] = '\n';
    std::memcpy(g_opt_text_edit.buf + at, line, static_cast<size_t>(n));
    g_opt_text_edit.len = at + n;
    g_opt_text_edit.buf[g_opt_text_edit.len] = '\0';
    return true;
}
int opt_voice() { return g_opt_voice; }
void set_opt_voice(int v) { g_opt_voice = v; }
int opt_cabin_voice() { return g_opt_cabin_voice; }
void set_opt_cabin_voice(int v) { g_opt_cabin_voice = v; }
const char* voice_list() { return g_voice_list.buf; }
int voice_list_len() { return g_voice_list.len; }

} // namespace drefs
