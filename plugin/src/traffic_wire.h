// The traffic wire: the bytes the app writes into xcopilot/traffic/roster and
// xcopilot/traffic/frames, decoded here. Pure C++ (no SDK, no XPMP2) so the
// host-side test (`make test`) compiles it alone.
//
// Every integer and float is LITTLE-ENDIAN; floats are IEEE 754. Both buffers
// open with a u16 version (kVersion) and a u16 count. A buffer with another
// version, shorter than its header plus count entries, or carrying a
// non-finite position is refused whole (the caller counts it); trailing bytes
// past the last entry are ignored.
//
// Roster (on change), header 8 bytes then `count` entries of 48 bytes:
//   0  u16 version      2  u16 count      4  u32 seq (the roster's own counter)
//   entry:
//   0  u32 id           4  u32 mode_s (24 bit, 0 = let XPMP2 pick)
//   8  u16 squawk (the four octal digits as a decimal number, 7000 = 7000)
//  10  u16 reserved (0)
//  12  char[4]  ICAO type      ("C172", "A320")
//  16  char[4]  ICAO airline   ("EZY", empty for GA)
//  20  char[16] livery         (empty = any)
//  36  char[12] callsign       (the label, "G-ABCD", "EZY12AB")
//   strings are NUL-padded and need not be NUL-terminated when full.
//
// Frames (4 Hz), header 16 bytes then `count` entries of 40 bytes:
//   0  u16 version      2  u16 count      4  u32 seq      8  f64 sim time (s,
//                                           sim/time/total_running_time_sec)
//   entry:
//   0  u32 id           4  f32 altitude ft MSL
//   8  f64 latitude deg 16 f64 longitude deg
//  24  u16 true heading, centidegrees (0..35999)
//  26  i16 pitch, centidegrees (nose up +)
//  28  i16 roll, centidegrees (right wing down +)
//  30  u16 groundspeed, tenths of a knot
//  32  i16 vertical speed, ft/min
//  34  u8  gear ratio     35 u8 flap ratio     36 u8 spoiler ratio
//  37  u8  reverser ratio (each 0..255 = 0..1)
//  38  u8  lights (bit 0 nav, 1 beacon, 2 strobe, 3 landing, 4 taxi)
//  39  u8  flags  (bit 0 on the ground, 1 engines running, 2 hidden)

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace traffic_wire {

constexpr uint16_t kVersion = 1;
constexpr size_t kBufBytes = 16384;

constexpr size_t kRosterHeader = 8;
constexpr size_t kRosterEntry = 48;
constexpr size_t kFrameHeader = 16;
constexpr size_t kFrameEntry = 40;

constexpr uint8_t kLightNav = 1u << 0;
constexpr uint8_t kLightBeacon = 1u << 1;
constexpr uint8_t kLightStrobe = 1u << 2;
constexpr uint8_t kLightLanding = 1u << 3;
constexpr uint8_t kLightTaxi = 1u << 4;

constexpr uint8_t kFlagOnGround = 1u << 0;
constexpr uint8_t kFlagEngines = 1u << 1;
constexpr uint8_t kFlagHidden = 1u << 2;

struct RosterEntry {
    uint32_t id = 0;
    uint32_t mode_s = 0;
    uint16_t squawk = 0;
    std::string type, airline, livery, callsign;
};

struct Roster {
    uint32_t seq = 0;
    std::vector<RosterEntry> entries;
};

// One aircraft in one frame, in engineering units.
struct FrameEntry {
    uint32_t id = 0;
    double lat = 0.0, lon = 0.0;
    float alt_ft = 0.0f;
    float hdg_deg = 0.0f, pitch_deg = 0.0f, roll_deg = 0.0f;
    float gs_kts = 0.0f, vs_fpm = 0.0f;
    float gear = 0.0f, flaps = 0.0f, spoiler = 0.0f, reverser = 0.0f;
    uint8_t lights = 0, flags = 0;
};

struct Frames {
    uint32_t seq = 0;
    double sim_time_s = 0.0;
    std::vector<FrameEntry> entries;
};

enum class Error { None, Short, Version, Value };
const char* error_name(Error e);

Error decode_roster(const uint8_t* buf, size_t len, Roster& out);
Error decode_frames(const uint8_t* buf, size_t len, Frames& out);

} // namespace traffic_wire
