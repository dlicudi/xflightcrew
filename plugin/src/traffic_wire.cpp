// The traffic wire decoder; the layout is in traffic_wire.h.

#include "traffic_wire.h"

#include <cmath>
#include <cstring>

namespace traffic_wire {

namespace {

// Byte-by-byte little-endian reads: correct on any host, no alignment needs.
uint16_t u16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
int16_t i16(const uint8_t* p) { return static_cast<int16_t>(u16(p)); }
uint32_t u32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
uint64_t u64(const uint8_t* p) {
    return static_cast<uint64_t>(u32(p)) | (static_cast<uint64_t>(u32(p + 4)) << 32);
}
float f32(const uint8_t* p) {
    const uint32_t b = u32(p);
    float f;
    std::memcpy(&f, &b, sizeof f);
    return f;
}
double f64(const uint8_t* p) {
    const uint64_t b = u64(p);
    double d;
    std::memcpy(&d, &b, sizeof d);
    return d;
}
std::string str(const uint8_t* p, size_t n) {
    size_t len = 0;
    while (len < n && p[len] != 0) ++len;
    return std::string(reinterpret_cast<const char*>(p), len);
}
float ratio(uint8_t b) { return static_cast<float>(b) / 255.0f; }

} // namespace

const char* error_name(Error e) {
    switch (e) {
        case Error::None: return "none";
        case Error::Short: return "short";
        case Error::Version: return "version";
        case Error::Value: return "value";
    }
    return "?";
}

Error decode_roster(const uint8_t* buf, size_t len, Roster& out) {
    if (!buf || len < kRosterHeader) return Error::Short;
    if (u16(buf) != kVersion) return Error::Version;
    const size_t count = u16(buf + 2);
    if (len < kRosterHeader + count * kRosterEntry) return Error::Short;
    Roster r;
    r.seq = u32(buf + 4);
    r.entries.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        const uint8_t* e = buf + kRosterHeader + i * kRosterEntry;
        RosterEntry re;
        re.id = u32(e);
        re.mode_s = u32(e + 4);
        re.squawk = u16(e + 8);
        re.type = str(e + 12, 4);
        re.airline = str(e + 16, 4);
        re.livery = str(e + 20, 16);
        re.callsign = str(e + 36, 12);
        r.entries.push_back(std::move(re));
    }
    out = std::move(r);
    return Error::None;
}

Error decode_frames(const uint8_t* buf, size_t len, Frames& out) {
    if (!buf || len < kFrameHeader) return Error::Short;
    if (u16(buf) != kVersion) return Error::Version;
    const size_t count = u16(buf + 2);
    if (len < kFrameHeader + count * kFrameEntry) return Error::Short;
    Frames f;
    f.seq = u32(buf + 4);
    f.sim_time_s = f64(buf + 8);
    if (!std::isfinite(f.sim_time_s)) return Error::Value;
    f.entries.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        const uint8_t* e = buf + kFrameHeader + i * kFrameEntry;
        FrameEntry fe;
        fe.id = u32(e);
        fe.alt_ft = f32(e + 4);
        fe.lat = f64(e + 8);
        fe.lon = f64(e + 16);
        if (!std::isfinite(fe.lat) || !std::isfinite(fe.lon) || !std::isfinite(fe.alt_ft) ||
            std::fabs(fe.lat) > 90.0 || std::fabs(fe.lon) > 180.0) {
            return Error::Value;
        }
        fe.hdg_deg = static_cast<float>(u16(e + 24) % 36000) / 100.0f;
        fe.pitch_deg = static_cast<float>(i16(e + 26)) / 100.0f;
        fe.roll_deg = static_cast<float>(i16(e + 28)) / 100.0f;
        fe.gs_kts = static_cast<float>(u16(e + 30)) / 10.0f;
        fe.vs_fpm = static_cast<float>(i16(e + 32));
        fe.gear = ratio(e[34]);
        fe.flaps = ratio(e[35]);
        fe.spoiler = ratio(e[36]);
        fe.reverser = ratio(e[37]);
        fe.lights = e[38];
        fe.flags = e[39];
        f.entries.push_back(fe);
    }
    out = std::move(f);
    return Error::None;
}

} // namespace traffic_wire
