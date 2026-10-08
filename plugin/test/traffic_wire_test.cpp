// Host-side test of the traffic wire decoder (`make test`): no SDK, no XPMP2.
//
// The fixtures are literal bytes laid out by hand from traffic_wire.h (no
// encoder here, so the test cannot agree with itself). The app's own golden
// bytes (the spec's "Wire (as built)") belong beside them once written: the
// same bytes must decode the same on both sides.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "traffic_wire.h"

using namespace traffic_wire;

static int g_failed = 0;
#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++g_failed;                                                   \
        }                                                                 \
    } while (0)

static bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// Two aircraft: a C172 G-ABCD (mode S 4CA123, squawk 7000) and an A320
// EZY12ABCDEFG (a full 12-byte callsign with no NUL), roster seq 7.
static const uint8_t kRoster[] = {
    0x01, 0x00, 0x02, 0x00, 0x07, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x23, 0xa1, 0x4c, 0x00,
    0x58, 0x1b, 0x00, 0x00, 0x43, 0x31, 0x37, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x2d, 0x41, 0x42,
    0x43, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x44, 0x64, 0x3c, 0x00,
    0xd2, 0x04, 0x00, 0x00, 0x41, 0x33, 0x32, 0x30, 0x45, 0x5a, 0x59, 0x00, 0x45, 0x5a, 0x59, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x45, 0x5a, 0x59, 0x31,
    0x32, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
};

// One aircraft, frame seq 812 at sim time 3605.25 s: id 1 at 36.6749 N
// 4.4991 W, 1523.5 ft, heading 130.50, pitch -2.50, roll 15.00, 85.3 kt,
// -650 fpm, gear 255, flaps 128, lights nav|beacon|landing|taxi, flags on
// the ground|engines.
static const uint8_t kFrames[] = {
    0x01, 0x00, 0x01, 0x00, 0x2c, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x2a, 0xac, 0x40,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x70, 0xbe, 0x44, 0x03, 0x09, 0x8a, 0x1f, 0x63, 0x56, 0x42, 0x40,
    0x02, 0xbc, 0x05, 0x12, 0x14, 0xff, 0x11, 0xc0, 0xfa, 0x32, 0x06, 0xff, 0xdc, 0x05, 0x55, 0x03,
    0x76, 0xfd, 0xff, 0x80, 0x00, 0x00, 0x1b, 0x03,
};

static void roster_decodes() {
    static_assert(sizeof kRoster == kRosterHeader + 2 * kRosterEntry, "roster fixture size");
    Roster r;
    CHECK(decode_roster(kRoster, sizeof kRoster, r) == Error::None);
    CHECK(r.seq == 7);
    CHECK(r.entries.size() == 2);
    if (r.entries.size() != 2) return;
    const RosterEntry& a = r.entries[0];
    CHECK(a.id == 1);
    CHECK(a.mode_s == 0x4CA123);
    CHECK(a.squawk == 7000);
    CHECK(a.type == "C172");
    CHECK(a.airline.empty());
    CHECK(a.livery.empty());
    CHECK(a.callsign == "G-ABCD");
    const RosterEntry& b = r.entries[1];
    CHECK(b.id == 2);
    CHECK(b.mode_s == 0x3C6444);
    CHECK(b.squawk == 1234);
    CHECK(b.type == "A320");
    CHECK(b.airline == "EZY");
    CHECK(b.livery == "EZY");
    CHECK(b.callsign == "EZY12ABCDEFG");
}

static void frames_decode() {
    static_assert(sizeof kFrames == kFrameHeader + kFrameEntry, "frames fixture size");
    Frames f;
    CHECK(decode_frames(kFrames, sizeof kFrames, f) == Error::None);
    CHECK(f.seq == 812);
    CHECK(f.sim_time_s == 3605.25);
    CHECK(f.entries.size() == 1);
    if (f.entries.size() != 1) return;
    const FrameEntry& e = f.entries[0];
    CHECK(e.id == 1);
    CHECK(e.lat == 36.6749);
    CHECK(e.lon == -4.4991);
    CHECK(e.alt_ft == 1523.5f);
    CHECK(near(e.hdg_deg, 130.5, 1e-4));
    CHECK(near(e.pitch_deg, -2.5, 1e-4));
    CHECK(near(e.roll_deg, 15.0, 1e-4));
    CHECK(near(e.gs_kts, 85.3, 1e-4));
    CHECK(e.vs_fpm == -650.0f);
    CHECK(e.gear == 1.0f);
    CHECK(near(e.flaps, 128.0 / 255.0, 1e-6));
    CHECK(e.spoiler == 0.0f);
    CHECK(e.reverser == 0.0f);
    CHECK(e.lights == (kLightNav | kLightBeacon | kLightLanding | kLightTaxi));
    CHECK(e.flags == (kFlagOnGround | kFlagEngines));
}

static void refusals() {
    Roster r;
    Frames f;
    // Short: below the header, and one byte short of the last entry.
    CHECK(decode_roster(kRoster, 7, r) == Error::Short);
    CHECK(decode_roster(kRoster, sizeof kRoster - 1, r) == Error::Short);
    CHECK(decode_frames(kFrames, 15, f) == Error::Short);
    CHECK(decode_frames(kFrames, sizeof kFrames - 1, f) == Error::Short);
    CHECK(decode_frames(nullptr, 0, f) == Error::Short);
    // A refused buffer leaves the last good decode alone.
    CHECK(decode_frames(kFrames, sizeof kFrames, f) == Error::None);
    CHECK(decode_frames(kFrames, 3, f) == Error::Short);
    CHECK(f.seq == 812 && f.entries.size() == 1);
    // Wrong version.
    std::vector<uint8_t> v(kFrames, kFrames + sizeof kFrames);
    v[0] = 2;
    CHECK(decode_frames(v.data(), v.size(), f) == Error::Version);
    std::vector<uint8_t> rv(kRoster, kRoster + sizeof kRoster);
    rv[1] = 1;
    CHECK(decode_roster(rv.data(), rv.size(), r) == Error::Version);
    // A count claiming more than the buffer holds.
    std::vector<uint8_t> c(kFrames, kFrames + sizeof kFrames);
    c[2] = 0xff;
    c[3] = 0xff;
    CHECK(decode_frames(c.data(), c.size(), f) == Error::Short);
    // A latitude past the pole (f64 at entry offset 8 set to 1e9).
    std::vector<uint8_t> lat(kFrames, kFrames + sizeof kFrames);
    const double big = 1e9;
    std::memcpy(&lat[kFrameHeader + 8], &big, sizeof big);  // the test host is little-endian
    CHECK(decode_frames(lat.data(), lat.size(), f) == Error::Value);
    // Trailing bytes past the last entry are ignored; an empty roster is valid.
    std::vector<uint8_t> t(kFrames, kFrames + sizeof kFrames);
    t.push_back(0xaa);
    CHECK(decode_frames(t.data(), t.size(), f) == Error::None);
    const uint8_t empty[] = {0x01, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00};
    CHECK(decode_roster(empty, sizeof empty, r) == Error::None);
    CHECK(r.seq == 9 && r.entries.empty());
}

// The app's golden bytes (`traffic::wire::tests::the_golden_bytes`, the spec's
// "Wire (as built)"): what the Rust encoder writes must decode to what it was given.
static std::vector<uint8_t> from_hex(const char* h) {
    std::vector<uint8_t> v;
    for (size_t i = 0; h[i] && h[i + 1]; i += 2) {
        unsigned b = 0;
        std::sscanf(h + i, "%2x", &b);
        v.push_back(static_cast<uint8_t>(b));
    }
    return v;
}

static void app_golden_bytes_decode() {
    const auto ros = from_hex(
        "01000200070000000100000000000000581b0000433137320000000000000000000000000000000000000000"
        "472d4142434400000000000002010000efcdab002509000041333230455a5900455a59000000000000000000"
        "00000000455a59313241420000000000");
    CHECK(ros.size() == 104);
    Roster r;
    CHECK(decode_roster(ros.data(), ros.size(), r) == Error::None);
    CHECK(r.seq == 7);
    CHECK(r.entries.size() == 2);
    if (r.entries.size() == 2) {
        CHECK(r.entries[0].id == 1);
        CHECK(r.entries[0].squawk == 7000);
        CHECK(r.entries[0].type == "C172");
        CHECK(r.entries[0].callsign == "G-ABCD");
        CHECK(r.entries[1].id == 258);
        CHECK(r.entries[1].mode_s == 0xabcdef);
        CHECK(r.entries[1].squawk == 2341);
        CHECK(r.entries[1].type == "A320");
        CHECK(r.entries[1].airline == "EZY");
        CHECK(r.entries[1].livery == "EZY");
        CHECK(r.entries[1].callsign == "EZY12AB");
    }

    const auto fr = from_hex(
        "010002002a00000000000000004a93400100000000007a44000000000040424000000000000011c02823fa00"
        "18fc57030cfeff8000000f02020100000000c8410000000000e040c00000000000e462409f8c000000000000"
        "0000ff00ffff1103");
    CHECK(fr.size() == 96);
    Frames f;
    CHECK(decode_frames(fr.data(), fr.size(), f) == Error::None);
    CHECK(f.seq == 42);
    CHECK(f.sim_time_s == 1234.5);
    CHECK(f.entries.size() == 2);
    if (f.entries.size() == 2) {
        const FrameEntry& a = f.entries[0];
        CHECK(a.id == 1);
        CHECK(a.alt_ft == 1000.0f);
        CHECK(a.lat == 36.5);
        CHECK(a.lon == -4.25);
        CHECK(near(a.hdg_deg, 90.0, 1e-4));
        CHECK(near(a.pitch_deg, 2.5, 1e-4));
        CHECK(near(a.roll_deg, -10.0, 1e-4));
        CHECK(near(a.gs_kts, 85.5, 1e-4));
        CHECK(a.vs_fpm == -500.0f);
        CHECK(a.gear == 1.0f);
        CHECK(near(a.flaps, 128.0 / 255.0, 1e-6));
        CHECK(a.lights == 0x0f);
        CHECK(a.flags == kFlagEngines);
        const FrameEntry& b = f.entries[1];
        CHECK(b.id == 258);
        CHECK(b.alt_ft == 25.0f);
        CHECK(b.lat == -33.75);
        CHECK(b.lon == 151.125);
        CHECK(near(b.hdg_deg, 359.99, 1e-3));
        CHECK(b.spoiler == 1.0f);
        CHECK(b.reverser == 1.0f);
        CHECK(b.lights == (kLightNav | kLightTaxi));
        CHECK(b.flags == (kFlagOnGround | kFlagEngines));
    }
}

int main() {
    roster_decodes();
    frames_decode();
    refusals();
    app_golden_bytes_decode();
    if (g_failed) {
        std::fprintf(stderr, "traffic_wire: %d check(s) failed\n", g_failed);
        return 1;
    }
    std::printf("traffic_wire: all checks passed\n");
    return 0;
}
