#include "audio.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "XPLMDataAccess.h"
#include "XPLMSound.h"
#include "XPLMUtilities.h"

#include "common.h"
#include "drefs.h"

namespace {

// The two logical channels the app speaks on, and the bus each rides.
constexpr int kChannels = 2;
constexpr XPLMAudioBus kBus[kChannels] = {xplm_AudioRadioCopilot, xplm_AudioRadioCom1};

TextRef g_play;                 // "ch|vol|path", written by the app
bool g_play_pending = false;
// Settings tab switch, polled by the app; -1 until the app publishes its
// truth (the poller's reload sentinel, like the other opt datarefs).
int g_opt_audio_bus = -1;
constexpr float kMaxVolume = 4.0f;

FMOD_CHANNEL* g_active[kChannels] = {nullptr, nullptr};
void* g_active_buf[kChannels] = {nullptr, nullptr};

XPLMDataRef g_play_dref = nullptr;
XPLMDataRef g_cut_dref = nullptr;
XPLMDataRef g_opt_dref = nullptr;

void OnComplete(void* refcon, FMOD_RESULT) {
    // The sample finished or was stopped: its buffer goes, and any channel
    // pointer we kept for it is dead now.
    for (int i = 0; i < kChannels; ++i) {
        if (g_active_buf[i] == refcon) {
            g_active_buf[i] = nullptr;
            g_active[i] = nullptr;
        }
    }
    std::free(refcon);
}

void stop_channel(int ch) {
    if (ch < 0 || ch >= kChannels || g_active[ch] == nullptr) return;
    FMOD_CHANNEL* c = g_active[ch];
    g_active[ch] = nullptr;      // invalid after the stop; the callback frees the buffer
    XPLMStopAudio(c);
}

// Minimal RIFF/WAVE reader for what the app writes: PCM16, one channel.
// Returns a malloc'd sample buffer (freed in OnComplete) or nullptr.
void* read_wav(const std::string& path, int* rate, int* channels, int* bytes) {
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return nullptr;
    char id[4];
    unsigned char hdr[12];
    if (std::fread(hdr, 1, 12, f) != 12 || std::memcmp(hdr, "RIFF", 4) != 0 ||
        std::memcmp(hdr + 8, "WAVE", 4) != 0) {
        std::fclose(f);
        return nullptr;
    }
    int fmt_tag = 0, fmt_ch = 0, fmt_rate = 0, fmt_bits = 0;
    void* data = nullptr;
    int data_len = 0;
    unsigned char sz[4];
    while (std::fread(id, 1, 4, f) == 4 && std::fread(sz, 1, 4, f) == 4) {
        const int len = sz[0] | (sz[1] << 8) | (sz[2] << 16) | (sz[3] << 24);
        if (std::memcmp(id, "fmt ", 4) == 0 && len >= 16) {
            unsigned char b[16];
            if (std::fread(b, 1, 16, f) != 16) break;
            fmt_tag = b[0] | (b[1] << 8);
            fmt_ch = b[2] | (b[3] << 8);
            fmt_rate = b[4] | (b[5] << 8) | (b[6] << 16) | (b[7] << 24);
            fmt_bits = b[14] | (b[15] << 8);
            if (len > 16) std::fseek(f, len - 16, SEEK_CUR);
        } else if (std::memcmp(id, "data", 4) == 0) {
            if (len <= 0 || len > 64 * 1024 * 1024) break;
            data = std::malloc(static_cast<size_t>(len));
            if (!data) break;
            data_len = static_cast<int>(std::fread(data, 1, static_cast<size_t>(len), f));
            break;
        } else {
            std::fseek(f, len + (len & 1), SEEK_CUR);
        }
    }
    std::fclose(f);
    if (!data || fmt_tag != 1 || fmt_bits != 16 || fmt_ch < 1 || fmt_rate <= 0) {
        std::free(data);
        return nullptr;
    }
    *rate = fmt_rate;
    *channels = fmt_ch;
    *bytes = data_len;
    return data;
}

void play_pending() {
    char buf[sizeof g_play.buf];
    std::memcpy(buf, g_play.buf, sizeof buf);
    buf[sizeof buf - 1] = 0;
    // "ch|vol|path" — the path may hold anything but '|'.
    int ch = 0;
    float vol = 1.0f;
    char path[512] = {0};
    if (std::sscanf(buf, "%d|%f|%511[^\n]", &ch, &vol, path) != 3) {
        XPLMDebugString("XFlightCrew plugin: audio/play payload not understood\n");
        return;
    }
    if (ch < 0 || ch >= kChannels) return;
    int rate = 0, channels = 0, bytes = 0;
    void* pcm = read_wav(path, &rate, &channels, &bytes);
    if (!pcm) {
        char msg[640];
        std::snprintf(msg, sizeof msg, "XFlightCrew plugin: audio/play could not read %s\n", path);
        XPLMDebugString(msg);
        return;
    }
    // One voice per channel: whatever was still playing yields.
    stop_channel(ch);
    FMOD_CHANNEL* c = XPLMPlayPCMOnBus(pcm, bytes, FMOD_SOUND_FORMAT_PCM16, rate, channels, 0,
                                       kBus[ch], OnComplete, pcm);
    if (!c) {
        std::free(pcm);
        XPLMDebugString("XFlightCrew plugin: XPLMPlayPCMOnBus refused the sample\n");
        return;
    }
    if (vol < 0.0f) vol = 0.0f;
    if (vol > kMaxVolume) vol = kMaxVolume;
    XPLMSetAudioVolume(c, vol);
    g_active[ch] = c;
    g_active_buf[ch] = pcm;
}

void WriteAudioPlay(void*, void* in, int offset, int len) {
    StoreText(&g_play, in, offset, len);
    g_play_pending = true;
}

// Write-only, edge-triggered: the channel number to silence; -1 = both.
void WriteCut(void*, int v) {
    if (v < 0) {
        for (int i = 0; i < kChannels; ++i) stop_channel(i);
    } else {
        stop_channel(v);
    }
}

int ReadOpt(void*) { return g_opt_audio_bus; }
void WriteOpt(void*, int v) { g_opt_audio_bus = v; }

} // namespace

namespace audio {

void register_datarefs() {
    g_play_dref = XPLMRegisterDataAccessor(
        "xcopilot/audio/play", xplmType_Data, 1,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, ReadText, WriteAudioPlay, &g_play, &g_play);
    g_cut_dref = XPLMRegisterDataAccessor(
        "xcopilot/audio/cut", xplmType_Int, 1,
        nullptr, WriteCut, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    g_opt_dref = XPLMRegisterDataAccessor(
        "xcopilot/opt/audio_bus", xplmType_Int, 1,
        ReadOpt, WriteOpt, nullptr, nullptr, nullptr, nullptr,
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
}

void unregister_datarefs() {
    for (int i = 0; i < kChannels; ++i) stop_channel(i);
    if (g_opt_dref) { XPLMUnregisterDataAccessor(g_opt_dref); g_opt_dref = nullptr; }
    if (g_cut_dref) { XPLMUnregisterDataAccessor(g_cut_dref); g_cut_dref = nullptr; }
    if (g_play_dref) { XPLMUnregisterDataAccessor(g_play_dref); g_play_dref = nullptr; }
}

void tick() {
    if (g_play_pending) {
        g_play_pending = false;
        play_pending();
    }
}

int opt_audio_bus() { return g_opt_audio_bus; }
void set_opt_audio_bus(int v) { g_opt_audio_bus = v; }

} // namespace audio
