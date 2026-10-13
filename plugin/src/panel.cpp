// ---- panel ------------------------------------------------------------------
// The Dear ImGui status panel and everything it owns: the transmission log,
// the tabbed window, the plugins-menu entry, and the bindable panel toggle.

#include "panel.h"

#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "XPLMDataAccess.h"
#include "XPLMDisplay.h"
#include "XPLMMenus.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMScenery.h"
#include "XPLMUtilities.h"

#include "imgui.h"
#include "imgui_xp.h"

#include "common.h"
#include "drefs.h"
#include "audio.h"
#include "fms.h"
#include "font_b612_regular.h"
#include "font_b612_bold.h"

// B612 — the Airbus-designed cockpit typeface (OFL, see
// plugin/third_party/b612/OFL.txt). Two faces, added once; ImGui 1.92
// rasterises them at whatever size a frame asks for (the atlas is dynamic
// on the native renderer), so UI scale is a real size change, not a swap
// between pre-baked copies, and every glyph the face carries is available.
static ImFont* g_font_body = nullptr;
static ImFont* g_font_head = nullptr;
// Index = the ui_scale option: 100% .. 200% in quarter steps.
static const float kUiScale[5] = {1.0f, 1.25f, 1.5f, 1.75f, 2.0f};
static const int kUiScaleCount = 5;
static const float kBodyPx = 16.0f;
static const float kHeadPx = 20.0f;

// Transmission log shown in the panel, each entry stamped with wall-clock time
// so a test flight can be reviewed afterwards.
struct LogEntry {
    std::string time;
    std::string tag;
    ImU32 color;
    std::string text;
    // Overheard, not addressed to this aircraft (the party line's ghost
    // exchanges) — rendered dim so calls with our name on them stay bright.
    bool ambient;
    // Who to whom, as the app labelled it ("WAT519 > TOWER"); empty from an
    // app that sends none. The chip names the FROM; the TO is not drawn (the
    // channel says it: the intercom has two ends, a radio call opens with
    // the callsign).
    std::string route;
    // Which channel it went out on: the radio (the frequency) or the
    // intercom (copilot and cabin). Decided once, at append.
    bool radio;
};
static std::vector<LogEntry> g_log;
static bool g_log_scroll = false;
static bool g_log_autoscroll = true;
// Live view: the channel tabs (ALL / INTERCOM / RADIO, the two half-duplex
// channels the audio has) and the party-line box (the overheard frequency,
// a declutter for a busy field). Never stored: a view, not a mode.
enum Channel { kChanAll = 0, kChanIntercom = 1, kChanRadio = 2 };
static int g_chan = kChanAll;
static bool g_flt_others = true;

// The channel a row went out on. The app's label names it: an intercom
// row is FROM YOU, COPILOT, CABIN or FLIGHT DECK (efb::transcript::label);
// a radio row is FROM a callsign or a position. A row without a label (an
// older app, a panel-local line) goes by its tag: the crew's own lines and
// the pilot's typed words are the intercom, a station's are the radio.
static bool RowIsRadio(const char* tag, const std::string& route) {
    if (!route.empty()) {
        const std::string from = route.substr(0, route.find(" > "));
        return !(from == "YOU" || from == "COPILOT" || from == "CABIN" || from == "FLIGHT DECK");
    }
    const std::string t = tag;
    return !(t == "PILOT" || t == "COPILOT" || t == "CABIN");
}
// New entries arrived since the Transcript tab was last shown — drives an unread dot
// on the tab so activity isn't missed while on Route/Settings.
static bool g_log_unseen = false;

static void AppendLog(const char* tag, ImU32 color, const char* text,
                      bool ambient = false, const char* route = nullptr) {
    if (text == nullptr || text[0] == '\0') return;
    char ts[16] = {0};
    time_t now = time(nullptr);
    struct tm lt;
    localtime_r(&now, &lt);
    strftime(ts, sizeof ts, "%H:%M:%S", &lt);
    const std::string r = route ? route : "";
    g_log.push_back({ts, tag, color, text, ambient, r, RowIsRadio(tag, r)});
    if (g_log.size() > 200) g_log.erase(g_log.begin());
    if (g_log_autoscroll) g_log_scroll = true;
    g_log_unseen = true;
}

// While the app is connected: 0/1 = resting ("CONNECTED" — with push-to-talk the
// mic is shut until you press, and the green [PTT] chip shows actual capture),
// 2 = working on a command, 3 = reading back. A stale heartbeat (handled in
// DrawPanel) shows NOT CONNECTED instead.
static const char* PhaseLabel(int p) {
    switch (p) {
        case 1: return "LISTENING";
        case 2: return "THINKING";
        case 3: return "SPEAKING";
        default: return "CONNECTED";
    }
}
static ImU32 PhaseColor(int p) {
    switch (p) {
        case 1: return COL_WHITE; // mic open — the brightest state on the strip
        case 2: return COL_AMBER;
        case 3: return COL_CYAN;
        default: return COL_GREEN;
    }
}

// A small outlined "chip" (like the mockup's RAMP tags), drawn in `col` at the
// current cursor; advances the cursor by its size so SameLine() text follows.
static void Chip(const char* label, ImU32 col) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 p = ImGui::GetCursorScreenPos();
    ImVec2 ts = ImGui::CalcTextSize(label);
    const float padx = 6.0f, pady = 2.0f;
    ImVec2 size(ts.x + padx * 2, ts.y + pady * 2);
    dl->AddRect(p, ImVec2(p.x + size.x, p.y + size.y), col, 3.0f, 1.2f);
    dl->AddText(ImVec2(p.x + padx, p.y + pady), col, label);
    ImGui::Dummy(size);
}

// The taxi turn card, GPS style: a dial with an arrow pointing where the
// route runs 25 m ahead RELATIVE TO THE NOSE (recomputed every frame from the
// live true heading, so it turns smoothly as the aircraft does), the distance
// to the next decision big, its label under it, then a cross-track bar.
// Payload: "kind|dist_m|label|aim_true_deg|xtk_m|turn_deg|remaining_m".
static void TaxiCard(const TextRef& card, float scale) {
    char buf[256];
    const int n = card.len < static_cast<int>(sizeof buf) - 1
                      ? card.len : static_cast<int>(sizeof buf) - 1;
    std::memcpy(buf, card.buf, static_cast<size_t>(n));
    buf[n] = '\0';
    // Split on '|' by hand (strtok would swallow an empty field).
    const char* f[7] = {"", "0", "", "0", "0", "0", "0"};
    int nf = 0;
    char* p = buf;
    while (nf < 7) {
        f[nf++] = p;
        char* bar = std::strchr(p, '|');
        if (bar == nullptr) break;
        *bar = '\0';
        p = bar + 1;
    }
    const char kind = f[0][0];
    if (kind == '\0') return;
    const int dist_m = std::atoi(f[1]);
    const char* label = f[2];
    const float aim = static_cast<float>(std::atof(f[3]));
    const float xtk = static_cast<float>(std::atof(f[4]));
    const float turn = static_cast<float>(std::atof(f[5]));

    const float headPx = kHeadPx * scale;
    const float bodyPx = kBodyPx * scale;
    const float S = headPx * 3.2f;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 o = ImGui::GetCursorScreenPos();
    const ImVec2 c(o.x + S * 0.5f, o.y + S * 0.5f);
    const float R = S * 0.42f;
    const ImU32 ring = (COL_DIM & 0x00FFFFFFu) | 0x50000000u;
    dl->AddCircle(c, R, ring, 48, 1.5f * scale);

    const bool off = (kind == 'X');
    const ImU32 accent = off ? COL_RED
                       : kind == 'C' ? COL_AMBER
                       : kind == 'H' ? COL_GREEN
                       : COL_CYAN;
    if (!off) {
        // Clockwise positive, straight up when the route runs along the nose.
        float rel = std::fmod(aim - drefs::heading_true(), 360.0f);
        if (rel < -180.0f) rel += 360.0f;
        if (rel > 180.0f) rel -= 360.0f;
        const float a = rel * 3.14159265f / 180.0f;
        const ImVec2 fw(std::sin(a), -std::cos(a));   // forward on screen
        const ImVec2 rt(std::cos(a), std::sin(a));    // right on screen
        auto P = [&](float x, float y) {
            return ImVec2(c.x + (x * rt.x + y * fw.x) * R, c.y + (x * rt.y + y * fw.y) * R);
        };
        dl->AddQuadFilled(P(-0.17f, 0.15f), P(0.17f, 0.15f),
                          P(0.17f, -0.75f), P(-0.17f, -0.75f), accent);
        dl->AddTriangleFilled(P(0.0f, 0.85f), P(0.52f, 0.12f), P(-0.52f, 0.12f), accent);
    }
    ImGui::Dummy(ImVec2(S, S));
    ImGui::SameLine();

    ImGui::BeginGroup();
    char big[48];
    ImU32 bigCol = COL_WHITE;
    if (off) {
        std::snprintf(big, sizeof big, "OFF ROUTE");
        bigCol = COL_RED;
    } else {
        std::snprintf(big, sizeof big, "%d m", dist_m);
        if (kind == 'C' || dist_m < 50) bigCol = COL_AMBER;
        if (kind == 'H') bigCol = COL_GREEN;
    }
    ImGui::PushFont(g_font_head, headPx);
    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(bigCol), "%s", big);
    ImGui::PopFont();
    if (kind == 'C') {
        ImGui::SameLine();
        Chip("RWY", COL_AMBER);
    }

    ImGui::PushTextWrapPos(0.0f);
    if (off) {
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_AMBER), "%.0f m %s",
                           std::fabs(xtk), xtk >= 0.0f ? "R" : "L");
    } else {
        char line[160];
        if (kind == 'T' && turn != 0.0f) {
            std::snprintf(line, sizeof line, "%s%s%s", turn < 0.0f ? "< " : "> ",
                          std::fabs(turn) >= 100.0f ? "sharp " : "", label);
        } else {
            std::snprintf(line, sizeof line, "%s", label);
        }
        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(COL_CYAN));
        ImGui::TextUnformatted(line);
        ImGui::PopStyleColor();
    }
    ImGui::PopTextWrapPos();

    // Cross-track bar: centre tick, marker offset by xtk (clamped to 20 m),
    // positive xtk (aircraft right of the route) to the right of centre.
    if (!off) {
        float w = ImGui::GetContentRegionAvail().x;
        if (w > bodyPx * 12.0f) w = bodyPx * 12.0f;
        if (w < bodyPx * 3.0f) w = bodyPx * 3.0f;
        const float h = 8.0f * scale;
        const ImVec2 b = ImGui::GetCursorScreenPos();
        const float mid = b.x + w * 0.5f, cy = b.y + h * 0.5f;
        dl->AddLine(ImVec2(b.x, cy), ImVec2(b.x + w, cy), ring, 2.0f * scale);
        dl->AddLine(ImVec2(mid, b.y), ImVec2(mid, b.y + h), COL_DIM, 1.5f * scale);
        float x = xtk;
        if (x > 20.0f) x = 20.0f;
        if (x < -20.0f) x = -20.0f;
        const float mx = mid + x / 20.0f * (w * 0.5f);
        dl->AddCircleFilled(ImVec2(mx, cy), h * 0.5f,
                            std::fabs(xtk) <= 3.0f ? COL_GREEN : COL_AMBER, 12);
        ImGui::Dummy(ImVec2(w, h));
    }
    ImGui::EndGroup();
}

// Log speaker tag: a fixed-width filled pill so every row's text starts at the
// same column — the log reads as a transcript, not a ragged debug feed. Fill
// is the tag's own hue at low alpha; the label carries the color.
static void TagChip(const char* label, ImU32 col, float width) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 p = ImGui::GetCursorScreenPos();
    ImVec2 ts = ImGui::CalcTextSize(label);
    const float pady = 2.0f;
    ImVec2 size(width, ts.y + pady * 2);
    ImU32 fill = (col & 0x00FFFFFFu) | 0x2E000000u;
    dl->AddRectFilled(p, ImVec2(p.x + size.x, p.y + size.y), fill, 3.0f);
    dl->AddText(ImVec2(p.x + (width - ts.x) * 0.5f, p.y + pady), col, label);
    ImGui::Dummy(size);
}

// Tab marker: a filled dot in space the label RESERVES (kTabPad trailing
// spaces), drawn after the tab item off its own rect. ImGui's
// UnsavedDocument flag is deliberately not used — it renders a superscript
// "*", which is the IDE "you have unsaved edits" idiom (nothing here is
// unsaved), and it reserves no width, so it clips the label it sits beside.
// Reserved space also means the bar never jitters as a marker lights.
//
// Color carries the URGENCY, and the two markers are not the same signal:
// amber = something is OWED (a readback the pilot must transmit), dim =
// something is merely UNREAD. One glyph for both was the old bug.
static const char* kTabPad = "   ";
static void TabMarker(bool lit, ImU32 col) {
    // Call straight after BeginTabItem, in or out of the block: the tab
    // header is the last item either way. IsItemVisible keeps a dot from
    // floating loose when a narrow window scrolls its tab off the bar.
    if (!lit || !ImGui::IsItemVisible()) return;
    ImVec2 lo = ImGui::GetItemRectMin();
    ImVec2 hi = ImGui::GetItemRectMax();
    const float r = ImGui::GetFontSize() * 0.19f;
    // Centred vertically in the tab, inside the space the padding reserved.
    ImVec2 c(hi.x - ImGui::GetStyle().FramePadding.x - r,
             (lo.y + hi.y) * 0.5f);
    ImGui::GetWindowDrawList()->AddCircleFilled(c, r, col, 12);
}

// ---- remembered geometry ---------------------------------------------------
// One line in the sim's preferences folder: "left top right bottom visible".
// Kept beside X-Plane's own prefs (not the app's prefs.json) because it is a
// property of this sim install's screen, not of the pilot's copilot.
static std::string geometry_path() {
    char prefs[512] = {0};
    XPLMGetPrefsPath(prefs);
    std::string p(prefs);
    const size_t slash = p.find_last_of('/');
    if (slash == std::string::npos) return std::string();
    return p.substr(0, slash + 1) + "xvoicelayer_panel.txt";
}
static bool load_geometry(int& l, int& t, int& r, int& b, bool& visible) {
    const std::string path = geometry_path();
    if (path.empty()) return false;
    FILE* f = std::fopen(path.c_str(), "r");
    if (f == nullptr) return false;
    int v = 0;
    const int n = std::fscanf(f, "%d %d %d %d %d", &l, &t, &r, &b, &v);
    std::fclose(f);
    visible = (v != 0);
    return n == 5;
}
static void save_geometry(int l, int t, int r, int b, bool visible) {
    const std::string path = geometry_path();
    if (path.empty()) return;
    FILE* f = std::fopen(path.c_str(), "w");
    if (f == nullptr) return;
    std::fprintf(f, "%d %d %d %d %d\n", l, t, r, b, visible ? 1 : 0);
    std::fclose(f);
}

class XCopilotPanel : public XpImguiWindow {
public:
    XCopilotPanel(int l, int t, int r, int b)
        : XpImguiWindow(l, t, r, b, xplm_WindowDecorationRoundRectangle,
                        xplm_WindowLayerFloatingWindows) {
        // Keep the title clean; version + build id go to Log.txt on startup instead.
        SetWindowTitle("XVoiceLayer");
        ImGuiIO& io = ImGui::GetIO();
        io.IniFilename = nullptr;
        // One visual system: dark slate instrument, grey controls, color used
        // ONLY as signal (green ok, amber ATC/caution, cyan info, red danger).
        // Everything blue that ImGui ships by default is restyled — a cockpit
        // panel shouldn't look like a debug tool.
        ImGuiStyle& s = ImGui::GetStyle();
        s.WindowRounding = 8.0f;
        s.ChildRounding = 6.0f;
        s.FrameRounding = 4.0f;
        s.TabRounding = 4.0f;
        s.GrabRounding = 4.0f;
        s.ItemSpacing = ImVec2(8, 6);
        s.FramePadding = ImVec2(8, 3);
        const ImVec4 slate0(0.09f, 0.10f, 0.12f, 0.96f);  // window
        const ImVec4 slate1(0.12f, 0.13f, 0.15f, 1.00f);  // wells
        const ImVec4 slate2(0.17f, 0.19f, 0.22f, 1.00f);  // controls
        const ImVec4 slate3(0.23f, 0.25f, 0.29f, 1.00f);  // hover
        const ImVec4 slate4(0.28f, 0.31f, 0.36f, 1.00f);  // active
        s.Colors[ImGuiCol_WindowBg]         = slate0;
        s.Colors[ImGuiCol_ChildBg]          = slate1;
        s.Colors[ImGuiCol_Border]           = ImVec4(0.26f, 0.28f, 0.32f, 0.55f);
        s.Colors[ImGuiCol_FrameBg]          = slate2;
        s.Colors[ImGuiCol_FrameBgHovered]   = slate3;
        s.Colors[ImGuiCol_FrameBgActive]    = slate4;
        s.Colors[ImGuiCol_Button]           = slate2;
        s.Colors[ImGuiCol_ButtonHovered]    = slate3;
        s.Colors[ImGuiCol_ButtonActive]     = slate4;
        s.Colors[ImGuiCol_Tab]              = slate1;
        s.Colors[ImGuiCol_TabHovered]       = slate3;
        s.Colors[ImGuiCol_TabSelected]        = slate2;
        s.Colors[ImGuiCol_TabDimmed]        = slate1;
        s.Colors[ImGuiCol_TabDimmedSelected] = slate2;
        s.Colors[ImGuiCol_CheckMark]        = ImVec4(0.47f, 0.88f, 0.55f, 1.0f);
        s.Colors[ImGuiCol_ScrollbarBg]      = slate1;
        s.Colors[ImGuiCol_ScrollbarGrab]    = slate2;
        s.Colors[ImGuiCol_ScrollbarGrabHovered] = slate3;
        s.Colors[ImGuiCol_ScrollbarGrabActive]  = slate4;
        s.Colors[ImGuiCol_TitleBg]          = slate0;
        s.Colors[ImGuiCol_TitleBgActive]    = slate0;
        s.Colors[ImGuiCol_Separator]        = ImVec4(0.26f, 0.28f, 0.32f, 0.55f);
        s.Colors[ImGuiCol_HeaderHovered]    = slate3;
        s.Colors[ImGuiCol_HeaderActive]     = slate4;
        s.Colors[ImGuiCol_TableRowBg]       = slate1;
        s.Colors[ImGuiCol_TableRowBgAlt]    = ImVec4(0.14f, 0.15f, 0.18f, 1.0f);
        SetWindowResizingLimits(260, 160, 1400, 1400);
        // The styled-but-unscaled baseline: scale switches restore this and
        // re-derive, because ScaleAllSizes is multiplicative and compounding
        // it would grow the paddings run over run.
        mBaseStyle = s;
    }

private:
    // Selected tab (0 = Flight, 1 = Transcript, 2 = ATC, 3 = Route, 4 = Settings).
    // Flight is first and default: the automation instrument is the panel's
    // reason to be on screen mid-flight.
    int mTab = 0;
    // The applied UI scale index; -1 forces the first frame to apply one.
    int mScaleIdx = -1;
    ImGuiStyle mBaseStyle;
    // The typed-command line (Transcript tab): what the pilot can type when
    // speaking is not an option, sent on Enter through xcopilot/typed.
    char mTyped[256] = {0};
    // A scale change asks the next frame (font pushed, metrics real) to
    // re-fit the window to its tab strip.
    bool mFitPending = false;
    // The Settings tab's text fields, by registry key: the edit buffer, the
    // app's value it was last filled from, and whether this edit already
    // went out on Enter (so the focus loss that follows does not send it
    // twice).
    struct TextField {
        char buf[512] = {0};
        std::string shown;
        bool sent_on_enter = false;
        bool active = false;
    };
    std::map<std::string, TextField> mTextFields;

    // Split one row of the app's text settings on '|' (see drefs.cpp).
    static std::vector<std::string> SplitBar(const std::string& row) {
        std::vector<std::string> out;
        size_t start = 0;
        for (;;) {
            size_t bar = row.find('|', start);
            out.push_back(row.substr(start, bar == std::string::npos ? std::string::npos : bar - start));
            if (bar == std::string::npos) break;
            start = bar + 1;
        }
        return out;
    }

    // The registry's text knobs (callsign, airline telephony), one field
    // each, from the rows the app publishes. Applied on Enter or when the
    // field loses focus, never per keystroke; the app validates, applies
    // live, persists, and republishes the value in force (a refusal comes
    // back as a notice under the field). A knob pinned by an environment
    // variable is shown read-only, naming it. Hidden until the app
    // publishes (no app, or an app built before the text fields).
    void buildTextSettings() {
        const TextRef& text = drefs::opt_text();
        if (text.len <= 0) return;
        const std::string all(text.buf, static_cast<size_t>(text.len));
        ImGui::Spacing();
        ImGui::TextUnformatted("Radio");
        mChoiceHeading = false;
        size_t pos = 0;
        while (pos <= all.size()) {
            size_t nl = all.find('\n', pos);
            const std::string row = all.substr(pos, nl == std::string::npos ? std::string::npos : nl - pos);
            pos = (nl == std::string::npos) ? all.size() + 1 : nl + 1;
            const std::vector<std::string> f = SplitBar(row);
            if (f.size() < 6 || f[0].empty()) continue;
            // A choice knob (a seventh field naming its options, comma
            // separated): a dropdown under its own heading, the pick sent
            // on the text edit channel the app already validates.
            if (f.size() >= 7 && !f[6].empty()) {
                buildChoiceRow(f, mChoiceHeading);
                continue;
            }
            const std::string& key = f[0];
            const std::string& label = f[1];
            const std::string& pinned = f[2];
            const std::string& value = f[3];
            const std::string& notice = f[4];
            const std::string& help = f[5];
            TextField& tf = mTextFields[key];
            const std::string id = "##text_" + key;
            ImGui::PushID(key.c_str());
            ImGui::TextUnformatted(label.c_str());
            ImGui::SetNextItemWidth(-FLT_MIN);
            if (!pinned.empty()) {
                // Read-only: the environment wins at every boot, so an edit
                // here would be a lie.
                char ro[512];
                std::snprintf(ro, sizeof ro, "%s", value.c_str());
                ImGui::BeginDisabled();
                ImGui::InputText(id.c_str(), ro, sizeof ro, ImGuiInputTextFlags_ReadOnly);
                ImGui::EndDisabled();
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                                   "Set by %s in your .env; edit it there.", pinned.c_str());
                tf.shown = value;
            } else {
                // Follow the app's value whenever the pilot is not typing
                // (the field's activity as of last frame).
                if (!tf.active && tf.shown != value) {
                    std::snprintf(tf.buf, sizeof tf.buf, "%s", value.c_str());
                    tf.shown = value;
                }
                const bool enter = ImGui::InputTextWithHint(
                    id.c_str(), help.c_str(), tf.buf, sizeof tf.buf,
                    ImGuiInputTextFlags_EnterReturnsTrue);
                tf.active = ImGui::IsItemActive();
                const bool left = ImGui::IsItemDeactivatedAfterEdit();
                bool send = false;
                if (enter) {
                    send = true;
                    tf.sent_on_enter = true;
                } else if (left) {
                    send = !tf.sent_on_enter;
                }
                if (ImGui::IsItemDeactivated()) tf.sent_on_enter = false;
                if (send && value != tf.buf) {
                    if (!drefs::send_text_edit(key.c_str(), tf.buf)) {
                        std::snprintf(tf.buf, sizeof tf.buf, "%s", value.c_str());
                    }
                }
                if (ImGui::IsItemHovered() && !help.empty()) {
                    ImGui::SetTooltip("%s", help.c_str());
                }
            }
            if (!notice.empty()) {
                ImGui::PushTextWrapPos(0.0f);
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_AMBER), "%s", notice.c_str());
                ImGui::PopTextWrapPos();
            }
            // Under the callsign: what the tower and the crew will say, and
            // where it came from.
            if (key == "callsign") {
                const TextRef& cs = drefs::callsign_status();
                const std::string status(cs.buf, static_cast<size_t>(cs.len > 0 ? cs.len : 0));
                const std::vector<std::string> sf = SplitBar(status);
                if (sf.size() >= 2 && !sf[0].empty()) {
                    ImGui::PushTextWrapPos(0.0f);
                    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_CYAN),
                                       "Spoken: %s (%s)", sf[0].c_str(), sf[1].c_str());
                    ImGui::PopTextWrapPos();
                } else {
                    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                                       "Spoken: not resolved yet");
                }
            }
            ImGui::PopID();
        }
    }

    // One choice row: `key|label|pinned_env|value|notice|help|a,b,c`. The
    // heading "Traffic" goes over the first. Pinned by the environment, the
    // dropdown is shown disabled, naming the variable.
    bool mChoiceHeading = false;
    void buildChoiceRow(const std::vector<std::string>& f, bool& heading) {
        const std::string& key = f[0];
        const std::string& label = f[1];
        const std::string& pinned = f[2];
        const std::string& value = f[3];
        const std::string& notice = f[4];
        const std::string& help = f[5];
        std::vector<std::string> options;
        size_t start = 0;
        for (;;) {
            size_t c = f[6].find(',', start);
            options.push_back(f[6].substr(start, c == std::string::npos ? std::string::npos : c - start));
            if (c == std::string::npos) break;
            start = c + 1;
        }
        if (!heading) {
            ImGui::Spacing();
            ImGui::TextUnformatted("Traffic");
            heading = true;
        }
        ImGui::PushID(key.c_str());
        ImGui::TextUnformatted(label.c_str());
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (!pinned.empty()) ImGui::BeginDisabled();
        if (ImGui::BeginCombo("##choice", value.c_str())) {
            for (const std::string& o : options) {
                const bool sel = o == value;
                if (ImGui::Selectable(o.c_str(), sel) && !sel) {
                    drefs::send_text_edit(key.c_str(), o.c_str());
                }
                if (sel) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        if (!pinned.empty()) {
            ImGui::EndDisabled();
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                               "Set by %s in your .env; edit it there.", pinned.c_str());
        } else if (ImGui::IsItemHovered() && !help.empty()) {
            ImGui::SetTooltip("%s", help.c_str());
        }
        if (!notice.empty()) {
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_AMBER), "%s", notice.c_str());
            ImGui::PopTextWrapPos();
        }
        ImGui::PopID();
    }

    // The minimum the tab strip needs at the current font: every label at
    // its padding, or the bar scrolls behind arrows. Measured, not guessed,
    // so a bigger face gets a wider floor; the window grows to it (right
    // and down, kept on screen) rather than truncating its own tabs.
    void fitToTabs() {
        const ImGuiStyle& st = ImGui::GetStyle();
        static const char* kTabs[] = {"Flight", "Transcript   ", "ATC   ", "Route", "Settings"};
        float tabs = st.WindowPadding.x * 2.0f + st.ScrollbarSize;
        for (const char* t : kTabs) {
            tabs += ImGui::CalcTextSize(t).x + st.FramePadding.x * 2.0f + st.ItemInnerSpacing.x;
        }
        const int minW = static_cast<int>(tabs + 8.0f);
        const int minH = static_cast<int>(220 * kUiScale[mScaleIdx]);
        SetWindowResizingLimits(minW, minH, 1400, 1400);
        int l, t, r, b;
        GetWindowGeometry(l, t, r, b);
        if (r - l < minW || t - b < minH) {
            int sl, stp, sr, sb;
            XPLMGetScreenBoundsGlobal(&sl, &stp, &sr, &sb);
            if (r - l < minW) r = l + minW;
            if (t - b < minH) b = t - minH;
            if (r > sr) { l -= r - sr; r = sr; }
            if (b < sb) { t += sb - b; b = sb; }
            SetWindowGeometry(l, t, r, b);
        }
    }

protected:
    // The status strip: crew state, aircraft, the two talk lights, and the
    // automation annunciator. Lives at the BOTTOM of the panel
    // (instrument-style: controls above, annunciators below).
    void drawStatusBar(bool connected) {
        if (connected) {
            Chip(PhaseLabel(drefs::phase()), PhaseColor(drefs::phase()));
            const TextRef& aircraft = drefs::aircraft();
            if (aircraft.len > 0) {
                ImGui::SameLine();
                Chip(aircraft.buf, COL_CYAN);
            }
        } else {
            Chip("NOT CONNECTED", COL_RED);
        }
        ImGui::SameLine();
        Chip("PTT", drefs::ptt_active() ? COL_GREEN : COL_DIM);
        ImGui::SameLine();
        // TX = the radio (ATC) push-to-talk, distinct from the intercom PTT.
        Chip("TX", drefs::radio_ptt_active() ? COL_AMBER : COL_DIM);
        ImGui::SameLine();
        // AUTO lights amber while any automation is flying/driving — visible
        // from every tab, so a running full auto is never out of sight.
        Chip("AUTO", (connected && drefs::auto_active()) ? COL_AMBER : COL_DIM);
        // A button press waiting for the app to consume it (~300 ms).
        if (connected && drefs::request() != 0) {
            ImGui::SameLine();
            Chip("SENT", COL_CYAN);
        }
    }

    void buildInterface() override {
        // Apply the UI scale: swap the pre-baked font size and re-derive the
        // style metrics from the unscaled baseline. The option dataref is the
        // instant source of truth — a Settings pick applies THIS frame.
        int scale = drefs::opt_ui_scale();
        if (scale < 0 || scale >= kUiScaleCount) scale = 0;
        if (scale != mScaleIdx) {
            mScaleIdx = scale;
            ImGuiStyle& s = ImGui::GetStyle();
            s = mBaseStyle;
            s.ScaleAllSizes(kUiScale[scale]);
            s.FontSizeBase = kBodyPx * kUiScale[scale];
            // The minimum that still fits the tab strip and the footer row
            // at this scale; a window left smaller than that by a scale-up
            // grows to it (right and down, kept on screen) rather than
            // truncating its own tabs.
            mFitPending = true;
        }
        ImGui::PushFont(g_font_body, kBodyPx * kUiScale[mScaleIdx]);
        if (mFitPending) {
            mFitPending = false;
            fitToTabs();
        }

        // The app heartbeats the phase ~1/s; a stale timestamp = not running.
        bool connected = drefs::app_connected();

        // Tabs at the very top: what you're looking AT comes first; state
        // annunciators live in the bottom status bar. Each tab owns its whole
        // content — no shared header rows competing for height.
        if (ImGui::BeginTabBar("xcp_tabs")) {
            if (ImGui::BeginTabItem("Flight")) {
                mTab = 0;
                ImGui::EndTabItem();
            }
            // Dim dot on Transcript while entries arrive unseen — cleared the
            // moment the tab is shown, so it only lights up from others. Dim
            // on purpose: unread rows on a scrolling record are information,
            // not a duty, and must not read as loud as the ATC dot below.
            {
                const bool open = ImGui::BeginTabItem(
                    ("Transcript" + std::string(kTabPad)).c_str());
                TabMarker(g_log_unseen, COL_DIM);
                if (open) {
                    g_log_unseen = false;
                    mTab = 1;
                    ImGui::EndTabItem();
                }
            }
            // Amber dot on ATC while a readback is OWED — a live condition,
            // not an unread counter: it stays lit until the readback is paid,
            // and it is a duty, so it gets the urgent color.
            {
                const bool open = ImGui::BeginTabItem(
                    ("ATC" + std::string(kTabPad)).c_str());
                TabMarker(drefs::coach_urgent(), COL_AMBER);
                if (open) {
                    mTab = 2;
                    ImGui::EndTabItem();
                }
            }
            if (ImGui::BeginTabItem("Route")) {
                mTab = 3;
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Brief")) {
                mTab = 4;
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("State")) {
                mTab = 5;
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Board")) {
                mTab = 7;
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Settings")) {
                mTab = 6;
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }

        // Body: the selected tab's content, in its OWN scroll region between
        // the fixed tab bar and the fixed status bar. Without this child, a
        // tab taller than the window (Settings, an expanded phraseology
        // reference) scrolled the whole window and carried the tab bar off
        // the top. Transparent bg so the body reads as the window, not a
        // well — the wells inside keep their slate.
        const float statusH = ImGui::GetFrameHeightWithSpacing()
            + ImGui::GetStyle().ItemSpacing.y;
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0, 0, 0, 0));
        ImGui::BeginChild("tab_body", ImVec2(0, -statusH), ImGuiChildFlags_None);
        ImGui::PopStyleColor();
        switch (mTab) {
            case 0: buildFlightTab(connected); break;
            case 1: buildLogTab(connected); break;
            case 2: buildAtcTab(connected); break;
            case 3: buildRouteTab(); break;
            case 4: buildBriefTab(); break;
            case 5: buildStateTab(); break;
            case 7: buildBoardTab(); break;
            default: buildSettingsTab(); break;
        }
        ImGui::EndChild();

        // Bottom status bar — annunciators live under the content, like a
        // panel's lamp row.
        ImGui::Separator();
        drawStatusBar(connected);

        ImGui::PopFont();
        rememberGeometry();
    }

    // The window's place and size outlive the session: written to the
    // preferences folder once a move/resize has settled (2 s), read back at
    // create. A panel that reopens where the pilot left it is one fewer
    // thing to do before every flight.
    int mSavedGeom[4] = {0, 0, 0, 0};
    float mGeomChangedAt = -1.0f;
    void rememberGeometry() {
        int g[4];
        GetWindowGeometry(g[0], g[1], g[2], g[3]);
        if (std::memcmp(g, mSavedGeom, sizeof g) == 0) {
            mGeomChangedAt = -1.0f;
            return;
        }
        const float now = XPLMGetElapsedTime();
        if (mGeomChangedAt < 0.0f) mGeomChangedAt = now;
        if (now - mGeomChangedAt < 2.0f) return;
        std::memcpy(mSavedGeom, g, sizeof g);
        mGeomChangedAt = -1.0f;
        save_geometry(g[0], g[1], g[2], g[3], GetVisible());
    }
public:
    void markGeometrySaved(int l, int t, int r, int b) {
        mSavedGeom[0] = l; mSavedGeom[1] = t; mSavedGeom[2] = r; mSavedGeom[3] = b;
    }
protected:

    // One request button: fixed width so rows form a grid, writes its code to
    // the request channel the app polls (each code injects the same English
    // phrase a pilot would speak — the buttons can't do anything the voice
    // can't).
    void requestButton(const char* label, int code, float width) {
        if (ImGui::Button(label, ImVec2(width, 0.0f))) drefs::set_request(code);
    }

    // The Flight tab: the automation instrument. What the copilot is flying
    // right now, the live strip, and the mouse fallback for every automation
    // the voice can arm.
    void buildFlightTab(bool connected) {
        if (!connected) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                               "Start XVoiceLayer to connect.");
            return;
        }

        // Live flight strip — what the copilot sees (emergency phase/runway,
        // cabin altitude, endurance, next-turn hint). Hidden when empty.
        const TextRef& strip = drefs::strip();
        if (strip.len > 0) {
            ImGui::PushFont(g_font_head, kHeadPx * kUiScale[mScaleIdx]);
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(COL_CYAN));
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextUnformatted(strip.buf);
            ImGui::PopTextWrapPos();
            ImGui::PopStyleColor();
            ImGui::PopFont();
        }

        // The taxi turn card, while a taxi guide or auto taxi has a route.
        const TextRef& taxi = drefs::taxi_card();
        if (taxi.len > 0) TaxiCard(taxi, kUiScale[mScaleIdx]);

        // Reserve the button block below the well: CANCEL + two arm rows +
        // the guarded row; the status bar is drawn by buildInterface after
        // the tab.
        const float rowH = ImGui::GetFrameHeightWithSpacing();
        const float footer = rowH * 4.0f + ImGui::GetStyle().ItemSpacing.y;

        // The automation status well: the auto feed, primary line first as
        // NAME|stage, the rest one dim line each.
        ImGui::BeginChild("auto_well", ImVec2(0, -footer), ImGuiChildFlags_Borders);
        const TextRef& feed = drefs::auto_status();
        if (feed.len == 0) {
            const char* hint = "No automation active.";
            ImVec2 ts = ImGui::CalcTextSize(hint);
            ImGui::SetCursorPos(ImVec2((ImGui::GetWindowWidth() - ts.x) * 0.5f,
                                       (ImGui::GetWindowHeight() - ts.y) * 0.5f));
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "%s", hint);
        } else {
            char buf[512];
            std::memcpy(buf, feed.buf, sizeof buf);
            char* line = buf;
            bool first = true;
            while (line != nullptr) {
                char* nl = std::strchr(line, '\n');
                if (nl != nullptr) *nl = '\0';
                if (first) {
                    // Primary: "NAME|stage" — the manager as the headline,
                    // the stage as a chip beside it.
                    char* bar = std::strchr(line, '|');
                    const char* stage = "";
                    if (bar != nullptr) {
                        *bar = '\0';
                        stage = bar + 1;
                    }
                    ImGui::PushFont(g_font_head, kHeadPx * kUiScale[mScaleIdx]);
                    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_WHITE),
                                       "%s", line);
                    ImGui::PopFont();
                    if (stage[0] != '\0') {
                        ImGui::SameLine();
                        Chip(stage, COL_AMBER);
                    }
                } else {
                    ImGui::PushStyleColor(ImGuiCol_Text,
                                          ImGui::ColorConvertU32ToFloat4(COL_DIM));
                    ImGui::PushTextWrapPos(0.0f);
                    ImGui::TextUnformatted(line);
                    ImGui::PopTextWrapPos();
                    ImGui::PopStyleColor();
                }
                first = false;
                line = (nl != nullptr) ? nl + 1 : nullptr;
            }
        }
        ImGui::EndChild();

        const ImGuiStyle& st = ImGui::GetStyle();
        const ImVec4 redBg(0.42f, 0.13f, 0.11f, 1.0f), redHov(0.55f, 0.17f, 0.14f, 1.0f);
        const ImVec4 greenBg(0.11f, 0.32f, 0.16f, 1.0f), greenHov(0.14f, 0.42f, 0.21f, 1.0f);

        // CANCEL — full width, ALWAYS. The app picks the right phrase for
        // whatever is active ("cancel full auto", "abort takeoff", ...) and
        // falls back to "my controls" when nothing is — which is exactly the
        // state that most needs the button: a missed-approach give-up leaves
        // every manager flag down while the AP still flies the aircraft
        // dirty and slow, and gating on the marker made CANCEL vanish at
        // that moment (audit, 2026-08-02). The marker now feeds only the
        // AUTO annunciator.
        {
            ImGui::PushStyleColor(ImGuiCol_Button, redBg);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, redHov);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, redHov);
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(COL_WHITE));
            requestButton("CANCEL", 11, ImGui::GetContentRegionAvail().x);
            ImGui::PopStyleColor(4);
        }

        // The arm grid: every automation the voice can start. Always enabled
        // — a tool that can't run right now refuses with a spoken reason,
        // exactly as it would a spoken command.
        const float w4 =
            (ImGui::GetContentRegionAvail().x - st.ItemSpacing.x * 3.0f) / 4.0f;
        requestButton("FULL AUTO", 4, w4); ImGui::SameLine();
        requestButton("FLY PLAN", 5, w4); ImGui::SameLine();
        requestButton("PUSH BACK", 6, w4); ImGui::SameLine();
        requestButton("TAXI", 7, w4);
        requestButton("LINE UP", 8, w4); ImGui::SameLine();
        requestButton("TAKEOFF", 9, w4); ImGui::SameLine();
        requestButton("CIRCUIT", 10, w4); ImGui::SameLine();
        requestButton("LAND", 2, w4);

        // The guarded row — the emergency switches. RECOVER takes the
        // aircraft (dark red), MY CONTROLS gives everything back (dark
        // green).
        const float w2 = (ImGui::GetContentRegionAvail().x
                          - st.ItemSpacing.x) / 2.0f;
        ImGui::PushStyleColor(ImGuiCol_Button, redBg);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, redHov);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, redHov);
        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(COL_WHITE));
        requestButton("RECOVER", 1, w2);
        ImGui::PopStyleColor(4);
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, greenBg);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, greenHov);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, greenHov);
        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(COL_WHITE));
        requestButton("MY CONTROLS", 3, w2);
        ImGui::PopStyleColor(4);
    }

    // Transmission log — recent heard/action lines with coloured tags, each
    // stamped so a test can be reviewed afterwards.
    void buildLogTab(bool connected) {
        // Columns measured from the actual font (time | speaker | text) so
        // nothing ever clips and every message starts at the same x — the log
        // reads as a transcript, not a ragged debug feed.
        const float pad = 8.0f;
        const float kTimeW = ImGui::CalcTextSize("88:88:88").x + pad;
        // The widest chip: the tags, or a row's FROM (a callsign, a position),
        // capped so a long callsign never squeezes the text out.
        float tagMax = ImGui::CalcTextSize("COPILOT").x;
        const float tagCap = ImGui::CalcTextSize("DEPARTURE").x;
        for (const LogEntry& e : g_log) {
            if (e.route.empty()) continue;
            const size_t gt = e.route.find(" > ");
            const std::string from = e.route.substr(0, gt);
            float w = ImGui::CalcTextSize(from.c_str()).x;
            if (w > tagCap) w = tagCap;
            if (w > tagMax) tagMax = w;
        }
        const float kTagW = tagMax + 14.0f;
        // The coach's suggested next call pins BELOW the well, a footer strip
        // that never scrolls away with the transcript (it lived as a ghost
        // row inside the log until 2026-07-13, where a busy frequency kept
        // pushing it around). Height measured up front so the well only
        // shrinks while there is a call to show.
        const TextRef& coachText = drefs::coach();
        const bool coach = connected && coachText.len > 0;
        float coach_h = 0.0f;
        if (coach) {
            float wrap_w = ImGui::GetContentRegionAvail().x - (kTagW + pad * 2.0f);
            if (wrap_w < 60.0f) wrap_w = 60.0f;
            coach_h = ImGui::CalcTextSize(coachText.buf, nullptr, false, wrap_w).y
                + ImGui::GetStyle().ItemSpacing.y * 2.0f;
        }
        // Two reserved rows below the well (the typed-command line and the
        // log controls; the status bar sits outside the tab body now), plus
        // the coach strip.
        // (Three rows when the controls wrap under the filters: measured the
        // same way the wrap decides, so the well shrinks only when they do.)
        const float ctlNeed = ImGui::CalcTextSize("party line").x
            + ImGui::GetFrameHeight() * 2.0f + ImGui::GetStyle().FramePadding.x * 6.0f
            + ImGui::GetStyle().ItemSpacing.x * 3.0f
            + ImGui::CalcTextSize("follow Clear").x;
        const float rows = ImGui::GetContentRegionAvail().x < ctlNeed ? 3.0f : 2.0f;
        const float footer = ImGui::GetFrameHeightWithSpacing() * rows
            + ImGui::GetStyle().ItemSpacing.y + coach_h;
        // The channel tabs, above the well: ALL keeps the two channels on
        // one timeline (a warning between two tower calls reads in order).
        if (ImGui::BeginTabBar("xcp_chan")) {
            static const char* kChanName[3] = {"All", "Intercom", "Radio"};
            for (int c = 0; c < 3; ++c) {
                if (ImGui::BeginTabItem(kChanName[c])) {
                    if (g_chan != c) { g_chan = c; g_log_scroll = g_log_autoscroll; }
                    ImGui::EndTabItem();
                }
            }
            ImGui::EndTabBar();
        }
        ImGui::BeginChild("log", ImVec2(0, -footer), ImGuiChildFlags_Borders);
        if (g_log.empty()) {
            // Empty state, centred in the well — not a footer mumble.
            const char* hint = connected ? "Hold PTT and speak." : "Start XVoiceLayer to connect.";
            ImVec2 ts = ImGui::CalcTextSize(hint);
            ImGui::SetCursorPos(ImVec2((ImGui::GetWindowWidth() - ts.x) * 0.5f,
                                       (ImGui::GetWindowHeight() - ts.y) * 0.5f));
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "%s", hint);
        }
        for (const LogEntry& e : g_log) {
            // The live filter: the channel tab, then the party line. Ghost
            // pilots (TRAFFIC) count as the party line even if a row arrived
            // undimmed.
            if (g_chan == kChanIntercom && e.radio) continue;
            if (g_chan == kChanRadio && !e.radio) continue;
            if (!g_flt_others && (e.ambient || e.tag == "TRAFFIC")) continue;
            // Ambience (overheard party-line exchanges) dims, chip and all —
            // calls with our name on them stay bright.
            const ImU32 col = e.ambient ? COL_DIM : e.color;
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "%s", e.time.c_str());
            ImGui::SameLine(kTimeW + pad);
            // FROM on the chip: who said it. A row without a route keeps its
            // tag (an older app). Our callsign keyed by the crew says so, dim,
            // so the pilot's own transmissions stay told apart.
            std::string chip = e.tag;
            if (!e.route.empty()) chip = e.route.substr(0, e.route.find(" > "));
            TagChip(chip.c_str(), col, kTagW);
            ImGui::SameLine(kTimeW + kTagW + pad * 2);
            if (e.radio && e.tag == "COPILOT") {
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "copilot");
                ImGui::SameLine();
            }
            // Wrap long readbacks at the panel edge instead of running off it.
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(col));
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextUnformatted(e.text.c_str());
            ImGui::PopTextWrapPos();
            ImGui::PopStyleColor();
        }
        if (g_log_scroll) { ImGui::SetScrollHereY(1.0f); g_log_scroll = false; }
        ImGui::EndChild();

        // The coach strip: the suggested next call, live state never stored —
        // it refreshes with the tower's state machine and vanishes once the
        // call is made. Dim when it's a hint; amber with an OWED tag when a
        // readback is due, because that line isn't a suggestion, it's a debt.
        if (coach) {
            TagChip(drefs::coach_urgent() ? "OWED" : "SAY",
                    drefs::coach_urgent() ? COL_AMBER : COL_CYAN, kTagW);
            ImGui::SameLine(kTagW + pad * 2);
            ImGui::PushStyleColor(
                ImGuiCol_Text,
                ImGui::ColorConvertU32ToFloat4(drefs::coach_urgent() ? COL_AMBER : COL_DIM));
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextUnformatted(coachText.buf);
            ImGui::PopTextWrapPos();
            ImGui::PopStyleColor();
        }

        // The typed line: the same words the pilot would speak, for when the
        // mic is the wrong tool (a quiet house, a test, a hoarse throat).
        // Enter sends it through xcopilot/typed; the app injects it on the
        // typed-command queue exactly like a terminal line or a button, so
        // every gate and readback still applies. Focus stays in the field
        // so a run of commands needs no re-click.
        {
            ImGui::SetNextItemWidth(-FLT_MIN);
            if (ImGui::InputTextWithHint("##typed", "Type a command and press Enter",
                                         mTyped, sizeof mTyped,
                                         ImGuiInputTextFlags_EnterReturnsTrue)) {
                if (mTyped[0] != '\0' && connected) {
                    drefs::set_typed(mTyped);
                    AppendLog("PILOT", COL_WHITE, mTyped);
                }
                mTyped[0] = '\0';
                ImGui::SetKeyboardFocusHere(-1);
            }
        }

        // Footer: the view filter on the left, log controls on the right —
        // state lives in the chips, version in Settings.
        ImGui::Checkbox("party line", &g_flt_others);
        // Right-aligned: auto-scroll toggle + clear. On a window too narrow
        // for both groups on one row (a large UI scale) they take the next
        // row instead of overprinting the filters.
        const char* clearLbl = "Clear";
        float ctlW = ImGui::CalcTextSize("Clear").x + ImGui::CalcTextSize("follow").x
            + ImGui::GetFrameHeight() + ImGui::GetStyle().FramePadding.x * 6.0f
            + ImGui::GetStyle().ItemSpacing.x * 2.0f;
        const float rowEnd = ImGui::GetItemRectMax().x - ImGui::GetWindowPos().x;
        const float rightX = ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x - ctlW;
        if (rightX > rowEnd + ImGui::GetStyle().ItemSpacing.x) {
            ImGui::SameLine(rightX);
        } else {
            ImGui::SetCursorPosX(rightX);
        }
        ImGui::Checkbox("follow", &g_log_autoscroll);
        ImGui::SameLine();
        if (ImGui::SmallButton(clearLbl)) {
            g_log.clear();
        }
    }

    // The route the app pushed (SimBrief import), visible before/after it lands
    // in the FMS — plus a manual re-push for after an aircraft or plan reload.
    // One load sheet row: the plan's figure against the aircraft's, the
    // aircraft's coloured by whether it agrees.
    void loadSheetRow(const char* label, double plan, double actual, double k,
                      ImU32 col) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "%s", label);
        ImGui::TableNextColumn();
        if (plan > 0.0) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_WHITE), "%.0f", plan * k);
        } else {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "-");
        }
        ImGui::TableNextColumn();
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(col), "%.0f", actual * k);
        ImGui::TableNextColumn();
        if (plan > 0.0) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(col), "%+.0f",
                               (actual - plan) * k);
        }
    }

    // The plan's load sheet against what the sim is carrying, with the
    // button that says "load the aircraft". The plan's side comes from the
    // app; the aircraft's side is read here, live.
    void buildLoadSheet() {
        const TextRef& feed = drefs::load_sheet();
        if (feed.len <= 0) return;
        char buf[sizeof feed.buf];
        std::memcpy(buf, feed.buf, sizeof buf);
        buf[sizeof buf - 1] = '\0';
        char* f[7] = {nullptr};
        char* save = nullptr;
        int n = 0;
        for (char* t = strtok_r(buf, "|", &save); t != nullptr && n < 7;
             t = strtok_r(nullptr, "|", &save)) {
            f[n++] = t;
        }
        if (n < 7) return;
        const double block = std::atof(f[2]);
        const double payload = std::atof(f[3]);
        const double max_tow = std::atof(f[4]);
        const int pax = std::atoi(f[5]);
        const bool lbs = std::strcmp(f[6], "lbs") == 0;
        const double k = lbs ? 2.20462 : 1.0;

        static XPLMDataRef fuel_ref = XPLMFindDataRef("sim/flightmodel/weight/m_fuel_total");
        static XPLMDataRef fixed_ref = XPLMFindDataRef("sim/flightmodel/weight/m_fixed");
        static XPLMDataRef total_ref = XPLMFindDataRef("sim/flightmodel/weight/m_total");
        if (!fuel_ref || !fixed_ref || !total_ref) return;
        const double fuel = XPLMGetDataf(fuel_ref);
        const double fixed = XPLMGetDataf(fixed_ref);
        const double total = XPLMGetDataf(total_ref);

        // The preflight's own tolerances: fuel within 3 % of the block (more
        // is never a fault), payload within 10 % and 20 kg, weight under the
        // plan's takeoff limit.
        const ImU32 fuel_col = (block <= 0.0 || fuel >= block * 0.97) ? COL_GREEN : COL_RED;
        const double off = std::fabs(fixed - payload);
        const ImU32 pay_col =
            (payload <= 0.0 || off <= 20.0 || off <= payload * 0.10) ? COL_GREEN : COL_AMBER;
        const ImU32 tow_col = (max_tow <= 0.0 || total <= max_tow * 1.005) ? COL_GREEN : COL_RED;

        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "LOAD SHEET  %s > %s  (%s)",
                           f[0], f[1], lbs ? "lb" : "kg");
        if (pax > 0) {
            ImGui::SameLine();
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "  %d pax", pax);
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Load aircraft")) drefs::set_request(12);
        if (ImGui::BeginTable("loadsheet", 4, ImGuiTableFlags_BordersInnerV)) {
            ImGui::TableSetupColumn("");
            ImGui::TableSetupColumn("PLAN");
            ImGui::TableSetupColumn("ACTUAL");
            ImGui::TableSetupColumn("DIFF");
            ImGui::TableHeadersRow();
            loadSheetRow("Fuel", block, fuel, k, fuel_col);
            loadSheetRow("Payload", payload, fixed, k, pay_col);
            loadSheetRow("Weight / max TOW", max_tow, total, k, tow_col);
            ImGui::EndTable();
        }
        ImGui::Separator();
    }

    // The fix being flown to, with its magnetic bearing and distance from
    // the aircraft, and the minutes to it at the present groundspeed.
    // Returns the route row to highlight, or -1.
    int buildActiveFix() {
        const fms::ActiveFix a = fms::active_fix();
        if (!a.valid) return -1;
        static XPLMDataRef lat_ref = XPLMFindDataRef("sim/flightmodel/position/latitude");
        static XPLMDataRef lon_ref = XPLMFindDataRef("sim/flightmodel/position/longitude");
        static XPLMDataRef gs_ref = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
        if (!lat_ref || !lon_ref) return a.row;
        const double lat = XPLMGetDatad(lat_ref);
        const double lon = XPLMGetDatad(lon_ref);
        const double nm = fms::distance_nm(lat, lon, a.lat, a.lon);
        float brg = XPLMDegTrueToDegMagnetic(
            static_cast<float>(fms::bearing_true_deg(lat, lon, a.lat, a.lon)));
        int deg = static_cast<int>(brg + 0.5f) % 360;
        if (deg <= 0) deg += 360;
        const double kts = gs_ref ? XPLMGetDataf(gs_ref) * 1.94384 : 0.0;
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "ACTIVE");
        ImGui::SameLine();
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_GREEN), "%s",
                           a.ident.empty() ? "(pos)" : a.ident.c_str());
        ImGui::SameLine();
        if (kts > 30.0) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_WHITE),
                               "  %03d M   %.1f nm   %.0f min", deg, nm, nm / kts * 60.0);
        } else {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_WHITE),
                               "  %03d M   %.1f nm", deg, nm);
        }
        return a.row;
    }

    // The app's view of the flight: the flight model and the duty engine,
    // one line each. A line with DISAGREES, EMERGENCY or CRASHED is amber.
    void buildStateTab() {
        const TextRef& feed = drefs::state_lines();
        if (feed.len <= 0) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                               "No flight state yet (is the copilot running?).");
            return;
        }
        std::string text(feed.buf, feed.len);
        size_t start = 0;
        while (start <= text.size()) {
            size_t end = text.find('\n', start);
            if (end == std::string::npos) end = text.size();
            std::string line = text.substr(start, end - start);
            const bool alarm = line.find("DISAGREES") != std::string::npos
                || line.find("EMERGENCY") != std::string::npos
                || line.find("CRASHED") != std::string::npos;
            ImGui::PushStyleColor(ImGuiCol_Text,
                ImGui::ColorConvertU32ToFloat4(alarm ? COL_AMBER : COL_WHITE));
            ImGui::TextWrapped("%s", line.c_str());
            ImGui::PopStyleColor();
            start = end + 1;
        }
    }

    // The flight information board (2026-10-13): Departures then Arrivals,
    // one table each, as an airport's FIDS shows them. Rows are
    // "D|sched|time|number|airline|city|type|stand|status|own"; the status
    // is coloured (green moving or done, amber delayed, cyan approaching
    // or boarding), our own flight in magenta.
    static ImU32 boardColour(const std::string& status) {
        if (status == "Delayed") return COL_AMBER;
        if (status == "Boarding" || status == "Approaching") return COL_CYAN;
        if (status == "Scheduled") return COL_WHITE;
        if (status == "At gate") return COL_DIM;
        return COL_GREEN;
    }
    static std::vector<std::string> splitBar(const std::string& line) {
        std::vector<std::string> out;
        size_t start = 0;
        while (true) {
            size_t bar = line.find('|', start);
            out.push_back(line.substr(start, bar == std::string::npos ? std::string::npos : bar - start));
            if (bar == std::string::npos) break;
            start = bar + 1;
        }
        return out;
    }
    void buildBoardTab() {
        const TextRef& feed = drefs::board();
        if (feed.len <= 0 || !drefs::board_fresh()) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                               "No board. It shows our own traffic at a field with a timetable");
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                               "(LXGB, LEMG) with XCOPILOT_TRAFFIC=ours and ATC on.");
            return;
        }
        std::vector<std::vector<std::string>> deps, arrs;
        std::string header;
        std::string text(feed.buf, feed.len);
        size_t start = 0;
        while (start < text.size()) {
            size_t end = text.find('\n', start);
            if (end == std::string::npos) end = text.size();
            std::vector<std::string> f = splitBar(text.substr(start, end - start));
            start = end + 1;
            if (f.empty()) continue;
            if (f[0] == "H" && f.size() >= 3) {
                header = f[1] + "   local " + f[2];
            } else if (f.size() >= 10 && f[0] == "D") {
                deps.push_back(f);
            } else if (f.size() >= 10 && f[0] == "A") {
                arrs.push_back(f);
            }
        }
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "%s", header.c_str());
        boardTable("DEPARTURES", "To", "board_dep", deps);
        ImGui::Spacing();
        boardTable("ARRIVALS", "From", "board_arr", arrs);
    }
    void boardTable(const char* title, const char* place, const char* id,
                    const std::vector<std::vector<std::string>>& rows) {
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_WHITE), "%s", title);
        if (rows.empty()) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "  none");
            return;
        }
        const ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit
            | ImGuiTableFlags_BordersInnerH;
        if (!ImGui::BeginTable(id, 8, flags)) return;
        const char* heads[8] = {"Time", "Exp", "Flight", "Airline", place, "Type", "Stand", "Status"};
        for (const char* h : heads) ImGui::TableSetupColumn(h);
        ImGui::TableHeadersRow();
        for (const auto& r : rows) {
            const bool own = r[9] == "1";
            ImGui::TableNextRow();
            for (int c = 0; c < 7; ++c) {
                ImGui::TableSetColumnIndex(c);
                // The expected time only where it differs from the timetable.
                // An expected time equal to the schedule reads "On time",
                // dimmed, as an airport board shows it (a blank read as a
                // missing estimate, 2026-10-13).
                const bool on_time = c == 1 && r[2] == r[1];
                const std::string v = on_time ? std::string("On time") : r[c + 1];
                const ImU32 col = own ? COL_MAGENTA : (on_time ? COL_DIM : (c == 1 ? COL_AMBER : COL_WHITE));
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(col), "%s", v.c_str());
            }
            ImGui::TableSetColumnIndex(7);
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(own ? COL_MAGENTA : boardColour(r[8])),
                               "%s", r[8].c_str());
        }
        ImGui::EndTable();
    }

    // The latest long report, read rather than recited: its first line is
    // the title ("Flight plan", "Full auto brief"), the rest the report.
    void buildBriefTab() {
        const TextRef& feed = drefs::brief();
        if (feed.len <= 0) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                               "No brief yet. Ask for the flight plan, or arm full auto.");
            return;
        }
        std::string text(feed.buf, feed.len);
        size_t nl = text.find('\n');
        std::string title = nl == std::string::npos ? std::string("Brief") : text.substr(0, nl);
        std::string body = nl == std::string::npos ? text : text.substr(nl + 1);
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "%s", title.c_str());
        ImGui::Separator();
        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(COL_WHITE));
        ImGui::TextWrapped("%s", body.c_str());
        ImGui::PopStyleColor();
    }

    void buildRouteTab() {
        buildLoadSheet();
        const std::vector<RoutePoint>& route_pts = fms::route_points();
        if (route_pts.empty()) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                               "No route loaded.");
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                               "Say \"load the flight plan\" to import from SimBrief.");
            return;
        }
        const RoutePoint& dep = route_pts.front();
        const RoutePoint& des = route_pts.back();
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_WHITE),
                           "%s  >  %s   (%d waypoints)", dep.ident.c_str(),
                           des.ident.c_str(), static_cast<int>(route_pts.size()));
        ImGui::SameLine();
        // Re-push the plan into the GPS/FMS, e.g. after switching aircraft.
        if (ImGui::SmallButton("Send to FMS")) {
            fms::load_route();
            AppendLog("COPILOT", COL_GREEN, "Flight plan sent to the FMS.");
        }
        const int active_row = buildActiveFix();
        buildFlightPlanTable(route_pts, active_row);
    }

    // The flight plan page, G1000 FPL style: per waypoint the desired track
    // (magnetic) and length of the leg INTO it, the distance still to fly to
    // it along the route, the time to it at the present groundspeed, and the
    // planned altitude. Flown legs are dim; the active leg is magenta and
    // measured live from the aircraft; the rest are the plan's own legs
    // summed onward from there.
    void buildFlightPlanTable(const std::vector<RoutePoint>& pts, int active) {
        static XPLMDataRef lat_ref = XPLMFindDataRef("sim/flightmodel/position/latitude");
        static XPLMDataRef lon_ref = XPLMFindDataRef("sim/flightmodel/position/longitude");
        static XPLMDataRef gs_ref = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
        const double lat = lat_ref ? XPLMGetDatad(lat_ref) : 0.0;
        const double lon = lon_ref ? XPLMGetDatad(lon_ref) : 0.0;
        const double kts = gs_ref ? XPLMGetDataf(gs_ref) * 1.94384 : 0.0;
        const bool live = active >= 0 && lat_ref && lon_ref;
        const size_t n = pts.size();
        auto mag = [](double true_deg) {
            int d = static_cast<int>(XPLMDegTrueToDegMagnetic(static_cast<float>(true_deg)) + 0.5f) % 360;
            return d <= 0 ? d + 360 : d;
        };
        // Leg i runs from point i-1 to point i (leg 0 has none), and the
        // distance still to fly to each point: live from the aircraft to the
        // active point, then leg by leg. -1 = behind us / unknown.
        std::vector<double> leg_nm(n, 0.0), rem_nm(n, -1.0);
        std::vector<int> leg_dtk(n, -1);
        double total = 0.0;
        for (size_t i = 1; i < n; ++i) {
            leg_nm[i] = fms::distance_nm(pts[i - 1].lat, pts[i - 1].lon, pts[i].lat, pts[i].lon);
            leg_dtk[i] = mag(fms::bearing_true_deg(pts[i - 1].lat, pts[i - 1].lon, pts[i].lat, pts[i].lon));
            total += leg_nm[i];
        }
        if (live) {
            double acc = fms::distance_nm(lat, lon, pts[active].lat, pts[active].lon);
            rem_nm[active] = acc;
            for (size_t i = active + 1; i < n; ++i) {
                acc += leg_nm[i];
                rem_nm[i] = acc;
            }
        }
        // Summary: the plan's length, what is left, and the time to go.
        const double to_go = live ? rem_nm[n - 1] : total;
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "TOTAL");
        ImGui::SameLine();
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_WHITE), "%.0f nm", total);
        if (live) {
            ImGui::SameLine();
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "   TO GO");
            ImGui::SameLine();
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_WHITE), "%.0f nm", to_go);
            if (kts > 30.0) {
                const int mins = static_cast<int>(to_go / kts * 60.0 + 0.5);
                ImGui::SameLine();
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "   ETE");
                ImGui::SameLine();
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_WHITE), "%d:%02d", mins / 60, mins % 60);
            }
        }
        static int scrolled_to = -2;
        if (ImGui::BeginTable("route", 6,
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY |
                              ImGuiTableFlags_BordersInnerV)) {
            const float num = ImGui::CalcTextSize("0000.0").x;
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("WAYPOINT");
            ImGui::TableSetupColumn("DTK", ImGuiTableColumnFlags_WidthFixed, ImGui::CalcTextSize("DTK ").x);
            ImGui::TableSetupColumn("DIS", ImGuiTableColumnFlags_WidthFixed, num);
            ImGui::TableSetupColumn("REM", ImGuiTableColumnFlags_WidthFixed, num);
            ImGui::TableSetupColumn("ETE", ImGuiTableColumnFlags_WidthFixed, ImGui::CalcTextSize("00:00").x);
            ImGui::TableSetupColumn("ALT", ImGuiTableColumnFlags_WidthFixed, ImGui::CalcTextSize("00000").x);
            ImGui::TableHeadersRow();
            for (size_t i = 0; i < n; ++i) {
                const RoutePoint& p = pts[i];
                const int row = static_cast<int>(i);
                const bool flown = live && row < active;
                const bool is_active = row == active;
                const bool endpoint = (i == 0 || i + 1 == n);
                const ImU32 col = is_active ? COL_MAGENTA
                                  : flown   ? COL_DIM
                                  : endpoint ? COL_CYAN
                                             : COL_WHITE;
                const ImVec4 c = ImGui::ColorConvertU32ToFloat4(col);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (is_active && scrolled_to != active) {
                    ImGui::SetScrollHereY(0.3f);
                    scrolled_to = active;
                }
                ImGui::TextColored(c, "%s%s", is_active ? "> " : "  ",
                                   p.ident.empty() ? "(pos)" : p.ident.c_str());
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("%.4f, %.4f", p.lat, p.lon);
                }
                // The active leg is flown from where we ARE: its track and
                // distance are the aircraft's, as the G1000 shows them.
                int dtk = leg_dtk[i];
                double dis = leg_nm[i];
                if (is_active && live) {
                    dtk = mag(fms::bearing_true_deg(lat, lon, p.lat, p.lon));
                    dis = rem_nm[i];
                }
                ImGui::TableNextColumn();
                if (dtk >= 0) ImGui::TextColored(c, "%03d", dtk);
                ImGui::TableNextColumn();
                if (i > 0 || is_active) ImGui::TextColored(c, "%.1f", dis);
                ImGui::TableNextColumn();
                if (rem_nm[i] >= 0.0) ImGui::TextColored(c, "%.0f", rem_nm[i]);
                ImGui::TableNextColumn();
                if (rem_nm[i] >= 0.0 && kts > 30.0) {
                    const int mins = static_cast<int>(rem_nm[i] / kts * 60.0 + 0.5);
                    ImGui::TextColored(c, "%d:%02d", mins / 60, mins % 60);
                }
                ImGui::TableNextColumn();
                if (p.alt > 0) ImGui::TextColored(c, "%d", p.alt);
            }
            ImGui::EndTable();
        }
    }

    // The radio picture, one tab: what to SAY NOW (live, from the tower's
    // state machine), the field's comm plan, every station in VHF range —
    // each frequency a click-to-tune button, the tuned one lit amber — and
    // the phraseology reference for learning the words. The active station
    // (the one that answers radio calls) is named in green.
    void buildAtcTab(bool connected) {
        if (!connected) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                               "Waiting for the app.");
            return;
        }
        // The coach line: an owed readback is not a suggestion — amber, and
        // named for what it is. Everything else is a calm cyan hint. Ask the
        // copilot "what do I say?" for the same line by voice.
        const TextRef& coachText = drefs::coach();
        if (coachText.len > 0) {
            ImU32 col = drefs::coach_urgent() ? COL_AMBER : COL_CYAN;
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(
                                   drefs::coach_urgent() ? COL_AMBER : COL_DIM),
                               drefs::coach_urgent() ? "READBACK OWED" : "SAY NOW");
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(col));
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextUnformatted(coachText.buf);
            ImGui::PopTextWrapPos();
            ImGui::PopStyleColor();
            ImGui::Separator();
        }
        // Radio row — the tuned field's comm plan as chips, the tuned service
        // lit amber ('>' prefix from the app). Reads like the radio stack.
        const TextRef& radio = drefs::radio_row();
        if (radio.len > 0) {
            char row[512];
            std::memcpy(row, radio.buf, sizeof row);
            char* seg = row;
            bool first = true;
            while (seg != nullptr) {
                char* bar = std::strstr(seg, " | ");
                if (bar != nullptr) *bar = '\0';
                bool lit = (seg[0] == '>');
                if (lit) seg++;
                if (!first) ImGui::SameLine();
                Chip(seg, lit ? COL_AMBER : COL_DIM);
                first = false;
                seg = (bar != nullptr) ? bar + 3 : nullptr;
            }
            ImGui::Separator();
        }
        const std::vector<AtcField>& atc_fields = drefs::atc_fields();
        int com1 = drefs::com1_khz();
        // One reserved row below the well: the tune hint (the status bar
        // sits outside the tab body now).
        const float footer = ImGui::GetFrameHeightWithSpacing()
            + ImGui::GetStyle().ItemSpacing.y;
        if (ImGui::BeginChild("atc_well", ImVec2(0, -footer))) {
            if (atc_fields.empty()) {
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                                   "No stations in VHF range.");
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                                   "Range grows with altitude.");
            }
            for (size_t i = 0; i < atc_fields.size(); ++i) {
                const AtcField& f = atc_fields[i];
                ImGui::PushID(static_cast<int>(i));
                ImU32 nameCol = f.active ? COL_GREEN : COL_WHITE;
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(nameCol), "%s",
                                   f.name.c_str());
                ImGui::SameLine();
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "%d nm%s",
                                   f.dist_nm, f.active ? "  (active)" : "");
                // Frequency chips flow left-to-right and WRAP at the panel
                // edge — Heathrow publishes six services, and clipped
                // buttons can't be clicked.
                const float wrap_x =
                    ImGui::GetCursorScreenPos().x + ImGui::GetContentRegionAvail().x;
                for (size_t j = 0; j < f.freqs.size(); ++j) {
                    const AtcService& s = f.freqs[j];
                    bool tuned = std::abs(com1 - s.khz) <= 5;
                    char label[32];
                    std::snprintf(label, sizeof label, "%s %.3f", s.label.c_str(),
                                  s.khz / 1000.0);
                    if (j > 0) {
                        const ImGuiStyle& st = ImGui::GetStyle();
                        float next_w =
                            ImGui::CalcTextSize(label).x + st.FramePadding.x * 2.0f;
                        if (ImGui::GetItemRectMax().x + st.ItemSpacing.x + next_w
                            < wrap_x) {
                            ImGui::SameLine();
                        }
                    }
                    ImGui::PushID(static_cast<int>(j));
                    ImU32 col = tuned ? COL_AMBER : COL_DIM;
                    ImGui::PushStyleColor(ImGuiCol_Text,
                                          ImGui::ColorConvertU32ToFloat4(col));
                    if (ImGui::SmallButton(label)) {
                        drefs::tune_com1(s.khz);
                    }
                    ImGui::PopStyleColor();
                    ImGui::PopID();
                }
                if (i + 1 < atc_fields.size()) ImGui::Separator();
                ImGui::PopID();
            }

            // The phraseology reference — the learning tool, collapsed under
            // the live picture. The group matching the flight phase (the
            // coach payload's group marker) is named "you are here" style.
            ImGui::Spacing();
            ImGui::Separator();
            struct Phrase { const char* say; const char* gets; };
            struct Group { const char* title; char key; const Phrase* items; int n; };
            static const Phrase dep[] = {
                {"<Station> Tower, <callsign>, with information <letter>, request departure clearance", "zone clearance + squawk"},
                {"Request taxi", "taxi route via named taxiways"},
                {"Holding point <name>, ready for departure", "takeoff clearance, or line up and wait"},
                {"Lining up runway <number>, <callsign>", "readback of line up and wait"},
                {"Cleared for takeoff runway <number>, <callsign>", "readback: runway clearances must be read back"},
            };
            static const Phrase arr[] = {
                {"<Station> Tower, <callsign>, 10 miles out, inbound with information <letter>", "join instruction"},
                {"Request touch and go", "circuit join, option to land"},
                {"Crosswind / Downwind / Turning base / Final", "sequenced around the circuit"},
                {"Cleared to land runway <number>, <callsign>", "readback of the landing clearance"},
                {"Going around", "climb-out and rejoin"},
                {"Runway vacated, request taxi to parking", "taxi in"},
            };
            static const Phrase misc[] = {
                {"Radio check", "reading you five"},
                {"Say wind / Say QNH", "current numbers"},
                {"What's my squawk", "your assigned code"},
                {"Request flight following", "radar advisories"},
                {"Request vectors", "headings to final"},
                {"Request frequency change", "change approved"},
            };
            static const Phrase emer[] = {
                {"Mayday, mayday, mayday, <callsign>, <problem>", "priority + cleared any runway"},
                {"Pan pan, pan pan, pan pan, <callsign>, <problem>", "urgency handling"},
            };
            static const Group groups[] = {
                {"DEPARTURE", 'D', dep, 5}, {"ARRIVAL AND CIRCUIT", 'A', arr, 6},
                {"ANY TIME", 0, misc, 6}, {"EMERGENCY", 0, emer, 2},
            };
            // Collapsible per group so the live picture keeps the space; the
            // group matching the live flight phase is highlighted and starts
            // open ("you are here"), the rest stay collapsed and dim.
            char activeGroup = (coachText.len > 0) ? drefs::coach_group() : 0;
            for (const Group& g : groups) {
                bool here = g.key != 0 && g.key == activeGroup;
                if (here) ImGui::SetNextItemOpen(true, ImGuiCond_Appearing);
                ImGui::PushStyleColor(
                    ImGuiCol_Text,
                    ImGui::ColorConvertU32ToFloat4(here ? COL_CYAN : COL_DIM));
                bool open = ImGui::CollapsingHeader(g.title);
                ImGui::PopStyleColor();
                if (!open) continue;
                for (int k = 0; k < g.n; ++k) {
                    ImGui::Bullet();
                    ImGui::SameLine();
                    ImGui::PushTextWrapPos(0.0f);
                    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_WHITE), "%s", g.items[k].say);
                    ImGui::PopTextWrapPos();
                    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(COL_DIM));
                    ImGui::Indent(18.0f);
                    ImGui::PushTextWrapPos(0.0f);
                    ImGui::TextUnformatted(g.items[k].gets);
                    ImGui::PopTextWrapPos();
                    ImGui::Unindent(18.0f);
                    ImGui::PopStyleColor();
                }
                ImGui::Spacing();
            }
        }
        ImGui::EndChild();
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                           "Click a frequency to tune COM1.");
    }

    // Live runtime switches. Each checkbox drives a dataref the app polls live;
    // changes apply on the next utterance/tick, no restart.
    void buildSettingsTab() {
        // App mode: which half of the product is on. -1 = the app hasn't
        // published yet (no app connected) — hide the control rather than
        // show radio buttons that would silently do nothing.
        int mode = drefs::opt_mode();
        if (mode >= 0) {
            ImGui::TextUnformatted("Mode");
            ImGui::RadioButton("Copilot + ATC", &mode, 0);
            ImGui::RadioButton("Copilot only (real/external ATC has the radio)", &mode, 1);
            ImGui::RadioButton("ATC only (no copilot)", &mode, 2);
            ImGui::RadioButton("Instructor (teaches and grades, never touches)", &mode, 3);
            if (mode != drefs::opt_mode()) drefs::set_opt_mode(mode);
            ImGui::Spacing();
            ImGui::Separator();
        }
        // UI scale — applied by buildInterface the frame after a pick, and
        // remembered by the app (publish/pick like the mode radios; hidden
        // until the app publishes).
        int uiScale = drefs::opt_ui_scale();
        if (uiScale >= 0) {
            ImGui::TextUnformatted("UI scale");
            static const char* kScaleLabels[kUiScaleCount] = {"100%", "125%", "150%", "175%", "200%"};
            // Wrap at the panel edge: a radio the pilot cannot see cannot
            // be picked (the 200% option fell off a 200% panel).
            const float wrap_x =
                ImGui::GetCursorScreenPos().x + ImGui::GetContentRegionAvail().x;
            for (int i = 0; i < kUiScaleCount; ++i) {
                if (i > 0) {
                    const ImGuiStyle& st = ImGui::GetStyle();
                    const float next_w = ImGui::GetFrameHeight() + st.ItemInnerSpacing.x
                        + ImGui::CalcTextSize(kScaleLabels[i]).x;
                    if (ImGui::GetItemRectMax().x + st.ItemSpacing.x + next_w < wrap_x) {
                        ImGui::SameLine();
                    }
                }
                ImGui::RadioButton(kScaleLabels[i], &uiScale, i);
            }
            if (uiScale != drefs::opt_ui_scale()) drefs::set_opt_ui_scale(uiScale);
            ImGui::Spacing();
            ImGui::Separator();
        }
        // In ATC-only mode the copilot's settings would be dead switches:
        // hidden, not greyed, so the tab reads as the mode it is in.
        const bool crew_off = (mode == 2);
        if (crew_off) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                               "Copilot settings hidden in ATC-only mode.");
        }
        if (!crew_off) {
        bool snap = drefs::opt_camera_snap() != 0;
        if (ImGui::Checkbox("Camera snap-to-panel on actions", &snap)) {
            drefs::set_opt_camera_snap(snap ? 1 : 0);
        }
        bool tone = drefs::opt_negative_tone() != 0;
        if (ImGui::Checkbox("Negative tone on unrecognized", &tone)) {
            drefs::set_opt_negative_tone(tone ? 1 : 0);
        }
        bool bus = audio::opt_audio_bus() == 1;
        if (ImGui::Checkbox("Voices through X-Plane audio (COM1 / copilot buses)", &bus)) {
            audio::set_opt_audio_bus(bus ? 1 : 0);
        }
        bool mute = drefs::opt_monitor_mute() != 0;
        if (ImGui::Checkbox("Mute monitor callouts", &mute)) {
            drefs::set_opt_monitor_mute(mute ? 1 : 0);
        }
        // Cabin crew staffing: Auto defers to the airframe profile (the
        // MD-82 has a crew, a 172 doesn't); Aboard/Off override it either
        // way. Same publish/pick protocol as the mode radios: -1 = the app
        // hasn't published yet, hide the control.
        int cabin = drefs::opt_cabin();
        if (cabin >= 0) {
            ImGui::Spacing();
            ImGui::TextUnformatted("Cabin crew");
            ImGui::RadioButton("Auto (airframe decides)", &cabin, 0);
            ImGui::RadioButton("Aboard (staff any airframe)", &cabin, 1);
            ImGui::RadioButton("Off (no cabin crew)", &cabin, 2);
            if (cabin != drefs::opt_cabin()) drefs::set_opt_cabin(cabin);
        }
        // Copilot voice: the roster the app published, one combo row. A pick
        // writes the index back; the app polls it, swaps the voice live, and
        // confirms in the new voice. Hidden until the app publishes (no app,
        // no models, or the `say` backend where piper voices don't apply).
        if (drefs::voice_list_len() > 0) {
            std::vector<std::string> names;
            {
                const char* p = drefs::voice_list();
                const char* end = p + drefs::voice_list_len();
                while (p < end) {
                    const char* nl = static_cast<const char*>(std::memchr(p, '\n', static_cast<size_t>(end - p)));
                    size_t n = nl ? static_cast<size_t>(nl - p) : static_cast<size_t>(end - p);
                    if (n > 0) names.emplace_back(p, n);
                    p += n + 1;
                }
            }
            const int voice = drefs::opt_voice();
            const bool picked = voice >= 0 && voice < static_cast<int>(names.size());
            const char* preview = picked ? names[static_cast<size_t>(voice)].c_str() : "(default)";
            if (ImGui::BeginCombo("Copilot voice", preview)) {
                for (int i = 0; i < static_cast<int>(names.size()); ++i) {
                    bool selected = (i == voice);
                    if (ImGui::Selectable(names[static_cast<size_t>(i)].c_str(), selected)) {
                        drefs::set_opt_voice(i);
                    }
                    if (selected) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
            // The cabin crew's voice, same roster. Picking the copilot's own
            // voice is refused by the app, which puts the old index back.
            const int cabin_voice = drefs::opt_cabin_voice();
            const bool cabin_picked =
                cabin_voice >= 0 && cabin_voice < static_cast<int>(names.size());
            const char* cabin_preview =
                cabin_picked ? names[static_cast<size_t>(cabin_voice)].c_str() : "(default)";
            if (ImGui::BeginCombo("Cabin crew voice", cabin_preview)) {
                for (int i = 0; i < static_cast<int>(names.size()); ++i) {
                    bool selected = (i == cabin_voice);
                    if (ImGui::Selectable(names[static_cast<size_t>(i)].c_str(), selected)) {
                        drefs::set_opt_cabin_voice(i);
                    }
                    if (selected) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
        }
        }
        // The radio callsign serves the tower in every mode, so it stands
        // outside the copilot block.
        buildTextSettings();
        ImGui::Spacing();
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                           "Changes apply live; no restart needed.");
        ImGui::Separator();
        // Dev corner: reload THIS plugin only (SDK 4.4: the other plugins
        // stay up, and a mid-flight rebuild no longer bounces the airframe's
        // own systems) and the build identity, out of the pilot's way.
        if (ImGui::SmallButton("Reload plugin")) {
            XPLMReloadThisPlugin(0);
        }
        ImGui::SameLine();
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                           "after a rebuild · v" XCOPILOT_VERSION " · " XCOPILOT_BUILD);
    }
};

// The floating callout toast is gone: a notification splashed over the scenery
// after every action proved to be noise — the voice readback confirms the
// action, and the panel log keeps the written record. The callout dataref
// stays registered (in drefs.cpp) so the app's writes keep succeeding; the
// plugin just no longer draws them.

static std::unique_ptr<XCopilotPanel> g_panel;
static XPLMMenuID g_menu = nullptr;

static void MenuHandler(void*, void*) {
    if (g_panel) g_panel->SetVisible(!g_panel->GetVisible());
}

// Bindable panel toggle (xcopilot/toggle_panel) — a joystick button or key
// beats reaching for the plugins menu mid-flight.
static XPLMCommandRef g_toggle_cmd = nullptr;
static int TogglePanelHandler(XPLMCommandRef, XPLMCommandPhase phase, void*) {
    if (phase == xplm_CommandBegin && g_panel) {
        g_panel->SetVisible(!g_panel->GetVisible());
    }
    return 0;
}

namespace panel {

void create() {
    int sl, st, sr, sb;
    XPLMGetScreenBoundsGlobal(&sl, &st, &sr, &sb);
    // Start well below the top edge so the panel clears the X-Plane menu bar.
    // Wide enough for the Flight tab's four-button rows in the 16px body font.
    int wl = sl + 50, wt = st - 150, wr = wl + 470, wb = wt - 340;
    bool visible = false;
    // Last session's place, if it still fits the screen we have now (a
    // monitor that went away must not swallow the panel).
    int gl, gt, gr, gb;
    bool gv;
    if (load_geometry(gl, gt, gr, gb, gv) && gr - gl >= 260 && gt - gb >= 160
        && gl >= sl && gr <= sr && gb >= sb && gt <= st) {
        wl = gl; wt = gt; wr = gr; wb = gb;
        visible = gv;
    }
    g_panel = std::make_unique<XCopilotPanel>(wl, wt, wr, wb);
    g_panel->markGeometrySaved(wl, wt, wr, wb);

    // The faces are added to THIS window's atlas, once, at no fixed size:
    // the frame picks the size (see buildInterface). The first face added is
    // the default, so even a path that never pushes a font renders B612.
    {
        ImFontConfig cfg;
        cfg.FontDataOwnedByAtlas = false;  // the arrays are static — never freed
        ImGuiIO& io = ImGui::GetIO();
        g_font_body = io.Fonts->AddFontFromMemoryTTF(
            B612_Regular_ttf, static_cast<int>(B612_Regular_ttf_len), 0.0f, &cfg);
        g_font_head = io.Fonts->AddFontFromMemoryTTF(
            B612_Bold_ttf, static_cast<int>(B612_Bold_ttf_len), 0.0f, &cfg);
    }
    if (visible) g_panel->SetVisible(true);
    g_toggle_cmd = XPLMCreateCommand("xcopilot/toggle_panel", "XVoiceLayer: toggle the status panel");
    XPLMRegisterCommandHandler(g_toggle_cmd, TogglePanelHandler, 1, nullptr);

    int idx = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "XVoiceLayer", nullptr, 0);
    g_menu = XPLMCreateMenu("XVoiceLayer", XPLMFindPluginsMenu(), idx, MenuHandler, nullptr);
    XPLMAppendMenuItem(g_menu, "Toggle status panel", (void*)"toggle", 0);
}

void destroy() {
    if (g_toggle_cmd) { XPLMUnregisterCommandHandler(g_toggle_cmd, TogglePanelHandler, 1, nullptr); g_toggle_cmd = nullptr; }
    if (g_panel) {
        int l, t, r, b;
        g_panel->GetWindowGeometry(l, t, r, b);
        save_geometry(l, t, r, b, g_panel->GetVisible());
    }
    g_panel.reset();
    g_font_body = nullptr;
    g_font_head = nullptr;
    if (g_menu) { XPLMDestroyMenu(g_menu); g_menu = nullptr; }
}

void append_log(const char* tag, ImU32 color, const char* text, bool ambient,
                const char* route) {
    AppendLog(tag, color, text, ambient, route);
}

} // namespace panel
