// ---- panel ------------------------------------------------------------------
// The Dear ImGui status panel and everything it owns: the transmission log,
// the tabbed window, the plugins-menu entry, and the bindable panel toggle.

#include "panel.h"

#include <cfloat>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <memory>
#include <string>
#include <vector>

#include "XPLMDataAccess.h"
#include "XPLMDisplay.h"
#include "XPLMMenus.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
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
};
static std::vector<LogEntry> g_log;
static bool g_log_scroll = false;
static bool g_log_autoscroll = true;
// Live view filter: PILOT rows always show (it's the pilot's own transcript),
// the boxes toggle the other participants. copilot = the intercom, atc =
// calls with our name on them, others = the overheard party line. All ticked
// by default and never stored: a declutter for a busy frequency, not a mode.
static bool g_flt_copilot = true;
static bool g_flt_atc = true;
static bool g_flt_others = true;
// New entries arrived since the Transcript tab was last shown — drives an unread dot
// on the tab so activity isn't missed while on Route/Settings.
static bool g_log_unseen = false;

static void AppendLog(const char* tag, ImU32 color, const char* text,
                      bool ambient = false) {
    if (text == nullptr || text[0] == '\0') return;
    char ts[16] = {0};
    time_t now = time(nullptr);
    struct tm lt;
    localtime_r(&now, &lt);
    strftime(ts, sizeof ts, "%H:%M:%S", &lt);
    g_log.push_back({ts, tag, color, text, ambient});
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
            if (ImGui::BeginTabItem("Settings")) {
                mTab = 4;
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
        const float kTagW = ImGui::CalcTextSize("COPILOT").x + 14.0f; // widest tag
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
        const float ctlNeed = ImGui::CalcTextSize("copilot atc others").x
            + ImGui::GetFrameHeight() * 4.0f + ImGui::GetStyle().FramePadding.x * 8.0f
            + ImGui::GetStyle().ItemSpacing.x * 5.0f
            + ImGui::CalcTextSize("follow Clear").x;
        const float rows = ImGui::GetContentRegionAvail().x < ctlNeed ? 3.0f : 2.0f;
        const float footer = ImGui::GetFrameHeightWithSpacing() * rows
            + ImGui::GetStyle().ItemSpacing.y + coach_h;
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
            // The live filter. PILOT rows never filter: a row doesn't record
            // which channel the pilot's words went out on, and their side of
            // the transcript is the anchor either way. Ghost pilots (TRAFFIC)
            // count as the party line even if a row arrived undimmed.
            if (e.tag != "PILOT") {
                const bool others = e.ambient || e.tag == "TRAFFIC";
                const bool show = e.tag == "COPILOT" ? g_flt_copilot
                                  : others           ? g_flt_others
                                                     : g_flt_atc;
                if (!show) continue;
            }
            // Ambience (overheard party-line exchanges) dims, chip and all —
            // calls with our name on them stay bright.
            const ImU32 col = e.ambient ? COL_DIM : e.color;
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "%s", e.time.c_str());
            ImGui::SameLine(kTimeW + pad);
            TagChip(e.tag.c_str(), col, kTagW);
            ImGui::SameLine(kTimeW + kTagW + pad * 2);
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
        ImGui::Checkbox("copilot", &g_flt_copilot);
        ImGui::SameLine();
        ImGui::Checkbox("atc", &g_flt_atc);
        ImGui::SameLine();
        ImGui::Checkbox("others", &g_flt_others);
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
    void buildRouteTab() {
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
        if (ImGui::BeginTable("route", 4,
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY |
                              ImGuiTableFlags_BordersInnerV)) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed, 28.0f);
            ImGui::TableSetupColumn("IDENT");
            ImGui::TableSetupColumn("ALT", ImGuiTableColumnFlags_WidthFixed, 60.0f);
            ImGui::TableSetupColumn("POSITION");
            ImGui::TableHeadersRow();
            for (size_t i = 0; i < route_pts.size(); ++i) {
                const RoutePoint& p = route_pts[i];
                bool endpoint = (i == 0 || i + 1 == route_pts.size());
                ImU32 col = endpoint ? COL_CYAN : COL_WHITE;
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "%d",
                                   static_cast<int>(i + 1));
                ImGui::TableNextColumn();
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(col), "%s",
                                   p.ident.empty() ? "(pos)" : p.ident.c_str());
                ImGui::TableNextColumn();
                if (p.alt > 0) {
                    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(col), "%d", p.alt);
                }
                ImGui::TableNextColumn();
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                                   "%.4f, %.4f", p.lat, p.lon);
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
        }
        }
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

void append_log(const char* tag, ImU32 color, const char* text, bool ambient) {
    AppendLog(tag, color, text, ambient);
}

} // namespace panel
