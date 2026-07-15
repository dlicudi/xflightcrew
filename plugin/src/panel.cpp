// ---- panel ------------------------------------------------------------------
// The Dear ImGui status panel and everything it owns: the transmission log,
// the tabbed window, the plugins-menu entry, and the bindable panel toggle.

#include "panel.h"

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
#include "XPLMUtilities.h"

#include "imgui.h"
#include "ImgWindow.h"
#include "ImgFontAtlas.h"

#include "common.h"
#include "drefs.h"
#include "fms.h"

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
// New entries arrived since the Log tab was last shown — drives an unread dot
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
    dl->AddRect(p, ImVec2(p.x + size.x, p.y + size.y), col, 3.0f, 0, 1.2f);
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

class XCopilotPanel : public ImgWindow {
public:
    XCopilotPanel(int l, int t, int r, int b)
        : ImgWindow(l, t, r, b, xplm_WindowDecorationRoundRectangle,
                    xplm_WindowLayerFloatingWindows) {
        // Keep the title clean; version + build id go to Log.txt on startup instead.
        SetWindowTitle("X-COPILOT");
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
        s.Colors[ImGuiCol_TabActive]        = slate2;
        s.Colors[ImGuiCol_TabUnfocused]     = slate1;
        s.Colors[ImGuiCol_TabUnfocusedActive] = slate2;
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
        // Allow shrinking down to the toolbar row (collapsed view).
        SetWindowResizingLimits(260, kCollapsedH, 1400, 1400);
    }

private:
    // Collapse to just the toolbar row (status chips under the title bar), so the
    // panel can tuck out of the way without closing. Remembers the expanded size.
    static constexpr int kCollapsedH = 46;
    bool mCollapsed = false;
    // Selected tab (0 = Log, 1 = Route, 2 = ATC, 3 = Calls, 4 = Settings) — content renders after
    // the shared context rows, so the tab bar itself sits at the very top.
    int mTab = 0;
    int mExpW = 420, mExpH = 300;

    void toggleCollapse() {
        int l, t, r, b;
        GetWindowGeometry(l, t, r, b);
        if (!mCollapsed) {
            mExpW = r - l;
            mExpH = t - b;
            SetWindowGeometry(l, t, r, t - kCollapsedH); // keep position, shrink height
            mCollapsed = true;
        } else {
            SetWindowGeometry(l, t, l + mExpW, t - mExpH); // restore size at current spot
            mCollapsed = false;
        }
    }

public:
    // Launch state: a slim status strip, not a window over the scenery. The
    // panel earns its full size when the pilot asks for it ("+"), instead of
    // greeting every flight as a splash screen.
    void startCollapsed() {
        if (!mCollapsed) toggleCollapse();
    }

protected:
    // The status strip: crew state, aircraft, and the two talk lights, with
    // the collapse/expand control right-aligned. Lives at the BOTTOM of the
    // expanded panel (instrument-style: controls above, annunciators below),
    // and is the only row shown when collapsed.
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
        const char* collapseLbl = mCollapsed ? "+" : "-";
        float collapseW = ImGui::CalcTextSize(collapseLbl).x
            + ImGui::GetStyle().FramePadding.x * 2.0f;
        ImGui::SameLine(ImGui::GetContentRegionMax().x - collapseW);
        if (ImGui::SmallButton(collapseLbl)) {
            toggleCollapse();
        }
    }

    void buildInterface() override {
        // The app heartbeats the phase ~1/s; a stale timestamp = not running.
        bool connected = drefs::app_connected();
        // Collapsed: just the status strip under the title bar.
        if (mCollapsed) {
            drawStatusBar(connected);
            return;
        }

        // Tabs at the very top: what you're looking AT comes first; state
        // annunciators live in the bottom status bar. Content renders after
        // the shared context rows, so those stay visible on every tab.
        if (ImGui::BeginTabBar("xcp_tabs")) {
            // Unread dot on the Log tab while entries arrive unseen — cleared
            // the moment the tab is shown, so it only lights up from others.
            ImGuiTabItemFlags logFlags =
                g_log_unseen ? ImGuiTabItemFlags_UnsavedDocument : 0;
            if (ImGui::BeginTabItem("Log", nullptr, logFlags)) {
                g_log_unseen = false;
                mTab = 0;
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Route")) {
                mTab = 1;
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("ATC")) {
                mTab = 2;
                ImGui::EndTabItem();
            }
            // A dot on Calls while a readback is OWED — a live condition,
            // not an unread counter: it stays lit until the readback is paid.
            ImGuiTabItemFlags callsFlags =
                drefs::coach_urgent() ? ImGuiTabItemFlags_UnsavedDocument : 0;
            if (ImGui::BeginTabItem("Calls", nullptr, callsFlags)) {
                mTab = 3;
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Settings")) {
                mTab = 4;
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }

        // Live flight strip — the app refreshes this with what the copilot sees
        // (emergency phase/runway, cabin altitude, endurance). Hidden when empty.
        const TextRef& strip = drefs::strip();
        if (connected && strip.len > 0) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(COL_CYAN));
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextUnformatted(strip.buf);
            ImGui::PopTextWrapPos();
            ImGui::PopStyleColor();
        }

        // Radio row — the field's comm plan as chips, the tuned one lit amber
        // ('>' prefix from the app). An instrument row, not a log line: it
        // reads like the radio stack, at a glance. Hidden when empty.
        const TextRef& radio = drefs::radio_row();
        if (connected && radio.len > 0) {
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
        }

        // Emergency buttons — the mouse fallback when voice fails mid-emergency.
        // Guarded-switch styling: the two that take the aircraft are filled dark
        // red, the one that gives it back dark green. Each writes a request code
        // the app polls and runs as the matching voice command.
        if (connected) {
            const ImVec4 redBg(0.42f, 0.13f, 0.11f, 1.0f), redHov(0.55f, 0.17f, 0.14f, 1.0f);
            const ImVec4 greenBg(0.11f, 0.32f, 0.16f, 1.0f), greenHov(0.14f, 0.42f, 0.21f, 1.0f);
            ImGui::PushStyleColor(ImGuiCol_Button, redBg);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, redHov);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, redHov);
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(COL_WHITE));
            if (ImGui::SmallButton("RECOVER")) drefs::set_request(1);
            ImGui::SameLine();
            if (ImGui::SmallButton("AUTOLAND")) drefs::set_request(2);
            ImGui::PopStyleColor(4);
            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_Button, greenBg);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, greenHov);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, greenHov);
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(COL_WHITE));
            if (ImGui::SmallButton("MY CONTROLS")) drefs::set_request(3);
            ImGui::PopStyleColor(4);
            if (drefs::request() != 0) {
                ImGui::SameLine();
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM), "sent...");
            }
            ImGui::Separator();
        }

        // Body: the selected tab's content. Log = the transmission record;
        // Route = what the app pushed for the FMS; Settings = the runtime
        // switches.
        switch (mTab) {
            case 0: buildLogTab(connected); break;
            case 1: buildRouteTab(); break;
            case 2: buildAtcTab(connected); break;
            case 3: buildCallsTab(connected); break;
            default: buildSettingsTab(); break;
        }

        // Bottom status bar — annunciators live under the content, like a
        // panel's lamp row.
        ImGui::Separator();
        drawStatusBar(connected);
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
        // Two reserved rows below the well (the log controls and the bottom
        // status bar drawn by buildInterface), plus the coach strip.
        const float footer = ImGui::GetFrameHeightWithSpacing() * 2.0f
            + ImGui::GetStyle().ItemSpacing.y + coach_h;
        ImGui::BeginChild("log", ImVec2(0, -footer), true);
        if (g_log.empty()) {
            // Empty state, centred in the well — not a footer mumble.
            const char* hint = connected ? "Hold PTT and speak." : "Start x-copilot to connect.";
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

        // Footer: the view filter on the left, log controls on the right —
        // state lives in the chips, version in Settings.
        ImGui::Checkbox("copilot", &g_flt_copilot);
        ImGui::SameLine();
        ImGui::Checkbox("atc", &g_flt_atc);
        ImGui::SameLine();
        ImGui::Checkbox("others", &g_flt_others);
        // Right-aligned: auto-scroll toggle + clear.
        const char* clearLbl = "Clear";
        float ctlW = ImGui::CalcTextSize("Clear").x + ImGui::CalcTextSize("follow").x
            + ImGui::GetFrameHeight() + ImGui::GetStyle().FramePadding.x * 6.0f
            + ImGui::GetStyle().ItemSpacing.x * 2.0f;
        ImGui::SameLine(ImGui::GetContentRegionMax().x - ctlW);
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

    // Every station in VHF range, nearest first, pushed by the app. The tab is
    // an instrument, not a listing: each frequency is a button — click to tune
    // COM1 — and the tuned one is lit amber. The active station (the one that
    // answers radio calls) is named in green.
    void buildAtcTab(bool connected) {
        if (!connected) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                               "Waiting for the app.");
            return;
        }
        const std::vector<AtcField>& atc_fields = drefs::atc_fields();
        if (atc_fields.empty()) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                               "No stations in VHF range.");
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                               "Range grows with altitude.");
            return;
        }
        int com1 = drefs::com1_khz();
        // Two reserved rows below the well: the tune hint and the bottom
        // status bar (drawn by buildInterface after the tab content).
        const float footer = ImGui::GetFrameHeightWithSpacing() * 2.0f
            + ImGui::GetStyle().ItemSpacing.y;
        if (ImGui::BeginChild("atc_well", ImVec2(0, -footer))) {
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
                    ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x;
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
        }
        ImGui::EndChild();
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                           "Click a frequency to tune COM1.");
    }

    // The radio coach: what to SAY NOW (live, from the tower's state machine,
    // pushed by the app) over a phraseology reference — the learning tool.
    // Ask the copilot "what do I say?" for the same line by voice.
    void buildCallsTab(bool connected) {
        const TextRef& coachText = drefs::coach();
        if (connected && coachText.len > 0) {
            // An owed readback is not a suggestion — amber, and named for
            // what it is. Everything else stays the calm cyan hint.
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
        // One reserved row below the well: the bottom status bar.
        const float footer = ImGui::GetFrameHeightWithSpacing()
            + ImGui::GetStyle().ItemSpacing.y;
        if (ImGui::BeginChild("calls_well", ImVec2(0, -footer))) {
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
            // The group matching the live flight phase (the coach payload's
            // group marker) reads "you are here"; the rest stay dim.
            char active = (connected && coachText.len > 0) ? drefs::coach_group() : 0;
            for (const Group& g : groups) {
                bool here = g.key != 0 && g.key == active;
                ImGui::TextColored(
                    ImGui::ColorConvertU32ToFloat4(here ? COL_CYAN : COL_DIM),
                    here ? "%s  <" : "%s", g.title);
                for (int i = 0; i < g.n; ++i) {
                    ImGui::Bullet();
                    ImGui::SameLine();
                    ImGui::PushTextWrapPos(0.0f);
                    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_WHITE), "%s", g.items[i].say);
                    ImGui::PopTextWrapPos();
                    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(COL_DIM));
                    ImGui::Indent(18.0f);
                    ImGui::PushTextWrapPos(0.0f);
                    ImGui::TextUnformatted(g.items[i].gets);
                    ImGui::PopTextWrapPos();
                    ImGui::Unindent(18.0f);
                    ImGui::PopStyleColor();
                }
                ImGui::Spacing();
            }
        }
        ImGui::EndChild();
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                           "Or ask the copilot: what do I say?");
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
            ImGui::RadioButton("ATC only (no copilot, no LLM)", &mode, 2);
            if (mode != drefs::opt_mode()) drefs::set_opt_mode(mode);
            ImGui::Spacing();
            ImGui::Separator();
        }
        // In ATC-only mode the copilot's settings would be dead switches
        // (this ImGui predates BeginDisabled, so hide rather than grey).
        const bool crew_off = (mode == 2);
        if (crew_off) {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(COL_DIM),
                               "Copilot settings hidden in ATC-only mode.");
        }
        if (!crew_off) {
        bool llm_on = (drefs::llm_enabled() == 1);
        if (ImGui::Checkbox("LLM fallback (answer questions)", &llm_on)) {
            drefs::set_llm_enabled(llm_on ? 1 : 0);
        }
        bool snap = drefs::opt_camera_snap() != 0;
        if (ImGui::Checkbox("Camera snap-to-panel on actions", &snap)) {
            drefs::set_opt_camera_snap(snap ? 1 : 0);
        }
        bool tone = drefs::opt_negative_tone() != 0;
        if (ImGui::Checkbox("Negative tone on unrecognized", &tone)) {
            drefs::set_opt_negative_tone(tone ? 1 : 0);
        }
        bool mute = drefs::opt_monitor_mute() != 0;
        if (ImGui::Checkbox("Mute monitor callouts", &mute)) {
            drefs::set_opt_monitor_mute(mute ? 1 : 0);
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
        // Dev corner: the plugin reload (bounces ALL plugins — the SDK can't
        // reload just one) and the build identity, out of the pilot's way.
        if (ImGui::SmallButton("Reload plugins")) {
            XPLMReloadPlugins();
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
    // One shared font atlas for all ImgWindows (uses ImGui's default font).
    ImgWindow::sFontAtlas = std::make_shared<ImgFontAtlas>();

    int sl, st, sr, sb;
    XPLMGetScreenBoundsGlobal(&sl, &st, &sr, &sb);
    // Start well below the top edge so the panel clears the X-Plane menu bar.
    int wl = sl + 50, wt = st - 150, wr = wl + 420, wb = wt - 300;
    g_panel = std::make_unique<XCopilotPanel>(wl, wt, wr, wb);
    // Launch as a slim status strip (chips only) — expand is one click away.
    g_panel->startCollapsed();
    g_toggle_cmd = XPLMCreateCommand("xcopilot/toggle_panel", "x-copilot: toggle the status panel");
    XPLMRegisterCommandHandler(g_toggle_cmd, TogglePanelHandler, 1, nullptr);

    int idx = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "X-COPILOT", nullptr, 0);
    g_menu = XPLMCreateMenu("X-COPILOT", XPLMFindPluginsMenu(), idx, MenuHandler, nullptr);
    XPLMAppendMenuItem(g_menu, "Toggle status panel", (void*)"toggle", 0);
}

void destroy() {
    if (g_toggle_cmd) { XPLMUnregisterCommandHandler(g_toggle_cmd, TogglePanelHandler, 1, nullptr); g_toggle_cmd = nullptr; }
    g_panel.reset();
    ImgWindow::sFontAtlas.reset();
    if (g_menu) { XPLMDestroyMenu(g_menu); g_menu = nullptr; }
}

void append_log(const char* tag, ImU32 color, const char* text, bool ambient) {
    AppendLog(tag, color, text, ambient);
}

} // namespace panel
