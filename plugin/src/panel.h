// The in-sim status panel: the Dear ImGui window with its tabs (Log, Route,
// ATC, Calls, Settings), the transmission log it renders, the plugins-menu
// entry, and the bindable toggle command.

#pragma once

#include "imgui.h"

namespace panel {

// Create the shared font atlas, the panel window (launched collapsed), the
// xcopilot/toggle_panel command, and the plugins-menu entry.
void create();
// Tear it all down again: toggle command, window, font atlas, menu.
void destroy();

// Append a row to the panel's transmission log. Called by the dataref write
// handlers in drefs.cpp as heard/action/alert text arrives from the app.
// `route` is the row's "FROM > TO" label (xcopilot/status/fromto; its FROM
// is the chip and names the channel, intercom or radio), or
// empty: the row then shows its tag alone, as before.
void append_log(const char* tag, ImU32 color, const char* text,
                bool ambient = false, const char* route = nullptr);

} // namespace panel
