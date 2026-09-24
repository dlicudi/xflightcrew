// Shared bits every module needs: version identity, the signal colours, and
// the fixed text-dataref buffer type.

#pragma once

#include "imgui.h"

// Project version: the date of the commit the build sits on (2026.9.22),
// injected by the Makefile (-DXCOPILOT_VERSION=...) exactly as the app's
// build.rs computes it, so app and plugin name the same build the same way.
// "dev" if built outside the Makefile.
#ifndef XCOPILOT_VERSION
#define XCOPILOT_VERSION "dev"
#endif
// Git build id, injected by the Makefile (-DXCOPILOT_BUILD=...). Lets us confirm
// from the log/panel exactly which build is loaded after a reload. "dev" if built
// outside the Makefile.
#ifndef XCOPILOT_BUILD
#define XCOPILOT_BUILD "dev"
#endif

// ---- colours ----------------------------------------------------------------

static const ImU32 COL_GREEN = IM_COL32(120, 225, 140, 255);
static const ImU32 COL_DIM   = IM_COL32(150, 155, 165, 255);
static const ImU32 COL_AMBER = IM_COL32(255, 185, 40, 255);
static const ImU32 COL_CYAN  = IM_COL32(90, 200, 255, 255);
static const ImU32 COL_WHITE = IM_COL32(235, 235, 240, 255);
static const ImU32 COL_RED   = IM_COL32(240, 90, 80, 255);
// The active leg, as a G1000 draws it.
static const ImU32 COL_MAGENTA = IM_COL32(235, 110, 235, 255);

// Fixed buffer for a text dataref the app PATCHes over the Web API.
struct TextRef { char buf[512]; int len; };
