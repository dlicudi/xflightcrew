# Build the XVoiceLayer X-Plane plugin (macOS, universal arm64 + x86_64).
#
# Prereq: the X-Plane SDK 4.4.0 or newer (free, developer.x-plane.com/sdk;
# the panel renders through the 4.4 panel-graphics API, so X-Plane 12.4.4+)
# extracted to plugin/SDK so that plugin/SDK/CHeaders/XPLM/XPLMPanelGraphics.h
# and plugin/SDK/Libraries/Mac/XPLM.framework exist. Then: `make` → build/mac_x64/xcopilot.xpl. `make install` copies it into
# X-Plane (override XPLANE=...). See README.md.

PLUGIN  := xcopilot
SDK     ?= ./SDK
XPLANE  ?= $(HOME)/X-Plane 12

CXX      := clang++
ARCHS    := -arch arm64 -arch x86_64
# Version: the date of the commit the build sits on (2026.9.22), computed the
# way the app's build.rs computes it, so the pair can be compared at a glance.
VERSION  := $(shell git log -1 --format=%cd --date=format:%Y.%-m.%-d 2>/dev/null || echo dev)
# Git build id (short SHA, with -dirty when there are uncommitted changes, staged
# ones included), so the plugin's startup log and panel title show exactly which
# build is loaded.
BUILD    := $(shell git rev-parse --short HEAD 2>/dev/null || echo unknown)$(shell test -z "$$(git status --porcelain --untracked-files=no 2>/dev/null)" || echo -dirty.$$(date +%H%M%S))
CXXFLAGS := -std=c++17 -fPIC -fvisibility=hidden -O2 -mmacosx-version-min=12.0 \
            -Wno-deprecated-declarations -Wno-exceptions \
            -DAPL=1 -DIBM=0 -DLIN=0 -DXPLM200=1 -DXPLM210=1 -DXPLM300=1 -DXPLM301=1 \
            -DXPLM400=1 -DXPLM410=1 -DXPLM420=1 -DXPLM430=1 -DXPLM440=1 \
            -DXCOPILOT_VERSION='"$(VERSION)"' -DXCOPILOT_BUILD='"$(BUILD)"' \
            -I$(SDK)/CHeaders/XPLM -Ithird_party/imgui
LDFLAGS  := -dynamiclib -mmacosx-version-min=12.0 -F$(SDK)/Libraries/Mac \
            -framework XPLM -framework ApplicationServices \
            -framework CoreFoundation -lobjc

SRCS := src/$(PLUGIN).cpp src/drefs.cpp src/panel.cpp src/fms.cpp src/flightinit.cpp src/audio.cpp src/imgui_xp.cpp src/cursor.cpp src/tiller.cpp \
        third_party/imgui/imgui.cpp third_party/imgui/imgui_draw.cpp \
        third_party/imgui/imgui_tables.cpp third_party/imgui/imgui_widgets.cpp
HDRS := $(wildcard src/*.h)

OUT := build/mac_x64/$(PLUGIN).xpl

$(OUT): $(SRCS) $(HDRS)
	@mkdir -p build/mac_x64
	$(CXX) $(ARCHS) $(CXXFLAGS) $(LDFLAGS) -o $@ $(SRCS)
	@echo "built $@"

.PHONY: install clean
install: $(OUT)
	@mkdir -p "$(XPLANE)/Resources/plugins/$(PLUGIN)/mac_x64"
	cp $(OUT) "$(XPLANE)/Resources/plugins/$(PLUGIN)/mac_x64/$(PLUGIN).xpl"
	@echo "installed to $(XPLANE)/Resources/plugins/$(PLUGIN)/mac_x64/"

clean:
	rm -rf build
