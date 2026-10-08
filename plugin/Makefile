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
            -I$(SDK)/CHeaders/XPLM -Ithird_party/imgui -Ithird_party/XPMP2/inc
LDFLAGS  := -dynamiclib -mmacosx-version-min=12.0 -F$(SDK)/Libraries/Mac \
            -framework XPLM -framework ApplicationServices \
            -framework CoreFoundation -lobjc

SRCS := src/$(PLUGIN).cpp src/drefs.cpp src/panel.cpp src/fms.cpp src/flightinit.cpp src/audio.cpp src/imgui_xp.cpp src/cursor.cpp src/tiller.cpp \
        third_party/imgui/imgui.cpp third_party/imgui/imgui_draw.cpp \
        third_party/imgui/imgui_tables.cpp third_party/imgui/imgui_widgets.cpp
HDRS := $(wildcard src/*.h)

# XPMP2 (github.com/TwinFan/XPMP2, MIT, tag v3.7.0), the traffic drawing
# library, vendored under third_party/XPMP2 (src + inc + LICENSE). Built WITHOUT
# INCLUDE_FMOD_SOUND (never define it: FMOD has its own licence). Its objects
# are compiled once with its own defines and warnings silenced (third party
# only), then linked into the one plugin binary.
XPMP2_DIR   := third_party/XPMP2
XPMP2_SRCS  := $(wildcard $(XPMP2_DIR)/src/*.cpp)
XPMP2_HDRS  := $(wildcard $(XPMP2_DIR)/src/*.h $(XPMP2_DIR)/inc/*.h)
XPMP2_OBJS  := $(patsubst $(XPMP2_DIR)/src/%.cpp,build/xpmp2/%.o,$(XPMP2_SRCS))
XPMP2_FLAGS := -std=c++17 -fPIC -fvisibility=hidden -O2 -mmacosx-version-min=12.0 -w \
               -DAPL=1 -DIBM=0 -DLIN=0 -DNDEBUG=1 \
               -DXPLM200=1 -DXPLM210=1 -DXPLM300=1 -DXPLM301=1 -DXPLM303=1 -DXPLM400=1 \
               -DXPMP2_VERSION='"3.7.0"' -DXPMP2_VER_MAJOR=3 -DXPMP2_VER_MINOR=7 -DXPMP2_VER_PATCH=0 \
               -I$(SDK)/CHeaders/XPLM -I$(XPMP2_DIR)/inc -I$(XPMP2_DIR)/src

OUT := build/mac_x64/$(PLUGIN).xpl

$(OUT): $(SRCS) $(HDRS) $(XPMP2_OBJS)
	@mkdir -p build/mac_x64
	$(CXX) $(ARCHS) $(CXXFLAGS) $(LDFLAGS) -o $@ $(SRCS) $(XPMP2_OBJS)
	@echo "built $@"

build/xpmp2/%.o: $(XPMP2_DIR)/src/%.cpp $(XPMP2_HDRS)
	@mkdir -p build/xpmp2
	$(CXX) $(ARCHS) $(XPMP2_FLAGS) -c -o $@ $<

.PHONY: install clean
install: $(OUT)
	@mkdir -p "$(XPLANE)/Resources/plugins/$(PLUGIN)/mac_x64"
	cp $(OUT) "$(XPLANE)/Resources/plugins/$(PLUGIN)/mac_x64/$(PLUGIN).xpl"
	@echo "installed to $(XPLANE)/Resources/plugins/$(PLUGIN)/mac_x64/"

clean:
	rm -rf build
