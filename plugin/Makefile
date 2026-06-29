# Build the x-copilot X-Plane plugin (macOS, universal arm64 + x86_64).
#
# Prereq: the X-Plane SDK (free, developer.x-plane.com/sdk) extracted to plugin/SDK
# so that plugin/SDK/CHeaders/XPLM/*.h and plugin/SDK/Libraries/Mac/XPLM.framework
# exist. Then: `make` → build/mac_x64/xcopilot.xpl. `make install` copies it into
# X-Plane (override XPLANE=...). See README.md.

PLUGIN  := xcopilot
SDK     ?= ./SDK
XPLANE  ?= $(HOME)/X-Plane 12

CXX      := clang++
ARCHS    := -arch arm64 -arch x86_64
CXXFLAGS := -std=c++17 -fPIC -fvisibility=hidden -O2 -mmacosx-version-min=11.0 \
            -Wno-deprecated-declarations \
            -DAPL=1 -DIBM=0 -DLIN=0 -DXPLM200=1 -DXPLM210=1 -DXPLM300=1 -DXPLM301=1 \
            -DXPLM400=1 -DXPLM410=1 \
            -I$(SDK)/CHeaders/XPLM -Ithird_party/imgui -Ithird_party/ImgWindow
LDFLAGS  := -dynamiclib -mmacosx-version-min=11.0 -F$(SDK)/Libraries/Mac \
            -framework XPLM -framework OpenGL -framework ApplicationServices \
            -framework CoreFoundation

SRCS := src/$(PLUGIN).cpp \
        third_party/imgui/imgui.cpp third_party/imgui/imgui_draw.cpp \
        third_party/imgui/imgui_tables.cpp third_party/imgui/imgui_widgets.cpp \
        third_party/ImgWindow/ImgWindow.cpp third_party/ImgWindow/ImgFontAtlas.cpp

OUT := build/mac_x64/$(PLUGIN).xpl

$(OUT): $(SRCS)
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
