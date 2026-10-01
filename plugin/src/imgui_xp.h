// Dear ImGui on X-Plane's native panel-graphics renderer (SDK 4.4.0,
// X-Plane 12.4.4+). One X-Plane window of content type PanelGraphics hosts one
// ImGui context. A per-frame flight loop runs the ImGui frame (input, the
// interface, Render) and answers the atlas's texture requests with
// XPLMCreateTexture/XPLMDestroyTexture; the window's draw callback then does
// ONE thing, establish the window's top-left ImGui transform and hand the
// retained draw data to XPLMDrawCalls. No OpenGL, no GL-on-Metal bridge, no
// hand-rolled renderer: the sim draws the panel with the same machinery as
// its own avionics.
//
// The split is a rule of the sim, not a style choice: panel graphics
// FORBIDS creating or destroying a draw-call texture from inside a draw
// callback and aborts the whole application on the first violation (the
// header's "use in callbacks" note does not say which; XLua's own ImGui
// window creates its atlas texture at context creation and never in draw,
// and this plugin took X-Plane down with it on 2026-09-10 before learning
// that). ImGui's draw data stays valid until the next NewFrame, so a frame
// built in the flight loop is safely drawn by the callback that follows it.
//
// This replaces the vendored ImgWindow (Christopher Collins, BSD-3), which
// rendered through the legacy fixed-pipeline OpenGL path and bound the font
// atlas once, for ever (the reason UI scale used to be a font SWAP between
// three pre-baked sizes). With ImGui 1.92's dynamic atlas and the texture
// update protocol below, fonts rasterise at whatever size the frame asks for.

#pragma once

#include <string>
#include <vector>

#include "XPLMDisplay.h"
#include "XPLMPanelGraphics.h"
#include "XPLMProcessing.h"

#include "imgui.h"

class XpImguiWindow {
public:
    virtual ~XpImguiWindow();
    XpImguiWindow(const XpImguiWindow&) = delete;
    XpImguiWindow& operator=(const XpImguiWindow&) = delete;

    void SetWindowTitle(const std::string& title);
    void SetWindowResizingLimits(int minW, int minH, int maxW, int maxH);
    void SetVisible(bool visible);
    bool GetVisible() const;
    // Global-desktop boxels, the coordinate space the window was created in.
    void GetWindowGeometry(int& left, int& top, int& right, int& bottom) const;
    void SetWindowGeometry(int left, int top, int right, int bottom);
    bool IsPoppedOut() const { return XPLMWindowIsPoppedOut(mWindowID) != 0; }
    bool IsInVR() const { return XPLMWindowIsInVR(mWindowID) != 0; }
    bool IsInsideSim() const { return !IsPoppedOut() && !IsInVR(); }
    XPLMWindowID GetWindowId() const { return mWindowID; }

protected:
    XpImguiWindow(int left, int top, int right, int bottom,
                  XPLMWindowDecoration decoration = xplm_WindowDecorationRoundRectangle,
                  XPLMWindowLayer layer = xplm_WindowLayerFloatingWindows);

    // Extra flags for the full-window ImGui::Begin (menu bar, etc.).
    virtual ImGuiWindowFlags beforeBegin() { return ImGuiWindowFlags_None; }
    // The interface, called every frame the window is drawn, inside a
    // borderless ImGui window that fills the X-Plane window.
    virtual void buildInterface() = 0;

private:
    static float LoopCB(float, float, int, void* refcon);
    static void DrawCB(XPLMWindowID, void* refcon);
    static int MouseCB(XPLMWindowID, int x, int y, XPLMMouseStatus, void* refcon);
    static int RightMouseCB(XPLMWindowID, int x, int y, XPLMMouseStatus, void* refcon);
    static void KeyCB(XPLMWindowID, char key, XPLMKeyFlags, char vkey, void* refcon,
                      int losingFocus);
    static XPLMCursorStatus CursorCB(XPLMWindowID, int x, int y, void* refcon);
    static int WheelCB(XPLMWindowID, int x, int y, int wheel, int clicks, void* refcon);

    // The ImGui frame plus texture uploads: flight loop only, never draw.
    void frame();
    // The draw calls: draw callback only.
    void render();
    void mouse(int x, int y, XPLMMouseStatus status, int button);
    // Global boxels to window-local ImGui pixels (top-left origin).
    void toLocal(int x, int y, float& lx, float& ly) const;
    void moveForVR();

    XPLMWindowID mWindowID = nullptr;
    XPLMFlightLoopID mLoop = nullptr;
    ImGuiContext* mCtx = nullptr;
    // A frame has been rendered since the last NewFrame and may be drawn.
    bool mHaveFrame = false;
    std::string mTitle;
    XPLMWindowLayer mPreferredLayer;
    // Geometry of the frame being drawn; mouse callbacks translate against it.
    int mLeft = 0, mTop = 0, mRight = 0, mBottom = 0;
    // Reused per frame so the draw path never allocates in steady state.
    std::vector<XPLMDrawCall_t> mCalls;
    // Last frame's hover verdict for the cursor callback (a button under the
    // pointer gets the hand; a text field the I-beam).
    bool mHoverItem = false;
    ImGuiMouseCursor mCursor = ImGuiMouseCursor_Arrow;
};
