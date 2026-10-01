// Dear ImGui on X-Plane's native panel-graphics renderer. See imgui_xp.h.

#include "imgui_xp.h"

#include <cfloat>
#include <cstdint>
#include <cstring>

#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"

// ---- sim datarefs -------------------------------------------------------------

static XPLMDataRef g_frame_period = nullptr;
static XPLMDataRef g_vr_enabled = nullptr;

// ---- key translation ----------------------------------------------------------

// X-Plane virtual key to ImGui key. Letters and digits are contiguous in both
// tables; the rest is enumerated. Unmapped keys are simply not reported (the
// character still arrives through the text path).
static ImGuiKey MapKey(unsigned char vk) {
    if (vk >= XPLM_VK_A && vk <= XPLM_VK_Z) return (ImGuiKey)(ImGuiKey_A + (vk - XPLM_VK_A));
    if (vk >= XPLM_VK_0 && vk <= XPLM_VK_9) return (ImGuiKey)(ImGuiKey_0 + (vk - XPLM_VK_0));
    if (vk >= XPLM_VK_NUMPAD0 && vk <= XPLM_VK_NUMPAD9)
        return (ImGuiKey)(ImGuiKey_Keypad0 + (vk - XPLM_VK_NUMPAD0));
    if (vk >= XPLM_VK_F1 && vk <= XPLM_VK_F12) return (ImGuiKey)(ImGuiKey_F1 + (vk - XPLM_VK_F1));
    switch (vk) {
        case XPLM_VK_TAB: return ImGuiKey_Tab;
        case XPLM_VK_LEFT: return ImGuiKey_LeftArrow;
        case XPLM_VK_RIGHT: return ImGuiKey_RightArrow;
        case XPLM_VK_UP: return ImGuiKey_UpArrow;
        case XPLM_VK_DOWN: return ImGuiKey_DownArrow;
        case XPLM_VK_PRIOR: return ImGuiKey_PageUp;
        case XPLM_VK_NEXT: return ImGuiKey_PageDown;
        case XPLM_VK_HOME: return ImGuiKey_Home;
        case XPLM_VK_END: return ImGuiKey_End;
        case XPLM_VK_INSERT: return ImGuiKey_Insert;
        case XPLM_VK_DELETE: return ImGuiKey_Delete;
        case XPLM_VK_BACK: return ImGuiKey_Backspace;
        case XPLM_VK_SPACE: return ImGuiKey_Space;
        case XPLM_VK_RETURN: return ImGuiKey_Enter;
        case XPLM_VK_ENTER: return ImGuiKey_KeypadEnter;
        case XPLM_VK_NUMPAD_ENT: return ImGuiKey_KeypadEnter;
        case XPLM_VK_ESCAPE: return ImGuiKey_Escape;
        case XPLM_VK_COMMA: return ImGuiKey_Comma;
        case XPLM_VK_PERIOD: return ImGuiKey_Period;
        case XPLM_VK_MINUS: return ImGuiKey_Minus;
        case XPLM_VK_EQUAL: return ImGuiKey_Equal;
        case XPLM_VK_SLASH: return ImGuiKey_Slash;
        case XPLM_VK_BACKSLASH: return ImGuiKey_Backslash;
        case XPLM_VK_SEMICOLON: return ImGuiKey_Semicolon;
        case XPLM_VK_QUOTE: return ImGuiKey_Apostrophe;
        case XPLM_VK_LBRACE: return ImGuiKey_LeftBracket;
        case XPLM_VK_RBRACE: return ImGuiKey_RightBracket;
        case XPLM_VK_BACKQUOTE: return ImGuiKey_GraveAccent;
        case XPLM_VK_ADD: return ImGuiKey_KeypadAdd;
        case XPLM_VK_SUBTRACT: return ImGuiKey_KeypadSubtract;
        case XPLM_VK_MULTIPLY: return ImGuiKey_KeypadMultiply;
        case XPLM_VK_DIVIDE: return ImGuiKey_KeypadDivide;
        case XPLM_VK_DECIMAL: return ImGuiKey_KeypadDecimal;
        default: return ImGuiKey_None;
    }
}

// ---- textures ---------------------------------------------------------------

// ImGui 1.92's texture protocol: the atlas grows and re-rasterises on demand
// and asks the backend to create/update/destroy GPU textures each frame. The
// sim's texture API is create-from-RGBA8 and destroy, both cheap enough to
// call per frame, so an update is a fresh upload of the whole buffer and a
// retirement of the old handle. Called from the flight loop and the
// destructor ONLY (see the header: a draw callback may not create or
// destroy textures, and the sim enforces it by aborting).
static void UpdateTexture(ImTextureData* tex) {
    switch (tex->Status) {
        case ImTextureStatus_WantCreate:
        case ImTextureStatus_WantUpdates: {
            void* old = tex->Status == ImTextureStatus_WantUpdates
                ? reinterpret_cast<void*>(static_cast<intptr_t>(tex->TexID))
                : nullptr;
            void* fresh = XPLMCreateTexture(
                static_cast<const unsigned char*>(tex->GetPixels()), tex->Width, tex->Height);
            if (old != nullptr) XPLMDestroyTexture(old);
            tex->SetTexID(static_cast<ImTextureID>(reinterpret_cast<intptr_t>(fresh)));
            tex->SetStatus(ImTextureStatus_OK);
            break;
        }
        case ImTextureStatus_WantDestroy: {
            void* h = reinterpret_cast<void*>(static_cast<intptr_t>(tex->TexID));
            if (h != nullptr) XPLMDestroyTexture(h);
            tex->SetTexID(ImTextureID_Invalid);
            tex->SetStatus(ImTextureStatus_Destroyed);
            break;
        }
        default:
            break;
    }
}

// ---- lifecycle --------------------------------------------------------------

XpImguiWindow::XpImguiWindow(int left, int top, int right, int bottom,
                             XPLMWindowDecoration decoration, XPLMWindowLayer layer)
    : mPreferredLayer(layer) {
    if (g_frame_period == nullptr) {
        g_frame_period = XPLMFindDataRef("sim/operation/misc/frame_rate_period");
        g_vr_enabled = XPLMFindDataRef("sim/graphics/VR/enabled");
    }

    mCtx = ImGui::CreateContext();
    ImGui::SetCurrentContext(mCtx);
    ImGuiIO& io = ImGui::GetIO();
    io.BackendPlatformName = "xplane";
    io.BackendRendererName = "xplane-panel-graphics";
    // We honour the atlas's texture requests every frame (dynamic fonts).
    io.BackendFlags |= ImGuiBackendFlags_RendererHasTextures;
    // XPLMDrawCall_t carries a base vertex, so meshes past 64k vertices work.
    io.BackendFlags |= ImGuiBackendFlags_RendererHasVtxOffset;
    io.Fonts->TexDesiredFormat = ImTextureFormat_RGBA32;
    // The sim reports Ctrl as Ctrl and Cmd not at all; keep ImGui's PC idiom.
    io.ConfigMacOSXBehaviors = false;
    // The X-Plane decoration owns moving and resizing.
    io.ConfigWindowsResizeFromEdges = false;
    io.ConfigWindowsMoveFromTitleBarOnly = true;
    io.IniFilename = nullptr;
    io.LogFilename = nullptr;

    XPLMCreateWindow_t p;
    std::memset(&p, 0, sizeof p);
    p.structSize = sizeof p;
    p.left = left;
    p.top = top;
    p.right = right;
    p.bottom = bottom;
    p.visible = 0;
    p.drawWindowFunc = DrawCB;
    p.handleMouseClickFunc = MouseCB;
    p.handleKeyFunc = KeyCB;
    p.handleCursorFunc = CursorCB;
    p.handleMouseWheelFunc = WheelCB;
    p.refcon = this;
    p.decorateAsFloatingWindow = decoration;
    p.layer = layer;
    p.handleRightClickFunc = RightMouseCB;
    p.contentType = xplm_WindowContentTypePanelGraphics;
    mWindowID = XPLMCreateWindowEx(&p);

    XPLMCreateFlightLoop_t fl;
    std::memset(&fl, 0, sizeof fl);
    fl.structSize = sizeof fl;
    fl.phase = xplm_FlightLoop_Phase_BeforeFlightModel;
    fl.callbackFunc = LoopCB;
    fl.refcon = this;
    mLoop = XPLMCreateFlightLoop(&fl);
    XPLMScheduleFlightLoop(mLoop, -1.0f, 1);
}

XpImguiWindow::~XpImguiWindow() {
    if (mLoop != nullptr) {
        XPLMDestroyFlightLoop(mLoop);
        mLoop = nullptr;
    }
    ImGui::SetCurrentContext(mCtx);
    // Retire every GPU texture the atlas still holds before the context goes.
    for (ImTextureData* tex : ImGui::GetPlatformIO().Textures) {
        if (tex->TexID != ImTextureID_Invalid) {
            tex->SetStatus(ImTextureStatus_WantDestroy);
            UpdateTexture(tex);
        }
    }
    XPLMDestroyWindow(mWindowID);
    ImGui::DestroyContext(mCtx);
}

void XpImguiWindow::SetWindowTitle(const std::string& title) {
    mTitle = title;
    XPLMSetWindowTitle(mWindowID, mTitle.c_str());
}

void XpImguiWindow::SetWindowResizingLimits(int minW, int minH, int maxW, int maxH) {
    XPLMSetWindowResizingLimits(mWindowID, minW, minH, maxW, maxH);
}

void XpImguiWindow::GetWindowGeometry(int& l, int& t, int& r, int& b) const {
    XPLMGetWindowGeometry(mWindowID, &l, &t, &r, &b);
}

void XpImguiWindow::SetWindowGeometry(int l, int t, int r, int b) {
    XPLMSetWindowGeometry(mWindowID, l, t, r, b);
}

void XpImguiWindow::SetVisible(bool visible) {
    if (visible) moveForVR();
    if (GetVisible() == visible) return;
    XPLMSetWindowIsVisible(mWindowID, visible ? 1 : 0);
}

bool XpImguiWindow::GetVisible() const { return XPLMGetWindowIsVisible(mWindowID) != 0; }

void XpImguiWindow::moveForVR() {
    if (g_vr_enabled != nullptr && XPLMGetDatai(g_vr_enabled)) {
        XPLMSetWindowPositioningMode(mWindowID, xplm_WindowVR, 0);
    } else if (IsInVR()) {
        XPLMSetWindowPositioningMode(mWindowID, mPreferredLayer, -1);
    }
}

// ---- frame ------------------------------------------------------------------

void XpImguiWindow::toLocal(int x, int y, float& lx, float& ly) const {
    lx = static_cast<float>(x - mLeft);
    ly = static_cast<float>(mTop - y);
}

void XpImguiWindow::frame() {
    if (!GetVisible()) {
        mHaveFrame = false;
        return;
    }
    ImGui::SetCurrentContext(mCtx);
    ImGuiIO& io = ImGui::GetIO();

    XPLMGetWindowGeometry(mWindowID, &mLeft, &mTop, &mRight, &mBottom);
    const float w = static_cast<float>(mRight - mLeft);
    const float h = static_cast<float>(mTop - mBottom);
    io.DisplaySize = ImVec2(w, h);
    io.DisplayFramebufferScale = ImVec2(1.0f, 1.0f);
    float dt = g_frame_period ? XPLMGetDataf(g_frame_period) : 0.0f;
    io.DeltaTime = dt > 0.0f ? dt : 1.0f / 60.0f;

    // The cursor callback only fires while the pointer is over the window,
    // so a pointer that LEFT would keep the last hover lit for ever. Poll the
    // global pointer once a frame (in-sim only: a popped-out window lives in
    // OS coordinates) and tell ImGui when it is gone.
    if (IsInsideSim()) {
        int mx = 0, my = 0;
        XPLMGetMouseLocationGlobal(&mx, &my);
        if (mx < mLeft || mx > mRight || my < mBottom || my > mTop) {
            io.AddMousePosEvent(-FLT_MAX, -FLT_MAX);
        }
    }

    ImGui::NewFrame();
    ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(w, h), ImGuiCond_Always);
    ImGui::Begin(mTitle.empty() ? "##xp" : mTitle.c_str(), nullptr,
                 beforeBegin() | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoSavedSettings);
    buildInterface();
    mHoverItem = ImGui::IsAnyItemHovered();
    ImGui::End();

    // Keyboard focus follows text fields: take it while one is editing, hand
    // it back the moment none is, so sim key commands keep working.
    const int has = XPLMHasKeyboardFocus(mWindowID);
    if (io.WantTextInput && !has) {
        XPLMTakeKeyboardFocus(mWindowID);
    } else if (!io.WantTextInput && has) {
        XPLMTakeKeyboardFocus(nullptr);
    }
    mCursor = ImGui::GetMouseCursor();

    ImGui::Render();
    // Texture requests are answered HERE, in the flight loop, so the draw
    // callback below finds every handle ready and never has to create one.
    ImDrawData* dd = ImGui::GetDrawData();
    if (dd->Textures != nullptr) {
        for (ImTextureData* tex : *dd->Textures) {
            if (tex->Status != ImTextureStatus_OK) UpdateTexture(tex);
        }
    }
    mHaveFrame = dd->Valid;
}

void XpImguiWindow::render() {
    static_assert(sizeof(ImDrawVert) == 5 * sizeof(float), "XPLMMesh_t expects ImDrawVert's 20-byte layout");
    static_assert(sizeof(ImDrawIdx) == sizeof(uint16_t), "XPLMMesh_t expects 16-bit indices");
    if (!mHaveFrame) return;
    ImGui::SetCurrentContext(mCtx);
    ImDrawData* dd = ImGui::GetDrawData();
    if (dd == nullptr || !dd->Valid) return;
    const float w = dd->DisplaySize.x, h = dd->DisplaySize.y;

    // X-Plane no longer gives XPLMDrawCalls a hidden window-local, top-left
    // coordinate system.  Like the mouse path in toLocal(), explicitly move
    // the panel origin to the window's top-left and turn its Y axis down for
    // ImGui.  Keep this scope around both vertices and their scissors.
    XPLMTransformPush();
    XPLMTransformTranslate(static_cast<float>(mLeft), static_cast<float>(mTop));
    XPLMTransformScale(1.0f, -1.0f);
    for (ImDrawList* list : dd->CmdLists) {
        if (list->VtxBuffer.Size == 0 || list->IdxBuffer.Size == 0) continue;
        XPLMMesh_t mesh;
        mesh.vertex_count = list->VtxBuffer.Size;
        mesh.vertices = reinterpret_cast<const float*>(list->VtxBuffer.Data);
        mesh.index_count = list->IdxBuffer.Size;
        mesh.indices = reinterpret_cast<const uint16_t*>(list->IdxBuffer.Data);
        mCalls.clear();
        for (const ImDrawCmd& cmd : list->CmdBuffer) {
            if (cmd.UserCallback != nullptr) {
                // No render-state reset to honour; user callbacks are unused.
                continue;
            }
            if (cmd.ElemCount == 0) continue;
            // Clip to the window: the sim's scissor is in window-local pixels
            // and a zero-area rect draws nothing, which is also right.
            float l = cmd.ClipRect.x - dd->DisplayPos.x, t = cmd.ClipRect.y - dd->DisplayPos.y;
            float r = cmd.ClipRect.z - dd->DisplayPos.x, b = cmd.ClipRect.w - dd->DisplayPos.y;
            if (l < 0.0f) l = 0.0f;
            if (t < 0.0f) t = 0.0f;
            if (r > w) r = w;
            if (b > h) b = h;
            if (r <= l || b <= t) continue;
            const ImTextureID tid = cmd.GetTexID();
            if (tid == ImTextureID_Invalid) continue;  // not uploaded yet: skip, never create here
            XPLMDrawCall_t c;
            c.tex_ref = reinterpret_cast<void*>(static_cast<intptr_t>(tid));
            c.scissors[0] = l;
            c.scissors[1] = t;
            c.scissors[2] = r;
            c.scissors[3] = b;
            c.idx_offset = static_cast<int>(cmd.IdxOffset);
            c.element_count = static_cast<int>(cmd.ElemCount);
            c.vtx_offset = static_cast<int>(cmd.VtxOffset);
            mCalls.push_back(c);
        }
        if (!mCalls.empty()) XPLMDrawCalls(&mesh, static_cast<int>(mCalls.size()), mCalls.data());
    }
    XPLMTransformPop();
}

// ---- callbacks --------------------------------------------------------------

float XpImguiWindow::LoopCB(float, float, int, void* refcon) {
    static_cast<XpImguiWindow*>(refcon)->frame();
    return -1.0f;  // every frame
}

void XpImguiWindow::DrawCB(XPLMWindowID, void* refcon) {
    static_cast<XpImguiWindow*>(refcon)->render();
}

void XpImguiWindow::mouse(int x, int y, XPLMMouseStatus status, int button) {
    ImGui::SetCurrentContext(mCtx);
    ImGuiIO& io = ImGui::GetIO();
    float lx, ly;
    toLocal(x, y, lx, ly);
    io.AddMousePosEvent(lx, ly);
    switch (status) {
        case xplm_MouseDown: io.AddMouseButtonEvent(button, true); break;
        case xplm_MouseUp: io.AddMouseButtonEvent(button, false); break;
        default: break;  // drag: position only, the button is already down
    }
}

int XpImguiWindow::MouseCB(XPLMWindowID, int x, int y, XPLMMouseStatus s, void* refcon) {
    static_cast<XpImguiWindow*>(refcon)->mouse(x, y, s, 0);
    return 1;
}

int XpImguiWindow::RightMouseCB(XPLMWindowID, int x, int y, XPLMMouseStatus s, void* refcon) {
    static_cast<XpImguiWindow*>(refcon)->mouse(x, y, s, 1);
    return 1;
}

void XpImguiWindow::KeyCB(XPLMWindowID, char key, XPLMKeyFlags flags, char vkey, void* refcon,
                          int losingFocus) {
    XpImguiWindow* self = static_cast<XpImguiWindow*>(refcon);
    ImGui::SetCurrentContext(self->mCtx);
    ImGuiIO& io = ImGui::GetIO();
    if (losingFocus) {
        // Drops every held key and modifier, so nothing sticks "down".
        io.AddFocusEvent(false);
        io.AddFocusEvent(true);
        return;
    }
    const bool down = (flags & xplm_DownFlag) != 0;
    io.AddKeyEvent(ImGuiMod_Shift, (flags & xplm_ShiftFlag) != 0);
    io.AddKeyEvent(ImGuiMod_Alt, (flags & xplm_OptionAltFlag) != 0);
    io.AddKeyEvent(ImGuiMod_Ctrl, (flags & xplm_ControlFlag) != 0);
    const ImGuiKey k = MapKey(static_cast<unsigned char>(vkey));
    if (k != ImGuiKey_None) io.AddKeyEvent(k, down);
    // The character path: the sim already applied shift and layout, so only
    // printable ASCII arrives here, on the down edge.
    const unsigned char c = static_cast<unsigned char>(key);
    if (down && c >= 0x20 && c != 0x7F) io.AddInputCharacter(c);
}

XPLMCursorStatus XpImguiWindow::CursorCB(XPLMWindowID, int x, int y, void* refcon) {
    XpImguiWindow* self = static_cast<XpImguiWindow*>(refcon);
    ImGui::SetCurrentContext(self->mCtx);
    float lx, ly;
    self->toLocal(x, y, lx, ly);
    ImGui::GetIO().AddMousePosEvent(lx, ly);
    if (self->mCursor == ImGuiMouseCursor_TextInput) return xplm_CursorText;
    if (self->mHoverItem) return xplm_CursorButton;
    return xplm_CursorArrow;
}

int XpImguiWindow::WheelCB(XPLMWindowID, int x, int y, int wheel, int clicks, void* refcon) {
    XpImguiWindow* self = static_cast<XpImguiWindow*>(refcon);
    ImGui::SetCurrentContext(self->mCtx);
    ImGuiIO& io = ImGui::GetIO();
    float lx, ly;
    self->toLocal(x, y, lx, ly);
    io.AddMousePosEvent(lx, ly);
    if (wheel == 0) io.AddMouseWheelEvent(0.0f, static_cast<float>(clicks));
    else if (wheel == 1) io.AddMouseWheelEvent(static_cast<float>(clicks), 0.0f);
    return 1;
}
