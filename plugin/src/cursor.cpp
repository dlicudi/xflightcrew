// Keep the mouse pointer visible.
//
// X-Plane on macOS hides the pointer two ways, with no preference behind
// either: [NSCursor setHiddenUntilMouseMoves:YES] on a key press and
// [NSCursor hide] when the mouse sits idle (both in the binary; the pilot
// kept losing the pointer over the cockpit, 2026-09-24). Cancelling the first
// every frame was not enough, so while this plugin is loaded both class
// methods are REPLACED in the process: `hide` does nothing and
// `setHiddenUntilMouseMoves:` always passes NO. `unhide` is left alone (an
// unhide with nothing hidden is harmless). The originals go back on plugin
// stop. No per-frame cost. The trade: the pointer also stays visible during a
// right-drag look, which X-Plane used to hide.
//
// macOS only, through the Objective-C runtime so the plugin stays C++.

#include "cursor.h"

#if defined(__APPLE__)
#include <objc/message.h>
#include <objc/runtime.h>
#endif

namespace {

#if defined(__APPLE__)
Method gHide = nullptr;
IMP gOrigHide = nullptr;
Method gHiddenUntil = nullptr;
IMP gOrigHiddenUntil = nullptr;

void no_hide(id, SEL) {}

void never_hidden_until_moves(id self, SEL sel, BOOL) {
    reinterpret_cast<void (*)(id, SEL, BOOL)>(gOrigHiddenUntil)(self, sel, NO);
}
#endif

} // namespace

namespace cursor {

void start() {
#if defined(__APPLE__)
    Class cls = objc_getClass("NSCursor");
    if (!cls) return;
    gHide = class_getClassMethod(cls, sel_registerName("hide"));
    if (gHide) {
        gOrigHide = method_setImplementation(gHide, reinterpret_cast<IMP>(no_hide));
    }
    gHiddenUntil = class_getClassMethod(cls, sel_registerName("setHiddenUntilMouseMoves:"));
    if (gHiddenUntil) {
        gOrigHiddenUntil =
            method_setImplementation(gHiddenUntil, reinterpret_cast<IMP>(never_hidden_until_moves));
        // Undo a hide already pending when we loaded.
        never_hidden_until_moves(reinterpret_cast<id>(cls), sel_registerName("setHiddenUntilMouseMoves:"), NO);
    }
#endif
}

void stop() {
#if defined(__APPLE__)
    if (gHide && gOrigHide) {
        method_setImplementation(gHide, gOrigHide);
    }
    if (gHiddenUntil && gOrigHiddenUntil) {
        method_setImplementation(gHiddenUntil, gOrigHiddenUntil);
    }
    gHide = gHiddenUntil = nullptr;
    gOrigHide = gOrigHiddenUntil = nullptr;
#endif
}

} // namespace cursor
