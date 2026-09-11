# XVoiceLayer X-Plane plugin

A tiny native plugin that gives XVoiceLayer a **dedicated push-to-talk** inside the
sim. It registers:

- a command **`xcopilot/ptt`** — bind it to any joystick/HOTAS button or key in
  X-Plane → Settings → Keyboard/Joystick;
- a read-only dataref **`xcopilot/ptt_active`** — `1` while that command is held,
  `0` otherwise.

The XVoiceLayer app polls `xcopilot/ptt_active` over the X-Plane Web API and opens
the mic only while it's `1`. Unlike `sim/operation/contact_atc_ptt`, this does
**not** key the radio, and unlike a global keyboard hook it needs **no macOS
Input-Monitoring permission** — the button press is read by X-Plane itself.

The panel is a Dear ImGui window (Flight | Transcript | ATC | Route | Settings)
with a status strip of annunciators along the bottom. It is drawn by X-Plane's
own panel-graphics renderer (SDK 4.4, `src/imgui_xp.cpp`): ImGui's vertex
buffers go to `XPLMDrawCalls`, its font atlas to `XPLMCreateTexture`, so there
is no OpenGL in the plugin at all and the panel costs what a stock avionics
screen costs. The Transcript tab has a typed-command line (Enter sends the
words on the same queue the voice uses) and the window remembers its place,
size and open state across sessions in `Output/preferences/xvoicelayer_panel.txt`.

## Vendored libraries

`third_party/` holds Dear ImGui (MIT, 1.92) and the B612 typeface (OFL) —
committed so the build is self-contained.

## Build (macOS)

Needs the free X-Plane SDK **4.4.0 or newer** from
<https://developer.x-plane.com/sdk/> (the plugin needs X-Plane **12.4.4+** to
load). Either copy it to `plugin/SDK` (so `SDK/CHeaders/XPLM/XPLMPanelGraphics.h`
and `SDK/Libraries/Mac/XPLM.framework` exist), or pass its path with `SDK=...`:

```
make
make SDK=/path/to/SDK
```

Output: `build/mac_x64/xcopilot.xpl` (universal arm64 + x86_64).

## Install

```
make install
make install XPLANE="/path/to/X-Plane 12"
```

This copies the `.xpl` to `<X-Plane 12>/Resources/plugins/xcopilot/mac_x64/`.
Restart X-Plane, then bind a button to the `xcopilot/ptt` command.

## Wire up XVoiceLayer

Point the app at the dataref:

```
export XCOPILOT_PTT_DATAREF=xcopilot/ptt_active
```

Hold your bound button to talk; release to send.
