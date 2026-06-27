# x-copilot X-Plane plugin

A tiny native plugin that gives x-copilot a **dedicated push-to-talk** inside the
sim. It registers:

- a command **`xcopilot/ptt`** — bind it to any joystick/HOTAS button or key in
  X-Plane → Settings → Keyboard/Joystick;
- a read-only dataref **`xcopilot/ptt_active`** — `1` while that command is held,
  `0` otherwise.

The x-copilot app polls `xcopilot/ptt_active` over the X-Plane Web API and opens
the mic only while it's `1`. Unlike `sim/operation/contact_atc_ptt`, this does
**not** key the radio, and unlike a global keyboard hook it needs **no macOS
Input-Monitoring permission** — the button press is read by X-Plane itself.

A status display (copilot phase / last heard / last action) is the planned phase 2.

## Build (macOS)

Needs the free X-Plane SDK from <https://developer.x-plane.com/sdk/>. Either copy
it to `plugin/SDK` (so `SDK/CHeaders/XPLM/*.h` and `SDK/Libraries/Mac/XPLM.framework`
exist), or pass its path with `SDK=...`:

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

## Wire up x-copilot

Point the app at the dataref:

```
export XCOPILOT_PTT_DATAREF=xcopilot/ptt_active
```

Hold your bound button to talk; release to send.
