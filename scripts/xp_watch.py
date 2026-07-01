#!/usr/bin/env python3
"""xp_watch — find which X-Plane dataref a cockpit control drives.

The recurring problem when building an aircraft profile: you flip a switch in the
3D cockpit and need to know *which* dataref (or command) it moved. DataRefTool
shows values, but the panel is a firehose — hundreds of datarefs tick on their own
(flight model, engines, time, weather), drowning the one you care about.

This tool treats it like a noise measurement: it samples the control-surface
datarefs, learns which ones change on their OWN (the noise floor), then — after you
flip the switch — reports only what changed that ISN'T noise. Usually one line.

Usage:
    scripts/xp_watch.py                  # interactive: baseline → flip switch → diff
    scripts/xp_watch.py --filter ice     # only datarefs whose name contains 'ice'
    scripts/xp_watch.py --all            # watch every dataref (slower, catches anything)
    scripts/xp_watch.py --secs 3         # longer noise-learning window
    scripts/xp_watch.py --base URL       # default http://127.0.0.1:8086

Needs the X-Plane Web API on (Settings → Network). Read-only; it never writes.
"""
import argparse
import concurrent.futures as cf
import json
import sys
import time
import urllib.request

# Substrings that mark a dataref as a likely COCKPIT SWITCH/control — the default
# watch set, kept TIGHT so a sample reads a few hundred datarefs, not thousands
# (broad hints like "prop"/"fuel"/"light" match half the sim and cause read
# failures). This covers the switch namespaces: sim/cockpit*/switches/*, the
# add-on's laminar/<acf>/*/switch/*, the ice panel, and the generic light banks.
# Use --filter <substr> for a specific system, or --all to watch everything.
CONTROL_HINTS = (
    "/switch/", "switches/", "generic_lights", "/ice/", "icerain",
    "/actuators/", "/electrical/switch",
)
# Never worth watching — internal/graphics/gauge noise, even if the name happens
# to match a control hint or filter (e.g. "device" contains "ice").
NOISE_HINTS = ("_needle", "gauge", "annun", "_pos_deg", "brightness",
               "sim/private", "/stats/", "managed_buffers", "/gfx/")


def _get(url):
    with urllib.request.urlopen(url, timeout=8) as r:
        return json.load(r)


def list_datarefs(base, filt, all_):
    data = _get(f"{base}/api/v1/datarefs")["data"]
    out = {}
    for d in data:
        n = d["name"]
        low = n.lower()
        if any(h in low for h in NOISE_HINTS):
            continue  # internal/graphics/gauge noise — never a control
        if filt:
            if filt.lower() not in low:
                continue
        elif not all_ and not any(h in low for h in CONTROL_HINTS):
            continue
        out[n] = d["id"]
    return out


def read_all(base, ids):
    """Read every (name→id) value concurrently. Returns name→raw value."""
    def one(item):
        name, i = item
        try:
            with urllib.request.urlopen(f"{base}/api/v1/datarefs/{i}/value", timeout=5) as r:
                return name, json.load(r).get("data")
        except Exception:
            return name, None
    snap = {}
    with cf.ThreadPoolExecutor(max_workers=32) as ex:
        for name, val in ex.map(one, ids.items()):
            snap[name] = val
    return snap


def changed(a, b, thr):
    """Did the value move MEANINGFULLY? A switch jumps by ~1 (or a rheostat by
    tenths); analog gauges (N1, oil/fuel pressure, turn rate) jitter by <0.01 and
    must not register. So numbers compare against a threshold, arrays element-wise,
    everything else exact."""
    if a is None or b is None:
        return False  # a read failed this pass — ignore, don't call a glitch a change
    if isinstance(a, bool) or isinstance(b, bool):
        return a != b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(b - a) >= thr
    if isinstance(a, list) and isinstance(b, list):
        return len(a) != len(b) or any(changed(x, y, thr) for x, y in zip(a, b))
    return a != b


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default="http://127.0.0.1:8086")
    ap.add_argument("--filter", default="")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--secs", type=float, default=10.0, help="noise-learning window")
    ap.add_argument("--samples", type=int, default=8)
    ap.add_argument("--threshold", type=float, default=0.05,
                    help="min change to count as real (filters analog jitter)")
    args = ap.parse_args()

    print("resolving datarefs…", flush=True)
    ids = list_datarefs(args.base, args.filter, args.all)
    if not ids:
        print("no datarefs matched — is the Web API up? try --all or --filter", file=sys.stderr)
        sys.exit(1)
    print(f"watching {len(ids)} datarefs", flush=True)

    # Noise floor: sample repeatedly while nothing is touched; anything that
    # varies across samples is noise. Keep the last snapshot as the baseline.
    print(f"learning the noise floor for {args.secs:.0f}s — don't touch anything…", flush=True)
    noise = set()
    first = read_all(args.base, ids)
    base_snap = first
    for _ in range(max(1, args.samples - 1)):
        time.sleep(args.secs / args.samples)
        cur = read_all(args.base, ids)
        # Compare to the FIRST sample, not the previous one, so a slow drift that
        # creeps under the threshold step-to-step still trips it over the full
        # window (that's the whole point of a longer --secs).
        for n, v in cur.items():
            if changed(first.get(n), v, args.threshold):
                noise.add(n)
        base_snap = cur
    stable = len(ids) - len(noise)
    print(f"noise floor: {len(noise)} changing, {stable} stable.", flush=True)

    input(">>> flip the switch/knob now, then press Enter…")

    # Two reads after the flip: a real switch steps to a new STABLE value, whereas
    # analog drift (a running engine's prop_ratio, N1, pressures) keeps moving. So
    # report a dataref only if it changed from baseline AND has now settled (the two
    # post-flip samples agree) — that filters the slow drift the noise pass missed.
    after1 = read_all(args.base, ids)
    time.sleep(0.5)
    after2 = read_all(args.base, ids)
    moved = []
    for n, v in after2.items():
        if n in noise:
            continue
        if not changed(base_snap.get(n), v, args.threshold):
            continue
        if changed(after1.get(n), v, args.threshold):
            continue  # still drifting → analog noise, not a switch
        moved.append((n, base_snap.get(n), v))

    if not moved:
        print("nothing changed and settled. Try a longer --secs, a lower --threshold, or a bigger switch throw.")
        return
    print(f"\n{len(moved)} dataref(s) moved:")
    for n, old, new in sorted(moved):
        print(f"  {n}\n      {old}  ->  {new}")


if __name__ == "__main__":
    main()
