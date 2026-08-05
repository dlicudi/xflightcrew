#!/usr/bin/env python3
"""Turn logs/blackbox.jsonl into subtitles for a flight recording.

Every spoken line the blackbox carries is already timestamped, so the captions
for a sortie video come out of the flight itself rather than being transcribed
by hand. Also emits YouTube-style chapter markers from the automation stages.

    scripts/blackbox_srt.py --zero 14:31:07 -o flight.srt
    scripts/blackbox_srt.py --chapters -o chapters.txt

The one number you must supply is --zero: the wall clock at which the video
recording starts. Everything else defaults to the last flight in the log.

Not every voice line reaches the blackbox: the copilot's tool readbacks are
spoken but never recorded, so they are absent from the captions until a
"speech" record exists. What IS covered: pilot utterances, every controller
transmission, ATIS, callouts, taxi cues, traffic advisories and cabin crew.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import sys

# kind -> (speaker, field). The speaker is a caption prefix, so keep it short
# enough to read at a glance on a 1080p frame.
SPEECH = {
    "heard": (None, "transcript"),  # speaker resolved per event (radio or not)
    "atc": (None, "reply"),  # speaker is the `service` field
    "atis": ("ATIS", "msg"),
    "vectors": ("APPROACH", "msg"),
    "atc_handoff": ("TOWER", "msg"),
    "traffic_advisory": ("ATC", "msg"),
    "atc_region": (None, "msg"),  # `crew` true means the copilot said it
    "callout": ("COPILOT", "msg"),
    "taxi_guide": ("COPILOT", "cue"),
    "cabin": ("CABIN", "reply"),
}

# kind -> field holding the stage name, for chapter markers.
STAGES = {
    "full_auto": "stage",
    "circuit": "event",
    "procedure": "event",
    "autotakeoff": "event",
    "pushback": "event",
    "taxi_in": "event",
}

# Reading speed, characters per second, for caption duration.
CPS = 15.0
MIN_SECS = 1.4
MAX_SECS = 7.0


def scrub(text: str) -> str:
    """Dashes choke the panel font and the sim's TTS; captions match the rest."""
    return text.replace(" — ", ", ").replace("—", ",").replace("–", "-")


def parse_clock(value: str, day: dt.date) -> float:
    """Accept a unix time, an ISO timestamp, or a bare HH:MM:SS on `day`."""
    try:
        return float(value)
    except ValueError:
        pass
    try:
        return dt.datetime.fromisoformat(value).timestamp()
    except ValueError:
        pass
    parts = [int(p) for p in value.split(":")]
    while len(parts) < 3:
        parts.append(0)
    return dt.datetime.combine(day, dt.time(*parts)).timestamp()


def srt_time(secs: float) -> str:
    if secs < 0:
        secs = 0.0
    ms = int(round(secs * 1000))
    h, ms = divmod(ms, 3_600_000)
    m, ms = divmod(ms, 60_000)
    s, ms = divmod(ms, 1000)
    return f"{h:02d}:{m:02d}:{s:02d},{ms:03d}"


def stamp(secs: float) -> str:
    m, s = divmod(int(secs), 60)
    h, m = divmod(m, 60)
    return f"{h}:{m:02d}:{s:02d}" if h else f"{m}:{s:02d}"


def speaker_for(event: dict, default: str | None) -> str:
    kind = event.get("kind")
    if kind == "heard":
        return "PILOT (radio)" if event.get("radio") else "PILOT"
    if kind == "atc":
        return str(event.get("service", "ATC")).upper()
    if kind == "atc_region":
        return "COPILOT" if event.get("crew") else "ATC"
    return default or "ATC"


def load(path: str) -> list[dict]:
    events = []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(event, dict) and isinstance(event.get("t"), (int, float)):
                events.append(event)
    events.sort(key=lambda e: e["t"])
    return events


def last_flight_start(events: list[dict], gap_secs: float) -> float:
    """Start of the most recent sortie: the last `flight_meta`, or failing that
    the far side of the last long gap in the log (one session per file is the
    exception, not the rule)."""
    for event in reversed(events):
        if event.get("kind") == "flight_meta":
            return event["t"]
    previous = None
    start = events[0]["t"] if events else 0.0
    for event in events:
        if previous is not None and event["t"] - previous > gap_secs:
            start = event["t"]
        previous = event["t"]
    return start


def collect(events: list[dict], since: float, until: float, want_chapters: bool, drop_ghosts: bool = False):
    captions, chapters = [], []
    last_stage = None
    for event in events:
        when = event["t"]
        if when < since or when > until:
            continue
        kind = event.get("kind")
        if kind in SPEECH:
            if drop_ghosts and event.get("route") == "party_line":
                continue
            default_speaker, field = SPEECH[kind]
            text = event.get(field)
            if isinstance(text, str) and text.strip():
                # A party-line exchange is logged as one record with the calls
                # slashed together; break it onto its own lines so the caption
                # reads like the radio sounds.
                body = "\n".join(part.strip() for part in scrub(text.strip()).split(" / "))
                captions.append((when, speaker_for(event, default_speaker), body))
        elif want_chapters and kind in STAGES:
            stage = event.get(STAGES[kind])
            if isinstance(stage, str) and stage and (kind, stage) != last_stage:
                last_stage = (kind, stage)
                chapters.append((when, f"{kind.replace('_', ' ')}: {stage.replace('_', ' ')}"))
    return captions, chapters


def render_srt(captions, zero: float, labels: bool) -> str:
    blocks = []
    for index, (when, speaker, text) in enumerate(captions, start=1):
        start = when - zero
        want = max(MIN_SECS, min(MAX_SECS, len(text) / CPS))
        if index < len(captions):
            # Never overlap the next line; a caption may be cut short but is
            # always at least readable-length or gone.
            want = min(want, max(MIN_SECS, captions[index][0] - when - 0.05))
        body = f"{speaker}: {text}" if labels else text
        blocks.append(f"{index}\n{srt_time(start)} --> {srt_time(start + want)}\n{body}\n")
    return "\n".join(blocks)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", nargs="?", default="logs/blackbox.jsonl")
    parser.add_argument("-o", "--out", help="write here instead of stdout")
    parser.add_argument("--zero", help="wall clock the recording starts (unix, ISO or HH:MM:SS); default is the first event")
    parser.add_argument("--since", help="first event to include (unix, ISO or HH:MM:SS)")
    parser.add_argument("--until", help="last event to include")
    parser.add_argument("--gap-secs", type=float, default=900.0, help="silence that separates two sorties (default 900)")
    parser.add_argument("--chapters", action="store_true", help="emit chapter markers instead of subtitles")
    parser.add_argument("--no-labels", action="store_true", help="drop the speaker prefixes")
    parser.add_argument("--no-ghosts", action="store_true", help="drop party-line chatter between other traffic")
    args = parser.parse_args()

    events = load(args.log)
    if not events:
        print(f"no events in {args.log}", file=sys.stderr)
        return 1

    day = dt.datetime.fromtimestamp(events[-1]["t"]).date()
    since = parse_clock(args.since, day) if args.since else last_flight_start(events, args.gap_secs)
    until = parse_clock(args.until, day) if args.until else float("inf")
    zero = parse_clock(args.zero, day) if args.zero else since

    captions, chapters = collect(events, since, until, args.chapters, args.no_ghosts)
    if args.chapters:
        text = "\n".join(f"{stamp(when - zero)} {title}" for when, title in chapters) + "\n"
        count, what = len(chapters), "chapters"
    else:
        text = render_srt(captions, zero, not args.no_labels)
        count, what = len(captions), "captions"

    if args.out:
        with open(args.out, "w", encoding="utf-8") as fh:
            fh.write(text)
    else:
        sys.stdout.write(text)

    window = dt.datetime.fromtimestamp(since).strftime("%H:%M:%S")
    print(f"{count} {what} from the sortie starting {window}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
