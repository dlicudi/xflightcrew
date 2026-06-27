#!/usr/bin/env python3
"""LLM tool-pick latency benchmark — compare models *and* engines (GGUF vs MLX).

Usage (needs a live `ollama serve`; from repo root):
    python3 bench/llm_latency.py qwen3:8b
    python3 bench/llm_latency.py qwen3:8b qwen2.5:7b-instruct <mlx-model-tag>

Measures the latency variable that actually matters for x-copilot: the
per-command cost of one tool-pick request, broken into **prefill** (processing
the prompt) vs **generation**, with a realistic, app-sized payload — ~32 tool
schemas + the system prompt + a live state line, ~3k tokens. Prefill of that
prompt is the dominant cost, so this is where GGUF (llama.cpp/Metal) vs MLX
shows up.

It runs three scenarios per model, mirroring real flight:
  cold    first call — KV cache empty (worst case / cache miss)
  warm    same prompt again — full prefix cache hit (best case)
  inflight  same system+tools, but the *state* changed — what every in-flight
            command actually is. Because we fold state into the LAST (user)
            message (see src/llm.rs chat_call), the stable system+tools prefix
            still caches, so this should land near `warm`, not `cold`. That delta
            is the KV-cache win; comparing `cold`/`inflight` across engines is the
            GGUF-vs-MLX comparison.

Engine is read back from /api/ps (`format`: gguf = llama.cpp/Metal; non-gguf =
MLX-eligible). NOTE: the tool schemas here are *representative*, sized to match
the real registry's token count — not byte-identical to it. We measure deltas;
absolute numbers track the machine.
"""
import json, sys, time, statistics, urllib.request

OLLAMA = "http://127.0.0.1:11434"
NUM_CTX = 8192          # matches src/llm.rs options.num_ctx
REPEATS = 4             # timed samples per scenario (median reported)

# A stand-in for the UNIFIED_PROMPT — same order of magnitude in tokens.
SYSTEM = (
    "You are the copilot of an aircraft, working with the pilot. Choose ONE of "
    "two responses. ACT: if the pilot wants something done, call the matching "
    "tool(s), one call per distinct action, including polite or indirect phrasing. "
    "ANSWER: only when the pilot asks about the flight and there is nothing to do, "
    "reply in one short spoken sentence using digits for numbers. Use only the "
    "state you are given; never invent numbers. Prefer ACT whenever an action is "
    "possible, and never both act and answer."
)

# ~32 representative tools, mirroring the C172/SR22 registry surface so the
# prompt's token count is in the real ~3k ballpark.
def build_tools():
    def tool(name, desc, props, required):
        return {"type": "function", "function": {
            "name": name, "description": desc,
            "parameters": {"type": "object", "properties": props, "required": required}}}
    enum = lambda *v: {"type": "string", "enum": list(v)}
    onoff = {"on": {"type": "boolean"}}
    D = ("Set the {0}; speech may be slightly misheard so map the closest phrasing. "
         "Re-reads the dataref after writing and confirms the actual value.")
    specs = [
        ("set_flaps", "flaps", {"notch": enum("up", "10", "20", "full")}, ["notch"]),
        ("set_lights", "lights", {"light": enum("landing", "taxi", "beacon", "strobe", "nav"), **onoff}, ["light", "on"]),
        ("set_flood_lights", "flood lights", {**onoff, "percent": {"type": "integer"}}, []),
        ("set_heading", "autopilot heading", {"heading": {"type": "integer"}}, ["heading"]),
        ("set_altitude", "autopilot altitude target", {"altitude": {"type": "integer"}}, ["altitude"]),
        ("set_vertical_speed", "autopilot vertical speed", {"fpm": {"type": "integer"}}, ["fpm"]),
        ("set_climb", "climb/descend to an altitude at a rate", {"altitude": {"type": "integer"}, "fpm": {"type": "integer"}}, ["altitude", "fpm"]),
        ("level_off", "level off at the current altitude", {}, []),
        ("autopilot_off", "disengage the autopilot", {}, []),
        ("set_throttle", "throttle", {"percent": {"type": "integer"}}, ["percent"]),
        ("set_mixture", "mixture", {"percent": {"type": "integer"}}, ["percent"]),
        ("start_engine", "start the engine", {}, []),
        ("stop_engine", "shut down the engine", {}, []),
        ("set_magnetos", "magnetos", {"position": enum("off", "left", "right", "both", "start")}, ["position"]),
        ("set_carb_heat", "carb heat", onoff, ["on"]),
        ("set_fuel_pump", "fuel pump", onoff, ["on"]),
        ("set_fuel_selector", "fuel selector", {"position": enum("left", "right", "both")}, ["position"]),
        ("set_pitot_heat", "pitot heat", onoff, ["on"]),
        ("set_pitch_trim", "pitch trim", {"action": enum("nose_up", "nose_down", "takeoff")}, ["action"]),
        ("set_parking_brake", "parking brake", onoff, ["on"]),
        ("set_electrical", "electrical system", {"system": enum("battery", "alternator", "avionics", "master", "all"), **onoff}, ["system", "on"]),
        ("set_com1", "com 1 frequency", {"mhz": {"type": "number"}}, ["mhz"]),
        ("set_com2", "com 2 frequency", {"mhz": {"type": "number"}}, ["mhz"]),
        ("set_nav1", "nav 1 frequency", {"mhz": {"type": "number"}}, ["mhz"]),
        ("set_obs", "OBS course", {"degrees": {"type": "integer"}}, ["degrees"]),
        ("swap_radio", "swap a radio's standby and active", {"radio": enum("com1", "com2", "nav1")}, ["radio"]),
        ("set_transponder", "transponder code", {"code": {"type": "integer"}}, ["code"]),
        ("set_transponder_mode", "transponder mode", {"mode": enum("standby", "altitude", "ident")}, ["mode"]),
        ("set_barometer", "altimeter setting", {"inhg": {"type": "number"}}, ["inhg"]),
        ("align_gyro", "align the directional gyro to the compass", {}, []),
        ("prepare_ils", "prepare the ILS for a runway", {"runway": {"type": "string"}}, ["runway"]),
        ("normal_landing", "run the landing checklist", {}, []),
    ]
    return [tool(n, D.format(d), p, r) for (n, d, p, r) in specs]

TOOLS = build_tools()

def chat(model, state, user, think_off, system=SYSTEM):
    body = {
        "model": model, "stream": False, "keep_alive": "10m",
        "messages": [
            {"role": "system", "content": system},
            # State folded into the user turn (matches src/llm.rs chat_call).
            {"role": "user", "content": f"{user}\n\n(current aircraft state: {state})"},
        ],
        "tools": TOOLS,
        "options": {"temperature": 0, "num_ctx": NUM_CTX},
    }
    if think_off:
        body["think"] = False
    req = urllib.request.Request(
        f"{OLLAMA}/api/chat", data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"})
    t = time.time()
    d = json.loads(urllib.request.urlopen(req, timeout=300).read())
    wall = time.time() - t
    m = d.get("message", {})
    return {
        "wall": wall,
        "prefill": d.get("prompt_eval_duration", 0) / 1e9,
        "gen": d.get("eval_duration", 0) / 1e9,
        "ptok": d.get("prompt_eval_count", 0),
        "gtok": d.get("eval_count", 0),
        "tool": bool(m.get("tool_calls")),
        "thinking": bool(m.get("thinking") or "<think>" in (m.get("content") or "")),
    }

def engine_of(model):
    try:
        d = json.loads(urllib.request.urlopen(f"{OLLAMA}/api/ps", timeout=10).read())
        for m in d.get("models", []):
            if m["name"] == model or m["name"].startswith(model):
                fmt = m.get("details", {}).get("format", "?")
                return f"{fmt} ({'llama.cpp/Metal' if fmt == 'gguf' else 'MLX-eligible'})"
    except Exception:
        pass
    return "?"

S1 = "altitude 3500 ft, heading 010, vertical speed 0 fpm, flaps 0%, engines running"
S2 = "altitude 3640 ft, heading 014, vertical speed +210 fpm, flaps 0%, engines running"

def med(xs):
    return statistics.median(xs)

def bench(model):
    think_off = model.startswith("qwen3:")
    # Warm the model + prime the cache with one throwaway call.
    chat(model, S1, "flaps ten", think_off)

    def samples(state, user):
        return [chat(model, state, user, think_off) for _ in range(REPEATS)]

    # cold: force a TRUE full prefill (cache miss) on a warm model by giving the
    # system prompt a never-seen marker, so the whole ~2.5k-token prefix is
    # reprocessed. This isolates raw prefill throughput — the number that differs
    # between GGUF (llama.cpp/Metal) and MLX. (Model already loaded, so no weight-
    # load time is conflated in.)
    cold = chat(model, "altitude 1200 ft, heading 270, on the ground", "gear up", think_off,
                system=SYSTEM + " [bench-cold-miss-7f3a]")
    warm = samples(S1, "flaps ten")                       # identical prompt → cache hit
    inflight = [chat(model, S2, "flaps twenty", think_off),  # state changed each call
                chat(model, "altitude 3700 ft, heading 020, vertical speed +180 fpm", "gear up", think_off),
                chat(model, "altitude 3820 ft, heading 025, vertical speed +160 fpm", "throttle 60 percent", think_off)]

    r0 = warm[0]
    print(f"\n  {model}")
    print(f"    engine:   {engine_of(model)}")
    print(f"    prompt:   {r0['ptok']} tok in, {r0['gtok']} tok out   "
          f"think_off={think_off} thinking={r0['thinking']} tool={r0['tool']}")
    print(f"    {'scenario':<10}{'prefill':>10}{'gen':>9}{'total':>9}")
    def row(name, rs):
        if isinstance(rs, dict):
            rs = [rs]
        print(f"    {name:<10}{med([r['prefill'] for r in rs]):>9.2f}s"
              f"{med([r['gen'] for r in rs]):>8.2f}s{med([r['wall'] for r in rs]):>8.2f}s")
    row("cold", cold)
    row("warm", warm)
    row("inflight", inflight)

def main():
    models = sys.argv[1:] or ["qwen3:8b"]
    print(f"LLM latency bench — {len(TOOLS)} tools, num_ctx={NUM_CTX}, {REPEATS} samples/scenario")
    print("  cold=cache miss  warm=same prompt (cache hit)  inflight=changing state (the real case)")
    for m in models:
        try:
            bench(m)
        except Exception as e:
            print(f"\n  {m}\n    FAILED: {e}")
    print()

if __name__ == "__main__":
    main()
