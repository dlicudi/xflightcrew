#!/usr/bin/env python3
"""Global route tables, generated offline (docs/tasks/global-routes/PLAN.md, stage 2).

    python3 -I tools/routes/build.py <cache_dir> --out <dir>

writes <dir>/fields.yaml, <dir>/carriers/<CODE>.yaml and <dir>/build.log, and
refits tools/routes/params.json. Python 3.9, stdlib only. Every download is
cached in <cache_dir> (1 request a second, a descriptive User-Agent, no
account), so a second run over the same cache gives the same bytes.

Network: every article transcluding {{Airport destination list}}, its
airlines and destinations resolved through Wikidata (P239, P230, P432,
P625, P421). Frequency: BTS T-100 per operator, else the Eurostat pair
total split by the heuristic's weights, else the heuristic, which is fitted
on BTS, Eurostat and UK CAA counts. CAA counts only FIT the heuristic and
are never written into a row (`check_no_caa`). Hand rows in overrides.yaml
win their pairs whole.
"""
import argparse, datetime, json, math, os, re, sys, time, unicodedata, zlib, zoneinfo
from collections import Counter, defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import freq, sources  # noqa: E402

REPO = os.path.dirname(os.path.dirname(HERE))
PERF = os.path.join(REPO, "crates", "traffic", "src", "perf.rs")
OVERRIDES = os.path.join(HERE, "overrides.yaml")
PARAMS = os.path.join(HERE, "params.json")
BTS_MONTHS = [(2026, 1), (2026, 7)]  # winter, summer
SOURCES = ("hand", "bts", "eurostat", "heuristic")
MAX_WEEK = 49  # the loader's ceiling: seven a day
# Still-air range, km, of every type the performance tables know: a row's
# types are the carrier's that reach its distance.
RANGE_KM = {
    "AT43": 1300, "AT45": 1300, "AT72": 1500, "AT76": 1500, "DH8A": 1500, "DH8B": 1500, "DH8C": 1500,
    "DH8D": 2000, "B350": 2500, "BE20": 2500, "BE9L": 2000, "C208": 1500, "PC12": 3000, "SF34": 1700,
    "D328": 1600, "JS41": 1400, "CRJ7": 2600, "CRJ9": 2900, "E170": 3700, "E175": 3700, "E190": 4500,
    "E195": 4200, "BCS1": 5700, "BCS3": 5700, "MD82": 4000, "MD83": 4600, "MD88": 4000, "A318": 5700,
    "A319": 6800, "A320": 6100, "A321": 5900, "A19N": 6800, "A20N": 6400, "A21N": 7400, "B736": 5900,
    "B737": 6200, "B738": 5400, "B739": 5000, "B38M": 6500, "B39M": 6500, "B752": 7200, "B753": 6300,
    "B763": 11000, "B764": 10000, "B772": 13000, "B77W": 13600, "B77L": 15800, "B788": 13500,
    "B789": 14000, "B78X": 11900, "A332": 13400, "A333": 11700, "A339": 13300, "A343": 13000,
    "A346": 14600, "A359": 15000, "A35K": 16000, "B744": 13400, "B748": 14300, "MD11": 12600,
    "A388": 15000,
}
# A carrier nothing tells us the fleet of flies these, by its routes' reach.
DEFAULT_FLEET = {"regional": [("AT76", 100)], "narrow": [("A320", 50), ("B738", 50)], "wide": [("B789", 50), ("A333", 50)]}


# ---------- inputs ----------

def known_types():
    """Every type perf.rs's class table knows; fails loudly when it cannot be read."""
    try:
        src = open(PERF, encoding="utf-8").read()
    except OSError as e:
        sys.exit("cannot read the performance tables %s: %s" % (PERF, e))
    out = set()
    for name in ("HEAVY", "SUPER", "MEDIUM_JET", "TURBOPROPS"):
        m = re.search(r"const %s: &\[&str\] = &\[(.*?)\];" % name, src, re.S)
        if not m:
            sys.exit("cannot read %s from %s: the known-type set is unknown" % (name, PERF))
        out |= set(re.findall(r'"([A-Z0-9]{3,4})"', m.group(1)))
    missing = sorted(out - set(RANGE_KM))
    if missing:
        sys.exit("types known to perf.rs with no range in build.py RANGE_KM: %s" % missing)
    return out


def scalar(tok):
    if tok.startswith('"'):
        return tok[1:-1].replace('\\"', '"').replace("\\\\", "\\")
    for f in (int, float):
        try:
            return f(tok)
        except ValueError:
            pass
    return {"true": True, "false": False}.get(tok, tok)


def flow(text):
    """One YAML flow value ({..}, [..] or a scalar) of the subset we write."""
    toks = re.findall(r'"(?:[^"\\]|\\.)*"|[{}\[\],:]|[^{}\[\],:\s][^{}\[\],:]*', text)
    toks = [t.strip() for t in toks]
    pos = [0]

    def val():
        t = toks[pos[0]]; pos[0] += 1
        if t == "{":
            d = {}
            while toks[pos[0]] != "}":
                k = scalar(toks[pos[0]]); pos[0] += 2
                d[k] = val()
                if toks[pos[0]] == ",": pos[0] += 1
            pos[0] += 1
            return d
        if t == "[":
            a = []
            while toks[pos[0]] != "]":
                a.append(val())
                if toks[pos[0]] == ",": pos[0] += 1
            pos[0] += 1
            return a
        return scalar(t)
    return val()


def read_yaml(path):
    """The overrides file: top-level sections of `- flow` items or `key: flow` entries."""
    out, sec = {}, None
    for line in open(path, encoding="utf-8"):
        s = re.sub(r'\s+#.*$|^\s*#.*$', "", line.rstrip("\n"))
        if not s.strip():
            continue
        if not s.startswith(" "):
            sec = s.rstrip(":").strip()
            out[sec] = None
        elif s.strip().startswith("- "):
            out[sec] = (out[sec] or []) + [flow(s.strip()[2:])]
        else:
            k, v = re.match(r'\s+("[^"]*"|[^:]+):\s*(.*)$', s).groups()
            out[sec] = out[sec] or {}
            out[sec][scalar(k.strip())] = flow(v)
    return out


# ---------- small helpers ----------

def ascii_text(s):
    s = re.sub(r"[‐-―−]", " ", s)
    s = unicodedata.normalize("NFKD", s).encode("ascii", "ignore").decode()
    return re.sub(r"\s+", " ", re.sub(r"\(.*?\)|'''?|\[|\]", "", s)).strip()


def no_dash(s):
    """Article titles keep their letters; their en and em dashes become hyphens."""
    return re.sub(r"[\u2010-\u2015\u2212]", "-", s)


def norm(s):
    return re.sub(r"[^a-z0-9]", "", ascii_text(s).lower())


def km(p, q):
    la, lb = math.radians(p[0]), math.radians(q[0])
    h = math.sin((lb - la) / 2) ** 2 + math.cos(la) * math.cos(lb) * math.sin(math.radians(q[1] - p[1]) / 2) ** 2
    return 2 * 6371.0 * math.asin(min(1.0, math.sqrt(h)))


def first(vals, pat):
    for v in vals or []:
        if isinstance(v, str) and re.fullmatch(pat, v):
            return v
    return None


def zone_clock(name):
    """(standard offset h, keeps summer time in the northern summer) of an IANA zone."""
    z = zoneinfo.ZoneInfo(name)
    jan = datetime.datetime(2026, 1, 15, tzinfo=z).utcoffset().total_seconds() / 3600
    jul = datetime.datetime(2026, 7, 15, tzinfo=z).utcoffset().total_seconds() / 3600
    return min(jan, jul), jul > jan, jan > jul


def offsets_of(tz):
    out = set(tz.get("P2907", []))
    m = re.match(r"UTC\s*([+−-])\s*(\d\d):(\d\d)$", tz.get("label", ""))
    if m:
        out.add((-1 if m.group(1) != "+" else 1) * (int(m.group(2)) + int(m.group(3)) / 60))
    if tz.get("label", "") in ("UTC", "UTC±00:00", "Coordinated Universal Time"):
        out.add(0.0)
    return out


def time_zones(http, ents, place_q, coords, log):
    """{icao: (utc_offset_h, dst)} from P421, walking up P131 then P17 to the first level that has one."""
    def fetch(ids):
        miss = sorted(set(i for i in ids if i and i not in ents))
        ents.update(sources.entities(http, miss))
    level = {i: [q] for i, q in place_q.items()}
    found = {}
    for depth in range(6):
        fetch(q for qs in level.values() for q in qs)
        nxt = {}
        for icao, qs in level.items():
            tzs = sorted({t for q in qs for t in ents.get(q, {}).get("P421", [])})
            if tzs:
                found[icao] = tzs
            else:
                up = sorted({u for q in qs for u in ents.get(q, {}).get("P131" if depth < 4 else "P17", [])})
                if up:
                    nxt[icao] = up[:4]
        level = nxt
        if not level:
            break
    countries = {i: ents.get(q, {}).get("P17", [])[:1] for i, q in place_q.items()}
    fetch(c for cs in countries.values() for c in cs)
    fetch(t for ts in found.values() for t in ts)
    fetch(t for cs in countries.values() for c in cs for t in ents.get(c, {}).get("P421", []))
    out, how = {}, Counter()

    def iana(ts):
        zs = []
        for t in ts:
            for z in ents.get(t, {}).get("P6687", []):
                try:
                    zs.append((z, zone_clock(z)))
                except (zoneinfo.ZoneInfoNotFoundError, ValueError):
                    pass
        return sorted(zs)
    for icao in sorted(place_q):
        ts = found.get(icao, [])
        zs = iana(ts)
        if zs:
            std, dst, south = zs[0][1]
            how["iana"] += 1
        else:
            offs = sorted({o for t in ts for o in offsets_of(ents.get(t, {}))})
            if offs and offs[-1] - offs[0] <= 1:
                std, dst, south = offs[0], len(offs) > 1, False
                cz = [c for z, c in iana(t for c in countries[icao] for t in ents.get(c, {}).get("P421", []))
                      if c[0] == std]
                if cz:
                    dst, south = cz[0][1], cz[0][2]
                how["offset"] += 1
            else:
                std, dst, south = round(coords[icao][1] / 15.0), False, False
                how["longitude"] += 1
                log["timezone"].append("%s: no usable P421 (%s), offset from longitude %+d" % (icao, ",".join(ts) or "none", std))
        if south:
            how["southern summer time not modelled"] += 1
        out[icao] = (std, dst)
    return out, how


# ---------- the build ----------

def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("cache_dir")
    ap.add_argument("--out", required=True, help="output directory (never routes/ by default)")
    ap.add_argument("--no-hand", action="store_true", help="leave the hand rows out, to compare against them")
    a = ap.parse_args()
    t0 = time.time()
    known = known_types()
    ov = read_yaml(OVERRIDES)
    type_map, codes = ov.get("type_map") or {}, ov.get("codes") or {}
    brands = {norm(k): v for k, v in (ov.get("brands") or {}).items()}
    log = defaultdict(list)
    http = sources.Http(a.cache_dir)

    # 1. fields and 2. network
    print("wikipedia: articles", file=sys.stderr)
    arts = sources.articles(http, sources.embedded(http))
    raw = {t: sources.rows_from(arts[t][2]) for t in sorted(arts)}
    print("wikipedia: links", file=sys.stderr)
    link_q = sources.qids(http, [r[0] for rs in raw.values() for r in rs] + [r[2] for rs in raw.values() for r in rs])
    print("wikidata: entities", file=sys.stderr)
    ents = sources.entities(http, set(link_q.values()) | {arts[t][1] for t in arts})

    def icao_of(q):
        return first(ents.get(q, {}).get("P239"), r"[A-Z]{4}")

    field_of, place_qs = {}, defaultdict(set)
    for t in sorted(arts):
        rev, q, text = arts[t]
        icao = icao_of(q) or sources.infobox_icao(text)
        if not icao:
            if raw[t]:
                log["unresolved field"].append("%s (%s): no ICAO, %d rows dropped" % (t, q, len(raw[t])))
            continue
        field_of[t] = icao
        if q:
            place_qs[icao].add(q)
    info, unresolved = {}, defaultdict(list)

    def carrier_of(target, label):
        q = link_q.get(target)
        for k in (target, label, ents.get(q, {}).get("label", "")):
            if norm(k) in brands:
                return tuple(brands[norm(k)])
        code = first(ents.get(q, {}).get("P230"), r"[A-Z]{3}")
        if not code:
            return None
        code = codes.get(code, code)
        if code not in info:
            e = ents[q]
            info[code] = {"name": ascii_text(e["label"] or label), "iata": first(e.get("P229"), r"[A-Z0-9]{2}"),
                          "telephony": first(e.get("P432"), r".+"), "q": q}
        return (code,)

    cand, labels = {}, defaultdict(Counter)
    for t in sorted(field_of):
        at = field_of[t]
        for atg, alb, dtg, dlb, flags in raw[t]:
            what = "%s %s -> %s" % (at, ascii_text(alb), ascii_text(dlb))
            drop = [f for f in flags if f in ("charter", "future", "suspended")]
            if drop:
                log["dropped row"].append("%s: %s" % (what, drop[0]))
                continue
            ops = carrier_of(atg, alb)
            if not ops:
                unresolved[ascii_text(atg)].append(at)
                log["dropped row"].append("%s: airline unresolved" % what)
                continue
            other = icao_of(link_q.get(dtg))
            if not other:
                log["dropped row"].append("%s: destination has no ICAO (%s)" % (what, ascii_text(dtg)))
                continue
            if other == at:
                continue
            place_qs[other].add(link_q[dtg])
            labels[other][ascii_text(dlb)] += 1
            key = (ops, min(at, other), max(at, other))
            c = cand.setdefault(key, {"seasonal": True, "listed": set(), "arts": set()})
            c["seasonal"] &= "seasonal" in flags
            c["listed"].add(at)
            c["arts"].add(t)

    # Places: coordinates and clocks for every end.
    hand = [] if a.no_hand else ov.get("hand") or []
    ends = {e for k in cand for e in k[1:]} | {r["at"] for r in hand} | {r["other"] for r in hand}
    place_q, coords = {}, {}
    for icao in sorted(ends):
        qs = sorted(place_qs.get(icao, ()), key=lambda q: (icao_of(q) != icao, "P625" not in ents.get(q, {}), q))
        if qs and "P625" in ents.get(qs[0], {}):
            place_q[icao], coords[icao] = qs[0], ents[qs[0]]["P625"][0]
    lost = sorted({r["at"] for r in hand} | {r["other"] for r in hand})
    lost = [e for e in lost if e not in coords]
    if lost:
        sys.exit("hand rows name places Wikidata did not give: %s" % lost)
    for k in sorted(cand):
        if k[1] not in coords or k[2] not in coords:
            log["dropped row"].append("%s %s-%s: no coordinates for %s" % ("/".join(k[0]), k[1], k[2],
                                      k[1] if k[1] not in coords else k[2]))
            del cand[k]
    print("wikidata: time zones", file=sys.stderr)
    clocks, tz_how = time_zones(http, ents, place_q, coords, log)

    # 4. BTS per operator, brands resolved to the operator flying most of the pair.
    print("frequency: BTS, Eurostat, CAA", file=sys.stderr)
    bts = freq.load_bts(http, a.cache_dir, BTS_MONTHS)
    iata = {i: first(ents.get(q, {}).get("P238"), r"[A-Z]{3}") for i, q in place_q.items()}
    for code, o in (ov.get("carriers") or {}).items():
        info.setdefault(code, {"name": code, "iata": None, "telephony": None, "q": None})
        info[code].update({k: v for k, v in o.items() if k in ("name", "iata", "telephony")})

    def bts_on(op, x, y):
        cia, xi, yi = (info.get(op) or {}).get("iata"), iata.get(x), iata.get(y)
        if not (cia and xi and yi):
            return []
        return [e for e in (bts.get((cia, xi, yi)), bts.get((cia, yi, xi))) if e]

    routes = {}
    for (ops, x, y), c in sorted(cand.items()):
        got = {op: bts_on(op, x, y) for op in ops}
        tot = {op: sum(sum(e["dep"].values()) for e in es) for op, es in got.items()}
        op = max(ops, key=lambda o: (tot[o], -ops.index(o)))
        es = [e for o in ops for e in got[o]]
        r = routes.setdefault((op, x, y), {"carrier": op, "a": x, "b": y, "seasonal": True, "listed": set(),
                                            "arts": set(), "bts": None})
        r["seasonal"] &= c["seasonal"]
        r["listed"] |= c["listed"]
        r["arts"] |= c["arts"]
        if es:
            w = sum(e["dep"].get(1, 0) for e in es) * freq.WEEK
            s = sum(e["dep"].get(7, 0) for e in es) * freq.WEEK
            ty = Counter()
            for e in es:
                ty.update(e["types"])
            if not r["bts"] or s + w > sum(r["bts"][:2]):
                r["bts"] = (s, w, ty)

    # The network's shape: what the heuristic reads.
    per_end, per_carrier, per_pair = Counter(), Counter(), Counter()
    for r in routes.values():
        per_end[r["a"]] += 1; per_end[r["b"]] += 1; per_carrier[r["carrier"]] += 1; per_pair[(r["a"], r["b"])] += 1
    for r in routes.values():
        r.update(km=km(coords[r["a"]], coords[r["b"]]), n_a=per_end[r["a"]], n_b=per_end[r["b"]],
                 n_carrier=per_carrier[r["carrier"]], k_pair=per_pair[(r["a"], r["b"])])

    # 5. the heuristic, fitted on BTS, Eurostat and CAA
    euro, euro_used = freq.load_eurostat(http)
    names = {"place": defaultdict(set), "carrier": {}}
    for icao in place_q:
        names["place"][icao] |= freq.words(ents[place_q[icao]]["label"])
        for lab in labels.get(icao, {}):
            names["place"][icao] |= freq.words(lab)
    for t, icao in field_of.items():
        names["place"][icao] |= freq.words(t)
    for code, i in info.items():
        names["carrier"][code] = freq.words(i["name"] or "")
    caa = freq.caa_samples(http, a.cache_dir, routes, names)
    samples = []
    for k, r in sorted(routes.items()):
        if r["bts"]:
            samples.append((freq.features(r), r["bts"][0], r["bts"][1], "bts"))
        pair = frozenset((r["a"], r["b"]))
        if r["k_pair"] == 1 and pair in euro:
            samples.append((freq.features(r), euro[pair][0], euro[pair][1], "eurostat"))
        if k in caa:
            samples.append((freq.features(r), caa[k][0], caa[k][1], "caa"))
    params = freq.fit(samples)
    params["data"] = {"bts_months": ["%d-%02d" % m for m in BTS_MONTHS], "eurostat": euro_used,
                      "caa_months": ["2025-07", "2026-01"]}
    with open(PARAMS, "w") as f:
        json.dump(params, f, indent=1, sort_keys=True)
        f.write("\n")

    # 4 again: each route's frequency, first source that has it.
    hand_pairs = {frozenset((h["at"], h["other"])) for h in hand}
    by_pair = defaultdict(list)
    for k, r in sorted(routes.items()):
        if frozenset((r["a"], r["b"])) in hand_pairs:
            log["replaced by hand rows"].append("%s %s-%s" % k)
            continue
        by_pair[(r["a"], r["b"])].append(r)
    for pair, rs in sorted(by_pair.items()):
        tot = euro.get(frozenset(pair))
        for r in rs:
            x = freq.features(r)
            r["pred"] = (freq.predict(params["summer"], x), freq.predict(params["winter"], x))
        rest = [r for r in rs if not r["bts"]]
        for r in rs:
            if r["bts"]:
                r["freq"], r["src"] = r["bts"][:2], "bts"
        if tot and rest:
            left = [max(0.0, tot[i] - sum(r["bts"][i] for r in rs if r["bts"])) for i in (0, 1)]
            for i in (0, 1):
                wsum = sum(r["pred"][i] for r in rest)
                for r in rest:
                    share = r["pred"][i] / wsum if wsum > 0 else 1.0 / len(rest)
                    r.setdefault("split", [0.0, 0.0])[i] = left[i] * share
            for r in rest:
                r["freq"], r["src"] = tuple(r["split"]), "eurostat"
        for r in rest:
            if "src" not in r:
                r["freq"], r["src"] = r["pred"], "heuristic"

    # 6. types
    def to_known(ty, why):
        out = Counter()
        for t, n in ty.items():
            k = t if t in known else type_map.get(t)
            if k in known:
                out[k] += n
                if k != t:
                    log["type mapped"].append("%s -> %s (%s)" % (t, k, why))
            else:
                log["type dropped"].append("%s (%s)" % (t, why))
        tot = sum(out.values())
        return [(t, max(1, round(100 * n / tot))) for t, n in sorted(out.items(), key=lambda kv: (-kv[1], kv[0]))
                if n / tot >= 0.03] if tot else []

    fleets = {}
    for code in sorted({r["carrier"] for rs in by_pair.values() for r in rs} | {h["carrier"] for h in hand}):
        o = (ov.get("carriers") or {}).get(code, {})
        if o.get("types"):
            fleets[code] = [(t, w) for t, w in o["types"]]
            continue
        cia = info.get(code, {}).get("iata")
        flown = any(r["bts"] for rs in by_pair.values() for r in rs if r["carrier"] == code)
        ty = Counter()
        if cia and flown:
            for (c, _, _), e in bts.items():
                if c == cia:
                    ty.update(e["types"])
        fleet = to_known(ty, code + " fleet from BTS") if ty else []
        if not fleet:
            dists = sorted(r["km"] for rs in by_pair.values() for r in rs if r["carrier"] == code) or [1000.0]
            fleet = list(DEFAULT_FLEET["regional" if dists[len(dists) // 2] < 600 else "narrow"])
            if dists[-1] > RANGE_KM[fleet[0][0]]:
                fleet += DEFAULT_FLEET["narrow"] if fleet[0][0] == "AT76" else []
            if dists[-1] > 5400:
                fleet += DEFAULT_FLEET["wide"]
            log["fleet defaulted"].append("%s: %s" % (code, " ".join(t for t, _ in fleet)))
        fleets[code] = fleet

    rows = defaultdict(list)
    for pair, rs in sorted(by_pair.items()):
        for r in rs:
            s, w = (min(MAX_WEEK, int(round(v))) for v in r["freq"])
            if max(r["freq"]) > MAX_WEEK + 0.5:
                log["capped at %d a week" % MAX_WEEK].append("%s %s-%s %.0f" % (r["carrier"], r["a"], r["b"], max(r["freq"])))
            if s == 0 and w == 0:
                s, w = (1, 0) if r["freq"][0] >= r["freq"][1] else (0, 1)
            types = to_known(r["bts"][2], "%s %s-%s" % (r["carrier"], r["a"], r["b"])) if r["src"] == "bts" else []
            if not types:
                types = [(t, n) for t, n in fleets[r["carrier"]] if RANGE_KM[t] >= r["km"] and n > 0]
                if not types:
                    log["dropped row"].append("%s %s-%s: no type in the fleet reaches %.0f km" % (r["carrier"], r["a"], r["b"], r["km"]))
                    continue
            at = sorted(r["listed"])[0]
            rows[r["carrier"]].append({"at": at, "other": r["b"] if at == r["a"] else r["a"], "summer": s, "winter": w,
                                       "types": None if types == fleets[r["carrier"]] else types,
                                       "src": r["src"], "arts": r["arts"]})
    for h in hand:
        row = {k: h[k] for k in ("at", "other", "number", "summer", "winter") }
        row.update(types=[tuple(t) for t in h["types"]] if "types" in h else None, windows=h.get("windows"),
                   based=h.get("based"), src="hand", arts=set())
        rows[h["carrier"]].append(row)

    # 7. numbers, inside each carrier's range
    for code in sorted(rows):
        lo, hi = (ov.get("carriers") or {}).get(code, {}).get("numbers", [1, 9999])
        used = set()

        def size(r):
            return 2 * max(1, math.ceil(max(r["summer"], r["winter"]) / 7))
        for r in rows[code]:
            if r["src"] == "hand":
                used |= set(range(r["number"], r["number"] + size(r)))
        keep = []
        for r in sorted(rows[code], key=lambda r: (r["src"] != "hand", r["at"], r["other"])):
            if r["src"] != "hand":
                span, n = hi - lo + 1, None
                start = (zlib.crc32(("%s %s %s" % (code, r["at"], r["other"])).encode()) % span) // 2 * 2
                for i in range(0, span, 2):
                    c = lo + (start + i) % span
                    if c + size(r) - 1 <= hi and not used & set(range(c, c + size(r))):
                        n = c
                        break
                if n is None:
                    log["dropped row"].append("%s %s-%s: no free flight number in %d..%d" % (code, r["at"], r["other"], lo, hi))
                    continue
                r["number"] = n
                used |= set(range(n, n + size(r)))
            keep.append(r)
        rows[code] = keep
        both = {(r["at"], r["other"]) for r in keep} & {(r["other"], r["at"]) for r in keep}
        if both:
            sys.exit("%s writes a pair from both ends: %s" % (code, sorted(both)))

    # 8. output
    write_all(a.out, rows, info, ov, fleets, arts, field_of, coords, clocks, labels, hand, log, unresolved)
    check_no_caa(a.out, rows)
    srcs = Counter(r["src"] for rs in rows.values() for r in rs)
    size = sum(os.path.getsize(os.path.join(d, f)) for d, _, fs in os.walk(a.out) for f in fs)
    print(json.dumps({"articles": len(arts), "fields_with_tables": len(set(field_of.values())),
                      "places": len(coords), "carriers": len(rows), "routes": sum(srcs.values()),
                      "sources": dict(sorted(srcs.items())), "unresolved_airlines": len(unresolved),
                      "dropped_rows": len(log["dropped row"]), "time_zones": dict(tz_how),
                      "fit_error": params["fit_error"], "output_bytes": size,
                      "requests": sources.STATS, "seconds": round(time.time() - t0)}, indent=1), file=sys.stderr)


# ---------- output ----------

LICENCE = [
    "Generated by tools/routes/build.py (docs/tasks/global-routes/PLAN.md). Regenerate, do not edit.",
    "Network and place names: English Wikipedia, the articles' {{Airport destination list}} tables,",
    "  licensed CC BY-SA 4.0 (https://creativecommons.org/licenses/by-sa/4.0/). This file is a derivative",
    "  of those articles at the revisions listed below and is shared under the same licence.",
    "Codes, telephony, coordinates and time zones: Wikidata (CC0).",
    "Frequencies (marked per row): bts = US DOT BTS T-100 Segment %s (public domain);",
    "  eurostat = Eurostat avia_par (c) European Union, reused with attribution, the pair's total split",
    "  by the heuristic's weights; heuristic = tools/routes/params.json; hand = tools/routes/overrides.yaml.",
]


def header(revs, extra=()):
    lines = [l % ", ".join("%d-%02d" % m for m in BTS_MONTHS) if "%s" in l else l for l in LICENCE]
    lines += list(extra)
    if revs:
        lines.append("Wikipedia revisions (title: revision id):")
        lines += ["  %s: %d" % (no_dash(t), r) for t, r in sorted(revs)]
    return "".join("# %s\n" % l for l in lines)


def q(s):
    return '"%s"' % str(s).replace("\\", "\\\\").replace('"', '\\"')


def num(x):
    return ("%.4f" % x).rstrip("0").rstrip(".") if x != int(x) else "%.1f" % x


def pairs(ts):
    return "[%s]" % ", ".join("[%s, %s]" % (a, num(b) if isinstance(b, float) else b) for a, b in ts)


def write_all(out, rows, info, ov, fleets, arts, field_of, coords, clocks, labels, hand, log, unresolved):
    os.makedirs(os.path.join(out, "carriers"), exist_ok=True)
    for f in os.listdir(os.path.join(out, "carriers")):
        if f.endswith(".yaml"):
            os.remove(os.path.join(out, "carriers", f))
    fo = ov.get("fields") or {}
    hand_city = {}
    for h in hand:
        hand_city.setdefault(h["other"], h.get("city"))
    title_of = {}
    for t, icao in sorted(field_of.items()):
        title_of.setdefault(icao, t)
    named = sorted({r["at"] for rs in rows.values() for r in rs} | {r["other"] for rs in rows.values() for r in rs})
    lines = []
    for icao in named:
        city = (fo.get(icao, {}).get("city") or hand_city.get(icao)
                or (sorted(labels[icao].items(), key=lambda kv: (-kv[1], kv[0]))[0][0] if labels.get(icao) else None)
                or re.sub(r"\s*(International\s+)?Airport$", "", ascii_text(title_of.get(icao, icao))))
        lat, lon = coords[icao]
        off, dst = clocks[icao]
        extra = "".join(", %s: %s" % (k, "true" if v is True else v) for k, v in sorted(fo.get(icao, {}).items())
                        if k != "city")
        lines.append("- {icao: %s, city: %s, lat: %.4f, lon: %.4f, utc_offset_h: %s, dst: %s%s}\n"
                     % (icao, q(city), lat, lon, num(float(off)), "true" if dst else "false", extra))
    revs = {(t, arts[t][0]) for t in field_of}
    open(os.path.join(out, "fields.yaml"), "w").write(
        header(revs, ["Every field a route names: city, position, standard UTC offset and EU-style summer time."])
        + "".join(lines))
    for code in sorted(rows):
        i, o = info.get(code, {}), (ov.get("carriers") or {}).get(code, {})
        tel = i.get("telephony")
        if not tel:
            tel = i.get("name") or code
            log["telephony from name"].append("%s: %s" % (code, tel))
        tel = tel if o.get("telephony") else " ".join(w.capitalize() for w in ascii_text(tel).split())
        lo, hi = o.get("numbers", [1, 9999])
        body = ["code: %s\n" % code, "name: %s\n" % q(ascii_text(i.get("name") or code))]
        body.append("iata: %s\n" % q(i.get("iata") or ""))
        body += ["telephony: %s\n" % q(tel), "types: %s\n" % pairs(fleets[code]), "numbers: [%d, %d]\n" % (lo, hi),
                 "routes:\n"]
        for r in sorted(rows[code], key=lambda r: (r["at"], r["other"], r["number"])):
            s = "  - {at: %s, other: %s, number: %d, summer: %d, winter: %d" % (
                r["at"], r["other"], r["number"], r["summer"], r["winter"])
            if r.get("types"):
                s += ", types: %s" % pairs(r["types"])
            if r.get("windows"):
                s += ", windows: [%s]" % ", ".join("[%s, %s]" % (num(float(a)), num(float(b))) for a, b in r["windows"])
            if r.get("based"):
                s += ", based: true"
            body.append(s + "}  # %s\n" % r["src"])
        revs = {(t, arts[t][0]) for r in rows[code] for t in r["arts"]}
        extra = ["Carrier: Wikidata %s." % i["q"]] if i.get("q") else []
        open(os.path.join(out, "carriers", code + ".yaml"), "w").write(header(revs, extra) + "".join(body))
    with open(os.path.join(out, "build.log"), "w") as f:
        f.write("# tools/routes/build.py: every dropped row and unresolved airline, sorted.\n")
        for name, ats in sorted(unresolved.items()):
            log["unresolved airline"].append("%s: %d rows (%s)" % (name, len(ats), " ".join(sorted(set(ats)))))
        for sec in sorted(log):
            f.write("\n## %s (%d)\n" % (sec, len(log[sec])))
            f.write("".join("%s\n" % no_dash(l) for l in sorted(set(log[sec]))))


def check_no_caa(out, rows):
    """CAA counts fit the heuristic only: no row may come from them, no file may name them."""
    bad = {r["src"] for rs in rows.values() for r in rs} - set(SOURCES)
    if bad:
        sys.exit("rows with a source other than %s: %s" % (SOURCES, sorted(bad)))
    for d, _, fs in os.walk(out):
        for f in fs:
            if f.endswith(".yaml") and re.search(r"(?i)punctuality|civil aviation authority", open(os.path.join(d, f)).read()):
                sys.exit("%s names the CAA data, which may only fit the heuristic" % f)


if __name__ == "__main__":
    main()
