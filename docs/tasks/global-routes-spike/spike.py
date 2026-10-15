"""Throwaway spike: Wikipedia "Airlines and destinations" -> route rows.

Python 3 stdlib only. Usage: python3 -I spike.py <cache_dir> <airlines.dat>
Fetches each sample airport's wikitext, parses the passenger table, resolves
destination and airline links to ICAO codes through Wikidata (P239 airport,
P230 airline, P432 callsign), resolves airline names through OpenFlights,
and diffs LXGB/LEMG/EGKK against routes/*.yaml. Prints the numbers.
"""
import csv, json, os, re, sys, time, urllib.parse, urllib.request

UA = "XFlightCrewRouteSpike/0.1 (research spike; https://github.com/dlicudi/xflightcrew)"
REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
SAMPLE = {
    "LXGB": "Gibraltar International Airport", "LEMG": "Málaga Airport",
    "EGKK": "Gatwick Airport", "EGLL": "Heathrow Airport", "EDDF": "Frankfurt Airport",
    "LFPG": "Charles de Gaulle Airport", "EIDW": "Dublin Airport", "LOWI": "Innsbruck Airport",
    "EGPE": "Inverness Airport", "KJFK": "John F. Kennedy International Airport",
    "KSEA": "Seattle–Tacoma International Airport", "KASE": "Aspen/Pitkin County Airport",
    "CYYZ": "Toronto Pearson International Airport",
    "SBGR": "São Paulo/Guarulhos International Airport", "FAOR": "O. R. Tambo International Airport",
    "OMDB": "Dubai International Airport", "VHHH": "Hong Kong International Airport",
    "RJTT": "Haneda Airport", "YSSY": "Sydney Airport", "NZQN": "Queenstown Airport",
}
CACHE = sys.argv[1]


def get(url):
    key = os.path.join(CACHE, re.sub(r"[^A-Za-z0-9]", "_", url)[-180:] + ".json")
    if os.path.exists(key):
        return json.load(open(key))
    time.sleep(1.0)
    req = urllib.request.Request(url, headers={"User-Agent": UA})
    data = json.load(urllib.request.urlopen(req, timeout=60))
    json.dump(data, open(key, "w"))
    return data


def wikitext(title):
    q = urllib.parse.urlencode({"action": "parse", "page": title, "prop": "wikitext",
                                "redirects": 1, "format": "json", "formatversion": 2})
    d = get("https://en.wikipedia.org/w/api.php?" + q)
    return d.get("parse", {}).get("wikitext", "")


def qids(titles):
    """Wikipedia titles -> Wikidata Q ids (redirects followed)."""
    out, titles = {}, sorted(set(titles))
    for i in range(0, len(titles), 50):
        chunk = titles[i:i + 50]
        q = urllib.parse.urlencode({"action": "query", "prop": "pageprops", "ppprop": "wikibase_item",
                                    "redirects": 1, "titles": "|".join(chunk), "format": "json",
                                    "formatversion": 2})
        d = get("https://en.wikipedia.org/w/api.php?" + q)["query"]
        alias = {}
        for k in ("normalized", "redirects"):
            for r in d.get(k, []):
                alias[r["from"]] = r["to"]
        page = {p["title"]: p.get("pageprops", {}).get("wikibase_item") for p in d.get("pages", [])}
        for t in chunk:
            u = t
            while u in alias:
                u = alias[u]
            out[t] = page.get(u)
    return out


def claims(ids, props):
    """Q ids -> {prop: first value} for the given string properties."""
    out, ids = {}, sorted(set(i for i in ids if i))
    for i in range(0, len(ids), 50):
        q = urllib.parse.urlencode({"action": "wbgetentities", "ids": "|".join(ids[i:i + 50]),
                                    "props": "claims", "format": "json"})
        d = get("https://www.wikidata.org/w/api.php?" + q)
        for qid, e in d.get("entities", {}).items():
            vals = {}
            for p in props:
                for c in e.get("claims", {}).get(p, []):
                    v = c["mainsnak"].get("datavalue", {}).get("value")
                    if isinstance(v, str) and c.get("rank") != "deprecated":
                        vals.setdefault(p, v)
            out[qid] = vals
    return out


# ---------- wikitext parsing ----------

def clean(t):
    t = re.sub(r"<!--.*?-->", "", t, flags=re.S)
    t = re.sub(r"<ref[^>]*/>", "", t)
    t = re.sub(r"<ref[^>]*>.*?</ref>", "", t, flags=re.S)
    return t


def section(text):
    """The passenger part of 'Airlines and destinations' (cargo cut off)."""
    m = re.search(r"^(=+)\s*Airlines and destinations\s*\1\s*$", text, re.M | re.I)
    if not m:
        return None, "no section"
    lvl = len(m.group(1))
    rest = text[m.end():]
    end = re.search(r"^={1,%d}[^=].*?={1,%d}\s*$" % (lvl, lvl), rest, re.M)
    body = rest[:end.start()] if end else rest
    cargo = re.search(r"^=+\s*Cargo\s*=+\s*$", body, re.M | re.I)
    return (body[:cargo.start()] if cargo else body), None


def split_top(s, sep="|"):
    """Split on sep at nesting depth 0 ({{ }} and [[ ]])."""
    parts, depth, cur, i = [], 0, [], 0
    while i < len(s):
        two = s[i:i + 2]
        if two in ("{{", "[["):
            depth += 1; cur.append(two); i += 2; continue
        if two in ("}}", "]]"):
            depth -= 1; cur.append(two); i += 2; continue
        if s[i] == sep and depth == 0:
            parts.append("".join(cur)); cur = []; i += 1; continue
        cur.append(s[i]); i += 1
    parts.append("".join(cur))
    return parts


def templates(body, names):
    """Bodies of top-level {{name ...}} templates."""
    out = []
    for m in re.finditer(r"\{\{\s*(%s)\s*\n?\|" % "|".join(names), body, re.I):
        i, depth = m.start(), 0
        while i < len(body):
            if body.startswith("{{", i): depth += 1; i += 2; continue
            if body.startswith("}}", i):
                depth -= 1; i += 2
                if depth == 0: break
                continue
            i += 1
        out.append(body[m.end():i - 2])
    return out


LINK = re.compile(r"\[\[([^\]|#]+)(?:#[^\]|]*)?(?:\|([^\]]*))?\]\]")
HEAD = re.compile(r"'''([^']+?)'''")


def airline_of(cell):
    m = LINK.search(cell)
    if m:
        return m.group(1).strip(), (m.group(2) or m.group(1)).strip()
    t = re.sub(r"\{\{[^{}]*\}\}|<[^>]+>|'''?", "", cell).strip()
    return t, t


def destinations(cell):
    """[(target, label, flags)] from a destinations cell."""
    out, mode = [], set()
    tokens = sorted([(m.start(), "h", m) for m in HEAD.finditer(cell)] +
                    [(m.start(), "l", m) for m in LINK.finditer(cell)], key=lambda x: x[0])
    for n, (pos, kind, m) in enumerate(tokens):
        if kind == "h":
            h = m.group(1).lower()
            mode = set()
            if "seasonal" in h: mode.add("seasonal")
            if "charter" in h: mode.add("charter")
            continue
        nxt = tokens[n + 1][0] if n + 1 < len(tokens) else len(cell)
        tail = cell[m.end():nxt].lower()
        f = set(mode)
        for w, flag in (("begins", "future"), ("resumes", "future"), ("ends", "ending"),
                        ("terminates", "ending"), ("suspended", "suspended")):
            if w in tail: f.add(flag)
        out.append((m.group(1).strip(), (m.group(2) or m.group(1)).strip(), sorted(f)))
    return out


def rows_from(body):
    """(variant, rows, note). rows: (airline_target, airline_label, dest_target, dest_label, flags)."""
    body = clean(body)
    rows = []
    tpls = templates(body, ["Airport destination list", "Airport-dest-list", "Airport dest list"])
    if tpls:
        for t in tpls:
            parts = [p for p in split_top(t)]
            named = [p for p in parts if re.match(r"\s*[\w ]+=", p) and not p.strip().startswith("[[")]
            pos = [p for p in parts if p not in named]
            width = 3 if any("3rdcoltitle" in p for p in named) else 2
            while pos and not pos[0].strip():
                pos = pos[1:]
            for i in range(0, len(pos) - 1, width):
                at, al = airline_of(pos[i])
                for dt, dl, f in destinations(pos[i + 1]):
                    rows.append((at, al, dt, dl, f))
        return "Airport destination list", rows, None
    if "{|" in body:
        for tbl in re.findall(r"\{\|.*?\n\|\}", body, re.S):
            for r in re.split(r"\n\|-[^\n]*", tbl)[1:]:
                cells = [c for c in re.split(r"\n\||\|\|", "\n" + r.strip()) if c.strip()]
                cells = [c.split("|", 1)[1] if re.match(r"\s*(style|align|rowspan|colspan)=", c) and "|" in c else c
                         for c in cells]
                if len(cells) >= 2 and not cells[0].lstrip().startswith("!"):
                    at, al = airline_of(cells[0])
                    for dt, dl, f in destinations(cells[1]):
                        rows.append((at, al, dt, dl, f))
        return "wikitable", rows, None
    return "none", rows, "no table in section"


def main_article(body):
    m = re.search(r"\{\{\s*(?:Main|Further|See also)\s*\|([^}|]+)", body, re.I)
    return m.group(1).strip() if m else None


# ---------- airlines ----------

def norm(s):
    s = s.lower().replace("&", "and")
    s = re.sub(r"\(.*?\)", "", s)
    return re.sub(r"[^a-z0-9]", "", s)


def openflights(path):
    by = {}
    for r in csv.reader(open(path, encoding="utf-8")):
        if len(r) < 8:
            continue
        _, name, alias, iata, icao, callsign, country, active = r[:8]
        icao = "" if icao in ("\\N", "N/A", "") else icao
        if not icao:
            continue
        cand = (active == "Y", icao, callsign if callsign != "\\N" else "")
        for k in {norm(name), norm(alias) if alias != "\\N" else ""} - {""}:
            if k not in by or cand > by[k]:
                by[k] = cand
    return by


def yaml_pairs(icao):
    consts = dict(re.findall(r'const (\w+): Carrier = c\(\s*"(\w+)"',
                             open(os.path.join(REPO, "crates/traffic/src/airlines.rs")).read()))
    pairs = set()
    for line in open(os.path.join(REPO, "routes", icao + ".yaml")):
        m = re.search(r"carrier:\s*(\w+).*?other:\s*(\w{4})", line)
        if m:
            pairs.add((consts.get(m.group(1), "?" + m.group(1)), m.group(2)))
    return pairs


def main():
    of = openflights(sys.argv[2])
    per, all_rows = {}, {}
    for icao, title in SAMPLE.items():
        text = wikitext(title)
        body, why = section(text) if text else (None, "no article")
        note = why
        if body is not None:
            variant, rows, note = rows_from(body)
            if not rows and main_article(body):
                sub = main_article(body)
                variant, rows, note = rows_from(clean(wikitext(sub)))
                variant += " (in '%s')" % sub
        else:
            variant, rows = "none", []
        # a dest equal to the airport itself or a non-article is noise; keep all, resolve later
        all_rows[icao] = rows
        per[icao] = {"variant": variant, "note": note}

    dest_titles = {r[2] for rs in all_rows.values() for r in rs}
    line_titles = {r[0] for rs in all_rows.values() for r in rs}
    dq = qids(dest_titles)
    lq = qids(line_titles)
    dc = claims(dq.values(), ["P239"])
    lc = claims(lq.values(), ["P230", "P432"])
    dest_icao = {t: dc.get(q, {}).get("P239") for t, q in dq.items()}
    line_wd = {t: lc.get(q, {}) for t, q in lq.items()}

    print("airport | variant | rows | dest resolved | unresolved examples")
    unresolved_all, tot_rows, tot_res = {}, 0, 0
    for icao, rows in all_rows.items():
        res = [r for r in rows if dest_icao.get(r[2])]
        un = sorted({r[2] for r in rows if not dest_icao.get(r[2])})
        unresolved_all[icao] = un
        tot_rows += len(rows); tot_res += len(res)
        flags = {}
        for r in rows:
            for f in r[4]:
                flags[f] = flags.get(f, 0) + 1
        print(f"{icao} | {per[icao]['variant']} | {len(rows)} | {len(res)} | "
              f"airlines={len({r[0] for r in rows})} flags={flags} note={per[icao]['note']} | {un[:4]}")
    print(f"TOTAL rows={tot_rows} resolved={tot_res}")

    # airline resolution, per distinct airline
    names = {}
    for rs in all_rows.values():
        for r in rs:
            names[r[0]] = r[1]
    of_hit = {t: of.get(norm(t)) or of.get(norm(l)) for t, l in names.items()}
    of_active = {t: v for t, v in of_hit.items() if v and v[0]}
    wd_hit = {t: line_wd.get(t, {}).get("P230") for t in names}
    either = [t for t in names if of_hit[t] or wd_hit[t]]
    disagree = [(t, of_hit[t][1], wd_hit[t]) for t in names if of_hit[t] and wd_hit[t] and of_hit[t][1] != wd_hit[t]]
    print(f"\nairlines distinct={len(names)} openflights={sum(1 for v in of_hit.values() if v)} "
          f"(active={len(of_active)}, callsign={sum(1 for v in of_hit.values() if v and v[2])}) wikidata_P230={sum(1 for v in wd_hit.values() if v)} "
          f"wikidata_callsign={sum(1 for t in names if line_wd.get(t, {}).get('P432'))} either={len(either)}")
    print("openflights misses:", sorted(t for t in names if not of_hit[t]))
    print("both miss:", sorted(t for t in names if not of_hit[t] and not wd_hit[t]))
    print("openflights vs wikidata disagree:", disagree)

    def code(t):
        return wd_hit.get(t) or (of_hit[t][1] if of_hit.get(t) else None)

    for icao in ("LXGB", "LEMG", "EGKK"):
        ours = yaml_pairs(icao)
        wiki = {(code(r[0]), dest_icao[r[2]]) for r in all_rows[icao]
                if code(r[0]) and dest_icao.get(r[2]) and "charter" not in r[4]}
        both = ours & wiki
        print(f"\n{icao}: ours={len(ours)} wiki(non-charter, resolved)={len(wiki)} match={len(both)}")
        print("  ours only:", sorted(ours - wiki))
        print("  wiki only (count):", len(wiki - ours), sorted(wiki - ours)[:40])

    json.dump({k: [list(r[:4]) + [r[4], dest_icao.get(r[2]), code(r[0])] for r in v]
               for k, v in all_rows.items()},
              open(os.path.join(CACHE, "network.json"), "w"), indent=0)


main()
