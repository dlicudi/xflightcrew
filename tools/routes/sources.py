"""Downloads (cached, polite) and the Wikipedia/Wikidata network.

Every request goes through `get`: one second apart, a descriptive
User-Agent, no account, and cached under <cache>/http by the request's
hash, so a second run reads only the cache and gives the same bytes.
The wikitext parser is the global-routes spike's
(docs/tasks/global-routes-spike/spike.py).
"""
import hashlib, json, os, re, sys, time, urllib.error, urllib.parse, urllib.request

UA = "XFlightCrewRoutes/1.0 (offline route table generator, open source; github.com/dlicudi/xflightcrew)"
WP = "https://en.wikipedia.org/w/api.php?"
WD = "https://www.wikidata.org/w/api.php?"
STATS = {"fetched": 0, "cached": 0}


class Http:
    def __init__(self, cache):
        self.dir = os.path.join(cache, "http")
        os.makedirs(self.dir, exist_ok=True)

    def get(self, url, data=None, opener=None):
        """The response body, or None for a 400 or 404 (cached as such too)."""
        key = hashlib.sha1((url + "\n" + (data or b"").decode()).encode()).hexdigest()
        path = os.path.join(self.dir, key)
        if os.path.exists(path):
            STATS["cached"] += 1
            return open(path, "rb").read()
        if os.path.exists(path + ".404"):
            return None
        time.sleep(1.0)
        STATS["fetched"] += 1
        req = urllib.request.Request(url, data=data, headers={"User-Agent": UA})
        for attempt in range(4):
            try:
                with (opener or urllib.request.build_opener()).open(req, timeout=300) as r:
                    body = r.read()
                break
            except urllib.error.HTTPError as e:
                if e.code in (400, 404):
                    open(path + ".404", "w").close()
                    return None
                if attempt == 3 or e.code not in (429, 500, 502, 503, 504):
                    raise
                time.sleep(30 * (attempt + 1))
            except urllib.error.URLError:
                if attempt == 3:
                    raise
                time.sleep(30 * (attempt + 1))
        open(path + ".tmp", "wb").write(body)
        os.replace(path + ".tmp", path)
        if STATS["fetched"] % 50 == 0:
            print("  %d fetched, %d cached" % (STATS["fetched"], STATS["cached"]), file=sys.stderr)
        return body

    def json(self, base, params):
        return json.loads(self.get(base + urllib.parse.urlencode(params)))


# ---------- Wikipedia ----------

def embedded(http):
    """Every article transcluding {{Airport destination list}}, sorted."""
    q = {"action": "query", "list": "embeddedin", "eititle": "Template:Airport destination list",
         "einamespace": 0, "eilimit": 500, "format": "json", "formatversion": 2}
    out, cont = [], {}
    while True:
        d = http.json(WP, {**q, **cont})
        out += [p["title"] for p in d["query"]["embeddedin"]]
        if "continue" not in d:
            return sorted(set(out))
        cont = d["continue"]


def articles(http, titles):
    """{title: (revid, qid, wikitext)} for the current revisions."""
    out = {}
    for i in range(0, len(titles), 50):
        chunk = titles[i:i + 50]
        q = {"action": "query", "prop": "revisions|pageprops", "rvprop": "ids|content", "rvslots": "main",
             "ppprop": "wikibase_item", "titles": "|".join(chunk), "format": "json", "formatversion": 2}
        cont = {}
        while True:
            d = http.json(WP, {**q, **cont})
            for p in d["query"].get("pages", []):
                revs = p.get("revisions")
                if revs and "content" in revs[0]["slots"]["main"]:
                    out[p["title"]] = (revs[0]["revid"], p.get("pageprops", {}).get("wikibase_item"),
                                       revs[0]["slots"]["main"]["content"])
            if "continue" not in d:
                break
            cont = d["continue"]
    return out


def qids(http, titles):
    """Wikipedia link targets -> Wikidata Q ids (redirects followed)."""
    out, titles = {}, sorted(set(t for t in titles if t and "|" not in t and len(t) < 250))
    for i in range(0, len(titles), 50):
        chunk = titles[i:i + 50]
        d = http.json(WP, {"action": "query", "prop": "pageprops", "ppprop": "wikibase_item", "redirects": 1,
                           "titles": "|".join(chunk), "format": "json", "formatversion": 2})["query"]
        alias = {}
        for k in ("normalized", "redirects"):
            for r in d.get(k, []):
                alias[r["from"]] = r["to"]
        page = {p["title"]: p.get("pageprops", {}).get("wikibase_item") for p in d.get("pages", [])}
        for t in chunk:
            u, n = t, 0
            while u in alias and n < 5:
                u, n = alias[u], n + 1
            out[t] = page.get(u)
    return out


# ---------- Wikidata ----------

PROPS = ("P239", "P238", "P230", "P229", "P432", "P625", "P421", "P131", "P17", "P6687", "P2907")


def _value(v):
    if isinstance(v, str):
        return v
    if "latitude" in v:
        return (v["latitude"], v["longitude"])
    if "amount" in v:
        return float(v["amount"])
    return v.get("id")


def entities(http, ids):
    """{qid: {"label": str, prop: [best-rank values]}} for the PROPS above."""
    out, ids = {}, sorted(set(i for i in ids if i))
    for i in range(0, len(ids), 50):
        d = http.json(WD, {"action": "wbgetentities", "ids": "|".join(ids[i:i + 50]), "props": "claims|labels",
                           "languages": "en", "format": "json"})
        for qid, e in d.get("entities", {}).items():
            rec = {"label": e.get("labels", {}).get("en", {}).get("value", "")}
            for p in PROPS:
                cs = [c for c in e.get("claims", {}).get(p, []) if c.get("rank") != "deprecated"
                      and c["mainsnak"].get("datavalue")]
                best = [c for c in cs if c.get("rank") == "preferred"] or cs
                vals = [_value(c["mainsnak"]["datavalue"]["value"]) for c in best]
                if vals:
                    rec[p] = vals
            out[qid] = rec
    return out


# ---------- wikitext parsing (the first spike's parser) ----------

def clean(t):
    t = re.sub(r"<!--.*?-->", "", t, flags=re.S)
    t = re.sub(r"<ref[^>]*/>", "", t)
    t = re.sub(r"<ref[^>]*>.*?</ref>", "", t, flags=re.S)
    return t


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


TEMPLATE = re.compile(r"\{\{\s*(Airport destination list|Airport-dest-list|Airport dest list)\s*\n?\|", re.I)
HEADING = re.compile(r"^(=+)\s*(.*?)\s*\1\s*$", re.M)
LINK = re.compile(r"\[\[([^\]|#]+)(?:#[^\]|]*)?(?:\|([^\]]*))?\]\]")
HEAD = re.compile(r"'''([^']+?)'''")


def templates(body):
    """(heading above it, body) of every destination-list template."""
    heads = [(m.start(), m.group(2)) for m in HEADING.finditer(body)]
    out = []
    for m in TEMPLATE.finditer(body):
        i, depth = m.start(), 0
        while i < len(body):
            if body.startswith("{{", i): depth += 1; i += 2; continue
            if body.startswith("}}", i):
                depth -= 1; i += 2
                if depth == 0: break
                continue
            i += 1
        head = [h for p, h in heads if p < m.start()]
        out.append((head[-1] if head else "", body[m.end():i - 2]))
    return out


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


def rows_from(text):
    """[(airline_target, airline_label, dest_target, dest_label, flags)], cargo tables left out."""
    rows = []
    for head, t in templates(clean(text)):
        if "cargo" in head.lower():
            continue
        parts = split_top(t)
        named = [p for p in parts if re.match(r"\s*[\w ]+=", p) and not p.strip().startswith("[[")]
        if any("cargo" in p.lower() for p in named if p.strip().lower().startswith("title")):
            continue
        pos = [p for p in parts if p not in named]
        width = 3 if any("3rdcoltitle" in p for p in named) else 2
        while pos and not pos[0].strip():
            pos = pos[1:]
        for i in range(0, len(pos) - 1, width):
            at, al = airline_of(pos[i])
            for dt, dl, f in destinations(pos[i + 1]):
                rows.append((at, al, dt, dl, f))
    return rows


def infobox_icao(text):
    m = re.search(r"\|\s*ICAO\s*=\s*([A-Z]{4})\b", text)
    return m.group(1) if m else None
