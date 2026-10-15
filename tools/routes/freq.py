"""Frequencies: BTS T-100, Eurostat avia_par, UK CAA (fit only), the heuristic.

Downloads follow the frequency spike (docs/tasks/global-routes-frequency-spike).
A weekly figure is ROTATIONS a week: a month's movements both ways, halved,
times 7 over the month's 31 days (July and January).
"""
import csv, gzip, io, math, os, re, unicodedata, urllib.parse, urllib.request, zipfile
from collections import defaultdict

BTS_FORM = "https://www.transtats.bts.gov/DL_SelectFields.aspx?gnoyr_VQ=FMG&QO_fu146_anzr=Nv4%20Pn44vr45"
BTS_TYPES = "https://www.transtats.bts.gov/Download_Lookup.asp?Y11x72=Y_NVePeNSg_glcR"
BTS_FIELDS = ["DEPARTURES_SCHEDULED", "DEPARTURES_PERFORMED", "SEATS", "PASSENGERS", "UNIQUE_CARRIER",
              "CARRIER_NAME", "ORIGIN", "ORIGIN_COUNTRY", "DEST", "DEST_COUNTRY", "AIRCRAFT_TYPE", "CLASS",
              "YEAR", "MONTH"]
EUROSTAT = ("https://ec.europa.eu/eurostat/api/dissemination/sdmx/2.1/data/avia_par_%s/M.FLIGHT.CAF_PAS."
            "?startPeriod=2022-01&format=SDMX-CSV&compressed=true")
# Every country reporting avia_par (the EU, EFTA, the UK and candidates); a missing one is a 404, skipped.
EURO_COUNTRIES = ("at be bg ch cy cz de dk ee el es fi fr hr hu ie is it lt lu lv me mk mt nl no pl pt ro rs "
                  "se si sk tr uk").split()
CAA_PAGE = "https://www.caa.co.uk/data-and-analysis/uk-aviation-market/flight-punctuality/uk-flight-punctuality-statistics/%s/"
WEEK = 7.0 / 31.0 / 2.0  # a 31-day month's movements both ways -> rotations a week


def bts_file(http, cache, year, month):
    """One T-100 Segment month (all carriers), downloaded once by the spike's form POST."""
    out = os.path.join(cache, "bts", "t100_%d_%02d.csv" % (year, month))
    if os.path.exists(out):
        return out
    os.makedirs(os.path.dirname(out), exist_ok=True)
    op = urllib.request.build_opener(urllib.request.HTTPCookieProcessor())
    h = http.get(BTS_FORM, opener=op).decode("utf-8", "replace")
    form = dict(re.findall(r'name="(__\w+)"[^>]*value="([^"]*)"', h))
    form.update({"cboGeography": "All", "cboYear": str(year), "cboPeriod": str(month), "btnDownload": "Download"})
    form.update({f: "on" for f in BTS_FIELDS})
    z = http.get(BTS_FORM, urllib.parse.urlencode(form).encode(), op)
    with zipfile.ZipFile(io.BytesIO(z)) as zf:
        name = [n for n in zf.namelist() if n.lower().endswith(".csv")][0]
        open(out, "wb").write(zf.read(name))
    return out


# BTS aircraft description -> ICAO designator (first match wins; order matters).
BTS_DESIGNATORS = [
    (r"max 800|max 8\b", "B38M"), (r"max 900|max 9\b", "B39M"), (r"737-800", "B738"), (r"737-900", "B739"),
    (r"737-700", "B737"), (r"737-600", "B736"), (r"717", "B712"), (r"757-200", "B752"), (r"757-300", "B753"),
    (r"767-300", "B763"), (r"767-400", "B764"), (r"767-200", "B762"), (r"777-300", "B77W"), (r"777-200", "B772"),
    (r"787-800", "B788"), (r"787-900", "B789"), (r"787-10", "B78X"), (r"747-400", "B744"), (r"747-8", "B748"),
    (r"a321neo|a321-200n", "A21N"), (r"a320-200n|a320neo", "A20N"), (r"a319neo|a319-100n", "A19N"),
    (r"a321", "A321"), (r"a320", "A320"), (r"a319", "A319"), (r"a318", "A318"),
    (r"a220-300|bd-500-1a11", "BCS3"), (r"a2[02]0-100|bd-500-1a10", "BCS1"),
    (r"a330-900|a330-9", "A339"), (r"a330-300|a330-333", "A333"), (r"a330-2", "A332"),
    (r"350-1000", "A35K"), (r"a350", "A359"), (r"a380", "A388"), (r"a340-600", "A346"), (r"a340-300", "A343"),
    (r"175", "E175"), (r"170", "E170"), (r"embraer 190|erj-190|emb-190", "E190"), (r"195", "E195"),
    (r"embraer-14[05]|embraer-135|erj-1[34]", "E145"), (r"crj ?9|rj-900", "CRJ9"),
    (r"crj ?7|rj-700|crj ?705|crj550", "CRJ7"), (r"rj-200|crj-?200|rj-440", "CRJ2"),
    (r"dhc8-400|dash-8-4|q400", "DH8D"), (r"dhc8-300", "DH8C"), (r"dhc8-100", "DH8A"),
    (r"atr-?72", "AT72"), (r"atr-?42", "AT45"), (r"saab 2000", "SB20"), (r"saab.*340", "SF34"),
    (r"328 jet", "J328"), (r"328", "D328"), (r"1900", "B190"), (r"twin otter|dhc-6", "DHC6"),
    (r"pc-12", "PC12"), (r"skycourier", "C408"), (r"208", "C208"), (r"king ?air", "BE20"),
    (r"402", "C402"), (r"pa-31|navajo", "PA31"), (r"islander", "BN2P"), (r"dhc2|beaver", "DHC2"),
    (r"dhc3|otter", "DHC3"), (r"airvan", "GA8"), (r"stationair", "C206"),
]


def bts_designator(desc):
    d = (desc or "").lower()
    for pat, code in BTS_DESIGNATORS:
        if re.search(pat, d):
            return code
    return None


def load_bts(http, cache, months):
    """{(carrier_iata, origin_iata, dest_iata): {"dep": {month: n}, "types": {designator: n}}}, scheduled pax."""
    look = {r["Code"]: r["Description"] for r in
            csv.DictReader(io.StringIO(http.get(BTS_TYPES).decode("utf-8", "replace")))}
    out = defaultdict(lambda: {"dep": defaultdict(float), "types": defaultdict(float)})
    for (y, m) in months:
        for r in csv.DictReader(open(bts_file(http, cache, y, m), encoding="utf-8")):
            n = float(r["DEPARTURES_PERFORMED"] or 0)
            if r["CLASS"] != "F" or n <= 0:
                continue
            e = out[(r["UNIQUE_CARRIER"], r["ORIGIN"], r["DEST"])]
            e["dep"][m] += n
            e["types"][bts_designator(look.get(r["AIRCRAFT_TYPE"])) or "?" + r["AIRCRAFT_TYPE"]] += n
    return out


def load_eurostat(http):
    """({frozenset(icao pair): (summer, winter)}, [periods used]) from each country's latest July and January."""
    pairs, used = {}, []
    for cc in EURO_COUNTRIES:
        raw = http.get(EUROSTAT % cc)
        if not raw:
            continue
        vals = defaultdict(dict)
        for r in csv.DictReader(io.TextIOWrapper(gzip.GzipFile(fileobj=io.BytesIO(raw)), encoding="utf-8")):
            if r["OBS_VALUE"] not in ("", None):
                p = r["airp_pr"].split("_")
                if len(p) == 4:
                    vals[frozenset((p[1], p[3]))][r["TIME_PERIOD"]] = float(r["OBS_VALUE"])
        july = sorted({t for v in vals.values() for t in v if t.endswith("-07")})
        if not july:
            continue
        s, w = july[-1], july[-1][:4] + "-01"
        used.append("%s %s/%s" % (cc.upper(), s, w))
        for k, v in sorted(vals.items(), key=lambda kv: sorted(kv[0])):
            if k not in pairs and len(k) == 2 and (v.get(s) or v.get(w)):
                pairs[k] = (v.get(s, 0.0) * WEEK, v.get(w, 0.0) * WEEK)
    return pairs, used


def words(x):
    x = unicodedata.normalize("NFKD", x).encode("ascii", "ignore").decode().upper()
    return set(re.sub(r"[^A-Z ]", " ", x).split()) - {"AIRPORT", "INTERNATIONAL", "AIRLINES", "AIRWAYS", "AIR", "LTD", "PLC", "UK", "LIMITED", "THE"}


def caa_file(http, cache, yyyymm):
    out = os.path.join(cache, "caa", yyyymm + ".csv")
    if not os.path.exists(out):
        page = http.get(CAA_PAGE % yyyymm[:4]).decode("utf-8", "replace")
        m = re.search(r'href="([^"]+)"(?:(?!href=).){0,600}?%s Punctuality Statistics Full Analysis \(CSV' % yyyymm,
                      page, re.S)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        open(out, "wb").write(http.get("https://www.caa.co.uk" + m.group(1)))
    return out


def caa_samples(http, cache, routes, names, months=("202507", "202601")):
    """UK CAA punctuality counts matched by NAME to network routes: {route key: (summer, winter)}.
    FOR FITTING ONLY: never written into a shipped file (the licence is not cleared)."""
    by_field = defaultdict(list)  # uk field -> [(dest words, carrier words, key)]
    for key, r in routes.items():
        for at, other in ((r["a"], r["b"]), (r["b"], r["a"])):
            if at.startswith("EG"):
                by_field[at].append((names["place"].get(other, set()), names["carrier"].get(r["carrier"], set()), key))
    got = defaultdict(lambda: [0.0, 0.0])
    for season, ym in enumerate(months):
        for r in csv.DictReader(open(caa_file(http, cache, ym), encoding="utf-8-sig")):
            n = float(r["number_flights_matched"] or 0)
            if n <= 0 or r["scheduled_charter"] != "S":
                continue
            rep = words(r["reporting_airport"])
            fields = [f for f in by_field if rep and rep <= names["place"].get(f, set())]
            if len(fields) != 1:
                continue
            dest, line = words(r["origin_destination"]), words(r["airline_name"])
            hits = {k for dw, cw, k in by_field[fields[0]] if dest and dest <= dw and line & cw}
            if len(hits) == 1:
                got[hits.pop()][season] += n * WEEK
    return {k: tuple(v) for k, v in got.items()}


# ---------- the heuristic ----------

FEATURES = ["const", "ln_end_routes", "ln_distance_km", "ln_carrier_routes", "seasonal", "ln_carriers_on_pair"]


def features(r):
    return [1.0, math.log(r["n_a"]) + math.log(r["n_b"]), math.log(max(r["km"], 50.0)),
            math.log(r["n_carrier"]), 1.0 if r["seasonal"] else 0.0, math.log(r["k_pair"])]


def solve(xs, ys, ridge=1e-6):
    """Least squares by the normal equations (Gaussian elimination, a whisker of ridge)."""
    n = len(xs[0])
    a = [[sum(x[i] * x[j] for x in xs) + (ridge if i == j else 0.0) for j in range(n)] for i in range(n)]
    b = [sum(x[i] * y for x, y in zip(xs, ys)) for i in range(n)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(a[r][c]))
        a[c], a[p], b[c], b[p] = a[p], a[c], b[p], b[c]
        for r in range(n):
            if r != c and a[c][c]:
                f = a[r][c] / a[c][c]
                a[r] = [u - f * v for u, v in zip(a[r], a[c])]
                b[r] -= f * b[c]
    return [b[i] / a[i][i] for i in range(n)]


def predict(beta, x):
    return max(0.0, math.exp(sum(b * v for b, v in zip(beta, x))) - 1.0)


def fit(samples):
    """samples: [(features, summer, winter, source)] -> params with the fit's error.
    The target is ln(rotations a week + 1); the error is reported in that unit
    (a factor), as a median in rotations a week, and per source."""
    xs = [s[0] for s in samples]
    params = {"features": FEATURES, "target": "ln(rotations_per_week + 1)"}
    for i, season in ((1, "summer"), (2, "winter")):
        beta = solve(xs, [math.log(s[i] + 1.0) for s in samples])
        params[season] = [round(b, 4) for b in beta]
    err = {}
    for src in sorted({s[3] for s in samples}) + ["all"]:
        sub = [s for s in samples if src in ("all", s[3])]
        res = [math.log(predict(params[season], s[0]) + 1.0) - math.log(s[i] + 1.0)
               for s in sub for i, season in ((1, "summer"), (2, "winter"))]
        absd = sorted(abs(predict(params[season], s[0]) - s[i]) for s in sub for i, season in ((1, "summer"), (2, "winter")))
        rmse = math.sqrt(sum(e * e for e in res) / len(res))
        err[src] = {"samples": len(sub), "rmse_ln": round(rmse, 3), "factor": round(math.exp(rmse), 2),
                    "median_abs_per_week": round(absd[len(absd) // 2], 2),
                    "within_factor_2": round(sum(abs(e) <= math.log(2) for e in res) / len(res), 3)}
    params["fit_error"] = err
    return params
