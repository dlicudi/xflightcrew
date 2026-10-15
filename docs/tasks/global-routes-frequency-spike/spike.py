"""Frequency spike (throwaway): BTS T-100, Eurostat avia_par, UK CAA.

python3 -I spike.py bts <cache_dir> <year> <month>   download one T-100 Segment month
python3 -I spike.py btscov <cache_dir> <network.json> <csv>...
python3 -I spike.py euro <cache_dir> <network.json> <country> <field>
Stdlib only, Python 3.9. Polite: 1 s between calls, cached, descriptive UA.
"""
import csv, gzip, io, json, os, re, sys, time, urllib.parse, urllib.request, zipfile
from collections import defaultdict

UA = "XFlightCrew-feasibility-spike/0.1 (open-source flight sim research)"
BTS = "https://www.transtats.bts.gov/DL_SelectFields.aspx?gnoyr_VQ=FMG&QO_fu146_anzr=Nv4%20Pn44vr45"
FIELDS = ["DEPARTURES_SCHEDULED", "DEPARTURES_PERFORMED", "SEATS", "PASSENGERS",
          "UNIQUE_CARRIER", "CARRIER_NAME", "ORIGIN", "ORIGIN_COUNTRY", "DEST",
          "DEST_COUNTRY", "AIRCRAFT_TYPE", "CLASS", "YEAR", "MONTH"]
# IATA (T-100) to ICAO for the spike's fields only.
US = {"JFK": "KJFK", "SEA": "KSEA", "ASE": "KASE"}


def get(url, data=None, opener=None):
    time.sleep(1)
    req = urllib.request.Request(url, data=data, headers={"User-Agent": UA})
    with (opener or urllib.request.build_opener()).open(req, timeout=300) as r:
        return r.read()


def bts(cache, year, month):
    out = os.path.join(cache, "t100_%s_%02d.csv" % (year, int(month)))
    if os.path.exists(out):
        return out
    op = urllib.request.build_opener(urllib.request.HTTPCookieProcessor())
    h = get(BTS, opener=op).decode("utf-8", "replace")
    form = {n: v for n, v in re.findall(r'name="(__\w+)"[^>]*value="([^"]*)"', h)}
    form.update({"cboGeography": "All", "cboYear": str(year), "cboPeriod": str(month),
                 "btnDownload": "Download"})
    for f in FIELDS:
        form[f] = "on"
    z = get(BTS, urllib.parse.urlencode(form).encode(), op)
    with zipfile.ZipFile(io.BytesIO(z)) as zf:
        name = [n for n in zf.namelist() if n.lower().endswith(".csv")][0]
        open(out, "wb").write(zf.read(name))
    return out


def network(path):
    """{field: {(carrier_icao, other_icao): seasonal}} from the first spike's cache."""
    net = {}
    for field, rows in json.load(open(path)).items():
        d = {}
        for r in rows:
            flags = [f.lower() for f in r[4]]
            if r[5] and "charter" not in flags:
                d[(r[6], r[5])] = d.get((r[6], r[5]), False) or "seasonal" in flags
        net[field] = d
    return net


def btscov(netpath, csvs):
    """Coverage of the Wikipedia network by T-100 (both directions, scheduled pax)."""
    net = network(netpath)
    ap = openflights("AIRPORTS_DAT", 4, 5)
    al = openflights("AIRLINES_DAT", 3, 4, active_col=7)
    for path in csvs:
        rows = [r for r in csv.DictReader(open(path, encoding="utf-8")) if r["CLASS"] == "F"]
        days = 31
        print(path, len(rows), "scheduled passenger rows")
        for iata, icao in US.items():
            pair, car = defaultdict(float), defaultdict(float)
            for r in rows:
                if r["ORIGIN"] == iata:
                    other = ap.get(r["DEST"], "?" + r["DEST"])
                    n = float(r["DEPARTURES_PERFORMED"] or 0)
                    pair[other] += n
                    car[(al.get(r["UNIQUE_CARRIER"], "?" + r["UNIQUE_CARRIER"]), other)] += n
            want = net.get(icao, {})
            others = {o for _, o in want}
            hp = sum(1 for o in others if pair.get(o, 0) > 0)
            hc = sum(1 for k in want if car.get(k, 0) > 0)
            hs = sum(1 for k, s in want.items() if s and car.get(k, 0) > 0)
            print(" %s: T-100 %d dest airports, %d carrier-pairs flown | Wikipedia %d carrier-pairs"
                  " (%d airports, %d seasonal): airport has freq %d/%d, carrier-pair has freq %d/%d"
                  " (seasonal %d/%d)" % (icao, sum(1 for v in pair.values() if v), sum(1 for v in car.values() if v),
                                         len(want), len(others), sum(want.values()), hp, len(others), hc,
                                         len(want), hs, sum(want.values())))
            miss = [k for k in want if car.get(k, 0) <= 0][:8]
            print("   misses e.g.:", miss)
            top = sorted(car.items(), key=lambda kv: -kv[1])[:6]
            print("   top:", ", ".join("%s-%s %.0f/wk" % (c, o, n * 7 / days) for (c, o), n in top))
            fc = {r["UNIQUE_CARRIER"] for r in rows if r["ORIGIN"] == iata and r["DEST_COUNTRY"] != "US"
                  and float(r["DEPARTURES_PERFORMED"] or 0) > 0}
            print("   carriers flying international from %s: %d, e.g. %s" % (iata, len(fc), sorted(fc)[:15]))


def openflights(env, kcol, vcol, active_col=None):
    m = {}
    for r in csv.reader(open(os.environ[env], encoding="utf-8")):
        if len(r) <= vcol or r[kcol] in ("", "\\N", "-") or r[vcol] in ("", "\\N"):
            continue
        if active_col is not None and r[active_col] != "Y":
            continue
        m.setdefault(r[kcol], r[vcol])
    return m


def euro(cache, netpath, country, field):
    """Eurostat avia_par_<cc>: flights and seats, per airport pair, no carrier."""
    ds = "avia_par_" + country
    url = ("https://ec.europa.eu/eurostat/api/dissemination/sdmx/2.1/data/%s?format=SDMX-CSV&compressed=true"
           % ds)
    out = os.path.join(cache, ds + ".csv.gz")
    if not os.path.exists(out):
        open(out, "wb").write(get(url))
    rows = csv.DictReader(io.TextIOWrapper(gzip.open(out), encoding="utf-8"))
    net = network(netpath).get(field, {})
    pref = field[:2]  # e.g. ES_LEMG_...
    flights = defaultdict(dict)  # other -> {period: value}
    units = set()
    for r in rows:
        units.add((r["unit"], r["tra_meas"], r["LAST UPDATE"]))
        rep = r["airp_pr"]
        if not rep.startswith(country.upper() + "_" + field + "_"):
            continue
        if r["unit"] != "FLIGHT" or r["tra_meas"] != "CAF_PAS" or r["freq"] != "M":
            continue
        other = rep.split("_")[3]
        flights[other][r["TIME_PERIOD"]] = r["OBS_VALUE"]
    print(ds, "unit/measure/updated:", sorted(units))
    others = {o for _, o in net}
    latest = max((p for v in flights.values() for p in v if v[p]), default="0000")
    y = latest[:4] if latest[5:] >= "07" else str(int(latest[:4]) - 1)
    per = [y + "-07", y + "-01"]
    hit = sum(1 for o in others if any(flights.get(o, {}).get(p) not in (None, "") for p in per))
    print(" %s: Wikipedia %d carrier-pairs over %d airports; Eurostat pairs %d; airports with flights %d/%d; latest %s"
          % (field, len(net), len(others), len(flights), hit, len(others), latest))
    for o in ["EGKK", "EIDW", "EHAM", "LEBL", "EGCC", "EGSS", "EGGD"]:
        v = flights.get(o, {})
        print("   %s %s %s, %s %s flights/month both ways = %s / %s rotations a week" % (
            o, per[0], v.get(per[0]), per[1], v.get(per[1]), wk(v.get(per[0]), 31), wk(v.get(per[1]), 31)))


def norm(x):
    import unicodedata
    x = unicodedata.normalize("NFKD", x).encode("ascii", "ignore").decode().upper()
    return re.sub(r"[^A-Z ]", " ", x).split()


def caa(netpath, field, name, csvs):
    """UK CAA punctuality Full Analysis: flights per airport, route, airline, month.
    Matched by name (CAA has no codes): destination by the Wikipedia label's
    words, carrier by the first word of the airline's name."""
    net = json.load(open(netpath))[field]
    wiki = [(norm(r[3]), norm(r[0])[:1], r[6], r[5]) for r in net
            if "charter" not in [f.lower() for f in r[4]]]
    for path in csvs:
        rows = [r for r in csv.DictReader(open(path, encoding="utf-8-sig")) if r["reporting_airport"] == name]
        flown = [r for r in rows if int(r["number_flights_matched"]) > 0]
        pairs = {r["origin_destination"] for r in flown}
        cars = {(r["airline_name"], r["origin_destination"]) for r in flown}
        hit_pair = hit_car = 0
        for lab, w0, car, other in wiki:
            dest = [d for d in pairs if set(norm(d)) & set(lab)]
            hit_pair += bool(dest)
            hit_car += any(a.split()[0] == (w0[0] if w0 else "") for a, d in cars if d in dest)
        print("%s %s: %d rows, %d flown destinations, %d airline-route rows (S %d, C %d) | Wikipedia %d carrier-pairs:"
              " destination matched %d, carrier+destination matched %d" % (
                  path[-10:], field, len(rows), len(pairs), len(cars),
                  sum(r["scheduled_charter"] == "S" for r in flown), sum(r["scheduled_charter"] == "C" for r in flown),
                  len(wiki), hit_pair, hit_car))
        for r in flown:
            if r["origin_destination"] in ("MALAGA", "DUBAI", "NEW YORK (JFK)", "ISTANBUL"):
                print("   %s %s %s/month = %s/wk rotations" % (r["origin_destination"], r["airline_name"],
                      r["number_flights_matched"], wk(r["number_flights_matched"], 31)))


def wk(x, days):
    try:
        return "%.0f" % (float(x) / 2 * 7 / days)
    except (TypeError, ValueError):
        return "-"


if __name__ == "__main__":
    cmd = sys.argv[1]
    if cmd == "bts":
        print(bts(sys.argv[2], sys.argv[3], sys.argv[4]))
    elif cmd == "btscov":
        btscov(sys.argv[3], sys.argv[4:])
    elif cmd == "caa":
        caa(sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5:])
    elif cmd == "euro":
        euro(sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])
