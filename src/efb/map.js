// The EFB's Map tab: ONE continuous world map on a canvas, north up, in Web
// Mercator. Mercator is what every world map's tiles use, and it is
// conformal: a field's diagram drawn in its own metres stays true, placed
// at any zoom by one transform, so the water, the route, every field, the
// traffic and our aircraft share one set of world coordinates.
//
// Layers by zoom: X-Plane's own water and coast (GET /basemap, only the
// view's tiles), the fields as dots then labels, the plan's route, our
// traffic and our aircraft at every zoom; closer in, the diagram of every
// field in view (GET /airport?icao=, built on demand by the server, kept
// here, ETag'd), then its stands and names. The live picture rides the page
// state (`state.map` in lat/lon, `state.airport` for the wanted field's taxi
// route, card and runway), handed over by page.html's render(). No library,
// no tiles, nothing from the network but this server.
(function () {
  'use strict';

  // ------------------------------------------------------------ projection (pure)

  // The nav crate's sphere (`airport::M_PER_DEG`): a field's metres here are
  // the metres the server measured.
  var M_PER_DEG = 3440.065 * 1852 * Math.PI / 180;
  var TILE = 256;          // px across the world at zoom 0
  var MAX_LAT = 85.05;
  // The widest view /basemap draws (`basemap::MAX_TILES`).
  var MAX_TILES = 1200;
  function mx(lon) { return (lon + 180) / 360; }
  function my(lat) {
    var s = Math.sin(Math.max(-MAX_LAT, Math.min(MAX_LAT, lat)) * Math.PI / 180);
    return 0.5 - Math.log((1 + s) / (1 - s)) / (4 * Math.PI);
  }
  function lonOf(X) { return X * 360 - 180; }
  function latOf(Y) { return Math.atan(Math.sinh(Math.PI * (1 - 2 * Y))) * 180 / Math.PI; }
  // World units per metre at a latitude, east and north alike (conformal).
  function kAt(lat) { return 1 / (360 * M_PER_DEG * Math.cos(lat * Math.PI / 180)); }
  // A field's frame (`airport::Frame`: east and north metres from its
  // reference point) placed on the map: east is exact (X is linear in
  // longitude); north is re-measured on Mercator's stretch once, at load
  // (`v`), so the whole field then places with one scale k: X = X0 + x k,
  // Y = Y0 - v k.
  function fieldFrame(lat0, lon0) {
    var X0 = mx(lon0), Y0 = my(lat0), k = kAt(lat0);
    return {
      X0: X0, Y0: Y0, k: k,
      v: function (y) { return (Y0 - my(lat0 + y / M_PER_DEG)) / k; },
      world: function (x, y) { return [X0 + x * k, my(lat0 + y / M_PER_DEG)]; }
    };
  }
  // The 1x1 degree tiles a box covers, as /basemap counts them.
  function tileCount(lat0, lon0, lat1, lon1) {
    return (Math.floor(lat1) - Math.floor(lat0) + 1) * (Math.floor(lon1) - Math.floor(lon0) + 1);
  }
  // The box /basemap is asked for: the view with a margin (the aircraft's
  // progress then asks rarely), narrower until it is within MAX_TILES;
  // null for a view wider than that. [lat0, lon0, lat1, lon1].
  function baseBox(lat0, lon0, lat1, lon1) {
    var margins = [0.3, 0.1, 0];
    for (var i = 0; i < margins.length; i++) {
      var m = margins[i], dla = (lat1 - lat0) * m, dlo = (lon1 - lon0) * m;
      var b = [Math.max(-MAX_LAT, lat0 - dla), Math.max(-180, lon0 - dlo), Math.min(MAX_LAT, lat1 + dla), Math.min(179.999, lon1 + dlo)];
      if (tileCount(b[0], b[1], b[2], b[3]) <= MAX_TILES) return b;
    }
    return null;
  }

  // Points along the great circle from a to b ([lat, lon]), one about
  // every 20 nm, both ends included: a whole flight's line (the server's
  // `traffic::track::gc_point`, the same slerp).
  function gcPoints(a, b) {
    var R = Math.PI / 180;
    var la = a[0] * R, oa = a[1] * R, lb = b[0] * R, ob = b[1] * R;
    var h = Math.pow(Math.sin((lb - la) / 2), 2) + Math.cos(la) * Math.cos(lb) * Math.pow(Math.sin((ob - oa) / 2), 2);
    var d = 2 * Math.asin(Math.min(1, Math.sqrt(h)));
    var n = Math.max(1, Math.min(120, Math.ceil(d * 3440 / 20)));
    if (d < 1e-9) return [a, b];
    var out = [];
    for (var i = 0; i <= n; i++) {
      var f = i / n, ka = Math.sin((1 - f) * d) / Math.sin(d), kb = Math.sin(f * d) / Math.sin(d);
      var x = ka * Math.cos(la) * Math.cos(oa) + kb * Math.cos(lb) * Math.cos(ob);
      var y = ka * Math.cos(la) * Math.sin(oa) + kb * Math.cos(lb) * Math.sin(ob);
      var z = ka * Math.sin(la) + kb * Math.sin(lb);
      out.push([Math.atan2(z, Math.hypot(x, y)) / R, Math.atan2(y, x) / R]);
    }
    return out;
  }

  var host = typeof window !== 'undefined' ? window : {};
  host.XvlMapMath = { mx: mx, my: my, lonOf: lonOf, latOf: latOf, kAt: kAt, fieldFrame: fieldFrame, tileCount: tileCount, baseBox: baseBox, gcPoints: gcPoints, M_PER_DEG: M_PER_DEG };
  if (typeof document === 'undefined') return;

  // ------------------------------------------------------------ state

  var KEY = 'xvl_efb_t';
  var PREF = 'xvl_efb_map';
  var token = '';
  try {
    token = new URLSearchParams(location.search).get('t') || localStorage.getItem(KEY) || '';
  } catch (e) {}

  var Z_MIN = 5.5, Z_MAX = 20.5;
  // Px per metre where each layer of a field comes in (the old diagram's
  // view scale, so a field reads as it did).
  var GEO_PPM = 0.02, FADE_PPM = [0.025, 0.05], DASH_PPM = 0.08, PAINT_PPM = 0.12, NAMES_PPM = 0.16;
  var STAND_PPM = 0.25, HOLD_PPM = 0.5, GROUND_CS_PPM = 0.5, STAND_NAME_PPM = 1.1;
  var FIELDS_KEPT = 24;

  var root = null, canvas = null, ctx = null;
  var cardBox = null, cardArrow = null, cardText = null, cardSub = null;
  var infoName = null, infoSub = null, statusBox = null, scaleBar = null, scaleText = null;
  var btnFollow = null;
  var W = 0, H = 0, dpr = 1;
  var view = { X: 0.5, Y: my(45), z: 6 };
  var scale = TILE * Math.pow(2, view.z);   // px per world unit
  var placed = false, follow = true, dirty = true;
  var C = {};
  var st = null;   // state.airport: the wanted field's overlay
  var mp = null;   // state.map: everything in lat/lon

  try {
    var saved = JSON.parse(localStorage.getItem(PREF) || '{}');
    if (typeof saved.follow === 'boolean') follow = saved.follow;
    if (isFinite(saved.z) && isFinite(saved.X) && isFinite(saved.Y)) {
      view.X = saved.X; view.Y = saved.Y; setZoom(saved.z); placed = true;
    }
  } catch (e) {}
  function savePrefs() {
    try { localStorage.setItem(PREF, JSON.stringify({ follow: follow, X: view.X, Y: view.Y, z: view.z })); } catch (e) {}
  }

  function setZoom(z) {
    view.z = Math.max(Z_MIN, Math.min(Z_MAX, z));
    scale = TILE * Math.pow(2, view.z);
  }
  function clampView() {
    view.X = Math.max(0, Math.min(1, view.X));
    view.Y = Math.max(my(MAX_LAT), Math.min(my(-MAX_LAT), view.Y));
  }
  function sx(X) { return W / 2 + (X - view.X) * scale; }
  function sy(Y) { return H / 2 + (Y - view.Y) * scale; }
  function worldAt(px, py) { return [view.X + (px - W / 2) / scale, view.Y + (py - H / 2) / scale]; }
  function ppmAt(lat) { return scale * kAt(lat); }
  function onScreen(x, y, m) { return x >= -m && y >= -m && x <= W + m && y <= H + m; }
  function smooth(v, a, b) { var t = Math.max(0, Math.min(1, (v - a) / (b - a))); return t * t * (3 - 2 * t); }

  // Local coordinates placed on the screen: (X0, Y0) plus u q east and
  // v q down (or north, flipped). The numbers handed to the canvas stay
  // small at every zoom, so no precision is lost to a world-sized offset.
  function localTransform(X0, Y0, q, north) {
    var a = q * scale;
    ctx.setTransform(dpr * a, 0, 0, dpr * (north ? -a : a), dpr * sx(X0), dpr * sy(Y0));
    return a;
  }
  function screenTransform() { ctx.setTransform(dpr, 0, 0, dpr, 0, 0); }

  // ------------------------------------------------------------ gliding

  // Positions arrive about once a second; our aircraft and the traffic
  // glide between them. A jump over 400 m (a reposition) is a jump.
  function angDiff(a, b) { return ((b - a + 540) % 360) - 180; }
  function glideNow(g, t) {
    if (!g) return null;
    var k = g.dur > 0 ? Math.min(1, (t - g.t0) / g.dur) : 1;
    return {
      X: g.from.X + (g.to.X - g.from.X) * k,
      Y: g.from.Y + (g.to.Y - g.from.Y) * k,
      h: (g.from.h + angDiff(g.from.h, g.to.h) * k + 360) % 360,
      moving: k < 1
    };
  }
  function glideTo(was, lat, lon, h, t) {
    var to = { X: mx(lon), Y: my(lat), h: h };
    var cur = glideNow(was, t);
    var from = cur ? { X: cur.X, Y: cur.Y, h: cur.h } : to;
    if (Math.hypot(to.X - from.X, to.Y - from.Y) / kAt(lat) > 400) from = to;
    return { from: from, to: to, t0: t, dur: was ? Math.min(1500, Math.max(300, t - was.t0)) : 0 };
  }
  var own = null;   // our aircraft's glide, and `ownD` its last state
  var ownD = null;
  var tr = {};      // id -> glide with `d`, the traffic's state
  // Where each aircraft has been since the page saw it: one point a second
  // at most, the last TRAIL_MAX; drawn for the selected one only.
  var TRAIL_MAX = 300, trails = {};
  function updateTraffic(list, t) {
    var seen = {};
    (list || []).forEach(function (a) {
      seen[a.id] = 1;
      var g = glideTo(tr[a.id], a.lat, a.lon, a.hdg, t);
      g.d = a;
      tr[a.id] = g;
      var tl = trails[a.id] || (trails[a.id] = []);
      var last = tl[tl.length - 1];
      if (!last || last.lat !== a.lat || last.lon !== a.lon) {
        tl.push({ lat: a.lat, lon: a.lon });
        if (tl.length > TRAIL_MAX) tl.shift();
      }
    });
    Object.keys(tr).forEach(function (k) { if (!seen[k]) { delete tr[k]; delete trails[k]; } });
  }
  // A [lat, lon] polyline on the screen (no wrap: the map never crosses
  // the antimeridian at these ranges).
  function strokeLatLon(pts, from) {
    ctx.beginPath();
    if (from) ctx.moveTo(from[0], from[1]);
    pts.forEach(function (p, i) {
      var x = sx(mx(p[1])), y = sy(my(p[0]));
      if (i === 0 && !from) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }
  // The selected aircraft's whole flight (2026-10-13, like FlightAware):
  // the great circle from its origin to where it is solid, its trail
  // behind it (fading), its route ahead and on to its destination dashed,
  // both ends marked; under the symbols.
  function drawSelectedPath(t) {
    if (!sel || sel.kind !== 'ac' || !tr[sel.id]) return;
    var a = tr[sel.id].d, n = glideNow(tr[sel.id], t);
    screenTransform();
    ctx.lineCap = 'round'; ctx.lineJoin = 'round';
    var j = a.journey, here = [latOf(n.Y), lonOf(n.X)];
    if (j) {
      ctx.globalAlpha = 0.7;
      ctx.strokeStyle = C.cyan; ctx.lineWidth = 2;
      strokeLatLon(gcPoints(j.from_pos, here));
      ctx.globalAlpha = 1;
    }
    var tl = trails[sel.id] || [];
    for (var i = 1; i < tl.length; i++) {
      ctx.beginPath();
      ctx.moveTo(sx(mx(tl[i - 1].lon)), sy(my(tl[i - 1].lat)));
      ctx.lineTo(sx(mx(tl[i].lon)), sy(my(tl[i].lat)));
      ctx.globalAlpha = 0.15 + 0.75 * i / tl.length;
      ctx.strokeStyle = C.cyan; ctx.lineWidth = 3; ctx.stroke();
    }
    ctx.globalAlpha = 1;
    ctx.setLineDash([8, 6]);
    ctx.strokeStyle = C.violet; ctx.lineWidth = 2.5;
    var tip = here;
    if (a.ahead && a.ahead.length) {
      // Far out (a modelled arrival) the way to its entry is a great
      // circle; the approach beyond it, and a simulated route, as drawn.
      var lead = gcPoints(here, a.ahead[0]);
      strokeLatLon(lead.concat(a.ahead.slice(1)), [sx(n.X), sy(n.Y)]);
      tip = a.ahead[a.ahead.length - 1];
    }
    if (j && Math.hypot(tip[0] - j.to_pos[0], (tip[1] - j.to_pos[1]) * Math.cos(tip[0] * Math.PI / 180)) > 0.03) {
      strokeLatLon(gcPoints(tip, j.to_pos));
    }
    ctx.setLineDash([]);
    if (j) {
      [[j.from_pos, j.from], [j.to_pos, j.to]].forEach(function (e) {
        var x = sx(mx(e[0][1])), y = sy(my(e[0][0]));
        if (!onScreen(x, y, 20)) return;
        ctx.beginPath(); ctx.arc(x, y, 5, 0, Math.PI * 2);
        ctx.fillStyle = C.cyan; ctx.fill();
        ctx.strokeStyle = '#000'; ctx.lineWidth = 1.5; ctx.stroke();
        tag(x, y - 14, e[1], { size: 11, bold: true, color: C.cyan, bg: 'rgba(7,9,12,0.75)' });
      });
    }
  }

  // ------------------------------------------------------------ the fields' diagrams

  // icao -> { geo, fr, pave, net, lines, rwys, taxiways, holds, stands, R, used }
  var fields = {};
  var wantedFetch = {}, retryAt = {}, inFlight = 0;

  function addFlat(p, a, fr, close) {
    if (!a || a.length < 4) return;
    p.moveTo(a[0], fr.v(a[1]));
    for (var i = 2; i + 1 < a.length; i += 2) p.lineTo(a[i], fr.v(a[i + 1]));
    if (close) p.closePath();
  }
  // A diagram, placed once: every vertex re-measured north (`fieldFrame`),
  // every layer a Path2D the canvas keeps.
  function loadField(g) {
    var fr = fieldFrame(g.ref_lat, g.ref_lon);
    var f = { geo: g, fr: fr, used: 0, version: g.version };
    f.pave = g.pavement.map(function (poly) {
      var p = new Path2D();
      poly.forEach(function (ring) { addFlat(p, ring, fr, true); });
      return p;
    });
    f.net = null;
    if (g.network && g.network.length) {
      f.net = new Path2D();
      g.network.forEach(function (e) { addFlat(f.net, e.pts, fr, false); });
    }
    f.lines = new Path2D();
    g.lines.forEach(function (l) { addFlat(f.lines, l, fr, false); });
    f.rwys = g.runways.map(function (r) {
      var a = { x: r.a.x, v: fr.v(r.a.y) }, b = { x: r.b.x, v: fr.v(r.b.y) };
      var dx = b.x - a.x, dv = b.v - a.v, L = Math.hypot(dx, dv) || 1;
      var nx = -dv / L * r.width_m / 2, nv = dx / L * r.width_m / 2;
      var body = new Path2D();
      body.moveTo(a.x + nx, a.v + nv); body.lineTo(b.x + nx, b.v + nv);
      body.lineTo(b.x - nx, b.v - nv); body.lineTo(a.x - nx, a.v - nv); body.closePath();
      var cl = new Path2D();
      cl.moveTo(a.x, a.v); cl.lineTo(b.x, b.v);
      return { r: r, a: a, b: b, L: L, ux: dx / L, uv: dv / L, body: body, cl: cl };
    });
    f.taxiways = g.taxiways.map(function (t) { return { name: t.name, x: t.x, v: fr.v(t.y) }; });
    f.holds = g.holds.map(function (h) { return { x: h.x, v: fr.v(h.y) }; });
    f.stands = g.stands.map(function (s) { return { s: s, x: s.x, v: fr.v(s.y) }; });
    var b = g.bounds;
    f.R = isFinite(b[0]) ? Math.max(Math.abs(b[0]), Math.abs(b[1]), Math.abs(b[2]), Math.abs(b[3])) : 3000;
    fields[g.icao] = f;
    trimFields();
    if (st && st.icao === g.icao) convertOverlay();
    dirty = true;
  }
  function trimFields() {
    var keys = Object.keys(fields);
    if (keys.length <= FIELDS_KEPT) return;
    keys.filter(function (k) { return !st || k !== st.icao; })
      .sort(function (a, b) { return fields[a].used - fields[b].used; })
      .slice(0, keys.length - FIELDS_KEPT)
      .forEach(function (k) { delete fields[k]; });
  }
  function fetchField(icao) {
    wantedFetch[icao] = true;
    inFlight++;
    fetch(withT('/airport?icao=' + encodeURIComponent(icao)), { headers: tokenHeaders(), credentials: 'same-origin' })
      .then(function (r) {
        if (r.status === 202) { retryAt[icao] = Date.now() + 800; return null; }
        if (r.status === 404) { retryAt[icao] = Date.now() + 120000; return null; }
        if (!r.ok) throw r.status;
        return r.json();
      })
      .then(function (g) { if (g) loadField(g); })
      .catch(function () { retryAt[icao] = Date.now() + 15000; })
      .then(function () { delete wantedFetch[icao]; inFlight--; wantFields(); });
  }
  // The fields to have: the wanted one always (at its state's version),
  // and every field near enough the view to draw, nearest first, two at a
  // time.
  function wantFields() {
    var now = Date.now(), want = [];
    function consider(icao, lat, lon, always) {
      if (!icao || wantedFetch[icao] || (retryAt[icao] || 0) > now) return;
      var f = fields[icao];
      if (always) { if (!f || f.version !== st.version) want.push([icao, -1]); return; }
      if (f || ppmAt(lat) < GEO_PPM) return;
      var x = sx(mx(lon)), y = sy(my(lat));
      if (!onScreen(x, y, Math.max(W, H) * 0.2)) return;
      want.push([icao, Math.hypot(x - W / 2, y - H / 2)]);
    }
    if (st && st.icao && st.status === 'ready') consider(st.icao, 0, 0, true);
    if (base) base.fields.forEach(function (f) { consider(f.icao, f.lat, f.lon, false); });
    if (mp) (mp.fields || []).forEach(function (f) { consider(f.icao, f.lat, f.lon, false); });
    want.sort(function (a, b) { return a[1] - b[1]; });
    for (var i = 0; i < want.length && inFlight < 2; i++) {
      if (!wantedFetch[want[i][0]]) fetchField(want[i][0]);
    }
    if (want.length && inFlight >= 2) return;
    // A 202 asks again shortly.
    var soon = Object.keys(retryAt).filter(function (k) { return retryAt[k] - now < 1000 && retryAt[k] > now; });
    if (soon.length) setTimeout(wantFields, 900);
  }
  // The fields to draw now: loaded, on the screen, with their scale.
  function visibleFields() {
    var out = [];
    Object.keys(fields).forEach(function (k) {
      var f = fields[k], ppm = scale * f.fr.k;
      var x = sx(f.fr.X0), y = sy(f.fr.Y0), r = f.R * ppm;
      if (!onScreen(x, y, r + 40)) return;
      f.ppm = ppm;
      f.alpha = smooth(ppm, FADE_PPM[0], FADE_PPM[1]);
      out.push(f);
    });
    return out;
  }

  // ------------------------------------------------------------ the basemap

  // One /basemap answer, placed once: the water as one Path2D (even-odd),
  // the coast as another (every edge but the tiles' own sides), local to
  // the answer's middle in units of Q per world unit.
  var base = null, baseBusy = false, baseRetry = 0, baseTimer = null;
  var Q = 40075;
  function sameLine(a, b) { var r = Math.round(a); return r === Math.round(b) && Math.abs(a - r) < 2e-4 && Math.abs(b - r) < 2e-4; }
  function loadBase(j, z) {
    var ox = mx((j.bbox[1] + j.bbox[3] + 1) / 2), oy = my((j.bbox[0] + j.bbox[2] + 1) / 2);
    var water = new Path2D(), coast = new Path2D();
    j.water.forEach(function (r) {
      for (var i = 0; i + 1 < r.length; i += 2) {
        var u = (mx(r[i + 1]) - ox) * Q, v = (my(r[i]) - oy) * Q;
        if (i) water.lineTo(u, v); else water.moveTo(u, v);
        if (i && !sameLine(r[i - 2], r[i]) && !sameLine(r[i - 1], r[i + 1])) {
          coast.moveTo((mx(r[i - 1]) - ox) * Q, (my(r[i - 2]) - oy) * Q);
          coast.lineTo(u, v);
        }
      }
      water.closePath();
    });
    base = {
      bbox: j.bbox, z: z, ox: ox, oy: oy, water: water, coast: coast, any: j.water.length > 0,
      fields: j.fields.map(function (f) { return { icao: f[0], lat: f[1], lon: f[2], towered: !!f[3], X: mx(f[2]), Y: my(f[1]) }; })
    };
    dirty = true;
    wantFields();
  }
  function wantBase() {
    if (baseBusy || Date.now() < baseRetry || !W) return;
    var tl = worldAt(0, 0), br = worldAt(W, H);
    var lat0 = latOf(Math.min(1, br[1])), lat1 = latOf(Math.max(0, tl[1]));
    var lon0 = lonOf(Math.max(0, tl[0])), lon1 = lonOf(Math.min(1, br[0]));
    var b = base && base.bbox;
    var held = b && lat0 >= b[0] && lon0 >= b[1] && lat1 <= b[2] + 1 && lon1 <= b[3] + 1 && Math.abs(view.z - base.z) < 0.85;
    if (held) return;
    var box = baseBox(lat0, lon0, lat1, lon1);
    if (!box) { if (base) { base = null; dirty = true; } return; }
    var px = Math.max(50, Math.round((mx(box[3]) - mx(box[1])) * scale));
    var z = view.z;
    baseBusy = true;
    fetch(withT('/basemap?bbox=' + box.map(function (v) { return v.toFixed(3); }).join(',') + '&px=' + px), { headers: tokenHeaders(), credentials: 'same-origin' })
      .then(function (r) { if (!r.ok) throw r.status; return r.json(); })
      .then(function (j) { baseBusy = false; loadBase(j, z); })
      .catch(function () { baseBusy = false; baseRetry = Date.now() + 15000; });
  }
  // Asked once the view has stood still a quarter of a second.
  var lastAsk = [NaN, NaN, NaN, 0, 0];
  function wantData() {
    if (lastAsk[0] === view.X && lastAsk[1] === view.Y && lastAsk[2] === view.z && lastAsk[3] === W && lastAsk[4] === H) return;
    lastAsk = [view.X, view.Y, view.z, W, H];
    if (baseTimer) clearTimeout(baseTimer);
    baseTimer = setTimeout(function () { baseTimer = null; wantBase(); wantFields(); savePrefs(); }, 250);
  }

  // ------------------------------------------------------------ DOM

  function withT(u) { return u + (token ? (u.indexOf('?') < 0 ? '?' : '&') + 't=' + encodeURIComponent(token) : ''); }
  function tokenHeaders() { return token ? { 'X-EFB-Token': token } : {}; }
  function el(tag, cls, text) {
    var e = document.createElement(tag);
    if (cls) e.className = cls;
    if (text !== undefined) e.textContent = text;
    return e;
  }
  var ICON = {
    follow: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="7"/><path d="M12 2v4M12 18v4M2 12h4M18 12h4"/><circle cx="12" cy="12" r="2" fill="currentColor"/></svg>',
    fit: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M4 9V4h5M20 9V4h-5M4 15v5h5M20 15v5h-5"/></svg>',
    plus: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4"><path d="M12 5v14M5 12h14"/></svg>',
    minus: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4"><path d="M5 12h14"/></svg>'
  };
  function ctl(icon, label, onClick) {
    var b = el('button', 'apt-btn');
    b.type = 'button';
    b.setAttribute('aria-label', label);
    b.title = label;
    b.innerHTML = ICON[icon];
    b.addEventListener('click', function (e) { e.stopPropagation(); onClick(); });
    return b;
  }
  function syncButtons() {
    if (btnFollow) btnFollow.setAttribute('aria-pressed', follow ? 'true' : 'false');
  }

  var STYLE = [
    '.apt-root{position:relative;flex:1 1 auto;min-height:0;width:100%;height:100%;overflow:hidden;background:var(--bg);touch-action:none;-webkit-user-select:none;user-select:none}',
    '.apt-root canvas{position:absolute;inset:0;width:100%;height:100%;display:block;touch-action:none;cursor:grab}',
    '.apt-root canvas.drag{cursor:grabbing}',
    '.apt-card{position:absolute;top:12px;left:50%;transform:translateX(-50%);width:min(560px,calc(100% - 24px));display:flex;align-items:center;gap:14px;padding:10px 16px 10px 10px;border-radius:14px;background:rgba(13,17,23,0.94);border:1px solid var(--cyan-line);box-shadow:0 6px 24px rgba(0,0,0,0.5);pointer-events:none}',
    '.apt-card[hidden]{display:none}',
    '.apt-card.x{border-color:var(--red-line)}',
    '.apt-card .arrow{width:54px;height:54px;flex:0 0 auto;border-radius:10px;background:#0B2A3C;display:flex;align-items:center;justify-content:center}',
    '.apt-card.x .arrow{background:var(--red-bg)}',
    '.apt-card .arrow svg{width:42px;height:42px;transition:transform 0.25s linear}',
    '.apt-card .txt{display:flex;flex-direction:column;gap:2px;min-width:0}',
    '.apt-card .main{font-size:22px;font-weight:700;color:var(--text);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}',
    '.apt-card .sub{font-size:14px;color:var(--muted);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}',
    '.apt-ctl{position:absolute;right:12px;bottom:12px;display:flex;flex-direction:column;gap:8px}',
    '.apt-btn{width:48px;height:48px;border-radius:12px;border:1px solid var(--line2);background:rgba(18,26,36,0.94);color:var(--soft);display:flex;align-items:center;justify-content:center;cursor:pointer;padding:0;touch-action:manipulation}',
    '.apt-btn svg{width:24px;height:24px}',
    '.apt-btn[aria-pressed="true"]{border-color:var(--cyan-line);background:var(--cyan-bg);color:var(--cyan)}',
    '.apt-btn:active{filter:brightness(1.3)}',
    '.apt-info{position:absolute;left:12px;bottom:12px;max-width:calc(100% - 96px);display:flex;flex-direction:column;gap:2px;padding:8px 12px;border-radius:10px;background:rgba(13,17,23,0.88);border:1px solid var(--line);pointer-events:none}',
    '.apt-info .n{font-size:16px;font-weight:700;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}',
    '.apt-info .n:empty,.apt-info .s:empty{display:none}',
    '.apt-info .s{font-size:13px;color:var(--muted);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}',
    '.apt-scale{display:flex;align-items:center;gap:8px;font-size:12px;color:var(--muted);font-family:var(--mono)}',
    '.apt-scale i{display:block;height:6px;border:2px solid var(--soft);border-top:0}',
    '.apt-status{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;padding:24px;text-align:center;color:var(--muted);font-size:16px;line-height:1.5;pointer-events:none}',
    '.apt-status[hidden]{display:none}',
    '.apt-pop{position:absolute;left:12px;top:12px;width:min(340px,calc(100% - 24px));max-height:calc(100% - 24px);overflow:auto;display:flex;flex-direction:column;gap:6px;padding:12px 14px;border-radius:12px;background:rgba(13,17,23,0.96);border:1px solid var(--line2);box-shadow:0 6px 24px rgba(0,0,0,0.5);color:var(--text);z-index:2}',
    '.apt-pop[hidden]{display:none}',
    '.apt-pop .hd{display:flex;align-items:flex-start;gap:8px}',
    '.apt-pop .ttl{flex:1 1 auto;min-width:0;font-size:19px;font-weight:700;overflow-wrap:anywhere}',
    '.apt-pop .x{flex:0 0 auto;width:36px;height:36px;margin:-6px -8px 0 0;border:0;background:none;color:var(--muted);font-size:22px;cursor:pointer;touch-action:manipulation}',
    '.apt-pop .sub{font-size:14px;color:var(--muted);overflow-wrap:anywhere}',
    '.apt-pop dl{display:grid;grid-template-columns:auto 1fr;gap:3px 12px;margin:2px 0 0;font-size:14px}',
    '.apt-pop dt{color:var(--muted)}',
    '.apt-pop dd{margin:0;overflow-wrap:anywhere}',
    '.apt-pop dd.prog{display:flex;align-items:center;gap:8px}',
    '.apt-pop dd.prog .bar{flex:1 1 auto;height:6px;border-radius:3px;background:var(--line2);overflow:hidden}',
    '.apt-pop dd.prog .fill{display:block;height:100%;background:var(--cyan)}',
    '.apt-pop .pact{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin-top:4px}',
    '.apt-pop .pact:empty{display:none}',
    '.apt-pop .pact span{flex:1 1 100%;font-size:14px;color:var(--amber)}',
    '.apt-pop .note{font-size:13px;color:var(--soft)}',
    '.apt-pop button.go{min-height:44px;padding:0 16px;border-radius:10px;border:1px solid var(--cyan-line);background:var(--cyan-bg);color:var(--cyan);font-size:15px;font-weight:700;cursor:pointer;touch-action:manipulation}',
    '.apt-pop button.go.warn{border-color:var(--amber);background:var(--amber-bg);color:var(--amber)}',
    '.apt-pop button.go.plain{border-color:var(--line2);background:none;color:var(--soft);font-weight:400}',
    '.apt-pop button.go:disabled{opacity:0.4;cursor:default}',
    '@media (max-width:700px){.apt-card .main{font-size:19px}.apt-card .arrow{width:46px;height:46px}.apt-card .arrow svg{width:36px;height:36px}}',
    '@media (max-height:500px){.apt-card{top:8px;padding:6px 12px 6px 6px;width:min(520px,calc(100% - 76px));left:8px;transform:none}.apt-card .main{font-size:18px}.apt-card .arrow{width:40px;height:40px}.apt-card .arrow svg{width:32px;height:32px}.apt-ctl{bottom:8px;right:8px;gap:6px}.apt-ctl .zoom{display:none}.apt-btn{width:44px;height:44px}.apt-info{bottom:8px;left:8px;padding:6px 10px}.apt-info .s{display:none}}'
  ].join('\n');

  function readColours() {
    // The map keeps the night palette in a day theme: read where it is drawn.
    var cs = getComputedStyle(root || document.documentElement);
    function v(n, d) { var x = cs.getPropertyValue(n).trim(); return x || d; }
    C = {
      bg: v('--bg', '#07090C'),
      text: v('--text', '#E6EDF3'),
      soft: v('--soft', '#B7C2CF'),
      muted: v('--muted', '#8B98A8'),
      cyan: v('--cyan', '#7FD3F7'),
      magenta: v('--magenta', '#EB6EEB'),
      green: v('--green', '#7EE2A4'),
      amber: v('--amber', '#FFC266'),
      red: v('--red', '#FF8A80'),
      violet: v('--violet', '#B9A6FF'),
      water: v('--water', '#0F1B26'),
      coast: v('--coast', '#2A4256'),
      sans: v('--sans', 'B612, system-ui, sans-serif'),
      mono: v('--mono', 'ui-monospace, monospace'),
      pave: '#1A222C',
      rwy: '#39424E',
      rwyEdge: '#5A6573',
      paint: 'rgba(214, 178, 60, 0.55)',
      route: '#E77BFF',
      routeCase: '#2A0B33',
      driven: 'rgba(160, 110, 180, 0.45)',
      hold: '#FF6B6B',
      stand: '#6F7E90',
      signBg: '#000000',
      signFg: '#FFD54A'
    };
  }

  var popBox = null, popTitle = null, popSub = null, popRows = null, popAct = null;
  function mount() {
    var hostEl = document.getElementById('apt-root');
    if (!hostEl || root) return;
    root = hostEl;
    var style = el('style');
    style.textContent = STYLE;
    document.head.appendChild(style);
    readColours();
    canvas = el('canvas');
    canvas.setAttribute('aria-label', 'Map');
    ctx = canvas.getContext('2d');
    root.appendChild(canvas);

    statusBox = el('div', 'apt-status', 'Waiting for the copilot');
    root.appendChild(statusBox);

    cardBox = el('div', 'apt-card');
    cardBox.hidden = true;
    cardArrow = el('div', 'arrow');
    var txt = el('div', 'txt');
    cardText = el('div', 'main', '');
    cardSub = el('div', 'sub', '');
    txt.appendChild(cardText);
    txt.appendChild(cardSub);
    cardBox.appendChild(cardArrow);
    cardBox.appendChild(txt);
    root.appendChild(cardBox);

    var info = el('div', 'apt-info');
    infoName = el('div', 'n', '');
    infoSub = el('div', 's', '');
    scaleBar = el('div', 'apt-scale');
    scaleText = el('span', null, '');
    scaleBar.appendChild(el('i'));
    scaleBar.appendChild(scaleText);
    info.appendChild(infoName);
    info.appendChild(infoSub);
    info.appendChild(scaleBar);
    root.appendChild(info);

    popBox = el('div', 'apt-pop');
    popBox.hidden = true;
    popBox.setAttribute('role', 'dialog');
    var hd = el('div', 'hd');
    popTitle = el('div', 'ttl', '');
    var shut = el('button', 'x', '×');
    shut.type = 'button';
    shut.setAttribute('aria-label', 'Close');
    shut.addEventListener('click', function () { select(null); });
    hd.appendChild(popTitle);
    hd.appendChild(shut);
    popSub = el('div', 'sub', '');
    popRows = el('dl');
    popAct = el('div', 'pact');
    popBox.appendChild(hd);
    popBox.appendChild(popSub);
    popBox.appendChild(popRows);
    popBox.appendChild(popAct);
    root.appendChild(popBox);

    var ctlBox = el('div', 'apt-ctl');
    var zin = ctl('plus', 'Zoom in', function () { zoomTo(view.z + 1, W / 2, H / 2); });
    var zout = ctl('minus', 'Zoom out', function () { zoomTo(view.z - 1, W / 2, H / 2); });
    zin.classList.add('zoom');
    zout.classList.add('zoom');
    ctlBox.appendChild(zin);
    ctlBox.appendChild(zout);
    ctlBox.appendChild(ctl('fit', 'Overview: the route ahead', overview));
    btnFollow = ctl('follow', 'Follow the aircraft', function () {
      setFollow(!follow);
      dirty = true;
    });
    ctlBox.appendChild(btnFollow);
    root.appendChild(ctlBox);
    syncButtons();

    // The page around the map: no padding, no scroll, while it shows.
    var page = document.getElementById('p-apt');
    var main = document.getElementById('main');
    function onShown() {
      var on = !!(page && page.classList.contains('on'));
      if (main) main.classList.toggle('apt-on', on);
      if (on) { resize(); dirty = true; }
    }
    if (page && window.MutationObserver) {
      new MutationObserver(onShown).observe(page, { attributes: true, attributeFilter: ['class'] });
    }
    onShown();
    if (window.ResizeObserver) new ResizeObserver(function () { resize(); }).observe(root);
    window.addEventListener('resize', resize);
    wireGestures();
    requestAnimationFrame(frame);
  }

  function resize() {
    if (!root) return;
    var w = root.clientWidth, h = root.clientHeight;
    var r = window.devicePixelRatio || 1;
    if (w === W && h === H && r === dpr) return;
    W = w; H = h; dpr = r;
    canvas.width = Math.max(1, Math.round(w * dpr));
    canvas.height = Math.max(1, Math.round(h * dpr));
    placeFirst();
    dirty = true;
  }

  // ------------------------------------------------------------ the view

  function setFollow(on) {
    if (on === follow) return;
    follow = on;
    if (on) { fling = null; anim = null; }
    savePrefs(); syncButtons();
  }
  // Frame a lat/lon box (the state's `map.view`).
  function fitBox(v) {
    var X0 = mx(v.lon0), X1 = mx(v.lon1), Y0 = my(v.lat1), Y1 = my(v.lat0);
    view.X = (X0 + X1) / 2; view.Y = (Y0 + Y1) / 2;
    setZoom(Math.log2(Math.min(W * 0.85 / Math.max(1e-9, X1 - X0), (H - 120) * 0.85 / Math.max(1e-9, Y1 - Y0)) / TILE));
    dirty = true;
  }
  function overview() {
    if (mp && mp.view) { setFollow(false); anim = null; fitBox(mp.view); }
  }
  // The first view, once there is a screen and a state: on the ground the
  // field round us, airborne the overview, centred on us while following;
  // with neither, western Europe. Nothing is asked of the server before.
  function placeFirst() {
    if (placed || !W || !H || !mp) return;
    var o = mp.own;
    if (o && o.on_ground) {
      view.X = mx(o.lon); view.Y = my(o.lat);
      setZoom(Math.log2(Math.min(W, H) / 2500 / kAt(o.lat) / TILE));
    } else if (mp.view) {
      fitBox(mp.view);
    } else if (o) {
      view.X = mx(o.lon); view.Y = my(o.lat); setZoom(9);
    }
    placed = true;
    dirty = true;
  }

  // ------------------------------------------------------------ gestures

  var pointers = {};
  var gesture = null;
  var lastTap = { t: 0, x: 0, y: 0 };
  var fling = null;   // { vx, vy } px per ms
  var anim = null;    // { z0, z1, px, py, A, t0 }: a smooth zoom about a point
  function local(e) {
    var r = canvas.getBoundingClientRect();
    return [e.clientX - r.left, e.clientY - r.top];
  }
  // Zoom by `k` about a screen point, the world under it staying put (about
  // the middle while following: the aircraft stays centred).
  function zoomAt(px, py, k) {
    if (follow) { px = W / 2; py = H / 2; }
    var a = worldAt(px, py);
    setZoom(view.z + Math.log2(k));
    view.X = a[0] - (px - W / 2) / scale;
    view.Y = a[1] - (py - H / 2) / scale;
    clampView();
    dirty = true;
  }
  function zoomTo(z, px, py) {
    if (follow) { px = W / 2; py = H / 2; }
    anim = { z0: view.z, z1: Math.max(Z_MIN, Math.min(Z_MAX, z)), px: px, py: py, A: worldAt(px, py), t0: performance.now() };
    fling = null;
    dirty = true;
  }
  function panBy(dx, dy) {
    view.X -= dx / scale;
    view.Y -= dy / scale;
    clampView();
    dirty = true;
  }
  function step(t, dt) {
    var busy = false;
    if (anim) {
      var k = Math.min(1, (t - anim.t0) / 250), e = 1 - Math.pow(1 - k, 3);
      setZoom(anim.z0 + (anim.z1 - anim.z0) * e);
      view.X = anim.A[0] - (anim.px - W / 2) / scale;
      view.Y = anim.A[1] - (anim.py - H / 2) / scale;
      clampView();
      if (k >= 1) anim = null;
      busy = true;
    }
    if (fling && dt > 0) {
      panBy(fling.vx * dt, fling.vy * dt);
      var d = Math.exp(-dt / 325);
      fling.vx *= d; fling.vy *= d;
      if (Math.hypot(fling.vx, fling.vy) < 0.02) fling = null;
      busy = true;
    }
    return busy;
  }
  function ids() { return Object.keys(pointers); }
  function wireGestures() {
    canvas.addEventListener('pointerdown', function (e) {
      if (canvas.setPointerCapture) canvas.setPointerCapture(e.pointerId);
      var p = local(e);
      pointers[e.pointerId] = p;
      canvas.classList.add('drag');
      fling = null; anim = null;
      gesture = { moved: 0, start: p, t: Date.now(), last: [p[0], p[1], e.timeStamp], prev: null };
      if (ids().length === 2) {
        var q = ids().map(function (k) { return pointers[k]; });
        gesture.pinch = { d: Math.hypot(q[0][0] - q[1][0], q[0][1] - q[1][1]), m: [(q[0][0] + q[1][0]) / 2, (q[0][1] + q[1][1]) / 2] };
      }
    });
    canvas.addEventListener('pointermove', function (e) {
      if (!(e.pointerId in pointers)) return;
      var prev = pointers[e.pointerId];
      var now = local(e);
      pointers[e.pointerId] = now;
      var k = ids();
      if (k.length >= 2 && gesture && gesture.pinch) {
        var p = k.slice(0, 2).map(function (i) { return pointers[i]; });
        var d = Math.hypot(p[0][0] - p[1][0], p[0][1] - p[1][1]);
        var m = [(p[0][0] + p[1][0]) / 2, (p[0][1] + p[1][1]) / 2];
        if (gesture.pinch.d > 0 && d > 0) zoomAt(m[0], m[1], d / gesture.pinch.d);
        if (!follow) panBy(m[0] - gesture.pinch.m[0], m[1] - gesture.pinch.m[1]);
        gesture.pinch = { d: d, m: m };
        gesture.moved += 10;
      } else if (k.length === 1 && gesture) {
        var dx = now[0] - prev[0], dy = now[1] - prev[1];
        gesture.moved += Math.abs(dx) + Math.abs(dy);
        gesture.prev = gesture.last;
        gesture.last = [now[0], now[1], e.timeStamp];
        // A pan is the pilot looking elsewhere: following stops.
        if (gesture.moved > 6) { setFollow(false); panBy(dx, dy); }
      }
    });
    function up(e) {
      if (!(e.pointerId in pointers)) return;
      var p = local(e);
      delete pointers[e.pointerId];
      if (!ids().length) canvas.classList.remove('drag');
      if (gesture && gesture.pinch && ids().length === 1) {
        gesture = { moved: 99, start: p, t: Date.now(), last: [p[0], p[1], e.timeStamp], prev: null };
        return;
      }
      if (gesture && !ids().length) {
        if (gesture.moved <= 6 && Date.now() - gesture.t < 350) {
          var t = Date.now();
          if (t - lastTap.t < 320 && Math.hypot(p[0] - lastTap.x, p[1] - lastTap.y) < 30) {
            zoomTo(view.z + 1, p[0], p[1]);
            lastTap.t = 0;
          } else {
            lastTap = { t: t, x: p[0], y: p[1] };
            select(hitAt(p[0], p[1]));
          }
        } else if (!follow && gesture.prev && !gesture.pinch) {
          // A flick carries on and slows.
          var a = gesture.prev, b = gesture.last, dt = b[2] - a[2];
          if (dt > 0 && e.timeStamp - b[2] < 80) {
            var vx = (b[0] - a[0]) / dt, vy = (b[1] - a[1]) / dt;
            if (Math.hypot(vx, vy) > 0.25) fling = { vx: vx, vy: vy };
          }
        }
        gesture = null;
      }
    }
    canvas.addEventListener('pointerup', up);
    canvas.addEventListener('pointercancel', up);
    canvas.addEventListener('dblclick', function (e) { e.preventDefault(); });
    canvas.addEventListener('wheel', function (e) {
      e.preventDefault();
      var p = local(e);
      anim = null;
      zoomAt(p[0], p[1], Math.exp(-e.deltaY * (e.deltaMode === 1 ? 0.05 : 0.0016)));
    }, { passive: false });
    // Safari's own pinch events would zoom the page.
    ['gesturestart', 'gesturechange'].forEach(function (n) {
      canvas.addEventListener(n, function (e) { e.preventDefault(); });
    });
  }

  // What a tap at screen (sx, sy) lands on: our traffic first (the nearest
  // within 22 px), then a stand of a field drawn close enough for its
  // stands (16 px), else nothing (the card closes).
  function hitAt(px, py) {
    var t = performance.now(), best = null, bd = 22;
    Object.keys(tr).forEach(function (k) {
      var n = glideNow(tr[k], t);
      var d = Math.hypot(sx(n.X) - px, sy(n.Y) - py);
      if (d < bd) { bd = d; best = { kind: 'ac', id: tr[k].d.id }; }
    });
    if (best) return best;
    bd = 16;
    visibleFields().forEach(function (f) {
      if (f.ppm <= STAND_PPM) return;
      f.stands.forEach(function (s) {
        var d = Math.hypot(sx(f.fr.X0 + s.x * f.fr.k) - px, sy(f.fr.Y0 - s.v * f.fr.k) - py);
        if (d < bd) { bd = d; best = { kind: 'stand', icao: f.geo.icao, name: s.s.name }; }
      });
    });
    return best;
  }
  var sel = null;          // { kind: 'ac', id } or { kind: 'stand', icao, name }
  var confirming = false;  // the stand card asks before it moves us
  var popNote = '', popSig = '';
  function select(s) {
    var same = sel && s && sel.kind === s.kind && sel.id === s.id && sel.name === s.name && sel.icao === s.icao;
    if (!same) { confirming = false; popNote = ''; }
    sel = s;
    renderPop();
    dirty = true;
  }

  // ------------------------------------------------------------ labels

  // Labels never overlap: each claims its box on the screen, the map's own
  // boxes (the card, the info, the buttons, a tap card) claimed first.
  var claimed = [];
  function free(b) {
    if (b[2] < 0 || b[3] < 0 || b[0] > W || b[1] > H) return false;
    for (var i = 0; i < claimed.length; i++) {
      var c = claimed[i];
      if (b[0] < c[2] && b[2] > c[0] && b[1] < c[3] && b[3] > c[1]) return false;
    }
    return true;
  }
  var widths = {}, nWidths = 0;
  function textW(font, text) {
    var key = font + '|' + text, w = widths[key];
    if (w === undefined) {
      if (nWidths > 3000) { widths = {}; nWidths = 0; }
      ctx.font = font;
      w = widths[key] = ctx.measureText(text).width;
      nWidths++;
    }
    return w;
  }
  // A tag centred on (x, y), the diagram's style (a background).
  function tag(x, y, text, o) {
    var font = (o.bold ? '700 ' : '400 ') + o.size + 'px ' + C.sans;
    var w = textW(font, text) + (o.bg ? 10 : 2), h = o.size + (o.bg ? 6 : 2);
    var b = [x - w / 2 - 2, y - h / 2 - 2, x + w / 2 + 2, y + h / 2 + 2];
    if (!free(b) && !o.force) return false;
    claimed.push(b);
    ctx.font = font;
    if (o.bg) {
      ctx.fillStyle = o.bg;
      ctx.fillRect(x - w / 2, y - h / 2, w, h);
      if (o.border) { ctx.strokeStyle = o.border; ctx.lineWidth = 1; ctx.strokeRect(x - w / 2 + 0.5, y - h / 2 + 0.5, w - 1, h - 1); }
    }
    ctx.fillStyle = o.color;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, x, y + 0.5);
    return true;
  }
  // A label beside a point, the route's style: the first free of eight
  // spots round it, on a halo; dropped with none free unless it must stay.
  function note(x, y, text, color, font, must) {
    var w = textW(font, text), h = parseInt(font.replace(/^bold /, ''), 10) + 2;
    var spots = [[x + 9, y - 8 - h], [x + 9, y + 6], [x - 9 - w, y - 8 - h], [x - 9 - w, y + 6], [x + 11, y - h / 2], [x - 11 - w, y - h / 2], [x - w / 2, y - 12 - h], [x - w / 2, y + 11]];
    var b = null;
    for (var i = 0; i < spots.length && !b; i++) {
      var c = [spots[i][0] - 2, spots[i][1] - 2, spots[i][0] + w + 2, spots[i][1] + h + 2];
      if (c[0] >= 2 && c[1] >= 2 && c[2] <= W - 2 && c[3] <= H - 2 && free(c)) b = c;
    }
    if (!b) { if (!must) return; b = [spots[0][0] - 2, spots[0][1] - 2, spots[0][0] + w + 2, spots[0][1] + h + 2]; }
    claimed.push(b);
    ctx.font = font;
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    ctx.lineWidth = 3; ctx.strokeStyle = C.bg; ctx.globalAlpha = 0.85;
    ctx.strokeText(text, b[0] + 2, b[1] + 3);
    ctx.globalAlpha = 1; ctx.fillStyle = color;
    ctx.fillText(text, b[0] + 2, b[1] + 3);
  }
  function overlayRects() {
    var out = [], b = root.getBoundingClientRect();
    root.querySelectorAll('.apt-card, .apt-info, .apt-ctl, .apt-pop').forEach(function (e) {
      if (e.hidden || e.offsetParent === null) return;
      var r = e.getBoundingClientRect();
      if (r.width) out.push([r.left - b.left, r.top - b.top, r.right - b.left, r.bottom - b.top]);
    });
    return out;
  }

  // ------------------------------------------------------------ drawing

  function drawBase() {
    if (!base || !base.any) return;
    var a = localTransform(base.ox, base.oy, 1 / Q, false);
    ctx.fillStyle = C.water;
    ctx.fill(base.water, 'evenodd');
    ctx.strokeStyle = C.coast;
    ctx.lineWidth = 1 / a;
    ctx.stroke(base.coast);
  }

  function sameEnd(a, b) {
    var n = function (s) { return String(s || '').trim().toUpperCase().replace(/^RW/, '').replace(/^0+(?=\d)/, ''); };
    return n(a) === n(b);
  }
  // The wanted field's marked runway end (cleared, arrival or planned).
  function markedRunway(f) {
    if (!st || !st.runway || st.icao !== f.geo.icao) return null;
    for (var i = 0; i < f.rwys.length; i++) {
      var r = f.rwys[i].r;
      if (sameEnd(r.a.id, st.runway.ident)) return { w: f.rwys[i], end: 'a' };
      if (sameEnd(r.b.id, st.runway.ident)) return { w: f.rwys[i], end: 'b' };
    }
    return null;
  }

  function drawField(f) {
    var a = localTransform(f.fr.X0, f.fr.Y0, f.fr.k, true);
    var px = function (n) { return n / a; };
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    if (f.alpha > 0) {
      ctx.globalAlpha = f.alpha;
      ctx.fillStyle = C.pave;
      f.pave.forEach(function (p) { ctx.fill(p, 'evenodd'); });
      if (f.net) {
        ctx.strokeStyle = C.pave;
        ctx.lineWidth = Math.max(f.geo.network_width_m || 20, px(3));
        ctx.stroke(f.net);
      }
      ctx.globalAlpha = 1;
    }
    var mark = markedRunway(f);
    f.rwys.forEach(function (w) {
      if (w.L * a < 8) return;
      var mine = mark && mark.w === w;
      ctx.fillStyle = mine ? '#1F4A3A' : (w.r.hard ? C.rwy : '#2D3B2A');
      ctx.fill(w.body);
      ctx.strokeStyle = mine ? C.green : C.rwyEdge;
      ctx.lineWidth = px(mine ? 2.5 : 1);
      ctx.stroke(w.body);
      if (f.ppm > DASH_PPM) {
        ctx.setLineDash([30, 20]);
        ctx.strokeStyle = 'rgba(230, 237, 243, 0.55)';
        ctx.lineWidth = px(1);
        ctx.stroke(w.cl);
        ctx.setLineDash([]);
      }
    });
    if (f.ppm > PAINT_PPM) {
      ctx.strokeStyle = C.paint;
      ctx.lineWidth = px(1);
      ctx.stroke(f.lines);
    }
  }

  // The wanted field's taxi route and its marks, converted into its field's
  // placed frame when the state or the field changes (`convertOverlay`).
  var ov = null;
  function convertOverlay() {
    ov = null;
    var f = st && st.route && fields[st.icao];
    if (!f) return;
    var r = st.route, fr = f.fr;
    function conv(a) { var o = []; for (var i = 0; i + 1 < (a || []).length; i += 2) o.push(a[i], fr.v(a[i + 1])); return o; }
    function path(a) { var p = new Path2D(); if (a.length >= 4) { p.moveTo(a[0], a[1]); for (var i = 2; i < a.length; i += 2) p.lineTo(a[i], a[i + 1]); } return p; }
    var driven = conv(r.driven), remaining = conv(r.remaining);
    var mark = function (m) { return m ? { x: m.x, v: fr.v(m.y), label: m.label, kind: m.kind } : null; };
    ov = {
      f: f, all: driven.concat(remaining),
      driven: driven.length >= 4 ? path(driven) : null,
      remaining: remaining.length >= 4 ? path(remaining) : null,
      holds: (r.holds || []).map(mark), dest: mark(r.dest)
    };
  }
  function routeDirAt(pts, x, v) {
    var best = 1e18, dir = 0;
    for (var i = 0; i + 3 < pts.length; i += 2) {
      var ax = pts[i], ay = pts[i + 1], bx = pts[i + 2], by = pts[i + 3];
      var dx = bx - ax, dy = by - ay, L2 = dx * dx + dy * dy;
      var t = L2 > 0 ? Math.max(0, Math.min(1, ((x - ax) * dx + (v - ay) * dy) / L2)) : 0;
      var d = Math.hypot(x - (ax + t * dx), v - (ay + t * dy));
      if (d < best && L2 > 0) { best = d; dir = Math.atan2(dx, dy); }
    }
    return dir;
  }
  function fieldScreen(f, x, v) { return [sx(f.fr.X0 + x * f.fr.k), sy(f.fr.Y0 - v * f.fr.k)]; }
  function holdBar(m, colour) {
    // A bar across the route, the way a holding line is painted.
    var s = fieldScreen(ov.f, m.x, m.v);
    screenTransform();
    ctx.save();
    ctx.translate(s[0], s[1]);
    ctx.rotate(routeDirAt(ov.all, m.x, m.v));
    ctx.fillStyle = '#000';
    ctx.fillRect(-13, -4, 26, 8);
    ctx.fillStyle = colour;
    ctx.fillRect(-11, -2.5, 22, 2);
    ctx.fillRect(-11, 1, 22, 2);
    ctx.restore();
  }
  // The taxi route rides its field's diagram: drawn once it is whole.
  function routeShown() { return ov && fields[ov.f.geo.icao] === ov.f && scale * ov.f.fr.k >= FADE_PPM[1]; }
  function drawTaxiRoute() {
    if (!routeShown()) return;
    var a = localTransform(ov.f.fr.X0, ov.f.fr.Y0, ov.f.fr.k, true);
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    if (ov.driven) {
      ctx.strokeStyle = C.driven;
      ctx.lineWidth = 4 / a;
      ctx.stroke(ov.driven);
    }
    if (ov.remaining) {
      ctx.strokeStyle = C.routeCase;
      ctx.lineWidth = 9 / a;
      ctx.stroke(ov.remaining);
      ctx.strokeStyle = C.route;
      ctx.lineWidth = 5 / a;
      ctx.stroke(ov.remaining);
    }
    ov.holds.forEach(function (m) { holdBar(m, C.hold); });
    if (ov.dest && ov.dest.kind === 'hold') holdBar(ov.dest, C.amber);
    if (ov.dest && ov.dest.kind === 'stand') {
      var s = fieldScreen(ov.f, ov.dest.x, ov.dest.v);
      screenTransform();
      ctx.fillStyle = C.route;
      ctx.beginPath(); ctx.arc(s[0], s[1], 7, 0, Math.PI * 2); ctx.fill();
      ctx.strokeStyle = '#000'; ctx.lineWidth = 2; ctx.stroke();
    }
  }
  function taxiRouteLabels() {
    if (!routeShown()) return;
    if (ov.dest) {
      var s = fieldScreen(ov.f, ov.dest.x, ov.dest.v);
      tag(s[0], s[1] - 20, ov.dest.label || (ov.dest.kind === 'stand' ? 'Stand' : 'Hold'), { size: 13, bold: true, color: '#1A0620', bg: C.route, force: true });
    }
    ov.holds.forEach(function (m) {
      var s = fieldScreen(ov.f, m.x, m.v);
      tag(s[0], s[1] - 18, 'Hold short ' + m.label, { size: 12, bold: true, color: '#FFFFFF', bg: '#8A1F1F' });
    });
  }

  // The plan, placed when the state changes: [X, Y] per fix.
  var plan = null;
  function convertPlan() {
    plan = null;
    if (!mp) return;
    var fx = (mp.fixes || []).map(function (f) { return { f: f, X: mx(f.lon), Y: my(f.lat) }; });
    plan = {
      fixes: fx,
      fields: (mp.fields || []).map(function (f) { return { f: f, X: mx(f.lon), Y: my(f.lat) }; }),
      tod: mp.tod ? { X: mx(mp.tod.lon), Y: my(mp.tod.lat), label: mp.tod.label } : null
    };
  }
  // The plan's legs fade out as a field fills the screen (its taxi route
  // and the diagram take over).
  function planAlpha() { return 1 - smooth(ppmAt(latOf(view.Y)), 0.06, 0.15); }
  function drawPlan(alpha) {
    if (!plan || alpha <= 0) return;
    screenTransform();
    ctx.lineCap = 'round'; ctx.lineJoin = 'round';
    var fx = plan.fixes;
    for (var i = 0; i + 1 < fx.length; i++) {
      var flown = fx[i + 1].f.done;
      ctx.strokeStyle = flown ? C.muted : C.magenta;
      ctx.globalAlpha = alpha * (flown ? 0.45 : 1);
      ctx.lineWidth = flown ? 2 : 3;
      ctx.beginPath(); ctx.moveTo(sx(fx[i].X), sy(fx[i].Y)); ctx.lineTo(sx(fx[i + 1].X), sy(fx[i + 1].Y)); ctx.stroke();
    }
    var planned = {};
    plan.fields.forEach(function (p) { planned[p.f.icao] = 1; });
    fx.forEach(function (p) {
      var x = sx(p.X), y = sy(p.Y);
      if (planned[p.f.id] || !onScreen(x, y, 20)) return;
      ctx.fillStyle = p.f.done ? C.muted : (p.f.active ? C.magenta : C.text);
      ctx.globalAlpha = alpha * (p.f.done ? 0.6 : 1);
      ctx.beginPath(); ctx.arc(x, y, p.f.active ? 5 : 3, 0, Math.PI * 2); ctx.fill();
      if (p.f.active) { ctx.strokeStyle = C.magenta; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(x, y, 10, 0, Math.PI * 2); ctx.stroke(); }
    });
    if (plan.tod) {
      ctx.globalAlpha = alpha;
      ctx.strokeStyle = C.amber; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(sx(plan.tod.X), sy(plan.tod.Y), 6, 0, Math.PI * 2); ctx.stroke();
    }
    ctx.globalAlpha = 1;
  }
  function activeFix() {
    if (!plan) return null;
    for (var i = 0; i < plan.fixes.length; i++) if (plan.fixes[i].f.active) return plan.fixes[i].f;
    return null;
  }
  function planFieldColour(p, active) {
    var dest = p.f.role === 'destination', isActive = active && active.id === p.f.icao;
    return isActive || dest ? C.magenta : (p.f.role === 'alternate' ? C.amber : C.soft);
  }
  // A field the map does not draw as a diagram (yet): a dot, or the plan's
  // ring. `shown` holds the diagrams drawn, by ICAO, with their alpha.
  function drawFieldMarks(shown) {
    screenTransform();
    var active = activeFix();
    if (base) {
      base.fields.forEach(function (f) {
        var g = 1 - (shown[f.icao] || 0);
        if (g <= 0.05 || (!f.towered && view.z < 8.5)) return;
        var x = sx(f.X), y = sy(f.Y);
        if (!onScreen(x, y, 4)) return;
        ctx.fillStyle = C.muted; ctx.globalAlpha = g * (f.towered ? 0.9 : 0.6);
        ctx.beginPath(); ctx.arc(x, y, f.towered ? 2.5 : 1.8, 0, Math.PI * 2); ctx.fill();
      });
    }
    if (plan) {
      plan.fields.forEach(function (p) {
        var g = 1 - (shown[p.f.icao] || 0);
        var x = sx(p.X), y = sy(p.Y);
        if (g <= 0.05 || !onScreen(x, y, 12)) return;
        var col = planFieldColour(p, active);
        ctx.globalAlpha = g;
        ctx.strokeStyle = col; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(x, y, 6, 0, Math.PI * 2); ctx.stroke();
        ctx.fillStyle = col; ctx.beginPath(); ctx.arc(x, y, 2, 0, Math.PI * 2); ctx.fill();
        if (active && active.id === p.f.icao) { ctx.beginPath(); ctx.arc(x, y, 11, 0, Math.PI * 2); ctx.stroke(); }
      });
    }
    ctx.globalAlpha = 1;
  }

  // A stand's state from the page state: 'ours', 'taken', 'free', or
  // 'unknown' while no traffic world simulates the field (never free).
  function standsOf(icao) {
    var all = (mp && mp.stands) || [];
    for (var i = 0; i < all.length; i++) if (all[i].icao === icao) return all[i];
    return null;
  }
  function standState(icao, name) {
    var s = standsOf(icao);
    var hit = s && (s.taken || []).filter(function (x) { return x.name === name; })[0];
    if (hit) return { state: hit.by === 'us' ? 'ours' : 'taken', by: hit.by, how: hit.how };
    return { state: s && s.known ? 'free' : 'unknown' };
  }
  function drawStands(f) {
    if (f.ppm <= STAND_PPM) return;
    screenTransform();
    var col = { free: C.green, taken: C.red, ours: '#FFFFFF', unknown: C.stand };
    var r = f.ppm > STAND_NAME_PPM ? 5 : 3.5;
    f.stands.forEach(function (s) {
      var p = fieldScreen(f, s.x, s.v);
      if (!onScreen(p[0], p[1], 10)) return;
      ctx.beginPath(); ctx.arc(p[0], p[1], r, 0, Math.PI * 2);
      ctx.fillStyle = col[standState(f.geo.icao, s.s.name).state];
      ctx.fill();
      ctx.strokeStyle = '#000'; ctx.lineWidth = 1; ctx.stroke();
      if (sel && sel.kind === 'stand' && sel.icao === f.geo.icao && sel.name === s.s.name) {
        ctx.beginPath(); ctx.arc(p[0], p[1], r + 6, 0, Math.PI * 2);
        ctx.strokeStyle = C.cyan; ctx.lineWidth = 2.5; ctx.stroke();
      }
    });
  }
  function fieldLabels(f, mark) {
    // Runway designators, just beyond each threshold.
    var a = f.ppm, off = Math.max(35, 22 / a);
    f.rwys.forEach(function (w) {
      if (w.L * a < 40) return;
      [[w.a, -1, 'a', w.r.a.id], [w.b, 1, 'b', w.r.b.id]].forEach(function (e) {
        var p = fieldScreen(f, e[0].x + e[1] * w.ux * off, e[0].v + e[1] * w.uv * off);
        var mine = mark && mark.w === w && mark.end === e[2];
        tag(p[0], p[1], e[3], { size: 14, bold: true, color: mine ? '#06140C' : C.text, bg: mine ? C.green : 'rgba(7,9,12,0.8)', force: mine || (st && st.icao === f.geo.icao) });
      });
    });
    // Taxiway names, like the signs: yellow on black.
    if (a > NAMES_PPM) {
      f.taxiways.forEach(function (t) {
        var p = fieldScreen(f, t.x, t.v);
        if (onScreen(p[0], p[1], 0)) tag(p[0], p[1], t.name, { size: 12, bold: true, color: C.signFg, bg: C.signBg, border: '#5A4A10' });
      });
    }
    if (a > HOLD_PPM) {
      ctx.fillStyle = C.hold;
      f.holds.forEach(function (h) {
        var p = fieldScreen(f, h.x, h.v);
        if (onScreen(p[0], p[1], 10)) ctx.fillRect(p[0] - 3, p[1] - 3, 6, 6);
      });
    }
    if (a > STAND_NAME_PPM) {
      f.stands.forEach(function (s) {
        var p = fieldScreen(f, s.x, s.v);
        if (s.s.name && onScreen(p[0], p[1], 10)) tag(p[0], p[1] + 12, s.s.name, { size: 11, color: C.muted });
      });
    }
  }

  // Our traffic: an airliner's plan form, violet, smaller than our own white
  // chevron, nose along its heading; smaller still far out.
  function drawTraffic(t) {
    screenTransform();
    var z = view.z < 9 ? 0.7 : 1;
    Object.keys(tr).forEach(function (k) {
      var a = tr[k].d, n = glideNow(tr[k], t);
      var x = sx(n.X), y = sy(n.Y);
      if (!onScreen(x, y, 30)) return;
      var zz = ppmAt(a.lat) > STAND_NAME_PPM ? 1.25 : z;
      // The whole flights far out (the model's): small, more so zoomed out.
      if (a.model) zz = view.z < 8 ? 0.5 : 0.75;
      ctx.save();
      ctx.translate(x, y);
      ctx.rotate(n.h * Math.PI / 180);
      ctx.scale(zz, zz);
      ctx.beginPath();
      ctx.moveTo(0, -11); ctx.lineTo(1.6, -7); ctx.lineTo(1.6, -2); ctx.lineTo(10, 3); ctx.lineTo(10, 5);
      ctx.lineTo(1.6, 2.5); ctx.lineTo(1.3, 7); ctx.lineTo(4, 9); ctx.lineTo(4, 10.5); ctx.lineTo(0, 9.5);
      ctx.lineTo(-4, 10.5); ctx.lineTo(-4, 9); ctx.lineTo(-1.3, 7); ctx.lineTo(-1.6, 2.5); ctx.lineTo(-10, 5);
      ctx.lineTo(-10, 3); ctx.lineTo(-1.6, -2); ctx.lineTo(-1.6, -7);
      ctx.closePath();
      ctx.lineJoin = 'round';
      ctx.strokeStyle = '#000'; ctx.lineWidth = 2.5; ctx.stroke();
      ctx.fillStyle = a.on_ground ? '#9C8CD8' : C.violet;
      if (a.model) ctx.globalAlpha = 0.8;
      ctx.fill();
      ctx.globalAlpha = 1;
      ctx.restore();
      if (sel && sel.kind === 'ac' && sel.id === a.id) {
        ctx.beginPath(); ctx.arc(x, y, 16 * zz, 0, Math.PI * 2);
        ctx.strokeStyle = C.cyan; ctx.lineWidth = 2.5; ctx.stroke();
      }
    });
  }
  function drawOwn(o) {
    if (!o) return;
    var x = sx(o.X), y = sy(o.Y);
    screenTransform();
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(o.h * Math.PI / 180);
    ctx.beginPath();
    ctx.moveTo(0, -17); ctx.lineTo(11, 11); ctx.lineTo(0, 8); ctx.lineTo(-11, 11);
    ctx.closePath();
    ctx.fillStyle = '#FFFFFF';
    ctx.strokeStyle = '#000000';
    ctx.lineWidth = 3;
    ctx.lineJoin = 'round';
    ctx.stroke();
    ctx.fill();
    ctx.restore();
    claimed.push([x - 16, y - 18, x + 16, y + 16]);
  }
  // Labels, by priority: the taxi route's marks and the wanted field's
  // runways never dropped; the active fix, the destination and the top of
  // descent always; the plan's fields; the fixes ahead (nearest first);
  // taxiway and stand names; the towered fields; the traffic's callsigns;
  // the flown fixes; every other field.
  function drawLabels(t, o, vis, alpha) {
    screenTransform();
    taxiRouteLabels();
    vis.forEach(function (f) { if (f.alpha > 0.3) fieldLabels(f, markedRunway(f)); });
    var shown = {};
    vis.forEach(function (f) { shown[f.geo.icao] = f.alpha; });
    var mono = C.mono, active = activeFix();
    if (plan && alpha > 0.3) {
      plan.fixes.forEach(function (p) {
        if (p.f.active && !planned(p.f.id)) note(sx(p.X), sy(p.Y), p.f.id, C.magenta, 'bold 12px ' + mono, true);
      });
    }
    if (plan) {
      plan.fields.forEach(function (p) {
        if ((shown[p.f.icao] || 0) > 0.5) return;
        var dest = p.f.role === 'destination', isActive = active && active.id === p.f.icao;
        var x = sx(p.X), y = sy(p.Y);
        if (onScreen(x, y, 0)) note(x, y, p.f.icao, planFieldColour(p, active), 'bold 12px ' + mono, dest || isActive);
      });
      if (plan.tod && alpha > 0.3 && onScreen(sx(plan.tod.X), sy(plan.tod.Y), 0)) note(sx(plan.tod.X), sy(plan.tod.Y), plan.tod.label || 'TOD', C.amber, 'bold 12px ' + mono, true);
    }
    if (plan && alpha > 0.3) {
      var here = o ? [sx(o.X), sy(o.Y)] : [W / 2, H / 2];
      plan.fixes.filter(function (p) { return !p.f.done && !p.f.active && !planned(p.f.id) && onScreen(sx(p.X), sy(p.Y), 0); })
        .sort(function (a, b) { return Math.hypot(sx(a.X) - here[0], sy(a.Y) - here[1]) - Math.hypot(sx(b.X) - here[0], sy(b.Y) - here[1]); })
        .forEach(function (p) { note(sx(p.X), sy(p.Y), p.f.id, C.text, '12px ' + mono, false); });
    }
    if (base && view.z >= 7) {
      base.fields.forEach(function (f) {
        if (!f.towered || (shown[f.icao] || 0) > 0.5 || planned(f.icao)) return;
        var x = sx(f.X), y = sy(f.Y);
        if (onScreen(x, y, 0)) note(x, y, f.icao, C.muted, '10px ' + mono, false);
      });
    }
    Object.keys(tr).forEach(function (k) {
      var a = tr[k].d, n = glideNow(tr[k], t);
      if (a.on_ground && ppmAt(a.lat) <= GROUND_CS_PPM) return;
      // Far out, the model's flights are named only once zoomed in a little
      // (or selected): a region's sky of callsigns is noise.
      if (a.model && view.z < 8 && !(sel && sel.id === a.id)) return;
      var x = sx(n.X), y = sy(n.Y);
      if (onScreen(x, y, 0)) tag(x, y + 16, a.callsign, { size: 11, bold: true, color: C.violet, bg: 'rgba(7,9,12,0.75)' });
    });
    if (plan && alpha > 0.3) {
      plan.fixes.forEach(function (p) {
        if (p.f.done && !planned(p.f.id) && onScreen(sx(p.X), sy(p.Y), 0)) note(sx(p.X), sy(p.Y), p.f.id, C.muted, '12px ' + mono, false);
      });
    }
    if (base && view.z >= 10) {
      base.fields.forEach(function (f) {
        if (f.towered || (shown[f.icao] || 0) > 0.5 || planned(f.icao)) return;
        var x = sx(f.X), y = sy(f.Y);
        if (onScreen(x, y, 0)) note(x, y, f.icao, C.muted, '10px ' + mono, false);
      });
    }
  }
  function planned(icao) {
    if (!plan) return false;
    for (var i = 0; i < plan.fields.length; i++) if (plan.fields[i].f.icao === icao) return true;
    return false;
  }

  function niceScale() {
    // A bar about 90 px long: metres to a kilometre and a half, nautical
    // miles beyond, 1, 2 or 5 times a power of ten.
    var m = 90 / ppmAt(latOf(view.Y));
    var nm = m >= 1500, u = nm ? m / 1852 : m;
    var p = Math.pow(10, Math.floor(Math.log10(u)));
    var n = u / p >= 5 ? 5 : u / p >= 2 ? 2 : 1;
    var len = n * p;
    scaleBar.firstChild.style.width = Math.round(len * (nm ? 1852 : 1) * ppmAt(latOf(view.Y))) + 'px';
    scaleText.textContent = nm ? len + ' nm' : (len >= 1000 ? (len / 1000) + ' km' : len + ' m');
  }

  var lastT = 0, lastDraw = 0;
  function frame(t) {
    requestAnimationFrame(frame);
    var dt = lastT ? Math.min(64, t - lastT) : 0;
    lastT = t;
    if (!root || !W || !H || !root.offsetParent) return;
    var busy = step(t, dt) || ids().length > 0;
    var o = glideNow(own, t);
    var moving = !!(o && o.moving);
    Object.keys(tr).forEach(function (k) { if (glideNow(tr[k], t).moving) moving = true; });
    if (follow && o) { view.X = o.X; view.Y = o.Y; }
    // A glide alone redraws at 20 a second; a finger or a zoom every frame.
    if (!dirty && !busy && !(moving && t - lastDraw > 50)) return;
    dirty = false;
    lastDraw = t;
    draw(t, o);
    if (placed) wantData();
  }
  function draw(t, o) {
    screenTransform();
    ctx.fillStyle = C.bg;
    ctx.fillRect(0, 0, W, H);
    claimed = overlayRects();
    drawBase();
    var vis = visibleFields(), shown = {};
    vis.forEach(function (f) { f.used = t; shown[f.geo.icao] = f.alpha; drawField(f); });
    var alpha = planAlpha();
    drawPlan(alpha);
    drawTaxiRoute();
    drawFieldMarks(shown);
    vis.forEach(drawStands);
    drawSelectedPath(t);
    drawTraffic(t);
    drawOwn(o);
    drawLabels(t, o, vis, alpha);
    niceScale();
    updateCardArrow(o);
  }

  // ------------------------------------------------------------ the turn card

  function arrowSvg(kind) {
    var fill = kind === 'X' ? C.red : kind === 'C' ? C.amber : kind === 'H' ? C.amber : C.cyan;
    if (kind === 'H') {
      return '<svg viewBox="0 0 48 48"><rect x="8" y="16" width="32" height="5" fill="' + fill + '"/><rect x="8" y="26" width="32" height="5" fill="' + fill + '"/></svg>';
    }
    if (kind === 'X') {
      return '<svg viewBox="0 0 48 48"><path d="M12 12 L36 36 M36 12 L12 36" stroke="' + fill + '" stroke-width="6" stroke-linecap="round"/></svg>';
    }
    return '<svg viewBox="0 0 48 48"><path d="M24 4 L38 22 L29 22 L29 44 L19 44 L19 22 L10 22 Z" fill="' + fill + '"/></svg>';
  }
  var cardKind = '';
  function updateCardArrow(o) {
    var c = st && st.card;
    if (!c || c.kind === 'H' || c.kind === 'X') return;
    var hdg = o ? o.h : (ownD ? ownD.heading : 0);
    var svg = cardArrow.firstChild;
    if (svg) svg.style.transform = 'rotate(' + angDiff(hdg, c.aim_true_deg).toFixed(1) + 'deg)';
  }
  function distWords(m) {
    if (m < 1000) return Math.round(m / 10) * 10 + ' m';
    return (m / 1000).toFixed(1) + ' km';
  }
  function renderCard() {
    var c = st && st.card;
    if (!c) { cardBox.hidden = true; return; }
    cardBox.hidden = false;
    cardBox.classList.toggle('x', c.kind === 'X');
    if (cardKind !== c.kind) { cardArrow.innerHTML = arrowSvg(c.kind); cardKind = c.kind; }
    cardText.textContent = c.text;
    var side = Math.abs(c.xtk_m) < 1 ? 'on the line' : Math.abs(c.xtk_m).toFixed(0) + ' m ' + (c.xtk_m > 0 ? 'right' : 'left') + ' of the line';
    var bits = [];
    if (c.kind !== 'X') bits.push(side);
    bits.push(distWords(c.remaining_m) + ' to go');
    if (st.route && st.route.via) bits.push('via ' + st.route.via);
    cardSub.textContent = bits.join(', ');
  }

  // ------------------------------------------------------------ the tap card

  var KIND = { gate: 'Gate', tie_down: 'Tie down', hangar: 'Hangar', misc: 'Other' };
  function clearEl(e) { while (e.firstChild) e.removeChild(e.firstChild); }
  function row(dt, dd) {
    if (dd === null || dd === undefined || dd === '') return;
    popRows.appendChild(el('dt', null, dt));
    popRows.appendChild(el('dd', null, String(dd)));
  }
  // "1 h 05 min", "35 min".
  function span(min) {
    min = Math.max(0, Math.round(min));
    var h = Math.floor(min / 60), m = min % 60;
    return h ? h + ' h ' + (m < 10 ? '0' : '') + m + ' min' : m + ' min';
  }
  // The share flown as FlightAware draws it: a bar and its percentage.
  function progressRow(p) {
    var pct = Math.round(Math.max(0, Math.min(1, p)) * 100);
    popRows.appendChild(el('dt', null, 'Progress'));
    var dd = el('dd', 'prog');
    var bar = el('span', 'bar');
    var fill = el('span', 'fill');
    fill.style.width = pct + '%';
    bar.appendChild(fill);
    dd.appendChild(bar);
    dd.appendChild(el('span', null, pct + '%'));
    popRows.appendChild(dd);
  }
  function button(text, cls, onClick, disabled) {
    var b = el('button', 'go' + (cls ? ' ' + cls : ''), text);
    b.type = 'button';
    b.disabled = !!disabled;
    b.addEventListener('click', function (e) { e.stopPropagation(); onClick(); });
    popAct.appendChild(b);
    return b;
  }
  function standWords(name) { return /^\d/.test(name) ? 'Stand ' + name : name; }
  function renderPop() {
    if (!popBox) return;
    var a = sel && sel.kind === 'ac' && tr[sel.id] ? tr[sel.id].d : null;
    var f = sel && sel.kind === 'stand' ? fields[sel.icao] : null;
    var s = f ? f.geo.stands.filter(function (x) { return x.name === sel.name; })[0] || null : null;
    if (!a && !s) { popBox.hidden = true; popSig = ''; if (sel) sel = null; return; }
    var mv = st && st.reposition;
    // Rebuilt only when what it says changed: a button rebuilt under a
    // finger would lose its tap.
    var sig = JSON.stringify(a ? [a.callsign, a.phase, a.stand, a.alt_ft, a.gs_kts, a.est, a.journey]
      : [sel.icao, s.name, standState(sel.icao, s.name), confirming, popNote, mv]);
    if (sig === popSig && !popBox.hidden) return;
    popSig = sig;
    popBox.hidden = false;
    clearEl(popRows);
    clearEl(popAct);
    if (a) {
      popTitle.textContent = a.callsign;
      popSub.textContent = [a.airline, a.spoken && a.spoken !== a.callsign ? '"' + a.spoken + '"' : ''].filter(Boolean).join(', ');
      var j = a.journey;
      row('Type', a.icao_type);
      row('Route', (a.from || '?') + ' to ' + (a.to || '?'));
      row('Doing', a.phase);
      if (j) {
        row('Off blocks', j.off + (j.since_min >= 0 ? ' (' + span(j.since_min) + ' ago)' : ' (in ' + span(-j.since_min) + ')'));
        row(j.to_go_min >= 0 ? 'Arrives' : 'Arrived', j.on + (j.to_go_min >= 0 ? ' (in ' + span(j.to_go_min) + ')' : ''));
        progressRow(j.progress);
      }
      row('Stand', a.stand);
      row('Slot', a.sched ? a.sched + (a.est && a.est !== a.sched ? ', expected ' + a.est : '') : null);
      row('Altitude', a.alt_ft !== null && a.alt_ft !== undefined ? Math.round(a.alt_ft).toLocaleString('en-GB') + ' ft' : null);
      row('Speed', a.gs_kts > 1 ? a.gs_kts + ' kt' : null);
      if (a.model) popAct.appendChild(el('div', 'note', 'Beyond our simulated airspace: placed by its timetable.'));
      return;
    }
    var ss = standState(sel.icao, s.name);
    popTitle.textContent = standWords(s.name);
    popSub.textContent = f.geo.icao + (f.geo.name ? ' ' + f.geo.name : '');
    row('Kind', KIND[s.kind] || s.kind);
    row('Takes', s.accepts && s.accepts.length ? s.accepts.join(', ') : 'any type');
    row('Size', s.size ? 'up to code ' + s.size : null);
    row('Status', ss.state === 'free' ? 'Free'
      : ss.state === 'ours' ? 'Our aircraft is on it'
      : ss.state === 'taken' ? 'Taken: ' + ss.by + ', ' + ss.how
      : 'Not known (no traffic of ours here)');
    if (mv && mv.stand === s.name) popAct.appendChild(el('div', 'note', mv.text));
    else if (popNote) popAct.appendChild(el('div', 'note', popNote));
    var taken = ss.state === 'taken' || ss.state === 'ours';
    if (confirming && !taken) {
      popAct.appendChild(el('span', null, 'Move to ' + standWords(s.name) + (f.geo.icao !== (st && st.icao) ? ' at ' + f.geo.icao : '') + '? The flight reloads.'));
      button('Move', 'warn', function () { moveTo(f.geo.icao, s.name); });
      button('Cancel', 'plain', function () { confirming = false; renderPop(); });
    } else {
      button('Move here', '', function () { confirming = true; popNote = ''; renderPop(); }, taken);
    }
  }
  // "Move here" is sim management: its own endpoint, never a spoken line.
  function moveTo(icao, name) {
    confirming = false;
    popNote = 'Asking...';
    renderPop();
    fetch(withT('/reposition'), {
      method: 'POST',
      headers: { 'Content-Type': 'application/json', 'X-EFB-Token': token },
      credentials: 'same-origin',
      body: JSON.stringify({ icao: icao, stand: name })
    }).then(function (r) {
      popNote = r.status === 202 ? 'Asked. The copilot checks the move, then the flight reloads.'
        : r.status === 429 ? 'One move at a time. Try again in a moment.'
        : r.status === 403 ? 'Refused: the link token is wrong.'
        : 'Not sent (' + r.status + ').';
      renderPop();
    }).catch(function () { popNote = 'Not sent: xfc unreachable.'; renderPop(); });
  }

  // ------------------------------------------------------------ the state

  var WHY = { departure: 'Departure', destination: 'Destination', here: 'This field' };
  function renderInfo() {
    var bits = [], name = '';
    if (st && st.icao) {
      var f = fields[st.icao];
      name = st.icao + (st.name ? ' ' + st.name : (f ? ' ' + f.geo.name : ''));
      if (st.why) bits.push(WHY[st.why] || st.why);
      if (st.runway) bits.push('runway ' + st.runway.ident + ' ' + (st.runway.why === 'the plan\'s' ? 'planned' : st.runway.why));
      if (st.status === 'loading') bits.push('drawing the field');
      else if (st.status === 'unavailable') bits.push('no diagram in the scenery data');
      if (f && f.geo.sources && f.geo.sources.pavement.indexOf('110') < 0) bits.push('pavement from the taxi network');
    }
    var act = activeFix();
    if (act) bits.push('next ' + act.id);
    infoName.textContent = name;
    infoSub.textContent = bits.join(', ');
  }

  function update(s) {
    if (!root) mount();
    if (!root) return;
    var stChanged = JSON.stringify(s && s.airport && [s.airport.icao, s.airport.version, s.airport.route]) !== JSON.stringify(st && [st.icao, st.version, st.route]);
    st = (s && s.airport) || null;
    mp = (s && s.map) || null;
    statusBox.hidden = true;
    var t = performance.now();
    var o = mp && mp.own;
    if (o) {
      // On the ground the nose; flying, the ground track.
      var h = !o.on_ground && o.track !== null && o.track !== undefined && o.gs_kts > 40 ? o.track : o.heading;
      own = glideTo(own, o.lat, o.lon, h, t);
    } else {
      own = null;
    }
    ownD = o || null;
    updateTraffic(mp && mp.traffic, t);
    convertPlan();
    if (stChanged) convertOverlay();
    placeFirst();
    renderInfo();
    renderCard();
    renderPop();
    wantFields();
    dirty = true;
  }

  window.XvlMap = { update: update, mount: mount };
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', mount);
  else mount();
})();
