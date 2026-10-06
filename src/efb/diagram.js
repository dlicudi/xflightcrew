// The EFB's Airport page: the field's diagram on a canvas, the aircraft,
// the taxi route held and the guide's turn card over it.
//
// Geometry comes once per field from GET /airport (a local frame: x east,
// y north, metres from the field's reference point, so nothing here does
// geodesy); the live picture rides the page state the page already
// receives (`state.airport`), handed over by page.html's render().
// No library, no tiles, nothing from the network but this server.
(function () {
  'use strict';

  var KEY = 'xvl_efb_t';
  var PREF = 'xvl_efb_apt';
  var token = '';
  try {
    token = new URLSearchParams(location.search).get('t') || localStorage.getItem(KEY) || '';
  } catch (e) {}

  var root = null, canvas = null, ctx = null;
  var cardBox = null, cardArrow = null, cardText = null, cardSub = null;
  var infoName = null, infoSub = null, statusBox = null, scaleBar = null, scaleText = null, northBox = null;
  var btnFollow = null, btnOrient = null;

  var geo = null;          // the diagram (GET /airport)
  var geoKey = '';         // icao + version of `geo`
  var fetchingKey = '';
  var st = null;           // state.airport
  var W = 0, H = 0, dpr = 1;
  var view = { cx: 0, cy: 0, s: 0.2, rot: 0 };
  var fitted = '';
  var follow = true, trackUp = false;
  var dirty = true;
  var C = {};

  try {
    var saved = JSON.parse(localStorage.getItem(PREF) || '{}');
    if (typeof saved.follow === 'boolean') follow = saved.follow;
    if (typeof saved.trackUp === 'boolean') trackUp = saved.trackUp;
  } catch (e) {}
  function savePrefs() {
    try { localStorage.setItem(PREF, JSON.stringify({ follow: follow, trackUp: trackUp })); } catch (e) {}
  }

  // ------------------------------------------------------------ the aircraft, interpolated

  // Updates arrive about once a second; the chevron glides between them.
  var own = null;   // { from, to, t0, dur } with x, y, h
  function angDiff(a, b) { return ((b - a + 540) % 360) - 180; }
  function ownNow(t) {
    if (!own) return null;
    var k = own.dur > 0 ? Math.min(1, (t - own.t0) / own.dur) : 1;
    return {
      x: own.from.x + (own.to.x - own.from.x) * k,
      y: own.from.y + (own.to.y - own.from.y) * k,
      h: (own.from.h + angDiff(own.from.h, own.to.h) * k + 360) % 360,
      moving: k < 1
    };
  }
  // ------------------------------------------------------------ frame

  function toScreen(x, y) {
    var c = Math.cos(view.rot), s = Math.sin(view.rot);
    var dx = x - view.cx, dy = y - view.cy;
    var xr = dx * c - dy * s, yr = dx * s + dy * c;
    return [W / 2 + view.s * xr, H / 2 - view.s * yr];
  }
  function toWorld(sx, sy) {
    var c = Math.cos(view.rot), s = Math.sin(view.rot);
    var xr = (sx - W / 2) / view.s, yr = -(sy - H / 2) / view.s;
    return [view.cx + xr * c + yr * s, view.cy - xr * s + yr * c];
  }
  function worldTransform() {
    var c = Math.cos(view.rot), s = Math.sin(view.rot), k = view.s;
    var a = k * c, b = -k * s, cc = -k * s, d = -k * c;
    var e = W / 2 - k * (c * view.cx - s * view.cy);
    var f = H / 2 + k * (s * view.cx + c * view.cy);
    ctx.setTransform(dpr * a, dpr * b, dpr * cc, dpr * d, dpr * e, dpr * f);
  }
  function screenTransform() { ctx.setTransform(dpr, 0, 0, dpr, 0, 0); }

  function fieldBounds() {
    if (!geo) return null;
    var b = geo.bounds;
    if (!isFinite(b[0]) || b[2] - b[0] < 1) return null;
    return b;
  }
  function minScale() {
    var b = fieldBounds();
    if (!b || !W) return 0.01;
    return Math.min(W / (b[2] - b[0]), H / (b[3] - b[1])) / 6;
  }
  function clampScale(s) { return Math.max(minScale(), Math.min(8, s)); }
  function fit() {
    var b = fieldBounds();
    if (!b || !W || !H) return;
    view.cx = (b[0] + b[2]) / 2;
    view.cy = (b[1] + b[3]) / 2;
    var pad = 0.9;
    view.s = Math.min(W * pad / (b[2] - b[0]), (H - 120) * pad / (b[3] - b[1]));
    view.s = Math.max(0.01, view.s);
    view.rot = 0;
    dirty = true;
  }
  function followScale() {
    // About 700 m across the shorter side when we start following.
    return clampScale(Math.min(W, H) / 700);
  }

  // ------------------------------------------------------------ DOM

  function el(tag, cls, text) {
    var e = document.createElement(tag);
    if (cls) e.className = cls;
    if (text !== undefined) e.textContent = text;
    return e;
  }
  var ICON = {
    follow: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="7"/><path d="M12 2v4M12 18v4M2 12h4M18 12h4"/><circle cx="12" cy="12" r="2" fill="currentColor"/></svg>',
    north: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M12 3l5 15-5-3-5 3z" fill="currentColor" fill-opacity="0.25"/><path d="M9 21h6"/></svg>',
    track: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M12 3l6 17-6-4-6 4z"/></svg>',
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
    if (!btnFollow) return;
    btnFollow.setAttribute('aria-pressed', follow ? 'true' : 'false');
    btnOrient.innerHTML = trackUp ? ICON.track : ICON.north;
    btnOrient.setAttribute('aria-label', trackUp ? 'Track up (tap for north up)' : 'North up (tap for track up)');
    btnOrient.title = btnOrient.getAttribute('aria-label');
    btnOrient.setAttribute('aria-pressed', trackUp ? 'true' : 'false');
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
    '.apt-info .s{font-size:13px;color:var(--muted);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}',
    '.apt-scale{display:flex;align-items:center;gap:8px;font-size:12px;color:var(--muted);font-family:var(--mono)}',
    '.apt-scale i{display:block;height:6px;border:2px solid var(--soft);border-top:0}',
    '.apt-north{position:absolute;right:12px;top:12px;width:40px;height:40px;border-radius:20px;background:rgba(13,17,23,0.88);border:1px solid var(--line);display:flex;align-items:center;justify-content:center;pointer-events:none}',
    '.apt-north svg{width:28px;height:28px}',
    '.apt-status{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;padding:24px;text-align:center;color:var(--muted);font-size:16px;line-height:1.5;pointer-events:none}',
    '.apt-status[hidden]{display:none}',
    '@media (max-width:700px){.apt-card .main{font-size:19px}.apt-card .arrow{width:46px;height:46px}.apt-card .arrow svg{width:36px;height:36px}.apt-info{bottom:12px}.apt-north{top:auto;bottom:290px}}',
    '@media (max-height:500px){.apt-card{top:8px;padding:6px 12px 6px 6px;width:min(520px,calc(100% - 76px));left:8px;transform:none}.apt-card .main{font-size:18px}.apt-card .arrow{width:40px;height:40px}.apt-card .arrow svg{width:32px;height:32px}.apt-ctl{bottom:8px;right:8px;gap:6px}.apt-ctl .zoom{display:none}.apt-btn{width:44px;height:44px}.apt-info{bottom:8px;left:8px;padding:6px 10px}.apt-info .s{display:none}.apt-north{top:8px;bottom:auto;right:10px}}'
  ].join('\n');

  function readColours() {
    // The map keeps the night palette in a day theme: read where it is drawn.
    var cs = getComputedStyle(root || document.documentElement);
    function v(n, d) { var x = cs.getPropertyValue(n).trim(); return x || d; }
    C = {
      bg: v('--bg', '#07090C'),
      text: v('--text', '#E6EDF3'),
      muted: v('--muted', '#8B98A8'),
      cyan: v('--cyan', '#7FD3F7'),
      green: v('--green', '#7EE2A4'),
      amber: v('--amber', '#FFC266'),
      red: v('--red', '#FF8A80'),
      violet: v('--violet', '#B9A6FF'),
      sans: v('--sans', 'B612, system-ui, sans-serif'),
      mono: v('--mono', 'ui-monospace, monospace'),
      pave: '#1A222C',
      paveEdge: '#26313E',
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

  function mount() {
    var host = document.getElementById('apt-root');
    if (!host || root) return;
    root = host;
    var style = el('style');
    style.textContent = STYLE;
    document.head.appendChild(style);
    readColours();
    canvas = el('canvas');
    canvas.setAttribute('aria-label', 'Airport diagram');
    ctx = canvas.getContext('2d');
    root.appendChild(canvas);

    statusBox = el('div', 'apt-status', 'Waiting for the copilot');
    root.appendChild(statusBox);

    cardBox = el('div', 'apt-card');
    cardBox.hidden = true;
    cardArrow = el('div', 'arrow');
    cardArrow.innerHTML = '<svg viewBox="0 0 48 48"><path d="M24 4 L38 22 L29 22 L29 44 L19 44 L19 22 L10 22 Z" fill="#7FD3F7"/></svg>';
    var txt = el('div', 'txt');
    cardText = el('div', 'main', '');
    cardSub = el('div', 'sub', '');
    txt.appendChild(cardText);
    txt.appendChild(cardSub);
    cardBox.appendChild(cardArrow);
    cardBox.appendChild(txt);
    root.appendChild(cardBox);

    northBox = el('div', 'apt-north');
    northBox.innerHTML = '<svg viewBox="0 0 28 28"><path d="M14 3 L19 15 L14 12.5 L9 15 Z" fill="#FF8A80"/><path d="M14 25 L9 13 L14 15.5 L19 13 Z" fill="#8B98A8"/></svg>';
    root.appendChild(northBox);

    var info = el('div', 'apt-info');
    infoName = el('div', 'n', '');
    infoSub = el('div', 's', '');
    scaleBar = el('div', 'apt-scale');
    var bar = el('i');
    scaleText = el('span', null, '');
    scaleBar.appendChild(bar);
    scaleBar.appendChild(scaleText);
    info.appendChild(infoName);
    info.appendChild(infoSub);
    info.appendChild(scaleBar);
    root.appendChild(info);

    var ctlBox = el('div', 'apt-ctl');
    btnFollow = ctl('follow', 'Follow the aircraft', function () {
      follow = !follow;
      if (follow) view.s = Math.max(view.s, followScale());
      savePrefs(); syncButtons(); dirty = true;
    });
    btnOrient = ctl('north', 'North up', function () {
      trackUp = !trackUp;
      if (trackUp && !follow) { follow = true; view.s = Math.max(view.s, followScale()); }
      savePrefs(); syncButtons(); dirty = true;
    });
    var zin = ctl('plus', 'Zoom in', function () { zoomAt(W / 2, H / 2, 1.6); });
    var zout = ctl('minus', 'Zoom out', function () { zoomAt(W / 2, H / 2, 1 / 1.6); });
    zin.classList.add('zoom');
    zout.classList.add('zoom');
    ctlBox.appendChild(zin);
    ctlBox.appendChild(zout);
    ctlBox.appendChild(ctl('fit', 'The whole field', function () {
      follow = false; savePrefs(); syncButtons(); fit();
    }));
    ctlBox.appendChild(btnOrient);
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
    var first = !W || !H;
    W = w; H = h; dpr = r;
    canvas.width = Math.max(1, Math.round(w * dpr));
    canvas.height = Math.max(1, Math.round(h * dpr));
    if (first && geo) placeView(true);
    dirty = true;
  }

  // ------------------------------------------------------------ gestures

  var pointers = {};
  var gesture = null;
  var lastTap = { t: 0, x: 0, y: 0 };
  function local(e) {
    var r = canvas.getBoundingClientRect();
    return [e.clientX - r.left, e.clientY - r.top];
  }
  function zoomAt(sx, sy, k) {
    var before = toWorld(sx, sy);
    view.s = clampScale(view.s * k);
    if (follow) { dirty = true; return; }
    var after = toWorld(sx, sy);
    view.cx += before[0] - after[0];
    view.cy += before[1] - after[1];
    dirty = true;
  }
  function panBy(dx, dy) {
    var a = toWorld(0, 0), b = toWorld(dx, dy);
    view.cx -= b[0] - a[0];
    view.cy -= b[1] - a[1];
    dirty = true;
  }
  function stopFollowing() {
    if (follow) { follow = false; savePrefs(); syncButtons(); }
  }
  function ids() { return Object.keys(pointers); }
  function wireGestures() {
    canvas.addEventListener('pointerdown', function (e) {
      canvas.setPointerCapture && canvas.setPointerCapture(e.pointerId);
      pointers[e.pointerId] = local(e);
      canvas.classList.add('drag');
      gesture = { moved: 0, start: local(e), t: Date.now() };
      if (ids().length === 2) {
        var p = ids().map(function (k) { return pointers[k]; });
        gesture.pinch = { d: Math.hypot(p[0][0] - p[1][0], p[0][1] - p[1][1]), m: [(p[0][0] + p[1][0]) / 2, (p[0][1] + p[1][1]) / 2] };
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
      } else if (k.length === 1) {
        var dx = now[0] - prev[0], dy = now[1] - prev[1];
        if (gesture) gesture.moved += Math.abs(dx) + Math.abs(dy);
        if (gesture && gesture.moved > 6) { stopFollowing(); panBy(dx, dy); }
      }
    });
    function up(e) {
      if (!(e.pointerId in pointers)) return;
      var p = local(e);
      delete pointers[e.pointerId];
      if (!ids().length) canvas.classList.remove('drag');
      if (gesture && gesture.pinch && ids().length === 1) {
        gesture = { moved: 99, start: p, t: Date.now() };
        return;
      }
      if (gesture && !ids().length && gesture.moved <= 6 && Date.now() - gesture.t < 350) {
        var t = Date.now();
        if (t - lastTap.t < 320 && Math.hypot(p[0] - lastTap.x, p[1] - lastTap.y) < 30) {
          recenter();
          lastTap.t = 0;
        } else {
          lastTap = { t: t, x: p[0], y: p[1] };
        }
      }
      if (!ids().length) gesture = null;
    }
    canvas.addEventListener('pointerup', up);
    canvas.addEventListener('pointercancel', up);
    canvas.addEventListener('dblclick', function (e) { e.preventDefault(); });
    canvas.addEventListener('wheel', function (e) {
      e.preventDefault();
      var p = local(e);
      var k = Math.exp(-e.deltaY * (e.deltaMode === 1 ? 0.05 : 0.0016));
      zoomAt(p[0], p[1], k);
    }, { passive: false });
    // Safari's own pinch events would zoom the page.
    ['gesturestart', 'gesturechange'].forEach(function (n) {
      canvas.addEventListener(n, function (e) { e.preventDefault(); });
    });
  }
  function recenter() {
    if (st && st.own) {
      follow = true;
      view.s = Math.max(view.s, followScale());
      savePrefs(); syncButtons();
    } else {
      fit();
    }
    dirty = true;
  }

  // ------------------------------------------------------------ drawing

  function pathFlat(a, close) {
    if (!a || a.length < 4) return;
    ctx.moveTo(a[0], a[1]);
    for (var i = 2; i < a.length; i += 2) ctx.lineTo(a[i], a[i + 1]);
    if (close) ctx.closePath();
  }
  function px(n) { return n / view.s; }

  function runwayCorners(r) {
    var dx = r.b.x - r.a.x, dy = r.b.y - r.a.y;
    var L = Math.hypot(dx, dy) || 1;
    var nx = -dy / L * r.width_m / 2, ny = dx / L * r.width_m / 2;
    return [[r.a.x + nx, r.a.y + ny], [r.b.x + nx, r.b.y + ny], [r.b.x - nx, r.b.y - ny], [r.a.x - nx, r.a.y - ny]];
  }
  function sameEnd(a, b) {
    var n = function (s) { return String(s || '').trim().toUpperCase().replace(/^0+(?=\d)/, ''); };
    return n(a) === n(b);
  }
  function clearedRunway() {
    if (!st || !st.runway || !geo || geo.icao !== st.icao) return null;
    for (var i = 0; i < geo.runways.length; i++) {
      var r = geo.runways[i];
      if (sameEnd(r.a.id, st.runway.ident)) return { r: r, end: 'a' };
      if (sameEnd(r.b.id, st.runway.ident)) return { r: r, end: 'b' };
    }
    return null;
  }

  // Labels never overlap: each claims its box on the screen.
  var claimed = [];
  function claim(x, y, w, h) {
    var b = [x - w / 2 - 2, y - h / 2 - 2, x + w / 2 + 2, y + h / 2 + 2];
    if (b[2] < 0 || b[3] < 0 || b[0] > W || b[1] > H) return false;
    for (var i = 0; i < claimed.length; i++) {
      var c = claimed[i];
      if (b[0] < c[2] && b[2] > c[0] && b[1] < c[3] && b[3] > c[1]) return false;
    }
    claimed.push(b);
    return true;
  }
  function tag(x, y, text, opts) {
    ctx.font = (opts.bold ? '700 ' : '400 ') + opts.size + 'px ' + C.sans;
    var w = ctx.measureText(text).width + (opts.bg ? 10 : 2);
    var h = opts.size + (opts.bg ? 6 : 2);
    if (!claim(x, y, w, h) && !opts.force) return false;
    if (opts.bg) {
      ctx.fillStyle = opts.bg;
      ctx.fillRect(x - w / 2, y - h / 2, w, h);
      if (opts.border) { ctx.strokeStyle = opts.border; ctx.lineWidth = 1; ctx.strokeRect(x - w / 2 + 0.5, y - h / 2 + 0.5, w - 1, h - 1); }
    }
    ctx.fillStyle = opts.color;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, x, y + 0.5);
    return true;
  }

  function drawGeometry() {
    worldTransform();
    // Pavement.
    ctx.fillStyle = C.pave;
    geo.pavement.forEach(function (poly) {
      ctx.beginPath();
      poly.forEach(function (ring) { pathFlat(ring, true); });
      ctx.fill('evenodd');
    });
    if (geo.network && geo.network.length) {
      ctx.strokeStyle = C.pave;
      ctx.lineWidth = Math.max(geo.network_width_m || 20, px(3));
      ctx.lineCap = 'round';
      ctx.lineJoin = 'round';
      ctx.beginPath();
      geo.network.forEach(function (e) { pathFlat(e.pts, false); });
      ctx.stroke();
    }
    // Runways.
    var cleared = clearedRunway();
    geo.runways.forEach(function (r) {
      var c = runwayCorners(r);
      ctx.beginPath();
      ctx.moveTo(c[0][0], c[0][1]);
      for (var i = 1; i < 4; i++) ctx.lineTo(c[i][0], c[i][1]);
      ctx.closePath();
      var mine = cleared && cleared.r === r;
      ctx.fillStyle = mine ? '#1F4A3A' : (r.hard ? C.rwy : '#2D3B2A');
      ctx.fill();
      ctx.strokeStyle = mine ? C.green : C.rwyEdge;
      ctx.lineWidth = px(mine ? 2.5 : 1);
      ctx.stroke();
      // Centreline, dashed in metres like the paint.
      if (view.s > 0.08) {
        ctx.save();
        ctx.setLineDash([30, 20]);
        ctx.strokeStyle = 'rgba(230, 237, 243, 0.55)';
        ctx.lineWidth = px(1);
        ctx.beginPath();
        ctx.moveTo(r.a.x, r.a.y);
        ctx.lineTo(r.b.x, r.b.y);
        ctx.stroke();
        ctx.restore();
      }
    });
    // The drawn taxi lines.
    if (geo.lines.length && view.s > 0.12) {
      ctx.strokeStyle = C.paint;
      ctx.lineWidth = px(1);
      ctx.lineCap = 'round';
      ctx.lineJoin = 'round';
      ctx.beginPath();
      geo.lines.forEach(function (l) { pathFlat(l, false); });
      ctx.stroke();
    }
  }

  function routeDirAt(pts, x, y) {
    var best = 1e18, dir = 0;
    for (var i = 0; i + 3 < pts.length; i += 2) {
      var ax = pts[i], ay = pts[i + 1], bx = pts[i + 2], by = pts[i + 3];
      var dx = bx - ax, dy = by - ay, L2 = dx * dx + dy * dy;
      var t = L2 > 0 ? Math.max(0, Math.min(1, ((x - ax) * dx + (y - ay) * dy) / L2)) : 0;
      var d = Math.hypot(x - (ax + t * dx), y - (ay + t * dy));
      if (d < best && L2 > 0) { best = d; dir = Math.atan2(dx, dy); }
    }
    return dir;
  }
  function holdBar(m, pts, colour) {
    // A bar across the route, the way a holding line is painted.
    var dir = routeDirAt(pts, m.x, m.y);
    var s = toScreen(m.x, m.y);
    var a = dir - view.rot;
    screenTransform();
    ctx.save();
    ctx.translate(s[0], s[1]);
    ctx.rotate(a);
    ctx.fillStyle = '#000';
    ctx.fillRect(-13, -4, 26, 8);
    ctx.fillStyle = colour;
    ctx.fillRect(-11, -2.5, 22, 2);
    ctx.fillRect(-11, 1, 22, 2);
    ctx.restore();
  }

  function drawRoute() {
    var r = st && st.route;
    if (!r || !geo || geo.icao !== st.icao) return;
    worldTransform();
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    if (r.driven && r.driven.length >= 4) {
      ctx.strokeStyle = C.driven;
      ctx.lineWidth = px(4);
      ctx.beginPath(); pathFlat(r.driven, false); ctx.stroke();
    }
    if (r.remaining && r.remaining.length >= 4) {
      ctx.strokeStyle = C.routeCase;
      ctx.lineWidth = px(9);
      ctx.beginPath(); pathFlat(r.remaining, false); ctx.stroke();
      ctx.strokeStyle = C.route;
      ctx.lineWidth = px(5);
      ctx.beginPath(); pathFlat(r.remaining, false); ctx.stroke();
    }
    var all = (r.driven || []).concat(r.remaining || []);
    (r.holds || []).forEach(function (m) { holdBar(m, all, C.hold); });
    if (r.dest) {
      if (r.dest.kind === 'hold') holdBar(r.dest, all, C.amber);
      var s = toScreen(r.dest.x, r.dest.y);
      screenTransform();
      if (r.dest.kind === 'stand') {
        ctx.fillStyle = C.route;
        ctx.beginPath(); ctx.arc(s[0], s[1], 7, 0, Math.PI * 2); ctx.fill();
        ctx.strokeStyle = '#000'; ctx.lineWidth = 2; ctx.stroke();
      }
      tag(s[0], s[1] - 20, r.dest.label || (r.dest.kind === 'stand' ? 'Stand' : 'Hold'), { size: 13, bold: true, color: '#1A0620', bg: C.route, force: true });
    }
    (r.holds || []).forEach(function (m) {
      var s = toScreen(m.x, m.y);
      screenTransform();
      tag(s[0], s[1] - 18, 'Hold short ' + m.label, { size: 12, bold: true, color: '#FFFFFF', bg: '#8A1F1F' });
    });
  }

  function drawLabels() {
    screenTransform();
    // Runway designators, just beyond each threshold.
    var cleared = clearedRunway();
    geo.runways.forEach(function (r) {
      var dx = r.b.x - r.a.x, dy = r.b.y - r.a.y, L = Math.hypot(dx, dy) || 1;
      var off = Math.max(35, px(22));
      [[r.a, -1, 'a'], [r.b, 1, 'b']].forEach(function (e) {
        var p = toScreen(e[0].x + e[1] * dx / L * off, e[0].y + e[1] * dy / L * off);
        var mine = cleared && cleared.r === r && cleared.end === e[2];
        tag(p[0], p[1], e[0].id, { size: 14, bold: true, color: mine ? '#06140C' : C.text, bg: mine ? C.green : 'rgba(7,9,12,0.8)', force: true });
      });
    });
    // Taxiway names, like the signs: yellow on black.
    if (view.s > 0.16) {
      geo.taxiways.forEach(function (t) {
        var p = toScreen(t.x, t.y);
        tag(p[0], p[1], t.name, { size: 12, bold: true, color: C.signFg, bg: C.signBg, border: '#5A4A10' });
      });
    }
    // Holding positions on the field.
    if (view.s > 0.5) {
      geo.holds.forEach(function (h) {
        var p = toScreen(h.x, h.y);
        if (p[0] < -10 || p[1] < -10 || p[0] > W + 10 || p[1] > H + 10) return;
        ctx.fillStyle = C.hold;
        ctx.fillRect(p[0] - 3, p[1] - 3, 6, 6);
      });
    }
    // Stands: dots, then names when close.
    if (view.s > 0.3) {
      geo.stands.forEach(function (s) {
        var p = toScreen(s.x, s.y);
        if (p[0] < -10 || p[1] < -10 || p[0] > W + 10 || p[1] > H + 10) return;
        ctx.fillStyle = C.stand;
        ctx.beginPath(); ctx.arc(p[0], p[1], 2.5, 0, Math.PI * 2); ctx.fill();
        if (view.s > 1.1 && s.name) tag(p[0], p[1] + 11, s.name, { size: 11, color: C.muted });
      });
    }
  }

  function drawOwn(o) {
    if (!o) return;
    var p = toScreen(o.x, o.y);
    screenTransform();
    ctx.save();
    ctx.translate(p[0], p[1]);
    ctx.rotate((o.h * Math.PI / 180) - view.rot);
    ctx.beginPath();
    ctx.moveTo(0, -17);
    ctx.lineTo(11, 11);
    ctx.lineTo(0, 8);
    ctx.lineTo(-11, 11);
    ctx.closePath();
    ctx.fillStyle = '#FFFFFF';
    ctx.strokeStyle = '#000000';
    ctx.lineWidth = 3;
    ctx.lineJoin = 'round';
    ctx.stroke();
    ctx.fill();
    ctx.restore();
    claimed.push([p[0] - 16, p[1] - 18, p[0] + 16, p[1] + 16]);
  }

  function niceScale() {
    // A bar of 1, 2 or 5 times a power of ten, about 90 px long.
    var m = 90 / view.s;
    var p = Math.pow(10, Math.floor(Math.log10(m)));
    var n = m / p >= 5 ? 5 : m / p >= 2 ? 2 : 1;
    var len = n * p;
    scaleBar.firstChild.style.width = Math.round(len * view.s) + 'px';
    scaleText.textContent = len >= 1000 ? (len / 1000) + ' km' : len + ' m';
  }

  // The boxes over the map: a label under one would be hidden, so they are
  // claimed before any label is placed.
  function overlayRects() {
    var out = [], base = root.getBoundingClientRect();
    root.querySelectorAll('.apt-card, .apt-info, .apt-ctl, .apt-north').forEach(function (e) {
      if (e.hidden || e.offsetParent === null) return;
      var r = e.getBoundingClientRect();
      if (!r.width) return;
      out.push([r.left - base.left, r.top - base.top, r.right - base.left, r.bottom - base.top]);
    });
    return out;
  }

  function frame(t) {
    requestAnimationFrame(frame);
    if (!root || !W || !H || !root.offsetParent) return;
    var o = ownNow(t);
    var moving = o && o.moving;
    var fb = fieldBounds();
    if (follow && o && st && st.own && !st.own.on_ground && fb) {
      // Airborne on the approach: the field and the aircraft together,
      // north up (following at taxi scale would leave the field off screen).
      var x0 = Math.min(fb[0], o.x), x1 = Math.max(fb[2], o.x), y0 = Math.min(fb[1], o.y), y1 = Math.max(fb[3], o.y);
      view.cx = (x0 + x1) / 2; view.cy = (y0 + y1) / 2;
      view.s = Math.max(0.0005, Math.min(W * 0.85 / Math.max(1, x1 - x0), (H - 140) * 0.85 / Math.max(1, y1 - y0)));
      view.rot = 0;
    } else if (follow && o) {
      // Track up: the glided heading, which is the ground track once flying.
      view.cx = o.x; view.cy = o.y;
      view.rot = trackUp ? o.h * Math.PI / 180 : 0;
    } else if (!follow) {
      view.rot = 0;
    }
    if (!dirty && !moving) return;
    dirty = false;
    screenTransform();
    ctx.fillStyle = C.bg;
    ctx.fillRect(0, 0, W, H);
    claimed = overlayRects();
    if (geo) {
      drawGeometry();
      drawRoute();
      drawOwn(o);
      drawLabels();
      niceScale();
      northBox.style.display = view.rot ? 'flex' : 'none';
      northBox.firstChild.style.transform = 'rotate(' + (-view.rot * 180 / Math.PI) + 'deg)';
    }
    updateCardArrow(o);
  }

  // ------------------------------------------------------------ the card

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
    var hdg = o ? o.h : (st.own ? st.own.heading : 0);
    var rel = angDiff(hdg, c.aim_true_deg);
    var svg = cardArrow.firstChild;
    if (svg) svg.style.transform = 'rotate(' + rel.toFixed(1) + 'deg)';
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

  // ------------------------------------------------------------ state

  function fetchGeo(icao, version) {
    var key = icao + '@' + version;
    if (fetchingKey === key) return;
    fetchingKey = key;
    var url = '/airport?icao=' + encodeURIComponent(icao) + (token ? '&t=' + encodeURIComponent(token) : '');
    fetch(url, { headers: token ? { 'X-EFB-Token': token } : {}, credentials: 'same-origin' })
      .then(function (r) { if (!r.ok) throw r.status; return r.json(); })
      .then(function (d) {
        geo = d;
        geoKey = d.icao + '@' + d.version;
        fetchingKey = '';
        placeView(true);
        renderInfo();
        dirty = true;
      })
      .catch(function () { fetchingKey = ''; setTimeout(function () { dirty = true; }, 2000); });
  }

  function placeView(newField) {
    if (!geo || !W || !H) return;
    if (newField && fitted !== geo.icao) {
      fitted = geo.icao;
      fit();
      if (follow && st && st.own) view.s = Math.max(view.s, followScale());
    }
  }

  var WHY = { departure: 'Departure', destination: 'Destination', here: 'This field' };
  function renderInfo() {
    if (!st) return;
    var name = st.icao ? st.icao + (st.name ? ' ' + st.name : (geo && geo.icao === st.icao ? ' ' + geo.name : '')) : 'No field';
    infoName.textContent = name;
    var bits = [];
    if (st.why) bits.push(WHY[st.why] || st.why);
    if (st.runway) bits.push('runway ' + st.runway.ident + ' ' + (st.runway.why === 'the plan\'s' ? 'planned' : st.runway.why));
    if (geo && geo.sources && geo.sources.pavement.indexOf('110') < 0) bits.push('pavement from the taxi network');
    infoSub.textContent = bits.join(', ');
    var msg = '';
    if (st.status === 'none') msg = 'No field to draw yet: load a flight plan, or tune a station.';
    else if (st.status === 'loading') msg = 'Drawing ' + st.icao + ' from the scenery data...';
    else if (st.status === 'unavailable') msg = 'The scenery data has no diagram for ' + st.icao + '.';
    var showing = geo && st.icao && geo.icao === st.icao;
    statusBox.hidden = !msg || showing;
    statusBox.textContent = msg;
  }

  function update(s) {
    if (!root) mount();
    if (!root) return;
    var a = s && s.airport;
    st = a || null;
    if (a && a.status === 'ready' && a.icao && a.version && geoKey !== a.icao + '@' + a.version) {
      fetchGeo(a.icao, a.version);
    }
    var o = a && a.own;
    var t = performance.now();
    if (o && geo && a.icao === geo.icao) {
      var cur = ownNow(t);
      var to = { x: o.x, y: o.y, h: o.heading };
      if (!o.on_ground && o.track !== null && o.track !== undefined && o.gs_kts > 40) to.h = o.track;
      var from = cur ? { x: cur.x, y: cur.y, h: cur.h } : to;
      var gap = own ? Math.min(1500, Math.max(300, t - own.t0)) : 0;
      // A jump (a reposition) is a jump, never a slide across the field.
      if (Math.hypot(to.x - from.x, to.y - from.y) > 400) from = to;
      own = { from: from, to: to, t0: t, dur: gap };
    } else {
      own = null;
    }
    renderInfo();
    renderCard();
    dirty = true;
  }

  window.XvlDiagram = { update: update, mount: mount };
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', mount);
  else mount();
})();
