/* Pantograph Studio
 * A linkage simulator: rigid bars with evenly spaced holes, joined by pivots,
 * solved with position-based constraint projection.
 */
(() => {
  'use strict';

  // ---------- Constants ----------
  const UNIT = 30;          // px between holes
  const BAR_W = 16;         // bar thickness
  const BAR_CAP = 11;       // how far a bar extends past its end holes
  const HOLE_R = 3.6;
  const HIT_R = 12;         // hole pick radius
  const SNAP_R = 18;        // joining radius
  const ITERS = 24;         // solver iterations per substep
  const STEP = 2.5;         // max px a dragged point moves per substep
  const MAX_SUB = 60;       // substeps per frame
  const TRAY = [4, 6, 8, 9, 12, 16];
  const MATERIALS = ['#b9844c', '#8e9aa8', '#c9a227', '#5f9e95', '#a8634a', '#7b6fa8'];
  // Orange is reserved for the tracer, so pens never use it.
  const PEN_COLORS = [
    { name: 'Red', c: '#e03131' },
    { name: 'Blue', c: '#1c7ed6' },
    { name: 'Green', c: '#2f9e44' },
    { name: 'Yellow', c: '#f2c200' },
    { name: 'Black', c: '#212529' },
  ];
  const TRACER_COLOR = '#e8590c';

  // ---------- DOM ----------
  const $ = (id) => document.getElementById(id);
  const canvas = $('board');
  const ctx = canvas.getContext('2d');
  const ink = document.createElement('canvas');
  const inkCtx = ink.getContext('2d');
  let W = 0, H = 0, DPR = 1;

  // ---------- State ----------
  const state = {
    bars: [],        // {id, n, p:[{x,y},{x,y}], color}
    links: [],       // {a:{bar,h}, b:{bar,h}}
    pins: [],        // {bar, h, x, y}
    tracer: null,    // {bar, h}
    pens: [],        // {bar, h, color}
    nextId: 1,
    mode: 'build',
    strokes: [],     // {color, width, pts:[], gpts:[], group}
    strokeGroup: 0,
    showGuides: true,
    penWidth: 2.5,
    image: null,     // {img, x, y, w, h, opacity}
    drag: null,
    drawing: null,
    snap: null,
    hover: null,
    mouse: { x: 0, y: 0 },
    shift: false,
    imgDrag: null,
  };

  // ---------- Geometry helpers ----------
  const key = (r) => r.bar + ':' + r.h;
  const barById = (id) => state.bars.find((b) => b.id === id);
  const lerp = (a, b, t) => ({ x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t });
  const dist = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);
  const pointAt = (bar, t, s = 0) => {
    const a = bar.p[0], b = bar.p[1];
    const dx = b.x - a.x, dy = b.y - a.y;
    return { x: a.x + t * dx - s * dy, y: a.y + t * dy + s * dx };
  };
  const holePos = (bar, h) => { const o = bar.holes[h]; return pointAt(bar, o.t, o.s); };
  const straightHoles = (n) => Array.from({ length: n + 1 }, (_, h) => ({ t: h / n, s: 0 }));
  const refPos = (r) => {
    if (!r) return null;
    const b = barById(r.bar);
    return b ? holePos(b, r.h) : null;
  };

  // Union-find over holes joined by links → joints
  function computeGroups() {
    const parent = new Map();
    const find = (k) => {
      while (parent.get(k) !== k) {
        parent.set(k, parent.get(parent.get(k)));
        k = parent.get(k);
      }
      return k;
    };
    for (const l of state.links) {
      const a = key(l.a), b = key(l.b);
      if (!parent.has(a)) parent.set(a, a);
      if (!parent.has(b)) parent.set(b, b);
      const ra = find(a), rb = find(b);
      if (ra !== rb) parent.set(ra, rb);
    }
    const groups = new Map();
    for (const k of parent.keys()) {
      const r = find(k);
      if (!groups.has(r)) groups.set(r, []);
      const [bar, h] = k.split(':').map(Number);
      groups.get(r).push({ bar, h });
    }
    return { root: (r) => (parent.has(key(r)) ? find(key(r)) : key(r)), groups };
  }

  function sameJoint(a, b, G = computeGroups()) {
    return !!a && !!b && G.root(a) === G.root(b);
  }

  function barsJoined(idA, idB, G = computeGroups()) {
    for (const members of G.groups.values()) {
      let hasA = false, hasB = false;
      for (const m of members) {
        if (m.bar === idA) hasA = true;
        if (m.bar === idB) hasB = true;
      }
      if (hasA && hasB) return true;
    }
    return false;
  }

  // ---------- Solver (position-based constraint projection) ----------
  // A point fixed to a bar is described in the bar's own frame: t along the
  // bar (0 = first end, 1 = last end) and s sideways (for angled arms).
  // P = p0 + t·d + s·perp(d), where d = p1 − p0 and perp(d) = (−dy, dx).
  // That is linear in p0 and p1, so each constraint projection is closed-form.
  const weight = (t, s) => (1 - t) ** 2 + t ** 2 + 2 * s * s;

  function nudge(bar, t, s, lx, ly) {
    const a = bar.p[0], b = bar.p[1], w = 1 - t;
    a.x += w * lx - s * ly; a.y += s * lx + w * ly;
    b.x += t * lx + s * ly; b.y += -s * lx + t * ly;
  }

  function projPoint(bar, t, s, x, y) {
    const p = pointAt(bar, t, s);
    const w = weight(t, s);
    nudge(bar, t, s, (x - p.x) / w, (y - p.y) / w);
  }

  function projLink(A, ta, sa, B, tb, sb) {
    const pa = pointAt(A, ta, sa), pb = pointAt(B, tb, sb);
    const w = weight(ta, sa) + weight(tb, sb);
    const lx = (pb.x - pa.x) / w, ly = (pb.y - pa.y) / w;
    nudge(A, ta, sa, lx, ly);
    nudge(B, tb, sb, -lx, -ly);
  }

  function projLen(bar) {
    const a = bar.p[0], b = bar.p[1];
    const dx = b.x - a.x, dy = b.y - a.y;
    const d = Math.hypot(dx, dy) || 1e-6;
    const k = (d - bar.n * UNIT) / d / 2;
    a.x += dx * k; a.y += dy * k;
    b.x -= dx * k; b.y -= dy * k;
  }

  // Resolve link/hole refs to bar objects once per solve
  function compiled() {
    const links = [];
    for (const l of state.links) {
      const A = barById(l.a.bar), B = barById(l.b.bar);
      if (A && B) {
        const ha = A.holes[l.a.h], hb = B.holes[l.b.h];
        links.push([A, ha.t, ha.s, B, hb.t, hb.s]);
      }
    }
    const pins = [];
    for (const p of state.pins) {
      const b = barById(p.bar);
      if (b) pins.push([b, b.holes[p.h].t, b.holes[p.h].s, p]);
    }
    return { links, pins };
  }

  function solve(iters, drag) {
    const { links, pins } = compiled();
    const dragBar = drag ? barById(drag.bar) : null;
    const settle = Math.max(4, Math.round(iters * 0.35));
    for (let i = 0; i < iters + settle; i++) {
      if (dragBar && i < iters) projPoint(dragBar, drag.t, drag.s || 0, drag.x, drag.y);
      for (const [b, t, s, p] of pins) projPoint(b, t, s, p.x, p.y);
      for (const [A, ta, sa, B, tb, sb] of links) projLink(A, ta, sa, B, tb, sb);
      for (const b of state.bars) projLen(b);
    }
  }

  // Move a dragged point toward the mouse in small substeps so the linkage
  // follows continuously (and the pen leaves a smooth line).
  function stepDrag() {
    const d = state.drag;
    if (!d) return;
    if (d.kind === 'pin') {
      const p = d.pin;
      for (let s = 0; s < MAX_SUB; s++) {
        const dd = Math.hypot(d.mx - p.x, d.my - p.y);
        if (dd < 0.3) break;
        const st = Math.min(STEP * 2, dd);
        p.x += ((d.mx - p.x) / dd) * st;
        p.y += ((d.my - p.y) / dd) * st;
        solve(ITERS, null);
      }
      return;
    }
    const bar = barById(d.bar);
    if (!bar) { state.drag = null; return; }
    for (let s = 0; s < MAX_SUB; s++) {
      const cur = pointAt(bar, d.t, d.s || 0);
      const dx = d.mx - cur.x, dy = d.my - cur.y;
      const dd = Math.hypot(dx, dy);
      if (dd < 0.25) break;
      const st = Math.min(STEP, dd);
      solve(ITERS, { bar: d.bar, t: d.t, s: d.s || 0, x: cur.x + (dx / dd) * st, y: cur.y + (dy / dd) * st });
      recordPen();
      const now = pointAt(bar, d.t, d.s || 0);
      if (Math.hypot(now.x - cur.x, now.y - cur.y) < 0.05) { d.stuck = true; break; }
      d.stuck = false;
    }
  }

  // ---------- Drawing (ink) ----------
  function recordPen() {
    const dr = state.drawing;
    if (!dr) return;
    const tp = refPos(state.tracer);
    if (!tp || !state.pens.length) return;
    if (state.shift) { dr.strokes = null; return; }
    if (!dr.strokes) {
      // One stroke per pen; they share a group so a single undo removes them all.
      const group = ++state.strokeGroup;
      dr.strokes = state.pens.map((pen, i) => {
        const pp = refPos(pen);
        const s = { color: pen.color, width: state.penWidth, pts: pp ? [pp] : [], gpts: i === 0 ? [tp] : [], group, pen };
        state.strokes.push(s);
        if (pp) inkDot(s);
        return s;
      });
      return;
    }
    for (const s of dr.strokes) {
      const pp = refPos(s.pen);
      if (!pp || !s.pts.length) continue;
      const lp = s.pts[s.pts.length - 1];
      if (dist(lp, pp) > 0.6) {
        s.pts.push(pp);
        inkSegment(lp, pp, s.color, s.width, 1);
      }
    }
    const g = dr.strokes[0];
    const lg = g.gpts[g.gpts.length - 1];
    if (dist(lg, tp) > 0.8) {
      g.gpts.push(tp);
      if (state.showGuides) inkSegment(lg, tp, TRACER_COLOR, 1.5, 0.45);
    }
  }

  function inkSegment(a, b, color, width, alpha) {
    inkCtx.save();
    inkCtx.globalAlpha = alpha;
    inkCtx.strokeStyle = color;
    inkCtx.lineWidth = width;
    inkCtx.lineCap = 'round';
    inkCtx.beginPath();
    inkCtx.moveTo(a.x, a.y);
    inkCtx.lineTo(b.x, b.y);
    inkCtx.stroke();
    inkCtx.restore();
  }

  function inkDot(s) {
    const p = s.pts[0];
    inkCtx.fillStyle = s.color;
    inkCtx.beginPath();
    inkCtx.arc(p.x, p.y, s.width / 2, 0, Math.PI * 2);
    inkCtx.fill();
  }

  function strokePath(c, pts, color, width, alpha) {
    if (pts.length < 2) return;
    c.save();
    c.globalAlpha = alpha;
    c.strokeStyle = color;
    c.lineWidth = width;
    c.lineCap = 'round';
    c.lineJoin = 'round';
    c.beginPath();
    c.moveTo(pts[0].x, pts[0].y);
    for (let i = 1; i < pts.length; i++) c.lineTo(pts[i].x, pts[i].y);
    c.stroke();
    c.restore();
  }

  function paintStrokes(c, withGuides) {
    if (withGuides) for (const s of state.strokes) strokePath(c, s.gpts, TRACER_COLOR, 1.5, 0.45);
    for (const s of state.strokes) {
      if (s.pts.length === 1) {
        c.fillStyle = s.color;
        c.beginPath();
        c.arc(s.pts[0].x, s.pts[0].y, s.width / 2, 0, Math.PI * 2);
        c.fill();
      } else strokePath(c, s.pts, s.color, s.width, 1);
    }
  }

  function rerenderInk() {
    inkCtx.setTransform(1, 0, 0, 1, 0, 0);
    inkCtx.clearRect(0, 0, ink.width, ink.height);
    inkCtx.setTransform(DPR, 0, 0, DPR, 0, 0);
    paintStrokes(inkCtx, state.showGuides);
  }

  // ---------- Mechanism editing ----------
  function addBar(n, x, y, angle = 0) {
    const L = n * UNIT;
    const c = Math.cos(angle), s = Math.sin(angle);
    const bar = {
      id: state.nextId++,
      n,
      color: MATERIALS[(state.nextId - 2) % MATERIALS.length],
      p: [{ x: x - (c * L) / 2, y: y - (s * L) / 2 }, { x: x + (c * L) / 2, y: y + (s * L) / 2 }],
      holes: straightHoles(n),
      arms: [],
    };
    state.bars.push(bar);
    return bar;
  }

  // Weld a rigid arm onto a bar at one of its holes, angled `angle` radians
  // from the bar (positive = clockwise on screen), `units` hole-spaces long.
  // Returns the index of the hole at the arm's tip.
  function addArm(bar, baseHole, angle, units) {
    const base = bar.holes[baseHole];
    const c = Math.cos(angle), s = Math.sin(angle);
    for (let j = 1; j <= units; j++) {
      bar.holes.push({ t: base.t + (c * j) / bar.n, s: base.s + (s * j) / bar.n });
    }
    bar.arms.push({ from: base, to: bar.holes[bar.holes.length - 1] });
    return bar.holes.length - 1;
  }

  function addBarFromTray(n) {
    const k = state.bars.length % 6;
    const x = W / 2 + (k % 2 ? 40 : -40);
    const y = Math.min(H - 60, 110 + k * 55);
    addBar(n, x, y, 0);
    hideEmpty();
    changed();
    toast(`Added a ${n}-space bar. Drag a hole onto another bar's hole to join them.`);
  }

  function removeBar(id) {
    state.bars = state.bars.filter((b) => b.id !== id);
    state.links = state.links.filter((l) => l.a.bar !== id && l.b.bar !== id);
    state.pins = state.pins.filter((p) => p.bar !== id);
    if (state.tracer && state.tracer.bar === id) state.tracer = null;
    state.pens = state.pens.filter((p) => p.bar !== id);
  }

  // ---------- Pens ----------
  const penAtJoint = (r, G = computeGroups()) => state.pens.find((p) => sameJoint(p, r, G));
  const removePensAt = (r, G = computeGroups()) => {
    const before = state.pens.length;
    state.pens = state.pens.filter((p) => !sameJoint(p, r, G));
    return state.pens.length !== before;
  };
  const colorName = (c) => (PEN_COLORS.find((p) => p.c === c) || { name: 'Pen' }).name;

  function nextPenColor() {
    const used = new Set(state.pens.map((p) => p.color));
    const free = PEN_COLORS.find((p) => !used.has(p.c));
    return free ? free.c : null;
  }

  function addPen(hole) {
    const color = nextPenColor();
    if (!color) { toast(`All ${PEN_COLORS.length} pens are in use — remove one first.`); return false; }
    state.pens.push({ bar: hole.bar, h: hole.h, color });
    return true;
  }

  // Joining bars can merge holes; keep one pen per joint and none on the tracer/pins.
  function tidyPens() {
    const G = computeGroups();
    const seen = new Set();
    state.pens = state.pens.filter((p) => {
      const r = G.root(p);
      if (seen.has(r)) return false;
      seen.add(r);
      if (state.tracer && sameJoint(state.tracer, p, G)) return false;
      if (pinAtJoint(p, G)) return false;
      return true;
    });
  }

  function linkHoles(a, b) {
    if (a.bar === b.bar) return false;
    const G = computeGroups();
    if (sameJoint(a, b, G)) return false;
    if (barsJoined(a.bar, b.bar, G)) {
      toast('Those two bars are already joined — a second joint would weld them solid.');
      return false;
    }
    state.links.push({ a: { ...a }, b: { ...b } });
    tidyPens();
    solve(200, null);
    return true;
  }

  function pinAtJoint(r, G = computeGroups()) {
    return state.pins.find((p) => sameJoint(p, r, G));
  }

  function clearMechanism() {
    state.bars = [];
    state.links = [];
    state.pins = [];
    state.tracer = null;
    state.pens = [];
    state.drag = null;
    state.drawing = null;
  }

  // Preset specs come from the Quick assemble menu, e.g. "scale:0.5", "flip:2", "tilt:20"
  function buildPreset(spec) {
    if (typeof spec === 'number') spec = 'scale:' + spec;
    const [kind, val] = String(spec).split(':');
    const v = parseFloat(val);
    if (kind === 'tilt') buildTilted(v);
    else buildClassic(v, kind === 'flip');
  }

  function presetDone(msg) {
    solve(100, null);
    hideEmpty();
    changed();
    setMode('draw');
    toast(msg);
  }

  // A classic 4-bar pantograph.
  // Pinned at the end: pen copy = k × tracing (same way up).
  // Pinned at the middle joint: pen copy = −(1−k)/k × tracing (upside-down).
  function buildClassic(scale, flip) {
    clearMechanism();
    const n = 12;
    let k;
    if (flip) k = 1 / (1 + scale);          // |(1−k)/k| = scale
    else k = scale < 1 ? scale : 1 / scale;
    const m = Math.round(k * n);
    const L = n * UNIT;
    const ang = (52 * Math.PI) / 180;
    const c = Math.cos(ang), s = Math.sin(ang);
    const span = 2 * L * c;
    const O = { x: Math.max(60, W / 2 - span / 2), y: Math.min(H - 70, H / 2 + (L * s) / 2) };
    const P1 = { x: O.x + L * c, y: O.y - L * s };
    const T = { x: P1.x + L * c, y: P1.y + L * s };
    const M1 = lerp(O, P1, m / n);
    const M2 = lerp(P1, T, m / n);
    const D = { x: M1.x + M2.x - P1.x, y: M1.y + M2.y - P1.y };
    const mk = (n_, a, b) => {
      const bar = addBar(n_, 0, 0);
      bar.p = [{ ...a }, { ...b }];
      return bar;
    };
    const b1 = mk(n, O, P1);
    const b2 = mk(n, P1, T);
    const b3 = mk(m, M1, D);
    const b4 = mk(n - m, D, M2);
    state.links.push(
      { a: { bar: b1.id, h: n }, b: { bar: b2.id, h: 0 } },
      { a: { bar: b1.id, h: m }, b: { bar: b3.id, h: 0 } },
      { a: { bar: b3.id, h: m }, b: { bar: b4.id, h: 0 } },
      { a: { bar: b4.id, h: n - m }, b: { bar: b2.id, h: m } },
    );
    const pen = (r) => [{ ...r, color: PEN_COLORS[0].c }];
    const end = { bar: b1.id, h: 0 }, far = { bar: b2.id, h: n }, mid = { bar: b3.id, h: m };
    if (flip) {
      state.pins.push({ ...mid, x: D.x, y: D.y });
      state.tracer = end;
      state.pens = pen(far);
      return presetDone('Ready: pinned in the middle — trace with the left end and the copy comes out upside-down.');
    }
    state.pins.push({ ...end, x: O.x, y: O.y });
    if (scale < 1) { state.tracer = far; state.pens = pen(mid); }
    else { state.tracer = mid; state.pens = pen(far); }
    presetDone(scale < 1
      ? 'Ready: trace near the far end — the pen draws a smaller copy.'
      : 'Ready: trace near the middle — the pen draws an enlarged copy.');
  }

  // Skew pantograph (Sylvester's plagiograph): a hinged rhombus O-A-C-B pinned
  // at O, with a rigid arm welded on at A (tracer) and at B (pen).
  // With u = A−O and v = B−O: tracer = O + u + rot(v, −θ), pen = O + v + rot(u, θ),
  // so pen − O = rot(tracer − O, θ): the copy is the same size, turned by θ.
  function buildTilted(deg) {
    clearMechanism();
    const n = 9;
    const L = n * UNIT;
    const th = (deg * Math.PI) / 180;
    const dir = (a) => ({ x: Math.cos(a) * L, y: Math.sin(a) * L });
    const u = dir((-15 * Math.PI) / 180);
    const v = dir((-80 * Math.PI) / 180);
    const O = { x: W / 2 - 0.65 * L, y: Math.min(H - 60, H / 2 + 0.7 * L) };
    const A = { x: O.x + u.x, y: O.y + u.y };
    const B = { x: O.x + v.x, y: O.y + v.y };
    const C = { x: A.x + v.x, y: A.y + v.y };
    const mk = (a, b) => {
      const bar = addBar(n, 0, 0);
      bar.p = [{ ...a }, { ...b }];
      return bar;
    };
    const bOA = mk(O, A);
    const bOB = mk(O, B);
    const bAC = mk(A, C);
    const bBC = mk(B, C);
    const tracerHole = addArm(bAC, 0, -th, n);
    const penHole = addArm(bBC, 0, th, n);
    state.links.push(
      { a: { bar: bOA.id, h: 0 }, b: { bar: bOB.id, h: 0 } },
      { a: { bar: bOA.id, h: n }, b: { bar: bAC.id, h: 0 } },
      { a: { bar: bOB.id, h: n }, b: { bar: bBC.id, h: 0 } },
      { a: { bar: bAC.id, h: n }, b: { bar: bBC.id, h: n } },
    );
    state.pins.push({ bar: bOA.id, h: 0, x: O.x, y: O.y });
    state.tracer = { bar: bAC.id, h: tracerHole };
    state.pens = [{ bar: bBC.id, h: penHole, color: PEN_COLORS[0].c }];
    const side = deg > 0 ? 'right (clockwise)' : 'left (counter-clockwise)';
    presetDone(`Ready: the angled arms make the pen draw a copy turned ${Math.abs(deg)}° to the ${side}.`);
  }

  // ---------- Hit testing ----------
  function hitHole(m, radius = HIT_R) {
    let best = null, bd = radius;
    for (let i = state.bars.length - 1; i >= 0; i--) {
      const b = state.bars[i];
      for (let h = 0; h < b.holes.length; h++) {
        const d = dist(holePos(b, h), m);
        if (d < bd) { bd = d; best = { bar: b.id, h }; }
      }
    }
    return best;
  }

  function hitBar(m) {
    for (let i = state.bars.length - 1; i >= 0; i--) {
      const b = state.bars[i];
      const a = b.p[0], c = b.p[1];
      const dx = c.x - a.x, dy = c.y - a.y;
      const L2 = dx * dx + dy * dy || 1;
      const L = Math.sqrt(L2);
      const t = ((m.x - a.x) * dx + (m.y - a.y) * dy) / L2;
      const tc = Math.max(-BAR_CAP / L, Math.min(1 + BAR_CAP / L, t));
      const p = { x: a.x + dx * tc, y: a.y + dy * tc };
      if (dist(p, m) < BAR_W / 2 + 2) return { bar: b.id, t: Math.max(0, Math.min(1, t)) };
    }
    return null;
  }

  function hitImage(m) {
    const im = state.image;
    if (!im) return null;
    if (Math.hypot(m.x - (im.x + im.w), m.y - (im.y + im.h)) < 14) return 'resize';
    if (m.x >= im.x && m.x <= im.x + im.w && m.y >= im.y && m.y <= im.y + im.h) return 'move';
    return null;
  }

  function bringToFront(id) {
    const i = state.bars.findIndex((b) => b.id === id);
    if (i >= 0) state.bars.push(state.bars.splice(i, 1)[0]);
  }

  function findSnap(d) {
    const gb = barById(d.bar);
    if (!gb || d.hole == null) return null;
    const gp = holePos(gb, d.hole.h);
    const G = computeGroups();
    let best = null, bd = SNAP_R;
    for (const b of state.bars) {
      if (b.id === gb.id || barsJoined(gb.id, b.id, G)) continue;
      for (let h = 0; h < b.holes.length; h++) {
        const dd = dist(holePos(b, h), gp);
        if (dd < bd) { bd = dd; best = { bar: b.id, h }; }
      }
    }
    return best;
  }

  // ---------- Pointer interaction ----------
  function pointer(e) {
    const r = canvas.getBoundingClientRect();
    return { x: e.clientX - r.left, y: e.clientY - r.top };
  }

  canvas.addEventListener('pointerdown', (e) => {
    if (e.button !== 0) return;
    const m = pointer(e);
    state.mouse = m;
    canvas.setPointerCapture(e.pointerId);
    const G = computeGroups();

    switch (state.mode) {
      case 'build': {
        const hole = hitHole(m);
        if (hole) {
          const pin = pinAtJoint(hole, G);
          if (pin) {
            state.drag = { kind: 'pin', pin, mx: m.x, my: m.y };
            return;
          }
          const b = barById(hole.bar);
          bringToFront(b.id);
          state.drag = { kind: 'build', bar: b.id, t: b.holes[hole.h].t, s: b.holes[hole.h].s, hole, mx: m.x, my: m.y };
          return;
        }
        const hb = hitBar(m);
        if (hb) {
          bringToFront(hb.bar);
          state.drag = { kind: 'build', bar: hb.bar, t: hb.t, hole: null, mx: m.x, my: m.y };
        }
        return;
      }
      case 'pin': {
        const hole = hitHole(m);
        if (!hole) { if (hitBar(m)) toast('Click on one of the holes.'); return; }
        const existing = pinAtJoint(hole, G);
        if (existing) state.pins = state.pins.filter((p) => p !== existing);
        else {
          if (state.tracer && sameJoint(state.tracer, hole, G)) state.tracer = null;
          removePensAt(hole, G);
          const p = refPos(hole);
          state.pins.push({ ...hole, x: p.x, y: p.y });
        }
        changed();
        return;
      }
      case 'tracer': {
        const hole = hitHole(m);
        if (!hole) { if (hitBar(m)) toast('Click on one of the holes.'); return; }
        removePensAt(hole, G);
        const pin = pinAtJoint(hole, G);
        if (pin) state.pins = state.pins.filter((p) => p !== pin);
        state.tracer = hole;
        changed();
        return;
      }
      case 'pen': {
        const hole = hitHole(m);
        if (!hole) { if (hitBar(m)) toast('Click on one of the holes.'); return; }
        // Clicking an existing pen lifts it off
        if (removePensAt(hole, G)) { changed(); return; }
        if (!nextPenColor()) { toast(`All ${PEN_COLORS.length} pens are in use — remove one first.`); return; }
        if (state.tracer && sameJoint(state.tracer, hole, G)) state.tracer = null;
        const pin = pinAtJoint(hole, G);
        if (pin) state.pins = state.pins.filter((p) => p !== pin);
        addPen(hole);
        changed();
        return;
      }
      case 'remove': {
        const hole = hitHole(m);
        if (hole) {
          // 1) roles on this joint
          let removed = false;
          const pin = pinAtJoint(hole, G);
          if (pin) { state.pins = state.pins.filter((p) => p !== pin); removed = true; }
          if (state.tracer && sameJoint(state.tracer, hole, G)) { state.tracer = null; removed = true; }
          if (removePensAt(hole, G)) removed = true;
          if (removed) { changed(); return; }
          // 2) joint
          const k = key(hole);
          const before = state.links.length;
          state.links = state.links.filter((l) => key(l.a) !== k && key(l.b) !== k);
          if (state.links.length !== before) {
            // nudge the freed bar so the separation is visible
            const b = barById(hole.bar);
            for (const p of b.p) { p.x += 8; p.y += 8; }
            changed();
            return;
          }
        }
        const hb = hitBar(m);
        if (hb) { removeBar(hb.bar); changed(); }
        return;
      }
      case 'image': {
        const h = hitImage(m);
        if (!state.image) { toast('Upload an image first (left panel).'); return; }
        if (h) state.imgDrag = { kind: h, ox: m.x - state.image.x, oy: m.y - state.image.y };
        return;
      }
      case 'draw': {
        if (!state.tracer || !state.pens.length) {
          toast(!state.tracer ? 'Place a tracer first (Tracer tool).' : 'Place a pen first (Pen tool).');
          return;
        }
        if (!state.pins.length) toast('Tip: pin a pivot so the pen scales your tracing.');
        const b = barById(state.tracer.bar);
        state.drag = { kind: 'trace', bar: b.id, t: b.holes[state.tracer.h].t, s: b.holes[state.tracer.h].s, mx: m.x, my: m.y };
        // Bring the tracer to the pointer with the pen lifted
        for (let i = 0; i < 20 && !state.drag.stuck; i++) {
          stepDrag();
          if (dist(refPos(state.tracer), m) < 0.5) break;
        }
        if (state.drag.stuck && dist(refPos(state.tracer), m) > 4) toast("The tracer can't reach there.");
        resetDrift();
        state.drawing = { strokes: null };
        recordPen();
        return;
      }
    }
  });

  canvas.addEventListener('pointermove', (e) => {
    const m = pointer(e);
    state.mouse = m;
    if (state.drag) { state.drag.mx = m.x; state.drag.my = m.y; }
    if (state.imgDrag && state.image) {
      const im = state.image;
      if (state.imgDrag.kind === 'move') { im.x = m.x - state.imgDrag.ox; im.y = m.y - state.imgDrag.oy; }
      else {
        const aspect = im.h / im.w;
        im.w = Math.max(40, m.x - im.x);
        im.h = im.w * aspect;
      }
    }
    updateCursor(m);
  });

  const endPointer = () => {
    const d = state.drag;
    if (d && d.kind === 'build' && state.snap) {
      if (linkHoles(d.hole, state.snap)) toast('Joined! Pivot created.');
      changed();
    }
    if (d && d.kind === 'pin') changed();
    state.drag = null;
    state.drawing = null;
    state.snap = null;
    state.imgDrag = null;
  };
  canvas.addEventListener('pointerup', endPointer);
  canvas.addEventListener('pointercancel', endPointer);

  canvas.addEventListener('wheel', (e) => {
    if (state.mode !== 'build') return;
    const m = pointer(e);
    const hb = hitBar(m);
    if (!hb) return;
    e.preventDefault();
    const b = barById(hb.bar);
    const c = pointAt(b, hb.t);
    const ang = (e.deltaY > 0 ? 1 : -1) * (Math.PI / 36);
    const cs = Math.cos(ang), sn = Math.sin(ang);
    for (const p of b.p) {
      const dx = p.x - c.x, dy = p.y - c.y;
      p.x = c.x + dx * cs - dy * sn;
      p.y = c.y + dx * sn + dy * cs;
    }
    solve(60, { bar: b.id, t: hb.t, x: c.x, y: c.y });
  }, { passive: false });

  function updateCursor(m) {
    let cur = 'default';
    state.hover = null;
    if (state.drag) cur = state.mode === 'draw' ? 'crosshair' : 'grabbing';
    else if (state.mode === 'build') {
      const h = hitHole(m);
      if (h) { cur = 'grab'; state.hover = h; }
      else if (hitBar(m)) cur = 'move';
    } else if (['pin', 'tracer', 'pen', 'remove'].includes(state.mode)) {
      const h = hitHole(m);
      if (h) { cur = 'pointer'; state.hover = h; }
      else if (state.mode === 'remove' && hitBar(m)) cur = 'pointer';
      else cur = 'crosshair';
    } else if (state.mode === 'image') {
      const h = hitImage(m);
      cur = h === 'resize' ? 'nwse-resize' : h === 'move' ? 'move' : 'default';
    } else if (state.mode === 'draw') cur = 'crosshair';
    canvas.style.cursor = cur;
  }

  // ---------- Rendering ----------
  function shade(hex, amt) {
    const c = parseInt(hex.slice(1), 16);
    let r = (c >> 16) & 255, g = (c >> 8) & 255, b = c & 255;
    const f = amt < 0 ? 0 : 255, p = Math.abs(amt);
    r = Math.round((f - r) * p + r); g = Math.round((f - g) * p + g); b = Math.round((f - b) * p + b);
    return `rgb(${r},${g},${b})`;
  }

  function roundRect(c, x, y, w, h, r) {
    c.beginPath();
    c.moveTo(x + r, y);
    c.arcTo(x + w, y, x + w, y + h, r);
    c.arcTo(x + w, y + h, x, y + h, r);
    c.arcTo(x, y + h, x, y, r);
    c.arcTo(x, y, x + w, y, r);
    c.closePath();
  }

  function drawPaper() {
    ctx.fillStyle = '#fbf8f1';
    ctx.fillRect(0, 0, W, H);
    ctx.lineWidth = 1;
    for (let x = 0; x <= W; x += UNIT) {
      ctx.strokeStyle = x % (UNIT * 5) === 0 ? '#e7dcc8' : '#f1eadc';
      ctx.beginPath(); ctx.moveTo(x + 0.5, 0); ctx.lineTo(x + 0.5, H); ctx.stroke();
    }
    for (let y = 0; y <= H; y += UNIT) {
      ctx.strokeStyle = y % (UNIT * 5) === 0 ? '#e7dcc8' : '#f1eadc';
      ctx.beginPath(); ctx.moveTo(0, y + 0.5); ctx.lineTo(W, y + 0.5); ctx.stroke();
    }
  }

  function drawImage() {
    const im = state.image;
    if (!im) return;
    ctx.save();
    ctx.globalAlpha = im.opacity;
    ctx.drawImage(im.img, im.x, im.y, im.w, im.h);
    ctx.restore();
    if (state.mode === 'image') {
      ctx.save();
      ctx.strokeStyle = '#1f2a37';
      ctx.setLineDash([6, 4]);
      ctx.strokeRect(im.x + 0.5, im.y + 0.5, im.w, im.h);
      ctx.setLineDash([]);
      ctx.fillStyle = '#fff';
      ctx.beginPath();
      ctx.arc(im.x + im.w, im.y + im.h, 7, 0, Math.PI * 2);
      ctx.fill(); ctx.stroke();
      ctx.restore();
    }
  }

  function drawBar(b, highlight) {
    const a = b.p[0], c = b.p[1];
    const ang = Math.atan2(c.y - a.y, c.x - a.x);
    const L = b.n * UNIT;
    // In this frame a local point (t, s) sits at (t·L, s·L)
    ctx.save();
    ctx.translate(a.x, a.y);
    ctx.rotate(ang);
    const segment = (x0, y0, x1, y1) => {
      const len = Math.hypot(x1 - x0, y1 - y0);
      ctx.save();
      ctx.translate(x0, y0);
      ctx.rotate(Math.atan2(y1 - y0, x1 - x0));
      ctx.shadowColor = 'rgba(60,40,10,.28)';
      ctx.shadowBlur = 6;
      ctx.shadowOffsetY = 3;
      const g = ctx.createLinearGradient(0, -BAR_W / 2, 0, BAR_W / 2);
      g.addColorStop(0, shade(b.color, 0.35));
      g.addColorStop(0.45, b.color);
      g.addColorStop(1, shade(b.color, -0.25));
      ctx.fillStyle = g;
      roundRect(ctx, -BAR_CAP, -BAR_W / 2, len + BAR_CAP * 2, BAR_W, BAR_W / 2);
      ctx.fill();
      ctx.shadowColor = 'transparent';
      ctx.lineWidth = highlight ? 2 : 1;
      ctx.strokeStyle = highlight ? '#1f2a37' : shade(b.color, -0.4);
      ctx.stroke();
      ctx.restore();
    };
    for (const arm of b.arms) segment(arm.from.t * L, arm.from.s * L, arm.to.t * L, arm.to.s * L);
    segment(0, 0, L, 0);
    for (const o of b.holes) {
      const x = o.t * L, y = o.s * L;
      ctx.fillStyle = 'rgba(40,25,5,.55)';
      ctx.beginPath();
      ctx.arc(x, y, HOLE_R, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = 'rgba(255,255,255,.35)';
      ctx.beginPath();
      ctx.arc(x, y + 1.2, HOLE_R, 0.15 * Math.PI, 0.85 * Math.PI);
      ctx.fill();
    }
    ctx.restore();
  }

  function drawRivet(p) {
    const g = ctx.createRadialGradient(p.x - 2, p.y - 2, 1, p.x, p.y, 7);
    g.addColorStop(0, '#ffffff');
    g.addColorStop(0.4, '#c3c9d1');
    g.addColorStop(1, '#5c6672');
    ctx.fillStyle = g;
    ctx.beginPath();
    ctx.arc(p.x, p.y, 6.5, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#3d4650';
    ctx.lineWidth = 1;
    ctx.stroke();
  }

  function label(text, p, color, dy = -22, textColor = '#fff') {
    ctx.save();
    ctx.font = '600 10.5px ui-sans-serif, -apple-system, sans-serif';
    const w = ctx.measureText(text).width + 10;
    ctx.fillStyle = color;
    roundRect(ctx, p.x - w / 2, p.y + dy - 8, w, 16, 8);
    ctx.fill();
    ctx.fillStyle = textColor;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, p.x, p.y + dy);
    ctx.restore();
  }

  function drawPin(p) {
    ctx.save();
    ctx.shadowColor = 'rgba(0,0,0,.35)';
    ctx.shadowBlur = 6;
    ctx.shadowOffsetY = 3;
    const g = ctx.createRadialGradient(p.x - 3, p.y - 3, 1, p.x, p.y, 10);
    g.addColorStop(0, '#ff8787');
    g.addColorStop(1, '#a51111');
    ctx.fillStyle = g;
    ctx.beginPath();
    ctx.arc(p.x, p.y, 9.5, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
    label('PIN', p, '#a51111');
  }

  function drawTracer(p) {
    ctx.save();
    ctx.strokeStyle = TRACER_COLOR;
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.arc(p.x, p.y, 10, 0, Math.PI * 2);
    ctx.stroke();
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(p.x - 15, p.y); ctx.lineTo(p.x - 5, p.y);
    ctx.moveTo(p.x + 5, p.y); ctx.lineTo(p.x + 15, p.y);
    ctx.moveTo(p.x, p.y - 15); ctx.lineTo(p.x, p.y - 5);
    ctx.moveTo(p.x, p.y + 5); ctx.lineTo(p.x, p.y + 15);
    ctx.stroke();
    ctx.fillStyle = TRACER_COLOR;
    ctx.beginPath();
    ctx.arc(p.x, p.y, 2, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
    label('TRACER', p, TRACER_COLOR, -26);
  }

  function drawPen(pen) {
    const p = refPos(pen);
    if (!p) return;
    const light = pen.color === '#f2c200';
    ctx.save();
    ctx.shadowColor = 'rgba(0,0,0,.3)';
    ctx.shadowBlur = 5;
    ctx.shadowOffsetY = 2;
    ctx.fillStyle = '#fff';
    ctx.beginPath();
    ctx.arc(p.x, p.y, 9, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
    ctx.fillStyle = pen.color;
    ctx.beginPath();
    ctx.arc(p.x, p.y, 6, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = '#fff';
    ctx.beginPath();
    ctx.arc(p.x, p.y, 1.6, 0, Math.PI * 2);
    ctx.fill();
    label(`${colorName(pen.color).toUpperCase()} PEN`, p, pen.color, -24, light ? '#1f2a37' : '#fff');
  }

  function ring(p, color, r = 11, dashed = false) {
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    if (dashed) ctx.setLineDash([4, 3]);
    ctx.beginPath();
    ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
    ctx.stroke();
    ctx.restore();
  }

  function render() {
    ctx.setTransform(DPR, 0, 0, DPR, 0, 0);
    drawPaper();
    drawImage();
    ctx.drawImage(ink, 0, 0, W, H);

    const dragBarId = state.drag && state.drag.bar;
    for (const b of state.bars) drawBar(b, b.id === dragBarId && state.mode === 'build');

    const G = computeGroups();
    for (const members of G.groups.values()) {
      let x = 0, y = 0, n = 0;
      for (const m of members) { const p = refPos(m); if (p) { x += p.x; y += p.y; n++; } }
      if (n) drawRivet({ x: x / n, y: y / n });
    }

    if (state.hover && !state.drag) {
      const p = refPos(state.hover);
      if (p) ring(p, state.mode === 'remove' ? '#c92a2a' : '#1f2a37', 9);
    }

    // snapping preview
    if (state.drag && state.drag.kind === 'build') {
      state.snap = findSnap(state.drag);
      if (state.drag.hole) {
        const gp = refPos(state.drag.hole);
        ring(gp, '#1f2a37', 9, true);
        if (state.snap) {
          const sp = refPos(state.snap);
          ring(sp, '#2f9e44', 13);
          ring(sp, '#2f9e44', 7);
        }
      }
    }

    for (const p of state.pins) drawPin(p);
    const tp = refPos(state.tracer);
    for (const pen of state.pens) drawPen(pen);
    if (tp) drawTracer(tp);

    if (state.drag && state.drag.kind === 'trace' && state.drag.stuck && tp) {
      ctx.save();
      ctx.strokeStyle = 'rgba(201,42,42,.6)';
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(tp.x, tp.y);
      ctx.lineTo(state.mouse.x, state.mouse.y);
      ctx.stroke();
      ctx.restore();
    }
    updateHud(tp);
  }

  // ---------- HUD & side panel ----------
  let lastHud = '';
  // Range of scale/angle each pen showed during the current trace. A true copy
  // keeps both constant; if they drift, that pen's drawing is distorted.
  const penDrift = new Map();
  const resetDrift = () => penDrift.clear();

  function describeTurn(deg) {
    const a = Math.abs(deg);
    if (a < 2) return 'same way up';
    if (a > 178) return 'upside-down';
    return `turned ${Math.round(a)}° ${deg > 0 ? 'right' : 'left'}`;
  }

  function updateHud(tp) {
    let html = '';
    if (tp && state.pens.length && state.pins.length) {
      const o = state.pins[0];
      const tx = tp.x - o.x, ty = tp.y - o.y;
      const dt = Math.hypot(tx, ty);
      if (dt > 1) {
        html = state.pens.map((pen) => {
          const pp = refPos(pen);
          const px = pp.x - o.x, py = pp.y - o.y;
          const ratio = Math.hypot(px, py) / dt;
          // Angle from tracer direction to pen direction (positive = clockwise on screen)
          const deg = (Math.atan2(tx * py - ty * px, tx * px + ty * py) * 180) / Math.PI;
          let drift = penDrift.get(pen);
          if (state.drawing) {
            if (!drift) penDrift.set(pen, (drift = { r0: ratio, r1: ratio, ref: deg, a0: 0, a1: 0 }));
            // Measure angle change relative to the first reading, wrapped to ±180°
            // so an upside-down pen hovering around ±180° isn't seen as drifting.
            const da = ((deg - drift.ref + 540) % 360) - 180;
            drift.r0 = Math.min(drift.r0, ratio); drift.r1 = Math.max(drift.r1, ratio);
            drift.a0 = Math.min(drift.a0, da); drift.a1 = Math.max(drift.a1, da);
          }
          const distorts = drift && (drift.r1 - drift.r0 > 0.03 * drift.r1 || drift.a1 - drift.a0 > 3);
          return `<span class="dot" style="background:${pen.color}"></span>${colorName(pen.color)} pen: <b>${ratio.toFixed(2)}×</b>, ${describeTurn(deg)}` +
            (distorts ? ` · <span class="warn" title="Scale or angle changed while tracing">distorts</span>`
              : drift ? ` · <span class="ok" title="Scale and angle stayed constant while tracing">true copy</span>` : '');
        }).join('<br>');
      }
    } else if (state.bars.length) {
      html = `Mobility: <b>${mobility()}</b> &nbsp;·&nbsp; ${state.bars.length} bars, ${state.links.length} joints`;
    }
    if (html !== lastHud) { $('hud').innerHTML = html; lastHud = html; }
  }

  function mobility() {
    // Planar Grübler count: 3 per bar, minus 2 per pivot link, minus 2 per pin
    return 3 * state.bars.length - 2 * state.links.length - 2 * state.pins.length;
  }

  function changed() {
    const list = $('checklist');
    const mob = mobility();
    const items = [
      [state.bars.length >= 4, 'Put bars on the table', `${state.bars.length} placed — a pantograph uses 4`],
      [state.links.length >= 4, 'Join holes into pivots', `${state.links.length} joints — a pantograph uses 4`],
      [state.pins.length > 0, 'Pin one end to the table', state.pins.length ? `${state.pins.length} pinned` : 'Pin tool → click a hole'],
      [!!state.tracer, 'Place the tracer', 'Tracer tool → click a hole'],
      [state.pens.length > 0, 'Place one or more pens',
        state.pens.length ? `${state.pens.length} of ${PEN_COLORS.length} pens placed` : 'Pen tool → click a hole'],
      [mob === 2 ? true : state.bars.length ? 'warn' : false, 'Mobility of 2',
        mob === 2 ? 'Free to follow any traced point' : mob > 2 ? `Mobility ${mob}: too floppy — add joints` : `Mobility ${mob}: too stiff to move freely`],
    ];
    list.innerHTML = items.map(([ok, t, s]) =>
      `<li class="${ok === true ? 'done' : ok === 'warn' ? 'warn' : ''}"><span class="mark">${ok === true ? '✓' : ok === 'warn' ? '!' : ''}</span><span>${t}<small>${s}</small></span></li>`
    ).join('');
    if (state.bars.length) hideEmpty();
    renderPenList();
  }

  const HINTS = {
    build: "Drag bars to move them · drop a hole onto another bar's hole to join · scroll over a bar to rotate",
    pin: 'Click a hole to pin it to the table (click again to unpin) · Build tool drags pins',
    tracer: 'Click a hole to make it the tracing stylus',
    pen: 'Click a hole to add a pen (up to 5, each a new colour) · click a pen again to lift it off',
    remove: 'Click a pin/tracer/pen to remove it, a joint to separate it, or a bar to delete it',
    image: 'Drag the image to move it · drag the corner handle to resize',
    draw: 'Press and drag to trace — every pen draws · hold Shift to move without drawing',
  };

  function setMode(mode) {
    state.mode = mode;
    for (const b of document.querySelectorAll('#tools button')) b.classList.toggle('active', b.dataset.mode === mode);
    $('hint').textContent = HINTS[mode];
    updateCursor(state.mouse);
  }

  let toastTimer = 0;
  function toast(msg) {
    const t = $('toast');
    t.textContent = msg;
    t.classList.add('show');
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => t.classList.remove('show'), 2600);
  }

  function hideEmpty() { $('empty').classList.add('hidden'); }

  // ---------- Image ----------
  function loadImageFile(file) {
    if (!file || !file.type.startsWith('image/')) { toast('Please choose an image file.'); return; }
    const reader = new FileReader();
    reader.onload = () => {
      const img = new Image();
      img.onload = () => {
        const maxW = W * 0.38, maxH = H * 0.5;
        const s = Math.min(maxW / img.width, maxH / img.height, 1);
        const w = img.width * s, h = img.height * s;
        const tp = refPos(state.tracer);
        const c = tp || { x: W / 2, y: H / 2 };
        state.image = {
          img, w, h,
          x: Math.max(0, Math.min(W - w, c.x - w / 2)),
          y: Math.max(0, Math.min(H - h, c.y - h / 2)),
          opacity: parseFloat($('imgOpacity').value),
        };
        toast('Image placed — use the Image tool to move or resize it.');
      };
      img.src = reader.result;
    };
    reader.readAsDataURL(file);
  }

  $('imageInput').addEventListener('change', (e) => {
    loadImageFile(e.target.files[0]);
    e.target.value = '';
  });
  $('imgOpacity').addEventListener('input', (e) => { if (state.image) state.image.opacity = parseFloat(e.target.value); });
  $('imgRemove').addEventListener('click', () => { state.image = null; });

  const stage = $('stage');
  stage.addEventListener('dragover', (e) => { e.preventDefault(); });
  stage.addEventListener('drop', (e) => {
    e.preventDefault();
    const f = e.dataTransfer.files && e.dataTransfer.files[0];
    if (f) loadImageFile(f);
  });

  // ---------- Side panel wiring ----------
  for (const b of document.querySelectorAll('#tools button')) b.addEventListener('click', () => setMode(b.dataset.mode));

  const tray = $('tray');
  for (const n of TRAY) {
    const btn = document.createElement('button');
    const pv = document.createElement('canvas');
    const pw = 150, ph = 18, pr = window.devicePixelRatio || 1;
    pv.width = pw * pr; pv.height = ph * pr; pv.style.width = pw + 'px'; pv.style.height = ph + 'px';
    const c = pv.getContext('2d');
    c.scale(pr, pr);
    const sp = (pw - 12) / 16;
    const len = n * sp;
    c.fillStyle = '#c9a36b';
    roundRect(c, 2, 3, len + 8, 12, 6);
    c.fill();
    c.fillStyle = 'rgba(40,25,5,.55)';
    for (let h = 0; h <= n; h++) { c.beginPath(); c.arc(6 + h * sp, 9, 1.6, 0, Math.PI * 2); c.fill(); }
    btn.appendChild(pv);
    const s = document.createElement('span');
    s.className = 'len';
    s.textContent = `${n} spaces`;
    btn.appendChild(s);
    btn.addEventListener('click', () => addBarFromTray(n));
    tray.appendChild(btn);
  }

  $('presetBtn').addEventListener('click', () => buildPreset($('presetScale').value));
  $('emptyPreset').addEventListener('click', () => buildPreset(0.5));
  $('emptyDismiss').addEventListener('click', hideEmpty);

  // Pen list: one row per pen, with a colour picker (only unused colours) and a remove button
  function renderPenList() {
    const box = $('penList');
    box.innerHTML = '';
    if (!state.pens.length) {
      box.innerHTML = '<p class="sub">No pens yet — use the Pen tool and click holes to add up to 5.</p>';
      return;
    }
    state.pens.forEach((pen, i) => {
      const row = document.createElement('div');
      row.className = 'pen-row';
      const used = new Set(state.pens.filter((p) => p !== pen).map((p) => p.color));
      const swatches = PEN_COLORS.map(({ name, c }) =>
        `<button class="sw${c === pen.color ? ' active' : ''}" data-c="${c}" title="${name}" style="background:${c}"${used.has(c) ? ' disabled' : ''}></button>`
      ).join('');
      row.innerHTML = `<span class="pen-name">Pen ${i + 1}</span><span class="sws">${swatches}</span>` +
        `<button class="x" title="Remove this pen">✕</button>`;
      row.querySelector('.x').addEventListener('click', () => {
        state.pens = state.pens.filter((p) => p !== pen);
        changed();
      });
      for (const b of row.querySelectorAll('.sw')) {
        b.addEventListener('click', () => { pen.color = b.dataset.c; changed(); });
      }
      box.appendChild(row);
    });
  }
  $('penWidth').addEventListener('input', (e) => { state.penWidth = parseFloat(e.target.value); });
  $('showGuides').addEventListener('change', (e) => { state.showGuides = e.target.checked; rerenderInk(); });

  function undo() {
    if (!state.strokes.length) return;
    // Remove every pen's stroke from the last tracing gesture
    const g = state.strokes[state.strokes.length - 1].group;
    while (state.strokes.length && state.strokes[state.strokes.length - 1].group === g) state.strokes.pop();
    rerenderInk();
  }
  $('undoBtn').addEventListener('click', undo);
  $('clearInk').addEventListener('click', () => { state.strokes = []; rerenderInk(); });
  $('resetAll').addEventListener('click', () => {
    if (!confirm('Remove all bars, the image, and the drawing?')) return;
    clearMechanism();
    state.strokes = [];
    state.image = null;
    rerenderInk();
    changed();
    $('empty').classList.remove('hidden');
    setMode('build');
  });

  $('savePng').addEventListener('click', () => {
    if (!state.strokes.length) { toast('Nothing drawn yet.'); return; }
    const out = document.createElement('canvas');
    out.width = W * DPR; out.height = H * DPR;
    const c = out.getContext('2d');
    c.fillStyle = '#ffffff';
    c.fillRect(0, 0, out.width, out.height);
    c.setTransform(DPR, 0, 0, DPR, 0, 0);
    paintStrokes(c, false);
    const a = document.createElement('a');
    a.download = 'pantograph-drawing.png';
    a.href = out.toDataURL('image/png');
    a.click();
  });

  // ---------- Keyboard ----------
  const KEYS = { v: 'build', p: 'pin', t: 'tracer', n: 'pen', x: 'remove', i: 'image', d: 'draw' };
  window.addEventListener('keydown', (e) => {
    if (e.key === 'Shift') state.shift = true;
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
    if ((e.metaKey || e.ctrlKey) && e.key.toLowerCase() === 'z') { e.preventDefault(); undo(); return; }
    if (e.metaKey || e.ctrlKey || e.altKey) return;
    const m = KEYS[e.key.toLowerCase()];
    if (m) setMode(m);
  });
  window.addEventListener('keyup', (e) => { if (e.key === 'Shift') state.shift = false; });
  window.addEventListener('blur', () => { state.shift = false; });

  // ---------- Layout & loop ----------
  function resize() {
    const r = stage.getBoundingClientRect();
    DPR = window.devicePixelRatio || 1;
    W = Math.max(1, Math.round(r.width));
    H = Math.max(1, Math.round(r.height));
    canvas.width = W * DPR; canvas.height = H * DPR;
    ink.width = W * DPR; ink.height = H * DPR;
    rerenderInk();
  }
  window.addEventListener('resize', resize);

  function frame() {
    if (state.drag) stepDrag();
    else if (state.bars.length) solve(6, null);
    render();
    requestAnimationFrame(frame);
  }

  resize();
  setMode('build');
  changed();
  requestAnimationFrame(frame);

  // Exposed for debugging in the console
  window.pantograph = { state, solve, buildPreset };
})();
