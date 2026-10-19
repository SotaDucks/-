import { SEGS, BODY_PROPS as PROPS, N, qrot, qconj } from './fish-sim.mjs';

// v2: clock, spawn generation, acknowledged impact; x/q/linear/angular velocity per segment.
export const HEADER = 3, STRIDE = 13;
export const POSE_LEN = HEADER + N * STRIDE;
export const LEGACY_POSE_LEN = 2 + N * 7;
const add = (a, b) => a.map((v, k) => v + b[k]);
const sub = (a, b) => a.map((v, k) => v - b[k]);
const scale = (a, s) => a.map(v => v * s);
const dot = (a, b) => a.reduce((s, v, k) => s + v * b[k], 0);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const length = a => Math.hypot(...a);
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const vec3 = a => Array.isArray(a) && a.length === 3 && a.every(Number.isFinite);
const multiply = (m, v) => m.map(row => dot(row, v));

export function packFish(fish, time, epoch, ack = 0) {
  const a = new Float64Array(POSE_LEN);
  a[0] = time; a[1] = epoch; a[2] = ack;
  fish.bodies.forEach((b, i) => {
    const o = HEADER + i * STRIDE;
    a.set(b.x, o); a.set(b.q, o + 3); a.set(b.v, o + 7); a.set(b.w, o + 10);
  });
  return a;
}

export function unpackFish(data) {
  if (!(data instanceof ArrayBuffer) || data.byteLength !== POSE_LEN * 8) return null;
  const a = new Float64Array(data);
  if (!a.every(Number.isFinite) || a[0] < 0 || !Number.isSafeInteger(a[1]) || a[1] < 0 || !Number.isSafeInteger(a[2]) || a[2] < 0) return null;
  const bodies = [];
  for (let i = 0; i < N; i++) {
    const o = HEADER + i * STRIDE, x = Array.from(a.slice(o, o + 3)), q = Array.from(a.slice(o + 3, o + 7));
    const v = Array.from(a.slice(o + 7, o + 10)), w = Array.from(a.slice(o + 10, o + 13));
    const norm = length(q);
    if (x.some(n => Math.abs(n) > 10000) || length(v) > 100 || length(w) > 500 || norm < 0.5 || norm > 1.5) return null;
    bodies.push({ x, q: scale(q, 1 / norm), v, w });
  }
  // Reject broken chains instead of allowing a malicious pose to hit across the room.
  if (bodies.some(b => length(sub(b.x, bodies[3].x)) > 3)) return null;
  return { bodies, time: a[0], epoch: a[1], ack: a[2] };
}

// Treat the articulated fish as one body for external impacts. Distributing the same
// rigid velocity change across its segments avoids tearing the joint chain apart.
function assembly(bodies) {
  const mass = PROPS.reduce((m, p) => m + p.m, 0);
  const center = [0, 0, 0], velocity = [0, 0, 0];
  bodies.forEach((b, i) => {
    for (let k = 0; k < 3; k++) { center[k] += b.x[k] * PROPS[i].m / mass; velocity[k] += b.v[k] * PROPS[i].m / mass; }
  });
  const tensor = Array.from({ length: 3 }, () => [0, 0, 0]);
  bodies.forEach((b, i) => {
    const r = sub(b.x, center), rr = dot(r, r), p = PROPS[i];
    const axes = [[1, 0, 0], [0, 1, 0], [0, 0, 1]].map(a => qrot(b.q, a));
    for (let j = 0; j < 3; j++) for (let k = 0; k < 3; k++) {
      tensor[j][k] += p.m * ((j === k ? rr : 0) - r[j] * r[k]);
      for (let d = 0; d < 3; d++) tensor[j][k] += p.inertia[d] * axes[d][j] * axes[d][k];
    }
  });
  const [[a, b, c], [, d, e], [, , f]] = tensor;
  const det = Math.max(1e-12, a * (d * f - e * e) - b * (b * f - c * e) + c * (b * e - c * d));
  const inv = [[d * f - e * e, c * e - b * f, b * e - c * d], [c * e - b * f, a * f - c * c, b * c - a * e], [b * e - c * d, b * c - a * e, a * d - b * b]].map(row => scale(row, 1 / det));
  return { mass, center, velocity, inv };
}

function capsule(b, i, offset) {
  const g = SEGS[i], cap = Math.min(g.ay, g.az), half = Math.max(0, g.L / 2 - cap * 0.5);
  const x = add(b.x, scale(b.v, offset)), axis = qrot(b.q, [half, 0, 0]);
  return { a: sub(x, axis), b: add(x, axis), x, q: b.q, radii: [cap, g.cay || g.ay, g.az] };
}

// Closest points on two finite center lines, including parallel and zero-length lines.
function closest(a, b) {
  const d1 = sub(a.b, a.a), d2 = sub(b.b, b.a), r = sub(a.a, b.a);
  const aa = dot(d1, d1), ee = dot(d2, d2), ff = dot(d2, r);
  let s = 0, t = 0;
  if (aa < 1e-12 && ee < 1e-12) return [a.a, b.a];
  if (aa < 1e-12) t = clamp(ff / ee, 0, 1);
  else {
    const cc = dot(d1, r);
    if (ee < 1e-12) s = clamp(-cc / aa, 0, 1);
    else {
      const bb = dot(d1, d2), den = aa * ee - bb * bb;
      if (den > 1e-12) s = clamp((bb * ff - cc * ee) / den, 0, 1);
      t = (bb * s + ff) / ee;
      if (t < 0) { t = 0; s = clamp(-cc / aa, 0, 1); }
      else if (t > 1) { t = 1; s = clamp((bb - cc) / aa, 0, 1); }
    }
  }
  return [add(a.a, scale(d1, s)), add(b.a, scale(d2, t))];
}

function radius(shape, n) {
  const local = qrot(qconj(shape.q), n);
  return Math.hypot(...local.map((v, k) => v * shape.radii[k]));
}

function contact(A, B, offset) {
  const ca = A.bodies.map((b, i) => capsule(b, i, offset));
  const cb = B.bodies.map((b, i) => capsule(b, i, offset));
  let best = null;
  for (let i = 0; i < N; i++) for (let j = 0; j < N; j++) {
    const a = ca[i], b = cb[j];
    if (length(sub(a.x, b.x)) > 0.9) continue;
    const [pa, pb] = closest(a, b), delta = sub(pa, pb), dist = length(delta);
    let normal;
    if (dist > 1e-7) normal = scale(delta, 1 / dist);
    else {
      const fallback = sub(A.bodies[3].x, B.bodies[3].x);
      normal = length(fallback) > 1e-7 ? scale(fallback, 1 / length(fallback)) : [0, 0, 1];
    }
    const ra = radius(a, normal), rb = radius(b, normal), depth = ra + rb - dist;
    if (depth <= 0 || (best && depth <= best.depth)) continue;
    const p = scale(add(sub(pa, scale(normal, ra)), add(pb, scale(normal, rb))), 0.5);
    best = { i, j, normal, depth, point: p, a, b };
  }
  return best;
}

export function findImpact(A, B, sweep = 0) {
  const sa = assembly(A.bodies), sb = assembly(B.bodies);
  const relativeSpeed = length(sub(sa.velocity, sb.velocity));
  if (length(sub(sa.center, sb.center)) > 2.8 + relativeSpeed * sweep) return null;
  let hit = contact(A, B, 0), swept = false;
  if (!hit && sweep > 0) {
    const steps = clamp(Math.ceil(relativeSpeed * sweep / 0.04), 1, 10);
    for (let s = steps; s >= 1 && !hit; s--) hit = contact(A, B, -sweep * s / steps);
    swept = !!hit;
  }
  if (!hit) return null;
  const { i, j, normal: n, point } = hit;
  const ra = sub(point, sa.center), rb = sub(point, sb.center);
  const va = add(A.bodies[i].v, cross(A.bodies[i].w, sub(point, hit.a.x)));
  const vb = add(B.bodies[j].v, cross(B.bodies[j].w, sub(point, hit.b.x)));
  const closing = dot(sub(va, vb), n);
  const armA = cross(ra, n), armB = cross(rb, n);
  const effective = 1 / sa.mass + 1 / sb.mass + dot(armA, multiply(sa.inv, armA)) + dot(armB, multiply(sb.inv, armB));
  const impulseSize = closing < -0.03 ? Math.min(-1.25 * closing / effective, Math.min(sa.mass, sb.mass) * 5) : 0;
  const separation = swept ? 0 : Math.min(0.16, Math.max(0, hit.depth - 0.002) * 0.65);
  if (impulseSize === 0 && separation < 0.0005) return null;
  const impulse = scale(n, impulseSize), correction = scale(n, separation / 2);
  return {
    a: { epoch: A.epoch, index: i, local: qrot(qconj(hit.a.q), sub(point, hit.a.x)), impulse, correction },
    b: { epoch: B.epoch, index: j, local: qrot(qconj(hit.b.q), sub(point, hit.b.x)), impulse: scale(impulse, -1), correction: scale(correction, -1) },
  };
}

export function applyImpact(fish, impact, epoch) {
  if (!impact || impact.epoch !== epoch || !Number.isInteger(impact.index) || impact.index < 0 || impact.index >= N ||
      !vec3(impact.local) || length(impact.local) > 1 || !vec3(impact.impulse) || !vec3(impact.correction) || length(impact.correction) > 0.2) return false;
  const s = assembly(fish.bodies), b = fish.bodies[impact.index];
  if (length(impact.impulse) > s.mass * 5 + 0.001) return false;
  const point = add(b.x, qrot(b.q, impact.local));
  const dv = scale(impact.impulse, 1 / s.mass);
  let dw = multiply(s.inv, cross(sub(point, s.center), impact.impulse));
  if (length(dw) > 8) dw = scale(dw, 8 / length(dw));
  for (const body of fish.bodies) {
    const change = add(dv, cross(dw, sub(body.x, s.center)));
    for (let k = 0; k < 3; k++) {
      body.v[k] += change[k]; body.w[k] += dw[k];
      body.x[k] += impact.correction[k]; body.px[k] += impact.correction[k];
    }
  }
  // An interrupted charge must not overwrite the hit with a precomputed launch.
  if (length(dv) > 0.12 && ['rest', 'land', 'charge', 'slap'].includes(fish.state)) {
    fish.hop = null; fish.traj = null; fish.enterState('drop');
  }
  return true;
}

export function resolveFishPair(a, b) {
  const impact = findImpact({ bodies: a.bodies, epoch: 0 }, { bodies: b.bodies, epoch: 0 });
  if (!impact) return false;
  applyImpact(a, impact.a, 0); applyImpact(b, impact.b, 0);
  return true;
}
