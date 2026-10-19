// Shared fish physics; used by the browser and headless collision tests.
const G = 9.81;

// ---------- small math ----------
const vadd = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const vsub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const vsc = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const vdot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const vcross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const vlen = (a) => Math.hypot(a[0], a[1], a[2]);
const vnorm = (a) => { const l = vlen(a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };
const clamp = (x, a, b) => Math.min(b, Math.max(a, x));
const smooth = (x) => { x = clamp(x, 0, 1); return x * x * (3 - 2 * x); };

function qmul(a, b) {
  return [
    a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
    a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
    a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
    a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
  ];
}
const qconj = (q) => [-q[0], -q[1], -q[2], q[3]];
function qrot(q, v) {
  const ux = q[0], uy = q[1], uz = q[2], w = q[3];
  const tx = 2 * (uy * v[2] - uz * v[1]), ty = 2 * (uz * v[0] - ux * v[2]), tz = 2 * (ux * v[1] - uy * v[0]);
  return [v[0] + w * tx + uy * tz - uz * ty, v[1] + w * ty + uz * tx - ux * tz, v[2] + w * tz + ux * ty - uy * tx];
}
const qaxis = (ax, ang) => { const s = Math.sin(ang / 2); return [ax[0] * s, ax[1] * s, ax[2] * s, Math.cos(ang / 2)]; };
function qaddRot(q, r) { // q += 0.5 * [r,0] * q  (small rotation vector r)
  const d = qmul([r[0], r[1], r[2], 0], q);
  q[0] += 0.5 * d[0]; q[1] += 0.5 * d[1]; q[2] += 0.5 * d[2]; q[3] += 0.5 * d[3];
  const n = Math.hypot(q[0], q[1], q[2], q[3]);
  q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n;
}
function qFromBasis(x, y, z) { // columns of rotation matrix
  const m00 = x[0], m10 = x[1], m20 = x[2], m01 = y[0], m11 = y[1], m21 = y[2], m02 = z[0], m12 = z[1], m22 = z[2];
  const tr = m00 + m11 + m22; let qx, qy, qz, qw;
  if (tr > 0) { const s = 0.5 / Math.sqrt(tr + 1); qw = 0.25 / s; qx = (m21 - m12) * s; qy = (m02 - m20) * s; qz = (m10 - m01) * s; }
  else if (m00 > m11 && m00 > m22) { const s = 2 * Math.sqrt(1 + m00 - m11 - m22); qw = (m21 - m12) / s; qx = 0.25 * s; qy = (m01 + m10) / s; qz = (m02 + m20) / s; }
  else if (m11 > m22) { const s = 2 * Math.sqrt(1 + m11 - m00 - m22); qw = (m02 - m20) / s; qx = (m01 + m10) / s; qy = 0.25 * s; qz = (m12 + m21) / s; }
  else { const s = 2 * Math.sqrt(1 + m22 - m00 - m11); qw = (m10 - m01) / s; qx = (m02 + m20) / s; qy = (m12 + m21) / s; qz = 0.25 * s; }
  const n = Math.hypot(qx, qy, qz, qw); return [qx / n, qy / n, qz / n, qw / n];
}

// ---------- rigid body ----------
class Body {
  constructor(m, I) {
    this.m = m; this.invM = 1 / m; this.invI = [1 / I[0], 1 / I[1], 1 / I[2]];
    this.x = [0, 0, 0]; this.q = [0, 0, 0, 1]; this.v = [0, 0, 0]; this.w = [0, 0, 0];
    this.px = [0, 0, 0]; this.pq = [0, 0, 0, 1];
  }
  invIw(u) { const b = qrot(qconj(this.q), u); return qrot(this.q, [b[0] * this.invI[0], b[1] * this.invI[1], b[2] * this.invI[2]]); }
  angInvMass(n) { const b = qrot(qconj(this.q), n); return b[0] * b[0] * this.invI[0] + b[1] * b[1] * this.invI[1] + b[2] * b[2] * this.invI[2]; }
  genInvMass(r, n) { return this.invM + this.angInvMass(vcross(r, n)); }
  applyPos(p, r, sign) {
    this.x[0] += sign * p[0] * this.invM; this.x[1] += sign * p[1] * this.invM; this.x[2] += sign * p[2] * this.invM;
    qaddRot(this.q, vsc(this.invIw(vcross(r, p)), sign));
  }
  applyRot(p, sign) { qaddRot(this.q, vsc(this.invIw(p), sign)); }
  applyVelImpulse(p, r, sign) {
    for (let k = 0; k < 3; k++) this.v[k] += sign * p[k] * this.invM;
    const dw = this.invIw(vcross(r, p)); for (let k = 0; k < 3; k++) this.w[k] += sign * dw[k];
  }
}

// XPBD positional correction: moves point p1 by a share of `corr`, point p2 by the opposite share.
function posCorr(b1, r1, b2, r2, corr, alphaT) {
  const c = vlen(corr); if (c < 1e-10) return 0;
  const n = vsc(corr, 1 / c);
  const w1 = b1 ? b1.genInvMass(r1, n) : 0, w2 = b2 ? b2.genInvMass(r2, n) : 0;
  const dl = c / (w1 + w2 + alphaT); const p = vsc(n, dl);
  if (b1) b1.applyPos(p, r1, +1); if (b2) b2.applyPos(p, r2, -1);
  return dl;
}
// XPBD angular correction: body2 rotates by +share of dphi, body1 by the opposite share.
function angCorr(b1, b2, dphi, alphaT) {
  const c = vlen(dphi); if (c < 1e-10) return 0;
  const n = vsc(dphi, 1 / c);
  const dl = c / (b1.angInvMass(n) + b2.angInvMass(n) + alphaT); const p = vsc(n, dl);
  b1.applyRot(p, -1); b2.applyRot(p, +1);
  return dl;
}

// ---------- world ----------
class World {
  constructor(boxes) { this.boxes = boxes || []; }
  height(x, z) {
    let h = 0;
    for (const b of this.boxes) if (Math.abs(x - b.cx) <= b.hx && Math.abs(z - b.cz) <= b.hz && b.h > h) h = b.h;
    return h;
  }
  boxAt(x, z) {
    let best = null;
    for (const b of this.boxes) if (Math.abs(x - b.cx) <= b.hx && Math.abs(z - b.cz) <= b.hz && (!best || b.h > best.h)) best = b;
    return best;
  }
}

// ---------- fish definition ----------
// Segments head -> tail. L: length, ay: dorsal half-height, az: lateral half-width (fish is laterally flat).
const SEGS = [
  { L: 0.52, ay: 0.016, az: 0.032, name: 'bill' },
  { L: 0.26, ay: 0.12, az: 0.075 },
  { L: 0.22, ay: 0.165, az: 0.09 },
  { L: 0.22, ay: 0.17, az: 0.092 },
  { L: 0.20, ay: 0.15, az: 0.082 },
  { L: 0.18, ay: 0.12, az: 0.066 },
  { L: 0.16, ay: 0.088, az: 0.05 },
  { L: 0.14, ay: 0.06, az: 0.036 },
  { L: 0.12, ay: 0.036, az: 0.026 },
  { L: 0.10, ay: 0.03, az: 0.022, cay: 0.26, name: 'tail' },
];
const N = SEGS.length;
const BODY_PROPS = SEGS.map((g, i) => {
  const m = Math.max(0.5, Math.PI * g.ay * g.az * g.L * 760 + (i === N - 1 ? 0.5 : 0));
  const ay = g.cay || g.ay;
  return { m, inertia: [m * (ay * ay + g.az * g.az) / 4, m * (g.az * g.az / 4 + g.L * g.L / 12), m * (ay * ay / 4 + g.L * g.L / 12)] };
});
const TOTAL = SEGS.reduce((s, g) => s + g.L, 0);
// Arc-length (from bill tip) of each segment's center, and joint boundaries.
const CENTERS = [], BOUNDS = [0];
{ let u = 0; for (const g of SEGS) { CENTERS.push(u + g.L / 2); u += g.L; BOUNDS.push(u); } }
// How strongly each body joint participates in the C-curl (joint i links seg i and i+1).
const CURL_PROFILE = [0, 0.55, 0.8, 1.0, 1.0, 1.0, 0.95, 0.85, 0.7];
const PROF_SUM = CURL_PROFILE.reduce((a, b) => a + b, 0);

// Tunable feel parameters (multipliers are 1 = original behaviour).
const TUNE_DEFAULTS = {
  soft: 1,      // muscle compliance multiplier: higher = softer, floppier muscles
  damp: 1,      // joint damping multiplier: higher = less wobble between segments
  landLimp: 1,  // extra muscle slack right after touchdown: higher = body goes limp on landing
  absorb: 7,    // landing energy absorption rate (1/s): higher = slides/spins stop sooner
  bounce: 0,    // share of rebound velocity soaked up at contacts: 1 = no bounce at all
  friction: 1,  // ground friction multiplier
  curl: 1,      // charge curl amplitude multiplier
  arch: 1,      // in-flight body arch multiplier
  idle: 1,      // resting tail-twitch amplitude multiplier
};
const STATE_LABEL = { drop: '下落', rest: '休息', charge: '蓄力', slap: '拍尾', air: '腾空', land: '落地' };

class Fish {
  constructor(world) {
    this.world = world;
    this.bodies = BODY_PROPS.map(p => new Body(p.m, p.inertia));
    this.mass = this.bodies.reduce((s, b) => s + b.m, 0);
    // contact sample positions along each segment's local x
    this.samples = SEGS.map((g, i) => i === 0 ? [0, 0.5 * g.L] : i === N - 1 ? [-0.5 * g.L, 0] : [-0.45 * g.L, 0, 0.45 * g.L]);
    this.theta = new Array(N - 1).fill(0);   // muscle target angles (rad), about each segment's dorsal axis
    this.alpha = new Array(N - 1).fill(4e-5); // muscle compliance (lower = stronger)
    this.kd = 4;                              // joint damping
    this.mu = 0.55;                           // ground friction
    this.substeps = 20;
    this.tune = Object.assign({}, TUNE_DEFAULTS);
    this.contacts = [];
    this.touch = new Array(N).fill(false);
    this.queue = [];
    this.energy = 1;
    this.state = 'rest'; this.stateT = 0; this.t = 0;
    this.hop = null; this.traj = null; this.lastJump = null;
    this.events = [];
    this.reset([0, 0, 0], 0, 1);
    // Measure the resting center-of-mass height once, used by the ballistic solver as landing height.
    for (let i = 0; i < 90; i++) this.physics(1 / 60);
    this.restH = this.com()[1];
    this.reset([0, 0, 0], 0, 1);
  }

  reset(pos, yaw, side) {
    side = side || 1;
    const hx = [Math.cos(yaw), 0, -Math.sin(yaw)];       // heading (local +x)
    const z = [0, side, 0];                              // lateral axis points up: lying on its side
    const y = vcross(z, hx);
    const q = qFromBasis(hx, y, z);
    // place COM (approximately the body middle) at pos
    const mid = TOTAL * 0.48;
    this.bodies.forEach((b, i) => {
      const off = mid - CENTERS[i];
      b.x = [pos[0] + hx[0] * off, Math.max(SEGS[i].az, 0.03) + 0.005 + this.world.height(pos[0], pos[2]), pos[2] + hx[2] * off];
      b.q = q.slice(); b.v = [0, 0, 0]; b.w = [0, 0, 0];
      b.px = b.x.slice(); b.pq = b.q.slice();
    });
    this.theta.fill(0); this.alpha.fill(4e-5);
    this.state = 'rest'; this.stateT = 0; this.hop = null; this.traj = null;
  }

  // Spawn in the sky: same lying pose, lifted by `height`, with a slow tumble so it lands a bit askew.
  drop(pos, yaw, height) {
    this.reset(pos, yaw, Math.random() < 0.5 ? 1 : -1);
    this.energy = 1; this.lastJump = null; this.queue.length = 0;
    const h = this.heading(), lat = vcross(h, [0, 1, 0]);
    const w = vadd(vsc(h, (Math.random() - 0.5) * 2.4), vsc(lat, (Math.random() - 0.5) * 1.2));
    for (const b of this.bodies) b.x[1] += height;
    const c = this.com();
    for (const b of this.bodies) { b.px = b.x.slice(); b.v = vcross(w, vsub(b.x, c)); b.w = w.slice(); }
    this.enterState('drop');
  }

  com() {
    const c = [0, 0, 0];
    for (const b of this.bodies) { c[0] += b.x[0] * b.m; c[1] += b.x[1] * b.m; c[2] += b.x[2] * b.m; }
    return vsc(c, 1 / this.mass);
  }
  comVel() {
    const c = [0, 0, 0];
    for (const b of this.bodies) { c[0] += b.v[0] * b.m; c[1] += b.v[1] * b.m; c[2] += b.v[2] * b.m; }
    return vsc(c, 1 / this.mass);
  }
  angMom() { // about COM
    const c = this.com(); let L = [0, 0, 0];
    for (const b of this.bodies) {
      L = vadd(L, vsc(vcross(vsub(b.x, c), b.v), b.m));
      const wb = qrot(qconj(b.q), b.w); const Iw = [wb[0] / b.invI[0], wb[1] / b.invI[1], wb[2] / b.invI[2]];
      L = vadd(L, qrot(b.q, Iw));
    }
    return L;
  }
  side() { const z = qrot(this.bodies[3].q, [0, 0, 1]); return z[1] >= 0 ? 1 : -1; }
  heading() {
    const a = this.bodies[1].x, b = this.bodies[8].x;
    const d = [a[0] - b[0], 0, a[2] - b[2]]; const l = Math.hypot(d[0], d[2]);
    return l < 1e-4 ? vnorm(qrot(this.bodies[3].q, [1, 0, 0]).map((v, k) => k === 1 ? 0 : v)) : [d[0] / l, 0, d[2] / l];
  }
  static yawBetween(h, d) { return Math.atan2(h[2] * d[0] - h[0] * d[2], h[0] * d[0] + h[2] * d[2]); }

  // ---------- physics ----------
  physics(dt) {
    this.touch.fill(false);
    const h = dt / this.substeps;
    for (let s = 0; s < this.substeps; s++) this.substep(h);
  }

  substep(h) {
    const B = this.bodies;
    // 1) integrate
    for (const b of B) {
      b.v[1] -= G * h;
      b.px[0] = b.x[0]; b.px[1] = b.x[1]; b.px[2] = b.x[2];
      b.pq[0] = b.q[0]; b.pq[1] = b.q[1]; b.pq[2] = b.q[2]; b.pq[3] = b.q[3];
      b.x[0] += b.v[0] * h; b.x[1] += b.v[1] * h; b.x[2] += b.v[2] * h;
      qaddRot(b.q, vsc(b.w, h));
    }
    // 2) joints (ball joints, rigid) and muscles (angular springs with target)
    const ih2 = 1 / (h * h);
    for (let i = 0; i < N - 1; i++) {
      const b1 = B[i], b2 = B[i + 1];
      const r1 = qrot(b1.q, [-SEGS[i].L / 2, 0, 0]), r2 = qrot(b2.q, [SEGS[i + 1].L / 2, 0, 0]);
      posCorr(b1, r1, b2, r2, vsub(vadd(b2.x, r2), vadd(b1.x, r1)), 0);
    }
    for (let i = 0; i < N - 1; i++) {
      const b1 = B[i], b2 = B[i + 1];
      const qt = qmul(b1.q, qaxis([0, 1, 0], this.theta[i]));
      let dq = qmul(qt, qconj(b2.q)); if (dq[3] < 0) dq = dq.map((c) => -c);
      const s = Math.hypot(dq[0], dq[1], dq[2]); if (s < 1e-9) continue;
      const ang = 2 * Math.atan2(s, dq[3]);
      angCorr(b1, b2, [dq[0] / s * ang, dq[1] / s * ang, dq[2] / s * ang], this.alpha[i] * ih2);
    }
    // 3) contacts with ground / boxes (lowest point of each elliptic cross-section)
    this.contacts.length = 0;
    // right after take-off the launch impulse already accounts for the ground push; skip contacts briefly
    const skipContacts = this.noContacts || (this.state === 'air' && this.stateT < 0.16);
    for (let i = 0; i < N && !skipContacts; i++) {
      const b = B[i], g = SEGS[i], ay = g.cay || g.ay, az = g.az;
      const down = qrot(qconj(b.q), [0, -1, 0]);
      const den = Math.sqrt(ay * ay * down[1] * down[1] + az * az * down[2] * down[2]);
      const ly = den > 1e-9 ? ay * ay * down[1] / den : 0, lz = den > 1e-9 ? az * az * down[2] / den : 0;
      for (const sx of this.samples[i]) {
        const rl = [sx, ly, lz];
        const r = qrot(b.q, rl); const P = vadd(b.x, r);
        const box = this.world.boxAt(P[0], P[2]);
        const H = box ? box.h : 0;
        if (P[1] >= H) continue;
        const Pprev = vadd(b.px, qrot(b.pq, rl));
        let corr, n;
        let side = false;
        if (box && Pprev[1] < H - 0.04) {
          // came in from the side: push out horizontally (only for shallow side penetration)
          const dxl = P[0] - (box.cx - box.hx), dxr = box.cx + box.hx - P[0], dzl = P[2] - (box.cz - box.hz), dzr = box.cz + box.hz - P[2];
          const m = Math.min(dxl, dxr, dzl, dzr);
          if (m < 0.1) {
            n = m === dxl ? [-1, 0, 0] : m === dxr ? [1, 0, 0] : m === dzl ? [0, 0, -1] : [0, 0, 1];
            corr = vsc(n, Math.min(m + 1e-4, 0.02)); side = true;
          }
        }
        if (!side) { n = [0, 1, 0]; corr = [0, Math.min(H - P[1], 0.05), 0]; }
        const dl = posCorr(b, r, null, null, corr, 0);
        // static friction (position level)
        const r2 = qrot(b.q, rl), P2 = vadd(b.x, r2);
        const dp = vsub(P2, Pprev); const dpt = vsub(dp, vsc(n, vdot(dp, n)));
        const lt = vlen(dpt);
        if (lt > 1e-9) {
          const t = vsc(dpt, 1 / lt); const wt = b.genInvMass(r2, t);
          if (lt / wt < this.mu * this.tune.friction * 1.2 * dl) posCorr(b, r2, null, null, vsc(dpt, -1), 0);
        }
        this.contacts.push({ i, rl, n, dl });
        this.touch[i] = true;
      }
    }
    // 4) velocities from positions
    for (const b of B) {
      b.v[0] = (b.x[0] - b.px[0]) / h; b.v[1] = (b.x[1] - b.px[1]) / h; b.v[2] = (b.x[2] - b.px[2]) / h;
      const dq = qmul(b.q, qconj(b.pq)); const sg = dq[3] < 0 ? -1 : 1;
      b.w[0] = sg * 2 * dq[0] / h; b.w[1] = sg * 2 * dq[1] / h; b.w[2] = sg * 2 * dq[2] / h;
    }
    // 5) velocity solve: dynamic friction, joint damping
    for (const c of this.contacts) {
      const b = B[c.i]; const r = qrot(b.q, c.rl);
      let vel = vadd(b.v, vcross(b.w, r));
      // soak up rebound: remove part of the separating normal velocity (wet flesh doesn't spring back)
      const vn0 = vdot(vel, c.n);
      if (vn0 > 0 && this.tune.bounce > 0) {
        b.applyVelImpulse(vsc(c.n, -vn0 * this.tune.bounce / b.genInvMass(r, c.n)), r, 1);
        vel = vadd(b.v, vcross(b.w, r));
      }
      const vn = vdot(vel, c.n); const vt = vsub(vel, vsc(c.n, vn)); const l = vlen(vt);
      if (l < 1e-7) continue;
      const t = vsc(vt, 1 / l); const fn = c.dl / (h * h);
      const dv = Math.min(h * this.mu * this.tune.friction * fn * b.invM, l);
      const p = vsc(t, -dv / b.genInvMass(r, t));
      b.applyVelImpulse(p, r, 1);
    }
    const k = Math.min(this.kd * this.tune.damp * h, 1);
    for (let i = 0; i < N - 1; i++) {
      const b1 = B[i], b2 = B[i + 1];
      const dw = vsub(b2.w, b1.w); const l = vlen(dw); if (l < 1e-9) continue;
      const n = vsc(dw, 1 / l);
      const mag = l * k / (b1.angInvMass(n) + b2.angInvMass(n));
      const p = vsc(n, mag);
      const a1 = b1.invIw(p), a2 = b2.invIw(p);
      for (let j = 0; j < 3; j++) { b1.w[j] += a1[j]; b2.w[j] -= a2[j]; }
    }
  }

  // ---------- brain ----------
  setPose(fn, alpha) { for (let i = 0; i < N - 1; i++) { this.theta[i] = fn(i, CURL_PROFILE[i] / PROF_SUM); this.alpha[i] = i === 0 ? 2e-7 : (typeof alpha === 'function' ? alpha(i) : alpha) * this.tune.soft; } }

  airPose(p, sd) {
    const k = this.hop && this.hop.kind === 'turn' ? 0.4 : 1;
    this.setPose((i, w) => sd * 0.55 * this.tune.arch * w * Math.sin(Math.PI * p) * k, 2e-5);
    this.kd = 1.5; this.mu = 0.55;
  }
  orient() { // body yaw (about +y) and pitch, read from front-half vs back-half centers of mass
    const cf = [0, 0, 0], cb = [0, 0, 0]; let mf = 0, mb = 0;
    this.bodies.forEach((b, i) => { const t = i < 5 ? cf : cb; for (let k = 0; k < 3; k++) t[k] += b.x[k] * b.m; if (i < 5) mf += b.m; else mb += b.m; });
    const d = vsub(vsc(cf, 1 / mf), vsc(cb, 1 / mb));
    return { yaw: Math.atan2(-d[2], d[0]), pitch: Math.atan2(d[1], Math.hypot(d[0], d[2])) };
  }
  // Forward-simulate the flight with the real muscles (no contacts) to see where the body will point on arrival.
  predictFlight(v, w, p0, T) {
    const snap = this.bodies.map((b) => [b.x.slice(), b.q.slice(), b.v.slice(), b.w.slice()]);
    const saved = [this.theta.slice(), this.alpha.slice(), this.kd, this.mu, this.substeps];
    const sd = this.side();
    for (const b of this.bodies) { b.v = vadd(v, vcross(w, vsub(b.x, p0))); b.w = w.slice(); }
    this.noContacts = true; this.substeps = 8;
    for (let t = 0; t < T; t += 1 / 40) { this.airPose(t / T, sd); this.physics(1 / 40); }
    const o = this.orient();
    this.bodies.forEach((b, i) => { b.x = snap[i][0]; b.q = snap[i][1]; b.v = snap[i][2]; b.w = snap[i][3]; b.px = b.x.slice(); b.pq = b.q.slice(); });
    [this.theta, this.alpha, this.kd, this.mu, this.substeps] = saved; this.noContacts = false;
    return o;
  }

  enterState(s) { this.state = s; this.stateT = 0; this.events.push(s); }

  update(dt, simulate = true) {
    this.t += dt; this.stateT += dt;
    const sd = this.side();
    const E = this.energy;
    const speed = vlen(this.comVel());
    switch (this.state) {
      case 'drop': {
        // falling from the sky: loose body, tail flailing; the landing is handled like any touchdown
        const tt = this.t;
        this.setPose((i, w) => sd * (0.5 * w * Math.sin(tt * 9 - i * 0.7)), 3e-5);
        this.kd = 2; this.mu = 0.55;
        if ((this.touch.some(Boolean) && this.stateT > 0.1) || this.stateT > 6) this.enterState('land');
        break;
      }
      case 'rest': {
        // idle: tail and gills twitch a little
        const tt = this.t;
        this.setPose((i) => i >= 5 ? sd * 0.07 * this.tune.idle * Math.sin(tt * 2.6 - i * 0.8) : 0, 6e-5);
        this.kd = 6; this.mu = 0.9;
        const recover = this.queue.length ? 0.045 : 0.14;
        this.energy = Math.min(1, this.energy + recover * dt);
        const delay = 0.25 + (1 - E) * 0.7;
        if (this.queue.length && this.stateT > delay && (speed < 0.35 || this.stateT > delay + 2)) this.plan();
        break;
      }
      case 'charge': {
        const hop = this.hop; const k = smooth(this.stateT / (hop.Tfull || hop.Tc));
        // tired fish trembles while it builds up the curl; a fully charged fish shivers with held tension
        const full = hop.manual && this.stateT >= hop.Tfull ? 0.03 : 0;
        const tremble = ((1 - E) * 0.06 + full) * Math.sin(this.t * 38);
        this.setPose((i, w) => sd * (hop.curl * w * k + (i > 4 ? tremble : 0)), 1.4e-5);
        this.kd = 6;
        if (!hop.manual && this.stateT >= hop.Tc + 0.06) this.enterState('slap');
        break;
      }
      case 'slap': {
        // muscles snap the other way; the light tail reaches the ground first
        const hop = this.hop;
        // the body uncurls; the light tail is driven hardest so it reaches the ground first
        this.setPose((i, w) => i >= 5 ? -sd * 0.3 * w : sd * hop.curl * w * 0.12, (i) => i >= 5 ? 3e-6 : 3e-5);
        this.kd = 3;
        const tail = this.touch[N - 1] || this.touch[N - 2];
        const tailDown = this.bodies[N - 1].v[1] < -0.2 || this.stateT > 0.05;
        if ((tail && tailDown && this.stateT > 0.035) || this.stateT > 0.2) this.launch();
        break;
      }
      case 'air': {
        const T = this.traj.T, p = clamp(this.stateT / T, 0, 1);
        // in flight: body arches, then straightens before touchdown (shape change only, no external torque)
        this.airPose(p, sd);
        const any = this.touch.some(Boolean);
        if ((any && this.stateT > 0.12 && this.comVel()[1] < 0) || this.stateT > T + 2) {
          this.enterState('land');
          const c = this.com();
          if (this.traj) this.traj.landErr = Math.hypot(c[0] - this.traj.p1[0], c[2] - this.traj.p1[2]);
          if (this.lastJump && this.traj) this.lastJump.err = this.traj.landErr;
        }
        break;
      }
      case 'land': {
        // muscles go slack for a moment after impact, then firm up again
        const limp = 1 + (this.tune.landLimp - 1) * (1 - smooth(this.stateT / 0.6));
        this.setPose(() => 0, 8e-5 * limp);
        this.kd = 9; this.mu = 1.1;
        // impact dissipation: a wet body hitting the ground loses most of its sliding and spinning energy
        if (this.touch.some(Boolean) && this.stateT < 0.7) {
          const f = Math.exp(-this.tune.absorb * dt);
          for (const b of this.bodies) { b.v[0] *= f; b.v[2] *= f; b.w[0] *= f; b.w[1] *= f; b.w[2] *= f; }
        }
        if ((this.stateT > 0.45 && speed < 0.3) || this.stateT > 3) {
          const tgt = this.queue[0];
          if (tgt) { const c = this.com(); if (Math.hypot(c[0] - tgt[0], c[2] - tgt[2]) < 0.45) { this.queue.shift(); this.events.push('reached'); } }
          this.enterState('rest');
        }
        break;
      }
    }
    if (simulate) this.physics(dt);
  }

  plan() {
    const tgt = this.queue[0]; const c = this.com();
    const d = [tgt[0] - c[0], 0, tgt[2] - c[2]]; const dist = Math.hypot(d[0], d[2]);
    if (dist < 0.4) { this.queue.shift(); this.events.push('reached'); return; }
    const dir = [d[0] / dist, 0, d[2] / dist];
    const h = this.heading(); const dyaw = Fish.yawBetween(h, dir);
    const E = this.energy;
    const reach = 3.1 * (0.35 + 0.65 * E);
    let hop;
    if (Math.abs(dyaw) > 1.15 && dist > 0.6) {
      const hd = Math.min(0.4, dist * 0.25);
      hop = { kind: 'turn', land: [c[0] + dir[0] * hd, 0, c[2] + dir[2] * hd], face: dir, dist: hd };
    } else {
      const hd = Math.min(dist, reach);
      hop = { kind: 'jump', land: [c[0] + dir[0] * hd, 0, c[2] + dir[2] * hd], face: dir, dist: hd };
    }
    hop.land[1] = this.world.height(hop.land[0], hop.land[2]);
    hop.Tc = 0.26 + 0.95 * Math.pow(1 - E, 1.4) + (hop.kind === 'turn' ? 0.04 : 0);
    hop.curl = 2.5 * this.tune.curl * (0.7 + 0.3 * E) * (hop.kind === 'turn' ? 0.85 : 1);
    this.hop = hop;
    this.enterState('charge');
  }

  // ---------- manual control: hold to charge, release to jump in a direction ----------
  chargeFull() { return 0.9 + 0.9 * Math.pow(1 - this.energy, 1.4); } // tired fish needs longer to reach full curl
  canCharge() { return this.state === 'rest' && this.stateT > 0; }
  beginCharge() {
    if (!this.canCharge()) return false;
    this.hop = { kind: 'jump', manual: true, Tfull: this.chargeFull(), Tc: Infinity, curl: 2.5 * this.tune.curl * (0.7 + 0.3 * this.energy) };
    this.enterState('charge');
    return true;
  }
  charging() { return this.state === 'charge' && !!this.hop && !!this.hop.manual; }
  chargePower() { return this.charging() ? clamp(this.stateT / this.hop.Tfull, 0, 1) : 0; }
  manualReach(power) { const reach = 3.1 * (0.35 + 0.65 * this.energy); return 0.35 + (reach - 0.35) * power; }
  release(dir) {
    if (!this.charging()) return false;
    const hop = this.hop, power = this.chargePower(), c = this.com();
    const l = Math.hypot(dir[0], dir[2]);
    const d = l > 1e-6 ? [dir[0] / l, 0, dir[2] / l] : this.heading();
    const dist = this.manualReach(power);
    hop.land = [c[0] + d[0] * dist, 0, c[2] + d[2] * dist];
    hop.land[1] = this.world.height(hop.land[0], hop.land[2]);
    hop.face = d; hop.dist = dist;
    if (Math.abs(Fish.yawBetween(this.heading(), d)) > 1.15) hop.kind = 'turn';
    hop.Tc = this.stateT;
    hop.curl *= Math.max(0.35, smooth(power)); // the uncurl starts from the curl actually reached
    this.enterState('slap');
    return true;
  }

  launch() {
    const hop = this.hop; const E = this.energy;
    const p0 = this.com();
    const p1 = [hop.land[0], hop.land[1] + this.restH, hop.land[2]];
    const hd = Math.hypot(p1[0] - p0[0], p1[2] - p0[2]);
    // highest obstacle under the path (sampled), so the arc clears step edges
    let obst = 0;
    for (let k = 0; k <= 16; k++) {
      const t = k / 16; obst = Math.max(obst, this.world.height(p0[0] + (p1[0] - p0[0]) * t, p0[2] + (p1[2] - p0[2]) * t));
    }
    let apex = Math.max(p0[1], p1[1]) + (hop.kind === 'turn' ? Math.max(0.62, 0.38 + 0.16 * hd) : 0.38 + 0.16 * hd) * (0.55 + 0.45 * E);
    apex = Math.max(apex, obst + 0.5);
    // ballistic solve from the COM where the muscles actually put it
    const vy = Math.sqrt(2 * G * Math.max(apex - p0[1], 0.01));
    const T = vy / G + Math.sqrt(2 * Math.max(apex - p1[1], 0.001) / G);
    const v = [(p1[0] - p0[0]) / T, vy, (p1[2] - p0[2]) / T];
    // ground reaction torque sets yaw (and pitch so the body comes down level); in the air L is conserved
    const h = this.heading();
    const dyaw = Fish.yawBetween(h, hop.face);
    // body pitch from the front-half COM vs back-half COM: roughly where a straightened body will point
    const cf = [0, 0, 0], cb = [0, 0, 0]; let mf = 0, mb = 0;
    this.bodies.forEach((b, i) => { const t = i < 5 ? cf : cb; for (let k = 0; k < 3; k++) t[k] += b.x[k] * b.m; if (i < 5) mf += b.m; else mb += b.m; });
    const ax = vnorm(vsub(vsc(cf, 1 / mf), vsc(cb, 1 / mb)));
    const pitch0 = Math.asin(clamp(ax[1], -1, 1));
    const flips = 0;
    let wy = dyaw / T, wp = (-pitch0 - flips * 2 * Math.PI) / T;
    const A = vnorm(vcross(h, [0, 1, 0]));
    // Shape changes in flight also rotate the body (angular momentum is conserved, inertia is not),
    // so refine the take-off spin by predicting the flight twice and correcting the arrival error.
    const yaw0 = this.orient().yaw, yawGoal = yaw0 + dyaw;
    const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
    for (let it = 0; it < 2; it++) {
      const o = this.predictFlight(v, vadd([0, wy, 0], vsc(A, wp)), p0, T);
      wy -= wrap(o.yaw - yawGoal) / T; wp -= o.pitch / T;
    }
    const w = vadd([0, wy, 0], vsc(A, wp));
    for (const b of this.bodies) {
      const r = vsub(b.x, p0);
      const vb = vadd(v, vcross(w, r));
      b.v = vb; b.w = w.slice();
    }
    this.traj = { p0, p1, v, T, apex };
    this.lastJump = { kind: hop.kind, dist: hd, T, yaw: dyaw, rise: p1[1] - this.restH - this.world.height(p0[0], p0[2]), flips, energy: E, err: null };
    const cost = hop.kind === 'turn' ? 0.06 : 0.07 + 0.07 * (hd / 3) + 0.08 * Math.max(0, p1[1] - p0[1]);
    this.energy = Math.max(0.05, this.energy - cost);
    this.enterState('air');
  }
}

export { TUNE_DEFAULTS, Fish, World, SEGS, BODY_PROPS, N, TOTAL, CENTERS, BOUNDS, STATE_LABEL, qrot, qconj, vlen };
