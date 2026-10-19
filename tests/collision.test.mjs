import test from 'node:test';
import assert from 'node:assert/strict';
import { Fish, World, SEGS, N } from '../shared/fish-sim.mjs';
import { packFish, unpackFish, findImpact, applyImpact, resolveFishPair, HEADER, POSE_LEN } from '../shared/fish-collision.mjs';
import { TestFishController } from '../shared/test-fish.mjs';

function makeFish(x = 0, z = 0, velocity = [0, 0, 0]) {
  const f = new Fish(new World()); f.reset([x, 0, z], 0, 1);
  for (const b of f.bodies) { b.x[1] += 1; b.px = b.x.slice(); b.v = velocity.slice(); }
  return f;
}
const state = (f, epoch = 1) => ({ bodies: f.bodies, epoch });
const finite = f => f.bodies.every(b => [...b.x, ...b.q, ...b.v, ...b.w].every(Number.isFinite));

test('pose round trip retains velocity, spawn generation and impact acknowledgement', () => {
  const f = makeFish(2, 3, [1, 2, 3]), packet = packFish(f, 123, 4, 567);
  assert.equal(packet.length, POSE_LEN);
  const result = unpackFish(packet.buffer);
  assert.equal(result.epoch, 4); assert.equal(result.ack, 567);
  assert.deepEqual(result.bodies[4].v, [1, 2, 3]);
  assert.deepEqual(result.bodies[4].x, f.bodies[4].x);
});

test('invalid packets and physically broken poses are rejected', () => {
  const f = makeFish();
  assert.equal(unpackFish(new ArrayBuffer(5)), null);
  for (const [offset, value] of [[0, NaN], [1, -1], [2, 1.5], [HEADER + 3, 0], [HEADER, 800], [HEADER + 7, 200]]) {
    const p = packFish(f, 1, 1);
    if (offset === HEADER + 3) p.fill(0, offset, offset + 4);
    else p[offset] = value;
    assert.equal(unpackFish(p.buffer), null, `accepted invalid offset ${offset}`);
  }
});

test('separated fish do not collide; lateral overlap exchanges equal opposite momentum', () => {
  const a = makeFish(0, 0, [0, 0, 2]), b = makeFish(0, 0.13, [0, 0, -2]);
  assert.equal(findImpact(state(a), state(makeFish(0, 4))), null);
  const hit = findImpact(state(a), state(b));
  assert.ok(hit); assert.ok(Math.hypot(...hit.a.impulse) > 0);
  hit.a.impulse.forEach((v, k) => assert.equal(v, -hit.b.impulse[k]));
  assert.ok(applyImpact(a, hit.a, 1)); assert.ok(applyImpact(b, hit.b, 1));
  assert.ok(a.comVel()[2] < 2); assert.ok(b.comVel()[2] > -2);
  const p = a.comVel().map((v, k) => v * a.mass + b.comVel()[k] * b.mass);
  assert.ok(Math.hypot(...p) < 1e-7);
  for (let i = 0; i < 120; i++) { a.update(1 / 60); b.update(1 / 60); }
  assert.ok(finite(a) && finite(b));
});

test('swept contact catches a fast fish that crossed between network snapshots', () => {
  const a = makeFish(0, 0.65, [0, 0, 26]), b = makeFish();
  assert.equal(findImpact(state(a), state(b)), null);
  const hit = findImpact(state(a), state(b), 0.05);
  assert.ok(hit); assert.ok(Math.hypot(...hit.a.impulse) > 0);
  assert.equal(Math.hypot(...hit.a.correction), 0);
});

test('stale spawn hits are ignored and an interrupted charge cannot launch later', () => {
  const a = makeFish(0, 0, [0, 0, 3]), b = makeFish(0, 0.1);
  const hit = findImpact(state(a, 2), state(b, 2));
  const before = a.bodies[3].v.slice();
  assert.equal(applyImpact(a, hit.a, 3), false);
  assert.deepEqual(a.bodies[3].v, before);
  a.state = 'rest'; a.stateT = 1; a.beginCharge();
  assert.ok(applyImpact(a, hit.a, 2));
  assert.equal(a.state, 'drop'); assert.equal(a.hop, null);
  assert.equal(a.release([1, 0, 0]), false);
});

test('local player and autonomous fish jump, collide and keep their joints stable', () => {
  const player = new Fish(new World()), bot = new Fish(new World());
  player.drop([0, 0, 0], 0, 0.5);
  let seed = 42;
  const random = () => ((seed = (seed * 1664525 + 1013904223) >>> 0) / 2 ** 32);
  const controller = new TestFishController(bot, player, random);
  let jumps = 0, hits = 0, maxJointGap = 0;
  for (let frame = 0; frame < 1800; frame++) {
    const wasAir = bot.state === 'air';
    player.update(1 / 60, false); controller.update(1 / 60);
    if (!wasAir && bot.state === 'air') jumps++;
    player.touch.fill(false); bot.touch.fill(false);
    for (let s = 0; s < 20; s++) {
      player.substep(1 / 1200); bot.substep(1 / 1200);
      if ((s + 1) % 5 === 0 && resolveFishPair(player, bot)) hits++;
    }
    assert.ok(finite(player) && finite(bot));
    for (const f of [player, bot]) for (let i = 0; i < N - 1; i++) {
      const gap = Math.hypot(...f.bodies[i].x.map((v, k) => v - f.bodies[i + 1].x[k]));
      maxJointGap = Math.max(maxJointGap, gap - (SEGS[i].L + SEGS[i + 1].L) / 2);
    }
  }
  assert.ok(jumps >= 3, `only ${jumps} jumps`);
  assert.ok(hits > 0, 'autonomous fish never touched the player');
  assert.ok(maxJointGap < 0.15, `joint gap grew by ${maxJointGap}`);
});
