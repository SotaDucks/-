import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { Fish, World } from '../shared/fish-sim.mjs';
import { packFish, applyImpact } from '../shared/fish-collision.mjs';

// Exercise the real room protocol with sockets that preserve attachments just as
// hibernatable Cloudflare sockets do; only the platform base class is substituted.
const source = (await readFile(new URL('../server/src/index.js', import.meta.url), 'utf8'))
  .replace("import { DurableObject } from 'cloudflare:workers';", 'const DurableObject = class { constructor(ctx) { this.ctx = ctx; } };')
  .replace("'../../shared/fish-collision.mjs'", JSON.stringify(new URL('../shared/fish-collision.mjs', import.meta.url).href));
const { Room } = await import('data:text/javascript;base64,' + Buffer.from(source).toString('base64'));

class Socket {
  constructor(id) { this.saved = { id }; this.messages = []; }
  serializeAttachment(value) { this.saved = structuredClone(value); }
  deserializeAttachment() { return structuredClone(this.saved); }
  send(value) { this.messages.push(value); }
  close() {}
  impacts() { return this.messages.filter(v => typeof v === 'string').map(JSON.parse).filter(v => v.t === 'impact'); }
}
function setup() {
  const a = new Socket(123), b = new Socket(456), context = { getWebSockets: () => [a, b] };
  const fa = new Fish(new World()), fb = new Fish(new World());
  fa.reset([0, 0, 0], 0, 1); fb.reset([0, 0, 0.12], 0, 1);
  fa.bodies.forEach(body => { body.v = [0, 0, 3]; });
  fb.bodies.forEach(body => { body.v = [0, 0, -3]; });
  return { a, b, fa, fb, context, room: new Room(context, {}) };
}
const send = (room, socket, fish, time, ack = 0, epoch = 1) => room.webSocketMessage(socket, packFish(fish, time, epoch, ack).buffer);

test('room relays a correctly encoded sender ID and arbitrates matching impulses', () => {
  const { room, a, b, fa, fb } = setup();
  send(room, a, fa, 1); send(room, b, fb, 1);
  const pa = a.impacts(), pb = b.impacts();
  assert.equal(pa.length, 1); assert.equal(pb.length, 1);
  assert.equal(pa[0].seq, pb[0].seq);
  pa[0].impulse.forEach((v, k) => assert.ok(Math.abs(v + pb[0].impulse[k]) < 1e-9));
  const relayed = b.messages.find(v => typeof v !== 'string');
  assert.equal(new Float64Array(relayed.buffer)[0], 123);
  assert.ok(applyImpact(fa, pa[0], 1)); assert.ok(applyImpact(fb, pb[0], 1));
  assert.ok(fa.comVel()[2] < 3); assert.ok(fb.comVel()[2] > -3);
});

test('room waits for both acknowledgements, including after hibernation', () => {
  const { room, a, b, fa, fb, context } = setup();
  send(room, a, fa, 1); send(room, b, fb, 1);
  const seq = a.impacts()[0].seq;
  const resumed = new Room(context, {});
  send(resumed, a, fa, 2); send(resumed, b, fb, 2);
  send(resumed, a, fa, 3, seq);
  assert.equal(a.impacts().length, 1);
  send(resumed, b, fb, 3, seq);
  assert.equal(a.impacts().length, 2);
  assert.ok(a.impacts()[1].seq > seq);
});

test('obsolete, malformed and disconnected snapshots cannot produce impacts', () => {
  const { room, a, b, fa, fb } = setup();
  send(room, a, fa, 10);
  const count = b.messages.length;
  send(room, a, fa, 9);
  assert.equal(b.messages.length, count);
  room.webSocketMessage(a, new ArrayBuffer(16));
  assert.equal(b.messages.length, count);
  room.players.get(a).received -= 1000;
  send(room, b, fb, 10);
  assert.equal(a.impacts().length, 0);
  room.leave(a);
  assert.equal(room.players.has(a), false);
});

test('server preserves spawn generation on impact messages', () => {
  const { room, a, b, fa, fb } = setup();
  send(room, a, fa, 1, 0, 7); send(room, b, fb, 1, 0, 9);
  assert.equal(a.impacts()[0].epoch, 7); assert.equal(b.impacts()[0].epoch, 9);
});
