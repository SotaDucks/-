// Optional integration check: start the local Worker, then run this file with Node.
import assert from 'node:assert/strict';
import { Fish, World } from '../shared/fish-sim.mjs';
import { packFish, applyImpact } from '../shared/fish-collision.mjs';

const room = `check-${Date.now()}`;
const peers = [0, 1].map(i => {
  const fish = new Fish(new World()); fish.reset([0, 0, i * 0.12], 0, 1);
  for (const b of fish.bodies) { b.x[1] += 1; b.v = [0, 0, i ? -3 : 3]; }
  const ws = new WebSocket(`ws://127.0.0.1:8787/room/${room}`);
  ws.binaryType = 'arraybuffer';
  return { ws, fish, impacts: [], poses: [], ack: 0, time: 0, leaves: [] };
});
let tick;
try {
  await Promise.race([
    Promise.all(peers.map(p => new Promise((resolve, reject) => {
      p.ws.addEventListener('error', () => reject(new Error('Local Worker WebSocket connection failed')));
      p.ws.addEventListener('message', e => {
        if (typeof e.data !== 'string') { p.poses.push(new Float64Array(e.data)); return; }
        const m = JSON.parse(e.data);
        if (m.t === 'hello') { p.id = m.id; assert.equal(m.collisions, 1); resolve(); }
        if (m.t === 'leave') p.leaves.push(m.id);
        if (m.t === 'impact') {
          if (m.seq <= p.ack) return;
          assert.ok(applyImpact(p.fish, m, 1));
          p.ack = m.seq; p.impacts.push(m);
        }
      });
    }))),
    new Promise((_, reject) => { tick = setTimeout(() => reject(new Error('Worker handshake timed out')), 5000); }),
  ]);
  clearTimeout(tick);
  for (let step = 0; step < 60; step++) {
    for (const p of peers) {
      p.fish.update(1 / 60);
      if (step % 3 === 0) p.ws.send(packFish(p.fish, ++p.time, 1, p.ack));
    }
    await new Promise(resolve => setTimeout(resolve, 17));
  }
  assert.ok(peers.every(p => p.impacts.length > 0), 'both clients must receive an impact');
  assert.equal(peers[0].impacts[0].seq, peers[1].impacts[0].seq);
  assert.ok(peers[0].poses.some(p => p[0] === peers[1].id));
  assert.ok(peers[1].poses.some(p => p[0] === peers[0].id));
  peers[0].ws.close();
  await new Promise(resolve => setTimeout(resolve, 100));
  assert.ok(peers[1].leaves.includes(peers[0].id));
  console.log('PASS: two WebSocket clients connected, exchanged poses, applied matching impacts, and cleaned up on leave.');
} finally {
  clearTimeout(tick);
  for (const p of peers) p.ws.close();
}
