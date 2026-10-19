// One room forwards poses and arbitrates fish impacts. Clients retain their own
// articulated physics; the room sends equal and opposite impulses to both owners.
//
// Protocol
//   client -> server  binary: Float64Array pose (see POSE_LEN in index.html)
//   server -> client  binary: [senderId, ...pose] as Float64Array
//                     text:   hello/join/leave, or {t:'impact', seq, epoch, index, local, impulse, correction}
import { DurableObject } from 'cloudflare:workers';
import { unpackFish, findImpact, POSE_LEN, LEGACY_POSE_LEN } from '../../shared/fish-collision.mjs';

const MAX_PLAYERS = 16;
const MAX_MSG = POSE_LEN * 8;
const MAX_AGE = 250; // Never hit a disconnected or background-tab fish using a stale pose.

export default {
  async fetch(req, env) {
    const m = new URL(req.url).pathname.match(/^\/room\/([A-Za-z0-9_-]{1,24})$/);
    if (!m) return new Response('xianyu relay ok', { headers: { 'content-type': 'text/plain; charset=utf-8' } });
    if (req.headers.get('Upgrade') !== 'websocket') return new Response('expected websocket', { status: 426 });
    return env.ROOMS.get(env.ROOMS.idFromName(m[1])).fetch(req);
  },
};

export class Room extends DurableObject {
  constructor(ctx, env) {
    super(ctx, env);
    this.ctx = ctx;
    this.players = new Map();
    this.sequence = Date.now() * 1000;
    // Keep the latest pose and the acknowledgement barrier through DO hibernation.
    for (const ws of ctx.getWebSockets()) {
      const saved = ws.deserializeAttachment();
      if (!saved?.pose) continue;
      const pose = unpackFish(saved.pose);
      if (pose) this.players.set(ws, { ...saved, state: pose });
      this.sequence = Math.max(this.sequence, saved.waiting || 0);
    }
  }

  async fetch() {
    const others = this.ctx.getWebSockets();
    if (others.length >= MAX_PLAYERS) return new Response('room full', { status: 503 });
    const [client, server] = Object.values(new WebSocketPair());
    let id;
    do { id = Math.floor(Math.random() * 2 ** 31); } while (others.some(ws => idOf(ws) === id));
    // hibernatable socket: the object can sleep between messages without dropping connections
    this.ctx.acceptWebSocket(server);
    server.serializeAttachment({ id });
    server.send(JSON.stringify({ t: 'hello', id, peers: others.map(idOf), collisions: 1 }));
    this.broadcast(server, JSON.stringify({ t: 'join', id }));
    return new Response(null, { status: 101, webSocket: client });
  }

  webSocketMessage(ws, msg) {
    if (!(msg instanceof ArrayBuffer) || msg.byteLength > MAX_MSG) return;
    const state = unpackFish(msg);
    // Older clients remain visible, but cannot take part in the new impact protocol.
    if (!state && !(msg.byteLength === LEGACY_POSE_LEN * 8 && new Float64Array(msg).every(Number.isFinite))) return;
    const previous = this.players.get(ws), now = Date.now();
    if (state && previous && state.time <= previous.state.time) return;
    if (state) {
      const player = { id: idOf(ws), pose: msg, state, received: now, waiting: previous?.waiting || 0 };
      this.players.set(ws, player);
      this.savePlayer(ws, player);
      for (const [otherWs, other] of this.players) {
        if (otherWs === ws || now - other.received > MAX_AGE || state.ack < player.waiting || other.state.ack < other.waiting) continue;
        // The other snapshot may be one packet older. Predict only a short distance;
        // sweep back over a send interval so fast, thin fish do not skip contacts.
        const predicted = { ...other.state, bodies: other.state.bodies.map(b => ({ ...b, x: b.x.map((v, k) => v + b.v[k] * Math.min(0.08, (now - other.received) / 1000)) })) };
        const impact = findImpact(state, predicted, 0.05);
        if (!impact) continue;
        const seq = ++this.sequence;
        player.waiting = seq; other.waiting = seq;
        this.savePlayer(ws, player); this.savePlayer(otherWs, other);
        ws.send(JSON.stringify({ t: 'impact', seq, ...impact.a }));
        try { otherWs.send(JSON.stringify({ t: 'impact', seq, ...impact.b })); } catch (_) {}
        break; // Both owners must acknowledge before either receives another impulse.
      }
    }
    const out = new Uint8Array(8 + msg.byteLength);
    new Float64Array(out.buffer, 0, 1)[0] = idOf(ws);
    out.set(new Uint8Array(msg), 8);
    this.broadcast(ws, out);
  }

  webSocketClose(ws) { this.leave(ws); }
  webSocketError(ws) { this.leave(ws); }

  leave(ws) {
    this.players.delete(ws);
    try { ws.close(1000); } catch (_) {}
    this.broadcast(ws, JSON.stringify({ t: 'leave', id: idOf(ws) }));
  }

  broadcast(from, data) {
    for (const ws of this.ctx.getWebSockets()) {
      if (ws === from) continue;
      try { ws.send(data); } catch (_) {}
    }
  }

  savePlayer(ws, player) {
    ws.serializeAttachment({ id: player.id, pose: player.pose, received: player.received, waiting: player.waiting });
  }
}

function idOf(ws) { return ws.deserializeAttachment()?.id ?? 0; }
