// Relay for the swordfish game: one Durable Object per room forwards each player's pose to everyone else.
// It never simulates anything, so the game's mechanics stay entirely in the browser.
//
// Protocol
//   client -> server  binary: Float64Array pose (see POSE_LEN in index.html)
//   server -> client  binary: [senderId, ...pose] as Float64Array
//                     text:   {"t":"hello","id":n,"peers":[...]} | {"t":"join","id":n} | {"t":"leave","id":n}
import { DurableObject } from 'cloudflare:workers';

const MAX_PLAYERS = 16;
const MAX_MSG = 1024; // bytes; a pose is ~580

export default {
  async fetch(req, env) {
    const m = new URL(req.url).pathname.match(/^\/room\/([A-Za-z0-9_-]{1,24})$/);
    if (!m) return new Response('xianyu relay ok', { headers: { 'content-type': 'text/plain; charset=utf-8' } });
    if (req.headers.get('Upgrade') !== 'websocket') return new Response('expected websocket', { status: 426 });
    return env.ROOMS.get(env.ROOMS.idFromName(m[1])).fetch(req);
  },
};

export class Room extends DurableObject {
  async fetch() {
    const others = this.ctx.getWebSockets();
    if (others.length >= MAX_PLAYERS) return new Response('room full', { status: 503 });
    const [client, server] = Object.values(new WebSocketPair());
    const id = Math.floor(Math.random() * 2 ** 31);
    // hibernatable socket: the object can sleep between messages without dropping connections
    this.ctx.acceptWebSocket(server);
    server.serializeAttachment({ id });
    server.send(JSON.stringify({ t: 'hello', id, peers: others.map(idOf) }));
    this.broadcast(server, JSON.stringify({ t: 'join', id }));
    return new Response(null, { status: 101, webSocket: client });
  }

  webSocketMessage(ws, msg) {
    if (typeof msg === 'string' || msg.byteLength > MAX_MSG || msg.byteLength % 8) return;
    const out = new Uint8Array(8 + msg.byteLength);
    new DataView(out.buffer).setFloat64(0, idOf(ws));
    out.set(new Uint8Array(msg), 8);
    this.broadcast(ws, out);
  }

  webSocketClose(ws) { this.leave(ws); }
  webSocketError(ws) { this.leave(ws); }

  leave(ws) {
    try { ws.close(1000); } catch (_) {}
    this.broadcast(ws, JSON.stringify({ t: 'leave', id: idOf(ws) }));
  }

  broadcast(from, data) {
    for (const ws of this.ctx.getWebSockets()) {
      if (ws === from) continue;
      try { ws.send(data); } catch (_) {}
    }
  }
}

function idOf(ws) { return ws.deserializeAttachment()?.id ?? 0; }
