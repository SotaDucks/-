// Local practice opponent: uses the same charging, jumping and collision physics.
export class TestFishController {
  constructor(fish, player, random = Math.random) {
    this.fish = fish; this.player = player; this.random = random;
    this.wait = 0.5; this.chargeFor = 0; this.direction = [1, 0, 0];
    this.respawn();
  }

  respawn() {
    const p = this.player.com(), a = this.random() * Math.PI * 2;
    this.fish.drop([p[0] + Math.cos(a) * 1.6, 0, p[2] + Math.sin(a) * 1.6], this.random() * Math.PI * 2, 0.7);
    this.wait = 0.5;
  }

  update(dt) {
    const f = this.fish, p = this.player.com(), c = f.com();
    if (c[1] < -3 || Math.hypot(c[0] - p[0], c[2] - p[2]) > 9) this.respawn();
    if (f.charging()) {
      if (f.stateT >= this.chargeFor) { f.release(this.direction); this.wait = 0.35 + this.random() * 0.9; }
    } else if (f.canCharge()) {
      this.wait -= dt;
      if (this.wait <= 0) {
        // Often aim close to the player, with occasional wider hops for variety.
        const angle = this.random() * Math.PI * 2, radius = this.random() < 0.65 ? 0.35 : 1.8;
        const dx = p[0] + Math.cos(angle) * radius - c[0], dz = p[2] + Math.sin(angle) * radius - c[2];
        const distance = Math.hypot(dx, dz);
        this.direction = [dx, 0, dz];
        f.energy = Math.max(f.energy, 0.65);
        const power = Math.min(1, Math.max(0.18, (distance - 0.35) / (f.manualReach(1) - 0.35)));
        this.chargeFor = f.chargeFull() * power;
        f.beginCharge();
      }
    }
    f.update(dt, false);
    f.events.length = 0;
  }
}
