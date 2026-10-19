// Animations du corps : une suite de poses (comme la commande POSE : x y z en
// mm, roulis tangage lacet en degrés), chacune atteinte en `t` secondes.
// L'interface interpole entre les poses et envoie POSE au robot.

const P = (x, y, z, roll, pitch, yaw, t) => ({ pose: [x, y, z, roll, pitch, yaw], t });

export const PRESETS = [
  {
    name: "Révérence",
    frames: [P(0, 0, 0, 0, 0, 0, 0.4), P(12, 0, -12, 0, 16, 0, 0.8), P(12, 0, -12, 0, 16, 0, 0.5), P(0, 0, 0, 0, 0, 0, 0.8)],
  },
  {
    name: "Regarder autour",
    frames: [P(0, 0, 5, 0, 0, 22, 0.9), P(0, 0, 5, 0, -6, 22, 0.5), P(0, 0, 5, 0, 0, -22, 1.4), P(0, 0, 5, 0, -6, -22, 0.5), P(0, 0, 0, 0, 0, 0, 0.9)],
  },
  {
    name: "Danse",
    frames: [
      P(0, 15, 0, -12, 0, 8, 0.35), P(0, -15, 0, 12, 0, -8, 0.35),
      P(0, 15, 0, -12, 0, 8, 0.35), P(0, -15, 0, 12, 0, -8, 0.35),
      P(10, 0, -15, 0, 10, 0, 0.35), P(-10, 0, 10, 0, -10, 0, 0.35),
      P(0, 0, 0, 0, 0, 0, 0.4),
    ],
  },
  {
    name: "Pompes",
    frames: [P(0, 0, -25, 0, 0, 0, 0.5), P(0, 0, 10, 0, 0, 0, 0.5), P(0, 0, -25, 0, 0, 0, 0.5), P(0, 0, 10, 0, 0, 0, 0.5), P(0, 0, 0, 0, 0, 0, 0.5)],
  },
  {
    name: "Cercle",
    frames: [...Array(13).keys()].map((i) => {
      const a = (i / 12) * 2 * Math.PI;
      return i === 12 ? P(0, 0, 0, 0, 0, 0, 0.4) : P(18 * Math.cos(a), 18 * Math.sin(a), 0, -8 * Math.sin(a), 8 * Math.cos(a), 0, i ? 0.25 : 0.5);
    }),
  },
];

const smooth = (s) => s * s * (3 - 2 * s);

/** Joue une animation : appeler `pose(now)` à chaque image, null quand c'est fini. */
export class Animator {
  constructor() {
    this.frames = null;
  }

  get playing() {
    return this.frames !== null;
  }

  play(frames, from, { loop = false } = {}) {
    if (!frames.length) return;
    this.frames = frames;
    this.loop = loop;
    this.index = 0;
    this.from = from.slice();
    this.start = performance.now();
  }

  stop() {
    this.frames = null;
  }

  pose(now = performance.now()) {
    if (!this.frames) return null;
    let frame = this.frames[this.index];
    let s = (now - this.start) / 1000 / Math.max(0.05, frame.t);
    while (s >= 1) {
      // Pose atteinte : on passe à la suivante.
      this.from = frame.pose.slice();
      this.start += Math.max(0.05, frame.t) * 1000;
      this.index++;
      if (this.index >= this.frames.length) {
        if (!this.loop) {
          this.frames = null;
          return this.from;
        }
        this.index = 0;
      }
      frame = this.frames[this.index];
      s = (now - this.start) / 1000 / Math.max(0.05, frame.t);
    }
    const k = smooth(Math.max(0, s));
    return this.from.map((v, i) => v + (frame.pose[i] - v) * k);
  }
}
