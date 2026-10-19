// Joystick analogique (souris ou tactile). value = {x, y} dans [-1, 1],
// y positif vers le haut ; revient au centre au relâchement.

const DEADZONE = 0.08;

export class Joystick {
  constructor(el, { horizontal = false } = {}) {
    this.el = el;
    this.knob = el.querySelector(".knob");
    this.horizontal = horizontal;
    this.pointer = null;
    this.touch = { x: 0, y: 0 }; // position tenue au doigt / à la souris
    this.external = { x: 0, y: 0 }; // clavier ou manette
    el.addEventListener("pointerdown", (e) => this._down(e));
    el.addEventListener("pointermove", (e) => this._move(e));
    for (const ev of ["pointerup", "pointercancel", "lostpointercapture"]) {
      el.addEventListener(ev, (e) => this._up(e));
    }
  }

  get active() {
    return this.pointer !== null;
  }

  get value() {
    const src = this.active ? this.touch : this.external;
    return { x: dead(src.x), y: this.horizontal ? 0 : dead(src.y) };
  }

  setExternal(x, y) {
    this.external = { x, y };
  }

  render() {
    const src = this.active ? this.touch : this.external;
    const r = this._radius();
    const x = src.x * r.x;
    const y = this.horizontal ? 0 : -src.y * r.y;
    this.knob.style.transform = `translate(${x}px, ${y}px)`;
  }

  _radius() {
    const b = this.el.getBoundingClientRect();
    const knob = this.knob.offsetWidth;
    return { x: (b.width - knob) / 2, y: (b.height - knob) / 2 };
  }

  _down(e) {
    this.pointer = e.pointerId;
    this.el.setPointerCapture(e.pointerId);
    this.el.classList.add("active");
    this._move(e);
  }

  _move(e) {
    if (e.pointerId !== this.pointer) return;
    const b = this.el.getBoundingClientRect();
    const r = this._radius();
    let x = (e.clientX - (b.left + b.width / 2)) / r.x;
    let y = -(e.clientY - (b.top + b.height / 2)) / (this.horizontal ? Infinity : r.y);
    const n = Math.hypot(x, y);
    if (n > 1) {
      x /= n;
      y /= n;
    }
    this.touch = { x, y };
  }

  _up(e) {
    if (e.pointerId !== this.pointer) return;
    this.pointer = null;
    this.touch = { x: 0, y: 0 };
    this.el.classList.remove("active");
  }
}

function dead(v) {
  if (Math.abs(v) < DEADZONE) return 0;
  return Math.sign(v) * (Math.abs(v) - DEADZONE) / (1 - DEADZONE);
}
