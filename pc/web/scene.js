// Vue 3D du robot : reconstruit les pattes à partir des angles articulaires
// reçus en télémétrie (même FK que hexapod/leg_ik.py).

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { CSS2DObject, CSS2DRenderer } from "three/addons/renderers/CSS2DRenderer.js";

// Repère robot : x avant, y gauche, z haut -> three.js : x droite, y haut, z arrière.
const v3 = (x, y, z) => new THREE.Vector3(-y, z, -x);

const COLORS = {
  leg: 0x4f8cff,
  legSwing: 0x8ab0ff,
  selected: 0xfbbf24,
  body: 0x283041,
  bodyTop: 0x3a4458,
  joint: 0xe6ebf5,
  contact: 0x34d399,
};

export class RobotView {
  constructor(container) {
    this.container = container;
    this.geom = null;
    this.legs = [];
    this.selected = null;

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer = renderer;

    // Étiquettes HTML (câblage des servos) par-dessus la 3D.
    this.labelRenderer = new CSS2DRenderer();
    this.labelRenderer.domElement.className = "labels";
    container.prepend(this.labelRenderer.domElement);
    container.prepend(renderer.domElement);
    this.showLabels = false;
    this.highlight = null; // [nom de patte, index d'articulation]

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0b0e13);
    scene.fog = new THREE.Fog(0x0b0e13, 600, 1400);
    this.scene = scene;

    scene.add(new THREE.HemisphereLight(0xcfe0ff, 0x1a1f29, 1.6));
    const sun = new THREE.DirectionalLight(0xffffff, 2.2);
    sun.position.set(180, 420, 220);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    Object.assign(sun.shadow.camera, { left: -300, right: 300, top: 300, bottom: -300 });
    scene.add(sun);
    const rim = new THREE.DirectionalLight(0x7c5cff, 0.8);
    rim.position.set(-300, 150, -300);
    scene.add(rim);

    this.ground = new THREE.Group();
    const floor = new THREE.Mesh(
      new THREE.CircleGeometry(1400, 64),
      new THREE.MeshStandardMaterial({ color: 0x0f131a, roughness: 1 }),
    );
    floor.rotation.x = -Math.PI / 2;
    floor.receiveShadow = true;
    const grid = new THREE.GridHelper(2000, 80, 0x2a3446, 0x1a212d);
    grid.position.y = 0.2;
    this.ground.add(floor, grid);
    scene.add(this.ground);

    this.robot = new THREE.Group();
    scene.add(this.robot);

    this.camera = new THREE.PerspectiveCamera(42, 1, 1, 5000);
    this.controls = new OrbitControls(this.camera, renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.maxPolarAngle = Math.PI * 0.49;
    this.resetCamera();

    this.mats = {
      leg: new THREE.MeshStandardMaterial({ color: COLORS.leg, roughness: 0.35, metalness: 0.15 }),
      swing: new THREE.MeshStandardMaterial({ color: COLORS.legSwing, roughness: 0.35, metalness: 0.15 }),
      selected: new THREE.MeshStandardMaterial({ color: COLORS.selected, roughness: 0.35, emissive: 0x3a2a00 }),
      joint: new THREE.MeshStandardMaterial({ color: COLORS.joint, roughness: 0.4, metalness: 0.3 }),
      contact: new THREE.MeshBasicMaterial({ color: COLORS.contact, transparent: true, opacity: 0.55 }),
    };

    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
  }

  span() {
    if (!this.geom) return 250;
    const g = this.geom;
    return Math.max(...g.legs.map((l) => Math.hypot(l.x, l.y))) + g.coxa + g.femur + g.tibia;
  }

  resetCamera(top = false) {
    const s = this.span();
    if (top) this.camera.position.set(0, s * 3, 0.01);
    else this.camera.position.set(s * 1.3, s * 1.05, s * 1.6);
    this.controls.target.set(0, -s * 0.1, 0);
    this.controls.update();
  }

  setGeometry(geom) {
    this.geom = geom;
    this.robot.clear();
    const order = ["RF", "RM", "RR", "LR", "LM", "LF"];
    const byName = Object.fromEntries(geom.legs.map((l) => [l.name, l]));
    const shape = new THREE.Shape(order.map((n) => new THREE.Vector2(-byName[n].body_y, byName[n].body_x)));
    const thick = Math.max(12, 0.22 * geom.tibia);

    const body = new THREE.Mesh(
      new THREE.ExtrudeGeometry(shape, { depth: thick, bevelEnabled: true, bevelSize: 3, bevelThickness: 3, bevelSegments: 3 }),
      new THREE.MeshStandardMaterial({ color: COLORS.body, roughness: 0.5, metalness: 0.4 }),
    );
    body.rotation.x = -Math.PI / 2;
    body.position.y = -thick / 2;
    body.castShadow = true;
    this.robot.add(body);

    const top = new THREE.Mesh(
      new THREE.ShapeGeometry(shape),
      new THREE.MeshStandardMaterial({ color: COLORS.bodyTop, roughness: 0.3, metalness: 0.5 }),
    );
    top.rotation.x = -Math.PI / 2;
    top.position.y = thick / 2 + 3.2;
    top.scale.setScalar(0.82);
    this.robot.add(top);

    // Flèche vers l'avant, posée sur le dessus.
    const len = 0.55 * Math.max(...geom.legs.map((l) => l.body_x));
    const arrow = new THREE.Shape([
      new THREE.Vector2(0, len), new THREE.Vector2(len * 0.35, len * 0.35), new THREE.Vector2(len * 0.12, len * 0.35),
      new THREE.Vector2(len * 0.12, -len * 0.4), new THREE.Vector2(-len * 0.12, -len * 0.4),
      new THREE.Vector2(-len * 0.12, len * 0.35), new THREE.Vector2(-len * 0.35, len * 0.35),
    ]);
    const arrowMesh = new THREE.Mesh(new THREE.ShapeGeometry(arrow), new THREE.MeshBasicMaterial({ color: COLORS.selected }));
    arrowMesh.rotation.x = -Math.PI / 2;
    arrowMesh.position.y = thick / 2 + 3.4;
    this.robot.add(arrowMesh);

    const r = Math.max(3, 0.055 * geom.tibia);
    const hex = (n) => "0x" + n.toString(16);
    this.legs = geom.legs.map((g) => {
      const leg = {
        segs: [this._segment(r * 0.8), this._segment(r * 1.1), this._segment(r), this._segment(r * 0.75)],
        joints: [0, 1, 2].map(() => this._sphere(r * 1.35, this.mats.joint)),
        labels: [],
        foot: this._sphere(r * 0.9, this.mats.joint),
        contact: new THREE.Mesh(new THREE.RingGeometry(r * 1.4, r * 2.4, 24), this.mats.contact),
      };
      leg.contact.rotation.x = -Math.PI / 2;
      this.scene.add(leg.contact);
      leg.labels = leg.joints.map((sphere, j) => {
        const [board, channel] = g.servos[j];
        const el = document.createElement("div");
        el.className = "servo-label";
        el.innerHTML = `<b>${hex(geom.addresses[board])}</b> · ${channel}`;
        const label = new CSS2DObject(el);
        label.position.set(0, r * 3, 0);
        sphere.add(label);
        return label;
      });
      return leg;
    });
    this.resetCamera();
  }

  _segment(radius) {
    const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, 1, 16), this.mats.leg);
    mesh.castShadow = true;
    this.robot.add(mesh);
    return mesh;
  }

  _sphere(radius, mat) {
    const mesh = new THREE.Mesh(new THREE.SphereGeometry(radius, 16, 12), mat);
    mesh.castShadow = true;
    this.robot.add(mesh);
    return mesh;
  }

  _place(mesh, a, b) {
    const dir = new THREE.Vector3().subVectors(b, a);
    const len = Math.max(dir.length(), 0.01);
    mesh.position.copy(a).addScaledVector(dir, 0.5);
    mesh.scale.set(1, len, 1);
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.normalize());
  }

  legPoints(leg, [c, f, t]) {
    const g = this.geom;
    const kr = g.coxa + g.femur * Math.cos(f);
    const kz = g.femur * Math.sin(f);
    const fr = kr + g.tibia * Math.cos(f + t);
    const fz = kz + g.tibia * Math.sin(f + t);
    const ca = Math.cos(leg.angle + c);
    const sa = Math.sin(leg.angle + c);
    const p = (r, z) => v3(leg.x + r * ca, leg.y + r * sa, z);
    return [v3(leg.body_x, leg.body_y, 0), p(0, 0), p(g.coxa, 0), p(kr, kz), p(fr, fz)];
  }

  update(angles, highlight) {
    if (!this.geom || !angles || !this.legs.length) return;
    const all = this.geom.legs.map((leg, i) => this.legPoints(leg, angles[i]));
    const lowest = Math.min(...all.map((pts) => pts[4].y));
    this.ground.position.y = lowest;
    const tolerance = Math.max(1.5, 0.02 * this.geom.tibia);

    this.geom.legs.forEach((leg, i) => {
      const pts = all[i];
      const m = this.legs[i];
      const onGround = pts[4].y - lowest < tolerance;
      const mat = leg.name === highlight ? this.mats.selected : onGround ? this.mats.leg : this.mats.swing;
      m.segs.forEach((seg, j) => {
        seg.material = mat;
        this._place(seg, pts[j], pts[j + 1]);
      });
      m.joints.forEach((s, j) => {
        s.position.copy(pts[j + 1]);
        const lit = this.highlight && this.highlight[0] === leg.name && this.highlight[1] === j;
        s.material = lit ? this.mats.selected : this.mats.joint;
        s.scale.setScalar(lit ? 1.6 : 1);
        m.labels[j].visible = this.showLabels;
        m.labels[j].element.classList.toggle("lit", lit);
      });
      m.foot.position.copy(pts[4]);
      m.contact.visible = onGround;
      m.contact.position.set(pts[4].x, lowest + 0.5, pts[4].z);
    });
  }

  resize() {
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;
    this.renderer.setSize(w, h);
    this.labelRenderer.setSize(w, h);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  render() {
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    this.labelRenderer.render(this.scene, this.camera);
  }
}
