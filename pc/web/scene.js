// Vue 3D du robot, dessinée d'après les pièces de hexapod-v2-7697 : châssis
// en croix, servos MG92B, cuisse, fémur à deux servos et tibia en treillis.
// Les pièces suivent les longueurs reçues (GEOM) : si le robot est agrandi,
// le modèle l'est aussi. Les pattes bougent avec les angles de la télémétrie
// (même convention que hexapod/leg_ik.py).

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { CSS2DObject, CSS2DRenderer } from "three/addons/renderers/CSS2DRenderer.js";

const REF_TIBIA = 89.07; // longueur du tibia de hexapod-v2-7697, sert d'échelle
const TRACE_POINTS = 150; // ~2,5 s d'historique par pied à 60 images/s

const COLORS = {
  frame: 0xb03aa8, // PLA magenta, comme les rendus du projet d'origine
  servo: 0x3550c8,
  servoDark: 0x1b1e26,
  horn: 0xe9e9ef,
  rubber: 0x23262d,
  selected: 0xfbbf24,
  limit: 0xef4444,
  contact: 0x34d399,
  swing: 0x60a5fa,
};

// Repère robot (x avant, y gauche, z haut) -> repère three.js (x droite, y haut, z arrière).
const ROBOT_TO_THREE = new THREE.Matrix4().set(0, -1, 0, 0, 0, 0, 1, 0, -1, 0, 0, 0, 0, 0, 0, 1);

export class RobotView {
  constructor(container) {
    this.container = container;
    this.geom = null;
    this.legs = [];
    this.parts = []; // { mesh, leg, link, servo, role }
    this.k = 1;

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer = renderer;

    // Étiquettes HTML (câblage des servos) par-dessus la 3D.
    this.labelRenderer = new CSS2DRenderer();
    this.labelRenderer.domElement.className = "labels";
    container.prepend(this.labelRenderer.domElement);
    container.prepend(renderer.domElement);
    this.showLabels = false;
    this.selection = null; // [nom de patte, articulation 0 coxa / 1 fémur / 2 tibia]
    this.focus = false; // atténue les autres pattes pour faire ressortir la sélection
    this.limits = new Set(); // servos en butée, "RF:1" = fémur avant droit
    this.showTraces = false; // trajectoire récente de chaque pied
    this.raycaster = new THREE.Raycaster();
    this._styleKey = "";

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0b0e13);
    scene.fog = new THREE.Fog(0x0b0e13, 600, 1400);
    this.scene = scene;

    scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x1a1f29, 1.5));
    const sun = new THREE.DirectionalLight(0xffffff, 2.4);
    sun.position.set(180, 420, 220);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    sun.shadow.bias = -0.0005;
    Object.assign(sun.shadow.camera, { left: -300, right: 300, top: 300, bottom: -300 });
    scene.add(sun);
    const rim = new THREE.DirectionalLight(0x9d7cff, 0.9);
    rim.position.set(-300, 150, -300);
    scene.add(rim);

    this.ground = new THREE.Group();
    const floor = new THREE.Mesh(
      new THREE.CircleGeometry(1400, 64),
      new THREE.MeshStandardMaterial({ color: 0x0f131a, roughness: 1 }),
    );
    floor.rotation.x = -Math.PI / 2;
    floor.receiveShadow = true;
    this.ground.add(floor);
    scene.add(this.ground);

    // Repère du sol : le robot reste au centre de la vue et c'est le sol qui
    // défile quand il marche (odométrie d'après les consignes). La grille et les
    // traces des pieds y sont accrochées.
    this.odo = { x: 0, y: 0, th: 0 };
    this.track = new THREE.Group();
    this.track.matrixAutoUpdate = false;
    scene.add(this.track);
    this.grid = new THREE.GridHelper(2000, 80, 0x2a3446, 0x1a212d);
    this.grid.rotation.x = Math.PI / 2; // grille dans le plan xy du repère robot
    this.track.add(this.grid);

    // Tout le robot est construit dans son propre repère.
    this.robot = new THREE.Group();
    this.robot.matrixAutoUpdate = false;
    this.robot.matrix.copy(ROBOT_TO_THREE);
    scene.add(this.robot);

    this.camera = new THREE.PerspectiveCamera(40, 1, 1, 5000);
    this.controls = new OrbitControls(this.camera, renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.maxPolarAngle = Math.PI * 0.49;
    this.controls.addEventListener("start", () => (this.cameraGoal = null));
    this.resetCamera();

    const std = (color, extra = {}) => new THREE.MeshStandardMaterial({ color, roughness: 0.55, metalness: 0.05, ...extra });
    const dim = (color) => std(color, { transparent: true, opacity: 0.14, depthWrite: false });
    const sel = (color) => std(color, { emissive: 0x6b4a00, emissiveIntensity: 0.9 });
    this.mats = {
      frame: std(COLORS.frame),
      servo: std(COLORS.servo, { roughness: 0.35 }),
      servoDark: std(COLORS.servoDark, { roughness: 0.5 }),
      horn: std(COLORS.horn, { roughness: 0.3 }),
      rubber: std(COLORS.rubber, { roughness: 0.9 }),
      frameDim: dim(COLORS.frame),
      servoDim: dim(COLORS.servo),
      darkDim: dim(COLORS.servoDark),
      hornDim: dim(COLORS.horn),
      frameSel: std(0xe98a2c, { emissive: 0x3a1800 }),
      servoSel: sel(COLORS.selected),
      servoLimit: std(COLORS.limit, { emissive: 0x5a0a0a, emissiveIntensity: 1 }),
      contact: new THREE.MeshBasicMaterial({ color: COLORS.contact, transparent: true, opacity: 0.55 }),
      halo: new THREE.MeshBasicMaterial({ color: COLORS.selected, transparent: true, opacity: 0.22, depthWrite: false }),
      trace: new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.9 }),
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

  // ------------------------------------------------------------- construction

  setGeometry(geom) {
    this.geom = geom;
    this.robot.clear();
    this.parts = [];
    for (const leg of this.legs) this.scene.remove(leg.contact);
    const k = geom.tibia / REF_TIBIA;
    this.k = k;

    this._buildBody(geom, k);
    this.legs = geom.legs.map((g) => this._buildLeg(geom, g, k));
    this._styleKey = "";
    this.resetCamera();
  }

  _add(parent, mesh, info) {
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    parent.add(mesh);
    if (info) {
      mesh.userData = { leg: info.leg, joint: info.servo !== undefined ? info.servo : info.link };
      this.parts.push({ mesh, ...info });
    }
    return mesh;
  }

  _buildBody(geom, k) {
    const body = new THREE.Group();
    this.robot.add(body);
    const info = { leg: null, link: -1, role: "frame" };
    const t = 3 * k;
    // Les servos de coxa sont pris entre les deux plaques du corps, leur axe sur
    // la fixation de chaque patte : le contour des plaques passe autour des axes.
    const zTop = 8 * k;
    const zBottom = -16 * k - t;
    const order = ["RF", "RM", "RR", "LR", "LM", "LF"].map((n) => geom.legs.findIndex((l) => l.name === n));
    const mounts = order.map((i) => new THREE.Vector2(geom.legs[i].x, geom.legs[i].y));
    const grow = (p, d) => p.clone().multiplyScalar(1 + d / p.length());

    for (const z of [zTop, zBottom]) {
      const plate = new THREE.Shape(mounts.map((p) => grow(p, 12 * k)));
      // Évidements triangulaires entre le centre et chaque paire de fixations.
      mounts.forEach((p, i) => {
        const q = mounts[(i + 1) % mounts.length];
        const a = grow(p, -32 * k);
        const b = grow(q, -32 * k);
        const c = a.clone().add(b).multiplyScalar(0.3);
        if (a.distanceTo(b) > 16 * k) plate.holes.push(new THREE.Path([c, b, a]));
      });
      mounts.forEach((p) => plate.holes.push(new THREE.Path().absarc(p.x, p.y, 2 * k, 0, Math.PI * 2, true)));
      const m = this._add(body, extrude(plate, t), info);
      m.position.z = z;
    }

    // Entretoises entre les plaques, à côté de chaque servo de coxa.
    for (const p of mounts) {
      const post = this._add(body, new THREE.Mesh(new THREE.CylinderGeometry(2.5 * k, 2.5 * k, zTop - zBottom - t, 12), this.mats.frame), info);
      const q = grow(p, -30 * k);
      post.rotation.x = Math.PI / 2;
      post.position.set(q.x, q.y, (zTop + zBottom + t) / 2);
    }

    // Batterie (LiPo 2S) au centre, sous le châssis.
    const battery = this._add(body, new THREE.Mesh(new THREE.BoxGeometry(48 * k, 26 * k, 14 * k), this.mats.servoDark));
    battery.position.z = zBottom - 7 * k;
  }

  _buildLeg(geom, g, k) {
    const leg = { name: g.name, labels: [] };
    const root = new THREE.Group();
    root.position.set(g.x, g.y, 0);
    root.rotation.z = g.angle;
    this.robot.add(root);

    const coxaJ = new THREE.Group();
    root.add(coxaJ);
    const femurJ = new THREE.Group();
    femurJ.position.x = geom.coxa;
    coxaJ.add(femurJ);
    const tibiaJ = new THREE.Group();
    tibiaJ.position.x = geom.femur;
    femurJ.add(tibiaJ);
    const tip = new THREE.Group();
    tip.position.x = geom.tibia;
    tibiaJ.add(tip);
    Object.assign(leg, { root, joints: [coxaJ, femurJ, tibiaJ], tip });

    const t = 3 * k;
    const info = (link) => ({ leg: g.name, link, role: "frame" });
    const X = new THREE.Vector3(1, 0, 0);
    const Y = new THREE.Vector3(0, 1, 0);
    const Z = new THREE.Vector3(0, 0, 1);

    // --- servo de coxa fixé dans le corps, boîtier vers l'intérieur ; la cuisse
    // serre le palonnier au-dessus et l'axe libre en dessous, puis porte la chape du fémur.
    this._servo(root, g.name, 0, Z, X.clone().negate(), new THREE.Vector3(0, 0, 11 * k), k, 0);
    for (const z of [15 * k, -22 * k - t]) {
      const c = geom.coxa;
      const s = new THREE.Shape();
      s.moveTo(0, 9 * k);
      s.absarc(0, 0, 9 * k, Math.PI / 2, Math.PI * 1.5, false);
      s.lineTo(c - 18 * k, -9 * k);
      s.lineTo(c - 8 * k, -20 * k);
      s.lineTo(c - 4 * k, -20 * k);
      s.lineTo(c - 4 * k, 20 * k);
      s.lineTo(c - 8 * k, 20 * k);
      s.lineTo(c - 18 * k, 9 * k);
      s.closePath();
      s.holes.push(new THREE.Path().absarc(0, 0, 2 * k, 0, Math.PI * 2, true));
      const m = this._add(coxaJ, extrude(s, t), info(0));
      m.position.z = z;
    }
    for (const side of [1, -1]) {
      const yoke = new THREE.Shape();
      // Flanc de la chape, du dessus au dessous de la cuisse, rond autour de l'axe du fémur.
      yoke.moveTo(geom.coxa - 8 * k, -22 * k - t);
      yoke.lineTo(geom.coxa - 2 * k, -22 * k - t);
      yoke.lineTo(geom.coxa, -10 * k);
      yoke.absarc(geom.coxa, 0, 10 * k, -Math.PI / 2, Math.PI / 2, false);
      yoke.lineTo(geom.coxa - 2 * k, 15 * k + t);
      yoke.lineTo(geom.coxa - 8 * k, 15 * k + t);
      yoke.closePath();
      yoke.holes.push(new THREE.Path().absarc(geom.coxa, 0, 2.5 * k, 0, Math.PI * 2, true));
      const m = this._add(coxaJ, sidePlate(yoke, t), info(0));
      m.position.y = side * 17 * k + (side > 0 ? t : 0);
    }

    // --- fémur : deux servos (fémur et tibia) tenus entre deux flancs ajourés
    this._servo(femurJ, g.name, 1, Y, X, new THREE.Vector3(0, 12 * k, 0), k, 1);
    this._servo(femurJ, g.name, 2, Y.clone().negate(), X.clone().negate(), new THREE.Vector3(geom.femur, -12 * k, 0), k, 1);
    const Lf = geom.femur;
    for (const side of [1, -1]) {
      const flank = roundedSlot(0, Lf, 9 * k);
      flank.holes.push(new THREE.Path().absarc(0, 0, 2.2 * k, 0, Math.PI * 2, true));
      flank.holes.push(new THREE.Path().absarc(Lf, 0, 2.2 * k, 0, Math.PI * 2, true));
      if (Lf > 30 * k) {
        flank.holes.push(rectPath(Lf * 0.24, -5 * k, Lf * 0.47, 5 * k));
        flank.holes.push(rectPath(Lf * 0.53, -5 * k, Lf * 0.76, 5 * k));
      }
      const m = this._add(femurJ, sidePlate(flank, t), info(1));
      m.position.y = side * 13.5 * k + (side > 0 ? t : 0);
    }

    // --- tibia : treillis effilé, deux flancs reliés par des traverses, embout caoutchouc
    const L = geom.tibia;
    const half = (u) => (u < 0.3 ? 11 * k : 11 * k - ((u - 0.3) / 0.7) * 8.5 * k);
    const truss = new THREE.Shape();
    truss.moveTo(0, -11 * k);
    truss.lineTo(L * 0.3, -half(0.3));
    truss.lineTo(L, -half(1));
    truss.lineTo(L, half(1));
    truss.lineTo(L * 0.3, half(0.3));
    truss.lineTo(0, 11 * k);
    truss.absarc(0, 0, 11 * k, Math.PI / 2, Math.PI * 1.5, false);
    truss.holes.push(new THREE.Path().absarc(0, 0, 2.5 * k, 0, Math.PI * 2, true));
    const nodes = [0.17, 0.37, 0.56, 0.74, 0.9];
    for (let i = 0; i < nodes.length - 1; i++) {
      const a = nodes[i] * L + 2 * k;
      const b = nodes[i + 1] * L - 2 * k;
      const ha = half(nodes[i]) * 0.62;
      const hb = half(nodes[i + 1]) * 0.62;
      const s = i % 2 === 0 ? 1 : -1;
      truss.holes.push(new THREE.Path().moveTo(a, -s * ha).lineTo(b, -s * hb).lineTo((a + b) / 2, s * (ha + hb) / 2).closePath());
    }
    for (const side of [1, -1]) {
      const m = this._add(tibiaJ, sidePlate(truss, t), info(2));
      m.position.y = side * 17 * k + (side > 0 ? t : 0);
    }
    const bar = this._add(tibiaJ, new THREE.Mesh(new THREE.BoxGeometry(4 * k, 34 * k + t, 4 * k), this.mats.frame), info(2));
    bar.position.set(0.47 * L, 0, -half(0.47) * 0.75);
    const tipBar = this._add(tibiaJ, new THREE.Mesh(new THREE.BoxGeometry(5 * k, 34 * k + t, 5 * k), this.mats.frame), info(2));
    tipBar.position.set(L - 3 * k, 0, 0);
    const foot = this._add(tip, new THREE.Mesh(new THREE.SphereGeometry(4.5 * k, 16, 12), this.mats.rubber), { leg: g.name, link: 2, role: "rubber" });
    foot.scale.set(1.2, 1, 1);

    // Repère au sol sous le pied.
    leg.contact = new THREE.Mesh(new THREE.RingGeometry(5 * k, 9 * k, 28), this.mats.contact);
    leg.contact.rotation.x = -Math.PI / 2;
    this.scene.add(leg.contact);

    // Trace du pied : les dernières positions, vertes au sol, bleues en l'air.
    const traceGeo = new THREE.BufferGeometry();
    traceGeo.setAttribute("position", new THREE.BufferAttribute(new Float32Array(TRACE_POINTS * 3), 3));
    traceGeo.setAttribute("color", new THREE.BufferAttribute(new Float32Array(TRACE_POINTS * 3), 3));
    traceGeo.setDrawRange(0, 0);
    leg.trace = new THREE.Line(traceGeo, this.mats.trace);
    leg.trace.frustumCulled = false;
    leg.trace.visible = false;
    leg.traceCount = 0;
    this.track.add(leg.trace);

    leg.halo = new THREE.Mesh(new THREE.SphereGeometry(20 * k, 24, 16), this.mats.halo);
    leg.halo.visible = false;
    this.robot.add(leg.halo);

    // Étiquettes carte · canal, posées sur l'axe de chaque servo.
    const hex = (n) => "0x" + n.toString(16);
    leg.labels = leg.joints.map((joint, j) => {
      const [board, channel] = g.servos[j];
      const el = document.createElement("div");
      el.className = "servo-label";
      el.innerHTML = `<b>${hex(geom.addresses[board])}</b> · ${channel}`;
      const label = new CSS2DObject(el);
      label.position.set(0, 0, 24 * k);
      joint.add(label);
      return label;
    });
    return leg;
  }

  /** Servo MG92B : axe de sortie le long de `axis` passant par `at`, boîtier allongé vers `bodyDir`. */
  _servo(parent, legName, servo, axis, bodyDir, at, k, link) {
    const group = new THREE.Group();
    const x = bodyDir.clone();
    const y = axis.clone().cross(x);
    group.matrix.makeBasis(x, y, axis).setPosition(at);
    group.matrixAutoUpdate = false;
    parent.add(group);
    const info = (role) => ({ leg: legName, link, servo, role });
    // Dimensions MG92B (mm) : boîtier 22.8 x 12 x 22.5, pattes de fixation 32.5, axe à 5.6 du centre.
    const box = (sx, sy, sz, cz, mat, role) => {
      const m = this._add(group, new THREE.Mesh(new THREE.BoxGeometry(sx * k, sy * k, sz * k), mat), info(role));
      m.position.set(5.6 * k, 0, cz * k);
      return m;
    };
    box(22.8, 12, 22.5, -13.5, this.mats.servo, "servo");
    box(32.5, 12, 1.8, -5.5, this.mats.servoDark, "servoDark");
    box(22.8, 12, 2.2, -1.4, this.mats.servoDark, "servoDark");
    const cyl = (r, h, z, mat, role) => {
      const m = this._add(group, new THREE.Mesh(new THREE.CylinderGeometry(r * k, r * k, h * k, 28), mat), info(role));
      m.rotation.x = Math.PI / 2;
      m.position.z = z * k;
    };
    cyl(5.8, 2, 0.4, this.mats.servo, "servo");
    cyl(7, 1.8, 2.2, this.mats.horn, "horn");
    cyl(2, 1.2, 3.6, this.mats.servoDark, "servoDark");
    return group;
  }

  // ------------------------------------------------------------- mise à jour

  /** Servo sous le pointeur (coordonnées écran), ou null. */
  pick(clientX, clientY) {
    const b = this.renderer.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(((clientX - b.left) / b.width) * 2 - 1, -((clientY - b.top) / b.height) * 2 + 1);
    this.raycaster.setFromCamera(ndc, this.camera);
    const meshes = this.parts.filter((p) => p.leg).map((p) => p.mesh);
    const hit = this.raycaster.intersectObjects(meshes, false)[0];
    if (!hit) return null;
    // Un servo touché désigne ce servo, une pièce désigne le servo qui la fait bouger.
    return { ...hit.object.userData };
  }

  /** Fait tourner la caméra pour montrer la patte `name` de face. */
  lookAtLeg(name) {
    const leg = this.geom && this.geom.legs.find((l) => l.name === name);
    if (!leg) return;
    const s = this.span();
    const out = new THREE.Vector3(Math.cos(leg.angle), Math.sin(leg.angle), 0).applyMatrix4(ROBOT_TO_THREE);
    this.cameraGoal = out.multiplyScalar(s * 2.1).add(new THREE.Vector3(0, s * 1.2, 0));
  }

  _applyStyles() {
    const sel = this.selection;
    const key = JSON.stringify([sel, this.focus, [...this.limits].sort()]);
    if (key === this._styleKey) return;
    this._styleKey = key;
    const m = this.mats;
    for (const p of this.parts) {
      const mine = sel && p.leg === sel[0];
      const dimmed = this.focus && sel && !mine;
      const lit = mine && (p.servo !== undefined ? p.servo === sel[1] : p.link === sel[1]);
      const atLimit = p.servo !== undefined && this.limits.has(p.leg + ":" + p.servo);
      let mat;
      if (p.role === "frame") mat = dimmed ? m.frameDim : lit ? m.frameSel : m.frame;
      else if (p.role === "servo" && atLimit && !dimmed) mat = m.servoLimit;
      else if (p.role === "servo" || p.role === "horn") mat = dimmed ? (p.role === "horn" ? m.hornDim : m.servoDim) : lit ? m.servoSel : m[p.role];
      else if (p.role === "rubber") mat = dimmed ? m.darkDim : m.rubber;
      else mat = dimmed ? m.darkDim : m.servoDark;
      p.mesh.material = mat;
    }
  }

  update(angles) {
    if (!this.geom || !angles || !this.legs.length) return;
    this._applyStyles();
    this.legs.forEach((leg, i) => {
      const [c, f, t] = angles[i];
      const [coxaJ, femurJ, tibiaJ] = leg.joints;
      coxaJ.rotation.z = c;
      femurJ.rotation.y = -f; // positif = fémur qui monte
      tibiaJ.rotation.y = -t;
    });
    this.robot.updateMatrixWorld(true);

    const tips = this.legs.map((leg) => leg.tip.getWorldPosition(new THREE.Vector3()));
    const footR = 4.5 * this.k;
    const lowest = Math.min(...tips.map((p) => p.y)) - footR;
    this.ground.position.y = lowest;
    this._updateTrack(lowest);
    const tolerance = Math.max(1.5, 0.02 * this.geom.tibia);
    const sel = this.selection;

    this.legs.forEach((leg, i) => {
      const onGround = tips[i].y - footR - lowest < tolerance;
      leg.contact.visible = onGround;
      leg.contact.position.set(tips[i].x, lowest + 0.5, tips[i].z);
      const j = sel && sel[0] === leg.name ? sel[1] : -1;
      leg.labels.forEach((label, n) => {
        const atLimit = this.limits.has(leg.name + ":" + n);
        label.visible = this.showLabels || n === j || atLimit;
        label.element.classList.toggle("lit", n === j);
        label.element.classList.toggle("limit", atLimit);
      });
      this._trace(leg, tips[i], onGround);
      leg.halo.visible = j >= 0;
      if (j >= 0) {
        const p = leg.joints[j].getWorldPosition(new THREE.Vector3());
        leg.halo.position.copy(this.robot.worldToLocal(p));
        leg.halo.scale.setScalar(1 + 0.12 * Math.sin(performance.now() / 180));
      }
    });
  }

  /** Avance le robot sur le sol : vx, vy en mm/s (repère robot), lacet en rad/s. */
  move(vx, vy, yawRate, dt) {
    const o = this.odo;
    o.th += yawRate * dt;
    o.x += (Math.cos(o.th) * vx - Math.sin(o.th) * vy) * dt;
    o.y += (Math.sin(o.th) * vx + Math.cos(o.th) * vy) * dt;
  }

  _updateTrack(groundY) {
    const o = this.odo;
    const pose = new THREE.Matrix4().makeRotationZ(o.th).setPosition(o.x, o.y, 0);
    this.track.matrix.copy(ROBOT_TO_THREE).multiply(pose.invert());
    this.track.matrixWorldNeedsUpdate = true;
    // La grille suit le robot par pas entiers de case, pour paraître infinie.
    const cell = 25;
    this.grid.position.set(Math.round(o.x / cell) * cell, Math.round(o.y / cell) * cell, groundY + 0.2);
    this.track.updateMatrixWorld(true);
  }

  _trace(leg, tipWorld, onGround) {
    leg.trace.visible = this.showTraces;
    if (!this.showTraces) {
      leg.traceCount = 0;
      return;
    }
    const pos = leg.trace.geometry.attributes.position;
    const col = leg.trace.geometry.attributes.color;
    // Décale le tampon d'un point quand il est plein, puis ajoute la position actuelle.
    if (leg.traceCount === TRACE_POINTS) {
      pos.array.copyWithin(0, 3);
      col.array.copyWithin(0, 3);
    } else {
      leg.traceCount++;
    }
    const at = (leg.traceCount - 1) * 3;
    const p = this.track.worldToLocal(tipWorld.clone());
    pos.array.set([p.x, p.y, p.z - 4 * this.k], at);
    const c = new THREE.Color(onGround ? COLORS.contact : COLORS.swing);
    col.array.set([c.r, c.g, c.b], at);
    leg.trace.geometry.setDrawRange(0, leg.traceCount);
    pos.needsUpdate = true;
    col.needsUpdate = true;
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
    if (this.cameraGoal) {
      this.camera.position.lerp(this.cameraGoal, 0.12);
      if (this.camera.position.distanceTo(this.cameraGoal) < 1) this.cameraGoal = null;
    }
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    this.labelRenderer.render(this.scene, this.camera);
  }
}

// ----------------------------------------------------------------- formes

const extrudeOpts = (depth) => ({ depth, bevelEnabled: true, bevelThickness: depth * 0.2, bevelSize: depth * 0.2, bevelSegments: 1, curveSegments: 18 });

/** Plaque horizontale (plan xy du lien), épaisseur vers +z. */
function extrude(shape, t) {
  return new THREE.Mesh(new THREE.ExtrudeGeometry(shape, extrudeOpts(t * 0.6)));
}

/** Flanc vertical : la forme est dessinée dans le plan (x, z) du lien, épaisseur vers -y. */
function sidePlate(shape, t) {
  const geo = new THREE.ExtrudeGeometry(shape, extrudeOpts(t * 0.6));
  geo.rotateX(Math.PI / 2);
  return new THREE.Mesh(geo);
}

/** Barre aux bouts arrondis de x0 à x1, demi-largeur r. */
function roundedSlot(x0, x1, r) {
  const s = new THREE.Shape();
  s.moveTo(x0, -r);
  s.lineTo(x1, -r);
  s.absarc(x1, 0, r, -Math.PI / 2, Math.PI / 2, false);
  s.lineTo(x0, r);
  s.absarc(x0, 0, r, Math.PI / 2, Math.PI * 1.5, false);
  return s;
}

function rectPath(x0, y0, x1, y1) {
  return new THREE.Path().moveTo(x0, y0).lineTo(x0, y1).lineTo(x1, y1).lineTo(x1, y0).closePath();
}
