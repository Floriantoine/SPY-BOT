import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const $ = (s) => document.querySelector(s);
const $$ = (s) => document.querySelectorAll(s);

// ---------------------------------------------------------------- liaison

async function post(path, body) {
  const res = await fetch(path, { method: "POST", body });
  const data = await res.json();
  if (data.error) log("! " + data.error);
  return data;
}

const send = (line) => post("/cmd", line);

function log(line) {
  const el = $("#log");
  el.textContent += line + "\n";
  const lines = el.textContent.split("\n");
  if (lines.length > 200) el.textContent = lines.slice(-200).join("\n");
  el.scrollTop = el.scrollHeight;
}

async function refreshPorts() {
  const { ports, link } = await (await fetch("/ports")).json();
  const sel = $("#port");
  sel.innerHTML = "";
  for (const p of ports) sel.add(new Option(`${p.device} — ${p.description}`, p.device));
  sel.add(new Option("Simulation (sans robot)", "sim"));
  if (link) sel.value = link;
}

$("#refresh").onclick = refreshPorts;
$("#connect").onclick = () => post("/connect", JSON.stringify({ port: $("#port").value }));
$("#disconnect").onclick = () => post("/disconnect");
for (const b of $$("button[data-cmd]")) b.onclick = () => send(b.dataset.cmd);

// ------------------------------------------------------------- état robot

let geom = null;
let angles = null; // 6 x [coxa, fémur, tibia] en radians
let calib = {};
let selectedLeg = "RF";
let mode = "off";

function setLink(connected, text) {
  $("#link-dot").classList.toggle("on", connected);
  $("#link-text").textContent = text;
}

const events = new EventSource("/events");
events.onmessage = (e) => {
  const line = e.data;
  const [kind, ...rest] = line.split(" ");
  if (kind === "T") {
    mode = rest[0];
    $("#mode").textContent = MODES[mode] || mode;
    const v = rest.slice(2).map((x) => Number(x) / 1000);
    angles = [0, 1, 2, 3, 4, 5].map((i) => v.slice(3 * i, 3 * i + 3));
    return;
  }
  if (kind === "GEOM") {
    geom = JSON.parse(rest.join(" "));
    buildRobot();
  } else if (kind === "CAL") {
    calib = JSON.parse(rest.join(" "));
    showTrims();
  } else if (kind === "LINK") {
    if (rest[0] === "connected") setLink(true, "Connecté : " + (rest[1] === "sim" ? "simulation" : rest[1]));
    else setLink(false, rest[0] === "error" ? "Erreur de liaison" : "Non connecté");
  } else if (kind === "OK" && rest[0] === "TRIM") {
    const [, leg, joint, value] = rest;
    (calib[leg] ||= [0, 0, 0])[JOINTS.indexOf(joint)] = Number(value);
  }
  log(line);
};

const MODES = {
  off: "relâché",
  sit: "assis",
  standing: "se lève…",
  stand: "debout",
  sitting: "s'assoit…",
  cal: "calibration",
};

// --------------------------------------------------------------- pilotage

const pressed = new Set();
const MOVE_KEYS = { z: "z", s: "s", q: "q", d: "d", a: "a", e: "e",
  arrowup: "z", arrowdown: "s", arrowleft: "a", arrowright: "e" };

function bindSlider(input, format) {
  const out = input.parentElement.querySelector("output");
  const update = () => (out.textContent = format(Number(input.value)));
  input.addEventListener("input", update);
  update();
}

bindSlider($("#speed"), (v) => `${v} mm/s`);
bindSlider($("#turn"), (v) => `${v} °/s`);

document.addEventListener("keydown", (e) => {
  if (e.target.tagName === "SELECT") return;
  const k = MOVE_KEYS[e.key.toLowerCase()];
  if (k) {
    pressed.add(k);
    e.preventDefault();
  } else if (e.key === " ") {
    pressed.clear();
    e.preventDefault();
  }
});
document.addEventListener("keyup", (e) => {
  const k = MOVE_KEYS[e.key.toLowerCase()];
  if (k) pressed.delete(k);
});
window.addEventListener("blur", () => pressed.clear());

for (const b of $$(".pad button[data-key]")) {
  const k = b.dataset.key;
  b.addEventListener("pointerdown", () => pressed.add(k));
  for (const ev of ["pointerup", "pointerleave", "pointercancel"]) b.addEventListener(ev, () => pressed.delete(k));
}
$("#stop").onclick = () => pressed.clear();

let moving = false;
setInterval(() => {
  const speed = Number($("#speed").value);
  const turn = Number($("#turn").value);
  const has = (k) => (pressed.has(k) ? 1 : 0);
  const vx = speed * (has("z") - has("s"));
  const vy = speed * (has("q") - has("d"));
  const yaw = turn * (has("a") - has("e"));
  for (const b of $$(".pad button[data-key]")) b.classList.toggle("active", pressed.has(b.dataset.key));
  // Le robot s'arrête seul s'il ne reçoit plus de MOVE pendant 0,5 s.
  if (vx || vy || yaw) {
    send(`MOVE ${vx} ${vy} ${yaw}`);
    moving = true;
  } else if (moving) {
    send("MOVE 0 0 0");
    moving = false;
  }
}, 100);

// ------------------------------------------------------------ corps, démarche

function throttle(fn, ms) {
  let timer = null;
  let pending = false;
  return () => {
    if (timer) {
      pending = true;
      return;
    }
    fn();
    timer = setTimeout(function tick() {
      timer = null;
      if (pending) {
        pending = false;
        fn();
        timer = setTimeout(tick, ms);
      }
    }, ms);
  };
}

const poseInputs = [...$$("input.pose")].sort((a, b) => a.dataset.i - b.dataset.i);
const sendPose = throttle(() => send("POSE " + poseInputs.map((i) => i.value).join(" ")), 60);
for (const input of poseInputs) {
  bindSlider(input, (v) => (Number(input.dataset.i) < 3 ? `${v} mm` : `${v}°`));
  input.addEventListener("input", sendPose);
}
$("#pose-reset").onclick = () => {
  for (const input of poseInputs) {
    input.value = 0;
    input.dispatchEvent(new Event("input"));
  }
};

const sendGait = throttle(() => send(`GAIT ${$("#period").value} ${$("#step").value}`), 150);
bindSlider($("#period"), (v) => v.toFixed(1));
bindSlider($("#step"), (v) => `${v}`);
$("#period").addEventListener("input", sendGait);
$("#step").addEventListener("input", sendGait);

// ------------------------------------------------------------- calibration

const JOINTS = ["coxa", "femur", "tibia"];
const LEG_LABELS = { LF: "Avant G", RF: "Avant D", LM: "Milieu G", RM: "Milieu D", LR: "Arrière G", RR: "Arrière D" };

for (const name of ["LF", "RF", "LM", "RM", "LR", "RR"]) {
  const b = document.createElement("button");
  b.textContent = LEG_LABELS[name];
  b.dataset.leg = name;
  b.onclick = () => {
    selectedLeg = name;
    showTrims();
  };
  $("#legs").append(b);
}

const trimInputs = [...$$("input.trim")];
for (const input of trimInputs) {
  bindSlider(input, (v) => `${v.toFixed(1)}°`);
  const sendTrim = throttle(() => send(`TRIM ${selectedLeg} ${input.dataset.joint} ${input.value}`), 80);
  input.addEventListener("input", sendTrim);
}

function showTrims() {
  for (const b of $$("#legs button")) b.classList.toggle("active", b.dataset.leg === selectedLeg);
  const t = calib[selectedLeg] || [0, 0, 0];
  for (const input of trimInputs) {
    input.value = t[JOINTS.indexOf(input.dataset.joint)];
    input.parentElement.querySelector("output").textContent = `${Number(input.value).toFixed(1)}°`;
  }
}
showTrims();

// ---------------------------------------------------------------------- 3D

// Repère robot : x avant, y gauche, z haut -> three.js : x droite, y haut, z arrière.
const v3 = (x, y, z) => new THREE.Vector3(-y, z, -x);

const view = $("#view");
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
view.append(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x12161c);
scene.add(new THREE.HemisphereLight(0xffffff, 0x334455, 2.2));
const sun = new THREE.DirectionalLight(0xffffff, 1.5);
sun.position.set(200, 400, 300);
scene.add(sun);

const camera = new THREE.PerspectiveCamera(45, 1, 1, 5000);
camera.position.set(260, 220, 320);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0);

const grid = new THREE.GridHelper(1000, 40, 0x3a4552, 0x262e38);
scene.add(grid);

const robot = new THREE.Group();
scene.add(robot);
let legMeshes = [];

const MAT_BODY = new THREE.MeshStandardMaterial({ color: 0x4a5563, metalness: 0.2, roughness: 0.6 });
const MAT_LEG = new THREE.MeshStandardMaterial({ color: 0x3d8bfd, roughness: 0.5 });
const MAT_SEL = new THREE.MeshStandardMaterial({ color: 0xf0a030, roughness: 0.5 });
const MAT_JOINT = new THREE.MeshStandardMaterial({ color: 0xdddddd });

function segment(radius) {
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, 1, 12), MAT_LEG);
  robot.add(mesh);
  return mesh;
}

function placeSegment(mesh, a, b) {
  const dir = new THREE.Vector3().subVectors(b, a);
  mesh.position.copy(a).addScaledVector(dir, 0.5);
  mesh.scale.set(1, Math.max(dir.length(), 0.01), 1);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.normalize());
}

function buildRobot() {
  robot.clear();
  const order = ["RF", "RM", "RR", "LR", "LM", "LF"];
  const byName = Object.fromEntries(geom.legs.map((l) => [l.name, l]));
  const shape = new THREE.Shape(order.map((n) => new THREE.Vector2(-byName[n].body_y, byName[n].body_x)));
  const thickness = 0.25 * geom.tibia;
  const body = new THREE.Mesh(new THREE.ExtrudeGeometry(shape, { depth: thickness, bevelEnabled: false }), MAT_BODY);
  body.rotation.x = -Math.PI / 2;
  body.position.y = -thickness / 2;
  robot.add(body);

  // Flèche vers l'avant.
  const arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), v3(0, 0, thickness / 2 + 1), 0.6 * geom.tibia, 0xf0a030);
  robot.add(arrow);

  const r = 0.06 * geom.tibia;
  legMeshes = geom.legs.map(() => ({
    root: segment(r * 0.8),
    coxa: segment(r),
    femur: segment(r),
    tibia: segment(r * 0.8),
    joints: [0, 1, 2, 3].map(() => {
      const s = new THREE.Mesh(new THREE.SphereGeometry(r * 1.3, 12, 8), MAT_JOINT);
      robot.add(s);
      return s;
    }),
  }));
  const span = Math.max(...geom.legs.map((l) => Math.hypot(l.x, l.y))) + geom.coxa + geom.femur + geom.tibia;
  camera.position.set(span * 1.2, span * 1.0, span * 1.5);
}

// Même FK que hexapod/leg_ik.py.
function legPoints(leg, [c, f, t]) {
  const kr = geom.coxa + geom.femur * Math.cos(f);
  const kz = geom.femur * Math.sin(f);
  const fr = kr + geom.tibia * Math.cos(f + t);
  const fz = kz + geom.tibia * Math.sin(f + t);
  const local = [[0, 0], [geom.coxa, 0], [kr, kz], [fr, fz]];
  const ca = Math.cos(leg.angle + c);
  const sa = Math.sin(leg.angle + c);
  return local.map(([r, z]) => v3(leg.x + r * ca, leg.y + r * sa, z));
}

function updateRobot() {
  if (!geom || !angles || !legMeshes.length) return;
  let lowest = 0;
  geom.legs.forEach((leg, i) => {
    const pts = legPoints(leg, angles[i]);
    const m = legMeshes[i];
    const mat = leg.name === selectedLeg && mode === "cal" ? MAT_SEL : MAT_LEG;
    for (const seg of [m.root, m.coxa, m.femur, m.tibia]) seg.material = mat;
    placeSegment(m.root, v3(leg.body_x, leg.body_y, 0), pts[0]);
    placeSegment(m.coxa, pts[0], pts[1]);
    placeSegment(m.femur, pts[1], pts[2]);
    placeSegment(m.tibia, pts[2], pts[3]);
    pts.forEach((p, j) => m.joints[j].position.copy(p));
    lowest = Math.min(lowest, pts[3].y);
  });
  // Le sol passe sous le pied le plus bas.
  grid.position.y = lowest;
}

function resize() {
  const w = view.clientWidth;
  const h = view.clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener("resize", resize);
resize();

renderer.setAnimationLoop(() => {
  updateRobot();
  controls.update();
  renderer.render(scene, camera);
});

refreshPorts();
