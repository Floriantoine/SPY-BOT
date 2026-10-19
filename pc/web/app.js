import { Animator, PRESETS } from "./anim.js";
import { Joystick } from "./joystick.js";
import { RobotView } from "./scene.js";

const $ = (s) => document.querySelector(s);
const $$ = (s) => document.querySelectorAll(s);

const MODES = {
  test: "Test servo",
  off: "Relâché",
  wake: "Démarrage…",
  sit: "Assis",
  standing: "Se lève…",
  stand: "Debout",
  sitting: "S'assoit…",
  cal: "Calibration",
};
const JOINTS = ["coxa", "femur", "tibia"];
const JOINT_LABELS = { coxa: "Coxa", femur: "Fémur", tibia: "Tibia" };

const state = {
  connected: false,
  mode: null,
  angles: null,
  calib: {},
  invert: {},
  selectedLeg: "RF",
};

const view = new RobotView($("#view"));
const animator = new Animator();
window.spybot = { view, state }; // pour déboguer depuis la console

// ---------------------------------------------------------------- liaison

async function post(path, body) {
  try {
    const res = await fetch(path, { method: "POST", body });
    const data = await res.json();
    if (data.error) toast(data.error, "err");
    return data;
  } catch {
    toast("Serveur PC injoignable", "err");
    return {};
  }
}

function send(line, quiet = false) {
  if (!quiet) log(line, "out");
  return post("/cmd", line);
}

async function refreshPorts() {
  const { ports = [], link } = await (await fetch("/ports")).json();
  const sel = $("#port");
  const previous = sel.value;
  sel.innerHTML = "";
  for (const p of ports) sel.add(new Option(`${p.device} · ${p.description}`, p.device));
  sel.add(new Option("Simulation (sans robot)", "sim"));
  sel.value = link || (ports.some((p) => p.device === previous) ? previous : sel.options[0].value);
}

$("#refresh").onclick = refreshPorts;
$("#connect").onclick = () => {
  if (state.connected) post("/disconnect");
  else post("/connect", JSON.stringify({ port: $("#port").value }));
};

// ------------------------------------------------------------ événements

const events = new EventSource("/events");
events.onmessage = (e) => {
  const line = e.data;
  const space = line.indexOf(" ");
  const kind = space < 0 ? line : line.slice(0, space);
  const rest = space < 0 ? "" : line.slice(space + 1);

  if (kind === "T") {
    const parts = rest.split(" ");
    setMode(parts[0]);
    $("#r-phase").textContent = state.mode === "stand" ? Number(parts[1]).toFixed(2) : "–";
    const v = parts.slice(2).map((x) => Number(x) / 1000);
    state.angles = [0, 1, 2, 3, 4, 5].map((i) => v.slice(3 * i, 3 * i + 3));
    return;
  }
  if (line === "OK PONG") return gotPong();
  log(line, kind === "ERR" ? "err" : "");
  if (kind === "GEOM") {
    const geom = JSON.parse(rest);
    state.geom = geom;
    view.setGeometry(geom);
    renderWiring(geom);
  } else if (kind === "CAL") {
    const { invert = {}, ...trims } = JSON.parse(rest);
    state.calib = trims;
    state.invert = invert;
    renderTrims();
  } else if (kind === "LINK") {
    const [what, ...name] = rest.split(" ");
    setConnected(what === "connected", name.join(" "));
    if (what === "error") toast("Liaison perdue : " + name.join(" "), "err");
    if (what === "reconnecting") {
      toast("Liaison perdue, reconnexion automatique…", "err");
      showLatency("reconnexion…", "bad");
    }
  } else if (kind === "ERR") {
    toast(rest, "err");
  } else if (kind === "OK") {
    const [cmd, leg, joint, value] = rest.split(" ");
    if (cmd === "TRIM") {
      (state.calib[leg] ||= [0, 0, 0])[JOINTS.indexOf(joint)] = Number(value);
      renderSelection();
    }
    if (cmd === "INVERT") {
      (state.invert[leg] ||= [0, 0, 0])[JOINTS.indexOf(joint)] = Number(value);
      renderSelection();
      send("GEOM", true); // sens des servos à jour pour l'affichage des butées
    }
    if (cmd === "SAVE") toast("Calibration enregistrée dans l'ESP32", "ok");
  } else if (kind === "HELLO") {
    toast(rest.includes("simulation") ? "Robot simulé prêt" : "Robot connecté", "ok");
  }
};
events.onerror = () => setConnected(false);

// Santé de la liaison : un PING toutes les 2 s, temps de réponse affiché.
let pingSentAt = 0;
let pongAt = 0;
function showLatency(text, cls = "") {
  const el = $("#latency");
  el.textContent = text;
  el.className = "latency " + cls;
}
function gotPong() {
  pongAt = performance.now();
  const ms = Math.round(pongAt - pingSentAt);
  showLatency(`${ms} ms`, ms > 150 ? "slow" : "good");
}
setInterval(() => {
  if (!state.connected) return;
  const now = performance.now();
  if (pingSentAt > pongAt && now - pingSentAt > 3000) showLatency("pas de réponse", "bad");
  if (pingSentAt <= pongAt || now - pingSentAt > 3000) {
    pingSentAt = now;
    send("PING", true);
  }
}, 2000);

function setConnected(connected, name = "") {
  state.connected = connected;
  if (!connected) showLatency("");
  if (!connected) setMode(null);
  $("#link-dot").classList.toggle("on", connected);
  const btn = $("#connect");
  btn.textContent = connected ? "Déconnecter" : "Connecter";
  btn.classList.toggle("connected", connected);
  $("#port").disabled = connected;
  if (connected && name) $("#port").value = name;
  updateControls();
}

function setMode(mode) {
  if (mode === state.mode) return;
  state.mode = mode;
  updateControls();
}

function updateControls() {
  const { connected, mode } = state;
  $("#mode").textContent = connected ? MODES[mode] || "…" : "Déconnecté";
  $("#mode-chip").dataset.mode = connected ? mode || "" : "";

  // Un seul bouton de posture : il propose toujours l'étape suivante.
  const stand = $("#btn-stand");
  const up = mode === "stand" || mode === "standing";
  stand.classList.toggle("down", up);
  stand.querySelector("span").textContent = up ? "S'asseoir" : "Se lever";
  stand.title = up ? "S'asseoir (L)" : "Se lever, en alimentant les servos si besoin (L)";

  for (const el of $$("[data-needs=link]")) el.disabled = !connected;
  $("#btn-relax").disabled = !connected || mode === "off";
  $("#view").classList.toggle("can-drive", connected && mode === "stand");
}

$("#btn-stand").onclick = () => send(state.mode === "stand" || state.mode === "standing" ? "SIT" : "STAND");
$("#btn-relax").onclick = () => send("POWER 0");
$("#btn-cal").onclick = () => send("CAL");
$("#btn-save").onclick = () => send("SAVE");

// --------------------------------------------------------------- pilotage

const stickMove = new Joystick($("#stick-move"));
const stickTurn = new Joystick($("#stick-turn"), { horizontal: true });
const keys = new Set();

const isTyping = (e) => ["INPUT", "SELECT", "TEXTAREA"].includes(e.target.tagName) && e.target.type !== "range";

document.addEventListener("keydown", (e) => {
  if (isTyping(e) || e.repeat && !"zqsdae".includes(e.key.toLowerCase())) return;
  const k = e.key.toLowerCase();
  if ("zqsdae".includes(k) && k.length === 1) {
    keys.add(k);
    e.preventDefault();
  } else if (k === "shift") {
    keys.add("shift");
  } else if (k === " ") {
    keys.clear();
    e.preventDefault();
  } else if (k === "l" && !e.repeat) {
    $("#btn-stand").click();
  } else if (k === "escape") {
    send("POWER 0");
  }
});
document.addEventListener("keyup", (e) => keys.delete(e.key.toLowerCase()));
window.addEventListener("blur", () => keys.clear());

function readKeyboard() {
  const has = (k) => (keys.has(k) ? 1 : 0);
  const gain = keys.has("shift") ? 1 : 0.5;
  let x = has("d") - has("q");
  let y = has("z") - has("s");
  const n = Math.hypot(x, y) || 1;
  return { x: (x / n) * gain, y: (y / n) * gain, turn: (has("e") - has("a")) * gain };
}

// Manette (API Gamepad) : stick gauche déplacement, stick droit rotation.
const padButtons = {};
function readGamepad() {
  const pad = [...(navigator.getGamepads?.() || [])].find(Boolean);
  if (!pad) return null;
  const pressed = (i) => pad.buttons[i]?.pressed;
  for (const [i, action] of [[0, "STAND"], [1, "SIT"]]) {
    if (pressed(i) && !padButtons[i]) send(action);
    padButtons[i] = pressed(i);
  }
  const [lx = 0, ly = 0, rx = 0] = pad.axes;
  return { x: lx, y: -ly, turn: rx };
}

function readDrive() {
  const kb = readKeyboard();
  const pad = readGamepad();
  const big = (a, b) => (Math.abs(a) >= Math.abs(b) ? a : b);
  const ext = {
    x: big(kb.x, pad ? pad.x : 0),
    y: big(kb.y, pad ? pad.y : 0),
    turn: big(kb.turn, pad ? pad.turn : 0),
  };
  stickMove.setExternal(ext.x, ext.y);
  stickTurn.setExternal(ext.turn, 0);
  const move = stickMove.value;
  const turn = stickTurn.value.x;
  const speed = Number($("#speed").value);
  const maxTurn = Number($("#turn").value);
  // Stick vers la droite = aller à droite (vy négatif) / tourner à droite (lacet négatif).
  return {
    vx: Math.round(move.y * speed),
    vy: Math.round(-move.x * speed),
    yaw: Math.round(-turn * maxTurn),
  };
}

let drive = { vx: 0, vy: 0, yaw: 0 };
let moving = false;
setInterval(() => {
  if (state.mode !== "stand") {
    moving = false;
    return;
  }
  const { vx, vy, yaw } = drive;
  // Le robot s'arrête seul s'il ne reçoit plus de MOVE pendant 0,5 s.
  if (vx || vy || yaw) {
    send(`MOVE ${vx} ${vy} ${yaw}`, true);
    moving = true;
  } else if (moving) {
    send("MOVE 0 0 0", true);
    moving = false;
  }
}, 100);

// ------------------------------------------------------- curseurs, réglages

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

function bindSlider(input, format) {
  const out = input.closest(".field, .trim-row")?.querySelector("output");
  const update = () => {
    const p = ((input.value - input.min) / (input.max - input.min)) * 100;
    input.style.setProperty("--p", p + "%");
    if (out) out.textContent = format(Number(input.value));
  };
  input.addEventListener("input", update);
  update();
  return update;
}

const sliders = {
  speed: bindSlider($("#speed"), (v) => `${v} mm/s`),
  turn: bindSlider($("#turn"), (v) => `${v} °/s`),
  period: bindSlider($("#period"), (v) => `${v.toFixed(1)} s`),
  step: bindSlider($("#step"), (v) => `${v} mm`),
};
const sendGait = throttle(() => send(`GAIT ${$("#period").value} ${$("#step").value}`, true), 150);
$("#period").addEventListener("input", sendGait);
$("#step").addEventListener("input", sendGait);

// Préréglages de vitesse : règlent d'un coup vitesse, rotation et démarche.
const PRESET_VALUES = {
  slow: { speed: 40, turn: 15, period: 1.4, step: 20 },
  normal: { speed: 80, turn: 30, period: 1.0, step: 25 },
  fast: { speed: 140, turn: 50, period: 0.6, step: 30 },
};
function markPreset() {
  for (const b of $$("#presets button")) {
    const v = PRESET_VALUES[b.dataset.preset];
    b.classList.toggle("active", Object.keys(v).every((k) => Number($("#" + k).value) === v[k]));
  }
}
for (const b of $$("#presets button")) {
  b.onclick = () => {
    for (const [k, v] of Object.entries(PRESET_VALUES[b.dataset.preset])) {
      $("#" + k).value = v;
      sliders[k]();
    }
    if (state.connected) sendGait();
    markPreset();
  };
}
for (const k of Object.keys(sliders)) $("#" + k).addEventListener("input", markPreset);
markPreset();

const poseInputs = [...$$("input.pose")].sort((a, b) => a.dataset.i - b.dataset.i);
const sendPose = throttle(() => send("POSE " + poseInputs.map((i) => i.value).join(" "), true), 60);
const poseUpdaters = poseInputs.map((input) => {
  input.addEventListener("input", () => {
    animator.stop();
    renderAnimState();
    sendPose();
  });
  return bindSlider(input, (v) => (Number(input.dataset.i) < 3 ? `${v} mm` : `${v}°`));
});
const currentPose = () => poseInputs.map((i) => Number(i.value));
function setPose(values) {
  poseInputs.forEach((input, i) => {
    input.value = Math.round(values[i] * 10) / 10;
    poseUpdaters[i]();
  });
  if (state.connected) sendPose();
}
$("#pose-reset").onclick = () => {
  animator.stop();
  renderAnimState();
  setPose([0, 0, 0, 0, 0, 0]);
};

// Onglets.
let activeTab = "drive";
for (const tab of $$(".tab")) {
  tab.onclick = () => selectTab(tab.dataset.tab);
}

function selectTab(name) {
  activeTab = name;
  for (const t of $$(".tab")) t.classList.toggle("active", t.dataset.tab === name);
  for (const p of $$(".tab-page")) p.classList.toggle("active", p.id === "tab-" + name);
  const picking = name === "cal" || name === "build";
  view.showLabels = name === "build";
  view.selection = picking ? [state.selectedLeg, state.selectedJoint] : null;
  view.focus = name === "cal";
  $("#view").classList.toggle("pickable", picking);
}

// ------------------------------------------------- sélection d'un servo

const LEG_NAMES_FR = { RF: "Avant droite", RM: "Milieu droite", RR: "Arrière droite", LF: "Avant gauche", LM: "Milieu gauche", LR: "Arrière gauche" };
state.selectedJoint = 0;

function select(leg, joint, { turnCamera = true } = {}) {
  if (turnCamera && leg !== state.selectedLeg) view.lookAtLeg(leg);
  state.selectedLeg = leg;
  state.selectedJoint = joint;
  if (activeTab === "cal" || activeTab === "build") view.selection = [leg, joint];
  renderSelection();
}

function servoPort(leg, joint) {
  const g = state.geom && state.geom.legs.find((l) => l.name === leg);
  if (!g) return "";
  const [board, channel] = g.servos[joint];
  return `PCA9685 0x${state.geom.addresses[board].toString(16)} · canal ${channel}`;
}

function trimOf(leg, joint) {
  return (state.calib[leg] || [0, 0, 0])[joint] || 0;
}

// Toucher la 3D (sans faire tourner la caméra) choisit le servo touché.
let downAt = null;
view.renderer.domElement.addEventListener("pointerdown", (e) => (downAt = [e.clientX, e.clientY]));
view.renderer.domElement.addEventListener("pointerup", (e) => {
  if (!downAt || Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) > 6) return;
  if (activeTab !== "cal" && activeTab !== "build") return;
  const hit = view.pick(e.clientX, e.clientY);
  if (hit) select(hit.leg, hit.joint, { turnCamera: false });
});

// ----------------------------------------------------------------- montage

function renderWiring(geom) {
  const boards = geom.addresses.map((addr, b) => ({ addr, side: b === 0 ? "pattes droites" : "pattes gauches", rows: [] }));
  for (const leg of geom.legs) {
    leg.servos.forEach(([board, channel], j) => boards[board].rows.push({ leg: leg.name, j, channel }));
  }
  const box = $("#wiring");
  box.innerHTML = "";
  for (const b of boards) {
    b.rows.sort((x, y) => x.channel - y.channel);
    const div = document.createElement("div");
    div.className = "board";
    div.innerHTML = `<h4>PCA9685 0x${b.addr.toString(16)} <small>${b.side}</small></h4>`;
    for (const r of b.rows) {
      const row = document.createElement("div");
      row.className = "wire";
      row.dataset.leg = r.leg;
      row.dataset.joint = r.j;
      row.innerHTML = `<span class="ch">${r.channel}</span>
        <span>${LEG_NAMES_FR[r.leg]} <small>· ${JOINT_LABELS[JOINTS[r.j]]}</small></span>
        <button class="btn" data-needs="link">Tester</button>`;
      row.onclick = () => select(r.leg, r.j);
      row.querySelector("button").onclick = () => send(`TEST ${r.leg} ${JOINTS[r.j]}`);
      div.append(row);
    }
    box.append(div);
  }
  renderSelection();
  updateControls();
}

$("#btn-center").onclick = () => send("CAL");

// ------------------------------------------------------------- calibration

const LEG_LAYOUT = [["LF", "Avant G"], ["RF", "Avant D"], ["LM", "Milieu G"], ["RM", "Milieu D"], ["LR", "Arrière G"], ["RR", "Arrière D"]];
const legMap = $("#legs");
const bodyCell = document.createElement("div");
bodyCell.className = "body";
bodyCell.textContent = "AVANT ↑";
legMap.append(bodyCell);
LEG_LAYOUT.forEach(([name, label], i) => {
  const b = document.createElement("button");
  b.textContent = label;
  b.dataset.leg = name;
  b.style.gridColumn = i % 2 ? 3 : 1;
  b.style.gridRow = Math.floor(i / 2) + 1;
  b.onclick = () => select(name, state.selectedJoint);
  legMap.append(b);
});

const jointButtons = JOINTS.map((joint, j) => {
  const b = document.createElement("button");
  b.innerHTML = `<span>${JOINT_LABELS[joint]}</span><b></b>`;
  b.onclick = () => select(state.selectedLeg, j);
  $("#joints").append(b);
  return b;
});

const selTrim = $("#sel-trim");
const updateSelSlider = bindSlider(selTrim, () => "");
const sendTrim = throttle(() => {
  const value = Number(selTrim.value);
  (state.calib[state.selectedLeg] ||= [0, 0, 0])[state.selectedJoint] = value;
  send(`TRIM ${state.selectedLeg} ${JOINTS[state.selectedJoint]} ${value}`, true);
}, 80);

function setTrim(value) {
  selTrim.value = Math.max(-30, Math.min(30, value));
  updateSelSlider();
  (state.calib[state.selectedLeg] ||= [0, 0, 0])[state.selectedJoint] = Number(selTrim.value);
  renderSelection();
  sendTrim();
}

selTrim.addEventListener("input", () => setTrim(Number(selTrim.value)));
for (const b of $$(".nudge button")) b.onclick = () => setTrim(Number(selTrim.value) + Number(b.dataset.d));
$("#sel-zero").onclick = () => setTrim(0);
$("#sel-test").onclick = () => send(`TEST ${state.selectedLeg} ${JOINTS[state.selectedJoint]}`);
$("#sel-invert").onclick = () => {
  const { selectedLeg: leg, selectedJoint: j } = state;
  send(`INVERT ${leg} ${JOINTS[j]} ${invertedOf(leg, j) ? 0 : 1}`);
};
const invertedOf = (leg, j) => Boolean(state.invert[leg]?.[j]);

document.addEventListener("keydown", (e) => {
  if (activeTab !== "cal" || isTyping(e)) return;
  const steps = { ArrowLeft: -0.5, ArrowRight: 0.5 };
  if (e.key in steps) {
    setTrim(Number(selTrim.value) + steps[e.key]);
    e.preventDefault();
  } else if (e.key === "ArrowUp" || e.key === "ArrowDown") {
    select(state.selectedLeg, (state.selectedJoint + (e.key === "ArrowDown" ? 1 : 2)) % 3);
    e.preventDefault();
  }
});

function renderSelection() {
  const { selectedLeg: leg, selectedJoint: j } = state;
  for (const b of legMap.querySelectorAll("button")) b.classList.toggle("active", b.dataset.leg === leg);
  jointButtons.forEach((b, i) => {
    b.classList.toggle("active", i === j);
    b.querySelector("b").textContent = `${trimOf(leg, i).toFixed(1)}°`;
  });
  for (const w of $$(".wire")) w.classList.toggle("lit", w.dataset.leg === leg && Number(w.dataset.joint) === j);
  $("#sel-name").textContent = `${LEG_NAMES_FR[leg]} · ${JOINT_LABELS[JOINTS[j]]}`;
  $("#sel-port").textContent = servoPort(leg, j) + (invertedOf(leg, j) ? " · sens inversé" : "");
  $("#sel-invert").classList.toggle("on", invertedOf(leg, j));
  const value = trimOf(leg, j);
  $("#sel-value").textContent = (value > 0 ? "+" : "") + value.toFixed(1);
  if (document.activeElement !== selTrim) {
    selTrim.value = value;
    updateSelSlider();
  }
}

// Compatibilité avec le reste du fichier.
const renderTrims = renderSelection;

// Sauvegarde des trims dans un fichier, pour les retrouver après un reflash.
$("#cal-export").onclick = () => {
  const data = { robot: "SPY-BOT", saved: new Date().toISOString(), trims: state.calib, invert: state.invert };
  const blob = new Blob([JSON.stringify(data, null, 2)], { type: "application/json" });
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = `spybot-calibration-${data.saved.slice(0, 10)}.json`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
};
$("#cal-import").onclick = () => $("#cal-file").click();
$("#cal-file").onchange = async (e) => {
  const file = e.target.files[0];
  e.target.value = "";
  if (!file) return;
  let trims, invert;
  try {
    const data = JSON.parse(await file.text());
    trims = data.trims || data;
    invert = data.invert || {};
    const ok = Object.entries(trims).every(([leg, v]) => LEG_NAMES_FR[leg] && Array.isArray(v) && v.length === 3 && v.every(Number.isFinite));
    if (!ok) throw new Error();
  } catch {
    toast("Fichier de calibration illisible", "err");
    return;
  }
  const lines = [];
  for (const [leg, values] of Object.entries(trims)) {
    values.forEach((v, j) => lines.push(`TRIM ${leg} ${JOINTS[j]} ${v}`));
    (invert[leg] || [0, 0, 0]).forEach((v, j) => lines.push(`INVERT ${leg} ${JOINTS[j]} ${v ? 1 : 0}`));
  }
  await send(lines.join("\n"), true);
  state.calib = trims;
  renderSelection();
  toast("Calibration importée : clique sur Enregistrer pour la garder dans l'ESP32", "ok");
};

// ------------------------------------------------------------- servos en butée

// Même calcul que hexapod/servo.py : un servo est en butée quand l'angle
// demandé dépasse ses limites (le robot le bloque alors à la limite).
function limitedServos() {
  const out = new Set();
  if (!state.geom || !state.angles || !state.connected || state.mode === "off") return out;
  state.geom.legs.forEach((leg, i) => {
    if (!leg.joints) return;
    leg.joints.forEach(([zero, dir, min, max], j) => {
      const deg = 90 + dir * ((state.angles[i][j] * 180) / Math.PI - zero) + trimOf(leg.name, j);
      if (deg <= min + 0.5 || deg >= max - 0.5) out.add(`${leg.name}:${j}`);
    });
  });
  return out;
}
let limitKey = "";
let limitToastAt = -1e9;
function updateLimits() {
  const limits = limitedServos();
  view.limits = limits;
  const key = [...limits].sort().join(",");
  const now = performance.now();
  if (key && key !== limitKey && now - limitToastAt > 4000) {
    limitToastAt = now;
    const names = [...limits].map((k) => {
      const [leg, j] = k.split(":");
      return `${LEG_NAMES_FR[leg]} · ${JOINT_LABELS[JOINTS[j]]}`;
    });
    toast(`En butée : ${names.slice(0, 3).join(", ")}${names.length > 3 ? "…" : ""}`, "err");
  }
  limitKey = key;
}

// -------------------------------------------------------------- animations

const ANIM_STORE = "spybot.animations";
function loadAnims() {
  try {
    return JSON.parse(localStorage.getItem(ANIM_STORE)) || [];
  } catch {
    return [];
  }
}
function storeAnims(list) {
  try {
    localStorage.setItem(ANIM_STORE, JSON.stringify(list));
  } catch {
    toast("Impossible d'enregistrer sur ce navigateur", "err");
  }
}

let draft = [];
let playingName = null;

function startAnimation(name, frames) {
  animator.play(frames, currentPose(), { loop: $("#anim-loop").checked });
  playingName = name;
  renderAnimState();
}

function playAnimation() {
  if (!animator.playing) return;
  const pose = animator.pose();
  if (pose) setPose(pose);
  if (!animator.playing) {
    playingName = null;
    renderAnimState();
  }
}

function renderAnimState() {
  if (!animator.playing) playingName = null;
  $("#anim-stop").disabled = !animator.playing;
  for (const b of $$("#anim-list .anim")) b.classList.toggle("active", b.dataset.name === playingName);
}

function renderAnimList() {
  const box = $("#anim-list");
  box.innerHTML = "";
  const mine = loadAnims();
  const add = (anim, saved) => {
    const item = document.createElement("div");
    item.className = "anim";
    item.dataset.name = anim.name;
    const play = document.createElement("button");
    play.className = "anim-play";
    play.innerHTML = `<svg viewBox="0 0 24 24"><path d="M8 5l11 7-11 7z"/></svg><span></span>`;
    play.querySelector("span").textContent = anim.name;
    play.onclick = () => startAnimation(anim.name, anim.frames);
    item.append(play);
    if (saved) {
      const edit = document.createElement("button");
      edit.className = "anim-edit";
      edit.title = "Modifier";
      edit.textContent = "✎";
      edit.onclick = () => {
        draft = anim.frames.map((f) => ({ pose: f.pose.slice(), t: f.t }));
        $("#anim-name").value = anim.name;
        renderDraft();
      };
      const del = document.createElement("button");
      del.className = "anim-edit";
      del.title = "Supprimer";
      del.textContent = "×";
      del.onclick = () => {
        storeAnims(loadAnims().filter((a) => a.name !== anim.name));
        renderAnimList();
      };
      item.append(edit, del);
    }
    box.append(item);
  };
  PRESETS.forEach((a) => add(a, false));
  mine.forEach((a) => add(a, true));
  renderAnimState();
}

const POSE_LABELS = ["av", "côté", "haut", "roul.", "tang.", "lacet"];
function renderDraft() {
  const list = $("#keyframes");
  list.innerHTML = "";
  draft.forEach((frame, i) => {
    const li = document.createElement("li");
    const desc = frame.pose
      .map((v, j) => (Math.abs(v) >= 0.5 ? `${POSE_LABELS[j]} ${v > 0 ? "+" : ""}${Math.round(v)}` : null))
      .filter(Boolean)
      .join(" · ") || "corps au centre";
    li.innerHTML = `<button class="kf-pose" title="Voir cette pose"></button>
      <label><input type="number" min="0.1" max="5" step="0.1"> s</label>
      <button class="kf-del" title="Retirer">×</button>`;
    li.querySelector(".kf-pose").textContent = desc;
    li.querySelector(".kf-pose").onclick = () => setPose(frame.pose);
    const t = li.querySelector("input");
    t.value = frame.t;
    t.onchange = () => (frame.t = Math.max(0.1, Math.min(5, Number(t.value) || 0.6)));
    li.querySelector(".kf-del").onclick = () => {
      draft.splice(i, 1);
      renderDraft();
    };
    list.append(li);
  });
  $("#keyframes-empty").hidden = draft.length > 0;
}

$("#kf-add").onclick = () => {
  draft.push({ pose: currentPose(), t: 0.6 });
  renderDraft();
};
$("#anim-preview").onclick = () => {
  if (draft.length) startAnimation(null, draft);
  else toast("Ajoute au moins une pose", "err");
};
$("#anim-save").onclick = () => {
  if (!draft.length) return toast("Ajoute au moins une pose", "err");
  const mine = loadAnims();
  const name = $("#anim-name").value.trim() || `Animation ${mine.length + 1}`;
  const frames = draft.map((f) => ({ pose: f.pose.slice(), t: f.t }));
  storeAnims([...mine.filter((a) => a.name !== name), { name, frames }]);
  renderAnimList();
  toast(`« ${name} » enregistrée`, "ok");
};
$("#anim-stop").onclick = () => {
  animator.stop();
  renderAnimState();
};
$("#anim-loop").onchange = () => (animator.loop = $("#anim-loop").checked);

renderAnimList();
renderDraft();

// ---------------------------------------------------------- journal, toasts

function log(line, cls = "") {
  const el = $("#log");
  const row = document.createElement("div");
  if (cls) row.className = cls;
  row.textContent = (cls === "out" ? "→ " : "") + line;
  el.append(row);
  while (el.childElementCount > 300) el.firstChild.remove();
  el.scrollTop = el.scrollHeight;
}

const recentToasts = new Map();
function toast(text, kind = "") {
  // Un même message répété (ex. MOVE refusé 10 fois par seconde) ne s'affiche qu'une fois.
  const now = performance.now();
  if (now - (recentToasts.get(text) || -1e9) < 3500) return;
  recentToasts.set(text, now);
  const el = document.createElement("div");
  el.className = "toast " + kind;
  el.textContent = text;
  $("#toasts").append(el);
  setTimeout(() => el.remove(), 3500);
}

$("#console").onsubmit = (e) => {
  e.preventDefault();
  const input = $("#console-input");
  if (input.value.trim()) send(input.value.trim());
  input.value = "";
};

// ------------------------------------------------------------------- 3D

// Traces des pieds au sol.
$("#traces").onclick = () => {
  view.showTraces = !view.showTraces;
  $("#traces").classList.toggle("on", view.showTraces);
};

// Mode pilote : la 3D et les joysticks en plein écran, idéal téléphone en paysage.
const phoneLandscape = matchMedia("(orientation: landscape) and (max-height: 520px)");
function updatePilot() {
  const on = phoneLandscape.matches || !!document.fullscreenElement;
  document.body.classList.toggle("pilot", on);
  $("#pilot").classList.toggle("on", !!document.fullscreenElement);
  if (on && activeTab !== "drive") selectTab("drive");
}
phoneLandscape.addEventListener("change", updatePilot);
document.addEventListener("fullscreenchange", updatePilot);
$("#pilot").onclick = async () => {
  try {
    if (document.fullscreenElement) await document.exitFullscreen();
    else {
      await document.documentElement.requestFullscreen();
      await screen.orientation?.lock?.("landscape").catch(() => {});
    }
  } catch {
    toast("Plein écran indisponible sur ce navigateur", "err");
  }
};
updatePilot();

let topView = false;
$("#cam-reset").onclick = () => view.resetCamera(topView);
$("#cam-follow").onclick = () => {
  topView = !topView;
  view.resetCamera(topView);
};

let lastFrame = performance.now();
function frame() {
  const now = performance.now();
  const dt = Math.min(0.5, (now - lastFrame) / 1000);
  lastFrame = now;
  drive = readDrive();
  // Le sol défile sous le robot à la vitesse demandée (estimation, pas de capteur).
  if (state.mode === "stand") view.move(drive.vx, drive.vy, (drive.yaw * Math.PI) / 180, dt);
  stickMove.render();
  stickTurn.render();
  $("#r-vx").textContent = drive.vx;
  $("#r-vy").textContent = drive.vy;
  $("#r-yaw").textContent = drive.yaw;
  playAnimation();
  updateLimits();
  view.update(state.angles);
  view.render();
  requestAnimationFrame(frame);
}

updateControls();
renderSelection();
refreshPorts();
requestAnimationFrame(frame);
