import { Joystick } from "./joystick.js";
import { RobotView } from "./scene.js";

const $ = (s) => document.querySelector(s);
const $$ = (s) => document.querySelectorAll(s);

const MODES = {
  test: "Test servo",
  off: "Relâché",
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
  selectedLeg: "RF",
};

const view = new RobotView($("#view"));
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
  log(line, kind === "ERR" ? "err" : "");
  if (kind === "GEOM") {
    const geom = JSON.parse(rest);
    state.geom = geom;
    view.setGeometry(geom);
    renderWiring(geom);
  } else if (kind === "CAL") {
    state.calib = JSON.parse(rest);
    renderTrims();
  } else if (kind === "LINK") {
    const [what, ...name] = rest.split(" ");
    setConnected(what === "connected", name.join(" "));
    if (what === "error") toast("Liaison perdue : " + name.join(" "), "err");
  } else if (kind === "ERR") {
    toast(rest, "err");
  } else if (kind === "OK") {
    const [cmd, leg, joint, value] = rest.split(" ");
    if (cmd === "TRIM") {
      (state.calib[leg] ||= [0, 0, 0])[JOINTS.indexOf(joint)] = Number(value);
      renderSelection();
    }
    if (cmd === "SAVE") toast("Calibration enregistrée dans l'ESP32", "ok");
  } else if (kind === "HELLO") {
    toast(rest.includes("simulation") ? "Robot simulé prêt" : "Robot connecté", "ok");
  }
};
events.onerror = () => setConnected(false);

function setConnected(connected, name = "") {
  state.connected = connected;
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
  const powered = connected && mode && mode !== "off" && mode !== "cal";
  $("#mode").textContent = connected ? MODES[mode] || "…" : "Déconnecté";
  $("#mode-chip").dataset.mode = connected ? mode || "" : "";

  const power = $("#btn-power");
  power.disabled = !connected;
  power.classList.toggle("on", powered);
  power.querySelector("span").textContent = powered ? "Activé" : "Activer";

  const stand = $("#btn-stand");
  const up = mode === "stand" || mode === "standing";
  stand.disabled = !powered;
  stand.classList.toggle("on", up);
  stand.querySelector("span").textContent = up ? "S'asseoir" : "Se lever";

  for (const el of $$("[data-needs=link]")) el.disabled = !connected;
  $("#view").classList.toggle("can-drive", connected && mode === "stand");
}

$("#btn-power").onclick = () => send(state.mode && state.mode !== "off" && state.mode !== "cal" ? "POWER 0" : "POWER 1");
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
  } else if (k === "p" && !e.repeat) {
    $("#btn-power").click();
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

bindSlider($("#speed"), (v) => `${v} mm/s`);
bindSlider($("#turn"), (v) => `${v} °/s`);
bindSlider($("#period"), (v) => `${v.toFixed(1)} s`);
bindSlider($("#step"), (v) => `${v} mm`);
const sendGait = throttle(() => send(`GAIT ${$("#period").value} ${$("#step").value}`, true), 150);
$("#period").addEventListener("input", sendGait);
$("#step").addEventListener("input", sendGait);

const poseInputs = [...$$("input.pose")].sort((a, b) => a.dataset.i - b.dataset.i);
const sendPose = throttle(() => send("POSE " + poseInputs.map((i) => i.value).join(" "), true), 60);
const poseUpdaters = poseInputs.map((input) => {
  input.addEventListener("input", sendPose);
  return bindSlider(input, (v) => (Number(input.dataset.i) < 3 ? `${v} mm` : `${v}°`));
});
$("#pose-reset").onclick = () => {
  poseInputs.forEach((input, i) => {
    input.value = 0;
    poseUpdaters[i]();
  });
  sendPose();
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
  $("#sel-port").textContent = servoPort(leg, j);
  const value = trimOf(leg, j);
  $("#sel-value").textContent = (value > 0 ? "+" : "") + value.toFixed(1);
  if (document.activeElement !== selTrim) {
    selTrim.value = value;
    updateSelSlider();
  }
}

// Compatibilité avec le reste du fichier.
const renderTrims = renderSelection;

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

function toast(text, kind = "") {
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

let topView = false;
$("#cam-reset").onclick = () => view.resetCamera(topView);
$("#cam-follow").onclick = () => {
  topView = !topView;
  view.resetCamera(topView);
};

function frame() {
  drive = readDrive();
  stickMove.render();
  stickTurn.render();
  $("#r-vx").textContent = drive.vx;
  $("#r-vy").textContent = drive.vy;
  $("#r-yaw").textContent = drive.yaw;
  view.update(state.angles);
  view.render();
  requestAnimationFrame(frame);
}

updateControls();
renderSelection();
refreshPorts();
requestAnimationFrame(frame);
