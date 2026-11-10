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
  legs: 6, // pattes qui marchent : 6, ou 4 (pattes du milieu repliées)
  invert: {},
  selectedLeg: "RF",
  sensor: { present: false, on: false, stop: 80, blocked: false },
  sim: false,
  imu: { present: false, level: false, roll: 0, pitch: 0, corrRoll: 0, corrPitch: 0, at: 0, axes: "+x+y+z", zero: [0, 0] },
  nav: "off",
  walk: { wave: "0", waveSpeed: 20, gait: "tripod", shift: 6 }, // réglages de la marche (ligne WALK)
  clickMode: null, // clic sur le sol : "obst", "goal" ou "person"
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
  if (kind === "S") {
    const [angle, dist] = rest.split(" ").map(Number);
    view.addScan(angle, dist);
    radar.add(angle, dist);
    return;
  }
  if (kind === "P") {
    const [x, y, th] = rest.split(" ").map(Number);
    view.setSimPose(x, y, th / 1000);
    return;
  }
  if (kind === "B") {
    const deci = rest.split(" ").map(Number);
    const [r, p, gr, gp] = deci.slice(0, 4).map((v) => (v / 1800) * Math.PI);
    view.setAttitude(r, p, gr, gp);
    if (deci.length > 4) view.setStance(deci[4]);
    metrics.att = deci.slice(0, 4).map((v) => v / 10);
    return;
  }
  if (kind === "M") {
    gotMetrics(rest.split(" ").map(Number));
    return;
  }
  if (kind === "PROF") {
    gotProf(rest);
    return;
  }
  if (kind === "G") {
    const [a, b, c, foot = -1, dz = 0] = rest.split(" ").map(Number);
    view.setTerrainPlane(a, b / 1e4, c / 1e4, foot, dz / 10);
    return;
  }
  if (kind === "TERRAINMAP") {
    const v = rest.split(" ").map(Number);
    const nw = v[0];
    const waves = [], rocks = [];
    for (let i = 0; i < nw; i++) waves.push(v.slice(1 + 4 * i, 5 + 4 * i));
    const nr = v[1 + 4 * nw];
    for (let i = 0; i < nr; i++) rocks.push(v.slice(2 + 4 * nw + 4 * i, 6 + 4 * nw + 4 * i));
    view.setTerrain({ waves, rocks });
    return;
  }
  if (kind === "I") {
    const [roll, pitch, cr, cp] = rest.split(" ").map((v) => Number(v) / 10);
    Object.assign(state.imu, { roll, pitch, corrRoll: cr, corrPitch: cp, at: performance.now() });
    return;
  }
  if (kind === "PERSON") {
    const [x, y] = rest.split(" ").map(Number);
    view.setPerson([x, y]);
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
    if (cmd === "LEGS") showLegs(Number(leg));
    if (cmd === "SAVE") toast("Calibration enregistrée dans l'ESP32", "ok");
  } else if (kind === "SENSOR") {
    const f = rest.split(" ");
    const value = (key) => Number(f[f.indexOf(key) + 1]);
    Object.assign(state.sensor, { present: value("present") === 1, on: value("on") === 1, stop: value("stop") });
    if (!state.sensor.on) state.sensor.blocked = false;
    renderSensor();
  } else if (kind === "IMU") {
    const f = rest.split(" ");
    const value = (key) => Number(f[f.indexOf(key) + 1]);
    Object.assign(state.imu, { present: value("present") === 1, level: value("level") === 1, feel: value("feel") === 1 });
    // Réglages de la carte (absents d'un ancien firmware) : orientation, zéro en 0,1°.
    const at = f.indexOf("zero");
    if (f.includes("axes")) state.imu.axes = f[f.indexOf("axes") + 1];
    if (at >= 0) state.imu.zero = [Number(f[at + 1]) / 10, Number(f[at + 2]) / 10];
    renderImu();
  } else if (kind === "WALK") {
    // Réglages de la marche : « wave 0|1|auto <seuil> gait tripod|wave ... ».
    const f = rest.split(" ");
    const at = (key) => f.indexOf(key) + 1;
    if (at("wave")) Object.assign(state.walk, { wave: f[at("wave")], waveSpeed: Number(f[at("wave") + 1]) });
    if (at("gait")) state.walk.gait = f[at("gait")];
    if (at("shift")) state.walk.shift = Number(f[at("shift")]);
    renderWalk();
  } else if (kind === "NAV") {
    if (rest === "arrived") toast("Arrivé au point", "ok");
    else if (rest === "stuck") toast("Point impossible à atteindre : le robot s'arrête", "err");
    else {
      state.nav = rest.split(" ")[0];
      const [, gx, gy] = rest.split(" ").map(Number);
      view.setGoal(state.nav === "goto" ? [gx, gy] : null);
    }
    renderNav();
  } else if (kind === "OBSTACLE") {
    state.sensor.blocked = rest === "1";
    renderSensor();
  } else if (kind === "TERRAIN") {
    $("#terrain").checked = rest.trim() === "1";
    if (rest.trim() !== "1") view.setTerrain(null);
  } else if (kind === "TILT") {
    if (rest.trim() === "1") toast("Arrêt de sécurité : le corps penche de plus de 20°", "err");
    else toast("Le robot est de nouveau droit : il peut repartir", "ok");
  } else if (kind === "LIMIT") {
    // Le robot réduit lui-même la pose du corps pour garder ses servos dans leur course.
    if (rest === "pose 1") toast("Pose du corps réduite : les pattes n'ont pas la course pour la tenir en marchant", "");
  } else if (kind === "OBST") {
    const v = rest.trim() ? rest.trim().split(/\s+/).map(Number) : [];
    const list = [];
    for (let i = 0; i + 2 < v.length; i += 3) list.push([v[i], v[i + 1], v[i + 2]]);
    view.setObstacles(list);
  } else if (kind === "HELLO") {
    state.sim = rest.includes("simulation");
    if (state.sim) send("OBST", true);
    $("#sensor-sim").hidden = !state.sim;
    $("#imu-sim").hidden = !state.sim;
    if (state.sim) {
      sendTilt();
      send(`TERRAIN ${$("#terrain").checked ? 1 : 0}`, true);
    }
    sendGait(); // le robot démarre avec ses propres réglages : on lui envoie ceux de l'interface
    if (state.legs !== 6) send(`LEGS ${state.legs}`, true);
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
  if (!connected) {
    state.sim = false;
    state.nav = "off";
    setClickMode(null);
    view.setObstacles([]);
    view.setGoal(null);
    view.setPerson(null);
    view.setAttitude(0, 0, 0, 0);
    view.setStance(null);
    $("#metrics-prof").checked = false;
    view.setTerrain(null);
    renderNav();
  }
  updateControls();
  renderSensor();
  renderImu();
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

// Touches repérées par leur position sur le clavier (e.code), pas par leur
// lettre : mêmes touches en AZERTY (Z Q S D, A E), QWERTY (W A S D, Q E) ou
// Dvorak (, A O E, ' .). Les flèches marchent aussi (sauf en calibration).
const DRIVE_KEYS = {
  KeyW: "fwd", KeyS: "back", KeyA: "left", KeyD: "right", KeyQ: "turnL", KeyE: "turnR",
  ArrowUp: "fwd", ArrowDown: "back", ArrowLeft: "left", ArrowRight: "right",
};
const driveKey = (e) => {
  const action = DRIVE_KEYS[e.code];
  if (!action || e.ctrlKey || e.metaKey || e.altKey) return null;
  if (e.code.startsWith("Arrow") && activeTab === "cal") return null; // flèches : réglage des trims
  return action;
};

document.addEventListener("keydown", (e) => {
  if (isTyping(e)) return;
  const action = driveKey(e);
  if (action) {
    keys.add(action);
    e.preventDefault();
    return;
  }
  if (e.repeat) return;
  if (e.key === "Shift") {
    keys.add("shift");
  } else if (e.code === "Space") {
    keys.clear();
    e.preventDefault();
  } else if (e.code === "KeyL") {
    $("#btn-stand").click();
  } else if (e.key === "Escape") {
    send("POWER 0");
  }
});
document.addEventListener("keyup", (e) => {
  const action = DRIVE_KEYS[e.code];
  if (action) keys.delete(action);
  if (e.key === "Shift") keys.delete("shift");
});
window.addEventListener("blur", () => keys.clear());

function readKeyboard() {
  const has = (k) => (keys.has(k) ? 1 : 0);
  const gain = keys.has("shift") ? 1 : 0.5;
  let x = has("right") - has("left");
  let y = has("fwd") - has("back");
  const n = Math.hypot(x, y) || 1;
  return { x: (x / n) * gain, y: (y / n) * gain, turn: (has("turnR") - has("turnL")) * gain };
}

// Clic sur la 3D : le focus va dans un champ invisible. Les extensions de
// type Vim (Vimium…) passent alors en mode saisie et ne prennent plus les
// touches (O, F…) ; le texte tapé n'y est jamais écrit.
const kbdCapture = $("#kbd-capture");
const kbdBadge = $("#kbd-badge");
kbdCapture.addEventListener("beforeinput", (e) => e.preventDefault());
kbdCapture.addEventListener("keydown", (e) => {
  if (!e.ctrlKey && !e.metaKey && (e.key.length === 1 || e.key === "Enter" || e.key === "Tab")) e.preventDefault();
});
const focusDrive = () => kbdCapture.focus({ preventScroll: true });
$("#view").addEventListener("pointerdown", (e) => {
  if (!e.target.closest("button, input, select, .stick")) setTimeout(focusDrive, 0);
});
kbdBadge.onclick = focusDrive;
const showKbd = () => {
  const on = document.activeElement === kbdCapture;
  kbdBadge.classList.toggle("on", on);
  kbdBadge.textContent = on ? "⌨ Clavier actif" : "⌨ Clavier";
};
kbdCapture.addEventListener("focus", showKbd);
kbdCapture.addEventListener("blur", () => { keys.clear(); showKbd(); });

// Aide : les lettres réelles de ces touches sur le clavier de l'utilisateur.
(async () => {
  const hint = $("#keys-hint");
  let label = (code, fallback) => fallback;
  try {
    const map = await navigator.keyboard?.getLayoutMap?.();
    if (map) label = (code, fallback) => (map.get(code) || fallback).toUpperCase();
  } catch {}
  const k = (code, fallback) => `<kbd>${label(code, fallback)}</kbd>`;
  hint.innerHTML =
    `Clavier : ${k("KeyW", "Z")} ${k("KeyA", "Q")} ${k("KeyS", "S")} ${k("KeyD", "D")} ou les flèches pour avancer, ` +
    `${k("KeyQ", "A")} ${k("KeyE", "E")} pour tourner, <kbd>Maj</kbd> pleine vitesse, <kbd>Espace</kbd> stop, ` +
    `${k("KeyL", "L")} debout / assis, <kbd>Échap</kbd> relâcher.`;
})();

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
  // Le robot s'arrête seul s'il ne reçoit plus de MOVE pendant 1 s.
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

// Marche à 6 ou 4 pattes.
function showLegs(n) {
  state.legs = n;
  for (const b of $$("#legs-mode button")) b.classList.toggle("active", Number(b.dataset.legs) === n);
}
for (const b of $$("#legs-mode button")) {
  b.onclick = () => {
    const n = Number(b.dataset.legs);
    if (!state.connected) return showLegs(n);
    if (drive.vx || drive.vy || drive.yaw) return toast("Arrête le robot avant de changer de démarche", "err");
    send(`LEGS ${n}`, true);
  };
}

// Démarche à 6 pattes : tripode, vague (5 pieds au sol) ou automatique selon la vitesse.
function renderWalk() {
  const w = state.walk;
  for (const b of $$("#wave-mode button")) b.classList.toggle("active", b.dataset.wave.toLowerCase() === w.wave);
  if (document.activeElement !== $("#wave-speed")) {
    $("#wave-speed").value = w.waveSpeed;
    sliders.waveSpeed();
  }
  $("#wave-speed").closest(".field").hidden = w.wave !== "auto";
  if (document.activeElement !== $("#wave-shift")) {
    $("#wave-shift").value = w.shift;
    sliders.waveShift();
  }
  $("#wave-shift").closest(".field").hidden = w.wave === "0";
  $("#walk-state").textContent = "Démarche à 6 pattes : " + (w.gait === "wave" ? "vague (5 pieds au sol)" : "tripode");
}
sliders.waveSpeed = bindSlider($("#wave-speed"), (v) => `${v} mm/s`);
sliders.waveShift = bindSlider($("#wave-shift"), (v) => (Number(v) ? `${v} mm` : "aucun"));
$("#wave-shift").addEventListener("input", throttle(() => send(`SHIFT ${$("#wave-shift").value}`, true), 150));
for (const b of $$("#wave-mode button")) b.onclick = () => send(`WAVE ${b.dataset.wave}`, true);
$("#wave-speed").addEventListener("input", throttle(() => send(`WAVE AUTO ${$("#wave-speed").value}`, true), 150));
renderWalk();

// Préréglages de vitesse : règlent d'un coup vitesse, rotation et démarche.
const PRESET_VALUES = {
  // Longueur d'un pas = vitesse × période / 2 : de grands pas plutôt que des pas rapides.
  slow: { speed: 30, turn: 15, period: 1.6, step: 15 },
  normal: { speed: 60, turn: 30, period: 1.4, step: 19 },
  fast: { speed: 100, turn: 50, period: 1.0, step: 19 },
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
  if (state.clickMode) {
    const p = view.floorPointAt(e.clientX, e.clientY);
    if (p) floorClick(p);
    return;
  }
  if (activeTab !== "cal" && activeTab !== "build") return;
  const hit = view.pick(e.clientX, e.clientY);
  if (hit) select(hit.leg, hit.joint, { turnCamera: false });
});

// ------------------------------------------------------------ capteur avant

/** Vue radar : les mesures récentes du balayage, robot en bas au centre. */
const radar = {
  canvas: $("#radar"),
  scans: new Map(), // angle -> { dist, t }
  angle: null,
  range: 1000, // mm au bord du radar
  add(angle, dist) {
    this.angle = angle;
    this.scans.set(angle, { dist, t: performance.now() });
  },
  draw() {
    const cv = this.canvas;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = cv.clientWidth || 320;
    const h = Math.round(w * 0.53);
    if (cv.width !== Math.round(w * dpr)) {
      cv.width = Math.round(w * dpr);
      cv.height = Math.round(h * dpr);
      cv.style.height = h + "px";
    }
    const ctx = cv.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const cx = w / 2, cy = h - 8, R = h - 16;
    const k = R / this.range;
    const on = state.sensor.on;
    const toXY = (deg, d) => {
      const a = (deg * Math.PI) / 180;
      return [cx - Math.sin(a) * d * k, cy - Math.cos(a) * d * k];
    };
    // Grille : arcs tous les 25 cm, secteur du balayage (±60°) et champ de vue (±13,5°) en plus.
    ctx.lineWidth = 1;
    ctx.font = "10px system-ui, sans-serif";
    ctx.fillStyle = "#5b6577";
    for (let d = 250; d <= this.range; d += 250) {
      ctx.strokeStyle = d === this.range ? "rgba(255,255,255,.16)" : "rgba(255,255,255,.07)";
      ctx.beginPath();
      ctx.arc(cx, cy, d * k, Math.PI + 0.1, -0.1);
      ctx.stroke();
      ctx.fillText(d / 1000 + " m", cx + 3, cy - d * k + 11);
    }
    // Pieds du robot (cercle) et marge d'arrêt autour.
    if (on) {
      ctx.fillStyle = state.sensor.blocked ? "rgba(248,113,113,.22)" : "rgba(52,211,153,.10)";
      ctx.beginPath();
      ctx.arc(cx, cy, (160 + state.sensor.stop) * k, Math.PI, 0);
      ctx.fill();
      ctx.fillStyle = "rgba(79,140,255,.18)";
      ctx.beginPath();
      ctx.arc(cx, cy, 160 * k, Math.PI, 0);
      ctx.fill();
    }
    ctx.strokeStyle = "rgba(255,255,255,.1)";
    for (const deg of [-60, 0, 60]) {
      const [x, y] = toXY(deg, this.range);
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(x, y);
      ctx.stroke();
    }
    const now = performance.now();
    if (on && this.angle !== null) {
      // Faisceau actuel.
      const a = (this.angle * Math.PI) / 180;
      const half = (13.5 * Math.PI) / 180;
      ctx.fillStyle = "rgba(248,113,113,.14)";
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.arc(cx, cy, R, -Math.PI / 2 - a - half, -Math.PI / 2 - a + half);
      ctx.closePath();
      ctx.fill();
    }
    // Mesures : plus elles sont proches, plus elles sont rouges ; elles pâlissent avec le temps.
    for (const [deg, { dist, t }] of this.scans) {
      const age = (now - t) / 4000;
      if (age > 1 || !on) {
        this.scans.delete(deg);
        continue;
      }
      if (!(dist > 0)) continue;
      const [x, y] = toXY(deg, Math.min(dist, this.range));
      const near = dist < state.sensor.stop + 160;
      ctx.globalAlpha = 1 - age * 0.8;
      ctx.fillStyle = dist > this.range ? "#5b6577" : near ? "#f87171" : "#fbbf24";
      ctx.beginPath();
      ctx.arc(x, y, 3.2, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.globalAlpha = 1;
    // Le robot.
    ctx.fillStyle = "#4f8cff";
    ctx.beginPath();
    ctx.moveTo(cx, cy - 9);
    ctx.lineTo(cx - 7, cy + 4);
    ctx.lineTo(cx + 7, cy + 4);
    ctx.closePath();
    ctx.fill();
    if (!on) {
      ctx.fillStyle = "#8a94a6";
      ctx.font = "13px system-ui, sans-serif";
      ctx.textAlign = "center";
      ctx.fillText(state.sensor.present ? "Capteur coupé" : state.connected ? "Pas de capteur détecté" : "Robot non connecté", cx, cy - R / 2);
      ctx.textAlign = "start";
    }
  },
};

function renderSensor() {
  const { present, on, stop, blocked } = state.sensor;
  const connected = state.connected;
  const el = $("#sensor-state");
  el.textContent = !connected ? "–" : !present ? "absent" : on ? (blocked ? "obstacle !" : "actif") : "détecté";
  el.className = "sensor-state " + (!connected || !present ? "" : blocked ? "bad" : on ? "good" : "idle");
  const box = $("#sensor-on");
  box.disabled = !connected || !present;
  box.checked = connected && on;
  const slider = $("#sensor-stop");
  if (document.activeElement !== slider) {
    slider.value = stop;
    sliders.sensorStop();
  }
  $("#obstacle-alert").hidden = !(connected && on && blocked);
  view.setSensor(connected && present, connected && on);
}

sliders.sensorStop = bindSlider($("#sensor-stop"), (v) => `${v} mm`);
$("#sensor-on").onchange = (e) => send(`SENSOR ${e.target.checked ? 1 : 0}`);
$("#sensor-stop").addEventListener("change", (e) => send(`SENSOR STOP ${e.target.value}`));
$("#obst-clear").onclick = () => send("OBST CLEAR");

// Boutons « clique ensuite sur le sol » : un seul actif à la fois.
const CLICK_LABELS = {};
for (const b of $$("[data-click]")) CLICK_LABELS[b.dataset.click] = b.textContent;
function setClickMode(mode) {
  state.clickMode = mode;
  for (const b of $$("[data-click]")) {
    const on = b.dataset.click === mode;
    b.classList.toggle("on", on);
    b.textContent = on ? "Clique sur le sol…" : CLICK_LABELS[b.dataset.click];
  }
}
for (const b of $$("[data-click]")) b.onclick = () => setClickMode(state.clickMode === b.dataset.click ? null : b.dataset.click);

function floorClick(p) {
  const x = Math.round(p.x), y = Math.round(p.y);
  if (state.clickMode === "obst" && state.sim) send(`OBST ADD ${x} ${y} 45`);
  if (state.clickMode === "person" && state.sim) send(`PERSON ${x} ${y}`);
  if (state.clickMode === "goal") {
    if (state.mode !== "stand") return toast("Lève le robot d'abord", "err");
    send(`GOTO ${x} ${y} ${$("#speed").value}`);
    setClickMode(null);
  }
}

const NAV_LABELS = { off: "manuel", goto: "va au point", follow: "suit" };
function renderNav() {
  const el = $("#nav-state");
  el.textContent = NAV_LABELS[state.nav] || state.nav;
  el.className = "sensor-state " + (state.nav === "off" ? "" : "good");
}
$("#nav-stop").onclick = () => send("NAV STOP");

// -------------------------------------------------------------- gyroscope

function renderImu() {
  const { present, level } = state.imu;
  const connected = state.connected;
  const el = $("#imu-state");
  el.textContent = !connected ? "–" : !present ? "absent" : level ? "équilibre" : "détecté";
  el.className = "sensor-state " + (!connected || !present ? "" : level ? "good" : "idle");
  const box = $("#imu-level");
  box.disabled = !connected || !present;
  box.checked = connected && level;
  const feel = $("#imu-feel");
  feel.disabled = !connected || !present;
  feel.checked = connected && state.imu.feel;
  for (const id of ["#imu-zero", "#imu-cal", "#imu-turn", "#imu-flip"]) $(id).disabled = !connected || !present;
  const deg = (v) => `${v > 0 ? "+" : ""}${v.toFixed(1)}°`;
  const [zr, zp] = state.imu.zero;
  $("#imu-setup-state").textContent = !connected || !present ? "Carte du gyroscope : –"
    : `Carte du gyroscope : axes ${state.imu.axes}, zéro ${deg(zr)} / ${deg(zp)}`;
}

// Orientation de la carte (commande IMU AXES) : pour l'avant, la gauche et le
// haut du robot, l'axe de la carte et son signe, ex. « +y-x+z ». Les deux
// boutons passent d'une façon de la visser à plat à une autre (8 en tout),
// toujours par une rotation : le robot refuserait autre chose.
function imuAxes() {
  const a = [...state.imu.axes.matchAll(/([+-])([xyz])/g)].map((m) => [m[2], m[1] === "-" ? -1 : 1]);
  return a.length === 3 ? a : [["x", 1], ["y", 1], ["z", 1]];
}
const sendAxes = (a) => send("IMU AXES " + a.map(([axis, sign]) => (sign < 0 ? "-" : "+") + axis).join(""));
$("#imu-turn").onclick = () => {
  const [x, y, z] = imuAxes();
  sendAxes([y, [x[0], -x[1]], z]); // un quart de tour autour de la verticale
  toast("Carte tournée d'un quart de tour : ne pas bouger le robot 1 s", "");
};
$("#imu-flip").onclick = () => {
  const [x, y, z] = imuAxes();
  sendAxes([x, [y[0], -y[1]], [z[0], -z[1]]]); // un demi-tour autour de l'axe avant
  toast("Carte retournée : ne pas bouger le robot 1 s", "");
};
$("#imu-zero").onclick = () => send("IMU ZERO");
$("#imu-cal").onclick = () => {
  send("IMU CAL");
  toast("Gyroscope : mesure du biais, ne pas bouger le robot 1 s", "");
};

function drawImu() {
  const imu = state.imu;
  const live = state.connected && imu.present && performance.now() - imu.at < 1000;
  const fmt = (v) => (live ? `${v > 0 ? "+" : ""}${v.toFixed(1)}°` : "–");
  $("#imu-roll").textContent = fmt(imu.roll);
  $("#imu-pitch").textContent = fmt(imu.pitch);
  $("#imu-corr").textContent = live && imu.level ? `${fmt(imu.corrRoll)} / ${fmt(imu.corrPitch)}` : "–";
  // Horizon artificiel : le ciel en haut, le sol en bas, vus depuis l'arrière du robot.
  const cv = $("#horizon");
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const w = cv.clientWidth || 120;
  if (cv.width !== Math.round(w * dpr)) {
    cv.width = cv.height = Math.round(w * dpr);
    cv.style.height = w + "px";
  }
  const ctx = cv.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const r = w / 2;
  ctx.clearRect(0, 0, w, w);
  ctx.save();
  ctx.beginPath();
  ctx.arc(r, r, r - 1, 0, Math.PI * 2);
  ctx.clip();
  ctx.translate(r, r);
  const deg = Math.PI / 180;
  // Roulis positif = côté gauche qui monte (le robot penche à droite) :
  // l'horizon tourne dans l'autre sens. Tangage positif = nez qui plonge :
  // l'horizon monte.
  ctx.rotate(live ? -imu.roll * deg : 0);
  const shift = -(live ? imu.pitch : 0) * (r / 20); // 20° = demi-hauteur
  ctx.fillStyle = "#1f3b63";
  ctx.fillRect(-w, -w, 2 * w, w + shift);
  ctx.fillStyle = "#4a3520";
  ctx.fillRect(-w, shift, 2 * w, w);
  ctx.strokeStyle = "rgba(255,255,255,.75)";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(-w, shift);
  ctx.lineTo(w, shift);
  ctx.stroke();
  ctx.strokeStyle = "rgba(255,255,255,.3)";
  ctx.lineWidth = 1;
  for (const d of [-10, 10]) {
    const y = shift - d * (r / 20);
    ctx.beginPath();
    ctx.moveTo(-r * 0.25, y);
    ctx.lineTo(r * 0.25, y);
    ctx.stroke();
  }
  ctx.restore();
  // Maquette du robot, fixe au centre.
  ctx.strokeStyle = "#fbbf24";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(r * 0.35, r);
  ctx.lineTo(r * 0.8, r);
  ctx.moveTo(r * 1.2, r);
  ctx.lineTo(r * 1.65, r);
  ctx.stroke();
  ctx.fillStyle = "#fbbf24";
  ctx.beginPath();
  ctx.arc(r, r, 3, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = "rgba(255,255,255,.16)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.arc(r, r, r - 1, 0, Math.PI * 2);
  ctx.stroke();
}

$("#imu-level").onchange = (e) => send(`LEVEL ${e.target.checked ? 1 : 0}`);
$("#imu-feel").onchange = (e) => send(`FEEL ${e.target.checked ? 1 : 0}`);
sliders.tiltRoll = bindSlider($("#tilt-roll"), (v) => `${v}°`);
sliders.tiltPitch = bindSlider($("#tilt-pitch"), (v) => `${v}°`);
const sendTilt = throttle(() => send(`TILT ${$("#tilt-roll").value} ${$("#tilt-pitch").value}`, true), 150);
$("#tilt-roll").addEventListener("input", sendTilt);
$("#tilt-pitch").addEventListener("input", sendTilt);
$("#terrain").onchange = (e) => send(`TERRAIN ${e.target.checked ? 1 : 0}`);

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

// ---------------------------------------------------------- mesures

// Une ligne M par télémétrie (20 par seconde) : on garde 30 s pour les copier.
const metrics = { rows: [], att: [0, 0, 0, 0], shownAt: 0, prof: [] };
const METRICS_KEEP = 30000; // ms

function gotMetrics([speed, wanted, margin, pose, lift, tickUs]) {
  const now = performance.now();
  const [roll, pitch, groundRoll, groundPitch] = metrics.att;
  metrics.rows.push([now, state.mode, speed, wanted, margin / 10, pose, lift,
    +(roll + groundRoll).toFixed(1), +(pitch + groundPitch).toFixed(1), tickUs / 1000]);
  while (metrics.rows.length && now - metrics.rows[0][0] > METRICS_KEEP) metrics.rows.shift();
  if (now - metrics.shownAt < 200) return; // 5 rafraîchissements par seconde suffisent
  metrics.shownAt = now;
  const set = (id, text, cls = "") => {
    const el = $(id);
    el.textContent = text;
    el.className = cls;
  };
  set("#m-speed", speed);
  set("#m-wanted", wanted);
  set("#m-margin", (margin / 10).toFixed(1), margin < 30 ? "bad" : margin < 60 ? "warn" : "");
  set("#m-scale", pose + " · " + lift, Math.min(pose, lift) < 95 ? "warn" : "");
  set("#m-att", (roll + groundRoll).toFixed(1) + " / " + (pitch + groundPitch).toFixed(1));
  set("#m-tick", (tickUs / 1000).toFixed(1), tickUs > 15000 ? "bad" : tickUs > 8000 ? "warn" : "");
}

// Lignes PROF (une par seconde, avec « Détail du calcul ») : temps moyen de
// chaque partie du calcul du robot, ajoutées à la copie.
function gotProf(rest) {
  const now = performance.now();
  metrics.prof.push([now, state.mode, rest]);
  while (metrics.prof.length && now - metrics.prof[0][0] > METRICS_KEEP) metrics.prof.shift();
}

$("#metrics-prof").addEventListener("change", (e) => {
  if (!state.connected) {
    e.target.checked = false;
    return toast("Connecte d'abord le robot ou la simulation", "err");
  }
  send("PROF " + (e.target.checked ? 1 : 0));
});

function metricsText() {
  const rows = metrics.rows;
  if (!rows.length) return "";
  const prof = metrics.prof.length
    ? ["# Détail du calcul (ms en moyenne par cycle, une ligne par seconde)",
       ...metrics.prof.map(([, mode, r]) => "# PROF " + mode + " " + r)]
    : [];
  const head = "# SPY-BOT mesures (" + (state.sim ? "simulation" : "robot") + "), " + new Date().toLocaleString("fr-FR") +
    "\n# t_s mode v_reelle v_demandee marge_servo_deg pose_% levee_% roulis_deg tangage_deg tick_ms";
  // Une ligne M par télémétrie (toutes les 50 ms en simulation, 100 ms sur
  // l'ESP32) ; elles arrivent parfois groupées : l'heure de réception est
  // arrondie à la période la plus proche.
  const step = rows.length > 1 ? Math.max(0.05, Math.round((rows[rows.length - 1][0] - rows[0][0]) / (rows.length - 1) / 50) * 0.05) : 0.05;
  const body = rows.map(([, ...r], i) => [(i * step).toFixed(2), ...r].join(" "));
  return [head, ...prof, ...body].join("\n");
}

$("#metrics-copy").addEventListener("click", async (e) => {
  const text = metricsText();
  if (!text) return toast("Pas encore de mesures : connecte le robot ou la simulation", "err");
  const area = $("#metrics-text");
  area.value = text;
  try {
    await navigator.clipboard.writeText(text);
    area.hidden = true;
    toast(metrics.rows.length + " mesures copiées, colle-les dans la discussion", "ok");
  } catch {
    // Presse-papiers refusé : on montre le texte sélectionné, Ctrl+C le copie.
    area.hidden = false;
    area.focus();
    area.select();
    toast("Copie bloquée : le texte est sélectionné, fais Ctrl+C", "");
  }
});

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
  const key = text.replace(/\d+/g, "#"); // « 14 écritures perdues » puis « 48 … » : même message
  if (now - (recentToasts.get(key) || -1e9) < 3500) return;
  recentToasts.set(key, now);
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

function frame() {
  drive = readDrive();
  stickMove.render();
  stickTurn.render();
  $("#r-vx").textContent = drive.vx;
  $("#r-vy").textContent = drive.vy;
  $("#r-yaw").textContent = drive.yaw;
  playAnimation();
  updateLimits();
  view.update(state.angles);
  view.render();
  if (activeTab === "drive") radar.draw();
  if (activeTab === "body") drawImu();
  requestAnimationFrame(frame);
}

updateControls();
renderSelection();
renderSensor();
renderImu();
refreshPorts();
requestAnimationFrame(frame);
