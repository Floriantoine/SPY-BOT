#include "controller.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "pyfmt.h"

static const char* const MODE_NAMES[] = {"off", "wake", "sit", "standing", "stand", "sitting", "cal", "test"};
static const char* const JOINT_NAMES[3] = {"coxa", "femur", "tibia"};

static double clampv(double v, double limit) { return pymax(-limit, pymin(limit, v)); }

static double py_sqrt_sum_sq(double a, double b) { return sqrt(pow(a, 2.0) + pow(b, 2.0)); }

// =============================================================================

Controller::Controller(ServoDriver* driver, CalibStore* store, Ranger* ranger, Imu* imu_)
    : telemetry_period(TELEMETRY_PERIOD),
      mode(MODE_OFF),
      angles(NULL),
      robot(driver, &pool_),
      scanner(ranger, driver),
      imu(imu_),
      tilted(false),
      driver_(driver),
      store_(store) {
    pool_.hold(&angles);
    pool_.hold(&margin_of_);
    pool_.hold(&wake_to_);
    margin_of_ = NULL;
    wake_to_ = NULL;
    wanted_ = gait_command();
    odom_[0] = odom_[1] = odom_[2] = 0.0;
    sit_drop_ = CFG_SIT_DROP;
    height_ = -sit_drop_;
    user_pose_ = body_pose();
    pose_limited_ = false;
    free_time_ = went_ = 0.0;
    tick_us_ = 0;
    margin_ = 0.0;
    since_move_ = 0.0;
    since_line_ = 0.0;
    since_telemetry_ = TELEMETRY_PERIOD;
    wake_known_ = false;
    wake_time_ = 0.0;
    stand_after_wake_ = false;
    test_time_ = 0.0;
    test_leg_ = test_joint_ = 0;
    upright_time_ = 0.0;
    walk_gait_ = robot.six;
    pending_len_ = 0;
    pending_[0] = 0;
    out_len_ = 0;
    prof_time_ = 0.0;
    prof_cycles_ = 0;
    memset(&g_prof, 0, sizeof(g_prof));
    // _safe_solve() or _zero_angles()
    apply_pose();
    AngleSet* a = pool_.alloc();
    if (robot.solve(robot.pose, 1.0, a)) {
        angles = a;
    } else {
        for (int i = 0; i < NLEGS; i++) {
            a->leg[i].coxa = g_legs[i].joint[0].zero_deg * DEG;
            a->leg[i].femur = g_legs[i].joint[1].zero_deg * DEG;
            a->leg[i].tibia = g_legs[i].joint[2].zero_deg * DEG;
        }
        angles = a;
    }
    load_calibration();
}

// --- sortie -------------------------------------------------------------------

void Controller::out_line(const char* fmt, ...) {
    if (out_len_ && out_len_ < OUT_SIZE - 1) out_[out_len_++] = '\n';
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(out_ + out_len_, OUT_SIZE - out_len_, fmt, ap);
    va_end(ap);
    if (n > 0) out_len_ += (size_t)n;
    if (out_len_ >= OUT_SIZE) out_len_ = OUT_SIZE - 1;
}

const char* Controller::err(const char* name, const char* fmt, ...) {
    int k = snprintf(reply_, OUT_SIZE, "ERR %s : ", name);
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(reply_ + k, OUT_SIZE - k, fmt, ap);
    va_end(ap);
    return reply_;
}

const char* Controller::err_float(const char* name, const char* arg) {
    char r[160];
    pyrepr_str(arg, r, sizeof(r));
    return err(name, "could not convert string to float: %s", r);
}

// --- boucle -------------------------------------------------------------------

void Controller::pend_line(const char* text) {
    size_t n = strlen(text);
    if (pending_len_ + n + 2 > sizeof(pending_)) return;
    if (pending_len_) pending_[pending_len_++] = '\n';
    memcpy(pending_ + pending_len_, text, n + 1);
    pending_len_ += n;
}

GaitCommand Controller::filter(const GaitCommand& wanted, bool to_pending) {
    // Consigne envoyée à la démarche : l'avance est coupée si un obstacle
    // barre le chemin (tourner et reculer restent possibles).
    bool was = scanner.blocked;
    bool blocked = scanner.check(odom_, wanted);
    if (blocked != was) {
        if (to_pending)
            pend_line(blocked ? "OBSTACLE 1" : "OBSTACLE 0");
        else
            out_line("OBSTACLE %d", (int)blocked);
    }
    if (blocked) return gait_command(0.0, 0.0, wanted.yaw_rate);
    return wanted;
}

void Controller::auto_sensor() {
    // Se déplacer seul sans regarder devant soi : le capteur s'allume.
    if (scanner.present() && !scanner.enabled) {
        scanner.enable(true);
        pend_line(sensor_status());
    }
}

void Controller::govern(double dt) {
    // Corps qui bascule vite : la démarche ralentit, puis reprend sa vitesse.
    Hexapod& r = robot;
    if (!r.feel_ground) {
        r.time_scale = 1.0;
        return;
    }
    double fr = r.feel_rate[0], fp = r.feel_rate[1];
    double rate = sqrt(fr * fr + fp * fp);
    double goal = pymax(SLOW_MIN, pymin(1.0, 1.0 - (rate - SLOW_RATE) / SLOW_SPAN));
    if (goal < r.time_scale)
        r.time_scale = goal;
    else
        r.time_scale = pymin(goal, r.time_scale + SLOW_RECOVER * dt);
}

void Controller::tilt_guard(double dt) {
    // Arrêt de sécurité : penché de plus de TILT_STOP ; repart sous TILT_OK depuis 1 s.
    const Attitude& att = imu->attitude;
    double tilt = pymax(fabs(att.roll), fabs(att.pitch));
    if (tilt > TILT_STOP) {
        upright_time_ = 0.0;
        if (!tilted) {
            tilted = true;
            out_line("TILT 1");
        }
    } else if (tilted) {
        upright_time_ = tilt < TILT_OK ? upright_time_ + dt : 0.0;
        if (upright_time_ >= 1.0) {
            tilted = false;
            out_line("TILT 0");
        }
    }
}

const char* Controller::tick(double dt) {
    unsigned long t0 = platform_micros();
    out_len_ = 0;
    out_[0] = 0;
    if (pending_len_) {  // lignes en attente (réponses différées des commandes)
        out_line("%s", pending_);
        pending_len_ = 0;
        pending_[0] = 0;
    }
    Hexapod& r = robot;
    char st[96];
    since_line_ += dt;
    if (mode != MODE_STAND) {
        wanted_ = gait_command();
        if (nav.active()) {
            nav.stop();
            nav.status(st, sizeof(st));
            out_line("%s", st);
        }
    }
    if (mode == MODE_STAND) {
        since_move_ += dt;
        if (nav.active() && since_line_ > LINK_TIMEOUT) {
            // Plus rien du PC (câble, appli fermée) : le robot ne continue pas seul.
            nav.stop();
            wanted_ = gait_command();
            nav.status(st, sizeof(st));
            out_line("%s", st);
        } else if (nav.active()) {
            // Le robot se pilote seul : pas besoin de MOVE.
            since_move_ = 0.0;
            NavEvent event;
            wanted_ = nav.command(odom_, &scanner, dt, &event);
            if (event != NAV_NONE) {
                out_line("NAV %s", event == NAV_ARRIVED ? "arrived" : "stuck");
                nav.status(st, sizeof(st));
                out_line("%s", st);
            }
        } else if (since_move_ > WATCHDOG) {
            wanted_ = gait_command();
        }
        r.command = filter(wanted_, false);
        if (tilted) r.command = gait_command();  // arrêt de sécurité : trop penché
    } else if (mode == MODE_STANDING) {
        height_ = pymin(0.0, height_ + sit_drop_ / RISE_TIME * dt);
        if (height_ == 0.0) mode = MODE_STAND;
    } else if (mode == MODE_SITTING) {
        r.command = gait_command();
        if (r.at_rest() && r.tuck_settled()) {
            height_ = pymax(-sit_drop_, height_ - sit_drop_ / RISE_TIME * dt);
            if (height_ == -sit_drop_) mode = MODE_SIT;
        }
    } else if (mode == MODE_WAKE) {
        wake_tick(dt);
    } else if (mode == MODE_TEST) {
        test_time_ += dt;
        angles = calibration_angles();
        if (test_time_ < TEST_DURATION) {
            LegAngles& a = angles->leg[test_leg_];
            double offset = TEST_AMPLITUDE * sin(2 * PY_PI * test_time_);
            if (test_joint_ == 0)
                a.coxa = a.coxa + offset;
            else if (test_joint_ == 1)
                a.femur = a.femur + offset;
            else
                a.tibia = a.tibia + offset;
        } else {
            mode = MODE_CAL;
        }
        send_angles();
    }

    if (mode == MODE_SIT || mode == MODE_STANDING || mode == MODE_STAND || mode == MODE_SITTING) {
        apply_pose();
        r.replace_feet = mode == MODE_STAND;  // pas sur place seulement debout
        AngleSet* a;
        if (r.update(dt, &a))
            angles = a;
        else
            r.command = gait_command();  // on garde la dernière position atteignable
        if (r.six != walk_gait_) out_line("%s", walk_status());  // tripode <-> vague (WAVE AUTO)
        integrate(r.travel);
        went_ += py_sqrt_sum_sq(r.travel[0], r.travel[1]);
        // Pose réduite par le robot (servos en butée sinon) : prévenir l'appli.
        free_time_ = r.pose_scale < 0.99 ? 0.0 : free_time_ + dt;
        if (r.pose_scale < 0.95 && !pose_limited_) {
            pose_limited_ = true;
            out_line("LIMIT pose 1");
        } else if (pose_limited_ && free_time_ > 1.5) {
            pose_limited_ = false;
            out_line("LIMIT pose 0");
        }
    }

    if (imu != NULL) {
        unsigned long tg = g_prof.on ? platform_micros() : 0;
        imu->update(dt);
        if (g_prof.on) g_prof.add(PROF_GYRO, tg);
        const Attitude& att = imu->attitude;
        if (mode == MODE_STAND && att.ready) {
            const BodyPose& u = user_pose_;
            leveler.update(dt, att.roll, att.pitch, u.roll, u.pitch);
            r.feel(dt, att.rate[0], att.rate[1]);
            govern(dt);
            tilt_guard(dt);
        } else {
            leveler.relax(dt);
            r.time_scale = 1.0;
            if (tilted && !att.ready) {
                // Assiette inconnue (gyroscope perdu, biais remesuré) : plus d'arrêt de sécurité.
                tilted = false;
                out_line("TILT 0");
            }
        }
    }
    if (mode != MODE_OFF) {  // servos relâchés : le balayage attend
        unsigned long ts = g_prof.on ? platform_micros() : 0;
        char line[64];
        if (scanner.tick(dt, odom_, wanted_, line, sizeof(line))) out_line("%s", line);
        if (g_prof.on) g_prof.add(PROF_CAPTEUR, ts);
    }

    since_telemetry_ += dt;
    if (since_telemetry_ >= telemetry_period) {
        unsigned long tt = g_prof.on ? platform_micros() : 0;
        telemetry();
        measures(since_telemetry_);
        if (g_prof.on) g_prof.add(PROF_TELEM, tt);
        since_telemetry_ = 0.0;
        // body_odom() : l'odométrie suit le centre de la démarche ; à 4 pattes le corps s'en écarte.
        double x = odom_[0], y = odom_[1], th = odom_[2];
        double sx = r.gait->shift_x, sy = r.gait->shift_y;
        double c = cos(th), s = sin(th);
        double bx = x + c * sx - s * sy, by = y + s * sx + c * sy;
        out_line("P %lld %lld %lld", pyround(bx), pyround(by), pyround(th * 1000));
        if (imu != NULL && imu->attitude.ready) {
            const Attitude& at = imu->attitude;
            out_line("I %lld %lld %lld %lld", pyround(at.roll * 1800 / PY_PI), pyround(at.pitch * 1800 / PY_PI),
                     pyround(leveler.roll * 1800 / PY_PI), pyround(leveler.pitch * 1800 / PY_PI));
        }
        // Assiette du corps par rapport aux pieds ; pente du sol : simulation seulement (0 0).
        const BodyPose& b = r.applied;
        out_line("B %lld %lld %lld %lld %d", pyround(b.roll * 1800 / PY_PI), pyround(b.pitch * 1800 / PY_PI),
                 pyround(0.0 * 1800 / PY_PI), pyround(0.0 * 1800 / PY_PI), stance_mask());
    }
    unsigned long spent = platform_micros() - t0;
    if (spent > tick_us_) tick_us_ = spent;
    if (g_prof.on) {
        g_prof.add(PROF_TICK, t0);
        prof_line(dt);
    }
    return out_len_ ? out_ : NULL;
}

void Controller::measures(double period) {
    // Ligne M : vitesse réelle et demandée (mm/s), marge (0,1°), pose et levée (%), tick (µs).
    if (margin_of_ != angles) {  // angles inchangés : même marge
        margin_of_ = angles;
        double m = leg_margin(g_legs[0], angles->leg[0]);
        for (int i = 1; i < NLEGS; i++) m = pymin(m, leg_margin(g_legs[i], angles->leg[i]));
        margin_ = m;
    }
    const GaitCommand& w = wanted_;
    long long v = period ? pyround(went_ / period) : 0;
    out_line("M %lld %lld %lld %lld %lld %lu", v, pyround(sqrt(w.vx * w.vx + w.vy * w.vy)), pyround(margin_ * 10),
             pyround(robot.pose_scale * 100), pyround(robot.lift_scale * 100), tick_us_);
    went_ = 0.0;
    tick_us_ = 0;
}

void Controller::telemetry() {
    if (out_len_ && out_len_ < OUT_SIZE - 1) out_[out_len_++] = '\n';
    int n = snprintf(out_ + out_len_, OUT_SIZE - out_len_, "T %s %.3f", MODE_NAMES[mode], robot.gait->phase);
    out_len_ += n;
    for (int i = 0; i < NLEGS; i++) {
        const LegAngles& a = angles->leg[i];
        n = snprintf(out_ + out_len_, OUT_SIZE - out_len_, " %lld %lld %lld", pyround(a.coxa * 1000),
                     pyround(a.femur * 1000), pyround(a.tibia * 1000));
        out_len_ += n;
    }
}

bool Controller::prof_line(double dt) {
    prof_time_ += dt;
    prof_cycles_ += 1;
    if (prof_time_ < 1.0) return false;
    long n = prof_cycles_;
    if (out_len_ && out_len_ < OUT_SIZE - 1) out_[out_len_++] = '\n';
    out_len_ += snprintf(out_ + out_len_, OUT_SIZE - out_len_, "PROF cycles %ld", n);
    for (int i = 0; i < PROF_PARTS; i++) {
        if (!g_prof.used[i]) continue;
        out_len_ += snprintf(out_ + out_len_, OUT_SIZE - out_len_, " %s %.1f", PROF_NAMES[i],
                             g_prof.us[i] / 1000.0 / n);
    }
    long mem = platform_free_memory();
    if (mem >= 0) out_len_ += snprintf(out_ + out_len_, OUT_SIZE - out_len_, " memoire_libre %ld", mem);
    for (int i = 0; i < PROF_PARTS; i++) {
        g_prof.us[i] = 0;
        g_prof.used[i] = false;
    }
    prof_cycles_ = 0;
    prof_time_ = 0.0;
    return true;
}

int Controller::stance_mask() const {
    int mask = 0;
    for (int i = 0; i < NLEGS; i++)
        if (robot.walking(i) && robot.offsets[i][2] < 0.5) mask |= 1 << i;
    return mask;
}

void Controller::integrate(const double travel[3]) {
    double dx = travel[0], dy = travel[1], dth = travel[2];
    double x = odom_[0], y = odom_[1], th = odom_[2];
    double mid = th + dth / 2;
    double c = cos(mid), s = sin(mid);
    odom_[0] = x + c * dx - s * dy;
    odom_[1] = y + s * dx + c * dy;
    odom_[2] = th + dth;
}

// --- calibration ----------------------------------------------------------------

void Controller::calibration_json(char* buf, size_t size) {
    size_t k = 0;
    char num[3][40];
    k += snprintf(buf + k, size - k, "{");
    for (int i = 0; i < NLEGS; i++) {
        for (int j = 0; j < 3; j++) {
            const JointServo& js = g_legs[i].joint[j];
            if (js.trim_is_int)
                snprintf(num[j], sizeof(num[j]), "%lld", (long long)js.trim_deg);
            else
                pyrepr_double(js.trim_deg, num[j], sizeof(num[j]));
        }
        k += snprintf(buf + k, size - k, "%s\"%s\": [%s, %s, %s]", i ? ", " : "", g_legs[i].name(), num[0], num[1],
                      num[2]);
    }
    k += snprintf(buf + k, size - k, ", \"invert\": {");
    for (int i = 0; i < NLEGS; i++) {
        const JointServo* j = g_legs[i].joint;
        k += snprintf(buf + k, size - k, "%s\"%s\": [%d, %d, %d]", i ? ", " : "", g_legs[i].name(),
                      (int)j[0].inverted(), (int)j[1].inverted(), (int)j[2].inverted());
    }
    snprintf(buf + k, size - k, "}}");
}

void Controller::load_calibration() {
    double trims[NJOINTS];
    bool inv[NJOINTS], is_int[NJOINTS];
    if (store_ && imu != NULL) {
        // Réglages du gyroscope ; absents d'une ancienne calibration : valeurs de départ.
        int axes[3][2];
        double zero[2];
        if (store_->load_imu(axes, zero) && imu_axes_ok(axes)) {
            memcpy(imu->axes, axes, sizeof(axes));
            imu->attitude.set_zero(zero[0], zero[1]);
        }
    }
    if (!store_ || !store_->load(trims, inv, is_int)) return;
    for (int i = 0; i < NLEGS; i++)
        for (int j = 0; j < 3; j++) {
            JointServo& s = g_legs[i].joint[j];
            s.trim_deg = trims[3 * i + j];
            s.trim_is_int = is_int[3 * i + j];
            s.direction = inv[3 * i + j] ? -s.default_direction : s.default_direction;
            s.refresh();
        }
}

// --- interne --------------------------------------------------------------------

void Controller::apply_pose() {
    const BodyPose& p = user_pose_;
    const Leveler& lv = leveler;
    double limit = 12 * DEG;  // pose + correction d'assiette : reste dans la course des servos
    robot.pose = body_pose(p.x, p.y, height_ + p.z, clampv(p.roll + lv.roll, limit), clampv(p.pitch + lv.pitch, limit),
                           p.yaw);
    robot.base_pose = body_pose(0.0, 0.0, height_);
}

void Controller::send_leg(int leg, const LegAngles& a) {
    unsigned long t0 = g_prof.on ? platform_micros() : 0;
    const JointServo* j = g_legs[leg].joint;
    driver_->set_angle(j[0].board, j[0].channel, ik_to_servo_deg(a.coxa, j[0]));
    driver_->set_angle(j[1].board, j[1].channel, ik_to_servo_deg(a.femur, j[1]));
    driver_->set_angle(j[2].board, j[2].channel, ik_to_servo_deg(a.tibia, j[2]));
    if (g_prof.on) g_prof.add(PROF_SERVOS, t0);
}

void Controller::send_angles() {
    for (int i = 0; i < NLEGS; i++) send_leg(i, angles->leg[i]);
}

AngleSet* Controller::calibration_angles() {
    AngleSet* a = pool_.alloc();
    for (int i = 0; i < NLEGS; i++) {
        a->leg[i].coxa = CALIBRATION_POSE[0];
        a->leg[i].femur = CALIBRATION_POSE[1];
        a->leg[i].tibia = CALIBRATION_POSE[2];
    }
    return a;
}

bool Controller::start_wake() {
    // Position de départ connue seulement si les servos tenaient déjà une pose.
    bool known = mode != MODE_OFF;
    robot.solve_key_valid = false;
    robot.written = NULL;
    wake_known_ = known;
    if (known) wake_from_ = *angles;
    wake_time_ = 0.0;
    stand_after_wake_ = false;
    mode = MODE_WAKE;
    height_ = -sit_drop_;
    robot.command = gait_command();
    robot.reset_gait();
    apply_pose();
    AngleSet* to = pool_.alloc();
    if (!robot.solve(robot.pose, 1.0, to)) {
        if (wake_to_ == NULL) mode = MODE_OFF;  // aucune posture connue : on reste relâché
        return false;
    }
    wake_to_ = to;
    wake_tick(0.0);
    return true;
}

void Controller::wake_tick(double dt) {
    wake_time_ += dt;
    bool done = true;
    for (int i = 0; i < NLEGS; i++) {
        const LegAngles& target = wake_to_->leg[i];
        LegAngles a;
        if (!wake_known_) {
            // Pattes activées l'une après l'autre, directement en posture assise.
            if (wake_time_ + 1e-9 < i * WAKE_STAGGER) {
                done = false;
                continue;
            }
            a = target;
        } else {
            double s = pymin(1.0, wake_time_ / WAKE_BLEND);
            s = s * s * (3 - 2 * s);
            if (s < 1.0) done = false;
            const LegAngles& b = wake_from_.leg[i];
            a.coxa = b.coxa + (target.coxa - b.coxa) * s;
            a.femur = b.femur + (target.femur - b.femur) * s;
            a.tibia = b.tibia + (target.tibia - b.tibia) * s;
        }
        angles->leg[i] = a;  // modifié sur place, comme le dictionnaire Python
        send_leg(i, a);
    }
    if (done) mode = stand_after_wake_ ? MODE_STANDING : MODE_SIT;
}

const char* Controller::sensor_status() {
    scanner.status(status_, sizeof(status_));
    return status_;
}

const char* Controller::imu_status() {
    char ax[8];
    if (imu != NULL) {
        imu_format_axes(imu->axes, ax);
    } else {
        imu_format_axes(IMU_AXES, ax);
    }
    double zr = imu != NULL ? imu->attitude.zero_roll : 0.0, zp = imu != NULL ? imu->attitude.zero_pitch : 0.0;
    snprintf(status_, sizeof(status_), "IMU present %d level %d feel %d ready %d axes %s zero %lld %lld",
             (int)(imu != NULL), (int)leveler.enabled, (int)robot.feel_ground,
             (int)(imu != NULL && imu->attitude.ready), ax, pyround(zr * 1800 / PY_PI), pyround(zp * 1800 / PY_PI));
    return status_;
}

const char* Controller::walk_status() {
    // Ligne WALK : réglages de la marche et démarche à 6 pattes en cours.
    static const char* const WAVE_NAMES[3] = {"0", "1", "auto"};
    const Hexapod& r = robot;
    walk_gait_ = r.six;
    snprintf(status_, sizeof(status_), "WALK wave %s %.0f gait %s shift %.1f", WAVE_NAMES[r.wave_mode], r.wave_speed,
             r.six == &r.wave ? "wave" : "tripod", r.wave_shift);
    return status_;
}

bool Controller::save_imu() {
    const double zero[2] = {imu->attitude.zero_roll, imu->attitude.zero_pitch};
    return store_ != NULL && store_->save_imu(imu->axes, zero);
}

const char* Controller::nav_status_reply(const char* name) {
    char st[96];
    if (nav.status(st, sizeof(st))) {
        snprintf(reply_, OUT_SIZE, "%s", st);
    } else if (strstr(st, "NaN")) {
        snprintf(reply_, OUT_SIZE, "ERR %s : %s", name, st);  // ValueError
    } else {
        snprintf(reply_, OUT_SIZE, "ERR interne : OverflowError('%s')", st);  // pas attrapée par handle()
    }
    return reply_;
}

// =============================================================================
// Commandes
// =============================================================================

struct CmdSpec {
    const char* name;  // en minuscules, comme cmd_<nom> en Python
    int nmin, nmax;    // nmax = -1 : nombre libre (*args)
    const char* params[6];
};

static const CmdSpec COMMANDS[] = {
    {"ping", 0, 0, {0}},
    {"power", 1, 1, {"on"}},
    {"stand", 0, 0, {0}},
    {"sit", 0, 0, {0}},
    {"move", 3, 3, {"vx", "vy", "yaw"}},
    {"pose", 6, 6, {"x", "y", "z", "roll", "pitch", "yaw"}},
    {"gait", 2, 2, {"period", "step_height"}},
    {"legs", 1, 1, {"n"}},
    {"wave", 0, -1, {0}},
    {"shift", 1, 1, {"mm"}},
    {"walk", 0, 0, {0}},
    {"cal", 0, 0, {0}},
    {"test", 2, 2, {"leg_name", "joint_name"}},
    {"trim", 3, 3, {"leg_name", "joint_name", "deg"}},
    {"invert", 3, 3, {"leg_name", "joint_name", "on"}},
    {"save", 0, 0, {0}},
    {"getcal", 0, 0, {0}},
    {"geom", 0, 0, {0}},
    {"sensor", 0, -1, {0}},
    {"goto", 2, 3, {"x", "y", "speed"}},
    {"follow", 0, 1, {"speed"}},
    {"nav", 0, -1, {0}},
    {"person", 2, 2, {"x", "y"}},
    {"imu", 0, -1, {0}},
    {"prof", 1, 1, {"on"}},
    {"feel", 1, 1, {"on"}},
    {"level", 1, 1, {"on"}},
    {"tilt", 2, 2, {"roll", "pitch"}},
    {"terrain", 0, 1, {"on"}},
    {"obst", 0, -1, {0}},
};

static bool is_space(char c) { return c == ' ' || (c >= '\t' && c <= '\r') || (c >= 0x1c && c <= 0x1f); }

const char* Controller::handle(const char* line) {
    enum { MAX_ARGS = 16 };
    static char buf[256];
    snprintf(buf, sizeof(buf), "%s", line);
    char* tokens[MAX_ARGS + 1];
    int ntok = 0, total = 0;
    char* p = buf;
    while (*p) {
        while (*p && is_space(*p)) p++;
        if (!*p) break;
        char* start = p;
        while (*p && !is_space(*p)) p++;
        if (*p) *p++ = 0;
        if (ntok <= MAX_ARGS) tokens[ntok++] = start;
        total++;
    }
    if (!total) return NULL;
    since_line_ = 0.0;  // s depuis la dernière ligne reçue du PC
    char name[64], lower[64];
    snprintf(name, sizeof(name), "%s", tokens[0]);
    for (char* c = name; *c; c++)
        if (*c >= 'a' && *c <= 'z') *c = (char)(*c - 'a' + 'A');
    snprintf(lower, sizeof(lower), "%s", name);
    for (char* c = lower; *c; c++)
        if (*c >= 'A' && *c <= 'Z') *c = (char)(*c - 'A' + 'a');
    const CmdSpec* spec = NULL;
    for (size_t i = 0; i < sizeof(COMMANDS) / sizeof(COMMANDS[0]); i++)
        if (strcmp(COMMANDS[i].name, lower) == 0) spec = &COMMANDS[i];
    if (!spec) {
        snprintf(reply_, OUT_SIZE, "ERR commande inconnue : %s", name);
        return reply_;
    }
    int nargs = total - 1;
    // Mauvais nombre d'arguments : même message que Python (TypeError).
    if (spec->nmax >= 0 && nargs > spec->nmax) {
        if (spec->nmin == spec->nmax)
            return err(name, "Controller.cmd_%s() takes %d positional argument%s but %d were given", lower,
                       spec->nmax + 1, spec->nmax + 1 == 1 ? "" : "s", nargs + 1);
        return err(name, "Controller.cmd_%s() takes from %d to %d positional arguments but %d were given", lower,
                   spec->nmin + 1, spec->nmax + 1, nargs + 1);
    }
    if (nargs < spec->nmin) {
        int missing = spec->nmin - nargs;
        char names[96];
        size_t k = 0;
        for (int i = nargs; i < spec->nmin; i++) {
            int idx = i - nargs;
            const char* sep = "";
            if (idx > 0) sep = (missing == 2) ? " and " : (idx == missing - 1 ? ", and " : ", ");
            k += snprintf(names + k, sizeof(names) - k, "%s'%s'", sep, spec->params[i]);
        }
        return err(name, "Controller.cmd_%s() missing %d required positional argument%s: %s", lower, missing,
                   missing == 1 ? "" : "s", names);
    }
    return dispatch(name, tokens + 1, nargs);
}

bool Controller::parse_joint(const char* name, const char* leg, const char* joint, JointServo** out,
                             const char** error) {
    char j[32], l[32];
    snprintf(j, sizeof(j), "%s", joint);
    for (char* c = j; *c; c++)
        if (*c >= 'A' && *c <= 'Z') *c = (char)(*c - 'A' + 'a');
    int ji = -1;
    for (int i = 0; i < 3; i++)
        if (strcmp(j, JOINT_NAMES[i]) == 0) ji = i;
    if (ji < 0) {
        *error = err(name, "articulation inconnue : %s", joint);
        return false;
    }
    snprintf(l, sizeof(l), "%s", leg);
    for (char* c = l; *c; c++)
        if (*c >= 'a' && *c <= 'z') *c = (char)(*c - 'a' + 'A');
    int li = leg_index(l);
    if (li < 0) {
        *error = err(name, "patte inconnue : %s", leg);
        return false;
    }
    *out = &g_legs[li].joint[ji];
    return true;
}

const char* Controller::cmd_power(const char* on) {
    if (strcmp(on, "1") == 0) {
        if (mode == MODE_OFF || mode == MODE_CAL || mode == MODE_TEST) {
            if (!start_wake()) return err("POWER", "%s", unreachable_message());
        }
        return "OK POWER 1";
    }
    mode = MODE_OFF;
    robot.command = gait_command();
    driver_->relax();
    return "OK POWER 0";
}

static void upper_copy(char* dst, size_t size, const char* src) {
    snprintf(dst, size, "%s", src);
    for (char* c = dst; *c; c++)
        if (*c >= 'a' && *c <= 'z') *c = (char)(*c - 'a' + 'A');
}

static void lower_copy(char* dst, size_t size, const char* src) {
    snprintf(dst, size, "%s", src);
    for (char* c = dst; *c; c++)
        if (*c >= 'A' && *c <= 'Z') *c = (char)(*c - 'A' + 'a');
}

const char* Controller::dispatch(const char* name, char** a, int n) {
    Hexapod& r = robot;
    const char* e = NULL;
    double v[6];
    if (!strcmp(name, "PING")) return "OK PONG";
    if (!strcmp(name, "POWER")) return cmd_power(a[0]);
    if (!strcmp(name, "STAND")) {
        if (mode == MODE_OFF || mode == MODE_CAL || mode == MODE_TEST) {
            // se lever depuis l'arrêt alimente d'abord les servos
            if (strncmp(cmd_power("1"), "ERR", 3) == 0) return err("STAND", "%s", unreachable_message());
        }
        if (mode == MODE_WAKE)
            stand_after_wake_ = true;  // se lèvera une fois le démarrage fini
        else if (mode != MODE_STAND)
            mode = MODE_STANDING;
        return "OK STAND";
    }
    if (!strcmp(name, "SIT")) {
        if (mode == MODE_WAKE) {
            stand_after_wake_ = false;
            return "OK SIT";
        }
        if (mode != MODE_STAND && mode != MODE_STANDING && mode != MODE_SIT) return "ERR SIT : le robot n'est pas debout";
        if (mode != MODE_SIT) mode = MODE_SITTING;
        return "OK SIT";
    }
    if (!strcmp(name, "MOVE")) {
        if (mode != MODE_STAND) return "ERR MOVE : le robot doit être debout";
        if (nav.active()) {
            nav.stop();  // on reprend la main
            char st[96];
            nav.status(st, sizeof(st));
            pend_line(st);
        }
        for (int i = 0; i < 3; i++)
            if (!pyfloat(a[i], &v[i])) return err_float(name, a[i]);
        // nan ou inf : la démarche et l'odométrie ne s'en remettraient pas.
        for (int i = 0; i < 3; i++)
            if (isnan(v[i]) || isinf(v[i])) return err(name, "vitesse invalide");
        wanted_ = gait_command(v[0], v[1], v[2] * DEG);
        r.command = filter(wanted_, true);
        since_move_ = 0.0;
        return NULL;  // envoyé en continu : pas d'accusé de réception
    }
    if (!strcmp(name, "POSE")) {
        for (int i = 0; i < 6; i++)
            if (!pyfloat(a[i], &v[i])) return err_float(name, a[i]);
        user_pose_ = body_pose(clampv(v[0], 20), clampv(v[1], 20), clampv(v[2], 15), clampv(v[3], 8) * DEG,
                               clampv(v[4], 8) * DEG, clampv(v[5], 20) * DEG);
        return NULL;
    }
    if (!strcmp(name, "GAIT")) {
        if (!pyfloat(a[0], &v[0])) return err_float(name, a[0]);
        r.tripod.period = pymax(0.4, pymin(3.0, v[0]));
        // Pas plus haut que prévu par la config : le tibia toucherait la coxa.
        double top = CFG_STEP_HEIGHT;
        if (!pyfloat(a[1], &v[1])) return err_float(name, a[1]);
        r.tripod.step_height = r.gait4.step_height = pymax(5.0, pymin(top, v[1]));
        // À 4 pattes, chaque patte fait son pas à tour de rôle : cycle deux fois plus long.
        r.gait4.period = r.wave.period = 2 * r.tripod.period;
        r.wave.step_height = r.tripod.step_height;
        snprintf(reply_, OUT_SIZE, "OK GAIT %.2f %.0f", r.tripod.period, r.tripod.step_height);
        return reply_;
    }
    if (!strcmp(name, "LEGS")) {
        long k;
        if (!pyint(a[0], &k)) {
            char rs[160];
            pyrepr_str(a[0], rs, sizeof(rs));
            return err(name, "invalid literal for int() with base 10: %s", rs);
        }
        if (k != r.legs && !r.command.is_zero()) return "ERR LEGS : arrêter le robot d'abord";
        int res = (k == 4 || k == 6) ? r.set_legs((int)k) : 1;
        if (res == 1) return "ERR LEGS : 4 ou 6 pattes";
        if (res == 2) return "ERR LEGS : attendre que les pieds soient posés";
        snprintf(reply_, OUT_SIZE, "OK LEGS %d", r.legs);
        return reply_;
    }
    if (!strcmp(name, "WAVE")) {
        if (n) {
            char a0[16];
            upper_copy(a0, sizeof(a0), a[0]);
            if (!strcmp(a0, "AUTO")) {
                if (n > 1) {
                    if (!pyfloat(a[1], &v[0])) return err_float(name, a[1]);
                    r.wave_speed = pymax(WAVE_SPEED_LO, pymin(WAVE_SPEED_HI, v[0]));
                }
                r.wave_mode = WAVE_AUTO;
            } else if (!strcmp(a0, "0") || !strcmp(a0, "1")) {
                r.wave_mode = a0[0] == '1' ? WAVE_ON : WAVE_OFF;
            } else {
                return err(name, "0, 1 ou AUTO [mm/s]");
            }
        }
        return walk_status();
    }
    if (!strcmp(name, "SHIFT")) {
        if (!pyfloat(a[0], &v[0])) return err_float(name, a[0]);
        r.wave_shift = pymax(0.0, pymin(WAVE_SHIFT_MAX, v[0]));
        return walk_status();
    }
    if (!strcmp(name, "WALK")) return walk_status();
    if (!strcmp(name, "CAL")) {
        mode = MODE_CAL;
        r.command = gait_command();
        angles = calibration_angles();
        send_angles();
        return "OK CAL";
    }
    if (!strcmp(name, "TEST")) {
        JointServo* j;
        if (!parse_joint(name, a[0], a[1], &j, &e)) return e;
        if (mode == MODE_STAND || mode == MODE_STANDING || mode == MODE_SITTING)
            return "ERR TEST : asseoir le robot d'abord";
        mode = MODE_TEST;
        r.command = gait_command();
        char l[32], jn[32];
        upper_copy(l, sizeof(l), a[0]);
        lower_copy(jn, sizeof(jn), a[1]);
        test_leg_ = leg_index(l);
        test_joint_ = (int)(j - g_legs[test_leg_].joint);
        test_time_ = 0.0;
        snprintf(reply_, OUT_SIZE, "OK TEST %s %s", l, jn);
        return reply_;
    }
    if (!strcmp(name, "TRIM")) {
        JointServo* j;
        if (!parse_joint(name, a[0], a[1], &j, &e)) return e;
        if (!pyfloat(a[2], &v[0])) return err_float(name, a[2]);
        // _clamp(v, 45) : max(-45, min(45, v)) rend la borne (entière) si v n'est pas dedans.
        bool is_int = !(v[0] < 45.0);
        double t = is_int ? 45.0 : v[0];
        if (!(t > -45.0)) {
            t = -45.0;
            is_int = true;
        }
        j->set_trim(t, is_int);
        if (mode == MODE_CAL) send_angles();
        char l[32], jn[32];
        upper_copy(l, sizeof(l), a[0]);
        lower_copy(jn, sizeof(jn), a[1]);
        snprintf(reply_, OUT_SIZE, "OK TRIM %s %s %.1f", l, jn, j->trim_deg);
        return reply_;
    }
    if (!strcmp(name, "INVERT")) {
        JointServo* j;
        if (!parse_joint(name, a[0], a[1], &j, &e)) return e;
        j->set_inverted(strcmp(a[2], "1") == 0);
        if (mode == MODE_CAL || mode == MODE_TEST) send_angles();
        char l[32], jn[32];
        upper_copy(l, sizeof(l), a[0]);
        lower_copy(jn, sizeof(jn), a[1]);
        snprintf(reply_, OUT_SIZE, "OK INVERT %s %s %d", l, jn, (int)j->inverted());
        return reply_;
    }
    if (!strcmp(name, "SAVE")) {
        double trims[NJOINTS];
        bool inv[NJOINTS], is_int[NJOINTS];
        for (int i = 0; i < NLEGS; i++)
            for (int j = 0; j < 3; j++) {
                trims[3 * i + j] = g_legs[i].joint[j].trim_deg;
                inv[3 * i + j] = g_legs[i].joint[j].inverted();
                is_int[3 * i + j] = g_legs[i].joint[j].trim_is_int;
            }
        if (!store_ || !store_->save(trims, inv, is_int)) return "ERR SAVE : écriture impossible dans la mémoire du robot";
        return "OK SAVE";
    }
    if (!strcmp(name, "GETCAL")) {
        int k = snprintf(reply_, OUT_SIZE, "CAL ");
        calibration_json(reply_ + k, OUT_SIZE - k);
        return reply_;
    }
    if (!strcmp(name, "GEOM")) {
        int d[NJOINTS];
        for (int i = 0; i < NLEGS; i++)
            for (int j = 0; j < 3; j++) d[3 * i + j] = g_legs[i].joint[j].direction;
        snprintf(reply_, OUT_SIZE, GEOM_FORMAT, d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7], d[8], d[9], d[10],
                 d[11], d[12], d[13], d[14], d[15], d[16], d[17]);
        return reply_;
    }
    // --- capteur avant, navigation ---
    if (!strcmp(name, "SENSOR")) {
        char a0[16];
        if (n) upper_copy(a0, sizeof(a0), a[0]);
        if (n && !strcmp(a0, "STOP")) {
            if (n < 2) {
                snprintf(reply_, OUT_SIZE, "ERR interne : IndexError('tuple index out of range')");
                return reply_;
            }
            if (!pyfloat(a[1], &v[0])) return err_float(name, a[1]);
            scanner.stop_dist = pymax(STOP_RANGE_LO, pymin(STOP_RANGE_HI, v[0]));
        } else if (n) {
            if (!strcmp(a[0], "1") && !scanner.present()) return "ERR SENSOR : capteur avant absent";
            scanner.enable(!strcmp(a[0], "1"));
        }
        snprintf(reply_, OUT_SIZE, "%s", sensor_status());
        return reply_;
    }
    if (!strcmp(name, "GOTO")) {
        if (mode != MODE_STAND) return "ERR GOTO : le robot doit être debout";
        auto_sensor();
        for (int i = 0; i < n; i++)
            if (!pyfloat(a[i], &v[i])) return err_float(name, a[i]);
        nav.go_to(v[0], v[1], n > 2, n > 2 ? v[2] : 0.0);
        return nav_status_reply(name);
    }
    if (!strcmp(name, "FOLLOW")) {
        if (mode != MODE_STAND) return "ERR FOLLOW : le robot doit être debout";
        if (!scanner.present()) return "ERR FOLLOW : il faut le capteur avant";
        auto_sensor();
        if (n && !pyfloat(a[0], &v[0])) return err_float(name, a[0]);
        nav.follow(n > 0, n ? v[0] : 0.0);
        return nav_status_reply(name);
    }
    if (!strcmp(name, "NAV")) {
        char a0[16];
        if (n) upper_copy(a0, sizeof(a0), a[0]);
        if (n && !strcmp(a0, "STOP")) {
            nav.stop();
            wanted_ = gait_command();
        }
        return nav_status_reply(name);
    }
    if (!strcmp(name, "PERSON")) return "ERR PERSON : seulement en simulation";
    if (!strcmp(name, "IMU")) {
        if (n) {
            if (imu == NULL) return "ERR IMU : pas de gyroscope";
            char a0[16];
            upper_copy(a0, sizeof(a0), a[0]);
            Attitude& att = imu->attitude;
            if (!strcmp(a0, "CAL")) {
                att.restart();  // biais du gyroscope mesuré de nouveau
            } else if (!strcmp(a0, "AXES")) {
                char spec[32];
                size_t k = 0;
                spec[0] = 0;
                for (int i = 1; i < n && k < sizeof(spec); i++) k += snprintf(spec + k, sizeof(spec) - k, "%s", a[i]);
                if (k >= sizeof(spec)) return err(name, "%s", AXES_HELP);
                int axes[3][2];
                if (!imu_parse_axes(spec, axes, &e)) return err(name, "%s", e);
                memcpy(imu->axes, axes, sizeof(axes));
                att.restart();  // le biais était mesuré avec les anciens axes
                if (!save_imu()) return "ERR IMU : écriture impossible dans la mémoire du robot";
            } else if (!strcmp(a0, "ZERO")) {
                double zr = 0.0, zp = 0.0;
                if (n > 1) {
                    if (strcmp(a[1], "0") != 0) return err(name, "ZERO, ou ZERO 0 pour l'effacer");
                } else {
                    if (!att.ready) return err(name, "gyroscope pas prêt (robot immobile 1 s)");
                    // À plat, le corps a l'assiette que lui donnent les pattes : le reste est l'écart de la carte.
                    zr = att.raw_roll - r.applied.roll;
                    zp = att.raw_pitch - r.applied.pitch;
                    if (pymax(fabs(zr), fabs(zp)) > ZERO_MAX)
                        return err(name, "penché de plus de 10° : poser le robot à plat (ou corriger IMU AXES)");
                }
                att.set_zero(zr, zp);
                if (!save_imu()) return "ERR IMU : écriture impossible dans la mémoire du robot";
            } else {
                return err(name, "CAL, ZERO ou AXES");
            }
        }
        return imu_status();
    }
    if (!strcmp(name, "PROF")) {
        if (strcmp(a[0], "1") != 0) {
            g_prof.on = false;
            return "OK PROF 0";
        }
        if (g_prof.on) return "OK PROF 1";
        memset(&g_prof, 0, sizeof(g_prof));
        g_prof.on = true;
        prof_time_ = 0.0;
        prof_cycles_ = 0;
        return "OK PROF 1";
    }
    if (!strcmp(name, "FEEL")) {
        bool on = !strcmp(a[0], "1");
        if (on && imu == NULL) return "ERR FEEL : pas de gyroscope";
        r.feel_ground = on;
        if (!on) r.time_scale = 1.0;
        return imu_status();
    }
    if (!strcmp(name, "LEVEL")) {
        bool on = !strcmp(a[0], "1");
        if (on && imu == NULL) return "ERR LEVEL : pas de gyroscope";
        leveler.enabled = on;
        return imu_status();
    }
    if (!strcmp(name, "TILT")) return "ERR TILT : seulement en simulation";
    if (!strcmp(name, "TERRAIN")) return "ERR TERRAIN : seulement en simulation";
    if (!strcmp(name, "OBST")) return "ERR OBST : seulement en simulation";
    snprintf(reply_, OUT_SIZE, "ERR commande inconnue : %s", name);
    return reply_;
}
