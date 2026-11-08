#include "robot.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pyfmt.h"

// =============================================================================
// Servos et pattes (config.py, servo.py, robot._joint)
// =============================================================================

Leg g_legs[NLEGS];

void JointServo::refresh() {
    // robot._joint : d = 90 + direction × (deg - zéro) + trim, borné à [min, max] ± marge.
    double u = (min_deg + SERVO_MARGIN - 90.0 - trim_deg) / direction + zero_deg;
    double w = (max_deg - SERVO_MARGIN - 90.0 - trim_deg) / direction + zero_deg;
    lo_rad = (pymin(u, w) - 1e-9) / DEG_PER_RAD;
    hi_rad = (pymax(u, w) + 1e-9) / DEG_PER_RAD;
    k7 = 90.0 - direction * zero_deg + trim_deg;
    k8 = direction * DEG_PER_RAD;
}

void legs_init() {
    for (int i = 0; i < NLEGS; i++) {
        Leg& leg = g_legs[i];
        leg.def = &CFG_LEGS[i];
        for (int j = 0; j < 3; j++) {
            const JointDef& d = CFG_LEGS[i].joint[j];
            JointServo& s = leg.joint[j];
            s.board = d.board;
            s.channel = d.channel;
            s.zero_deg = d.zero_deg;
            s.default_direction = s.direction = d.direction;
            s.trim_deg = 0.0;
            s.trim_is_int = false;
            s.min_deg = d.min_deg;
            s.max_deg = d.max_deg;
            s.refresh();
        }
    }
}

int leg_index(const char* name) {
    for (int i = 0; i < NLEGS; i++)
        if (strcmp(CFG_LEGS[i].name, name) == 0) return i;
    return -1;
}

double ik_to_servo_deg(double angle_rad, const JointServo& j) {
    double deg = 90.0 + j.direction * (angle_rad * SERVO_DEG_PER_RAD - j.zero_deg) + j.trim_deg;
    return pymax(j.min_deg, pymin(j.max_deg, deg));
}

// =============================================================================
// Cinématique inverse (leg_ik.py)
// =============================================================================

static char g_unreachable[96];

const char* unreachable_message() { return g_unreachable; }

bool leg_ik(double x, double y, double z, LegAngles* out) {
    static const double c = CFG_COXA, f = CFG_FEMUR, t = CFG_TIBIA;
    static const double k3 = f + t, k4 = fabs(f - t), k5 = f * f + t * t, k6 = 2 * f * t, k7 = f * f - t * t,
                        k8 = 2 * f;
    double r = sqrt(x * x + y * y) - c;
    double dd = r * r + z * z;
    double d = sqrt(dd);
    if (d > k3 || d < k4 || d == 0) {
        snprintf(g_unreachable, sizeof(g_unreachable), "pied hors de portée : (%.1f, %.1f, %.1f), d=%.1f", x, y, z, d);
        return false;
    }
    double cos_knee = (k5 - dd) / k6;
    double cos_shoulder = (k7 + dd) / (k8 * d);
    cos_knee = cos_knee < -1.0 ? -1.0 : cos_knee > 1.0 ? 1.0 : cos_knee;
    cos_shoulder = cos_shoulder < -1.0 ? -1.0 : cos_shoulder > 1.0 ? 1.0 : cos_shoulder;
    out->coxa = atan2(y, x);
    out->femur = atan2(z, r) + acos(cos_shoulder);
    out->tibia = acos(cos_knee) - PY_PI;
    return true;
}

// tibia_fold_limit - FOLD_MARGIN, en radians (robot._fold_limit_rad).
static double fold_limit_rad(double femur, bool middle) {
    double f0 = FOLD_RAD[0][0], t0 = FOLD_RAD[0][1];
    double v;
    if (femur <= f0) {
        v = t0;
    } else {
        v = FOLD_RAD[FOLD_POINTS - 1][1];
        for (int i = 1; i < FOLD_POINTS; i++) {
            double f1 = FOLD_RAD[i][0], t1 = FOLD_RAD[i][1];
            if (femur <= f1) {
                v = t0 + (t1 - t0) * (femur - f0) / (f1 - f0);
                break;
            }
            f0 = f1;
            t0 = t1;
        }
    }
    if (femur <= BATTERY_RAD[0] && v > BATTERY_RAD[1]) v = BATTERY_RAD[1];
    if (middle && femur <= STRAP_RAD[0] && v > STRAP_RAD[1]) v = STRAP_RAD[1];
    return v;
}

double leg_margin(const Leg& leg, const LegAngles& a) {
    const JointServo* c = &leg.joint[0];
    double worst = a.coxa - c->lo_rad;
    double v = c->hi_rad - a.coxa;
    if (v < worst) worst = v;
    c = &leg.joint[1];
    v = a.femur - c->lo_rad;
    if (v < worst) worst = v;
    v = c->hi_rad - a.femur;
    if (v < worst) worst = v;
    c = &leg.joint[2];
    v = a.tibia - c->lo_rad;
    if (v < worst) worst = v;
    v = c->hi_rad - a.tibia;
    if (v < worst) worst = v;
    double fold = fold_limit_rad(a.femur, leg.def->middle) + a.tibia + K_HALF_PI;
    worst = worst * DEG_PER_RAD + SERVO_MARGIN;
    fold = fold * DEG_PER_RAD + FOLD_MARGIN;
    return fold < worst ? fold : worst;
}

bool leg_ok(const Leg& leg, const LegAngles& a) {
    const JointServo* c = &leg.joint[0];
    double v = a.coxa;
    if (v < c->lo_rad || v > c->hi_rad) return false;
    c = &leg.joint[1];
    double femur = a.femur;
    if (femur < c->lo_rad || femur > c->hi_rad) return false;
    c = &leg.joint[2];
    v = a.tibia;
    if (v < c->lo_rad || v > c->hi_rad) return false;
    if (femur <= K_MINUS_HALF_PI || -v - K_HALF_PI > fold_limit_rad(femur, leg.def->middle)) return false;
    // Fémur levé : la coxa doit moins tourner (bord de la plaque du haut).
    return femur <= RAD_45 || fabs(a.coxa) <= RAD_55 - (femur - RAD_45) / 3.0;
}

bool neighbours_ok(const AngleSet& angles) {
    for (int i = 0; i < NLEGS; i++) {
        int n = CFG_AROUND[i], m = CFG_AROUND[(i + 1) % NLEGS];
        if ((angles.leg[n].coxa - angles.leg[m].coxa) * DEG_PER_RAD > NEIGHBOUR_LIMIT) return false;
    }
    return true;
}

// =============================================================================
// Corps (body_ik.py)
// =============================================================================

struct Rot {
    double m[3][3];
};

static Rot rotation(double roll, double pitch, double yaw) {
    double cr = cos(roll), sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw), sy = sin(yaw);
    Rot r = {{{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
              {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
              {-sp, cp * sr, cp * cr}}};
    return r;
}

// Pied dans le repère du corps déplacé (foot_in_body), puis de la patte (body_to_leg_frame).
static void foot_to_leg(const double foot[3], const BodyPose& pose, const Rot& r, int leg, double out[3]) {
    double dx = foot[0] - pose.x, dy = foot[1] - pose.y, dz = foot[2] - pose.z;
    double bx = r.m[0][0] * dx + r.m[1][0] * dy + r.m[2][0] * dz;
    double by = r.m[0][1] * dx + r.m[1][1] * dy + r.m[2][1] * dz;
    double bz = r.m[0][2] * dx + r.m[1][2] * dy + r.m[2][2] * dz;
    const LegDef& d = CFG_LEGS[leg];
    double ex = bx - d.mount_x, ey = by - d.mount_y;
    double c = d.cos_a, s = d.sin_a;
    out[0] = c * ex + s * ey;
    out[1] = -s * ex + c * ey;
    out[2] = bz;
}

// =============================================================================
// Réserve d'AngleSet
// =============================================================================

AngleSet* AnglePool::alloc() {
    for (int i = 0; i < SLOTS; i++) {
        bool used = false;
        for (int h = 0; h < nholders_ && !used; h++) used = *holders_[h] == &slots_[i];
        if (!used) return &slots_[i];
    }
    return &slots_[0];  // n'arrive pas : moins de détenteurs que de cases
}

// =============================================================================
// Démarche tripode (gait.py)
// =============================================================================

void TripodGait::advance(double dt, bool moving) {
    double new_phase = phase + dt / period;
    if (!moving) {
        if (at_rest()) return;
        new_phase = pymin(new_phase, phase < 0.5 ? 0.5 : 1.0);
    }
    phase = pymod(new_phase, 1.0);
}

double TripodGait::swing(int leg) const {
    double p = CFG_LEGS[leg].tripod_a ? phase : pymod(phase + 0.5, 1.0);
    return p < 0.5 ? p / 0.5 : -1.0;
}

double TripodGait::time_to_lift(int leg) const {
    double p = CFG_LEGS[leg].tripod_a ? phase : pymod(phase + 0.5, 1.0);
    return (1.0 - p) * period;
}

// =============================================================================
// PROF
// =============================================================================

const char* const PROF_NAMES[PROF_PARTS] = {"i2c", "ik", "pieds", "place", "servos", "telem", "tick", "update"};
Profiler g_prof;

void Profiler::add(int part, unsigned long t0) {
    us[part] += platform_micros() - t0;
    used[part] = true;
}

struct ProfScope {
    int part;
    unsigned long t0;
    explicit ProfScope(int p) : part(p), t0(g_prof.on ? platform_micros() : 0) {}
    ~ProfScope() {
        if (g_prof.on) g_prof.add(part, t0);
    }
};

// =============================================================================
// Hexapod (robot.py)
// =============================================================================

static double py_sq(double v) { return pow(v, 2.0); }  // v ** 2 de Python

static double rate(double value, double target, double accel, double decel) {
    // Freinage (vers zéro, ou changement de sens) plus fort que l'accélération.
    return (value * target > 0 && fabs(target) > fabs(value)) ? accel : (value != 0.0 ? decel : accel);
}

static double approach(double value, double target, double step) {
    if (value < target) return pymin(target, value + step);
    return pymax(target, value - step);
}

static double fraction_inside(double x, double y, double mx, double my, double radius) {
    if (py_sq(x + mx) + py_sq(y + my) <= radius * radius) return 1.0;
    double a = mx * mx + my * my;
    double b = 2 * (x * mx + y * my);
    double c = x * x + y * y - radius * radius;
    if (c >= 0) return b <= 0 ? 1.0 : 0.0;
    return pymax(0.0, pymin(1.0, (-b + sqrt(b * b - 4 * a * c)) / (2 * a)));
}

Hexapod::Hexapod(ServoDriver* d, AnglePool* p) : driver(d), pool(p) {
    tripod.period = GAIT_PERIOD;
    tripod.step_height = CFG_STEP_HEIGHT;
    tripod.max_stride = CFG_MAX_STRIDE;
    tripod.phase = 0.0;
    tripod.shift_x = tripod.shift_y = 0.0;
    for (int i = 0; i < NLEGS; i++) {
        neutral[i][0] = CFG_LEGS[i].neutral_x;
        neutral[i][1] = CFG_LEGS[i].neutral_y;
        neutral[i][2] = CFG_LEGS[i].neutral_z;
    }
    command = current = gait_command();
    pose = base_pose = applied = body_pose();
    pose_scale = lift_scale = 1.0;
    memset(offsets, 0, sizeof(offsets));
    memset(ground, 0, sizeof(ground));
    memset(has_lift_, 0, sizeof(has_lift_));
    memset(placed_, 0, sizeof(placed_));
    rest_key_valid_ = false;
    rest_key_phase_ = 0.0;
    last_angles = written = NULL;
    solve_key_valid = false;
    travel[0] = travel[1] = travel[2] = 0.0;
    for (int i = 0; i < NLEGS; i++) {
        double dx = neutral[i][0] - CFG_LEGS[i].mount_x, dy = neutral[i][1] - CFG_LEGS[i].mount_y;
        double dd = sqrt(dx * dx + dy * dy);
        radial_[i][0] = dx / dd;
        radial_[i][1] = dy / dd;
    }
    reach_table(0);
    for (int i = 0; i < NLEGS; i++) home_target_[i] = home_margin(i, body_pose(), 0.0);
    home_wait_ = 0.0;
    replace_feet = true;
    memset(home, 0, sizeof(home));
    memset(home_goal, 0, sizeof(home_goal));
    memset(home_key_, 0, sizeof(home_key_));
    home_checked_valid_ = false;
    reach_cache_valid_ = false;
    memset(halves_, 0, sizeof(halves_));
    pool->hold(&last_angles);
    pool->hold(&written);
}

void Hexapod::reset_gait() {
    tripod.reset();
    current = gait_command();
    memset(offsets, 0, sizeof(offsets));
    memset(has_lift_, 0, sizeof(has_lift_));
    memset(placed_, 0, sizeof(placed_));
    rest_key_valid_ = false;
    memset(home, 0, sizeof(home));
    memset(home_goal, 0, sizeof(home_goal));
    memset(home_key_, 0, sizeof(home_key_));
    home_checked_valid_ = false;
    reach_cache_valid_ = false;
    memset(ground, 0, sizeof(ground));
}

bool Hexapod::at_rest() const {
    double sx = tripod.shift_x, sy = tripod.shift_y;
    return current.is_zero() && tripod.at_rest() && sx * sx + sy * sy < 0.25 && feet_settled();
}

BodyPose Hexapod::effective_pose(double scale) const {
    const BodyPose &b = base_pose, &p = pose;
    if (scale == 1.0) return p;
    return body_pose(b.x + scale * (p.x - b.x), b.y + scale * (p.y - b.y), b.z + scale * (p.z - b.z),
                     b.roll + scale * (p.roll - b.roll), b.pitch + scale * (p.pitch - b.pitch),
                     b.yaw + scale * (p.yaw - b.yaw));
}

// solve(pose, lift, check) : sans contrôle (check=None) quand `checked` est faux.
// Retourne 1 si réussi, 0 si une patte ne passe pas le contrôle, -1 si hors de portée.
static int solve_impl(const Hexapod& h, const BodyPose& pose, double lift, bool checked, AngleSet* out) {
    Rot R = rotation(pose.roll, pose.pitch, pose.yaw);
    double sx = h.tripod.shift_x, sy = h.tripod.shift_y;
    double bx = sx + pose.x, by = sy + pose.y, bz = pose.z;
    for (int i = 0; i < NLEGS; i++) {
        const double* p = h.neutral[i];
        const double* o = h.offsets[i];
        double dx = p[0] + o[0] - bx;
        double dy = p[1] + o[1] - by;
        double dz = p[2] + lift * o[2] + h.ground[i] - bz;
        double fx = R.m[0][0] * dx + R.m[1][0] * dy + R.m[2][0] * dz;
        double fy = R.m[0][1] * dx + R.m[1][1] * dy + R.m[2][1] * dz;
        double fz = R.m[0][2] * dx + R.m[1][2] * dy + R.m[2][2] * dz;
        const LegDef& d = CFG_LEGS[i];
        double ca = d.cos_a, sa = d.sin_a;
        double ex = fx - d.mount_x, ey = fy - d.mount_y;
        if (!leg_ik(ca * ex + sa * ey, ca * ey - sa * ex, fz, &out->leg[i])) return -1;
        if (checked && !leg_ok(g_legs[i], out->leg[i])) return 0;
    }
    return 1;
}

bool Hexapod::solve(const BodyPose& p, double lift, AngleSet* out) { return solve_impl(*this, p, lift, false, out) == 1; }

bool Hexapod::solve_checked(const BodyPose& p, double lift, AngleSet* out) {
    return solve_impl(*this, p, lift, true, out) == 1;
}

bool Hexapod::try_(double scale, double lift, AngleSet* out) {
    return solve_checked(effective_pose(scale), lift, out) && neighbours_ok(*out);
}

bool Hexapod::update(double dt, AngleSet** out) {
    ProfScope prof(PROF_UPDATE);
    // 6 pattes : tuck reste à 0, pas de repli.
    GaitCommand target = command;
    GaitCommand c = current;
    current = gait_command(approach(c.vx, target.vx, rate(c.vx, target.vx, ACCEL, DECEL) * dt),
                           approach(c.vy, target.vy, rate(c.vy, target.vy, ACCEL, DECEL) * dt),
                           approach(c.yaw_rate, target.yaw_rate,
                                    rate(c.yaw_rate, target.yaw_rate, YAW_ACCEL, YAW_DECEL) * dt));
    home_wait_ -= dt;
    if (home_wait_ <= 0.0) {
        bool changed;
        {
            ProfScope pp(PROF_PLACE);
            changed = refresh_home();
        }
        if (changed) home_wait_ = HOME_PERIOD;
    }
    double gdt = dt * 1.0;  // time_scale : 1 sans gyroscope
    tripod.advance(gdt, !(current.is_zero() && feet_settled()));
    {
        ProfScope pp(PROF_PIEDS);
        move_feet(gdt);
    }
    bool unreachable = false;
    AngleSet* angles;
    {
        ProfScope pp(PROF_IK);
        angles = solve_limited(dt, &unreachable);
    }
    if (unreachable) return false;
    if (angles != written) {  // mêmes angles qu'au tick d'avant : déjà envoyés
        ProfScope pp(PROF_SERVOS);
        write_servos(*angles);
        written = angles;
    }
    *out = angles;
    return true;
}

AngleSet* Hexapod::solve_limited(double dt, bool* unreachable) {
    double s = pose_scale, l = lift_scale;
    double rs = pymin(1.0, s + POSE_RECOVER * dt), rl = pymin(1.0, l + POSE_RECOVER * dt);
    const BodyPose &p = pose, &b = base_pose;
    double key[KEY_LEN];
    int k = 0;
    for (int i = 0; i < NLEGS; i++)
        for (int j = 0; j < 3; j++) key[k++] = offsets[i][j];
    const double rest[] = {p.x, p.y, p.z, p.roll, p.pitch, p.yaw, b.x, b.y, b.z, b.roll, b.pitch, b.yaw, s, l, rs, rl};
    for (int i = 0; i < 16; i++) key[k++] = rest[i];
    if (solve_key_valid && last_angles != NULL) {
        bool same = true;
        for (int i = 0; i < KEY_LEN && same; i++) same = key[i] == solve_key_[i];
        if (same) return last_angles;
    }
    AngleSet work;
    bool found = false;
    double fs = 0.0, fl = 0.0;
    double tried[4][2];
    int ntried = 0;
    const double cands[4][2] = {{rs, rl}, {rs, l}, {s, l}, {s, pymax(LIFT_MIN, l - LIFT_STEP)}};
    for (int c = 0; c < 4; c++) {
        double cs = cands[c][0], cl = cands[c][1];
        bool seen = false;
        for (int t = 0; t < ntried; t++) seen = seen || (tried[t][0] == cs && tried[t][1] == cl);
        if (seen) continue;
        tried[ntried][0] = cs;
        tried[ntried][1] = cl;
        ntried++;
        if (try_(cs, cl, &work)) {
            found = true;
            fs = cs;
            fl = cl;
            break;
        }
    }
    if (!found) found = search(rs, rl, &fs, &fl, &work);
    if (!found) {
        // Même sans pose et sans lever les pieds, c'est trop : derniers angles valides.
        if (last_angles != NULL) return last_angles;
        pose_scale = lift_scale = 1.0;
        applied = pose;
        AngleSet* out = pool->alloc();
        if (!solve(pose, 1.0, out)) {
            *unreachable = true;
            return NULL;
        }
        return out;
    }
    pose_scale = fs;
    lift_scale = fl;
    applied = effective_pose(pose_scale);
    AngleSet* out = pool->alloc();
    *out = work;
    last_angles = out;
    solve_key_valid = (rs == fs && rl == fl);
    if (solve_key_valid) memcpy(solve_key_, key, sizeof(key));
    return out;
}

bool Hexapod::search(double top_s, double top_l, double* s_out, double* l_out, AngleSet* out) {
    // _bisect(lambda v: _try(v, LIFT_MIN), top_s)
    AngleSet best, a;
    double s;
    if (try_(top_s, LIFT_MIN, &best)) {
        s = top_s;
    } else {
        double lo = 0.0, hi = top_s;
        if (!try_(0.0, LIFT_MIN, &best)) return false;
        for (int i = 0; i < BISECT; i++) {
            double mid = (lo + hi) / 2;
            if (!try_(mid, LIFT_MIN, &a)) {
                hi = mid;
            } else {
                lo = mid;
                best = a;
            }
        }
        s = lo;
    }
    // _bisect(lambda v: _try(s, LIFT_MIN + (1 - LIFT_MIN) * v), (top_l - LIFT_MIN) / (1 - LIFT_MIN), angles)
    double top = (top_l - LIFT_MIN) / (1.0 - LIFT_MIN);
    double v;
    if (try_(s, LIFT_MIN + (1.0 - LIFT_MIN) * top, &a)) {
        v = top;
        best = a;
    } else {
        double lo = 0.0, hi = top;
        for (int i = 0; i < BISECT; i++) {
            double mid = (lo + hi) / 2;
            if (!try_(s, LIFT_MIN + (1.0 - LIFT_MIN) * mid, &a)) {
                hi = mid;
            } else {
                lo = mid;
                best = a;
            }
        }
        v = lo;
    }
    *s_out = s;
    *l_out = LIFT_MIN + (1.0 - LIFT_MIN) * v;
    *out = best;
    return true;
}

bool Hexapod::feet_settled() const {
    for (int n = 0; n < NLEGS; n++) {
        const double *o = offsets[n], *h = home[n], *g = home_goal[n];
        if (py_sq(o[0] - h[0]) + py_sq(o[1] - h[1]) > STOP_TOL * STOP_TOL) return false;
        if (replace_feet && py_sq(h[0] - g[0]) + py_sq(h[1] - g[1]) > HOME_REPLACE * HOME_REPLACE) return false;
    }
    return true;
}

// --- place des pieds selon la pose ------------------------------------------

static double py_degrees(double x) { return x * (180.0 / PY_PI); }   // math.degrees
static double py_radians(double x) { return x * (PY_PI / 180.0); }   // math.radians

bool Hexapod::refresh_home() {
    const BodyPose &p = pose, &b = base_pose;
    double key[6] = {p.x - b.x, p.y - b.y, p.z - b.z, py_degrees(p.roll - b.roll), py_degrees(p.pitch - b.pitch),
                     py_degrees(p.yaw - b.yaw)};
    if (home_checked_valid_) {
        bool same = true;
        for (int i = 0; i < 6 && same; i++) same = key[i] == home_checked_[i];
        if (same) return false;  // rien n'a changé depuis le dernier tour complet
    }
    for (int n = 0; n < NLEGS; n++) {
        double m = fabs(key[0] - home_key_[n][0]);
        for (int i = 1; i < 6; i++) m = pymax(m, fabs(key[i] - home_key_[n][i]));
        if (m <= HOME_POSE_MM) continue;
        memcpy(home_key_[n], key, sizeof(key));
        BodyPose q = body_pose(key[0], key[1], key[2], py_radians(key[3]), py_radians(key[4]), py_radians(key[5]));
        double r = best_radius(n, q);
        home_goal[n][0] = r * radial_[n][0];
        home_goal[n][1] = r * radial_[n][1];
        return true;
    }
    memcpy(home_checked_, key, sizeof(key));
    home_checked_valid_ = true;
    return false;
}

double Hexapod::best_radius(int leg, const BodyPose& q) {
    double want = home_target_[leg];
    double f0 = home_margin(leg, q, 0.0);
    if (f0 >= want) return 0.0;
    double lo = HOME_RANGE_LO, hi = HOME_RANGE_HI;
    double out = home_margin(leg, q, HOME_STEP), inside = home_margin(leg, q, -HOME_STEP);
    if (pymax(out, inside) <= f0) return 0.0;  // le neutre est déjà le mieux
    double sign = out >= inside ? 1.0 : -1.0;
    double r = sign * HOME_STEP, f = pymax(out, inside);
    double prev_r = 0.0, prev_f = f0;
    while (f < want) {
        double nr = r + sign * HOME_STEP;
        if (nr < lo || nr > hi) return r;
        double nf = home_margin(leg, q, nr);
        if (nf <= f) return r;  // plus rien à gagner plus loin
        prev_r = r;
        prev_f = f;
        r = nr;
        f = nf;
    }
    return prev_r + (r - prev_r) * (want - prev_f) / (f - prev_f);
}

double Hexapod::home_margin(int leg, const BodyPose& q, double r) {
    const double* p = neutral[leg];
    double ux = radial_[leg][0], uy = radial_[leg][1];
    double x0 = p[0] + r * ux, y0 = p[1] + r * uy;
    double h = 0.35 * tripod.max_stride;
    double worst = 1e9;
    const double ks[4][2] = {{0.0, 0.0}, {h, 0.0}, {-h, 0.0}, {0.0, tripod.step_height}};
    Rot R = rotation(q.roll, q.pitch, q.yaw);
    for (int i = 0; i < 4; i++) {
        double k = ks[i][0], dz = ks[i][1];
        double foot[3] = {x0 + k * ux, y0 + k * uy, p[2] + dz};
        double lf[3];
        foot_to_leg(foot, q, R, leg, lf);
        LegAngles a;
        if (!leg_ik(lf[0], lf[1], lf[2], &a)) return -1e9;
        worst = pymin(worst, leg_margin(g_legs[leg], a));
    }
    return worst;
}

void Hexapod::reach_table(int leg) {
    const double* p = neutral[leg];
    const LegDef& d = CFG_LEGS[leg];
    double dist[REACH_DIRS];
    for (int i = 0; i < REACH_DIRS; i++) {
        double a = 2 * PY_PI * i / REACH_DIRS;
        double ux = cos(d.mount_angle + a), uy = sin(d.mount_angle + a);
        // ok(d) : pied à d mm dans cette direction, servos loin des butées.
        struct Ok {
            static bool at(int leg, const double* p, double ux, double uy, double dd) {
                double foot[3] = {p[0] + dd * ux, p[1] + dd * uy, p[2]};
                const LegDef& d = CFG_LEGS[leg];
                double ex = foot[0] - d.mount_x, ey = foot[1] - d.mount_y;
                double x = d.cos_a * ex + d.sin_a * ey, y = -d.sin_a * ex + d.cos_a * ey;
                LegAngles ang;
                if (!leg_ik(x, y, foot[2], &ang)) return false;
                return leg_margin(g_legs[leg], ang) >= REACH_MARGIN && fabs(ang.coxa * DEG_PER_RAD) <= REACH_COXA;
            }
        };
        double lo = 0.0, hi = REACH_MAX;
        if (Ok::at(leg, p, ux, uy, hi)) {
            dist[i] = hi;
            continue;
        }
        for (int k = 0; k < 8; k++) {
            double mid = (lo + hi) / 2;
            if (Ok::at(leg, p, ux, uy, mid))
                lo = mid;
            else
                hi = mid;
        }
        dist[i] = lo;
    }
    int half = REACH_DIRS / 2;
    for (int i = 0; i < REACH_DIRS; i++) reach_[i] = pymin(dist[i], dist[(i + half) % REACH_DIRS]);
}

double Hexapod::half_stride(int leg, double vx, double vy) const {
    const TripodGait& g = tripod;
    double speed = sqrt(vx * vx + vy * vy);
    if (speed < 1e-6) return g.max_stride / 2;
    double a = pymod(atan2(vy, vx) - CFG_LEGS[leg].mount_angle, 2 * PY_PI) * REACH_DIRS / (2 * PY_PI);
    long ia = (long)a;
    int i = (int)(ia % REACH_DIRS);
    double f = a - ia;
    const double* t = reach_;
    double base = g.max_stride / 2;
    double extra = pymax(0.0, t[i] + (t[(i + 1) % REACH_DIRS] - t[i]) * f - base);
    return base + extra * reach_share();
}

double Hexapod::reach_share() const {
    const BodyPose &p = pose, &b = base_pose;
    double deg = DEG_PER_RAD;
    double vals[6] = {fabs(p.x - b.x), fabs(p.y - b.y), fabs(p.z - b.z), 2 * deg * fabs(p.roll - b.roll),
                      2 * deg * fabs(p.pitch - b.pitch), 2 * deg * fabs(p.yaw - b.yaw)};
    double worst = vals[0];
    for (int i = 1; i < 6; i++) worst = pymax(worst, vals[i]);
    return pymax(0.0, 1.0 - worst / REACH_POSE);
}

GaitCommand Hexapod::command_in_reach() {
    const TripodGait& g = tripod;
    const GaitCommand& cmd = current;
    const BodyPose &p = pose, &b = base_pose;
    double key[12] = {cmd.vx, cmd.vy, cmd.yaw_rate, g.stance_time(), g.max_stride, p.x - b.x,
                      p.y - b.y, p.z - b.z, p.roll - b.roll, p.pitch - b.pitch, p.yaw - b.yaw, 0.0};
    if (reach_cache_valid_) {
        bool same = true;
        for (int i = 0; i < 12 && same; i++) same = key[i] == reach_cache_key_[i];
        if (same) return reach_cache_out_;
    }
    double k = 1.0;
    for (int n = 0; n < NLEGS; n++) {
        // ground_velocity(neutral, cmd)
        double vx = -(cmd.vx - cmd.yaw_rate * neutral[n][1]);
        double vy = -(cmd.vy + cmd.yaw_rate * neutral[n][0]);
        double half = halves_[n] = half_stride(n, vx, vy);
        double step = sqrt(vx * vx + vy * vy) * g.stance_time();
        if (step > 1e-9) k = pymin(k, 0.9 * 2 * half / step);
    }
    GaitCommand out = k >= 1.0 ? cmd : gait_command(cmd.vx * k, cmd.vy * k, cmd.yaw_rate * k);
    memcpy(reach_cache_key_, key, sizeof(key));
    reach_cache_out_ = out;
    reach_cache_valid_ = true;
    return out;
}

void Hexapod::move_feet(double dt) {
    TripodGait& g = tripod;
    GaitCommand cmd = command_in_reach();
    bool rest = cmd.is_zero() && g.at_rest();
    if (rest && rest_key_valid_ && rest_key_phase_ == g.phase) {
        // Immobile, tous les pieds posés, et déjà calculé une fois ainsi : rien ne bouge.
        travel[0] = travel[1] = travel[2] = 0.0;
        return;
    }
    double cvx = cmd.vx, cvy = cmd.vy, yaw = cmd.yaw_rate;
    double st2 = g.stance_time() / 2;
    int stance[NLEGS];
    int nstance = 0;
    for (int n = 0; n < NLEGS; n++) {
        double dx = offsets[n][0], dy = offsets[n][1];
        const double* p = neutral[n];
        double s = g.swing(n);
        if (s < 0.0) {  // au sol
            placed_[n] = false;
            if (has_lift_[n]) {
                has_lift_[n] = false;
                // Il vient de se poser : il finit son pas sur le point visé.
                double hx = home[n][0], hy = home[n][1];
                dx = hx - (yaw * p[1] - cvx) * st2;
                dy = hy + (cvy + yaw * p[0]) * st2;
                offsets[n][0] = dx;
                offsets[n][1] = dy;
                offsets[n][2] = 0.0;
            }
            stance[nstance++] = n;
        } else {
            // En l'air : de là où il s'est levé jusqu'à un demi-pas en avant.
            if (!has_lift_[n]) {
                has_lift_[n] = true;
                lift_[n][0] = dx;
                lift_[n][1] = dy;
            }
            double lx = lift_[n][0], ly = lift_[n][1];
            if (s > 0.0 && !placed_[n]) {
                placed_[n] = true;
                home[n][0] = home_goal[n][0];  // il se pose à sa meilleure place
                home[n][1] = home_goal[n][1];
            }
            double hx = home[n][0], hy = home[n][1];
            double tx = hx - (yaw * p[1] - cvx) * st2, ty = hy + (cvy + yaw * p[0]) * st2;
            double k = s * s * (3 - 2 * s);
            ground[n] = ground[n] * (1.0 - k);
            double z = g.step_height * sin(PY_PI * s);
            offsets[n][0] = lx + (tx - lx) * k;
            offsets[n][1] = ly + (ty - ly) * k;
            offsets[n][2] = z;
        }
    }
    // Au sol, les pieds reculent à la vitesse du corps (même réduction pour tous).
    double moves[NLEGS][2];
    double k = 1.0;
    for (int i = 0; i < nstance; i++) {
        int n = stance[i];
        double dx = offsets[n][0], dy = offsets[n][1];
        const double* p = neutral[n];
        double x = p[0] + dx, y = p[1] + dy;
        double vx = yaw * y - cvx, vy = -cvy - yaw * x;
        moves[n][0] = vx * dt;
        moves[n][1] = vy * dt;
        double lift = g.time_to_lift(n);
        double hx = home[n][0], hy = home[n][1];
        double half = halves_[n];
        if (lift > dt) {
            k = pymin(k, fraction_inside(dx - hx, dy - hy, vx * lift, vy * lift, half));
        } else {
            half += sqrt(vx * vx + vy * vy) * dt;
        }
        k = pymin(k, fraction_inside(dx - hx, dy - hy, vx * dt, vy * dt, half));
    }
    for (int i = 0; i < nstance; i++) {
        int n = stance[i];
        offsets[n][0] = offsets[n][0] + moves[n][0] * k;
        offsets[n][1] = offsets[n][1] + moves[n][1] * k;
        offsets[n][2] = 0.0;
    }
    if (nstance) {
        travel[0] = cmd.vx * k * dt;
        travel[1] = cmd.vy * k * dt;
        travel[2] = cmd.yaw_rate * k * dt;
    } else {
        travel[0] = travel[1] = travel[2] = 0.0;
    }
    rest_key_valid_ = rest;
    rest_key_phase_ = g.phase;
}

void Hexapod::write_servos(const AngleSet& angles) {
    // Angle servo de chaque articulation (= ik_to_servo_deg, constantes gardées).
    bool batch = driver->has_batch();
    double deg[CFG_NBOARDS][16];
    bool have[CFG_NBOARDS][16];
    int board_order[CFG_NBOARDS];
    int nboards = 0;
    memset(have, 0, sizeof(have));
    for (int i = 0; i < NLEGS; i++) {
        const Leg& leg = g_legs[i];
        const double v3[3] = {angles.leg[i].coxa, angles.leg[i].femur, angles.leg[i].tibia};
        for (int j = 0; j < 3; j++) {
            const JointServo& js = leg.joint[j];
            double d = js.k7 + js.k8 * v3[j];
            d = d < js.min_deg ? js.min_deg : d > js.max_deg ? js.max_deg : d;
            if (!batch) {
                driver->set_angle(js.board, js.channel, d);
                continue;
            }
            bool known = false;
            for (int b = 0; b < nboards; b++) known = known || board_order[b] == js.board;
            if (!known) board_order[nboards++] = js.board;
            deg[js.board][js.channel] = d;
            have[js.board][js.channel] = true;
        }
    }
    // Voies qui se suivent sur une carte : une seule écriture I²C.
    for (int b = 0; b < nboards; b++) {
        int board = board_order[b];
        int start = -1, len = 0;
        double run[16];
        for (int ch = 0; ch < 16; ch++) {
            if (!have[board][ch]) continue;
            if (len && ch != start + len) {
                driver->set_angles(board, start, run, len);
                len = 0;
            }
            if (!len) start = ch;
            run[len++] = deg[board][ch];
        }
        if (len) driver->set_angles(board, start, run, len);
    }
}
