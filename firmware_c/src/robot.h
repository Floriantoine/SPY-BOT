// Cinématique inverse, démarche tripode et suivi des pieds : portage de
// hexapod/leg_ik.py, body_ik.py, gait.py et robot.py (6 pattes seulement).
// Les calculs suivent le Python opération par opération (mêmes résultats).
#pragma once
#include "driver.h"
#include "robot_config.h"

struct LegAngles {
    double coxa, femur, tibia;
};

// Angles des 6 pattes. Python crée un nouveau dictionnaire à chaque calcul
// et compare parfois leur identité (« is ») : ici, un pointeur vers une case
// d'une petite réserve (AnglePool) joue ce rôle.
struct AngleSet {
    LegAngles leg[NLEGS];
};

struct BodyPose {
    double x, y, z, roll, pitch, yaw;
};

struct GaitCommand {
    double vx, vy, yaw_rate;
    bool is_zero() const { return vx == 0 && vy == 0 && yaw_rate == 0; }
};

static inline GaitCommand gait_command(double vx = 0.0, double vy = 0.0, double yaw = 0.0) {
    GaitCommand c = {vx, vy, yaw};
    return c;
}

static inline BodyPose body_pose(double x = 0.0, double y = 0.0, double z = 0.0, double roll = 0.0,
                                 double pitch = 0.0, double yaw = 0.0) {
    BodyPose p = {x, y, z, roll, pitch, yaw};
    return p;
}

// --- servos et pattes (config.py) ------------------------------------------

struct JointServo {
    int board, channel;
    double zero_deg;
    int default_direction, direction;
    double trim_deg, min_deg, max_deg;
    // Trim entier en Python (borné à ±45 par _clamp, qui rend alors la borne
    // entière) : GETCAL l'écrit « -45 » et non « -45.0 ».
    bool trim_is_int;
    // Constantes de robot._joint : bornes IK (rad) et angle servo = k7 + k8 × angle.
    double lo_rad, hi_rad, k7, k8;
    void refresh();
    bool inverted() const { return direction != default_direction; }
    void set_inverted(bool v) {
        direction = v ? -default_direction : default_direction;
        refresh();
    }
    void set_trim(double t, bool is_int) {
        trim_deg = t;
        trim_is_int = is_int;
        refresh();
    }
};

struct Leg {
    const LegDef* def;
    JointServo joint[3];  // coxa, fémur, tibia
    const char* name() const { return def->name; }
};

extern Leg g_legs[NLEGS];
void legs_init();                   // servos aux valeurs de la config (trims à 0)
int leg_index(const char* name);    // -1 si inconnue

// angle servo pour un angle IK (servo.ik_to_servo_deg)
double ik_to_servo_deg(double angle_rad, const JointServo& j);

// --- cinématique inverse (leg_ik.py) -----------------------------------------

// Faux si le pied est hors de portée (UnreachableError) ; le message est
// alors dans unreachable_message().
bool leg_ik(double x, double y, double z, LegAngles* out);
const char* unreachable_message();

double leg_margin(const Leg& leg, const LegAngles& a);
bool leg_ok(const Leg& leg, const LegAngles& a);
bool neighbours_ok(const AngleSet& angles);

// --- réserve d'AngleSet -------------------------------------------------------

class AnglePool {
public:
    enum { SLOTS = 8, MAX_HOLDERS = 8 };
    AnglePool() : nholders_(0) {}
    // Déclare une variable qui garde un AngleSet (case pas réutilisée tant qu'elle y pointe).
    void hold(AngleSet** holder) { holders_[nholders_++] = holder; }
    AngleSet* alloc();

private:
    AngleSet slots_[SLOTS];
    AngleSet** holders_[MAX_HOLDERS];
    int nholders_;
};

// --- démarche tripode (gait.py) ------------------------------------------------

struct TripodGait {
    double period, step_height, max_stride, phase;
    double shift_x, shift_y;
    double stance_time() const { return period / 2; }
    bool at_rest() const { return phase == 0.0 || phase == 0.5; }
    void reset() { phase = 0.0; }
    void advance(double dt, bool moving);
    // Avancement (0..1) du pas, ou -1 si le pied est au sol (None en Python).
    double swing(int leg) const;
    double time_to_lift(int leg) const;
};

// Pour PROF : temps passé dans chaque partie.
enum ProfPart { PROF_I2C, PROF_IK, PROF_PIEDS, PROF_PLACE, PROF_SERVOS, PROF_TELEM, PROF_TICK, PROF_UPDATE, PROF_PARTS };
extern const char* const PROF_NAMES[PROF_PARTS];
struct Profiler {
    bool on;
    unsigned long us[PROF_PARTS];
    bool used[PROF_PARTS];
    void add(int part, unsigned long t0);
};
extern Profiler g_prof;

// --- Hexapod (robot.py) ---------------------------------------------------------

class Hexapod {
public:
    Hexapod(ServoDriver* driver, AnglePool* pool);

    ServoDriver* driver;
    AnglePool* pool;
    TripodGait tripod;
    double neutral[NLEGS][3];
    GaitCommand command, current;
    BodyPose pose, base_pose, applied;
    double pose_scale, lift_scale;
    double offsets[NLEGS][3];
    double ground[NLEGS];  // toujours 0 (pas de gyroscope)
    double travel[3];
    bool replace_feet;
    double home[NLEGS][2], home_goal[NLEGS][2];
    AngleSet* last_angles;  // _last_angles
    AngleSet* written;      // _written
    bool solve_key_valid;   // _solve_key (None si faux)

    bool at_rest() const;
    bool tuck_settled() const { return true; }  // 6 pattes : rien de replié
    void reset_gait();
    // solve() : faux si une patte ne peut pas atteindre sa cible (UnreachableError).
    bool solve(const BodyPose& pose, double lift, AngleSet* out);
    // update() : avance la démarche et envoie les angles. Faux si UnreachableError.
    bool update(double dt, AngleSet** out);
    AngleSet* solve_limited(double dt, bool* unreachable);
    BodyPose effective_pose(double scale) const;

private:
    enum { KEY_LEN = 3 * NLEGS + 16 };
    double lift_[NLEGS][2];
    bool has_lift_[NLEGS];
    bool placed_[NLEGS];
    bool rest_key_valid_;
    double rest_key_phase_;
    double solve_key_[KEY_LEN];
    double radial_[NLEGS][2];
    double reach_[REACH_DIRS];
    double home_target_[NLEGS];
    double home_wait_;
    double home_key_[NLEGS][6];
    bool home_checked_valid_;
    double home_checked_[6];
    bool reach_cache_valid_;
    double reach_cache_key_[12];
    GaitCommand reach_cache_out_;
    double halves_[NLEGS];

    bool solve_checked(const BodyPose& pose, double lift, AngleSet* out);
    bool try_(double scale, double lift, AngleSet* out);
    bool search(double top_s, double top_l, double* s_out, double* l_out, AngleSet* out);
    bool feet_settled() const;
    bool refresh_home();
    double best_radius(int leg, const BodyPose& pose);
    double home_margin(int leg, const BodyPose& pose, double r);
    void reach_table(int leg);
    double half_stride(int leg, double vx, double vy) const;
    double reach_share() const;
    GaitCommand command_in_reach();
    void move_feet(double dt);
    void write_servos(const AngleSet& angles);
};
