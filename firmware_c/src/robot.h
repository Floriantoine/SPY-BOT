// Cinématique inverse, démarche tripode et suivi des pieds : portage de
// hexapod/leg_ik.py, body_ik.py, gait.py et robot.py (6 et 4 pattes, sol deviné).
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

// --- démarches (gait.py) ------------------------------------------------------

class Hexapod;

// TripodGait, WaveGait ou QuadGait (Hexapod.gait pointe sur l'une des trois).
struct Gait {
    double period, step_height, max_stride, phase;
    double shift_x, shift_y;  // décalage du corps (balancement à 4 pattes)
    virtual ~Gait() {}
    virtual double stance_time() const = 0;
    virtual bool at_rest() const = 0;
    virtual void reset() = 0;
    virtual void advance(double dt, bool moving) = 0;
    // Avancement (0..1) du pas, ou -1 si le pied est au sol (None en Python).
    virtual double swing(int leg) const = 0;
    virtual double time_to_lift(int leg) const = 0;
};

struct TripodGait : public Gait {
    double stance_time() const { return period / 2; }
    bool at_rest() const { return phase == 0.0 || phase == 0.5; }
    void reset() { phase = 0.0; }
    void advance(double dt, bool moving);
    double swing(int leg) const;
    double time_to_lift(int leg) const;
};

// Marche « vague » : une patte en l'air à la fois, 5 pieds au sol.
struct WaveGait : public Gait {
    double swing_part() const { return 1.0 / 6; }
    double stance_time() const { return (1.0 - swing_part()) * period; }
    bool at_rest() const;
    void reset() { phase = 0.0; }
    void advance(double dt, bool moving);
    double swing(int leg) const;
    double time_to_lift(int leg) const;
};

struct QuadGait : public Gait {
    double sway;
    Hexapod* planner;  // plan(slot) = Hexapod::plan_sway
    bool has_target;
    double target_x, target_y;
    int slot;
    double from_x, from_y;
    double swing_part() const;
    double stance_time() const { return (1.0 - swing_part()) * period; }
    bool at_rest() const;
    void reset();
    void advance(double dt, bool moving);
    double swing(int leg) const;
    double time_to_lift(int leg) const;
};

// Pour PROF : temps passé dans chaque partie.
enum ProfPart { PROF_CAPTEUR, PROF_GYRO, PROF_I2C, PROF_IK, PROF_PIEDS, PROF_PLACE, PROF_SERVOS, PROF_TELEM, PROF_TICK, PROF_UPDATE, PROF_PARTS };
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
    QuadGait gait4;
    WaveGait wave;
    int wave_mode;      // WAVE_OFF, WAVE_ON ou WAVE_AUTO
    double wave_speed;  // seuil de WAVE AUTO (mm/s)
    Gait* six;          // démarche à 6 pattes en cours (tripode ou vague)
    double wave_shift;  // décalage du corps en vague (mm), 0 = aucun
    Gait* gait;
    int legs;     // pattes qui marchent : 6, ou 4 (pattes du milieu repliées)
    double tuck;  // 0 = pattes du milieu au sol, 1 = repliées
    double neutral[NLEGS][3];
    GaitCommand command, current;
    BodyPose pose, base_pose, applied;
    double pose_scale, lift_scale;
    double offsets[NLEGS][3];
    double ground[NLEGS];  // hauteur du sol devinée au gyroscope (feel)
    double feel_rate[2];
    bool feel_ground;
    double time_scale;
    double travel[3];
    bool replace_feet;
    double home[NLEGS][2], home_goal[NLEGS][2];
    AngleSet* last_angles;  // _last_angles
    AngleSet* written;      // _written
    bool solve_key_valid;   // _solve_key (None si faux)

    bool at_rest() const;
    bool tuck_settled() const { return tuck == (legs == 4 ? 1.0 : 0.0); }
    bool walking(int leg) const { return !(legs == 4 && CFG_LEGS[leg].tucked); }
    // set_legs : 0 si fait, 1 si n n'est pas 4 ou 6, 2 si le robot n'est pas arrêté pieds posés.
    int set_legs(int n);
    void reset_gait();
    void feel(double dt, double roll_rate, double pitch_rate);
    // solve() : faux si une patte ne peut pas atteindre sa cible (UnreachableError).
    bool solve(const BodyPose& pose, double lift, AngleSet* out);
    // update() : avance la démarche et envoie les angles. Faux si UnreachableError.
    bool update(double dt, AngleSet** out);
    AngleSet* solve_limited(double dt, bool* unreachable);
    BodyPose effective_pose(double scale) const;
    void plan_sway(int slot, double* x, double* y);

private:
    enum { KEY_LEN = 4 * NLEGS + 20 };
    double lift_[NLEGS][2];
    bool has_lift_[NLEGS];
    double ground_lift_[NLEGS];
    bool has_ground_lift_[NLEGS];
    bool touched_[NLEGS];
    bool placed_[NLEGS];
    bool rest_key_valid_;
    const Gait* rest_key_gait_;
    int rest_key_legs_;
    double rest_key_phase_;
    double solve_key_[KEY_LEN];
    double radial_[NLEGS][2];
    double reach_[REACH_DIRS];
    double home_target_[NLEGS];
    double home_wait_;
    double home_key_[NLEGS][6];
    bool home_checked_valid_;
    int home_checked_legs_;
    double home_checked_[6];
    bool reach_cache_valid_;
    const Gait* reach_cache_gait_;
    double reach_cache_key_[12];
    GaitCommand reach_cache_out_;
    double halves_[NLEGS];
    bool wave_key_valid_;
    double wave_key_[4];  // dernière consigne vue par wanted_gait() et sa vitesse

    double shift_amp_, shift_mm_, shift_dir_[2];

    Gait* wanted_gait();
    void wave_sway(double dt, bool moving);
    double foot_speed(const GaitCommand& c) const;
    void switch_gait(Gait* next);
    void forget_ground();
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
