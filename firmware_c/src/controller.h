// Machine d'états du robot et protocole texte avec le PC : portage de
// hexapod/controller.py (vrai robot : pas les commandes de la simulation).
#pragma once
#include <stddef.h>

#include "imu.h"
#include "nav.h"
#include "robot.h"
#include "sensor.h"

// Où garder la calibration (NVS sur l'ESP32, mémoire en test).
class CalibStore {
public:
    virtual ~CalibStore() {}
    // trims[3 × patte + articulation] en degrés, inv[...] = servo inversé,
    // is_int[...] = trim entier (écrit sans « .0 », comme Python).
    virtual bool load(double trims[NJOINTS], bool inv[NJOINTS], bool is_int[NJOINTS]) = 0;
    virtual bool save(const double trims[NJOINTS], const bool inv[NJOINTS], const bool is_int[NJOINTS]) = 0;
    // Réglages du gyroscope (IMU AXES, IMU ZERO), gardés à part des trims.
    // load_imu : faux s'il n'y en a pas (ancienne calibration).
    virtual bool load_imu(int axes[3][2], double zero[2]) = 0;
    virtual bool save_imu(const int axes[3][2], const double zero[2]) = 0;
    // Course réelle des servos (TRAVEL), µs par degré, gardée à part elle aussi.
    // load_travel : faux s'il n'y en a pas (ancienne calibration).
    virtual bool load_travel(double us[NJOINTS]) {
        (void)us;
        return false;
    }
    virtual bool save_travel(const double us[NJOINTS]) {
        (void)us;
        return true;
    }
};

enum Mode { MODE_OFF, MODE_WAKE, MODE_SIT, MODE_STANDING, MODE_STAND, MODE_SITTING, MODE_CAL, MODE_TEST };

class Controller {
private:
    // Construits en premier : les servos (trims à 0) puis la réserve d'angles,
    // avant le robot (même ordre que Python : calibration chargée après).
    struct LegsInit {
        LegsInit() { legs_init(); }
    } legs_init_;
    AnglePool pool_;

public:
    enum { OUT_SIZE = 4096 };

    // ranger, imu : capteur avant et gyroscope, NULL s'ils sont absents.
    Controller(ServoDriver* driver, CalibStore* store, Ranger* ranger = NULL, Imu* imu = NULL);

    // À appeler à ~50 Hz. Retourne les lignes à envoyer (séparées par \n), ou NULL.
    const char* tick(double dt);
    // Exécute une ligne de commande. Retourne la réponse, ou NULL.
    const char* handle(const char* line);

    double telemetry_period;
    Mode mode;
    AngleSet* angles;
    SmoothDriver smooth;  // sauts de consigne adoucis (SLEW), avant le robot
    Hexapod robot;
    Scanner scanner;
    Navigator nav;
    Imu* imu;
    Leveler leveler;
    bool tilted;  // corps trop penché : arrêt de sécurité

    const char* sensor_status();  // ligne SENSOR (Scanner.status)
    const char* imu_status();     // ligne IMU (cmd_imu)
    const char* walk_status();    // ligne WALK (réglages de la marche)

private:
    ServoDriver* driver_;
    CalibStore* store_;
    GaitCommand wanted_;
    double odom_[3];
    double sit_drop_, height_;
    BodyPose user_pose_;  // pose suivie : rejoint pose_goal_ en douceur
    BodyPose pose_goal_;  // pose demandée (POSE)
    bool pose_moving_;
    double pose_vel_[6];
    void slew_pose(double dt);
    bool pose_limited_;
    double free_time_, went_;
    unsigned long tick_us_;
    AngleSet* margin_of_;
    double margin_;
    double since_move_, since_line_, since_telemetry_;
    bool wake_known_;
    AngleSet wake_from_;
    AngleSet* wake_to_;
    double wake_time_;
    bool stand_after_wake_;
    double test_time_;
    int test_leg_, test_joint_;
    double upright_time_;
    const Gait* walk_gait_;  // démarche à 6 pattes de la dernière ligne WALK
    double prof_time_;
    long prof_cycles_;

    char out_[OUT_SIZE];
    size_t out_len_;
    char pending_[1024];  // lignes à envoyer avec la prochaine télémétrie
    size_t pending_len_;
    char reply_[OUT_SIZE];
    char status_[128];

    void out_line(const char* fmt, ...);
    void pend_line(const char* text);
    GaitCommand filter(const GaitCommand& wanted, bool to_pending);
    void auto_sensor();
    void govern(double dt);
    void tilt_guard(double dt);
    void telemetry();
    void measures(double period);
    bool prof_line(double dt);

    void calibration_json(char* buf, size_t size);
    void load_calibration();
    void apply_pose();
    void send_angles();
    void send_leg(int leg, const LegAngles& a);
    AngleSet* calibration_angles();
    bool start_wake();
    void wake_tick(double dt);
    void integrate(const double travel[3]);
    int stance_mask() const;

    // Commandes : retournent la réponse (dans reply_) ou NULL.
    const char* dispatch(const char* name, char** args, int nargs);
    const char* cmd_power(const char* on);
    const char* err(const char* name, const char* fmt, ...);
    const char* err_float(const char* name, const char* arg);
    const char* nav_status_reply(const char* name);
    bool save_imu();
    bool parse_joint(const char* name, const char* leg, const char* joint, JointServo** out, const char** error);
};
