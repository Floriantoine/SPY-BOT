// Machine d'états du robot et protocole texte avec le PC : portage de
// hexapod/controller.py, sans gyroscope, capteur avant ni navigation.
#pragma once
#include <stddef.h>

#include "robot.h"

// Où garder la calibration (NVS sur l'ESP32, mémoire en test).
class CalibStore {
public:
    virtual ~CalibStore() {}
    // trims[3 × patte + articulation] en degrés, inv[...] = servo inversé,
    // is_int[...] = trim entier (écrit sans « .0 », comme Python).
    virtual bool load(double trims[NJOINTS], bool inv[NJOINTS], bool is_int[NJOINTS]) = 0;
    virtual bool save(const double trims[NJOINTS], const bool inv[NJOINTS], const bool is_int[NJOINTS]) = 0;
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

    Controller(ServoDriver* driver, CalibStore* store);

    // À appeler à ~50 Hz. Retourne les lignes à envoyer (séparées par \n), ou NULL.
    const char* tick(double dt);
    // Exécute une ligne de commande. Retourne la réponse, ou NULL.
    const char* handle(const char* line);

    double telemetry_period;
    Mode mode;
    AngleSet* angles;
    Hexapod robot;

    const char* sensor_status();  // ligne SENSOR (capteur absent)
    const char* imu_status();     // ligne IMU (gyroscope absent)

private:
    ServoDriver* driver_;
    CalibStore* store_;
    GaitCommand wanted_;
    double odom_[3];
    double sit_drop_, height_;
    BodyPose user_pose_;
    bool pose_limited_;
    double free_time_, went_;
    unsigned long tick_us_;
    AngleSet* margin_of_;
    double margin_;
    double since_move_, since_telemetry_;
    bool wake_known_;
    AngleSet wake_from_;
    AngleSet* wake_to_;
    double wake_time_;
    bool stand_after_wake_;
    double test_time_;
    int test_leg_, test_joint_;
    double stop_dist_;  // SENSOR STOP (capteur absent : seulement retenu)
    double prof_time_;
    long prof_cycles_;

    char out_[OUT_SIZE];
    size_t out_len_;
    char reply_[OUT_SIZE];
    char status_[64];

    void out_line(const char* fmt, ...);
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
    bool parse_joint(const char* name, const char* leg, const char* joint, JointServo** out, const char** error);
};
