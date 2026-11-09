// Centrale inertielle MPU6050 : portage de hexapod/imu.py (Attitude,
// Leveler, pilote MPU6050). Les calculs suivent le Python opération par opération.
#pragma once
#include "i2c_bus.h"
#include "robot_config.h"

// Roulis et tangage estimés à partir des mesures brutes (g et rad/s).
struct Attitude {
    double roll, pitch;
    bool ready;      // biais mesuré, angles initialisés
    double rate[2];  // vitesses de roulis et de tangage (rad/s), biais retiré
    Attitude();
    void update(double dt, const double accel[3], const double gyro[3]);
    // Refait la mesure du biais (IMU CAL) : pas prêt pendant BIAS_TIME, robot immobile.
    void restart();
    // Le gyroscope ne répond plus : assiette inconnue jusqu'à la prochaine mesure (biais gardé).
    void lose();

private:
    double bias_[3];
    double lo_[3], hi_[3];  // mesures extrêmes du gyroscope pendant la mesure du biais
    long bias_n_;
    double bias_t_;
    bool lost_;  // biais mesuré, mais le gyroscope ne répondait plus
};

// Auto-équilibrage : correction (rad) ajoutée à la pose du corps.
struct Leveler {
    bool enabled;
    double roll, pitch;
    Leveler() : enabled(false), roll(0.0), pitch(0.0) {}
    void reset() { roll = pitch = 0.0; }
    void relax(double dt);
    void update(double dt, double measured_roll, double measured_pitch, double wanted_roll, double wanted_pitch);
};

// Gyroscope vu par le Controller (MPU6050 sur l'ESP32, ou faux en test).
class Imu {
public:
    Attitude attitude;
    virtual ~Imu() {}
    virtual void update(double dt) = 0;
};

class Mpu6050 : public Imu {
public:
    explicit Mpu6050(I2CBus* bus) : errors(0), bus_(bus), fails_(0) {}
    // MPU6050.probe : vrai s'il répond à 0x68 et accepte sa configuration.
    bool probe();
    // Accélérations (g) et vitesses (rad/s) dans le repère corps ; faux si la lecture échoue.
    bool read(double accel[3], double gyro[3]);
    void update(double dt);
    unsigned long errors;

private:
    I2CBus* bus_;
    int fails_;  // lectures ratées de suite
};
