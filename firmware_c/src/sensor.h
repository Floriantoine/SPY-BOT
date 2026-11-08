// Capteur de distance avant (VL53L1X sur un servo de balayage) : portage de
// hexapod/sensor.py (Scanner et pilote VL53L1X).
#pragma once
#include <stddef.h>

#include "driver.h"
#include "i2c_bus.h"
#include "robot.h"

// Source des mesures (VL53L1X sur l'ESP32, faux capteur en test).
class Ranger {
public:
    virtual ~Ranger() {}
    // Distance en mm ; faux si aucune mesure n'est prête (None en Python).
    // (x, y, cap) : position du capteur sur le sol, ignorée par le vrai capteur.
    virtual bool read(double x, double y, double heading, double* d) = 0;
};

class Vl53l1x : public Ranger {
public:
    explicit Vl53l1x(I2CBus* bus) : bus_(bus) {}
    // VL53L1X.probe : vrai s'il répond à 0x29 et que l'initialisation passe.
    bool probe();
    bool read(double x, double y, double heading, double* d);

private:
    I2CBus* bus_;
    bool write(int reg, const uint8_t* data, size_t n);
    bool write8(int reg, uint8_t value);
    bool read_reg(int reg, uint8_t* out, size_t n);
    bool read8(int reg, uint8_t* v);
    bool read16(int reg, int* v);
    bool ready(bool* r);
};

struct ScanPoint {
    double x, y, t;  // repère du sol, instant de la mesure
};

class Scanner {
public:
    // Mesures gardées POINT_LIFE s, une toutes les SCAN_PERIOD s au plus.
    enum { MAX_POINTS = SCAN_MAX_POINTS };

    Scanner(Ranger* ranger, ServoDriver* driver);
    Ranger* ranger;
    ServoDriver* driver;
    bool enabled;
    double stop_dist;
    double angle;  // deg, + = vers la gauche
    int direction;
    double wait, time;
    ScanPoint points[MAX_POINTS];
    int npoints;
    bool blocked;

    bool present() const { return ranger != NULL; }
    void status(char* buf, size_t size) const;
    void enable(bool on);
    // Avance le balayage ; écrit la ligne « S angle distance » s'il y a une mesure.
    bool tick(double dt, const double pose[3], const GaitCommand& wanted, char* line, size_t size);
    // Vrai si un obstacle est dans le couloir de la marche demandée.
    bool check(const double pose[3], const GaitCommand& wanted);
    bool path_blocked(const double pose[3], const GaitCommand& wanted, double distance);

private:
    bool in_path(const double pose[3], const GaitCommand& wanted, double distance, double radius) const;
    void step(const GaitCommand& wanted);
    void point_servo();
};
