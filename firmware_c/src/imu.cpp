#include "imu.h"

#include <math.h>

#include "pyfmt.h"

static double clampv(double v, double limit) { return pymax(-limit, pymin(limit, v)); }

// Roulis et tangage d'après la pesanteur mesurée (imu._accel_angles).
static void accel_angles(double ax, double ay, double az, double* roll, double* pitch) {
    *roll = atan2(ay, az);
    *pitch = atan2(-ax, sqrt(ay * ay + az * az));
}

Attitude::Attitude() : roll(0.0), pitch(0.0), ready(false), bias_n_(0), bias_t_(0.0), lost_(false) {
    rate[0] = rate[1] = 0.0;
    bias_[0] = bias_[1] = bias_[2] = 0.0;
    lo_[0] = lo_[1] = lo_[2] = 0.0;
    hi_[0] = hi_[1] = hi_[2] = 0.0;
}

void Attitude::lose() {
    if (ready) {
        ready = false;
        lost_ = true;
    }
}

void Attitude::restart() {
    ready = false;
    lost_ = false;
    rate[0] = rate[1] = 0.0;
    bias_[0] = bias_[1] = bias_[2] = 0.0;
    bias_n_ = 0;
    bias_t_ = 0.0;
}

void Attitude::update(double dt, const double accel[3], const double gyro[3]) {
    double ax = accel[0], ay = accel[1], az = accel[2];
    if (lost_) {
        // Le gyroscope répond de nouveau : on repart de l'inclinaison de l'accéléromètre.
        accel_angles(ax, ay, az, &roll, &pitch);
        rate[0] = rate[1] = 0.0;
        lost_ = false;
        ready = true;
        return;
    }
    if (!ready) {
        // Robot immobile : on moyenne le gyroscope, l'accéléromètre donne l'inclinaison.
        bool moved = false;
        for (int i = 0; i < 3; i++) {
            double g = gyro[i];
            if (bias_n_ == 0 || g < lo_[i]) lo_[i] = g;
            if (bias_n_ == 0 || g > hi_[i]) hi_[i] = g;
            if (hi_[i] - lo_[i] > BIAS_SPREAD) moved = true;
        }
        if (moved) {
            restart();  // on a bougé le robot : la mesure recommence
            return;
        }
        for (int i = 0; i < 3; i++) bias_[i] = bias_[i] + gyro[i];
        bias_n_ += 1;
        bias_t_ += dt;
        if (bias_t_ >= BIAS_TIME) {
            for (int i = 0; i < 3; i++) bias_[i] = bias_[i] / bias_n_;
            accel_angles(ax, ay, az, &roll, &pitch);
            ready = true;
        }
        return;
    }
    double gx = gyro[0] - bias_[0], gy = gyro[1] - bias_[1], gz = gyro[2] - bias_[2];
    // Vitesses du corps -> dérivées du roulis et du tangage.
    double sr = sin(roll), cr = cos(roll);
    double cp = cos(pitch);
    double tp = fabs(cp) > 1e-3 ? sin(pitch) / cp : 0.0;
    double droll = gx + (gy * sr + gz * cr) * tp;
    double dpitch = gy * cr - gz * sr;
    rate[0] = droll;
    rate[1] = dpitch;
    double r = roll + droll * dt;
    double p = pitch + dpitch * dt;
    double g = sqrt(ax * ax + ay * ay + az * az);
    if (ACCEL_OK_LO < g && g < ACCEL_OK_HI) {
        double ar, ap;
        accel_angles(ax, ay, az, &ar, &ap);
        double trust = pymin(1.0, ACCEL_TRUST * dt / 0.02);  // même constante de temps si un tick déborde
        r += (ar - r) * trust;
        p += (ap - p) * trust;
    }
    roll = r;
    pitch = p;
}

void Leveler::relax(double dt) {
    // Hors de la marche : la correction revient doucement à zéro.
    double step = LEVEL_LIMIT * dt;
    roll = roll > 0 ? pymax(0.0, roll - step) : pymin(0.0, roll + step);
    pitch = pitch > 0 ? pymax(0.0, pitch - step) : pymin(0.0, pitch + step);
}

void Leveler::update(double dt, double mr, double mp, double wr, double wp) {
    if (!enabled) {
        relax(dt);  // LEVEL 0 : la correction s'en va doucement, sans à-coup
        return;
    }
    double k = pymin(1.0, LEVEL_GAIN * dt);
    roll = clampv(roll - (mr - wr) * k, LEVEL_LIMIT);
    pitch = clampv(pitch - (mp - wp) * k, LEVEL_LIMIT);
}

// --- pilote MPU6050 -------------------------------------------------------------

bool Mpu6050::probe() {
    if (!bus_->scan_has(IMU_ADDRESS)) return false;
    // Même séquence que MPU6050.__init__ (une OSError arrête tout).
    static const uint8_t init[4][2] = {
        {0x6B, 0x01},  // réveil, horloge du gyroscope X
        {0x1A, 0x03},  // filtre passe-bas ~44 Hz (vibrations des servos)
        {0x1B, 0x08},  // gyroscope ±500 °/s
        {0x1C, 0x08},  // accéléromètre ±4 g
    };
    for (int i = 0; i < 4; i++)
        if (!bus_->writeto_mem(IMU_ADDRESS, init[i][0], &init[i][1], 1)) return false;
    return true;
}

static int s16(uint8_t hi, uint8_t lo) {
    int v = (hi << 8) | lo;
    return v & 0x8000 ? v - 65536 : v;
}

bool Mpu6050::read(double accel[3], double gyro[3]) {
    uint8_t b[14];
    if (!bus_->readfrom_mem(IMU_ADDRESS, 0x3B, b, 14)) {
        errors++;
        return false;
    }
    static const int at[6] = {0, 2, 4, 8, 10, 12};
    double v[6];
    for (int i = 0; i < 3; i++) v[i] = s16(b[at[i]], b[at[i] + 1]) / IMU_ACCEL_SCALE;
    for (int i = 3; i < 6; i++) v[i] = s16(b[at[i]], b[at[i] + 1]) / IMU_GYRO_SCALE;
    // _to_body : axes de la carte vus du corps.
    for (int k = 0; k < 3; k++) {
        accel[k] = v[IMU_AXES[k][0]] * IMU_AXES[k][1];
        gyro[k] = v[3 + IMU_AXES[k][0]] * IMU_AXES[k][1];
    }
    return true;
}

void Mpu6050::update(double dt) {
    double a[3], g[3];
    if (read(a, g)) {
        fails_ = 0;
        attitude.update(dt, a, g);
    } else {
        fails_ += 1;
        if (fails_ == IMU_FAIL_MAX) attitude.lose();
    }
}
