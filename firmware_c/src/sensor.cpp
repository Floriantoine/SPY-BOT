#include "sensor.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pyfmt.h"

static double py_sq(double v) { return pow(v, 2.0); }  // v ** 2 de Python

// =============================================================================
// Scanner
// =============================================================================

Scanner::Scanner(Ranger* r, ServoDriver* d)
    : ranger(r),
      driver(d),
      enabled(false),
      stop_dist(STOP_DEFAULT),
      angle(0.0),
      direction(1),
      wait(0.0),
      time(0.0),
      npoints(0),
      blocked(false) {}

void Scanner::status(char* buf, size_t size) const {
    // « stop %d » : int() du float, comme Python.
    snprintf(buf, size, "SENSOR present %d on %d stop %lld", (int)present(), (int)enabled, (long long)stop_dist);
}

void Scanner::enable(bool on) {
    enabled = on && present();
    npoints = 0;
    blocked = false;
    wait = 0.0;
    if (enabled) {
        angle = 0.0;
        point_servo();
    }
}

bool Scanner::tick(double dt, const double pose[3], const GaitCommand& wanted, char* line, size_t size) {
    time += dt;
    if (!enabled) return false;
    wait += dt;
    if (wait < SCAN_PERIOD) return false;
    double x = pose[0], y = pose[1], th = pose[2];
    double heading = th + angle * SENSOR_DEG;
    double cx = x + cos(th) * PAN_AXIS_X + cos(heading) * CHIP_OFFSET;
    double cy = y + sin(th) * PAN_AXIS_X + sin(heading) * CHIP_OFFSET;
    double d;
    bool have = ranger->read(cx, cy, heading, &d);
    if (!have && wait < SCAN_TIMEOUT) return false;  // mesure pas encore prête
    bool out = false;
    if (have) {
        bool seen = MIN_RANGE <= d && d <= MAX_RANGE;
        snprintf(line, size, "S %lld %lld", pyround(angle), seen ? pyround(d) : 0LL);
        out = true;
        if (seen) {
            if (npoints == MAX_POINTS) {  // n'arrive pas en marche normale : on oublie la plus vieille
                memmove(points, points + 1, sizeof(ScanPoint) * (MAX_POINTS - 1));
                npoints--;
            }
            ScanPoint& p = points[npoints++];
            p.x = cx + cos(heading) * d;
            p.y = cy + sin(heading) * d;
            p.t = time;
        }
    }
    int k = 0;
    for (int i = 0; i < npoints; i++)
        if (time - points[i].t < POINT_LIFE) points[k++] = points[i];
    npoints = k;
    wait = 0.0;
    step(wanted);
    point_servo();
    return out;
}

bool Scanner::check(const double pose[3], const GaitCommand& wanted) {
    blocked = enabled && in_path(pose, wanted, stop_dist, FOOT_RADIUS);
    return blocked;
}

bool Scanner::path_blocked(const double pose[3], const GaitCommand& wanted, double distance) {
    return in_path(pose, wanted, distance, FOOT_RADIUS + PATH_MARGIN);
}

bool Scanner::in_path(const double pose[3], const GaitCommand& wanted, double distance, double radius) const {
    // Suit la trajectoire de la consigne (arc si le robot tourne en avançant)
    // sur `distance` mm : bloqué si un point mesuré entre dans le cercle des pieds.
    double speed = sqrt(wanted.vx * wanted.vx + wanted.vy * wanted.vy);
    if (speed < 1.0 || !npoints) return false;
    double x = pose[0], y = pose[1], th = pose[2];
    double c = cos(th), s = sin(th);
    static double near[MAX_POINTS][3];  // pas sur la pile (petite sur l'ESP32)
    int nnear = 0;
    double reach = radius + distance + 10.0;
    for (int i = 0; i < npoints; i++) {
        double dx = points[i].x - x, dy = points[i].y - y;
        double bx = c * dx + s * dy, by = -s * dx + c * dy;  // dans le repère corps
        double d2 = bx * bx + by * by;
        if (d2 < reach * reach) {
            near[nnear][0] = bx;
            near[nnear][1] = by;
            near[nnear][2] = d2;
            nnear++;
        }
    }
    if (!nnear) return false;
    double w = wanted.yaw_rate;
    double horizon = pymin(3.0, distance / speed);
    long long nn = (long long)(distance / 15.0);
    long long n = nn > 2 ? nn : 2;
    double r2 = radius * radius;
    for (long long i = 1; i <= n; i++) {
        double t = horizon * (double)i / (double)n;
        double cx, cy;
        if (fabs(w) < 1e-4) {
            cx = wanted.vx * t;
            cy = wanted.vy * t;
        } else {
            double a = w * t;
            cx = (wanted.vx * sin(a) + wanted.vy * (cos(a) - 1.0)) / w;
            cy = (wanted.vx * (1.0 - cos(a)) + wanted.vy * sin(a)) / w;
        }
        for (int k = 0; k < nnear; k++) {
            double e2 = py_sq(near[k][0] - cx) + py_sq(near[k][1] - cy);
            if (e2 < r2 && e2 < near[k][2] - 25.0) return true;
        }
    }
    return false;
}

void Scanner::step(const GaitCommand& wanted) {
    double lo = -PAN_LIMIT, hi = PAN_LIMIT;
    if (wanted.vx > 0) {
        // En marche vers l'avant : balayage resserré autour du cap suivi.
        double centre = atan2(wanted.vy, wanted.vx) / SENSOR_DEG;
        centre = pymax(-PAN_LIMIT + WALK_HALF, pymin(PAN_LIMIT - WALK_HALF, centre));
        lo = centre - WALK_HALF;
        hi = centre + WALK_HALF;
    }
    double a = angle + direction * SCAN_STEP;
    if (a > hi) {
        direction = -1;
        a = pymax(lo, pymin(angle - SCAN_STEP, hi));
    } else if (a < lo) {
        direction = 1;
        a = pymin(hi, pymax(angle + SCAN_STEP, lo));
    }
    angle = a;
}

void Scanner::point_servo() {
    if (driver != NULL) driver->set_angle(PAN_BOARD, PAN_CHANNEL, 90.0 + PAN_DIRECTION * angle);
}

// =============================================================================
// Pilote VL53L1X (même séquence de registres que sensor.VL53L1X)
// =============================================================================

bool Vl53l1x::write(int reg, const uint8_t* data, size_t n) {
    uint8_t buf[2 + VL53_CONFIG_LEN];
    if (n > VL53_CONFIG_LEN) return false;
    buf[0] = (uint8_t)(reg >> 8);
    buf[1] = (uint8_t)(reg & 0xFF);
    memcpy(buf + 2, data, n);
    return bus_->writeto(VL53_ADDRESS, buf, n + 2);
}

bool Vl53l1x::write8(int reg, uint8_t value) { return write(reg, &value, 1); }

bool Vl53l1x::read_reg(int reg, uint8_t* out, size_t n) {
    uint8_t idx[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
    if (!bus_->writeto(VL53_ADDRESS, idx, 2)) return false;
    return bus_->readfrom(VL53_ADDRESS, out, n);
}

bool Vl53l1x::read8(int reg, uint8_t* v) { return read_reg(reg, v, 1); }

bool Vl53l1x::read16(int reg, int* v) {
    uint8_t b[2];
    if (!read_reg(reg, b, 2)) return false;
    *v = (b[0] << 8) | b[1];
    return true;
}

bool Vl53l1x::ready(bool* r) {
    uint8_t a, b;
    if (!read8(0x0030, &a)) return false;
    int polarity = (a & 0x10) ? 0 : 1;
    if (!read8(0x0031, &b)) return false;
    *r = (b & 0x01) == polarity;
    return true;
}

bool Vl53l1x::probe() {
    if (!bus_->scan_has(VL53_ADDRESS)) return false;
    int id;
    if (!read16(0x010F, &id) || id != VL53_MODEL_ID) return false;  // pas un VL53L1X
    for (int i = 0; i < 100; i++) {  // attente de la fin du démarrage interne
        uint8_t v;
        if (!read8(0x00E5, &v)) return false;
        if (v & 0x01) break;
        platform_delay_ms(2);
    }
    if (!write(0x002D, VL53_DEFAULT_CONFIG, VL53_CONFIG_LEN)) return false;
    // Première mesure pour finir l'initialisation (séquence ULD) : elle dure
    // plus de 100 ms sur la vraie puce, on lui laisse au moins 0,5 s.
    for (int i = 0; i < VL53_FIRST_TRIES; i++) {
        bool r;
        if (!ready(&r)) return false;
        if (r) break;
        platform_delay_ms(2);
    }
    if (!write8(0x0086, 0x01)) return false;  // efface l'interruption
    if (!write8(0x0087, 0x00)) return false;  // arrêt
    if (!write8(0x0008, 0x09)) return false;  // VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND
    if (!write8(0x000B, 0x00)) return false;
    // Champ de vue réduit de 27° à ~15° (ROI de 4×4 SPAD au centre).
    if (!write8(0x0080, 0x33)) return false;
    return write8(0x0087, 0x40);  // mesure continue
}

bool Vl53l1x::read(double, double, double, double* d) {
    // Distance en mm si une mesure est prête ; invalide (rien en vue) : MAX_RANGE + 1.
    bool r;
    if (!ready(&r)) return false;
    if (!r) return false;
    uint8_t status;
    int dist;
    if (!read8(0x0089, &status)) return false;
    if (!read16(0x0096, &dist)) return false;
    if (!write8(0x0086, 0x01)) return false;  // efface l'interruption : mesure suivante
    *d = (status & 0x1F) == 9 ? (double)dist : MAX_RANGE + 1;
    return true;
}
