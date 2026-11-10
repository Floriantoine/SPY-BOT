#include "nav.h"

#include <math.h>
#include <stdio.h>

#include "pyfmt.h"

static double py_sq(double v) { return pow(v, 2.0); }  // v ** 2 de Python

Navigator::Navigator()
    : mode(NAV_OFF),
      goal_x(0.0),
      goal_y(0.0),
      speed(NAV_SPEED),
      yaw_rate(NAV_YAW_RATE),
      last_(0.0),
      has_target_(false),
      target_x_(0.0),
      target_y_(0.0),
      seen_(0.0),
      has_best_(false),
      best_(0.0),
      since_best_(0.0),
      has_plan_(false),
      plan_(gait_command()),
      plan_age_(0.0) {}

void Navigator::go_to(double x, double y, bool has_speed, double s) {
    mode = NAV_GOTO;
    goal_x = x;
    goal_y = y;
    if (has_speed && s != 0.0) speed = s;  // « if speed: » (0 garde l'ancienne)
    last_ = 0.0;
    has_best_ = false;
    since_best_ = 0.0;
    has_plan_ = false;
    plan_age_ = 0.0;
}

void Navigator::follow(bool has_speed, double s) {
    mode = NAV_FOLLOW;
    has_target_ = false;
    if (has_speed && s != 0.0) speed = s;
}

// round(v) écrit avec %d ; faux pour inf et nan (exceptions de Python).
static bool round_text(double v, char* out, size_t size, char* err, size_t esize) {
    if (isnan(v)) {
        snprintf(err, esize, "cannot convert float NaN to integer");
        return false;
    }
    if (isinf(v)) {
        snprintf(err, esize, "cannot convert float infinity to integer");
        return false;
    }
    double r = rint(v) + 0.0;  // au pair le plus proche ; -0 -> 0
    if (fabs(r) < 9e18)
        snprintf(out, size, "%lld", (long long)r);
    else
        snprintf(out, size, "%.0f", r);  // entier exact (grand double)
    return true;
}

bool Navigator::status(char* buf, size_t size) const {
    if (mode == NAV_GOTO) {
        char x[400], y[400];
        if (!round_text(goal_x, x, sizeof(x), buf, size) || !round_text(goal_y, y, sizeof(y), buf, size))
            return false;
        snprintf(buf, size, "NAV goto %s %s", x, y);
        return true;
    }
    snprintf(buf, size, "NAV %s", mode == NAV_FOLLOW ? "follow" : "off");
    return true;
}

GaitCommand Navigator::command(const double pose[3], Scanner* scanner, double dt, NavEvent* event) {
    *event = NAV_NONE;
    if (mode == NAV_GOTO) return go(pose, scanner, dt, event);
    if (mode == NAV_FOLLOW) return follow_cmd(pose, scanner, dt);
    return gait_command();
}

GaitCommand Navigator::go(const double pose[3], Scanner* scanner, double dt, NavEvent* event) {
    double x = pose[0], y = pose[1], th = pose[2];
    double dx = goal_x - x, dy = goal_y - y;
    double dist = sqrt(dx * dx + dy * dy);
    if (dist < NAV_ARRIVE) {
        stop();
        *event = NAV_ARRIVED;
        return gait_command();
    }
    // But collé à un obstacle, ou aucun passage : on ne tourne pas en rond indéfiniment.
    if (!has_best_ || dist < best_ - NAV_STUCK_GAIN) {
        has_best_ = true;
        best_ = dist;
        since_best_ = 0.0;
    } else {
        since_best_ += dt;
        if (since_best_ > NAV_STUCK_TIME) {
            stop();
            *event = NAV_STUCK;
            return gait_command();
        }
    }
    double c = cos(th), s = sin(th);
    double bearing = atan2(-s * dx + c * dy, c * dx + s * dy);  // but vu du corps
    double sp = pymax(NAV_MIN_SPEED, pymin(speed, dist));
    double look = pymin(NAV_LOOKAHEAD, dist + 50.0);
    double b = bearing / NAV_DEG;
    plan_age_ += dt;
    if (has_plan_ && plan_age_ < NAV_REPLAN) return plan_;
    plan_age_ = 0.0;
    plan_ = choose(scanner, pose, b, bearing, sp, look);
    has_plan_ = true;
    return plan_;
}

static bool blocked(Scanner* scanner, const double pose[3], double a, double speed, double look) {
    if (scanner == NULL || !scanner->enabled) return false;
    return scanner->path_blocked(pose, gait_command(speed * cos(a), speed * sin(a), 0.0), look);
}

GaitCommand Navigator::choose(Scanner* scanner, const double pose[3], double b, double bearing, double sp,
                              double look) {
    enum { N = 2 * NAV_SIDE / NAV_STEP + 1 };
    double opts[N + 1], keys[N];
    int n = 0;
    for (int a = -NAV_SIDE; a <= NAV_SIDE; a += NAV_STEP) {
        opts[n] = a;
        keys[n] = fabs(a - b) + 0.3 * fabs(a - last_);
        n++;
    }
    // Tri stable (comme list.sort) : la plus proche du but d'abord ; à écart
    // égal, celle qu'on suivait déjà.
    for (int i = 1; i < n; i++) {
        double o = opts[i], k = keys[i];
        int j = i;
        while (j > 0 && k < keys[j - 1]) {
            opts[j] = opts[j - 1];
            keys[j] = keys[j - 1];
            j--;
        }
        opts[j] = o;
        keys[j] = k;
    }
    // Décalées d'une case : la direction exacte du but passe en tête si elle est permise.
    for (int i = N; i > 0; i--) opts[i] = opts[i - 1];
    int start = 1;
    if (fabs(b) <= NAV_SIDE) {
        opts[0] = b;
        start = 0;
    }
    for (int i = start; i <= N; i++) {
        double a = opts[i];
        if (!blocked(scanner, pose, a * NAV_DEG, sp, look)) {
            last_ = a;
            return walk(a * NAV_DEG, sp);
        }
    }
    // Rien de libre devant : tourner sur place, du côté du but, pour chercher un passage.
    double side = fabs(bearing) > 1e-3 ? bearing : last_;
    double turn = side >= 0 ? yaw_rate : -yaw_rate;
    return gait_command(0.0, 0.0, turn);
}

GaitCommand Navigator::walk(double a, double sp) const {
    double yaw = pymax(-yaw_rate, pymin(yaw_rate, NAV_YAW_GAIN * a));
    if (fabs(a) > NAV_SEE) return gait_command(0.0, 0.0, yaw);  // le capteur ne voit pas par là : tourner d'abord
    double k = cos(a);  // plus lent tant que le corps n'est pas tourné vers la direction
    return gait_command(sp * k * cos(a), sp * k * sin(a), yaw);
}

// (distance au centre, direction vue du corps, x, y) du point récent le plus
// proche devant ; s'il y a une cible, seulement autour d'elle.
static bool nearest_ahead(Scanner* scanner, const double pose[3], bool has_near, double nx, double ny, double out[4]) {
    if (scanner == NULL || !scanner->enabled) return false;
    double x = pose[0], y = pose[1], th = pose[2];
    double c = cos(th), s = sin(th);
    bool found = false;
    for (int i = 0; i < scanner->npoints; i++) {
        const ScanPoint& p = scanner->points[i];
        if (scanner->time - p.t > NAV_FOLLOW_FRESH) continue;
        if (has_near && py_sq(p.x - nx) + py_sq(p.y - ny) > NAV_FOLLOW_TRACK * NAV_FOLLOW_TRACK) continue;
        double dx = p.x - x, dy = p.y - y;
        double bx = c * dx + s * dy, by = -s * dx + c * dy;
        double d = sqrt(bx * bx + by * by);
        double a = atan2(by, bx);
        if (d < NAV_FOLLOW_RANGE && fabs(a) < NAV_FOLLOW_SIDE && (!found || d < out[0])) {
            out[0] = d;
            out[1] = a;
            out[2] = p.x;
            out[3] = p.y;
            found = true;
        }
    }
    return found;
}

GaitCommand Navigator::follow_cmd(const double pose[3], Scanner* scanner, double dt) {
    seen_ += dt;
    if (seen_ > NAV_FOLLOW_LOST) has_target_ = false;
    double f[4] = {0.0, 0.0, 0.0, 0.0};
    if (!nearest_ahead(scanner, pose, has_target_, target_x_, target_y_, f))
        return gait_command();  // plus rien devant : attendre (le capteur balaye)
    double d = f[0], a = f[1];
    has_target_ = true;
    target_x_ = f[2];
    target_y_ = f[3];
    seen_ = 0.0;
    double yaw = pymax(-yaw_rate, pymin(yaw_rate, NAV_YAW_GAIN * a));
    double err = d - NAV_FOLLOW_DIST;
    if (fabs(err) < NAV_FOLLOW_DEAD || fabs(a) > NAV_SEE) return gait_command(0.0, 0.0, yaw);
    double v = pymax(-speed / 2,
                     pymin(speed, 1.2 * (err > 0 ? err - NAV_FOLLOW_DEAD : err + NAV_FOLLOW_DEAD)));
    return gait_command(v * cos(a), v * sin(a), yaw);
}
