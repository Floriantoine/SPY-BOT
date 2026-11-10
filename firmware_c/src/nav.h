// Déplacements autonomes (GOTO, FOLLOW) : portage de hexapod/nav.py.
#pragma once
#include <stddef.h>

#include "robot.h"
#include "sensor.h"

enum NavMode { NAV_OFF, NAV_GOTO, NAV_FOLLOW };
enum NavEvent { NAV_NONE, NAV_ARRIVED, NAV_STUCK };

class Navigator {
public:
    Navigator();
    NavMode mode;
    double goal_x, goal_y;
    double speed;     // mm/s
    double yaw_rate;  // rad/s

    bool active() const { return mode != NAV_OFF; }
    void go_to(double x, double y, bool has_speed, double speed);
    void follow(bool has_speed, double speed);
    void stop() { mode = NAV_OFF; }
    // Ligne « NAV ... » ; faux si le but ne s'écrit pas en entier (inf, nan),
    // avec alors le message d'erreur de Python dans buf.
    bool status(char* buf, size_t size) const;
    // Consigne de marche pour ce pas de temps (et l'événement, s'il y en a un).
    GaitCommand command(const double pose[3], Scanner* scanner, double dt, NavEvent* event);

private:
    double last_;  // dernière direction choisie (deg)
    bool has_target_;
    double target_x_, target_y_;
    double seen_;
    bool has_best_;
    double best_, since_best_;
    bool has_plan_;
    GaitCommand plan_;
    double plan_age_;

    GaitCommand go(const double pose[3], Scanner* scanner, double dt, NavEvent* event);
    GaitCommand choose(Scanner* scanner, const double pose[3], double b, double bearing, double speed, double look);
    GaitCommand walk(double a, double speed) const;
    GaitCommand follow_cmd(const double pose[3], Scanner* scanner, double dt);
};
