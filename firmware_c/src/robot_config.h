// Fichier GÉNÉRÉ par firmware_c/tools/gen_config.py : ne pas modifier à la main.
// Source : hexapod/config.py, robot.py, gait.py, controller.py, servo.py.
#pragma once

#define NLEGS 6
#define NJOINTS (3 * NLEGS)

// --- géométrie (config.py) ---
static const double CFG_COXA = 30.0;
static const double CFG_FEMUR = 55.0;
static const double CFG_TIBIA = 94.5;
static const double CFG_STANCE_RADIUS = 78.35866003106372;
static const double CFG_STANCE_HEIGHT = 68.3;
static const double CFG_SIT_DROP = 17.075;
static const double CFG_STEP_HEIGHT = 18.900000000000002;
static const double CFG_MAX_STRIDE = 51.975;
static const int CFG_I2C_ADDRESSES[2] = {0x40, 0x41};
#define CFG_NBOARDS 2

struct JointDef { int board, channel; double zero_deg; int direction; double min_deg, max_deg; };
struct LegDef {
    const char* name;
    double mount_x, mount_y, mount_angle, cos_a, sin_a;  // cos/sin calculés par Python
    double neutral_x, neutral_y, neutral_z;
    int middle;    // patte du milieu (sangle de la batterie)
    int tripod_a;  // trépied A (RF, LM, RR)
    JointDef joint[3];  // coxa, fémur, tibia
};
static const LegDef CFG_LEGS[NLEGS] = {
    {"RF", 67.55, -39.0, -0.5235987755982988, 0.8660254037844387, -0.49999999999999994, 135.41059019340952, -78.17933001553186, -68.3, 0, 1, {{0, 5, 0.0, 1, 35.0, 145.0}, {0, 6, 0.0, -1, 30.0, 150.0}, {0, 7, -90.0, -1, 30.0, 145.0}}},
    {"RM", 0.0, -78.0, -1.5707963267948966, 6.123233995736766e-17, -1.0, 4.7980841096258915e-15, -156.35866003106372, -68.3, 1, 0, {{1, 2, 0.0, 1, 35.0, 145.0}, {1, 3, 0.0, -1, 30.0, 150.0}, {1, 4, -90.0, -1, 30.0, 145.0}}},
    {"RR", -67.55, -39.0, -2.6179938779914944, -0.8660254037844387, -0.49999999999999994, -135.41059019340952, -78.17933001553186, -68.3, 0, 1, {{1, 8, 0.0, 1, 35.0, 145.0}, {1, 9, 0.0, -1, 30.0, 150.0}, {1, 10, -90.0, -1, 30.0, 145.0}}},
    {"LF", 67.55, 39.0, 0.5235987755982988, 0.8660254037844387, 0.49999999999999994, 135.41059019340952, 78.17933001553186, -68.3, 0, 0, {{0, 2, 0.0, 1, 35.0, 145.0}, {0, 3, 0.0, -1, 30.0, 150.0}, {0, 4, -90.0, -1, 30.0, 145.0}}},
    {"LM", 0.0, 78.0, 1.5707963267948966, 6.123233995736766e-17, 1.0, 4.7980841096258915e-15, 156.35866003106372, -68.3, 1, 1, {{0, 8, 0.0, 1, 35.0, 145.0}, {0, 9, 0.0, -1, 30.0, 150.0}, {0, 10, -90.0, -1, 30.0, 145.0}}},
    {"LR", -67.55, 39.0, 2.6179938779914944, -0.8660254037844387, 0.49999999999999994, -135.41059019340952, 78.17933001553186, -68.3, 0, 0, {{1, 5, 0.0, 1, 35.0, 145.0}, {1, 6, 0.0, -1, 30.0, 150.0}, {1, 7, -90.0, -1, 30.0, 145.0}}},
};
// Voisines dans le sens trigo (robot.AROUND), en indices de CFG_LEGS.
static const int CFG_AROUND[NLEGS] = {3, 4, 5, 2, 1, 0};

// --- repli du tibia (robot.py, en radians, marge comprise) ---
#define FOLD_POINTS 11
static const double FOLD_RAD[FOLD_POINTS][2] = {{-1.5707963267948966, 0.06981317007977318}, {-1.3089969389957472, 0.3141592653589793}, {-1.0471975511965976, 0.7330382858376184}, {-0.7853981633974483, 0.8552113334772214}, {-0.5235987755982988, 0.9599310885968813}, {-0.2617993877991494, 1.0471975511965976}, {0.0, 1.0821041362364843}, {0.2617993877991494, 0.9948376736367679}, {0.5235987755982988, 0.9424777960769379}, {1.3089969389957472, 0.9424777960769379}, {1.5707963267948966, 0.9773843811168246}};
static const double BATTERY_RAD[2] = {-1.2740903539558606, 0.4886921905584123};
static const double STRAP_RAD[2] = {-0.9250245035569946, 0.6632251157578453};
static const double RAD_45 = 0.7853981633974483;
static const double RAD_55 = 0.9599310885968813;
static const double K_HALF_PI = 1.5707963267948966;
static const double K_MINUS_HALF_PI = -1.5707963267948966;
static const double DEG_PER_RAD = 57.29577951308232;
static const double SERVO_DEG_PER_RAD = 57.29577951308232;
static const double PY_PI = 3.141592653589793;

// --- robot.py ---
static const double ACCEL = 250.0;
static const double YAW_ACCEL = 1.5;
static const double DECEL = 600.0;
static const double YAW_DECEL = 4.0;
static const double STOP_TOL = 20.0;
static const double TUCK_TIME = 1.0;
static const double POSE_RECOVER = 0.5;
static const double SERVO_MARGIN = 1.0;
static const double FOLD_MARGIN = 2.0;
static const double NEIGHBOUR_LIMIT = 70.0;
static const double HOME_STEP = 10.0;
static const double HOME_REPLACE = 8.0;
static const double HOME_PERIOD = 0.06;
static const double HOME_POSE_MM = 2.0;
static const double REACH_MARGIN = 3.0;
static const double REACH_COXA = 33.0;
static const double REACH_MAX = 70.0;
static const double REACH_POSE = 10.0;
static const double LIFT_MIN = 0.5;
static const double LIFT_STEP = 0.1;
static const double HOME_RANGE_LO = -30.0;
static const double HOME_RANGE_HI = 60.0;
#define REACH_DIRS 24
#define BISECT 4

// --- gait.py ---
static const double GAIT_PERIOD = 1.4;  // TripodGait() par défaut

// --- controller.py ---
static const double RISE_TIME = 2.0;
static const double WAKE_STAGGER = 0.25;
static const double WAKE_BLEND = 1.0;
static const double WATCHDOG = 1.0;
static const double TELEMETRY_PERIOD = 0.05;
static const double TEST_DURATION = 3.0;
static const double TEST_AMPLITUDE = 0.3490658503988659;
static const double DEG = 0.017453292519943295;
static const double CALIBRATION_POSE[3] = {0.0, 0.0, -1.5707963267948966};

// --- sensor.py (capteur absent : seulement l'état affiché) ---
static const double STOP_DEFAULT = 80.0;
static const double STOP_RANGE_LO = 30.0;
static const double STOP_RANGE_HI = 300.0;

// --- servo.py (PCA9685Driver) ---
static const double SERVO_SDA = 21.0;
static const double SERVO_SCL = 22.0;
static const double SERVO_CENTER_US = 1400.0;
static const double SERVO_US_PER_DEG = 10.0;
static const double SERVO_MIN_US = 800.0;
static const double SERVO_MAX_US = 2000.0;
static const double PCA_FREQ = 50.0;

// Réponse GEOM de Python, sens des 18 servos en %d (patte, puis coxa fémur tibia).
static const char GEOM_FORMAT[] =
    "GEOM {\"coxa\": 30.0, \"femur\": 55.0, \"tibia\": 94.5, \"addresses\": [64, 65], \"legs\": [{\"name\": \"RF\", \"x\""
    ": 67.55, \"y\": -39.0, \"angle\": -0.5235987755982988, \"body_x\": 67.55, \"body_y\": -39.0, \"servos\": [[0, "
    "5], [0, 6], [0, 7]], \"joints\": [[0.0, %d, 35.0, 145.0], [0.0, %d, 30.0, 150.0], [-90.0, %d, 30.0, 14"
    "5.0]]}, {\"name\": \"RM\", \"x\": 0.0, \"y\": -78.0, \"angle\": -1.5707963267948966, \"body_x\": 0.0, \"body_y\": "
    "-78.0, \"servos\": [[1, 2], [1, 3], [1, 4]], \"joints\": [[0.0, %d, 35.0, 145.0], [0.0, %d, 30.0, 150.0]"
    ", [-90.0, %d, 30.0, 145.0]]}, {\"name\": \"RR\", \"x\": -67.55, \"y\": -39.0, \"angle\": -2.6179938779914944, "
    "\"body_x\": -67.55, \"body_y\": -39.0, \"servos\": [[1, 8], [1, 9], [1, 10]], \"joints\": [[0.0, %d, 35.0, 1"
    "45.0], [0.0, %d, 30.0, 150.0], [-90.0, %d, 30.0, 145.0]]}, {\"name\": \"LF\", \"x\": 67.55, \"y\": 39.0, \"an"
    "gle\": 0.5235987755982988, \"body_x\": 67.55, \"body_y\": 39.0, \"servos\": [[0, 2], [0, 3], [0, 4]], \"join"
    "ts\": [[0.0, %d, 35.0, 145.0], [0.0, %d, 30.0, 150.0], [-90.0, %d, 30.0, 145.0]]}, {\"name\": \"LM\", \"x\""
    ": 0.0, \"y\": 78.0, \"angle\": 1.5707963267948966, \"body_x\": 0.0, \"body_y\": 78.0, \"servos\": [[0, 8], [0,"
    " 9], [0, 10]], \"joints\": [[0.0, %d, 35.0, 145.0], [0.0, %d, 30.0, 150.0], [-90.0, %d, 30.0, 145.0]]}"
    ", {\"name\": \"LR\", \"x\": -67.55, \"y\": 39.0, \"angle\": 2.6179938779914944, \"body_x\": -67.55, \"body_y\": 39"
    ".0, \"servos\": [[1, 5], [1, 6], [1, 7]], \"joints\": [[0.0, %d, 35.0, 145.0], [0.0, %d, 30.0, 150.0], ["
    "-90.0, %d, 30.0, 145.0]]}]}"
    ;
