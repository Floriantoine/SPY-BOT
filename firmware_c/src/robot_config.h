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
    int quad_rank; // rang dans gait.QUAD_ORDER, -1 si repliée à 4 pattes
    int wave_rank; // rang dans gait.WAVE_ORDER (marche « vague »)
    int tucked;    // repliée à 4 pattes (gait.QUAD_TUCKED)
    int diagonal;  // patte opposée en diagonale (gait._diagonal), -1 sinon
    double tuck_x, tuck_y, tuck_z;  // pied replié, repère corps (Hexapod.tucked_foot)
    JointDef joint[3];  // coxa, fémur, tibia
};
static const LegDef CFG_LEGS[NLEGS] = {
    {"RF", 67.55, -39.0, -0.5235987755982988, 0.8660254037844387, -0.49999999999999994, 135.41059019340952, -78.17933001553186, -68.3, 0, 1, 3, 3, 0, 5, 173.45636190949824, -100.14506655734274, -27.337837438848887, {{0, 5, 0.0, 1, 35.0, 145.0}, {0, 6, 0.0, -1, 30.0, 150.0}, {0, 7, -90.0, -1, 30.0, 145.0}}},
    {"RM", 0.0, -78.0, -1.5707963267948966, 6.123233995736766e-17, -1.0, 4.7980841096258915e-15, -156.35866003106372, -68.3, 1, 0, -1, 4, 1, -1, 7.488111004310166e-15, -200.2901331146855, -27.337837438848887, {{1, 2, 0.0, 1, 35.0, 145.0}, {1, 3, 0.0, -1, 30.0, 150.0}, {1, 4, -90.0, -1, 30.0, 145.0}}},
    {"RR", -67.55, -39.0, -2.6179938779914944, -0.8660254037844387, -0.49999999999999994, -135.41059019340952, -78.17933001553186, -68.3, 0, 1, 2, 5, 0, 3, -173.45636190949824, -100.14506655734274, -27.337837438848887, {{1, 8, 0.0, 1, 35.0, 145.0}, {1, 9, 0.0, -1, 30.0, 150.0}, {1, 10, -90.0, -1, 30.0, 145.0}}},
    {"LF", 67.55, 39.0, 0.5235987755982988, 0.8660254037844387, 0.49999999999999994, 135.41059019340952, 78.17933001553186, -68.3, 0, 0, 1, 2, 0, 2, 173.45636190949824, 100.14506655734274, -27.337837438848887, {{0, 2, 0.0, 1, 35.0, 145.0}, {0, 3, 0.0, -1, 30.0, 150.0}, {0, 4, -90.0, -1, 30.0, 145.0}}},
    {"LM", 0.0, 78.0, 1.5707963267948966, 6.123233995736766e-17, 1.0, 4.7980841096258915e-15, 156.35866003106372, -68.3, 1, 1, -1, 1, 1, -1, 7.488111004310166e-15, 200.2901331146855, -27.337837438848887, {{0, 8, 0.0, 1, 35.0, 145.0}, {0, 9, 0.0, -1, 30.0, 150.0}, {0, 10, -90.0, -1, 30.0, 145.0}}},
    {"LR", -67.55, 39.0, 2.6179938779914944, -0.8660254037844387, 0.49999999999999994, -135.41059019340952, 78.17933001553186, -68.3, 0, 0, 0, 0, 0, 0, -173.45636190949824, 100.14506655734274, -27.337837438848887, {{1, 5, 0.0, 1, 35.0, 145.0}, {1, 6, 0.0, -1, 30.0, 150.0}, {1, 7, -90.0, -1, 30.0, 145.0}}},
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
static const double TOUCH_FROM = 0.5;
static const double TOUCH_RATE = 0.1;
static const double FEEL_SMOOTH = 0.06;
static const double SINK_WINDOW = 0.5;
static const double EXTEND_SPEED = 150.0;
static const double GROUND_MAX = 25.0;
static const double WAVE_SPEED = 20.0;
static const double WAVE_BACK = 0.8;
static const double WAVE_ACCEL = 0.3;
static const double WAVE_ENTER = 0.9;
static const double WAVE_SPEED_LO = 5.0;
static const double WAVE_SPEED_HI = 60.0;
enum { WAVE_OFF = 0, WAVE_ON = 1, WAVE_AUTO = 2 };
static const double HOME_RANGE_LO = -30.0;
static const double HOME_RANGE_HI = 60.0;
#define REACH_DIRS 24
#define BISECT 4

// --- gait.py ---
static const double GAIT_PERIOD = 1.4;  // TripodGait() par défaut
static const double QUAD_SHIFT = 0.35;
// Rang dans QUAD_ORDER -> indice de CFG_LEGS.
static const int CFG_QUAD_ORDER[4] = {5, 3, 2, 0};
// QuadGait de Hexapod (robot.py) : réglages de départ.
static const double QUAD_PERIOD = 2.8;
static const double QUAD_STEP_HEIGHT = 18.900000000000002;
static const double QUAD_MAX_STRIDE = 34.019999999999996;
static const double QUAD_SWAY = 28.349999999999998;
// Rang dans WAVE_ORDER -> indice de CFG_LEGS.
static const int CFG_WAVE_ORDER[NLEGS] = {5, 4, 3, 0, 1, 2};
static const double WAVE_PERIOD = 2.8;  // WaveGait de Hexapod
static const double TRIPOD_STEP_HEIGHT = 18.900000000000002;
static const double TRIPOD_MAX_STRIDE = 51.975;

// --- controller.py ---
static const double RISE_TIME = 2.0;
static const double WAKE_STAGGER = 0.25;
static const double WAKE_BLEND = 1.0;
static const double WATCHDOG = 1.0;
static const double LINK_TIMEOUT = 5.0;
static const double TELEMETRY_PERIOD = 0.05;
static const double TEST_DURATION = 3.0;
static const double TEST_AMPLITUDE = 0.3490658503988659;
static const double DEG = 0.017453292519943295;
static const double TILT_STOP = 0.3490658503988659;
static const double TILT_OK = 0.20943951023931953;
static const double SLOW_RATE = 0.3;
static const double SLOW_SPAN = 0.6;
static const double SLOW_MIN = 0.3;
static const double SLOW_RECOVER = 1.5;
static const double CALIBRATION_POSE[3] = {0.0, 0.0, -1.5707963267948966};

// --- sensor.py ---
static const double STOP_DEFAULT = 80.0;
static const double STOP_RANGE_LO = 30.0;
static const double STOP_RANGE_HI = 300.0;
static const double PAN_AXIS_X = 8.0;
static const double CHIP_OFFSET = 8.0;
static const double PAN_LIMIT = 60.0;
static const double SCAN_STEP = 5.0;
static const double SCAN_PERIOD = 0.1;
static const double SCAN_TIMEOUT = 0.25;
static const double WALK_HALF = 40.0;
static const double MIN_RANGE = 120.0;
static const double MAX_RANGE = 4000.0;
static const double POINT_LIFE = 4.0;
static const double FOOT_RADIUS = 160.0;
static const double SENSOR_DEG = 0.017453292519943295;
// Mesures gardées au plus (une toutes les SCAN_PERIOD s pendant POINT_LIFE s), avec de la marge.
#define SCAN_MAX_POINTS 56
static const double PATH_MARGIN = 30.0;  // Scanner.path_blocked(margin=...)
#define PAN_BOARD 1
#define PAN_CHANNEL 11
#define PAN_DIRECTION 1
#define VL53_ADDRESS 0x29
#define VL53_FIRST_TRIES 250
#define VL53_MODEL_ID 0xeacc
#define VL53_CONFIG_LEN 91
// Registres 0x2D.. du VL53L1X (sensor._DEFAULT_CONFIG).
static const unsigned char VL53_DEFAULT_CONFIG[VL53_CONFIG_LEN] = {
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x0A, 0x21,
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xDB, 0x0F, 0x01, 0xF1, 0x0D, 0x01, 0x68, 0x00, 0x80, 0x08, 0xB8, 0x00, 0x00, 0x00,
    0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00,
    0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x01, 0x40,
};

// --- imu.py ---
#define IMU_ADDRESS 0x68
static const int IMU_AXES[3][2] = {{0, 1}, {1, 1}, {2, 1}};
static const double ACCEL_TRUST = 0.02;
static const double BIAS_TIME = 1.0;
static const double BIAS_SPREAD = 0.05;
static const double LEVEL_GAIN = 2.0;
static const double LEVEL_LIMIT = 0.17453292519943295;
static const double ZERO_MAX = 0.17453292519943295;
static const char AXES_HELP[] = "axes : trois axes de la carte parmi x y z, avec leur signe (ex. +y-x+z)";
static const char AXES_MIRROR[] = "axes : ce n'est pas une rotation (inverser ou échanger un autre axe aussi)";
#define IMU_FAIL_MAX 5
static const double ACCEL_OK_LO = 0.8;
static const double ACCEL_OK_HI = 1.2;
static const double IMU_ACCEL_SCALE = 8192.0;
static const double IMU_GYRO_SCALE = 3752.8735581068922;

// --- nav.py ---
static const double NAV_ARRIVE = 30.0;
static const double NAV_LOOKAHEAD = 300.0;
static const double NAV_SEE = 0.8726646259971648;
static const double NAV_YAW_GAIN = 1.5;
static const double NAV_MIN_SPEED = 20.0;
static const double NAV_FOLLOW_DIST = 400.0;
static const double NAV_FOLLOW_RANGE = 1200.0;
static const double NAV_FOLLOW_SIDE = 1.2217304763960306;
static const double NAV_FOLLOW_FRESH = 1.0;
static const double NAV_FOLLOW_DEAD = 40.0;
static const double NAV_FOLLOW_TRACK = 250.0;
static const double NAV_FOLLOW_LOST = 2.5;
static const double NAV_STUCK_TIME = 10.0;
static const double NAV_STUCK_GAIN = 30.0;
static const double NAV_REPLAN = 0.1;
static const double NAV_DEG = 0.017453292519943295;
#define NAV_STEP 10
#define NAV_SIDE 90
static const double NAV_SPEED = 60.0;  // Navigator() par défaut
static const double NAV_YAW_RATE = 0.6;

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
