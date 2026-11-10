// SPY-BOT, version C++ (V2) : programme de l'ESP32.
//
// Remplace main.py de la version MicroPython, avec le même protocole texte
// sur le port série USB (115200 bauds) : l'appli PC (pc/server.py) marche
// sans changement. Les servos restent relâchés jusqu'à POWER 1.
//
// Gyroscope MPU6050 (0x68) et capteur avant VL53L1X (0x29) sur le même bus
// I²C que les PCA9685, s'ils sont branchés (cherchés au démarrage, comme main.py).
#ifdef ARDUINO
#include <Arduino.h>
#include <Preferences.h>
#include <esp_system.h>
#include <stdio.h>
#include <string.h>

#include "controller.h"
#include "pca9685.h"
#include "wire_bus.h"

static const unsigned long LOOP_MS = 20;  // 50 Hz
static const size_t CMD_LINE_MAX = 120;

unsigned long platform_micros() { return micros(); }
long platform_free_memory() { return (long)ESP.getFreeHeap(); }
void platform_delay_ms(unsigned ms) { delay(ms); }

// Calibration (trims, sens des servos) dans la mémoire NVS de l'ESP32.
class NvsStore : public CalibStore {
public:
    struct Blob {
        uint32_t version;
        double trims[NJOINTS];
        uint8_t inv[NJOINTS];
        uint8_t is_int[NJOINTS];
    };
    bool load(double trims[NJOINTS], bool inv[NJOINTS], bool is_int[NJOINTS]) {
        Preferences p;
        if (!p.begin("spybot", true)) return false;
        Blob b;
        size_t n = p.getBytesLength("calib") == sizeof(b) ? p.getBytes("calib", &b, sizeof(b)) : 0;
        p.end();
        if (n != sizeof(b) || b.version != 1) return false;
        for (int i = 0; i < NJOINTS; i++) {
            trims[i] = b.trims[i];
            inv[i] = b.inv[i] != 0;
            is_int[i] = b.is_int[i] != 0;
        }
        return true;
    }
    bool save(const double trims[NJOINTS], const bool inv[NJOINTS], const bool is_int[NJOINTS]) {
        Blob b;
        memset(&b, 0, sizeof(b));
        b.version = 1;
        for (int i = 0; i < NJOINTS; i++) {
            b.trims[i] = trims[i];
            b.inv[i] = inv[i] ? 1 : 0;
            b.is_int[i] = is_int[i] ? 1 : 0;
        }
        Preferences p;
        if (!p.begin("spybot", false)) return false;
        size_t n = p.putBytes("calib", &b, sizeof(b));
        p.end();
        return n == sizeof(b);
    }
    // Réglages du gyroscope : autre clé, les anciennes calibrations restent lisibles.
    struct ImuBlob {
        uint32_t version;
        int8_t axes[3][2];
        double zero[2];
    };
    bool load_imu(int axes[3][2], double zero[2]) {
        Preferences p;
        if (!p.begin("spybot", true)) return false;
        ImuBlob b;
        size_t n = p.getBytesLength("imu") == sizeof(b) ? p.getBytes("imu", &b, sizeof(b)) : 0;
        p.end();
        if (n != sizeof(b) || b.version != 1) return false;
        for (int k = 0; k < 3; k++) {
            axes[k][0] = b.axes[k][0];
            axes[k][1] = b.axes[k][1];
        }
        zero[0] = b.zero[0];
        zero[1] = b.zero[1];
        return true;
    }
    bool save_imu(const int axes[3][2], const double zero[2]) {
        ImuBlob b;
        memset(&b, 0, sizeof(b));
        b.version = 1;
        for (int k = 0; k < 3; k++) {
            b.axes[k][0] = (int8_t)axes[k][0];
            b.axes[k][1] = (int8_t)axes[k][1];
        }
        b.zero[0] = zero[0];
        b.zero[1] = zero[1];
        Preferences p;
        if (!p.begin("spybot", false)) return false;
        size_t n = p.putBytes("imu", &b, sizeof(b));
        p.end();
        return n == sizeof(b);
    }
    // Course réelle des servos (TRAVEL) : autre clé encore, les anciennes
    // calibrations restent lisibles (et gardent 10 µs par degré).
    struct TravelBlob {
        uint32_t version;
        double us[NJOINTS];
    };
    bool load_travel(double us[NJOINTS]) {
        Preferences p;
        if (!p.begin("spybot", true)) return false;
        TravelBlob b;
        size_t n = p.getBytesLength("travel") == sizeof(b) ? p.getBytes("travel", &b, sizeof(b)) : 0;
        p.end();
        if (n != sizeof(b) || b.version != 1) return false;
        for (int i = 0; i < NJOINTS; i++) us[i] = b.us[i];
        return true;
    }
    bool save_travel(const double us[NJOINTS]) {
        TravelBlob b;
        memset(&b, 0, sizeof(b));
        b.version = 1;
        for (int i = 0; i < NJOINTS; i++) b.us[i] = us[i];
        Preferences p;
        if (!p.begin("spybot", false)) return false;
        size_t n = p.putBytes("travel", &b, sizeof(b));
        p.end();
        return n == sizeof(b);
    }
};

static PCA9685Driver driver;
static NvsStore store;
static WireBus bus;
static Vl53l1x ranger_dev(&bus);
static Mpu6050 imu_dev(&bus);
static Controller* ctrl = NULL;
static char line_buf[CMD_LINE_MAX + 1];
static size_t line_len = 0;
static unsigned long last_ms = 0;
static unsigned long reported_errors = 0;
static unsigned long reported_at = 0;
static bool imu_present = false;
static unsigned long imu_errors = 0;
static unsigned long imu_at = 0;

// Pourquoi l'ESP32 a (re)démarré (noms de machine.reset_cause() en MicroPython).
static const char* reset_cause() {
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON: return "PWRON_RESET";
        case ESP_RST_EXT: return "HARD_RESET";
        case ESP_RST_SW: return "SOFT_RESET";
        case ESP_RST_PANIC: return "PANIC_RESET";
        case ESP_RST_INT_WDT:
        case ESP_RST_TASK_WDT:
        case ESP_RST_WDT: return "WDT_RESET";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP_RESET";
        case ESP_RST_BROWNOUT: return "BROWNOUT_RESET";
        default: return "?";
    }
}

// Redémarrage logiciel (esp_restart) : servos relâchés avant.
static void relax_on_shutdown() { driver.relax(); }

static void read_commands() {
    while (Serial.available() > 0) {
        int ch = Serial.read();
        if (ch < 0) break;
        if (ch == '\r' || ch == '\n') {
            if (line_len) {
                line_buf[line_len] = 0;
                const char* reply = ctrl->handle(line_buf);
                if (reply) Serial.println(reply);
            }
            line_len = 0;
        } else if (line_len < CMD_LINE_MAX) {
            line_buf[line_len++] = (char)ch;
        }
    }
}

void setup() {
    setCpuFrequencyMhz(240);
    Serial.setRxBufferSize(1024);
    Serial.setTxBufferSize(4096);
    Serial.begin(115200);
    driver.begin();
    // Servos relâchés jusqu'à POWER 1 (aussi après un redémarrage sur erreur).
    driver.relax();
    esp_register_shutdown_handler(relax_on_shutdown);
    // Capteur avant (VL53L1X) sur le même bus I²C, s'il est branché.
    Ranger* ranger = ranger_dev.probe() ? &ranger_dev : NULL;
    // Gyroscope MPU6050 (0x68), même bus. Robot immobile au démarrage : biais
    // mesuré pendant la première seconde.
    imu_present = imu_dev.probe();
    Imu* imu = imu_present ? &imu_dev : NULL;
    static Controller controller(&driver, &store, ranger, imu);
    ctrl = &controller;
    Serial.println("HELLO SPY-BOT");
    Serial.print("OK BOOT ");
    Serial.println(reset_cause());
    Serial.println(ctrl->sensor_status());
    Serial.println(ctrl->imu_status());
    for (int b = 0; b < CFG_NBOARDS; b++) {
        if (!driver.present(b)) {
            char msg[80];
            snprintf(msg, sizeof(msg), "ERR carte PCA9685 0x%02x absente : ses servos sont ignores", CFG_I2C_ADDRESSES[b]);
            Serial.println(msg);
        }
    }
    last_ms = reported_at = imu_at = millis();
}

void loop() {
    unsigned long start = millis();
    read_commands();
    double dt = (start - last_ms) / 1000.0;
    last_ms = start;
    const char* lines = ctrl->tick(dt);
    if (lines) Serial.println(lines);
    if (driver.errors != reported_errors && start - reported_at > 1000) {
        reported_errors = driver.errors;
        reported_at = start;
        char msg[96];
        snprintf(msg, sizeof(msg), "ERR I2C : %lu écritures perdues (câbles SDA/SCL, parasites ?)", reported_errors);
        Serial.println(msg);
    }
    if (imu_present && imu_dev.errors != imu_errors && start - imu_at > 1000) {
        imu_errors = imu_dev.errors;
        imu_at = start;
        char msg[96];
        snprintf(msg, sizeof(msg), "ERR I2C gyroscope : %lu lectures perdues (câbles SDA/SCL, parasites ?)", imu_errors);
        Serial.println(msg);
    }
    unsigned long spent = millis() - start;
    if (spent < LOOP_MS) delay(LOOP_MS - spent);
}
#endif
