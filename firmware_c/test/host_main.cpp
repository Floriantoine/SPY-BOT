// Banc de test sur PC : le cœur C++ (sans matériel) piloté par des lignes
// sur l'entrée standard, pour le comparer au Controller Python
// (voir compare.py).
//
//   C <commande>   -> exécute la commande ; affiche « R <réponse> » (ou « R- »)
//   D imu ax ay az t gx gy gz panne
//                  -> valeurs brutes (int16) que rendra le faux MPU6050
//   D rng prête statut distance panne
//                  -> état du faux VL53L1X jusqu'à la prochaine ligne D rng
//   K <dt>         -> un tick ; affiche les lignes (« L ... »), les 18 angles
//                     (« A ... »), les angles servo envoyés (« S ... ») et
//                     les échanges sur le bus I²C (« X ... »)
//
// Arguments : [--imu ok|absent|badinit] [--rng ok|absent|badid]
// [--imucal axes zéro_roulis zéro_tangage] [calibration]
// (fichier de 18 trims puis 18 inversions). Au démarrage, avec des appareils :
// « X ... » (recherche des appareils), puis « H <ligne SENSOR> » et « H <ligne IMU> ».
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/controller.h"

unsigned long platform_micros() { return 0; }
long platform_free_memory() { return -1; }
void platform_delay_ms(unsigned) {}

class TestDriver : public ServoDriver {
public:
    double deg[CFG_NBOARDS][16];
    bool set[CFG_NBOARDS][16];
    int relaxed;
    TestDriver() : relaxed(0) { memset(set, 0, sizeof(set)); }
    void set_angle(int board, int channel, double d) {
        deg[board][channel] = d;
        set[board][channel] = true;
    }
    void relax() { relaxed++; }
};

class MemStore : public CalibStore {
public:
    bool have;
    double trims[NJOINTS];
    bool inv[NJOINTS], is_int[NJOINTS];
    bool have_imu;
    int axes[3][2];
    double zero[2];
    MemStore() : have(false), have_imu(false) { memset(is_int, 0, sizeof(is_int)); }
    bool load_imu(int a[3][2], double z[2]) {
        if (!have_imu) return false;
        memcpy(a, axes, sizeof(axes));
        memcpy(z, zero, sizeof(zero));
        return true;
    }
    bool save_imu(const int a[3][2], const double z[2]) {
        memcpy(axes, a, sizeof(axes));
        memcpy(zero, z, sizeof(zero));
        have_imu = true;
        return true;
    }
    bool load(double t[NJOINTS], bool i[NJOINTS], bool n[NJOINTS]) {
        if (!have) return false;
        memcpy(t, trims, sizeof(trims));
        memcpy(i, inv, sizeof(inv));
        memcpy(n, is_int, sizeof(is_int));
        return true;
    }
    bool save(const double t[NJOINTS], const bool i[NJOINTS], const bool n[NJOINTS]) {
        memcpy(trims, t, sizeof(trims));
        memcpy(inv, i, sizeof(inv));
        memcpy(is_int, n, sizeof(is_int));
        have = true;
        return true;
    }
};

// Faux bus I²C : même modèle des appareils que FakeBus dans compare.py, et
// journal des échanges dans le même format.
class FakeBus : public I2CBus {
public:
    // MPU6050
    bool imu_on, imu_badinit, imu_fail;
    int imu_raw[7];
    // VL53L1X
    bool rng_on, rng_badid, rng_fail, rng_ready;
    int rng_status, rng_dist, boot_polls, wake_polls, pointer;
    char log[16384];
    size_t len;

    FakeBus()
        : imu_on(false), imu_badinit(false), imu_fail(false), rng_on(false), rng_badid(false), rng_fail(false),
          rng_ready(false), rng_status(0), rng_dist(0), boot_polls(2), wake_polls(2), pointer(0), len(0) {
        memset(imu_raw, 0, sizeof(imu_raw));
        log[0] = 0;
    }

    void add(int ok, const uint8_t* data, size_t n, const char* fmt, ...) {
        char head[32];
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(head, sizeof(head), fmt, ap);
        va_end(ap);
        len += snprintf(log + len, sizeof(log) - len, "%s%s", len ? " " : "", head);
        for (size_t i = 0; i < n && len < sizeof(log) - 4; i++) len += snprintf(log + len, sizeof(log) - len, "%02x", data[i]);
        if (!ok && len < sizeof(log) - 2) len += snprintf(log + len, sizeof(log) - len, "!");
    }

    bool fails(uint8_t addr) const {
        if (addr == IMU_ADDRESS) return !imu_on || imu_fail;
        if (addr == VL53_ADDRESS) return !rng_on || rng_fail;
        return true;
    }

    bool scan_has(uint8_t addr) {
        add(1, NULL, 0, "scan");
        return (addr == IMU_ADDRESS && imu_on) || (addr == VL53_ADDRESS && rng_on);
    }

    bool writeto(uint8_t addr, const uint8_t* d, size_t n) {
        bool ok = !fails(addr);
        add(ok, d, n, "w%02x:", addr);
        if (!ok) return false;
        if (addr == VL53_ADDRESS && n >= 2) {
            int reg = (d[0] << 8) | d[1];
            if (n == 2)
                pointer = reg;
            else if (reg == 0x0086 && d[2] == 0x01)
                rng_ready = false;  // interruption effacée : mesure suivante
        }
        return true;
    }

    bool readfrom(uint8_t addr, uint8_t* d, size_t n) {
        bool ok = !fails(addr);
        add(ok, NULL, 0, "r%02x:%d", addr, (int)n);
        if (!ok) return false;
        memset(d, 0, n);
        if (addr != VL53_ADDRESS) return true;
        switch (pointer) {
            case 0x010F:
                d[0] = rng_badid ? 0xEE : 0xEA;
                if (n > 1) d[1] = 0xCC;
                break;
            case 0x00E5:
                if (boot_polls > 0)
                    boot_polls--;
                else
                    d[0] = 1;
                break;
            case 0x0030: d[0] = 0x01; break;
            case 0x0031:
                if (!rng_ready && wake_polls > 0) {
                    wake_polls--;
                    if (wake_polls == 0) rng_ready = true;
                }
                d[0] = rng_ready ? 1 : 0;
                break;
            case 0x0089: d[0] = (uint8_t)rng_status; break;
            case 0x0096:
                d[0] = (uint8_t)(rng_dist >> 8);
                if (n > 1) d[1] = (uint8_t)(rng_dist & 0xFF);
                break;
        }
        return true;
    }

    bool writeto_mem(uint8_t addr, uint8_t reg, const uint8_t* d, size_t n) {
        bool ok = !fails(addr) && !(addr == IMU_ADDRESS && imu_badinit && reg == 0x1B);
        add(ok, d, n, "W%02x:%02x:", addr, reg);
        return ok;
    }

    bool readfrom_mem(uint8_t addr, uint8_t reg, uint8_t* d, size_t n) {
        bool ok = !fails(addr);
        add(ok, NULL, 0, "R%02x:%02x:%d", addr, reg, (int)n);
        if (!ok) return false;
        memset(d, 0, n);
        if (addr == IMU_ADDRESS && reg == 0x3B)
            for (size_t i = 0; i < 7 && 2 * i + 1 < n; i++) {
                int v = imu_raw[i] & 0xFFFF;
                d[2 * i] = (uint8_t)(v >> 8);
                d[2 * i + 1] = (uint8_t)(v & 0xFF);
            }
        return true;
    }

    void flush() {
        printf("X%s%s\n", len ? " " : "", log);
        len = 0;
        log[0] = 0;
    }
};

static void print_prefixed(const char* prefix, const char* text) {
    const char* p = text;
    while (true) {
        const char* nl = strchr(p, '\n');
        size_t n = nl ? (size_t)(nl - p) : strlen(p);
        printf("%s%.*s\n", prefix, (int)n, p);
        if (!nl) break;
        p = nl + 1;
    }
}

int main(int argc, char** argv) {
    static TestDriver driver;
    static MemStore store;
    static FakeBus bus;
    const char* imu_mode = "absent";
    const char* rng_mode = "absent";
    const char* calib = NULL;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--imu") && i + 1 < argc)
            imu_mode = argv[++i];
        else if (!strcmp(argv[i], "--rng") && i + 1 < argc)
            rng_mode = argv[++i];
        else if (!strcmp(argv[i], "--imucal") && i + 3 < argc) {
            // Réglages du gyroscope déjà en mémoire : axes (comme IMU AXES), zéro (rad).
            const char* error;
            store.have_imu = true;
            if (!imu_parse_axes(argv[++i], store.axes, &error)) store.axes[0][1] = 7;  // invalide : ignoré
            store.zero[0] = atof(argv[++i]);
            store.zero[1] = atof(argv[++i]);
        }
        else
            calib = argv[i];
    }
    if (calib) {
        FILE* f = fopen(calib, "r");
        if (!f) return 2;
        int v;
        for (int i = 0; i < NJOINTS; i++)
            if (fscanf(f, "%lf", &store.trims[i]) != 1) return 2;
        for (int i = 0; i < NJOINTS; i++) {
            if (fscanf(f, "%d", &v) != 1) return 2;
            store.inv[i] = v != 0;
        }
        store.have = true;
        fclose(f);
    }
    bool devices = strcmp(imu_mode, "absent") || strcmp(rng_mode, "absent");
    bus.imu_on = strcmp(imu_mode, "absent") != 0;
    bus.imu_badinit = !strcmp(imu_mode, "badinit");
    bus.rng_on = strcmp(rng_mode, "absent") != 0;
    bus.rng_badid = !strcmp(rng_mode, "badid");
    // Comme main.py : capteur avant, puis gyroscope.
    static Vl53l1x ranger_dev(&bus);
    static Mpu6050 imu_dev(&bus);
    Ranger* ranger = NULL;
    Imu* imu = NULL;
    if (devices) {
        ranger = ranger_dev.probe() ? &ranger_dev : NULL;
        imu = imu_dev.probe() ? &imu_dev : NULL;
    }
    static Controller ctrl(&driver, &store, ranger, imu);
    if (devices) {
        bus.flush();
        printf("H %s\n", ctrl.sensor_status());
        printf("H %s\n", ctrl.imu_status());
    }
    char line[512];
    while (fgets(line, sizeof(line), stdin)) {
        size_t n = strlen(line);
        while (n && (line[n - 1] == '\n' || line[n - 1] == '\r')) line[--n] = 0;
        if (line[0] == 'C') {
            const char* r = ctrl.handle(line + 2);
            if (r)
                print_prefixed("R ", r);
            else
                printf("R-\n");
        } else if (line[0] == 'D') {
            int v[8], fail;
            if (!strncmp(line, "D imu ", 6)) {
                if (sscanf(line + 6, "%d %d %d %d %d %d %d %d", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &fail) != 8)
                    return 3;
                memcpy(bus.imu_raw, v, sizeof(bus.imu_raw));
                bus.imu_fail = fail != 0;
            } else if (!strncmp(line, "D rng ", 6)) {
                if (sscanf(line + 6, "%d %d %d %d", &v[0], &v[1], &v[2], &fail) != 4) return 3;
                bus.rng_ready = v[0] != 0;
                bus.rng_status = v[1];
                bus.rng_dist = v[2];
                bus.rng_fail = fail != 0;
            } else {
                return 3;
            }
        } else if (line[0] == 'K') {
            const char* out = ctrl.tick(atof(line + 2));
            if (out) print_prefixed("L ", out);
            printf("A");
            for (int i = 0; i < NLEGS; i++) {
                const LegAngles& a = ctrl.angles->leg[i];
                printf(" %.17g %.17g %.17g", a.coxa, a.femur, a.tibia);
            }
            printf("\nS");
            for (int b = 0; b < CFG_NBOARDS; b++)
                for (int c = 0; c < 16; c++)
                    if (driver.set[b][c]) printf(" %d:%d=%.17g", b, c, driver.deg[b][c]);
            printf("\n");
            if (devices) bus.flush();
        }
        fflush(stdout);
    }
    return 0;
}
