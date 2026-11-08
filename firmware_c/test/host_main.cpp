// Banc de test sur PC : le cœur C++ (sans matériel) piloté par des lignes
// sur l'entrée standard, pour le comparer au Controller Python
// (voir compare.py).
//
//   C <commande>   -> exécute la commande ; affiche « R <réponse> » (ou « R- »)
//   K <dt>         -> un tick ; affiche les lignes (« L ... »), les 18 angles
//                     (« A ... ») et les angles servo envoyés (« S ... »)
//
// Argument : fichier de calibration de départ (18 trims puis 18 inversions), facultatif.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/controller.h"

unsigned long platform_micros() { return 0; }
long platform_free_memory() { return -1; }

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
    MemStore() : have(false) { memset(is_int, 0, sizeof(is_int)); }
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
    if (argc > 1) {
        FILE* f = fopen(argv[1], "r");
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
    static Controller ctrl(&driver, &store);
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
        }
        fflush(stdout);
    }
    return 0;
}
