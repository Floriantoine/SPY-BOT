#include "pyfmt.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

long long pyround(double x) {
    // rint arrondit au pair le plus proche (mode d'arrondi par défaut), comme round().
    return (long long)rint(x);
}

double pymod(double x, double y) {
    double m = fmod(x, y);
    if (m != 0.0) {
        if ((y < 0) != (m < 0)) m += y;
    } else {
        m = copysign(0.0, y);
    }
    return m;
}

static bool is_digit(char c) { return c >= '0' && c <= '9'; }

static bool ieq(const char* a, const char* b) {
    for (; *a && *b; a++, b++) {
        char c = *a;
        if (c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');
        if (c != *b) return false;
    }
    return *a == 0 && *b == 0;
}

// Chiffres avec des « _ » seulement entre deux chiffres (syntaxe Python).
// Copie les chiffres dans out. Retourne le nombre de chiffres, -1 si invalide.
static int digits(const char*& p, char*& out) {
    int n = 0;
    while (true) {
        if (is_digit(*p)) {
            *out++ = *p++;
            n++;
        } else if (*p == '_' && n > 0 && is_digit(p[1])) {
            p++;
        } else {
            break;
        }
    }
    return n;
}

bool pyfloat(const char* s, double* out) {
    // Les arguments viennent de split() : pas d'espace autour.
    char buf[160];
    if (strlen(s) >= sizeof(buf) - 8) return false;
    const char* p = s;
    char* o = buf;
    if (*p == '+' || *p == '-') *o++ = *p++;
    const char* rest = p;
    if (ieq(rest, "inf") || ieq(rest, "infinity")) {
        *out = (buf[0] == '-') ? -INFINITY : INFINITY;
        return true;
    }
    if (ieq(rest, "nan")) {
        *out = NAN;
        return true;
    }
    int n = digits(p, o);
    int m = 0;
    if (*p == '.') {
        *o++ = *p++;
        m = digits(p, o);
    }
    if (n + m == 0) return false;
    if (*p == 'e' || *p == 'E') {
        *o++ = *p++;
        if (*p == '+' || *p == '-') *o++ = *p++;
        if (digits(p, o) == 0) return false;
    }
    if (*p != 0) return false;
    *o = 0;
    *out = strtod(buf, NULL);
    return true;
}

bool pyint(const char* s, long* out) {
    char buf[64];
    if (strlen(s) >= sizeof(buf) - 2) return false;
    const char* p = s;
    char* o = buf;
    if (*p == '+' || *p == '-') *o++ = *p++;
    if (digits(p, o) == 0 || *p != 0) return false;
    *o = 0;
    *out = strtol(buf, NULL, 10);
    return true;
}

void pyrepr_str(const char* s, char* out, size_t size) {
    bool single = strchr(s, '\'') != NULL;
    bool dbl = strchr(s, '"') != NULL;
    char q = (single && !dbl) ? '"' : '\'';
    size_t i = 0;
    if (size < 3) {
        if (size) out[0] = 0;
        return;
    }
    out[i++] = q;
    for (const char* p = s; *p && i + 4 < size; p++) {
        if (*p == '\\' || *p == q) out[i++] = '\\';
        out[i++] = *p;
    }
    out[i++] = q;
    out[i] = 0;
}

void pyrepr_double(double v, char* out, size_t size) {
    if (isnan(v)) {
        snprintf(out, size, "NaN");  // json.dumps
        return;
    }
    if (isinf(v)) {
        snprintf(out, size, v > 0 ? "Infinity" : "-Infinity");
        return;
    }
    // Le plus court qui redonne exactement v (comme repr() de Python).
    char e[40];
    int prec = 1;
    for (; prec <= 17; prec++) {
        snprintf(e, sizeof(e), "%.*e", prec - 1, v);
        if (strtod(e, NULL) == v) break;
    }
    // e = [-]d[.ddd]e[+-]XX
    char digs[24];
    int nd = 0;
    bool neg = e[0] == '-';
    const char* p = e + (neg ? 1 : 0);
    for (; *p && *p != 'e'; p++)
        if (is_digit(*p)) digs[nd++] = *p;
    int exp = atoi(p + 1);
    while (nd > 1 && digs[nd - 1] == '0') nd--;
    digs[nd] = 0;
    char res[64];
    int k = 0;
    if (neg) res[k++] = '-';
    if (exp >= -4 && exp < 16) {
        if (exp < 0) {
            res[k++] = '0';
            res[k++] = '.';
            for (int i = 0; i < -exp - 1; i++) res[k++] = '0';
            for (int i = 0; i < nd; i++) res[k++] = digs[i];
        } else {
            for (int i = 0; i <= exp; i++) res[k++] = i < nd ? digs[i] : '0';
            res[k++] = '.';
            if (nd > exp + 1) {
                for (int i = exp + 1; i < nd; i++) res[k++] = digs[i];
            } else {
                res[k++] = '0';
            }
        }
        res[k] = 0;
    } else {
        res[k++] = digs[0];
        if (nd > 1) {
            res[k++] = '.';
            for (int i = 1; i < nd; i++) res[k++] = digs[i];
        }
        snprintf(res + k, sizeof(res) - k, "e%c%02d", exp < 0 ? '-' : '+', exp < 0 ? -exp : exp);
    }
    snprintf(out, size, "%s", res);
}
