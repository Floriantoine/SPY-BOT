// Petits outils pour écrire et lire les nombres exactement comme Python :
// le PC reçoit les mêmes lignes qu'avec la version MicroPython.
#pragma once
#include <stddef.h>

// min(a, b) et max(a, b) de Python : le premier, sauf si le second est
// strictement plus petit (plus grand). Compte pour les NaN.
static inline double pymin(double a, double b) { return b < a ? b : a; }
static inline double pymax(double a, double b) { return b > a ? b : a; }

// round(x) de Python (au pair le plus proche), en entier.
long long pyround(double x);

// x % y de Python pour des flottants (résultat du signe de y).
double pymod(double x, double y);

// float(s) de Python. Faux si la chaîne n'est pas un nombre.
bool pyfloat(const char* s, double* out);

// int(s, 10) de Python. Faux si la chaîne n'est pas un entier.
bool pyint(const char* s, long* out);

// repr(s) d'une chaîne Python ('abc', "l'a"...).
void pyrepr_str(const char* s, char* out, size_t size);

// Nombre écrit comme json.dumps() de Python (repr d'un float : 0.0, 1.25, 1e-05).
void pyrepr_double(double v, char* out, size_t size);
