// Interface des servos : PCA9685 sur l'ESP32, ou pilote factice pour les tests sur PC.
#pragma once

class ServoDriver {
public:
    virtual ~ServoDriver() {}
    // Un servo (carte, voie) à `deg` degrés servo (0..180).
    virtual void set_angle(int board, int channel, double deg) = 0;
    // Vrai si set_angles() est disponible (sinon un set_angle par servo).
    virtual bool has_batch() const { return false; }
    // Servos sur des voies consécutives d'une même carte, en une écriture.
    virtual void set_angles(int board, int channel, const double* degs, int n) {
        for (int i = 0; i < n; i++) set_angle(board, channel + i, degs[i]);
    }
    // Relâche tous les servos (plus de consigne).
    virtual void relax() {}
};

// Durée écoulée en µs (micros() sur l'ESP32), pour la ligne M et PROF.
unsigned long platform_micros();
// Mémoire libre (octets), pour PROF ; -1 si inconnue.
long platform_free_memory();
