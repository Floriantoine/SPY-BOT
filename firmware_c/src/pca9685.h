// Deux PCA9685 sur le bus I²C de l'ESP32 (portage de hexapod/servo.py,
// PCA9685Driver). Seulement pour l'ESP32 (Arduino).
#pragma once
#ifdef ARDUINO
#include <stdint.h>

#include "driver.h"
#include "robot_config.h"

class PCA9685Driver : public ServoDriver {
public:
    PCA9685Driver();
    // Démarre le bus (SDA 21, SCL 22, 400 kHz) et les cartes présentes.
    void begin();
    bool present(int board) const { return present_[board]; }
    unsigned long errors;  // écritures I²C perdues

    void set_angle(int board, int channel, double deg);
    bool has_batch() const { return true; }
    void set_angles(int board, int channel, const double* degs, int n);
    void relax();

private:
    bool present_[CFG_NBOARDS];
    // Dernière valeur écrite par voie (écriture évitée si inchangée).
    uint16_t last_[CFG_NBOARDS][16];
    bool valid_[CFG_NBOARDS][16];

    bool write_reg(int board, uint8_t reg, uint8_t value);
    bool read_reg(int board, uint8_t reg, uint8_t* value);
    bool init_board(int board);
    void write_offs(int board, int channel, const uint16_t* offs, int n);
};
#endif
