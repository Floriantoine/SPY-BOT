#ifdef ARDUINO
#include "pca9685.h"

#include <Arduino.h>
#include <Wire.h>

#include "robot.h"

static const uint8_t MODE1 = 0x00;
static const uint8_t PRESCALE = 0xFE;
static const uint8_t LED0_ON_L = 0x06;

PCA9685Driver::PCA9685Driver() : errors(0) {
    for (int b = 0; b < CFG_NBOARDS; b++) {
        present_[b] = false;
        for (int c = 0; c < 16; c++) valid_[b][c] = false;
    }
}

void PCA9685Driver::begin() {
    // 400 kHz : 4 fois moins de temps pour écrire les 18 servos.
    Wire.begin((int)SERVO_SDA, (int)SERVO_SCL, 400000);
    for (int b = 0; b < CFG_NBOARDS; b++) present_[b] = init_board(b);
}

bool PCA9685Driver::write_reg(int board, uint8_t reg, uint8_t value) {
    Wire.beginTransmission((uint8_t)CFG_I2C_ADDRESSES[board]);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool PCA9685Driver::read_reg(int board, uint8_t reg, uint8_t* value) {
    uint8_t addr = (uint8_t)CFG_I2C_ADDRESSES[board];
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;
    *value = (uint8_t)Wire.read();
    return true;
}

bool PCA9685Driver::init_board(int board) {
    // Comme PCA9685.__init__ et set_freq : 50 Hz, puis auto-incrément.
    if (!write_reg(board, MODE1, 0x00)) return false;
    int prescale = (int)(25000000.0 / 4096.0 / PCA_FREQ + 0.5) - 1;
    uint8_t old;
    if (!read_reg(board, MODE1, &old)) return false;
    if (!write_reg(board, MODE1, (uint8_t)((old & 0x7F) | 0x10))) return false;  // veille
    if (!write_reg(board, PRESCALE, (uint8_t)prescale)) return false;
    if (!write_reg(board, MODE1, old)) return false;
    delayMicroseconds(500);
    return write_reg(board, MODE1, (uint8_t)(old | 0xA1));  // redémarrage + auto-incrément
}

void PCA9685Driver::write_offs(int board, int channel, const uint16_t* offs, int n) {
    bool same = true;
    for (int i = 0; i < n && same; i++) same = valid_[board][channel + i] && last_[board][channel + i] == offs[i];
    if (same) return;  // inchangé : moins de trafic I²C, moins de risque de parasite
    uint8_t data[4 * 16];
    for (int i = 0; i < n; i++) {
        valid_[board][channel + i] = false;  // si l'écriture échoue, on réessaiera
        data[4 * i] = 0;
        data[4 * i + 1] = 0;
        data[4 * i + 2] = (uint8_t)(offs[i] & 0xFF);
        data[4 * i + 3] = (uint8_t)(offs[i] >> 8);
    }
    unsigned long t0 = g_prof.on ? platform_micros() : 0;
    Wire.beginTransmission((uint8_t)CFG_I2C_ADDRESSES[board]);
    Wire.write((uint8_t)(LED0_ON_L + 4 * channel));
    Wire.write(data, (size_t)(4 * n));
    bool ok = Wire.endTransmission() == 0;
    if (g_prof.on) g_prof.add(PROF_I2C, t0);
    if (!ok) {
        // Écriture perdue (parasites des servos) : la consigne suivante la remplace.
        errors++;
        return;
    }
    for (int i = 0; i < n; i++) {
        last_[board][channel + i] = offs[i];
        valid_[board][channel + i] = true;
    }
}

void PCA9685Driver::set_angle(int board, int channel, double deg) {
    if (!present_[board]) return;
    double us = SERVO_CENTER_US + (deg - 90.0) * SERVO_US_PER_DEG;
    us = us < SERVO_MIN_US ? SERVO_MIN_US : us > SERVO_MAX_US ? SERVO_MAX_US : us;
    uint16_t off = (uint16_t)(int)(us * PCA_FREQ * 4096 / 1000000);
    write_offs(board, channel, &off, 1);
}

void PCA9685Driver::set_angles(int board, int channel, const double* degs, int n) {
    if (!present_[board] || n <= 0 || channel + n > 16) return;
    // Angle -> valeur du registre en une multiplication (comme PCA9685Driver.set_angles).
    double k = PCA_FREQ * 4096 / 1000000;
    double lo = SERVO_MIN_US * k, hi = SERVO_MAX_US * k;
    double a = (SERVO_CENTER_US - 90.0 * SERVO_US_PER_DEG) * k;
    double m = SERVO_US_PER_DEG * k;
    uint16_t offs[16];
    for (int i = 0; i < n; i++) {
        double o = a + degs[i] * m;
        offs[i] = (uint16_t)(int)(o < lo ? lo : o > hi ? hi : o);
    }
    write_offs(board, channel, offs, n);
}

void PCA9685Driver::relax() {
    // Bit « full off » : les servos ne reçoivent plus de consigne.
    for (int b = 0; b < CFG_NBOARDS; b++) {
        if (!present_[b]) continue;
        uint8_t data[4 * 16];
        for (int c = 0; c < 16; c++) {
            valid_[b][c] = false;
            data[4 * c] = 0;
            data[4 * c + 1] = 0;
            data[4 * c + 2] = 0;
            data[4 * c + 3] = 0x10;
        }
        Wire.beginTransmission((uint8_t)CFG_I2C_ADDRESSES[b]);
        Wire.write(LED0_ON_L);
        Wire.write(data, sizeof(data));
        Wire.endTransmission();
    }
}
#endif
