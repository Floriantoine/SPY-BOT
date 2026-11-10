#ifdef ARDUINO
#include "wire_bus.h"

#include <Arduino.h>
#include <Wire.h>

// Mêmes transactions que machine.I2C de MicroPython :
//   writeto      START adr+W données STOP
//   readfrom     START adr+R n octets STOP
//   writeto_mem  START adr+W reg données STOP
//   readfrom_mem START adr+W reg RESTART adr+R n octets STOP

bool WireBus::scan_has(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

bool WireBus::writeto(uint8_t addr, const uint8_t* data, size_t n) {
    Wire.beginTransmission(addr);
    if (Wire.write(data, n) != n) {
        Wire.endTransmission();
        return false;
    }
    return Wire.endTransmission() == 0;
}

static bool read_bytes(uint8_t addr, uint8_t* data, size_t n) {
    if (Wire.requestFrom(addr, (uint8_t)n) != (uint8_t)n) return false;
    for (size_t i = 0; i < n; i++) {
        int c = Wire.read();
        if (c < 0) return false;
        data[i] = (uint8_t)c;
    }
    return true;
}

bool WireBus::readfrom(uint8_t addr, uint8_t* data, size_t n) { return read_bytes(addr, data, n); }

bool WireBus::writeto_mem(uint8_t addr, uint8_t reg, const uint8_t* data, size_t n) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.write(data, n) != n) {
        Wire.endTransmission();
        return false;
    }
    return Wire.endTransmission() == 0;
}

bool WireBus::readfrom_mem(uint8_t addr, uint8_t reg, uint8_t* data, size_t n) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;  // RESTART, comme MicroPython
    return read_bytes(addr, data, n);
}
#endif
