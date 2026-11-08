// Bus I²C partagé par le gyroscope et le capteur avant : Wire sur l'ESP32
// (wire_bus.cpp), faux bus scripté pour les tests sur PC. Mêmes opérations
// que machine.I2C de MicroPython ; faux = l'OSError de MicroPython.
#pragma once
#include <stddef.h>
#include <stdint.h>

class I2CBus {
public:
    virtual ~I2CBus() {}
    // Un appareil répond à cette adresse (« addr in i2c.scan() »).
    virtual bool scan_has(uint8_t addr) = 0;
    virtual bool writeto(uint8_t addr, const uint8_t* data, size_t n) = 0;
    virtual bool readfrom(uint8_t addr, uint8_t* data, size_t n) = 0;
    virtual bool writeto_mem(uint8_t addr, uint8_t reg, const uint8_t* data, size_t n) = 0;
    virtual bool readfrom_mem(uint8_t addr, uint8_t reg, uint8_t* data, size_t n) = 0;
};
