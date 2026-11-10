// Bus I²C de l'ESP32 (Wire, partagé avec les PCA9685) pour le gyroscope et
// le capteur avant. Seulement pour l'ESP32 (Arduino).
#pragma once
#ifdef ARDUINO
#include "i2c_bus.h"

class WireBus : public I2CBus {
public:
    bool scan_has(uint8_t addr);
    bool writeto(uint8_t addr, const uint8_t* data, size_t n);
    bool readfrom(uint8_t addr, uint8_t* data, size_t n);
    bool writeto_mem(uint8_t addr, uint8_t reg, const uint8_t* data, size_t n);
    bool readfrom_mem(uint8_t addr, uint8_t reg, uint8_t* data, size_t n);
};
#endif
