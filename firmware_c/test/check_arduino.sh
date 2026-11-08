#!/bin/sh
# Compile (sans lier) tout src/ avec les macros d'Arduino-ESP32 imposées :
# attrape sur PC les collisions de noms (HALF_PI, sq, LINE_MAX...) avant
# une compilation ESP32. ARDUINO est défini : main.cpp, pca9685.cpp et
# wire_bus.cpp sont vérifiés aussi. Usage : sh firmware_c/test/check_arduino.sh
cd "$(dirname "$0")/.." || exit 1
for f in src/*.cpp; do
    g++ -std=gnu++11 -fsyntax-only -Wall -DARDUINO=10819 -DESP32 -Itest/arduino_stub -Isrc -include Arduino.h "$f" || exit 1
done
echo "OK : src/ compile avec les macros Arduino"
