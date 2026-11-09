#!/bin/sh
# Compile (sans lier) tout src/ avec les macros d'Arduino-ESP32 imposées :
# attrape sur PC les collisions de noms (HALF_PI, sq, LINE_MAX...) avant
# une compilation ESP32. ARDUINO est défini : main.cpp, pca9685.cpp et
# wire_bus.cpp sont vérifiés aussi. Usage : sh firmware_c/test/check_arduino.sh
cd "$(dirname "$0")/.." || exit 1
# newlib xtensa : sys/types.h fait "#define quad quad_t" (et physadr), mais
# pas dans toutes les unités de compilation : interdire ces noms dans src/.
# De même : types BSD de sys/types.h (uint, ushort, ulong, u_char...) et
# constantes binaires d'Arduino (binary.h : B0, B1, B00 ... B11111111).
if grep -nwE 'quad|physadr|uint|ushort|ulong|u_char|u_short|u_int|u_long|B[01]{1,8}' src/*.cpp src/*.h; then
    echo "ERREUR : nom réservé par newlib ou Arduino ci-dessus"; exit 1
fi
for f in src/*.cpp; do
    g++ -std=gnu++11 -fsyntax-only -Wall -DARDUINO=10819 -DESP32 -Itest/arduino_stub -Isrc -include Arduino.h "$f" || exit 1
done
echo "OK : src/ compile avec les macros Arduino"
