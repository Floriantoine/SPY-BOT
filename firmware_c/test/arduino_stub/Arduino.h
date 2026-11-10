// Faux Arduino.h pour vérifier sur PC que le code compile malgré les macros
// d'Arduino-ESP32 2.x (copiées ici : PI, HALF_PI, sq(), degrees()...).
// Ne sert qu'à test/check_arduino.sh, jamais à la compilation ESP32.
#pragma once
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <limits.h>
/* newlib xtensa (sys/types.h) : macros BSD visibles sous Arduino-ESP32 */
#define quad quad_t
#define physadr physadr_t
#ifndef LINE_MAX
#define LINE_MAX 2048  // newlib/xtensa : limits.h via FreeRTOS
#endif
#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352
#define SERIAL 0x0
#define DISPLAY 0x1
#define LSBFIRST 0
#define MSBFIRST 1
#define LOW 0x0
#define HIGH 0x1
#define INPUT 0x01
#define OUTPUT 0x03
#define PULLUP 0x04
#define INPUT_PULLUP 0x05
#define PULLDOWN 0x08
#define INPUT_PULLDOWN 0x09
#define OPEN_DRAIN 0x10
#define OUTPUT_OPEN_DRAIN 0x13
#define ANALOG 0xC0
#define DISABLED 0x00
#define RISING 0x01
#define FALLING 0x02
#define CHANGE 0x03
#define ONLOW 0x04
#define ONHIGH 0x05
#define ONLOW_WE 0x0C
#define ONHIGH_WE 0x0D
#define DEFAULT 1
#define EXTERNAL 0
#define _min(a, b) ((a) < (b) ? (a) : (b))
#define _max(a, b) ((a) > (b) ? (a) : (b))
#define _abs(x) ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define _round(x) ((x) >= 0 ? (long)((x) + 0.5) : (long)((x) - 0.5))
#define radians(deg) ((deg) * DEG_TO_RAD)
#define degrees(rad) ((rad) * RAD_TO_DEG)
#define sq(x) ((x) * (x))
#define sei()
#define cli()
#define interrupts() sei()
#define noInterrupts() cli()
#define lowByte(w) ((uint8_t)((w) & 0xff))
#define highByte(w) ((uint8_t)((w) >> 8))
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitToggle(value, bit) ((value) ^= (1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
#define bit(b) (1UL << (b))
#define _BV(b) (1UL << (b))
#define F(s) (s)
#define B0 0
#define B1 1
#define B10 2
#define B11 3
#define B100 4
#define B101 5
#define B110 6
#define B111 7
#define NOT_A_PIN -1
#define NOT_A_PORT -1
#define NOT_AN_INTERRUPT -1
#define NOT_ON_TIMER 0
#define word(...) makeWord(__VA_ARGS__)
#define NOP() asm volatile("nop")
/* newlib : ctype.h, sys/param.h ; gcc xtensa en mode gnu++ ; ESP-IDF */
#define _U 01
#define _L 02
#define _N 04
#define _S 010
#define _P 020
#define _C 040
#define _X 0100
#define _B 0200
#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif
#define HZ (60)
#define NBBY 8
#define howmany(x, y) (((x) + ((y) - 1)) / (y))
#define xtensa 1
#define BIT(nr) (1UL << (nr))
/* variants/esp32/pins_arduino.h : constantes globales (pas des macros) */
static const uint8_t TX = 1, RX = 3, SDA = 21, SCL = 22, SS = 5, MOSI = 23, MISO = 19, SCK = 18;
static const uint8_t A0 = 36, A3 = 39, A4 = 32, A5 = 33, A6 = 34, A7 = 35, A10 = 4, A11 = 0, A12 = 2, A13 = 15;
static const uint8_t A14 = 13, A15 = 12, A16 = 14, A17 = 27, A18 = 25, A19 = 26;
static const uint8_t T0 = 4, T1 = 0, T2 = 2, T3 = 15, T4 = 13, T5 = 12, T6 = 14, T7 = 27, T8 = 33, T9 = 32;
static const uint8_t DAC1 = 25, DAC2 = 26;
typedef bool boolean;
typedef uint8_t byte;
typedef unsigned int word;
unsigned long millis();
unsigned long micros();
void delay(uint32_t);
void delayMicroseconds(uint32_t);
bool setCpuFrequencyMhz(uint32_t);
class HardwareSerial {
public:
    void begin(unsigned long) {}
    size_t setRxBufferSize(size_t n) { return n; }
    size_t setTxBufferSize(size_t n) { return n; }
    int available() { return 0; }
    int read() { return -1; }
    size_t print(const char*) { return 0; }
    size_t println(const char*) { return 0; }
    size_t println() { return 0; }
    size_t write(const uint8_t*, size_t n) { return n; }
};
extern HardwareSerial Serial;
class EspClass {
public:
    uint32_t getFreeHeap() { return 0; }
};
extern EspClass ESP;
