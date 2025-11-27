#pragma once
#include <iostream>
#include <string>
#include <cmath>
#include <stdint.h>
#include <algorithm>
#include <cstdarg>  // <--- THIS WAS MISSING! Needed for va_list, va_start
#include <cstdio>   // Needed for vsnprintf

// --- Types ---
typedef std::string String;
typedef bool boolean;
typedef uint8_t byte;

// --- Constants ---
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define FILE_WRITE "w"
#define FILE_READ "r"
#define HSPI 1

// --- Mock Serial ---
class MockSerial {
public:
    void begin(long baud) {}
    void print(const char* str) { std::cout << str; }
    void print(double val) { std::cout << val; }
    void print(int val) { std::cout << val; }
    void println(const char* str = "") { std::cout << str << std::endl; }
    void println(double val) { std::cout << val << std::endl; }
    void println(int val) { std::cout << val << std::endl; }
    
    void printf(const char* format, ...) {
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, 256, format, args);
        va_end(args);
        std::cout << buffer;
    }
};
extern MockSerial Serial;

// --- Mock Time ---
extern unsigned long _mockMillis;
inline unsigned long millis() { return _mockMillis; }
inline void delay(unsigned long ms) { _mockMillis += ms; }

// --- Mock GPIO ---
extern int _pinStates[100]; 
inline void pinMode(int pin, int mode) {}
inline void digitalWrite(int pin, int val) { _pinStates[pin] = val; }
inline int digitalRead(int pin) { return _pinStates[pin]; }
inline int analogRead(int pin) { return 0; }

// --- Math Helpers ---
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define abs(x) ((x)>0?(x):-(x))
#define min(a,b) std::min(a,b)
#define max(a,b) std::max(a,b)
#define pow(a,b) std::pow(a,b)