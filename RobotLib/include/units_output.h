// ============================================================================
// units_output.h - Clean Output Utilities for RobotLib
// ============================================================================
// Purpose: Provide simple, platform-agnostic logging and output functions
//          to keep example code clean and readable
//
// Key features:
// - Automatic platform detection (Arduino vs. standard C++)
// - Clean API that hides std::cout complexity
// - Zero overhead when disabled
// - Type-safe unit printing
// ============================================================================

#ifndef ROBOTICS_UNITS_OUTPUT_H
#define ROBOTICS_UNITS_OUTPUT_H

#include "units_core.h"

// Detect platform
#if defined(ARDUINO)
    #define ROBOTLIB_ARDUINO
    #include <Arduino.h>
#endif

namespace robotlib {
namespace output {

// ============================================================================
// Platform-agnostic output backend
// ============================================================================
namespace detail {
    #ifdef ROBOTLIB_ARDUINO
        // Arduino: use Serial
        inline void print_impl(const char* str) { Serial.print(str); }
        inline void print_impl(double val) { Serial.print(val, 6); }
        inline void print_impl(int val) { Serial.print(val); }
        inline void print_impl(long val) { Serial.print(val); }
        inline void print_impl(unsigned long val) { Serial.print(val); }
        inline void println_impl() { Serial.println(); }
    #else
        // Standard C++: use std::cout
        #include <iostream>
        #include <iomanip>
        inline void print_impl(const char* str) { std::cout << str; }
        inline void print_impl(double val) { std::cout << std::fixed << std::setprecision(6) << val; }
        inline void print_impl(int val) { std::cout << val; }
        inline void print_impl(long val) { std::cout << val; }
        inline void print_impl(unsigned long val) { std::cout << val; }
        inline void println_impl() { std::cout << std::endl; }
    #endif
}

// ============================================================================
// Simple logging functions - clean, easy API
// ============================================================================

// Print a single value
template<typename T>
inline void print(T value) {
    detail::print_impl(value);
}

// Print with newline
template<typename T>
inline void println(T value) {
    detail::print_impl(value);
    detail::println_impl();
}

// Print newline only
inline void println() {
    detail::println_impl();
}

// Print multiple values on same line
template<typename T1, typename T2>
inline void print(T1 v1, T2 v2) {
    print(v1); print(v2);
}

template<typename T1, typename T2, typename T3>
inline void print(T1 v1, T2 v2, T3 v3) {
    print(v1); print(v2); print(v3);
}

template<typename T1, typename T2, typename T3, typename T4>
inline void print(T1 v1, T2 v2, T3 v3, T4 v4) {
    print(v1); print(v2); print(v3); print(v4);
}

template<typename T1, typename T2, typename T3, typename T4, typename T5>
inline void print(T1 v1, T2 v2, T3 v3, T4 v4, T5 v5) {
    print(v1); print(v2); print(v3); print(v4); print(v5);
}

// Print multiple values with newline
template<typename T1, typename T2>
inline void println(T1 v1, T2 v2) {
    print(v1, v2); println();
}

template<typename T1, typename T2, typename T3>
inline void println(T1 v1, T2 v2, T3 v3) {
    print(v1, v2, v3); println();
}

template<typename T1, typename T2, typename T3, typename T4>
inline void println(T1 v1, T2 v2, T3 v3, T4 v4) {
    print(v1, v2, v3, v4); println();
}

template<typename T1, typename T2, typename T3, typename T4, typename T5>
inline void println(T1 v1, T2 v2, T3 v3, T4 v4, T5 v5) {
    print(v1, v2, v3, v4, v5); println();
}

// ============================================================================
// Unit-aware printing (automatically includes units)
// ============================================================================

// Print Distance with units
template<typename R>
inline void printUnit(const units::Distance<R>& d) {
    print(d.toMeters());
    print(" m");
}

// Print Time with units
template<typename R>
inline void printUnit(const units::Time<R>& t) {
    print(t.toSeconds());
    print(" s");
}

// Print Velocity with units
template<typename DR, typename TR>
inline void printUnit(const units::Velocity<DR, TR>& v) {
    print(v.toMetersPerSecond());
    print(" m/s");
}

// Print Acceleration with units
template<typename DR, typename TR>
inline void printUnit(const units::Acceleration<DR, TR>& a) {
    print(a.toMetersPerSecondSquared());
    print(" m/s²");
}

// Print Force with units
template<typename R>
inline void printUnit(const units::Force<R>& f) {
    print(f.toNewtons());
    print(" N");
}

// Print Mass with units
template<typename R>
inline void printUnit(const units::Mass<R>& m) {
    print(m.toKilograms());
    print(" kg");
}

// Print Angle with units
template<typename R>
inline void printUnit(const units::Angle<R>& a) {
    print(a.toRadians());
    print(" rad (");
    print(a.toDegrees());
    print("°)");
}

// Print Angular Velocity with units
template<typename R>
inline void printUnit(const units::AngularVelocity<R>& w) {
    print(w.toRadiansPerSecond());
    print(" rad/s (");
    print(w.toRPM());
    print(" RPM)");
}

// Print Power with units
template<typename R>
inline void printUnit(const units::Power<R>& p) {
    print(p.toWatts());
    print(" W");
}

// Print Energy with units
template<typename R>
inline void printUnit(const units::Energy<R>& e) {
    print(e.toJoules());
    print(" J");
}

// Print Voltage with units
template<typename R>
inline void printUnit(const units::Voltage<R>& v) {
    print(v.toVolts());
    print(" V");
}

// Print Current with units
template<typename R>
inline void printUnit(const units::Current<R>& i) {
    print(i.toAmperes());
    print(" A");
}

// Print Resistance with units
template<typename R>
inline void printUnit(const units::Resistance<R>& r) {
    print(r.toOhms());
    print(" Ω");
}

// Print unit value with newline
template<typename T>
inline void printlnUnit(const T& value) {
    printUnit(value);
    println();
}

// ============================================================================
// Formatted logging with labels
// ============================================================================

// Print labeled value
template<typename T>
inline void log(const char* label, T value) {
    print(label);
    print(": ");
    println(value);
}

// Print labeled unit value
template<typename T>
inline void logUnit(const char* label, const T& value) {
    print(label);
    print(": ");
    printlnUnit(value);
}

// ============================================================================
// Initialization
// ============================================================================

// Initialize output system (must call this in setup() for Arduino)
inline void begin(long baudRate = 115200) {
    #ifdef ROBOTLIB_ARDUINO
        Serial.begin(baudRate);
        // Wait for serial to be ready
        while (!Serial) { /* wait */ }
    #endif
}

// ============================================================================
// Debug macros (can be disabled for production)
// ============================================================================

#ifndef ROBOTLIB_DEBUG
    #define ROBOTLIB_DEBUG 1  // Enable debug output by default
#endif

#if ROBOTLIB_DEBUG
    #define DEBUG_PRINT(...)   robotlib::output::print(__VA_ARGS__)
    #define DEBUG_PRINTLN(...) robotlib::output::println(__VA_ARGS__)
    #define DEBUG_LOG(label, value) robotlib::output::log(label, value)
    #define DEBUG_LOG_UNIT(label, value) robotlib::output::logUnit(label, value)
#else
    // Debug disabled - all debug output becomes no-ops
    #define DEBUG_PRINT(...)   ((void)0)
    #define DEBUG_PRINTLN(...) ((void)0)
    #define DEBUG_LOG(label, value) ((void)0)
    #define DEBUG_LOG_UNIT(label, value) ((void)0)
#endif

} // namespace output
} // namespace robotlib

#endif // ROBOTICS_UNITS_OUTPUT_H
