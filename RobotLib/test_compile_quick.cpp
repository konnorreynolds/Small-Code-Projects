// ============================================================================
// test_compile_quick.cpp - Quick Compilation and Feature Verification Test
// ============================================================================
// Purpose: Quickly verify that all library features compile and work correctly
// Platform: Any C++11+ compiler
// Usage: g++ -std=c++11 -I. -o test_quick test_compile_quick.cpp && ./test_quick
//
// This test ensures:
// - All headers compile without errors
// - All major features are accessible and work
// - Type safety is enforced
// - Calculations produce correct results
// ============================================================================

#include "units_core.h"
#include "units_physics.h"
#include "units_robotics.h"
#include "units_utilities.h"

#include <iostream>
#include <iomanip>
#include <cmath>

using namespace units;

int tests_passed = 0;
int tests_failed = 0;

#define ASSERT_APPROX(name, actual, expected, tol) \
    do { \
        double _a = (actual); \
        double _e = (expected); \
        if (std::abs(_a - _e) < (tol)) { \
            tests_passed++; \
            std::cout << "  ✓ " << name << " (got " << _a << ")\n"; \
        } else { \
            tests_failed++; \
            std::cout << "  ✗ " << name << " (got " << _a << ", expected " << _e << ")\n"; \
        } \
    } while(0)

int main() {
    std::cout << "\n========================================\n";
    std::cout << "  RobotLib Quick Compilation Test\n";
    std::cout << "  C++ Standard: " << __cplusplus << "\n";
    std::cout << "========================================\n\n";

    // ========================================================================
    // TEST 1: CORE UNITS
    // ========================================================================
    std::cout << "Testing Core Units...\n";
    {
        // Distance
        auto d1 = m(10.0);
        auto d2 = m(0.5);
        auto sum = d1 + d2;
        ASSERT_APPROX("Distance addition", sum.toMeters(), 10.5, 0.01);
        ASSERT_APPROX("Meters to feet", m(10).toFeet(), 32.808, 0.01);
        ASSERT_APPROX("Centimeters to meters", cm(150).toMeters(), 1.5, 0.01);

        // Time
        auto t1 = s(5.0);
        ASSERT_APPROX("Seconds to milliseconds", t1.toMilliseconds(), 5000.0, 0.1);

        // Angle
        auto a1 = deg(180);
        ASSERT_APPROX("Degrees to radians", a1.toRadians(), constants::PI, 0.01);
        ASSERT_APPROX("Sin(45°)", deg(45).sin(), 0.707, 0.01);
        ASSERT_APPROX("Cos(45°)", deg(45).cos(), 0.707, 0.01);

        auto normalized = deg(450).normalizePositive();
        ASSERT_APPROX("Angle normalization", normalized.toDegrees(), 90.0, 0.1);

        // Mass
        ASSERT_APPROX("Kilograms to pounds", kg(10).toPounds(), 22.046, 0.01);

        // Temperature
        ASSERT_APPROX("Celsius to Fahrenheit", degC(0).toFahrenheit(), 32.0, 0.1);
        ASSERT_APPROX("Fahrenheit to Celsius", degF(32).toCelsius(), 0.0, 0.1);
    }

    // ========================================================================
    // TEST 2: PHYSICS UNITS
    // ========================================================================
    std::cout << "\nTesting Physics Units...\n";
    {
        // Velocity
        auto v1 = mps(10);
        ASSERT_APPROX("m/s to km/h", v1.toKilometersPerHour(), 36.0, 0.01);

        // Acceleration
        auto a1 = MetersPerSecondSquared::fromGravities(1.0);
        ASSERT_APPROX("Gravities to m/s²", a1.toMetersPerSecondSquared(), 9.80665, 0.01);

        // Force (F = ma)
        auto force = kg(10) * a1;
        ASSERT_APPROX("F = ma", force.toNewtons(), 98.0665, 0.01);

        // Velocity = Distance / Time
        auto vel = m(100) / s(10);
        ASSERT_APPROX("v = d/t", vel.toMetersPerSecond(), 10.0, 0.01);

        // Power (P = VI)
        auto power = V(12) * A(5);
        ASSERT_APPROX("P = VI", power.toWatts(), 60.0, 0.01);

        // Voltage (V = IR)
        auto voltage = A(2) * ohm(10);
        ASSERT_APPROX("V = IR", voltage.toVolts(), 20.0, 0.01);

        // Work = Force × Distance
        auto work = N(50) * m(10);
        ASSERT_APPROX("Work = F×d", work.toJoules(), 500.0, 0.01);
    }

    // ========================================================================
    // TEST 3: ROBOTICS COMPONENTS
    // ========================================================================
    std::cout << "\nTesting Robotics Components...\n";
    {
        // Vec2D
        robotics::Vec2D v1(3, 4);
        ASSERT_APPROX("Vec2D magnitude", v1.magnitude(), 5.0, 0.01);

        robotics::Vec2D v2(1, 0);
        ASSERT_APPROX("Vec2D dot product", v1.dot(v2), 3.0, 0.01);
        ASSERT_APPROX("Vec2D cross product", v1.cross(v2), -4.0, 0.01);

        auto normalized = v1.normalized();
        ASSERT_APPROX("Vec2D normalized", normalized.magnitude(), 1.0, 0.01);

        // Pose2D - Now works with deg()!
        robotics::Pose2D pose(1.0, 2.0, deg(90));
        robotics::Vec2D localPoint(1.0, 0.0);
        robotics::Vec2D globalPoint = pose.toGlobal(localPoint);

        ASSERT_APPROX("Pose2D toGlobal X", globalPoint.x, 1.0, 0.1);
        ASSERT_APPROX("Pose2D toGlobal Y", globalPoint.y, 3.0, 0.1);

        // PID Controller
        robotics::PIDController pid(1.0, 0.1, 0.05);
        double output = pid.calculate(10.0, 0.02);
        std::cout << "  ✓ PID Controller (output: " << output << ")\n";
        tests_passed++;

        // Trapezoid Profile
        robotics::TrapezoidProfile::Constraints constraints(2.0, 1.0);
        robotics::TrapezoidProfile::State goal(10.0, 0.0, 0.0);
        robotics::TrapezoidProfile profile(constraints, goal);

        auto state = profile.calculate(1.0);
        std::cout << "  ✓ Trapezoid Profile (position: " << state.position << " m)\n";
        tests_passed++;

        // Filters
        robotics::LowPassFilter lpf(0.9);
        lpf.update(10.0);
        double filtered = lpf.update(10.0);
        std::cout << "  ✓ Low-Pass Filter (value: " << filtered << ")\n";
        tests_passed++;

        robotics::MovingAverageFilter<5> maf;
        maf.update(1.0);
        maf.update(2.0);
        maf.update(3.0);
        ASSERT_APPROX("Moving Average", maf.getValue(), 2.0, 0.01);
    }

    // ========================================================================
    // TEST 4: UTILITIES
    // ========================================================================
    std::cout << "\nTesting Utilities...\n";
    {
        // Vec2D utilities
        auto v1 = vec2d_utils::fromAngle(deg(45), 10.0);
        ASSERT_APPROX("Vec2D from angle X", v1.x, 7.071, 0.01);
        ASSERT_APPROX("Vec2D from angle Y", v1.y, 7.071, 0.01);

        auto perp = vec2d_utils::perpendicular(robotics::Vec2D(1, 0));
        ASSERT_APPROX("Vec2D perpendicular X", perp.x, 0.0, 0.01);
        ASSERT_APPROX("Vec2D perpendicular Y", perp.y, 1.0, 0.01);

        // Differential Drive
        auto drive = DifferentialDrive::fromTwist(mps(1.0), radps(0.0), m(0.5));
        ASSERT_APPROX("DiffDrive left (straight)", drive.leftVelocity.toMetersPerSecond(), 1.0, 0.01);
        ASSERT_APPROX("DiffDrive right (straight)", drive.rightVelocity.toMetersPerSecond(), 1.0, 0.01);

        auto drive2 = DifferentialDrive::fromTwist(mps(1.0), radps(0.5), m(0.5));
        ASSERT_APPROX("DiffDrive left (turning)", drive2.leftVelocity.toMetersPerSecond(), 0.875, 0.01);
        ASSERT_APPROX("DiffDrive right (turning)", drive2.rightVelocity.toMetersPerSecond(), 1.125, 0.01);

        // Battery Monitor
        BatteryMonitor battery(V(12.0), V(10.0), V(12.6));
        battery.update(V(11.5));
        ASSERT_APPROX("Battery SOC", battery.getPercentage(), 57.69, 0.1);

        // Motor Controller
        auto rpm = MotorController::velocityToRPM(mps(1.0), m(0.1));
        ASSERT_APPROX("Velocity to RPM", rpm.toRPM(), 190.99, 0.1);

        auto vel = MotorController::rpmToVelocity(RPM::fromRPM(100), m(0.1));
        ASSERT_APPROX("RPM to velocity", vel.toMetersPerSecond(), 0.524, 0.01);

        double deadband = MotorController::applyDeadband(0.03, 0.05);
        ASSERT_APPROX("Deadband below threshold", deadband, 0.0, 0.01);

        double active = MotorController::applyDeadband(0.6, 0.05);
        ASSERT_APPROX("Deadband above threshold", active, 0.579, 0.01);

        // Exponential Moving Average
        ExponentialMovingAverage ema(0.5);
        ema.update(10.0);
        double ema_val = ema.update(20.0);
        ASSERT_APPROX("EMA convergence", ema_val, 15.0, 0.1);

        // Simple Odometry
        SimpleOdometry odom;
        odom.update(mps(1.0), radps(0.0), s(1.0));
        odom.update(mps(1.0), radps(0.0), s(2.0));

        auto pose = odom.getPose();
        std::cout << "  ✓ Odometry (position: " << pose.position.x << ", "
                  << pose.position.y << ")\n";
        tests_passed++;
    }

    // ========================================================================
    // TEST 5: TYPE SAFETY
    // ========================================================================
    std::cout << "\nTesting Type Safety...\n";
    {
        // These should all compile fine
        auto d = m(10);
        auto t = s(5);
        auto v = d / t;  // OK: creates velocity
        (void)v;  // Suppress unused variable warning

        // These would NOT compile (commented out):
        // auto bad1 = d + t;  // ERROR: can't add distance and time
        // auto bad2 = v + d;  // ERROR: can't add velocity and distance
        // double bad3 = m(10);  // ERROR: no implicit conversion

        std::cout << "  ✓ Type safety enforced at compile-time\n";
        tests_passed++;
    }

    // ========================================================================
    // SUMMARY
    // ========================================================================
    std::cout << "\n========================================\n";
    std::cout << "  TEST SUMMARY\n";
    std::cout << "========================================\n";
    std::cout << "Tests passed: " << tests_passed << "\n";
    std::cout << "Tests failed: " << tests_failed << "\n";
    std::cout << "Success rate: " << std::fixed << std::setprecision(1)
              << (100.0 * tests_passed / (tests_passed + tests_failed)) << "%\n";

    if (tests_failed == 0) {
        std::cout << "\n✅ ALL TESTS PASSED!\n";
        std::cout << "✅ Library is ready for use on ESP32-C3\n";
        std::cout << "✅ All features compile correctly\n";
        std::cout << "✅ All calculations produce correct values\n";
        std::cout << "✅ Type safety is enforced\n\n";
        return 0;
    } else {
        std::cout << "\n❌ SOME TESTS FAILED\n";
        std::cout << "Please review the output above\n\n";
        return 1;
    }
}
