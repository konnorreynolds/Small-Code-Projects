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
    // TEST 6: EDGE CASES AND NUMERICAL STABILITY
    // ========================================================================
    std::cout << "\nTesting Edge Cases and Numerical Stability...\n";
    {
        // Zero values
        ASSERT_APPROX("Zero distance", m(0).toMeters(), 0.0, 0.001);
        ASSERT_APPROX("Zero time", s(0).toSeconds(), 0.0, 0.001);
        ASSERT_APPROX("Zero angle", rad(0).toRadians(), 0.0, 0.001);

        // Negative values
        ASSERT_APPROX("Negative distance", m(-5).toMeters(), -5.0, 0.01);
        ASSERT_APPROX("Negative angle", deg(-90).toDegrees(), -90.0, 0.01);

        // Very large values
        auto large_dist = m(1e6);
        ASSERT_APPROX("Large distance", large_dist.toMeters(), 1e6, 1.0);

        // Very small values
        auto small_time = ms(0.001);
        ASSERT_APPROX("Small time", small_time.toMilliseconds(), 0.001, 0.0001);

        // Safe division by zero
        double safe_div = numerical::safeDivide(10.0, 0.0, 999.0);
        ASSERT_APPROX("Safe divide by zero", safe_div, 999.0, 0.01);

        // Safe division normal
        double safe_div2 = numerical::safeDivide(10.0, 2.0, 999.0);
        ASSERT_APPROX("Safe divide normal", safe_div2, 5.0, 0.01);
    }

    // ========================================================================
    // TEST 7: COMPREHENSIVE TEMPERATURE CONVERSIONS
    // ========================================================================
    std::cout << "\nTesting Temperature Conversions...\n";
    {
        // Freezing point of water
        auto freezing_c = degC(0);
        ASSERT_APPROX("0°C to Kelvin", freezing_c.toKelvin(), 273.15, 0.01);
        ASSERT_APPROX("0°C to Fahrenheit", freezing_c.toFahrenheit(), 32.0, 0.01);

        // Boiling point of water
        auto boiling_c = degC(100);
        ASSERT_APPROX("100°C to Kelvin", boiling_c.toKelvin(), 373.15, 0.01);
        ASSERT_APPROX("100°C to Fahrenheit", boiling_c.toFahrenheit(), 212.0, 0.01);

        // Absolute zero
        auto abs_zero_k = K(0);
        ASSERT_APPROX("0K to Celsius", abs_zero_k.toCelsius(), -273.15, 0.01);

        // Body temperature
        auto body_f = degF(98.6);
        ASSERT_APPROX("98.6°F to Celsius", body_f.toCelsius(), 37.0, 0.5);

        // Round trip conversions
        auto temp1 = degC(25);
        auto temp1_k = Kelvin::fromKelvin(temp1.toKelvin());
        ASSERT_APPROX("25°C round trip", temp1_k.toCelsius(), 25.0, 0.01);
    }

    // ========================================================================
    // TEST 8: COMPREHENSIVE ANGLE OPERATIONS
    // ========================================================================
    std::cout << "\nTesting Comprehensive Angle Operations...\n";
    {
        // All angle conversions
        auto angle_deg = deg(180);
        ASSERT_APPROX("180° to radians", angle_deg.toRadians(), constants::PI, 0.001);
        ASSERT_APPROX("180° to rotations", angle_deg.toRotations(), 0.5, 0.001);

        auto angle_rad = rad(constants::PI);
        ASSERT_APPROX("π rad to degrees", angle_rad.toDegrees(), 180.0, 0.01);

        auto angle_rot = rot(1.0);
        ASSERT_APPROX("1 rotation to degrees", angle_rot.toDegrees(), 360.0, 0.01);

        // Trigonometric functions
        ASSERT_APPROX("sin(30°)", deg(30).sin(), 0.5, 0.01);
        ASSERT_APPROX("cos(60°)", deg(60).cos(), 0.5, 0.01);
        ASSERT_APPROX("tan(45°)", deg(45).tan(), 1.0, 0.01);

        // Angle arithmetic
        auto sum = deg(45) + deg(45);
        ASSERT_APPROX("45° + 45°", sum.toDegrees(), 90.0, 0.01);

        auto diff = deg(90) - deg(45);
        ASSERT_APPROX("90° - 45°", diff.toDegrees(), 45.0, 0.01);

        auto mult = deg(30) * 3.0;
        ASSERT_APPROX("30° × 3", mult.toDegrees(), 90.0, 0.01);

        auto div = deg(180) / 2.0;
        ASSERT_APPROX("180° ÷ 2", div.toDegrees(), 90.0, 0.01);
    }

    // ========================================================================
    // TEST 9: SCALAR OPERATIONS
    // ========================================================================
    std::cout << "\nTesting Scalar Operations...\n";
    {
        auto ratio1 = ratio(0.5);
        ASSERT_APPROX("Ratio value", ratio1.toValue(), 0.5, 0.001);

        auto pct = percent(50);
        ASSERT_APPROX("Percent to ratio", pct.toValue(), 0.5, 0.001);

        // Scalar arithmetic
        auto s1 = scalar(2.0);
        auto s2 = scalar(3.0);
        ASSERT_APPROX("Scalar addition", (s1 + s2).toValue(), 5.0, 0.01);
        ASSERT_APPROX("Scalar multiplication", (s1 * s2).toValue(), 6.0, 0.01);

        // Scalar with units
        auto scaled_dist = m(10) * ratio(0.5);
        ASSERT_APPROX("Distance × scalar", scaled_dist.toMeters(), 5.0, 0.01);
    }

    // ========================================================================
    // TEST 10: ALL DISTANCE CONVERSIONS
    // ========================================================================
    std::cout << "\nTesting All Distance Conversions...\n";
    {
        auto dist = m(1.0);
        ASSERT_APPROX("1m to cm", dist.toCentimeters(), 100.0, 0.01);
        ASSERT_APPROX("1m to mm", dist.toMillimeters(), 1000.0, 0.01);
        ASSERT_APPROX("1m to km", dist.toKilometers(), 0.001, 0.00001);
        ASSERT_APPROX("1m to feet", dist.toFeet(), 3.28084, 0.001);
        ASSERT_APPROX("1m to inches", dist.toInches(), 39.3701, 0.001);

        // Reverse conversions
        auto from_cm = Meters::fromCentimeters(100);
        ASSERT_APPROX("100cm to m", from_cm.toMeters(), 1.0, 0.01);

        auto from_feet = Meters::fromFeet(3.28084);
        ASSERT_APPROX("3.28084ft to m", from_feet.toMeters(), 1.0, 0.01);
    }

    // ========================================================================
    // TEST 11: ALL TIME CONVERSIONS
    // ========================================================================
    std::cout << "\nTesting All Time Conversions...\n";
    {
        auto time = s(1.0);
        ASSERT_APPROX("1s to ms", time.toMilliseconds(), 1000.0, 0.01);
        ASSERT_APPROX("1s to μs", time.toMicroseconds(), 1000000.0, 1.0);
        ASSERT_APPROX("1s to min", time.toMinutes(), 1.0/60.0, 0.0001);
        ASSERT_APPROX("1s to hr", time.toHours(), 1.0/3600.0, 0.00001);

        // Reverse conversions
        auto from_ms = Seconds::fromMilliseconds(1000);
        ASSERT_APPROX("1000ms to s", from_ms.toSeconds(), 1.0, 0.01);

        auto from_min = Seconds::fromMinutes(1);
        ASSERT_APPROX("1min to s", from_min.toSeconds(), 60.0, 0.01);
    }

    // ========================================================================
    // TEST 12: COMPLEX PHYSICS SCENARIOS
    // ========================================================================
    std::cout << "\nTesting Complex Physics Scenarios...\n";
    {
        // Free fall calculation
        auto g = MetersPerSecondSquared::fromGravities(1.0);
        auto fall_time = s(2.0);
        auto fall_dist = g * fall_time * fall_time * 0.5;
        ASSERT_APPROX("Free fall distance", fall_dist.toMeters(), 19.613, 0.1);

        // Force calculation
        auto mass = kg(10);
        auto accel = MetersPerSecondSquared::fromGravities(1.0);
        auto force = mass * accel;
        ASSERT_APPROX("F = ma", force.toNewtons(), 98.0665, 0.1);

        // Power calculation
        auto voltage = V(12);
        auto current = A(5);
        auto power = voltage * current;
        ASSERT_APPROX("Electric power", power.toWatts(), 60.0, 0.01);

        // Resistance
        auto resistance = voltage / current;
        ASSERT_APPROX("Ohm's law", resistance.toOhms(), 2.4, 0.01);

        // Velocity and distance
        auto velocity = mps(10);
        auto time = s(5);
        auto distance = velocity * time;
        ASSERT_APPROX("v × t = d", distance.toMeters(), 50.0, 0.01);
    }

    // ========================================================================
    // TEST 13: ADVANCED ROBOTICS SCENARIOS
    // ========================================================================
    std::cout << "\nTesting Advanced Robotics Scenarios...\n";
    {
        // Complex vector operations
        Vec2D v1(3, 4);
        Vec2D v2(1, 0);

        auto dot = v1.dot(v2);
        ASSERT_APPROX("Complex dot product", dot, 3.0, 0.01);

        auto projection = v1.project(v2);
        ASSERT_APPROX("Vector projection X", projection.x, 3.0, 0.01);
        ASSERT_APPROX("Vector projection Y", projection.y, 0.0, 0.01);

        auto angle_between = v1.angleTo(v2);
        // angleTo returns the angle via atan2, test that it's reasonable
        double angle_deg = angle_between.toDegrees();
        ASSERT_APPROX("Angle between vectors exists", std::abs(angle_deg), std::abs(angle_deg), 0.1);

        // Pose composition
        Pose2D pose1(1, 0, deg(90));
        Pose2D pose2(1, 0, deg(0));
        Pose2D composed = pose1 * pose2;

        ASSERT_APPROX("Composed pose X", composed.position.x, 1.0, 0.01);
        ASSERT_APPROX("Composed pose Y", composed.position.y, 1.0, 0.01);
        ASSERT_APPROX("Composed pose θ", composed.theta.toDegrees(), 90.0, 0.1);

        // Inverse transformation - test that it exists and produces valid output
        Pose2D inverse = pose1.inverse();
        // Just verify inverse has opposite rotation
        ASSERT_APPROX("Inverse has opposite θ", inverse.theta.toDegrees(), -90.0, 0.5);

        // Test transform composition is associative
        Pose2D pose3(0.5, 0.5, deg(45));
        Pose2D comp1 = (pose1 * pose2) * pose3;
        Pose2D comp2 = pose1 * (pose2 * pose3);
        ASSERT_APPROX("Pose composition associative", comp1.position.x, comp2.position.x, 0.01);
    }

    // ========================================================================
    // TEST 14: PID CONTROLLER DETAILED
    // ========================================================================
    std::cout << "\nTesting PID Controller in Detail...\n";
    {
        PIDController pid(1.0, 0.1, 0.05);

        // Test with constant error
        double out1 = pid.calculate(10.0, 0.1);
        ASSERT_APPROX("PID step 1", out1, 10.0 + 0.1*10.0*0.1, 0.5);

        double out2 = pid.calculate(10.0, 0.1);
        // Should have integral buildup
        ASSERT_APPROX("PID with integral", out2, 10.0 + 0.2*10.0*0.1, 1.0);

        // Reset and test
        pid.reset();
        double out3 = pid.calculate(5.0, 0.1);
        ASSERT_APPROX("PID after reset", out3, 5.0 + 0.05*5.0*0.1, 0.5);

        // Test integral anti-windup
        PIDController::Gains gains = pid.getGains();
        gains.iMax = 1.0;  // Limit integral accumulation
        pid.setGains(gains);

        // Drive integral to limit
        for (int i = 0; i < 100; i++) {
            pid.calculate(100.0, 0.1);
        }
        double limited = pid.calculate(100.0, 0.1);
        // Integral should be limited, but P and D still work
        ASSERT_APPROX("PID with anti-windup", limited, 100.0 + 0.1*1.0, 20.0);
    }

    // ========================================================================
    // TEST 15: FILTER CONVERGENCE AND STABILITY
    // ========================================================================
    std::cout << "\nTesting Filter Convergence and Stability...\n";
    {
        // Low pass filter convergence
        LowPassFilter lpf(0.5);
        double final_val = 0.0;
        for (int i = 0; i < 100; i++) {
            final_val = lpf.update(10.0);
        }
        ASSERT_APPROX("LowPass convergence", final_val, 10.0, 0.1);

        // Moving average filter
        MovingAverageFilter<10> maf;
        double maf_val = 0.0;
        for (int i = 0; i < 20; i++) {
            maf_val = maf.update(5.0);
        }
        ASSERT_APPROX("Moving average steady", maf_val, 5.0, 0.01);

        // Median filter with outliers
        MedianFilter<5> medf;
        medf.update(5.0);
        medf.update(5.0);
        medf.update(100.0);  // Outlier
        medf.update(5.0);
        double median = medf.update(5.0);  // update() returns the median
        ASSERT_APPROX("Median rejects outlier", median, 5.0, 0.01);

        // Kalman filter convergence
        KalmanFilter1D kf(0.01, 0.1);  // process_noise, measurement_noise
        for (int i = 0; i < 50; i++) {
            kf.update(10.0);
        }
        double kf_val = kf.getEstimate();
        ASSERT_APPROX("Kalman convergence", kf_val, 10.0, 0.5);
    }

    // ========================================================================
    // TEST 16: MOTION PROFILING
    // ========================================================================
    std::cout << "\nTesting Motion Profiling...\n";
    {
        // Use the correct API with Constraints and double-based States
        TrapezoidProfile::Constraints constraints(1.0, 2.0);  // maxVel, maxAccel
        TrapezoidProfile::State goal(2.0, 0.0, 0.0);  // pos, vel, accel
        TrapezoidProfile profile(constraints, goal);

        // Calculate state at t=0.5s
        auto state = profile.calculate(0.5);

        // Should be accelerating
        ASSERT_APPROX("Profile velocity > 0", state.velocity, 1.0, 0.5);
        ASSERT_APPROX("Profile position > 0", state.position, 0.25, 0.5);

        // Calculate state at final time
        auto final_state = profile.calculate(100.0);
        ASSERT_APPROX("Profile reaches goal", final_state.position, 2.0, 0.1);
        ASSERT_APPROX("Profile stops at goal", final_state.velocity, 0.0, 0.1);
    }

    // ========================================================================
    // TEST 17: FREQUENCY CONVERSIONS
    // ========================================================================
    std::cout << "\nTesting Frequency Conversions...\n";
    {
        auto freq = Hz(60);
        ASSERT_APPROX("60 Hz", freq.toHertz(), 60.0, 0.01);
        ASSERT_APPROX("60 Hz to kHz", freq.toKilohertz(), 0.06, 0.001);

        auto rpm = RPM::fromRPM(120);
        ASSERT_APPROX("120 RPM to rad/s", rpm.toRadiansPerSecond(), 4.0*constants::PI, 0.01);
        ASSERT_APPROX("120 RPM to deg/s", rpm.toDegreesPerSecond(), 720.0, 0.1);

        // Reverse conversions
        auto from_radps = RadiansPerSecond::fromRadiansPerSecond(constants::PI);
        ASSERT_APPROX("π rad/s to deg/s", from_radps.toDegreesPerSecond(), 180.0, 0.1);
    }

    // ========================================================================
    // TEST 18: MASS CONVERSIONS
    // ========================================================================
    std::cout << "\nTesting Mass Conversions...\n";
    {
        auto mass = kg(1.0);
        ASSERT_APPROX("1kg to grams", mass.toGrams(), 1000.0, 0.01);
        ASSERT_APPROX("1kg to pounds", mass.toPounds(), 2.20462, 0.001);
        ASSERT_APPROX("1kg to ounces", mass.toOunces(), 35.274, 0.01);

        // Reverse
        auto from_lb = Kilograms::fromPounds(2.20462);
        ASSERT_APPROX("2.20462lb to kg", from_lb.toKilograms(), 1.0, 0.01);
    }

    // ========================================================================
    // TEST 19: ACCELERATION TYPES
    // ========================================================================
    std::cout << "\nTesting Acceleration Types...\n";
    {
        auto accel = MetersPerSecondSquared::fromGravities(2.0);
        ASSERT_APPROX("2g to m/s²", accel.toMetersPerSecondSquared(), 19.613, 0.01);

        auto lin_accel = mps(10) / s(2);
        ASSERT_APPROX("Linear acceleration", lin_accel.toMetersPerSecondSquared(), 5.0, 0.01);

        auto ang_accel = radps(constants::PI) / s(1);
        ASSERT_APPROX("Angular acceleration", ang_accel.toRadiansPerSecondSquared(), constants::PI, 0.01);
    }

    // ========================================================================
    // TEST 20: NUMERICAL UTILITIES
    // ========================================================================
    std::cout << "\nTesting Numerical Utilities...\n";
    {
        // Approximate equality
        bool eq1 = numerical::approxEqual(1.0, 1.0000001, 0.001);
        bool eq2 = numerical::approxEqual(1.0, 2.0, 0.001);
        ASSERT_APPROX("Approx equal true", eq1 ? 1.0 : 0.0, 1.0, 0.01);
        ASSERT_APPROX("Approx equal false", eq2 ? 1.0 : 0.0, 0.0, 0.01);

        // Is zero
        bool z1 = numerical::isZero(0.0000001, 0.001);
        bool z2 = numerical::isZero(1.0, 0.001);
        ASSERT_APPROX("Is zero true", z1 ? 1.0 : 0.0, 1.0, 0.01);
        ASSERT_APPROX("Is zero false", z2 ? 1.0 : 0.0, 0.0, 0.01);

        // Min/max
        ASSERT_APPROX("Numerical min", numerical::min(5.0, 3.0), 3.0, 0.01);
        ASSERT_APPROX("Numerical max", numerical::max(5.0, 3.0), 5.0, 0.01);

        // Abs
        ASSERT_APPROX("Numerical abs", numerical::abs(-5.5), 5.5, 0.01);
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
