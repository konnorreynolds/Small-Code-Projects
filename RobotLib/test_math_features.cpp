// ============================================================================
// test_math_features.cpp - Comprehensive Test for units_math.h
// ============================================================================
// Tests all advanced mathematical functions added in units_math.h
// ============================================================================

#include "units_core.h"
#include "units_physics.h"
#include "units_robotics.h"
#include "units_math.h"

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>

using namespace units;

// Test counter
int tests_passed = 0;
int tests_failed = 0;

// Assertion macro
#define ASSERT_APPROX(name, actual, expected, tolerance) \
    do { \
        if (std::abs((actual) - (expected)) <= (tolerance)) { \
            std::cout << "  ✓ " << name << " (got " << actual << ")\n"; \
            tests_passed++; \
        } else { \
            std::cout << "  ✗ " << name << " (got " << actual << ", expected " << expected << ")\n"; \
            tests_failed++; \
        } \
    } while(0)

#define ASSERT_TRUE(name, condition) \
    do { \
        if (condition) { \
            std::cout << "  ✓ " << name << "\n"; \
            tests_passed++; \
        } else { \
            std::cout << "  ✗ " << name << " (condition was false)\n"; \
            tests_failed++; \
        } \
    } while(0)

int main() {
    std::cout << "========================================\n";
    std::cout << "  units_math.h Feature Test\n";
    std::cout << "  C++ Standard: " << __cplusplus << "\n";
    std::cout << "========================================\n";

    // ========================================================================
    // TEST 1: SQUARE ROOT AND POWER OPERATIONS
    // ========================================================================
    std::cout << "\nTesting Square Root and Power Operations...\n";
    {
        // Squared operations
        auto dist = m(5);
        double area = squared(dist);
        ASSERT_APPROX("Distance squared", area, 25.0, 0.01);

        auto vel = mps(10);
        double velSquared = squared(vel);
        ASSERT_APPROX("Velocity squared", velSquared, 100.0, 0.01);

        // Cubed operations
        double volume = cubed(m(3));
        ASSERT_APPROX("Distance cubed (volume)", volume, 27.0, 0.01);

        // Square root of area
        auto side = sqrtArea(25.0);
        ASSERT_APPROX("Square root of area", side.toMeters(), 5.0, 0.01);

        // Absolute value
        auto absVal = abs(m(-5.5));
        ASSERT_APPROX("Abs of distance", absVal.toMeters(), 5.5, 0.01);

        auto absAngle = abs(deg(-45));
        ASSERT_APPROX("Abs of angle", absAngle.toDegrees(), 45.0, 0.01);
    }

    // ========================================================================
    // TEST 2: SIGN FUNCTION
    // ========================================================================
    std::cout << "\nTesting Sign Function...\n";
    {
        ASSERT_TRUE("Sign of positive", sign(5.0) == 1);
        ASSERT_TRUE("Sign of negative", sign(-3.0) == -1);
        ASSERT_TRUE("Sign of zero", sign(0.0) == 0);

        ASSERT_APPROX("SignDouble of positive", signDouble(10.0), 1.0, 0.01);
        ASSERT_APPROX("SignDouble of negative", signDouble(-7.0), -1.0, 0.01);
    }

    // ========================================================================
    // TEST 3: INTERPOLATION FUNCTIONS
    // ========================================================================
    std::cout << "\nTesting Interpolation...\n";
    {
        // Linear interpolation
        auto lerp_result = lerp(m(0), m(10), 0.5);
        ASSERT_APPROX("Lerp at 0.5", lerp_result.toMeters(), 5.0, 0.01);

        auto lerp_quarter = lerp(m(0), m(100), 0.25);
        ASSERT_APPROX("Lerp at 0.25", lerp_quarter.toMeters(), 25.0, 0.01);

        // Inverse lerp
        double t = inverseLerp(0.0, 10.0, 7.5);
        ASSERT_APPROX("Inverse lerp", t, 0.75, 0.01);

        // Smoothstep
        double smooth = smoothstep(0.5);
        ASSERT_APPROX("Smoothstep at 0.5", smooth, 0.5, 0.1);

        double smootherStep = smootherstep(0.5);
        ASSERT_APPROX("Smootherstep at 0.5", smootherStep, 0.5, 0.1);
    }

    // ========================================================================
    // TEST 4: RAMP LIMITER
    // ========================================================================
    std::cout << "\nTesting Ramp Limiter...\n";
    {
        RampLimiter limiter(10.0);  // Max 10 units/second

        // Start at 0, target 100, dt = 0.1s
        // Should increase by 1.0 (10 * 0.1)
        double val1 = limiter.update(100.0, 0.1);
        ASSERT_APPROX("Ramp step 1", val1, 1.0, 0.01);

        double val2 = limiter.update(100.0, 0.1);
        ASSERT_APPROX("Ramp step 2", val2, 2.0, 0.01);

        // Sudden decrease should also be limited
        RampLimiter limiter2(10.0, 5.0);  // Rising 10, falling 5
        limiter2.setValue(100.0);
        double val3 = limiter2.update(0.0, 0.1);
        ASSERT_APPROX("Ramp decrease", val3, 99.5, 0.01);
    }

    // ========================================================================
    // TEST 5: DEADBAND
    // ========================================================================
    std::cout << "\nTesting Deadband...\n";
    {
        // Simple deadband
        double db1 = applyDeadbandSimple(0.05, 0.1);
        ASSERT_APPROX("Deadband below threshold", db1, 0.0, 0.01);

        double db2 = applyDeadbandSimple(0.5, 0.1);
        ASSERT_APPROX("Deadband above threshold", db2, 0.5, 0.01);

        // Scaled deadband
        double db3 = applyDeadband(0.05, 0.1);
        ASSERT_APPROX("Scaled deadband below", db3, 0.0, 0.01);

        double db4 = applyDeadband(1.0, 0.1);
        ASSERT_APPROX("Scaled deadband at max", db4, 1.0, 0.01);
    }

    // ========================================================================
    // TEST 6: ANGLE WRAPPING
    // ========================================================================
    std::cout << "\nTesting Angle Wrapping...\n";
    {
        // Wrap to [-π, π]
        auto wrapped1 = wrapToPi(deg(450));
        ASSERT_APPROX("Wrap 450° to ±180°", wrapped1.toDegrees(), 90.0, 0.01);

        auto wrapped2 = wrapToPi(deg(-270));
        ASSERT_APPROX("Wrap -270° to ±180°", wrapped2.toDegrees(), 90.0, 0.01);

        // Wrap to [0, 2π]
        auto wrapped3 = wrapTo2Pi(deg(-90));
        ASSERT_APPROX("Wrap -90° to [0,360°]", wrapped3.toDegrees(), 270.0, 0.01);

        // Shortest angular distance
        auto diff = shortestAngularDistance(deg(10), deg(350));
        ASSERT_APPROX("Shortest angle 10°→350°", diff.toDegrees(), -20.0, 0.5);

        auto diff2 = shortestAngularDistance(deg(350), deg(10));
        ASSERT_APPROX("Shortest angle 350°→10°", diff2.toDegrees(), 20.0, 0.5);
    }

    // ========================================================================
    // TEST 7: STATISTICAL FUNCTIONS
    // ========================================================================
    std::cout << "\nTesting Statistical Functions...\n";
    {
        std::vector<double> data = {1.0, 2.0, 3.0, 4.0, 5.0};

        double avg = mean(data);
        ASSERT_APPROX("Mean of 1-5", avg, 3.0, 0.01);

        double var = variance(data);
        ASSERT_APPROX("Variance of 1-5", var, 2.5, 0.01);

        double std = standardDeviation(data);
        ASSERT_APPROX("Std dev of 1-5", std, 1.58, 0.01);

        std::vector<double> data2 = {1.0, 2.0, 3.0, 4.0, 5.0};
        double med = median(data2);
        ASSERT_APPROX("Median of 1-5", med, 3.0, 0.01);
    }

    // ========================================================================
    // TEST 8: RANGE MAPPING
    // ========================================================================
    std::cout << "\nTesting Range Mapping...\n";
    {
        // Map 512 from [0,1023] to [0,5]
        double mapped = map(512.0, 0.0, 1023.0, 0.0, 5.0);
        ASSERT_APPROX("Map ADC to voltage", mapped, 2.5, 0.01);

        // Map with clamping
        double clamped = mapClamped(1500.0, 0.0, 1023.0, 0.0, 5.0);
        ASSERT_APPROX("Map clamped (high)", clamped, 5.0, 0.01);

        double clamped2 = mapClamped(-100.0, 0.0, 1023.0, 0.0, 5.0);
        ASSERT_APPROX("Map clamped (low)", clamped2, 0.0, 0.01);
    }

    // ========================================================================
    // TEST 9: HYSTERESIS
    // ========================================================================
    std::cout << "\nTesting Hysteresis...\n";
    {
        Hysteresis hyst(2.0, 4.0);

        ASSERT_TRUE("Hysteresis initial", !hyst.update(3.0));
        ASSERT_TRUE("Hysteresis high", hyst.update(5.0));
        ASSERT_TRUE("Hysteresis stays high", hyst.update(3.0));
        ASSERT_TRUE("Hysteresis goes low", !hyst.update(1.0));
        ASSERT_TRUE("Hysteresis stays low", !hyst.update(3.0));
    }

    // ========================================================================
    // TEST 10: SATURATION
    // ========================================================================
    std::cout << "\nTesting Saturation...\n";
    {
        double sat1 = saturate(150.0, 0.0, 100.0);
        ASSERT_APPROX("Saturate above max", sat1, 100.0, 0.01);

        double sat2 = saturate(-50.0, 0.0, 100.0);
        ASSERT_APPROX("Saturate below min", sat2, 0.0, 0.01);

        double sat3 = saturate(50.0, 0.0, 100.0);
        ASSERT_APPROX("Saturate in range", sat3, 50.0, 0.01);

        double sat4 = saturateSymmetric(150.0, 100.0);
        ASSERT_APPROX("Symmetric saturate", sat4, 100.0, 0.01);
    }

    // ========================================================================
    // TEST 11: NUMERICAL DERIVATIVE
    // ========================================================================
    std::cout << "\nTesting Numerical Derivative...\n";
    {
        NumericalDerivative<double> deriv;

        double d1 = deriv.calculate(0.0, 0.1);
        ASSERT_APPROX("Derivative initial", d1, 0.0, 0.01);

        double d2 = deriv.calculate(1.0, 0.1);
        ASSERT_APPROX("Derivative step", d2, 10.0, 0.01);

        double d3 = deriv.calculate(2.0, 0.1);
        ASSERT_APPROX("Derivative constant", d3, 10.0, 0.01);
    }

    // ========================================================================
    // TEST 12: FIRST-ORDER LAG FILTER
    // ========================================================================
    std::cout << "\nTesting First-Order Lag Filter...\n";
    {
        FirstOrderLag<double> lag(0.1);  // Time constant 0.1s

        double val1 = lag.update(10.0, 0.05);
        // First call initializes to input value
        ASSERT_APPROX("Lag filter initial", val1, 10.0, 0.01);

        // Change target and verify lag behavior
        double val2 = lag.update(0.0, 0.05);
        ASSERT_TRUE("Lag filter response", val2 > 0.0 && val2 < 10.0);

        // After many updates should converge
        for (int i = 0; i < 50; i++) {
            lag.update(0.0, 0.05);
        }
        double valFinal = lag.getValue();
        ASSERT_APPROX("Lag filter convergence", valFinal, 0.0, 0.1);
    }

    // ========================================================================
    // SUMMARY
    // ========================================================================
    std::cout << "\n========================================\n";
    std::cout << "  TEST SUMMARY\n";
    std::cout << "========================================\n";
    std::cout << "Tests passed: " << tests_passed << "\n";
    std::cout << "Tests failed: " << tests_failed << "\n";

    if (tests_failed == 0) {
        std::cout << "Success rate: 100.0%\n\n";
        std::cout << "✅ ALL MATH TESTS PASSED!\n";
        std::cout << "✅ units_math.h is working correctly\n";
        std::cout << "✅ All mathematical functions verified\n";
        return 0;
    } else {
        double rate = 100.0 * tests_passed / (tests_passed + tests_failed);
        std::cout << "Success rate: " << std::fixed << std::setprecision(1) << rate << "%\n\n";
        std::cout << "❌ SOME TESTS FAILED\n";
        return 1;
    }
}
