// ============================================================================
// RobotLib Comprehensive Test Program for ESP32-C3
// ============================================================================
// Purpose: Test all major features of the RobotLib units library
// Platform: Seeed Studio XIAO ESP32-C3
// ============================================================================

#include <Arduino.h>

// Include RobotLib headers in correct order
#include "units_core.h"
#include "units_physics.h"
#include "units_robotics.h"

using namespace units;

// Test result tracking
int totalTests = 0;
int passedTests = 0;
int failedTests = 0;

// Helper function to print test results
void printTestResult(const char* testName, bool passed) {
    totalTests++;
    if (passed) {
        passedTests++;
        Serial.print("[PASS] ");
    } else {
        failedTests++;
        Serial.print("[FAIL] ");
    }
    Serial.println(testName);
}

// Helper function to compare doubles with tolerance
bool approxEqual(double a, double b, double tolerance = 0.001) {
    return abs(a - b) < tolerance;
}

// ============================================================================
// TEST SECTION 1: CORE UNITS
// ============================================================================
void testCoreUnits() {
    Serial.println("\n=== Testing Core Units ===");

    // Distance tests
    auto dist1 = m(10.0);
    auto dist2 = cm(50.0);
    auto totalDist = dist1 + dist2;
    printTestResult("Distance addition", approxEqual(totalDist.toMeters(), 10.5));
    printTestResult("Distance to feet", approxEqual(dist1.toFeet(), 32.808, 0.01));

    // Time tests
    auto time1 = s(5.0);
    auto time2 = ms(500.0);
    auto totalTime = time1 + time2;
    printTestResult("Time addition", approxEqual(totalTime.toSeconds(), 5.5));
    printTestResult("Time to milliseconds", approxEqual(time1.toMilliseconds(), 5000.0));

    // Angle tests
    auto angle1 = deg(90.0);
    auto angle2 = rad(constants::PI / 2.0);
    printTestResult("Degrees to radians", approxEqual(angle1.toRadians(), constants::PI / 2.0, 0.01));
    printTestResult("Angle equality", approxEqual(angle1.toRadians(), angle2.toRadians(), 0.01));

    // Trigonometry tests
    auto angle45 = deg(45.0);
    printTestResult("Sin(45°)", approxEqual(angle45.sin(), 0.707, 0.01));
    printTestResult("Cos(45°)", approxEqual(angle45.cos(), 0.707, 0.01));

    // Mass tests
    auto mass1 = kg(10.0);
    auto mass2 = g(500.0);
    printTestResult("Mass to grams", approxEqual(mass1.toGrams(), 10000.0));
    printTestResult("Mass to pounds", approxEqual(mass1.toPounds(), 22.046, 0.01));

    // Temperature tests
    auto tempC = degC(25.0);
    auto tempF = degF(77.0);
    printTestResult("Celsius to Fahrenheit", approxEqual(tempC.toFahrenheit(), 77.0, 0.1));
    printTestResult("Fahrenheit to Celsius", approxEqual(tempF.toCelsius(), 25.0, 0.1));

    // Scalar tests
    auto scalar1 = Scalar::fromPercent(50.0);
    printTestResult("Scalar percent", approxEqual(scalar1.toValue(), 0.5));
}

// ============================================================================
// TEST SECTION 2: PHYSICS UNITS
// ============================================================================
void testPhysicsUnits() {
    Serial.println("\n=== Testing Physics Units ===");

    // Velocity tests
    auto vel1 = MetersPerSecond::fromMetersPerSecond(10.0);
    auto vel2 = KilometersPerHour::fromKilometersPerHour(36.0);
    printTestResult("Velocity m/s to km/h", approxEqual(vel1.toKilometersPerHour(), 36.0));
    printTestResult("Velocity km/h to m/s", approxEqual(vel2.toMetersPerSecond(), 10.0));

    // Acceleration tests
    auto accel = MetersPerSecondSquared::fromGravities(1.0);
    printTestResult("Acceleration gravities", approxEqual(accel.toMetersPerSecondSquared(), 9.80665, 0.01));

    // Force tests
    auto force = N(100.0);
    printTestResult("Force Newtons to pounds", approxEqual(force.toPounds(), 22.48, 0.1));

    // Energy tests
    auto energy = J(1000.0);
    printTestResult("Energy Joules to kJ", approxEqual(energy.toKilojoules(), 1.0));
    printTestResult("Energy Joules to Wh", approxEqual(energy.toWattHours(), 0.2778, 0.01));

    // Power tests
    auto power = W(1000.0);
    printTestResult("Power Watts to kW", approxEqual(power.toKilowatts(), 1.0));
    printTestResult("Power Watts to HP", approxEqual(power.toHorsepower(), 1.341, 0.01));

    // Torque tests
    auto torque = Nm(100.0);
    printTestResult("Torque Nm to lb-ft", approxEqual(torque.toPoundFeet(), 73.756, 0.1));

    // Angular velocity tests
    auto angVel = RPM::fromRPM(60.0);
    printTestResult("Angular velocity RPM to rad/s", approxEqual(angVel.toRadiansPerSecond(), constants::TWO_PI, 0.01));

    // Electrical tests
    auto voltage = V(12.0);
    auto current = A(2.0);
    auto resistance = ohm(6.0);
    printTestResult("Voltage value", approxEqual(voltage.toVolts(), 12.0));
    printTestResult("Current value", approxEqual(current.toAmperes(), 2.0));
    printTestResult("Resistance value", approxEqual(resistance.toOhms(), 6.0));
}

// ============================================================================
// TEST SECTION 3: PHYSICS OPERATIONS
// ============================================================================
void testPhysicsOperations() {
    Serial.println("\n=== Testing Physics Operations ===");

    // F = ma
    auto mass = kg(10.0);
    auto accel = MetersPerSecondSquared::fromMetersPerSecondSquared(2.0);
    auto force = mass * accel;
    printTestResult("F = ma", approxEqual(force.toNewtons(), 20.0));

    // v = d / t
    auto dist = m(100.0);
    auto time = s(10.0);
    auto velocity = dist / time;
    printTestResult("v = d/t", approxEqual(velocity.toMetersPerSecond(), 10.0));

    // P = VI
    auto volt = V(12.0);
    auto curr = A(5.0);
    auto pwr = volt * curr;
    printTestResult("P = VI", approxEqual(pwr.toWatts(), 60.0));

    // V = IR
    auto current = A(2.0);
    auto resist = ohm(10.0);
    auto voltage = current * resist;
    printTestResult("V = IR", approxEqual(voltage.toVolts(), 20.0));

    // Work = F × d
    auto f = N(50.0);
    auto d = m(10.0);
    auto work = f * d;
    printTestResult("Work = F×d", approxEqual(work.toJoules(), 500.0));
}

// ============================================================================
// TEST SECTION 4: ROBOTICS COMPONENTS
// ============================================================================
void testRoboticsComponents() {
    Serial.println("\n=== Testing Robotics Components ===");

    // Vec2D tests
    Vec2D vec1(3.0, 4.0);
    Vec2D vec2(1.0, 0.0);

    printTestResult("Vec2D magnitude", approxEqual(vec1.magnitude(), 5.0));

    auto dot = vec1.dot(vec2);
    printTestResult("Vec2D dot product", approxEqual(dot, 3.0));

    auto cross = vec1.cross(vec2);
    printTestResult("Vec2D cross product", approxEqual(cross, -4.0));

    auto normalized = vec1.normalized();
    printTestResult("Vec2D normalization", approxEqual(normalized.magnitude(), 1.0));

    // Pose2D tests
    Pose2D pose1(1.0, 2.0, deg(90.0));
    Vec2D localPoint(1.0, 0.0);
    Vec2D globalPoint = pose1.toGlobal(localPoint);
    printTestResult("Pose2D to global", approxEqual(globalPoint.x, 1.0, 0.1) &&
                                        approxEqual(globalPoint.y, 3.0, 0.1));

    // Test inverse transformation
    Vec2D backToLocal = pose1.toLocal(globalPoint);
    printTestResult("Pose2D to local", approxEqual(backToLocal.x, localPoint.x, 0.1) &&
                                       approxEqual(backToLocal.y, localPoint.y, 0.1));
}

// ============================================================================
// TEST SECTION 5: CONTROL SYSTEMS
// ============================================================================
void testControlSystems() {
    Serial.println("\n=== Testing Control Systems ===");

    // PID Controller test
    PIDController::Gains gains(1.0, 0.1, 0.05);
    gains.outputMin = -100.0;
    gains.outputMax = 100.0;
    PIDController pid(gains);

    double error = 10.0;
    double dt = 0.02;
    double output = pid.calculate(error, dt);
    printTestResult("PID controller output", output > 0.0 && output <= 100.0);

    // Test PID reset
    pid.reset();
    printTestResult("PID reset", pid.getIntegral() == 0.0);

    // Trapezoid profile test
    TrapezoidProfile::Constraints constraints(2.0, 1.0);
    TrapezoidProfile::State goal(10.0, 0.0, 0.0);
    TrapezoidProfile profile(constraints, goal);

    auto state = profile.calculate(1.0);
    printTestResult("Trapezoid profile", state.position > 0.0 && state.position < 10.0);
    printTestResult("Trapezoid finished", profile.isFinished(profile.totalTime()));

    // Low-pass filter test
    LowPassFilter lpf(0.9);
    double filtered1 = lpf.update(10.0);
    double filtered2 = lpf.update(10.0);
    printTestResult("Low-pass filter convergence", abs(filtered2 - 10.0) < abs(filtered1 - 10.0));

    // Moving average filter test
    MovingAverageFilter<5> maf;
    maf.update(1.0);
    maf.update(2.0);
    maf.update(3.0);
    double avg = maf.getValue();
    printTestResult("Moving average", approxEqual(avg, 2.0));

    // Rate limiter test
    RateLimiter limiter(1.0); // 1 unit per second max
    double limited = limiter.calculate(10.0, 0.1); // Try to jump to 10 in 0.1s
    printTestResult("Rate limiter", limited < 10.0);
}

// ============================================================================
// TEST SECTION 6: NUMERICAL UTILITIES
// ============================================================================
void testNumericalUtilities() {
    Serial.println("\n=== Testing Numerical Utilities ===");

    // Safe division
    double result = numerical::safeDivide(10.0, 0.0, 999.0);
    printTestResult("Safe divide by zero", approxEqual(result, 999.0));

    double normalDiv = numerical::safeDivide(10.0, 2.0);
    printTestResult("Safe divide normal", approxEqual(normalDiv, 5.0));

    // Approximate equality
    bool equal = numerical::approxEqual(1.0000001, 1.0);
    printTestResult("Approximate equality", equal);

    // Clamp
    double clamped = numerical::clamp(15.0, 0.0, 10.0);
    printTestResult("Clamp upper", approxEqual(clamped, 10.0));

    clamped = numerical::clamp(-5.0, 0.0, 10.0);
    printTestResult("Clamp lower", approxEqual(clamped, 0.0));

    // Min/Max
    double minVal = numerical::min(5.0, 10.0);
    double maxVal = numerical::max(5.0, 10.0);
    printTestResult("Min function", approxEqual(minVal, 5.0));
    printTestResult("Max function", approxEqual(maxVal, 10.0));
}

// ============================================================================
// TEST SECTION 7: EDGE CASES
// ============================================================================
void testEdgeCases() {
    Serial.println("\n=== Testing Edge Cases ===");

    // Zero values
    auto zeroDistance = m(0.0);
    printTestResult("Zero distance", zeroDistance.isZero());

    // Negative values
    auto negDist = m(-10.0);
    auto absDist = negDist.abs();
    printTestResult("Absolute value", approxEqual(absDist.toMeters(), 10.0));

    // Very small values
    auto tiny = m(0.0000001);
    printTestResult("Very small value", !tiny.isZero() || tiny.isZero(0.001));

    // Angle normalization
    auto bigAngle = deg(450.0);
    auto normalized = bigAngle.normalizePositive();
    printTestResult("Angle normalization", approxEqual(normalized.toDegrees(), 90.0, 0.1));

    // Signed angle normalization
    auto signedAngle = deg(270.0);
    auto normSigned = signedAngle.normalizeSigned();
    printTestResult("Signed angle norm", approxEqual(normSigned.toDegrees(), -90.0, 0.1));

    // Overflow protection in integral
    PIDController::Gains gains(1.0, 0.1, 0.0);
    gains.iMax = 10.0;
    PIDController pid(gains);

    // Try to overflow integral
    for (int i = 0; i < 1000; i++) {
        pid.calculate(100.0, 0.1);
    }
    printTestResult("Integral windup protection", pid.getIntegral() <= 10.0);
}

// ============================================================================
// TEST SECTION 8: MEMORY AND PERFORMANCE
// ============================================================================
void testMemoryAndPerformance() {
    Serial.println("\n=== Testing Memory and Performance ===");

    // Check size of fundamental types
    Serial.print("Size of Meters: ");
    Serial.print(sizeof(Meters));
    Serial.println(" bytes");
    printTestResult("Meters size", sizeof(Meters) == sizeof(double));

    Serial.print("Size of Seconds: ");
    Serial.print(sizeof(Seconds));
    Serial.println(" bytes");
    printTestResult("Seconds size", sizeof(Seconds) == sizeof(double));

    Serial.print("Size of Vec2D: ");
    Serial.print(sizeof(Vec2D));
    Serial.println(" bytes");
    printTestResult("Vec2D size", sizeof(Vec2D) == 2 * sizeof(double));

    Serial.print("Size of Pose2D: ");
    Serial.print(sizeof(Pose2D));
    Serial.println(" bytes");

    // Performance test - operations per second
    unsigned long startTime = micros();
    double sum = 0.0;
    for (int i = 0; i < 10000; i++) {
        auto d = m(i * 0.1);
        sum += d.toMeters();
    }
    unsigned long endTime = micros();
    unsigned long duration = endTime - startTime;

    Serial.print("10000 unit operations took ");
    Serial.print(duration);
    Serial.println(" microseconds");
    printTestResult("Performance reasonable", duration < 100000); // Less than 100ms

    // Prevent optimization
    if (sum < 0) Serial.println("Impossible");
}

// ============================================================================
// SETUP AND MAIN LOOP
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(2000); // Wait for serial monitor

    Serial.println("\n\n");
    Serial.println("========================================");
    Serial.println("  RobotLib Test Suite for ESP32-C3");
    Serial.println("  Version 2.1");
    Serial.println("========================================");

    // Print system information
    Serial.print("CPU Frequency: ");
    Serial.print(ESP.getCpuFreqMHz());
    Serial.println(" MHz");

    Serial.print("Free Heap: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes");

    Serial.print("Chip Model: ");
    Serial.println(ESP.getChipModel());

    Serial.print("Flash Size: ");
    Serial.print(ESP.getFlashChipSize());
    Serial.println(" bytes");

    // Run all tests
    unsigned long testStartTime = millis();

    testCoreUnits();
    testPhysicsUnits();
    testPhysicsOperations();
    testRoboticsComponents();
    testControlSystems();
    testNumericalUtilities();
    testEdgeCases();
    testMemoryAndPerformance();

    unsigned long testEndTime = millis();

    // Print summary
    Serial.println("\n========================================");
    Serial.println("  TEST SUMMARY");
    Serial.println("========================================");
    Serial.print("Total tests: ");
    Serial.println(totalTests);
    Serial.print("Passed: ");
    Serial.print(passedTests);
    Serial.print(" (");
    Serial.print((passedTests * 100) / totalTests);
    Serial.println("%)");
    Serial.print("Failed: ");
    Serial.print(failedTests);
    Serial.print(" (");
    Serial.print((failedTests * 100) / totalTests);
    Serial.println("%)");
    Serial.print("Test duration: ");
    Serial.print(testEndTime - testStartTime);
    Serial.println(" ms");
    Serial.println("========================================");

    if (failedTests == 0) {
        Serial.println("\n✓ ALL TESTS PASSED!");
    } else {
        Serial.println("\n✗ SOME TESTS FAILED - Check output above");
    }

    Serial.print("\nFinal Free Heap: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes");
}

void loop() {
    // Nothing to do in loop
    delay(1000);
}
