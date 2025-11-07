// ============================================================================
// Example 2: PID Controller Tuning Guide
// ============================================================================
// This example demonstrates how to use and tune a PID controller for
// various robotics applications (position control, velocity control, etc.)
//
// Topics covered:
// - Basic PID usage
// - Tuning methodology (Ziegler-Nichols)
// - Anti-windup demonstration
// - Feedforward addition
// - Step response analysis
// ============================================================================

#include "../../include/units_core.h"
#include "../../include/units_physics.h"
#include "../../include/units_robotics.h"

#include <vector>

using namespace units;

// ============================================================================
// Simulated System (First-Order with Delay)
// ============================================================================
class SimulatedMotor {
private:
    double position_;
    double velocity_;
    double timeConstant_;  // How fast the motor responds
    double gain_;          // Output scaling

public:
    SimulatedMotor(double timeConstant = 0.1, double gain = 1.0)
        : position_(0), velocity_(0), timeConstant_(timeConstant), gain_(gain) {}

    // Update motor with control input
    void update(double controlInput, double dt) {
        // First-order system: dv/dt = (input - v) / tau
        double targetVelocity = controlInput * gain_;
        velocity_ += (targetVelocity - velocity_) / timeConstant_ * dt;
        position_ += velocity_ * dt;
    }

    double getPosition() const { return position_; }
    double getVelocity() const { return velocity_; }

    void reset() {
        position_ = 0;
        velocity_ = 0;
    }
};

// ============================================================================
// Test Scenario: Step Response
// ============================================================================
struct StepResponse {
    std::vector<double> time;
    std::vector<double> setpoint;
    std::vector<double> measured;
    std::vector<double> error;
    std::vector<double> output;

    void print(size_t maxSamples = 20) const {
        print(, );
        println("\nTime(s) | Setpoint | Measured | Error | Output");
        println("--------|----------|----------|-------|--------");

        size_t step = time.size() / maxSamples;
        if (step < 1) step = 1;

        for (size_t i = 0; i < time.size(); i += step) {
            print(, time[i], " | ");
            print(, setpoint[i], " | ");
            print(, measured[i], " | ");
            print(, error[i], " | ");
            println(, output[i], "");
        }
    }

    // Calculate performance metrics
    void analyzePerformance() const {
        double overshoot = 0;
        double settlingTime = 0;
        double steadyStateError = 0;
        double riseTime = 0;

        double finalSetpoint = setpoint.back();
        double tolerance = 0.02 * finalSetpoint;  // 2% tolerance

        // Find overshoot
        for (size_t i = 0; i < measured.size(); i++) {
            double percentOver = (measured[i] - finalSetpoint) / finalSetpoint * 100.0;
            if (percentOver > overshoot) {
                overshoot = percentOver;
            }
        }

        // Find settling time (last time outside 2% band)
        for (int i = measured.size() - 1; i >= 0; i--) {
            if (std::abs(measured[i] - finalSetpoint) > tolerance) {
                settlingTime = time[i];
                break;
            }
        }

        // Find rise time (10% to 90%)
        double target10 = finalSetpoint * 0.1;
        double target90 = finalSetpoint * 0.9;
        double time10 = 0, time90 = 0;

        for (size_t i = 0; i < measured.size(); i++) {
            if (measured[i] >= target10 && time10 == 0) {
                time10 = time[i];
            }
            if (measured[i] >= target90 && time90 == 0) {
                time90 = time[i];
                break;
            }
        }
        riseTime = time90 - time10;

        // Steady-state error
        steadyStateError = finalSetpoint - measured.back();

        println("\nPerformance Metrics:");
        println("  Overshoot: ", , overshoot, "%");
        println("  Settling time (2%): ", settlingTime, " s");
        println("  Rise time (10%-90%): ", riseTime, " s");
        println("  Steady-state error: ", steadyStateError, "");
    }
};

// ============================================================================
// Run Simulation with PID Controller
// ============================================================================
StepResponse runSimulation(PIDController& pid, SimulatedMotor& motor,
                          double setpoint, double duration, double dt) {
    StepResponse response;

    for (double t = 0; t <= duration; t += dt) {
        // Get current position
        double measured = motor.getPosition();
        double error = setpoint - measured;

        // Calculate PID output
        double output = pid.calculate(error, dt);

        // Apply to motor
        motor.update(output, dt);

        // Record data
        response.time.push_back(t);
        response.setpoint.push_back(setpoint);
        response.measured.push_back(measured);
        response.error.push_back(error);
        response.output.push_back(output);
    }

    return response;
}

// ============================================================================
// Main Program
// ============================================================================
int main() {
    println("========================================");
    println("  PID Controller Tuning Guide");
    println("========================================\n");

    const double dt = 0.01;      // 10ms timestep
    const double duration = 2.0; // 2 second simulation
    const double setpoint = 10.0; // Target position

    // ========================================================================
    // Test 1: P-Only Control
    // ========================================================================
    println("Test 1: Proportional-Only Control (kP=1.0)");
    println("===========================================");
    println("Expected: Fast response but steady-state error");

    SimulatedMotor motor1;
    PIDController pid1(1.0, 0.0, 0.0);  // P-only

    auto response1 = runSimulation(pid1, motor1, setpoint, duration, dt);
    response1.print(15);
    response1.analyzePerformance();

    // ========================================================================
    // Test 2: PI Control
    // ========================================================================
    println("\n\nTest 2: Proportional-Integral Control (kP=1.0, kI=0.5)");
    println("======================================================");
    println("Expected: Eliminates steady-state error, might overshoot");

    SimulatedMotor motor2;
    PIDController pid2(1.0, 0.5, 0.0);  // PI

    auto response2 = runSimulation(pid2, motor2, setpoint, duration, dt);
    response2.print(15);
    response2.analyzePerformance();

    // ========================================================================
    // Test 3: PID Control
    // ========================================================================
    println("\n\nTest 3: Full PID Control (kP=1.2, kI=0.5, kD=0.1)");
    println("==================================================");
    println("Expected: Fast response, no overshoot, no steady-state error");

    SimulatedMotor motor3;
    PIDController pid3(1.2, 0.5, 0.1);  // Full PID

    auto response3 = runSimulation(pid3, motor3, setpoint, duration, dt);
    response3.print(15);
    response3.analyzePerformance();

    // ========================================================================
    // Test 4: Aggressive vs Conservative Tuning
    // ========================================================================
    println("\n\nTest 4: Aggressive Tuning (kP=3.0, kI=2.0, kD=0.5)");
    println("==================================================");
    println("Expected: Very fast but might oscillate");

    SimulatedMotor motor4;
    PIDController pid4(3.0, 2.0, 0.5);

    auto response4 = runSimulation(pid4, motor4, setpoint, duration, dt);
    response4.print(15);
    response4.analyzePerformance();

    // ========================================================================
    // Test 5: Anti-Windup Demonstration
    // ========================================================================
    println("\n\nTest 5: Anti-Windup Protection");
    println("================================");
    println("Testing integral windup protection with output limits");

    SimulatedMotor motor5;
    PIDController::Gains gains5(1.0, 1.0, 0.1);
    gains5.outputMin = -5.0;
    gains5.outputMax = 5.0;
    gains5.iMax = 10.0;  // Limit integral accumulation
    PIDController pid5(gains5);

    auto response5 = runSimulation(pid5, motor5, setpoint, duration, dt);
    println("Integral value at end: ", pid5.getIntegral(), " (limited to ±10.0)");
    response5.print(15);
    response5.analyzePerformance();

    // ========================================================================
    // Tuning Guidelines
    // ========================================================================
    println("\n\n========================================");
    println("  PID Tuning Guidelines");
    println("========================================\n");

    println("Manual Tuning Method:");
    println("1. Start with kP=0, kI=0, kD=0");
    println("2. Increase kP until system oscillates");
    println("3. Set kP to 50-60% of that value");
    println("4. Increase kI to eliminate steady-state error");
    println("5. Add kD to reduce overshoot and oscillation\n");

    println("Effect of Each Gain:");
    println("  kP (Proportional):");
    println("    ↑ kP → Faster response, more overshoot");
    println("    ↓ kP → Slower response, less overshoot\n");

    println("  kI (Integral):");
    println("    ↑ kI → Eliminates steady-state error faster");
    println("    ↓ kI → Slower to eliminate error");
    println("    ⚠️  Too high kI → Oscillation, instability\n");

    println("  kD (Derivative):");
    println("    ↑ kD → Reduces overshoot, dampens oscillation");
    println("    ↓ kD → More overshoot possible");
    println("    ⚠️  Too high kD → Amplifies noise\n");

    println("Ziegler-Nichols Method:");
    println("1. Set kI=0, kD=0");
    println("2. Increase kP until sustained oscillation");
    println("3. Note critical gain (Ku) and oscillation period (Tu)");
    println("4. Calculate:");
    println("   kP = 0.6 * Ku");
    println("   kI = 2 * kP / Tu");
    println("   kD = kP * Tu / 8\n");

    println("Common Issues:");
    println("  • Oscillation → Reduce kP or increase kD");
    println("  • Slow response → Increase kP");
    println("  • Steady-state error → Increase kI");
    println("  • Overshoot → Increase kD or reduce kP");
    println("  • Noise sensitivity → Reduce kD, add filtering\n");

    println("Advanced Tips:");
    println("  1. Always set output limits (outputMin, outputMax)");
    println("  2. Set integral limits (iMax) to prevent windup");
    println("  3. Consider derivative filtering for noisy sensors");
    println("  4. Add feedforward for known system dynamics");
    println("  5. Use different gains for different operating ranges\n");

    println("========================================");
    println("  Example Complete!");
    println("========================================\n");

    return 0;
}

/*
This example teaches:
1. How to use the PID controller
2. Effect of each gain (P, I, D)
3. Performance metrics (overshoot, settling time, rise time)
4. Manual tuning process
5. Ziegler-Nichols tuning method
6. Common problems and solutions
7. Anti-windup importance
8. Best practices for robust control
*/
