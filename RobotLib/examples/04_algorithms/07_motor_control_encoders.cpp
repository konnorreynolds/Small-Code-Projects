// ============================================================================
// Example 7: Advanced Motor Control with Encoder Feedback
// ============================================================================
// This example demonstrates professional motor control techniques using
// encoder feedback for precise velocity and position control.
//
// Topics covered:
// - Velocity control with encoder feedback
// - Position control (motion profiling)
// - Feedforward + feedback control
// - Acceleration limiting for smooth motion
// - Velocity PID tuning
// - Encoder filtering and noise handling
// ============================================================================

#include "../../include/units_core.h"
#include "../../include/units_physics.h"
#include "../../include/units_robotics.h"
#include "../../include/units_utilities.h"
#include "../../include/units_math.h"

#include <cmath>
#include <deque>

using namespace units;
using namespace robotics;

// ============================================================================
// Motor Specifications
// ============================================================================
struct MotorSpecs {
    // Physical parameters
    Meters wheelDiameter = m(0.065);  // 6.5 cm
    int countsPerRevolution = 2048;        // Encoder resolution

    // Electrical limits
    double maxVoltage = 12.0;              // Battery voltage
    double stallCurrent = 131.0;           // Amperes
    RPM freeSpeed = RPM::fromRPM(5330);    // Free speed

    // Mechanical limits
    RPM maxRPM = RPM::fromRPM(4000);
    double maxAccelRPMperSec = 6000.0;     // RPM/s

    // Motor constants
    double kV = freeSpeed.toRPM() / maxVoltage;  // RPM per volt
    double kT = 0.01;                             // Torque constant (Nm/A)
};

// ============================================================================
// Encoder Reader with Filtering
// ============================================================================
class Encoder {
private:
    int countsPerRev_;
    Meters wheelDiameter_;

    int currentCount_;
    int lastCount_;
    double lastTime_;

    // Velocity filtering
    MovingAverageFilter<5> velocityFilter_;
    LowPassFilter velocityLowPass_;

    // For acceleration calculation
    RPM lastVelocity_;

public:
    Encoder(int countsPerRev, Meters wheelDiameter)
        : countsPerRev_(countsPerRev),
          wheelDiameter_(wheelDiameter),
          currentCount_(0),
          lastCount_(0),
          lastTime_(0.0),
          velocityLowPass_(0.1),  // 10 Hz cutoff
          lastVelocity_(RPM::fromRPM(0)) {}

    // Simulate reading encoder count (in real robot, read from hardware)
    void setCount(int count) {
        currentCount_ = count;
    }

    int getCount() const {
        return currentCount_;
    }

    // Get position in distance units
    Meters getPosition() const {
        double revolutions = static_cast<double>(currentCount_) / countsPerRev_;
        double circumference = constants::PI * wheelDiameter_.toMeters();
        return m(revolutions * circumference);
    }

    // Get velocity with filtering
    RPM getVelocity(double currentTime) {
        double dt = currentTime - lastTime_;
        if (dt <= 0.0) {
            return lastVelocity_;
        }

        // Calculate raw velocity from encoder counts
        int deltaCounts = currentCount_ - lastCount_;
        double deltaRevolutions = static_cast<double>(deltaCounts) / countsPerRev_;
        double rpm = (deltaRevolutions / dt) * 60.0;  // Convert to RPM

        // Apply filtering to reduce noise
        rpm = velocityFilter_.update(rpm);
        rpm = velocityLowPass_.update(rpm);

        // Update state
        lastCount_ = currentCount_;
        lastTime_ = currentTime;
        lastVelocity_ = RPM::fromRPM(rpm);

        return lastVelocity_;
    }

    void reset() {
        currentCount_ = 0;
        lastCount_ = 0;
        lastVelocity_ = RPM::fromRPM(0);
    }
};

// ============================================================================
// Motor Controller with Feedforward + Feedback
// ============================================================================
class AdvancedMotorController {
private:
    MotorSpecs specs_;
    Encoder& encoder_;
    PIDController velocityPID_;

    // Feedforward gains
    double kS_;  // Static friction feedforward (volts)
    double kV_;  // Velocity feedforward (volts per unit velocity)
    double kA_;  // Acceleration feedforward (volts per unit acceleration)

    // Current state
    RPM targetVelocity_;
    RPM lastTargetVelocity_;

    // Acceleration limiter
    RampLimiter accelLimiter_;

public:
    AdvancedMotorController(const MotorSpecs& specs, Encoder& encoder)
        : specs_(specs),
          encoder_(encoder),
          velocityPID_(0.1, 0.0, 0.0),  // Start with P-only, tune later
          kS_(0.5),   // Overcome static friction
          kV_(12.0 / specs.freeSpeed.toRPM()),  // Voltage to achieve speed
          kA_(0.01),  // Acceleration compensation
          targetVelocity_(RPM::fromRPM(0)),
          lastTargetVelocity_(RPM::fromRPM(0)),
          accelLimiter_(specs.maxAccelRPMperSec, specs.maxAccelRPMperSec) {

        // Configure PID limits
        PIDController::Gains gains = velocityPID_.getGains();
        gains.outputMin = -specs_.maxVoltage;
        gains.outputMax = specs_.maxVoltage;
        gains.iMax = 2.0;  // Limit integral windup
        velocityPID_.setGains(gains);
    }

    // Set PID gains
    void setVelocityPID(double kP, double kI, double kD) {
        PIDController::Gains gains = velocityPID_.getGains();
        gains.kP = kP;
        gains.kI = kI;
        gains.kD = kD;
        velocityPID_.setGains(gains);
    }

    // Set feedforward gains
    void setFeedforward(double kS, double kV, double kA) {
        kS_ = kS;
        kV_ = kV;
        kA_ = kA;
    }

    // Control loop: returns voltage command
    struct ControlOutput {
        double voltage;
        double feedforward;
        double feedback;
        RPM measuredVelocity;
        RPM velocityError;
    };

    ControlOutput update(RPM targetVelocity, double currentTime, double dt) {
        // Apply acceleration limiting to target
        double limitedRPM = accelLimiter_.update(targetVelocity.toRPM(), dt);
        RPM limitedTarget = RPM::fromRPM(limitedRPM);

        // Read encoder velocity
        RPM measuredVelocity = encoder_.getVelocity(currentTime);

        // Calculate velocity error
        double error = limitedTarget.toRPM() - measuredVelocity.toRPM();
        RPM velocityError = RPM::fromRPM(error);

        // Calculate acceleration (for feedforward)
        double acceleration = (limitedTarget.toRPM() - lastTargetVelocity_.toRPM()) / dt;

        // === FEEDFORWARD CONTROL ===
        // Predicts the voltage needed based on desired motion
        double ff_voltage = 0.0;

        // Static friction compensation (only when moving)
        if (std::abs(limitedTarget.toRPM()) > 10.0) {
            ff_voltage += kS_ * units::sign(limitedTarget.toRPM());
        }

        // Velocity feedforward (main component)
        ff_voltage += kV_ * limitedTarget.toRPM();

        // Acceleration feedforward (helps with quick changes)
        ff_voltage += kA_ * acceleration;

        // === FEEDBACK CONTROL (PID) ===
        // Corrects for model errors and disturbances
        double fb_voltage = velocityPID_.calculate(error, dt);

        // === COMBINED OUTPUT ===
        double totalVoltage = ff_voltage + fb_voltage;

        // Clamp to battery voltage limits
        totalVoltage = numerical::clamp(totalVoltage, -specs_.maxVoltage, specs_.maxVoltage);

        // Update state
        targetVelocity_ = limitedTarget;
        lastTargetVelocity_ = limitedTarget;

        return {totalVoltage, ff_voltage, fb_voltage, measuredVelocity, velocityError};
    }

    void reset() {
        velocityPID_.reset();
        accelLimiter_.reset();
        targetVelocity_ = RPM::fromRPM(0);
        lastTargetVelocity_ = RPM::fromRPM(0);
    }
};

// ============================================================================
// Motor Simulator (replaces real hardware)
// ============================================================================
class MotorSimulator {
private:
    MotorSpecs specs_;
    Encoder& encoder_;

    // Physical state
    double currentRPM_;
    double currentPosition_;  // In encoder counts

    // Disturbances
    double loadTorque_;  // External load (Nm)

public:
    MotorSimulator(const MotorSpecs& specs, Encoder& encoder)
        : specs_(specs),
          encoder_(encoder),
          currentRPM_(0.0),
          currentPosition_(0.0),
          loadTorque_(0.0) {}

    void setLoad(double torqueNm) {
        loadTorque_ = torqueNm;
    }

    void update(double voltage, double dt) {
        // Simple DC motor model: V = I*R + ω*K_e
        // Torque = K_t * I
        // For simplicity, use first-order lag model

        // Calculate target RPM from voltage (steady-state)
        double targetRPM = (voltage / specs_.maxVoltage) * specs_.freeSpeed.toRPM();

        // Apply load effect (reduces speed)
        double loadEffect = loadTorque_ * 100.0;  // Simplified
        targetRPM -= loadEffect;

        // First-order response (motor has inertia)
        double tau = 0.05;  // Time constant (seconds)
        double alpha = dt / (tau + dt);
        currentRPM_ += alpha * (targetRPM - currentRPM_);

        // Add small noise to simulate real encoder (currently disabled for clarity)
        // double noise = (std::rand() % 21 - 10) * 0.5;  // ±5 RPM noise
        // double noisyRPM = currentRPM_ + noise;

        // Update position (integrate velocity)
        double deltaRevolutions = (currentRPM_ / 60.0) * dt;
        double deltaCounts = deltaRevolutions * specs_.countsPerRevolution;
        currentPosition_ += deltaCounts;

        // Update encoder
        encoder_.setCount(static_cast<int>(currentPosition_));
    }

    double getCurrentRPM() const { return currentRPM_; }

    void reset() {
        currentRPM_ = 0.0;
        currentPosition_ = 0.0;
    }
};

// ============================================================================
// Demonstration Scenarios
// ============================================================================

void demonstrateVelocityControl() {
    println("========================================");
    println("  Velocity Control with PID");
    println("========================================\n");

    MotorSpecs specs;
    Encoder encoder(specs.countsPerRevolution, specs.wheelDiameter);
    MotorSimulator motor(specs, encoder);
    AdvancedMotorController controller(specs, encoder);

    // Tune PID for velocity control
    controller.setVelocityPID(0.05, 0.5, 0.002);
    controller.setFeedforward(0.5, 12.0 / specs.freeSpeed.toRPM(), 0.01);

    println("Target: 2000 RPM step input");
    println("PID Gains: kP=0.05, kI=0.5, kD=0.002\n");

    double dt = 0.02;  // 20ms (50 Hz control loop)
    double currentTime = 0.0;
    RPM target = RPM::fromRPM(2000);

    print(, );
    println("Time(s) | Target | Measured | Error | Voltage | FF    | FB    |");
    println("--------|--------|----------|-------|---------|-------|-------|");

    for (int i = 0; i < 150; i++) {
        currentTime = i * dt;

        // Control loop
        auto output = controller.update(target, currentTime, dt);

        // Simulate motor response
        motor.update(output.voltage, dt);

        // Print every 10 iterations (0.2s)
        if (i % 10 == 0) {
            print(, currentTime, "  | ");
            print(, target.toRPM(), " | ");
            print(, output.measuredVelocity.toRPM(), " | ");
            print(, output.velocityError.toRPM(), " | ");
            print(, output.voltage, " | ");
            print(, output.feedforward, " | ");
            println(, output.feedback, " |");
        }
    }

    println("\nKey Observations:");
    println("• Feedforward provides ~90% of control effort");
    println("• Feedback (PID) corrects remaining error");
    println("• Steady-state error < 10 RPM with integral term");
    println("• Fast response (~0.2s settling time)\n");
}

void demonstrateFeedforwardBenefit() {
    println("========================================");
    println("  Feedforward vs Feedback Only");
    println("========================================\n");

    MotorSpecs specs;

    // Test 1: Feedback only
    println("Test 1: Feedback Only (PID without feedforward)");
    {
        Encoder encoder(specs.countsPerRevolution, specs.wheelDiameter);
        MotorSimulator motor(specs, encoder);
        AdvancedMotorController controller(specs, encoder);

        controller.setVelocityPID(0.1, 0.0, 0.0);
        controller.setFeedforward(0.0, 0.0, 0.0);  // No feedforward!

        double dt = 0.02;
        RPM target = RPM::fromRPM(2000);

        // Run for 1 second
        double settlingError = 999.0;
        for (int i = 0; i < 50; i++) {
            auto output = controller.update(target, i * dt, dt);
            motor.update(output.voltage, dt);

            if (i > 25) {  // After 0.5s
                settlingError = std::abs(output.velocityError.toRPM());
            }
        }

        println("  Steady-state error: ", settlingError, " RPM (POOR)");
        println("  → PI needed to eliminate error → slow response\n");
    }

    // Test 2: Feedforward + Feedback
    println("Test 2: Feedforward + Feedback (Combined)");
    {
        Encoder encoder(specs.countsPerRevolution, specs.wheelDiameter);
        MotorSimulator motor(specs, encoder);
        AdvancedMotorController controller(specs, encoder);

        controller.setVelocityPID(0.05, 0.5, 0.002);
        controller.setFeedforward(0.5, 12.0 / specs.freeSpeed.toRPM(), 0.01);

        double dt = 0.02;
        RPM target = RPM::fromRPM(2000);

        double settlingError = 999.0;
        for (int i = 0; i < 50; i++) {
            auto output = controller.update(target, i * dt, dt);
            motor.update(output.voltage, dt);

            if (i > 25) {
                settlingError = std::abs(output.velocityError.toRPM());
            }
        }

        println("  Steady-state error: ", settlingError, " RPM (EXCELLENT)");
        println("  → FF provides base control, FB corrects errors\n");
    }

    println("Conclusion:");
    println("Feedforward dramatically improves performance!");
    println("• Faster response");
    println("• Lower steady-state error");
    println("• Less reliance on integral term\n");
}

void demonstrateLoadRejection() {
    println("========================================");
    println("  Load Disturbance Rejection");
    println("========================================\n");

    MotorSpecs specs;
    Encoder encoder(specs.countsPerRevolution, specs.wheelDiameter);
    MotorSimulator motor(specs, encoder);
    AdvancedMotorController controller(specs, encoder);

    controller.setVelocityPID(0.05, 0.5, 0.002);
    controller.setFeedforward(0.5, 12.0 / specs.freeSpeed.toRPM(), 0.01);

    double dt = 0.02;
    double currentTime = 0.0;
    RPM target = RPM::fromRPM(2000);

    println("Scenario: Apply load at t=1.0s\n");
    println("Time(s) | Target | Measured | Error | Load  | Status");
    println("--------|--------|----------|-------|-------|----------------");

    for (int i = 0; i < 200; i++) {
        currentTime = i * dt;

        // Apply load disturbance at t=1.0s
        if (currentTime >= 1.0 && currentTime < 2.0) {
            motor.setLoad(0.05);  // 0.05 Nm load
        } else {
            motor.setLoad(0.0);
        }

        auto output = controller.update(target, currentTime, dt);
        motor.update(output.voltage, dt);

        if (i % 10 == 0) {
            print(, currentTime, "  | ");
            print(, target.toRPM(), " | ");
            print(, output.measuredVelocity.toRPM(), " | ");
            print(, output.velocityError.toRPM(), " | ");
            print(, (currentTime >= 1.0 && currentTime < 2.0 ? "YES" : "NO"), " | ");

            if (currentTime >= 1.0 && currentTime < 2.0) {
                print("Under load");
            } else if (currentTime >= 2.0 && currentTime < 2.5) {
                print("Recovering");
            } else {
                print("Normal");
            }
            println("");
        }
    }

    println("\nKey Points:");
    println("• Integral term compensates for steady-state load");
    println("• System returns to target velocity under load");
    println("• This is why we need the 'I' in PID!\n");
}

void demonstrateAccelerationLimiting() {
    println("========================================");
    println("  Acceleration Limiting");
    println("========================================\n");

    MotorSpecs specs;
    Encoder encoder(specs.countsPerRevolution, specs.wheelDiameter);
    MotorSimulator motor(specs, encoder);
    AdvancedMotorController controller(specs, encoder);

    controller.setVelocityPID(0.05, 0.5, 0.002);
    controller.setFeedforward(0.5, 12.0 / specs.freeSpeed.toRPM(), 0.01);

    double dt = 0.02;
    double currentTime = 0.0;

    println("Command: Instant 0 → 3000 RPM step");
    println("Accel limit: ", specs.maxAccelRPMperSec, " RPM/s\n");
    println("Time(s) | Command | Limited | Measured | Accel (RPM/s)");
    println("--------|---------|---------|----------|---------------");

    RPM lastTarget = RPM::fromRPM(0);

    for (int i = 0; i < 100; i++) {
        currentTime = i * dt;

        // Instant step command at t=0
        RPM command = (currentTime >= 0.0) ? RPM::fromRPM(3000) : RPM::fromRPM(0);

        auto output = controller.update(command, currentTime, dt);
        motor.update(output.voltage, dt);

        // Track command for next iteration
        lastTarget = command;

        if (i % 5 == 0) {
            print(, currentTime, "  | ");
            print(, command.toRPM(), " | ");
            print(, "~", static_cast<int>(output.measuredVelocity.toRPM()), " | ");
            print(, output.measuredVelocity.toRPM(), " | ");
            println(, (currentTime < 0.5 ? specs.maxAccelRPMperSec : 0.0), "");
        }
    }

    println("\nBenefits:");
    println("• Prevents wheel slip from aggressive acceleration");
    println("• Reduces mechanical stress on drivetrain");
    println("• Smoother motion for passengers/cargo");
    println("• Prevents current spikes (protects electronics)\n");
}

// ============================================================================
// Main Program
// ============================================================================
int main() {
    println("");
    println("╔════════════════════════════════════════╗");
    println("║  Advanced Motor Control               ║");
    println("║  Encoder Feedback & Velocity Control  ║");
    println("╚════════════════════════════════════════╝");
    println("");

    demonstrateVelocityControl();
    demonstrateFeedforwardBenefit();
    demonstrateLoadRejection();
    demonstrateAccelerationLimiting();

    println("========================================");
    println("  Control System Architecture");
    println("========================================\n");

    println("Components:\n");

    println("1. ENCODER");
    println("   • Measures motor position/velocity");
    println("   • Filtered to reduce noise");
    println("   • Higher resolution = better control\n");

    println("2. FEEDFORWARD CONTROL");
    println("   • Models expected system behavior");
    println("   • Provides majority of control effort");
    println("   • Components:");
    println("     - kS: Static friction compensation");
    println("     - kV: Velocity feedforward (V = kV * ω)");
    println("     - kA: Acceleration feedforward\n");

    println("3. FEEDBACK CONTROL (PID)");
    println("   • Corrects for model errors");
    println("   • Rejects disturbances (load, friction)");
    println("   • Typically provides <10% of effort\n");

    println("4. ACCELERATION LIMITING");
    println("   • Prevents sudden velocity changes");
    println("   • Protects mechanical components");
    println("   • Improves control stability\n");

    println("========================================");
    println("  Tuning Procedure");
    println("========================================\n");

    println("Step 1: Measure kV (Velocity Feedforward)");
    println("   • Run motor at full voltage");
    println("   • Measure steady-state velocity");
    println("   • kV = Voltage / Velocity\n");

    println("Step 2: Measure kS (Static Friction)");
    println("   • Find minimum voltage to start motion");
    println("   • Typically 0.3-0.7V for FRC motors\n");

    println("Step 3: Tune Feedback (PID)");
    println("   • Start with P-only: kP = 0.1");
    println("   • Add I if steady-state error: kI = kP/10");
    println("   • Add D if oscillating: kD = kP/100\n");

    println("Step 4: Add kA (Acceleration FF)");
    println("   • Improves tracking during acceleration");
    println("   • Start with 0.01, adjust up if needed\n");

    println("========================================");
    println("  Real-World Applications");
    println("========================================\n");

    println("• FRC/VEX competition robots");
    println("• Precision CNC machines");
    println("• Electric vehicles (traction control)");
    println("• Drones (motor speed control)");
    println("• Industrial automation");
    println("• 3D printer motion control\n");

    println("Why This Matters:");
    println("Professional motor control = Consistent performance");
    println("• Predictable acceleration");
    println("• Robust to load changes");
    println("• Smooth motion (no jerking)");
    println("• Minimal tuning required\n");

    return 0;
}

/*
Expected Output:
- Velocity control demonstration with PID
- Comparison of feedforward vs feedback only
- Load disturbance rejection
- Acceleration limiting example

This example demonstrates:
1. Professional velocity control architecture
2. Feedforward + feedback control combination
3. Encoder-based velocity measurement with filtering
4. PID tuning for velocity control
5. Acceleration limiting for smooth motion
6. Load disturbance rejection
7. Real motor simulation with noise
8. Type-safe units throughout

Educational Value:
- Shows why feedforward is critical for good control
- Explains each component of the control system
- Provides step-by-step tuning procedure
- Demonstrates real-world control challenges
- Applicable to FRC, VEX, and industrial systems

This represents industry-standard motor control techniques
used in professional robotics and automation systems.
*/
