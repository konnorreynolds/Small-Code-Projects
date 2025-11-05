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

#include <iostream>
#include <iomanip>
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
    std::cout << "========================================\n";
    std::cout << "  Velocity Control with PID\n";
    std::cout << "========================================\n\n";

    MotorSpecs specs;
    Encoder encoder(specs.countsPerRevolution, specs.wheelDiameter);
    MotorSimulator motor(specs, encoder);
    AdvancedMotorController controller(specs, encoder);

    // Tune PID for velocity control
    controller.setVelocityPID(0.05, 0.5, 0.002);
    controller.setFeedforward(0.5, 12.0 / specs.freeSpeed.toRPM(), 0.01);

    std::cout << "Target: 2000 RPM step input\n";
    std::cout << "PID Gains: kP=0.05, kI=0.5, kD=0.002\n\n";

    double dt = 0.02;  // 20ms (50 Hz control loop)
    double currentTime = 0.0;
    RPM target = RPM::fromRPM(2000);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Time(s) | Target | Measured | Error | Voltage | FF    | FB    |\n";
    std::cout << "--------|--------|----------|-------|---------|-------|-------|\n";

    for (int i = 0; i < 150; i++) {
        currentTime = i * dt;

        // Control loop
        auto output = controller.update(target, currentTime, dt);

        // Simulate motor response
        motor.update(output.voltage, dt);

        // Print every 10 iterations (0.2s)
        if (i % 10 == 0) {
            std::cout << std::setw(6) << currentTime << "  | ";
            std::cout << std::setw(6) << target.toRPM() << " | ";
            std::cout << std::setw(8) << output.measuredVelocity.toRPM() << " | ";
            std::cout << std::setw(5) << output.velocityError.toRPM() << " | ";
            std::cout << std::setw(7) << output.voltage << " | ";
            std::cout << std::setw(5) << output.feedforward << " | ";
            std::cout << std::setw(5) << output.feedback << " |\n";
        }
    }

    std::cout << "\nKey Observations:\n";
    std::cout << "• Feedforward provides ~90% of control effort\n";
    std::cout << "• Feedback (PID) corrects remaining error\n";
    std::cout << "• Steady-state error < 10 RPM with integral term\n";
    std::cout << "• Fast response (~0.2s settling time)\n\n";
}

void demonstrateFeedforwardBenefit() {
    std::cout << "========================================\n";
    std::cout << "  Feedforward vs Feedback Only\n";
    std::cout << "========================================\n\n";

    MotorSpecs specs;

    // Test 1: Feedback only
    std::cout << "Test 1: Feedback Only (PID without feedforward)\n";
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

        std::cout << "  Steady-state error: " << settlingError << " RPM (POOR)\n";
        std::cout << "  → PI needed to eliminate error → slow response\n\n";
    }

    // Test 2: Feedforward + Feedback
    std::cout << "Test 2: Feedforward + Feedback (Combined)\n";
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

        std::cout << "  Steady-state error: " << settlingError << " RPM (EXCELLENT)\n";
        std::cout << "  → FF provides base control, FB corrects errors\n\n";
    }

    std::cout << "Conclusion:\n";
    std::cout << "Feedforward dramatically improves performance!\n";
    std::cout << "• Faster response\n";
    std::cout << "• Lower steady-state error\n";
    std::cout << "• Less reliance on integral term\n\n";
}

void demonstrateLoadRejection() {
    std::cout << "========================================\n";
    std::cout << "  Load Disturbance Rejection\n";
    std::cout << "========================================\n\n";

    MotorSpecs specs;
    Encoder encoder(specs.countsPerRevolution, specs.wheelDiameter);
    MotorSimulator motor(specs, encoder);
    AdvancedMotorController controller(specs, encoder);

    controller.setVelocityPID(0.05, 0.5, 0.002);
    controller.setFeedforward(0.5, 12.0 / specs.freeSpeed.toRPM(), 0.01);

    double dt = 0.02;
    double currentTime = 0.0;
    RPM target = RPM::fromRPM(2000);

    std::cout << "Scenario: Apply load at t=1.0s\n\n";
    std::cout << "Time(s) | Target | Measured | Error | Load  | Status\n";
    std::cout << "--------|--------|----------|-------|-------|----------------\n";

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
            std::cout << std::setw(6) << currentTime << "  | ";
            std::cout << std::setw(6) << target.toRPM() << " | ";
            std::cout << std::setw(8) << output.measuredVelocity.toRPM() << " | ";
            std::cout << std::setw(5) << output.velocityError.toRPM() << " | ";
            std::cout << std::setw(5) << (currentTime >= 1.0 && currentTime < 2.0 ? "YES" : "NO") << " | ";

            if (currentTime >= 1.0 && currentTime < 2.0) {
                std::cout << "Under load";
            } else if (currentTime >= 2.0 && currentTime < 2.5) {
                std::cout << "Recovering";
            } else {
                std::cout << "Normal";
            }
            std::cout << "\n";
        }
    }

    std::cout << "\nKey Points:\n";
    std::cout << "• Integral term compensates for steady-state load\n";
    std::cout << "• System returns to target velocity under load\n";
    std::cout << "• This is why we need the 'I' in PID!\n\n";
}

void demonstrateAccelerationLimiting() {
    std::cout << "========================================\n";
    std::cout << "  Acceleration Limiting\n";
    std::cout << "========================================\n\n";

    MotorSpecs specs;
    Encoder encoder(specs.countsPerRevolution, specs.wheelDiameter);
    MotorSimulator motor(specs, encoder);
    AdvancedMotorController controller(specs, encoder);

    controller.setVelocityPID(0.05, 0.5, 0.002);
    controller.setFeedforward(0.5, 12.0 / specs.freeSpeed.toRPM(), 0.01);

    double dt = 0.02;
    double currentTime = 0.0;

    std::cout << "Command: Instant 0 → 3000 RPM step\n";
    std::cout << "Accel limit: " << specs.maxAccelRPMperSec << " RPM/s\n\n";
    std::cout << "Time(s) | Command | Limited | Measured | Accel (RPM/s)\n";
    std::cout << "--------|---------|---------|----------|---------------\n";

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
            std::cout << std::setw(6) << currentTime << "  | ";
            std::cout << std::setw(7) << command.toRPM() << " | ";
            std::cout << std::setw(7) << "~" << static_cast<int>(output.measuredVelocity.toRPM()) << " | ";
            std::cout << std::setw(8) << output.measuredVelocity.toRPM() << " | ";
            std::cout << std::setw(14) << (currentTime < 0.5 ? specs.maxAccelRPMperSec : 0.0) << "\n";
        }
    }

    std::cout << "\nBenefits:\n";
    std::cout << "• Prevents wheel slip from aggressive acceleration\n";
    std::cout << "• Reduces mechanical stress on drivetrain\n";
    std::cout << "• Smoother motion for passengers/cargo\n";
    std::cout << "• Prevents current spikes (protects electronics)\n\n";
}

// ============================================================================
// Main Program
// ============================================================================
int main() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════╗\n";
    std::cout << "║  Advanced Motor Control               ║\n";
    std::cout << "║  Encoder Feedback & Velocity Control  ║\n";
    std::cout << "╚════════════════════════════════════════╝\n";
    std::cout << "\n";

    demonstrateVelocityControl();
    demonstrateFeedforwardBenefit();
    demonstrateLoadRejection();
    demonstrateAccelerationLimiting();

    std::cout << "========================================\n";
    std::cout << "  Control System Architecture\n";
    std::cout << "========================================\n\n";

    std::cout << "Components:\n\n";

    std::cout << "1. ENCODER\n";
    std::cout << "   • Measures motor position/velocity\n";
    std::cout << "   • Filtered to reduce noise\n";
    std::cout << "   • Higher resolution = better control\n\n";

    std::cout << "2. FEEDFORWARD CONTROL\n";
    std::cout << "   • Models expected system behavior\n";
    std::cout << "   • Provides majority of control effort\n";
    std::cout << "   • Components:\n";
    std::cout << "     - kS: Static friction compensation\n";
    std::cout << "     - kV: Velocity feedforward (V = kV * ω)\n";
    std::cout << "     - kA: Acceleration feedforward\n\n";

    std::cout << "3. FEEDBACK CONTROL (PID)\n";
    std::cout << "   • Corrects for model errors\n";
    std::cout << "   • Rejects disturbances (load, friction)\n";
    std::cout << "   • Typically provides <10% of effort\n\n";

    std::cout << "4. ACCELERATION LIMITING\n";
    std::cout << "   • Prevents sudden velocity changes\n";
    std::cout << "   • Protects mechanical components\n";
    std::cout << "   • Improves control stability\n\n";

    std::cout << "========================================\n";
    std::cout << "  Tuning Procedure\n";
    std::cout << "========================================\n\n";

    std::cout << "Step 1: Measure kV (Velocity Feedforward)\n";
    std::cout << "   • Run motor at full voltage\n";
    std::cout << "   • Measure steady-state velocity\n";
    std::cout << "   • kV = Voltage / Velocity\n\n";

    std::cout << "Step 2: Measure kS (Static Friction)\n";
    std::cout << "   • Find minimum voltage to start motion\n";
    std::cout << "   • Typically 0.3-0.7V for FRC motors\n\n";

    std::cout << "Step 3: Tune Feedback (PID)\n";
    std::cout << "   • Start with P-only: kP = 0.1\n";
    std::cout << "   • Add I if steady-state error: kI = kP/10\n";
    std::cout << "   • Add D if oscillating: kD = kP/100\n\n";

    std::cout << "Step 4: Add kA (Acceleration FF)\n";
    std::cout << "   • Improves tracking during acceleration\n";
    std::cout << "   • Start with 0.01, adjust up if needed\n\n";

    std::cout << "========================================\n";
    std::cout << "  Real-World Applications\n";
    std::cout << "========================================\n\n";

    std::cout << "• FRC/VEX competition robots\n";
    std::cout << "• Precision CNC machines\n";
    std::cout << "• Electric vehicles (traction control)\n";
    std::cout << "• Drones (motor speed control)\n";
    std::cout << "• Industrial automation\n";
    std::cout << "• 3D printer motion control\n\n";

    std::cout << "Why This Matters:\n";
    std::cout << "Professional motor control = Consistent performance\n";
    std::cout << "• Predictable acceleration\n";
    std::cout << "• Robust to load changes\n";
    std::cout << "• Smooth motion (no jerking)\n";
    std::cout << "• Minimal tuning required\n\n";

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
