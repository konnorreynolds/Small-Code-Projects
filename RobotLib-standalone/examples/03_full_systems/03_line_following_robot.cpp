// ============================================================================
// Example 3: Line Following Robot with Sensor Filtering
// ============================================================================
// This example demonstrates how to build a line-following robot using:
// - Multiple IR sensors for line detection
// - Sensor filtering to reduce noise
// - PID control for smooth path tracking
// - Motor control with differential drive
//
// Topics covered:
// - Sensor array processing
// - Weighted line position calculation
// - Moving average filtering
// - PID tuning for line following
// - Differential drive control
// ============================================================================

#include "../../include/units_core.h"
#include "../../include/units_physics.h"
#include "../../include/units_robotics.h"
#include "../../include/units_utilities.h"

#include <vector>
#include <array>

using namespace units;
using namespace robotics;

// ============================================================================
// Configuration
// ============================================================================
struct RobotConfig {
    // Physical dimensions
    Meters wheelbase = m(0.15);            // 15 cm between wheels
    Meters wheelDiameter = m(0.065);       // 6.5 cm wheel diameter

    // Motion limits
    MetersPerSecond baseSpeed = mps(0.3);  // Base forward speed
    MetersPerSecond maxSpeed = mps(0.5);   // Maximum speed
    MetersPerSecond minSpeed = mps(0.1);   // Minimum speed

    // Sensor configuration
    static constexpr size_t NUM_SENSORS = 5;
    Meters sensorSpacing = m(0.01);        // 1 cm between sensors

    // PID gains for line following
    double kP = 2.5;    // Proportional gain
    double kI = 0.0;    // Integral gain (often zero for line following)
    double kD = 1.2;    // Derivative gain

    // Filtering
    static constexpr size_t FILTER_SIZE = 3;  // Moving average window
};

// ============================================================================
// Sensor Array Manager
// ============================================================================
class SensorArray {
private:
    static constexpr size_t NUM_SENSORS = RobotConfig::NUM_SENSORS;
    std::array<double, NUM_SENSORS> rawValues_;
    std::array<MovingAverageFilter<RobotConfig::FILTER_SIZE>, NUM_SENSORS> filters_;

    // Calibration values
    std::array<double, NUM_SENSORS> whiteCalibration_;
    std::array<double, NUM_SENSORS> blackCalibration_;
    bool calibrated_;

public:
    SensorArray() : calibrated_(false) {
        // Initialize with default values
        rawValues_.fill(0.0);
        whiteCalibration_.fill(0.0);
        blackCalibration_.fill(1.0);
    }

    // Calibrate sensors on white and black surfaces
    void calibrateWhite(const std::array<double, NUM_SENSORS>& readings) {
        whiteCalibration_ = readings;
    }

    void calibrateBlack(const std::array<double, NUM_SENSORS>& readings) {
        blackCalibration_ = readings;
        calibrated_ = true;
    }

    // Read and filter sensor values
    void update(const std::array<double, NUM_SENSORS>& readings) {
        for (size_t i = 0; i < NUM_SENSORS; i++) {
            // Apply calibration if available
            double normalized = readings[i];
            if (calibrated_) {
                // Map to 0.0 (white) to 1.0 (black)
                double range = blackCalibration_[i] - whiteCalibration_[i];
                if (std::abs(range) > 0.001) {
                    normalized = (readings[i] - whiteCalibration_[i]) / range;
                    normalized = numerical::clamp(normalized, 0.0, 1.0);
                }
            }

            // Apply moving average filter
            rawValues_[i] = filters_[i].update(normalized);
        }
    }

    // Get filtered sensor values (0.0 = white, 1.0 = black)
    const std::array<double, NUM_SENSORS>& getValues() const {
        return rawValues_;
    }

    // Calculate weighted line position (-1.0 to +1.0)
    // -1.0 = line far left, 0.0 = centered, +1.0 = line far right
    double getLinePosition() const {
        double weightedSum = 0.0;
        double totalWeight = 0.0;

        // Weight each sensor by its position
        for (size_t i = 0; i < NUM_SENSORS; i++) {
            double position = static_cast<double>(i) - (NUM_SENSORS - 1) / 2.0;
            weightedSum += position * rawValues_[i];
            totalWeight += rawValues_[i];
        }

        // Avoid division by zero
        if (totalWeight < 0.01) {
            return 0.0;  // No line detected
        }

        // Normalize to -1.0 to +1.0 range
        double normalizedPosition = weightedSum / totalWeight;
        normalizedPosition /= ((NUM_SENSORS - 1) / 2.0);

        return numerical::clamp(normalizedPosition, -1.0, 1.0);
    }

    // Check if line is lost (all sensors see white)
    bool isLineLost() const {
        double sum = 0.0;
        for (double val : rawValues_) {
            sum += val;
        }
        return sum < 0.5;  // Threshold: less than half a sensor on black
    }

    // Check if at intersection (all sensors see black)
    bool isAtIntersection() const {
        double sum = 0.0;
        for (double val : rawValues_) {
            sum += val;
        }
        return sum > (NUM_SENSORS - 0.5);  // Almost all sensors on black
    }
};

// ============================================================================
// Line Following Controller
// ============================================================================
class LineFollowingRobot {
private:
    RobotConfig config_;
    SensorArray sensors_;
    PIDController pid_;

    // State
    MetersPerSecond currentSpeed_;
    double lastLinePosition_;
    bool lineLost_;

public:
    LineFollowingRobot(const RobotConfig& config = RobotConfig())
        : config_(config),
          pid_(config.kP, config.kI, config.kD),
          currentSpeed_(config.baseSpeed),
          lastLinePosition_(0.0),
          lineLost_(false) {

        // Set PID output limits (steering adjustment)
        PIDController::Gains gains = pid_.getGains();
        gains.outputMin = -1.0;  // Maximum left turn
        gains.outputMax = 1.0;   // Maximum right turn
        pid_.setGains(gains);
    }

    // Calibration methods
    void calibrateOnWhite(const std::array<double, 5>& readings) {
        sensors_.calibrateWhite(readings);
    }

    void calibrateOnBlack(const std::array<double, 5>& readings) {
        sensors_.calibrateBlack(readings);
    }

    // Main control loop
    struct MotorCommands {
        RPM leftMotor;
        RPM rightMotor;
        double linePosition;
        double steering;
        bool lineLost;
    };

    MotorCommands update(const std::array<double, 5>& sensorReadings, double dt) {
        // Update sensor array with filtering
        sensors_.update(sensorReadings);

        // Get line position
        double linePosition = sensors_.getLinePosition();

        // Check for special conditions
        lineLost_ = sensors_.isLineLost();
        bool atIntersection = sensors_.isAtIntersection();
        (void)atIntersection;  // Reserved for future use

        if (lineLost_) {
            // Line lost - use last known position
            linePosition = lastLinePosition_;
        } else {
            lastLinePosition_ = linePosition;
        }

        // Calculate steering correction using PID
        // Error is negative when line is left, positive when right
        double error = linePosition;
        double steering = pid_.calculate(error, dt);

        // Adjust speed based on steering (slow down for sharp turns)
        double speedFactor = 1.0 - std::abs(steering) * 0.5;
        MetersPerSecond adjustedSpeed = config_.baseSpeed * speedFactor;
        adjustedSpeed = MetersPerSecond::fromMetersPerSecond(
            numerical::clamp(adjustedSpeed.toMetersPerSecond(),
                            config_.minSpeed.toMetersPerSecond(),
                            config_.maxSpeed.toMetersPerSecond())
        );

        // Convert steering to angular velocity
        // Steering of ±1.0 corresponds to maximum turn rate
        double maxTurnRate = 2.0;  // rad/s
        RadiansPerSecond angularVel = radps(steering * maxTurnRate);

        // Calculate differential drive velocities
        auto drive = DifferentialDrive::fromTwist(
            adjustedSpeed,
            angularVel,
            config_.wheelbase
        );

        // Convert to motor RPM
        RPM leftRPM = MotorController::velocityToRPM(
            drive.leftVelocity, config_.wheelDiameter
        );
        RPM rightRPM = MotorController::velocityToRPM(
            drive.rightVelocity, config_.wheelDiameter
        );

        return {leftRPM, rightRPM, linePosition, steering, lineLost_};
    }

    void reset() {
        pid_.reset();
        lastLinePosition_ = 0.0;
        lineLost_ = false;
    }

    const SensorArray& getSensors() const { return sensors_; }
};

// ============================================================================
// Simulation
// ============================================================================
class LineFollowingSimulation {
private:
    double robotX_;
    double robotY_;
    double robotHeading_;

    // Simulated line path (sine wave)
    double getLineX(double y) const {
        return 0.3 * std::sin(y * 0.5);  // Oscillating line
    }

public:
    LineFollowingSimulation() : robotX_(0), robotY_(0), robotHeading_(constants::PI / 2) {}

    // Generate simulated sensor readings based on robot position
    std::array<double, 5> readSensors(double sensorSpacing) const {
        std::array<double, 5> readings;

        // Calculate sensor positions relative to robot
        double sensorY = robotY_ + 0.05;  // Sensors 5cm ahead

        for (size_t i = 0; i < 5; i++) {
            // Sensor position in robot frame
            double sensorOffsetX = (static_cast<double>(i) - 2.0) * sensorSpacing;

            // Transform to global frame
            double globalX = robotX_ + sensorOffsetX * std::cos(robotHeading_);

            // Calculate distance from sensor to line
            double lineX = getLineX(sensorY);
            double distanceToLine = std::abs(globalX - lineX);

            // Convert to sensor reading (0.0 = white, 1.0 = black)
            // Line width is ~2 cm
            double lineWidth = 0.02;
            readings[i] = distanceToLine < lineWidth ? 1.0 : 0.0;

            // Add some Gaussian-like smoothing
            if (distanceToLine < lineWidth * 2) {
                readings[i] = std::max(0.0, 1.0 - distanceToLine / lineWidth);
            }
        }

        return readings;
    }

    // Update robot position based on motor commands
    void update(const RPM& leftRPM, const RPM& rightRPM,
                const Meters& wheelDiameter, const Meters& wheelbase, double dt) {
        // Convert RPM to velocities
        auto leftVel = MotorController::rpmToVelocity(leftRPM, wheelDiameter);
        auto rightVel = MotorController::rpmToVelocity(rightRPM, wheelDiameter);

        // Calculate robot velocities
        double v = (leftVel.toMetersPerSecond() + rightVel.toMetersPerSecond()) / 2.0;
        double w = (rightVel.toMetersPerSecond() - leftVel.toMetersPerSecond()) /
                   wheelbase.toMeters();

        // Update position and heading
        robotX_ += v * std::cos(robotHeading_) * dt;
        robotY_ += v * std::sin(robotHeading_) * dt;
        robotHeading_ += w * dt;
    }

    double getX() const { return robotX_; }
    double getY() const { return robotY_; }
    double getHeading() const { return robotHeading_; }
};

// ============================================================================
// Main Program
// ============================================================================
int main() {
    println("========================================");
    println("  Line Following Robot Example");
    println("========================================\n");

    // Create robot with default configuration
    RobotConfig config;
    LineFollowingRobot robot(config);
    LineFollowingSimulation sim;

    // Calibration phase
    println("Calibration Phase:");
    println("------------------");
    std::array<double, 5> whiteReadings = {0.1, 0.1, 0.1, 0.1, 0.1};
    std::array<double, 5> blackReadings = {0.9, 0.9, 0.9, 0.9, 0.9};
    robot.calibrateOnWhite(whiteReadings);
    robot.calibrateOnBlack(blackReadings);
    println("✓ Calibrated on white surface");
    println("✓ Calibrated on black surface\n");

    // Simulation parameters
    const double dt = 0.02;  // 20ms update rate (50 Hz)
    const double duration = 5.0;  // 5 second simulation

    println("Running simulation (", duration, " seconds)...");
    print("Robot config: wheelbase=" ,  config.wheelbase.toCentimeters() , "cm, ");

    print(, );
    println("Time(s) | Sensors [L->R] | LinePos | Steer | Left RPM | Right RPM | Position");
    println("--------|----------------|---------|-------|----------|-----------|----------");

    // Run simulation
    for (double t = 0; t <= duration; t += dt) {
        // Read sensors
        auto sensorReadings = sim.readSensors(config.sensorSpacing.toMeters());

        // Update controller
        auto commands = robot.update(sensorReadings, dt);

        // Update simulation
        sim.update(commands.leftMotor, commands.rightMotor,
                  config.wheelDiameter, config.wheelbase, dt);

        // Print status every 0.1 seconds
        if (static_cast<int>(t / 0.1) != static_cast<int>((t - dt) / 0.1)) {
            print(, t, "  | ");

            // Print sensor array
            for (double val : robot.getSensors().getValues()) {
                print((val > 0.5 ? "█" : "·"));
            }
            print(" | ");

            print(, commands.linePosition, " | ");
            print(, commands.steering, " | ");
            print(, commands.leftMotor.toRPM(), " | ");
            print(, commands.rightMotor.toRPM(), " | ");
            print("(" , sim.getX() , ", ");

            if (commands.lineLost) {
                print(" [LOST]");
            }

            println("");
        }
    }

    println("\n========================================");
    println("  Demonstration Complete!");
    println("========================================\n");

    println("Key Learnings:");
    println("1. Sensor Filtering: Moving average reduces noise");
    println("2. Weighted Position: Center of mass algorithm finds line");
    println("3. PID Control: Smooth corrections without oscillation");
    println("4. Speed Adaptation: Slow down for sharp turns");
    println("5. Type Safety: All units verified at compile-time\n");

    println("Tuning Tips:");
    println("• Increase kP for faster response to line position");
    println("• Increase kD to reduce oscillation/overshoot");
    println("• kI usually not needed for line following");
    println("• Reduce base speed if robot oscillates");
    println("• Increase filter size if sensors are very noisy\n");

    return 0;
}

/*
Expected output:
- Sensor array visualization showing which sensors see the line
- Line position ranging from -1.0 (left) to +1.0 (right)
- Steering correction from PID controller
- Motor RPM commands for left and right wheels
- Robot position tracking over time

This example demonstrates:
1. Real-world sensor processing with noise filtering
2. Weighted line position calculation
3. PID control for smooth path tracking
4. Differential drive motor control
5. Type-safe units throughout the system
6. Modular design for easy hardware integration
*/
