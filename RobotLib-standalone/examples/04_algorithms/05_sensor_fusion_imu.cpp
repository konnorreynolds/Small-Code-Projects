// ============================================================================
// Example 5: IMU Sensor Fusion
// ============================================================================
// This example demonstrates how to fuse data from multiple sensors to get
// accurate orientation and position estimates. We combine:
// - Accelerometer (measures gravity and linear acceleration)
// - Gyroscope (measures angular velocity)
// - Optional: Magnetometer (measures heading)
//
// Topics covered:
// - Complementary filter for tilt angle
// - Kalman filter for improved estimates
// - Sensor noise handling
// - Bias compensation
// - Quaternions vs Euler angles (2D simplified version)
// ============================================================================

#include "../../include/units_core.h"
#include "../../include/units_physics.h"
#include "../../include/units_robotics.h"
#include "../../include/units_utilities.h"

#include <vector>
#include <cmath>
#include <random>

using namespace units;
using namespace robotics;

// ============================================================================
// Simulated IMU Sensor
// ============================================================================
class SimulatedIMU {
private:
    // True state
    Radians trueAngle_;
    RadiansPerSecond trueAngularVelocity_;

    // Noise parameters
    double accelNoise_;
    double gyroNoise_;
    double gyroBias_;

    std::default_random_engine generator_;
    std::normal_distribution<double> distribution_;

public:
    struct IMUReading {
        // Accelerometer (in g's) - measures gravity + linear acceleration
        double accelX;  // Forward/backward
        double accelY;  // Left/right
        double accelZ;  // Up/down

        // Gyroscope (rad/s) - measures angular velocity
        RadiansPerSecond gyroX;  // Roll rate
        RadiansPerSecond gyroY;  // Pitch rate
        RadiansPerSecond gyroZ;  // Yaw rate
    };

    SimulatedIMU(double accelNoise = 0.05, double gyroNoise = 0.01, double gyroBias = 0.002)
        : trueAngle_(rad(0)),
          trueAngularVelocity_(radps(0)),
          accelNoise_(accelNoise),
          gyroNoise_(gyroNoise),
          gyroBias_(gyroBias),
          distribution_(0.0, 1.0) {}

    // Update true state (simulation)
    void updateTrueState(const Radians& angle, const RadiansPerSecond& angularVel) {
        trueAngle_ = angle;
        trueAngularVelocity_ = angularVel;
    }

    // Generate noisy sensor reading
    IMUReading read() {
        IMUReading reading;

        // Simulate tilt angle using accelerometer
        // Accelerometer measures gravity vector rotated by tilt angle
        double noise = distribution_(generator_);
        double tiltAngle = trueAngle_.toRadians();

        // For 2D tilt (roll), accel measures components of gravity
        reading.accelX = std::sin(tiltAngle) + accelNoise_ * noise;
        reading.accelY = 0.0;
        reading.accelZ = std::cos(tiltAngle) + accelNoise_ * distribution_(generator_);

        // Gyroscope measures angular velocity with bias and noise
        noise = distribution_(generator_);
        reading.gyroX = radps(trueAngularVelocity_.toRadiansPerSecond() +
                             gyroBias_ + gyroNoise_ * noise);
        reading.gyroY = radps(0);
        reading.gyroZ = radps(0);

        return reading;
    }

    Radians getTrueAngle() const { return trueAngle_; }
    RadiansPerSecond getTrueAngularVelocity() const { return trueAngularVelocity_; }
};

// ============================================================================
// Complementary Filter
// ============================================================================
class ComplementaryFilterOrientation {
private:
    Radians angle_;
    double alpha_;  // Weight: 0 = trust accel, 1 = trust gyro

public:
    explicit ComplementaryFilterOrientation(double alpha = 0.98)
        : angle_(rad(0)), alpha_(alpha) {}

    // Update filter with IMU data
    Radians update(const SimulatedIMU::IMUReading& imu, double dt) {
        // Calculate angle from accelerometer (instant but noisy)
        double accelAngle = std::atan2(imu.accelX, imu.accelZ);

        // Integrate gyroscope (accurate short-term, drifts long-term)
        double gyroAngle = angle_.toRadians() + imu.gyroX.toRadiansPerSecond() * dt;

        // Complementary filter: blend both estimates
        // High alpha = trust gyro more (filters noise but allows drift)
        // Low alpha = trust accel more (prevents drift but more noise)
        double fusedAngle = alpha_ * gyroAngle + (1.0 - alpha_) * accelAngle;

        angle_ = rad(fusedAngle);
        return angle_;
    }

    Radians getAngle() const { return angle_; }
    void reset() { angle_ = rad(0); }
};

// ============================================================================
// Extended Kalman Filter for Orientation
// ============================================================================
class KalmanFilterOrientation {
private:
    // State: [angle, angular_velocity_bias]
    double angle_;
    double gyroBias_;

    // Covariance matrix (2x2)
    double P11_, P12_, P21_, P22_;

    // Process noise
    double Q_angle_;
    double Q_bias_;

    // Measurement noise
    double R_accel_;

public:
    KalmanFilterOrientation()
        : angle_(0),
          gyroBias_(0),
          P11_(1), P12_(0), P21_(0), P22_(1),
          Q_angle_(0.001),
          Q_bias_(0.003),
          R_accel_(0.03) {}

    Radians update(const SimulatedIMU::IMUReading& imu, double dt) {
        // ====================================================================
        // PREDICTION STEP
        // ====================================================================

        // State prediction: angle += (gyro - bias) * dt
        double gyroRate = imu.gyroX.toRadiansPerSecond();
        angle_ += (gyroRate - gyroBias_) * dt;
        // gyroBias_ stays constant (random walk model)

        // Covariance prediction
        double P11_temp = P11_ + dt * (P21_ + P12_) + dt * dt * P22_ + Q_angle_;
        double P12_temp = P12_ - dt * P22_;
        double P21_temp = P21_ - dt * P22_;
        double P22_temp = P22_ + Q_bias_;

        // ====================================================================
        // UPDATE STEP
        // ====================================================================

        // Measurement: angle from accelerometer
        double accelAngle = std::atan2(imu.accelX, imu.accelZ);

        // Innovation (measurement residual)
        double y = accelAngle - angle_;

        // Innovation covariance
        double S = P11_temp + R_accel_;

        // Kalman gain
        double K1 = P11_temp / S;
        double K2 = P21_temp / S;

        // State update
        angle_ += K1 * y;
        gyroBias_ += K2 * y;

        // Covariance update
        P11_ = P11_temp - K1 * P11_temp;
        P12_ = P12_temp - K1 * P12_temp;
        P21_ = P21_temp - K2 * P11_temp;
        P22_ = P22_temp - K2 * P12_temp;

        return rad(angle_);
    }

    Radians getAngle() const { return rad(angle_); }
    double getEstimatedBias() const { return gyroBias_; }

    void reset() {
        angle_ = 0;
        gyroBias_ = 0;
        P11_ = 1; P12_ = 0; P21_ = 0; P22_ = 1;
    }
};

// ============================================================================
// Simple Gyro Integration (for comparison)
// ============================================================================
class SimpleGyroIntegration {
private:
    Radians angle_;

public:
    SimpleGyroIntegration() : angle_(rad(0)) {}

    Radians update(const SimulatedIMU::IMUReading& imu, double dt) {
        // Just integrate gyroscope (will drift over time)
        angle_ = rad(angle_.toRadians() + imu.gyroX.toRadiansPerSecond() * dt);
        return angle_;
    }

    Radians getAngle() const { return angle_; }
    void reset() { angle_ = rad(0); }
};

// ============================================================================
// Simple Accelerometer Tilt (for comparison)
// ============================================================================
class SimpleAccelTilt {
public:
    static Radians calculate(const SimulatedIMU::IMUReading& imu) {
        // Calculate tilt from accelerometer (noisy but no drift)
        return rad(std::atan2(imu.accelX, imu.accelZ));
    }
};

// ============================================================================
// Test Scenario
// ============================================================================
class TiltTestScenario {
private:
    double time_;
    Radians currentAngle_;
    RadiansPerSecond currentAngularVel_;

public:
    TiltTestScenario() : time_(0), currentAngle_(rad(0)), currentAngularVel_(radps(0)) {}

    struct State {
        Radians angle;
        RadiansPerSecond angularVelocity;
    };

    // Generate test motion profile
    State update(double dt) {
        time_ += dt;

        // Create interesting motion profile:
        // 0-2s: Tilt to 30 degrees
        // 2-4s: Hold at 30 degrees
        // 4-6s: Tilt back to 0
        // 6-8s: Tilt to -20 degrees
        // 8-10s: Return to 0

        if (time_ < 2.0) {
            // Ramp up to 30°
            currentAngle_ = Radians::fromDegrees(15.0 * time_);
            currentAngularVel_ = radps(Radians::fromDegrees(15).toRadians());
        } else if (time_ < 4.0) {
            // Hold at 30°
            currentAngle_ = Radians::fromDegrees(30);
            currentAngularVel_ = radps(0);
        } else if (time_ < 6.0) {
            // Return to 0°
            currentAngle_ = Radians::fromDegrees(30 - 15.0 * (time_ - 4.0));
            currentAngularVel_ = radps(-deg(15).toRadians());
        } else if (time_ < 8.0) {
            // Tilt to -20°
            currentAngle_ = Radians::fromDegrees(-10.0 * (time_ - 6.0));
            currentAngularVel_ = radps(-deg(10).toRadians());
        } else {
            // Return to 0°
            currentAngle_ = Radians::fromDegrees(-20 + 10.0 * (time_ - 8.0));
            currentAngularVel_ = radps(Radians::fromDegrees(10).toRadians());
        }

        return {currentAngle_, currentAngularVel_};
    }

    void reset() {
        time_ = 0;
        currentAngle_ = rad(0);
        currentAngularVel_ = radps(0);
    }
};

// ============================================================================
// Main Program
// ============================================================================
int main() {
    println("========================================");
    println("  IMU Sensor Fusion Example");
    println("========================================\n");

    // Setup
    const double dt = 0.02;  // 50 Hz update rate
    const double duration = 10.0;  // 10 second test

    SimulatedIMU imu(0.05, 0.01, 0.002);  // accel noise, gyro noise, gyro bias
    TiltTestScenario scenario;

    // Create different filters for comparison
    SimpleGyroIntegration gyroOnly;
    ComplementaryFilterOrientation compFilter(0.98);
    KalmanFilterOrientation kalmanFilter;

    println("Simulation Parameters:");
    println("  Sample rate: ", 1.0/dt, " Hz");
    println("  Duration: ", duration, " seconds");
    println("  Accelerometer noise: 0.05 g");
    println("  Gyroscope noise: 0.01 rad/s");
    println("  Gyroscope bias: 0.002 rad/s\n");

    println("Running sensor fusion comparison...\n");

    print(, );
    println("Time | True | Accel | Gyro | Comp.Filt | Kalman | Errors (°)");
    println("(s)  | (°)  |  (°)  | (°)  |    (°)    |  (°)   | Acl|Gyr|Cmp|Kal");
    println("-----+------+-------+------+-----------+--------+----+---+---+---");

    // Statistics
    double accelErrorSum = 0, gyroErrorSum = 0, compErrorSum = 0, kalmanErrorSum = 0;
    int sampleCount = 0;

    // Run simulation
    for (double t = 0; t <= duration; t += dt) {
        // Update true state
        auto state = scenario.update(dt);
        imu.updateTrueState(state.angle, state.angularVelocity);

        // Read sensors
        auto reading = imu.read();

        // Update all filters
        auto accelAngle = SimpleAccelTilt::calculate(reading);
        auto gyroAngle = gyroOnly.update(reading, dt);
        auto compAngle = compFilter.update(reading, dt);
        auto kalmanAngle = kalmanFilter.update(reading, dt);

        // Print results every 0.2 seconds
        if (static_cast<int>(t / 0.2) != static_cast<int>((t - dt) / 0.2)) {
            double trueAngleDeg = state.angle.toDegrees();
            double accelAngleDeg = accelAngle.toDegrees();
            double gyroAngleDeg = gyroAngle.toDegrees();
            double compAngleDeg = compAngle.toDegrees();
            double kalmanAngleDeg = kalmanAngle.toDegrees();

            double accelError = std::abs(accelAngleDeg - trueAngleDeg);
            double gyroError = std::abs(gyroAngleDeg - trueAngleDeg);
            double compError = std::abs(compAngleDeg - trueAngleDeg);
            double kalmanError = std::abs(kalmanAngleDeg - trueAngleDeg);

            print(, t, " | ");
            print(, trueAngleDeg, " | ");
            print(, accelAngleDeg, " | ");
            print(, gyroAngleDeg, " | ");
            print(, compAngleDeg, " | ");
            print(, kalmanAngleDeg, " | ");
            print(, accelError, "|");
            print(, gyroError, "|");
            print(, compError, "|");
            println(, kalmanError, "");
        }

        // Accumulate errors for statistics
        accelErrorSum += std::abs(accelAngle.toDegrees() - state.angle.toDegrees());
        gyroErrorSum += std::abs(gyroAngle.toDegrees() - state.angle.toDegrees());
        compErrorSum += std::abs(compAngle.toDegrees() - state.angle.toDegrees());
        kalmanErrorSum += std::abs(kalmanAngle.toDegrees() - state.angle.toDegrees());
        sampleCount++;
    }

    // Print statistics
    println("\n========================================");
    println("  Performance Statistics");
    println("========================================\n");

    print(, );
    println("Mean Absolute Error:");
    println("  Accelerometer only: ", accelErrorSum / sampleCount, "°");
    println("  Gyroscope only:     ", gyroErrorSum / sampleCount, "°");
    println("  Complementary filt: ", compErrorSum / sampleCount, "°");
    println("  Kalman filter:      ", kalmanErrorSum / sampleCount, "°\n");

    print("Estimated gyro bias (Kalman): "
              , kalmanFilter.getEstimatedBias()
              , " rad/s (true: 0.002)\n\n");

    // Summary
    println("========================================");
    println("  Key Insights");
    println("========================================\n");

    println("Accelerometer-only:");
    println("  ✓ No drift over time");
    println("  ✗ Very noisy");
    println("  ✗ Affected by linear acceleration");
    println("  Use case: Static tilt measurement\n");

    println("Gyroscope-only:");
    println("  ✓ Smooth, low noise");
    println("  ✗ Drifts over time due to bias");
    println("  ✗ Integration amplifies errors");
    println("  Use case: Short-term orientation tracking\n");

    println("Complementary Filter:");
    println("  ✓ Simple to implement");
    println("  ✓ Low computational cost");
    println("  ✓ Combines best of both sensors");
    println("  ✓ No drift, reduced noise");
    println("  Use case: Most embedded applications\n");

    println("Kalman Filter:");
    println("  ✓ Optimal fusion (statistically)");
    println("  ✓ Estimates sensor bias");
    println("  ✓ Best accuracy");
    println("  ~ More complex implementation");
    println("  ~ Higher computational cost");
    println("  Use case: High-precision applications\n");

    println("Recommendations:");
    println("• For hobby robots: Use complementary filter (α ≈ 0.98)");
    println("• For drones/aircraft: Use Kalman or extended Kalman filter");
    println("• Always calibrate sensors before use");
    println("• Add magnetometer for absolute heading (prevents yaw drift)");
    println("• Consider madgwick/mahony filters for full 3D orientation\n");

    return 0;
}

/*
This example demonstrates:
1. IMU sensor modeling with realistic noise and bias
2. Four different orientation estimation approaches:
   - Accelerometer-only (noisy but no drift)
   - Gyroscope integration (smooth but drifts)
   - Complementary filter (simple fusion)
   - Kalman filter (optimal fusion with bias estimation)
3. Quantitative performance comparison
4. Real-world tradeoffs between approaches

Extensions for your robot:
- Add magnetometer for heading estimation
- Implement full 3D orientation (quaternions)
- Add GPS fusion for position estimation
- Tune filter parameters for your sensors
- Add automatic bias calibration routine

Related topics:
- Madgwick filter (efficient quaternion-based fusion)
- Mahony filter (alternative to Madgwick)
- Extended Kalman Filter (EKF) for nonlinear systems
- Unscented Kalman Filter (UKF) for highly nonlinear systems
*/
