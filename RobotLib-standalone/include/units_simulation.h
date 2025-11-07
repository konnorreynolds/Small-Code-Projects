// ============================================================================
// units_simulation.h - Robot Physics Simulation
// ============================================================================
// Purpose: Provide physics simulation for testing robot code
//
// Features:
// - 2D physics simulation
// - Differential drive robot model
// - Obstacle collision detection
// - Sensor simulation (ultrasonic, IR, encoders)
// - Same RobotLib units throughout
//
// Usage: #define ROBOTLIB_SIMULATION before including this file
// ============================================================================

#ifndef ROBOTICS_UNITS_SIMULATION_H
#define ROBOTICS_UNITS_SIMULATION_H

#include "units_core.h"
#include "units_physics.h"
#include "units_robotics.h"

#include <vector>
#include <cmath>

namespace robotlib {
namespace simulation {

using namespace units;
using namespace robotics;

// ============================================================================
// 2D Point (for obstacles, waypoints)
// ============================================================================
struct Point2D {
    double x, y;

    Point2D(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}

    double distanceTo(const Point2D& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    Point2D operator+(const Point2D& other) const {
        return Point2D(x + other.x, y + other.y);
    }

    Point2D operator-(const Point2D& other) const {
        return Point2D(x - other.x, y - other.y);
    }

    Point2D operator*(double scalar) const {
        return Point2D(x * scalar, y * scalar);
    }
};

// ============================================================================
// Rectangle Obstacle
// ============================================================================
struct Rectangle {
    double x, y;        // Center position
    double width, height;

    Rectangle(double x_ = 0, double y_ = 0, double w = 1, double h = 1)
        : x(x_), y(y_), width(w), height(h) {}

    bool contains(double px, double py) const {
        return px >= (x - width/2) && px <= (x + width/2) &&
               py >= (y - height/2) && py <= (y + height/2);
    }

    bool intersectsCircle(double cx, double cy, double radius) const {
        // Find closest point on rectangle to circle center
        double closestX = std::max(x - width/2, std::min(cx, x + width/2));
        double closestY = std::max(y - height/2, std::min(cy, y + height/2));

        // Check distance
        double dx = cx - closestX;
        double dy = cy - closestY;
        return (dx * dx + dy * dy) <= (radius * radius);
    }
};

// ============================================================================
// Simulated Differential Drive Robot
// ============================================================================
class DifferentialDriveSimulator {
private:
    // Robot state
    double x_;           // meters
    double y_;           // meters
    double theta_;       // radians
    double wheelbase_;   // meters
    double robotRadius_; // meters (for collision)

    // Motor state
    double leftVelocity_;   // m/s
    double rightVelocity_;  // m/s

    // Encoder simulation
    long leftEncoderCount_;
    long rightEncoderCount_;
    double countsPerMeter_;

    // World
    std::vector<Rectangle> obstacles_;
    double worldWidth_;
    double worldHeight_;

    // Sensor noise
    double positionNoise_;
    double velocityNoise_;

public:
    DifferentialDriveSimulator(
        double wheelbase = 0.15,      // 15cm wheelbase
        double robotRadius = 0.10,     // 10cm radius
        double countsPerMeter = 1000,  // Encoder resolution
        double worldWidth = 4.0,       // 4m world
        double worldHeight = 3.0)      // 3m world
        : x_(0), y_(0), theta_(0)
        , wheelbase_(wheelbase)
        , robotRadius_(robotRadius)
        , leftVelocity_(0), rightVelocity_(0)
        , leftEncoderCount_(0), rightEncoderCount_(0)
        , countsPerMeter_(countsPerMeter)
        , worldWidth_(worldWidth), worldHeight_(worldHeight)
        , positionNoise_(0.001), velocityNoise_(0.01)
    {}

    // ========================================
    // Robot Control
    // ========================================

    void setWheelVelocities(double leftVel, double rightVel) {
        leftVelocity_ = leftVel;
        rightVelocity_ = rightVel;
    }

    void setMotorPWM(double leftPWM, double rightPWM) {
        // Simple model: PWM (-1 to 1) maps to velocity (-0.5 to 0.5 m/s)
        leftVelocity_ = leftPWM * 0.5;
        rightVelocity_ = rightPWM * 0.5;
    }

    void stop() {
        leftVelocity_ = 0;
        rightVelocity_ = 0;
    }

    // ========================================
    // Physics Update
    // ========================================

    void update(double dt) {
        // Differential drive kinematics
        double v = (leftVelocity_ + rightVelocity_) / 2.0;  // Linear velocity
        double omega = (rightVelocity_ - leftVelocity_) / wheelbase_;  // Angular velocity

        // Add noise for realism
        v += (std::rand() / (double)RAND_MAX - 0.5) * velocityNoise_;
        omega += (std::rand() / (double)RAND_MAX - 0.5) * velocityNoise_;

        // Update pose using simple Euler integration
        double newX = x_ + v * std::cos(theta_) * dt;
        double newY = y_ + v * std::sin(theta_) * dt;
        double newTheta = theta_ + omega * dt;

        // Check collisions
        bool collision = false;
        for (const auto& obs : obstacles_) {
            if (obs.intersectsCircle(newX, newY, robotRadius_)) {
                collision = true;
                break;
            }
        }

        // Check world bounds
        if (newX - robotRadius_ < 0 || newX + robotRadius_ > worldWidth_ ||
            newY - robotRadius_ < 0 || newY + robotRadius_ > worldHeight_) {
            collision = true;
        }

        // Only update if no collision
        if (!collision) {
            x_ = newX;
            y_ = newY;
            theta_ = newTheta;

            // Normalize angle to [-PI, PI]
            while (theta_ > M_PI) theta_ -= 2 * M_PI;
            while (theta_ < -M_PI) theta_ += 2 * M_PI;

            // Update encoders
            leftEncoderCount_ += (long)(leftVelocity_ * dt * countsPerMeter_);
            rightEncoderCount_ += (long)(rightVelocity_ * dt * countsPerMeter_);
        } else {
            // Collision! Stop the robot
            leftVelocity_ = 0;
            rightVelocity_ = 0;
        }
    }

    // ========================================
    // Sensors
    // ========================================

    // Simulate ultrasonic sensor
    double getUltrasonicDistance(double sensorAngle = 0) const {
        // Ray cast from robot position in sensor direction
        double rayAngle = theta_ + sensorAngle;
        double dx = std::cos(rayAngle);
        double dy = std::sin(rayAngle);

        double minDist = 4.0;  // Max sensor range

        // Check walls
        if (dx > 0) minDist = std::min(minDist, (worldWidth_ - x_) / dx);
        if (dx < 0) minDist = std::min(minDist, -x_ / dx);
        if (dy > 0) minDist = std::min(minDist, (worldHeight_ - y_) / dy);
        if (dy < 0) minDist = std::min(minDist, -y_ / dy);

        // Check obstacles (simplified - check center distance)
        for (const auto& obs : obstacles_) {
            Point2D obsCenter(obs.x, obs.y);
            Point2D robotPos(x_, y_);
            double dist = obsCenter.distanceTo(robotPos) - robotRadius_ -
                         std::max(obs.width, obs.height) / 2;
            if (dist > 0 && dist < minDist) {
                // Check if obstacle is in sensor direction
                double angleToObs = std::atan2(obs.y - y_, obs.x - x_);
                double angleDiff = std::abs(angleToObs - rayAngle);
                while (angleDiff > M_PI) angleDiff -= 2 * M_PI;
                if (std::abs(angleDiff) < 0.3) {  // ~17 degree cone
                    minDist = std::min(minDist, dist);
                }
            }
        }

        // Add sensor noise
        minDist += (std::rand() / (double)RAND_MAX - 0.5) * 0.02;

        return minDist;
    }

    // Simulate IR line sensor (returns true if line detected)
    bool getLineSensor(double offsetX, double offsetY) const {
        // Simple model: "line" is at y = worldHeight_ / 2
        double sensorX = x_ + offsetX * std::cos(theta_) - offsetY * std::sin(theta_);
        double sensorY = y_ + offsetX * std::sin(theta_) + offsetY * std::cos(theta_);

        double lineY = worldHeight_ / 2;
        return std::abs(sensorY - lineY) < 0.02;  // 2cm tolerance
    }

    // Get encoder counts
    long getLeftEncoder() const { return leftEncoderCount_; }
    long getRightEncoder() const { return rightEncoderCount_; }

    void resetEncoders() {
        leftEncoderCount_ = 0;
        rightEncoderCount_ = 0;
    }

    // ========================================
    // World Management
    // ========================================

    void addObstacle(const Rectangle& obs) {
        obstacles_.push_back(obs);
    }

    void clearObstacles() {
        obstacles_.clear();
    }

    void setPosition(double x, double y, double theta) {
        x_ = x;
        y_ = y;
        theta_ = theta;
    }

    // ========================================
    // Getters
    // ========================================

    double getX() const { return x_; }
    double getY() const { return y_; }
    double getTheta() const { return theta_; }
    double getWheelbase() const { return wheelbase_; }
    double getRobotRadius() const { return robotRadius_; }
    double getWorldWidth() const { return worldWidth_; }
    double getWorldHeight() const { return worldHeight_; }
    const std::vector<Rectangle>& getObstacles() const { return obstacles_; }

    double getLeftVelocity() const { return leftVelocity_; }
    double getRightVelocity() const { return rightVelocity_; }
};

} // namespace simulation
} // namespace robotlib

#endif // ROBOTICS_UNITS_SIMULATION_H
