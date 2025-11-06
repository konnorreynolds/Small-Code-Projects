#include "include/RobotLib.h"
#include <iostream>

int main() {
    using namespace units;

    std::cout << "Testing RobotLib v" << robotlib::VERSION_STRING << std::endl;
    std::cout << "Built: " << robotlib::BUILD_DATE << " " << robotlib::BUILD_TIME << std::endl;
    std::cout << std::endl;

    // Test 1: Basic units
    std::cout << "Test 1: Basic Units" << std::endl;
    auto distance = m(10.0);
    auto velocity = mps(2.5);
    auto timeValue = distance.toMeters() / velocity.toMetersPerSecond();
    std::cout << "  ✓ Distance: " << distance.toMeters() << " m" << std::endl;
    std::cout << "  ✓ Velocity: " << velocity.toMetersPerSecond() << " m/s" << std::endl;
    std::cout << "  ✓ Time: " << timeValue << " s" << std::endl;

    // Test 2: PID Controller
    std::cout << "\nTest 2: PID Controller" << std::endl;
    units::robotics::PIDController pid(1.0, 0.1, 0.05);
    double output = pid.calculate(10.0, 5.0, 0.02);
    std::cout << "  ✓ PID output: " << output << std::endl;

    // Test 3: Vector Math
    std::cout << "\nTest 3: Vector Math" << std::endl;
    robotics::Vec2D v1(3.0, 4.0);
    double magnitude = v1.magnitude();
    std::cout << "  ✓ Vector magnitude: " << magnitude << std::endl;

    // Test 4: Quaternions
    std::cout << "\nTest 4: Quaternions" << std::endl;
    spatial::Quaternion q = spatial::Quaternion::fromAxisAngle(spatial::Vec3D(0, 0, 1), M_PI/4);
    std::cout << "  ✓ Quaternion created: w=" << q.w << std::endl;

    // Test 5: Kalman Filter
    std::cout << "\nTest 5: Extended Kalman Filter" << std::endl;
    estimation::EKF2DPositionVelocity ekf(0.01, 0.1);
    std::cout << "  ✓ EKF initialized" << std::endl;

    // Test 6: Path Planning
    std::cout << "\nTest 6: A* Path Planner" << std::endl;
    planning::AStarPlanner planner(10, 10);
    std::cout << "  ✓ A* planner created" << std::endl;

    // Test 7: Fluent API
    std::cout << "\nTest 7: Fluent API" << std::endl;
    robotlib::Arm myArm = robotlib::Arm()
        .withPID(1.5, 0.1, 0.05)
        .withLimits(deg(-90), deg(90));
    std::cout << "  ✓ Arm created with fluent API" << std::endl;

    // Test 8: Utilities
    std::cout << "\nTest 8: Utilities" << std::endl;
    utilities::DifferentialDrive drive = utilities::DifferentialDrive::fromTwist(
        mps(1.0),
        rad(0.5) / s(1.0),
        m(0.5)
    );
    std::cout << "  ✓ Differential drive kinematics: left="
              << drive.leftVelocity.toMetersPerSecond() << " m/s, right="
              << drive.rightVelocity.toMetersPerSecond() << " m/s" << std::endl;

    std::cout << "\n✅ All tests passed! RobotLib is working correctly." << std::endl;
    std::cout << "\nYou can now #include <RobotLib.h> in your projects!" << std::endl;

    return 0;
}
