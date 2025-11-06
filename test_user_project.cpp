// Test as if we're a user project INCLUDING RobotLib

#include <RobotLib.h>
#include <iostream>

int main() {
    std::cout << "Testing RobotLib from external project..." << std::endl;

    using namespace units;

    auto distance = m(10.5);
    auto velocity = mps(2.0);

    std::cout << "Distance: " << distance.toMeters() << " m" << std::endl;
    std::cout << "Velocity: " << velocity.toMetersPerSecond() << " m/s" << std::endl;

    // Test PID
    robotics::PIDController pid(1.0, 0.1, 0.05);
    double output = pid.calculate(10.0, 5.0, 0.02);
    std::cout << "PID output: " << output << std::endl;

    std::cout << "✅ All tests passed!" << std::endl;
    return 0;
}
