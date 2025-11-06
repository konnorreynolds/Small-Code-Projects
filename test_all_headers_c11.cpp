// Comprehensive C++11 test for all RobotLib headers
#include <RobotLib.h>

int main() {
    using namespace units;
    using namespace robotics;
    using namespace utilities;
    using namespace spatial;

    // Test basic units
    auto dist = m(5.0);
    auto time = s(2.0);
    auto velocity = dist / time;

    // Test physics units
    auto force = N(10.0);
    auto energy = J(100.0);
    auto power = W(50.0);

    // Test electrical
    auto voltage = V(12.0);
    auto current = A(2.0);
    auto resistance = voltage / current;

    // Test angular
    auto angle = rad(1.57);
    auto angvel = radps(3.14);

    // Test robotics
    PIDController pid(1.0, 0.1, 0.01);
    pid.calculate(10.0, 8.0, 0.02);

    // Test differential drive
    auto leftVel = mps(1.0);
    auto rightVel = mps(0.8);

    return 0;
}
