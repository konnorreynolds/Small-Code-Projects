// Quick test to verify RobotLib includes work correctly

#include "include/RobotLib.h"
#include <iostream>

int main() {
    std::cout << "✅ RobotLib.h included successfully!" << std::endl;
    std::cout << "Version: " << robotlib::VERSION_STRING << std::endl;

    // Test basic functionality
    using namespace units;
    auto dist = m(5.0);
    std::cout << "Distance test: " << dist.toMeters() << " m" << std::endl;

    return 0;
}
