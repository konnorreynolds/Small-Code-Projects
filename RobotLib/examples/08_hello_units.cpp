// ============================================================================
// Example 8: Hello Units - Your First Program with Type-Safe Units
// ============================================================================
// This is the simplest possible introduction to RobotLib. If you're new to
// the library, start here!
//
// Topics covered:
// - Creating distance, time, and velocity values
// - Basic arithmetic with units
// - Type safety (what you CAN'T do)
// - Simple conversions
// ============================================================================

#include "../units_core.h"
#include "../units_physics.h"

#include <iostream>

using namespace units;

int main() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════╗\n";
    std::cout << "║  Hello Units!                          ║\n";
    std::cout << "║  Your First Type-Safe Robotics Program║\n";
    std::cout << "╚════════════════════════════════════════╝\n";
    std::cout << "\n";

    // ========================================================================
    // STEP 1: Creating unit values
    // ========================================================================
    std::cout << "Step 1: Creating Values\n";
    std::cout << "=======================\n\n";

    // Create a distance of 5 meters
    auto distance = m(5.0);
    std::cout << "Distance: " << distance.toMeters() << " meters\n";
    std::cout << "          " << distance.toFeet() << " feet\n";
    std::cout << "          " << distance.toCentimeters() << " centimeters\n\n";

    // Create a time of 2 seconds
    auto time = s(2.0);
    std::cout << "Time: " << time.toSeconds() << " seconds\n";
    std::cout << "      " << time.toMilliseconds() << " milliseconds\n\n";

    // ========================================================================
    // STEP 2: Doing math with units
    // ========================================================================
    std::cout << "Step 2: Unit Arithmetic\n";
    std::cout << "=======================\n\n";

    // Calculate velocity = distance / time
    // The library automatically figures out that this should be m/s!
    auto velocity = distance / time;
    std::cout << "Velocity = Distance / Time\n";
    std::cout << "         = " << distance.toMeters() << " m / "
              << time.toSeconds() << " s\n";
    std::cout << "         = " << velocity.toMetersPerSecond() << " m/s\n\n";

    // Convert to other units
    std::cout << "Same velocity in different units:\n";
    std::cout << "  " << velocity.toKilometersPerHour() << " km/h\n";
    std::cout << "  " << velocity.toFeetPerSecond() << " ft/s\n\n";

    // ========================================================================
    // STEP 3: More arithmetic
    // ========================================================================
    std::cout << "Step 3: More Operations\n";
    std::cout << "=======================\n\n";

    // Add distances
    auto distance2 = m(3.0);
    auto total_distance = distance + distance2;
    std::cout << distance.toMeters() << " m + " << distance2.toMeters()
              << " m = " << total_distance.toMeters() << " m\n\n";

    // Multiply by scalar
    auto double_distance = distance * 2.0;
    std::cout << distance.toMeters() << " m × 2 = "
              << double_distance.toMeters() << " m\n\n";

    // Calculate new time from distance and velocity: time = distance / velocity
    // Extract the raw values and compute
    double time_value = total_distance.toMeters() / velocity.toMetersPerSecond();
    auto new_time = s(time_value);
    std::cout << "Time to travel " << total_distance.toMeters()
              << " m at " << velocity.toMetersPerSecond() << " m/s:\n";
    std::cout << "  " << new_time.toSeconds() << " seconds\n\n";

    // ========================================================================
    // STEP 4: Type safety in action
    // ========================================================================
    std::cout << "Step 4: Type Safety\n";
    std::cout << "===================\n\n";

    std::cout << "✅ Things you CAN do:\n";
    std::cout << "   - Add meters to meters\n";
    std::cout << "   - Divide meters by seconds (gets m/s)\n";
    std::cout << "   - Multiply meters by a number\n";
    std::cout << "   - Compare meters to meters\n\n";

    std::cout << "❌ Things you CAN'T do (compiler prevents!):\n";
    std::cout << "   - Add meters to seconds (makes no sense!)\n";
    std::cout << "   - Assign meters to a velocity variable\n";
    std::cout << "   - Multiply meters by meters and assign to distance\n";
    std::cout << "   - Mix up units accidentally\n\n";

    // These would all cause COMPILE ERRORS:
    // auto bad1 = distance + time;           // Can't add distance to time!
    // MetersPerSecond bad2 = distance;       // Can't assign distance to velocity!
    // auto bad3 = distance + velocity;       // Can't add distance to velocity!

    // ========================================================================
    // STEP 5: Working with angles
    // ========================================================================
    std::cout << "Step 5: Angles\n";
    std::cout << "==============\n\n";

    auto angle_deg = deg(90.0);
    auto angle_rad = rad(constants::PI / 2.0);

    std::cout << "90 degrees = " << angle_deg.toRadians() << " radians\n";
    std::cout << "π/2 radians = " << angle_rad.toDegrees() << " degrees\n\n";

    // Trigonometry
    auto angle_45 = deg(45.0);
    std::cout << "sin(45°) = " << angle_45.sin() << "\n";
    std::cout << "cos(45°) = " << angle_45.cos() << "\n";
    std::cout << "tan(45°) = " << angle_45.tan() << "\n\n";

    // ========================================================================
    // STEP 6: Temperature
    // ========================================================================
    std::cout << "Step 6: Temperature\n";
    std::cout << "===================\n\n";

    auto temp_c = degC(25.0);
    std::cout << "Room temperature: " << temp_c.toCelsius() << " °C\n";
    std::cout << "                  " << temp_c.toFahrenheit() << " °F\n";
    std::cout << "                  " << temp_c.toKelvin() << " K\n\n";

    // ========================================================================
    // STEP 7: Mass
    // ========================================================================
    std::cout << "Step 7: Mass\n";
    std::cout << "============\n\n";

    auto robot_mass = kg(45.0);
    std::cout << "Robot mass: " << robot_mass.toKilograms() << " kg\n";
    std::cout << "            " << robot_mass.toPounds() << " lbs\n";
    std::cout << "            " << robot_mass.toGrams() << " g\n\n";

    // ========================================================================
    // Summary
    // ========================================================================
    std::cout << "========================================\n";
    std::cout << "  Summary\n";
    std::cout << "========================================\n\n";

    std::cout << "🎉 Congratulations! You've learned:\n\n";
    std::cout << "  ✓ How to create unit values (m, s, deg, etc.)\n";
    std::cout << "  ✓ How to do math with units\n";
    std::cout << "  ✓ How to convert between units\n";
    std::cout << "  ✓ Why type safety prevents bugs\n";
    std::cout << "  ✓ How to work with angles and temperature\n\n";

    std::cout << "Next Steps:\n";
    std::cout << "  → Try example 02_pid_tuning_guide.cpp\n";
    std::cout << "  → Try example 09_temperature_monitor.cpp\n";
    std::cout << "  → Read the QUICKSTART.md guide\n\n";

    std::cout << "Pro Tip:\n";
    std::cout << "  The compiler is your friend! If you try to add\n";
    std::cout << "  meters to seconds, it won't compile. This catches\n";
    std::cout << "  bugs at compile-time instead of runtime! 🛡️\n\n";

    return 0;
}

/*
Compile and run:
    g++ -std=c++11 -I.. 08_hello_units.cpp -o hello && ./hello

Expected output:
    - Clear demonstration of creating units
    - Examples of conversions
    - Explanation of type safety
    - Friendly introduction for beginners

This example is perfect for:
    - First-time users of RobotLib
    - Teaching the basics of type-safe units
    - Understanding the value of compile-time checking
    - Getting comfortable with the syntax
*/
