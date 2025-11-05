// ============================================================================
// Hello Units - Your Very First Program!
// ============================================================================
// Welcome to RobotLib! This is the easiest possible introduction.
// You'll learn how to use type-safe units to prevent bugs in your robot code.
//
// What you'll learn:
// - How to create distances, times, and speeds
// - How units prevent mistakes
// - How to convert between units (meters, feet, etc.)
// - Why this is better than using raw numbers
// ============================================================================

#include "../../include/units_core.h"
#include "../../include/units_physics.h"
#include <iostream>

using namespace units;

int main() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════╗\n";
    std::cout << "║  👋 Hello Units!                       ║\n";
    std::cout << "║  Your First Type-Safe Program          ║\n";
    std::cout << "╚════════════════════════════════════════╝\n";
    std::cout << "\n";

    // ========================================
    // STEP 1: Create a distance
    // ========================================
    std::cout << "STEP 1: Creating a distance\n";
    std::cout << "==========================\n\n";

    auto distance = m(5.0);  // Create 5 meters

    std::cout << "Created: 5 meters\n";
    std::cout << "  In meters: " << distance.toMeters() << " m\n";
    std::cout << "  In feet: " << distance.toFeet() << " ft\n";
    std::cout << "  In centimeters: " << distance.toCentimeters() << " cm\n\n";

    std::cout << "💡 Same value, different units! No manual conversion needed.\n\n";

    // ========================================
    // STEP 2: Create a time
    // ========================================
    std::cout << "STEP 2: Creating a time\n";
    std::cout << "======================\n\n";

    auto time = s(2.0);  // Create 2 seconds

    std::cout << "Created: 2 seconds\n";
    std::cout << "  In seconds: " << time.toSeconds() << " s\n";
    std::cout << "  In milliseconds: " << time.toMilliseconds() << " ms\n\n";

    // ========================================
    // STEP 3: Calculate speed automatically!
    // ========================================
    std::cout << "STEP 3: Calculate speed (distance ÷ time)\n";
    std::cout << "=========================================\n\n";

    auto speed = distance / time;  // Automatically becomes m/s!

    std::cout << "Speed = " << distance.toMeters() << " m ÷ "
              << time.toSeconds() << " s\n";
    std::cout << "      = " << speed.toMetersPerSecond() << " m/s\n\n";

    std::cout << "Same speed in different units:\n";
    std::cout << "  " << speed.toMetersPerSecond() << " m/s\n";
    std::cout << "  " << speed.toKilometersPerHour() << " km/h\n";
    std::cout << "  " << speed.toFeetPerSecond() << " ft/s\n\n";

    std::cout << "💡 The library knows that distance/time = speed!\n\n";

    // ========================================
    // STEP 4: Add distances together
    // ========================================
    std::cout << "STEP 4: Adding distances\n";
    std::cout << "=======================\n\n";

    auto distance1 = m(5.0);   // 5 meters
    auto distance2 = m(3.0);   // 3 meters
    auto total = distance1 + distance2;

    std::cout << distance1.toMeters() << " m + "
              << distance2.toMeters() << " m = "
              << total.toMeters() << " m\n\n";

    std::cout << "💡 You can only add things with the same unit type!\n\n";

    // ========================================
    // STEP 5: Type safety prevents mistakes!
    // ========================================
    std::cout << "STEP 5: How units prevent bugs\n";
    std::cout << "==============================\n\n";

    std::cout << "✅ Things you CAN do:\n";
    std::cout << "   m(5) + m(3)           →  Add distances together\n";
    std::cout << "   m(10) / s(2)          →  Divide distance by time\n";
    std::cout << "   m(5) * 2              →  Multiply distance by number\n";
    std::cout << "   m(5) > m(3)           →  Compare distances\n\n";

    std::cout << "❌ Things you CAN'T do (won't compile!):\n";
    std::cout << "   m(5) + s(2)           →  Can't add distance + time\n";
    std::cout << "   m(5) + kg(10)         →  Can't add distance + mass\n";
    std::cout << "   m(5) == 5             →  Can't compare distance to raw number\n\n";

    std::cout << "💡 The compiler catches these mistakes for you!\n";
    std::cout << "   No more 'oops, I used feet instead of meters' bugs.\n\n";

    // ========================================
    // STEP 6: Working with angles
    // ========================================
    std::cout << "STEP 6: Working with angles\n";
    std::cout << "==========================\n\n";

    auto angle = deg(90);  // 90 degrees

    std::cout << "Created: 90 degrees\n";
    std::cout << "  In degrees: " << angle.toDegrees() << "°\n";
    std::cout << "  In radians: " << angle.toRadians() << " rad\n\n";

    std::cout << "💡 Perfect for robot arm joints and wheel rotations!\n\n";

    // ========================================
    // Summary
    // ========================================
    std::cout << "╔════════════════════════════════════════╗\n";
    std::cout << "║  🎉 Congratulations!                   ║\n";
    std::cout << "╚════════════════════════════════════════╝\n\n";

    std::cout << "You learned:\n";
    std::cout << "  ✓ Create units: m(5), s(2), deg(90)\n";
    std::cout << "  ✓ Convert units: .toMeters(), .toFeet()\n";
    std::cout << "  ✓ Do math: distance / time = speed\n";
    std::cout << "  ✓ Type safety prevents bugs!\n\n";

    std::cout << "What's next?\n";
    std::cout << "  → Try '01_basics/simple_motor.cpp' to control a motor\n";
    std::cout << "  → Try '01_basics/simple_servo.cpp' to control a servo\n\n";

    std::cout << "Why use type-safe units?\n";
    std::cout << "  ❌ Old way: double distance = 5;  // Is this meters or feet?\n";
    std::cout << "  ✅ New way: auto distance = m(5); // Clearly 5 meters!\n\n";

    return 0;
}
