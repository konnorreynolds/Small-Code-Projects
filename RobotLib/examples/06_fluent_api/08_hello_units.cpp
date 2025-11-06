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

#include "../../include/RobotLib.h"

using namespace units;
using namespace robotlib::output;

int main() {
    println("");
    println("╔════════════════════════════════════════╗");
    println("║  👋 Hello Units!                       ║");
    println("║  Your First Type-Safe Program          ║");
    println("╚════════════════════════════════════════╝");
    println("");

    // ========================================
    // STEP 1: Create a distance
    // ========================================
    println("STEP 1: Creating a distance");
    println("==========================\n");

    auto distance = m(5.0);  // Create 5 meters

    println("Created: 5 meters");
    println("  In meters: ", distance.toMeters(), " m");
    println("  In feet: ", distance.toFeet(), " ft");
    println("  In centimeters: ", distance.toCentimeters(), " cm\n");

    println("💡 Same value, different units! No manual conversion needed.\n");

    // ========================================
    // STEP 2: Create a time
    // ========================================
    println("STEP 2: Creating a time");
    println("======================\n");

    auto time = s(2.0);  // Create 2 seconds

    println("Created: 2 seconds");
    println("  In seconds: ", time.toSeconds(), " s");
    println("  In milliseconds: ", time.toMilliseconds(), " ms\n");

    // ========================================
    // STEP 3: Calculate speed automatically!
    // ========================================
    println("STEP 3: Calculate speed (distance ÷ time)");
    println("=========================================\n");

    auto speed = distance / time;  // Automatically becomes m/s!

    print("Speed = ", distance.toMeters(), " m ÷ ", time.toSeconds(), " s\n");
    println("      = ", speed.toMetersPerSecond(), " m/s\n");

    println("Same speed in different units:");
    println("  ", speed.toMetersPerSecond(), " m/s");
    println("  ", speed.toKilometersPerHour(), " km/h");
    println("  ", speed.toFeetPerSecond(), " ft/s\n");

    println("💡 The library knows that distance/time = speed!\n");

    // ========================================
    // STEP 4: Add distances together
    // ========================================
    println("STEP 4: Adding distances");
    println("=======================\n");

    auto distance1 = m(5.0);   // 5 meters
    auto distance2 = m(3.0);   // 3 meters
    auto total = distance1 + distance2;

    print(distance1.toMeters() , " m + ");

    println("💡 You can only add things with the same unit type!\n");

    // ========================================
    // STEP 5: Type safety prevents mistakes!
    // ========================================
    println("STEP 5: How units prevent bugs");
    println("==============================\n");

    println("✅ Things you CAN do:");
    println("   m(5) + m(3)           →  Add distances together");
    println("   m(10) / s(2)          →  Divide distance by time");
    println("   m(5) * 2              →  Multiply distance by number");
    println("   m(5) > m(3)           →  Compare distances\n");

    println("❌ Things you CAN'T do (won't compile!):");
    println("   m(5) + s(2)           →  Can't add distance + time");
    println("   m(5) + kg(10)         →  Can't add distance + mass");
    println("   m(5) == 5             →  Can't compare distance to raw number\n");

    println("💡 The compiler catches these mistakes for you!");
    println("   No more 'oops, I used feet instead of meters' bugs.\n");

    // ========================================
    // STEP 6: Working with angles
    // ========================================
    println("STEP 6: Working with angles");
    println("==========================\n");

    auto angle = deg(90);  // 90 degrees

    println("Created: 90 degrees");
    println("  In degrees: ", angle.toDegrees(), "°");
    println("  In radians: ", angle.toRadians(), " rad\n");

    println("💡 Perfect for robot arm joints and wheel rotations!\n");

    // ========================================
    // Summary
    // ========================================
    println("╔════════════════════════════════════════╗");
    println("║  🎉 Congratulations!                   ║");
    println("╚════════════════════════════════════════╝\n");

    println("You learned:");
    println("  ✓ Create units: m(5), s(2), deg(90)");
    println("  ✓ Convert units: .toMeters(), .toFeet()");
    println("  ✓ Do math: distance / time = speed");
    println("  ✓ Type safety prevents bugs!\n");

    println("What's next?");
    println("  → Try '01_basics/simple_motor.cpp' to control a motor");
    println("  → Try '01_basics/simple_servo.cpp' to control a servo\n");

    println("Why use type-safe units?");
    println("  ❌ Old way: double distance = 5;  // Is this meters or feet?");
    println("  ✅ New way: auto distance = m(5); // Clearly 5 meters!\n");

    return 0;
}
