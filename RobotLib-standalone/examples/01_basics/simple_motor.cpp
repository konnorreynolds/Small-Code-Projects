// ============================================================================
// Simple Motor Control - No Sensor Feedback
// ============================================================================
// Learn the basics: control a motor without any encoders or sensors.
// Just set the speed and let it run!
//
// What you'll learn:
// - How to create a motor controller
// - How to set motor speed (duty cycle)
// - How to make timed movements
// - How to ramp speed up/down
// ============================================================================

#include "../../include/RobotLib.h"

using namespace robotlib;
using namespace robotlib::output;

int main() {
    println();
    println("========================================");
    println("  Simple Motor Control");
    println("  (No Encoders Required!)");
    println("========================================");
    println();

    // ========================================
    // STEP 1: Create the motor
    // ========================================
    println("STEP 1: Creating motor controller...");
    println();

    Arm motor = Arm();  // Create a motor controller

    // 🔌 HARDWARE SETUP:
    // Connect motor driver (like L298N or motor shield) to:
    // - Arduino Pin 9 (PWM) → Motor speed control
    // - Arduino Pin 8 (DIR) → Motor direction
    // - Motor driver → DC motor
    // - Power supply → Motor driver VCC (6-12V for most motors)

    println("✓ Motor created!");
    println("  Ready to control");
    println();

    // ========================================
    // STEP 2: Spin the motor forward
    // ========================================
    println("STEP 2: Spin motor forward at 50% power");
    println();

    motor.setDutyCycle(0.5);  // 0.5 = 50% power forward

    // 🔧 WHAT HAPPENS IN REAL HARDWARE:
    // digitalWrite(8, HIGH);              // Set direction to forward
    // analogWrite(9, 0.5 * 255);          // Set PWM to 128 (50% of 255)
    // The motor spins forward at half speed

    // ⏱️ IN YOUR REAL CODE, ADD:
    // delay(2000);  // Let motor run for 2 seconds

    print("  Power level: ", motor.getDutyCycle() * 100.0, "%\n");
    println("  Direction: Forward");
    println("  💡 In real hardware, let this run for 2 seconds");
    println();

    // ========================================
    // STEP 3: Spin the motor backward
    // ========================================
    println("STEP 3: Spin motor backward at 30% power");
    println();

    motor.setDutyCycle(-0.3);  // Negative = reverse direction

    // 🔧 WHAT HAPPENS IN REAL HARDWARE:
    // digitalWrite(8, LOW);               // Set direction to reverse
    // analogWrite(9, 0.3 * 255);          // Set PWM to 77 (30% of 255)
    // The motor spins backward at slower speed

    // ⏱️ IN YOUR REAL CODE, ADD:
    // delay(1000);  // Let motor run for 1 second

    print("  Power level: ", motor.getDutyCycle() * -100.0, "%\n");
    println("  Direction: Backward");
    println("  💡 In real hardware, let this run for 1 second");
    println();

    // ========================================
    // STEP 4: Stop the motor
    // ========================================
    println("STEP 4: Stop the motor");
    println();

    motor.stop();  // Stop the motor

    // 🔧 WHAT HAPPENS IN REAL HARDWARE:
    // analogWrite(9, 0);                  // Set PWM to 0
    // The motor stops spinning (coasts to a stop)
    // For instant stop (brake), you'd short both motor terminals

    print("  Power level: ", motor.getDutyCycle() * 100.0, "%\n");
    println("  Motor is now stopped");
    println();

    // ========================================
    // STEP 5: Smoothly ramp up speed
    // ========================================
    println("STEP 5: Smoothly ramp up from 0% to 100%");
    println();
    println("  Speed  | Power");
    println("  -------|-------");

    // Gradually increase from 0 to 100% in 25% steps
    for (int speed = 0; speed <= 100; speed += 25) {
        double power = speed / 100.0;  // Convert to 0.0-1.0 range
        motor.setDutyCycle(power);

        // 🔧 WHAT HAPPENS IN REAL HARDWARE:
        // analogWrite(9, power * 255);    // PWM increases: 0, 64, 128, 191, 255
        // The motor gradually speeds up

        // ⏱️ IN YOUR REAL CODE, ADD:
        // delay(500);  // Wait 0.5s between speed changes

        print("  ", speed, "% | ", motor.getDutyCycle() * 100.0, "%\n");
    }

    println();
    println("  💡 In real hardware, add a small delay between steps");
    println("     This prevents sudden jumps that could damage motor/gears");
    println();

    // ========================================
    // Summary
    // ========================================
    println("========================================");
    println("✓ You learned:");
    println("  1. Create a motor: Arm motor = Arm()");
    println("  2. Spin forward: motor.setDutyCycle(0.5)");
    println("  3. Spin backward: motor.setDutyCycle(-0.5)");
    println("  4. Stop: motor.stop()");
    println("  5. Ramp speed: Loop and change gradually");
    println();

    println("========================================");
    println("🔧 COMPLETE ARDUINO IMPLEMENTATION:");
    println("========================================");
    println();
    println("const int PWM_PIN = 9;    // Motor speed");
    println("const int DIR_PIN = 8;    // Motor direction");
    println();
    println("void setup() {");
    println("  pinMode(PWM_PIN, OUTPUT);");
    println("  pinMode(DIR_PIN, OUTPUT);");
    println("}");
    println();
    println("void loop() {");
    println("  // Get duty cycle from RobotLib");
    println("  double duty = motor.getDutyCycle();");
    println();
    println("  // Set direction based on sign");
    println("  digitalWrite(DIR_PIN, duty >= 0 ? HIGH : LOW);");
    println();
    println("  // Set speed (PWM is 0-255)");
    println("  analogWrite(PWM_PIN, abs(duty) * 255);");
    println("}");
    println();

    println("Next step: Try 'simple_servo.cpp' for servo control!");
    println("========================================");
    println();

    return 0;
}
