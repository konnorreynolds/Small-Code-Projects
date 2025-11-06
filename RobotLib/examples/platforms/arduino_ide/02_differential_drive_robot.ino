// ============================================================================
// Differential Drive Robot with Odometry - Arduino IDE Example
// ============================================================================
// Hardware:
// - Arduino Uno/Nano/Mega
// - L298N Motor Driver
// - 2x DC Motors with encoders
// - Battery pack (7.4V)
//
// Connections:
// - Left Motor: IN1=5, IN2=6, PWM=3
// - Right Motor: IN3=9, IN4=10, PWM=11
// - Left Encoder: A=2, B=4 (interrupt pins)
// - Right Encoder: A=3, B=7
// ============================================================================

#include <RobotLib.h>

using namespace units;
using namespace robotics;
using namespace robotlib::output;

// ============================================================================
// Pin Configuration
// ============================================================================
const int MOTOR_LEFT_FWD = 5;
const int MOTOR_LEFT_REV = 6;
const int MOTOR_LEFT_PWM = 3;

const int MOTOR_RIGHT_FWD = 9;
const int MOTOR_RIGHT_REV = 10;
const int MOTOR_RIGHT_PWM = 11;

const int ENCODER_LEFT_A = 2;   // Must be interrupt pin
const int ENCODER_LEFT_B = 4;
const int ENCODER_RIGHT_A = 3;  // Must be interrupt pin
const int ENCODER_RIGHT_B = 7;

// ============================================================================
// Robot Physical Parameters
// ============================================================================
const auto WHEEL_DIAMETER = cm(6.5);      // 65mm wheels
const auto WHEEL_BASE = cm(15.0);         // 15cm between wheels
const int ENCODER_COUNTS_PER_REV = 360;   // Encoder resolution

// Derived constants
const auto WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER.toMeters() * 3.14159;
const auto METERS_PER_COUNT = WHEEL_CIRCUMFERENCE / ENCODER_COUNTS_PER_REV;

// ============================================================================
// Global Variables
// ============================================================================
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Robot pose (position and orientation)
double robotX = 0.0;      // meters
double robotY = 0.0;      // meters
double robotTheta = 0.0;  // radians

// ============================================================================
// Encoder Interrupt Handlers
// ============================================================================
void leftEncoderISR() {
    if (digitalRead(ENCODER_LEFT_B) == HIGH) {
        leftEncoderCount++;
    } else {
        leftEncoderCount--;
    }
}

void rightEncoderISR() {
    if (digitalRead(ENCODER_RIGHT_B) == HIGH) {
        rightEncoderCount++;
    } else {
        rightEncoderCount--;
    }
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
    begin(115200);
    println("=== Differential Drive Robot ===");
    println("With odometry and position tracking");
    println();

    // Motor pins
    pinMode(MOTOR_LEFT_FWD, OUTPUT);
    pinMode(MOTOR_LEFT_REV, OUTPUT);
    pinMode(MOTOR_LEFT_PWM, OUTPUT);
    pinMode(MOTOR_RIGHT_FWD, OUTPUT);
    pinMode(MOTOR_RIGHT_REV, OUTPUT);
    pinMode(MOTOR_RIGHT_PWM, OUTPUT);

    // Encoder pins
    pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightEncoderISR, RISING);

    log("Wheel diameter", WHEEL_DIAMETER.toMeters());
    log("Wheel base", WHEEL_BASE.toMeters());
    log("Meters per encoder count", METERS_PER_COUNT);
    println();
    println("Ready!");
    delay(1000);
}

// ============================================================================
// Motor Control
// ============================================================================
void setMotorSpeed(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    // Left motor
    if (leftSpeed >= 0) {
        digitalWrite(MOTOR_LEFT_FWD, HIGH);
        digitalWrite(MOTOR_LEFT_REV, LOW);
        analogWrite(MOTOR_LEFT_PWM, leftSpeed);
    } else {
        digitalWrite(MOTOR_LEFT_FWD, LOW);
        digitalWrite(MOTOR_LEFT_REV, HIGH);
        analogWrite(MOTOR_LEFT_PWM, -leftSpeed);
    }

    // Right motor
    if (rightSpeed >= 0) {
        digitalWrite(MOTOR_RIGHT_FWD, HIGH);
        digitalWrite(MOTOR_RIGHT_REV, LOW);
        analogWrite(MOTOR_RIGHT_PWM, rightSpeed);
    } else {
        digitalWrite(MOTOR_RIGHT_FWD, LOW);
        digitalWrite(MOTOR_RIGHT_REV, HIGH);
        analogWrite(MOTOR_RIGHT_PWM, -rightSpeed);
    }
}

// ============================================================================
// Odometry Update
// ============================================================================
void updateOdometry() {
    static long lastLeftCount = 0;
    static long lastRightCount = 0;

    // Get current counts (atomic read)
    noInterrupts();
    long currentLeft = leftEncoderCount;
    long currentRight = rightEncoderCount;
    interrupts();

    // Calculate distance traveled by each wheel
    long deltaLeft = currentLeft - lastLeftCount;
    long deltaRight = currentRight - lastRightCount;

    double leftDistance = deltaLeft * METERS_PER_COUNT;
    double rightDistance = deltaRight * METERS_PER_COUNT;

    // Update last counts
    lastLeftCount = currentLeft;
    lastRightCount = currentRight;

    // Differential drive odometry math
    double centerDistance = (leftDistance + rightDistance) / 2.0;
    double deltaTheta = (rightDistance - leftDistance) / WHEEL_BASE.toMeters();

    // Update robot pose
    robotX += centerDistance * cos(robotTheta + deltaTheta / 2.0);
    robotY += centerDistance * sin(robotTheta + deltaTheta / 2.0);
    robotTheta += deltaTheta;

    // Normalize angle to [-PI, PI]
    while (robotTheta > 3.14159) robotTheta -= 2 * 3.14159;
    while (robotTheta < -3.14159) robotTheta += 2 * 3.14159;
}

// ============================================================================
// Demo Movement Patterns
// ============================================================================
void driveSquare() {
    println("Driving in a square...");

    for (int i = 0; i < 4; i++) {
        // Drive forward 0.5m
        println("Forward");
        setMotorSpeed(150, 150);
        delay(2000);

        // Stop
        setMotorSpeed(0, 0);
        delay(500);

        // Turn 90 degrees
        println("Turning");
        setMotorSpeed(120, -120);
        delay(800);

        // Stop
        setMotorSpeed(0, 0);
        delay(500);
    }

    setMotorSpeed(0, 0);
}

void driveCircle() {
    println("Driving in a circle...");

    // Left motor slower than right = turn left
    setMotorSpeed(100, 150);
    delay(5000);

    setMotorSpeed(0, 0);
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
    // Update odometry
    updateOdometry();

    // Print position every 200ms
    static unsigned long lastPrintTime = 0;
    unsigned long currentTime = millis();

    if (currentTime - lastPrintTime > 200) {
        print("Position: X=");
        print(robotX);
        print("m  Y=");
        print(robotY);
        print("m  Theta=");
        print(robotTheta * 57.3);  // Convert to degrees
        println("°");

        lastPrintTime = currentTime;
    }

    // Demo movements (comment out for manual control)
    static unsigned long lastDemoTime = 0;
    if (currentTime - lastDemoTime > 10000) {  // Every 10 seconds
        driveSquare();
        delay(2000);
        lastDemoTime = currentTime;
    }

    delay(10);
}
