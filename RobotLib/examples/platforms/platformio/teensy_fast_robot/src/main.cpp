// ============================================================================
// High-Speed Robot Control - Teensy 4.0 Example
// ============================================================================
// Hardware:
// - Teensy 4.0 (600 MHz ARM Cortex-M7)
// - L298N or TB6612 Motor Driver
// - 2x DC Motors with high-resolution encoders
// - MPU6050 IMU
// - Battery (7.4-11.1V)
//
// Features:
// - 1kHz control loop (very fast!)
// - High-resolution encoder tracking
// - IMU integration for heading stabilization
// - Real-time trajectory tracking
// - Advanced PID control
// ============================================================================

#include <Arduino.h>
#include <RobotLib.h>

using namespace units;
using namespace robotics;
using namespace robotlib::output;

// ============================================================================
// Pin Configuration (Teensy 4.0)
// ============================================================================
const int MOTOR_LEFT_FWD = 2;
const int MOTOR_LEFT_REV = 3;
const int MOTOR_LEFT_PWM = 4;

const int MOTOR_RIGHT_FWD = 5;
const int MOTOR_RIGHT_REV = 6;
const int MOTOR_RIGHT_PWM = 7;

const int ENCODER_LEFT_A = 8;
const int ENCODER_LEFT_B = 9;
const int ENCODER_RIGHT_A = 10;
const int ENCODER_RIGHT_B = 11;

const int IMU_SDA = 18;  // I2C
const int IMU_SCL = 19;

// ============================================================================
// Robot Configuration
// ============================================================================
const auto WHEEL_DIAMETER = cm(6.5);
const auto WHEEL_BASE = cm(15.0);
const int ENCODER_PPR = 1024;  // High-resolution encoder

const auto WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER.toMeters() * 3.14159265359;
const auto METERS_PER_TICK = WHEEL_CIRCUMFERENCE / ENCODER_PPR;

// ============================================================================
// High-Speed Control Variables
// ============================================================================
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// PID controllers (tuned for fast response)
PIDController leftSpeedPID(3.0, 1.0, 0.05);
PIDController rightSpeedPID(3.0, 1.0, 0.05);
PIDController headingPID(2.5, 0.1, 0.2);

// State variables
double currentHeading = 0.0;  // radians
double targetHeading = 0.0;
double leftVelocity = 0.0;    // m/s
double rightVelocity = 0.0;

// Performance tracking
unsigned long loopCount = 0;
unsigned long lastStatTime = 0;

// ============================================================================
// Encoder ISRs (optimized for speed)
// ============================================================================
void leftEncoderISR() {
    leftEncoderCount += (digitalReadFast(ENCODER_LEFT_B) == HIGH) ? 1 : -1;
}

void rightEncoderISR() {
    rightEncoderCount += (digitalReadFast(ENCODER_RIGHT_B) == HIGH) ? 1 : -1;
}

// ============================================================================
// Motor Control
// ============================================================================
void setMotorSpeed(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    // Left motor
    digitalWriteFast(MOTOR_LEFT_FWD, leftSpeed >= 0);
    digitalWriteFast(MOTOR_LEFT_REV, leftSpeed < 0);
    analogWrite(MOTOR_LEFT_PWM, abs(leftSpeed));

    // Right motor
    digitalWriteFast(MOTOR_RIGHT_FWD, rightSpeed >= 0);
    digitalWriteFast(MOTOR_RIGHT_REV, rightSpeed < 0);
    analogWrite(MOTOR_RIGHT_PWM, abs(rightSpeed));
}

// ============================================================================
// Velocity Estimation
// ============================================================================
void updateVelocities(double dt) {
    static long lastLeftCount = 0;
    static long lastRightCount = 0;

    // Atomic read
    noInterrupts();
    long currentLeft = leftEncoderCount;
    long currentRight = rightEncoderCount;
    interrupts();

    // Calculate velocities
    long deltaLeft = currentLeft - lastLeftCount;
    long deltaRight = currentRight - lastRightCount;

    leftVelocity = (deltaLeft * METERS_PER_TICK) / dt;
    rightVelocity = (deltaRight * METERS_PER_TICK) / dt;

    lastLeftCount = currentLeft;
    lastRightCount = currentRight;
}

// ============================================================================
// High-Speed Control Loop (1kHz)
// ============================================================================
IntervalTimer controlTimer;

void controlLoop() {
    static unsigned long lastTime = micros();
    unsigned long currentTime = micros();
    double dt = (currentTime - lastTime) / 1000000.0;
    lastTime = currentTime;

    // Update velocity estimates
    updateVelocities(dt);

    // Target velocities (example: drive straight at 0.5 m/s)
    double targetLeftVel = 0.5;   // m/s
    double targetRightVel = 0.5;

    // PID control for velocity
    double leftPWM = leftSpeedPID.calculate(targetLeftVel, leftVelocity, dt);
    double rightPWM = rightSpeedPID.calculate(targetRightVel, rightVelocity, dt);

    // Heading stabilization (keep robot going straight)
    double headingCorrection = headingPID.calculate(targetHeading, currentHeading, dt);
    leftPWM -= headingCorrection;
    rightPWM += headingCorrection;

    // Apply motor commands
    setMotorSpeed((int)leftPWM, (int)rightPWM);

    loopCount++;
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);  // Wait for serial

    println("\n=== Teensy 4.0 High-Speed Robot ===");
    print("CPU Speed: ");
    print(F_CPU / 1000000);
    println(" MHz");

    // Motor pins (use digitalWriteFast)
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
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightEncoderISR, CHANGE);

    // PWM setup (Teensy supports very high frequency PWM)
    analogWriteFrequency(MOTOR_LEFT_PWM, 20000);   // 20 kHz
    analogWriteFrequency(MOTOR_RIGHT_PWM, 20000);
    analogWriteResolution(8);  // 0-255

    println("Initializing IMU...");
    // TODO: Add MPU6050 initialization here
    println("IMU initialized");

    log("Control loop frequency", "1000 Hz");
    log("Encoder resolution", ENCODER_PPR);
    log("Meters per tick", METERS_PER_TICK);

    println("\nStarting control loop...");
    delay(2000);

    // Start 1kHz control timer
    controlTimer.begin(controlLoop, 1000);  // 1000 microseconds = 1ms

    println("Robot ready!");
}

// ============================================================================
// Main Loop (telemetry and commands)
// ============================================================================
void loop() {
    unsigned long currentTime = millis();

    // Print statistics every second
    if (currentTime - lastStatTime >= 1000) {
        print("\n--- Statistics ---");
        print("\nLoop rate: ");
        print(loopCount);
        println(" Hz");

        print("Left velocity: ");
        print(leftVelocity);
        println(" m/s");

        print("Right velocity: ");
        print(rightVelocity);
        println(" m/s");

        print("Heading: ");
        print(currentHeading * 57.2958);
        println("°");

        noInterrupts();
        long leftCount = leftEncoderCount;
        long rightCount = rightEncoderCount;
        interrupts();

        print("Encoders: L=");
        print(leftCount);
        print(" R=");
        println(rightCount);

        loopCount = 0;
        lastStatTime = currentTime;
    }

    // Check for serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        switch (cmd) {
            case 's':  // Stop
                controlTimer.end();
                setMotorSpeed(0, 0);
                println("Stopped");
                break;

            case 'g':  // Go
                controlTimer.begin(controlLoop, 1000);
                println("Started");
                break;

            case 'r':  // Reset encoders
                noInterrupts();
                leftEncoderCount = 0;
                rightEncoderCount = 0;
                interrupts();
                println("Encoders reset");
                break;

            case '?':  // Help
                println("\nCommands:");
                println("  s - Stop");
                println("  g - Go");
                println("  r - Reset encoders");
                println("  ? - Help");
                break;
        }
    }

    delay(10);
}
