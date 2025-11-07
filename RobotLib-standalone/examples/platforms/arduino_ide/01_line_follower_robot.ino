// ============================================================================
// Line Follower Robot - Arduino IDE Example
// ============================================================================
// Hardware:
// - Arduino Uno/Nano/Mega
// - L298N Motor Driver
// - 2x DC Motors with wheels
// - 5x IR Line Sensors (digital)
// - Battery pack (7.4V recommended)
//
// Connections:
// - Motors: IN1=5, IN2=6, IN3=9, IN4=10, ENA=3, ENB=11
// - IR Sensors: pins 2, 4, 7, 8, 12 (left to right)
// ============================================================================

#include <RobotLib.h>

using namespace units;
using namespace robotlib;
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

const int SENSOR_L2 = 2;   // Far left
const int SENSOR_L1 = 4;   // Left
const int SENSOR_CENTER = 7;
const int SENSOR_R1 = 8;   // Right
const int SENSOR_R2 = 12;  // Far right

// ============================================================================
// Robot Configuration
// ============================================================================
const auto BASE_SPEED = 150;  // Base motor speed (0-255)
const auto MAX_SPEED = 200;   // Maximum motor speed

// PID Controller for line following
robotics::PIDController linePID(
    2.0,   // Kp - Proportional gain (increase for faster response)
    0.5,   // Ki - Integral gain (increase to eliminate steady-state error)
    0.1    // Kd - Derivative gain (increase to reduce oscillations)
);

// ============================================================================
// Setup
// ============================================================================
void setup() {
    // Initialize output system
    begin(115200);
    println("=== Line Follower Robot ===");
    println("Initializing...");

    // Motor pins
    pinMode(MOTOR_LEFT_FWD, OUTPUT);
    pinMode(MOTOR_LEFT_REV, OUTPUT);
    pinMode(MOTOR_LEFT_PWM, OUTPUT);
    pinMode(MOTOR_RIGHT_FWD, OUTPUT);
    pinMode(MOTOR_RIGHT_REV, OUTPUT);
    pinMode(MOTOR_RIGHT_PWM, OUTPUT);

    // Sensor pins
    pinMode(SENSOR_L2, INPUT);
    pinMode(SENSOR_L1, INPUT);
    pinMode(SENSOR_CENTER, INPUT);
    pinMode(SENSOR_R1, INPUT);
    pinMode(SENSOR_R2, INPUT);

    println("Ready! Place robot on line.");
    delay(2000);
}

// ============================================================================
// Read Sensors
// ============================================================================
// Returns line position: -2 (far left) to +2 (far right), 0 = centered
double readLinePosition() {
    bool l2 = digitalRead(SENSOR_L2) == LOW;  // LOW = line detected
    bool l1 = digitalRead(SENSOR_L1) == LOW;
    bool c  = digitalRead(SENSOR_CENTER) == LOW;
    bool r1 = digitalRead(SENSOR_R1) == LOW;
    bool r2 = digitalRead(SENSOR_R2) == LOW;

    // Calculate weighted position
    // Far left = -2, left = -1, center = 0, right = 1, far right = 2
    double position = 0;
    int sensorCount = 0;

    if (l2) { position += -2; sensorCount++; }
    if (l1) { position += -1; sensorCount++; }
    if (c)  { position +=  0; sensorCount++; }
    if (r1) { position +=  1; sensorCount++; }
    if (r2) { position +=  2; sensorCount++; }

    if (sensorCount > 0) {
        return position / sensorCount;
    }

    return 0;  // No line detected
}

// ============================================================================
// Motor Control
// ============================================================================
void setMotorSpeed(int leftSpeed, int rightSpeed) {
    // Constrain speeds to valid range
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

void stopMotors() {
    setMotorSpeed(0, 0);
}

// ============================================================================
// Main Loop
// ============================================================================
unsigned long lastUpdateTime = 0;

void loop() {
    // Calculate time step
    unsigned long currentTime = millis();
    double dt = (currentTime - lastUpdateTime) / 1000.0;  // Convert to seconds
    lastUpdateTime = currentTime;

    // Skip first iteration
    if (dt > 0.1) {  // Sanity check
        dt = 0.02;   // Default to 20ms
    }

    // Read line position (-2 to +2)
    double linePosition = readLinePosition();

    // PID control: target is 0 (centered on line)
    double correction = linePID.calculate(0.0, linePosition, dt);

    // Calculate motor speeds
    // When line is left (negative), turn left (slow left motor)
    // When line is right (positive), turn right (slow right motor)
    int leftSpeed = BASE_SPEED - correction;
    int rightSpeed = BASE_SPEED + correction;

    // Apply speeds
    setMotorSpeed(leftSpeed, rightSpeed);

    // Debug output (every 500ms)
    static unsigned long lastDebugTime = 0;
    if (currentTime - lastDebugTime > 500) {
        print("Position: ");
        print(linePosition);
        print("  Correction: ");
        print(correction);
        print("  Speeds: L=");
        print(leftSpeed);
        print(" R=");
        println(rightSpeed);
        lastDebugTime = currentTime;
    }

    delay(10);  // Small delay for stability
}
