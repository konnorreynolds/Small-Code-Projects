// ============================================================================
// Obstacle Avoidance Robot - Arduino IDE Example
// ============================================================================
// Hardware:
// - Arduino Uno/Nano
// - L298N Motor Driver
// - 2x DC Motors
// - HC-SR04 Ultrasonic Sensor
// - Battery pack (7.4V)
//
// Connections:
// - Motors: IN1=5, IN2=6, IN3=9, IN4=10, ENA=3, ENB=11
// - Ultrasonic: TRIG=12, ECHO=13
// ============================================================================

#include <RobotLib.h>

using namespace units;
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

const int ULTRASONIC_TRIG = 12;
const int ULTRASONIC_ECHO = 13;

// ============================================================================
// Robot Configuration
// ============================================================================
const auto CRUISE_SPEED = 150;        // Normal driving speed
const auto TURN_SPEED = 120;          // Turning speed
const auto SAFE_DISTANCE = cm(30);    // Stop if closer than 30cm
const auto WARNING_DISTANCE = cm(50); // Slow down if closer than 50cm

// ============================================================================
// Setup
// ============================================================================
void setup() {
    begin(115200);
    println("=== Obstacle Avoidance Robot ===");
    println("Using ultrasonic sensor");
    println();

    // Motor pins
    pinMode(MOTOR_LEFT_FWD, OUTPUT);
    pinMode(MOTOR_LEFT_REV, OUTPUT);
    pinMode(MOTOR_LEFT_PWM, OUTPUT);
    pinMode(MOTOR_RIGHT_FWD, OUTPUT);
    pinMode(MOTOR_RIGHT_REV, OUTPUT);
    pinMode(MOTOR_RIGHT_PWM, OUTPUT);

    // Ultrasonic pins
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);

    println("Ready!");
    delay(1000);
}

// ============================================================================
// Ultrasonic Distance Measurement
// ============================================================================
Distance<> measureDistance() {
    // Trigger ultrasonic pulse
    digitalWrite(ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);

    // Measure echo pulse duration
    long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000);  // 30ms timeout

    // Calculate distance: duration (microseconds) to cm
    // Speed of sound = 343 m/s = 0.0343 cm/μs
    // Distance = (duration * 0.0343) / 2  (divide by 2 for round trip)
    double distanceCm = duration * 0.01715;

    // Handle timeout (no echo received)
    if (duration == 0) {
        return cm(400);  // Return max distance on timeout
    }

    return cm(distanceCm);
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

void moveForward(int speed = CRUISE_SPEED) {
    setMotorSpeed(speed, speed);
}

void moveBackward(int speed = CRUISE_SPEED) {
    setMotorSpeed(-speed, -speed);
}

void turnRight(int speed = TURN_SPEED) {
    setMotorSpeed(speed, -speed);
}

void turnLeft(int speed = TURN_SPEED) {
    setMotorSpeed(-speed, speed);
}

void stopMotors() {
    setMotorSpeed(0, 0);
}

// ============================================================================
// Behavior State Machine
// ============================================================================
enum RobotState {
    CRUISING,       // Normal forward movement
    SLOWING,        // Approaching obstacle
    AVOIDING,       // Actively avoiding obstacle
    REVERSING       // Backing up
};

RobotState currentState = CRUISING;
unsigned long stateStartTime = 0;

void setState(RobotState newState) {
    if (newState != currentState) {
        currentState = newState;
        stateStartTime = millis();

        // Print state change
        print("State: ");
        switch (newState) {
            case CRUISING:  println("CRUISING"); break;
            case SLOWING:   println("SLOWING"); break;
            case AVOIDING:  println("AVOIDING"); break;
            case REVERSING: println("REVERSING"); break;
        }
    }
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
    // Measure distance to obstacle
    auto distance = measureDistance();
    unsigned long stateTime = millis() - stateStartTime;

    // Debug output
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 300) {
        print("Distance: ");
        print(distance.toCentimeters());
        println(" cm");
        lastDebugTime = millis();
    }

    // Behavior based on current state
    switch (currentState) {
        case CRUISING:
            moveForward(CRUISE_SPEED);

            if (distance < SAFE_DISTANCE) {
                setState(AVOIDING);
            } else if (distance < WARNING_DISTANCE) {
                setState(SLOWING);
            }
            break;

        case SLOWING:
            // Slow down as we approach obstacle
            int slowSpeed = map(
                distance.toCentimeters(),
                SAFE_DISTANCE.toCentimeters(),
                WARNING_DISTANCE.toCentimeters(),
                50,  // Minimum speed
                CRUISE_SPEED
            );
            moveForward(slowSpeed);

            if (distance < SAFE_DISTANCE) {
                setState(AVOIDING);
            } else if (distance >= WARNING_DISTANCE) {
                setState(CRUISING);
            }
            break;

        case AVOIDING:
            stopMotors();
            delay(200);

            // Back up a bit
            setState(REVERSING);
            break;

        case REVERSING:
            moveBackward(100);

            // Reverse for 500ms, then turn
            if (stateTime > 500) {
                stopMotors();
                delay(200);

                // Turn right (randomly choose left/right for variety)
                if (random(0, 2) == 0) {
                    println("Turning right...");
                    turnRight();
                } else {
                    println("Turning left...");
                    turnLeft();
                }

                delay(800);  // Turn for 800ms (~90 degrees)

                stopMotors();
                delay(200);

                setState(CRUISING);
            }
            break;
    }

    delay(50);  // Small delay for sensor stability
}
