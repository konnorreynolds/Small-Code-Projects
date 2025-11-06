// ============================================================================
// STM32 Robot with FreeRTOS - PlatformIO Example
// ============================================================================
// Hardware:
// - STM32F103C8 (Blue Pill)
// - L298N Motor Driver
// - 2x DC Motors with encoders
// - HC-SR04 Ultrasonic sensor
// - ST-Link programmer
//
// Features:
// - FreeRTOS multitasking
// - Concurrent motor control, sensing, and telemetry
// - Clean task separation
// - Type-safe units from RobotLib
// ============================================================================

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <RobotLib.h>

using namespace units;
using namespace robotics;
using namespace robotlib::output;

// ============================================================================
// Pin Configuration (STM32F103C8)
// ============================================================================
const int MOTOR_LEFT_FWD = PA0;
const int MOTOR_LEFT_REV = PA1;
const int MOTOR_LEFT_PWM = PA2;

const int MOTOR_RIGHT_FWD = PA3;
const int MOTOR_RIGHT_REV = PA4;
const int MOTOR_RIGHT_PWM = PA5;

const int ENCODER_LEFT_A = PB6;
const int ENCODER_LEFT_B = PB7;
const int ENCODER_RIGHT_A = PB8;
const int ENCODER_RIGHT_B = PB9;

const int ULTRASONIC_TRIG = PA6;
const int ULTRASONIC_ECHO = PA7;

const int LED = PC13;  // Built-in LED

// ============================================================================
// Shared Variables (protected by mutexes)
// ============================================================================
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
volatile double currentDistance = 0.0;

SemaphoreHandle_t motorMutex;
SemaphoreHandle_t sensorMutex;

// ============================================================================
// Motor Control (mutex-protected)
// ============================================================================
void setMotorSpeed(int leftSpeed, int rightSpeed) {
    if (xSemaphoreTake(motorMutex, portMAX_DELAY)) {
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

        xSemaphoreGive(motorMutex);
    }
}

// ============================================================================
// Encoder ISRs
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
// RTOS Task: Motor Control
// ============================================================================
void taskMotorControl(void *pvParameters) {
    (void) pvParameters;

    PIDController speedPID(1.5, 0.2, 0.05);
    const int TARGET_SPEED = 150;

    for (;;) {
        // Simple cruise control maintaining constant speed
        int motorSpeed = TARGET_SPEED;

        // Check if obstacle detected
        if (xSemaphoreTake(sensorMutex, 10)) {
            if (currentDistance < 30.0) {  // Obstacle within 30cm
                motorSpeed = 0;  // Stop
            } else if (currentDistance < 50.0) {  // Slow zone
                motorSpeed = 80;
            }
            xSemaphoreGive(sensorMutex);
        }

        setMotorSpeed(motorSpeed, motorSpeed);

        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms update rate
    }
}

// ============================================================================
// RTOS Task: Ultrasonic Sensor
// ============================================================================
void taskUltrasonicSensor(void *pvParameters) {
    (void) pvParameters;

    for (;;) {
        // Trigger pulse
        digitalWrite(ULTRASONIC_TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(ULTRASONIC_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(ULTRASONIC_TRIG, LOW);

        // Measure echo
        long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000);
        double distanceCm = duration * 0.01715;

        // Update shared variable
        if (xSemaphoreTake(sensorMutex, portMAX_DELAY)) {
            currentDistance = (duration == 0) ? 400.0 : distanceCm;
            xSemaphoreGive(sensorMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // 100ms sensor update
    }
}

// ============================================================================
// RTOS Task: Telemetry
// ============================================================================
void taskTelemetry(void *pvParameters) {
    (void) pvParameters;

    for (;;) {
        // Get current sensor readings
        double distance = 0;
        if (xSemaphoreTake(sensorMutex, 10)) {
            distance = currentDistance;
            xSemaphoreGive(sensorMutex);
        }

        // Get encoder counts
        long leftCount, rightCount;
        noInterrupts();
        leftCount = leftEncoderCount;
        rightCount = rightEncoderCount;
        interrupts();

        // Print telemetry
        print("Distance: ");
        print(distance);
        print(" cm | Encoders: L=");
        print(leftCount);
        print(" R=");
        print(rightCount);
        print(" | Free heap: ");
        print(xPortGetFreeHeapSize());
        println(" bytes");

        vTaskDelay(pdMS_TO_TICKS(500));  // 500ms telemetry rate
    }
}

// ============================================================================
// RTOS Task: LED Heartbeat
// ============================================================================
void taskHeartbeat(void *pvParameters) {
    (void) pvParameters;

    for (;;) {
        digitalWrite(LED, LOW);   // LED on
        vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(LED, HIGH);  // LED off
        vTaskDelay(pdMS_TO_TICKS(900));
    }
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
    Serial.begin(115200);
    println("\n=== STM32 Robot with FreeRTOS ===");

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

    // Sensor pins
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);

    // LED
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);  // Off initially

    // Attach encoder interrupts
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightEncoderISR, RISING);

    // Create mutexes
    motorMutex = xSemaphoreCreateMutex();
    sensorMutex = xSemaphoreCreateMutex();

    if (motorMutex == NULL || sensorMutex == NULL) {
        println("ERROR: Failed to create mutexes!");
        while (1);
    }

    // Create RTOS tasks
    xTaskCreate(taskMotorControl, "Motor", 128, NULL, 2, NULL);
    xTaskCreate(taskUltrasonicSensor, "Sensor", 128, NULL, 2, NULL);
    xTaskCreate(taskTelemetry, "Telemetry", 256, NULL, 1, NULL);
    xTaskCreate(taskHeartbeat, "Heartbeat", 64, NULL, 1, NULL);

    println("Tasks created. Starting scheduler...");

    // Start FreeRTOS scheduler (this never returns)
    vTaskStartScheduler();

    // Should never reach here
    println("ERROR: Scheduler failed to start!");
    while (1);
}

// ============================================================================
// Main Loop (not used with FreeRTOS)
// ============================================================================
void loop() {
    // Empty - FreeRTOS tasks handle everything
}
