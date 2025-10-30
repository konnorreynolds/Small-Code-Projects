#include <main.h>

Motor leftMotor = {GPIO_NUM_32, GPIO_NUM_33, 0, 1};
Motor rightMotor = {GPIO_NUM_25, GPIO_NUM_26, 2, 3};

/*
 * Test Sequence to go half-duty cycle, full-duty cycle, then slow duty cycle.
 */
void dutyCycleTest(Motor motor);
void driveMotorSpeeds();

// Forward Definitions
unsigned long prevMillis = 0;
constexpr long interval = 500;
void updateLED();
void onUpdate(const XboxControlsEvent& event);
void onConnect(const XboxController& controller);
void onDisconnect(const XboxController& controller);

XboxController xboxController;
bool connected = false;


void setup() {
// write your initialization code here
    Serial.begin(115200);
    while (!Serial) {
        // wait for serial port to connect.
    }
    xboxController.begin();
    delay(1000);
    xboxController.onConnect(onConnect); // LED Should Blink
    xboxController.onDisconnect(onDisconnect); // No LED
    pinMode(GPIO_NUM_2, OUTPUT);
}

void loop() {
    if (xboxController.isConnected()) {
        driveMotorSpeeds();
        updateLED();
    }
}

void updateLED() {
    unsigned long currentMillis = millis();
    // If the controller is connected
        // If time since last toggle is greater or equal to the interval
        if (currentMillis - prevMillis >= interval) {
            prevMillis = currentMillis;
            // Toggle LED
            if (digitalRead(GPIO_NUM_2) == HIGH) {
                digitalWrite(GPIO_NUM_2, LOW);
            } else {
                digitalWrite(GPIO_NUM_2, HIGH);
            }
        }
}

void driveMotorSpeeds() {
    XboxControlsEvent event;
    xboxController.read(event);
    const MotorSpeeds speeds = tankDrive(event.leftStickY, event.rightStickY);
    dutyCycleControl(leftMotor, speeds.leftSpeed);
    dutyCycleControl(rightMotor, speeds.rightSpeed);
}

void onConnect(const XboxController& controller) {
    Serial.printf("controller connected, address: %s\n", controller.getAddress().toString().c_str());
    XboxVibrationsCommand vibrationCommand;
    vibrationCommand.leftMotor = 1.0f;
    vibrationCommand.rightMotor = 1.0f;
    xboxController.write(vibrationCommand);
    connected = true;
}

void onDisconnect(const XboxController& controller) {
    Serial.printf("controller disconnected, address: %s\n", controller.getAddress().toString().c_str());
    connected = false;
}