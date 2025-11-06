// ============================================================================
// ESP32 WiFi-Controlled Robot - PlatformIO Example
// ============================================================================
// Hardware:
// - ESP32 DevKit
// - L298N Motor Driver
// - 2x DC Motors
// - Battery (7.4V for motors, USB for ESP32)
//
// Features:
// - WiFi web server control interface
// - Real-time telemetry
// - Mobile-responsive UI
// - Speed control
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <RobotLib.h>

using namespace units;
using namespace robotics;
using namespace robotlib::output;

// ============================================================================
// WiFi Configuration
// ============================================================================
const char* WIFI_SSID = "YourWiFiSSID";
const char* WIFI_PASSWORD = "YourPassword";

WebServer server(80);

// ============================================================================
// Pin Configuration (ESP32)
// ============================================================================
const int MOTOR_LEFT_FWD = 25;
const int MOTOR_LEFT_REV = 26;
const int MOTOR_LEFT_PWM = 32;

const int MOTOR_RIGHT_FWD = 27;
const int MOTOR_RIGHT_REV = 14;
const int MOTOR_RIGHT_PWM = 33;

// ============================================================================
// Robot State
// ============================================================================
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
unsigned long totalDistance = 0;
unsigned long lastUpdateTime = 0;

// ============================================================================
// Motor Control
// ============================================================================
void setMotorSpeed(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    currentLeftSpeed = leftSpeed;
    currentRightSpeed = rightSpeed;

    // Left motor
    if (leftSpeed >= 0) {
        digitalWrite(MOTOR_LEFT_FWD, HIGH);
        digitalWrite(MOTOR_LEFT_REV, LOW);
        ledcWrite(0, leftSpeed);  // Use LEDC for PWM on ESP32
    } else {
        digitalWrite(MOTOR_LEFT_FWD, LOW);
        digitalWrite(MOTOR_LEFT_REV, HIGH);
        ledcWrite(0, -leftSpeed);
    }

    // Right motor
    if (rightSpeed >= 0) {
        digitalWrite(MOTOR_RIGHT_FWD, HIGH);
        digitalWrite(MOTOR_RIGHT_REV, LOW);
        ledcWrite(1, rightSpeed);
    } else {
        digitalWrite(MOTOR_RIGHT_FWD, LOW);
        digitalWrite(MOTOR_RIGHT_REV, HIGH);
        ledcWrite(1, -rightSpeed);
    }
}

void stopMotors() {
    setMotorSpeed(0, 0);
}

// ============================================================================
// Web Server Handlers
// ============================================================================

void handleRoot() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 Robot Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {
            font-family: Arial, sans-serif;
            max-width: 500px;
            margin: 0 auto;
            padding: 20px;
            background: #1a1a1a;
            color: #fff;
        }
        h1 { text-align: center; color: #00ff00; }
        .control-pad {
            display: grid;
            grid-template-columns: 1fr 1fr 1fr;
            gap: 10px;
            margin: 20px 0;
        }
        button {
            background: #444;
            color: #fff;
            border: 2px solid #666;
            padding: 30px;
            font-size: 24px;
            border-radius: 10px;
            cursor: pointer;
        }
        button:active {
            background: #00ff00;
            color: #000;
        }
        .speed-control {
            margin: 20px 0;
        }
        input[type="range"] {
            width: 100%;
        }
        .status {
            background: #222;
            padding: 15px;
            border-radius: 10px;
            margin: 20px 0;
        }
        .stop-btn {
            background: #ff0000;
            font-size: 32px;
            padding: 40px;
        }
    </style>
</head>
<body>
    <h1>🤖 ESP32 Robot</h1>

    <div class="status">
        <div>Left Speed: <span id="leftSpeed">0</span></div>
        <div>Right Speed: <span id="rightSpeed">0</span></div>
        <div>Status: <span id="status">Ready</span></div>
    </div>

    <div class="speed-control">
        <label>Speed: <span id="speedValue">150</span></label>
        <input type="range" id="speed" min="0" max="255" value="150">
    </div>

    <div class="control-pad">
        <div></div>
        <button onclick="move('forward')">⬆️</button>
        <div></div>
        <button onclick="move('left')">⬅️</button>
        <button class="stop-btn" onclick="move('stop')">⏹️</button>
        <button onclick="move('right')">➡️</button>
        <div></div>
        <button onclick="move('backward')">⬇️</button>
        <div></div>
    </div>

    <script>
        const speedSlider = document.getElementById('speed');
        const speedValue = document.getElementById('speedValue');

        speedSlider.oninput = function() {
            speedValue.textContent = this.value;
        };

        function move(direction) {
            const speed = speedSlider.value;
            fetch('/move?dir=' + direction + '&speed=' + speed)
                .then(response => response.json())
                .then(data => {
                    document.getElementById('leftSpeed').textContent = data.left;
                    document.getElementById('rightSpeed').textContent = data.right;
                    document.getElementById('status').textContent = direction;
                });
        }

        // Update status periodically
        setInterval(() => {
            fetch('/status')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('leftSpeed').textContent = data.left;
                    document.getElementById('rightSpeed').textContent = data.right;
                });
        }, 500);
    </script>
</body>
</html>
)rawliteral";

    server.send(200, "text/html", html);
}

void handleMove() {
    String direction = server.arg("dir");
    int speed = server.arg("speed").toInt();

    if (direction == "forward") {
        setMotorSpeed(speed, speed);
    } else if (direction == "backward") {
        setMotorSpeed(-speed, -speed);
    } else if (direction == "left") {
        setMotorSpeed(-speed/2, speed/2);
    } else if (direction == "right") {
        setMotorSpeed(speed/2, -speed/2);
    } else if (direction == "stop") {
        stopMotors();
    }

    String json = "{\"left\":" + String(currentLeftSpeed) +
                  ",\"right\":" + String(currentRightSpeed) + "}";
    server.send(200, "application/json", json);
}

void handleStatus() {
    String json = "{\"left\":" + String(currentLeftSpeed) +
                  ",\"right\":" + String(currentRightSpeed) +
                  ",\"uptime\":" + String(millis()/1000) + "}";
    server.send(200, "application/json", json);
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
    Serial.begin(115200);
    println("\n\n=== ESP32 WiFi Robot ===");

    // Motor pins
    pinMode(MOTOR_LEFT_FWD, OUTPUT);
    pinMode(MOTOR_LEFT_REV, OUTPUT);
    pinMode(MOTOR_RIGHT_FWD, OUTPUT);
    pinMode(MOTOR_RIGHT_REV, OUTPUT);

    // Configure LEDC (PWM) on ESP32
    ledcSetup(0, 5000, 8);  // Channel 0, 5kHz, 8-bit
    ledcSetup(1, 5000, 8);  // Channel 1, 5kHz, 8-bit
    ledcAttachPin(MOTOR_LEFT_PWM, 0);
    ledcAttachPin(MOTOR_RIGHT_PWM, 1);

    // Connect to WiFi
    println("Connecting to WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        println("\nWiFi connected!");
        print("IP address: ");
        println(WiFi.localIP().toString().c_str());
    } else {
        println("\nWiFi connection failed!");
        println("Starting in offline mode...");
    }

    // Setup web server
    server.on("/", handleRoot);
    server.on("/move", handleMove);
    server.on("/status", handleStatus);
    server.begin();

    println("Web server started");
    println("Ready!");
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
    server.handleClient();

    // Auto-stop after 2 seconds of no commands (safety feature)
    if (millis() - lastUpdateTime > 2000) {
        if (currentLeftSpeed != 0 || currentRightSpeed != 0) {
            stopMotors();
            lastUpdateTime = millis();
        }
    }

    delay(10);
}
