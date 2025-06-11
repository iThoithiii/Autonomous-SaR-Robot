#include <Arduino.h>

// Pin Definitions
#define TRIG_PIN     9           // Ultrasonic sensor TRIG pin
#define ECHO_PIN     8           // Ultrasonic sensor ECHO pin
#define CO2_SENSOR   A0          // CO₂ gas sensor (MG-811)
#define TEMP_SENSOR  A1          // Temperature sensor (LM35)
#define SO2_SENSOR   A2          // SO₂ gas sensor (MQ-136)
#define PIR_SENSOR   7           // Motion sensor for human detection
#define BUZZER_PIN   6           // Audible alarm output
#define LED_PIN      5           // Visual alert indicator (LED)
#define IN1          2           // Motor control pin 1
#define IN2          3           // Motor control pin 2
#define IN3          4           // Motor control pin 3
#define IN4          10          // Motor control pin 4

// Thresholds (Can be calibrated later)
#define TEMP_THRESHOLD_LOW   36.5   // Minimum human body temperature (°C)
#define TEMP_THRESHOLD_HIGH  42.0   // Maximum safe human body temperature (°C)
#define CO2_THRESHOLD        600    // CO₂ gas level threshold (analog units)
#define SO2_THRESHOLD        700    // SO₂ gas level threshold (analog units)
#define DIST_THRESHOLD_CM    20     // Safe distance from obstacles (cm)

// Timing Variables
unsigned long motionLastDetected = 0;            // Timestamp of last detected motion
const unsigned long humanSearchDuration = 5000;  // Keep moving for 5s after losing motion

// Function Declarations
float getDistance(void);            // Measures distance using ultrasonic sensor
float getTemperature(void);         // Reads temperature from LM35
void stopMoving(void);              // Stops robot motors
void moveForward(void);             // Moves robot forward
void moveBackward(void);            // Moves robot backward
void moveTowardHuman(void);         // Approaches detected human
void alert(void);                   // Activates buzzer and LED for alert
void checkForGas(void);             // Detects CO₂ and SO₂ levels
void checkForHumanPresence(void);   // Detects motion and verifies temperature

void setup() {
    Serial.begin(9600);

    // Sensor input pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(CO2_SENSOR, INPUT);
    pinMode(SO2_SENSOR, INPUT);
    pinMode(PIR_SENSOR, INPUT);
    pinMode(TEMP_SENSOR, INPUT);

    // Output pins
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Initialize alerts to OFF
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
}

void loop() {
    float distance = getDistance();

    // Obstacle avoidance check
    if (distance < DIST_THRESHOLD_CM) {
        Serial.println("Obstacle detected! Reversing...");
        stopMoving();
        delay(300);
        moveBackward();       // Move back briefly to avoid
        delay(800);
        stopMoving();
    } else {
        moveForward();        // Path is clear
    }

    checkForGas();
    checkForHumanPresence();

    delay(100); // Small delay for sensor stability
}

// Measure distance using ultrasonic sensor
float getDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH);
    return duration * 0.034 / 2;  // Convert duration to cm
}

// Read temperature from LM35 sensor
float getTemperature() {
    int value = analogRead(TEMP_SENSOR);
    float voltage = value * 5.0 / 1023.0;
    return voltage * 100.0; // Convert voltage to °C
}

// Moves the robot forward
void moveForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

// Moves the robot backward
void moveBackward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

// Stop all motors
void stopMoving() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

// Move briefly toward detected motion
void moveTowardHuman() {
    Serial.println("Moving toward human...");
    moveForward();
    delay(1000); // Move forward for 1 second
    stopMoving();
}

// Activate LED and buzzer in alert pattern
void alert() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        digitalWrite(LED_PIN, HIGH);
        delay(300);
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
        delay(200);
    }
}

// Differentiating CO₂ and SO₂ gases using two sensors
void checkForGas() {
    int co2Value = analogRead(CO2_SENSOR);
    int so2Value = analogRead(SO2_SENSOR);

    Serial.print("CO₂ Reading: ");
    Serial.println(co2Value);
    Serial.print("SO₂ Reading: ");
    Serial.println(so2Value);

    bool gasAlert = false;

    if (co2Value > CO2_THRESHOLD) {
        Serial.println("High CO₂ concentration detected!");
        gasAlert = true;
    }

    if (so2Value > SO2_THRESHOLD) {
        Serial.println("High SO₂ concentration detected!");
        gasAlert = true;
    }

    if (gasAlert) {
        alert(); // Trigger warning if any gas exceeds safe threshold
    }
}

// Detects motion and verifies if it’s a human based on body temp
void checkForHumanPresence() {
    int motion = digitalRead(PIR_SENSOR);
    float temperature = getTemperature();

    if (motion == HIGH) {
        Serial.println("Motion detected!");
        moveTowardHuman();
        motionLastDetected = millis(); // Reset the human search timer

        if (temperature >= TEMP_THRESHOLD_LOW && temperature <= TEMP_THRESHOLD_HIGH) {
            Serial.println("Valid human body temperature detected!");
            alert(); // Person might need help or attention
        } else {
            Serial.println("No valid body heat. Could be a false trigger.");
        }
    }

    // Resume normal patrol if no motion recently
    if (millis() - motionLastDetected > humanSearchDuration) {
        moveForward();
    }
}