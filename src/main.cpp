#include <Arduino.h>

// Pin Definitions
#define TRIG1_PIN     9           // Ultrasonic sensor 1 TRIG pin
#define ECHO1_PIN     8           // Ultrasonic sensor 1 ECHO pin
#define TRIG2_PIN     12          // Ultrasonic sensor 2 TRIG pin (new)
#define ECHO2_PIN     11          // Ultrasonic sensor 2 ECHO pin (new)
#define MQ2_SENSOR    A0          // MQ-2 Gas sensor
#define TEMP_SENSOR   A1          // Temperature sensor (LM35)
#define PIR_SENSOR    7           // Motion sensor
#define BUZZER_PIN    6           // Alarm buzzer
#define RED_PIN       3           // RGB LED Red
#define GREEN_PIN     4           // RGB LED Green
#define BLUE_PIN      5           // RGB LED Blue
#define IN1           2           // Motor driver pin
#define IN2           13          // Motor driver pin
#define IN3           A2          // Motor driver pin
#define IN4           A3          // Motor driver pin

// Thresholds
#define TEMP_THRESHOLD_LOW   36.5
#define TEMP_THRESHOLD_HIGH  42.0
#define GAS_THRESHOLD        450   // MQ-2 analog value (adjustable)
#define DIST_THRESHOLD_CM    20

// Timing Variables
unsigned long motionLastDetected = 0;
const unsigned long humanSearchDuration = 5000;

// Function Declarations
float getDistance(uint8_t trigPin, uint8_t echoPin);
float getTemperature();
void stopMoving();
void moveForward();
void moveBackward();
void moveTowardHuman();
void alert();
void checkForGas();
void checkForHumanPresence();

void setup() {
    Serial.begin(9600);

    pinMode(TRIG1_PIN, OUTPUT);
    pinMode(ECHO1_PIN, INPUT);
    pinMode(TRIG2_PIN, OUTPUT);
    pinMode(ECHO2_PIN, INPUT);
    pinMode(MQ2_SENSOR, INPUT);
    pinMode(TEMP_SENSOR, INPUT);
    pinMode(PIR_SENSOR, INPUT);

    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(BLUE_PIN, OUTPUT);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    digitalWrite(BUZZER_PIN, LOW);
    analogWrite(RED_PIN, 0);
    analogWrite(GREEN_PIN, 0);
    analogWrite(BLUE_PIN, 0);
}

void loop() {
    float distance1 = getDistance(TRIG1_PIN, ECHO1_PIN);
    float distance2 = getDistance(TRIG2_PIN, ECHO2_PIN);

    if (distance1 < DIST_THRESHOLD_CM || distance2 < DIST_THRESHOLD_CM) {
        Serial.println("Obstacle detected! Reversing...");
        stopMoving();
        delay(300);
        moveBackward();
        delay(800);
        stopMoving();
    } else {
        moveForward();
    }

    checkForGas();
    checkForHumanPresence();
    delay(100);
}

float getDistance(uint8_t trigPin, uint8_t echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    return duration * 0.034 / 2;
}

float getTemperature() {
    int value = analogRead(TEMP_SENSOR);
    float voltage = value * 5.0 / 1023.0;
    return voltage * 100.0;
}

void moveForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void moveBackward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void stopMoving() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void moveTowardHuman() {
    Serial.println("Moving toward human...");
    moveForward();
    delay(1000);
    stopMoving();
}

void alert() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        analogWrite(RED_PIN, 255);
        analogWrite(GREEN_PIN, 0);
        analogWrite(BLUE_PIN, 0);
        delay(300);
        digitalWrite(BUZZER_PIN, LOW);
        analogWrite(RED_PIN, 0);
        delay(200);
    }
}

void checkForGas() {
    int gasVal = analogRead(MQ2_SENSOR);
    Serial.print("MQ-2 Gas Reading: ");
    Serial.println(gasVal);
    if (gasVal > GAS_THRESHOLD) {
        Serial.println("High gas concentration detected!");
        alert();
    }
}

void checkForHumanPresence() {
    int motion = digitalRead(PIR_SENSOR);
    float temperature = getTemperature();

    if (motion == HIGH) {
        Serial.println("Motion detected!");
        moveTowardHuman();
        motionLastDetected = millis();

        if (temperature >= TEMP_THRESHOLD_LOW && temperature <= TEMP_THRESHOLD_HIGH) {
            Serial.println("Valid human body temperature detected!");
            alert();
        } else {
            Serial.println("No valid body heat. Could be a false trigger.");
        }
    }

    if (millis() - motionLastDetected > humanSearchDuration) {
        moveForward();
    }
}