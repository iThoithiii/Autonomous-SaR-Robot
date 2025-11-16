#include <Arduino.h>
#include <Adafruit_MLX90614.h>

// Pin Definitions
#define TRIG_R_PIN    7           // Ultrasonic sensor 1 TRIG pin
#define ECHO_R_PIN    8           // Ultrasonic sensor 1 ECHO pin
#define TRIG_L_PIN    16          // Ultrasonic sensor 2 TRIG pin (new)
#define ECHO_L_PIN    14          // Ultrasonic sensor 2 ECHO pin (new)
#define MQ2_SENSOR    A0          // MQ-2 Gas sensor
#define BUZZER_PIN    4           // Alarm buzzer
#define MOTOR_LEFT    10          // Left motor 
#define MOTOR_RIGHT   9           // Right Motor

// Thresholds
#define TEMP_THRESHOLD_LOW   32.5
#define TEMP_THRESHOLD_HIGH  42.0
#define GAS_THRESHOLD        450   // MQ-2 analog value (adjustable)
#define DIST_THRESHOLD_CM    10

// Temperature sensor
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// Timing Variables
unsigned long motionLastDetected = 0;
const unsigned long humanSearchDuration = 5000;

// Function Declarations
float getDistance(uint8_t trigPin, uint8_t echoPin);
float getAmbientTemperature();
float getObjectTemperature();

void stopMoving();
void moveForward();
void moveBackward();
void moveTowardHuman();
void alert();
void checkForGas();
bool checkForHumanPresence();
void search(float distanceRight, float distanceLeft);

void setup() {
    Serial.begin(9600);

    mlx.begin();

    pinMode(TRIG_R_PIN, OUTPUT);
    pinMode(ECHO_R_PIN, INPUT);
    pinMode(TRIG_L_PIN, OUTPUT);
    pinMode(ECHO_L_PIN, INPUT);
    pinMode(MQ2_SENSOR, INPUT);

    pinMode(BUZZER_PIN, OUTPUT);

    pinMode(MOTOR_LEFT, OUTPUT);
    pinMode(MOTOR_RIGHT, OUTPUT);

    digitalWrite(BUZZER_PIN, LOW);
}

void loop() {

    search(getDistance(TRIG_R_PIN, ECHO_R_PIN), getDistance(TRIG_L_PIN, ECHO_L_PIN));

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

float getAmbientTemperature() {
    return mlx.readAmbientTempC();
}

float getObjectTemperature() {
    return mlx.readObjectTempC();
}

void moveForward() {
    digitalWrite(MOTOR_LEFT, HIGH);
    digitalWrite(MOTOR_RIGHT, HIGH);
}

void turnLeft() {
    digitalWrite(MOTOR_LEFT, LOW);
    digitalWrite(MOTOR_RIGHT, HIGH);
}

void turnRight() {
    digitalWrite(MOTOR_LEFT, HIGH);
    digitalWrite(MOTOR_RIGHT, LOW);
}

void stopMoving() {
    digitalWrite(MOTOR_LEFT, LOW);
    digitalWrite(MOTOR_RIGHT, LOW);
}

void moveTowardHuman() {
    Serial.println("Moving toward human...");
    moveForward();
    delay(1000);
    stopMoving();
}

void alert() {
      tone(BUZZER_PIN, 1000, 100);
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

bool checkForHumanPresence() {
    float temperature = getObjectTemperature();

    Serial.print("Object Temperature: "); Serial.print(temperature); Serial.println(" C");

    if (temperature >= TEMP_THRESHOLD_LOW) {
        Serial.println("Valid human body temperature detected!");
        return true;
    } else {
        Serial.print("No valid body heat. Could be a false trigger.");
        Serial.print("\tTemp is ");
        Serial.println(temperature);
    }
  return false;
}

void search(float distanceRight, float distanceLeft)
{
    Serial.print("distanceRight="); Serial.print(distanceRight);
    Serial.print("\tdistanceLeft="); Serial.println(distanceLeft);

    if(checkForHumanPresence())
        alert();
    else if(distanceLeft < distanceRight)
    {
        if(distanceLeft > DIST_THRESHOLD_CM){
            turnLeft();
            delay(300);
            stopMoving();
            delay(500);
        }
        if(!checkForHumanPresence()){
            turnRight();
            delay(600);
            stopMoving();
        }
    }
    else
    {
        if(distanceRight > DIST_THRESHOLD_CM){
            turnRight();
            delay(300);
            stopMoving();
            delay(500);
        }
        if(!checkForHumanPresence()){

            turnLeft();
            delay(600);
            stopMoving();
        }
    }

}