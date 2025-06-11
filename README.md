
# Smart Patrol Robot with Environmental and Human Monitoring

This project implements a mobile robotic system that patrols an area, avoiding obstacles, detecting motion, verifying human presence via body temperature, and monitoring for toxic gases such as **CO₂** and **SO₂**. It uses various sensors and outputs alerts using a buzzer and LED when potential hazards or humans in distress are detected.

## Features

- **Autonomous Navigation** with ultrasonic obstacle avoidance  
- **Human Presence Detection** using PIR sensor and temperature validation  
- **Temperature Monitoring** with LM35 analog sensor  
- **Gas Detection** using MQ-136 (SO₂) and MG-811 (CO₂) sensors  
- **Audible and Visual Alerts** with buzzer and LED  
- **Motion-based human tracking** for rescue or safety monitoring scenarios

## Hardware Components

| Component             | Description                            |
|----------------------|----------------------------------------|
| Arduino Board         | Microcontroller (Uno3.)      |
| HC-SR04              | Ultrasonic Distance Sensor              |
| LM35                 | Temperature Sensor                      |
| MG-811               | CO₂ Gas Sensor                          |
| MQ-136               | SO₂ Gas Sensor                          |
| PIR Motion Sensor    | For detecting human motion              |
| Buzzer               | Audible alert system                    |
| LED                  | Visual alert indicator                  |
| Motor Driver & Motors| For forward/backward movement           |
| Power Source         | Battery pack or external power          |
| Miscellaneous        | Jumper wires, resistors, breadboard     |

## Pin Configuration

```c
#define TRIG_PIN     9     // Ultrasonic TRIG
#define ECHO_PIN     8     // Ultrasonic ECHO
#define CO2_SENSOR   A0    // CO₂ analog input
#define TEMP_SENSOR  A1    // LM35 analog input
#define SO2_SENSOR   A2    // SO₂ analog input
#define PIR_SENSOR   7     // PIR motion detection
#define BUZZER_PIN   6     // Buzzer output
#define LED_PIN      5     // LED indicator
#define IN1          2     // Motor control pin 1
#define IN2          3     // Motor control pin 2
#define IN3          4     // Motor control pin 3
#define IN4          10    // Motor control pin 4
```

## How It Works

1. **Obstacle Avoidance**  
   The HC-SR04 measures the distance in front of the robot. If an object is closer than 20 cm, it stops to avoid collision.

2. **Gas Monitoring**  
   The robot continuously checks analog values from the CO₂ and SO₂ sensors. If levels exceed the set thresholds, it triggers an alert.

3. **Human Detection**  
   The PIR sensor detects motion. If detected, it verifies human presence by checking body temperature using the LM35. If within the human range (36.5°C - 42°C), it raises an alert.

4. **Motion Behavior**  
   If motion is detected, the robot briefly moves forward toward the motion source and stops. If no motion is detected for 5 seconds, it resumes normal patrol.

## Threshold Values

| Parameter            | Threshold        |
|---------------------|------------------|
| Distance (cm)        | `< 20`           |
| CO₂ Analog Value     | `> 600`          |
| SO₂ Analog Value     | `> 700`          |
| Temperature (°C)     | `36.5 – 42.0`    |

You can calibrate these values based on your sensor models and environmental conditions.

## Example Output (Serial Monitor)

```
Distance: 15.32 cm
Obstacle too close. Stopping.

CO₂ Reading: 685
SO₂ Reading: 701
High CO₂ concentration detected!
High SO₂ concentration detected!
ALERT: Toxic gas detected!

Motion detected!
Temperature: 37.2 °C
Valid human body temperature detected!
ALERT: Human presence verified!
```

## Setup Instructions

1. Assemble the hardware according to the pin layout.
2. Upload the provided `.ino` code to your Arduino board using the Arduino IDE.
3. Open the Serial Monitor at 9600 baud to view live sensor data and debug information.
4. Power the robot and observe its patrol, alert, and movement behaviors.

## File Structure

```
Autonomous-SaR/
├── .pio/                   # PlatformIO build system files
├── .vscode/                # VS Code settings
├── include/                # Header files (if any)
├── lib/
│   └── README.md           # Library documentation or notes
├── src/
│   └── main.cpp            # Main Arduino source code
├── test/                   # Unit or integration tests
    └── README              # Unit tests using PlatformIO Test Runner
├── .gitignore              # Git ignore rules
├── platformio.ini          # PlatformIO project configuration
└── README.md               # Project documentation
```

## Future Improvements

- Add GPS module for location tracking
- Integrate a GSM module for SMS alerts
- Enable wireless remote control
- Add camera for visual monitoring

## License

This project is open-source and available under the [MIT License](https://opensource.org/licenses/MIT).
