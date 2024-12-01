# self-balancing-robot-

# Self-Balancing Robot

This project is a self-balancing robot that uses an **MPU-6050** IMU (Inertial Measurement Unit) to detect the robot's tilt and uses a **PID controller** to adjust the motor speed to balance the robot in an upright position.

## Features

- **MPU-6050** accelerometer and gyroscope sensor for tilt detection.
- **PID control** to maintain balance by adjusting motor speeds.
- **Arduino platform** for controlling the robot.

## Components

- **Arduino Board** (e.g., Arduino Uno, Nano)
- **MPU-6050** IMU sensor
- **DC motors** (with H-bridge motor driver, such as L298N or similar)
- **Power supply** (e.g., 7.4V Li-ion battery pack)
- **Wires and breadboard** (optional for prototyping)

## Wiring

1. **MPU-6050**:
   - VCC -> 3.3V or 5V (depending on your Arduino model)
   - GND -> GND
   - SDA -> A4 (on Arduino Uno/Nano) or corresponding SDA pin on other boards
   - SCL -> A5 (on Arduino Uno/Nano) or corresponding SCL pin on other boards

2. **Motors**:
   - Motor driver pins connected to motor driver IC (L298N, L293D, etc.).
   - Arduino pins (e.g., pins 3, 4, 5, 6) to control the motors through the motor driver.

## Software

### Libraries

- **Wire**: Arduino library to communicate with the MPU-6050 sensor via I2C.
- **MPU6050**: Library to interface with the MPU-6050 sensor.
- **PID_v1**: PID library to control the balancing mechanism.

To install these libraries, go to the **Arduino IDE** and navigate to **Sketch > Include Library > Manage Libraries**, then search for and install the following libraries:

- MPU6050 by Electronic Cats
- PID_v1 by Brett Beauregard

### Arduino Sketch

The main logic of the self-balancing robot is contained within the `self_balancing_robot.ino` sketch. The code reads the accelerometer and gyroscope data from the MPU-6050 sensor, applies a complementary filter to compute the angle, and then uses a PID controller to adjust the motor speeds and maintain balance.

The PID constants (`Kp`, `Ki`, `Kd`) can be tuned based on the robot's response to improve performance.

```cpp
// Code from the provided sketch
