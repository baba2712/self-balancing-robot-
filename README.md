# Self-Balancing Robot

This project is a self-balancing robot that uses an **MPU-6050 IMU** (Inertial Measurement Unit) to detect the robot's tilt and a **PID controller** to adjust the motor speed, keeping the robot balanced in an upright position.

## Features

- **MPU-6050** accelerometer and gyroscope sensor for tilt detection.
- **PID control** to maintain balance by adjusting motor speeds.
- **Arduino platform** for controlling the robot.

## Components

- **Arduino Board** (e.g., Arduino Uno, Nano)
- **MPU-6050 IMU sensor**
- **DC motors** (with H-bridge motor driver, such as L298N or similar)
- **Power supply** (e.g., 7.4V Li-ion battery pack)
- **Wires and breadboard** (optional for prototyping)

## Wiring

### MPU-6050:

- **VCC** -> 3.3V or 5V (depending on your Arduino model)
- **GND** -> GND
- **SDA** -> A4 (on Arduino Uno/Nano) or corresponding SDA pin on other boards
- **SCL** -> A5 (on Arduino Uno/Nano) or corresponding SCL pin on other boards

### Motors:

- Motor driver pins connected to motor driver IC (L298N, L293D, etc.).
- Arduino pins (e.g., pins 3, 4, 5, 6) control the motors through the motor driver.

## Software

### Libraries

- **Wire**: Arduino library to communicate with the MPU-6050 sensor via I2C.
- **MPU6050**: Library to interface with the MPU-6050 sensor.
- **PID_v1**: PID library to control the balancing mechanism.

To install these libraries, go to the **Arduino IDE** and navigate to **Sketch > Include Library > Manage Libraries**. Then search for and install the following libraries:

- **MPU6050** by Electronic Cats
- **PID_v1** by Brett Beauregard

### Arduino Sketch

The main logic of the self-balancing robot is contained in the `self_balancing_robot.ino` sketch. The code reads the accelerometer and gyroscope data from the MPU-6050 sensor, applies a complementary filter to compute the angle, and then uses a PID controller to adjust the motor speeds and maintain balance.

The PID constants (**Kp**, **Ki**, **Kd**) can be tuned based on the robot's response to improve performance.

## How to Use

1. **Hardware Setup**: Connect all the components as described in the wiring section above.
2. **Upload Code**: Open the `self_balancing_robot.ino` file in the Arduino IDE and upload it to your Arduino board.
3. **Power On**: Power the robot, and it should begin balancing on its own.
4. **PID Tuning**: Adjust the PID constants in the code if the robot doesn't balance correctly. Start with adjusting **Kp**, then **Kd**, and finally **Ki** if needed.

## PID Tuning

- **Kp (Proportional)**: Controls the response to the current error (tilt). A higher value makes the robot react more quickly to tilts.
- **Ki (Integral)**: Controls the correction based on the sum of past errors. It helps eliminate small steady-state errors.
- **Kd (Derivative)**: Controls the rate of change of the error to help reduce oscillations and improve stability.

### Example PID Tuning Process:

1. Start by adjusting **Kp** to make the robot react to tilts.
2. If the robot oscillates too much, increase **Kd** to reduce overshooting.
3. Adjust **Ki** if there is a persistent drift in one direction.

## Troubleshooting

- **Robot doesn't balance**:
  - Check the wiring and ensure the MPU-6050 sensor is connected correctly.
  - Verify the PID constants are suitable for your robot’s design.
  
- **Robot doesn't move or is jittery**:
  - Ensure the motor driver and motor connections are correct and functional.
  - Make sure the robot is placed on a flat surface during testing.

## License

This project is open-source and available under the MIT license. Feel free to modify, improve, and use the code as needed.

## Acknowledgements

- **MPU6050 Library** by Electronic Cats: [GitHub Link](https://github.com/jrowberg/i2cdevlib)
- **PID_v1 Library** by Brett Beauregard: [GitHub Link](https://github.com/br3ttb/Arduino-PID-Library)
