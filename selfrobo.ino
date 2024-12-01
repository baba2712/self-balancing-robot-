#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>

#define motorPin1 3
#define motorPin2 4
#define motorPin3 5
#define motorPin4 6

MPU6050 mpu;

double input, output, setpoint = 0;
double Kp = 35, Ki = 20, Kd = 10;

double accelAngle, gyroAngle, angle;

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  myPID.SetMode(ACTIVE);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetTunings(Kp, Ki, Kd);
  
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);
  
  accelAngle = atan2(ay, az) * 180.0 / PI;
  gyroAngle = gx / 131.0;

  angle = 0.98 * (angle + gyroAngle * 0.01) + 0.02 * accelAngle;

  input = angle;
  myPID.Compute();

  if (output > 0) {
    analogWrite(motorPin1, output);
    analogWrite(motorPin2, 0);
    analogWrite(motorPin3, 0);
    analogWrite(motorPin4, -output);
  } else {
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, -output);
    analogWrite(motorPin3, -output);
    analogWrite(motorPin4, 0);
  }

  Serial.print("Angle: ");
  Serial.print(angle);
  Serial.print(" PID Output: ");
  Serial.println(output);

  delay(10);
}

