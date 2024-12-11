#include <Wire.h>
#include <Servo.h>

const int motor1Pin1 = 3;
const int motor1Pin2 = 5;
const int motor2Pin1 = 6;
const int motor2Pin2 = 9;

const int MPU = 0x68;
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float angleX, angleY;
float baseAngle = 0;
float error, prevError, totalError, pidOutput;
float kp = 12, ki = 0.005, kd = 1.5;
unsigned long prevTime;

void setup() {
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);
    
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    
    prevTime = millis();
}

void loop() {
    unsigned long currentTime = millis();
    float elapsedTime = (currentTime - prevTime) / 1000.0;
    prevTime = currentTime;
    
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 14, true);
    
    accX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    accY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    accZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
    gyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
    gyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    gyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
    
    float angleAccX = atan2(accY, accZ) * 180 / 3.14159;
    angleX = 0.98 * (angleX + gyroX * elapsedTime) + 0.02 * angleAccX;
    
    error = baseAngle - angleX;
    totalError += error * elapsedTime;
    float rateError = (error - prevError) / elapsedTime;
    prevError = error;
    
    pidOutput = kp * error + ki * totalError + kd * rateError;
    
    if (pidOutput > 255) pidOutput = 255;
    if (pidOutput < -255) pidOutput = -255;
    
    if (pidOutput > 0) {
        analogWrite(motor1Pin1, pidOutput);
        analogWrite(motor1Pin2, 0);
        analogWrite(motor2Pin1, pidOutput);
        analogWrite(motor2Pin2, 0);
    } else {
        analogWrite(motor1Pin1, 0);
        analogWrite(motor1Pin2, -pidOutput);
        analogWrite(motor2Pin1, 0);
        analogWrite(motor2Pin2, -pidOutput);
    }
} 
