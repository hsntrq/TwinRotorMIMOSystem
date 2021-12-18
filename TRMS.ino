#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include "I2C.h"
#include "motor.h"
#include "rpm.h"

//#define MEASURE_RPM

Kalman kalmanX; // Create the Kalman instance

/* IMU Data */
double accY, accZ;
double gyroX;

double pitch;

double gyroXrate;
double gyroXangle; // Angle calculate using the gyro only
double kalAngleX; // Calculated angle using a Kalman filter

uint32_t timer;
double dt;
uint8_t i2cData[14]; // Buffer for I2C data

double PID, error, previous_error;
double pid_p=0;
double pid_i=0;
double pid_d=0;
/////////////////PID CONSTANTS/////////////////
double kp=1.5;//3.55
double ki=0.004;//0.003
double kd=0.01;//2.05
///////////////////////////////////////////////

//double controlSpeed=0; //initial value of throttle to the motors
double desired_angle = 10;


void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  timer = micros();
  initMPU();

  #ifdef MEASURE_RPM
    pinMode(3, INPUT);
    attachInterrupt(0, RPM, FALLING);
  #endif
}

void loop() {
  readMPU();
  Serial.print("Angle: ");Serial.print(kalAngleX);Serial.print("\t\t");Serial.print("PID: ");
    
  //control logic
  previous_error = error;
  error = kalAngleX - desired_angle;
  pid_p = kp*error;
  
  if(-3 <error <3)
  {
    pid_i = pid_i+(ki*error);  
  }

  pid_d = kd*((error - previous_error)/dt);

  PID = pid_p + pid_i + pid_d;
  if (PID>35) PID = 35;
  else if (PID<-35) PID = -35;
  Serial.print(PID);
//  PID = 30;
//  Serial.print(timer);Serial.print(",");Serial.println(kalAngleX);
  actuate();
}


void actuate()
{
  motorSpeedA = map(abs(PID), 0, 35, 0, 255);
  motorSpeedB = map(abs(PID), 0, 35, 0, 255);
  Serial.print("\t\tmotor speed ");Serial.println(motorSpeedA);
  if (PID < 0)
  {
    thrustAUp();
    thrustBDown();
  }
  else if (PID > 0)
  {
    thrustBUp();
    thrustADown();
  }
}

void initMPU()
{
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    //while (1);
  }
  delay(100); // Wait for sensor to stabilize

}

void readMPU()
{
  while (i2cRead(0x3B, i2cData, 14));
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);

  dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  pitch  = atan2(accY, accZ) * RAD_TO_DEG;
  gyroXrate = gyroX / 131.0; // Convert to deg/s

  if ((pitch < -90 && kalAngleX > 90) || (pitch > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(pitch);
    kalAngleX = pitch;
    gyroXangle = pitch;
  } else
    kalAngleX = kalmanX.getAngle(pitch, gyroXrate, dt); // Calculate the angle using a Kalman filter
}
