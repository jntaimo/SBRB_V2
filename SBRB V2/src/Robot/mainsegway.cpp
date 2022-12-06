#include <Arduino.h>
#include "imu.h"
#include "PID.h"
//Delay in milliseconds
long loopDelay = 10;
long prevLoopTime = 0;
float gyroYaw, gyroPitch, gyroRoll, gyroPitchRate;


void setup() {
  Serial.begin(115200);
  motorSetup();
  imuSetup();

}



void loop() {
  readGyro(gyroYaw, gyroRoll, gyroPitch);
  readPitchRate(gyroPitchRate);
  if (millis() > prevLoopTime + loopDelay){
    prevLoopTime = millis();
    runPID(gyroYaw, gyroPitch, gyroPitchRate);
   // Serial.printf("Yaw: %.5f, Pitch: %.5f, Roll: %.5f\n", gyroYaw, gyroPitch, gyroRoll);
  }


}
