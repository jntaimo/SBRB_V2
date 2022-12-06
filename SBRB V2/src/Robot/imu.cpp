#include <string.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void imuSetup()
{
    if(!bno.begin())
    {
    /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    }
    delay(1000);
    bno.setExtCrystalUse(true);
}

// Loop that gets gyro sensor values and prints them out continuously
void imuLoop() {
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");
  
  //delay(100);
}

// Takes in references to variables to store gyro info, and directly changes them
void readGyro(float &x, float &y, float &z){
    sensors_event_t event; 
    bno.getEvent(&event);
    x = event.orientation.x;
    y = event.orientation.y;
    z = event.orientation.z;
    
}

void readPitchRate(float &pitchRate){
imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  pitchRate = gyro.z();
}
