#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
void storeData(sensors_event_t* event);
void printData();
void readIMU();

void setup(void)
{
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
}
float roll, pitch, yaw, rollRate, pitchRate, yawRate;
void loop(void)
{
  readIMU();
  printData();
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
void readIMU(){
    sensors_event_t orientationData , angVelocityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    storeData(&orientationData);
    storeData(&angVelocityData);
}
void storeData(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    yaw = event->orientation.x;
    pitch = event->orientation.y;
    roll = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    yawRate = event->gyro.x;
    pitchRate = event->gyro.y;
    rollRate = event->gyro.z;
  }
}

void printData(){
  Serial.print("Orient:"); 
  Serial.print("\ty= ");
  Serial.print(yaw);
  Serial.print(" |\tp= ");
  Serial.print(pitch);
  Serial.print(" |\tr= ");
  Serial.println(roll);    

  Serial.print("Gyro:");
  Serial.print("\tyr= ");
  Serial.print(yawRate);
  Serial.print(" |\tpr= ");
  Serial.print(pitchRate);
  Serial.print(" |\trr= ");
  Serial.println(rollRate);  
}