#include "mpu.h"


bool _imu_connect; 
bool _connect = false;



// ----------- ultrasonic sensor - distance

void setup(){

  Serial.begin(112500);

  // check that the IMU initializes correctly
  _imu_connect = imu_setup();

  if(_imu_connect == 0) {
   
    digitalWrite(2, HIGH); 
  }
}


void loop(){




  float* imu_ypr = imu_get_ypr();  
  //retunr from + pi to -pi 
  Serial.print("Orientation ");
  Serial.print(imu_ypr[0], 5);
  Serial.print(", ");
  Serial.print(imu_ypr[1],5);
  Serial.print(", ");
  Serial.print(imu_ypr[2],5);
  Serial.println("");
  
}   