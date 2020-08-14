/*******************************************************************************
* MMA8452Q Demo (v1.0)
* 
* Simple program to read the acceleration data from the MMA8452Q
* and output it to the serial communication.
* 
* Copyright 2020 RoboCore.
* Written by Francois (05/08/20).
* 
* 
* This file is part of the MMA8452Q library ("MMA8452Q-lib").
* 
* "MMA8452Q-lib" is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* "MMA8452Q-lib" is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public License
* along with "MMA8452Q-lib". If not, see <https://www.gnu.org/licenses/>
*******************************************************************************/

// --------------------------------------------------
// Libraries

#include "RoboCore_MMA8452Q.h"

// --------------------------------------------------
// Variables

MMA8452Q accelerometer;

// --------------------------------------------------
// --------------------------------------------------

void setup(){
  // start the UART
  Serial.begin(115200);
  Serial.println("--- MMA8452Q Demo ---");

  // initialize the accelerometer with default paramenters (8G and 400 Hz)
  accelerometer.init();
}

// --------------------------------------------------
// --------------------------------------------------

void loop(){
  // read the values of the device
  accelerometer.read();

  // print the data (x,y,z)
  Serial.print(accelerometer.x);
  Serial.print('(');
  Serial.print(accelerometer.raw_x);
  Serial.print(")\t");
  Serial.print(accelerometer.y);
  Serial.print('(');
  Serial.print(accelerometer.raw_y);
  Serial.print(")\t");
  Serial.print(accelerometer.z);
  Serial.print('(');
  Serial.print(accelerometer.raw_z);
  Serial.println(')');

  delay(250); // wait some time
}

// --------------------------------------------------
// --------------------------------------------------
