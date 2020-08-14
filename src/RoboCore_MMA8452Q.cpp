/*******************************************************************************
* RoboCore MMA8452Q Library (v1.0)
* 
* Library to use the MMA8452Q accelerometer.
* 
* Copyright 2020 RoboCore.
* Written by Francois (17/07/20).
* Based on the library by Jim Lindblom @ SparkFun Electronics
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
#include <Wire.h>


// --------------------------------------------------
// --------------------------------------------------

// Constructor
//  @param (address) : the address of the device in the I2C bus [uint8_t]
MMA8452Q::MMA8452Q(uint8_t address){
  _address = address;
  _fast_read = MMA8452Q_FastRead::FAST_READ_OFF;
}

// --------------------------------------------------

// Set the Active mode
//  @returns the result of the write operation (0 on success) [uint8_t]
uint8_t MMA8452Q::active(void){
  uint8_t c = readRegister(MMA8452Q_Register::CTRL_REG1);
  return writeRegister(MMA8452Q_Register::CTRL_REG1, (c | 0x01));
}

// --------------------------------------------------

// Check if new data is available
//  @returns 1 if a new set of data is avaible, 0 otherwise [uint8_t]
uint8_t MMA8452Q::available(void){
  uint8_t c = readRegister(MMA8452Q_Register::STATUS);
  c &= 0x08;
  return (c >> 3);
}

// --------------------------------------------------

// Get the Read Mode
//  @returns 1 if the mode is set to fast read, 0 otherwise [uint8_t]
uint8_t MMA8452Q::getFastRead(void){
  uint8_t c = readRegister(MMA8452Q_Register::CTRL_REG1);
  c &= 0x02;
  c >>= 1;
  return c;
}

// --------------------------------------------------

// Check if the High-Pass Output is enabled
//  @returns 1 if the high-pass filter is on, 0 otherwise [uint8_t]
uint8_t MMA8452Q::getHighPassOutput(void){
  uint8_t c = readRegister(MMA8452Q_Register::XYZ_DATA_CFG);
  c &= 0x10;
  return (c >> 4);
}

// --------------------------------------------------

// Get the current Oversampling Mode
//  @param (sleep_mode) : true for sleep mode [bool]
//  @returns the oversampling mode [uint8_t]
uint8_t MMA8452Q::getMode(bool sleep_mode){
  uint8_t c = readRegister(MMA8452Q_Register::CTRL_REG2);
  if(sleep_mode){
    c &= 0x18;
    return (c >> 3);
  } else {
    return (c & 0x03);
  }
}

// --------------------------------------------------

// Get the current Output Data Rate
//  @returns the ODR [uint8_t]
uint8_t MMA8452Q::getODR(void){
  uint8_t c = readRegister(MMA8452Q_Register::CTRL_REG1);
  c &= 0x38;
  return (c >> 3);
}

// --------------------------------------------------

// Get the current Full Scale Range
//  @returns the scale [uint8_t]
uint8_t MMA8452Q::getScale(void){
  uint8_t c = readRegister(MMA8452Q_Register::XYZ_DATA_CFG);
  c &= 0x03;
  return c;
}

// --------------------------------------------------

// Get the current System Mode
//  @returns the system mode [MMA8452Q_SYSMOD]
MMA8452Q_SYSMOD MMA8452Q::getState(void){
  return static_cast<MMA8452Q_SYSMOD>(readRegister(MMA8452Q_Register::SYSMOD));
}

// --------------------------------------------------

// Initialize the module
//  @param (scale) : the scale to configure [MMA8452Q_Scale]
//         (odr) : the output data rate to configure [MMA8452Q_ODR]
//  @returns 0 on invalid signature, 1 otherwise [uint8_t]
uint8_t MMA8452Q::init(MMA8452Q_Scale scale, MMA8452Q_ODR odr){
  Wire.begin(); // initialize I2C

  // check the device identification (should always be 0x2A)
  uint8_t c = readRegister(MMA8452Q_Register::WHO_AM_I);
  if (c != 0x2A) {
    return 0;
  }
  
  standby(); // must be in standby to change registers
  
  setScale(scale); // set up accelerometer scale
  setODR(odr); // set up output data rate
  
  active(); // set to active to start reading
  
  return 1;
}

// --------------------------------------------------

// Read the acceleration data
void MMA8452Q::read(void){
  float factor = 1.0 / (float)_scale;
  
  if(_fast_read == MMA8452Q_FastRead::FAST_READ_OFF){
    factor *= 2048.0; // 12 bits resolution
    
    uint8_t rawData[6]; // x/y/z acceleration register data stored here
    
    readRegisters(MMA8452Q_Register::OUT_X_MSB, rawData, 6); // read the six raw data registers into data array
    
    raw_x = ((int16_t)((rawData[0] << 8) | rawData[1])) >> 4;
    raw_y = ((int16_t)((rawData[2] << 8) | rawData[3])) >> 4;
    raw_z = ((int16_t)((rawData[4] << 8) | rawData[5])) >> 4;
    
  } else {
    factor *= 128.0; // 8 bits resolution
    
    uint8_t rawData[3]; // x/y/z acceleration register data stored here
    
    readRegisters(MMA8452Q_Register::OUT_X_MSB, rawData, 3); // read the three raw data registers into data array

    // careful with negative values in 8-bit mode (shift operations handle the signal bit)
    raw_x = ((int16_t)rawData[0] << 8) >> 8;
    raw_y = ((int16_t)rawData[1] << 8) >> 8;
    raw_z = ((int16_t)rawData[2] << 8) >> 8;
  }
  
  // convert
  x = (float) raw_x / factor;
  y = (float) raw_y / factor;
  z = (float) raw_z / factor;
}

// --------------------------------------------------

// Set the Fast Read mode
//  @param (fs) : the fast read mode to set [MMA8452Q_FastRead]
//  @returns 0xFF if not in standby or the result of the write operation (0 on success) [uint8_t]
uint8_t MMA8452Q::setFastRead(MMA8452Q_FastRead fs){
  // check if in standby mode
  if(getState() != MMA8452Q_SYSMOD::SYSMOD_STANDBY){
    return 0xFF;
  }
  
  uint8_t ctrl = readRegister(MMA8452Q_Register::CTRL_REG1);
  if((bool)fs){
    ctrl |= 0x02;
  } else {
    ctrl &= 0xFD;
  }
  uint8_t res = writeRegister(MMA8452Q_Register::CTRL_REG1, ctrl);
  if(res == 0)
    _fast_read = fs;
  
  return res;
}

// --------------------------------------------------

// Set the High-Pass Output
//  @param (set) : true to enable the high-pass filter [bool]
//  @returns 0xFF if not in standby or the result of the write operation (0 on success) [uint8_t]
uint8_t MMA8452Q::setHighPassOutput(bool set){
  // check if in standby mode
  if(getState() != MMA8452Q_SYSMOD::SYSMOD_STANDBY){
    return 0xFF;
  }
  
  // check if sleep mode
  uint8_t cfg = readRegister(MMA8452Q_Register::XYZ_DATA_CFG);
  if(set){
    cfg |= 0x10;
  } else {
    cfg &= 0xEF;
  }
  return writeRegister(MMA8452Q_Register::XYZ_DATA_CFG, cfg);
}

// --------------------------------------------------

// Set the Oversampling Mode
//  @param (mode) : the oversampling mode to set [MMA8452Q_Mode]
//         (sleep) : true to select the sleep mode [bool]
//  @returns 0xFF if not in standby or the result of the write operation (0 on success) [uint8_t]
uint8_t MMA8452Q::setMode(MMA8452Q_Mode mode, bool sleep){
  // check if in standby mode
  if(getState() != MMA8452Q_SYSMOD::SYSMOD_STANDBY){
    return 0xFF;
  }
  
  // check if sleep mode
  uint8_t toset = (uint8_t)mode;
  uint8_t ctrl = readRegister(MMA8452Q_Register::CTRL_REG2);
  if(sleep){
    ctrl &= 0xE7; // mask out SMOD bits
    ctrl |= (toset << 3);
  } else {
    ctrl &= 0xFC; // mask out MOD bits
    ctrl |= toset;
  }
  
  return writeRegister(MMA8452Q_Register::CTRL_REG2, ctrl);
}

// --------------------------------------------------

// Set the Outuput Data Rate
//  @param (odr) : the oversampling mode to set [MMA8452Q_ODR]
//         (sleep) : true to select the sleep mode [bool]
//  @returns 0xFF if not in standby or the result of the write operation (0 on success) [uint8_t]
uint8_t MMA8452Q::setODR(MMA8452Q_ODR odr, bool sleep){
  // check if in standby mode
  if(getState() != MMA8452Q_SYSMOD::SYSMOD_STANDBY){
    return 0xFF;
  }
  
  // check if sleep mode
  uint8_t toset = (uint8_t)odr;
  uint8_t ctrl = readRegister(MMA8452Q_Register::CTRL_REG1);
  if(sleep){
    toset &= 0xFF;
    ctrl &= 0x3F; // mask out ASLP data rate bits
    ctrl |= (toset << 6);
  } else {
    ctrl &= 0xC7; // mask out data rate bits
    ctrl |= (toset << 3);
  }
  
  return writeRegister(MMA8452Q_Register::CTRL_REG1, ctrl);
}

// --------------------------------------------------

// Set the Full Scale Range
//  @param (fsr) : the full scale range to set [MMA8452Q_Scale]
//  @returns 0xFF if not in standby or the result of the write operation (0 on success) [uint8_t]
uint8_t MMA8452Q::setScale(MMA8452Q_Scale fsr){
  // check if in standby mode
  if(getState() != MMA8452Q_SYSMOD::SYSMOD_STANDBY){
    return 0xFF;
  }
  
  uint8_t cfg = readRegister(MMA8452Q_Register::XYZ_DATA_CFG);
  cfg &= 0xFC; // mask out scale bits
  cfg |= ((uint8_t)fsr >> 2);  // neat trick, see page 22 : 00 = 2G, 01 = 4A, 10 = 8G
  uint8_t res = writeRegister(MMA8452Q_Register::XYZ_DATA_CFG, cfg);
  if(res == 0)
    _scale = fsr;

  return res;
}

// --------------------------------------------------

// Set the Standby mode
//  @returns the result of the write operation (0 on success) [uint8_t]
uint8_t MMA8452Q::standby(void){
  uint8_t c = readRegister(MMA8452Q_Register::CTRL_REG1);
  return writeRegister(MMA8452Q_Register::CTRL_REG1, (c & 0xFE));
}

// --------------------------------------------------
// --------------------------------------------------

// Read a register
//  @param (reg) : the register to read [MMA8452Q_Register]
//  @returns the number of bytes read [uint8_t]
uint8_t MMA8452Q::readRegister(MMA8452Q_Register reg){
  // select the register
  Wire.beginTransmission(_address);
  Wire.write((uint8_t)reg);
  Wire.endTransmission(false); // end the transmission but keep the connection active

  // request the data
  Wire.requestFrom(_address, (uint8_t) 1); // ask for 1 uint8_t, once done, bus is released by default
  while(!Wire.available()){ /* wait for the data to come back */ }
  
  return Wire.read();
}

// --------------------------------------------------

// Read from multiple registers
//  @param (reg) : the starting register to read [MMA8452Q_Register]
//         (data) : the buffer to store the data read [uint8_t *]
//         (length) : the number of registers (8-bit) to read [uint8_t]
void MMA8452Q::readRegisters(MMA8452Q_Register reg, uint8_t *data, uint8_t length){
  // select the register
  Wire.beginTransmission(_address);
  Wire.write((uint8_t)reg);
  Wire.endTransmission(false); // end the transmission but keep the connection active
  
  Wire.requestFrom(_address, length); // ask for N uint8_ts, once done, bus is released by default
  
  while(Wire.available() < length){ /* wait for the data to come back */ }
  
  for(uint8_t i = 0 ; i < length ; i++){
    data[i] = Wire.read();
  }
}

// --------------------------------------------------

// Write to a register
//  @param (reg) : the register to write to [MMA8452Q_Register]
//         (data) : the data to write [uint8_t]
//  @returns the result of <endTransmission()> (0 on success) [uint8_t]
uint8_t MMA8452Q::writeRegister(MMA8452Q_Register reg, uint8_t data){
  return writeRegisters(reg, &data, 1);
}

// --------------------------------------------------

// Write to multiple registers
//  @param (reg) : the starting register to write to [MMA8452Q_Register]
//         (data) : the buffer containing the data to write [uint8_t *]
//         (length) : the number of registers (8-bit) to write to [uint8_t]
//  @returns the result of <endTransmission()> (0 on success) [uint8_t]
uint8_t MMA8452Q::writeRegisters(MMA8452Q_Register reg, uint8_t *data, uint8_t length){
  Wire.beginTransmission(_address);
  Wire.write((uint8_t)reg);
  for(uint8_t i=0 ; i < length ; i++){
    Wire.write(data[i]);
  }
  return Wire.endTransmission();
}

// --------------------------------------------------
// --------------------------------------------------
