#ifndef MMA8452Q_H
#define MMA8452Q_H

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

#include <Arduino.h>

// --------------------------------------------------
// Registers

enum class MMA8452Q_Register : uint8_t {
  STATUS            = 0x00,
  OUT_X_MSB         = 0x01,
  OUT_X_LSB         = 0x02,
  OUT_Y_MSB         = 0x03,
  OUT_Y_LSB         = 0x04,
  OUT_Z_MSB         = 0x05,
  OUT_Z_LSB         = 0x06,
  SYSMOD            = 0x0B,
  INT_SOURCE        = 0x0C,
  WHO_AM_I          = 0x0D,
  XYZ_DATA_CFG      = 0x0E,
  HP_FILTER_CUTOFF  = 0x0F,
  PL_STATUS         = 0x10,
  PL_CFG            = 0x11,
  PL_COUNT          = 0x12,
  PL_BF_ZCOMP       = 0x13,
  P_L_THS_REG       = 0x14,
  FF_MT_CFG         = 0x15,
  FF_MT_SRC         = 0x16,
  FF_MT_THS         = 0x17,
  FF_MT_COUNT       = 0x18,
  TRANSIENT_CFG     = 0x1D,
  TRANSIENT_SRC     = 0x1E,
  TRANSIENT_THS     = 0x1F,
  TRANSIENT_COUNT   = 0x20,
  PULSE_CFG         = 0x21,
  PULSE_SRC         = 0x22,
  PULSE_THSX        = 0x23,
  PULSE_THSY        = 0x24,
  PULSE_THSZ        = 0x25,
  PULSE_TMLT        = 0x26,
  PULSE_LTCY        = 0x27,
  PULSE_WIND        = 0x28,
  ASLP_COUNT        = 0x29,
  CTRL_REG1         = 0x2A,
  CTRL_REG2         = 0x2B,
  CTRL_REG3         = 0x2C,
  CTRL_REG4         = 0x2D,
  CTRL_REG5         = 0x2E,
  OFF_X             = 0x2F,
  OFF_Y             = 0x30,
  OFF_Z             = 0x31
};

// --------------------------------------------------
// Constants

enum class MMA8452Q_FastRead : uint8_t    { FAST_READ_OFF = 0 , FAST_READ_ON = 1 };
enum class MMA8452Q_Mode : uint8_t        { MODE_NORMAL = 0x00 , MODE_LNLP = 0x01 , MODE_HIGH_RES = 0x02 , MODE_LOW_POWER = 0x03 };
enum class MMA8452Q_ODR : uint8_t         { ODR_800 = 0 , ODR_400 , ODR_200 , ODR_100 , ODR_50 , ODR_12 , ODR_6 , ODR_1 };
enum class MMA8452Q_Scale : uint8_t       { SCALE_2G = 2 , SCALE_4G = 4 , SCALE_8G = 8 };
enum class MMA8452Q_SYSMOD : uint8_t      { SYSMOD_STANDBY = 0x00 , SYSMOD_WAKE = 0x01 , SYSMOD_SLEEP = 0x02 };

// --------------------------------------------------
// Class

class MMA8452Q {
  public:
    int16_t raw_x, raw_y, raw_z;
    float x, y, z;
    
    MMA8452Q(uint8_t = 0x1D);

    uint8_t available(void);
    uint8_t active(void);
    uint8_t getFastRead(void);
    uint8_t getHighPassOutput(void);
    uint8_t getMode(bool);
    uint8_t getODR(void);
    uint8_t getScale(void);
    MMA8452Q_SYSMOD getState(void);
    uint8_t init(MMA8452Q_Scale = MMA8452Q_Scale::SCALE_8G, MMA8452Q_ODR = MMA8452Q_ODR::ODR_400);
    void read(void);
    uint8_t setFastRead(MMA8452Q_FastRead);
    uint8_t setHighPassOutput(bool);
    uint8_t setMode(MMA8452Q_Mode, bool);
    uint8_t setODR(MMA8452Q_ODR, bool = false);
    uint8_t setScale(MMA8452Q_Scale);
    uint8_t standby(void);
    
    uint8_t readRegister(MMA8452Q_Register);
    void readRegisters(MMA8452Q_Register reg, uint8_t *, uint8_t);
    uint8_t writeRegister(MMA8452Q_Register, uint8_t);
    uint8_t writeRegisters(MMA8452Q_Register, uint8_t *, uint8_t);

  private:
    uint8_t _address;
    MMA8452Q_FastRead _fast_read;
    MMA8452Q_Scale _scale;
    
};

// --------------------------------------------------

#endif  // MMA8452Q_H
