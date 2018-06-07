/*
 *  Copyright (c) 2018, Sensirion AG <joahnnes.winkelmann@sensirion.com>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Sensirion AG nor the names of its
 *        contributors may be used to endorse or promote products derived
 *        from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <Arduino.h>

#include "scdsensor.h"
#include "i2chelper.h"

const static int I2C_ADDR = 0x61;

static const uint16_t CMD_CONT_MEASUREMENT_LEN = 4;
static const uint8_t CMD_CONT_MEASUREMENT[CMD_CONT_MEASUREMENT_LEN] =
    { 0x00, 0x10, 0x00, 0x00 };

// Note: in this library, we're hardcoding the interval to 2 seconds
static const uint16_t CMD_SET_INTERVAL_LEN = 4;
static const uint8_t CMD_SET_INTERVAL[CMD_SET_INTERVAL_LEN] =
    { 0x46, 0x00, 0x00, 0x02 };

static const uint16_t CMD_STOP_MEASUREMENT_LEN = 2;
static const uint8_t CMD_STOP_MEASUREMENT[CMD_STOP_MEASUREMENT_LEN] =
    { 0x01, 0x04 };

static const uint16_t CMD_READ_MEASUREMENT_LEN = 2;
static const uint16_t CMD_READ_MEASUREMENT_RESULT_LEN = 18;
static const uint8_t CMD_READ_MEASUREMENT[CMD_READ_MEASUREMENT_LEN] =
    { 0x03, 0x00 };

static const uint16_t CMD_DATA_READY_LEN = 2;
static const uint16_t CMD_DATA_READY_RESULT_LEN = 20;
static const uint8_t CMD_DATA_READY[CMD_DATA_READY_LEN] =
    { 0x02, 0x02 };


float ScdSensor::convertToFloat(uint8_t* data)
{
  uint32_t val = (((uint32_t)data[0] << 24) |
                  ((uint32_t)data[1] << 16) |
                  ((uint32_t)data[3] <<  8) |
                  ((uint32_t)data[4]));
  return *(float*)&val;
}


int ScdSensor::init()
{
  int ret = I2CHelper::i2c_write(I2C_ADDR, CMD_SET_INTERVAL, CMD_SET_INTERVAL_LEN, true);
  if (ret != 0) {
    return 1;
  }

  // workaround for firmware bug in early samples
  delay(100);

  ret = I2CHelper::i2c_write(I2C_ADDR, CMD_CONT_MEASUREMENT, CMD_CONT_MEASUREMENT_LEN, true);
  if (ret != 0) {
    return 2;
  }

  return 0;
}

int ScdSensor::stopMeasurement()
{
  return I2CHelper::i2c_write(I2C_ADDR, CMD_STOP_MEASUREMENT, CMD_STOP_MEASUREMENT_LEN);
}

bool ScdSensor::validateCRC(uint8_t* dataBuf, uint8_t blockCount, uint8_t blockLength)
{
  // Verify uniform data block with multiple CRC sums
  //
  // data format: N blocks of M bytes, where
  //   N = blockCount
  // and
  //   M = blockLength
  //
  // Blocks are defined to have (blockLength-1) bytes of information, plus one
  // byte of CRC

  const uint8_t dataLength = blockLength-1;
  for (int i = 0; i < blockCount; ++i) {
    uint8_t pos = i * blockLength;
    if (I2CHelper::crc8(dataBuf + pos, dataLength) != dataBuf[pos + dataLength]) {
      return false;
    }
  }
  return true;
}

int ScdSensor::readSample()
{
  int ret = I2CHelper::i2c_write(I2C_ADDR, CMD_READ_MEASUREMENT, CMD_READ_MEASUREMENT_LEN);
  if (ret != 0) {
    return 1;
  }

  const uint16_t dataBufLen = CMD_READ_MEASUREMENT_RESULT_LEN;
  uint8_t dataBuf[CMD_READ_MEASUREMENT_RESULT_LEN] = { 0 };
  ret = I2CHelper::i2c_read(I2C_ADDR, dataBuf, dataBufLen);
  if (ret != 0) {
    return 1;
  }

  if (!validateCRC(dataBuf, 6, 3)) {
    return 2;
  }

  mCO2  = convertToFloat(dataBuf);
  mTemp = convertToFloat(dataBuf + 6);
  mRH   = convertToFloat(dataBuf + 12);
  return 0;
}

bool ScdSensor::checkDataReady()
{
  bool dataReady = false;
  int ret = I2CHelper::i2c_write(I2C_ADDR, CMD_DATA_READY, CMD_DATA_READY_LEN);

  if (ret != 0) {
    return 1;
  } else {
    const uint16_t bufLen = CMD_DATA_READY_RESULT_LEN;
    uint8_t buf[bufLen] = { 0 };
    ret = I2CHelper::i2c_read(I2C_ADDR, buf, bufLen);
    if (ret != 0) {
      return 2;
    } else {
      dataReady = (buf[1] == 1);
    }
  }
  return dataReady;
}


float ScdSensor::getCO2() const
{
    return mCO2;
}

float ScdSensor::getTemp() const
{
    return mTemp;
}

float ScdSensor::getRH() const
{
    return mRH;
}
