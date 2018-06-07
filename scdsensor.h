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

#ifndef SCDSENSOR_H
#define SCDSENSOR_H

#include <inttypes.h>


class ScdSensor
{
public:
  /**
   * initialize the sensor and start continuous measurement
   * @return 0 on sucess, error code otherwise
   */
  int init();

  /**
   * Check whether there is data available to be read
   * @return true if data is ready
   */
  bool checkDataReady();

  /**
   * read sensor data from sensor
   * Note that this does NOT check whether data is ready, use checkDataReady()
   * for that
   * @return 0 on success, error code otherwise
   */
  int readSample();

  /**
   * stop the continuous measurement
   * To restart, call the init() function
   * @return 0 on success, error code otherwise
   */
  int stopMeasurement();

  /**
   * Returns the last CO2 value read - does NOT trigger a new measurement
   * @return last co2 value read
   */
  float getCO2() const;

  /**
   * Returns the last temperature value read - does NOT trigger a new measurement
   * @return last temperature value read
   */
  float getTemp() const;
  /**
   * Returns the last relative humidty value read - does NOT trigger a new measurement
   * @return last humidity value read
   */
  float getRH() const;

private:
  static float convertToFloat(uint8_t* data);
  bool  static validateCRC(uint8_t* dataBuf, uint8_t blockCount, uint8_t blockLength);

  float mCO2;
  float mTemp;
  float mRH;
};

#endif /* SCDSENSOR */
