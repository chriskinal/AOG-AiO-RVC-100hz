//#include <stdint.h>
//#include "usb_serial.h"
/*!
 * 	This is a library for use with thethe Adafruit BNO08x breakout:
 * 	https://www.adafruit.com/products/4754
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 */

#ifndef _BNO_rvc_H
#define _BNO_rvc_H

constexpr auto MILLI_G_TO_MS2 = 0.0098067; ///< Scalar to convert milli-gs to m/s^2;
constexpr auto DEGREE_SCALE = 0.1;        ///< To convert the degree values

#include "Arduino.h"
#include <Wire.h>
#include "elapsedMillis.h"

struct BNO_RVC_DATA {
    int16_t yawX10;       // Yaw in Degrees x 10
    int16_t pitchX10;     // Pitch in Degrees x 10
    int16_t rollX10;      // Roll in Degrees x 10
    int16_t yawX100;      // Yaw in original x100
    int16_t angVel;       // running total of angular velocity
};

class BNO_RVC
{
private:
  HardwareSerial *serial_dev;
  int16_t prevYAw;
  elapsedMillis timeoutTimer;
  const uint8_t timeoutPeriod = 15;         // (ms) We expect a BNO update every 10ms


public:
  uint32_t angCounter;
  bool isSwapXY = false;
  bool isActive;
  BNO_RVC_DATA rvcData;

  bool begin(HardwareSerial* theSerial)
  {
    serial_dev = theSerial;
    serial_dev->begin(115200);  // This is the baud rate specified by the BNO datasheet for serial RVC
    elapsedMillis timeout = 0;
    while (!read())   // wait/check for BNO RVC serial data
    {
      if (timeout > 25)   // data should arrive every 10ms
      {
        Serial.print("\r\n-* No BNO-085 in RVC mode detected *");
        return false;
      }
    }
    
    Serial.print("\r\n- BNO-085 RVC detected");
    Serial.printf("\r\n  - yaw:%i , pitch:%i , roll:%i", rvcData.yawX10, rvcData.pitchX10, rvcData.rollX10);
    return true;
  }

//read the 16 byte sentence AA AA Index Yaw Pitch Roll LSB MSB
  bool read(bool _clear = 0)
  {
    if (timeoutTimer > timeoutPeriod && isActive) {
      isActive = false;
      if (!_clear) Serial.print("\r\n*** BNO missed update ***");
      /*rvcData.pitchX10 = 0;
      rvcData.rollX10 = 0;
      rvcData.yawX10 = 65535;
      rvcData.yawX100 = 0;*/
      //timeoutTimer = 0;
    }

    if (!serial_dev->available()) return false;

    if (serial_dev->peek() != 0xAA)
    {
      serial_dev->read();
      return false;
    }

    if (serial_dev->available() < 19) return false;     // are all 19 bytes ready?

    // at this point we know there's at least 19 bytes available and the first is AA
    if (serial_dev->read() != 0xAA) return false;

    // make sure the next byte is the second 0xAA
    if (serial_dev->read() != 0xAA) return false;

    uint8_t buffer[19];
    if (!serial_dev->readBytes(buffer, 17)) return false;

    // get checksum ready
    uint8_t sum = 0;
    for (uint8_t i = 0; i < 16; i++) sum += buffer[i];

    if (sum != buffer[16]) return false;

    //clean out any remaining bytes in case teensy was busy
    uint16_t extra = serial_dev->available();
    if (extra > 0 && !_clear) { Serial.print((String)"\r\n" + millis() + " *** BNO serial input buffer had " + extra + " bytes leftover! ***"); }
    while (serial_dev->available() > 0) serial_dev->read();

    //Serial.print((String)"\r\nBNO update " + millis());

    int16_t temp;

    if (angCounter < 20)
    {
      temp = buffer[1] + (buffer[2] << 8);
      rvcData.yawX100 = temp; //For angular velocity calc
      rvcData.angVel += (temp - prevYAw);
      angCounter++;
      prevYAw = temp;
    }
    else
    {
      angCounter = 0;
      prevYAw = rvcData.angVel = 0;
      temp = 0;
    }

    rvcData.yawX10 = (int16_t)((float)temp * DEGREE_SCALE);
    if (rvcData.yawX10 < 0) rvcData.yawX10 += 3600;

    temp = buffer[3] + (buffer[4] << 8);
    rvcData.pitchX10 = (int16_t)((float)temp * DEGREE_SCALE);

    temp = buffer[5] + (buffer[6] << 8);
    rvcData.rollX10 = (int16_t)((float)temp * DEGREE_SCALE); //Confirmed X as stock direction of travel

    isActive = true;
    timeoutTimer = 0;
    return true;
  }
};

#endif