// RVC data X10 and other parts barrowed from BNO_RVC.h by Matt Elias.

#include "Arduino.h"
#include <Wire.h>
#include "elapsedMillis.h"

class CAN_RVC
{

private:
    bool gotAAAA = false;
    bool gotMSG2 = false;
    byte rvc_data[17];
    int16_t prevYAw;
    const float MILLI_G_TO_MS2 = 0.0098067; ///< Scalar to convert milli-gs to m/s^2;
    const float DEGREE_SCALE = 0.1;        ///< To convert the degree values
    elapsedMillis timeoutTimer;
    const uint8_t timeoutPeriod = 15;         // (ms) We expect a BNO update every 10ms
    struct BNO_RVC_DATA {
    int16_t yawX10;       // Yaw in Degrees x 10
    int16_t pitchX10;     // Pitch in Degrees x 10
    int16_t rollX10;      // Roll in Degrees x 10
    int16_t yawX100;      // Yaw in original x100
    int16_t angVel;       // running total of angular velocity
    };

public:
    uint32_t angCounter;
    bool isSwapXY = false;
    bool isActive;
    BNO_RVC_DATA rvcData;

    void Sensor_Setup()
    {
        Sensor_Bus.begin();
        Sensor_Bus.setBaudRate(250000); // For TTL-CAN board
        delay(100);
        Serial.print("\r\nInitialised Sensor CANBUS @ ");
        Serial.print(Sensor_Bus.getBaudRate());
        Serial.println("bps");

        CAN_message_t SensorData;
        elapsedMillis timeout = 0;

        while ( -1 )
        {
            if ( SensorData.buf[0] == 0xAA && SensorData.buf[1] == 0xAA  )
            {
                //canrvcPresent = true;
                Serial.print("\r\n-* CANbus BNO-085 in RVC mode detected *");
                break;
            }

            if ( timeout > 25 )
            {
                Serial.print("\r\n-* No CANbus BNO-085 in RVC mode detected *");
                break;
            }
        }
    }

    void SensorBus_Receive()
    {
        CAN_message_t SensorData;

        if (Sensor_Bus.read(SensorData))
        {
            Serial.print("Millis: ");
            Serial.print(millis());
            Serial.print(" ID: ");
            Serial.print(SensorData.id, HEX);
            Serial.print(" Len: ");
            Serial.print(SensorData.len, HEX);
            Serial.print(" Data: ");
            for (uint8_t i = 0; i < 8; i++)
            {
                Serial.print(SensorData.buf[i], HEX);
                Serial.print(" ");
            }
            Serial.println();

            // Check for RVC data header in current CAN frame.
            if ((SensorData.buf[0] == 0xAA) && (SensorData.buf[1] == 0xAA))
            {
                for (uint8_t i = 2; i < 8; i++)
                {
                    rvc_data[i - 2] = SensorData.buf[i];
                }
                gotAAAA = true;
                return;
            }
            // Next frame after the header. All RVC data so stick it in the rvc_data buffer
            if (gotAAAA)
            {
                for (uint8_t i = 0; i < 8; i++)
                {
                    rvc_data[i + 6] = SensorData.buf[i];
                }
                gotMSG2 = true;
                gotAAAA = false;
                return;
            }
            // Last frame is a runt so only stick the first 3 data bytes in the rvc_data buffer.
            if (gotMSG2)
            {
                for (uint8_t i = 0; i < 3; i++)
                {
                    rvc_data[i + 14] = SensorData.buf[i];
                }
                gotMSG2 = false;
                gotAAAA = false;
                for (uint8_t i = 0; i < 17; i++)
                {
                    Serial.print(rvc_data[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();

                //Do the checksum.
                uint8_t sum = 0;
                for (uint8_t i = 0; i < 16; i++) sum += rvc_data[i];
                    if (sum != rvc_data[16]) return;
                
                // Put the data into the struct for use outside the the function.
                int16_t temp;
                if (angCounter < 20)
                {
                temp = rvc_data[1] + (rvc_data[2] << 8);
                rvcData.yawX100 = temp; //For angular velocity calc
                if ( !BNO.isActive )
                {
                 // Serial BNO is not active so we can put the CANRVC data into its' public data structure
                 BNO.rvcData.yawX100 = rvcData.yawX100;
                }
                rvcData.angVel += (temp - prevYAw);
                if ( !BNO.isActive )
                {
                 // Serial BNO is not active so we can put the CANRVC data into its' public data structure
                 BNO.rvcData.angVel = rvcData.angVel;
                }
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
                if ( !BNO.isActive )
                {
                 // Serial BNO is not active so we can put the CANRVC data into its' public data structure
                 BNO.rvcData.yawX10 = rvcData.yawX10;
                }

                temp = rvc_data[3] + (rvc_data[4] << 8);
                rvcData.pitchX10 = (int16_t)((float)temp * DEGREE_SCALE);
                if ( !BNO.isActive )
                {
                 // Serial BNO is not active so we can put the CANRVC data into its' public data structure
                 BNO.rvcData.pitchX10 = rvcData.pitchX10;
                }

                temp = rvc_data[5] + (rvc_data[6] << 8);
                rvcData.rollX10 = (int16_t)((float)temp * DEGREE_SCALE); //Confirmed X as stock direction of travel
                if ( !BNO.isActive )
                {
                 // Serial BNO is not active so we can put the CANRVC data into its' public data structure
                 BNO.rvcData.rollX10 = rvcData.rollX10;
                }

                isActive = true;
                timeoutTimer = 0;
                Serial.print("Millis: ");Serial.print(millis());Serial.print(" YawX10: "); Serial.print(rvcData.yawX10); Serial.print(" PitchX10: "); Serial.print(rvcData.pitchX10); Serial.print(" RollX10: "); Serial.print(rvcData.rollX10); Serial.print(" YawX100: "); Serial.print(rvcData.yawX100); Serial.print(" AngVel: "); Serial.println(rvcData.angVel);
                return;
                
            }
        } 
    }
};