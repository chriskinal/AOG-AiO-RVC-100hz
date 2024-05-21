

class CAN_RVC
{

private:
    bool gotAAAA = false;
    bool gotMSG2 = false;
    byte rvc_data[17];
    int16_t prevYAw;
    const float MILLI_G_TO_MS2 = 0.0098067; ///< Scalar to convert milli-gs to m/s^2;
    const float DEGREE_SCALE = 0.1;        ///< To convert the degree values

public:
    struct BNO_RVC_DATA {
        int16_t yawX10;       // Yaw in Degrees x 10
        int16_t pitchX10;     // Pitch in Degrees x 10
        int16_t rollX10;      // Roll in Degrees x 10
        int16_t yawX100;      // Yaw in original x100
        int16_t angVel;       // running total of angular velocity
    };
    uint32_t angCounter;
    bool isSwapXY = false;
    bool isActive;
    BNO_RVC_DATA rvcData;

    void Sensor_Setup()
    {
        Sensor_Bus.begin();
        Sensor_Bus.setBaudRate(250000); // For TTL-CAN board
        delay(100);
        Serial.print("Initialised Sensor CANBUS @ ");
        Serial.print(Sensor_Bus.getBaudRate());
        Serial.println("bps");
    }

    void SensorBus_Receive()
    {
        CAN_message_t SensorData;

        if (Sensor_Bus.read(SensorData))
        {
            Serial.print("ID: ");
            Serial.print(SensorData.id, HEX);
            Serial.print(" Len: ");
            Serial.print(SensorData.len, HEX);
            Serial.print(" Data: ");
            // for ( int element : SensorData.buf )
            //     Serial.print(element); Serial.print(" ");
            for (uint8_t i = 0; i < 8; i++)
            {
                Serial.print(SensorData.buf[i], HEX);
                Serial.print(" ");
            }
            Serial.println();

            if ((SensorData.buf[0] == 0xAA) && (SensorData.buf[1] == 0xAA))
            {
                for (uint8_t i = 2; i < 8; i++)
                {
                    rvc_data[i - 2] = SensorData.buf[i];
                }
                gotAAAA = true;
                return;
                // for ( uint8_t i = 0; i < 19; i++ ) {
                //     Serial.print(rvc_data[i], HEX); Serial.print(" ");
                // }
                // Serial.println();
            }
            if (gotAAAA)
            {
                for (uint8_t i = 0; i < 8; i++)
                {
                    rvc_data[i + 6] = SensorData.buf[i];
                }
                gotMSG2 = true;
                gotAAAA = false;
                return;
                // for ( uint8_t i = 0; i < 19; i++ ) {
                //     Serial.print(rvc_data[i], HEX); Serial.print(" ");
                // }
                // Serial.println();
            }
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
                uint8_t sum = 0;
                for (uint8_t i = 0; i < 16; i++) sum += rvc_data[i];
                    // Serial.print("Sum: "); Serial.print(sum, HEX); Serial.print(" Check Digit: "); Serial.println(rvc_data[16], HEX);
                    if (sum != rvc_data[16]) return;
                
                int16_t temp;
                if (angCounter < 20)
                {
                temp = rvc_data[1] + (rvc_data[2] << 8);
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

                temp = rvc_data[3] + (rvc_data[4] << 8);
                rvcData.pitchX10 = (int16_t)((float)temp * DEGREE_SCALE);

                temp = rvc_data[5] + (rvc_data[6] << 8);
                rvcData.rollX10 = (int16_t)((float)temp * DEGREE_SCALE); //Confirmed X as stock direction of travel

                //isActive = true;
                //timeoutTimer = 0;
                Serial.print("YawX10: "); Serial.print(rvcData.yawX10); Serial.print(" PitchX10: "); Serial.print(rvcData.pitchX10); Serial.print(" RollX10: "); Serial.print(rvcData.rollX10); Serial.print(" YawX100: "); Serial.print(rvcData.yawX100); Serial.print(" AngVel: "); Serial.println(rvcData.angVel);
                return;
                
            }
        }
    }
};