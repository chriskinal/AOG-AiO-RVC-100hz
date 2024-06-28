/*

  UDP - Receive PGNs sent from AgIO - sent to port 8888

  a lot of this code taken from old AIO I2C firmware

*/

#define UDP_MAX_PACKET_SIZE 40         // Buffer For Receiving 8888 UDP PGN Data
//uint32_t pgn254Time, pgn254MaxDelay, pgn254AveDelay, pgn254MinDelay = 99999;

void checkForPGNs()
{
  #ifdef AIOv50a
  ESP32usage.timeIn();
  if (SerialESP32->available())
  {
    static uint8_t incomingBytes[50];
    static uint8_t incomingIndex;
    incomingBytes[incomingIndex] = SerialESP32->read();
    incomingIndex++;
    /*Serial.print("\r\nindex: "); Serial.print(incomingIndex);
    Serial.print(" ");
    for (byte i = 0; i < incomingIndex; i++) {
      Serial.print(incomingBytes[i]);
      Serial.print(" ");
    }*/

    if (incomingBytes[incomingIndex - 2] == 13 && incomingBytes[incomingIndex - 1] == 10)
    {
      if (incomingBytes[0] == 128 && incomingBytes[1] == 129)
      {

        // Modules--Wifi:9999-->ESP32--serial-->Teensy--ethernet:9999-->AgIO
        UDP.SendUdpByte(incomingBytes, incomingIndex - 2, UDP.broadcastIP, UDP.portAgIO_9999);

        //pass data to USB for debug
        /*Serial.print("\r\nE32-s->T41-e:9999->AgIO ");
        for (byte i = 0; i < incomingIndex - 2; i++) {
          Serial.print(incomingBytes[i]);
          Serial.print(" ");
        }
        Serial.print((String)" (" + SerialESP32->available() + ")");
        */
      } else {
        Serial.print("\r\n\nCR/LF detected but [0]/[1] bytes != 128/129\r\n");
      }
      incomingIndex = 0;
    }
  }
  ESP32usage.timeOut();
  #endif  // AIOv50a


  PGNusage.timeIn();
  static uint32_t pgnCheckTime;
  uint32_t millisNow = millis();
  if (millisNow < pgnCheckTime) return;   // only need to check for new PGN data every ms, not 100s of times per ms
  //Serial.print((String)"\r\n" + millisNow + " PGN check " + pgnCheckTime);
  pgnCheckTime = millisNow + 1;     // allow check every ms

  if (!UDP.isRunning) return;                           // When ethernet is not running, return directly. parsePacket() will block when we don't
  uint16_t len = UDP.PGN.parsePacket();                 //get data from AgIO sent by 9999 to this 8888
  if (UDP.PGN.remotePort() != 9999 || len < 5) return;  //make sure from AgIO

  uint8_t udpData[UDP_MAX_PACKET_SIZE];  // UDP_TX_PACKET_MAX_SIZE is not large enough for machine pin settings PGN
  UDP.PGN.read(udpData, UDP_MAX_PACKET_SIZE);

  if (udpData[0] != 0x80 || udpData[1] != 0x81 || udpData[2] != 0x7F) return;  // verify first 3 PGN header bytes
  bool pgnMatched = false;

  #ifdef AIOv50a
  if (udpData[3] != 100) {
    ESP32usage.timeIn();
    SerialESP32->write(udpData, len);
    SerialESP32->println();   // to signal end of PGN
    /*Serial.print("\r\nAgIO-e:8888->T41-s->E32 ");
    for (uint8_t i = 0; i < len; i++) {
      Serial.print(udpData[i]); Serial.print(" ");
    }*/
    ESP32usage.timeOut();
  }
  #endif

  // changed to multiple IF statements instead of IF ELSE so that AgIO Hello and Scan Request PGNs can be pickedup by other object/classes (ie machine)

  if (udpData[3] == 100 && len == 22)  // 0x64 (100) - Corrected Position
  {
    //printPgnAnnoucement(udpData[3], (char*)"Corrected Position", len);
    /*
    union {           // both variables in the union share the same memory space
      byte array[8];  // fill "array" from an 8 byte array converted in AOG from the "double" precision number we want to send
      double number;  // and the double "number" has the original "double" precision number from AOG
    } lat, lon;

    for (byte i = 0; i < 8; i++)
    {
      lon.array[i] = udpData[i+5];
      lat.array[i] = udpData[i+13];
    }*/
    /*Serial.print("\r\n");
    Serial.print(lat.number, 13);
    Serial.print(" ");
    Serial.print(lon.number, 13);*/

    //buildNMEA(lat.number, lon.number);

    return;                    // no other processing needed
  }


  if (udpData[3] == 200 && len == 9)  // 0xC8 (200) - Hello from AgIO
  {
    //printPgnAnnoucement(udpData[3], (char*)"Hello from AgIO", len);
    LEDs.set(LED_ID::PWR_ETH, PWR_ETH_STATE::AGIO_CONNECTED, true);

    //Serial.print("\r\n***** AgIO Hello byte 5-7: "); Serial.print(udpData[5]); Serial.print(" "); Serial.print(udpData[6]); Serial.print(" "); Serial.print(udpData[7]); Serial.print(" ");

    // reply as Steer Module
    uint8_t helloFromAutoSteer[] = { 0x80, 0x81, 126, 126, 5, 0, 0, 0, 0, 0, 71 };
    if (autoSteerEnabled) {
      int16_t sa = (int16_t)(steerAngleActual * 100);

      helloFromAutoSteer[5] = (uint8_t)sa;
      helloFromAutoSteer[6] = sa >> 8;

      uint16_t helloSteerPosition = steeringPosition; // - 6800; steeringPosition is already centered & offset in Autosteer.ino
      helloFromAutoSteer[7] = (uint8_t)helloSteerPosition;
      helloFromAutoSteer[8] = helloSteerPosition >> 8;
      helloFromAutoSteer[9] = switchByte;

      UDP_Susage.timeIn();
      UDP.SendUdpByte(helloFromAutoSteer, sizeof(helloFromAutoSteer), UDP.broadcastIP, UDP.portAgIO_9999);
      UDP_Susage.timeOut();
    }

    // reply as IMU if equipped
    if (BNO.isActive) {
      uint8_t helloFromIMU[] = { 128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71 };
      UDP_Susage.timeIn();
      UDP.SendUdpByte(helloFromIMU, sizeof(helloFromIMU), UDP.broadcastIP, UDP.portAgIO_9999);
      UDP_Susage.timeOut();
    }

    #ifdef MACHINE_H
      if (machine.isInit) {
        uint8_t helloFromMachine[] = { 0x80, 0x81, 123, 123, 5, 0, 0, 0, 0, 0, 71 };
        helloFromMachine[5] = B10101010;  // should be changed to read actual machine output states
        helloFromMachine[6] = B01010101;
        UDP_Susage.timeIn();
        UDP.SendUdpByte(helloFromMachine, sizeof(helloFromMachine), UDP.broadcastIP, UDP.portAgIO_9999);
        UDP_Susage.timeOut();
      }
    #endif
        
    pgnMatched = true;
    //return;         // no return, allow machine object to process machine reply below
  } // 0xC8 (200) - Hello from AgIO


  if (udpData[3] == 201 && len == 11)  // 0xC9 (201) - Subnet Change
  {
    printPgnAnnoucement(udpData[3], (char*)"Subnet Change", len);
    if (udpData[4] == 5 && udpData[5] == 201 && udpData[6] == 201)  //save in EEPROM and restart
    {
      Serial << "\r\n- IP changed from " << UDP.myIP;
      UDP.myIP[0] = udpData[7];
      UDP.myIP[1] = udpData[8];
      UDP.myIP[2] = udpData[9];

      Serial << " to " << UDP.myIP;
      Serial << "\r\n- Saving to EEPROM and restarting Teensy";

      UDP.SaveModuleIP();  //save in EEPROM and restart
      delay(10);
      SCB_AIRCR = 0x05FA0004;  //Teensy Reset
    }
    return;                    // no other processing needed
  }  // 0xC9 (201) - Subnet Change


  if (udpData[3] == 202 && len == 9)    // 0xCA (202) - Scan Request
  {
    printPgnAnnoucement(udpData[3], (char*)"Scan Request", len);
    if (udpData[4] == 3 && udpData[5] == 202 && udpData[6] == 202) {
      IPAddress rem_ip = UDP.PGN.remoteIP();
      IPAddress ipDest = { 255, 255, 255, 255 };

      uint8_t scanReplySteer[] = { 128, 129, 126, 203, 7,
                              UDP.myIP[0], UDP.myIP[1], UDP.myIP[2], UDP.myIP[3],
                              rem_ip[0], rem_ip[1], rem_ip[2], 23 };
      int16_t CK_A = 0;
      for (uint8_t i = 2; i < sizeof(scanReplySteer) - 1; i++) {
        CK_A = (CK_A + scanReplySteer[i]);
      }
      scanReplySteer[sizeof(scanReplySteer) - 1] = CK_A;
      UDP_Susage.timeIn();
      UDP.SendUdpByte(scanReplySteer, sizeof(scanReplySteer), ipDest, UDP.portAgIO_9999);
      UDP_Susage.timeOut();

      if (BNO.isActive) {
        uint8_t scanReplyIMU[] = { 128, 129, 121, 203, 7,
                                UDP.myIP[0], UDP.myIP[1], UDP.myIP[2], UDP.myIP[3],
                                rem_ip[0], rem_ip[1], rem_ip[2], 23 };
        CK_A = 0;
        for (uint8_t i = 2; i < sizeof(scanReplyIMU) - 1; i++) {
          CK_A = (CK_A + scanReplyIMU[i]);
        }
        scanReplyIMU[sizeof(scanReplyIMU) - 1] = CK_A;
        UDP_Susage.timeIn();
        UDP.SendUdpByte(scanReplyIMU, sizeof(scanReplyIMU), ipDest, UDP.portAgIO_9999);
        UDP_Susage.timeOut();
      }

      #ifdef MACHINE_H
      if (machine.isInit) {
        uint8_t scanReplyMachine[] = { 128, 129, 123, 203, 7,
                                UDP.myIP[0], UDP.myIP[1], UDP.myIP[2], UDP.myIP[3],
                                rem_ip[0], rem_ip[1], rem_ip[2], 23 };
        CK_A = 0;
        for (uint8_t i = 2; i < sizeof(scanReplyMachine) - 1; i++) {
          CK_A = (CK_A + scanReplyMachine[i]);
        }
        scanReplyMachine[sizeof(scanReplyMachine) - 1] = CK_A;
        UDP_Susage.timeIn();
        UDP.SendUdpByte(scanReplyMachine, sizeof(scanReplyMachine), ipDest, UDP.portAgIO_9999);
        UDP_Susage.timeOut();
      }
      #endif

      Serial.printf("\r\n ---------\r\n%s\r\nCPU Temp:%.1f CPU Speed:%iMhz GPS Baud:%i", inoVersion, tempmonGetTemp(), F_CPU_ACTUAL / 1000000, baudGPS);
      Serial.print("\r\nAgIO IP:   "); Serial.print(rem_ip);
      Serial.print("\r\nModule IP: "); Serial.print(UDP.myIP);

      /*if (!Autosteer_running) Serial.println("\r\n!! Autosteer disabled... Check ADS1115");
      else if (PWM_Frequency == 0) Serial.println("\r\nAutosteer running, PWM Frequency = 490hz");
      else if (PWM_Frequency == 1) Serial.println("\r\nAutosteer running, PWM Frequency = 122hz");
      else if (PWM_Frequency == 2) Serial.println("\r\nAutosteer running, PWM Frequency = 3921hz");*/

      if (BNO.isActive) Serial.print("\r\nBNO08x available via Serial/RVC Mode");
      else Serial.print("\r\n* No IMU available *");

      /*if (GGA_Available == false) Serial.println("\r\n!! GPS Data Missing... Check F9P Config");
      else if (!useDual) Serial.println("\r\nGPS Single GPS mode");
      else if (useDual && !dualDataFail && !dualRTKFail && !dualBaselineFail) Serial.println("\r\nGPS Dual GPS mode");
      else if (dualDataFail) Serial.println("\r\n!! Dual Data Checksum Failed");
      else if (dualRTKFail) Serial.println("\r\n!! Dual RTK/Quality Failed... Check Antennas");
      else if (dualBaselineFail) Serial.println("\r\n!! Dual Baseline Moving Too Much... Check Antennas");*/
      
      Serial.println("\r\r\n ---------");
    }
    return;
  } // 0xCA (202) - Scan Request


  if (udpData[3] == 0xFB && len == 14)  // 0xFB (251) - SteerConfig
  {
    printPgnAnnoucement(udpData[3], (char*)"Steer Config", len);
    uint8_t sett = udpData[5]; //setting0
    if (bitRead(sett, 0)) steerConfig.InvertWAS = 1; else steerConfig.InvertWAS = 0;
    if (bitRead(sett, 1)) steerConfig.IsRelayActiveHigh = 1; else steerConfig.IsRelayActiveHigh = 0;
    if (bitRead(sett, 2)) steerConfig.MotorDriveDirection = 1; else steerConfig.MotorDriveDirection = 0;
    if (bitRead(sett, 3)) steerConfig.SingleInputWAS = 1; else steerConfig.SingleInputWAS = 0;
    if (bitRead(sett, 4)) steerConfig.CytronDriver = 1; else steerConfig.CytronDriver = 0;
    if (bitRead(sett, 5)) steerConfig.SteerSwitch = 1; else steerConfig.SteerSwitch = 0;
    if (bitRead(sett, 6)) steerConfig.SteerButton = 1; else steerConfig.SteerButton = 0;
    if (bitRead(sett, 7)) steerConfig.ShaftEncoder = 1; else steerConfig.ShaftEncoder = 0;

    steerConfig.PulseCountMax = udpData[6];
    steerConfig.MinSpeed = udpData[7];

    sett = udpData[8]; //setting1 - Danfoss valve etc
    if (bitRead(sett, 0)) steerConfig.IsDanfoss = 1; else steerConfig.IsDanfoss = 0;
    if (bitRead(sett, 1)) steerConfig.PressureSensor = 1; else steerConfig.PressureSensor = 0;
    if (bitRead(sett, 2)) steerConfig.CurrentSensor = 1; else steerConfig.CurrentSensor = 0;
    if (bitRead(sett, 3)) steerConfig.IsUseY_Axis = 1; else steerConfig.IsUseY_Axis = 0;

    Serial.print("\r\nInvertWAS "); Serial.print(steerConfig.InvertWAS);
    Serial.print("\r\nIsRelayActiveHigh "); Serial.print(steerConfig.IsRelayActiveHigh);
    Serial.print("\r\nMotorDriveDirection "); Serial.print(steerConfig.MotorDriveDirection);
    Serial.print("\r\nSingleInputWAS "); Serial.print(steerConfig.SingleInputWAS);
    Serial.print("\r\nCytronDriver "); Serial.print(steerConfig.CytronDriver);
    Serial.print("\r\nSteerSwitch "); Serial.print(steerConfig.SteerSwitch);
    Serial.print("\r\nSteerButton "); Serial.print(steerConfig.SteerButton);
    Serial.print("\r\nShaftEncoder "); Serial.print(steerConfig.ShaftEncoder);
    Serial.print("\r\nIsDanfoss "); Serial.print(steerConfig.IsDanfoss);
    Serial.print("\r\nPressureSensor "); Serial.print(steerConfig.PressureSensor);
    Serial.print("\r\nCurrentSensor "); Serial.print(steerConfig.CurrentSensor);
    Serial.print("\r\nIsUseY_Axis "); Serial.print(steerConfig.IsUseY_Axis);
    Serial.print("\r\nPulseCountMax "); Serial.print(steerConfig.PulseCountMax);
    Serial.print("\r\nMinSpeed "); Serial.print(steerConfig.MinSpeed);
    Serial.println();

    EEPROM.put(40, steerConfig);            
    steerConfigInit();  // Re-Init
    return;             // no other processing needed
  }  // 0xFB (251) - SteerConfig


  if (udpData[3] == 0xFC && len == 14)         // 0xFC (252) - Steer Settings
  {
    printPgnAnnoucement(udpData[3], (char*)"Steer Settings", len);
    //PID values
    steerSettings.Kp = ((float)udpData[5]);    // read Kp from AgOpenGPS
    steerSettings.highPWM = udpData[6];        // read high pwm
    steerSettings.lowPWM = (float)udpData[7];  // read lowPWM from AgOpenGPS
    steerSettings.minPWM = udpData[8];         // read the minimum amount of PWM for instant on

    float temp = (float)steerSettings.minPWM * 1.2;
    steerSettings.lowPWM = (byte)temp;

    steerSettings.steerSensorCounts = udpData[9];   // sent as setting displayed in AOG
    
    //steerSettings.wasOffset = (udpData[10]);        // read was zero offset Lo
    //steerSettings.wasOffset |= (udpData[11] << 8);  // read was zero offset Hi
    int16_t newWasOffset = (udpData[10]);        // read was zero offset Lo
    newWasOffset        |= (udpData[11] << 8);   // read was zero offset Hi

    #ifdef JD_DAC_H
      jdDac.setMaxPWM(steerSettings.highPWM);
      if (newWasOffset != steerSettings.wasOffset) {
        jdDac.centerDac();
      }
    #endif

    steerSettings.wasOffset = newWasOffset;
    steerSettings.AckermanFix = (float)udpData[12] * 0.01;

    Serial.print("\r\n Kp "); Serial.print(steerSettings.Kp);
    Serial.print("\r\n highPWM "); Serial.print(steerSettings.highPWM);
    Serial.print("\r\n lowPWM "); Serial.print(steerSettings.lowPWM);
    Serial.print("\r\n minPWM "); Serial.print(steerSettings.minPWM);
    Serial.print("\r\n steerSensorCounts "); Serial.print(steerSettings.steerSensorCounts);
    Serial.print("\r\n wasOffset "); Serial.print(steerSettings.wasOffset);
    Serial.print("\r\n AckermanFix "); Serial.print(steerSettings.AckermanFix);

    EEPROM.put(10, steerSettings);
    steerSettingsInit();  // Re-Init steer settings
    return;               // no other processing needed
  }  // 0xFC (252) - Steer Settings


  if (udpData[3] == 0xFE && len == 14)        // 0xFE (254) - Steer Data (sent at GPS freq, ie 10hz (100ms))
  {
    //printPgnAnnoucement(udpData[3], (char*)"Steer Data", len);

    if (aogGpsToAutoSteerLoopTimerEnabled)
    {
      aogGpsToAutoSteerLoopTimerEnabled = false;
      Serial.print((String)"\r\nGPS out to Steer Data in delay: " + aogGpsToAutoSteerLoopTimer);
    }
    /*Serial.printf(" %6i", micros() - pgn254Time);
    pgn254Time = micros();
    uint32_t pgn254Delay = pgn254Time - nmeaPgnSendTime;
    if (pgn254Delay < pgn254MinDelay) pgn254MinDelay = pgn254Delay;
    if (pgn254Delay > pgn254MaxDelay) pgn254MaxDelay = pgn254Delay;
    if (pgn254AveDelay == 0) pgn254AveDelay = pgn254Delay;
    else pgn254AveDelay = pgn254AveDelay * 0.99 + pgn254Delay * 0.01;
    Serial.printf("->PGN254 delay: %4iuS  %4i %4i %4i", pgn254Delay, pgn254MinDelay, pgn254AveDelay, pgn254MaxDelay);*/
    gpsSpeed = ((float)(udpData[5] | udpData[6] << 8)) * 0.1;   // speed data comes in as km/hr x10
    //Serial << "\r\n speed:" << gpsSpeed << " "; Serial.print(udpData[5], BIN); Serial << " "; Serial.print(udpData[6], BIN);
    speedPulse.updateSpeed(gpsSpeed);

    prevGuidanceStatus = guidanceStatus;
    guidanceStatus = udpData[7];
    guidanceStatusChanged = (guidanceStatus != prevGuidanceStatus);

    //Bit 8,9    set point steer angle * 100 is sent
    steerAngleSetPoint = ((float)(udpData[8] | ((int8_t)udpData[9]) << 8)) * 0.01;  //high low bytes

    //udpData[10] is XTE (cross track error)
    //udpData[11 & 12] is section 1-16

    if ((bitRead(guidanceStatus, 0) == 0) || (steerState == 0)) { // || (gpsSpeed < 0.1)) {
      watchdogTimer = WATCHDOG_FORCE_VALUE;  //turn off steering motor
      //Serial.print(" OFF");
    } else {                                 //valid conditions to turn on autosteer
      watchdogTimer = 0;                     //reset watchdog
      //Serial.print(" ON");
    }

    //Bit 10 XTE
    xte = udpData[10];
    //Serial.print("\r\nXTE:"); Serial.print(xte-127);

    //Bit 11
    //relay = udpData[11];

    //Bit 12
    //relayHi = udpData[12];

    //----------------------------------------------------------------------------
    // Reply to send to AgIO
    // fromAutoSteerData FD 253 - ActualSteerAngle*100 -56, SwitchByte-7, pwmDisplay-8
    uint8_t PGN_253[] = { 0x80, 0x81, 126, 0xFD, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC };

    int16_t sa = (int16_t)(steerAngleActual * 100);
    PGN_253[5] = (uint8_t)sa;
    PGN_253[6] = sa >> 8;

    // heading
    PGN_253[7] = (uint8_t)9999;
    PGN_253[8] = 9999 >> 8;

    // roll
    PGN_253[9] = (uint8_t)8888;
    PGN_253[10] = 8888 >> 8;

    PGN_253[11] = switchByte;
    PGN_253[12] = (uint8_t)abs(pwmDisplay);

    //checksum
    int16_t CK_A = 0;
    for (uint8_t i = 2; i < sizeof(PGN_253) - 1; i++)
      CK_A = (CK_A + PGN_253[i]);

    PGN_253[sizeof(PGN_253) - 1] = CK_A;

    //off to AOG
    UDP_Susage.timeIn();
    UDP.SendUdpByte(PGN_253, sizeof(PGN_253), UDP.broadcastIP, UDP.portAgIO_9999);
    UDP_Susage.timeOut();

    //Steer Data 2 -------------------------------------------------
    /*if (steerConfig.PressureSensor || steerConfig.CurrentSensor) {
      if (aog2Count++ > 2) {                                // send 1/3 of Steer Data rate (GPS hz / 3)
        // fromAutoSteerData FD 250 - sensor values etc
        uint8_t PGN_250[] = { 0x80, 0x81, 126, 0xFA, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC };

        //Send fromAutosteer2
        PGN_250[5] = (byte)sensorReading;

        //add the checksum for AOG2
        CK_A = 0;

        for (uint8_t i = 2; i < sizeof(PGN_250) - 1; i++) {
          CK_A = (CK_A + PGN_250[i]);
        }

        PGN_250[sizeof(PGN_250) - 1] = CK_A;

        //off to AOG
        UDP_Susage.timeIn();
        UDP.SendUdpByte(PGN_250, sizeof(PGN_250), UDP.broadcastIP, UDP.portAgIO_9999);
        UDP_Susage.timeOut();
        aog2Count = 0;
      }
    }*/

    if (aog2Count++ > 1) {                                // send 1/2 of Steer Data rate (GPS hz / 2)
      // fromAutoSteerData FD 250 - sensor values etc
      uint8_t PGN_250[] = { 0x80, 0x81, 126, 0xFA, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC };

      if (steerConfig.PressureSensor || steerConfig.CurrentSensor) {
        PGN_250[5] = (byte)sensorReading;
      } else {
        PGN_250[5] = (byte)pulseCount;
      }

      CK_A = 0;
      for (uint8_t i = 2; i < sizeof(PGN_250) - 1; i++) {
        CK_A = (CK_A + PGN_250[i]);
      }
      PGN_250[sizeof(PGN_250) - 1] = CK_A;

      UDP_Susage.timeIn();
      UDP.SendUdpByte(PGN_250, sizeof(PGN_250), UDP.broadcastIP, UDP.portAgIO_9999);
      UDP_Susage.timeOut();
      aog2Count = 0;
    }
    return;                     // no other processing needed
  }  // 0xFE (254) - Steer Data



  #ifdef MACHINE_H
    PGNusage.timeOut();
    MACHusage.timeIn();
    //IPAddress ipDest = UDP.broadcastIP;
    //uint8_t machineReplyData[] = { 0x80, 0x81, 123, 0, 0, 0, 0, 0, 0, 0, 0, 0, 23 };  // long enough for machine scan reply len 13, AgIO Hello reply len 11
    //uint8_t machineReplyData[] = { 0x80, 0x81, 123, 203, 7, UDP.myIP[0], UDP.myIP[1], UDP.myIP[2], UDP.myIP[3], rem_ip[0], rem_ip[1], rem_ip[2], 23 };
    //uint8_t machineReplyLen = 0;    // set default len of 0, which means there's no reply data to send

    if (machine.parsePGN(udpData, len)) //, machineReplyData, &machineReplyLen))//, &ipDest))    // look for Machine PGNs
    {
      pgnMatched = true;
    }
    MACHusage.timeOut();
    PGNusage.timeIn();
  #endif



  if (!pgnMatched) printPgnAnnoucement(udpData[3], (char*)"Unprocessed PGN", len);
}

void printPgnAnnoucement(uint8_t _pgnNum, char* _pgnName, uint8_t _len)
{
  Serial.print("\r\n\n0x"); Serial.print(_pgnNum, HEX);
  Serial.print(" ("); Serial.print(_pgnNum); Serial.print(") - ");
  Serial.print(_pgnName); Serial.print(", "); Serial.print(_len); Serial.print(" bytes ");
  Serial.print(millis());

}
/*
* To receive NMEA sent via UDP from GPS Source
   - listen on port 2211 GPS1 (single/dual)
   - additional related port number 2222 GPS2 (dual?)
   - additional related port number 2233 RTCM
* Character data
*/
void udpNMEA() {
  if (!UDP.isRunning) return;  // When ethernet is not running, return directly. parsePacket() will block when we don't

  int packetLength = UDP.NMEA.parsePacket();
  if (packetLength > 0) {
    char NMEA_packetBuffer[256];       // buffer for receiving NMEA sentence
    UDP.NMEA.read(NMEA_packetBuffer, packetLength);
    for (int i = 0; i < packetLength; i++) {
      nmeaParser << NMEA_packetBuffer[i];
    }
  }
}


/*
*  To receive RTCM sent via UDP from AgIO NTRIP client - listen on port 2233 RTCM
*/
void udpNtrip() {
  NTRIPusage.timeIn();
  static uint32_t ntripCheckTime;//, ntripUpdateTime;
  // When ethernet is not running, return directly. parsePacket() will block when we don't
  if (UDP.isRunning) {
    if (millis() > ntripCheckTime) {  // limit update rate to save cpu time
    

      unsigned int packetLength = UDP.RTCM.parsePacket(); // this uses most of the cpu time in this function unless SerialGPS has low baud
      ntripCheckTime = millis();                          // make sure we wait at least 1ms before checking again to avoid excessive cpu usage

      if (packetLength > 0) {
        //Serial.print("\r\nNTRIP "); Serial.print(millis() - ntripUpdateTime); Serial.print(" len:"); Serial.print(packetLength);
        char RTCM_packetBuffer[buffer_size];
        UDP.RTCM.read(RTCM_packetBuffer, buffer_size);
        SerialGPS->write(RTCM_packetBuffer, buffer_size);
        LEDs.queueBlueFlash(LED_ID::GPS);

        // up to 256 byte packets are sent from AgIO and most NTRIP RTCM updates are larger so there's usually two packets per update
        // this doesn't seem necessary, the above 1ms update limit already reduces cpu usage enough
        /*if (packetLength < buffer_size){    // if buffer was not full, then end of NTRIP packet, can wait 800ms until we start checking again
          ntripCheckTime = millis() + 800;  // most base stations send updates every 1000ms
        }*/
          
        //ntripUpdateTime = millis();   // only used in Serial debug above, AgIO always delays sequential ntrip packets by about 52-70ms
      }
    }
  }
  NTRIPusage.timeOut();
}

