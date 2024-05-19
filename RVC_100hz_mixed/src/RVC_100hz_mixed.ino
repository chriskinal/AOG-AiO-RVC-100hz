/*

See HWv??.h for hardware (board specfic) definitions (IO & Serial)
See common.h for library & other variable definitions
See debug.ino for optional debug commands
See PGN.ino for PGN parsing
See notes.ino for additional information

*/

// pick only one or the other board file
//#include "HWv50a.h"
#include "HWv4x.h"

const uint8_t encoderType = 1;  // 1 - single input
                                // 2 - dual input (quadrature encoder), uses Kickout_A (Pressure) & Kickout_D (Remote) inputs
                                // 3 - variable duty cycle, for future updates

#include "common.h"
//#include "JD_DAC.h"   // experimental JD 2 track DAC steering & SCV/remote hyd control
//JD_DAC jdDac(Wire1, 0x60, &Serial);

void setup()
{
  //Serial.begin(115200);                   // Teensy doesn't need it
  Serial.print("\r\n\n\n*********************\r\nStarting setup...\r\n");
  Serial.print(inoVersion);
  LEDs.set(LED_ID::PWR_ETH, PWR_ETH_STATE::PWR_ON);

  setCpuFrequency(600 * 1000000);           // Set CPU speed, default is 600mhz, 150mhz still seems fast enough, setup.ino
  serialSetup();                            // setup.ino
  parserSetup();                            // setup.ino
  BNO.begin(SerialIMU);                     // BNO_RVC.cpp

  // v5 has machine outputs, v4 fails outputs.begin so machine is also not init'd
  if (outputs.begin()) {                    // clsPCA9555.cpp
    Serial.print("\r\nSection outputs (PCA9555) detected (8 channels, low side switching)");
    machine.init(&outputs, pcaOutputPinNumbers, pcaInputPinNumbers, 100); // mach.h
  }

  if (UDP.init())                           // Eth_UDP.h
    LEDs.set(LED_ID::PWR_ETH, PWR_ETH_STATE::ETH_READY);
  else
    LEDs.set(LED_ID::PWR_ETH, PWR_ETH_STATE::NO_ETH);

  autosteerSetup();                         // Autosteer.ino

  Serial.println("\r\n\nEnd of setup, waiting for GPS...\r\n"); 
  delay(1);
  resetStartingTimersBuffers();             // setup.ino
}



void loop()
{
  SensorBus_Receive();                      // Check for sensor bus data
  checkForPGNs();                           // zPGN.ino, check for AgIO or SerialESP32 Sending PGNs
  PGNusage.timeOut();
  autoSteerUpdate();                        // Autosteer.ino, update AS loop every 10ms (100hz) regardless of whether there is a BNO installed
  udpNMEA();                                // check for NMEA via UDP
  udpNtrip();                               // check for RTCM via UDP (AgIO NTRIP client)
    
  if (SerialRTK.available()) {              // Check for RTK Radio RTCM data
    SerialGPS->write(SerialRTK.read());     // send to GPS1
    LEDs.queueBlueFlash(LED_ID::GPS);
  }

  #ifdef AIOv50a
    RS232usage.timeIn();
    if (SerialRS232->available()) {           // Check for RS232 data
      Serial.write(SerialRS232->read());      // just print to USB for testing
    }
    RS232usage.timeOut();
  #endif

  BNOusage.timeIn();
  if (BNO.read()) {                         // there should be new data every 10ms (100hz)
    bnoStats.incHzCount();
    bnoStats.update(1);                     // 1 dummy value
  }
  BNOusage.timeOut();

  // wait 40 msec (F9P) from prev GGA update, then update imu data for next PANDA sentence
  if (imuPandaSyncTrigger && imuPandaSyncTimer >= 40) {
    prepImuPandaData();
    imuPandaSyncTrigger = false;       // wait for next GGA update before resetting imuDelayTimer again
  }



  // ******************* "Right" Dual or Single GPS1 (position) *******************
  GPS1usage.timeIn();
  int16_t gps1Available = SerialGPS->available();
  if (gps1Available)    // "if" is very crucial here, using "while" causes BNO overflow
  {
    if (gps1Available > buffer_size - 50) {   // this should not trigger except maybe at boot up
      SerialGPS->clear();
      Serial.print((String)"\r\n" + millis() + " *** SerialGPS buffer cleared! ***");
      return;
    }
    gps1Stats.update(gps1Available);

    uint8_t gps1Read = SerialGPS->read();
    if (nmeaDebug) Serial.write(gps1Read);
    nmeaParser << gps1Read;
    
    #ifdef AIOv50a
      GPS1usage.timeOut();
      RS232usage.timeIn();
      SerialRS232->write(gps1Read);
      RS232usage.timeOut();
    #endif

    //Serial.write(gps1Read);
    //Serial.print((String)"\nSerialGPS update " + SerialGPS->available() + " " + millis() + " d:" + (char)gps1Read);
  }
  GPS1usage.timeOut();


  // ******************* "Left" Dual GPS2 (heading) *******************
  GPS2usage.timeIn();
  int16_t gps2Available = SerialGPS2->available();
  if (gps2Available)
  {
    if (gps2Available > buffer_size - 50) {   // this should not trigger except maybe at boot up
      SerialGPS2->clear();
      Serial.print((String)"\r\n" + millis() + " *** SerialGPS2 buffer cleared! ***");
      return;
    }
    gps2Stats.update(gps2Available);

    uint8_t gps2Read = SerialGPS2->read();
    ubxParser.parse(gps2Read);

    /*#ifdef AIOv50a
      GPS2usage.timeOut();
      RS232usage.timeIn();
      SerialRS232->write(gps2Read);
      RS232usage.timeOut();
    #endif*/
  }
  GPS2usage.timeOut();


  // ******************* For DUAL mode *******************
  if (ubxParser.relPosNedReady && ggaReady) {   // if both GGA & relposNED are ready
      buildPandaOrPaogi(PAOGI_DUAL);                 // build a PAOGI msg
      ubxParser.relPosNedReady = false;         // reset for next relposned trigger
      ubxParser.relPosNedRcvd = false;
      ggaReady = false;
    }

  if (imuPandaSyncTimer > 50 && extraCRLF && nmeaDebug) {
    Serial.print("\r\n");
    extraCRLF = false;
  }

  if (imuPandaSyncTimer > 150) {
    imuPandaSyncTimer -= 100;
    ggaMissed++;
    if (nmeaDebug) Serial.println();
    Serial.print("\r\n"); Serial.print(millis()); Serial.print(" ");
    Serial.printf("                 *** GGA was missed or late! *** (%i)\r\n", ggaMissed);
    ggaReady = false;
    ubxParser.relPosNedReady = false;
  }

  if (ubxParser.relPosTimer > 150) {
    ubxParser.relPosTimer -= 100;
    ubxParser.relMissed++;
    if (nmeaDebug) Serial.println();
    Serial.print("\r\n"); Serial.print(millis()); Serial.print(" ");
    Serial.printf("                   *** relposNED was missed or late! *** (%i)\r\n", ubxParser.relMissed);
    ubxParser.clearCount();
    ggaReady = false;
    ubxParser.relPosNedReady = false;
  }

  /*if (ubxParser.pvtTimer > 150) {
    ubxParser.pvtTimer -= 100;
    Serial.print("\r\n\n"); Serial.print(millis()); Serial.print(" ");
    Serial.print("                 *** PVT was missed or late! ***\r\n");
  }*/

  // *************************************************************************************************
  // ************************************* UPDATE OTHER STUFF *************************************
  // *************************************************************************************************


  // this is only for dual stats monitoring
  if (dualTime != ubxParser.ubxData.iTOW)
  {
    gps2Stats.incHzCount();
    relJitterStats.update(ubxParser.msgPeriod);
    relTtrStats.update(ubxParser.msgReadTime);
    dualTime = ubxParser.ubxData.iTOW;
  }
  
  if (bufferStatsTimer > 5000) printTelem();
  
  LEDSusage.timeIn();
  LEDs.updateLoop();                  // LEDS.h
  LEDSusage.timeOut();

  MACHusage.timeIn();
  machine.watchdogCheck();            // machine.h, run machine class for v4.x to suppress unprocessed PGN messages, also reduces #ifdefs
  MACHusage.timeOut();

  checkUSBSerial();                   // debug.ino
  speedPulse.update();                // misc.h

  #ifdef RESET_H
    teensyReset.update();             // reset.h
  #endif

  // to count loop hz & get baseline cpu "idle" time
  LOOPusage.timeIn();
  testCounter++;
  LOOPusage.timeOut();

} // end of loop()




