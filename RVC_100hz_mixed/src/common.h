#include "HardwareSerial.h"
#include "elapsedMillis.h"
#include <stdint.h>
#include <Streaming.h>
#include "zADS1115.h"

#include "LEDS.h"
LEDS LEDs = LEDS(1000, 255, 64, 127);   // 1000ms RGB update, 255/64/127 RGB brightness balance levels for v5.0a

ProcessorUsage BNOusage           ((char*)"BNO   ");
ProcessorUsage GPS1usage          ((char*)"GPS1  ");
ProcessorUsage GPS2usage          ((char*)"GPS2  ");
ProcessorUsage PGNusage           ((char*)"PGN   ");
ProcessorUsage ASusage            ((char*)"AG    ");
ProcessorUsage NTRIPusage         ((char*)"NTRIP ");
ProcessorUsage RS232usage         ((char*)"RS232 ");  
ProcessorUsage LOOPusage          ((char*)"Loop  ");
ProcessorUsage IMU_Husage         ((char*)"IMU_H ");
ProcessorUsage NMEA_Pusage        ((char*)"NMEA_H");
ProcessorUsage RTKusage           ((char*)"Radio ");
ProcessorUsage UBX_Pusage         ((char*)"UBX_H ");
ProcessorUsage UDP_Susage         ((char*)"UDP_S ");
ProcessorUsage DACusage           ((char*)"DAC   ");
ProcessorUsage MACHusage          ((char*)"MACH  ");
ProcessorUsage LEDSusage          ((char*)"LEDS  ");
ProcessorUsage ESP32usage         ((char*)"ESP32 ");
const uint8_t numCpuUsageTasks = 17;
ProcessorUsage* cpuUsageArray[numCpuUsageTasks] = { 
  &BNOusage, &GPS1usage, &GPS2usage, &PGNusage, &ASusage, &NTRIPusage,
  &RS232usage, &LOOPusage, &IMU_Husage, &NMEA_Pusage, &RTKusage, &UBX_Pusage,
  &UDP_Susage, &DACusage, &MACHusage, &LEDSusage, &ESP32usage
};
HighLowHzStats gps2Stats;
HighLowHzStats gps1Stats;
HighLowHzStats relJitterStats;
HighLowHzStats relTtrStats;
HighLowHzStats bnoStats;

elapsedMillis bufferStatsTimer;
uint32_t testCounter;
bool printCpuUsages = false;
bool printStats = false;
uint16_t ggaMissed;

//#include "reset.h"    // no on board buttons for reset
//const uint8_t RESET_BTN = A14;          // A13 on Matt's v4.0 test board, A14 ununsed on v5.0a
//RESET teensyReset(RESET_BTN, 2000, LED_BUILTIN);           // reset.h - btn IO, factory reset PERIOD (ms), led IO

#include <FlexCAN_T4.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_256> CAN1_Bus;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_256> CAN2_Bus;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_256> Sensor_Bus;

#include "Encoder.h"
Encoder encoder(KICKOUT_D_PIN, KICKOUT_A_PIN);    // read single or double input values in Autosteer.ino

#include <ADC.h>
#include <ADC_util.h>
ADC* teensyADC = new ADC();                        // 16x oversampling medium speed 12 bit A/D object (set in Autosteer.ino)
ADS1115_lite ads1115(ADS1115_DEFAULT_ADDRESS);     // Use this for the 16-bit version ADS1115
const int16_t ANALOG_TRIG_THRES = 100;
const uint8_t ANALOG_TRIG_HYST = 10;

#include "BNO_RVC.h"
BNO_RVC BNO;                                            // Roomba Vac mode for BNO085

#include "Eth_UDP.h"
Eth_UDP UDP = Eth_UDP();

#include "clsPCA9555.h" // https://github.com/nicoverduin/PCA9555
PCA9555 outputs(0x20);  // 0x20 - I2C addr (A0-A2 grounded), interrupt pin causes boot loop
#include "machine.h"
MACHINE machine;      // also used for v4 as it suppressing machine PGN debug messages
const uint8_t pcaOutputPinNumbers[8] = { 1, 0, 12, 15, 9, 8, 6, 7 };    // all 8 PCA9555 section/machine output pin numbers on v5.0a
const uint8_t pcaInputPinNumbers[]  = { 14, 13, 11, 10, 2, 3, 4, 5 };   // all 8 PCA9555 section/machine output "sensing" pin numbers on v5.0a

#include "zNMEA.h"
NMEAParser<4> nmeaParser;                               // A parser is declared with 3 handlers at most
bool nmeaDebug = 0, extraCRLF;

#include "zUBXParser.h"
UBX_Parser ubxParser;

#include "zFUSEImu.h"
FUSE_Imu fuseImu;

#include "zCANRVC.h"
CAN_RVC canRvc;


#define PANDA_SINGLE 1
#define PAOGI_DUAL 0
bool startup = false;
//elapsedMillis gpsLostTimer;
elapsedMillis LEDTimer;
elapsedMillis imuPandaSyncTimer;
bool ggaReady, imuPandaSyncTrigger;
bool ggaTimeout, relposnedTimeout;
bool SerialGPSactive, SerialGPS2active;
uint32_t dualTime;

constexpr int buffer_size = 256;
uint8_t GPSrxbuffer[buffer_size];       // Extra GPS1 serial rx buffer
uint8_t GPStxbuffer[buffer_size];       // Extra GPS1 serial tx buffer
uint8_t GPS2rxbuffer[buffer_size];      // Extra GPS2 serial rx buffer
uint8_t GPS2txbuffer[buffer_size];      // Extra GPS2 serial tx buffer
uint8_t RTKrxbuffer[buffer_size];       // Extra RTK serial rx buffer
#ifdef AIOv50a
  uint8_t RS232txbuffer[buffer_size];     // Extra RS232 serial tx buffer
  //uint8_t RS232rxbuffer[buffer_size];   // Extra RS232 serial rx buffer (not needed unless custom rs232 rx code is added)
  uint8_t ESP32rxbuffer[buffer_size];     // Extra ESP32 serial rx buffer
  uint8_t ESP32txbuffer[buffer_size];     // Extra ESP32 serial tx buffer
#endif

extern "C" uint32_t set_arm_clock(uint32_t frequency);  // required prototype to set CPU speed




