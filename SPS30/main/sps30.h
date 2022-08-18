/*
 * SPS30.h
 *
 *  Created on: Aug 18, 2022
 *      Author: Admin
 */

#ifndef MAIN_SPS30_H_
#define MAIN_SPS30_H_
#define SPS30_H
#include "esp_system.h"
#include "driver/uart.h"
#include "string.h"
#include "esp_log.h"
#define SPS30_UART1 UART_NUM_1
#define DRIVER_MAJOR 1
#define DRIVER_MINOR 4
#define SPS30_DEBUGSERIAL Serial // default
#define INCLUDE_FWCHECK 1
#define INCLUDE_UART 1
/* used to get single value */
#define v_MassPM1 1
#define v_MassPM2 2
#define v_MassPM4 3
#define v_MassPM10 4
#define v_NumPM0 5
#define v_NumPM1 6
#define v_NumPM2 7
#define v_NumPM4 8
#define v_NumPM10 9
#define v_PartSize 10
/*************************************************************/
/* error codes */
#define SPS30_ERR_OK 0x00
#define SPS30_ERR_DATALENGTH 0X01
#define SPS30_ERR_UNKNOWNCMD 0x02
#define SPS30_ERR_ACCESSRIGHT 0x03
#define SPS30_ERR_PARAMETER 0x04
#define SPS30_ERR_OUTOFRANGE 0x28
#define SPS30_ERR_CMDSTATE 0x43
#define SPS30_ERR_TIMEOUT 0x50
#define SPS30_ERR_PROTOCOL 0x51
#define SPS30_ERR_FIRMWARE 0x88 // added version 1.4
#define MAXRECVBUFLENGTH 128

#define START_MEASURE_FLOAT 0X03
#define START_MEASURE_UNS16 0X05

/*************************************************************/
/* SERIAL COMMUNICATION INFORMATION */
#define SER_START_MEASUREMENT 0x00
#define SER_STOP_MEASUREMENT 0x01
#define SER_READ_MEASURED_VALUE 0x03
#define SER_SLEEP 0x10  // added 1.4
#define SER_WAKEUP 0x11 // added 1.4
#define SER_START_FAN_CLEANING 0x56
#define SER_RESET 0xD3

#define SER_AUTO_CLEANING_INTERVAL 0x80 // Generic autoclean request
#define SER_READ_AUTO_CLEANING 0x81     // read autoclean
#define SER_WRITE_AUTO_CLEANING 0x82    // write autoclean

#define SER_READ_DEVICE_INFO 0xD0         // GENERIC device request
#define SER_READ_DEVICE_PRODUCT_TYPE 0xF0 // CHANGED 1.4
#define SER_READ_DEVICE_RESERVED1 0xF1    // CHANGED 1.4
#define SER_READ_DEVICE_RESERVED2 0xF2    // CHANGED 1.4
#define SER_READ_DEVICE_SERIAL_NUMBER 0xF3

#define SER_READ_VERSION 0xD1 // Added 1.4
#define SER_READ_STATUS 0xD2  // Added 1.4

#define SHDLC_IND 0x7e  // header & trailer
#define TIME_OUT 5000   // timeout to prevent deadlock read
#define RX_DELAY_MS 100 // wait between write and read
/////////////////////////////////////////////////////////////
/* define driver debug
   0 : no messages
   1 : request sending and receiving
   2 : request sending and receiving + show protocol errors */
//////////////////////////////////////////////////////////////
#define DEBUG 0

/** shared variables */
uint8_t _Receive_BUF[MAXRECVBUFLENGTH]; // buffers
uint8_t _Send_BUF[10];
uint8_t _Receive_BUF_Length;
uint8_t _Send_BUF_Length;

int _SPS30_Debug; // program debug level
bool _started;                // indicate the measurement has started
bool _sleep;                  // indicate that SPS30 is in sleep (added 1.4)
bool _WasStarted;             // restart if SPS30 was started before setting sleep (added 1.4)
uint8_t Reported[11];         // use as cache indicator single value
uint8_t _FW_Major, _FW_Minor; // holds firmware major (added 1.4)

/* structure to return all values */
struct sps_values
{
  float MassPM1;  // Mass Concentration PM1.0 [μg/m3]
  float MassPM2;  // Mass Concentration PM2.5 [μg/m3]
  float MassPM4;  // Mass Concentration PM4.0 [μg/m3]
  float MassPM10; // Mass Concentration PM10 [μg/m3]
  float NumPM0;   // Number Concentration PM0.5 [#/cm3]
  float NumPM1;   // Number Concentration PM1.0 [#/cm3]
  float NumPM2;   // Number Concentration PM2.5 [#/cm3]
  float NumPM4;   // Number Concentration PM4.0 [#/cm3]
  float NumPM10;  // Number Concentration PM4.0 [#/cm3]
  float PartSize; // Typical Particle Size [μm]
};
typedef struct{
  uint8_t major; // Firmware level
  uint8_t minor;
  uint8_t HW_version;  // zero on I2C
  uint8_t SHDLC_major; // zero on I2C
  uint8_t SHDLC_minor; // zero on I2C
  uint8_t DRV_major;
  uint8_t DRV_minor;
}SPS30_version;



/* needed for conversion float IEE754 */
typedef union{
  uint8_t array[4];
  float value;
} ByteToFloat;

/* needed for auto interval timing */
typedef union{
  uint8_t array[4];
  uint32_t value;
} ByteToU32;



struct Description{
  uint8_t code;
  char desc[80];
};

enum SPS_status{
  STATUS_OK = 0,
  STATUS_SPEED_ERROR = 1,
  STATUS_LASER_ERROR = 2,
  STATUS_FAN_ERROR = 4
};
bool reset();
bool start();
bool stop();
bool clean();
void SPS30();
bool Instruct(uint8_t type);
uint8_t SetOpMode(uint8_t mode); // added 1.4
/** shared supporting routines */
uint8_t Get_Device_info(uint8_t type, char *ser, uint8_t len);
void DebugPrintf(const char *pcFmt, ...);
bool FWCheck(uint8_t major, uint8_t minor); // added 1.4
float byte_to_float(int x);
uint32_t byte_to_U32(int x);
float Get_Single_Value(uint8_t value);
/** UART / serial related */
// calls
uint8_t ReadFromSerial();
uint8_t SerialToBuffer();
uint8_t SendToSerial();
bool SHDLC_fill_buffer(uint8_t command, uint32_t parameter);
uint8_t SHDLC_calc_CRC(uint8_t *buf, uint8_t first, uint8_t last);
int ByteStuff(uint8_t b, int off);
uint8_t ByteUnStuff(uint8_t b);
void EnableDebugging(uint8_t act);
bool probe();
uint8_t GetAutoCleanInt(uint32_t *val);
uint8_t SetAutoCleanInt(uint32_t val);
void GetErrDescription(uint8_t code, char *buf, int len);
uint8_t GetVersion(SPS30_version *v);
uint8_t GetStatusReg(uint8_t *status);
uint8_t GetValues(struct sps_values *v);
// function prototypes (sometimes the pre-processor does not create prototypes themself on ESPxx)
void ErrtoMess(char *mess, uint8_t r);
void Errorloop(char *mess, uint8_t r);
void GetDeviceInfo();
bool read_all();



#endif /* MAIN_SPS30_H_ */
