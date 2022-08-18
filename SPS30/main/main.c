#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "stdio.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <string.h>
#include "esp_err.h"
#include "nvs_flash.h"

#define SPS30_H
#define DRIVER_MAJOR 1
#define DRIVER_MINOR 4
#define SPS30_DEBUGSERIAL Serial // default

static const int PMS_BUF_SIZE = 1024;
#define TXD0_PIN (GPIO_NUM_1)
#define RXD0_PIN (GPIO_NUM_3)
#define pms0_uart UART_NUM_0
#define TXD1_PIN (GPIO_NUM_26)
#define RXD1_PIN (GPIO_NUM_25)
#define SIM7600_uart UART_NUM_1
#define pms0_uart UART_NUM_0
static const char *TAG_RX = "RX";
//enum debug_serial
//{
//  STANDARD = 0, // default
//#ifdef SPS30_DEBUGSERIAL_SODAQ
//  SODAQ = 1
//#endif
//};

uint32_t IRAM_ATTR millis() {
	return xTaskGetTickCount() * portTICK_PERIOD_MS;
}
#define INCLUDE_FWCHECK 1
#define INCLUDE_UART 1

enum serial_port
{
  I2C_COMMS = 0,
  SOFTWARE_SERIAL = 1,
  SERIALPORT = 2,
  SERIALPORT1 = 3,
  SERIALPORT2 = 4,
  SERIALPORT3 = 5,
  COMM_TYPE_SERIAL = 6, // added 1.4.2
  NONE = 6
};

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

/* needed for conversion float IEE754 */
typedef union
{
  uint8_t array[4];
  float value;
} ByteToFloat;

/* needed for auto interval timing */
typedef union
{
  uint8_t array[4];
  uint32_t value;
} ByteToU32;

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

struct Description
{
  uint8_t code;
  char desc[80];
};
struct Description SPS30_ERR_desc[11] =
{
  {SPS30_ERR_OK, "All good"},
  {SPS30_ERR_DATALENGTH, "Wrong data length for this command (too much or little data)"},
  {SPS30_ERR_UNKNOWNCMD, "Unknown command"},
  {SPS30_ERR_ACCESSRIGHT, "No access right for command"},
  {SPS30_ERR_PARAMETER, "Illegal command parameter or parameter out of allowed range"},
  {SPS30_ERR_OUTOFRANGE, "Internal function argument out of range"},
  {SPS30_ERR_CMDSTATE, "Command not allowed in current state"},
  {SPS30_ERR_TIMEOUT, "No response received within timeout period"},
  {SPS30_ERR_PROTOCOL, "Protocol error"},
  {SPS30_ERR_FIRMWARE, "Not supported on this SPS30 firmware level"},
  {0xff, "Unknown Error"}
};
/**
   added version 1.4

   New call was explained to obtain the version levels
   datasheet SPS30 March 2020, page 14

*/


/**
   added version 1.4

   Status register result

   REQUIRES FIRMWARE LEVEL 2.2
*/
enum SPS_status
{
  STATUS_OK = 0,
  STATUS_SPEED_ERROR = 1,
  STATUS_LASER_ERROR = 2,
  STATUS_FAN_ERROR = 4
};

/**
   added version 1.4

   Measurement can be done in FLOAR or unsigned 16bits
   page 6 datasheet SPS30 page 6.

   This driver only uses float
*/
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

// class SPS30
// {
// public:
void SPS30();
bool Instruct(uint8_t type);
uint8_t SetOpMode(uint8_t mode); // added 1.4
/** shared supporting routines */
uint8_t Get_Device_info(uint8_t type, char *ser, uint8_t len);
void DebugPrintf(const char *pcFmt, ...);

/** shared variables */
uint8_t _Receive_BUF[MAXRECVBUFLENGTH]; // buffers
uint8_t _Send_BUF[10];
uint8_t _Receive_BUF_Length;
uint8_t _Send_BUF_Length;

int _SPS30_Debug; // program debug level
// debug_serial _SPS30_Debug_Serial; // serial debug-port to use
bool _started;                // indicate the measurement has started
bool _sleep;                  // indicate that SPS30 is in sleep (added 1.4)
bool _WasStarted;             // restart if SPS30 was started before setting sleep (added 1.4)
uint8_t Reported[11];         // use as cache indicator single value
uint8_t _FW_Major, _FW_Minor; // holds firmware major (added 1.4)

// bool Instruct(uint8_t type);

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
/**
   @brief  Enable or disable the printing of sent/response HEX values.

   @param act : level of debug to set
    0 : no debug message
    1 : sending and receiving data
    2 : 1 +  protocol progress

   @param SelectDebugSerial : select Serial port (see top of SPS30.h)
    This will allow to select a different port than Serial for debug
    messages. As real example an SODAQ NB board is using SerialUSB.
*/
void EnableDebugging(uint8_t act);

/**
   @brief : Perform SPS-30 instructions
*/
bool probe();
bool reset() {
  return (Instruct(SER_RESET));
}
bool start() {
  return (Instruct(SER_START_MEASUREMENT));
}
bool stop() {
  return (Instruct(SER_STOP_MEASUREMENT));
}
bool clean() {
  return (Instruct(SER_START_FAN_CLEANING));
}

/**
   Added 1.4
   @brief Set SPS30 to sleep or wakeup
   Requires Firmwarelevel 2.0
*/
//uint8_t sleep() {
//  return (SetOpMode(SER_SLEEP));
//}
uint8_t wakeup() {
  return (SetOpMode(SER_WAKEUP));
}

/**
   @brief : Set or get Auto Clean interval
*/
uint8_t GetAutoCleanInt(uint32_t *val);
uint8_t SetAutoCleanInt(uint32_t val);

/**
   @brief : retrieve Error message details
*/
void GetErrDescription(uint8_t code, char *buf, int len);

/**
   @brief : retrieve device information from the SPS-30

   On none of the device so far Article code and Product name are
   available.
*/
uint8_t GetSerialNumber(char *ser, uint8_t len) {
  return (Get_Device_info(SER_READ_DEVICE_SERIAL_NUMBER, ser, len));
}
uint8_t GetProductName(char *ser, uint8_t len) {
  return (Get_Device_info(SER_READ_DEVICE_PRODUCT_TYPE, ser, len));  // CHANGED 1.4
}



/** ADDED 1.4
   @brief : retrieve software/hardware version information from the SPS-30

*/
uint8_t GetVersion(SPS30_version *v);

/** ADDED 1.4
   @brief : Read Device Status from the SPS-30

   REQUIRES FIRMWARE 2.2
   The commands are accepted and positive acknowledged on lower level
   firmware, but do not execute.

   @param  *status
    return status as an 'or':
     STATUS_OK = 0,
     STATUS_SPEED_ERROR = 1,
     STATUS_SPEED_CURRENT_ERROR = 2,
     STATUS_FAN_ERROR = 4

   @return
    SPS30_ERR_OK = ok, no isues found
    else SPS30_ERR_OUTOFRANGE, issues found
*/
uint8_t GetStatusReg(uint8_t *status);

/**
   @brief : retrieve all measurement values from SPS-30
*/
uint8_t GetValues(struct sps_values *v);

/**
   @brief : retrieve a specific value from the SPS-30
*/
float GetMassPM1() {
  return (Get_Single_Value(v_MassPM1));
}
float GetMassPM2() {
  return (Get_Single_Value(v_MassPM2));
}
float GetMassPM4() {
  return (Get_Single_Value(v_MassPM4));
}
float GetMassPM10() {
  return (Get_Single_Value(v_MassPM10));
}
float GetNumPM0() {
  return (Get_Single_Value(v_NumPM0));
}
float GetNumPM1() {
  return (Get_Single_Value(v_NumPM1));
}
float GetNumPM2() {
  return (Get_Single_Value(v_NumPM2));
}
float GetNumPM4() {
  return (Get_Single_Value(v_NumPM4));
}
float GetNumPM10() {
  return (Get_Single_Value(v_NumPM10));
}
float GetPartSize() {
  return (Get_Single_Value(v_PartSize));
}




bool Instruct(uint8_t type)
{
  uint8_t ret;

  if (type == SER_START_FAN_CLEANING)
  {
    if (!_started)
    {
      DebugPrintf("ERROR: Sensor is not in measurement mode\n");
      return (false);
    }
  }
  if (SHDLC_fill_buffer(type, 0) != true)
    return (false); // update version 1.4.7

  ret = ReadFromSerial();
  if (ret == SPS30_ERR_OK)
  {

    if (type == SER_START_MEASUREMENT)
    {
      _started = true;
      vTaskDelay(1000/portTICK_RATE_MS);
    }
    else if (type == SER_STOP_MEASUREMENT)
      _started = false;

    else if (type == SER_RESET)
    {
      _started = false;

      vTaskDelay(2000/portTICK_RATE_MS);
    }

    return (true);
  }

  DebugPrintf("instruction failed\n");
  return (false);
}
void SPS30()
{
  _Send_BUF_Length = 0;
  _Receive_BUF_Length = 0;
  _SPS30_Debug = 0;
  _started = false;
  _sleep = false;
  _FW_Major = _FW_Minor = 0;
  memset(Reported, 0x1, sizeof(Reported)); // Trigger reading single value cache
}
int level = 0; // used to show protocol errors as well
static char prfbuf[256];
void DebugPrintf(const char *pcFmt, ...)
{
  va_list pArgs;

  // detect protocol errors
  if (_SPS30_Debug > level)
  {

    va_start(pArgs, pcFmt);
    vsprintf(prfbuf, pcFmt, pArgs);
    va_end(pArgs);
    printf("%s",prfbuf);
//    ESP_LOGI(TAG_RX,"%s",prfbuf);
  }
  // reset check-level
  level = 0;
}

void EnableDebugging(uint8_t act)
{
  _SPS30_Debug = act;
  // _SPS30_Debug_Serial = SelectDebugSerial;
}

bool probe()
{
  SPS30_version v;
  if (GetVersion(&v) == SPS30_ERR_OK)
  {
    _FW_Major = v.major;
    _FW_Minor = v.minor;
    return (true);
  }
  return (false);
}

bool FWCheck(uint8_t major, uint8_t minor)
{

  if (!INCLUDE_FWCHECK)
    return (true);

  // do we have the current FW level
  if (_FW_Major == 0)
  {
    if (!probe())
      return (false);
  }

  // if requested level is HIGHER than current
  if (major > _FW_Major)
    return (false);
  if (minor > _FW_Minor)
    return (false);
  return (true);
}

uint8_t GetStatusReg(uint8_t *status)
{
  uint8_t ret, offset;
  *status = 0x0;
  // check for minimum Firmware level
  if (!FWCheck(2, 2))
    return (SPS30_ERR_FIRMWARE);
  // fill buffer to read_status register and clear after reading
  if (!SHDLC_fill_buffer(SER_READ_STATUS, 0))
    return (SPS30_ERR_PARAMETER);
  ret = ReadFromSerial();
  offset = 5;
  if (ret != SPS30_ERR_OK)
    return (ret);
  if (_Receive_BUF[offset + 1] & 0b00100000)
    *status |= STATUS_SPEED_ERROR;
  if (_Receive_BUF[offset + 3] & 0b00100000)
    *status |= STATUS_LASER_ERROR;
  if (_Receive_BUF[offset + 3] & 0b00010000)
    *status |= STATUS_FAN_ERROR;
  if (*status != 0x0)
    return (SPS30_ERR_OUTOFRANGE);
  return (SPS30_ERR_OK);
}
uint8_t SetOpMode(uint8_t mode)
{ // check for minimum Firmware level
  if (!FWCheck(2, 0))
    return (SPS30_ERR_FIRMWARE);

  // set to sleep
  if (mode == SER_SLEEP)
  {

    // if already in sleep
    if (_sleep)
      return (SPS30_ERR_OK);

    // if not idle
    if (_started)
    {
      if (!stop())
        return (SPS30_ERR_PROTOCOL);
      _WasStarted = true;
    }
    else
      _WasStarted = false;

    // go to sleep
    if (!Instruct(SER_SLEEP))
      return (SPS30_ERR_PROTOCOL);
    _sleep = true;
  }
  // wake-up
  else if (mode == SER_WAKEUP)
  { // if not in sleep
    if (!_sleep)
      return (SPS30_ERR_OK);
    char *command = "0xff";
    int len = strlen(command);
    uart_write_bytes(SIM7600_uart, command, len);
//    Serial2.write(0xff);
    // give some time for the SPS30 to act on toggle as WAKEUP must be sent in 100mS
    vTaskDelay(10/portTICK_RATE_MS);

    if (!Instruct(SER_WAKEUP))
      return (SPS30_ERR_PROTOCOL);

    // give time for SPS30 to go idle
    vTaskDelay(100/portTICK_RATE_MS);

    // indicate not in sleep anymore
    _sleep = false;

    // was SPS30 started before instructed to go to sleep
    if (_WasStarted)
    {
      if (!start())
        return (SPS30_ERR_PROTOCOL);
    }
  }
  else
    return (SPS30_ERR_PARAMETER);

  return (SPS30_ERR_OK);
}

uint8_t GetVersion(SPS30_version *v)
{
  uint8_t ret, offset;
  memset(v, 0x0, sizeof(SPS30_version));
  // fill buffer to send
  if (!SHDLC_fill_buffer(SER_READ_VERSION, 0))
    return (SPS30_ERR_PARAMETER);

  ret = ReadFromSerial();
  offset = 5;
  v->major = _Receive_BUF[offset + 0];
  v->minor = _Receive_BUF[offset + 1];
  v->HW_version = _Receive_BUF[offset + 3];
  v->SHDLC_major = _Receive_BUF[offset + 5];
  v->SHDLC_minor = _Receive_BUF[offset + 6];
  v->DRV_major = DRIVER_MAJOR;
  v->DRV_minor = DRIVER_MINOR;
  return (ret);
}

uint8_t Get_Device_info(uint8_t type, char *ser, uint8_t len)
{
  uint8_t ret, i, offset;
  // fill buffer to send
  if (!SHDLC_fill_buffer((type), 0))
    return (SPS30_ERR_PARAMETER);
  ret = ReadFromSerial();
  offset = 5;
  if (ret != SPS30_ERR_OK)
    return (ret);
  // get data
  for (i = 0; i < len; i++)
  {
    ser[i] = _Receive_BUF[i + offset];
    if (ser[i] == 0x0)
      break;
  }
  return (SPS30_ERR_OK);
}
uint8_t SetAutoCleanInt(uint32_t val)
{
  // fill buffer to send
  if (SHDLC_fill_buffer(SER_WRITE_AUTO_CLEANING, val) != true)
    return (SPS30_ERR_PARAMETER);

  return (ReadFromSerial());
}

float Get_Single_Value(uint8_t value)
{
  static struct sps_values v;

  if (value > v_PartSize)
    return (-1);

  // if already reported this value
  if (Reported[value])
  {
    // do a reload
    if (GetValues(&v) != SPS30_ERR_OK)
      return (-1);
    memset(Reported, 0x0, sizeof(Reported));
  }

  Reported[value] = 1;

  switch (value)
  {
    case v_MassPM1:
      return (v.MassPM1);
    case v_MassPM2:
      return (v.MassPM2);
    case v_MassPM4:
      return (v.MassPM4);
    case v_MassPM10:
      return (v.MassPM10);
    case v_NumPM0:
      return (v.NumPM0);
    case v_NumPM1:
      return (v.NumPM1);
    case v_NumPM2:
      return (v.NumPM2);
    case v_NumPM4:
      return (v.NumPM4);
    case v_NumPM10:
      return (v.NumPM10);
    case v_PartSize:
      return (v.PartSize);
  }

  return (0);
}

uint8_t GetAutoCleanInt(uint32_t *val)
{
  uint8_t ret, offset;
  // fill buffer to send
  if (SHDLC_fill_buffer(SER_READ_AUTO_CLEANING, 0) != true)
    return (SPS30_ERR_PARAMETER);
  ret = ReadFromSerial();
  offset = 5;
  // get data
  *val = byte_to_U32(offset);
  return (ret);
}

void GetErrDescription(uint8_t code, char *buf, int len)
{
  int i = 0;

  while (SPS30_ERR_desc[i].code != 0xff)
  {
    if (SPS30_ERR_desc[i].code == code)
      break;
    i++;
  }

  strncpy(buf, SPS30_ERR_desc[i].desc, len);
}

uint8_t GetValues(struct sps_values *v)
{
  uint8_t ret;
  uint8_t offset;

  // measurement started already?
  if (!_started)
  {
    if (!start())
      return (SPS30_ERR_CMDSTATE);
  }
  offset = 5;

  // fill buffer to send
  if (SHDLC_fill_buffer(SER_READ_MEASURED_VALUE, 0) != true)
    return (SPS30_ERR_PARAMETER);
  ret = ReadFromSerial();
  if (ret != SPS30_ERR_OK)
    return (ret);
  /// buffer : hdr addr cmd state length data....data crc hdr
  ///           0    1   2    3     4     5
  // check length
  if (_Receive_BUF[4] != 0x28)
  {
    DebugPrintf("%d Not enough bytes for all values\n", _Receive_BUF[4]);
    return (SPS30_ERR_DATALENGTH);
  }
  memset(v, 0x0, sizeof(struct sps_values));
  // get data
  v->MassPM1 = byte_to_float(offset);
  v->MassPM2 = byte_to_float(offset + 4);
  v->MassPM4 = byte_to_float(offset + 8);
  v->MassPM10 = byte_to_float(offset + 12);
  v->NumPM0 = byte_to_float(offset + 16);
  v->NumPM1 = byte_to_float(offset + 20);
  v->NumPM2 = byte_to_float(offset + 24);
  v->NumPM4 = byte_to_float(offset + 28);
  v->NumPM10 = byte_to_float(offset + 32);
  v->PartSize = byte_to_float(offset + 36);
  return (SPS30_ERR_OK);
}

float byte_to_float(int x)
{
  ByteToFloat conv;

  for (int i = 0; i < 4; i++)
  {
    conv.array[3 - i] = _Receive_BUF[x + i]; // or conv.array[i] = _Receive_BUF[x+i]; depending on endianness
  }

  return conv.value;
}

uint32_t byte_to_U32(int x)
{
  ByteToU32 conv;

  for (int i = 0; i < 4; i++)
  {
    conv.array[3 - i] = _Receive_BUF[x + i]; // or conv.array[i] = _Receive_BUF[x+i]; depending on endianness
  }

  return conv.value;
}

int ByteStuff(uint8_t b, int off)
{
  uint8_t x = 0;

  switch (b)
  {
    case 0x11:
      {
        x = 0x31;
        break;
      }
    case 0x13:
      {
        x = 0x33;
        break;
      }
    case 0x7d:
      {
        x = 0x5d;
        break;
      }
    case 0x7e:
      {
        x = 0x5e;
        break;
      }
  }

  if (x == 0)
    _Send_BUF[off++] = b;
  else
  {
    _Send_BUF[off++] = 0x7D;
    _Send_BUF[off++] = x;
  }

  return (off);
}

uint8_t ByteUnStuff(uint8_t b)
{
  switch (b)
  {
    case 0x31:
      return (0x11);
    case 0x33:
      return (0x13);
    case 0x5d:
      return (0x7d);
    case 0x5e:
      return (0x7e);

    default:
      level = 1;
      DebugPrintf("Incorrect byte Unstuffing. Got: 0x%02X\n", b);
      return (0);
  }
}

bool SHDLC_fill_buffer(uint8_t command, uint32_t parameter)
{
  parameter = 0;
  memset(_Send_BUF, 0x0, sizeof(_Send_BUF));
  _Send_BUF_Length = 0;

  int i = 0;
  uint8_t tmp;

  _Send_BUF[i++] = SHDLC_IND;
  _Send_BUF[i++] = 0x0; // address SPS30 is zero
  _Send_BUF[i++] = command;

  switch (command)
  {

    case SER_START_MEASUREMENT:
      _Send_BUF[i++] = 2; // length
      _Send_BUF[i++] = 0x1;
      _Send_BUF[i++] = START_MEASURE_FLOAT; // CHANGED 1.4
      break;

    case SER_READ_STATUS:
      _Send_BUF[i++] = 1; // length
      _Send_BUF[i++] = 1; // Clear bits after reading (as the condition might have been cleared) //1.4.4
      break;

    case SER_STOP_MEASUREMENT:
    case SER_READ_MEASURED_VALUE:
    case SER_START_FAN_CLEANING:
    case SER_READ_VERSION:
    case SER_RESET:
    case SER_WAKEUP:
    case SER_SLEEP:
      _Send_BUF[i++] = 0; // length
      break;

    case SER_READ_DEVICE_PRODUCT_TYPE:
    case SER_READ_DEVICE_SERIAL_NUMBER:
      _Send_BUF[2] = SER_READ_DEVICE_INFO;
      _Send_BUF[i++] = 1; // length
      _Send_BUF[i++] = command & 0x0f;
      break;

    case SER_READ_AUTO_CLEANING:
      _Send_BUF[2] = SER_AUTO_CLEANING_INTERVAL;
      _Send_BUF[i++] = 1; // length
      _Send_BUF[i++] = 0; // Subcommand, this value must be set to 0x00
      break;

    case SER_WRITE_AUTO_CLEANING:
      _Send_BUF[2] = SER_AUTO_CLEANING_INTERVAL;

      _Send_BUF[i++] = 5;           // length
      _Send_BUF[i++] = 0;           // Subcommand, this value must be set to 0x00
      tmp = parameter >> 24 & 0xff; // change order depending on the endians...
      i = ByteStuff(tmp, i);
      tmp = parameter >> 16 & 0xff;
      i = ByteStuff(tmp, i);
      tmp = parameter >> 8 & 0xff;
      i = ByteStuff(tmp, i);
      tmp = parameter & 0xff;
      i = ByteStuff(tmp, i);
      break;

    default:
      return (false);
      break;
  }

  // add CRC and check for byte stuffing
  tmp = SHDLC_calc_CRC(_Send_BUF, 1, i);
  i = ByteStuff(tmp, i);
  _Send_BUF[i] = SHDLC_IND;
  _Send_BUF_Length = ++i;
  return (true);
}

uint8_t SHDLC_calc_CRC(uint8_t *buf, uint8_t first, uint8_t last)
{
  uint8_t i;
  uint32_t ret = 0;

  for (i = first; i <= last; i++)
    ret += buf[i];
  return (~(ret & 0xff));
}

uint8_t SendToSerial()
{
  uint8_t i;

  if (_Send_BUF_Length == 0)
    return (SPS30_ERR_DATALENGTH);

  if (_SPS30_Debug)
  {
    DebugPrintf("Sending: ");
    for (i = 0; i < _Send_BUF_Length; i++)
      DebugPrintf(" 0x%02X", _Send_BUF[i]);
    DebugPrintf("\n");
  }

  for (i = 0; i < _Send_BUF_Length; i++)
	  uart_write_bytes(SIM7600_uart,&_Send_BUF[i], 1);

  // indicate that command has been sent
  _Send_BUF_Length = 0;
  return (SPS30_ERR_OK);
}

uint8_t ReadFromSerial()
{
  uint8_t ret;
uart_flush(SIM7600_uart); // flush anything pending
  // write to serial
  ret = SendToSerial();
  if (ret != SPS30_ERR_OK)
    return (ret);

  // wait
  vTaskDelay(10/portTICK_RATE_MS);

  // read serial
  ret = SerialToBuffer();
  if (ret != SPS30_ERR_OK)
    return (ret);

  /**
     check CRC.
     CRC MIGHT have been byte stuffed as well but that is handled
     in SerialtoBuffer !
     buffer : hdr addr cmd state length data....data crc hdr
               0    1   2    3     4     5       -2   -1  -0
  */

  ret = SHDLC_calc_CRC(_Receive_BUF, 1, _Receive_BUF_Length - 2);
  if (_Receive_BUF[_Receive_BUF_Length - 1] != ret)
  {
    DebugPrintf("CRC error. expected 0x%02X, got 0x%02X\n", _Receive_BUF[_Receive_BUF_Length - 1], ret);
    return (SPS30_ERR_PROTOCOL);
  }

  // check status
  if (_Receive_BUF[3] != SPS30_ERR_OK)
  {
    DebugPrintf("%x : state error\n", _Receive_BUF[3]);
  }

  return (_Receive_BUF[3]);
}

uint8_t SerialToBuffer()
{
  uint32_t startTime;
  bool byte_stuff = false;
  uint8_t i;

  startTime = millis();
  i = 0;

  // read until last 0x7E
  while (true)
  {
    // prevent deadlock
    if (millis() - startTime > TIME_OUT)
    {
      level = 1;
      DebugPrintf("TimeOut during reading byte %d\n", i);
      return (SPS30_ERR_TIMEOUT);
    }
//    vTaskDelay(10/portTICK_PERIOD_MS);
    const int rxBytes =uart_read_bytes(SIM7600_uart,&_Receive_BUF[i], 1, 1000 / portTICK_RATE_MS);
//    ESP_LOGI(TAG_RX,"rx bytes %d",rxBytes);
//    ESP_LOGI(TAG_RX,"data %x",_Receive_BUF[i]);

      // check for good header
      if (i == 0)
      {

        if (_Receive_BUF[i] != SHDLC_IND)
        {
          level = 1;
          ESP_LOGI(TAG_RX,"Incorrect Header.... Expected 0x7E got 0x02X %c",_Receive_BUF[i]);
          DebugPrintf("Incorrect Header. Expected 0x7E got 0x02X\n", _Receive_BUF[i]);
          return (SPS30_ERR_PROTOCOL);
        }
      }
      else
      {

        // detect Byte stuffing
        if (_Receive_BUF[i] == 0x7D)
        {
          i--; // remove stuffing byte
          byte_stuff = true;
        }

        // handle byte stuffing
        else if (byte_stuff)
        {
          _Receive_BUF[i] = ByteUnStuff(_Receive_BUF[i]);
          byte_stuff = false;
        }

        // check last byte received
        else if (_Receive_BUF[i] == SHDLC_IND)
        {

          _Receive_BUF_Length = i;

          if (_SPS30_Debug)
          {
            DebugPrintf("Received: ");
            for (i = 0; i < _Receive_BUF_Length + 1; i++)
              DebugPrintf("0x%02X ", _Receive_BUF[i]);
            DebugPrintf("length: %d\n\n", _Receive_BUF_Length);
          }

          /* if a board can not handle 115K you get uncontrolled input
             that can result in short /wrong messages
          */
          if (_Receive_BUF_Length < 3)
            return (SPS30_ERR_PROTOCOL);

          return (SPS30_ERR_OK);
        }
      }

      i++;

      if (i > MAXRECVBUFLENGTH)
      {
        DebugPrintf("\nReceive buffer full\n");
        return (SPS30_ERR_PROTOCOL);
      }
  }
}
/////////////////////////////////////////////////////////////
/* define driver debug
   0 : no messages
   1 : request sending and receiving
   2 : request sending and receiving + show protocol errors */
//////////////////////////////////////////////////////////////
#define DEBUG 0
// function prototypes (sometimes the pre-processor does not create prototypes themself on ESPxx)
void ErrtoMess(char *mess, uint8_t r);
void Errorloop(char *mess, uint8_t r);
void GetDeviceInfo();
bool read_all();

/**
   @brief : read and display device info
*/
void GetDeviceInfo()
{
  char buf[32];
  uint8_t ret;
  SPS30_version v;
  ret = GetSerialNumber(buf, 32);
  if (ret == SPS30_ERR_OK) {
	  ESP_LOGI(TAG_RX,"Serial number : ");
    if (strlen(buf) > 0)  ESP_LOGI(TAG_RX,"%s",buf);
    else ESP_LOGI(TAG_RX,"not available");
  }
  else
    ErrtoMess((char *) "could not get serial number", ret);

  // try to get product name
  ret = GetProductName(buf, 32);
  if (ret == SPS30_ERR_OK)  {
	  ESP_LOGI(TAG_RX,"Product name  : ");
    if (strlen(buf) > 0)   ESP_LOGI(TAG_RX,"%s",buf);
    else ESP_LOGI(TAG_RX,"not available");
  }
  else
    ErrtoMess((char *) "could not get product name.", ret);

  // try to get version info
  ret = GetVersion(&v);
  if (ret != SPS30_ERR_OK) {
	  ESP_LOGI(TAG_RX,"Can not read version info");
    return;
  }
  ESP_LOGI(TAG_RX,"Firmware level: %d.%d",v.major,v.minor);
  ESP_LOGI(TAG_RX,"Hardware level: %d",v.HW_version);
  ESP_LOGI(TAG_RX,"SHDLC protocol: %d.%d",v.SHDLC_major,v.SHDLC_minor);
  ESP_LOGI(TAG_RX,"Library level : %d.%d",v.DRV_major,v.DRV_minor);
}

/**
   @brief : read and display all values
*/
bool read_all()
{
  static bool header = true;
  uint8_t ret, error_cnt = 0;
  struct sps_values val;

  // loop to get data
  do {

    ret = GetValues(&val);

    // data might not have been ready
    if (ret == SPS30_ERR_DATALENGTH) {

      if (error_cnt++ > 3) {
        ErrtoMess((char *) "Error during reading values: ", ret);
        return (false);
      }
      vTaskDelay(1000/portTICK_RATE_MS);
    }

    // if other error
    else if (ret != SPS30_ERR_OK) {
      ErrtoMess((char *) "Error during reading values: ", ret);
      return (false);
    }

  } while (ret != SPS30_ERR_OK);

  // only print header first time
  if (header) {
	  ESP_LOGI(TAG_RX,"-------------Mass -----------    ------------- Number --------------   -Average-");
	  ESP_LOGI(TAG_RX,"     Concentration [μg/m3]             Concentration [#/cm3]             [μm]");
	  ESP_LOGI(TAG_RX,"P1.0\tP2.5\tP4.0\tP10\tP0.5\tP1.0\tP2.5\tP4.0\tP10\tPartSize\n");
    header = false;
  }
  ESP_LOGI(TAG_RX,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",val.MassPM1,val.MassPM2,val.MassPM4,val.MassPM10,val.NumPM0,val.NumPM1,val.NumPM2,val.NumPM4,val.NumPM10,val.PartSize);
  return (true);
}

/**
    @brief : continued loop after fatal error
    @param mess : message to display
    @param r : error code

    if r is zero, it will only display the message
*/
void Errorloop(char *mess, uint8_t r)
{
  if (r) ErrtoMess(mess, r);
  else ESP_LOGI(TAG_RX,"%s",mess);
  ESP_LOGI(TAG_RX,"Program on hold");
  for (;;) vTaskDelay(100000/portTICK_RATE_MS);
}

/**
    @brief : display error message
    @param mess : message to display
    @param r : error code

*/
void ErrtoMess(char *mess, uint8_t r)
{
  char buf[80];
  ESP_LOGI(TAG_RX,"%s",mess);
  GetErrDescription(r, buf, 80);
  ESP_LOGI(TAG_RX,"%s",buf);
}
void inituart(int baud, uart_port_t uart_no, int tx_pin, int rx_pin, int rx_buffer) {
  const uart_config_t uart_config = {
    .baud_rate = baud,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(uart_no, &uart_config);
  uart_set_pin(uart_no, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(uart_no, rx_buffer, 0, 0, NULL, 0);
}
void app_main(void)
{
	esp_err_t ret;
		// Initialise flash in esp32
		ret = nvs_flash_init();
		if ( ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND ) {
			ESP_ERROR_CHECK(nvs_flash_erase());
			ret = nvs_flash_init();
		}
		ESP_ERROR_CHECK(ret);
	  EnableDebugging(DEBUG);

	  inituart(115200, pms0_uart, TXD0_PIN, RXD0_PIN, PMS_BUF_SIZE);
	  inituart(115200, SIM7600_uart, TXD1_PIN, RXD1_PIN, PMS_BUF_SIZE);
	  ESP_LOGI(TAG_RX,"Trying to connect");

	  // check for SPS30 connection
	  if (! probe()) Errorloop((char *) "could not probe / connect with SPS30.", 0);
	  else ESP_LOGI(TAG_RX,"Detected SPS30.");

	  // reset SPS30 connection
	  if (! reset()) Errorloop((char *) "could not reset.", 0);

	  // start measurement
	  if (start()) ESP_LOGI(TAG_RX,"Measurement started");
	  else Errorloop((char *) "Could NOT start measurement", 0);
	  SetAutoCleanInt(20);
	  clean();

	  while(1){
		    read_all();
		    vTaskDelay(1000/portTICK_RATE_MS);
	  }
}
