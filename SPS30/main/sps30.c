/*
 * SPS30.c
 *
 *  Created on: Aug 18, 2022
 *      Author: Admin
 */



#include "sps30.h"

static const char *TAG_SPS30 = "SPS30";

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

uint32_t IRAM_ATTR millis() {
	return xTaskGetTickCount() * portTICK_PERIOD_MS;
}
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
    return (false);

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
  if (_SPS30_Debug > level){
    va_start(pArgs, pcFmt);
    vsprintf(prfbuf, pcFmt, pArgs);
    va_end(pArgs);
    printf("%s",prfbuf);
  }
  // reset check-level
  level = 0;
}

void EnableDebugging(uint8_t act)
{
  _SPS30_Debug = act;
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

uint8_t GetStatusReg(uint8_t *status){
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
uint8_t SetOpMode(uint8_t mode){
	// check for minimum Firmware level
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
    uart_write_bytes(UART_NUM_1, command, len);;
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
uint8_t wakeup() {
  return (SetOpMode(SER_WAKEUP));
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
uint8_t GetSerialNumber(char *ser, uint8_t len) {
  return (Get_Device_info(SER_READ_DEVICE_SERIAL_NUMBER, ser, len));
}
uint8_t GetProductName(char *ser, uint8_t len) {
  return (Get_Device_info(SER_READ_DEVICE_PRODUCT_TYPE, ser, len));  // CHANGED 1.4
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
	  uart_write_bytes(UART_NUM_1,&_Send_BUF[i], 1);

  // indicate that command has been sent
  _Send_BUF_Length = 0;
  return (SPS30_ERR_OK);
}

uint8_t ReadFromSerial()
{
  uint8_t ret;
uart_flush(UART_NUM_1); // flush anything pending
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
    uart_read_bytes(UART_NUM_1,&_Receive_BUF[i], 1, 1000 / portTICK_RATE_MS);
      // check for good header
      if (i == 0)
      {

        if (_Receive_BUF[i] != SHDLC_IND)
        {
          level = 1;
          ESP_LOGI(TAG_SPS30,"Incorrect Header.... Expected 0x7E got 0x02X %c",_Receive_BUF[i]);
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
	  ESP_LOGI(TAG_SPS30,"Serial number : ");
    if (strlen(buf) > 0)  ESP_LOGI(TAG_SPS30,"%s",buf);
    else ESP_LOGI(TAG_SPS30,"not available");
  }
  else
    ErrtoMess((char *) "could not get serial number", ret);

  // try to get product name
  ret = GetProductName(buf, 32);
  if (ret == SPS30_ERR_OK)  {
	  ESP_LOGI(TAG_SPS30,"Product name  : ");
    if (strlen(buf) > 0)   ESP_LOGI(TAG_SPS30,"%s",buf);
    else ESP_LOGI(TAG_SPS30,"not available");
  }
  else
    ErrtoMess((char *) "could not get product name.", ret);

  // try to get version info
  ret = GetVersion(&v);
  if (ret != SPS30_ERR_OK) {
	  ESP_LOGI(TAG_SPS30,"Can not read version info");
    return;
  }
  ESP_LOGI(TAG_SPS30,"Firmware level: %d.%d",v.major,v.minor);
  ESP_LOGI(TAG_SPS30,"Hardware level: %d",v.HW_version);
  ESP_LOGI(TAG_SPS30,"SHDLC protocol: %d.%d",v.SHDLC_major,v.SHDLC_minor);
  ESP_LOGI(TAG_SPS30,"Library level : %d.%d",v.DRV_major,v.DRV_minor);
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
	  ESP_LOGI(TAG_SPS30,"-------------Mass -----------    ------------- Number --------------   -Average-");
	  ESP_LOGI(TAG_SPS30,"     Concentration [μg/m3]             Concentration [#/cm3]             [μm]");
	  ESP_LOGI(TAG_SPS30,"P1.0\tP2.5\tP4.0\tP10\tP0.5\tP1.0\tP2.5\tP4.0\tP10\tPartSize\n");
    header = false;
  }
  ESP_LOGI(TAG_SPS30,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",val.MassPM1,val.MassPM2,val.MassPM4,val.MassPM10,val.NumPM0,val.NumPM1,val.NumPM2,val.NumPM4,val.NumPM10,val.PartSize);
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
  else ESP_LOGI(TAG_SPS30,"%s",mess);
  ESP_LOGI(TAG_SPS30,"Program on hold");
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
  ESP_LOGI(TAG_SPS30,"%s",mess);
  GetErrDescription(r, buf, 80);
  ESP_LOGI(TAG_SPS30,"%s",buf);
}




//void inituart(int baud, uart_port_t uart_no, int tx_pin, int rx_pin, int rx_buffer) {
//  const uart_config_t uart_config = {
//    .baud_rate = baud,
//    .data_bits = UART_DATA_8_BITS,
//    .parity = UART_PARITY_DISABLE,
//    .stop_bits = UART_STOP_BITS_1,
//    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
//  };
//  uart_param_config(uart_no, &uart_config);
//  uart_set_pin(uart_no, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
//  uart_driver_install(uart_no, rx_buffer, 0, 0, NULL, 0);
//}



