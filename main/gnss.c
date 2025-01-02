#include "common.h"

#include <string.h>

#include "myi2c.h"
#include "sync.h"
#include "ubxdefs.h"
#include "ubx.h"
#include "nmea.h"
#include "rtcm.h"

#include "gnss.h"

int gnss_process_data(uint8_t b);
int gnss_reset();

uint8_t gnssProcessedMsg = DEFAULT_GNSS_MSG;
uint8_t gnssRate = DEFAULT_GNSS_RATE;
gnss_msg_t gnssCurrentMsg = NO_MSG;   
uint8_t i2cBuffer[MAX_BUFFER];
bool gnssInitialized = false;

//static const char* TAG = __FILE__;
static const char* TAG = "gnss";

void gnss_init(uint16_t processedMsg, uint8_t rate)
//  initialized the gnss config using ubx protocol
{

  ubx_init();
  nmea_init();

  STOP_ON_ERROR(gnss_reset());

  pause_ms(100);

  gnss_set_processed_msg(processedMsg);

  gnss_set_rate(rate);

  pause_ms(100);
  
  ESP_LOGI(TAG, "Initialized");

  gnssInitialized = 1;
}

esp_err_t gnss_poll_data()
// poll data from gnss through I2C connection
{
  esp_err_t ret = ESP_FAIL;
  const char* msg;

  // read data 
  ret = gnss_i2c_read_data();
  if (ret != ESP_OK)
    return ret;
  
  // print gnss data
  if (gnssProcessedMsg & UBX_MSG)
  {
    msg = ubx_get_output_str();
    if (msg)
    {
      TickType_t timestamp = sync_get_gnss_time_reference()
        + ms_to_tick(ubx_get_output_time_offset_since_pps());
      printf("%03d;UBX;%s\n", timestamp,msg);
    }
  }
  if (gnssProcessedMsg & NMEA_MSG)
  {
    msg = nmea_get_output_str();
    if (msg)
      printf("%03d;NMEA;%s\n",xTaskGetTickCount(),msg);
  }
  if (gnssProcessedMsg & RTCM_MSG)
  {
    msg = rtcm_get_output_str();
    if (msg)
      printf("%03d;RTCM;%s\n",xTaskGetTickCount(),msg);
  }
  
  return ret;
}

int gnss_process_data(uint8_t b)
// process received data
{
  // check for message headers
  if (b == UBX_SYNC1)
  {
    gnssCurrentMsg = UBX_MSG;
    ubx_decode_reset();
  }
  else if (b == NMEA_PREAMBLE)
  {
    gnssCurrentMsg = NMEA_MSG;
    nmea_decode_reset();
  }

  // parse the message 
  switch (gnssCurrentMsg)
  {
  case UBX_MSG:
    if (gnssProcessedMsg & UBX_MSG)
      return ubx_decode_data(b);
    break;
  case RTCM_MSG:
    if (gnssProcessedMsg & RTCM_MSG)
      return rtcm_decode_data(b);
    break;
  case NMEA_MSG:
    if (gnssProcessedMsg & NMEA_MSG)
      return nmea_decode_data(b);
    break;
  default:
    ESP_LOGD(TAG,"unprocessed garbage");
    return ESP_OK;
  }
  return ESP_OK;
}

esp_err_t gnss_reset()
// reset the gnss module (but do not clear the stored information)
{
  return ubx_reset(hot);
}

esp_err_t  gnss_set_rate(uint8_t rate)
// set the frequency for the calcul of GNSS solutions
{
  if (rate == 0)
  {
    ESP_LOGE(TAG,"invalid gnss frequency (%d)",rate);
    return ESP_FAIL;
  }

  esp_err_t ret = ubx_set_rate(rate);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG,"unable to configure gnss Ubx message (message collision?)");
  }
  else
  {
    gnssRate = rate;
    ESP_LOGI(TAG,"rate: %dHz", rate);
  }
  
  return ret;
}

uint8_t gnss_get_rate()
{
  return gnssRate;
}

esp_err_t gnss_set_processed_msg(uint8_t msgMask)
// set the mask for the gnss msg to be processed and displayed
// (UBX_MSG | NMEA_MSG | RTCM_MSG)
{
  esp_err_t ret = ubx_set_processed_msg(msgMask);
  if ( ret != ESP_OK)
    ESP_LOGE(TAG,"unable to configure gnss Ubx message (message collision?)");
  else    
    gnssProcessedMsg = msgMask;
                                        
  ESP_LOGI(TAG,"UBX_MSG: %d, NMEA_MSG: %d, RTCM_MSG: %d",
             (gnssProcessedMsg & UBX_MSG)  ? 1 : 0
           , (gnssProcessedMsg & NMEA_MSG) ? 1 : 0
           , (gnssProcessedMsg & RTCM_MSG) ? 1 : 0);

  return ret;
}

uint8_t gnss_get_processed_msg(void)
// get the mask for the currently processed gnss msg 
// (UBX_MSG | NMEA_MSG | RTCM_MSG)
{
  return gnssProcessedMsg;
}

void gnss_print_format()
{
  if (gnssProcessedMsg & UBX_MSG)
    ESP_LOGI("gnss ubx output format","%s",ubx_get_output_format());
}

bool gnss_is_initialized()
{
  return gnssInitialized;
}

esp_err_t gnss_i2c_read_data()
// read data from gnss through I2C connection
{
  esp_err_t ret;
  uint16_t bytesAvailable = 0;
//printf("begin read\n");
  // get the number of bytes available from GNSS
  // read registers 0xFD (MSB) and 0xFE (LSB) that contain the number of
  // available bytes that can be currently read
  ret = myi2c_read_register(GNSS_I2C_ADDR, 0xFD, i2cBuffer, 2);
  if (ret != ESP_OK)
    return ESP_FAIL;
  bytesAvailable = (uint16_t) (i2cBuffer[0]<< 8 | i2cBuffer[1]);

//printf("###########%d \n",bytesAvailable);
  // if no bytes available, leave
  if (!bytesAvailable || bytesAvailable == 0xFFF)
    return ESP_OK;

  // read the available data from register 0xFF
  // and pass each read byte to the parsers
  uint16_t byteToRead = MIN(bytesAvailable, MAX_BUFFER);
  ret = myi2c_read_register(GNSS_I2C_ADDR, 0xFF, i2cBuffer, byteToRead);
  for (int i = 0; i < byteToRead; i++)
    gnss_process_data(i2cBuffer[i]);
//printf("end read\n");

  return ESP_OK;
}

esp_err_t gnss_i2c_write_data(uint8_t *data, uint16_t length)
// write data from gnss through I2C connection
{
  esp_err_t ret = myi2c_write_register(GNSS_I2C_ADDR, 0xFF, data, length);
  
  return ret;
}

