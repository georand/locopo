#ifndef __COMMON_H__
#define __COMMON_H__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_spi_flash.h"
#include "sdkconfig.h"

#define PROJECT_NAME "locopo"

/************************************************************************************* 
 DEFAULT SYSTEM SETTNGS
 these settings are used if the nvram (flash memory) is empty
 otherwise they are replaced by the ones stored in the flash memory
*************************************************************************************/ 

// frequency at wich the trigers are sent to the camera (Hz)
// duration of the trigger = 1000/(2*triggerRate) 
#define DEFAULT_TRIGGER_RATE 4

// GNSS solution calcul frequency (Hz) 
#define DEFAULT_GNSS_RATE 4

// types of GNSS messages to be processed and displayed (UBX_MSG | NMEA_MSG | RTM_MSG)
#define DEFAULT_GNSS_MSG UBX_MSG 

// IMU sensing frequency (Hz) 
#define DEFAULT_IMU_RATE 4

// types of IMU messagesreports to be processed and displayed
// (ROTATION_RPT | ACCELERATION_LINEAR_RPT)
#define DEFAULT_IMU_RPT ROTATION_RPT | ROTATION_GAME_RPT | ACCELERATION_LINEAR_RPT

// log level receive on USB serial 
//ESP_LOG_INFO  : display only GNSS solution and IMU data
//                message format is defined in ubx.c:ubx_print_nav_sol()
//ESP_LOG_DEBUG : display debug info
#define DEFAULT_LOG_LEVEL ESP_LOG_INFO

/************************************************************************************* 
 I2C SETTINGS
*************************************************************************************/ 
#define GNSS_I2C_ADDR 0x42 // ZED-F9P  https://www.sparkfun.com/products/15136
#define IMU_I2C_ADDR 0x4b // ZED-F9P https://www.sparkfun.com/products/14686
#define MY_I2C_PORT 0
#define MY_I2C_SDA 23 
#define MY_I2C_SCL 22
#define MY_I2C_WAIT 100/portTICK_PERIOD_MS // I2C max wait (100ms)
#define MY_I2C_MASTER_TOUT_CNUM 1048575    // I2C timeout delay (see https://github.com/espressif/esp-idf/issues/680)

/************************************************************************************* 
 GPIO SETTINGS
*************************************************************************************/ 

#define LED_GPIO         13 // red led (and GPIO 13) used for gnss pps check
#define TRIGGER_OUT_GPIO 26 // A0 pin -> trigger output
#define IMU_RST_OUT_GPIO 25 // A1 pin -> imu reset output
#define IMU_INT_IN_GPIO  15 // 15 pin <- imu interupt input
#define GNSS_PPS_IN_GPIO 14 // 14 pin <- gnss pps input

/************************************************************************************* 
*************************************************************************************/ 

#define SWAP16(X) ((((X) >>  8) & 0x00ff) | (((X) << 8) & 0xff00))
#define SIGN(X)   ((X < 0) ? -X : X)
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#define STOP_ON_ERROR(x) {                                                    \
        esp_err_t __err_rc = (x);                                             \
        if (__err_rc != ESP_OK) {                                             \
         print_error(__err_rc,                       \
                                      __FILE__, __LINE__,__ASSERT_FUNC, #x);  \
        while(1) pause_ms(1000);                                              \
        }}

#ifndef LOG_LOCAL_LEVEL
  #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG 
#endif
#include <esp_log.h>

#define MAX_BUFFER 256 // should be enough for any gnss and imu command or message 

// settings read from and saved to flash memory 
// 1 byte alignment
#pragma pack(push, 1)
typedef struct
{
  uint16_t gnssProcessedMsg;
  uint8_t gnssRate;
  uint16_t imuProcessedRpt;
  uint8_t imuRate;
  uint8_t triggerRate;
  uint8_t triggerManual; 
  uint8_t triggerDisplay;
  uint8_t syncDisplay;
  uint8_t logLevel;
} settings_t;
#pragma pack(pop)

void display_settings(settings_t settings);
void load_settings(settings_t* settings);
void save_settings(settings_t* settings);
void pause_ms(uint32_t t);
uint32_t tick_to_ms(TickType_t tck);
TickType_t ms_to_tick(uint32_t ms);
void init_log(uint8_t logLevel);
void toggle_log_level(char* tag);
uint8_t debug();
void print_error(esp_err_t rc, const char *file,
                 int line, const char *function, const char *expression); 
void my_vTaskGetRunTimeStats();
#endif //define __COMMON_H__
