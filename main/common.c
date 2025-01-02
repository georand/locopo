#include "common.h"

#include <string.h>

#include "gnss.h"
#include "sync.h"
#include "trigger.h"
#include "imu.h"
#include "mynvs.h"

//static const char* TAG = __FILE__;
static const char* TAG = "common";

esp_log_level_t mainLogLevel = LOG_LOCAL_LEVEL;


void display_settings(settings_t settings)
{
  ESP_LOGI("Settings",
           "\n\t\tgnssProcessedMsg=%d\n\t\tgnssRate=%d\n\t\timuProcessedRpt=%d\n\t\timuRate=%d\n\t\ttriggerRate=%d\n\t\ttriggerManual=%d\n\t\ttriggerDisplay=%d\n\t\tsyncDisplay=%d\n\t\tlogLevel=%d",
           settings.gnssProcessedMsg, settings.gnssRate,
           settings.imuProcessedRpt, settings.imuRate,
           settings.triggerRate,settings.triggerManual,settings.triggerDisplay,
           settings.syncDisplay,settings.logLevel);
}

void load_settings(settings_t* settings)
{
  if (!settings)
    return;

  nvs_handle_t nvsHandle;
  esp_err_t ret;

  mynvs_flash_init();
  ret = mynvs_open(&nvsHandle,PROJECT_NAME);
  if (ret == ESP_OK)
     mynvs_load(nvsHandle,"settings",settings,sizeof(*settings));
  mynvs_close(nvsHandle);
  mynvs_flash_deinit();
  if (ret != ESP_OK || !settings->logLevel)
  // bad settings, fall back to default, erase NVS and save the default settings   
  {
    ESP_LOGE(TAG,"bad settings storage. Falling back to default settings");
    settings->gnssProcessedMsg = DEFAULT_GNSS_MSG;
    settings->gnssRate = DEFAULT_GNSS_RATE;
    settings->imuProcessedRpt = DEFAULT_IMU_RPT;
    settings->imuRate = DEFAULT_IMU_RATE;
    settings->triggerRate = DEFAULT_TRIGGER_RATE;
    settings->triggerManual = 0;
    settings->triggerDisplay = 1;
    settings->syncDisplay = 1;    
    settings->logLevel = DEFAULT_LOG_LEVEL;    
    mynvs_flash_erase();
    save_settings(settings);
  }
}

void save_settings(settings_t* settings)
{
  if (!settings)
    return;

  nvs_handle_t nvsHandle;
  esp_err_t ret;

  settings->gnssProcessedMsg = gnss_get_processed_msg();
  settings->gnssRate = gnss_get_rate();
  settings->imuProcessedRpt = imu_get_processed_rpt();
  settings->imuRate = imu_get_rate();
  settings->triggerRate = trigger_get_rate();
  settings->triggerManual = trigger_is_manual();
  settings->triggerDisplay = trigger_is_displayed();
  settings->syncDisplay = sync_is_displayed();
  settings->logLevel = mainLogLevel;

  mynvs_flash_init();
  ret = mynvs_open(&nvsHandle,PROJECT_NAME);
  if (ret == ESP_OK)
    mynvs_save(nvsHandle,"settings",settings,sizeof(*settings));  
  mynvs_close(nvsHandle);
  mynvs_flash_deinit();
  if (ret != ESP_OK)
    ESP_LOGE(TAG,"error saving settings");
}

void pause_ms(uint32_t t)
// pause in millisecond
{
  vTaskDelay(t / portTICK_PERIOD_MS);
}

uint32_t tick_to_ms(TickType_t tck)
// from task ticks to millisecond
{
  return tck * portTICK_PERIOD_MS;
}

TickType_t ms_to_tick(uint32_t ms)
// from millisecond to task ticks
{
  return ms / portTICK_PERIOD_MS;
}

void init_log(uint8_t logLevel)
{
  mainLogLevel = logLevel; 
  esp_log_level_set("*",mainLogLevel);
}

void toggle_log_level(char* tag)
{
  if (mainLogLevel == ESP_LOG_INFO)
    mainLogLevel = ESP_LOG_DEBUG;
  else
    mainLogLevel = ESP_LOG_INFO;
  esp_log_level_set(tag,mainLogLevel);
  ESP_LOGI(TAG,"log level toggled to:%s",
           mainLogLevel == ESP_LOG_INFO ? "INFO" : "DEBUG");
}

uint8_t debug()
{
  return (mainLogLevel >= ESP_LOG_DEBUG);
}

void print_error(esp_err_t rc, const char *file,
                 int line, const char *function, const char *expression)
{                                                                               
    ets_printf("\nERROR: esp_err_t 0x%x", rc);                           
#ifdef CONFIG_ESP_ERR_TO_NAME_LOOKUP                                            
    ets_printf(" (%s)", esp_err_to_name(rc));                                   
#endif //CONFIG_ESP_ERR_TO_NAME_LOOKUP                                          
    ets_printf(" at 0x%08x\n", (intptr_t)__builtin_return_address(0) - 3);      
    if (spi_flash_cache_enabled()) { // strings may be in flash cache           
        ets_printf("\tfile: \"%s\" line %d\n\tfunc: %s\n\texpression: %s\n", file,    
line, function, expression);                                                    
    }                                                                           
}

// This example demonstrates how a human readable table of run time stats
// information is generated from raw data provided by uxTaskGetSystemState().
// The human readable table is written to pcWriteBuffer
// !!!!!!!!!!!!!
// requires make menuconfig COMPONENTS/FREERTOS...
// Enable FreeRTOS trace facility
// Enable FreeRTOS stats formatting functions
// !!!!!!!!!!!!!

TickType_t statsCounter=0;

void my_vTaskGetRunTimeStats()
{
#if defined(CONFIG_FREERTOS_USE_TRACE_FACILITY) && defined(CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS)
  // only once very 10s
  TickType_t c = xTaskGetTickCount();
  if (c - statsCounter < 10000)
    return;
  statsCounter = c;

  char pcWriteBuffer[1024];
  xTaskGetTickCount();
  TaskStatus_t *pxTaskStatusArray;
  volatile UBaseType_t uxArraySize, x;k
  uint32_t ulTotalRunTime, ulStatsAsPercentage;

  // Make sure the write buffer does not contain a string.
  *pcWriteBuffer = 0x00;

  // Take a snapshot of the number of tasks in case it changes while this
  // function is executing.
  uxArraySize = uxTaskGetNumberOfTasks();

  // Allocate a TaskStatus_t structure for each task.  An array could be
  // allocated statically at compile time.
  pxTaskStatusArray = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

  if( pxTaskStatusArray != NULL )
  {
    // Generate raw status information about each task.
    uxArraySize = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, &ulTotalRunTime );

    // For percentage calculations.
    ulTotalRunTime /= 100UL;

    // Avoid divide by zero errors.
    if( ulTotalRunTime > 0 )
    {
      // For each populated position in the pxTaskStatusArray array,
      // format the raw data as human readable ASCII data
      for( x = 0; x < uxArraySize; x++ )
      {
        // What percentage of the total run time has the task used?
        // This will always be rounded down to the nearest integer.
        // ulTotalRunTimeDiv100 has already been divided by 100.
        ulStatsAsPercentage = pxTaskStatusArray[ x ].ulRunTimeCounter / ulTotalRunTime;

        sprintf( pcWriteBuffer+strlen((char*)pcWriteBuffer), "%10s%10u%10u%%%10u\r\n",
                 pxTaskStatusArray[x].pcTaskName,
                 pxTaskStatusArray[ x ].ulRunTimeCounter,
                 ulStatsAsPercentage,
                 pxTaskStatusArray[x].usStackHighWaterMark);
      }
    }

    // The array is no longer needed, free the memory it consumes.
    vPortFree( pxTaskStatusArray );
  }

  printf("=======================================================\n");
  printf("minimal config stat size: %d\n", configMINIMAL_STACK_SIZE);
  printf("  taskName   runTime      usage   unusedStack\n");
  printf("%s\n",pcWriteBuffer);
  printf("=======================================================\n");

#endif
}
