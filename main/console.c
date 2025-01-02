#include "common.h"

#include <errno.h>
#include <string.h>
#include <ctype.h>
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"

#include "gnss.h"
#include "imu.h"
#include "sync.h"
#include "trigger.h"
#include "mynvs.h"
#include "console.h"

//static const char* TAG = __FILE__;
static const char* TAG = "";//console";

extern settings_t settings;
extern bool suspendTasks;

void console_init()
{
    /* Disable buffering on stdin */
    setvbuf(stdin, NULL, _IONBF, 0);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_uart_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_uart_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    /* Configure UART. Note that REF_TICK is used so that the baud rate remains
     * correct while APB frequency is changing in light sleep mode.
     */
    const uart_config_t uart_config = {
            .baud_rate = CONFIG_CONSOLE_UART_BAUDRATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .use_ref_tick = true
    };
    ESP_ERROR_CHECK( uart_param_config(CONFIG_CONSOLE_UART_NUM, &uart_config) );

    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK( uart_driver_install(CONFIG_CONSOLE_UART_NUM,
            256, 0, 0, NULL, 0) );

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(CONFIG_CONSOLE_UART_NUM);

    /* Initialize the console */
    esp_console_config_t console_config = {
            .max_cmdline_args = 8,
            .max_cmdline_length = 256,
    };
    ESP_ERROR_CHECK( esp_console_init(&console_config) );

    /* Configure linenoise line completion library */
    /* Enable multiline editing. If not set, long commands will scroll within
     * single line.
     */
//    linenoiseSetMultiLine(1);

    /* Tell linenoise where to get command completions and hints */
//    linenoiseSetCompletionCallback(&esp_console_get_completion);
//    linenoiseSetHintsCallback((linenoiseHintsCallback*) &esp_console_get_hint);

    /* Set command history size */
//    linenoiseHistorySetMaxLen(100);
}

int extractToken(char* token, const char* separators, char** p0)
{ 
  uint8_t l;
  char* p1;
  *p0 = *p0+strspn(*p0,separators);
  p1 = *p0+strcspn(*p0,separators);
  strncpy(token,*p0,p1-*p0);
  l = p1-*p0;
  *p0=p1;
  token[l]='\0';
  return l;
}

void console_poll_data()
{
  char* line = linenoise("");

  // manage help display with suspend 
  if (line == NULL)
  {
    linenoiseFree(line);
    if (suspendTasks)
    {
      suspendTasks = false;
      pause_ms(100);
    }
    else
    {
      suspendTasks = true;
      pause_ms(100);
      console_help();
    }
    return;
  }
  // parse console input 
  char sep[] = " =:;"; 
  char cmd[16];
  char arg[16];
  char *p=line;
  extractToken(cmd,sep,&p);  
  extractToken(arg,sep,&p);

  if (!strcasecmp(cmd,"reset")) 
  {
		esp_restart();
  }
  else if (!strcasecmp(cmd,"lt")) 
  {
    toggle_log_level("*");
    save_settings(&settings);
  }
  else if (!strcasecmp(cmd,"erase"))  
  {
    nvs_handle_t nvsHandle;
    mynvs_open(&nvsHandle,PROJECT_NAME);
    mynvs_erase(nvsHandle);
    mynvs_close(nvsHandle);
    ESP_LOGI(TAG," flash memory reset");
  }
  else if (!strcasecmp(cmd,"gf")) 
  {
    suspendTasks = true;
    pause_ms(100);
    uint8_t freq = strtol(arg, NULL, 10);
    if (!errno)
      gnss_set_rate(freq);
    
    save_settings(&settings);
  }
  else if (!strcasecmp(cmd,"gubx")) 
  {
    suspendTasks = true;
    pause_ms(100);
    uint16_t mask = gnss_get_processed_msg();
    if (mask & UBX_MSG)
      mask &= ~UBX_MSG;
    else
      mask |= UBX_MSG;
    gnss_set_processed_msg(mask);
    save_settings(&settings);
  }
  else if (!strcasecmp(cmd,"gnmea")) 
  {
    suspendTasks = true;
    pause_ms(100);
    uint16_t mask = gnss_get_processed_msg();
    if (mask & NMEA_MSG)
      mask &= ~NMEA_MSG;
    else
      mask |= NMEA_MSG;
    gnss_set_processed_msg(mask);
    save_settings(&settings);
  }
  else if (!strcasecmp(cmd,"grtcm")) 
  {
    suspendTasks = true;
    pause_ms(100);
    uint16_t mask = gnss_get_processed_msg();
    if (mask & RTCM_MSG)
      mask &= ~RTCM_MSG;
    else
      mask |= RTCM_MSG;
    gnss_set_processed_msg(mask);
    save_settings(&settings);
  }
  else if (!strcasecmp(cmd,"if")) 
  {
    suspendTasks = true;
    pause_ms(100);
    uint8_t freq = strtol(arg, NULL, 10);
    if (!errno)
      imu_set_rate(freq);
    
    save_settings(&settings);
  }
  else if (!strcasecmp(cmd,"ical"))
  {
    suspendTasks = true;
    pause_ms(100);
    imu_toggle_calibration();
  }
  else if (!strcasecmp(cmd,"irot")) 
  {
    suspendTasks = true;
    pause_ms(100);
    uint16_t mask = imu_get_processed_rpt();
    if (mask & ROTATION_RPT)
      mask &= ~ROTATION_RPT;
    else
      mask |= ROTATION_RPT;
    imu_set_processed_rpt(mask);
    save_settings(&settings);
  }
  else if (!strcasecmp(cmd,"iacc")) 
  {
    suspendTasks = true;
    pause_ms(100);
    uint16_t mask = imu_get_processed_rpt();
    if (mask & ACCELERATION_LINEAR_RPT)
      mask &= ~ACCELERATION_LINEAR_RPT;
    else
      mask |= ACCELERATION_LINEAR_RPT;
    imu_set_processed_rpt(mask);
    save_settings(&settings);
  }
  else if (!strcasecmp(cmd,"iaccg")) 
  {
    suspendTasks = true;
    pause_ms(100);
    uint16_t mask = imu_get_processed_rpt();
    if (mask & ACCELERATION_GRAVITY_RPT)
      mask &= ~ACCELERATION_GRAVITY_RPT;
    else
      mask |= ACCELERATION_GRAVITY_RPT;
    imu_set_processed_rpt(mask);
    save_settings(&settings);
  }
  else if (!strcasecmp(cmd,"imag")) 
  {
    suspendTasks = true;
    pause_ms(100);
    uint16_t mask = imu_get_processed_rpt();
    if (mask & MAGNETIC_FIELD_CALIBRATED_RPT)
      mask &= ~MAGNETIC_FIELD_CALIBRATED_RPT;
    else
      mask |= MAGNETIC_FIELD_CALIBRATED_RPT;
    imu_set_processed_rpt(mask);
    save_settings(&settings);
  }
  else if (!strcasecmp(cmd,"igyr")) 
  {
    suspendTasks = true;
    pause_ms(100);
    uint16_t mask = imu_get_processed_rpt();
    if (mask & GYROSCOPE_CALIBRATED_RPT)
      mask &= ~GYROSCOPE_CALIBRATED_RPT;
    else
      mask |= GYROSCOPE_CALIBRATED_RPT;
    imu_set_processed_rpt(mask);
    save_settings(&settings);
  }
  else if (!strcasecmp(cmd,"irotg")) 
  {
    suspendTasks = true;
    pause_ms(100);
    uint16_t mask = imu_get_processed_rpt();
    if (mask & ROTATION_GAME_RPT)
      mask &= ~ROTATION_GAME_RPT;
    else
      mask |= ROTATION_GAME_RPT;
    imu_set_processed_rpt(mask);
    save_settings(&settings);
  }
  else if (!strcasecmp(cmd,"imagu")) 
  {
    suspendTasks = true;
    pause_ms(100);
    uint16_t mask = imu_get_processed_rpt();
    if (mask & MAGNETIC_FIELD_UNCALIBRATED_RPT)
      mask &= ~MAGNETIC_FIELD_UNCALIBRATED_RPT;
    else
      mask |= MAGNETIC_FIELD_UNCALIBRATED_RPT;
    imu_set_processed_rpt(mask);
    save_settings(&settings);
  }
  else if (!strcasecmp(cmd,"igyru")) 
  {
    suspendTasks = true;
    pause_ms(100);
    uint16_t mask = imu_get_processed_rpt();
    if (mask & GYROSCOPE_UNCALIBRATED_RPT)
      mask &= ~GYROSCOPE_UNCALIBRATED_RPT;
    else
      mask |= GYROSCOPE_UNCALIBRATED_RPT;
    imu_set_processed_rpt(mask);
    save_settings(&settings);
  }
  else if (!strcasecmp(cmd,"tt")) 
  {
    extern TaskHandle_t trigger_task_handle;
    xTaskNotify(trigger_task_handle,0,eIncrement);
  }
  else if (!strcasecmp(cmd,"tm")) 
  {
    trigger_toggle_manual();
    save_settings(&settings);
  }
  else if (!strcasecmp(cmd,"tf")) 
  {
    uint8_t freq = strtol(arg, NULL, 10);
    if (!errno)
      trigger_set_rate(freq);
    save_settings(&settings);
  }
  else if (!strcasecmp(cmd,"td")) 
  {
    trigger_set_display(trigger_is_displayed() ? 0: 1);
    save_settings(&settings);
  }
  else if (!strcasecmp(cmd,"sd")) 
  {
    sync_set_display(sync_is_displayed() ? 0: 1);
    save_settings(&settings);
  }

  if (suspendTasks)
  {
    suspendTasks = false;
    pause_ms(100);
  }

  linenoiseFree(line);
  pause_ms(10);
}

void console_help()
{
  ESP_LOGI("","===================================================================");
  display_settings(settings);
  ESP_LOGI("","===================================================================");
  ESP_LOGI("","CTRL-S: stop scrolling    CTRL-Q: resume scrolling");  
  ESP_LOGI("","commands");
  ESP_LOGI("","    ENTER:           show this message");
  ESP_LOGI("","    reset+ENTER:     reset the system but not the camera!!!!");
  ESP_LOGI("","    erase+ENTER:     erase setting stored in flash memory");
  ESP_LOGI("","    lt+ENTER:        toggle debug logging");
  ESP_LOGI("","  gnss");  
  ESP_LOGI("","    gf=<freq>+ENTER: change gnss frequence");
  ESP_LOGI("","    gUBX+ENTER:      toggle gnss UBX display");
  ESP_LOGI("","    gNMEA+ENTER:     toggle gnss NMEA display");
  ESP_LOGI("","    gRTCM+ENTER:     toggle gnss RTCM display");
  ESP_LOGI("","  imu");  
  ESP_LOGI("","    if=<freq>+ENTER: change imu frequence");
  ESP_LOGI("","    iCAL+ENTER:      toggle imu calibration");
  ESP_LOGI("","    iROT+ENTER:      toggle imu geomagnetic rotation vector display");
  ESP_LOGI("","    iACC+ENTER:      toggle imu linear acceleration display");
  ESP_LOGI("","    iGYR+ENTER:      toggle imu gyroscope calibrated display");  
  ESP_LOGI("","    iMAG+ENTER:      toggle imu magnetometer field display");  
  ESP_LOGI("","    iROTG+ENTER:     toggle imu game rotation vector display");
  ESP_LOGI("","    iACCG+ENTER:     toggle imu gravity acceleration display");
  ESP_LOGI("","    iGYRU+ENTER:     toggle imu uncalibrated gyroscope display");  
  ESP_LOGI("","    iMAGU+ENTER:     toggle imu uncalibrated magnetometre display");
  ESP_LOGI("","  trigger");  
  ESP_LOGI("","    tt+ENTER:        send one trigger (in manual mode)");
  ESP_LOGI("","    tm+ENTER:        toggle manual/automatic triggering");
  ESP_LOGI("","    tf=<freq>+ENTER: change triggering frequence");
  ESP_LOGI("","    td+ENTER:        toggle trigger display");
  ESP_LOGI("","  sync");  
  ESP_LOGI("","    sd+ENTER:        toggle gnss_pps and imu_int display");
  ESP_LOGI("","GNSS UBX output format");
  ESP_LOGI("","===================================================================");
  gnss_print_format();
  ESP_LOGI("","===================================================================");
  imu_print_format();
  ESP_LOGI("","===================================================================");
}

