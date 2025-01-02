
#include "common.h"

#include <driver/gpio.h>

#include "myi2c.h"
#include "gnss.h"
#include "imu.h"
#include "sync.h"
#include "trigger.h"
#include "console.h"
#include "mynvs.h"

static const char* TAG = "main";

TaskHandle_t sync_gnss_task_handle = NULL;
TaskHandle_t sync_imu_task_handle = NULL;
TaskHandle_t trigger_task_handle = NULL;

bool suspendTasks = false;
bool initializationCompleted = false;
settings_t settings;

void console_task(void *pvParameter)
{
  console_init();

  while (true)
  {
    console_poll_data();
    pause_ms(10);
  }
}

void trigger_task(void *pvParameter)
{
  uint32_t triggerCount;

  while (!trigger_is_initialized())
    pause_ms(10);

  while (true)
  {
    // if suspend, wait for resume
    while (suspendTasks)
      pause_ms(10);

    // wait for trigger notification
    xTaskNotifyWait(0,0,&triggerCount,portMAX_DELAY);

    trigger_send();
  }
}

void sync_gnss_task(void *pvParameter)
{
  int cnt = 0;
  uint32_t arg = 0;

  // configure the led GPIO (13)
  gpio_pad_select_gpio(LED_GPIO);
  gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

  while (!sync_is_initialized())
    pause_ms(10);

  while (true)
  {
    // if suspend, wait for resume
    while (suspendTasks)
      pause_ms(10);

    // wait for gnss pps notification
    xTaskNotifyWait(0,0,&arg,portMAX_DELAY);

    // store the imu timeref
    sync_set_gnss_time_reference();

    // synchronize camera trigger with the GNSS
    sync_synchronize_trigger_timer_with_gnss();

    // do some "fancy" led blinking
    gpio_set_level(LED_GPIO, (cnt++)%2);
  }
}

void sync_imu_task(void *pvParameter)
{
  uint32_t arg = 0;

  while (!sync_is_initialized())
    pause_ms(10);

  while (true)
  {
    // if suspend, wait for resume
    while (suspendTasks)
      pause_ms(10);

    // wait for imu int notification
    xTaskNotifyWait(0,0,&arg,portMAX_DELAY);

    // store the imu timeref
    sync_set_imu_time_reference();
  }
}

void imu_task(void *pvParameter)
{
  while (!initializationCompleted)
    pause_ms(10);

  while (true)
  {
    // if suspend, wait for resume
    while (suspendTasks)
      pause_ms(10);

    imu_poll_data();
    pause_ms(10);
  }
}

void gnss_task(void *pvParameter)
{
  while (!initializationCompleted)
    pause_ms(10);

  while (true)
  {
    // if suspend, wait for resume
    while (suspendTasks)
      pause_ms(10);

    gnss_poll_data();
    pause_ms(10);
  }
}

void app_main()
{
  pause_ms(10);

  ESP_LOGI(TAG,"==============================================================");

  // load settings
  load_settings(&settings);

  // some system initialization
  gpio_install_isr_service(0);
  init_log(settings.logLevel);
  myi2c_init();

  // console
  xTaskCreate(&console_task, "console_task", 4*1024, NULL,  2, NULL);

  // trigger (get a task handle before initializing)
  xTaskCreate(&trigger_task, "trigger_task", 4*1024, NULL, 10, &trigger_task_handle);
  trigger_init(settings.triggerRate, settings.triggerManual, settings.triggerDisplay);

  // synchronization (get a task handle before initializing)
  xTaskCreate(&sync_gnss_task, "sync_gnss_task", 2*1024, NULL, 10,
              &sync_gnss_task_handle);
  xTaskCreate(&sync_imu_task, "sync_imu_task", 2*1024, NULL, 10,
              &sync_imu_task_handle);
  sync_init(false); //init sync tasks but inhibit display until full init is completed

  // imu
  xTaskCreate(&imu_task, "imu_task", 4*1024, NULL, 5, NULL);
  imu_init(settings.imuProcessedRpt, settings.imuRate);

  // gnss
  xTaskCreate(&gnss_task, "gnss_task", 4*1024, NULL,  5, NULL);
  gnss_init(settings.gnssProcessedMsg, settings.gnssRate);

  console_help();

  // check initialization
  if (imu_is_initialized() && gnss_is_initialized())
  {
    initializationCompleted = true;
    sync_set_display(settings.syncDisplay);
  }
  else
    ESP_LOGE(TAG,"initalization error GSS:%d IMU:%d",
             imu_is_initialized(), gnss_is_initialized());

  printf("%d;LoCoPo;started\n",xTaskGetTickCount());
}
