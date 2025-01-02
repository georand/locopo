/*
see examples here:
  https://github.com/espressif/esp-idf/tree/58df1d93b/examples/peripherals/gpio
*/
#include "common.h"

#include "driver/gpio.h"

#include "trigger.h"

#include "sync.h"

//static const char* TAG = __FILE__;
static const char* TAG = "sync";

extern TaskHandle_t sync_imu_task_handle;
extern TaskHandle_t sync_gnss_task_handle;
extern TaskHandle_t trigger_task_handle;

uint8_t syncDisplay = 1;
uint8_t syncInitialized = 0;
TickType_t syncImuTimeRef = 0;
TickType_t syncGnssTimeRef = 0;

static void IRAM_ATTR sync_gnss_isr_handler(void* arg)
// input gnss PPS interrupt handler (from gpio)
{
  uint32_t gpioIdx = (int)arg;
  BaseType_t xHigherPriorityTaskWoken = pdTRUE;

  // Notify the main program task for the resynchronization of the trigger timer
  if (xTaskNotifyFromISR(sync_gnss_task_handle, gpioIdx, eSetValueWithoutOverwrite,
                         &xHigherPriorityTaskWoken) == pdPASS)
  {
    if (xHigherPriorityTaskWoken == pdTRUE)
      portYIELD_FROM_ISR();                                       
  }
}

static void IRAM_ATTR sync_imu_isr_handler(void* arg)
// input imu INT interrupt handler (from gpio)
{
  uint32_t gpioIdx = (int)arg;
  BaseType_t xHigherPriorityTaskWoken = pdTRUE;

  // Notify the main program task for imu  synchronization
  if (xTaskNotifyFromISR(sync_imu_task_handle, gpioIdx, eSetValueWithoutOverwrite,
                         &xHigherPriorityTaskWoken) == pdPASS)
  {
    if (xHigherPriorityTaskWoken == pdTRUE)
      portYIELD_FROM_ISR();                                       
  }
}

void sync_init(uint8_t display)
// initialize the gnss PPS and imu INT  GPIOs and register the interrupt handlers
{
  gpio_config_t gpioConfig;
  
  // GPIO listenning to GNSS PPS 
  gpioConfig.pin_bit_mask = 1ULL<<GNSS_PPS_IN_GPIO;
  gpioConfig.mode = GPIO_MODE_INPUT;
  gpioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
  gpioConfig.pull_down_en = GPIO_PULLDOWN_ENABLE;
  gpioConfig.intr_type = GPIO_INTR_POSEDGE;
  gpio_config(&gpioConfig);

  // GPIO listenning to IMU INT
  gpioConfig.pin_bit_mask = 1ULL<<IMU_INT_IN_GPIO;
  gpioConfig.mode = GPIO_MODE_INPUT;
  gpioConfig.pull_up_en = GPIO_PULLUP_ENABLE;
  gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpioConfig.intr_type = GPIO_INTR_NEGEDGE;
  gpio_config(&gpioConfig);

  //hook isr handler for specific gpio pin and put GNSS_PPS_GPIO in the handler argument
  ESP_ERROR_CHECK(gpio_isr_handler_add(GNSS_PPS_IN_GPIO, sync_gnss_isr_handler,
                                       (void*)GNSS_PPS_IN_GPIO));
  //hook isr handler for specific gpio pin and put IMU_INT_GPIO in the handler argument
  ESP_ERROR_CHECK(gpio_isr_handler_add(IMU_INT_IN_GPIO, sync_imu_isr_handler,
                                       (void*)IMU_INT_IN_GPIO));

  sync_set_display(display);
  ESP_LOGI(TAG, "Initialized. Listening to GNSS_PPS (GPIO %d) and IMU_INT (GPIO %d)",
           GNSS_PPS_IN_GPIO, IMU_INT_IN_GPIO);

  syncInitialized = 1;
}

void sync_synchronize_trigger_timer_with_gnss()
{
  // resynchronize the triggering timer 
  trigger_timer_restart();

  // manually send a trigger in case the first trigger has been skipped during sync
  if (!trigger_is_manual())
    xTaskNotify(trigger_task_handle,0,eIncrement);
  
}

void sync_set_gnss_time_reference()
{
  syncGnssTimeRef = xTaskGetTickCount();

  if (syncDisplay)
    printf("%d;gnss_pps\n",syncGnssTimeRef);
}

TickType_t sync_get_gnss_time_reference()
{
  return syncGnssTimeRef;
}

void sync_set_imu_time_reference()
{
  syncImuTimeRef = xTaskGetTickCount();
}

TickType_t sync_get_imu_time_reference()
{
  return syncImuTimeRef;
}

void sync_set_display(uint8_t display)
{
  syncDisplay = display;
}

uint8_t sync_is_displayed()
{
  return syncDisplay;
}

uint8_t sync_is_initialized()
{
  return syncInitialized;
}
