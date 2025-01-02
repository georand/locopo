/*
see examples here:
  https://github.com/espressif/esp-idf/tree/39f090a4f/examples/peripherals/timer_group
*/

#include "common.h"

#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp_timer.h"
#include "trigger.h"

#define GPIO_OUTPUT_PIN_SEL  (1ULL<<TRIGGER_OUT_GPIO)
#define ESP_INTR_FLAG_DEFAULT 0
#define TIMER_DIVIDER 500  //  Hardware timer clock divider
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_RELOAD 1 // auto reload

//static const char* TAG = __FILE__;
static const char* TAG = "trigger";

extern TaskHandle_t trigger_task_handle;

uint8_t triggerRate = DEFAULT_TRIGGER_RATE;
esp_timer_handle_t triggerTimerHandle;
triggerState_t triggerState;
uint32_t triggerDuration;
TickType_t triggerPeriodTickCount;
TickType_t triggerLastTriggerTick = 0;
uint32_t triggerCount = 0;
uint8_t triggerCountPerSecond = 0;
uint8_t triggerLock = 0;
uint8_t triggerDisplay = 1;
uint8_t triggerInitialized = 0;

void trigger_timer_callback(void* arg)
// notify trigger task
{
  xTaskNotify(trigger_task_handle,0,eIncrement);
}

void trigger_init(uint8_t rate, uint8_t manual, uint8_t display)
// initialize a timer and its interrupt handler and configure the trigger gpio
{
  // configure the trigger GPIO
  gpio_config_t gpioConfig;
  gpioConfig.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  gpioConfig.mode = GPIO_MODE_OUTPUT;
  gpioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
  gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpioConfig.intr_type = GPIO_PIN_INTR_DISABLE;
  gpio_config(&gpioConfig);

  triggerState = trigger_waitForPPS;

  // set trigger rate and GPIO trigger duration
  trigger_set_rate(rate);

  // create the timer but do no start it
  const esp_timer_create_args_t timer_args = {
    .callback = &trigger_timer_callback,
    .name = "trigger"
    };
  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &triggerTimerHandle));

  // set manual mode
  //if (!trigger_is_manual())
  //  trigger_toggle_manual();

  trigger_set_display(display);

  ESP_LOGI(TAG, "Initialized. Waiting for PPS before triggering on GPIO %d at %dHz",
           TRIGGER_OUT_GPIO, triggerRate);

  triggerInitialized = 1;
}

esp_err_t trigger_send()
// send trigger on gpio
{
  TickType_t currentTick = xTaskGetTickCount();

  // check valid conditions and  avoid trigger collision (when triggering manually)
  if (triggerState == trigger_unitialized || triggerLock)
    return ESP_FAIL;

  // do not send trigger if the last one  was too close in time
  // (cheap fix to timer resynchronization problem)
  if (currentTick - triggerLastTriggerTick < triggerPeriodTickCount * 3/4)
    return ESP_FAIL;

  // lock triggering
  triggerLock = 1;

  // send trigger on GPIO
  gpio_set_level(TRIGGER_OUT_GPIO, 1);
  if (triggerDisplay)
    printf("%d;trigger;%03d;%d\n", xTaskGetTickCount(),
           triggerCountPerSecond, triggerCount);
  pause_ms(triggerDuration);
  gpio_set_level(TRIGGER_OUT_GPIO, 0);

  // update counters
  triggerCount++;
  triggerCountPerSecond = (triggerCountPerSecond+1) % triggerRate;
  triggerLastTriggerTick = currentTick;

  // unlock trigerring
  triggerLock = 0;
  return ESP_OK;
}

void trigger_timer_restart()
// resynchronize the timer used for triggering
{
  if (triggerState != trigger_unitialized && triggerState != trigger_stopped)
  {
    if (triggerState == trigger_periodic)
      ESP_ERROR_CHECK(esp_timer_stop(triggerTimerHandle));
    // start periodic timer with period in microsecond
    ESP_ERROR_CHECK(esp_timer_start_periodic(triggerTimerHandle,
                                             1000000/triggerRate));
    triggerState = trigger_periodic;
  }
}

void trigger_set_rate(uint8_t rate)
// set trigger rate and GPIO trigger duration
{
  if (rate == 0)
  {
    ESP_LOGE(TAG,"invalid trigger frequency");
    return;
  }
  triggerRate = rate;
  triggerDuration = 1000/(2*triggerRate);
  triggerPeriodTickCount  = 1000/triggerRate/portTICK_PERIOD_MS;
  trigger_set_state(trigger_waitForPPS);
  ESP_LOGI(TAG, "rate: %dHz", triggerRate);
}

uint8_t trigger_get_rate()
{
  return triggerRate;
}

TickType_t trigger_get_period_tick_count()
{
  return triggerPeriodTickCount;
}

void trigger_set_state(triggerState_t state)
// swicth triggering between trigger_stopped, trigger_waitForPPS and trigger_periodic
{
  if (triggerState == trigger_unitialized)
    ESP_ERROR_CHECK(ESP_FAIL);

  switch (state)
  {
  case trigger_waitForPPS:
    if (triggerState == trigger_periodic)
      ESP_ERROR_CHECK(esp_timer_stop(triggerTimerHandle));
    ESP_LOGI(TAG,"waiting for a pps to start periodic triggering");
    triggerState = trigger_waitForPPS;
    break;
  case trigger_periodic:
    if (triggerState != trigger_waitForPPS)
      ESP_ERROR_CHECK(ESP_FAIL);
    triggerState = trigger_periodic;
    // start periodic timer with period in microsecond
    ESP_ERROR_CHECK(esp_timer_start_periodic(triggerTimerHandle,
                                             1000000/triggerRate));
    break;
  case trigger_stopped:
    if (triggerState == trigger_periodic)
      ESP_ERROR_CHECK(esp_timer_stop(triggerTimerHandle));
    triggerState = trigger_stopped;
    ESP_LOGI(TAG,"switching to manual triggering");
    break;
  default:
    ESP_ERROR_CHECK(ESP_FAIL);
  }
}

void trigger_toggle_manual()
{
  if (triggerState == trigger_stopped)
    trigger_set_state(trigger_waitForPPS);
  else
    trigger_set_state(trigger_stopped);
}

uint8_t trigger_is_manual()
{
  return (triggerState == trigger_stopped ? 1 : 0);
}

void trigger_set_display(uint8_t display)
{
  triggerDisplay = display;
}

uint8_t trigger_is_displayed()
{
  return triggerDisplay;
}

uint8_t trigger_is_initialized()
{
  return triggerInitialized;
}
