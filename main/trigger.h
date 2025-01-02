typedef enum
{
  trigger_unitialized = 0,
  trigger_waitForPPS = 1,
  trigger_periodic = 2,
  trigger_stopped = 3,
} triggerState_t;

void trigger_init(uint8_t rate, uint8_t manual, uint8_t display);
esp_err_t trigger_send();
void trigger_timer_restart();
void trigger_set_rate(uint8_t rate);
void trigger_set_state(triggerState_t state);
uint8_t trigger_get_rate();
void trigger_toggle_manual();
uint8_t trigger_is_manual();
void trigger_set_display(uint8_t display);
uint8_t trigger_is_displayed();
uint8_t trigger_is_initialized();
