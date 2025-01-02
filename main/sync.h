                        
void sync_init(uint8_t display);
void sync_synchronize_trigger_timer_with_gnss();
void sync_set_display(uint8_t display);
void sync_set_gnss_time_reference();
TickType_t sync_get_gnss_time_reference();
void sync_set_imu_time_reference();
TickType_t sync_get_imu_time_reference();
uint8_t sync_is_displayed();
uint8_t sync_is_initialized();
