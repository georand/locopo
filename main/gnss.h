// msg types
typedef enum {
  NO_MSG    = 0b00000000,
  UBX_MSG   = 0b00000001,
  RTCM_MSG  = 0b00000010,
  NMEA_MSG  = 0b00000100,
} gnss_msg_t;

void gnss_init(uint16_t processedMsg, uint8_t rate);
esp_err_t gnss_poll_data();
esp_err_t gnss_set_rate(uint8_t freq);
uint8_t gnss_get_rate();
esp_err_t gnss_set_processed_msg(uint8_t msgMask);
uint8_t gnss_get_processed_msg(void);
void gnss_print_format();
bool gnss_is_initialized();
esp_err_t gnss_i2c_read_data();
esp_err_t gnss_i2c_write_data(uint8_t *data, uint16_t length);

