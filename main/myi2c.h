#include <driver/i2c.h>

typedef enum
{
  myi2cStartNStop,
  myi2cStart,
  myi2cStop,
} myi2c_signal_t;
  
void myi2c_init();
esp_err_t myi2c_write(uint8_t addr, uint8_t* data, int length);
esp_err_t myi2c_write_register(uint8_t addr, uint8_t reg, uint8_t* data, int length);
esp_err_t myi2c_read(uint8_t addr, uint8_t* data, int length,  myi2c_signal_t signal);
esp_err_t myi2c_read_register(uint8_t addr, uint8_t reg, uint8_t* data, int length);
void myi2c_scan();
esp_err_t myi2c_disable();
esp_err_t myi2c_print_rsl(esp_err_t r);
esp_err_t myi2c_wait_for_unlock(uint16_t timeout);
void myi2c_lock();
void myi2c_unlock();
bool myi2c_is_locked();

