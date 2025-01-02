#include "common.h"

#include "myi2c.h"

// I2C defines
#define MY_I2C_FREQUENCY 400000
#define MY_I2C_BUFFER_LENGTH I2C_FIFO_LEN  // I2C hw fifo length = 32
#define MY_I2C_WRITE_BIT I2C_MASTER_WRITE  // I2C master write
#define MY_I2C_READ_BIT I2C_MASTER_READ    // I2C master read
#define MY_I2C_ACK_CHECK_EN 0x1            // I2C master will check ack from slave
#define MY_I2C_ACK_CHECK_DIS 0x0           // I2C master will not check ack from slave
#define MY_I2C_ACK_VAL I2C_MASTER_ACK      // I2C ack value
#define MY_I2C_NACK_VAL I2C_MASTER_NACK    // I2C nack value
#define MY_I2C_LOCK_TIMEOUT 300            // lock timeout (ms)

//static const char* TAG = __FILE__;
static const char* TAG = "i2c";

esp_err_t myi2c_wait_for_unlock(uint16_t timeout);

bool myi2cLocked = false; // avoid i2c collision

void myi2c_init()
{
    i2c_config_t conf =
    {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MY_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = MY_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = MY_I2C_FREQUENCY
    };
    i2c_param_config(MY_I2C_PORT, &conf);
    STOP_ON_ERROR(i2c_driver_install(MY_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGD(TAG, "I2C master mode started on port %d", MY_I2C_PORT);
}

esp_err_t myi2c_write(uint8_t addr, uint8_t* data, int length)
{
  esp_err_t ret;
  i2c_cmd_handle_t cmd;

  if (!addr || data == NULL || !length)
    return ESP_FAIL;

  // wait for other imu i/o to finish
  if (myi2c_wait_for_unlock(MY_I2C_LOCK_TIMEOUT) != ESP_OK)
    return ESP_FAIL;
  myi2cLocked = true;

  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  // request a write command to the slave whose ID is <addr>
  i2c_master_write_byte(cmd, (addr << 1) | MY_I2C_WRITE_BIT, MY_I2C_ACK_CHECK_EN);
  // write data
  i2c_master_write(cmd, data, length, MY_I2C_ACK_CHECK_EN);
  // ubox specific
  //"The number of data bytes must be at least 2 to properly distinguish
  //from the write access to set the address counter in random read accesses."
//  if (length % 2)
//    i2c_master_write_byte(cmd, *data, MY_I2C_ACK_CHECK_EN);
  i2c_master_stop(cmd);
  i2c_set_timeout(MY_I2C_PORT, MY_I2C_MASTER_TOUT_CNUM);
  ret = i2c_master_cmd_begin(MY_I2C_PORT, cmd, MY_I2C_WAIT);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
  {
    myi2cLocked = false;
    return myi2c_print_rsl(ret);
  }
  
  myi2cLocked = false;
  return ESP_OK;
}

esp_err_t myi2c_write_register(uint8_t addr, uint8_t reg, uint8_t* data, int length)
{
  esp_err_t ret;
  i2c_cmd_handle_t cmd;

  if (!addr || data == NULL || !length)
    return ESP_FAIL;

  // wait for other imu i/o to finish
  if (myi2c_wait_for_unlock(MY_I2C_LOCK_TIMEOUT) != ESP_OK)
    return ESP_FAIL;
  myi2cLocked = true;

  // request a write command to the slave whose ID is <addr>
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  // request a write command to the slave whose ID is <addr>
  i2c_master_write_byte(cmd, (addr << 1) | MY_I2C_WRITE_BIT, MY_I2C_ACK_CHECK_EN);
  // specify the slave register to write in
  i2c_master_write_byte(cmd, reg, MY_I2C_ACK_CHECK_EN);
  i2c_set_timeout(MY_I2C_PORT, MY_I2C_MASTER_TOUT_CNUM);
  ret = i2c_master_cmd_begin(MY_I2C_PORT, cmd, MY_I2C_WAIT);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
  {
    myi2cLocked = false;
    return myi2c_print_rsl(ret);
  }

  // write data
  cmd = i2c_cmd_link_create();
  i2c_master_write(cmd, data, length, MY_I2C_ACK_CHECK_EN);
  // ubox specific
  //"The number of data bytes must be at least 2 to properly distinguish
  //from the write access to set the address counter in random read accesses."
  //if (length % 2)
  //  i2c_master_write_byte(cmd, *data, MY_I2C_ACK_CHECK_EN);
  i2c_master_stop(cmd);
  i2c_set_timeout(MY_I2C_PORT, MY_I2C_MASTER_TOUT_CNUM);
  ret = i2c_master_cmd_begin(MY_I2C_PORT, cmd, MY_I2C_WAIT);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
  {
    myi2cLocked = false;
    return myi2c_print_rsl(ret);
  }
  
  myi2cLocked = false;
  return ESP_OK;
}

esp_err_t myi2c_read(uint8_t addr, uint8_t* data, int length, myi2c_signal_t signal)
{
  if (!addr || data == NULL || !length)
    return ESP_FAIL;

  esp_err_t ret;
  i2c_cmd_handle_t cmd;

  // wait for other i2c i/o to finish unless this is a continuation
  if (signal == myi2cStart || signal == myi2cStartNStop)
  {
    if (myi2c_wait_for_unlock(MY_I2C_LOCK_TIMEOUT) != ESP_OK)
      return ESP_FAIL;
    myi2cLocked = true;
  }

  // read the data
  cmd = i2c_cmd_link_create();
  if (signal == myi2cStart || signal == myi2cStartNStop)
  {
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | MY_I2C_READ_BIT, MY_I2C_ACK_CHECK_EN);
  }
  if (signal == myi2cStop || signal == myi2cStartNStop)
  {
    if (length > 1)
      i2c_master_read(cmd, data, length-1, MY_I2C_ACK_VAL);
    i2c_master_read_byte(cmd, data+length-1, MY_I2C_NACK_VAL);
    i2c_master_stop(cmd);
  }
  else if (signal == myi2cStart)
    i2c_master_read(cmd, data, length, MY_I2C_ACK_VAL);
  i2c_set_timeout(MY_I2C_PORT, MY_I2C_MASTER_TOUT_CNUM);
  ret = i2c_master_cmd_begin(MY_I2C_PORT, cmd, MY_I2C_WAIT);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
  {
    myi2cLocked = false;
    return myi2c_print_rsl(ret);
  }
  
  // unlock i2c channel unless this connection is to be continued 
  if (signal == myi2cStop || signal == myi2cStartNStop)
    myi2cLocked = false;
  
  return ESP_OK;
}

esp_err_t myi2c_read_register(uint8_t addr, uint8_t reg, uint8_t* data, int length)
{
  if (!addr || data == NULL || !length)
    return ESP_FAIL;

  esp_err_t ret;
  i2c_cmd_handle_t cmd;

  // wait for other i2c i/o to finish
  if (myi2c_wait_for_unlock(MY_I2C_LOCK_TIMEOUT) != ESP_OK)
    return ESP_FAIL;
  myi2cLocked = true;

  // prepare to read register
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  // request a write command to the slave whose ID is <addr>
  i2c_master_write_byte(cmd, (addr << 1) | MY_I2C_WRITE_BIT, MY_I2C_ACK_CHECK_EN);
  // specify the slave register to be read
  i2c_master_write_byte(cmd, reg, MY_I2C_ACK_CHECK_EN);
  i2c_set_timeout(MY_I2C_PORT, MY_I2C_MASTER_TOUT_CNUM);
  ret = i2c_master_cmd_begin(MY_I2C_PORT, cmd, MY_I2C_WAIT);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
  {
    myi2cLocked = false;
    return myi2c_print_rsl(ret);
  }

  // read the data
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | MY_I2C_READ_BIT, MY_I2C_ACK_CHECK_EN);
  if (data != NULL && length)
  {
    if (length > 1)
      i2c_master_read(cmd, data, length-1, MY_I2C_ACK_VAL);
    i2c_master_read_byte(cmd, data+length-1, MY_I2C_NACK_VAL);
    i2c_master_stop(cmd);
  }
  i2c_set_timeout(MY_I2C_PORT, MY_I2C_MASTER_TOUT_CNUM);
  ret = i2c_master_cmd_begin(MY_I2C_PORT, cmd, MY_I2C_WAIT);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
  {
    myi2cLocked = false;
    return myi2c_print_rsl(ret);
  }
  
  myi2cLocked = false;  
  return ESP_OK;
}

esp_err_t myi2c_wait_for_unlock(uint16_t timeout)
// wait for unlock and lock other i2c i/o
{
  if (myi2cLocked)
  {
    TickType_t tickTimeout = xTaskGetTickCount() +  ms_to_tick(timeout);
    while(myi2cLocked && xTaskGetTickCount() <= tickTimeout)
      pause_ms(1);
  }
  if (myi2cLocked)
    ESP_LOGW(TAG,"LOCK TIMEOUT!!!");
  if (myi2cLocked)
    return ESP_FAIL;
  else
    return ESP_OK;  
}

void myi2c_scan()
{
  uint8_t address;
  printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
  for (int i = 0; i < 128; i += 16) {
    printf("%02x: ", i);
    for (int j = 0; j < 16; j++) {
      fflush(stdout);
      address = i + j;
      i2c_cmd_handle_t cmd = i2c_cmd_link_create();
      i2c_master_start(cmd);
      i2c_master_write_byte(cmd, (address << 1) | MY_I2C_WRITE_BIT,
                            MY_I2C_ACK_CHECK_EN);
      i2c_master_stop(cmd);
      i2c_set_timeout(MY_I2C_PORT, MY_I2C_MASTER_TOUT_CNUM);
      esp_err_t ret = i2c_master_cmd_begin(MY_I2C_PORT, cmd, MY_I2C_WAIT);
      i2c_cmd_link_delete(cmd);
      if (ret == ESP_OK) {
        printf("%02x ", address);
      } else if (ret == ESP_ERR_TIMEOUT) {
        printf("UU ");
      } else {
        printf("-- ");
      }
    }
    printf("\r\n");
  }
}

esp_err_t myi2c_disable()
{
  return i2c_driver_delete(I2C_NUM_0);
}

void myi2c_lock()
{
  myi2cLocked = true;
}

void myi2c_unlock()
{
  myi2cLocked = false;
}

bool myi2c_is_locked()
{
  return myi2cLocked;
}

esp_err_t myi2c_print_rsl(esp_err_t r)
{
  switch (r)
  {
  case ESP_OK:
    ESP_LOGD(TAG, "Transfer success");
    break;
  case ESP_ERR_TIMEOUT: //0x0107
    ESP_LOGD(TAG, "Bus is busy");
    break;
  case ESP_FAIL: //0xffff
      ESP_LOGD(TAG, "Transfer failed");
    break;
  case ESP_ERR_INVALID_ARG: //0x0102
    ESP_LOGD(TAG, "Parameter error");
    break;
  case ESP_ERR_INVALID_STATE: //0x103
    ESP_LOGD(TAG, "Driver not installed");
    break;
  }
  return r;
}

