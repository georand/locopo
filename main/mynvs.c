/*
  from:
   https://github.com/espressif/esp-idf/tree/master/examples/storage/nvs_rw_value
   and
   https://github.com/espressif/esp-idf/tree/master/examples/storage/nvs_rw_blob
*/

#include "common.h"

#include "nvs_flash.h"

#include "mynvs.h"

//static const char* TAG = __FILE__;
static const char* TAG = "nvs";

void mynvs_flash_init()
// Initialize Non-Volatile Storage (NVS)
{
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    // NVS partition was truncated and needs to be erased before retrying nvs_flash_init
    ESP_LOGE(TAG,"Error initiliazing  NVS -> reformat");
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
}

void mynvs_flash_deinit()
{
  nvs_flash_deinit();
}

void mynvs_flash_erase()
// erase the entire flash memory  
{
  ESP_ERROR_CHECK(nvs_flash_erase());  
}

esp_err_t mynvs_open(nvs_handle_t *handle, const char* namespace)
// open a NVS namespace
{
  esp_err_t ret;
  ret = nvs_open(namespace, NVS_READWRITE, handle);
  if (ret != ESP_OK) 
    ESP_LOGE(TAG,"Error (%s) opening NVS handle", esp_err_to_name(ret));
  return ret;
}

void mynvs_close(nvs_handle_t handle)
// close a NVS namespace
{
  nvs_close(handle);
}

esp_err_t mynvs_erase(nvs_handle_t handle)
// erase all the key in a NVS namespace
{
  esp_err_t ret;
  ret = nvs_erase_all(handle);
  if (ret != ESP_OK) 
    ESP_LOGE(TAG,"Error (%s) erasing NVS handle", esp_err_to_name(ret));
  return ret;
}

esp_err_t mynvs_load(nvs_handle_t handle, const char* key,
                     void* value, const size_t valueSize)
// read a blob from nvs.
// handle: the handle to the NVS (receive during openning)
// key: the key string identifying the blob
// value: the memory chunk where the blob will be copied
// valueSize: the allocated size of the memory chunk 
{
  esp_err_t ret;
  size_t blobSize;

  // check that blob exists and get the blob size 
  ret = nvs_get_blob(handle, key, NULL, &blobSize);            
  if (ret != ESP_OK)
  {
    ESP_LOGD(TAG,"Unable to read NVS  (%s)", esp_err_to_name(ret));
    return ret;
  }
  if (valueSize != blobSize)
  {
    ESP_LOGD(TAG,"Unable to read NVS: %dB expected, %dB received)",
             valueSize, blobSize);
    return ESP_FAIL;
  }
  // read the blob
  return nvs_get_blob(handle, key, value, &blobSize);
}

esp_err_t mynvs_save(nvs_handle_t handle, const char* key,
                     void* value, const size_t valueSize)
// write a blob from nvs.
// handle: the handle to the NVS (receive during openning)
// key: the key string identifying the blob
// value: the memory chunk to be saved in NVS
// valueSize: the allocated size of the memory chunk 
{
  esp_err_t ret;
  
  // write
  ret = nvs_set_blob(handle, key, value, valueSize);    
  if (ret != ESP_OK)
  {
    ESP_LOGD(TAG,"Unable to write NVS handle (%s)\n", esp_err_to_name(ret));
    return ret;
  }

  // commit
  if (ret == ESP_OK)
    ret = nvs_commit(handle);
  if (ret != ESP_OK)
    ESP_LOGD(TAG,"Unable to commit NVS write operations (%s)\n",
             esp_err_to_name(ret));
  
  return ret;
}
