#include "nvs.h"

typedef nvs_handle nvs_handle_t;
void mynvs_flash_init();
void mynvs_flash_deinit();
void mynvs_flash_erase();
esp_err_t mynvs_open(nvs_handle_t *handle, const char* namespace);
void mynvs_close(nvs_handle_t handle);
esp_err_t mynvs_erase(nvs_handle_t handle);
esp_err_t mynvs_load(nvs_handle_t handle, const char* key,
                      void* value, const size_t valueSize);
esp_err_t mynvs_save(nvs_handle_t handle, const char* key,
                      void* value, const size_t valueSize);
