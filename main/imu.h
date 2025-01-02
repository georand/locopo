#include "imudefs.h"

/// List of all sensor reports that the IMU supports.
typedef enum 
{
  NO_RPT                           = 0b00000000,
  ROTATION_RPT                     = 0b00000001,
  ROTATION_GAME_RPT                = 0b00000010,
  ACCELERATION_LINEAR_RPT          = 0b00000100,
  ACCELERATION_GRAVITY_RPT         = 0b00001000,
  MAGNETIC_FIELD_CALIBRATED_RPT    = 0b00010000,
  MAGNETIC_FIELD_UNCALIBRATED_RPT  = 0b00100000,
  GYROSCOPE_CALIBRATED_RPT         = 0b01000000,
  GYROSCOPE_UNCALIBRATED_RPT       = 0b10000000,
}  imu_report_id_t;


// 1 byte alignment
#pragma pack(push, 1)

typedef struct {
  uint16_t length;
  uint8_t  channel;
  uint8_t  seqNum;
} imu_header_t;

#pragma pack(pop)


void imu_init(uint16_t processedMsg, uint8_t rate);
esp_err_t imu_poll_data();
esp_err_t imu_set_rate(uint8_t rate);
uint8_t imu_get_rate();
esp_err_t imu_set_processed_rpt(uint8_t rptMask);
bool imu_toggle_calibration();
uint8_t imu_get_processed_rpt(void);
void imu_print_format();
uint8_t imu_is_initialized();

