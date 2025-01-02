/***********************************************************

based on

https://github.com/hcrest/bno080-driver

and

https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library

***********************************************************/

#include "common.h"

#include <string.h>
#include <math.h>

#include "driver/gpio.h"

#include "sync.h"
#include "myi2c.h"

#include "imudefs.h"
#include "imu.h"

#define IMU_HEADER_SIZE 4
#define IMU_RESET_TIMEOUT 200 // ms  
#define IMU_DEFAULT_TIMEOUT 100 // ms  
#define IMU_STATUS_ARRAY_LEN MAX_SENSOR_REPORTID + 1

// sizes of various sensor data packet elements
#define SIZEOF_BASE_TIMESTAMP 5
#define SIZEOF_TIMESTAMP_REBASE 5
#define SIZEOF_ROTATION_VECTOR 14
#define SIZEOF_ROTATION_GAME_VECTOR 12
#define SIZEOF_ACCELEROMETER 10
#define SIZEOF_ACCELERATION_LINEAR 10
#define SIZEOF_ACCELERATION_GRAVITY 10
#define SIZEOF_GYROSCOPE_CALIBRATED 10
#define SIZEOF_GYROSCOPE_UNCALIBRATED 16
#define SIZEOF_MAGNETIC_FIELD_CALIBRATED 10
#define SIZEOF_MAGNETIC_FIELD_UNCALIBRATED 16
#define SIZEOF_GEOMAGNETIC_ROTATION_VECTOR 14
#define SIZEOF_TAP_DETECTOR 5
#define SIZEOF_STABILITY_REPORT 6
#define SIZEOF_STEP_DETECTOR 8
#define SIZEOF_STEP_COUNTER 12
#define SIZEOF_SIGNIFICANT_MOTION 6
#define SIZEOF_SHAKE_DETECTOR 6

//static const char* TAG = __FILE__;
static const char* TAG = "imu"; 

esp_err_t imu_process_packet();
esp_err_t imu_poll_data();
esp_err_t imu_parse_report_data();
void imu_print_data();
esp_err_t imu_set_feature_command(uint8_t reportID, uint16_t milliseconds,
                               uint32_t specificConfig);
esp_err_t imu_save_calibration_command();
esp_err_t imu_calibration_command(bool acc, bool gyro, bool mag);
esp_err_t imu_print_calibration_state();
esp_err_t imu_send_command(uint8_t command);
esp_err_t imu_wait_for_packet(uint8_t channel, uint8_t reportID, uint16_t timeout);
esp_err_t imu_send_packet(uint8_t channelNumber, uint8_t dataLength);
esp_err_t imu_receive_packet(uint16_t timeout);
esp_err_t imu_get_product_info();
TickType_t imu_compute_report_timestamp(int32_t timestampOffset,
                                        uint16_t reportDelay);
float qToFloat(int16_t fixedPointValue, uint8_t qPoint);
float qToFloat_dword(uint32_t fixedPointValue, int16_t qPoint);
int16_t floatToQ(float qFloat, uint8_t qPoint);
int32_t floatToQ_dword(float qFloat, uint16_t qPoint);

typedef struct
{
  // report ID
  imu_report_id_t reportId;
  // report time stamp (microcontroller tick count)
  TickType_t timeStamp;
  // report accuracy status (0:unreliable, 1:Low, 2:medium, 3:high)
  uint8_t accuracyStatus;
  //Fused rotation using all three sensors (quaternion) -> [i,j,k,real_part]
  float rotationVector[4]; 
  // estimated accuracy of the rotation vector (rad)
  float rotationVectorAccuracy;
  //Fused rotation using accelerometer and gyroscope (quaternion) -> [i,j,k,real_part]
  float rotationGameVector[4]; 
  //acceleration not including the force of gravity (m/s^2) -> [x, y, z]
  float accelerationLinear[3];
  //gravity acceleration (m/s^2) -> [x, y, z]
  float accelerationGravity[3];
  // calibrated Earth's magnetic field levels (uT) -> [x, y, z]
  float magneticFieldCalibrated[3];
  // calibrated Earth's magnetic field levels (uT) -> [x, y, z]
  float magneticFieldUncalibrated[3];
  // magnetic field distorsion (uT) -> [x, y, z]
  float magneticHardIronBias[3];
  // calibrated gyroscope orientation (rad/s) -> [x, y, z]
  float gyroscopeCalibrated[3];
  // uncalibrated gyroscope orientation (rad/s) -> [x, y, z]
  float gyroscopeUncalibrated[3];
  // drift estimated on the gryroscope (rad/s) -> [x, y, z]
  float gyroscopeEstimatedDrift[3];
}imu_data_t;

imu_data_t imuData;
uint8_t imuPacket[MAX_BUFFER];
imu_header_t *imuPktHeader;
uint8_t *imuPktData;
uint8_t imuProcessedRpt = DEFAULT_IMU_RPT;
uint8_t imuRate = DEFAULT_IMU_RATE;
bool imuOngoingCalibration = false;
bool imuInitialized = false;

// tick count corresponding to the last imu interrupt (H_INTN) 
TickType_t imuHostInterrupt;

// Current sequence number for each channel, incremented after transmission. 
uint8_t imuSequenceNumber[6];

// Commands have a seqNum as well. These are inside command packet,
// the header uses its own seqNum per channel
uint8_t imuCommandSequenceNumber;
  
void imu_init(uint16_t processedRpt, uint8_t rate)
{
  esp_err_t ret;  
  TickType_t currentTick = 0;
  TickType_t startTick = xTaskGetTickCount();
  TickType_t timeout = startTick + IMU_RESET_TIMEOUT / portTICK_PERIOD_MS;

  // point to the begining of the data in the packet buffer
  imuPktHeader = (imu_header_t*) imuPacket;
  imuPktData = imuPacket+IMU_HEADER_SIZE;
  
  // config GPIO RESET (INT signal is processed in sync.c)  
  gpio_config_t gpioConfig;  
  gpioConfig.pin_bit_mask = (1ULL<<IMU_RST_OUT_GPIO);
  gpioConfig.mode = GPIO_MODE_OUTPUT;
  gpioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
  gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpioConfig.intr_type = GPIO_PIN_INTR_DISABLE;
  gpio_config(&gpioConfig);

  // reset IMU
  gpio_set_level(IMU_RST_OUT_GPIO, 0);
  pause_ms(2);
  gpio_set_level(IMU_RST_OUT_GPIO, 1);

  // wait for a falling edge on the INT pin to denote startup
  while (currentTick <= startTick && xTaskGetTickCount() < timeout)
    currentTick=sync_get_imu_time_reference();
  if (currentTick <= startTick)
  {
    ESP_LOGE(TAG, "reset timed out, no answer on INT PIN (GPIO %d)",
             IMU_INT_IN_GPIO);
    STOP_ON_ERROR(ESP_FAIL);
    return;
  }

  ESP_LOGI(TAG,"reset done");
  pause_ms(100);

	// after startup, the BNO80 will send several packets
  // let's flush the stack of awaiting packets 
  ESP_LOGI(TAG,"flushing incoming packets");
  while (imu_receive_packet(timeout) == ESP_OK)
  {
    pause_ms(1);
  }

	// Next, officially tell it to initialize, and wait for a successful Initialize Response
  memset(imuPacket, 0, MAX_BUFFER);
	imu_send_command(COMMAND_INITIALIZE);
  ret = imu_wait_for_packet(CHANNEL_CONTROL, SHTP_REPORT_COMMAND_RESPONSE,
                            IMU_DEFAULT_TIMEOUT);
	if(ret != ESP_OK || imuPktData[2] != COMMAND_INITIALIZE || imuPktData[5] != 0)
	{
		ESP_LOGE(TAG,"initialization failed");
    STOP_ON_ERROR(ESP_FAIL);
	}  
  pause_ms(100);

  // set processed reports and sensing rate
  imuRate = rate;
  imu_set_processed_rpt(processedRpt);  
//  imu_set_processed_rpt(ROTATION_GAME_RPT | MAGNETIC_FIELD_CALIBRATED_RPT | MAGNETIC_FIELD_UNCALIBRATED_RPT);
  pause_ms(100);

  ESP_LOGI(TAG, "initialized");
  
  memset(&imuData, 0, sizeof(imuData));
  memset(imuPacket, 0, MAX_BUFFER);
  imuInitialized = 1;
}

esp_err_t imu_poll_data()
// wait for signal and process all the available packets (stop on error)
{
  esp_err_t ret = ESP_OK;

  // wait for packet
  while (gpio_get_level(IMU_INT_IN_GPIO) != 0)
    pause_ms(1);

  // before reading, store the H_INTN for later synchronization
  imuHostInterrupt = sync_get_imu_time_reference();

  // read the packet 
  ret = imu_receive_packet(IMU_DEFAULT_TIMEOUT);
  if (ret != ESP_OK)
    return ret;

  // if this looks like valid report(s), parse the report data
  if( (imuPktHeader->channel == CHANNEL_REPORTS
       || imuPktHeader->channel == CHANNEL_WAKE_REPORTS)
      && imuPktData[0] == SHTP_REPORT_BASE_TIMESTAMP )
    imu_parse_report_data();
  /*
  else
  {

    for (int i=0; i<20; i++)
      {
        if (i==4) printf("   ");
        printf("0x%02X ",imuPacket[i]);
      }
    printf("\n");
  }
  */
  
	return ret;
}

esp_err_t imu_parse_report_data()
{
  uint8_t *data = imuPktData;
  uint8_t reportNum = 0;
	size_t currReportOffset = 0;
  int32_t timestampOffset = 0;
  uint16_t reportDelay = 0;
  uint16_t val[6];
  
	// every sensor data report first contains a base timestamp offset
  // between the host interrupt (HINT) and the packet transmission (see SH2-Ref 7.2.1)
  timestampOffset = - *((uint32_t*)(data+1)); 

	currReportOffset += SIZEOF_BASE_TIMESTAMP;

	while(currReportOffset < imuPktHeader->length - IMU_HEADER_SIZE)
	{
    // clear data
    memset(&imuData,0,sizeof(imuData));

    if(currReportOffset >= MAX_BUFFER - IMU_HEADER_SIZE )
		{
		  ESP_LOGE(TAG,"sensor report longer than packet buffer!");
			return ESP_FAIL;
		}

		reportNum = data[currReportOffset];
    //    printf("num:0x%02x offset:%d\n",reportNum, currReportOffset);
    
    if (reportNum != SENSOR_REPORTID_TIMESTAMP_REBASE)
		{      
      // lots of sensor reports use 3 16-bit numbers stored in bytes 4 through 9
      // some use 3 more numbers, some not. compute them anyway
      val[0] = (uint16_t)data[currReportOffset + 5] << 8
        | data[currReportOffset + 4];
      val[1] = (uint16_t)data[currReportOffset + 7] << 8
        | data[currReportOffset + 6];
      val[2] = (uint16_t)data[currReportOffset + 9] << 8
        | data[currReportOffset + 8];
      val[3] = (uint16_t)data[currReportOffset + 11] << 8
        | data[currReportOffset + 10];
      val[4] = (uint16_t)data[currReportOffset + 13] << 8
        | data[currReportOffset + 12];
      val[5] = (uint16_t)data[currReportOffset + 15] << 8
        | data[currReportOffset + 14];
    
			// set status from byte 2 (see SH-2 Ref 6.5.1)
			imuData.accuracyStatus = data[currReportOffset + 2] & 0b11;

      // get report delay 6.5.1 SH2-Ref specifies a "report delay" to be added
		  reportDelay = (uint16_t) (((data[2] & 0xFC) << 6) + data[3]); 
		}

		switch (reportNum)
		{
    case SENSOR_REPORTID_TIMESTAMP_REBASE:
      //printf("TIMESTAMP_REBASE\n");

      // update delta since last HINT signal (SH2-Ref 7.2.1)
      timestampOffset += *((int32_t*)(data+currReportOffset+1))/10;
    
      currReportOffset += SIZEOF_TIMESTAMP_REBASE;
      break;
      
    case SENSOR_REPORTID_ROTATION_VECTOR:
      imuData.reportId = ROTATION_RPT;
      imuData.rotationVector[0]=qToFloat(val[0], ROTATION_Q_POINT);
      imuData.rotationVector[1]=qToFloat(val[1], ROTATION_Q_POINT);
      imuData.rotationVector[2]=qToFloat(val[2], ROTATION_Q_POINT);
      //real part
      imuData.rotationVector[3]=qToFloat(val[3], ROTATION_Q_POINT);
      imuData.rotationVectorAccuracy = qToFloat(val[4], ROTATION_ACCURACY_Q_POINT);
      currReportOffset += SIZEOF_ROTATION_VECTOR;
      break;

    case SENSOR_REPORTID_ROTATION_GAME_VECTOR:
      imuData.reportId = ROTATION_GAME_RPT;
      imuData.rotationGameVector[0]=qToFloat(val[0], ROTATION_Q_POINT);
      imuData.rotationGameVector[1]=qToFloat(val[1], ROTATION_Q_POINT);
      imuData.rotationGameVector[2]=qToFloat(val[2], ROTATION_Q_POINT);
      // real part
      imuData.rotationGameVector[3]=qToFloat(val[3], ROTATION_Q_POINT);
      currReportOffset += SIZEOF_ROTATION_GAME_VECTOR;
      break;

    case SENSOR_REPORTID_ACCELERATION_LINEAR:
      imuData.reportId = ACCELERATION_LINEAR_RPT;
      imuData.accelerationLinear[0]=qToFloat(val[0], ACCELEROMETER_Q_POINT);
      imuData.accelerationLinear[1]=qToFloat(val[1], ACCELEROMETER_Q_POINT);
      imuData.accelerationLinear[2]=qToFloat(val[2], ACCELEROMETER_Q_POINT);      
      currReportOffset += SIZEOF_ACCELERATION_LINEAR;
      break;
      
    case SENSOR_REPORTID_ACCELERATION_GRAVITY:      
      imuData.reportId = ACCELERATION_GRAVITY_RPT;
      imuData.accelerationGravity[0]=qToFloat(val[0], ACCELEROMETER_Q_POINT);
      imuData.accelerationGravity[1]=qToFloat(val[1], ACCELEROMETER_Q_POINT);
      imuData.accelerationGravity[2]=qToFloat(val[2], ACCELEROMETER_Q_POINT);      
      currReportOffset += SIZEOF_ACCELERATION_GRAVITY;
      break;
      
    case SENSOR_REPORTID_MAGNETIC_FIELD_CALIBRATED:      
      imuData.reportId = MAGNETIC_FIELD_CALIBRATED_RPT;
      imuData.magneticFieldCalibrated[0]=qToFloat(val[0], MAGNETOMETER_Q_POINT);
      imuData.magneticFieldCalibrated[1]=qToFloat(val[1], MAGNETOMETER_Q_POINT);
      imuData.magneticFieldCalibrated[2]=qToFloat(val[2], MAGNETOMETER_Q_POINT);      
      currReportOffset += SIZEOF_MAGNETIC_FIELD_CALIBRATED;
      break;
      
    case SENSOR_REPORTID_MAGNETIC_FIELD_UNCALIBRATED:      
      imuData.reportId = MAGNETIC_FIELD_UNCALIBRATED_RPT;
      imuData.magneticFieldUncalibrated[0]=qToFloat(val[0], MAGNETOMETER_Q_POINT);
      imuData.magneticFieldUncalibrated[1]=qToFloat(val[1], MAGNETOMETER_Q_POINT);
      imuData.magneticFieldUncalibrated[2]=qToFloat(val[2], MAGNETOMETER_Q_POINT);
      imuData.magneticHardIronBias[0]=qToFloat(val[3], MAGNETOMETER_Q_POINT);
      imuData.magneticHardIronBias[1]=qToFloat(val[4], MAGNETOMETER_Q_POINT);
      imuData.magneticHardIronBias[2]=qToFloat(val[5], MAGNETOMETER_Q_POINT);
      currReportOffset += SIZEOF_MAGNETIC_FIELD_UNCALIBRATED;
      break;
      
		case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
      imuData.reportId = GYROSCOPE_CALIBRATED_RPT;
      imuData.gyroscopeCalibrated[0]=qToFloat(val[0], GYRO_Q_POINT);
      imuData.gyroscopeCalibrated[1]=qToFloat(val[1], GYRO_Q_POINT);
      imuData.gyroscopeCalibrated[2]=qToFloat(val[2], GYRO_Q_POINT);      
      currReportOffset += SIZEOF_GYROSCOPE_CALIBRATED;
      break;

		case SENSOR_REPORTID_GYROSCOPE_UNCALIBRATED:
      imuData.reportId = GYROSCOPE_UNCALIBRATED_RPT;
      imuData.gyroscopeUncalibrated[0]=qToFloat(val[0], GYRO_Q_POINT);
      imuData.gyroscopeUncalibrated[1]=qToFloat(val[1], GYRO_Q_POINT);
      imuData.gyroscopeUncalibrated[2]=qToFloat(val[2], GYRO_Q_POINT);
      imuData.gyroscopeEstimatedDrift[0]=qToFloat(val[3], GYRO_Q_POINT);
      imuData.gyroscopeEstimatedDrift[1]=qToFloat(val[4], GYRO_Q_POINT);
      imuData.gyroscopeEstimatedDrift[2]=qToFloat(val[5], GYRO_Q_POINT);
      currReportOffset += SIZEOF_GYROSCOPE_UNCALIBRATED;
      break;

    default:
      ESP_LOGE(TAG,"unrecognized or not yet implemented report ID: %20x",
               data[currReportOffset]);
		}
    
    if (reportNum != SENSOR_REPORTID_TIMESTAMP_REBASE)
		{
      // get the tick count corresponding to the time at which the report was acquired 
      imuData.timeStamp = imu_compute_report_timestamp(timestampOffset,
                                                       reportDelay);      
    }

    // print report data
    imu_print_data();
	}
  return ESP_OK;
}

void imu_print_format()
{
  ESP_LOGI("imu output format","  \n\t\tROT(quaternion);accuracyStatus;qX;qY;qZ;qW;accuracy\n\t\tACC(m/s^2);accuracyStatus;X;Y;Z\n\t\tGYRO(rad/s);accuracyStatus;X;Y;Z\n\t\tMAG(uTesla);accuracy;X;Y;Z\n\t\twhere accuracyStatus in [0(worst); 3(best)]");
}

void imu_print_data()
{
  if (imuData.reportId == ROTATION_RPT)
  {
    printf("%d;ROTATION;%d;%f;%f;%f;%f\n",imuData.timeStamp,
           imuData.accuracyStatus, imuData.rotationVector[0],
           imuData.rotationVector[1], imuData.rotationVector[2],
           imuData.rotationVector[3]);
  }
  if (imuData.reportId == ROTATION_GAME_RPT)
  {
    printf("%d;ROTATION_GAME;%d;%f;%f;%f;%f;%f\n",imuData.timeStamp,
           imuData.accuracyStatus, imuData.rotationGameVector[0],
           imuData.rotationGameVector[1], imuData.rotationGameVector[2],
           imuData.rotationGameVector[3], imuData.rotationVectorAccuracy);
  }
  else if (imuData.reportId == ACCELERATION_LINEAR_RPT)
  {
    printf("%d;ACCELERATION;%d;%f;%f;%f\n",imuData.timeStamp,
           imuData.accuracyStatus,imuData.accelerationLinear[0],
           imuData.accelerationLinear[1], imuData.accelerationLinear[2]);
  }
  else if (imuData.reportId == ACCELERATION_GRAVITY_RPT)
  {
    printf("%d;ACCELERATION_GRAVITY;%d;%f;%f;%f\n",imuData.timeStamp,
           imuData.accuracyStatus,imuData.accelerationGravity[0],
           imuData.accelerationGravity[1],imuData.accelerationGravity[2]);
  }
  else if (imuData.reportId == MAGNETIC_FIELD_CALIBRATED_RPT)
  {
    printf("%d;MAGNETOMETER;%d;%f;%f;%f\n",imuData.timeStamp,
           imuData.accuracyStatus, imuData.magneticFieldCalibrated[0],
           imuData.magneticFieldCalibrated[1], imuData.magneticFieldCalibrated[2]);
  }
  else if (imuData.reportId == MAGNETIC_FIELD_UNCALIBRATED_RPT)
  {
    printf("%d;MAGNETOMETER_UNCALIBRATED;%d;%f;%f;%f;%f;%f;%f\n",
           imuData.timeStamp, imuData.accuracyStatus,
           imuData.magneticFieldUncalibrated[0],imuData.magneticFieldUncalibrated[1],
           imuData.magneticFieldUncalibrated[2],imuData.magneticHardIronBias[0],
           imuData.magneticHardIronBias[1],imuData.magneticHardIronBias[2]);
  }
  else if (imuData.reportId == GYROSCOPE_CALIBRATED_RPT)
  {
    printf("%d;GYROSCOPE;%d;%f;%f;%f\n",imuData.timeStamp,
           imuData.accuracyStatus, imuData.gyroscopeCalibrated[0],
           imuData.gyroscopeCalibrated[1], imuData.gyroscopeCalibrated[2]);
  }
  else if (imuData.reportId == GYROSCOPE_UNCALIBRATED_RPT)
  {
    printf("%d;GYROSCOPE_UNCALIBRATED;%d;%f;%f;%f;%f;%f;%f\n",
           imuData.timeStamp, imuData.accuracyStatus,
           imuData.gyroscopeUncalibrated[0], imuData.gyroscopeUncalibrated[1],
           imuData.gyroscopeUncalibrated[2], imuData.gyroscopeEstimatedDrift[0],
           imuData.gyroscopeEstimatedDrift[1], imuData.gyroscopeEstimatedDrift[2]);
  }
}

esp_err_t imu_set_rate(uint8_t rate)
// set the frequency for the imu sensing
{
  uint16_t interval = 1000/rate;
  
  if (imuProcessedRpt & ROTATION_RPT)
    imu_set_feature_command(SENSOR_REPORTID_ROTATION_VECTOR, interval, 0);
  else
    imu_set_feature_command(SENSOR_REPORTID_ROTATION_VECTOR, 0, 0);
  
  if (imuProcessedRpt & ROTATION_GAME_RPT)
    imu_set_feature_command(SENSOR_REPORTID_ROTATION_GAME_VECTOR, interval, 0);
  else
    imu_set_feature_command(SENSOR_REPORTID_ROTATION_GAME_VECTOR, 0, 0);
  
  if (imuProcessedRpt & ACCELERATION_LINEAR_RPT)
    imu_set_feature_command(SENSOR_REPORTID_ACCELERATION_LINEAR, interval, 0);
  else
    imu_set_feature_command(SENSOR_REPORTID_ACCELERATION_LINEAR, 0, 0);
  
  if (imuProcessedRpt & ACCELERATION_GRAVITY_RPT)
    imu_set_feature_command(SENSOR_REPORTID_ACCELERATION_GRAVITY, interval, 0);
  else
    imu_set_feature_command(SENSOR_REPORTID_ACCELERATION_GRAVITY, 0, 0);
  
  if (imuProcessedRpt & MAGNETIC_FIELD_CALIBRATED_RPT)
    imu_set_feature_command(SENSOR_REPORTID_MAGNETIC_FIELD_CALIBRATED, interval, 0);
  else
    imu_set_feature_command(SENSOR_REPORTID_MAGNETIC_FIELD_CALIBRATED, 0, 0);
  
  if (imuProcessedRpt & MAGNETIC_FIELD_UNCALIBRATED_RPT)
    imu_set_feature_command(SENSOR_REPORTID_MAGNETIC_FIELD_UNCALIBRATED, interval, 0);
  else
    imu_set_feature_command(SENSOR_REPORTID_MAGNETIC_FIELD_UNCALIBRATED, 0, 0);
  
  if (imuProcessedRpt & GYROSCOPE_CALIBRATED_RPT)
    imu_set_feature_command(SENSOR_REPORTID_GYROSCOPE_CALIBRATED, interval, 0);
  else
    imu_set_feature_command(SENSOR_REPORTID_GYROSCOPE_CALIBRATED, 0, 0);
  
  if (imuProcessedRpt & GYROSCOPE_UNCALIBRATED_RPT)
    imu_set_feature_command(SENSOR_REPORTID_GYROSCOPE_UNCALIBRATED, interval, 0);
  else
    imu_set_feature_command(SENSOR_REPORTID_GYROSCOPE_UNCALIBRATED, 0, 0);
  
  imuRate = rate;
  
  ESP_LOGI(TAG,"rate: %dHz", rate);
  
  return ESP_OK;
}

uint8_t imu_get_rate()
{
  return imuRate;
}

esp_err_t imu_set_processed_rpt(uint8_t rptMask)
{
  imuProcessedRpt = rptMask;
  
  imu_set_rate(imuRate);
  
  ESP_LOGI(TAG,"ROTATION_RPT:%d, ACCELERATION_LINEAR_RPT:%d, GYROSCOPE_CALIBRATED_RPT:%d, MAGNETOMETER_CALIBRATED_RPT:%d, ROTATION_GAME_RPT:%d, ACCELERATION_GRAVITY:%d, GYROSCOPE_UNCALIBRATED_RPT:%d, MAGNETOMETER_UNCALIBRATED_RPT:%d ", 
           (imuProcessedRpt & ROTATION_RPT)  ? 1 : 0,
           (imuProcessedRpt & ACCELERATION_LINEAR_RPT) ? 1 : 0,
           (imuProcessedRpt & GYROSCOPE_CALIBRATED_RPT) ? 1 : 0,
           (imuProcessedRpt & MAGNETIC_FIELD_CALIBRATED_RPT) ? 1 : 0,
           (imuProcessedRpt & ROTATION_GAME_RPT) ? 1 : 0,
           (imuProcessedRpt & ACCELERATION_GRAVITY_RPT) ? 1 : 0,
           (imuProcessedRpt & GYROSCOPE_UNCALIBRATED_RPT) ? 1 : 0,
           (imuProcessedRpt & MAGNETIC_FIELD_UNCALIBRATED_RPT) ? 1 : 0
    );
  
  return ESP_OK; 
}

uint8_t imu_get_processed_rpt(void)
// get the mask for the currently processed imu report 
// (ROTATION | ACCELERATION_LINEAR | GRAVITY_ACCELERATION | MAGNETIC_FIELD)
{
  return imuProcessedRpt;
}

bool imu_toggle_calibration()
// toggle on/off clibration and return calibration status
{
  if (imuOngoingCalibration)
    imu_calibration_command(false, false, false);
  else
    imu_calibration_command(true, true, true);
  
  imu_print_calibration_state();
  
  return imuOngoingCalibration;
}

esp_err_t imu_calibration_command(bool acc, bool gyro, bool mag)
// start/stop Motion Engine Calibration (SH-2 Ref 6.4.6)
// by default the accelerometer and magnetometer calibration are enabled
{
  esp_err_t ret;
  
  // format the command (0:stop 1:start) and send the command
  memset(imuPacket, 0, MAX_BUFFER);
  imuPktData[3]  = acc; // acceleration
  imuPktData[4]  = gyro; // gyroscope
  imuPktData[5]  = mag; // magnetometer
  imuPktData[6]  = 0; // configure ME Calibration (0) or get ME Calibration (1)
  imuPktData[7]  = 0; // ??? planar acel ???
  imuPktData[8]  = 0; // ??? on Table Cal Enable ??? 
  imuPktData[9]  = 0; // reserved
  imuPktData[10] = 0; // reserved
  imuPktData[11] = 0; // reserved  
	ret = imu_send_command(COMMAND_ME_CALIBRATE);
	if(ret != ESP_OK)
    return ESP_FAIL;

  // verify that calibration is active 
  memset(imuPacket, 0, MAX_BUFFER);
  imuPktData[6]=1;  // get ME Calibration
	imu_send_command(COMMAND_ME_CALIBRATE);
  ret = imu_wait_for_packet(CHANNEL_CONTROL, SHTP_REPORT_COMMAND_RESPONSE,
                            IMU_DEFAULT_TIMEOUT);
	if(ret != ESP_OK || imuPktData[5] != 0
     || imuPktData[6] != acc || imuPktData[7] != gyro || imuPktData[8] != mag)
	{
		ESP_LOGE(TAG,"failing to start/stop calibration ");
    return ESP_FAIL;
	}  
  if (acc || gyro || mag) 
    imuOngoingCalibration = true;
  else
    imuOngoingCalibration = false;

  return ESP_OK;  
}

esp_err_t imu_print_calibration_state()
// show calibration state according to SH2-Ref 6.4.6.3
{
  esp_err_t ret;
  
  // verify that calibration is active 
  memset(imuPacket, 0, MAX_BUFFER);
  imuPktData[6]=1;  // get ME Calibration
	imu_send_command(COMMAND_ME_CALIBRATE);
  ret = imu_wait_for_packet(CHANNEL_CONTROL, SHTP_REPORT_COMMAND_RESPONSE,
                            IMU_DEFAULT_TIMEOUT);
  if (ret != ESP_OK)
  {
		ESP_LOGE(TAG,"unable to get calibration status ");
    return ESP_FAIL;
  }
  
  ESP_LOGI(TAG,"calibration status: accel:%d gyro:%d mag:%d",
           imuPktData[6],imuPktData[7],imuPktData[8]);

  return ESP_OK;
}

esp_err_t imu_save_calibration_command()
// save calibration (SH-2 Ref 6.4.5)
{
  memset(imuPacket, 0, MAX_BUFFER);
	return(imu_send_command(COMMAND_SAVE_DCD));
}

//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
//Also sets the specific config word. Useful for personal activity classifier
esp_err_t imu_set_feature_command(uint8_t reportID, uint16_t milliseconds,
                               uint32_t specificConfig)
{
	const uint32_t batchMicros = 0;
  uint32_t microseconds = 1000 * milliseconds;
  
	imuPktData[0]  = SHTP_REPORT_SET_FEATURE_COMMAND; 
	imuPktData[1]  = reportID; //Feature Report ID. 
	imuPktData[2]  = 0; //Feature flags
	imuPktData[3]  = 0; //Change sensitivity (LSB)
	imuPktData[4]  = 0; //Change sensitivity (MSB)
	imuPktData[5]  = (microseconds >> 0) & 0xFF; //Report interval (LSB) in microseconds. 
	imuPktData[6]  = (microseconds >> 8) & 0xFF; //Report interval
	imuPktData[7]  = (microseconds >> 16) & 0xFF; //Report interval
	imuPktData[8]  = (microseconds >> 24) & 0xFF; //Report interval (MSB)
	imuPktData[9]  = (batchMicros >> 0) & 0xFF;  //Batch Interval (LSB)
	imuPktData[10] = (batchMicros >> 8) & 0xFF; //Batch Interval
	imuPktData[11] = (batchMicros >> 16) & 0xFF;//Batch Interval
	imuPktData[12] = (batchMicros >> 24) & 0xFF;//Batch Interval (MSB)
	imuPktData[13] = (specificConfig >> 0) & 0xFF; //Sensor-specific config (LSB)
	imuPktData[14] = (specificConfig >> 8) & 0xFF; //Sensor-specific config
	imuPktData[15] = (specificConfig >> 16) & 0xFF; //Sensor-specific config
	imuPktData[16] = (specificConfig >> 24) & 0xFF; //Sensor-specific config (MSB)

	//Transmit packet on channel 2, 17 bytes
	return imu_send_packet(CHANNEL_CONTROL, 17);
}

esp_err_t imu_send_command(uint8_t command)
//Send a command to the IMU (see 6.3.8 of SH-2 Ref)
{
	imuPktData[0] = SHTP_REPORT_COMMAND_REQUEST; //Command Request
	imuPktData[1] = imuCommandSequenceNumber++; //Increments automatically each function call
	imuPktData[2] = command; //Command

	//Transmit packet on channel 2, 12 bytes
	return imu_send_packet(CHANNEL_CONTROL, 12);
}

esp_err_t imu_wait_for_packet(uint8_t channel, uint8_t reportID, uint16_t timeout)
{
  TickType_t tickTimeout = xTaskGetTickCount() + timeout / portTICK_PERIOD_MS;

  while(xTaskGetTickCount() <= tickTimeout)
	{
    if (imu_receive_packet(timeout) != ESP_OK)
    {
      ESP_LOGD(TAG,"read error while waiting for packet");
      return ESP_FAIL;
    }
    else if (channel != imuPktHeader->channel || reportID != imuPktData[0])
    {
      ESP_LOGD(TAG,"wrong packet received : %02x<>%02x %02x<>%02x",
               channel,imuPktHeader->channel,reportID,imuPktData[0]);
      return ESP_FAIL;
    }
    else
      return ESP_OK;
	}

	ESP_LOGD(TAG,"Packet wait timeout");
	return ESP_FAIL;
}

esp_err_t imu_send_packet(uint8_t channelNumber, uint8_t dataLength)
{
  imuPktHeader->length = dataLength + IMU_HEADER_SIZE; //Add four bytes for the header
  imuPktHeader->channel = channelNumber;
  imuPktHeader->seqNum = imuSequenceNumber[channelNumber]++;

  esp_err_t ret = myi2c_write(IMU_I2C_ADDR, imuPacket, imuPktHeader->length);

  return ret;
}
  
esp_err_t imu_receive_packet(uint16_t timeout)
// check HINT to see if there is any new data available
// and read the content
{
  esp_err_t ret;
  TickType_t tickTimeout = xTaskGetTickCount() + ms_to_tick(timeout);

  // wait for interupt signaling available data
	while(gpio_get_level(IMU_INT_IN_GPIO) != 0)
	{
		if(xTaskGetTickCount() > tickTimeout)
		{
			ESP_LOGD(TAG,"HINT wait timeout");
			return ESP_FAIL;
		}
	}

  // open the i2c connection and read the 4 bytes header
  ret = myi2c_read(IMU_I2C_ADDR, (uint8_t*) imuPktHeader, IMU_HEADER_SIZE, myi2cStart);
  if (ret != ESP_OK || (imuPktHeader->length == 0xFFFF))
  {
  	// 0XFFFF length mean invalid packet according to BNO080 datasheet section 1.4.1
    ESP_LOGE(TAG,"0x%02x - reading packet header ",ret);
    return ESP_FAIL;
  }

	// clear the MSbit which indicates if this package is a continuation of the last.
  // don't really know what this means... 
	imuPktHeader->length &= ~(1 << 15);

  // if packet length is null, leave after unlocking i2c 
  if (!imuPktHeader->length)
  {
    myi2c_unlock();
    return ESP_OK;
  }

  uint16_t byteToRead = MIN(imuPktHeader->length, MAX_BUFFER); 
  // read the packet data and close the connection
  ret = myi2c_read(IMU_I2C_ADDR, (uint8_t*)imuPktData,
                   byteToRead, myi2cStop);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG,"0x%02x - reading packet data",ret);
    return ESP_FAIL;
  }
  
  /*
  for (int i=0; i<20; i++)
  {
    if (i==4) printf("   ");
      printf("0x%02X ",imuPacket[i]);
  }
  printf("\n");
  */
  
  return ESP_OK;
}

uint8_t imu_is_initialized()
{
  return imuInitialized;
}

esp_err_t imu_get_product_info()
// get product model and version.
{
  esp_err_t ret;
  
  memset(imuPacket, 0, MAX_BUFFER);
  imuPktData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
	imu_send_packet(CHANNEL_CONTROL, 2);

 ret = imu_wait_for_packet(CHANNEL_CONTROL, SHTP_REPORT_PRODUCT_ID_RESPONSE, 5);

  if (ret != ESP_OK || imuPktData[0] != SHTP_REPORT_PRODUCT_ID_RESPONSE)
	{
		ESP_LOGE(TAG,"Bad response from product ID command.\n");
    return ESP_FAIL;
  }
  uint8_t *majorSoftwareVersion = (uint8_t*)(imuPktData+2);
  uint8_t *minorSoftwareVersion = (uint8_t*)(imuPktData+3);
  uint16_t *patchSoftwareVersion = (uint16_t*)(imuPktData+12);
  uint32_t *partNumber = (uint32_t*)(imuPktData+4);
  uint32_t *buildNumber = (uint32_t*)(imuPktData+8);

  printf("BNO080 reports as SW version %hhu.%hhu.%hu, build %u, part no. %u\n",
         *majorSoftwareVersion, *minorSoftwareVersion, *patchSoftwareVersion,
         *buildNumber, *partNumber);
  return ESP_OK;
}

// Produce the tick count timestamp timestamp of the current  sensor event
TickType_t imu_compute_report_timestamp(int32_t timestampOffset,
                                        uint16_t reportDelay)      
// given a  HINT timestamp (see datasheet 1.4.5.3 and SH2-Ref 7.2)
// 7.2.1 SH2-Ref specifies a "base timestamp" to be removed from HINT 
// 7.2.2 SH2-Ref a "rebase timestamp" to be added
// 6.5.1 SH2-Ref a "report delay" to be added
// all these in 100 microseconds  
{
  // timestampOffset and reportDelay are in 100microsecond
  // so we convert it to ms (+ 5 for rounding)
  TickType_t offset = ms_to_tick((timestampOffset + reportDelay + 5) / 10);

  return imuHostInterrupt + offset ;
}

//Given a register value and a Q point, convert to float
//See https://en.wikipedia.org/wiki/Q_(number_format)
float qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
	float qFloat = fixedPointValue;
	qFloat *= pow(2, qPoint * -1);
	return (qFloat);
}

float qToFloat_dword(uint32_t fixedPointValue, int16_t qPoint)
{
	float qFloat = fixedPointValue;
	qFloat *= pow(2, qPoint * -1);
	return (qFloat);
}

//Given a floating point value and a Q point, convert to Q
//See https://en.wikipedia.org/wiki/Q_(number_format)
int16_t floatToQ(float qFloat, uint8_t qPoint)
{
	int16_t qVal = qFloat * pow(2, qPoint);
	return qVal;
}

int32_t floatToQ_dword(float qFloat, uint16_t qPoint)
{
	int32_t qVal = qFloat * pow(2, qPoint);
	return qVal;
}

