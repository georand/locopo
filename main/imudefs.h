// Channels
#define CHANNEL_COMMAND 0
#define CHANNEL_EXECUTABLE 1
#define CHANNEL_CONTROL 2
#define CHANNEL_REPORTS 3
#define CHANNEL_WAKE_REPORTS 4
#define CHANNEL_GYRO 5

// Report IDs on the command channel.
// Unlike the other constants, these come from the Sensor Hub Transport Protocol datasheet, section 5.1
#define COMMAND_REPORTID_ADVERTISEMENT 0x0
#define COMMAND_REPORTID_ERRORLIST 0x1

//All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_FRS_WRITE_RESPONSE 0xF5
#define SHTP_REPORT_FRS_WRITE_DATA 0xF6
#define SHTP_REPORT_FRS_WRITE_REQUEST 0xF7
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD
#define SHTP_REPORT_GET_FEATURE_RESPONSE 0xFC

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_TIMESTAMP_REBASE 0xFA
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE_CALIBRATED 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD_CALIBRATED 0x03
#define SENSOR_REPORTID_ACCELERATION_LINEAR 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_ACCELERATION_GRAVITY 0x06
#define SENSOR_REPORTID_GYROSCOPE_UNCALIBRATED 0x07
#define SENSOR_REPORTID_ROTATION_GAME_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_MAGNETIC_FIELD_UNCALIBRATED 0x0F

// Q points for various sensor data elements
#define ACCELEROMETER_Q_POINT 8 // for accelerometer based data
#define GYRO_Q_POINT 9 // for gyroscope data
#define MAGNETOMETER_Q_POINT 4 // for magnetometer data
#define ROTATION_Q_POINT 14 // for rotation data
#define ROTATION_ACCURACY_Q_POINT 12 // for rotation accuracy data
#define POWER_Q_POINT 10 // for power information in the metadata
#define ORIENTATION_QUAT_Q_POINT 14 // for the set orientation command
#define FRS_ORIENTATION_Q_POINT 30 // for the sensor orientation FRS record

// Report IDs on the Executable channel
// See Figure 1-27 in the BNO080 datasheet
#define EXECUTABLE_REPORTID_RESET 0x1

//Record IDs from SH-2 figure 28
//These are used to read and set various configuration options
#define FRS_RECORDID_SERIAL_NUMBER 0x4B4B
#define FRS_RECORDID_SYSTEM_ORIENTATION 0x2D3E

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 0x01
#define COMMAND_COUNTER 0x02
#define COMMAND_TARE 0x03
#define COMMAND_INITIALIZE 0x04
#define COMMAND_SAVE_DCD 0x06
#define COMMAND_ME_CALIBRATE 0x07
#define COMMAND_DCD_PERIOD_SAVE 0x09
#define COMMAND_OSCILLATOR 0x0A
#define COMMAND_CLEAR_DCD 0x0B
#define COMMAND_UNSOLICITED_INITIALIZE 0x84



