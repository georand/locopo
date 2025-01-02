/***********************************************************

based on

u-blox ZED-F9P Interface Description - Manual
https://www.u-blox.com/sites/default/files/u-blox_ZED-F9P_InterfaceDescription_%28UBX-18010854%29.pdf

and

PX4 Drone Autopilot library (BSD3 licence)
https://github.com/PX4/GpsDrivers

***********************************************************/

#include "common.h"

#include <string.h>

#include "gnss.h"

#include "ubxdefs.h"
#include "ubx.h"

#define UBX_CONFIG_TIMEOUT	2500		// ms, timeout for waiting ACK
#define UBX_PACKET_TIMEOUT	2		// ms, if now data during this delay assume that full

// tx functions
int ubx_send_message(const uint16_t msg);
void ubx_init_config_valset();
void ubx_config_valset(uint32_t key_id, void* value, uint16_t valueSize);
int ubx_wait_for_ack(const uint16_t msg, const unsigned timeout);
void ubx_calc_checksum(const uint8_t *buffer, const uint16_t length, ubx_checksum_t *checksum);

// rx functions
int ubx_payload_rx_init();
int ubx_payload_rx_add(uint8_t b);
int ubx_payload_rx_add_nav_sat(uint8_t b);
int ubx_payload_rx_add_nav_sv_info(uint8_t b);
int ubx_payload_rx_add_mon_ver(uint8_t b);
int ubx_payload_rx_done();
void ubx_add_byte_to_checksum(const uint8_t b);

//static const char* TAG = __FILE__;
static const char* TAG = "ubx";

//The major datums we want to globally store
typedef struct
{
  uint32_t iTOW;		    // GPS Time of Week [ms]
  uint16_t year;        // year (UTC)
  uint8_t  month;       // mon  (UTC)
  uint8_t  day;         // day  (UTC)
  uint8_t  hour;        // hour (UTC)
  uint8_t  min;         // min  (UTC)
  uint8_t  sec;         // sec  (UTC)
  int32_t	 nano;		    // fraction of second (UTC) [-1e9...1e9 ns]

  uint8_t  fixType;		  // 0 no fix, 1: dead reckoning, 2: 2D, 3: 3D, 4: GNSS+ dead reckoning, 4: time only
  uint8_t  velNedValid;
  uint8_t  numSV;       // Number of sattelites used in Nav solution

  int32_t  lat;		      // Degrees * 1e+7 (more accurate than floats)
  int32_t  lon;		      // Degrees * 1e+7 (more accurate than floats)
  int32_t  height;		  // Height above ellipsoid (mm)
  int32_t  heightMSL;	  // Height above mean sea level (mm)

  int32_t gSpeed;       // Ground 2-D Speed (mm/s)
  int32_t velN;         // NED north velocity (mm/s)
  int32_t velE;         // NED east velocity (mm/s)
  int32_t velD;         // NED down velocity (mm/s)
  int32_t headMot;      // Heading of 2-D motion (deg * 1e+5f)
  uint32_t magDec;      // Magnetic declination (deg * 1e+2)

  uint32_t hAcc;        // Horizontal accuracy estimate (mm)
  uint32_t vAcc;        // Vertical accuracy estimate (mm)
  uint32_t sAcc;        // Speed accuracy estimate (mm/s)
  uint32_t headAcc;     // Heading accuracy estimate (deg * 1e+5)
  uint32_t tAcc;        // UTC time accuracy estimate (ns)
  uint32_t magAcc;      // Magnetic declination accuracy (deg * 1e+2)

  uint16_t gDOP;			  // Geometric DOP  (*1e+2)
  uint16_t pDOP;			  // Position DOP   (*1e+2)
  uint16_t tDOP;			  // Time DOP       (*1e+2)
  uint16_t vDOP;			  // Vertical DOP   (*1e+2)
  uint16_t hDOP;			  // Horizontal DOP (*1e+2)
  uint16_t nDOP;			  // Northing Dop   (*1e+2)
  uint16_t eDOP;			  // Easting DOP    (*1e+2)

  uint8_t  PVT;         // 1 if valid PVT packet
  uint8_t  DOP;         // 1 if valid DOP packet

}ubx_data_t;

ubx_data_t ubxData;
uint8_t ubxRxBuffer[MAX_BUFFER];
uint8_t ubxTxBuffer[MAX_BUFFER];
ubx_rx_buf_t ubxRx;
ubx_tx_buf_t ubxTx;
char ubxOutputStr[MAX_BUFFER];
uint16_t ubxOutputMsSincePPS = 0;
bool ubxOutputAvailable = false;

inline void ubx_formatDecInt(char* strOut, int32_t N, uint32_t dec, char c)
{
  int32_t NInt = N / dec;
  sprintf(strOut,"%d.%03d%c", NInt, N - NInt * dec, c);
}

void ubx_init()
{
  ubxRx.buffer = ubxRxBuffer;
  ubxTx.buffer = ubxTxBuffer;
  memset(&ubxData, 0, sizeof(ubxData));
  ubx_decode_reset();
}

void ubx_format_output_str()
{
  char *s= ubxOutputStr;

  ubxOutputMsSincePPS = ubxData.nano/1000000;

  sprintf(s,"%02d/%02d/%04d;%02d:%02d:%02d%+04d;",
          ubxData.month, ubxData.day, ubxData.year,
          ubxData.hour, ubxData.min, ubxData.sec, ubxOutputMsSincePPS);
  ubx_formatDecInt(s+strlen(s), ubxData.lat, 10000000, ';');
  ubx_formatDecInt(s+strlen(s), ubxData.lon, 10000000, ';');
  ubx_formatDecInt(s+strlen(s), ubxData.heightMSL, 1000, ';');
  ubx_formatDecInt(s+strlen(s), ubxData.hDOP, 100, ';');
  ubx_formatDecInt(s+strlen(s), ubxData.vDOP, 100, ';');
  ubx_formatDecInt(s+strlen(s), ubxData.hAcc, 1000, ';');
  ubx_formatDecInt(s+strlen(s), ubxData.vAcc, 1000, ' ');

//  ESP_LOGD(TAG,"NAV_PVT & NAVDOP received (%+010dns since laste pps)",ubxData.nano);

  // make output available for printing
  ubxOutputAvailable = true;
}

char* ubx_get_output_str()
{
  if (ubxOutputAvailable)
  {
    ubxOutputAvailable = false;
    return ubxOutputStr;
  }
  else
    return NULL;
}

uint16_t ubx_get_output_time_offset_since_pps()
{
  return ubxOutputMsSincePPS;
}

const char* ubx_get_output_format()
{
  return ("\n\t\tm/d/y;h:mn:s+ms;\n\t\tlat(decdeg);lon(decdeg);hMSL(m);\n\t\thDop;vDop;hAcc(m);vAcc(m)");
}

/**********************************************************************************
                             UBX commands
**********************************************************************************/

int ubx_reset(ubx_restart_t restartType)
{
  int ret = ESP_OK;

  memset(ubxTx.buffer, 0, MAX_BUFFER);
  ubxTx.bufPos = sizeof(ubx_header_t);
  ubx_payload_tx_cfg_rst_t *p = (ubx_payload_tx_cfg_rst_t*) (ubxTx.buffer+ubxTx.bufPos);

  switch (restartType)
  {
  case hot:
    p->navBbrMask = UBX_TX_CFG_RST_BBR_MODE_HOT_START;
    break;
  case warm:
    p->navBbrMask = UBX_TX_CFG_RST_BBR_MODE_WARM_START;
    break;
  case cold:
    p->navBbrMask = UBX_TX_CFG_RST_BBR_MODE_COLD_START;
    break;
  default:
    ret = ESP_FAIL;
  }

  p->resetMode = 0; // HW reset immediately

  if (ret == ESP_OK)
    ret = ubx_send_message(UBX_MSG_CFG_RST);

  if (ret != ESP_OK)
    ESP_LOGE(TAG,"unable to reset gnss");

  pause_ms(100);
  return ret;
}

uint8_t ubx_set_rate(uint16_t freq)
{
  int ret = ESP_FAIL;

  if (freq == 0 || freq > 100)
  {
    ESP_LOGW(TAG, "unable to set such a rate");
    return ESP_FAIL;
  }

  // Navigation and Measurement Rate Configuration (CFG-RATE)
  uint16_t meas = 1000/freq; // time between two measurements
  uint16_t nav = 1; // 1 measurement per navigation solution
  uint16_t timeRef = 0; //UTC
  ubx_init_config_valset();
  // Measurement rate, measurements are taken every measRate milliseconds (1000 = 1hz)
  if (meas >= 10)
    ubx_config_valset(UBX_CFG_KEY_RATE_MEAS, &meas, sizeof(meas));
  //Navigation rate, number of measurements used for a navigation solution
  if (nav < 127)
    ubx_config_valset(UBX_CFG_KEY_RATE_NAV, &nav, sizeof(nav));
  //Alignment to reference time: 0=UTC, 1=GPS, 2=GLO, 3=BDS, 4=GAL
  if (timeRef < 5)
    ubx_config_valset(UBX_CFG_KEY_RATE_TIMEREF, &timeRef, sizeof(timeRef));

  ret = ubx_send_message(UBX_MSG_CFG_VALSET);
  pause_ms(100);
  if (ret == ESP_OK)
    ret = ubx_wait_for_ack(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT);
  if (ret != ESP_OK)
    ESP_LOGE(TAG,"unable to configure gps");

  return ret;
}

uint8_t ubx_set_processed_msg(uint16_t processedMsg)
// set the message types (UBX_MSG|NMEA_MSG|RTCM_MSG) to be procesed
// using I2C message rate configuration (CFG-MSGOUT*I2C in the ublox doc)
// ublox doc is ambiguous: looks like the settings must be
// 1 (1 output per solution) or 0 (none)
{
  uint8_t msgRate ;
  int ret = ESP_FAIL;

  ubx_init_config_valset();

  if (processedMsg & UBX_MSG)
    msgRate = 1;
  else
    msgRate = 0;
  // enable NAV_PVT and NAV_DOP messages and disable the rest
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_UBX_NAV_PVT_I2C, &msgRate, sizeof(msgRate));
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_UBX_NAV_DOP_I2C, &msgRate, sizeof(msgRate));
  msgRate = 0;
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_UBX_NAV_SVIN_I2C, &msgRate, sizeof(msgRate));
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_UBX_NAV_SAT_I2C, &msgRate, sizeof(msgRate));

  if (processedMsg & NMEA_MSG)
    msgRate = 1;
  else
    msgRate = 0;
  // enable RMC messages and disable the rest of NMEA messages
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_NMEA_ID_RMC_I2C, &msgRate, sizeof(msgRate));
  msgRate = 0;
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_NMEA_ID_DTM_I2C, &msgRate, sizeof(msgRate));
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_NMEA_ID_GBS_I2C, &msgRate, sizeof(msgRate));
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_NMEA_ID_GGA_I2C, &msgRate, sizeof(msgRate));
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_NMEA_ID_GLL_I2C, &msgRate, sizeof(msgRate));
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_NMEA_ID_GNS_I2C, &msgRate, sizeof(msgRate));
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_NMEA_ID_GRS_I2C, &msgRate, sizeof(msgRate));
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_NMEA_ID_GSA_I2C, &msgRate, sizeof(msgRate));
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_NMEA_ID_GST_I2C, &msgRate, sizeof(msgRate));
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_NMEA_ID_GSV_I2C, &msgRate, sizeof(msgRate));
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_NMEA_ID_VLW_I2C, &msgRate, sizeof(msgRate));
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_NMEA_ID_VTG_I2C, &msgRate, sizeof(msgRate));
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_NMEA_ID_ZDA_I2C, &msgRate, sizeof(msgRate));

  if (processedMsg & RTCM_MSG)
    msgRate = 1;
  else
    msgRate = 0;
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1005_I2C, &msgRate, sizeof(msgRate));
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1074_I2C, &msgRate, sizeof(msgRate));
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1077_I2C, &msgRate, sizeof(msgRate));
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1084_I2C, &msgRate, sizeof(msgRate));
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1087_I2C, &msgRate, sizeof(msgRate));
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1094_I2C, &msgRate, sizeof(msgRate));
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1097_I2C, &msgRate, sizeof(msgRate));
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1124_I2C, &msgRate, sizeof(msgRate));
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1127_I2C, &msgRate, sizeof(msgRate));
  ubx_config_valset(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1230_I2C, &msgRate, sizeof(msgRate));

  ret = ubx_send_message(UBX_MSG_CFG_VALSET);
  pause_ms(100);

  if (!(processedMsg & UBX_MSG)) // no ACK is to be waited for
    return ESP_OK;
  if (ret == ESP_OK)
    ret = ubx_wait_for_ack(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT);
  if (ret != ESP_OK)
    ESP_LOGE(TAG,"unable to configure gps");

  return ret;
}

int ubx_send_message(const uint16_t msg)
{
  ubx_header_t *p1 = (ubx_header_t*) ubxTx.buffer;
  p1->sync1 = UBX_SYNC1; // 0XB5
  p1->sync2 = UBX_SYNC2; // 0x62
  p1->class = msg & 0xff; // Message class
  p1->id = (msg >> 8) & 0xff; // Message ID
  p1->length = ubxTx.bufPos - sizeof(ubx_header_t);

  // compute and store checksum at the end of the packet
  ubx_checksum_t *p2 = (ubx_checksum_t*) (ubxTx.buffer+ubxTx.bufPos);
  ubxTx.bufPos+=sizeof(*p2);
  ubx_calc_checksum(ubxTx.buffer, ubxTx.bufPos, p2);

  if (debug())
  {
    if (msg == UBX_MSG_CFG_VALSET)
      ESP_LOGD(TAG,"Tx CFG-VALSET");
    else if (msg == UBX_MSG_CFG_RST)
      ESP_LOGD(TAG,"Tx CFG-RST");
    else
      ESP_LOGD(TAG,"Tx 0x%02x 0x%02x", p1->class, p1->id);
  }

  // send command
  return (gnss_i2c_write_data(ubxTx.buffer, ubxTx.bufPos));
}

int	// ESP_FAIL = NAK, error or timeout, ESP_OK = ACK
ubx_wait_for_ack(const uint16_t msg, const unsigned timeout)
{
  // if ubx message are not processed then waiting for ack is useless
  if (!(gnss_get_processed_msg() & UBX_MSG))
    return ESP_OK;

  int ret = ESP_FAIL;

  ubxRx.ackState = UBX_ACK_WAITING;
  ubxRx.ackWaitingMsg = msg;	// memorize sent msg class&ID for ACK check

  TickType_t startTime = xTaskGetTickCount();
  while (ubxRx.ackState == UBX_ACK_WAITING
         && xTaskGetTickCount() - startTime < timeout * portTICK_PERIOD_MS)
  {
    gnss_i2c_read_data(); //See if new data is available. Process bytes as they come in.

    if (ubxRx.ackState == UBX_ACK_GOT_ACK)
      ret = ESP_OK;	// ACK received ok
  }
  ubxRx.ackState = UBX_ACK_IDLE;
  return ret;
}

void ubx_init_config_valset()
{
  memset(ubxTx.buffer, 0, MAX_BUFFER);
  ubxTx.bufPos = sizeof(ubx_header_t);

  // CFG_VAL_SET header
  ubx_payload_tx_cfg_valset_t *p = (ubx_payload_tx_cfg_valset_t*) (ubxTx.buffer+ubxTx.bufPos);
  p->version = 0; //Message version, set to 0;
  p->layers = UBX_CFG_LAYER_RAM; //The layers where the config should be applied
  ubxTx.bufPos += sizeof(*p);
}

void ubx_config_valset(uint32_t key_id, void* value, uint16_t valueSize)
{
  if (ubxTx.bufPos + valueSize >= MAX_BUFFER - 1)
  {
    ESP_LOGE(TAG,"ubx buffer overflow");
    return;
  }

  // CFG_VAL_SET configuration data
  memcpy(ubxTx.buffer+ubxTx.bufPos, &key_id, sizeof(key_id)); ubxTx.bufPos+=sizeof(key_id);
  memcpy(ubxTx.buffer+ubxTx.bufPos, value, valueSize); ubxTx.bufPos+=valueSize;
}

void ubx_calc_checksum(const uint8_t *buffer, const uint16_t length, ubx_checksum_t *checksum)
// compute the 8-Bit Fletcher checksum of the packet
// WARNING: the 2 first and 2 last bytes must be ommitted
{
  for (uint16_t i = 2; i < length-2; i++) {
    checksum->ck_a = checksum->ck_a + buffer[i];
    checksum->ck_b = checksum->ck_b + checksum->ck_a;
  }
}

/**********************************************************************************
                             UBX message decoding
**********************************************************************************/

void ubx_decode_reset()
{
  ubxRx.decodeState = UBX_DECODE_SYNC1;
  memset(ubxRx.buffer, 0, MAX_BUFFER);
  ubxRx.bufPos = 0;
  ubxRx.ckA = 0;
  ubxRx.ckB = 0;
}

int	// 0 = decoding, 1 = message handled, 2 = sat info message handled
ubx_decode_data(uint8_t b)
// process received data
{
  int ret = ESP_OK;

  switch (ubxRx.decodeState) {

  /* Expecting Sync1 */
  case UBX_DECODE_SYNC1:
    if (b == UBX_SYNC1) 	// Sync1 found --> expecting Sync2
    {
      //ESP_LOGD(TAG,"A");
      ubxRx.decodeState = UBX_DECODE_SYNC2;
    }
    break;

  /* Expecting Sync2 */
  case UBX_DECODE_SYNC2:
    if (b == UBX_SYNC2) {	// Sync2 found --> expecting Class
      //ESP_LOGD(TAG,"B");
      ubxRx.decodeState = UBX_DECODE_CLASS;
    }
    else // Sync1 not followed by Sync2: reset parser
      ubx_decode_reset();
    break;

  /* Expecting Class */
  case UBX_DECODE_CLASS:
    //ESP_LOGD(TAG,"C");
    ubx_add_byte_to_checksum(b);  // checksum is calculated for everything except Sync and Checksum bytes
    ubxRx.msg = b;
    ubxRx.decodeState = UBX_DECODE_ID;
    break;

  /* Expecting ID */
  case UBX_DECODE_ID:
    //ESP_LOGD(TAG,"D");
    ubx_add_byte_to_checksum(b);
    ubxRx.msg |= b << 8;
    ubxRx.decodeState = UBX_DECODE_LENGTH1;
    break;

  /* Expecting first length byte */
  case UBX_DECODE_LENGTH1:
    //ESP_LOGD(TAG,"E");
    ubx_add_byte_to_checksum(b);
    ubxRx.payloadLength = b;
    ubxRx.decodeState = UBX_DECODE_LENGTH2;
    break;

  /* Expecting second length byte */
  case UBX_DECODE_LENGTH2:
    //ESP_LOGD(TAG,"F");
    ubx_add_byte_to_checksum(b);
    ubxRx.payloadLength |= b << 8;	// calculate payload size
    if (ubx_payload_rx_init() != 0) 	// start payload reception
      // payload will not be handled, discard message
      ubx_decode_reset();
    else
      ubxRx.decodeState = (ubxRx.payloadLength > 0) ? UBX_DECODE_PAYLOAD : UBX_DECODE_CHKSUM1;
    break;

  /* Expecting payload */
  case UBX_DECODE_PAYLOAD:
    ubx_add_byte_to_checksum(b);
    ret = ubx_payload_rx_add(b);		// add a payload byte
    if (ret < 0)
    {
      //ESP_LOGD(TAG,"0x%04x (bad)",SWAP16(ubxRx.msg));
      // payload not handled, discard message
      ubx_decode_reset();
    }
    else if (ret > 0)
    {
      //ESP_LOGD(TAG,"0x%04x",SWAP16(ubxRx.msg));
      // payload complete, expecting checksum
      ubxRx.decodeState = UBX_DECODE_CHKSUM1;
    }
    else
    {
      // expecting more payload, stay in state UBX_DECODE_PAYLOAD
    }
    ret = 0;
    break;

  /* Expecting first checksum byte */
  case UBX_DECODE_CHKSUM1:
    if (ubxRx.ckA != b)
    {
      ESP_LOGD(TAG,"ubx checksum err");
      ubx_decode_reset();
    }
    else
      ubxRx.decodeState = UBX_DECODE_CHKSUM2;
    break;

  /* Expecting second checksum byte */
  case UBX_DECODE_CHKSUM2:
    if (ubxRx.ckB != b)
      ESP_LOGD(TAG,"ubx checksum err");
    else
    {
      // finalize payload processing
      ret = ubx_payload_rx_done();

      // format the received data (if valid) for output
      if (ret && ubxData.PVT && ubxData.DOP)
      {
        ubx_format_output_str();
        memset(&ubxData, 0, sizeof(ubxData));
      }
    }
    ubx_decode_reset();
    break;

  default:
    break;
  }

  return ret;
}

int	// -1 = abort, 0 = continue
ubx_payload_rx_init()
// start payload rx
{
  int ret = 0;

  ubxRx.state = UBX_RXMSG_HANDLE;	// handle by default

  switch (ubxRx.msg) {
  case UBX_MSG_NAV_PVT:
    if (ubxRx.payloadLength != sizeof(ubx_payload_rx_nav_pvt_t)) {
      ubxRx.state = UBX_RXMSG_ERROR_LENGTH;
    }
    break;

  case UBX_MSG_NAV_DOP:
    if (ubxRx.payloadLength != sizeof(ubx_payload_rx_nav_dop_t)) {
      ubxRx.state = UBX_RXMSG_ERROR_LENGTH;
    }
    break;

  case UBX_MSG_ACK_ACK:
    if (ubxRx.payloadLength != sizeof(ubx_payload_rx_ack_ack_t)) {
      ubxRx.state = UBX_RXMSG_ERROR_LENGTH;
    }
    break;

  case UBX_MSG_ACK_NAK:
    if (ubxRx.payloadLength != sizeof(ubx_payload_rx_ack_nak_t)) {
      ubxRx.state = UBX_RXMSG_ERROR_LENGTH;
    }
    break;

  case UBX_MSG_INF_DEBUG:
  case UBX_MSG_INF_ERROR:
  case UBX_MSG_INF_NOTICE:
  case UBX_MSG_INF_WARNING:
    if (ubxRx.bufPos >= MAX_BUFFER-2) {
      ubxRx.bufPos = MAX_BUFFER-2; //avoid buffer overflow
    }
    break;

  default:
    ubxRx.state = UBX_RXMSG_DISABLE;	// disable all other messages
    break;
  }

  switch (ubxRx.state) {
  case UBX_RXMSG_HANDLE:	// handle message
  case UBX_RXMSG_IGNORE:	// ignore message but don't report error
    ret = 0;
    break;

  case UBX_RXMSG_DISABLE:	// disable unexpected messages
    ESP_LOGD(TAG,"ubx msg 0x%04x len %u unexpected",
             SWAP16((unsigned)ubxRx.msg), (unsigned)ubxRx.payloadLength);
    ret = -1;	// return error, abort handling this message
    break;

  case UBX_RXMSG_ERROR_LENGTH:	// error: invalid length
    ESP_LOGW(TAG,"ubx msg 0x%04x invalid len %u",
             SWAP16((unsigned)ubxRx.msg), (unsigned)ubxRx.payloadLength);
    ret = -1;	// return error, abort handling this message
    break;

  default:	// invalid message state
    ESP_LOGW(TAG,"ubx internal err1");
    ret = -1;	// return error, abort handling this message
    break;
  }

  return ret;
}

int	// -1 = error, 0 = ok, 1 = payload completed
ubx_payload_rx_add(uint8_t b)
// add payload rx byte
{
  int ret = 0;

  ubxRx.buffer[ubxRx.bufPos] = b;
  if (++ubxRx.bufPos >= ubxRx.payloadLength) {
    ret = 1;	// payload received completely
  }

  return ret;
}

int	// 0 = no message handled, 1 = message handled, 2 = sat info message handled
ubx_payload_rx_done()
// finish payload rx
{
  int ret = 0;

  // return if no message handled
  if (ubxRx.state != UBX_RXMSG_HANDLE)
    return ret;

  // handle message
  switch (ubxRx.msg)
  {

  case UBX_MSG_INF_DEBUG:
  case UBX_MSG_INF_NOTICE:
  {
      ubxRx.buffer[ubxRx.payloadLength] = 0;
      ESP_LOGD(TAG,"ubx msg: %s", ubxRx.buffer);
  }
  break;

  case UBX_MSG_INF_ERROR:
  case UBX_MSG_INF_WARNING:
  {
      ubxRx.buffer[ubxRx.payloadLength] = 0;
      ESP_LOGW(TAG,"ubx msg: %s", ubxRx.buffer);
  }
  break;

  case UBX_MSG_NAV_PVT:
  {
    //ESP_LOGD(TAG,"Rx NAV-PVT");
    ubx_payload_rx_nav_pvt_t *p = (ubx_payload_rx_nav_pvt_t*)ubxRx.buffer;
    //Check if position fix flag is good
    if ((p->flags1 & UBX_RX_NAV_PVT_FLAGS_GNSSFIXOK) == 1)
    {
      // make sure NAV-PVT and DOP msg are synchronized
      if (ubxData.iTOW != 0 && ubxData.iTOW != p->iTOW)
        memset(&ubxData, 0, sizeof(ubxData));

      ubxData.iTOW	= p->iTOW;

      ubxData.fixType	= p->fixType;

      if (p->flags1 &  UBX_RX_NAV_PVT_FLAGS_DIFFSOLN)
        ubxData.fixType = 4; //DGPS

      uint8_t carr_soln = p->flags1 >> 6;

      if (carr_soln == 1)
      {
        ubxData.fixType = 5; //Float RTK
      }
      else if (carr_soln == 2)
      {
        ubxData.fixType = 6; //Fixed RTK
      }

      ubxData.velNedValid = 0;

    }
    else
    {
      ubxData.fixType = 0;
      ubxData.velNedValid = 0;
    }

    ubxData.numSV	    = p->numSV;

    ubxData.lat		    = p->lat;
    ubxData.lon		    = p->lon;
    ubxData.heightMSL = p->hMSL;
    ubxData.height	  = p->height;

    ubxData.gSpeed	  = p->gSpeed;
    ubxData.velN	    = p->velN;
    ubxData.velE	    = p->velE;
    ubxData.velD	    = p->velD;
    ubxData.headMot	  = p->headMot;
    ubxData.magDec    = p->magDec;

    ubxData.hAcc		  = p->hAcc;
    ubxData.vAcc		  = p->vAcc;
    ubxData.sAcc	    = p->sAcc;
    ubxData.headAcc   = p->headAcc;
    ubxData.tAcc      = p->tAcc;
    ubxData.magAcc    = p->magAcc;

    //Check if time and date fix flags are good
    if ((p->valid & UBX_RX_NAV_PVT_VALID_VALIDDATE)
        && (p->valid & UBX_RX_NAV_PVT_VALID_VALIDTIME)
        && (p->valid & UBX_RX_NAV_PVT_VALID_FULLYRESOLVED))
    {
      ubxData.year	 = p->year;
      ubxData.month  = p->month;
      ubxData.day	   = p->day;
      ubxData.hour	 = p->hour;
      ubxData.min	   = p->min;
      ubxData.sec	   = p->sec;
      ubxData.nano	 = p->nano;
    }

    ubxData.PVT = 1;

    ret = 1;
  }
  break;

  case UBX_MSG_NAV_DOP:
  {
    //ESP_LOGD(TAG,"Rx NAV-DOP");
    ubx_payload_rx_nav_dop_t *p = (ubx_payload_rx_nav_dop_t*)ubxRx.buffer;

    // make sure NAV-PVT and DOP msg are synchronized
    if (ubxData.iTOW != 0 && ubxData.iTOW != p->iTOW)
      memset(&ubxData, 0, sizeof(ubxData));

    ubxData.iTOW	= p->iTOW;

    ubxData.gDOP	= p->gDOP;
    ubxData.pDOP	= p->pDOP;
    ubxData.hDOP	= p->tDOP;
    ubxData.vDOP	= p->vDOP;
    ubxData.hDOP	= p->hDOP;
    ubxData.nDOP	= p->nDOP;
    ubxData.eDOP	= p->eDOP;

    ubxData.DOP = 1;

    ret = 1;
  }
  break;

  case UBX_MSG_ACK_ACK:
  {
    ESP_LOGD(TAG,"Rx ACK-ACK");
    ubx_payload_rx_ack_ack_t *p = (ubx_payload_rx_ack_ack_t*)ubxRx.buffer;
    if ((ubxRx.ackState == UBX_ACK_WAITING) && (p->msg == ubxRx.ackWaitingMsg))
      ubxRx.ackState = UBX_ACK_GOT_ACK;
    ret = 1;
  }
  break;

  case UBX_MSG_ACK_NAK:
  {
    ESP_LOGD(TAG,"Rx ACK-NACK");
    ubx_payload_rx_ack_ack_t *p = (ubx_payload_rx_ack_ack_t*)ubxRx.buffer;
    if ((ubxRx.ackState == UBX_ACK_WAITING) && (p->msg == ubxRx.ackWaitingMsg))
      ubxRx.ackState = UBX_ACK_GOT_NAK;
    ret = 1;
  }
  break;

  default:
    break;
  }

  return ret;
}

void ubx_add_byte_to_checksum(const uint8_t b)
{
  ubxRx.ckA = ubxRx.ckA + b;
  ubxRx.ckB = ubxRx.ckB + ubxRx.ckA;
}
