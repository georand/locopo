/**
*********************************************************

based on

u-blox ZED-F9P Interface Description - Manual
https://www.u-blox.com/sites/default/files/u-blox_ZED-F9P_InterfaceDescription_%28UBX-18010854%29.pdf

and

PX4 Drone Autopilot library
https://github.com/PX4/GpsDrivers


MON-VER
  Software Version : EXT CORE 1.00 (94e56e)
  Hardware Version : 00190000
  Extensions :
    ROM BASE 0x118B2060
    FWVER=HPG 1.11
    PROTVER=27.10
    MODE=ZED-F9P
    GPS ;GLO ;GAL ;BDS ;
    QZZZ

*********************************************************
**/

// 1 byte alignment
#pragma pack(push, 1)

/* General: Header */
typedef struct {
  uint8_t		sync1;
  uint8_t		sync2;
  uint8_t	  class;
  uint8_t	  id;
  uint16_t	length;
} ubx_header_t;

/* General: Checksum */
typedef struct {
  uint8_t		ck_a;
  uint8_t		ck_b;
} ubx_checksum_t;

/* Rx NAV-PVT (ubx8) */
typedef struct {
  uint32_t	iTOW;		/**< GPS Time of Week [ms] */
  uint16_t	year; 		/**< Year (UTC)*/
  uint8_t		month; 		/**< Month, range 1..12 (UTC) */
  uint8_t		day; 		/**< Day of month, range 1..31 (UTC) */
  uint8_t		hour; 		/**< Hour of day, range 0..23 (UTC) */
  uint8_t		min; 		/**< Minute of hour, range 0..59 (UTC) */
  uint8_t		sec;		/**< Seconds of minute, range 0..60 (UTC) */
  uint8_t		valid; 		/**< Validity flags (see UBX_RX_NAV_PVT_VALID_...) */
  uint32_t	tAcc; 		/**< Time accuracy estimate (UTC) [ns] */
  int32_t		nano;		/**< Fraction of second (UTC) [-1e9...1e9 ns] */
  uint8_t		fixType;	/**< GNSSfix type: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GNSS + dead reckoning, 5 = time only fix */
  uint8_t		flags1;		/**< Fix Status Flags (see UBX_RX_NAV_PVT_FLAGS_...) */
  uint8_t		flags2;		/**< Fix Status Flags (see UBX_RX_NAV_PVT_FLAGS_...) */
  uint8_t		numSV;		/**< Number of SVs used in Nav Solution */
  int32_t		lon;		/**< Longitude [1e-7 deg] */
  int32_t		lat;		/**< Latitude [1e-7 deg] */
  int32_t		height;		/**< Height above ellipsoid [mm] */
  int32_t		hMSL;		/**< Height above mean sea level [mm] */
  uint32_t	hAcc;  		/**< Horizontal accuracy estimate [mm] */
  uint32_t	vAcc;  		/**< Vertical accuracy estimate [mm] */
  int32_t		velN;		/**< NED north velocity [mm/s]*/
  int32_t		velE;		/**< NED east velocity [mm/s]*/
  int32_t		velD;		/**< NED down velocity [mm/s]*/
  int32_t		gSpeed;		/**< Ground Speed (2-D) [mm/s] */
  int32_t		headMot;	/**< Heading of motion (2-D) [1e-5 deg] */
  uint32_t	sAcc;		/**< Speed accuracy estimate [mm/s] */
  uint32_t	headAcc;	/**< Heading accuracy estimate (motion and vehicle) [1e-5 deg] */
  uint16_t	pDOP;		/**< Position DOP [0.01] */
  uint16_t	reserved1;
  uint16_t	reserved2;
  uint16_t	reserved3;
  int32_t		headVeh;	/**< (ubx8+ only) Heading of vehicle (2-D) [1e-5 deg] */
  int16_t		magDec;	/**< (ubx8+ only) Magnetic declination [1e-2 deg] */
  uint16_t		magAcc;	/**< (ubx8+ only) Magnetic declination accuracy [1e-2 deg] */
} ubx_payload_rx_nav_pvt_t;

/* Rx NAV-DOP */
typedef struct {
  uint32_t	iTOW;		/**< GPS Time of Week [ms] */
  uint16_t	gDOP;		/**< Geometric DOP [0.01] */
  uint16_t	pDOP;		/**< Position DOP [0.01] */
  uint16_t	tDOP;		/**< Time DOP [0.01] */
  uint16_t	vDOP;		/**< Vertical DOP [0.01] */
  uint16_t	hDOP;		/**< Horizontal DOP [0.01] */
  uint16_t	nDOP;		/**< Northing DOP [0.01] */
  uint16_t	eDOP;		/**< Easting DOP [0.01] */
} ubx_payload_rx_nav_dop_t;

/* Rx ACK-ACK */
typedef	union {
  uint16_t	msg;
  struct {
    uint8_t	clsID;
    uint8_t	msgID;
  };
} ubx_payload_rx_ack_ack_t;

/* Rx ACK-NAK */
typedef	union {
  uint16_t	msg;
  struct {
    uint8_t	clsID;
    uint8_t	msgID;
  };
} ubx_payload_rx_ack_nak_t;

/* Tx CFG-RATE */
typedef struct {
  uint16_t	measRate;	/**< Measurement Rate, GPS measurements are taken every measRate milliseconds */
  uint16_t	navRate;	/**< Navigation Rate, in number of measurement cycles. This parameter cannot be changed, and must be set to 1 */
  uint16_t	timeRef;	/**< Alignment to reference time: 0 = UTC time, 1 = GPS time */
} ubx_payload_tx_cfg_rate_t;

/* Tx CFG-CFG */
typedef struct {
  uint32_t	clearMask;	/**< Clear settings */
  uint32_t	saveMask;	/**< Save settings */
  uint32_t	loadMask;	/**< Load settings */
  uint8_t		deviceMask; /**< Storage devices to apply this top */
} ubx_payload_tx_cfg_cfg_t;

/* Tx CFG-VALSET (protocol version 27+) */
typedef struct {
  uint8_t     version;	/**< Message version, set to 0 */
  uint8_t     layers;	/**< The layers where the configuration should be applied (@see UBX_CFG_LAYER_*) */
  uint8_t     reserved[2];
} ubx_payload_tx_cfg_valset_t;

/* tx cfg-rst */
typedef struct {
  uint16_t	navBbrMask;
  uint8_t		resetMode;
  uint8_t		reserved1;
} ubx_payload_tx_cfg_rst_t;

#pragma pack(pop)

// reset types
typedef enum {
  hot = 0,
  warm,
  cold,
}ubx_restart_t;

// Decoder state
typedef enum {
  UBX_DECODE_SYNC1 = 0,
  UBX_DECODE_SYNC2,
  UBX_DECODE_CLASS,
  UBX_DECODE_ID,
  UBX_DECODE_LENGTH1,
  UBX_DECODE_LENGTH2,
  UBX_DECODE_PAYLOAD,
  UBX_DECODE_CHKSUM1,
  UBX_DECODE_CHKSUM2,
} ubx_decode_state_t;

// Rx message state
typedef enum {
  UBX_RXMSG_IGNORE = 0,
  UBX_RXMSG_HANDLE,
  UBX_RXMSG_DISABLE,
  UBX_RXMSG_ERROR_LENGTH
} ubx_rxmsg_state_t;

// ACK state
typedef enum {
  UBX_ACK_IDLE = 0,
  UBX_ACK_WAITING,
  UBX_ACK_GOT_ACK,
  UBX_ACK_GOT_NAK
} ubx_ack_state_t;

// General message and payload tx buffer structure
typedef struct {
  uint16_t bufPos;
  uint8_t *buffer;
} ubx_tx_buf_t;

// General message and payload rx structure
typedef struct {
  ubx_decode_state_t decodeState;
  ubx_rxmsg_state_t	state;
  uint16_t msg;
  uint16_t payloadLength;
  ubx_ack_state_t ackState;
  uint8_t	ckA;
  uint8_t	ckB;
  uint16_t ackWaitingMsg;
  uint16_t bufPos;
  uint8_t *buffer;
} ubx_rx_buf_t;

void ubx_init();
void ubx_format_output_str();
char* ubx_get_output_str();
uint16_t ubx_get_output_time_offset_since_pps();
const char* ubx_get_output_format();

// tx functions
int ubx_reset(ubx_restart_t restartType);
uint8_t ubx_set_rate(uint16_t freq);
uint8_t ubx_set_processed_msg(uint16_t processedMsg);

// rx functions
void ubx_decode_reset();
int ubx_decode_data(uint8_t c);

