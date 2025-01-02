#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

/* Message Classes */
#define UBX_CLASS_NAV		0x01
#define UBX_CLASS_INF		0x04
#define UBX_CLASS_ACK		0x05
#define UBX_CLASS_CFG		0x06
#define UBX_CLASS_MON		0x0A
#define UBX_CLASS_RTCM3	0xF5

/* Message IDs */
#define UBX_ID_NAV_POSLLH	    0x02
#define UBX_ID_NAV_STATUS		  0x03
#define UBX_ID_NAV_DOP		    0x04
#define UBX_ID_NAV_PVT		    0x07
#define UBX_ID_NAV_VELECF     0x11
#define UBX_ID_NAV_VELNED	    0x12
#define UBX_ID_NAV_HPPOSECEF	0x13
#define UBX_ID_NAV_TIMEUTC	  0x21
#define UBX_ID_NAV_SAT		    0x35
#define UBX_ID_NAV_SVIN  	    0x3B
#define UBX_ID_NAV_RELPOSNED  0x3C
#define UBX_ID_INF_DEBUG  	  0x04
#define UBX_ID_INF_ERROR  	  0x00
#define UBX_ID_INF_NOTICE  	  0x02
#define UBX_ID_INF_WARNING 	  0x01
#define UBX_ID_ACK_NAK		0x00
#define UBX_ID_ACK_ACK		0x01
#define UBX_ID_CFG_PRT		0x00 // deprecated in protocol version >= 27 -> use CFG_VALSET
#define UBX_ID_CFG_MSG		0x01 // deprecated in protocol version >= 27 -> use CFG_VALSET
#define UBX_ID_CFG_RATE	  0x08 // deprecated in protocol version >= 27 -> use CFG_VALSET
#define UBX_ID_CFG_CFG		0x09 // deprecated in protocol version >= 27 -> use CFG_VALSET
#define UBX_ID_CFG_NAV5	  0x24 // deprecated in protocol version >= 27 -> use CFG_VALSET
#define UBX_ID_CFG_RST	  0x04
#define UBX_ID_CFG_SBAS	  0x16
#define UBX_ID_CFG_TMODE3	0x71 // deprecated in protocol version >= 27 -> use CFG_VALSET
#define UBX_ID_CFG_VALSET	0x8A
#define UBX_ID_CFG_VALGET	0x8B
#define UBX_ID_CFG_VALDEL	0x8C
#define UBX_ID_MON_VER		0x04
#define UBX_ID_MON_HW		  0x09 // deprecated in protocol version >= 27 -> use MON_RF
#define UBX_ID_MON_RF		  0x38

/* UBX ID for RTCM3 output messages */
/* Minimal messages for RTK: 1005, 1077 + (1087 or 1127) */
/* Reduced message size using MSM4: 1005, 1074 + (1084 or 1124)  */
#define UBX_ID_RTCM3_1005	0x05	/**< Stationary RTK reference station ARP */
#define UBX_ID_RTCM3_1074	0x4A	/**< GPS MSM4 */
#define UBX_ID_RTCM3_1077	0x4D	/**< GPS MSM7 */
#define UBX_ID_RTCM3_1084	0x54	/**< GLONASS MSM4 */
#define UBX_ID_RTCM3_1087	0x57	/**< GLONASS MSM7 */
#define UBX_ID_RTCM3_1094	0x5E	/**< Galileo MSM4 */
#define UBX_ID_RTCM3_1097	0x61	/**< Galileo MSM7 */
#define UBX_ID_RTCM3_1124	0x7C	/**< BeiDou MSM4 */
#define UBX_ID_RTCM3_1127	0x7F	/**< BeiDou MSM7 */
#define UBX_ID_RTCM3_1230	0xE6	/**< GLONASS code-phase biases */
#define UBX_ID_RTCM3_4072	0xFE	/**< Reference station PVT (u-blox proprietary RTCM Message) - Used for moving baseline */


/* Message Classes & IDs */
#define UBX_MSG_NAV_POSLLH	  ((UBX_CLASS_NAV) | UBX_ID_NAV_POSLLH << 8)
#define UBX_MSG_NAV_SOL		    ((UBX_CLASS_NAV) | UBX_ID_NAV_SOL << 8)
#define UBX_MSG_NAV_DOP		    ((UBX_CLASS_NAV) | UBX_ID_NAV_DOP << 8)
#define UBX_MSG_NAV_PVT		    ((UBX_CLASS_NAV) | UBX_ID_NAV_PVT << 8)
#define UBX_MSG_NAV_VELNED	  ((UBX_CLASS_NAV) | UBX_ID_NAV_VELNED << 8)
#define UBX_MSG_NAV_TIMEUTC	  ((UBX_CLASS_NAV) | UBX_ID_NAV_TIMEUTC << 8)
#define UBX_MSG_NAV_SAT	      ((UBX_CLASS_NAV) | UBX_ID_NAV_SAT << 8)
#define UBX_MSG_NAV_SVIN	    ((UBX_CLASS_NAV) | UBX_ID_NAV_SVIN << 8)
#define UBX_MSG_NAV_RELPOSNED	((UBX_CLASS_NAV) | UBX_ID_NAV_RELPOSNED << 8)
#define UBX_MSG_INF_DEBUG	    ((UBX_CLASS_INF) | UBX_ID_INF_DEBUG << 8)
#define UBX_MSG_INF_ERROR	    ((UBX_CLASS_INF) | UBX_ID_INF_ERROR << 8)
#define UBX_MSG_INF_NOTICE	  ((UBX_CLASS_INF) | UBX_ID_INF_NOTICE << 8)
#define UBX_MSG_INF_WARNING	  ((UBX_CLASS_INF) | UBX_ID_INF_WARNING << 8)
#define UBX_MSG_ACK_NAK		    ((UBX_CLASS_ACK) | UBX_ID_ACK_NAK << 8)
#define UBX_MSG_ACK_ACK		    ((UBX_CLASS_ACK) | UBX_ID_ACK_ACK << 8)
#define UBX_MSG_CFG_PRT		    ((UBX_CLASS_CFG) | UBX_ID_CFG_PRT << 8)
#define UBX_MSG_CFG_MSG		    ((UBX_CLASS_CFG) | UBX_ID_CFG_MSG << 8)
#define UBX_MSG_CFG_RATE	    ((UBX_CLASS_CFG) | UBX_ID_CFG_RATE << 8)
#define UBX_MSG_CFG_CFG		    ((UBX_CLASS_CFG) | UBX_ID_CFG_CFG << 8)
#define UBX_MSG_CFG_NAV5	    ((UBX_CLASS_CFG) | UBX_ID_CFG_NAV5 << 8)
#define UBX_MSG_CFG_RST 	    ((UBX_CLASS_CFG) | UBX_ID_CFG_RST << 8)
#define UBX_MSG_CFG_SBAS	    ((UBX_CLASS_CFG) | UBX_ID_CFG_SBAS << 8)
#define UBX_MSG_CFG_TMODE3	  ((UBX_CLASS_CFG) | UBX_ID_CFG_TMODE3 << 8)
#define UBX_MSG_CFG_VALGET	  ((UBX_CLASS_CFG) | UBX_ID_CFG_VALGET << 8)
#define UBX_MSG_CFG_VALSET	  ((UBX_CLASS_CFG) | UBX_ID_CFG_VALSET << 8)
#define UBX_MSG_CFG_VALDEL	  ((UBX_CLASS_CFG) | UBX_ID_CFG_VALDEL << 8)
#define UBX_MSG_MON_HW		    ((UBX_CLASS_MON) | UBX_ID_MON_HW << 8)
#define UBX_MSG_MON_VER		    ((UBX_CLASS_MON) | UBX_ID_MON_VER << 8)
#define UBX_MSG_MON_RF		    ((UBX_CLASS_MON) | UBX_ID_MON_RF << 8)
#define UBX_MSG_RTCM3_1005	  ((UBX_CLASS_RTCM3) | UBX_ID_RTCM3_1005 << 8)
#define UBX_MSG_RTCM3_1077	  ((UBX_CLASS_RTCM3) | UBX_ID_RTCM3_1077 << 8)
#define UBX_MSG_RTCM3_1087	  ((UBX_CLASS_RTCM3) | UBX_ID_RTCM3_1087 << 8)
#define UBX_MSG_RTCM3_1074	  ((UBX_CLASS_RTCM3) | UBX_ID_RTCM3_1074 << 8)
#define UBX_MSG_RTCM3_1084	  ((UBX_CLASS_RTCM3) | UBX_ID_RTCM3_1084 << 8)
#define UBX_MSG_RTCM3_1094	  ((UBX_CLASS_RTCM3) | UBX_ID_RTCM3_1094 << 8)
#define UBX_MSG_RTCM3_1097	  ((UBX_CLASS_RTCM3) | UBX_ID_RTCM3_1097 << 8)
#define UBX_MSG_RTCM3_1124	  ((UBX_CLASS_RTCM3) | UBX_ID_RTCM3_1124 << 8)
#define UBX_MSG_RTCM3_1127	  ((UBX_CLASS_RTCM3) | UBX_ID_RTCM3_1127 << 8)
#define UBX_MSG_RTCM3_1230	  ((UBX_CLASS_RTCM3) | UBX_ID_RTCM3_1230 << 8)
#define UBX_MSG_RTCM3_4072	  ((UBX_CLASS_RTCM3) | UBX_ID_RTCM3_4072 << 8)

/* RX NAV-PVT message content details */
/*   Bitfield "valid" masks */
#define UBX_RX_NAV_PVT_VALID_VALIDDATE		0x01	/**< validDate (Valid UTC Date) */
#define UBX_RX_NAV_PVT_VALID_VALIDTIME		0x02	/**< validTime (Valid UTC Time) */
#define UBX_RX_NAV_PVT_VALID_FULLYRESOLVED	0x04	/**< fullyResolved (1 = UTC Time of Day has been fully resolved (no seconds uncertainty)) */

/*   Bitfield "flags1" masks */
#define UBX_RX_NAV_PVT_FLAGS_GNSSFIXOK		0x01	/**< gnssFixOK (A valid fix (i.e within DOP & accuracy masks)) */
#define UBX_RX_NAV_PVT_FLAGS_DIFFSOLN		  0x02	/**< diffSoln (1 if differential corrections were applied) */
#define UBX_RX_NAV_PVT_FLAGS_PSMSTATE		  0x1C	/**< psmState (Power Save Mode state (see Power Management)) */
#define UBX_RX_NAV_PVT_FLAGS_HEADVEHVALID	0x20	/**< headVehValid (Heading of vehicle is valid) */
#define UBX_RX_NAV_PVT_FLAGS_CARRSOLN		  0xC0	/**< Carrier phase range solution (RTK mode) */

/*   Bitfield "flags2" masks */
#define UBX_RX_NAV_PVT_FLAGS_CONFIRMEDTIME 0x20	/**< confirmTime (1 = UTC Time validity confirmed) */
#define UBX_RX_NAV_PVT_FLAGS_CONFIRMEDDATE 0x40	/**< confirmDate (1 = UTC Date validity confirmed) */
#define UBX_RX_NAV_PVT_FLAGS_CONFIRMAVAI	 0x80 /**< confirmedAvail (1 = available time and date confirmation) */

/* RX NAV-TIMEUTC message content details */
/*   Bitfield "valid" masks */
#define UBX_RX_NAV_TIMEUTC_VALID_VALIDTOW	0x01	/**< validTOW (1 = Valid Time of Week) */
#define UBX_RX_NAV_TIMEUTC_VALID_VALIDKWN	0x02	/**< validWKN (1 = Valid Week Number) */
#define UBX_RX_NAV_TIMEUTC_VALID_VALIDUTC	0x04	/**< validUTC (1 = Valid UTC Time) */
#define UBX_RX_NAV_TIMEUTC_VALID_UTCSTANDARD	0xF0	/**< utcStandard (0..15 = UTC standard identifier) */

/* TX CFG-RST message contents
 */
#define UBX_TX_CFG_RST_BBR_MODE_HOT_START 	0
#define UBX_TX_CFG_RST_BBR_MODE_WARM_START 	1
#define UBX_TX_CFG_RST_BBR_MODE_COLD_START 	0xFFFF
#define UBX_TX_CFG_RST_MODE_HARDWARE 		0
#define UBX_TX_CFG_RST_MODE_SOFTWARE 		1

// Key ID's for CFG-VAL{GET,SET,DEL}
#define UBX_CFG_KEY_CFG_UART1_BAUDRATE           0x40520001
#define UBX_CFG_KEY_CFG_UART1_STOPBITS           0x20520002
#define UBX_CFG_KEY_CFG_UART1_DATABITS           0x20520003
#define UBX_CFG_KEY_CFG_UART1_PARITY             0x20520004
#define UBX_CFG_KEY_CFG_UART1_ENABLED            0x20520005
#define UBX_CFG_KEY_CFG_UART1_REMAP              0x20520006
#define UBX_CFG_KEY_CFG_UART1INPROT_UBX          0x10730001
#define UBX_CFG_KEY_CFG_UART1INPROT_NMEA         0x10730002
#define UBX_CFG_KEY_CFG_UART1INPROT_RTCM3X       0x10730004
#define UBX_CFG_KEY_CFG_UART1OUTPROT_UBX         0x10740001
#define UBX_CFG_KEY_CFG_UART1OUTPROT_NMEA        0x10740002
#define UBX_CFG_KEY_CFG_UART1OUTPROT_RTCM3X      0x10740004

#define UBX_CFG_KEY_CFG_UART2_BAUDRATE           0x40530001
#define UBX_CFG_KEY_CFG_UART2_STOPBITS           0x20530002
#define UBX_CFG_KEY_CFG_UART2_DATABITS           0x20530003
#define UBX_CFG_KEY_CFG_UART2_PARITY             0x20530004
#define UBX_CFG_KEY_CFG_UART2_ENABLED            0x20530005
#define UBX_CFG_KEY_CFG_UART2_REMAP              0x20530006
#define UBX_CFG_KEY_CFG_UART2INPROT_UBX          0x10750001
#define UBX_CFG_KEY_CFG_UART2INPROT_NMEA         0x10750002
#define UBX_CFG_KEY_CFG_UART2INPROT_RTCM3X       0x10750004
#define UBX_CFG_KEY_CFG_UART2OUTPROT_UBX         0x10760001
#define UBX_CFG_KEY_CFG_UART2OUTPROT_NMEA        0x10760002
#define UBX_CFG_KEY_CFG_UART2OUTPROT_RTCM3X      0x10760004

#define UBX_CFG_KEY_CFG_USB_ENABLED              0x10650001
#define UBX_CFG_KEY_CFG_USBINPROT_UBX            0x10770001
#define UBX_CFG_KEY_CFG_USBINPROT_NMEA           0x10770002
#define UBX_CFG_KEY_CFG_USBINPROT_RTCM3X         0x10770004
#define UBX_CFG_KEY_CFG_USBOUTPROT_UBX           0x10780001
#define UBX_CFG_KEY_CFG_USBOUTPROT_NMEA          0x10780002
#define UBX_CFG_KEY_CFG_USBOUTPROT_RTCM3X        0x10780004

#define UBX_CFG_KEY_CFG_SPIINPROT_UBX            0x10790001
#define UBX_CFG_KEY_CFG_SPIINPROT_NMEA           0x10790002
#define UBX_CFG_KEY_CFG_SPIINPROT_RTCM3X         0x10790004
#define UBX_CFG_KEY_CFG_SPIOUTPROT_UBX           0x107a0001
#define UBX_CFG_KEY_CFG_SPIOUTPROT_NMEA          0x107a0002
#define UBX_CFG_KEY_CFG_SPIOUTPROT_RTCM3X        0x107a0004

#define UBX_CFG_KEY_NAVHPG_DGNSSMODE             0x20140011

#define UBX_CFG_KEY_NAVSPG_FIXMODE               0x20110011
#define UBX_CFG_KEY_NAVSPG_UTCSTANDARD           0x2011001c
#define UBX_CFG_KEY_NAVSPG_DYNMODEL              0x20110021

#define UBX_CFG_KEY_ODO_USE_ODO                  0x10220001
#define UBX_CFG_KEY_ODO_USE_COG                  0x10220002
#define UBX_CFG_KEY_ODO_OUTLPVEL                 0x10220003
#define UBX_CFG_KEY_ODO_OUTLPCOG                 0x10220004

#define UBX_CFG_KEY_RATE_MEAS                    0x30210001
#define UBX_CFG_KEY_RATE_NAV                     0x30210002
#define UBX_CFG_KEY_RATE_TIMEREF                 0x20210003

#define UBX_CFG_KEY_TMODE_MODE                   0x20030001
#define UBX_CFG_KEY_TMODE_POS_TYPE               0x20030002
#define UBX_CFG_KEY_TMODE_LAT                    0x40030009
#define UBX_CFG_KEY_TMODE_LON                    0x4003000a
#define UBX_CFG_KEY_TMODE_HEIGHT                 0x4003000b
#define UBX_CFG_KEY_TMODE_LAT_HP                 0x2003000c
#define UBX_CFG_KEY_TMODE_LON_HP                 0x2003000d
#define UBX_CFG_KEY_TMODE_HEIGHT_HP              0x2003000e
#define UBX_CFG_KEY_TMODE_FIXED_POS_ACC          0x4003000f
#define UBX_CFG_KEY_TMODE_SVIN_MIN_DUR           0x40030010
#define UBX_CFG_KEY_TMODE_SVIN_ACC_LIMIT         0x40030011

#define UBX_CFG_KEY_MSGOUT_UBX_MON_RF_I2C        0x20910359
#define UBX_CFG_KEY_MSGOUT_UBX_NAV_SVIN_I2C      0x20910088
#define UBX_CFG_KEY_MSGOUT_UBX_NAV_SAT_I2C       0x20910015
#define UBX_CFG_KEY_MSGOUT_UBX_NAV_DOP_I2C       0x20910038
#define UBX_CFG_KEY_MSGOUT_UBX_NAV_PVT_I2C       0x20910006
#define UBX_CFG_KEY_MSGOUT_NMEA_ID_DTM_I2C       0x209100a6
#define UBX_CFG_KEY_MSGOUT_NMEA_ID_GBS_I2C       0x209100dd
#define UBX_CFG_KEY_MSGOUT_NMEA_ID_GGA_I2C       0x209100ba
#define UBX_CFG_KEY_MSGOUT_NMEA_ID_GLL_I2C       0x209100c9
#define UBX_CFG_KEY_MSGOUT_NMEA_ID_GNS_I2C       0x209100b5
#define UBX_CFG_KEY_MSGOUT_NMEA_ID_GRS_I2C       0x209100ce
#define UBX_CFG_KEY_MSGOUT_NMEA_ID_GSA_I2C       0x209100bf
#define UBX_CFG_KEY_MSGOUT_NMEA_ID_GST_I2C       0x209100d3
#define UBX_CFG_KEY_MSGOUT_NMEA_ID_GSV_I2C       0x209100c4
#define UBX_CFG_KEY_MSGOUT_NMEA_ID_RMC_I2C       0x209100ab
#define UBX_CFG_KEY_MSGOUT_NMEA_ID_VLW_I2C       0x209100e7
#define UBX_CFG_KEY_MSGOUT_NMEA_ID_VTG_I2C       0x209100b0
#define UBX_CFG_KEY_MSGOUT_NMEA_ID_ZDA_I2C       0x209100d8
#define UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1005_I2C  0x209102bd
#define UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1074_I2C  0x2091035e
#define UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1077_I2C  0x209102cc
#define UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1084_I2C  0x20910363
#define UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1087_I2C  0x209102d1
#define UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1094_I2C  0x20910368
#define UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1097_I2C  0x20910318
#define UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1124_I2C  0x2091036d
#define UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1127_I2C  0x209102d6
#define UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1230_I2C  0x20910303

// ubx cfg layers
#define UBX_CFG_LAYER_RAM   (1 << 0)
#define UBX_CFG_LAYER_BBR   (1 << 1)
#define UBX_CFG_LAYER_FLASH (1 << 2)

