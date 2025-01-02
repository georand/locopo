#define NMEA_PREAMBLE '$'
#define NMEA_CLOSING  '*'

void nmea_init();
void nmea_format_output_str();
const char* nmea_get_output_str();
void nmea_decode_reset();
int nmea_decode_data(uint8_t c);
int nmea_checksum_verif();
