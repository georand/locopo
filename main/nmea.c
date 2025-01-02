#include "common.h"

#include <string.h>

#include "nmea.h"

char nmeaOutputStr[MAX_BUFFER];
uint8_t nmeaBuffer[MAX_BUFFER];
uint16_t nmeaBufPos = 0;
uint8_t nmeaOutputAvailable = 0;

//static const char* TAG = __FILE__;
static const char* TAG = "nmea";

void nmea_init()
{
  nmea_decode_reset();
}

void nmea_format_output_str()
{
  // store output
  memcpy(nmeaOutputStr,nmeaBuffer,MAX_BUFFER);

  // make it available for printing
  nmeaOutputAvailable = 1;
}

const char* nmea_get_output_str()
{
  if (nmeaOutputAvailable)
  {
    nmeaOutputAvailable = 0;
    return nmeaOutputStr;
  }
  else
    return NULL;
}

void nmea_decode_reset()
{
  nmeaBufPos = 0;
}

int nmea_decode_data(uint8_t c)
{
  int ret = ESP_OK;

  if (c == '$' || nmeaBufPos >= MAX_BUFFER-1)
    nmea_decode_reset();

  nmeaBuffer[nmeaBufPos++]=c;

  if ((nmeaBufPos >= 3) && (nmeaBuffer[0] == NMEA_PREAMBLE
                        && nmeaBuffer[nmeaBufPos-3] == NMEA_CLOSING ))
  {
    nmeaBuffer[nmeaBufPos]='\0';
    ret = nmea_checksum_verif();
    if (ret == ESP_OK)
       nmea_format_output_str();
    else
      ESP_LOGD(TAG,"bad checksum: %s", nmeaBuffer);
    nmea_decode_reset();
  }
  return ret;
}

int nmea_checksum_verif()
{
  uint8_t calcChk=0;
  uint8_t nmeaChk=0;

  for (int i=1; i<nmeaBufPos-3; i++)
    calcChk ^= (uint8_t)nmeaBuffer[i];
  char hx[3];
  hx[0] = nmeaBuffer[nmeaBufPos-2];
  hx[1] = nmeaBuffer[nmeaBufPos-1];
  hx[2]='\0';
  nmeaChk = strtol(hx,NULL,16);

  return (calcChk == nmeaChk? ESP_OK : ESP_FAIL);
}

