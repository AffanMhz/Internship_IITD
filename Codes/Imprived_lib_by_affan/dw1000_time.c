#include "dw1000_time.h"

#include "dw1000_time.h"

const double DW1000_TIME_RES = 0.000015650040064103f;
const double DW1000_TIME_RES_INV = 63897.6f;

const double DW1000_TIME_DISTANCE_OF_RADIO = 0.0046917639786159f;
const double DW1000_TIME_DISTANCE_OF_RADIO_INV = 213.139451293f;

dw1000_timestamp_t DW1000_Time_MicrosecondsToTimestamp(double time_us) {
  return (dw1000_timestamp_t)(time_us * DW1000_TIME_RES_INV);
}

double DW1000_Time_TimestampToMicroseconds(dw1000_timestamp_t timestamp) {
  return timestamp*DW1000_TIME_RES;
}

double DW1000_Time_TimestampToMeters(dw1000_timestamp_t timestamp) {
  return timestamp*DW1000_TIME_DISTANCE_OF_RADIO;
}
