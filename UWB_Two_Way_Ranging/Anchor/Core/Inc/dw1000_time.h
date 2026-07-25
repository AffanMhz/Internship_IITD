#ifndef DW1000_TIME_H_
#define DW1000_TIME_H_

#include <stdint.h>

typedef uint64_t dw1000_timestamp_t;

extern const double DW1000_TIME_RES;
extern const double DW1000_TIME_RES_INV;

extern const double DW1000_TIME_DISTANCE_OF_RADIO;
extern const double DW1000_TIME_DISTANCE_OF_RADIO_INV;

dw1000_timestamp_t DW1000_Time_MicrosecondsToTimestamp(double time_us);
double DW1000_Time_TimestampToMicroseconds(dw1000_timestamp_t timestamp);
double DW1000_Time_TimestampToMeters(dw1000_timestamp_t timestamp);
#endif /* DW1000_TIME_H_ */
