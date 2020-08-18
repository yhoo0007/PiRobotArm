#ifndef DEBUG_H
#define DEBUG_H

#include "HardwareSerial.h"

// #define DEBUG

#ifdef DEBUG
#define DPRINT_BEGIN(baud) Serial.begin(baud)
#define DPRINT(string) Serial.print(string)
#define DPRINTLN(string) Serial.println(string)
#define DPRINTLN_ARR(arr, n) for (int _iterator_ = 0; _iterator_ < n; _iterator_++) Serial.println(arr[_iterator_])
#else
#define DPRINT_BEGIN(baud)
#define DPRINT(string)
#define DPRINTLN(string)
#define DPRINTLN_ARR(arr, n)
#endif

#endif
