#ifndef PROJ_TYPES_H
#define PROJ_TYPES_H

#include "Arduino.h"

typedef struct{
  uint32_t delta, current, previous, interval;
} schedule_t;


void referee_request(const char* mess);


#endif // PROJ_TYPES_H
