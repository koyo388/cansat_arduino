#ifndef SD_H_INCLUDED
#define SD_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define sd A1 

void sd_init() ;
void save_log(float originlat , float originlon) ;

#endif
