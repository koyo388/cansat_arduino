#include "sd.h"
#include <SPI.h>
#include <SD.h>

void sd_init() {
  Serial.begin(9600) ;
  SD.begin(sd) ;  
}

void save_log(float originlat , float originlon) {
  File Tsat = SD.open("t-sat.txt",FILE_WRITE) ;
  Tsat.println(originlat,originlon) ;
  Tsat.close() ; 
}
