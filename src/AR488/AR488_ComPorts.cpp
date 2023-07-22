#include <Arduino.h>
#include "AR488_ComPorts.h"

/***** AR488_ComPorts.cpp, ver. 0.51.09, 20/06/2022 *****/
/*
 * Serial Port implementation
 */

/*****************************/ 
/*****  DATA SERIAL PORT *****/
/*****************************/

Stream& dataPort = AR_SERIAL_PORT;

void startDataPort() {
  AR_SERIAL_PORT.begin(AR_SERIAL_SPEED);
}
