#ifndef AR488_COMPORTS_H
#define AR488_COMPORTS_H

#include <Arduino.h>
#include "AR488_Config.h"

/***** AR488_ComPorts.cpp, ver. 0.51.09, 20/06/2022 *****/
/*
 * Serial Port definition
 */
extern Stream& dataPort;
void startDataPort();

#define DATAPORT_START() startDataPort()
#define DATA_RAW_PRINT(str) dataPort.print(str)
#define DATA_RAW_PRINTLN(str) dataPort.println(str)

#define DEBUG_START()
#define DB_PRINT(msg1,msg2)
#define DB_RAW_PRINT(msg)
#define DB_RAW_PRINTLN(msg)
#define DB_HEX_PRINT(byteval)
#define DB_HEXB_PRINT(buf, bsize)

#endif  // AR488_COMPORTS_H
