#ifndef AR488_CONFIG_H
#define AR488_CONFIG_H

/*********************************************/
/***** AR488 GLOBAL CONFIGURATION HEADER *****/
/***** vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv *****/


/***** Firmware version *****/
#define FWVER "GPIB, ver. 0.60.0, 03/09/2022, ProLogix Version 4.40"

/*** Board/layout selection ***/
#define AR488_MEGA32U4_MICRO  // Artag's design for Micro board
#define DATAPORT_ENABLE
#define AR_SERIAL_PORT Serial
#define AR_SERIAL_SPEED 250000
#define USE_INTERRUPTS

/*********************************************/
/***** MISCELLANEOUS DECLARATIONS *****/
/******vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv******/

#define AR_CFG_SIZE 84

/******^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^******/
/***** MISCELLANEOUS DECLARATIONS *****/
/*********************************************/

#endif // AR488_CONFIG_H
