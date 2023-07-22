#ifndef AR488_LAYOUTS_H
#define AR488_LAYOUTS_H

#include <Arduino.h>

#include "AR488_Config.h"

/***** AR488_Hardware.h, ver. 0.51.09, 20/06/2022 *****/
/*
 * Hardware pin layout definitions
 */

/****************************************************************/
/***** MICRO PRO (32u4) LAYOUT DEFINITION for MICRO (Artag) *****/
/***** vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv *****/
#ifdef AR488_MEGA32U4_MICRO

#define DIO1  3   /* GPIB 1  : PORTD bit 0   data pins assigned for minimum shifting */
#define DIO2  15  /* GPIB 2  : PORTB bit 1 */
#define DIO3  16  /* GPIB 3  : PORTB bit 2 */
#define DIO4  14  /* GPIB 4  : PORTB bit 3 */
#define DIO5  8   /* GPIB 13 : PORTB bit 4 */
#define DIO6  9   /* GPIB 14 : PORTB bit 5 */
#define DIO7  10  /* GPIB 15 : PORTB bit 6 */
#define DIO8  6   /* GPIB 16 : PORTD bit 7 */

#define IFC   4   /* GPIB 9  : PORTD bit 4 */
#define NDAC  A3  /* GPIB 8  : PORTF bit 4   fast control pins assigned to same port */
#define NRFD  A2  /* GPIB 7  : PORTF bit 5 */
#define DAV   A1  /* GPIB 6  : PORTF bit 6 */
#define EOI   A0  /* GPIB 5  : PORTF bit 7 */
#define REN   5   /* GPIB 17 : PORTC bit 6 */
#define SRQ   7   /* GPIB 10 : PORTE bit 6 */
#define ATN   2   /* GPIB 11 : PORTD bit 1 */

/*
#ifdef USE_INTERRUPTS
  #define ATNPREG PIND
  #define SRQPREG PINE
  void interruptsEn();
#endif  // USE_INTERRUPTS
*/
#endif  // AR488_MEGA32U4_MICRO

/**************************************/
/***** GLOBAL DEFINITIONS SECTION *****/
/***** vvvvvvvvvvvvvvvvvvvvvvvvvv *****/

enum GPIB_CONTROL : uint8_t {
  SIG_NONE = 0x00,
  SIG_IFC = 0x01,
  SIG_NDAC = 0x02,
  SIG_NRFD = 0x04,
  SIG_DAV = 0x08,
  SIG_EOI = 0x10,
  SIG_REN = 0x20,
  SIG_SRQ = 0x40,
  SIG_ATN = 0x80,
  SIG_ALL = 0xFF
};

inline GPIB_CONTROL operator|(GPIB_CONTROL a, GPIB_CONTROL b)
{
    return static_cast<GPIB_CONTROL>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

enum STATE_MODE : uint8_t {
  STATE = 0,
  DIRECTION = 1
};

void readyGpibDbus();
uint8_t readGpibDbus();
void setGpibDbus(uint8_t db);
void setGpibState(uint8_t bits, uint8_t mask, STATE_MODE mode);
void setGpibState(GPIB_CONTROL bits, GPIB_CONTROL mask, STATE_MODE mode);
uint8_t getGpibPinState(uint8_t pin);

#endif // AR488_LAYOUTS_H
