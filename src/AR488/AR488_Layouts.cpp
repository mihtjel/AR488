#include <Arduino.h>

#include "AR488_Config.h"
#include "AR488_Layouts.h"

/***** AR488_Hardware.cpp, ver. 0.51.09, 20/06/2022 *****/
/*
 * Hardware layout function definitions
 */

/***********************************************************/
/***** MICRO PRO (32u4) BOARD LAYOUT for MICRO (Artag) *****/
/***** vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv *****/
#ifdef AR488_MEGA32U4_MICRO

/***** Set the GPIB data bus to input pullup *****/
void readyGpibDbus() {
  // Set data pins to input
  DDRB  &= 0b10000001 ;
  DDRD  &= 0b01111110 ;
  PORTB |= 0b01111110; // PORTB bits 6,5,4,3,2,1 input_pullup
  PORTD |= 0b10000001; // PORTD bits 7,0 input_pullup

  // Read the byte of data on the bus
  // DIO8 -> PORTD bit 7, DIO7 -> PORTE bit 5, DIO6-DIO1 -> PORTB bit 451326
}

/***** Read the GPIB data bus wires to collect the byte of data *****/
uint8_t readGpibDbus() {
  return ~((PIND & 0b10000001) | (PINB & 0b01111110)) ;
}

/***** Set the GPIB data bus to output and with the requested byte *****/
void setGpibDbus(uint8_t db) {
  // Set data pins as outputs
  DDRB |= 0b01111110;
  DDRD |= 0b10000001;

  // GPIB states are inverted
  db = ~db;

  // Set data bus
  PORTB = (PORTB & ~0b01111110) | (db & 0b01111110) ;
  PORTD = (PORTD & ~0b10000001) | (db & 0b10000001);
}

/***** Set the direction and state of the GPIB control lines *****/
/*
 * Bits control lines as follows: 7-ATN, 6-SRQ, 5-REN, 4-EOI, 3-DAV, 2-NRFD, 1-NDAC, 0-IFC
    bits (databits) : State - 0=LOW, 1=HIGH/INPUT_PULLUP; Direction - 0=input, 1=output;
    mask (mask)     : 0=unaffected, 1=enabled
    mode (mode)     : 0=set pin state, 1=set pin direction
 * Arduino Pro Micro pin to Port/bit to direction/state byte map:
 * IFC   4   PORTD bit 4   byte bit 0
 * NDAC  A3  PORTF bit 4   byte bit 1
 * NRFD  A2  PORTF bit 5   byte bit 2
 * DAV   A1  PORTF bit 6   byte bit 3
 * EOI   A0  PORTF bit 7   byte bit 4
 * REN   5   PORTC bit 6   byte bit 5
 * SRQ   7   PORTE bit 6   byte bit 6
 * ATN   2   PORTD bit 1   byte bit 7
 * 
 * It would be more efficient (and easier to read the code) if the bits in the above
 * control word were assigned by name to match suitable port bits : then NDAC,NRFD and DAV
 * could be positioned at bits 4,5,6 to be placed in port F without shifting.
 */
void setGpibState(GPIB_CONTROL bits, GPIB_CONTROL mask, STATE_MODE mode) {
  setGpibState((uint8_t) bits, (uint8_t) mask, mode);
}


void setGpibState(uint8_t bits, uint8_t mask, STATE_MODE mode) {
  // most of the time, only these bits change
  if (mask & (SIG_NDAC | SIG_NRFD | SIG_DAV | SIG_EOI)) {
    // PORTF - NDAC, NRFD, DAV and EOI bits 1-4 rotated into bits 4-7
    uint8_t portFb = (bits & 0x1e) << 3;
    uint8_t portFm = (mask & 0x1e) << 3;

    // Set registers: register = (register & ~bitmask) | (value & bitmask)
    // Mask: 0=unaffected; 1=to be changed

    switch (mode) {
      case STATE:
        // Set pin states using mask
        PORTF = ( (PORTF & ~portFm) | (portFb & portFm) );
        break;
      case DIRECTION:
        // Set pin direction registers using mask
        DDRF = ( (DDRF & ~portFm) | (portFb & portFm) );
        break;
    }
  }

  // slow due to messy port layout but infrequently needed
  if (mask & (SIG_IFC | SIG_REN | SIG_SRQ | SIG_ATN)) {
    // PORTC - REN bit 5 rotated into bit 6
    uint8_t portCb = (bits & 0x20) << 1;
    uint8_t portCm = (mask & 0x20) << 1;
    // PORTD - IFC bit 0 rotated into bit 4 and ATN bit 7 rotated into 1
    uint8_t portDb = ((bits & 0x01) << 4) | ((bits & 0x80) >> 6);
    uint8_t portDm = ((mask & 0x01) << 4) | ((mask & 0x80) >> 6);
    // PORT E - SRQ bit 6  in bit 6
    uint8_t portEb = (bits & 0x40);
    uint8_t portEm = (mask & 0x40);

    // Set registers: register = (register & ~bitmask) | (value & bitmask)
    // Mask: 0=unaffected; 1=to be changed

    switch (mode) {
      case STATE:
        // Set pin states using mask
        PORTC = ( (PORTC & ~portCm) | (portCb & portCm) );
        PORTD = ( (PORTD & ~portDm) | (portDb & portDm) );
        PORTE = ( (PORTE & ~portEm) | (portEb & portEm) );
        break;
      case DIRECTION:
        // Set pin direction registers using mask
        DDRC = ( (DDRC & ~portCm) | (portCb & portCm) );
        DDRD = ( (DDRD & ~portDm) | (portDb & portDm) );
        DDRE = ( (DDRE & ~portEm) | (portEb & portEm) );
        break;
    }
  }
}


/***** Enable interrupts *****/
/*
#ifdef USE_INTERRUPTS

volatile uint8_t atnPinMem = ATNPREG;
volatile uint8_t srqPinMem = SRQPREG;
static const uint8_t ATNint = 0b00000010;
static const uint8_t SRQint = 0b01000000;

void pin_change_interrupt(void) {

  // Has the status of the ATN pin interrupt changed?
  if ((ATNPREG ^ atnPinMem) & ATNint) {
    // Set the current status of ATN
    isATN = (ATNPREG & ATNint) == 0;
  }

  // Has the status of the SRQ pin interrupt changed?
  if ((SRQPREG ^ srqPinMem) & SRQint) {
    // Set the current status of SRQ
    isSRQ = (SRQPREG & SRQint) == 0;
  }

  // Save current state of the interrupt registers
  atnPinMem = ATNPREG;
  srqPinMem = SRQPREG;
}

void interruptsEn(){
  cli();
  attachInterrupt(digitalPinToInterrupt(ATN), pin_change_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SRQ), pin_change_interrupt, CHANGE);
  sei();  sei();
}

#endif  // USE_INTERRUPTS
*/
#endif  // AR488_MEGA32U4_MICRO
/***** ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ *****/
/***** MICRO PRO (32u4) BOARD LAYOUT for MICRO (Artag) *****/
/***********************************************************/

/************************************/
/***** COMMON FUNCTIONS SECTION *****/
/***** vvvvvvvvvvvvvvvvvvvvvvvv *****/

#if not defined(AR488_MCP23S17) && not defined(AR488_MCP23017)
uint8_t getGpibPinState(uint8_t pin){
  return digitalRead(pin);
}
#endif

/***** ^^^^^^^^^^^^^^^^^^^^^^^^ *****/
/***** COMMON FUNCTIONS SECTION *****/
/************************************/
