#include <Arduino.h>
#include "AR488_Config.h"
#include "AR488_GPIBbus.h"

/***** AR488_GPIB.cpp, ver. 0.51.09, 20/06/2022 *****/

/****** Process status values *****/
#define OK  false
#define ERR true

/***** Control characters *****/
#define ESC  0x1B   // the USB escape char
#define CR   0x0D   // Carriage return
#define LF   0x0A   // Newline/linefeed
#define PLUS 0x2B   // '+' character

/***************************************/
/***** GPIB CLASS PUBLIC FUNCTIONS *****/
/***** vvvvvvvvvvvvvvvvvvvvvvvvvvv *****/

/********** PUBLIC FUNCTIONS **********/

/***** Class constructor *****/
GPIBbus::GPIBbus() {
  // Default configuration values
  setDefaultCfg();
  cstate = 0;
//  addressingSuppressed = false;
//  dataContinuity = false;
  deviceAddressed = false;
  deviceAddressedState = DIDS;
}

/***** Start the bus in controller or device mode depending on config *****/
void GPIBbus::begin() {
  if (isController()) {
    startControllerMode();
  } else {
    startDeviceMode();
  }
}

/***** Stops active mode and brings control and data bus to inactive state *****/
void GPIBbus::stop() {
  cstate = 0;
  // Set control bus to idle state (all lines input_pullup)
  // Input_pullup
  setGpibState(SIG_NONE, SIG_ALL, DIRECTION);
  // All lines HIGH
  setGpibState(SIG_ALL, SIG_ALL, STATE);
  // Set data bus to default state (all lines input_pullup)
  readyGpibDbus();
}

/***** Initialise the interface *****/
void GPIBbus::setDefaultCfg() {
  // Set default values ({'\0'} sets version string array to null)
  cfg = {false, false, 2, 0, 1, 0, 0, 0, 0, 1200, 0, {'\0'}, 0, {'\0'}, 0, 0};
}

/***** Set bus into Device mode *****/
void GPIBbus::startDeviceMode() {
  // Stop current mode
  stop();
  delayMicroseconds(200); // Allow settling time
  // Start device mode
  cfg.cmode = 1;
  // Set GPIB control bus to device idle mode
  setControls(DINI);
  // Initialise GPIB data lines (sets to INPUT_PULLUP)
  readyGpibDbus();
}

/***** Set interface into Controller mode *****/
void GPIBbus::startControllerMode() {
  // Send request to clear all devices on bus to local mode
  sendAllClear();
  // Stop current mode
  stop();
  delayMicroseconds(200); // Allow settling time
  // Start controller mode
  cfg.cmode = 2;
  // Set GPIB control bus to controller idle mode
  setControls(CINI);
  // Initialise GPIB data lines (sets to INPUT_PULLUP)
  readyGpibDbus();
  // Assert IFC to signal controller in charge (CIC)
  sendIFC();
  // Attempt to address device to listen
  if (cfg.paddr > 1)
    addressDevice(cfg.paddr, 0);
}

/***** Return current cinterface mode *****/
bool GPIBbus::isController() {
  return (cfg.cmode == 2);
}


/***** Detect selected pin state *****/
bool GPIBbus::isAsserted(uint8_t gpibsig){
  return (getGpibPinState(gpibsig) == LOW);
}

/***** Send the device status byte *****/
void GPIBbus::sendStatus() {
  // Have been addressed and polled so send the status byte
  if (!(cstate==DTAS))
    setControls(DTAS);
  writeByte(cfg.stat, NO_EOI);
  setControls(DIDS);
  // Clear the SRQ bit
  cfg.stat = cfg.stat & ~0x40;
  // De-assert the SRQ signal
  clrSrqSig();
}

/***** Set the status byte *****/
void GPIBbus::setStatus(uint8_t statusByte) {
  cfg.stat = statusByte;
  if (statusByte & 0x40) {
    // If SRQ bit is set then assert the SRQ signal
    setSrqSig();
  } else {
    // If SRQ bit is NOT set then de-assert the SRQ signal
    clrSrqSig();
  }
}

/***** Send IFC *****/
void GPIBbus::sendIFC() {
  // Assert IFC
  setGpibState(0, SIG_IFC, STATE);
  delayMicroseconds(150);
  // De-assert IFC
  setGpibState(SIG_IFC, SIG_IFC, STATE);
}


/***** Send SDC GPIB command *****/
bool GPIBbus::sendSDC() {
  if (addressDevice(cfg.paddr, 0))
    return ERR;  // Send SDC to currently addressed device
  if (sendCmd(GC_SDC))
     return ERR;
  // Unlisten bus
  if (unAddressDevice())
    return ERR;
  return OK;
}


/***** Send LLO GPIB command *****/
bool GPIBbus::sendLLO() {
  if (addressDevice(cfg.paddr, 0))
    return ERR;
  // Send LLO to currently addressed device
  if (sendCmd(GC_LLO))
     return ERR;  // Unlisten bus
  if (unAddressDevice())
    return ERR;
  return OK;
}

/***** Send LOC GPIB command *****/
bool GPIBbus::sendGTL() {
  if (addressDevice(cfg.paddr, 0)) {
    return ERR;
  }
  // Send GTL
  if (sendCmd(GC_GTL)) {
    return ERR;
  }
  // Unlisten bus
  if (unAddressDevice()) {
    return ERR;  
  }
  return OK;
}

/***** Send GET (trigger) command *****/
bool GPIBbus::sendGET(uint8_t addr) {
  if (addressDevice(addr, 0))
    return ERR;

  // Send GET
  if (sendCmd(GC_GET))
    return ERR;

  // Unlisten bus
  if (unAddressDevice())
    return ERR;  
  return OK;
}

/***** Send request to clear to all devices to local *****/
void GPIBbus::sendAllClear(){
  // Un-assert REN
  setControlVal(0b00100000, 0b00100000, STATE);
  delay(40);
  // Simultaneously assert ATN and REN
  setControlVal(0b00000000, 0b10100000, STATE);
  delay(40);
  // Unassert ATN
  setControlVal(0b10000000, 0b10000000, STATE);
}

/***** Request device to talk *****/
bool GPIBbus::sendMTA(){
  if (cstate!=CCMS)
    setControls(CCMS);
  if (addressDevice(cfg.paddr, 1))
    return ERR;
  return OK;
}

/***** Request device to listen *****/
bool GPIBbus::sendMLA(){
  if (cstate!=CCMS)
    setControls(CCMS);
  if (addressDevice(cfg.paddr, 0))
    return ERR;
  return OK;
}
    
/***** Send secondary address command *****/
bool GPIBbus::sendMSA(uint8_t addr) {
  // Send address
  if (sendCmd(addr))
    return ERR;
  // Unassert ATN
  setControlVal(0b10000000, 0b10000000, STATE);
  return OK;
}

/***** Send untalk (SAD mode) *****/
bool GPIBbus::sendUNT(){
  if (sendCmd(GC_UNT))
    return ERR;
 
  setControls(CIDS);
  deviceAddressed = false;
  return OK;
}


/***** Send unlisten *****/
bool GPIBbus::sendUNL(){
  if (sendCmd(GC_UNL))
    return ERR;

  setControls(CIDS);
  deviceAddressed = false;
  return OK;
}


/*****  Send a single byte GPIB command *****/
bool GPIBbus::sendCmd(uint8_t cmdByte){
  bool stat = false;
  // Set lines for command and assert ATN
  if (cstate!=CCMS)
    setControls(CCMS);

  // Send the command
  return writeByte(cmdByte, NO_EOI);
}


/***** Receive data from the GPIB bus ****/
/*
 * Readbreak:
 * 7 - command received via serial
 */
bool GPIBbus::receiveData(Stream& dataStream, bool detectEoi, bool detectEndByte, uint8_t endByte) {

  uint8_t r = 0; //, db;
  uint8_t bytes[3] = {0};
  uint8_t eor = cfg.eor&7;
  int x = 0;
  bool readWithEoi = false;
  bool eoiDetected = false;

  endByte = endByte;  // meaningless but defeats vcompiler warning!

  // Reset transmission break flag
  txBreak = 0;

  // EOI detection required ?
  if (cfg.eoi || detectEoi || (cfg.eor==7))
    readWithEoi = true;    // Use EOI as terminator

  if (cfg.cmode == 2) {   // Controler mode
    // Address device to talk
    // Wait for instrument ready
    // Set GPIB control lines to controller read mode
    addressDevice(cfg.paddr, 1);  
    setControls(CLAS);
  } else {  // Device mode
    // Set GPIB controls to device read mode
    setControls(DLAS);
    readWithEoi = true;  // In device mode we read with EOI by default
  }

  // Ready the data bus
  readyGpibDbus();

  // Perform read of data (r=0: data read OK; r>0: GPIB read error);
  while (r == 0) {
    // Tranbreak > 0 indicates break condition
    if (txBreak)
      break;

    // Read the next character on the GPIB bus
    r = readByte(&bytes[0], readWithEoi, &eoiDetected);

    // If IFC or ATN asserted then break here
    if ((r==1) || (r==2))
      break;

    // If successfully received character
    if (r==0) {
      // Output the character to the serial port
      dataStream.print((char)bytes[0]);
      x++;

      // EOI detection enabled and EOI detected?
      if (readWithEoi) {
        if (eoiDetected)
          break;
      } else {
        // Has a termination sequence been found ?
        if (detectEndByte) {
          if (r == endByte)
            break;
        } else {
          if (isTerminatorDetected(bytes, eor))
            break;
        }
      }

      // Shift last three bytes in memory
      bytes[2] = bytes[1];
      bytes[1] = bytes[0];
    } else {
      // Stop (error or timeout)
      break;
    }
  }

  // Detected that EOI has been asserted
  if (eoiDetected && cfg.eot_en)
    dataStream.print(cfg.eot_ch);
  // Return controller to idle state
  if (cfg.cmode == 2) {
    // Untalk bus and unlisten controller
    unAddressDevice();

    // Set controller back to idle state
    setControls(CIDS);

  } else {
    // Set device back to idle state
    setControls(DIDS);
  }

  // Reset break flag
  txBreak = false;

  return (r > 0); // true == ERR
}


/***** Send a series of characters as data to the GPIB bus *****/
void GPIBbus::sendData(char *data, uint8_t dsize) {
  bool err = false;
  // Controler can unlisten bus and address devices
  if (cfg.cmode == 2) {
    // Set control lines to write data (ATN unasserted)
    setControls(CTAS);
  } else {
    setControls(DTAS);
  }

  // Write the data string
  for (int i = 0; i < dsize; i++) {
    // If EOI asserting is on
    if (cfg.eoi) {
      // Send all characters
      err = writeByte(data[i], NO_EOI);
    } else {
      // Otherwise ignore non-escaped CR, LF and ESC
      if ((data[i] != CR) || (data[i] != LF) || (data[i] != ESC))
        err = writeByte(data[i], NO_EOI);
    }
    if (err)
      break;
  }

//  if (!err  && !dataContinuity) {
  if (!err) {
    // Write terminators according to EOS setting
    // Do we need to write a CR?
    if ((cfg.eos & 0x2) == 0) {
      writeByte(CR, NO_EOI);
    }
    // Do we need to write an LF?
    if ((cfg.eos & 0x1) == 0) {
      writeByte(LF, NO_EOI);
    }
  }

  // If EOI enabled and no more data to follow then assert EOI
//  if (cfg.eoi && !dataContinuity) {
  if (cfg.eoi) {
    setGpibState(SIG_EOI, SIG_EOI, DIRECTION);
    setGpibState(0, SIG_EOI, STATE);
    delayMicroseconds(40);
    setGpibState(SIG_EOI, SIG_EOI, STATE);
  }

  if (cfg.cmode == 2) {   // Controller mode
    // Controller - set lines to idle?
    setControls(CIDS);

  } else  {    // Device mode
    // Set control lines to idle
    setControls(DIDS);
  }
}

/***** Signal to break a GPIB transmission *****/
void GPIBbus::signalBreak(){
  txBreak = true;
}


/***** Control the GPIB bus - set various GPIB states *****/
/*
 * state is a predefined state (CINI, CIDS, CCMS, CLAS, CTAS, DINI, DIDS, DLAS, DTAS);
 * Bits control lines as follows: 8-ATN, 7-SRQ, 6-REN, 5-EOI, 4-DAV, 3-NRFD, 2-NDAC, 1-IFC
 * setGpibState byte1 (databits) : State - 0=LOW, 1=HIGH/INPUT_PULLUP; Direction - 0=input, 1=output;
 * setGpibState byte2 (mask)     : 0=unaffected, 1=enabled
 * setGpibState byte3 (mode)     : 0=set pin state, 1=set pin direction
 */
void GPIBbus::setControls(uint8_t state) {
  // Switch state
  switch (state) {
    // Controller states
    case CINI:  // Initialisation
      setGpibState((SIG_ATN|0      |SIG_REN|SIG_EOI|SIG_DAV), SIG_ALL, DIRECTION);
      setGpibState((SIG_ATN|SIG_SRQ|0      |SIG_EOI|SIG_DAV|SIG_NRFD|SIG_NDAC|SIG_IFC), SIG_ALL, STATE);
      break;

    case CIDS:  // Controller idle state 
      setGpibState((SIG_ATN|SIG_EOI|SIG_DAV|0       |0),
                   (SIG_ATN|SIG_EOI|SIG_DAV|SIG_NRFD|SIG_NDAC),
                   DIRECTION);
      setGpibState((SIG_ATN|SIG_SRQ|SIG_EOI|SIG_DAV|SIG_NRFD|SIG_NDAC|SIG_IFC),
                   (SIG_ATN|0      |SIG_EOI|SIG_DAV|SIG_NRFD|SIG_NDAC|0),
                   STATE);
      break;

    case CCMS:  // Controller active - send commands
      setGpibState((SIG_ATN|SIG_REN|SIG_EOI|SIG_DAV|0       |0       |SIG_IFC), 
                   (SIG_ATN|0      |SIG_EOI|SIG_DAV|SIG_NRFD|SIG_NDAC|SIG_IFC),
                   DIRECTION);
      setGpibState((0      |SIG_SRQ|SIG_EOI|SIG_DAV|SIG_NRFD|SIG_NDAC|SIG_IFC),
                   (SIG_ATN|0      |SIG_EOI|SIG_DAV|SIG_NRFD|SIG_NDAC|SIG_IFC),
                   STATE);
      break;

    case CLAS:  // Controller - read data bus
      // Set state for receiving data
      
      // 166 = 0b10100110
      // 158 = 0b10011110
      setGpibState((SIG_ATN|SIG_REN|0      |0      |SIG_NRFD|SIG_NDAC),
                   (SIG_ATN|0      |SIG_EOI|SIG_DAV|SIG_NRFD|SIG_NDAC),
                   DIRECTION);
               
      // 216 = 0b11011000
      // 158 = 0b10011110
      setGpibState((SIG_ATN|SIG_SRQ|SIG_EOI|SIG_DAV|0       |0),
                   (SIG_ATN|0      |SIG_EOI|SIG_DAV|SIG_NRFD|SIG_NDAC),
                   STATE);
      break;

    case CTAS:  // Controller - write data bus
      setGpibState((SIG_ATN|SIG_REN|SIG_EOI|SIG_DAV|0       |0       |SIG_IFC),
                   (SIG_ATN|0      |SIG_EOI|SIG_DAV|SIG_NRFD|SIG_NDAC|0),
                   DIRECTION);
      setGpibState((SIG_ATN|SIG_SRQ|SIG_EOI|SIG_DAV|SIG_NRFD|SIG_NDAC|SIG_IFC),
                   (SIG_ATN|0      |SIG_EOI|SIG_DAV|SIG_NRFD|SIG_NDAC|0),
                   STATE);
      break;

    /* Bits control lines as follows: 8-ATN, 7-SRQ, 6-REN, 5-EOI, 4-DAV, 3-NRFD, 2-NDAC, 1-IFC */

    // Listener states
    case DINI:  // Listner initialisation
      setGpibState(SIG_NONE, SIG_ALL, DIRECTION);
      setGpibState(SIG_ALL, SIG_ALL, STATE);
      // Set data bus to idle state
      readyGpibDbus();
      break;

    case DIDS:  // Device idle state
      setGpibState(SIG_NONE, (SIG_DAV|SIG_NRFD|SIG_NDAC), DIRECTION);
      setGpibState(SIG_ALL, (SIG_DAV|SIG_NRFD|SIG_NDAC), STATE);
      // Set data bus to idle state
      readyGpibDbus();
      break;

    case DLAS:  // Device listner active (actively listening - can handshake)   
      setGpibState((SIG_NRFD|SIG_NDAC),
                   (SIG_EOI|SIG_DAV|SIG_NRFD|SIG_NDAC),
                   DIRECTION);
      setGpibState((SIG_ALL & ~(SIG_NRFD|SIG_NDAC)),
                   (SIG_EOI|SIG_DAV|SIG_NRFD|SIG_NDAC),
                   STATE);
      break;

    case DTAS:  // Device talker active (sending data)
      setGpibState((SIG_EOI|SIG_DAV),
                   (SIG_EOI|SIG_DAV|SIG_NRFD|SIG_NDAC),
                   DIRECTION);
      setGpibState((SIG_ALL & ~(SIG_NRFD|SIG_NDAC)),
                   (SIG_EOI|SIG_DAV|SIG_NRFD|SIG_NDAC),
                   STATE);
      break;
  }

  // Save state
  cstate = state;

  // GPIB bus delay (to allow state to settle)
//  delayMicroseconds(AR488.tmbus);

}


/***** Set GPI control state using numeric input (xdiag_h) *****/
void GPIBbus::setControlVal(uint8_t value, uint8_t mask, STATE_MODE mode) {
  setGpibState(value, mask, mode);
}


/***** Set GPIB data bus to specific value (xdiag_h) *****/
void GPIBbus::setDataVal(uint8_t value) {
  setGpibDbus(value);
}

/***** Unaddress device *****/
bool GPIBbus::unAddressDevice() {
  // De-bounce
  delayMicroseconds(30);
  // Utalk/unlisten
  if (sendCmd(GC_UNL))
    return ERR;
  if (sendCmd(GC_UNT))
    return ERR;
  // Clear flag
  deviceAddressed = false;
  return OK;
}


/***** Untalk bus then address a device *****/
/*
 * talk: false=listen; true=talk;
 */
bool GPIBbus::addressDevice(uint8_t addr, bool talk) {
  if (sendCmd(GC_UNL))
    return ERR;
  if (talk) {
    // Device to talk, controller to listen
    if (sendCmd(GC_TAD + addr))
      return ERR;
  } else {
    // Device to listen, controller to talk
    if (sendCmd(GC_LAD + addr))
      return ERR;
  }

  // Set flag
  deviceAddressed = true;
  return OK;
}


/***** Returns status of controller device addressing *****/
/*
 * true = device addressed; false = device is not addressed
 */
bool GPIBbus::haveAddressedDevice() {
  return deviceAddressed;
}


/***** Device is addressed to listen? *****/
bool GPIBbus::isDeviceAddressedToListen() {
  return (deviceAddressedState == DLAS);
}


/***** Device is addressed to talk? *****/
bool GPIBbus::isDeviceAddressedToTalk() {
  return (deviceAddressedState == DTAS);
}


/***** Device is not addressed? *****/
bool GPIBbus::isDeviceInIdleState() {
  return (cstate == DIDS);
}


/***** Clear the data bus - set to listen state *****/
void GPIBbus::clearDataBus() {
  readyGpibDbus();
}


/***** Read a SINGLE BYTE of data from the GPIB bus using 3-way handshake *****/
/*
 * (- this function is called in a loop to read data    )
 * (- the GPIB bus must already be configured to listen )
 */
uint8_t GPIBbus::readByte(uint8_t *db, bool readWithEoi, bool *eoi) {

  unsigned long startMillis = millis();
  unsigned long currentMillis = startMillis + 1;
  const unsigned long timeval = cfg.rtmo;
  uint8_t stage = 4;

  bool atnStat = isAsserted(ATN);
  *eoi = false;

  // Wait for interval to expire
  while ( (unsigned long)(currentMillis - startMillis) < timeval ) {
    if (cfg.cmode == 1) {
      // If IFC has been asserted then abort
      if (isAsserted(IFC)) {
        stage = 1;
        break;    
      }

      // ATN unasserted during handshake - not ready yet so abort (and exit ATN loop)
      if ( atnStat && !isAsserted(ATN) ){
        stage = 2;
        break;
      }
    }  

    if (stage == 4) {
      // Unassert NRFD (we are ready for more data)
      setGpibState(SIG_NRFD, SIG_NRFD, STATE);
      stage = 6;
    }

    if (stage == 6) {
      // Wait for DAV to go LOW indicating talker has finished setting data lines..
      if (digitalRead(DAV) == LOW) {
        // Assert NRFD (Busy reading data)
        setGpibState(SIG_NONE, SIG_NRFD, STATE);
        stage = 7;
      }
    }

    if (stage == 7) {
      // Check for EOI signal
      if (readWithEoi && isAsserted(EOI))
        *eoi = true;
      // read from DIO
      *db = readGpibDbus();
      // Unassert NDAC signalling data accepted
      setGpibState(SIG_NDAC, SIG_NDAC, STATE);
      stage = 8;
    }

    if (stage == 8) {
      // Wait for DAV to go HIGH indicating data no longer valid (i.e. transfer complete)
      if (digitalRead(DAV) == HIGH) {
        // Re-assert NDAC - handshake complete, ready to accept data again
        setGpibState(SIG_NONE, SIG_NDAC, STATE);
        stage = 9;
        break;     
      }
    }
    // Increment time
    currentMillis = millis();
  }

  // Completed
  if (stage == 9)
    return 0;

  return stage;

}


uint8_t GPIBbus::writeByte(uint8_t db, bool isLastByte) {
  unsigned long startMillis = millis();
  unsigned long currentMillis = startMillis + 1;
  const unsigned long timeval = cfg.rtmo;
  uint8_t stage = 4;

  // Wait for interval to expire
  while ( (unsigned long)(currentMillis - startMillis) < timeval ) {

    if (cfg.cmode == 1) {
      // If IFC has been asserted then abort
      if (isAsserted(IFC)) {
        setControls(DLAS);       
        stage = 1;
        break;    
      }

      // If ATN has been asserted we need to abort and listen
      if (isAsserted(ATN)) {
        setControls(DLAS);       
        stage = 2;
        break;
      }
    }

    // Wait for NDAC to go LOW (indicating that devices (stage==4) || (stage==8) ) are at attention)
    if (stage == 4) {
      if (digitalRead(NDAC) == LOW)
        stage = 5;
    }

    // Wait for NRFD to go HIGH (indicating that receiver is ready)
    if (stage == 5) {
      if (digitalRead(NRFD) == HIGH)
        stage = 6;
    }

    if (stage == 6){
      // Place data on the bus
      setGpibDbus(db);
      if (cfg.eoi && isLastByte) {
        // If EOI enabled and this is the last byte then assert DAV and EOI
        setGpibState(SIG_NONE, SIG_DAV|SIG_EOI, STATE);
      } else {
        // Assert DAV (data is valid - ready to collect)
        setGpibState(SIG_NONE, SIG_DAV, STATE);
      }
      stage = 7;
    }

    if (stage == 7) {
      // Wait for NRFD to go LOW (receiver accepting data)
      if (digitalRead(NRFD) == LOW)
        stage = 8;
    }

    if (stage == 8) {
      // Wait for NDAC to go HIGH (data accepted)
      if (digitalRead(NDAC) == HIGH) {
        stage = 9;
        break;
      }
    }

    // Increment time
    currentMillis = millis();
  }

  // Handshake complete
  if (stage == 9) {
    if (cfg.eoi && isLastByte) {
      // If EOI enabled and this is the last byte then un-assert both DAV and EOI
      if (cfg.eoi && isLastByte)
        setGpibState(SIG_DAV|SIG_EOI, SIG_DAV|SIG_EOI, STATE);
    } else {
      // Unassert DAV
      setGpibState(SIG_DAV, SIG_DAV, STATE);
    }
    // Reset the data bus
    setGpibDbus(0);
    return 0;
  }
  return stage;
}


/***** ^^^^^^^^^^^^^^^^^^^^^^^^^^^ *****/
/***** GPIB CLASS PUBLIC FUNCTIONS *****/
/***************************************/




/***************************************/
/***** GPIB CLASS PRIVATE FUNCTIONS *****/
/***** ^^^^^^^^^^^^^^^^^^^^^^^^^^^ *****/



/********** PRIVATE FUNCTIONS **********/


/***** Check for terminator *****/
bool GPIBbus::isTerminatorDetected(uint8_t bytes[3], uint8_t eorSequence) {
  // Look for specified terminator (CR+LF by default)
  switch (eorSequence) {
    case 0:
        // CR+LF terminator
        return (bytes[0]==LF && bytes[1]==CR);
    case 1:
        // CR only as terminator
        return (bytes[0]==CR);
    case 2:
        // LF only as terminator
        return (bytes[0]==LF);
    case 3:
        return false;
    case 4:
        // Keithley can use LF+CR instead of CR+LF
        return (bytes[0]==CR && bytes[1]==LF);
    case 5:
        // Solarton (possibly others) can also use ETX (0x03)
        return (bytes[0]==0x03);
    case 6:
        // Solarton (possibly others) can also use CR+LF+ETX (0x03)
        return (bytes[0]==0x03 && bytes[1]==LF && bytes[2]==CR);
    default:
        // Use CR+LF terminator by default
        return (bytes[0]==LF && bytes[1]==CR);
  }
}


/***** Set the SRQ signal *****/
void GPIBbus::setSrqSig() {
  // Set SRQ line to OUTPUT HIGH (asserted)
  setGpibState(SIG_SRQ, SIG_SRQ, DIRECTION);
  setGpibState(SIG_NONE, SIG_SRQ, STATE);
}


/***** Clear the SRQ signal *****/
void GPIBbus::clrSrqSig() {
  // Set SRQ line to INPUT_PULLUP (un-asserted)
  setGpibState(SIG_NONE, SIG_SRQ, DIRECTION);
  setGpibState(SIG_SRQ, SIG_SRQ, STATE);
}


/***** ^^^^^^^^^^^^^^^^^^^^^^^^^^^^ *****/
/***** GPIB CLASS PRIVATE FUNCTIONS *****/
/****************************************/
