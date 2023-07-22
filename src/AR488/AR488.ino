//#pragma GCC diagnostic push
//#pragma GCC diagnostic ignored "-Wtype-limits"
//#pragma GCC diagnostic ignored "-Wunused-variable"

#ifdef __AVR__
  #include <avr/wdt.h>
#endif

//#pragma GCC diagnostic pop

#include "AR488_Config.h"
#include "AR488_GPIBbus.h"
#include "AR488_ComPorts.h"
#include "AR488_Eeprom.h"


/*
#ifdef USE_INTERRUPTS
  #ifdef __AVR__
    #include <avr/interrupt.h>
  #endif
#endif
*/


/***** FWVER "AR488 GPIB controller, ver. 0.51.09, 20/06/2022" *****/
/*
  Arduino IEEE-488 implementation by John Chajecki

  Inspired by the original work of Emanuele Girlando, licensed under a Creative
  Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
  Any code in common with the original work is reproduced here with the explicit
  permission of Emanuele Girlando, who has kindly reviewed and tested this code.

  Thanks also to Luke Mester for comparison testing against the Prologix interface.
  AR488 is Licenced under the GNU Public licence.

  Thanks to 'maxwell3e10' on the EEVblog forum for suggesting additional auto mode
  settings and the macro feature.

  Thanks to 'artag' on the EEVblog forum for providing code for the 32u4.
*/

/*
   Implements most of the CONTROLLER functions;
   Substantially compatible with 'standard' Prologix "++" commands
   (see +savecfg command in the manual for differences)

   Principle of operation:
   - Commands received from USB are buffered and whole terminated lines processed
   - Interface commands prefixed with "++" are passed to the command handler
   - Instrument commands and data not prefixed with '++' are sent directly to the GPIB bus.
   - To receive from the instrument, issue a ++read command or put the controller in auto mode (++auto 1|2)
   - Characters received over the GPIB bus are unbuffered and sent directly to USB
   NOTES:
   - GPIB line in a HIGH state is un-asserted
   - GPIB line in a LOW state is asserted
   - The ATMega processor control pins have a high impedance when set as inputs
   - When set to INPUT_PULLUP, a 10k pull-up (to VCC) resistor is applied to the input
*/

/*
static const char helpText[] PROGMEM = R"EOF(
Standard commands

   ++addr         - display/set device address
   ++auto         - automatically request talk and read response
   ++clr          - send Selected Device Clear to current GPIB address
   ++eoi          - enable/disable assertion of EOI signal
   ++eos          - specify GPIB termination character
   ++eot_enable   - enable/disable appending user specified character to USB
                    output on EOI detection
   ++eot_char     - set character to append to USB output when EOT enabled
   ++ifc          - assert IFC signal for 150 miscoseconds - make AR488
                    controller in charge
   ++llo          - local lockout - disable front panel operation on instrument
   ++loc          - enable front panel operation on instrument
   ++lon          - put controller in listen-only mode (listen to all traffic)
   ++mode         - set the interface mode (0=controller/1=device)
   ++read         - read data from instrument
   ++read_tmo_ms  - read timeout specified between 1 - 3000 milliseconds
   ++rst          - reset the controller
   ++savecfg      - save configration
   ++spoll        - serial poll the addressed host or all instruments
   ++srq          - return status of srq signal (1-asserted/0-not asserted)
   ++status       - set the status byte to be returned on being polled (bit 6 = RQS, i.e SRQ asserted)
   ++trg          - send trigger to selected devices (up to 15 addresses)
   ++ver          - display firmware version

Proprietry commands:

   ++aspoll       - serial poll all instruments (alias: ++spoll all)
   ++default      - set configuration to controller default settings
   ++dcl          - send unaddressed (all) device clear  [power on reset] (is the rst?)
   ++id name      - show/set the name of the interface
   ++id serial    - show/set the serial number of the interface
   ++id verstr    - show/set the version string (replaces setvstr)
   ++idn          - enable/disable reply to *idn? (disabled by default)
   ++ren          - assert or unassert the REN signal
   ++ppoll        - conduct a parallel poll
   ++setvstr      - set custom version string (to identify controller, e.g. "GPIB-USB"). Max 47 chars, excess truncated.
   ++srqauto      - automatically condiuct serial poll when SRQ is asserted
   ++ton          - put controller in talk-only mode (send data only)
   ++verbose      - verbose (human readable) mode
   ++xonxoff
)EOF";
*/

/*
   NOT YET IMPLEMENTED
   ++myaddr   - set the controller address
*/

/*
   For information regarding the GPIB firmware by Emanualle Girlando see:
   http://egirland.blogspot.com/2014/03/arduino-uno-as-usb-to-gpib-controller.html
*/


/*
   Pin mapping between the Arduino pins and the GPIB connector.
   NOTE:
   GPIB pins 10 and 18-24 are connected to GND
   GPIB pin 12 should be connected to the cable shield (might be n/c)
   Pin mapping follows the layout originally used by Emanuelle Girlando, but adds
   the SRQ line (GPIB 10) on pin 2 and the REN line (GPIB 17) on pin 13. The program
   should therefore be compatible with the original interface design but for full
   functionality will need the remaining two pins to be connected.
   For further information about the AR488 see the AR488 Manual. 
*/


/*******************************/
/***** SERIAL PARSE BUFFER *****/
/***** vvvvvvvvvvvvvvvvvvv *****/
/*
 * Note: Ardiono serial input buffer is 64 
 */
// Serial input parsing buffer
static const uint8_t PBSIZE = 128;
char pBuf[PBSIZE];
uint8_t pbPtr = 0;

/***** ^^^^^^^^^^^^^^^^^^^ *****/
/***** SERIAL PARSE BUFFER *****/
/*******************************/


/*************************/
/***** HELP MESSAGES *****/
/****** vvvvvvvvvvvv *****/

static const char cmdHelp[] PROGMEM = {
  "addr:P Display/set device address\n"
  "auto:P Automatically request talk and read response\n"
  "clr:P Send Selected Device Clear to current GPIB address\n"
  "eoi:P Enable/disable assertion of EOI signal\n"
  "eor:P Show or set end of receive character(s)\n"
  "eos:P Specify GPIB termination character\n"
  "eot_char:P Set character to append to USB output when EOT enabled\n"
  "eot_enable:P Enable/Disable appending user specified character to USB output on EOI detection\n"
  "help:P This message\n"
  "ifc:P Assert IFC signal for 150 miscoseconds - make AR488 controller in charge\n"
  "llo:P Local lockout - disable front panel operation on instrument\n"
  "loc:P Enable front panel operation on instrument\n"
  "lon:P Put controller in listen-only mode (listen to all traffic)\n"
  "mode:P Set the interface mode (0=controller/1=device)\n"
  "read:P Read data from instrument\n"
  "read_tmo_ms:P Read timeout specified between 1 - 3000 milliseconds\n"
  "rst:P Reset the controller\n"
  "savecfg:P Save configration\n"
  "spoll:P Serial poll the addressed host or all instruments\n"
  "srq:P Return status of srq signal (1-srq asserted/0-srq not asserted)\n"
  "status:P Set the status byte to be returned on being polled (bit 6 = RQS, i.e SRQ asserted)\n"
  "trg:P Send trigger to selected devices (up to 15 addresses)\n"
  "ver:P Display firmware version\n"
  "aspoll:C Serial poll all instruments (alias: ++spoll all)\n"
  "dcl:C Send unaddressed (all) device clear  [power on reset] (is the rst?)\n"
  "default:C Set configuration to controller default settings\n"
  "id:C Show interface ID information - see also: 'id name'; 'id serial'; 'id verstr'\n"
  "id name:C Show/Set the name of the interface\n"
  "id serial:C Show/Set the serial number of the interface\n"
  "id verstr:C Show/Set the version string sent in reply to ++ver e.g. \"GPIB-USB\"). Max 47 chars, excess truncated.\n"
  "idn:C Enable/Disable reply to *idn? (disabled by default)\n"
  "ppoll:C Conduct a parallel poll\n"
  "ren:C Assert or Unassert the REN signal\n"
  "repeat:C Repeat a given command and return result\n"
  "setvstr:C DEPRECATED - see id verstr\n"
  "srqauto:C Automatically condiuct serial poll when SRQ is asserted\n"
  "ton:C Put controller in talk-only mode (send data only)\n"
  "verbose:C Verbose (human readable) mode\n"
  "tmbus:C Timing parameters (see the doc)\n"
  "xdiag:C Bus diagnostics (see the doc)\n"
};

/************************************/
/***** COMMON VARIABLES SECTION *****/
/***** vvvvvvvvvvvvvvvvvvvvvvvv *****/

/****** Process status values *****/
#define OK 0
#define ERR 1

/***** Control characters *****/
#define ESC  0x1B   // the USB escape char
#define CR   0xD    // Carriage return
#define LF   0xA    // Newline/linefeed
#define PLUS 0x2B   // '+' character

/****** Global variables with volatile values related to controller state *****/

// GPIB bus object
GPIBbus gpibBus;

// GPIB control state
//uint8_t cstate = 0;

// Verbose mode
bool isVerb = false;

// CR/LF terminated line ready to process
uint8_t lnRdy = 0;      

// GPIB data receive flags
bool autoRead = false;              // Auto reading (auto mode 3) GPIB data in progress
bool readWithEoi = false;           // Read eoi requested
bool readWithEndByte = false;       // Read with specified terminator character
bool isQuery = false;               // Direct instrument command is a query
uint8_t tranBrk = 0;                // Transmission break on 1=++, 2=EOI, 3=ATN 4=UNL
uint8_t endByte = 0;                // Termination character

// Device mode - send data
//bool snd = false;

// Escaped character flag
bool isEsc = false;           // Charcter escaped
bool isPlusEscaped = false;   // Plus escaped

// Read only mode flag
bool isRO = false;

// Talk only mode flag
bool isTO = false;

// GPIB command parser
bool aTt = false;
bool aTl = false;

// Data send mode flags
bool dataBufferFull = false;    // Flag when parse buffer is full

// State flags set by interrupt being triggered
//extern volatile bool isATN;  // has ATN been asserted?
//extern volatile bool isSRQ;  // has SRQ been asserted?

// SRQ auto mode
bool isSrqa = false;

// Interrupt without handler fired
//volatile bool isBAD = false;

// Send response to *idn?
bool sendIdn = false;

// Xon/Xoff flag (off by default)
//bool xonxoff = false;

/***** ^^^^^^^^^^^^^^^^^^^^^^^^ *****/
/***** COMMON VARIABLES SECTION *****/
/************************************/



/*******************************/
/***** COMMON CODE SECTION *****/
/***** vvvvvvvvvvvvvvvvvvv *****/


/******  Arduino standard SETUP procedure *****/
void setup() {

  // Disable the watchdog (needed to prevent WDT reset loop)
#ifdef __AVR__
  wdt_disable();
#endif

  // Turn off internal LED (set OUPTUT/LOW)
#ifdef LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
#endif

// Using AVR board with PCINT interrupts
/*
#ifdef USE_INTERRUPTS
  // Turn on interrupts on port
  interruptsEn();
#endif
*/

  // Initialise parse buffer
  flushPbuf();

  // Initialise serial at the configured baud rate
  AR_SERIAL_PORT.begin(AR_SERIAL_SPEED);

// Un-comment for diagnostic purposes
/* 
  #if defined(__AVR_ATmega32U4__)
    while(!*arSerial)
    ;
//    Serial.print(F("Starting "));
    for(int i = 0; i < 20; ++i) {  // this gives you 10 seconds to start programming before it crashes
      Serial.print(".");
      delay(500);
    }
    Serial.println("@>");
  #endif // __AVR_ATmega32U4__
*/
// Un-comment for diagnostic purposes

  // Initialise
//  initAR488();

#ifdef E2END
  // Read data from non-volatile memory
  if (!isEepromClear()) {
    if (!epReadData(gpibBus.cfg.db, GPIB_CFG_SIZE)) {
      epErase();
      gpibBus.setDefaultCfg();
      epWriteData(gpibBus.cfg.db, GPIB_CFG_SIZE);
    }
  }
#endif
  // Start the interface in the configured mode
  gpibBus.begin();
  dataPort.flush();
}
/****** End of Arduino standard SETUP procedure *****/


/***** ARDUINO MAIN LOOP *****/
void loop() {

  bool errFlg = false; 

/*** Process the buffer ***/
/* Each received char is passed through parser until an un-escaped 
 * CR is encountered. If we have a command then parse and execute.
 * If the line is data (inclding direct instrument commands) then
 * send it to the instrument.
 * NOTE: parseInput() sets lnRdy in serialEvent, readBreak or in the
 * above loop
 * lnRdy=1: process command;
 * lnRdy=2: send data to Gpib
 */

/*
if (lnRdy>0){
  dataPort.print(F("lnRdy: "));
  dataPort.println(lnRdy);
}
*/

  // lnRdy=1: received a command so execute it...
  if (lnRdy == 1) {
    if (autoRead) {
      // Issuing any command stops autoread mode
      autoRead = false;
      gpibBus.unAddressDevice();
    }
    execCmd(pBuf, pbPtr);
  }

  // Controller mode:
  if (gpibBus.isController()) {
    // lnRdy=2: received data - send it to the instrument...
    if (lnRdy == 2) {
      sendToInstrument(pBuf, pbPtr);
      // Auto-read data from GPIB bus following any command
      if (gpibBus.cfg.amode == 1) {
        //        delay(10);
        errFlg = gpibBus.receiveData(dataPort, gpibBus.cfg.eoi, false, 0);
      }
      // Auto-receive data from GPIB bus following a query command
      if (gpibBus.cfg.amode == 2 && isQuery) {
        //        delay(10);
        errFlg = gpibBus.receiveData(dataPort, gpibBus.cfg.eoi, false, 0);
        isQuery = false;
      }
    }

    // Check status of SRQ and SPOLL if asserted
//    if (isSRQ && isSrqa) {
/*
    if (gpibBus.isAsserted(SRQ) && isSrqa) {
      spoll_h(NULL);
//      isSRQ = false;
    }
*/
    // Automatic serial poll?
    if (isSrqa) {
      if (gpibBus.isAsserted(SRQ))
        spoll_h(NULL);
    }

    // Continuous auto-receive data from GPIB bus
    if ((gpibBus.cfg.amode==3) && autoRead) {
      // Nothing is waiting on the serial input so read data from GPIB
      if (lnRdy==0) {
        errFlg = gpibBus.receiveData(dataPort, readWithEoi, readWithEndByte, endByte);
      }
/*      
      else{
        // Otherwise clear auto-read flag and unaddress device
        autoReading = false;
        gpibBus.unAddressDevice();
      }
*/
    }

    // Did we get an error during read?
    if (errFlg && isVerb) {
      dataPort.println(F("Error while receiving data."));
      errFlg = false;
    }
  }

  // Device mode:
  if (gpibBus.isController()==false) {
    if (isTO) {
      if (lnRdy == 2)
        sendToInstrument(pBuf, pbPtr);
    } else if (isRO) {
      lonMode();
    } else if (gpibBus.isAsserted(ATN)) {
//      dataPort.println(F("Attention signal detected"));
      attnRequired();
//      dataPort.println(F("ATN loop finished"));
    }
/*    
    else{
      if (lnRdy == 2) sendToInstrument(pBuf, pbPtr);
    }
*/
  }

  // Reset line ready flag
//  lnRdy = 0;

  // IDN query ?
  if (sendIdn) {
    if (gpibBus.cfg.idn==1)
      dataPort.println(gpibBus.cfg.sname);
    else if (gpibBus.cfg.idn==2) {
      dataPort.print(gpibBus.cfg.sname);
      dataPort.print("-");
      dataPort.println(gpibBus.cfg.serial);
    }
    sendIdn = false;
  }

  // If charaters waiting in the serial input buffer then call handler
  if (dataPort.available())
    lnRdy = serialIn_h();

  delayMicroseconds(5);
}
/***** END MAIN LOOP *****/


/***** Initialise the interface *****/
/*
void initAR488() {
  // Set default values ({'\0'} sets version string array to null)
  gpibBus.cfg = {false, false, 2, 0, 1, 0, 0, 0, 0, 1200, 0, {'\0'}, 0, {'\0'}, 0, 0};
}
*/

/***** Initialise device mode *****/
void initDevice() {
  gpibBus.stop();
  gpibBus.startDeviceMode();
}


/***** Initialise controller mode *****/
void initController() {
  gpibBus.stop();
  gpibBus.startControllerMode();
}


/***** Serial event handler *****/
/*
 * Note: the Arduino serial buffer is 64 characters long. Characters are stored in
 * this buffer until serialEvent_h() is called. parsedInput() takes a character at 
 * a time and places it into the 256 character parse buffer whereupon it is parsed
 * to determine whether a command or data are present.
 * lnRdy=0: terminator not detected yet
 * lnRdy=1: terminator detected, sequence in parse buffer is a ++ command
 * lnRdy=2: terminator detected, sequence in parse buffer is data or direct instrument command
 */ 
uint8_t serialIn_h() {
  uint8_t bufferStatus = 0;
  // Parse serial input until we have detected a line terminator
  while (dataPort.available() && bufferStatus==0) {   // Parse while characters available and line is not complete
    bufferStatus = parseInput(dataPort.read());
  }
  return bufferStatus;
}


/***** Detect pin state *****/
/*
 * When interrupts are being used the pin state is automatically flagged 
 * when an interrupt is triggered. Where interrupts cannot be used, the
 * state of the pin is read.
 * 
 * Read pin state using digitalRead for Arduino pins or getGpibPinState
 * for MCP23S17 pins.
 */
/* 
bool isAsserted(uint8_t gpibsig) {
#if defined(AR488_MCP23S17) || defined(AR488_MCP23017)
  // Use MCP function to get MCP23S17 or MCP23017 pin state.
  // If interrupt flagged then update mcpPinAssertedReg register
  if (mcpIntA){
//dataPort.println(F("Interrupt flagged - pin state checked"));
    // Clear mcpIntA flag
    mcpIntA = false;
    // Get inverse of pin status at interrupt (0 = true [asserted]; 1 = false [unasserted])
    mcpPinAssertedReg = ~getMcpIntAPinState();
//dataPort.print(F("mcpPinAssertedReg: "));
//dataPort.println(mcpPinAssertedReg, BIN);
  }
  return (mcpPinAssertedReg & (1<<gpibsig));
#else
  // Use digitalRead function to get current Arduino pin state
  return (digitalRead(gpibsig) == LOW) ? true : false;
#endif
}
*/

/*************************************/
/***** Device operation routines *****/
/*************************************/

/***** Verbose println *****/
#define verb_println(p) if (isVerb) dataPort.println(p)
#define verb_print(p) if (isVerb) dataPort.print(p)

/***** Unrecognized command *****/
void errBadCmd() {
  dataPort.println(F("Unrecognized command"));
}


/***** Add character to the buffer and parse *****/
uint8_t parseInput(char c) {
  uint8_t r = 0;
  // Read until buffer full
  if (pbPtr < PBSIZE) {
    if (isVerb && c != LF)
      dataPort.print(c);
    // Actions on specific characters
    switch (c) {
      // Carriage return or newline? Then process the line
      case CR:
      case LF:
        // If escaped just add to buffer
        if (isEsc) {
          addPbuf(c);
          isEsc = false;
        } else {
          // Carriage return on blank line?
          // Note: for data CR and LF will always be escaped
          if (pbPtr == 0) {
            flushPbuf();
            if (isVerb) {
              dataPort.println();
              showPrompt();
            }
            return 0;
          } else {
//            if (isVerb) dataPort.println();  // Move to new line
            // Buffer starts with ++ and contains at least 3 characters - command?
            if (pbPtr>2 && isCmd(pBuf) && !isPlusEscaped) {
              // Exclamation mark (break read loop command)
              if (pBuf[2]==0x21) {
                r = 3;
                flushPbuf();
              // Otherwise flag command received and ready to process 
              } else {
                r = 1;
              }
            // Buffer contains *idn? query and interface to respond
            } else if (pbPtr>3 && gpibBus.cfg.idn>0 && isIdnQuery(pBuf)) {
              sendIdn = true;
              flushPbuf();
            // Buffer has at least 1 character = instrument data to send to gpib bus
            }else if (pbPtr > 0) {
              r = 2;
            }
            isPlusEscaped = false;
//            return r;
          }
        }
        break;
      case ESC:
        // Handle the escape character
        if (isEsc) {
          // Add character to buffer and cancel escape
          addPbuf(c);
          isEsc = false;
        } else {
          // Set escape flag
          isEsc  = true;  // Set escape flag
        }
        break;
      case PLUS:
        if (isEsc) {
          isEsc = false;
          if (pbPtr < 2) isPlusEscaped = true;
        }
        addPbuf(c);
//        if (isVerb) dataPort.print(c);
        break;
      // Something else?
      default: // any char other than defined above
//        if (isVerb) dataPort.print(c);  // Humans like to see what they are typing...
        // Buffer contains '++' (start of command). Stop sending data to serial port by halting GPIB receive.
        addPbuf(c);
        isEsc = false;
    }
  }
  if (pbPtr >= PBSIZE) {
    if (isCmd(pBuf) && !r) {  // Command without terminator and buffer full
      verb_println(F("ERROR - Command buffer overflow!"));
      flushPbuf();
    }else{  // Buffer contains data and is full, so process the buffer (send data via GPIB)
      dataBufferFull = true;
      // Signal to GPIB object that more data will follow (suppress GPIB addressing)
//      gpibBus.setDataContinuity(true);
      r = 2;
    }
  }
  return r;
}


/***** Is this a command? *****/
bool isCmd(char *buffr) {
  return (buffr[0] == PLUS && buffr[1] == PLUS);
}


/***** Is this an *idn? query? *****/
bool isIdnQuery(char *buffr) {
  // Check for upper or lower case *idn?
  return (strncasecmp(buffr, "*idn?", 5) == 0);
}


/***** ++read command detected? *****/
bool isRead(char *buffr) {
  char cmd[4];
  // Copy 2nd to 5th character
  for (int i = 2; i < 6; i++) {
    cmd[i - 2] = buffr[i];
  }
  // Compare with 'read'
  return (strncmp(cmd, "read", 4) == 0);
}


/***** Add character to the buffer *****/
void addPbuf(char c) {
  pBuf[pbPtr] = c;
  pbPtr++;
}


/***** Clear the parse buffer *****/
void flushPbuf() {
  memset(pBuf, '\0', PBSIZE);
  pbPtr = 0;
}


/***** Comand function record *****/
struct cmdRec { 
  const char* token; 
  int opmode;
  void (*handler)(char *);
};


/***** Array containing index of accepted ++ commands *****/
/*
 * Commands without parameters require casting to a pointer
 * requiring a char* parameter. The functon is called with
 * NULL by the command processor.
 * 
 * Format: token, mode, function_ptr
 * Mode: 1=device; 2=controller; 3=both; 
 */
static cmdRec cmdHidx [] = { 
  { "addr",        3, addr_h      }, 
  { "allspoll",    2, (void(*)(char*)) aspoll_h  },
  { "auto",        2, amode_h     },
  { "clr",         2, (void(*)(char*)) clr_h     },
  { "dcl",         2, (void(*)(char*)) dcl_h     },
  { "default",     3, (void(*)(char*)) default_h },
  { "eoi",         3, eoi_h       },
  { "eor",         3, eor_h       },
  { "eos",         3, eos_h       },
  { "eot_char",    3, eot_char_h  },
  { "eot_enable",  3, eot_en_h    },
  { "help",        3, help_h      },
  { "ifc",         2, (void(*)(char*)) ifc_h     },
  { "id",          3, id_h        },
  { "idn",         3, idn_h       },
  { "llo",         2, llo_h       },
  { "loc",         2, loc_h       },
  { "lon",         1, lon_h       },
  { "mla",         2, (void(*)(char*)) sendmla_h },
  { "mode" ,       3, cmode_h     },
  { "msa",         2, sendmsa_h   },
  { "mta",         2, (void(*)(char*)) sendmta_h },
  { "ppoll",       2, (void(*)(char*)) ppoll_h   },
  { "read",        2, read_h      },
  { "read_tmo_ms", 2, rtmo_h      },
  { "ren",         2, ren_h       },
  { "repeat",      2, repeat_h    },
  { "rst",         3, (void(*)(char*)) rst_h     },
  { "trg",         2, trg_h       },
  { "savecfg",     3, (void(*)(char*)) save_h    },
  { "setvstr",     3, setvstr_h   },
  { "spoll",       2, spoll_h     },
  { "srq",         2, (void(*)(char*)) srq_h     },
  { "srqauto",     2, srqa_h      },
  { "status",      1, stat_h      },
  { "ton",         1, ton_h       },
  { "unl",         2, (void(*)(char*)) unlisten_h  },
  { "unt",         2, (void(*)(char*)) untalk_h    },
  { "ver",         3, ver_h       },
  { "verbose",     3, (void(*)(char*)) verb_h    },
  { "xdiag",       3, xdiag_h     }
};


/***** Show a prompt *****/
void showPrompt() {
  // Print a prompt
  dataPort.print("> ");
}


/****** Send data to instrument *****/
/* Processes the parse buffer whenever a full CR or LF
 * and sends data to the instrument
 */
void sendToInstrument(char *buffr, uint8_t dsize) {
  // Is this an instrument query command (string ending with ?)
  if (buffr[dsize-1] == '?')
    isQuery = true;

  // Has controller already addressed the device? - if not then address it
  if (!gpibBus.haveAddressedDevice())
    gpibBus.addressDevice(gpibBus.cfg.paddr, LISTEN);

  // Send string to instrument
  gpibBus.sendData(buffr, dsize);

  // Address device
  if (dataBufferFull) {
    dataBufferFull = false;
  } else {
    gpibBus.unAddressDevice();
  }

  // Show a prompt on completion?
  if (isVerb)
    showPrompt();

  // Flush the parse buffer
  flushPbuf();
  lnRdy = 0;
}


/***** Execute a command *****/
void execCmd(char *buffr, uint8_t dsize) {
  char line[PBSIZE];

  // Copy collected chars to line buffer
  memcpy(line, buffr, dsize);

  // Flush the parse buffer
  flushPbuf();
  lnRdy = 0;

  // Its a ++command so shift everything two bytes left (ignore ++) and parse
  for (int i = 0; i < dsize-2; i++) {
    line[i] = line[i + 2];
  }
  // Replace last two bytes with a null (\0) character
  line[dsize - 2] = '\0';
  line[dsize - 1] = '\0';
  // Execute the command
  if (isVerb)
    dataPort.println(); // Shift output to next line
  getCmd(line);

  // Show a prompt on completion?
  if (isVerb)
    showPrompt();
}


/***** Extract command and pass to handler *****/
void getCmd(char *buffr) {
  char *token;  // Pointer to command token
  char *params; // Pointer to parameters (remaining buffer characters)
  
  int casize = sizeof(cmdHidx) / sizeof(cmdHidx[0]);
  int i = 0;

  // If terminator on blank line then return immediately without processing anything 
  switch (buffr[0]) {
    case 0:
    case CR:
    case LF:
      return;
  }

  // Get the first token
  token = strtok(buffr, " \t");

  // Check whether it is a valid command token
  i = 0;
  do {
    if (strcasecmp(cmdHidx[i].token, token) == 0)
      break;
    i++;
  } while (i < casize);

  if (i < casize) {
    // We have found a valid command and handler
    // If command is relevant to mode then execute it
    if (cmdHidx[i].opmode & gpibBus.cfg.cmode) {
      // If its a command with parameters
      // Copy command parameters to params and call handler with parameters
      params = token + strlen(token) + 1;
  
      // If command parameters were specified
      if (strlen(params) > 0) {
        // Call handler with parameters specified
        cmdHidx[i].handler(params);
      } else {
        // Call handler without parameters
        cmdHidx[i].handler(NULL);
      }
    } else {
      errBadCmd();
      verb_println(F("getCmd: command not available in this mode."));
    }
  } else {
    // No valid command found
    errBadCmd();
  }
 
}

/***** Check whether a parameter is in range *****/
/* Convert string to integer and check whether value is within
 * lowl to higl inclusive. Also returns converted text in param
 * to a uint16_t integer in rval. Returns true if successful, 
 * false if not
*/
bool notInRange(char *param, uint16_t lowl, uint16_t higl, uint16_t &rval) {
  // Null string passed?
  if (strlen(param) == 0)
    return true;

  // Convert to integer
  rval = 0;
  rval = atoi(param);

  // Check range
  if (rval < lowl || rval > higl) {
    errBadCmd();
    if (isVerb) {
      dataPort.print(F("Valid range is between "));
      dataPort.print(lowl);
      dataPort.print(F(" and "));
      dataPort.println(higl);
    }
    return true;
  }
  return false;
}


/*************************************/
/***** STANDARD COMMAND HANDLERS *****/
/*************************************/

/***** Show or change device address *****/
void addr_h(char *params) {
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 1, 30, val))
      return;
    if (val == gpibBus.cfg.caddr) {
      errBadCmd();
      verb_println(F("That is my address! Address of a remote device is required."));
      return;
    }
    gpibBus.cfg.paddr = val;
    verb_print(F("Set device primary address to: "));
    verb_println(val);
  } else {
    dataPort.println(gpibBus.cfg.paddr);
  }
}


/***** Show or set read timout *****/
void rtmo_h(char *params) {
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 1, 32000, val))
      return;
    gpibBus.cfg.rtmo = val;
    verb_print(F("Set [read_tmo_ms] to: "));
    verb_print(val);
    verb_println(F(" milliseconds"));
  } else {
    dataPort.println(gpibBus.cfg.rtmo);
  }
}


/***** Show or set end of send character *****/
void eos_h(char *params) {
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 0, 3, val))
      return;
    gpibBus.cfg.eos = (uint8_t)val;
    verb_print(F("Set EOS to: "));
    verb_println(val);
  } else {
    dataPort.println(gpibBus.cfg.eos);
  }
}


/***** Show or set EOI assertion on/off *****/
void eoi_h(char *params) {
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 0, 1, val))
      return;
    gpibBus.cfg.eoi = val ? true : false;
    verb_print(F("Set EOI assertion: "));
    verb_println(val ? "ON" : "OFF");
  } else {
    dataPort.println(gpibBus.cfg.eoi);
  }
}


/***** Show or set interface to controller/device mode *****/
void cmode_h(char *params) {
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 0, 1, val))
      return;
    switch (val) {
      case 0:
        gpibBus.startDeviceMode();
        break;
      case 1:
        gpibBus.startControllerMode();
        break;
    }
    verb_print(F("Interface mode set to: "));
    verb_println(val ? "CONTROLLER" : "DEVICE");
  } else {
    dataPort.println(gpibBus.isController());
  }
}


/***** Show or enable/disable sending of end of transmission character *****/
void eot_en_h(char *params) {
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 0, 1, val))
      return;
    gpibBus.cfg.eot_en = val ? true : false;
      verb_print(F("Appending of EOT character: "));
      verb_println(val ? "ON" : "OFF");
  } else {
    dataPort.println(gpibBus.cfg.eot_en);
  }
}


/***** Show or set end of transmission character *****/
void eot_char_h(char *params) {
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 0, 255, val))
      return;
    gpibBus.cfg.eot_ch = (uint8_t)val;
    if (isVerb) {
      dataPort.print(F("EOT set to ASCII character: "));
      dataPort.println(val);
    };
  } else {
    dataPort.println(gpibBus.cfg.eot_ch, DEC);
  }
}


/***** Show or enable/disable auto mode *****/
void amode_h(char *params) {
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 0, 3, val))
      return;
    if (val > 0 && isVerb) {
      dataPort.println(F("WARNING: automode ON can cause some devices to generate"));
      dataPort.println(F("         'addressed to talk but nothing to say' errors"));
    }
    gpibBus.cfg.amode = (uint8_t)val;
    if (gpibBus.cfg.amode < 3)
      autoRead = false;
    if (isVerb) {
      dataPort.print(F("Auto mode: "));
      dataPort.println(gpibBus.cfg.amode);
    }
  } else {
    dataPort.println(gpibBus.cfg.amode);
  }
}


/***** Display the controller version string *****/
void ver_h(char *params) {
  // If "real" requested
  if (params != NULL && strncasecmp(params, "real", 3) == 0) {
    dataPort.println(F(FWVER));
    // Otherwise depends on whether we have a custom string set
  } else {
    if (strlen(gpibBus.cfg.vstr) > 0) {
      dataPort.println(gpibBus.cfg.vstr);
    } else {
      dataPort.println(F(FWVER));
    }
  }
}


/***** Address device to talk and read the sent data *****/
void read_h(char *params) {
  // Clear read flags
  readWithEoi = false;
  readWithEndByte = false;
  endByte = 0;
  // Read any parameters
  if (params != NULL) {
    if (strlen(params) > 3) {
      verb_println(F("Invalid parameter - ignored!"));
    } else if (strncasecmp(params, "eoi", 3) == 0) { // Read with eoi detection
      readWithEoi = true;
    } else { // Assume ASCII character given and convert to an 8 bit byte
      readWithEndByte = true;
      endByte = atoi(params);
    }
  }
  if (gpibBus.cfg.amode == 3) {
    // In auto continuous mode we set this flag to indicate we are ready for continuous read
    autoRead = true;
  } else {
    // If auto mode is disabled we do a single read
    gpibBus.addressDevice(gpibBus.cfg.paddr, TALK);
    gpibBus.receiveData(dataPort, readWithEoi, readWithEndByte, endByte);
  }
}


/***** Send device clear (usually resets the device to power on state) *****/
void clr_h() {
  if (gpibBus.sendSDC())  {
    verb_println(F("Failed to send SDC"));
    return;
  }
  // Set GPIB controls back to idle state
  gpibBus.setControls(CIDS);
}


/***** Send local lockout command *****/
void llo_h(char *params) {
  // NOTE: REN *MUST* be asserted (LOW)
  if (digitalRead(REN)==LOW) {
    // For 'all' send LLO to the bus without addressing any device
    // Devices will show REM as soon as they are addressed and need to be released with LOC
    if (params != NULL) {
      if (0 == strncmp(params, "all", 3)) {
        if (gpibBus.sendCmd(GC_LLO)) {
          verb_println(F("Failed to send universal LLO."));
        }
      }
    } else {
      // Send LLO to currently addressed device
      if (gpibBus.sendLLO()){
        verb_println(F("Failed to send LLO!"));
      }
    }
  }
  // Set GPIB controls back to idle state
  gpibBus.setControls(CIDS);
}


/***** Send Go To Local (GTL) command *****/
void loc_h(char *params) {
  // REN *MUST* be asserted (LOW)
  if (digitalRead(REN) == LOW) {
    if (params != NULL) {
      if (strncmp(params, "all", 3) == 0) {
        // Send request to clear all devices to local
        gpibBus.sendAllClear();
      }
    } else {
      // Send GTL to addressed device
      if (gpibBus.sendGTL()) {
        verb_println(F("Failed to send LOC!"));
      }
      // Set GPIB controls back to idle state
      gpibBus.setControls(CIDS);
    }
  }
}


/***** Assert IFC for 150 microseconds *****/
/* This indicates that the AR488 the Controller-in-Charge on
 * the bus and causes all interfaces to return to their idle 
 * state
 */
void ifc_h() {
  if (gpibBus.cfg.cmode==2) {
    // Assert IFC
    gpibBus.setControlVal(0b00000000, 0b00000001, STATE);
    delayMicroseconds(150);
    // De-assert IFC
    gpibBus.setControlVal(0b00000001, 0b00000001, STATE);
    verb_println(F("IFC signal asserted for 150 microseconds"));
  }
}


/***** Send a trigger command *****/
void trg_h(char *params) {
  char *param;
  uint8_t addrs[15] = {0};
  uint16_t val = 0;
  uint8_t cnt = 0;

  addrs[0] = addrs[0]; // Meaningless as both are zero but defaults compiler warning!

  // Read parameters
  if (params == NULL) {
    // No parameters - trigger addressed device only
    addrs[0] = gpibBus.cfg.paddr;
    cnt++;
  } else {
    // Read address parameters into array
    while (cnt < 15) {
      if (cnt == 0) {
        param = strtok(params, " \t");
      } else {
        param = strtok(NULL, " \t");
      }
      if (param == NULL) {
        break;  // Stop when there are no more parameters
      } else {    
        if (notInRange(param, 1, 30, val))
          return;
        addrs[cnt] = (uint8_t)val;
        cnt++;
      }
    }
  }

  // If we have some addresses to trigger....
  if (cnt > 0) {
    for (int i = 0; i < cnt; i++) {
      // Sent GET to the requested device
      if (gpibBus.sendGET(addrs[i]))  {
        verb_println(F("Failed to trigger device!"));
        return;
      }
    }

    // Set GPIB controls back to idle state
    gpibBus.setControls(CIDS);

    verb_println(F("Group trigger completed."));
  }
}


/***** Reset the controller *****/
/*
 * Arduinos can use the watchdog timer to reset the MCU
 * For other devices, we restart the program instead by
 * jumping to address 0x0000. This is not a hardware reset
 * and will not reset a crashed MCU, but it will re-start
 * the interface program and re-initialise all parameters. 
 */
void rst_h() {
  unsigned long tout;
  tout = millis() + 2000;
  wdt_enable(WDTO_1S);
  while (millis() < tout)
    {}
  // Should never reach here....
  verb_println(F("Reset FAILED."));
}


/***** Serial Poll Handler *****/
void spoll_h(char *params) {
  char *param;
  uint8_t addrs[15];
  uint8_t sb = 0;
  uint8_t r;
  //  uint8_t i = 0;
  uint8_t j = 0;
  uint16_t val = 0;
  bool all = false;
  bool eoiDetected = false;

  // Initialise address array
  for (int i = 0; i < 15; i++) {
    addrs[i] = 0;
  }

  // Read parameters
  if (params == NULL) {
    // No parameters - trigger addressed device only
    addrs[0] = gpibBus.cfg.paddr;
    j = 1;
  } else {
    // Read address parameters into array
    while (j < 15) {
      if (j == 0) {
        param = strtok(params, " \t");
      } else {
        param = strtok(NULL, " \t");
      }
      // The 'all' parameter given?
      if (strncmp(param, "all", 3) == 0) {
        all = true;
        j = 30;
        verb_println(F("Serial poll of all devices requested..."));
        break;
        // Read all address parameters
      } else if (strlen(params) < 3) { // No more than 2 characters
        if (notInRange(param, 1, 30, val))
          return;
        addrs[j] = (uint8_t)val;
        j++;
      } else {
        errBadCmd();
        verb_println(F("Invalid parameter"));
        return;
      }
    }
  }

  // Send Unlisten [UNL] to all devices
  if (gpibBus.sendCmd(GC_UNL))
    return;

  // Controller addresses itself as listner
  if (gpibBus.sendCmd(GC_LAD + gpibBus.cfg.caddr))
    return;

  // Send Serial Poll Enable [SPE] to all devices
  if (gpibBus.sendCmd(GC_SPE))
    return;

  // Poll GPIB address or addresses as set by i and j
  for (int i = 0; i < j; i++) {
    // Set GPIB address in val
    if (all) {
      val = i;
    } else {
      val = addrs[i];
    }

    // Don't need to poll own address
    if (val != gpibBus.cfg.caddr) {
      // Address a device to talk
      if ( gpibBus.sendCmd(GC_TAD + val) )
        return;

      // Set GPIB control to controller active listner state (ATN unasserted)
      gpibBus.setControls(CLAS);

      // Read the response byte (usually device status) using handshake - suppress EOI detection
      r = gpibBus.readByte(&sb, false, &eoiDetected);

      // If we successfully read a byte
      if (!r) {
        if (j > 1) {
          // If all, return specially formatted response: SRQ:addr,status
          // but only when RQS bit set
          if (sb & 0x40) {
            dataPort.print(F("SRQ:"));
            dataPort.print(i);
            dataPort.print(F(","));
            dataPort.println(sb, DEC);
            i = j;
          }
        } else {
          // Return decimal number representing status byte
          dataPort.println(sb, DEC);
          if (isVerb) {
            dataPort.print(F("Received status byte ["));
            dataPort.print(sb);
            dataPort.print(F("] from device at address: "));
            dataPort.println(val);
          }
          i = j;
        }
      } else {
        verb_println(F("Failed to retrieve status byte"));
      }
    }
  }
  if (all)
    dataPort.println();

  // Send Serial Poll Disable [SPD] to all devices
  if ( gpibBus.sendCmd(GC_SPD) )
    return;

  // Send Untalk [UNT] to all devices
  if ( gpibBus.sendCmd(GC_UNT) )
    return;

  // Unadress listners [UNL] to all devices
  if ( gpibBus.sendCmd(GC_UNL) )
    return;

  // Set GPIB control to controller idle state
  gpibBus.setControls(CIDS);

  verb_println(F("Serial poll completed."));

}


/***** Return status of SRQ line *****/
void srq_h() {
  //NOTE: LOW=0=asserted, HIGH=1=unasserted
//  dataPort.println(!digitalRead(SRQ));
  dataPort.println(gpibBus.isAsserted(SRQ));
}


/***** Set the status byte (device mode) *****/
void stat_h(char *params) {
  uint16_t statusByte = 0;
  // A parameter given?
  if (params != NULL) {
    // Byte value given?
    if (notInRange(params, 0, 255, statusByte))
      return;
    gpibBus.setStatus((uint8_t)statusByte);
  } else {
    // Return the currently set status byte
    dataPort.println(gpibBus.cfg.stat);
  }
}


/***** Save controller configuration *****/
void save_h() {
  epWriteData(gpibBus.cfg.db, GPIB_CFG_SIZE);
  verb_println(F("Settings saved."));
}


/***** Show state or enable/disable listen only mode *****/
void lon_h(char *params) {
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 0, 1, val)) return;
    isRO = val ? true : false;
    if (isRO)
      isTO = false; // Talk-only mode must be disabled!
    if (isVerb) {
      dataPort.print(F("LON: "));
      dataPort.println(val ? "ON" : "OFF") ;
    }
  } else {
    dataPort.println(isRO);
  }
}


/***** Show help message *****/
void help_h(char *params) {
  char c, t;
  char token[20];
  uint8_t i;

  i = 0;
  for (size_t k = 0; k < strlen_P(cmdHelp); k++) {
    c = pgm_read_byte_near(cmdHelp + k);
    if (i < 20) {
      if(c == ':') {
        token[i] = 0;
        if((params == NULL) || (strcmp(token, params) == 0)) {
          dataPort.print(F("++"));
          dataPort.print(token);
          dataPort.print(c);
          k++;
          t = pgm_read_byte_near(cmdHelp + k);
          dataPort.print(F(" ["));
          dataPort.print(t);
          dataPort.print(F("]"));
          i = 255; // means we need to print until \n
        }
        
      } else {
        token[i] = c;
        i++;
      }
    }
    else if (i == 255) {
      dataPort.print(c);
    }
    if (c == '\n') {
      i = 0;
    }
  }
}

/***********************************/
/***** CUSTOM COMMAND HANDLERS *****/
/***********************************/

/***** All serial poll *****/
/*
 * Polls all devices, not just the currently addressed instrument
 * This is an alias wrapper for ++spoll all
 */
void aspoll_h() {
  //  char all[4];
  //  strcpy(all, "all\0");
  spoll_h((char*)"all");
}


/***** Send Universal Device Clear *****/
/*
 * The universal Device Clear (DCL) is unaddressed and affects all devices on the Gpib bus.
 */
void dcl_h() {
  if ( gpibBus.sendCmd(GC_DCL) )  {
    verb_println(F("Sending DCL failed"));
    return;
  }
  // Set GPIB controls back to idle state
  gpibBus.setControls(CIDS);
}


/***** Re-load default configuration *****/
void default_h() {
  gpibBus.setDefaultCfg();
}


/***** Show or set end of receive character(s) *****/
void eor_h(char *params) {
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 0, 15, val))
      return;
    gpibBus.cfg.eor = (uint8_t)val;
    if (isVerb) {
      dataPort.print(F("Set EOR to: "));
      dataPort.println(val);
    }
  } else {
    if (gpibBus.cfg.eor>7)
      gpibBus.cfg.eor = 0;  // Needed to reset FF read from EEPROM after FW upgrade
    dataPort.println(gpibBus.cfg.eor);
  }
}


/***** Parallel Poll Handler *****/
/*
 * Device must be set to respond on DIO line 1 - 8
 */
void ppoll_h() {
  uint8_t sb = 0;

  // Poll devices
  // Start in controller idle state
  gpibBus.setControls(CIDS);
  delayMicroseconds(20);
  // Assert ATN and EOI
  gpibBus.setControlVal(0b00000000, 0b10010000, STATE);
  //  setGpibState(0b10010000, 0b00000000, 0b10010000);
  delayMicroseconds(20);
  // Read data byte from GPIB bus without handshake
  sb = readGpibDbus();
  // Return to controller idle state (ATN and EOI unasserted)
  gpibBus.setControls(CIDS);

  // Output the response byte
  dataPort.println(sb, DEC);

  verb_println(F("Parallel poll completed."));
}


/***** Assert or de-assert REN 0=de-assert; 1=assert *****/
void ren_h(char *params) {
  // char *stat;
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 0, 1, val))
      return;
    digitalWrite(REN, (val ? LOW : HIGH));
    if (isVerb) {
      dataPort.print(F("REN: "));
      dataPort.println(val ? "REN asserted" : "REN un-asserted") ;
    };
  } else {
    dataPort.println(digitalRead(REN) ? 0 : 1);
  }
}


/***** Enable verbose mode 0=OFF; 1=ON *****/
void verb_h() {
  isVerb = !isVerb;
  dataPort.print("Verbose: ");
  dataPort.println(isVerb ? "ON" : "OFF");
}


/***** Set version string *****/
/* Replace the standard AR488 version string with something else
 *  NOTE: some instrument software requires a sepcific version string to ID the interface
 */
void setvstr_h(char *params) {
  uint8_t plen;
  char idparams[64];
  plen = strlen(params);
  memset(idparams, '\0', 64);
  strncpy(idparams, "verstr ", 7);
  if (plen  > 47)
    plen = 47; // Ignore anything over 47 characters
  strncat(idparams, params, plen);

  id_h(idparams);
}


/***** Talk only mode *****/
void ton_h(char *params) {
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 0, 1, val))
      return;
    isTO = val ? true : false;
    if (isTO)
      isRO = false; // Read-only mode must be disabled in TO mode!
    if (isVerb) {
      dataPort.print(F("TON: "));
      dataPort.println(val ? "ON" : "OFF") ;
    }
  } else {
    dataPort.println(isTO);
  }
}


/***** SRQ auto - show or enable/disable automatic spoll on SRQ *****/
/*
 * In device mode, when the SRQ interrupt is triggered and SRQ
 * auto is set to 1, a serial poll is conducted automatically
 * and the status byte for the instrument requiring service is
 * automatically returned. When srqauto is set to 0 (default)
 * an ++spoll command needs to be given manually to return
 * the status byte.
 */
void srqa_h(char *params) {
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 0, 1, val))
      return;
    switch (val) {
      case 0:
        isSrqa = false;
        break;
      case 1:
        isSrqa = true;
        break;
    }
    verb_println(isSrqa ? "SRQ auto ON" : "SRQ auto OFF") ;
  } else {
    dataPort.println(isSrqa);
  }
}


/***** Repeat a given command and return result *****/
void repeat_h(char *params) {
  uint16_t count;
  uint16_t tmdly;
  char *param;

  if (params != NULL) {
    // Count (number of repetitions)
    param = strtok(params, " \t");
    if (strlen(param) > 0) {
      if (notInRange(param, 2, 255, count))
        return;
    }
    // Time delay (milliseconds)
    param = strtok(NULL, " \t");
    if (strlen(param) > 0) {
      if (notInRange(param, 0, 30000, tmdly))
        return;
    }

    // Pointer to remainder of parameters string
    param = strtok(NULL, "\n\r");
    if (strlen(param) > 0) {
      for (uint16_t i = 0; i < count; i++) {
        // Send string to instrument
        gpibBus.sendData(param, strlen(param));
        delay(tmdly);
        gpibBus.receiveData(dataPort, gpibBus.cfg.eoi, false, 0);
      }
    } else {
      errBadCmd();
      verb_println(F("Missing parameter"));
      return;
    }
  } else {
    errBadCmd();
    verb_println(F("Missing parameters"));
  }

}


/***** Bus diagnostics *****/
/*
 * Usage: xdiag mode byte
 * mode: 0=data bus; 1=control bus
 * byte: byte to write on the bus
 * Note: values to switch individual bits = 1,2,4,8,10,20,40,80
 */
void xdiag_h(char *params){
  char *param;
  uint8_t mode = 0;
  uint8_t val = 0;
  
  // Get first parameter (mode = 0 or 1)
  param = strtok(params, " \t");
  if (param != NULL) {
    if (strlen(param)<4){
      mode = atoi(param);
      if (mode>2) {
        dataPort.println(F("Invalid: 0=data bus; 1=control bus"));
        return;
      }
    }
  }
  // Get second parameter (8 bit byte)
  param = strtok(NULL, " \t");
  if (param != NULL) {
    if (strlen(param)<4){
      val = atoi(param);
    }

    switch (mode) {
      case 0:
          // Set to required value
          gpibBus.setDataVal(val);
          // Reset after 10 seconds
          delay(10000);
          gpibBus.setDataVal(0);
          break;
      case 1:
          // Set to required state
          gpibBus.setControlVal(0xFF, 0xFF, DIRECTION);  // Set direction
          gpibBus.setControlVal(~val, 0xFF, STATE);  // Set state (low=asserted so must be inverse of value)
          // Reset after 10 seconds
          delay(10000);
          if (gpibBus.cfg.cmode==2) {
            gpibBus.setControls(CINI);
          }else{
            gpibBus.setControls(DINI);
          }
          break;
    }
  }
}


/***** Set device ID *****/
/*
 * Sets the device ID parameters including:
 * ++id verstr - version string (same as ++setvstr)
 * ++id name   - short name of device (e.g. HP3478A) up to 15 characters
 * ++id serial - serial number up to 9 digits long
 */
void id_h(char *params) {
  uint8_t dlen = 0;
  char * keyword; // Pointer to keyword following ++id
  char * datastr; // Pointer to supplied data (remaining characters in buffer)
  char serialStr[10];

  if (params != NULL) {
    keyword = strtok(params, " \t");
    datastr = keyword + strlen(keyword) + 1;
    dlen = strlen(datastr);
    if (dlen) {
      if (strncasecmp(keyword, "verstr", 6)==0) {
        if (dlen>0 && dlen<48) {
          memset(gpibBus.cfg.vstr, '\0', 48);
          strncpy(gpibBus.cfg.vstr, datastr, dlen);
          verb_println(F("VerStr: "));
          dataPort.println(gpibBus.cfg.vstr);
        } else {
          verb_println(F("Length of version string must not exceed 48 characters!"));
          errBadCmd();
        }
        return;
      }
      if (strncasecmp(keyword, "name", 4)==0) {
        if (dlen>0 && dlen<16) {
          memset(gpibBus.cfg.sname, '\0', 16);
          strncpy(gpibBus.cfg.sname, datastr, dlen);
        } else {
          verb_println(F("Length of name must not exceed 15 characters!"));
          errBadCmd();
        }
        return;
      }
      if (strncasecmp(keyword, "serial", 6)==0) {
        if (dlen < 10) {
          gpibBus.cfg.serial = atol(datastr);
        } else {
          verb_println(F("Serial number must not exceed 9 characters!"));
          errBadCmd();
        }
        return;
      }
    } else {
      if (strncasecmp(keyword, "verstr", 6) == 0) {
        dataPort.println(gpibBus.cfg.vstr);
        return;
      } else if (strncasecmp(keyword, "fwver", 6) == 0) {
        dataPort.println(F(FWVER));
        return;
      } else if (strncasecmp(keyword, "name", 4) == 0) {
        dataPort.println(gpibBus.cfg.sname);
        return;      
      } else if (strncasecmp(keyword, "serial", 6) == 0) {
        memset(serialStr, '\0', 10);
        snprintf(serialStr, 10, "%09lu", gpibBus.cfg.serial);  // Max str length = 10-1 i.e 9 digits + null terminator 
        dataPort.println(serialStr);
        return;    
      }
    }
  }
  errBadCmd();
}


void idn_h(char * params){
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 0, 2, val))
      return;
    gpibBus.cfg.idn = (uint8_t)val;
    if (isVerb) {
      dataPort.print(F("Sending IDN: "));
      dataPort.print(val ? "Enabled" : "Disabled"); 
      if (val==2)
        dataPort.print(F(" with serial number"));
      dataPort.println();
    };
  } else {
    dataPort.println(gpibBus.cfg.idn, DEC);
  }  
}


/***** Send device clear (usually resets the device to power on state) *****/
void sendmla_h() {
  if (gpibBus.sendMLA())  {
    verb_println(F("Failed to send MLA"));
    return;
  }
}


/***** Send device clear (usually resets the device to power on state) *****/
void sendmta_h() {
  if (gpibBus.sendMTA())  {
    verb_println(F("Failed to send MTA"));
    return;
  }
}


/***** Show or set read timout *****/
void sendmsa_h(char *params) {
  uint16_t saddr;
  char * param;
  if (params != NULL) {
    // Secondary address
    param = strtok(params, " \t");
    if (strlen(param) > 0) {
      if (notInRange(param, 96, 126, saddr))
        return;
      if (gpibBus.sendMSA(saddr)) {
        verb_println(F("Failed to send MSA"));
        return;
      }
    }
    // Secondary address command parameter
    param = strtok(NULL, " \t");
    if (strlen(param)>0) {
      gpibBus.setControls(CTAS);
      gpibBus.sendData(param, strlen(param));
      gpibBus.setControls(CLAS);
    }
//    addressingSuppressed = true;
  }
}


/***** Send device clear (usually resets the device to power on state) *****/
void unlisten_h() {
  if (gpibBus.sendUNL())  {
    verb_println(F("Failed to send UNL"));
    return;
  }
  // Set GPIB controls back to idle state
  gpibBus.setControls(CIDS);
//  addressingSuppressed = false;
}


/***** Send device clear (usually resets the device to power on state) *****/
void untalk_h() {
  if (gpibBus.sendUNT())  {
    verb_println(F("Failed to send UNT"));
    return;
  }
  // Set GPIB controls back to idle state
  gpibBus.setControls(CIDS);
//  addressingSuppressed = false;
}

/******************************************************/
/***** Device mode GPIB command handling routines *****/
/******************************************************/

/***** Attention handling routine *****/
/*
 * In device mode is invoked whenever ATN is asserted
 */
void attnRequired() {

  const uint8_t cmdbuflen = 35;
  uint8_t cmdbytes[5] = {0};
  uint8_t db = 0;
  uint8_t stat = 0;
  bool eoiDetected = false;
  uint8_t gpibcmd = 0;
  uint8_t bytecnt = 0;
  uint8_t atnstat = 0;
  uint8_t ustat = 0;
  bool addressed = false;

  // Set device listner active state (assert NDAC+NRFD (low), DAV=INPUT_PULLUP)
  gpibBus.setControls(DLAS);

  /***** ATN read loop *****/

  // Read bytes
  while ( (gpibBus.isAsserted(ATN)) && (bytecnt<cmdbuflen)  && (stat==0) ) {
    // Read the next byte from the bus, no EOI detection
    stat = gpibBus.readByte(&db, false, &eoiDetected);
    if (!stat) {
      // Untalk or unlisten
      if ( (db == 0x5F) || (db == 0x3F) ) {
        if (db == 0x3F) {
          if (device_unl_h()) ustat |= 0x01;
        }
        if (db == 0x5F) {
          if (device_unt_h()) ustat |= 0x02; 
        }
      // Other command
      }else{
        cmdbytes[bytecnt] = db;
        bytecnt++;
      }
    }
  }

  atnstat |= 0x01;

  /***** Command process loop *****/

  if (bytecnt>0) {  // Some command tokens to process

    // Process received command tokens
    for (uint8_t i=0; i<bytecnt; i++) { 

      if (!cmdbytes[i]) break;  // End loop on zero

      db = cmdbytes[i];

      // Device is addressed to listen
      if (gpibBus.cfg.paddr == (db ^ 0x20)) { // MLA = db^0x20
        atnstat |= 0x02;
        addressed = true;
        gpibBus.setControls(DLAS);

      // Device is addressed to talk
      } else if (gpibBus.cfg.paddr == (db ^ 0x40)) { // MLA = db^0x40
        // Call talk handler to send data
        atnstat |= 0x04;
        addressed = true;
        gpibBus.setControls(DTAS);

      } else if (db<0x20) {
        // Primary command received
        if (addressed) { // If we have been addressed (talk or listen)
          gpibcmd = db;
          atnstat |= 0x08;
        }
      }
    }   // End for

    // If we have not been adressed then back to idle and exit loop
    if (!addressed) {
      gpibBus.setControls(DINI);      
      return;
    }

    // If we have been adressed, then execute commands

    /***** Perform GPIB primary command actions *****/
    if (gpibcmd) {
      // Respond to GPIB command
      execGpibCmd(gpibcmd);
      // Clear flags
      gpibcmd = 0;
      atnstat |= 0x20;
    }

    /***** Otherwise perform controller mode read or write *****/
    if (gpibBus.cfg.cmode == 2) { 

      // Listen for data
      if (gpibBus.isDeviceAddressedToListen()) {
        device_listen_h();
        atnstat |= 0x80;
      }

      // Talk (send data)
      if (gpibBus.isDeviceAddressedToTalk()) {
        device_talk_h();
        atnstat |= 0x80;
          
      }

    }  // End mode = 2
  }
}


/***** Execute GPIB command *****/
void execGpibCmd(uint8_t gpibcmd){

  // Respond to GPIB command
  switch (gpibcmd) {
    case GC_SPE:
      // Serial Poll enable request
        device_spe_h();
        break;
      case GC_SPD:
        // Serial poll disable request
        device_spd_h();
        break;       
    case GC_UNL:
        // Unlisten
        device_unl_h();
        break;
    case GC_UNT:
        // Untalk
        device_unt_h();
        break;
    case GC_SDC:
        // Device clear (reset)
        device_sdc_h();
        break;
  } // End switch
}


/***** Device is addressed to listen - so listen *****/
void device_listen_h(){
  // Receivedata params: stream, detectEOI, detectEndByte, endByte
  gpibBus.receiveData(dataPort, false, false, 0x0);
}


/***** Device is addressed to talk - so send data *****/
void device_talk_h(){
  if (lnRdy == 2)
    sendToInstrument(pBuf, pbPtr);
}


/***** Selected Device Clear *****/
void device_sdc_h() {
  // If being addressed then reset
  if (isVerb)
    dataPort.println(F("Resetting..."));

  rst_h();

  if (isVerb)
    dataPort.println(F("Reset failed."));
}


/***** Serial Poll Disable *****/
/***** Serial Poll Disable *****/
void device_spd_h() {
//  gpibBus.setDeviceAddressedState(DIDS);
  gpibBus.setControls(DIDS);
}


/***** Serial Poll Enable *****/
void device_spe_h() {
  gpibBus.sendStatus();
  // Check if SRQ bit is set and clear it
  if (gpibBus.cfg.stat & 0x40) {
    gpibBus.setStatus(gpibBus.cfg.stat & ~0x40);
  }
}


/***** Unlisten *****/
bool device_unl_h() {
  // Stop receiving and go to idle
  readWithEoi = false;
  // Immediate break - shouldn't ATN do this anyway?
  tranBrk = 3;  // Stop receving transmission
  // Clear addressed state flag and set controls to idle
  if (gpibBus.isDeviceAddressedToListen()) {
    gpibBus.setControls(DIDS);
    return true;
  }
  return false;
}


/***** Untalk *****/
bool device_unt_h(){
  // Stop sending data and go to idle
  // Clear addressed state flag and set controls to listen
  if (gpibBus.isDeviceAddressedToTalk()) {
    gpibBus.setControls(DIDS);
    gpibBus.clearDataBus();
    return true;
  }
  return false;
}

void lonMode(){

  gpibBus.receiveData(dataPort, false, false, 0);

  // Clear the buffer to prevent it getting blocked
  if (lnRdy==2) flushPbuf();
}
