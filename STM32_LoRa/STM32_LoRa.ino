//***************************************************************************************************
//*  STM_LoRa.ino                                                                                   *
//*  By Ed Smallenburg.                                                                             *
//***************************************************************************************************
//                                                                                                  *
// Connect to TTN and send sensor data (for example GPS data or temperature).                       *
// The program will run every "TX_INTERVAL" seconds.  See the definition below.                     *
// The modules sensor.cpp and sensor.h can be modified easily to interface with the sensor(s) of    *
// your choise.  The supplied version generates a "Hello world!" staring for payload.               *
// The SOC goes into deep sleep mode after the data has been sent to TTN.                           *
// Version for STM32F103C8T6 (power LED removed, 46 mA run, 0,8 mA sleep).                          *
// The first time, the device will connect over OTAA.  TTN will return a fresh network session key  *
// and an application session key that should be used for the next connection (after reset).        *
// Note that a succesful JOIN can take more than 10 minutes.                                        *
// Therefore the keys are saved in non-volitile storage after joining in RTC memory.                *
// After every message, the keys (and the sequence number) are updated in RTC memory.               *
// Every 100 messages, the key are also save in EEPROM.                                             *
// On start-up, the keys and sequence number are retrieved from RTC memory and/or EEPROM.           *
// The payload for messages to TTN are:                                                             *
// ..                                                                                               *
// ..                                                                                               *
//                                                                                                  *
// Software is based on ttn_otaa by Thomas Telkamp and Matthijs Kooijman.                           *
//                                                                                                  *
// 06-06-2018, ES: First set-up.                                                                    *
//***************************************************************************************************
#if !(defined(__STM32F1__))
  #error Project is meant to be for the STM32F103 SOC
#endif
#include <string.h> 
#include <EEPROM.h>
#include <stdarg.h>
#include <stdio.h>
#include <libmaple/pwr.h>                             // Power control functions
#include <libmaple/scb.h>                             // Sleep functions
#include <RTClock.h>
#include "sensor.h"                                   // Definitions for the sensor(s) connected
#include "STM32Sleep.h"
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

//***************************************************************************************************
//    W I R I N G                                                                                   *
//***************************************************************************************************
// Pins used for lmic
// SPI SCK   PA5 BOARD_SPI1_SCK_PIN                   // Defaults (for SPI-1)
// SPI MISO  PA6 BOARD_SPI1_MISO_PIN
// SPI MOSI  PA7 BOARD_SPI1_MOSI_PIN
#define RESET 0xFF                                    // Reset not used
#define DIO0  PA0 
#define DIO1  PA1
#define SS    PA4                                     // SPI-1 default CS
#define DIO2  0xFF                                    // DI02 not used

// Some general definitions:
#define DEBUG_BUFFER_SIZE 150                         // Max. linelength of DEBUG lines
#define BKP_REG_BASE      ((uint32_t *)(0x40006C04))  // Address of RTC memory
#define EE_OFFSET         0x100                       // Startaddress data in EEPROM

#if defined(DISABLE_INVERT_IQ_ON_RX)
#error This example requires DISABLE_INVERT_IQ_ON_RX to be unset. Update \
       config.h in the lmic library to unset it.
#endif

u1_t NWKSKEY[16] ;                               // LoRaWAN NwkSKey, network session key.
u1_t APPSKEY[16] ;                               // LoRaWAN AppSKey, application session key.
u4_t DEVADDR ;                                   // LoraWAN devaddr, end node device address

//***************************************************************************************************
//  C O N F I G U R A T I O N  S E C T I O N                                                        *
//***************************************************************************************************
#define TX_INTERVAL 120                          // Schedule TX every this many seconds.
#define DEBUG       true                         // Debugging on or not
// Edit the following 3 arrays according to your registered application and device at TTN.
// Do NOT swap the order of the bytes (Little/Big Endian).  The software takes care of that.
//
// 1 - Copied from TTN Console, Application, Application EUIS:
static const u1_t APPEUI[8]  = { 0x70, 0xB3, 0xD4, 0xE5, 0xD0, 0x00, 0xED, 0x89 } ;
// 2 - Copied from TTN Console, Device overview, App Key:
static const u1_t APPKEY[16] = { 0x07, 0xCD, 0x3E, 0x36, 0xA2, 0x0D, 0x9B, 0xC7,
                                 0x44, 0x54, 0x53, 0xCC, 0x9D, 0x41, 0x9C, 0x53 } ;
// 3 - Copied from TTN Console, Device overview, Device EUI:
static const u1_t DEVEUI[8]  = { 0x00, 0x7F, 0x18, 0x31, 0x04, 0xB8, 0x8C, 0x2B } ;
// End of TTN configuration
#define DATAVALID 0xACF2BFD2                     // Random pattern for data valid in EEPROM/RTC mem
                                                 // Change if you want OTA to renew keys.
#define LED   PB12                               // Onboard LED, may be different for your board
//***************************************************************************************************
//  E N D  O F  C O N F I G U R A T I O N  S E C T I O N                                            *
//***************************************************************************************************

#define RTC_LEN    6                             // Length of RTC data in savdata_t [uint16_t]
#define EE_LEN     (sizeof(savdata_t) / 2)       // Total length of savdata [uint16_t]

struct savdata_t                                 // Structure of data to be saved over reset
{                                                // Note: size must be an even number of bytes 
  // First part.  Length is SAV_L1 bytes.  Present in limited RTC memory and EEPROM.
  u4_t     dataValid ;                           // DATAVALID if valid data (joined)
  u4_t     seqnoUp ;                             // Sequence number                      
  u4_t     devaddr[4] ;                          // Device address after join
  // Second part.  Together with first part present in simulated EEPROM
  u1_t     nwkKey[16] ;                          // Network session key after join
  u1_t     artKey[16] ;                          // Aplication session key after join
} ;


//***************************************************************************************************
// Global data.                                                                                     *
//***************************************************************************************************
RTClock   rt ( RTCSEL_LSE ) ;                    // Instance of RTC clock with 1 second alarm
bool      OTAA = true ;                          // Assume connection through OTAA
u1_t      ttndata[64] ;                          // Data to TTN
u1_t      ttndatalen = 0 ;                       // Number of bytes in ttndata
osjob_t   initjob ;                              // Lmic handle for init job
osjob_t   sendjob ;                              // Lmic handle for send job
savdata_t savdata __attribute__((aligned)) ;     // Data to be saved over reset
char      dummy[200] ;
bool      sleepreq = false ;                     // Request to go to sleep

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = SS,
    .rxtx = 0xFF,
    .rst = RESET,
    .dio = { DIO0, DIO1, DIO2 },
} ;


//***************************************************************************************************
//                                  O S _ G E T A R T E U I                                         *
//***************************************************************************************************
void os_getArtEui ( u1_t* buf )
{
  for ( int i = 0 ; i < 8 ; i++ )
  {
    buf[i] = APPEUI[7-i] ;
  }
}


//***************************************************************************************************
//                                  O S _ G E T D E V E U I                                         *
//***************************************************************************************************
void os_getDevEui ( u1_t* buf ) 
{
  for ( int i = 0 ; i < 8 ; i++ )
  {
    buf[i] = DEVEUI[7-i] ;
  }
}


//***************************************************************************************************
//                                O S _ G E T D E V K E Y                                           *
//***************************************************************************************************
void os_getDevKey ( u1_t* buf )
{
  for ( int i = 0 ; i < 16 ; i++ )
  {
    buf[i] = APPKEY[i] ;
  }
}


//**************************************************************************************************
//                                          D B G P R I N T                                        *
//**************************************************************************************************
// Send a line of info to serial output.  Works like vsprintf(), but checks the DEBUG flag.        *
// Print only if DEBUG flag is true.  Always returns the formatted string.                         *
//**************************************************************************************************
char* dbgprint ( const char* format, ... )
{
  static char sbuf[DEBUG_BUFFER_SIZE] ;                // For debug lines
  va_list     varArgs ;                                // For variable number of params

  va_start ( varArgs, format ) ;                       // Prepare parameters
  vsnprintf ( sbuf, sizeof(sbuf), format, varArgs ) ;  // Format the message
  va_end ( varArgs ) ;                                 // End of using parameters
  if ( DEBUG )                                         // DEBUG on?
  {
    Serial1.print ( "D: " ) ;                           // Yes, print prefix
    Serial1.println ( sbuf ) ;                          // and the info
  }
  return sbuf ;                                        // Return stored string
}


//***************************************************************************************************
//                                      M E M D M P                                                 *
//***************************************************************************************************
// Dump memory for debug
//***************************************************************************************************
void memdmp ( const char* header, uint8_t* x, uint16_t len )
{
  uint16_t i ;                                                        // Loop control
  char     formatbuf[100] ;
  char*    p = formatbuf ;

  for ( i = 0 ; i < len ; i++ )
  {
    if ( ( i & 0x0F ) == 0 )                                          // Continue opn next line?
    {
      if ( i > 0 )                                                    // Yes, continuation line?
      {
        p += sprintf ( p, "\n" ) ;                                    // Yes, print it
      }
      p += sprintf ( p, "%04X: ", i ) ;                               // Print index
    }
    p += sprintf ( p, "%02X ", *x++ ) ;                               // Print one data byte
  }
  dbgprint ( "%s %s", header, formatbuf ) ;
}


//***************************************************************************************************
//                            R T C U S E R M E M O R Y R E A D                                     *
//***************************************************************************************************
// Retrieve saved data from RTC memory.  Max len is 40 (bytes).                                     *
//***************************************************************************************************
void rtcUserMemoryRead ( uint16_t* p, uint16_t len )
{
  if ( len > 10 )                                         // Reduce to max length
  {
    len = 10 ; 
  }
  for ( int i = 0 ; i < len ; i++ )                       // Read 2 bytes per loop
  {
    *p++ = BKP_REG_BASE[i] ;
  }
}


//***************************************************************************************************
//                            R T C U S E R M E M O R Y W R I T E                                   *
//***************************************************************************************************
// Save data to RTC memory.  Max len is 20 (bytes).                                                 *
//***************************************************************************************************
void rtcUserMemoryWrite ( uint16_t* p, uint16_t len )
{
  if ( len > 10 )                                         // Reduce to max length
  {
    len = 10 ; 
  }
  for ( int i = 0 ; i < len ; i++ )                       // Read 2 bytes per loop
  {
    BKP_REG_BASE[i] = *p++ ;
  }
}


//***************************************************************************************************
//                                    S A V E T O R T C                                             *
//***************************************************************************************************
// Save data in RTC memory.  Every 100th call the data will also be saved in EEPROM memory.         *
// The EEPROM is also updates if OTAA was used.                                                     *
// The space in RTC memory is limited to 512 bytes.                                                 *
//***************************************************************************************************
void saveToRTC()
{
  uint16_t        eaddr ;                                  // Relative address in EEPROM
  uint16_t*       p ;                                      // Points into savdata

  dbgprint ( "Save data to RTC memory" ) ;
  memcpy ( savdata.devaddr, &LMIC.devaddr, 4 ) ;           // Fill struct to save
  memcpy ( savdata.nwkKey,  LMIC.nwkKey, 16 ) ;
  memcpy ( savdata.artKey,  LMIC.artKey, 16 ) ;
  savdata.seqnoUp = LMIC.seqnoUp ;
  savdata.dataValid = DATAVALID ;
  memdmp ( "devaddr:", (uint8_t*)savdata.devaddr, 4 ) ;
  memdmp ( "nwkKey:",  (uint8_t*)savdata.nwkKey, 16 ) ;
  memdmp ( "artKey:",  (uint8_t*)savdata.artKey, 16 ) ;
  dbgprint ( "SeqnoUp is %d", savdata.seqnoUp ) ;
  dbgprint ( "SeqnoDown is %d", LMIC.seqnoDn ) ;
  rtcUserMemoryWrite ( (uint16_t*) &savdata, RTC_LEN ) ;
  if ( ( ( LMIC.seqnoUp % 100 ) == 0 ) || OTAA )           // Need to save data in EEPROM?
  {
    dbgprint ( "Saving to EEPROM" ) ;
    p = (uint16_t*)&savdata ;                              // Set target pointer
    for ( eaddr = 0 ; eaddr < EE_LEN ; eaddr++ )
    {
      EEPROM.write ( EE_OFFSET + eaddr, *p++ ) ;           // Write to EEPROM
    }
  }
}


//***************************************************************************************************
//                                    I N I T F U N C                                               *
//***************************************************************************************************
static void initfunc (osjob_t* j)
{
    if ( OTAA )
    {
      // start joining
      dbgprint ( "Start joining" ) ;
      LMIC_startJoining() ;
    }
    else
    {
      memdmp ( "Set Session, DEVADDR:", (uint8_t*)&DEVADDR, 4 ) ;
      memdmp ( "NWKSKEY:", NWKSKEY, 16 ) ;
      memdmp ( "APPSKEY:", APPSKEY, 16 ) ;
      dbgprint ( "Seqnr set to %d", savdata.seqnoUp ) ;
      LMIC_setSession ( 0x1, DEVADDR, NWKSKEY, APPSKEY ) ;
      LMIC.seqnoUp = savdata.seqnoUp ;                      // Correction counter
      do_send ( &sendjob ) ;
    }
    //dbgprint ( "Initfunc finished" ) ;    // init done - onEvent() callback will be invoked...
}


//***************************************************************************************************
//                                         O N E V E N T                                            *
//***************************************************************************************************
void onEvent (ev_t ev)
{
    const char* p ;
    
    switch ( ev )
    {
        case EV_SCAN_TIMEOUT:
            p = "EV_SCAN_TIMEOUT" ;
            break;
        case EV_BEACON_FOUND:
            p =  "EV_BEACON_FOUND" ;
            break;
        case EV_BEACON_MISSED:
            p =  "EV_BEACON_MISSED" ;
            break;
        case EV_BEACON_TRACKED:
            p =  "EV_BEACON_TRACKED" ;
            break;
        case EV_JOINING:
            p =  "EV_JOINING" ;
            break;
        case EV_JOINED:
            p =  "EV_JOINED" ;
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(2), do_send);
            break;
        case EV_RFU1:
            p =  "EV_RFU1" ;
            break;
        case EV_JOIN_FAILED:
            p =  "EV_JOIN_FAILED" ; 
            break;
        case EV_REJOIN_FAILED:
            p =  "EV_REJOIN_FAILED" ;
            break;
        case EV_TXCOMPLETE:
            p =  "EV_TXCOMPLETE (includes waiting for RX windows)" ;
            sleepreq = true ;                                 // Request to go to sleep
            break;
        case EV_LOST_TSYNC:
            p =  "EV_LOST_TSYNC" ;
            break;
        case EV_RESET:
            p =  "EV_RESET" ;
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            p =  "EV_RXCOMPLETE" ;
            break;
        case EV_LINK_DEAD:
            p =  "EV_LINK_DEAD" ;
            break;
        case EV_LINK_ALIVE:
            p =  "EV_LINK_ALIVE" ;
            break;
         default:
            p =  "Unknown event" ;
            break;
    }
    dbgprint ( "%d:%s", os_getTime(), p ) ;
    if ( ev == EV_TXCOMPLETE )
    {
      if ( LMIC.txrxFlags & TXRX_ACK )
      {
        dbgprint ( "Received ack" ) ;
      }
      if ( LMIC.dataLen ) 
      {
        dbgprint ( "Received %d bytes of payload: ", LMIC.dataLen ) ;
      }
    }
}


//***************************************************************************************************
//                                            D O _ S E N D                                         *
//***************************************************************************************************
// Send a message to TTN.                                                                           *
//***************************************************************************************************
void do_send ( osjob_t* j )
{
  // Check if there is not a current TX/RX job running
  if ( LMIC.opmode & OP_TXRXPEND )
  {
      dbgprint ( "OP_TXRXPEND, not sending" ) ;
  }
  else
  {
      // Prepare upstream data transmission at the next possible time.
      LMIC_setTxData2 ( 1, (xref2u1_t)ttndata, ttndatalen, 0 ) ;
      dbgprint ( "Packet queued" ) ;
  }
  // Next TX is scheduled some time after TX_COMPLETE event.
}


//***************************************************************************************************
//                                R E T R I E V E K E Y S                                           *
//***************************************************************************************************
// Try to retrieve the keys en seqnr from non-volitile memory.                                      *
//***************************************************************************************************
void retrieveKeys()
{
  uint16_t eaddr ;                                          // Address in EEPROM
  uint16_t* p ;                                             // Pointer into savdata
  
  // return ;                                               // Return if OTAA is required
  rtcUserMemoryRead ( (uint16_t*)&savdata,                  // Retrieve saved data from RTC memory
                      RTC_LEN ) ;
  if ( savdata.dataValid == DATAVALID )                     // DATA in RTC memory valid?
  {
    dbgprint ( "Data retrieved from RTC memory" ) ;         // Show retrieve result 
    // Data in RTC is just the first part of the total data.  Read the rest form EEPROM.
    p = (uint16_t*)&savdata + RTC_LEN ;                     // Point to 2nd part of savdata
    for ( eaddr = RTC_LEN ; eaddr < EE_LEN ; eaddr++ )      // Loop for adresses in EEPROM.
    {
      *p++ = EEPROM.read ( EE_OFFSET + eaddr ) ;            // Move 2 bytes to savdata
    }
  }
  else
  {
    p = (uint16_t*)&savdata ;                               // No data vailable in RTC memory,
    for ( eaddr = 0 ; eaddr < EE_LEN ; eaddr++ )            // use full EEPROM data.
    {
      *p++ = EEPROM.read ( EE_OFFSET + eaddr ) ;            // Move 2 bytes to savdata
    }
    savdata.seqnoUp += 100 ;                                // Counter may be not up-to-date
  }
  if ( savdata.dataValid == DATAVALID )                     // DATA in RTC or EEPROM memory valid?
  {
    dbgprint ( "Valid data in NVS" ) ;                      // Yes, show
    memdmp ( "devaddr is:",
             (uint8_t*)savdata.devaddr, 4 ) ;
    memdmp ( "nwksKey is:",
             (uint8_t*)savdata.nwkKey, 16 ) ;
    memdmp ( "appsKey is:",
             (uint8_t*)savdata.artKey, 16 ) ;
    dbgprint ( "seqnr is %d", savdata.seqnoUp ) ;
    memcpy ( (uint8_t*)&DEVADDR,
             savdata.devaddr, sizeof(DEVADDR) ) ;          // LoraWAN DEVADDR, end node device address
    memcpy ( NWKSKEY,
             savdata.nwkKey,  sizeof(NWKSKEY) ) ;          // LoRaWAN NwkSKey, network session key.
    memcpy ( APPSKEY,
             savdata.artKey,  sizeof(APPSKEY) ) ;          // LoRaWAN AppSKey, application session key.
    OTAA = false ;                                         // Do not use OTAA
  }
  else
  {
    dbgprint ( "No saved data, using OTAA" ) ;
  }
}


//***************************************************************************************************
//                                     S E T U P C H A N N N E L S                                  *
//***************************************************************************************************
// Set up the channels used by the Things Network, which corresponds to the defaults of most        *
// gateways. Without this, only three base channels from the LoRaWAN specification are used, which  *
// certainly works, so it is good for debugging, but can overload those frequencies, so be sure to  *
// configure the full frequency range of your network here (unless your network autoconfigures      *
// them).  Setting up channels should happen after LMIC_setSession, as that configures the minimal  *
// channel set.                                                                                     *
// TTN defines an additional channel at 869.525Mhz using SF9 for class B devices' ping slots.       *
// LMIC does not have an easy way to define set this frequency and support for class B is spotty    *
// and untested, so this frequency is not configured here.                                          *
//***************************************************************************************************
void setupChannels()
{
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
}


//***************************************************************************************************
//                                              S E T U P                                           *
//***************************************************************************************************
void setup()
{
  setGPIOModeToAllPins ( GPIO_INPUT_ANALOG ) ;          // All pins to input
  Serial1.begin ( 115200 ) ;
  dbgprint ( "" ) ;
  dbgprint ( "Starting..." ) ;
  pinMode ( SS, OUTPUT ) ;                              // Chip select for SPI
  digitalWrite ( SS, HIGH ) ;
  pinMode ( LED, OUTPUT ) ;                             // Enable LED
  digitalWrite ( LED, LOW ) ;                           // LED On, signal is reversed
  sensor_init() ;                                       // Initialize sensor
  ttndatalen = sensor_getdata ( ttndata ) ;             // Get sensor data
  dbgprint ( "Got sensordata with len is %d",           // Report for debug
             ttndatalen ) ;
  retrieveKeys() ;                                      // Retrieve session keys and seqnr (if any)
  os_init() ;                                           // Init lmic OS
  LMIC_reset() ;                                        // Reset the MAC state
  setupChannels() ;                                     // Set LoRa channels to use
  LMIC_setLinkCheckMode ( 0 ) ;
  LMIC_setDrTxpow ( DR_SF7, 14 ) ;
  os_setCallback ( &initjob, initfunc ) ;               // Start with initfunc
}


//***************************************************************************************************
//                                        L O O P                                                   *
//***************************************************************************************************
// The main loop of the program.                                                                    *
//***************************************************************************************************
void loop()
{
  uint32_t        sleeptime ;                                 // Time to sleep to next sample
  uint32_t        tnow ;                                      // Current runtime in seconds
  
  os_runloop_once() ;                                         // Keep lmic os happy
  if ( sleepreq )                                             // Time to go to sleep?
  {
    saveToRTC() ;                                             // Save to RTC memory
    tnow = millis() / 1000 ;                                  // Current runtime in seconds
    sleeptime = TX_INTERVAL ;                                 // Run interval in seconds
    if ( sleeptime > tnow )                                   // Prevent negative sleeptime
    {
      sleeptime = sleeptime - tnow ;                          // Correction for duration of program
    }
    dbgprint ( "Going to sleep for %ld seconds....",
               sleeptime ) ;
    delay ( 10 ) ;
    adc_disable_all() ;                                       // Disable all ADC's
    setGPIOModeToAllPins ( GPIO_INPUT_ANALOG ) ;  
    sleepAndWakeUp ( STANDBY, &rt, sleeptime ) ;
    // Will never arrive here...
  }
  delay ( 10 ) ;
}
