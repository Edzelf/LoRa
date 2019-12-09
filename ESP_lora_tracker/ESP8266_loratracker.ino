//***************************************************************************************************
//*  ESP_lora_tracker.ino                                                                           *
//*  By Ed Smallenburg.                                                                             *
//***************************************************************************************************
//                                                                                                  *
// Connect to TTN and send sensor data (in this case a GPS module).                                 *
// Version for ESP8266.                                                                             *
// The first time, the device will connect over OTAA.  TTN will return a fresh network session key  *
// and an application session key that should be used for the next connection (after reset).        *
// Therefore the keys are saved in non-volitile storage after joining in RTC memory.                *
// After every message, the keys (and the sequence number) are updated in RTC memory.               *
// Every 100 messages, the key are also save in EEPROM.                                             *
// On start-up, the keys and sequence number are retrieved from RTC memory or EEPROM.               *
// Note: Serial input is also used for the GPS receiver.  So disconnect before upload.              *
// Note: for deep sleep, the D0 pin (GPIO16) must be connected to RST.                              *
// Note: the first OTAA join may take a long time.  Sometimes a reset helps.                        *
// The payload for messages to TTN are:                                                             *
// ..                                                                                               *
// ..                                                                                               *
//                                                                                                  *
// Software is based on ttn_otaa by Thomas Telkamp and Matthijs Kooijman.                           *
//                                                                                                  *
// 14-05-2018, ES: First set-up.                                                                    *
// 09-12-2019, ES: Add new events for MCCI LMIC.                                                    *
//***************************************************************************************************
#include <lmic.h>                                   // MCCI version is used
#include <hal/hal.h>
#include <SPI.h>
#include <EEPROM.h>
#include <TinyGPS.h>

// Pins used for lmic
#define SS    15
#define RESET 0xFF
#define DIO0  4 
#define DIO1  5
#define DIO2  0xFF

TinyGPS        gps ;                                  // Instance for gps class
 
#if defined(DISABLE_INVERT_IQ_ON_RX)
#error This example requires DISABLE_INVERT_IQ_ON_RX to be unset. Update \
       config.h in the lmic library to unset it.
#endif

u1_t NWKSKEY[16] ;                               // LoRaWAN NwkSKey, network session key.
u1_t APPSKEY[16] ;                               // LoRaWAN AppSKey, application session key.
u4_t DEVADDR ;                                   // LoraWAN devaddr, end node device address

static const u1_t APPEUI[8]= { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x00, 0xED, 0x89 } ;
// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t APPKEY[16] = { 0x07, 0xCD, 0x3E, 0x37, 0xA1, 0x0D, 0x9B, 0xC7,
                                 0x44, 0x54, 0x53, 0xCB, 0x9D, 0x41, 0x9C, 0x53 } ;
static const u1_t DEVEUI[8]  = { 0x00, 0x7F, 0x18, 0x32, 0x03, 0xB8, 0x8C, 0x2B } ;

#define DATAVALID 0xACF2AFC2                     // Pattern for data valid in EEPROM/RTC memory
                                                 // Change if you want OTA to renew keys.

struct savdata_t                                 // Structure of data to be saved over reset
{
  uint32_t dataValid ;                           // DATAVALID if valid data (joined)
  uint8_t  devaddr[4] ;                          // Device address after join
  uint8_t  nwkKey[16] ;                          // Network session key after join
  uint8_t  artKey[16] ;                          // Aplication session key after join
  uint32_t seqnoUp ;                             // Sequence number                      
} ;

//***************************************************************************************************
// Global data.                                                                                     *
//***************************************************************************************************
bool      OTAA = true ;                          // Assume connection through OTAA
uint32_t  data[2] ;                              // Data to TTN: N and E integer (multiplied by 1E6)
osjob_t   initjob ;                              // Lmic handle for init job
osjob_t   sendjob ;                              // Lmic handle for send job
savdata_t savdata ;                              // Data to be saved over reset
bool      sleepreq = false ;                     // Request to go to sleep

// Schedule TX every this many seconds.
const unsigned TX_INTERVAL = 120 ;

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
  Serial.printf ( "os_getArtEui called, returned " ) ;
  for ( int i = 0 ; i < 8 ; i++ )
  {
    buf[i] = APPEUI[7-i] ;
    Serial.printf ( "0x%02X ", buf[i] ) ;
  }
  Serial.printf ( "\n" ) ;
}

//***************************************************************************************************
//                                  O S _ G E T D E V E U I                                         *
//***************************************************************************************************
void os_getDevEui ( u1_t* buf ) 
{
  Serial.printf ( "os_getDevEui called, returned " ) ;
  for ( int i = 0 ; i < 8 ; i++ )
  {
    buf[i] = DEVEUI[7-i] ;
    Serial.printf ( "0x%02X ", buf[i] ) ;
  }
  Serial.printf ( "\n" ) ;
}


//***************************************************************************************************
//                                      M E M D M P                                                 *
//***************************************************************************************************
// Dump memory for debug
//***************************************************************************************************
void memdmp ( const char* header, uint8_t* p, uint16_t len )
{
  uint16_t i ;                                                        // Loop control

  Serial.print ( header ) ;                                           // Show header
  for ( i = 0 ; i < len ; i++ )
  {
    if ( ( i & 0x0F ) == 0 )                                          // Continue opn next line?
    {
      if ( i > 0 )                                                    // Yes, continuation line?
      {
        Serial.printf ( "\n" ) ;                                      // Yes, print it
      }
      Serial.printf ( "%04X: ", i ) ;                                 // Print index
    }
    Serial.printf ( "%02X ", *p++ ) ;                                 // Print one data byte
  }
  Serial.println() ;
}


//***************************************************************************************************
//                                O S _ G E T D E V K E Y                                           *
//***************************************************************************************************
void os_getDevKey ( u1_t* buf )
{
  Serial.printf ( "os_getDevKey called, returned " ) ;
  for ( int i = 0 ; i < 16 ; i++ )
  {
    Serial.printf ( "0x%02X ", APPKEY[i] ) ;
  }
  Serial.printf ( "\n" ) ;
  memcpy_P(buf, APPKEY, 16);  // Why this call returns APPKEY?
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
  uint16_t        eaddr ;                                  // Address in EEPROM
  uint8_t*        p ;                                      // Points into savdata

  Serial.printf ( "Save data to RTC memory\n" ) ;
  memcpy ( savdata.devaddr, &LMIC.devaddr, 4 ) ;           // Fill struct to save
  memcpy ( savdata.nwkKey,  LMIC.nwkKey, 16 ) ;
  memcpy ( savdata.artKey,  LMIC.artKey, 16 ) ;
  savdata.seqnoUp = LMIC.seqnoUp ;
  savdata.dataValid = DATAVALID ;
  memdmp ( "devaddr:", savdata.devaddr, 4 ) ;
  memdmp ( "nwkKey:",  savdata.nwkKey, 16 ) ;
  memdmp ( "artKey:",  savdata.artKey, 16 ) ;
  Serial.printf ( "SeqnoUp is %d\n", savdata.seqnoUp ) ;
  Serial.printf ( "SeqnoDown is %d\n", LMIC.seqnoDn ) ;
  ESP.rtcUserMemoryWrite ( 0, (uint32_t*) &savdata, sizeof(savdata) ) ;
  if ( ( ( LMIC.seqnoUp % 100 ) == 0 ) || OTAA )           // Need to save data in EEPROM?
  {
    Serial.println ( "Saving to EEPROM" ) ;
    p = (uint8_t*)&savdata ;                               // set target pointer
    for ( eaddr = 0 ; eaddr < sizeof(savdata) ; eaddr++ )
    {
      EEPROM.write ( eaddr, *p++ ) ;                       // Write to EEPROM
    }
    EEPROM.commit() ;                                      // Commit data to EEPROM
  }
}


//***************************************************************************************************
//                                    I N I T F U N C                                               *
//***************************************************************************************************
static void initfunc (osjob_t* j)
{
    // reset MAC state
    Serial.println ( "Reset MAC" ) ;
    LMIC_reset();
    LMIC_setLinkCheckMode ( 0 ) ;
    LMIC_setDrTxpow ( DR_SF7, 14 ) ;
    if ( OTAA )
    {
      // start joining
      Serial.println ( "Start joining" ) ;
      LMIC_startJoining();
    }
    else
    {
      memdmp ( "Set Session, DEVADDR:", (uint8_t*)&DEVADDR, 4 ) ;
      memdmp ( "NWKSKEY:", NWKSKEY, 16 ) ;
      memdmp ( "APPSKEY:", APPSKEY, 16 ) ;
      Serial.printf ( "Seqnr set to %d\n", savdata.seqnoUp ) ;
      LMIC_setSession ( 0x1, DEVADDR, NWKSKEY, APPSKEY ) ;
      LMIC.seqnoUp = savdata.seqnoUp ;                      // Correction counter
      do_send ( &sendjob ) ;
    }
    Serial.println ( "Initfunc finished" ) ;
    // init done - onEvent() callback will be invoked...
}


//***************************************************************************************************
//                                         O N E V E N T                                            *
//***************************************************************************************************
void onEvent (ev_t ev)
{
    Serial.println() ;
    Serial.print ( os_getTime() ) ;
    Serial.print ( ": ") ;
    switch ( ev )
    {
        case EV_SCAN_TIMEOUT:
            Serial.println ( "EV_SCAN_TIMEOUT" ) ;
            break ;
        case EV_BEACON_FOUND:
            Serial.println ( "EV_BEACON_FOUND" ) ;
            break ;
        case EV_BEACON_MISSED:
            Serial.println ( "EV_BEACON_MISSED" ) ;
            break ;
        case EV_BEACON_TRACKED:
            Serial.println ( "EV_BEACON_TRACKED" ) ;
            break ;
        case EV_JOINING:
            Serial.println ( "EV_JOINING" ) ;
            break ;
        case EV_JOINED:
            Serial.println ( "EV_JOINED" ) ;
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(2), do_send);
            break ;
        case EV_RFU1:
            Serial.println ( "EV_RFU1" ) ;
            break ;
        case EV_JOIN_TXCOMPLETE:
           // A join transmit cycle has completed without success
           Serial.println ( "EV_JOIN_TXCOMPLETE without success" ) ;
           break ;
        case EV_JOIN_FAILED:
            Serial.println ( "EV_JOIN_FAILED" ) ; 
            break ;
        case EV_REJOIN_FAILED:
            Serial.println ( "EV_REJOIN_FAILED" ) ;
            break ;
        case EV_TXSTART:
            Serial.println ( "EV_TXSTART" ) ;
            break ;
        case EV_TXCOMPLETE:
            Serial.println ( "EV_TXCOMPLETE (includes waiting for RX windows)" ) ;
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println ( "Received ack" ) ;
            if (LMIC.dataLen) 
            {
              Serial.printf ( "Received %d bytes of payload: ", LMIC.dataLen ) ;
              for ( int i = 0 ; i < LMIC.dataLen ; i++ )
              {
                Serial.printf ( "0x%02X ", LMIC.frame[LMIC.dataBeg + i] ) ;
              }
              Serial.printf ( "\n") ;
            }
            sleepreq = true ;                                 // Request to go to sleep
            // Schedule next transmission
            //os_setTimedCallback ( &sendjob,
            //                      os_getTime() + sec2osticks ( TX_INTERVAL ),
            //                      do_send ) ;
            break ;
        case EV_LOST_TSYNC:
            Serial.println ( "EV_LOST_TSYNC" ) ;
            break ;
        case EV_RESET:
            Serial.println ( "EV_RESET" ) ;
            break ;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println ( "EV_RXCOMPLETE" ) ;
            break ;
        case EV_LINK_DEAD:
            Serial.println ( "EV_LINK_DEAD" ) ;
            break ;
        case EV_LINK_ALIVE:
            Serial.println ( "EV_LINK_ALIVE" ) ;
            break ;
         default:
            Serial.printf ( "Unknown event %d", ev ) ;
            break ;
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
      Serial.println ( "OP_TXRXPEND, not sending" ) ;
  }
  else
  {
      // Prepare upstream data transmission at the next possible time.
      LMIC_setTxData2 ( 1, (xref2u1_t)data, 8, 0 ) ;
      Serial.println ( "Packet queued" ) ;
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
  uint8_t* p ;                                              // Pointer into savdata
  
  // return ;                                               // Return if OTAA is required
  ESP.rtcUserMemoryRead ( 0, (uint32_t*)&savdata,           // Retriev saved data from RTC memory
                          sizeof(savdata) ) ;

  if ( savdata.dataValid == DATAVALID )                     // DATA in RTC memory valid?
  {
    Serial.println ( "Keys retrieved from RTC memory\n" ) ; // Show retrieve result 
  }
  else
  {
    // No data vailable in RTC memory.  Use EEPROM data.
    p = (uint8_t*)&savdata ;
    for ( eaddr = 0 ; eaddr < sizeof(savdata) ; eaddr++ )
    {
      *p++ = EEPROM.read ( eaddr ) ;                        // Move one byte to savdata
    }
    savdata.seqnoUp += 100 ;                                // Counter may be not up-to-date
  }
  if ( savdata.dataValid == DATAVALID )                     // DATA in RTC or EEPROM memory valid?
  {
    Serial.printf ( "Valid data in NVS\n" ) ;               // Yes, show
    memdmp ( "devaddr is:",
             savdata.devaddr, 4 ) ;
    memdmp ( "nwksKey is:",
             savdata.nwkKey, 16 ) ;
    memdmp ( "appsKey is:",
             savdata.artKey, 16 ) ;
    Serial.printf ( "seqnr is %d\n", savdata.seqnoUp ) ;
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
    Serial.printf ( "No saved data, using OTAA\n" ) ;
  }
}


//***************************************************************************************************
//                                         G E T G P S D A T A                                      *
//***************************************************************************************************
void getGPSdata()
{
  bool           newData = false ;
  unsigned long  chars ;
  unsigned short sentences, failed ;
  float          flat, flon ;           // N and E floating
  unsigned long  age ;

  while ( ! newData )
  {
    // For one second we parse GPS data and report some key values
    for (unsigned long start = millis(); millis() - start < 1000;)
    {
      while ( Serial.available() )
      {
        char c = Serial.read() ;
        if ( gps.encode ( c ) )                               // Did a new valid sentence come in?
        {
          newData = true ;
        }
      }
    }
    if ( newData )
    {
      gps.f_get_position ( &flat, &flon, &age ) ;
      Serial.print ( "LAT=" ) ;
      Serial.print ( flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6 ) ;
      Serial.print ( " LON=" ) ;
      Serial.print ( flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6 ) ;
      data[0] = flon * 1E6 ;
      data[1] = flat * 1E6 ;
    }
    gps.stats ( &chars, &sentences, &failed ) ;
    Serial.print ( " CHARS=" ) ;
    Serial.print ( chars ) ;
    Serial.print ( " SENTENCES=" ) ;
    Serial.print ( sentences ) ;
    Serial.print ( " CSUM ERR=" ) ;
    Serial.print ( failed ) ;
    if ( chars == 0 )
    {
      Serial.println ( "** No characters received from GPS: check wiring **" ) ;
    }
    Serial.println() ;
  }
}


//***************************************************************************************************
//                                              S E T U P                                           *
//***************************************************************************************************
void setup()
{
  Serial.begin ( 9600 ) ;
  Serial.println ( "\nStarting" ) ;
  EEPROM.begin ( 512 ) ;                                // Init EEPROM, 512 byte should be sufficient
  getGPSdata() ;                                        // Get GPS data
  retrieveKeys() ;                                      // Retrieve session keys and seqnr (if any)
  // LMIC init
  LMIC_setupChannel ( 0, 868100000,
                      DR_RANGE_MAP ( DR_SF12, DR_SF7 ),
                      BAND_CENTI ) ;                    // g-band
  os_init() ;
  os_setCallback ( &initjob, initfunc ) ;
}


//***************************************************************************************************
//                                        L O O P                                                   *
//***************************************************************************************************
// The main loop of the program.                                                                    *
//***************************************************************************************************
void loop()
{
  uint32_t sleeptime ;                                        // Time to sleep to next sample
  uint32_t tnow ;                                             // Current runtime in microseconds
  
  os_runloop_once() ;
  if ( sleepreq )                                             // Time to go to sleep?
  {
    tnow = millis() * 1000 ;                                  // Current runtime in microseconds
    saveToRTC() ;                                             // Save to RTC memory
    sleeptime = TX_INTERVAL * 1000000 ;                       // Sleeptime in microseconds
    if ( sleeptime > tnow )                                   // Prevent negative sleeptime
    {
      sleeptime = sleeptime - tnow ;                          // Correction for duration of program
    }
    Serial.printf ( "Going to sleep for %ld seconds....",
                    sleeptime / 1000000 ) ;
    ESP.deepSleep ( sleeptime, WAKE_RF_DEFAULT ) ;            // Sleep for about 10 minutes
    // Will never arrive here...
  }
}
