// ESP8266_loratracker.ino
//
// For the uses receiver, I had to modify the TinyGPS.cpp source:
// #define _GPRMC_TERM   "GNRMC"
// #define _GPGGA_TERM   "GNGGA"


// Wiring:
// GPS tx -> RX (Serial)

#include <TinyGPS.h>

TinyGPS gps ;

void setup()
{
  delay ( 3000 ) ;
  Serial.begin(9600);
  Serial.println() ;
  Serial.print ( "Simple TinyGPS library v. " ) ;
  Serial.println ( TinyGPS::library_version() ) ;
  Serial.println ( "by Mikal Hart" ) ;
  Serial.println() ;
}

void loop()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial.available())
    {
      char c = Serial.read();
      if ( start < 10000 )
      {
        Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      }
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    //Serial.print(" SAT=");
    //Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    //Serial.print(" PREC=");
    //Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }
  
  gps.stats ( &chars, &sentences, &failed ) ;
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.print(failed);
  if (chars == 0)
  {
    Serial.println("** No characters received from GPS: check wiring **");
  }
  Serial.println() ;
}
