/* A GPS Grid Square readout using a NEO 7M GPS Module
   By Wayne VK5APN 11/07/2018.
   Modified from David's VK5KK version of ON7EQ project. Revised 25/5/2018
   Now using the latest Arduino IDE v1.8.3
   TinyGPS++ can be downloaded from http://arduiniana.org/libraries/tinygpsplus/

   Added i2c and DS-1820 support 22/03/2020, Lou Blasco VK3ALB.

*/

/* TinyGPS++ library uses both the GPGGA and the GPRMC STRING's

     "$GNGGA,123519,4807.03800,N,01131.00000,E,1,08,0.9,545.4,M,46.9,M,,*47"

     GPGGA          Global Positioning System Fix Data
     123519         Fix taken at 12:35:19 UTC
     4807.03800,N   Latitude 48 deg 07.038' N  Note 5 Decimal points
     01131.00000,E  Longitude 11 deg 31.000' E Note 5 Decimal points
     1              Fix quality:0 = invalid
                                1 = GPS fix (SPS) Use for GPS lock
                                2 = DGPS fix
                                3 = PPS fix
                                4 = Real Time Kinematic
                                5 = Float RTK
                                6 = estimated (dead reckoning) (2.3 feature)
                                7 = Manual input mode
                                8 = Simulation mode
     08             Number of satellites being tracked
     0.9            Horizontal dilution of position
     545.4,M        Altitude, Meters, above mean sea level
     46.9,M         Height of geoid (mean sea level) above WGS84 ellipsoid
     (empty field)  Time in seconds since last DGPS update
     (empty field)  DGPS station ID number
      47            The checksum data, always begins with

   If the height of geoid is missing then the altitude should be suspect.
   Some non-standard implementations report altitude with respect to the ellipsoid rather than geoid altitude.
   Some units do not report negative altitudes at all. This is the only sentence that reports altitude.


     $GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68

     GPRMC          Global Positioning System Fix Data
     225446         Time of fix 22:54:46 UTC
     A              Navigation receiver warning A = OK, V = warning
     4916.45,N      Latitude 49 deg. 16.45 min North
     12311.12,W     Longitude 123 deg. 11.12 min West
     000.5          Speed over ground, Knots
     054.7          Course Made Good deg True
     191194         Date of fix 19 November 1994
     020.3,E        Magnetic variation 20.3 deg East
      68            Mandatory checksum

*/


//////////////////// LIBRARY AND VARIABLES ////////////////////
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 20 chars and 4 line display
#include <OneWire.h>
#include <DallasTemperature.h>

// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(4);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
float dsTemp;   // Temperature variable

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// LCD specific characters 'degrees'

byte degree [8] = {
  B00100,
  B01010,
  B00100,
  B00000,
  B00000,
  B00000,
  B00000,
};

// Voltmeter and low voltage Function: The voltage divider at A3, a maximum of 20 Volts ratio R1/R2 is 3:1
#define R1           (10)   // from GND to A3, express in 100R (10 = 1000 Ohm)
#define R2           (34)   // from + power supply to A3, express in 100R (30 = 3000 Ohm)
#define VoltSupplyMini (105) // Alarm battery voltage setting in 100mV (105 =  10.5 Volts)
unsigned int SupplyVoltage = (0); // Power supply voltage
unsigned long BattCheck = 0;      // Battery Check Interval

// NEO 7M SET TO 9600 Baud
static const int RXPin = 2, TXPin = 3;
#define GPSBaud 9600
SoftwareSerial ss(RXPin, TXPin);// The serial connection to the GPS device

// Global variables
const char* UpperCaseCharString[] = {"A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R"};
const char* NumberCharString[] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9"};
const char* LowerCaseCharString[] = {"a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m", "n", "o", "p", "q", "r", "s", "t", "u", "v", "w", "x"};
const char* MonthCharString[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

float loclatitude;  // variable for LOC calculations
float loclongitude;  // variable for LOC calculations
float RemainderLongA, RemainderLongB, RemainderLongC, RemainderLongD;  // variables for longitudinal grid square calculations
float RemainderLatA, RemainderLatB, RemainderLatC, RemainderLatD;  // variables for latitudinal grid square calculations

// The TinyGPS++ object
TinyGPSPlus gps;


//////////////////// SETUP ////////////////////

void setup()
{
  sensors.begin();  // Start the temperature library
  // Set up the LCD's number of columns and rows
  lcd.init();                      // initialize the lcd
  lcd.backlight();
  // Wait to allow LCD display to init
  delay (200);

  // Set GPS Baud rate
  ss.begin(GPSBaud);

  // Print a message to the LCD.
  lcd.setCursor(6, 0);
  lcd.print("VK3ALB/P");
  lcd.setCursor(0, 1);
  lcd.print("Portable GPS Display");
  lcd.setCursor(4, 2);
  lcd.print("Version 2.03");
  delay (10000);

  // Calculate and Print supply voltage.
  SupplyVoltage = analogRead(A3);        // Read power supply voltage.
  SupplyVoltage = map(SupplyVoltage, 0, 1023, 0, (50 * (R2 + R1) / R1));
  lcd.setCursor(2, 3);
  delay (100);

  lcd.print("Batt Volt=");
  if (SupplyVoltage < 100) {
    lcd.print(" ");
  }
  if (SupplyVoltage < 10) {
    lcd.print(" ");
  }
  lcd.print((SupplyVoltage / 10), DEC);
  lcd.print(".");
  lcd.print((SupplyVoltage) % 10, DEC);
  lcd.print(" v");
  delay (3000);

  // Create special deg character
  lcd.createChar(0, degree);
  PrintTemplate();
}
//////////////////// END SETUP ////////////////////

//////////////////// LOOP ////////////////////

void loop() {

  // Test Battery status
  if ((millis() - BattCheck) > 20000) {  //every 20s check battery
    BattCheck = millis();
    SupplyVoltage = analogRead(A3);        // Read power supply voltage
    SupplyVoltage = map(SupplyVoltage, 0, 1023, 0, (50 * (R2 + R1) / R1));
    if  (SupplyVoltage <= VoltSupplyMini)   {  // We have a low voltage
      lcd.clear();
      lcd.setCursor(1, 1);
      lcd.print("Batt Volts=");
      if (SupplyVoltage < 100) {
        lcd.print(" ");
      }
      if (SupplyVoltage < 10) {
        lcd.print(" ");
      }
      lcd.print((SupplyVoltage / 10), DEC);
      lcd.print(".");
      lcd.print((SupplyVoltage) % 10, DEC);
      lcd.print(" v");
      lcd.setCursor(4, 3);
      lcd.print("LOW BATTERY !");
      digitalWrite(3, HIGH);  /// BEEP
      delay(20);
      digitalWrite(3, LOW);
      delay(20);
      digitalWrite(3, HIGH);
      delay(20);
      digitalWrite(3, LOW);
      delay (2000);
      PrintTemplate();
    }
  }
  // End BattCheck

  LCDprint();

}
//////////////////// END LOOP ////////////////////

//////////////////// LCDPrint ////////////////////
void LCDprint() {

  // Dispatch incoming characters
  while (ss.available() > 0)
    gps.encode(ss.read());

  // First lines Latitude
  if (gps.location.isUpdated())
  {
    lcd.setCursor(0, 0);
    lcd.print(gps.location.rawLat().negative ? "S" : "N");
    lcd.setCursor(3, 0);
    if ((gps.location.rawLat().deg) < 10) { //Lat values between 0 and 90 deg - leading spaces for uniform display formatting.
      lcd.print(" ");
    }
    lcd.print(gps.location.rawLat().deg);
    lcd.write(byte(0));
    if (((gps.location.rawLat().billionths) * 0.00000006) < 10) { //If Less than 10 print a leading zero
      lcd.print("0");
    }
    lcd.print(((gps.location.rawLat().billionths) * 0.00000006), 2); //9 digit number so divide by 1,000,000,000 and multiply by 60 for Minutes or multiply by 0.00000006 and 2 decimal places
    // Second line Longitude
    lcd.setCursor(0, 1);
    lcd.print(gps.location.rawLng().negative ? "W" : "E");
    lcd.setCursor(2 , 1);
    if ((gps.location.rawLng().deg) < 100) { //Lon values between 0 and 180 deg - leading spaces for uniform display formatting.
      lcd.print(" ");
    }
    if ((gps.location.rawLng().deg) < 10) {
      lcd.print(" ");
    }
    lcd.print(gps.location.rawLng().deg);
    lcd.write(byte(0));
    if (((gps.location.rawLng().billionths) * 0.00000006) < 10) { //If Less than 10 print a leading zero
      lcd.print("0");
    }
    lcd.print(((gps.location.rawLng().billionths) * 0.00000006), 2); //9 digit number so divide by 1,000,000,000 and multiply by 60 for Minutes or multiply by 0.00000006 and 2 decimal places
  }

  // First line Number of Satellites
  else if (gps.satellites.isUpdated())
  {
    lcd.setCursor(15, 0);
    lcd.print(" Sats");
    lcd.setCursor(13, 0);
    if ((gps.satellites.value()) < 10) {
      lcd.print(" ");
    }
    lcd.print(gps.satellites.value());
  }

  // Third line Meters above sea level
  else if (gps.altitude.isUpdated())
  {
    lcd.setCursor(11, 2);
    if ((gps.altitude.meters()) < 1000) {
      lcd.print(" ");
    }
    if ((gps.altitude.meters()) < 100) {
      lcd.print(" ");
    }
    if ((gps.altitude.meters()) < 10) {
      lcd.print(" ");
    }
    lcd.print(gps.altitude.meters(), 0);
    lcd.setCursor(15, 2);
    lcd.print(" mASL");
  }

  // Fourth line UTC Time
  else if (gps.time.isUpdated())
  {
    lcd.setCursor(0, 3);
    if ((gps.time.hour()) < 10) {
      lcd.print("0");
    }
    lcd.print(gps.time.hour());
    lcd.setCursor(2, 3);
    lcd.print(":");
    if ((gps.time.minute()) < 10) {
      lcd.print("0");
    }
    lcd.print(gps.time.minute());
    lcd.setCursor(5, 3);
    lcd.print(":");
    if ((gps.time.second()) < 10) {
      lcd.print("0");
    }
    lcd.print(gps.time.second());

    // Fourth line Time and Date Zone (UTC)
    lcd.setCursor(9, 3);
    lcd.print("UTC");
    {
      sensors.requestTemperatures(); // Get temperature
      dsTemp = sensors.getTempCByIndex(0);
      lcd.setCursor(13, 1);
      lcd.print("    ");
      lcd.setCursor(13, 1);
      lcd.print(dsTemp, 1);
      lcd.print(" ");
      lcd.write(byte(0));
      lcd.print("C");
    }
  }

  // Fourth line UTC Date
  else if (gps.date.isUpdated())
  {
    lcd.setCursor(13, 3);
    if ((gps.date.day()) < 10) {
      lcd.print("0");
    }
    lcd.print(gps.date.day());
    lcd.print(MonthCharString[gps.date.month() - 1]);
    lcd.print(gps.date.year() - 2000);
  }



  // GRID SQUARE LOCATOR CALCULATION //
  /*  The Maidenhead Locator System is a method for identifying positions on the Earth and is commonly used by VHF/UHF enthusiasts.
      A maidenhead locator represents a position on the Earth based on latitude and longitude.  Pairs of characters alternate between
      letters and numbers that indicate a zone or sub-zones.  Longitude values are always first, followed by the latitude for each pair.
      While it is common to represent a location using the first two or three pair’s, additional accuracy can be gained by including the
      fourth or fifth pairs.  Especially when using the higher microwave bands.
  */


  // FIRST PAIR (AA-RR)
  /*  The World is divided into (18) 20 degree Longitudinal by (18) 10 degree Latitudinal Zones commonly known as Fields.
      In order to avoid negative numbers the system also specifies that the latitude is measured from the South Pole to the North Pole
      and longitude is measured eastwards from the antemeridian of Greenwich, giving the Prime Meridian a false easting of 180 degrees
      and the equator a false northing of 90 degrees.  The first character encodes the longitude and the second encodes the latitude
      with letters "A" through "R" (R being the 18th letter of the alphabet).
  */


  // --== NOTE NOTE NOTE ==--  To make it easier to do calculations with, the values are multiplied by 1,000,000 to make the 6 decimal places now a whole number).


  // Remove the negative sign from the longitude location if there is one.
  // West are negative values. For West locations, subtract your location from 180 degrees. For East Locations, add 180 degrees.

  if ((gps.location.rawLng().negative) == 1) {                       // If West then do this, (note: returns a 1 if negative).
    loclongitude = ((180000000 - (gps.location.lng()) * (-1000000)));
  }
  else {                                                             // If East then do this.
    loclongitude = (((gps.location.lng()) * (1000000)) + 180000000);
  }


  // Remove the negative sign from the latitude location if there is one.
  // South are negative values. For South locations, subtract your location from 90 degrees. For North Locations, add 90 degrees.

  if ((gps.location.rawLat().negative) == 1) {                       // If South then do this, (note: returns a 1 if negative).
    loclatitude = ((90000000 - (gps.location.lat()) * (-1000000)));
  }
  else {                                                             // If North then do this.
    loclatitude = (((gps.location.lat()) * (1000000)) + 90000000);
  }

  // Where do you want to place the Grid locator info on the screen.
  lcd.setCursor(0 , 2);

  // FIRST CHARACTER - longitude based (every 20° = 1 gridsq).  Value is the integer of the longitude divided by 20 (degrees in each group).
  lcd.print(UpperCaseCharString[int(loclongitude / 20000000)]);

  // SECOND CHARACTER - latitude based (every 10° = 1 gridsq).  Value is the integer of the latitude divided by 10 (degrees in each group).
  lcd.print(UpperCaseCharString[int(loclatitude / 10000000)]);



  // SECOND PAIR (00-99)
  /*  Each field can be further divided into (10) 2 degree longitudinal by (10) 1 degree latitudinal zones commonly known as squares.
      The first character encodes the longitude and the second encodes the latitude with numbers “0” thru “9”.
  */

  // Need to obtain the Remainders from the previous equations.
  RemainderLongA = loclongitude  -   (20000000 * int (loclongitude / 20000000));
  RemainderLatA = loclatitude  -   (10000000 * int (loclatitude / 10000000));

  // THIRD CHARACTER - longitude based (every 2° = 1 gridsq).  Value is the integer of the longitude remainder from previous calculation divided by 2 (degrees in each group).
  lcd.print(NumberCharString[int(RemainderLongA / 2000000)]);

  // FOURTH CHARACTER - latitude based (every 1° = 1 gridsq).  Value is the integer of the latitude remainder from previous calculation divided by 1 (degrees in each group).
  lcd.print(NumberCharString[int(RemainderLatA / 1000000)]);


  // THIRD PAIR (aa-xx)
  /*  Each field can be further divided into (24) 5 minutes longitudinal by (24) 2.5 minute latitudinal zones.
      The first character encodes the longitude and the second encodes the latitude with letters “a” thru “x” (x being the 24th letter of the alphabet).
  */

  // Need to obtain the Remainders from the previous equations.
  RemainderLongB = RemainderLongA - (2000000 * int(RemainderLongA / 2000000));
  RemainderLatB = RemainderLatA - (1000000 * int(RemainderLatA / 1000000));

  // FIFTH CHARACTER - longitude based (every 5' = 1 gridsq).  Value is the integer of the longitude remainder from previous calculation divided by 2 degrees (2,000,000) which is divided by 24.
  lcd.print(LowerCaseCharString[int(RemainderLongB / (2000000 / 24))]);

  // SIXTH CHARACTER - latitude based (every 2.5' = 1 gridsq)).  Value is the integer of the latitude remainder from previous calculation divided by 1 degrees (1,000,000) which is divided by 24.
  lcd.print(LowerCaseCharString[int(RemainderLatB / (1000000 / 24))]);


  // FOURTH PAIR (00-99)
  /*  Each field can be further subdivided into (10) 30 seconds longitudinal by (10) 15 seconds latitudinal zones.
      The first character encodes the longitude and the second encodes the latitude with numbers “0” thru “9”.
  */

  // Need to obtain the Remainders from the previous equations.
  RemainderLongC = RemainderLongB - ((2000000 / 24) * int(RemainderLongB / (2000000 / 24)));
  RemainderLatC = RemainderLatB - ((1000000 / 24) * int(RemainderLatB / (1000000 / 24)));

  // SEVENTH CHARACTER - longitude based (every 30" = 1 gridsq).  Value is based on the 2 degree sub-grid.
  // Value is the integer of the longitude remainder from previous calculation divided by 2 degrees (2,000,000) which is divided by 240 (divided by 24 previously and now a further 10 (0-9 values)).
  lcd.print(NumberCharString[int(RemainderLongC / (2000000 / 240))]);

  // EIGHTH CHARACTER - latitude based (every 15" = 1 gridsq).  Value is based on the 1 degree sub-grid.
  // Value is the integer of the latitude remainder from previous calculation divided by 1 degrees (1,000,000) which is divided by 240 (divided by 24 previously and now a further 10 (0-9 values)).
  lcd.print(NumberCharString[int(RemainderLatC / (1000000 / 240))]);

  // FIFTH PAIR (aa-xx)
  /*  Each field can be further subdivided into (24) 1.25 seconds longitudinal by (24) 0.625 seconds latitudinal zones.
      The first character encodes the longitude and the second encodes the latitude with letters “a” thru “x” (x being the 24th letter of the alphabet).
  */

  // Need to obtain the Remainders from the previous equations.
  RemainderLongD = RemainderLongC - ((2000000 / 240) * int(RemainderLongC / (2000000 / 240)));
  RemainderLatD = RemainderLatC - ((1000000 / 240) * int(RemainderLatC / (1000000 / 240)));

  // NINTH CHARACTER - longitude based (every 1.25" = 1 gridsq).  Value is based on the 2 degree sub-grid.
  // Value is the integer of the longitude remainder from previous calculation divided by 2 degrees (2,000,000) which is divided by 5760 (divided by 240 previously and now a further 24 (a-x values)).
  lcd.print(LowerCaseCharString[int(RemainderLongD / (2000000 / 5760))]);

  // TENTH CHARACTER - latitude based (every 0.625" = 1 gridsq).  Value is based on the 1 degree sub-grid.
  // Value is the integer of the latitude remainder from previous calculation divided by 1 degrees (1,000,000) which is divided by 5760 (divided by 240 previously and now a further 24 (a-x values)).
  lcd.print(LowerCaseCharString[int(RemainderLatD / (1000000 / 5760))]);
}
////////////////////  END LCDprint ////////////////////

////////////////////  PRINT TEMPLATE ////////////////////

void PrintTemplate()
{
  lcd.clear();

  // 1st Line N/S LATITUDE, No of SATS
  lcd.setCursor(3, 0);
  lcd.print("--");
  lcd.write(byte(0));
  lcd.print("--.--");

  lcd.setCursor(15, 0);
  lcd.print(" Sats");
  lcd.setCursor(13, 0);
  lcd.print("--");

  //2nd Line E/W LONGITUDE, SPEED
  lcd.setCursor(2, 1);
  lcd.print("---");
  lcd.write(byte(0));
  lcd.print("--.--");

  //  lcd.setCursor(12, 1);
  //  lcd.print("---");
  //  lcd.setCursor(18, 1);
  //  lcd.write(byte(0));
  //  lcd.print("C");

  // 3rd Line GRID LOC, ALTITUDE
  lcd.setCursor(0, 2);
  lcd.print("----------");

  lcd.setCursor(11, 2);
  lcd.print("----");
  lcd.setCursor(15, 2);
  lcd.print(" mASL");

  // 4th Line TIME UTC DATE
  lcd.setCursor(0, 3);
  lcd.print("HH:MM:SS");

  lcd.setCursor(9, 3);
  lcd.print("UTC");

  lcd.setCursor(13, 3);
  lcd.print("DDMMMYY");
}
//////////////////// END PRINT TEMPLATE ////////////////////