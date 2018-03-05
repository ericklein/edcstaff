/*
  Project Name:   gps
  Developer:      Eric Klein Jr. (temp2@ericklein.com)
  Description:    Communication with gps receiver

  See README.md for target information, revision history, feature requests, etc.
*/

// Library initialization
// LCD screen via i2c
#include <Adafruit_LiquidCrystal.h>
#include <Wire.h>
// 900Mhz radio
#include <RH_RF69.h>
#include <SPI.h>
// GPS receiver
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Assign Arduino pins
// GPS receiver
#define GPSTXPin 9
#define GPSRXPin 8
// 900Mhz radio
#define RFM69_INT     3
#define RFM69_CS      4
#define RFM69_RST     2 

#define GPSBaud 9600
#define ConsoleBaud 115200
#define RF69_FREQ 915.0

// GPS location (for testing)
static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

// Hardware initialization
// 900 Mhz radio
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// LCD connection via i2c
Adafruit_LiquidCrystal lcd(0);

// GPS receiver
SoftwareSerial GPSSerial(GPSTXPin,GPSRXPin);
// TinyGPS++ object
TinyGPSPlus gps;
unsigned long lastUpdateTime = 0;

void setup() 
{
  Serial.begin(ConsoleBaud);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  GPSSerial.begin(GPSBaud);

  // 900Mhz radio
  
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

  lcd.begin(16, 2);
  lcd.setBacklight(HIGH);
}


void loop() {

String localGPSLocation;

 while (GPSSerial.available() > 0)
    gps.encode(GPSSerial.read());

  // Every 5 seconds, do an update.
  if (millis() - lastUpdateTime >= 5000)
  {
    lastUpdateTime = millis();
  // Debug output of basic GPS data
  Serial.print("At : ");
  printDateTime(gps.date, gps.time);
  Serial.print(" Lat = ");
  Serial.print(gps.location.lat(), 6);
  Serial.print(" Long = ");
  Serial.println(gps.location.lng(), 6);

  // This code will be replaced with distance between two receivers
  unsigned long distanceKmToLondon =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON) / 1000;
  Serial.print("Distance to London is ");
  Serial.print(distanceKmToLondon);
  Serial.print(" km, heading ");

  double courseToLondon =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON);
  Serial.print(courseToLondon,2);
  Serial.println(" degrees");

  //Dump local GPS on the second line of the LCD display

  localGPSLocation = " Lat : " + String(gps.location.lat()) + " Long : " + String(gps.location.lng());
  displayLCDMessage(localGPSLocation, 1);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
  }

  if (rf69.available()) {
    // Should be a message for us now   
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf69.recv(buf, &len)) {
      if (!len) return;
      buf[len] = 0;
      // Dump message to serial
      Serial.print("Received: ");
      Serial.println((char*)buf);
      // Dump message to LCD
      displayLCDMessage((char*)buf,0);
      if (strstr((char *)buf, "Hello World")) {
        // Send a reply!
        uint8_t data = strtol(localGPSLocation.c_str(),NULL,2);
        //uint8_t data[] = localGPSLocation;
        rf69.send(data, sizeof(data));
        rf69.waitPacketSent();
        Serial.println("Sent a reply");
      }
    } else {
      Serial.println("Receive failed");
    }
  }
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }
}

void displayLCDMessage(String message,int line)
{
  lcd.setCursor(0, line);
  lcd.print(message);
}
