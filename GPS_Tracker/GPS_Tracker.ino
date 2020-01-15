// Need to include the Adafruit FONA Library
// This a modified version of the FONA library and can be found here: https://github.com/botletics/SIM7000-LTE-Shield
#include "Adafruit_FONA.h"

// For lowering Arduino power consumption
#include <avr/sleep.h>
#include <avr/power.h>

// Define the type of Botletics Shield kit that is being used, in our case SIM7000A (the North American version)
#define SIMCOM_7000

// Define the different ports that will be used
#define FONA_PWRKEY 6 // Shield On/Off
#define FONA_RST 7
#define FONA_TX 10 // Microcontroller RX
#define FONA_RX 11 // Microcontroller TX

// Define the protocol that will be used to send the data to the endpoint
#define PROTOCOL_HTTP_POST

// The number of seconds in between posts
#define samplingRate 30

// The following line can be used to turn off the shield after posting data. This
// could be useful for saving energy for sparse readings but keep in mind that it
// will take longer to get a fix on location after turning back on than if it had
// already been on. Comment out to leave the shield on after it posts data.
//#define turnOffShield // Turn off shield after posting data

// Large buffer for replies
char replybuffer[255];

// Buffers for data
char URL[200];  // Request URL
char body[100]; // POST body
char latBuff[12], longBuff[12], locBuff[50], speedBuff[12],
     headBuff[12], altBuff[12], tempBuff[12], battBuff[12];

// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines 
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// Hardware serial is also possible!
//HardwareSerial *fonaSerial = &Serial1;

// The SIM7000 uses LTE CAT-M/NB-IoT
Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();

// Variables
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;
char imei[16] = {0}; // 16 character buffer for IMEI
uint16_t battLevel = 0; // Battery level (percentage)
float latitude, longitude, speed_kph, heading, altitude, second;
uint16_t year;
uint8_t month, day, hour, minute;
uint8_t counter = 0;

// Start setup
void setup() {
  Serial.begin(9600);
  Serial.println(F("*** SIM7000A GPS Tracker ***"));

  pinMode(FONA_RST, OUTPUT);
  digitalWrite(FONA_RST, HIGH); // Default state

  pinMode(FONA_PWRKEY, OUTPUT);
  powerOn(); // Power on the module
  moduleSetup(); // Initializes the shield
  getIMEINum(); // Gets and prints IMEI number of shield

  // Set shield modem to full functionality
  fona.setFunctionality(1); // AT+CFUN=1

  // Enable sleep mode for the shield
  fona.enableSleepMode(true);

  // Configure a GPRS APN, username, and password if needed
  // Username and password are optional and can be removed, but APN is required
  //fona.setNetworkSettings(F("your APN"), F("your username"), F("your password"));
  fona.setNetworkSettings(F("hologram")); // For Hologram SIM card

  // Perform first-time GPS/GPRS setup if the shield is going to remain on,
  // otherwise these won't be enabled in loop()
  #ifndef turnOffShield
  
  // Enable GPS if turnOffShield is not defined. If it is defined skip to **
  while (!fona.enableGPS(true)) {
    Serial.println(F("Failed to turn on GPS, retrying..."));
    delay(2000); // Retry every 2s
  }
  Serial.println(F("Turned on GPS!"));
  
  // Disable GPRS just to make sure it was actually off so that we can turn it on
  if (!fona.enableGPRS(false)) {
    Serial.println(F("Failed to disable GPRS!"));
  }
  
  // Turn on GPRS
  while (!fona.enableGPRS(true)) {
    Serial.println(F("Failed to enable GPRS, retrying..."));
    delay(2000); // Retry every 2s
  }
  Serial.println(F("Enabled GPRS!"));
  #endif
  // **
}

void loop() {
}

// Power on the module
void powerOn() {
  digitalWrite(FONA_PWRKEY, LOW);
  delay(100); // For SIM7000 power needs to be pulsed for a set amount of time
  digitalWrite(FONA_PWRKEY, HIGH);
}

// Initialize the SIM7000A
void moduleSetup() {
  // SIM7000A takes about 3s to turn on
  fonaSS.begin(115200); // Default SIM7000A baud rate
  
  Serial.println(F("Changing to 9600 baud"));
  fonaSS.println("AT+IPR=9600"); // Set baud rate to 9600
  delay(100);
  
  fonaSS.begin(9600);
  if (! fona.begin(fonaSS)) {
    Serial.println(F("Couldn't find FONA"));
    while(1); // Don't proceed if it couldn't find the device
  }

  type = fona.type();
  Serial.println(F("FONA is OK"));
  Serial.print(F("Found "));
  if (type == SIM7500A) {
      Serial.println(F("SIM7500A (American)"));
  } else {
      Serial.println(F("???"));
  }
}

// Get and print IMEI number of shield
void getIMEINum() {
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: "); Serial.println(imei);
  }
}
