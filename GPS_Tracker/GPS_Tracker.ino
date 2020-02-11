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

// The number of seconds in between posts
#define samplingRate 30

// The following line can be used to turn off the shield after posting data. This
// could be useful for saving energy for sparse readings but keep in mind that it
// will take longer to get a fix on location after turning back on than if it had
// already been on. Comment out to leave the shield on after it posts data.
// #define turnOffShield // Turn off shield after posting data

// Large buffer for replies
char replybuffer[255];

// Buffers for data
char URL[200] = "http://50.116.63.34/api/Bus/UpdateBusLocation";  // Request URL
char body[100]; // POST body
char latBuff[12], longBuff[12], locBuff[50], speedBuff[12],
     headBuff[12], altBuff[12];

// We default to using software serial
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// The SIM7000 uses LTE CAT-M/NB-IoT
Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();

// Variables
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;
char imei[16] = {0}; // 16 character buffer for IMEI
float latitude, longitude, speed_kph, heading, altitude, second;
uint16_t year;
uint8_t month, day, hour, minute;
uint8_t attempts = 0;

// Start setup
void setup() {
  Serial.begin(9600);
  Serial.println(F("* SIM7000A GPS Tracker *"));

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
  // ex: fona.setNetworkSettings(F("your APN"), F("your username"), F("your password"));
  fona.setNetworkSettings(F("hologram")); // For Hologram SIM card

  // Perform first-time GPS/GPRS setup if the shield is going to remain on,
  // otherwise these won't be enabled in loop()
  #ifndef turnOffShield
  
  // Enable GPS if turnOffShield is not defined. If it is defined skip to **
  while (!fona.enableGPS(true)) {
    Serial.println(F("Failed to turn on GPS, retrying..."));
    delay(2000); // Retry every 2s
  }
  Serial.println(F("Turned on GPS"));
  
  // Disable GPRS just to make sure it was actually off so that we can turn it on
  if (!fona.enableGPRS(false)) {
    Serial.println(F("Failed to disable GPRS"));
  }
  
  // Turn on GPRS
  while (!fona.enableGPRS(true)) {
    Serial.println(F("Failed to enable GPRS, retrying..."));
    delay(2000); // Retry every 2s
  }
  Serial.println(F("Enabled GPRS"));
  #endif
  // **
}

void loop() {
  // Connect to cell network and verify connection
  while (!netStatus()) {
    Serial.println(F("Failed to connect to cell network, retrying..."));
    delay(2000); // Retry every 2s
  }
  Serial.println(F("Connected to cell network"));

  // Turn on GPS if it wasn't on already
  #ifdef turnOffShield
  while (!fona.enableGPS(true)) {
    Serial.println(F("Failed to turn on GPS, retrying..."));
    delay(2000); // Retry every 2s
  }
  Serial.println(F("Turned on GPS"));
  #endif

  // Get a fix on location
  // Use the top line if you want to parse UTC time data as well, the line below it if you don't care
  // while (!fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude, &year, &month, &day, &hour, &minute, &second)) {
  while (!fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude)) {
    Serial.println(F("Failed to get GPS location, retrying..."));
    delay(2000); // Retry every 2s
  }
  Serial.print(F("Latitude: ")); Serial.println(latitude, 6);
  Serial.print(F("Longitude: ")); Serial.println(longitude, 6);
  Serial.print(F("Speed: ")); Serial.println(speed_kph);
  Serial.print(F("Heading: ")); Serial.println(heading);
  Serial.print(F("Altitude: ")); Serial.println(altitude);
  /*
  // Uncomment this if you care about parsing UTC time
  Serial.print(F("Year: ")); Serial.println(year);
  Serial.print(F("Month: ")); Serial.println(month);
  Serial.print(F("Day: ")); Serial.println(day);
  Serial.print(F("Hour: ")); Serial.println(hour);
  Serial.print(F("Minute: ")); Serial.println(minute);
  Serial.print(F("Second: ")); Serial.println(second);
  */

  // If the shield was turned off turn it back on otherwise skip
  #if defined(turnOffShield)
    if (!fona.enableGPRS(false)) Serial.println(F("Failed to disable GPRS!"));

    while (!fona.enableGPRS(true)) {
      Serial.println(F("Failed to enable GPRS, retrying..."));
      delay(2000); // Retry every 2s
    }
    Serial.println(F("Enabled GPRS!"));
  #endif

  // Format the floating point numbers
  dtostrf(latitude, 1, 6, latBuff);
  dtostrf(longitude, 1, 6, longBuff);
  dtostrf(speed_kph, 1, 0, speedBuff);
  dtostrf(heading, 1, 0, headBuff);
  dtostrf(altitude, 1, 1, altBuff);

  // Also construct a combined, comma-separated location array
  // (many platforms require this for dashboards, like Adafruit IO):
  sprintf(locBuff, "%s,%s,%s,%s", speedBuff, latBuff, longBuff, altBuff); // This could look like "10,33.123456,-85.123456,120.5"
  
  // Construct the appropriate URL's and body
  // The IMEI is our device ID
  // This code uses a POST request to send the data

  // Setting up the POST request
  attempts = 0; // This counts the number of failed attempts

 // The Body of the data that will be sent to the endpoint
  sprintf(body, "{\"imei\":%s,\"xCoordinate\":%s,\"yCoordinate\":%s,\"routeId\":%d}", imei, latBuff, longBuff, 1);
  Serial.println(URL);
  Serial.println(body);
  
  while (attempts < 3 && !fona.postData("POST", URL, body)) {
    Serial.println(F("Failed to complete HTTP POST..."));
    attempts++;
    delay(2000);
  }
  delay(30000);
  
  // The code below will only run if turnOffShield is defined
  // It will turn off the shield after posting data
  #ifdef turnOffShield
    // Disable GPRS
    if (!fona.enableGPRS(false)) Serial.println(F("Failed to disable GPRS!"));

    // Turn off GPS
    if (!fona.enableGPS(false)) Serial.println(F("Failed to turn off GPS!"));
  
    // Power off the module. Note that you could instead put it in minimum functionality mode
    // instead of completely turning it off. Experiment different ways depending on your application!
    // You should see the "PWR" LED turn off after this command
    // if (!fona.powerDown()) Serial.println(F("Failed to power down FONA!")); // No retries
    attempt = 0;
    while (attempt < 3 && !fona.powerDown()) { // Try shutting down 
      Serial.println(F("Failed to power down FONA!"));
      attempt++; // Increment counter
      delay(1000);
    }
  #endif
  
  // Alternative to the AT command method above:
  // If your FONA has a PWRKEY pin connected to your MCU, you can pulse PWRKEY
  // LOW for a little bit, then pull it back HIGH, like this:
  // digitalWrite(PWRKEY, LOW);
  // delay(600); // Minimum of 64ms to turn on and 500ms to turn off for FONA 3G. Check spec sheet for other types
  // delay(1300); // Minimum of 1.2s for SIM7000
  // digitalWrite(PWRKEY, HIGH);

  // Shut down the MCU to save power
  /* #ifndef samplingRate
    Serial.println(F("Shutting down..."));
    delay(5); // This is just to read the response of the last AT command before shutting down
    MCU_powerDown(); // You could also write your own function to make it sleep for a certain duration instead
  #else
    // The following lines are for if you want to periodically post data (like GPS tracker)
    Serial.print(F("Waiting for ")); Serial.print(samplingRate); Serial.println(F(" seconds\r\n"));
    delay(samplingRate * 1000UL); // Delay
  
    powerOn(); // Powers on the module if it was off previously

    // Only run the initialization again if the module was powered off
    // since it resets back to 115200 baud instead of 4800.
    #ifdef turnOffShield
      moduleSetup();
      getIMEINum();
    #endif
    
  #endif */
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
      Serial.println(F("SIM7500A"));
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

// Check the network status of the SIM
bool netStatus() {
  int n = fona.getNetworkStatus();
  
  Serial.print(F("Network status ")); Serial.print(n); Serial.print(F(": "));
  if (n == 0) Serial.println(F("Not registered"));
  if (n == 1) Serial.println(F("Registered (home)"));
  if (n == 2) Serial.println(F("Not registered (searching)"));
  if (n == 3) Serial.println(F("Denied"));
  if (n == 4) Serial.println(F("Unknown"));
  if (n == 5) Serial.println(F("Registered roaming"));

  if (!(n == 1 || n == 5)) return false;
  else return true;
}

// Turn off the MCU completely. Can only wake up from RESET button
// However, this can be altered to wake up via a pin change interrupt
void MCU_powerDown() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  ADCSRA = 0; // Turn off ADC
  power_all_disable ();  // Power off ADC, Timer 0 and 1, serial interface
  sleep_enable();
  sleep_cpu();
}
