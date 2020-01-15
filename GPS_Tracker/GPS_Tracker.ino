// Need to include the Adafruit FONA Library
// This a modified version of the FONA library and can be found here: https://github.com/botletics/SIM7000-LTE-Shield
#include "Adafruit_FONA.h"

// Define the type of Botletics Shield kit that is being used, in our case SIM7000A (the North American version)
#define SIMCOM_7000

// Define the different ports that will be used
#define FONA_PWRKEY 6 // Shield On/Off
#define FONA_RST 7
#define FONA_TX 10 // Microcontroller RX
#define FONA_RX 11 // Microcontroller TX

// Define the protocol that will be used to send the data to the endpoint
#define PROTOCOL_HTTP_POST

// Large buffer for replies
char replybuffer[255];

// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines 
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// Hardware serial is also possible!
//HardwareSerial *fonaSerial = &Serial1;

Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;
char imei[16] = {0}; // 16 character buffer for IMEI
