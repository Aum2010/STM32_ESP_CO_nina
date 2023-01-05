/*
 * This example check if the firmware loaded on the NINA module
 * is updated.
 *
 * modified by Renzo Mischianti <www.mischianti.org>
 *
 * www.mischianti.org
 *
 */
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
//#include <LwIP.h>
//#include <STM32Ethernet.h>
//#include <EthernetUdp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008
 
// Configure the pins used for the ESP32 connection
#if defined(ADAFRUIT_FEATHER_M4_EXPRESS) || \
  defined(ADAFRUIT_FEATHER_M0) || \
  defined(ADAFRUIT_FEATHER_M0_EXPRESS) || \
  defined(ARDUINO_AVR_FEATHER32U4) || \
  defined(ARDUINO_NRF52840_FEATHER) || \
  defined(ADAFRUIT_ITSYBITSY_M0) || \
  defined(ADAFRUIT_ITSYBITSY_M4_EXPRESS) || \
  defined(ARDUINO_AVR_ITSYBITSY32U4_3V) || \
  defined(ARDUINO_NRF52_ITSYBITSY)
  // Configure the pins used for the ESP32 connection
  #define SPIWIFI       SPI  // The SPI port
  #define SPIWIFI_SS    13   // Chip select pin
  #define ESP32_RESETN  12   // Reset pin
  #define SPIWIFI_ACK   11   // a.k.a BUSY or READY pin
  #define ESP32_GPIO0   -1
#elif defined(ARDUINO_AVR_FEATHER328P)
  #define SPIWIFI       SPI  // The SPI port
  #define SPIWIFI_SS     4   // Chip select pin
  #define ESP32_RESETN   3   // Reset pin
  #define SPIWIFI_ACK    2   // a.k.a BUSY or READY pin
  #define ESP32_GPIO0   -1
#elif defined(TEENSYDUINO)
  #define SPIWIFI       SPI  // The SPI port
  #define SPIWIFI_SS     5   // Chip select pin
  #define ESP32_RESETN   6   // Reset pin
  #define SPIWIFI_ACK    9   // a.k.a BUSY or READY pin
  #define ESP32_GPIO0   -1
#elif defined(ARDUINO_NRF52832_FEATHER )
  #define SPIWIFI       SPI  // The SPI port
  #define SPIWIFI_SS    16   // Chip select pin
  #define ESP32_RESETN  15   // Reset pin
  #define SPIWIFI_ACK    7   // a.k.a BUSY or READY pin
  #define ESP32_GPIO0   -1
#elif defined(ARDUINO_ARCH_STM32) // Here my STM32 configuration
  #define SPIWIFI       SPI  // The SPI port
  #define SPIWIFI_SS    PA4   // Chip select pin
  #define ESP32_RESETN  PA2   // Reset pin
  #define SPIWIFI_ACK   PA3   // a.k.a BUSY or READY pin
  #define ESP32_GPIO0   -1
#elif !defined(SPIWIFI_SS)   // if the wifi definition isnt in the board variant
  // Don't change the names of these #define's! they match the variant ones
  #define SPIWIFI       SPI
  #define SPIWIFI_SS    10   // Chip select pin
  #define SPIWIFI_ACK    7   // a.k.a BUSY or READY pin
  #define ESP32_RESETN   5   // Reset pin
  #define ESP32_GPIO0   -1   // Not connected
#endif

//WiFi variable setup
char ssid[] = "3bb";     // your network SSID (name)
char pass[] = "0844025188";    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;

//UDP variables setup
unsigned int port_udp = 57546;      // local port to listen on
char packetBuffer[256]; //buffer to hold incoming packet
char  data_to_send[] = "h";       // a string to send back
//IPAddress ip_server(192, 168, 1, 57);
const char * udpAddress = "192.168.1.57";
const int udpPort = 57546;
WiFiUDP Udp; 
//EthernetUDP Udp;

//Time variables
unsigned long previous_timer;
unsigned long timer=10000;
 
void setup() {
  // Initialize serial
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
 
  // Print a welcome message
  Serial.println("WiFiNINA firmware check.");
  Serial.println();
 
  // Set up the pins!
  WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESETN, ESP32_GPIO0, &SPIWIFI);
//  WiFi.setPins(13, 14, 16, -1, &SPI);
 
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }
 
  // Print firmware version on the module
  String fv = WiFi.firmwareVersion();
  String latestFv;
  Serial.print("Firmware version installed: ");
  Serial.println(fv);
 
  latestFv = WIFI_FIRMWARE_LATEST_VERSION;
 
  // Print required firmware version
  Serial.print("Latest firmware version available : ");
  Serial.println(latestFv);
 
  // Check if the latest version is installed
  Serial.println();
  if (fv >= latestFv) {
    Serial.println("Check result: PASSED");
  } else {
    Serial.println("Check result: NOT PASSED");
    Serial.println(" - The firmware version on the module do not match the");
    Serial.println("   version required by the library, you may experience");
    Serial.println("   issues or failures.");
  }

  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  Serial.println("Connected to WiFi");
  printWifiStatus();

  WiFi.noLowPowerMode();

  //Inizialization time variable
  previous_timer=micros();
  Udp.begin(udpPort);

  
}
 
void loop() {
  
  //Send data according to timer 
  if(micros() - previous_timer >= timer){
    previous_timer = micros();
    
    //Sending data, with checks to see if sending is successiful or not
    if(Udp.beginPacket(udpAddress, udpPort)){
      Udp.write(data_to_send);
      if( Udp.endPacket()){
        Serial.println("data sent");
      }else{
        Serial.println("error sending data");
      }
    }else{
      Serial.println("error inizializing data sending");
    }
  }

    while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print gateway's IP address:
  IPAddress ip2 = WiFi.gatewayIP();
  Serial.print("Gateway IP Address: ");
  Serial.println(ip2);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
