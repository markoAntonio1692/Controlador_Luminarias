/**************************************************************
 *
 * This script tries to auto-detect the baud rate
 * and allows direct AT commands access
 *
 * TinyGSM Getting Started guide:
 *   https://tiny.cc/tinygsm-readme
 *
 **************************************************************/

// Select your modem:
//#define TINY_GSM_MODEM_SIM800
// #define TINY_GSM_MODEM_SIM900
// #define TINY_GSM_MODEM_SIM808
// #define TINY_GSM_MODEM_SIM868
// #define TINY_GSM_MODEM_UBLOX
// #define TINY_GSM_MODEM_M95
// #define TINY_GSM_MODEM_BG96
// #define TINY_GSM_MODEM_A6
// #define TINY_GSM_MODEM_A7
// #define TINY_GSM_MODEM_M590
// #define TINY_GSM_MODEM_MC60
// #define TINY_GSM_MODEM_MC60E
// #define TINY_GSM_MODEM_ESP8266
// #define TINY_GSM_MODEM_XBEE

 #define TINY_GSM_MODEM_SIM7000SSL

// Set serial for debug console (to the Serial Monitor, speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
#ifndef __AVR_ATmega328P__
#define SerialAT Serial1

// or Software Serial on Uno, Nano
#else
#include <SoftwareSerial.h>
SoftwareSerial SerialAT(2, 3);  // RX, TX
#endif

#define TINY_GSM_DEBUG SerialMon
#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 115200

#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(SerialAT);
#endif

// Module baud rate
uint32_t rate = 0; // Set to 0 for Auto-Detect

#define UART_BAUD   9600
#define PIN_DTR     25
#define PIN_TX      27
#define PIN_RX      26
#define PWR_PIN     4  //SIM700g
#define RST_PIN     5  //RST SIM700G
#define LED_PIN     12

void setup() {
 // Set console baud rate
  SerialMon.begin(115200);
  delay(10);
 // Serial1.begin(9600, SERIAL_8N1, 18, 5 ); // A GPS NEO 6M
  pinMode(LED_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);//PIN RESET SIM700G
  
//  WiFi.mode( WIFI_MODE_NULL );
//  btStop();
//  WiFi.disconnect(true);  // Disconnect from the network
//  WiFi.mode(WIFI_OFF);    // Switch WiFi off
  //setCpuFrequencyMhz(80);  //SET FRECUENCIA
  SerialMon.print("CPU Freq: ");
  SerialMon.println(getCpuFrequencyMhz());

  SerialMon.println("");
  SerialMon.println("INICIANDO...");
  
  //setCpuFrequencyMhz(10);  //SET FRECUENCIA

  pinMode(PWR_PIN, OUTPUT);
  for (int x = 0; x <= 10; x++)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
  //digitalWrite(RST_PIN, LOW);
  
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(PWR_PIN, HIGH);
  delay(1000);
  digitalWrite(PWR_PIN, LOW);
  //delay(5000);

  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  SerialMon.println(">>Modem Encendido");
  SerialMon.println(">>Inicializando...");

  //TinyGsmAutoBaud(SerialAT, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);
  // SerialAT.begin(9600);
  modem.restart();
  for (int x = 0; x <= 10; x++)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
  digitalWrite(LED_PIN, HIGH);
}

void loop() {

//  if (!rate) {
//    rate = TinyGsmAutoBaud(SerialAT);
//  }
//
//  if (!rate) {
//    SerialMon.println(F("***********************************************************"));
//    SerialMon.println(F(" Module does not respond!"));
//    SerialMon.println(F("   Check your Serial wiring"));
//    SerialMon.println(F("   Check the module is correctly powered and turned on"));
//    SerialMon.println(F("***********************************************************"));
//    delay(30000L);
//    return;
//  }
//
//  SerialAT.begin(rate);

  // Access AT commands from Serial Monitor
  SerialMon.println(F("***********************************************************"));
  SerialMon.println(F(" You can now send AT commands"));
  SerialMon.println(F(" Enter \"AT\" (without quotes), and you should see \"OK\""));
  SerialMon.println(F(" If it doesn't work, select \"Both NL & CR\" in Serial Monitor"));
  SerialMon.println(F("***********************************************************"));

  while(true) {
    if (SerialAT.available()) {
      SerialMon.write(SerialAT.read());
    }
    if (SerialMon.available()) {
      SerialAT.write(SerialMon.read());
    }
   // delay(0);
  }
}
