/*
Basiert auf den ESP32-SBus-Switch Ziege-One (Der RC-Modelbauer)
es wurden hier die CRSF Erweiterungen von Wilhem Meier zur Steuerung
der Funktionen genutzt. Es ist nicht möglich die Einstellungen über den Sender
vorzunehmen nur über die Webseite. Es wird KEIN RC Kanal belegt.
Dieser ESP32-CRFS-Switch funktioniert nur mit der Wilhelm Meier ELRS Version
Allow other extended addresses than just 0xc8 to be routed from TX <-> RX (3.x.x maintenance) #2975
oder ELRS Version 4

CORE 0: CRSF Datenverarbeitung und PWM-Steuerung (Priorität 5)
CORE 1: Arduino loop(), Webserver (Setup-Modus)

*/
//Version 1.1 - Core 0 Auslagerung
/*
   ESP32-CRSF-Switch
   Modifiziert von ESP32-SBus-Switch
   Zweck: CRSF Multiswitch Empfänger mit 8 PWM Ausgängen

 /////Pin Belegung////
 GPIO 13: Wifi Pin (Setup Mode: LOW)
 GPIO 16: CRSF in (RX2)
 GPIO 18: Ausgang 1 (PWM)
 GPIO 19: Ausgang 2 (PWM)
 GPIO 21: Ausgang 3 (PWM)
 GPIO 22: Ausgang 4 (PWM)
 GPIO 23: Ausgang 5 (PWM)
 GPIO 25: Ausgang 6 (PWM)
 GPIO 26: Ausgang 7 (PWM)
 GPIO 27: Ausgang 8 (PWM)
*/

// ==============================================================================
// 1. INCLUDES & GLOBALE DEFINITIONEN
// ==============================================================================

#include <WiFi.h>
#include <EEPROM.h>
#include <stdint.h>
#include <HardwareSerial.h>
#include <freertos/task.h> // Für FreeRTOS Task-Erstellung

// CRSF Protocol Constants
#define RADIO_ADDRESS 0xEE
#define UART_SYNC 0xC8
#define DirectCommands 0x32
#define MWswitch 0x01
#define MWprop 0x02
#define MWset4 0x07
#define MWset4m 0x09
#define Multiswitch 0xA1

uint8_t ModulNR = 1;

// --- EEPROM Adressen und Speicher ---

#define EEPROM_SIZE             64
#define adr_eprom_Modul         0   // Moduladresse (0-255)

#define adr_eprom_pwm_0         1   // Ausgang 1 Default PWM Wert (0-255)
#define adr_eprom_pwm_1         2
#define adr_eprom_pwm_2         3
#define adr_eprom_pwm_3         4
#define adr_eprom_pwm_4         5
#define adr_eprom_pwm_5         6
#define adr_eprom_pwm_6         7
#define adr_eprom_pwm_7         8

#define adr_eprom_mode_0        9   // Ausgang 1 Blinkmodus (0-8)
#define adr_eprom_mode_1        10
#define adr_eprom_mode_2        11
#define adr_eprom_mode_3        12
#define adr_eprom_mode_4        13
#define adr_eprom_mode_5        14
#define adr_eprom_mode_6        15
#define adr_eprom_mode_7        16

#define adr_eprom_intervall_0   17  // Ausgang 1 Blinkmodus (0-8)
#define adr_eprom_intervall_1   18
#define adr_eprom_intervall_2   19
#define adr_eprom_intervall_3   20
#define adr_eprom_intervall_4   21
#define adr_eprom_intervall_5   22
#define adr_eprom_intervall_6   23
#define adr_eprom_intervall_7   24

#define adr_eprom_3pos_0        25
#define adr_eprom_3pos_1        26
#define adr_eprom_3pos_2        27
#define adr_eprom_3pos_3        28
#define adr_eprom_3pos_4        29
#define adr_eprom_3pos_5        30
#define adr_eprom_3pos_6        31
#define adr_eprom_3pos_7        32

// --- GLOBALE VARIABLEN ---

const float Version = 0.4; // Versionsnummer angepasst
uint8_t addressSW = 0;
uint8_t stateSW = 0;
uint8_t addressprop = 0;
uint8_t channelprop = 0;
uint8_t dutyprop = 0;
uint8_t addressset4 = 0;
uint8_t stateHset4 = 0;
uint8_t stateLset4 = 0;
uint8_t countset4m = 0;
uint8_t duty = 0;
uint8_t three_pos_state[8] = {0, 0, 0, 0, 0, 0, 0, 0};
bool propUSE[8] = {false,false,false,false,false,false,false,false};


// PWM
const int freq = 5000;
const int resolution = 8; // 0-255

volatile unsigned char OutPin[8]  ={18, 19, 21, 22, 23, 25, 26, 27}; // Pin-Ausgang

// EEPROM Werte
uint8_t modul_address = 0x01;
uint8_t pwm_wert[8] = {255, 255, 255, 255, 255, 255, 255, 255};
int intervall_wert[9] = {0, 8000, 4000, 2000, 1000, 500, 250, 125, 62};
uint8_t mode[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// Aktuelle Zustände (volatile für Multicore-Zugriff)
volatile uint8_t output_pwm_target[8] = {0};
volatile uint8_t output_pwm_current[8] = {0};

// Wifi/Web
const char* ssid     = "ESP32-CRSF-Switch";
const char* password = "123456789";
WiFiServer server(80);
String header;
unsigned long currentTime;
unsigned long previousTime = 0;
unsigned long currentTime2 = 0;
volatile unsigned long previousTimeLED[8] =  {0, 0, 0, 0, 0, 0, 0, 0}; // volatile für Multicore-Zugriff
const long timeoutTime = 5000;
int Menu = 0;
volatile int x = 0;
volatile unsigned char WifiPin = 13;
volatile unsigned char LedPin = 2;

// Funktions-Prototypen für die Task-Funktion und Helfer
void readCrsfData();
void output_manager();
void Webpage();

// ==============================================================================
// 2. CRC TABELLEN UND FUNKTIONEN
// (Unverändert übernommen)
// ==============================================================================

unsigned char command_crc8tab[256] = {
    0x00, 0xBA, 0xCE, 0x74, 0x26, 0x9C, 0xE8, 0x52, 0x4C, 0xF6, 0x82, 0x38, 0x6A, 0xD0, 0xA4, 0x1E,
    0x98, 0x22, 0x56, 0xEC, 0xBE, 0x04, 0x70, 0xCA, 0xD4, 0x6E, 0x1A, 0xA0, 0xF2, 0x48, 0x3C, 0x86,
    0x8A, 0x30, 0x44, 0xFE, 0xAC, 0x16, 0x62, 0xD8, 0xC6, 0x7C, 0x08, 0xB2, 0xE0, 0x5A, 0x2E, 0x94,
    0x12, 0xA8, 0xDC, 0x66, 0x34, 0x8E, 0xFA, 0x40, 0x5E, 0xE4, 0x90, 0x2A, 0x78, 0xC2, 0xB6, 0x0C,
    0xAE, 0x14, 0x60, 0xDA, 0x88, 0x32, 0x46, 0xFC, 0xE2, 0x58, 0x2C, 0x96, 0xC4, 0x7E, 0x0A, 0xB0,
    0x36, 0x8C, 0xF8, 0x42, 0x10, 0xAA, 0xDE, 0x64, 0x7A, 0xC0, 0xB4, 0x0E, 0x5C, 0xE6, 0x92, 0x28,
    0x24, 0x9E, 0xEA, 0x50, 0x02, 0xB8, 0xCC, 0x76, 0x68, 0xD2, 0xA6, 0x1C, 0x4E, 0xF4, 0x80, 0x3A,
    0xBC, 0x06, 0x72, 0xC8, 0x9A, 0x20, 0x54, 0xEE, 0xF0, 0x4A, 0x3E, 0x84, 0xD6, 0x6C, 0x18, 0xA2,
    0xE6, 0x5C, 0x28, 0x92, 0xC0, 0x7A, 0x0E, 0xB4, 0xAA, 0x10, 0x64, 0xDE, 0x8C, 0x36, 0x42, 0xF8,
    0x7E, 0xC4, 0xB0, 0x0A, 0x58, 0xE2, 0x96, 0x2C, 0x32, 0x88, 0xFC, 0x46, 0x14, 0xAE, 0xDA, 0x60,
    0x6C, 0xD6, 0xA2, 0x18, 0x4A, 0xF0, 0x84, 0x3E, 0x20, 0x9A, 0xEE, 0x54, 0x06, 0xBC, 0xC8, 0x72,
    0xF4, 0x4E, 0x3A, 0x80, 0xD2, 0x68, 0x1C, 0xA6, 0xB8, 0x02, 0x76, 0xCC, 0x9E, 0x24, 0x50, 0xEA,
    0x48, 0xF2, 0x86, 0x3C, 0x6E, 0xD4, 0xA0, 0x1A, 0x04, 0xBE, 0xCA, 0x70, 0x22, 0x98, 0xEC, 0x56,
    0xD0, 0x6A, 0x1E, 0xA4, 0xF6, 0x4C, 0x38, 0x82, 0x9C, 0x26, 0x52, 0xE8, 0xBA, 0x00, 0x74, 0xCE,
    0xC2, 0x78, 0x0C, 0xB6, 0xE4, 0x5E, 0x2A, 0x90, 0x8E, 0x34, 0x40, 0xFA, 0xA8, 0x12, 0x66, 0xDC,
    0x5A, 0xE0, 0x94, 0x2E, 0x7C, 0xC6, 0xB2, 0x08, 0x16, 0xAC, 0xD8, 0x62, 0x30, 0x8A, 0xFE, 0x44};

unsigned char crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

uint8_t crc8_standard(const uint8_t * ptr, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
        crc = crc8tab[crc ^ *ptr++];
    return crc;
}

uint8_t crc8_extended(const uint8_t * ptr, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
        crc = command_crc8tab[crc ^ *ptr++];
    return crc;
}

// ==============================================================================
// 3. EEPROM FUNKTIONEN
// ==============================================================================

void EEprom_Load()
{
  EEPROM.begin(EEPROM_SIZE); // EEPROM Initialisierung hinzugefügt
  modul_address = EEPROM.read(adr_eprom_Modul);
  if (modul_address == 0xFF) modul_address = 1;

  for(int i = 0; i <=7; i++)
  {
    pwm_wert[i] = EEPROM.read(adr_eprom_pwm_0 + i);
    if (pwm_wert[i] == 0xFF) pwm_wert[i] = 255;

    mode[i] = EEPROM.read(adr_eprom_mode_0 + i);
    if (mode[i] == 0xFF) mode[i] = 0;

    intervall_wert[i] = EEPROM.read(adr_eprom_intervall_0 + i);
    if (intervall_wert[i] == 0x0) intervall_wert[i] = 0;

  }
  Serial.println("EEPROM gelesen.");
}

void EEprom_Save() {
  EEPROM.writeInt(adr_eprom_Modul, modul_address);
  Serial.printf("ModulID: %d\n",modul_address);
  EEPROM.writeInt(adr_eprom_pwm_0, pwm_wert[0]);
  EEPROM.writeInt(adr_eprom_pwm_1, pwm_wert[1]);
  EEPROM.writeInt(adr_eprom_pwm_2, pwm_wert[2]);
  EEPROM.writeInt(adr_eprom_pwm_3, pwm_wert[3]);
  EEPROM.writeInt(adr_eprom_pwm_4, pwm_wert[4]);
  EEPROM.writeInt(adr_eprom_pwm_5, pwm_wert[5]);
  EEPROM.writeInt(adr_eprom_pwm_6, pwm_wert[6]);
  EEPROM.writeInt(adr_eprom_pwm_7, pwm_wert[7]);
  EEPROM.writeInt(adr_eprom_mode_0, mode[0]);
  EEPROM.writeInt(adr_eprom_mode_1, mode[1]);
  EEPROM.writeInt(adr_eprom_mode_2, mode[2]);
  EEPROM.writeInt(adr_eprom_mode_3, mode[3]);
  EEPROM.writeInt(adr_eprom_mode_4, mode[4]);
  EEPROM.writeInt(adr_eprom_mode_5, mode[5]);
  EEPROM.writeInt(adr_eprom_mode_6, mode[6]);
  EEPROM.writeInt(adr_eprom_mode_7, mode[7]);
  EEPROM.write(adr_eprom_3pos_0, three_pos_state[0]);
  EEPROM.write(adr_eprom_3pos_1, three_pos_state[1]);
  EEPROM.write(adr_eprom_3pos_2, three_pos_state[2]);
  EEPROM.write(adr_eprom_3pos_3, three_pos_state[3]);
  EEPROM.write(adr_eprom_3pos_4, three_pos_state[4]);
  EEPROM.write(adr_eprom_3pos_5, three_pos_state[5]);
  EEPROM.write(adr_eprom_3pos_6, three_pos_state[6]);
  EEPROM.write(adr_eprom_3pos_7, three_pos_state[7]);

  EEPROM.commit();
  Serial.println("EEPROM gespeichert.\n");
}

// ==============================================================================
// 4. CRSF DATENVERARBEITUNG (CORE 0 TASK)
// ==============================================================================

HardwareSerial CrsfLink(2); // Use UART2
uint8_t rxBuffer[64];
uint8_t rxBufferCount = 0;

void processCrsfFrame() {
  uint8_t length = rxBuffer[1];
  uint8_t frameType = rxBuffer[2];
  uint8_t received_crc = rxBuffer[length + 1]; // CRC ist das letzte Byte im Puffer
  //uint8_t dst = rxBuffer[3]; // Nicht verwendet, aber im Original
  //uint8_t CommandRealm = rxBuffer[5]; // Nicht verwendet, aber im Original
  uint8_t CommandID = rxBuffer[6];
  //uint8_t ModulID = rxBuffer[7]; // Nicht verwendet, aber im Original
  
  // Die CRC-Prüfung beginnt nach den ersten 2 Bytes (Adresse, Länge)
  // Die zu prüfende Länge ist 'length - 1' (Payload + FrameType), da die CRC selbst nicht geprüft wird.
  const uint8_t *data_to_crc = &rxBuffer[2]; // Zeiger auf Frame Type (erstes Payload-Byte)
  uint8_t crc_data_len = length - 1;
  uint8_t calculated_crc;
  bool crc_ok = false;

  // Überprüfung des FrameType, um die richtige CRC-Tabelle zu wählen
  // Direkt CRC8 (z.B. RC-Channels-Pakete, FrameType < 0x32)
  if (frameType < DirectCommands) { 
    calculated_crc = crc8_standard(data_to_crc, crc_data_len);
  } else { // CRC8 Extended (z.B. DirectCommands, FrameType >= 0x32)
    calculated_crc = crc8_standard(data_to_crc, crc_data_len);
  }
  
  if (calculated_crc == received_crc) {crc_ok = true;}

  // Nur weiter verarbeiten, wenn die CRC korrekt ist
  if (!crc_ok) {
    Serial.println("CRC-Fehler, Frame wird verworfen.");
    return;
  }

  if ((frameType == DirectCommands) && (rxBuffer[5] == Multiswitch)){
//      Serial.println("Befehlverarbeitung");
      switch (CommandID) {
        case MWswitch: 
          if (length >= 0x09) 
          {
            addressSW = rxBuffer[7];
            stateSW = rxBuffer[8];
//                Serial.printf("[MWSwitch] Addresse: %d, State: %d\n",addressSW, stateSW);
                if (modul_address == addressSW)
                {
                  for (int i = 0; i < 8; i++) 
                  {
                    // Wichtig: volatile Variablen werden hier aktualisiert, Kern 0 schreibt.
                    if (bitRead(stateSW, i)) 
                    {
                      output_pwm_target[i] = pwm_wert[i]; // Eigener PWM Wert (Ein)
                    } 
                    else 
                    {
                      output_pwm_target[i] = 0; // Aus
                    }
                  }
                }     
              }
              break; 
            case MWprop: 
              if (length == 0x0A) {          
               addressprop = rxBuffer[7];
               channelprop = rxBuffer[8];
               dutyprop = rxBuffer[9];
               duty = map(dutyprop,0,100, 0, 255);

                if ((modul_address == addressprop)&&channelprop < 8) 
                {
                    // Wichtig: volatile Variablen werden hier aktualisiert, Kern 0 schreibt.
                    output_pwm_target[channelprop] = duty;
                    if (dutyprop > 1) {propUSE[channelprop] = true;} else {if (dutyprop == 0) {propUSE[channelprop] = false;}}                                      
                }
              }
              break;
              case MWset4:
              if (length == 0x0A) 
              {
               addressset4 = rxBuffer[7];
               stateHset4 = rxBuffer[8];
               stateLset4 = rxBuffer[9];
                // Wichtig: volatile Variablen werden hier aktualisiert, Kern 0 schreibt.
                //Ausgang1
                if ((three_pos_state[0] == false) && (propUSE[0] == false) && (modul_address == addressset4) && (bitRead (stateLset4 ,(0))==HIGH)) {output_pwm_target[0] = 255;} else {if (propUSE[0] == false) {output_pwm_target[0] = 0;}};
                if ((three_pos_state[0] == true)  && (propUSE[0] == false) && (modul_address == addressset4) && (bitRead (stateLset4 ,(0))==HIGH) && (bitRead (stateLset4 ,(1))==LOW)) {output_pwm_target[0] = 255;};
                if ((three_pos_state[0] == true)  && (propUSE[0] == false) && (modul_address == addressset4) && (bitRead (stateLset4 ,(0))==LOW)  && (bitRead (stateLset4 ,(1))==LOW)) {output_pwm_target[0] = 128;};
                if ((three_pos_state[0] == true)  && (propUSE[0] == false) && (modul_address == addressset4) && (bitRead (stateLset4 ,(0))==LOW)  && (bitRead (stateLset4 ,(1))==HIGH)) {output_pwm_target[0] = 0;};
                //Ausgang2
                if ((three_pos_state[1] == false) && (propUSE[1] == false) && (modul_address == addressset4) && (bitRead (stateLset4 ,(2))==HIGH)) {output_pwm_target[1] = 255;} else {if (propUSE[1] == false) {output_pwm_target[1] = 0;}};
                if ((three_pos_state[1] == true)  && (propUSE[1] == false) && (modul_address == addressset4) && (bitRead (stateLset4 ,(2))==HIGH) && (bitRead (stateLset4 ,(3))==LOW)) {output_pwm_target[1] = 255;};
                if ((three_pos_state[1] == true)  && (propUSE[1] == false) && (modul_address == addressset4) && (bitRead (stateLset4 ,(2))==LOW)  && (bitRead (stateLset4 ,(3))==LOW)) {output_pwm_target[1] = 128;};
                if ((three_pos_state[1] == true)  && (propUSE[1] == false) && (modul_address == addressset4) && (bitRead (stateLset4 ,(2))==LOW)  && (bitRead (stateLset4 ,(3))==HIGH)) {output_pwm_target[1] = 0;};
                //Ausgang3
                if ((three_pos_state[2] == false) && (propUSE[2] == false) && (modul_address == addressset4) && (bitRead (stateLset4 ,(4))==HIGH)) {output_pwm_target[2] = 255;} else {if (propUSE[2] == false) {output_pwm_target[2] = 0;}};
                if ((three_pos_state[2] == true)  && (propUSE[2] == false) && (modul_address == addressset4) && (bitRead (stateLset4 ,(4))==HIGH) && (bitRead (stateLset4 ,(5))==LOW)) {output_pwm_target[2] = 255;};
                if ((three_pos_state[2] == true)  && (propUSE[2] == false) && (modul_address == addressset4) && (bitRead (stateLset4 ,(4))==LOW)  && (bitRead (stateLset4 ,(5))==LOW)) {output_pwm_target[2] = 128;};
                if ((three_pos_state[2] == true)  && (propUSE[2] == false) && (modul_address == addressset4) && (bitRead (stateLset4 ,(4))==LOW)  && (bitRead (stateLset4 ,(5))==HIGH)) {output_pwm_target[2] = 0;};
                //Ausgang4
                if ((three_pos_state[3] == false) && (propUSE[3] == false) && (modul_address == addressset4) && (bitRead (stateLset4 ,(6))==HIGH)) {output_pwm_target[3] = 255;} else {if (propUSE[3] == false) {output_pwm_target[3] = 0;}};
                if ((three_pos_state[3] == true)  && (propUSE[3] == false) && (modul_address == addressset4) && (bitRead (stateLset4 ,(6))==HIGH) && (bitRead (stateLset4 ,(7))==LOW)) {output_pwm_target[3] = 255;};
                if ((three_pos_state[3] == true)  && (propUSE[3] == false) && (modul_address == addressset4) && (bitRead (stateLset4 ,(6))==LOW)  && (bitRead (stateLset4 ,(7))==LOW)) {output_pwm_target[3] = 128;};
                if ((three_pos_state[3] == true)  && (propUSE[3] == false) && (modul_address == addressset4) && (bitRead (stateLset4 ,(6))==LOW)  && (bitRead (stateLset4 ,(7))==HIGH)) {output_pwm_target[3] = 0;};
                //Ausgang5
                if ((three_pos_state[4] == false) && (propUSE[4] == false) && (modul_address == addressset4) && (bitRead (stateHset4 ,(0))==HIGH)) {output_pwm_target[4] = 255;} else {if (propUSE[4] == false) {output_pwm_target[4] = 0;}};
                if ((three_pos_state[4] == true)  && (propUSE[4] == false) && (modul_address == addressset4) && (bitRead (stateHset4 ,(0))==HIGH) && (bitRead (stateHset4 ,(1))==LOW)) {output_pwm_target[4] = 255;};
                if ((three_pos_state[4] == true)  && (propUSE[4] == false) && (modul_address == addressset4) && (bitRead (stateHset4 ,(0))==LOW)  && (bitRead (stateHset4 ,(1))==LOW)) {output_pwm_target[4] = 128;};
                if ((three_pos_state[4] == true)  && (propUSE[4] == false) && (modul_address == addressset4) && (bitRead (stateHset4 ,(0))==LOW)  && (bitRead (stateHset4 ,(1))==HIGH)) {output_pwm_target[4] = 0;};
                //Ausgang6
                if ((three_pos_state[5] == false) && (propUSE[5] == false) && (modul_address == addressset4) && (bitRead (stateHset4 ,(2))==HIGH)) {output_pwm_target[5] = 255;} else {if (propUSE[5] == false) {output_pwm_target[5] = 0;}};
                if ((three_pos_state[5] == true)  && (propUSE[5] == false) && (modul_address == addressset4) && (bitRead (stateHset4 ,(2))==HIGH) && (bitRead (stateHset4 ,(3))==LOW)) {output_pwm_target[5] = 255;};
                if ((three_pos_state[5] == true)  && (propUSE[5] == false) && (modul_address == addressset4) && (bitRead (stateHset4 ,(2))==LOW)  && (bitRead (stateHset4 ,(3))==LOW)) {output_pwm_target[5] = 128;};
                if ((three_pos_state[5] == true)  && (propUSE[5] == false) && (modul_address == addressset4) && (bitRead (stateHset4 ,(2))==LOW)  && (bitRead (stateHset4 ,(3))==HIGH)) {output_pwm_target[5] = 0;};
                //Ausgang7
                if ((three_pos_state[6] == false) && (propUSE[6] == false) && (modul_address == addressset4) && (bitRead (stateHset4 ,(4))==HIGH)) {output_pwm_target[6] = 255;} else {if (propUSE[6] == false) {output_pwm_target[6] = 0;}};
                if ((three_pos_state[6] == true)  && (propUSE[6] == false) && (modul_address == addressset4) && (bitRead (stateHset4 ,(4))==HIGH) && (bitRead (stateHset4 ,(5))==LOW)) {output_pwm_target[6] = 255;};
                if ((three_pos_state[6] == true)  && (propUSE[6] == false) && (modul_address == addressset4) && (bitRead (stateHset4 ,(4))==LOW)  && (bitRead (stateHset4 ,(5))==LOW)) {output_pwm_target[6] = 128;};
                if ((three_pos_state[6] == true)  && (propUSE[6] == false) && (modul_address == addressset4) && (bitRead (stateHset4 ,(4))==LOW)  && (bitRead (stateHset4 ,(5))==HIGH)) {output_pwm_target[6] = 0;};
                //Ausgang8
                if ((three_pos_state[7] == false) && (propUSE[7] == false) && (modul_address == addressset4) && (bitRead (stateHset4 ,(6))==HIGH)) {output_pwm_target[7] = 255;} else {if (propUSE[7] == false) {output_pwm_target[7] = 0;}};            
                if ((three_pos_state[7] == true)  && (propUSE[7] == false) && (modul_address == addressset4) && (bitRead (stateHset4 ,(6))==HIGH) && (bitRead (stateHset4 ,(7))==LOW)) {output_pwm_target[7] = 255;};
                if ((three_pos_state[7] == true)  && (propUSE[7] == false) && (modul_address == addressset4) && (bitRead (stateHset4 ,(6))==LOW)  && (bitRead (stateHset4 ,(7))==LOW)) {output_pwm_target[7] = 128;};
                if ((three_pos_state[7] == true)  && (propUSE[7] == false) && (modul_address == addressset4) && (bitRead (stateHset4 ,(6))==LOW)  && (bitRead (stateHset4 ,(7))==HIGH)) {output_pwm_target[7] = 0;};
              }
              break; 
            case MWset4m: //Wenn mehr als 8Schaltflaechen im Widget verwendet werden
              if (length >= 0x0C) {
               uint8_t countset4m = rxBuffer[7];
               uint8_t t = 8;
               uint8_t addressset4mArry [countset4m];
               uint8_t stateHset4mArry [countset4m];
               uint8_t stateLset4mArry [countset4m];

               // Stelle sicher, dass genügend Daten im Puffer sind
               if (length >= (1 + 1 + 1 + 1 + 1 + 1 + 1 + countset4m * 3 + 1)) 
               { // Adr(1)+Len(1)+FrameT(1)+Dst(1)+Src(1)+Multi(1)+Cmd(1) + Count(1) + N*(Addr+SH+SL) + CRC(1)
                 for(uint8_t j = 0; j < countset4m; ++j) 
                 {
                    addressset4mArry[j] = rxBuffer[t];
                    t++;
                    stateHset4mArry[j] = rxBuffer[t];
                    t++;
                    stateLset4mArry[j] = rxBuffer[t];
    
                    // Wichtig: volatile Variablen werden hier aktualisiert, Kern 0 schreibt.
                    if ((three_pos_state[0] == false) && (propUSE[0] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateLset4mArry[j] ,(0))==HIGH)) {output_pwm_target[0] = 255;} else {if (propUSE[0] == false) {output_pwm_target[0] = 0;}};
                    if ((three_pos_state[0] == true)  && (propUSE[0] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateLset4mArry[j] ,(0))==HIGH) && (bitRead (stateLset4mArry[j] ,(1))==LOW)) {output_pwm_target[0] = 255;};
                    if ((three_pos_state[0] == true)  && (propUSE[0] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateLset4mArry[j] ,(0))==LOW)  && (bitRead (stateLset4mArry[j] ,(1))==LOW)) {output_pwm_target[0] = 128;};
                    if ((three_pos_state[0] == true)  && (propUSE[0] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateLset4mArry[j] ,(0))==LOW)  && (bitRead (stateLset4mArry[j] ,(1))==HIGH)) {output_pwm_target[0] = 0;};
 
                    if ((three_pos_state[1] == false) && (propUSE[1] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateLset4mArry[j] ,(2))==HIGH)) {output_pwm_target[1] = 255;} else {if (propUSE[1] == false) {output_pwm_target[1] = 0;}};
                    if ((three_pos_state[1] == true)  && (propUSE[1] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateLset4mArry[j] ,(2))==HIGH) && (bitRead (stateLset4mArry[j] ,(3))==LOW)) {output_pwm_target[1] = 255;};
                    if ((three_pos_state[1] == true)  && (propUSE[1] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateLset4mArry[j] ,(2))==LOW)  && (bitRead (stateLset4mArry[j] ,(3))==LOW)) {output_pwm_target[1] = 128;};
                    if ((three_pos_state[1] == true)  && (propUSE[1] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateLset4mArry[j] ,(2))==LOW)  && (bitRead (stateLset4mArry[j] ,(3))==HIGH)) {output_pwm_target[1] = 0;};

                    if ((three_pos_state[2] == false) && (propUSE[2] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateLset4mArry[j] ,(4))==HIGH)) {output_pwm_target[2] = 255;} else {if (propUSE[2] == false) {output_pwm_target[2] = 0;}};
                    if ((three_pos_state[2] == true)  && (propUSE[2] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateLset4mArry[j] ,(4))==HIGH) && (bitRead (stateLset4mArry[j] ,(5))==LOW)) {output_pwm_target[2] = 255;};
                    if ((three_pos_state[2] == true)  && (propUSE[2] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateLset4mArry[j] ,(4))==LOW)  && (bitRead (stateLset4mArry[j] ,(5))==LOW)) {output_pwm_target[2] = 128;};
                    if ((three_pos_state[2] == true)  && (propUSE[2] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateLset4mArry[j] ,(4))==LOW)  && (bitRead (stateLset4mArry[j] ,(5))==HIGH)) {output_pwm_target[2] = 0;};

                    if ((three_pos_state[3] == false) && (propUSE[3] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateLset4mArry[j] ,(6))==HIGH)) {output_pwm_target[3] = 255;} else {if (propUSE[3] == false) {output_pwm_target[3] = 0;}};
                    if ((three_pos_state[3] == true)  && (propUSE[3] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateLset4mArry[j] ,(6))==HIGH) && (bitRead (stateLset4mArry[j] ,(7))==LOW)) {output_pwm_target[3] = 255;};
                    if ((three_pos_state[3] == true)  && (propUSE[3] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateLset4mArry[j] ,(6))==LOW)  && (bitRead (stateLset4mArry[j] ,(7))==LOW)) {output_pwm_target[3] = 128;};
                    if ((three_pos_state[3] == true)  && (propUSE[3] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateLset4mArry[j] ,(6))==LOW)  && (bitRead (stateLset4mArry[j] ,(7))==HIGH)) {output_pwm_target[3] = 0;};

                    if ((three_pos_state[4] == false) && (propUSE[4] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateHset4mArry[j] ,(0))==HIGH)) {output_pwm_target[4] = 255;} else {if (propUSE[4] == false) {output_pwm_target[4] = 0;}};
                    if ((three_pos_state[4] == true)  && (propUSE[4] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateHset4mArry[j] ,(0))==HIGH) && (bitRead (stateHset4mArry[j] ,(1))==LOW)) {output_pwm_target[4] = 255;};
                    if ((three_pos_state[4] == true)  && (propUSE[4] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateHset4mArry[j] ,(0))==LOW)  && (bitRead (stateHset4mArry[j] ,(1))==LOW)) {output_pwm_target[4] = 128;};
                    if ((three_pos_state[4] == true)  && (propUSE[4] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateHset4mArry[j] ,(0))==LOW)  && (bitRead (stateHset4mArry[j] ,(1))==HIGH)) {output_pwm_target[4] = 0;};

                    if ((three_pos_state[5] == false) && (propUSE[5] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateHset4mArry[j] ,(2))==HIGH)) {output_pwm_target[5] = 255;} else {if (propUSE[5] == false) {output_pwm_target[5] = 0;}};
                    if ((three_pos_state[5] == true)  && (propUSE[5] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateHset4mArry[j] ,(2))==HIGH) && (bitRead (stateHset4mArry[j] ,(3))==LOW)) {output_pwm_target[5] = 255;};
                    if ((three_pos_state[5] == true)  && (propUSE[5] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateHset4mArry[j] ,(2))==LOW)  && (bitRead (stateHset4mArry[j] ,(3))==LOW)) {output_pwm_target[5] = 128;};
                    if ((three_pos_state[5] == true)  && (propUSE[5] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateHset4mArry[j] ,(2))==LOW)  && (bitRead (stateHset4mArry[j] ,(3))==HIGH)) {output_pwm_target[5] = 0;};

                    if ((three_pos_state[6] == false) && (propUSE[6] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateHset4mArry[j] ,(4))==HIGH)) {output_pwm_target[6] = 255;} else {if (propUSE[6] == false) {output_pwm_target[6] = 0;}};
                    if ((three_pos_state[6] == true)  && (propUSE[6] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateHset4mArry[j] ,(4))==HIGH) && (bitRead (stateHset4mArry[j] ,(5))==LOW)) {output_pwm_target[6] = 255;};
                    if ((three_pos_state[6] == true)  && (propUSE[6] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateHset4mArry[j] ,(4))==LOW)  && (bitRead (stateHset4mArry[j] ,(5))==LOW)) {output_pwm_target[6] = 128;};
                    if ((three_pos_state[6] == true)  && (propUSE[6] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateHset4mArry[j] ,(4))==LOW)  && (bitRead (stateHset4mArry[j] ,(5))==HIGH)) {output_pwm_target[6] = 0;};

                    if ((three_pos_state[7] == false) && (propUSE[7] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateHset4mArry[j] ,(6))==HIGH)) {output_pwm_target[7] = 255;} else {if (propUSE[7] == false) {output_pwm_target[7] = 0;}};  
                    if ((three_pos_state[7] == true)  && (propUSE[7] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateHset4mArry[j] ,(6))==HIGH) && (bitRead (stateHset4mArry[j] ,(7))==LOW)) {output_pwm_target[7] = 255;};
                    if ((three_pos_state[7] == true)  && (propUSE[7] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateHset4mArry[j] ,(6))==LOW)  && (bitRead (stateHset4mArry[j] ,(7))==LOW)) {output_pwm_target[7] = 128;};
                    if ((three_pos_state[7] == true)  && (propUSE[7] == false) && (modul_address == addressset4mArry[j]) && (bitRead (stateHset4mArry[j] ,(6))==LOW)  && (bitRead (stateHset4mArry[j] ,(7))==HIGH)) {output_pwm_target[7] = 0;};

                    t+=4; // Korrektur der Zählweise für den nächsten Satz an Daten
//                    Serial.printf("t %d\n",t); // Debug-Ausgabe
                 }
               }
//              Serial.printf("[MWsetm] Count: %d AdresseSet4M: %d StateH1: %d StateL1: %d AdresseSet4M: %d StateH2: %d StateL2: %d\n",countset4m, addressset4mArry[0],  stateHset4mArry[0], stateLset4mArry[0], addressset4mArry[1],  stateHset4mArry[1], stateLset4mArry[1]);
              }
              break;
          }
      }
}

void readCrsfData()
{
  while (CrsfLink.available())
  {
    uint8_t data = CrsfLink.read();

    // Frame Assembly State Machine
    if (rxBufferCount == 0 && (data == UART_SYNC || data == RADIO_ADDRESS))
    {
      rxBuffer[rxBufferCount++] = data;
    }
    else if (rxBufferCount == 1)
    {
      if (data >= 2 && (data + 2) <= sizeof(rxBuffer))
      {
        rxBuffer[rxBufferCount++] = data;
      }
      else
      {
        rxBufferCount = 0; // Reset on invalid length
      }
    }
    else
    {
      if (rxBufferCount < sizeof(rxBuffer))
      {
        rxBuffer[rxBufferCount++] = data;
      }
      // Check if a complete packet has been received
      // Packet length = Address(1) + Length(1) + Payload(N) + CRC(1)
      if (rxBufferCount == (rxBuffer[1] + 2))
      {
        processCrsfFrame();
        rxBufferCount = 0;
      }
    }
  }
}

// ==============================================================================
// 5. PWM STEUERUNG (CORE 0 TASK)
// ==============================================================================
void output_manager() {
  for(x = 0; x <=7; x++)
  {
    // Kern 0 liest (volatile)
    uint8_t target = output_pwm_target[x]; 
    uint8_t current_mode = mode[x];
    
    // Die Blinklogik nur ausführen, wenn der Ausgang vom CRSF-Kommando Angeschaltet werden soll (target > 0)
    if (target > 0) 
    {
      if (current_mode > 0) // Blinklicht Modus (mode 1-8)
      {
//        unsigned long interval_ms = 8000UL / current_mode;
//       Serial.printf("Mode: %d, \n",current_mode);
        unsigned long interval_ms = intervall_wert[current_mode];        
        // Kern 0 liest und schreibt (volatile)
        currentTime2 = millis(); 
        if (currentTime2 - previousTimeLED[x] >= interval_ms)
        {
          previousTimeLED[x] = currentTime2; // Kern 0 schreibt (volatile)
          
          if (output_pwm_current[x] > 0) // Aktuell AN -> AUSschalten
          {
            output_pwm_current[x] = 0; // Kern 0 schreibt (volatile)
          }
          else // Aktuell AUS -> ANschalten (mit Target-Wert)
          {
            output_pwm_current[x] = target; // Kern 0 schreibt (volatile)
          }
        }
      }
      else // Dauerlicht (mode 0)
      {
        output_pwm_current[x] = target; // Kern 0 schreibt (volatile)
      }
    }
    else // Ausgang ist AUS (target == 0), Zustand zurücksetzen
    {
        output_pwm_current[x] = 0; // Kern 0 schreibt (volatile)
    }

    // PWM Wert setzen. Die ledcWrite Funktion ist in der Regel Thread-sicher.
    ledcWrite(OutPin[x], output_pwm_current[x]); 
  }
}
// ==============================================================================
// 6. WEBOBERFLÄCHE (CORE 1 LOOP)
// ==============================================================================

void Webpage()
{
    // Logik unverändert auf Core 1 (Webpage ist sehr aufwendig)
    WiFiClient client = server.available();
    bool saveNeeded = false; // Flag, um unnötiges Speichern zu verhindern

    if (client) {
      currentTime = millis();
      previousTime = currentTime;
      // Serial.println("New Client connected. Waiting for HTTP request...");
      String currentLine = "";
      header = "";

      // Timeout-Prüfung muss innerhalb der Schleife erfolgen
      unsigned long clientStartTime = millis();

      while (client.connected() && (millis() - clientStartTime <= timeoutTime)) {
        if (client.available()) {
          char c = client.read();
          header += c;
          if (c == '\n') {
            if (currentLine.length() == 0) {

              // === Webseiten Eingaben abfragen und Werte aktualisieren ===

              // Moduladresse abfragen und speichern
              if(header.indexOf("GET /save?Moduladresse=")>=0) {
                int pos1 = header.indexOf('=');
                int pos2 = header.indexOf('&');
                String valueString = header.substring(pos1+1, pos2);
                modul_address = constrain(valueString.toInt(), 0, 255);
                Serial.printf("ModulID: %d\n",modul_address);
                saveNeeded = true;
              }

              // Blinkmodus (Mode) für Ausgang X abfragen
              bool modeUpdate = false;
              if (header.indexOf("GET /save?AusgangMode0=") >= 0) { // Prüfen, ob das Modes-Formular gesendet wurde
                for(int i = 0; i < 8; i++) {
                    String searchString = "AusgangMode" + String(i) + "=";
                    int pos1 = header.indexOf(searchString);
                    if (pos1 >= 0) {
                        int pos2 = header.indexOf('&', pos1);
                        if (pos2 < 0) pos2 = header.indexOf(' ', pos1); // Letzter Parameter
                        
                        int startValue = header.indexOf('=', pos1) + 1; 
                        String valueString = header.substring(startValue, pos2);
                        
                        mode[i] = constrain(valueString.toInt(), 0, 8);
                        modeUpdate = true;
                    }
                }
              }

              if (modeUpdate) {
                Serial.printf("Modes aktualisiert: %d %d %d %d %d %d %d %d\n",mode[0], mode[1],mode[2],mode[3],mode[4],mode[5],mode[6],mode[7]);
                saveNeeded = true;
              }
              
              // Verarbeitung der 3-Positionen-Switch Toggle-Eingabe
              bool threePosUpdate = false;
              for(int i = 0; i < 8; i++) {
                String toggleOn = "GET /toggle?AusgangToggle" + String(i) + "=1";
                String toggleOff = "GET /toggle?AusgangToggle" + String(i) + "=0";

                if (header.indexOf(toggleOn) >= 0) {
                  three_pos_state[i] = 1; // AN
                  Serial.printf("Ausgang %d Toggle: AN\n", i+1);
                  threePosUpdate = true;
                } else if (header.indexOf(toggleOff) >= 0) {
                  three_pos_state[i] = 0; // AUS
                  Serial.printf("Ausgang %d Toggle: AUS\n", i+1);
                  threePosUpdate = true;
                }
              }

              if (threePosUpdate) {
                // Das Speichern der three_pos_state wird über das saveNeeded Flag im EEprom_Save() durchgeführt
                saveNeeded = true;
              }
              
              // Expliziter globaler Speichern-Befehl (von Link unten)
              if (header.indexOf("GET /save\n") >= 0 || header.indexOf("GET /save ") >= 0) {  
                Serial.println("Expliziter Speichern-Befehl erhalten.");
                saveNeeded = true;
              } 
              
              // Reset-Befehl
              if (header.indexOf("GET /reset") >= 0) {  
                modul_address = 1;
                for(x = 0; x <=7; x++)
                {
                  mode[x] = 0;
                  pwm_wert[x] = 255;
                  three_pos_state[x] = 0; // Reset der neuen Variable
                }
                saveNeeded = true; // Nach dem Reset speichern
                Serial.println("Auf Werkseinstellungen zurückgesetzt.");
              } 
              
              // *** KORREKTUR: Speichern nur ausführen, wenn nötig ***
              if (saveNeeded) {
                  EEprom_Save();
              }
              
              // === HTML Seite senden ===
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type:text/html");
              client.println("Connection: close");
              client.println();
    
              // === HTML Seite anzeigen (CRFS Multiswitch 8) ===              
              client.println("<!DOCTYPE html><html>");
              client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
              client.println("<meta http-equiv='refresh' content='10'>"); // Aktualisierung alle 10 Sekunden
              client.println("<link rel=\"icon\" href=\"data:,\">");
              
              // CSS für mobiles Design (iPhone 13)
              client.println("<style>");
              client.println("html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
              client.println("h1 { font-size: 28px; color: #333; }");
              client.println(".container { max-width: 400px; margin: 0 auto; padding: 20px; background: #f4f4f9; border-radius: 10px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }");
              client.println(".card { background: white; padding: 15px; margin-bottom: 15px; border-radius: 8px; text-align: left; }");
              client.println(".input-group label { display: block; font-weight: bold; margin-bottom: 5px; color: #555; }");
              client.println(".input-group input, .input-group select { width: 100%; padding: 10px; margin-bottom: 10px; border: 1px solid #ccc; border-radius: 4px; box-sizing: border-box; font-size: 16px; }");
              client.println(".button-save { background-color: #4CAF50; color: white; padding: 12px 20px; border: none; border-radius: 4px; cursor: pointer; font-size: 18px; width: 100%; margin-top: 10px; }");
              client.println(".button-reset { background-color: #f44336; color: white; padding: 12px 20px; border: none; border-radius: 4px; cursor: pointer; font-size: 18px; width: 100%; margin-top: 5px; }");
              // CSS für Statuszeile (Live-Status)
              client.println(".status-row { display: flex; justify-content: space-around; padding: 10px 0; border: 1px solid #ccc; border-radius: 8px; margin-bottom: 15px; background: #fff; }");
              client.println(".status-indicator-container { text-align: center; width: 12.5%; }");
              client.println(".status-indicator-label { font-size: 11px; color: #555; margin-top: 3px; }");
              // CSS für Statuspunkte
              client.println(".status-indicator { height: 16px; width: 16px; background-color: #bbb; border-radius: 50%; display: block; margin: 0 auto; }");
              client.println(".status-red { background-color: #f44336; }");     // LOW / 0
              client.println(".status-yellow { background-color: #ffeb3b; }");  // NEUTRAL / 128
              client.println(".status-green { background-color: #4CAF50; }");   // HIGH / 255
              //CSS für das Tabellen-Layout
              client.println(".setting-table { width: 100%; border-collapse: collapse; }");
              client.println(".setting-row { display: flex; align-items: center; border-bottom: 1px solid #eee; padding: 16px 0; }");
              client.println(".col-output { flex-basis: 25%; text-align: left; font-weight: bold; }"); // Ausgangsnummer (15%)
              client.println(".col-mode { flex-basis: 45%; }"); // Blink-Mode (55%)
              client.println(".col-toggle { flex-basis: 30%; text-align: right; }"); // Toggle Switch (30%)
              client.println(".table-header { display: flex; font-weight: bold; padding: 12px 0; border-bottom: 2px solid #ccc; }");
              client.println(".header-output { flex-basis: 30%; text-align: left; }");
              client.println(".header-mode { flex-basis: 40%; text-align: left; }");
              client.println(".header-toggle { flex-basis: 30%; text-align: right; }");
              // CSS für den Toggle Switch (nur AN/AUS)
              client.println(".switch { position: relative; display: inline-block; width: 60px; height: 34px; }");
              client.println(".switch input { opacity: 0; width: 0; height: 0; }");
              client.println(".slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #ccc; -webkit-transition: .4s; transition: .4s; border-radius: 34px; }");
              client.println(".slider:before { position: absolute; content: \"\"; height: 26px; width: 26px; left: 4px; bottom: 4px; background-color: white; -webkit-transition: .4s; transition: .4s; border-radius: 50%; }");
              client.println("input:checked + .slider { background-color: #4CAF50; }");
              client.println("input:focus + .slider { box-shadow: 0 0 1px #4CAF50; }");
              client.println("input:checked + .slider:before { -webkit-transform: translateX(26px); -ms-transform: translateX(26px); transform: translateX(26px); }");
              client.println(".slider.round { border-radius: 34px; }");
              client.println(".slider.round:before { border-radius: 50%; }");
              
              client.println("</style></head>");
              
              // Webseiten-Inhalt
              client.println("<body><div class='container'>");
              client.println("<h1>CRFS Multiswitch 8</h1>");
              
              // Live-Status Anzeige
              client.println("<div class='card'>");
              client.println("<h2>Live-Status Ausgaenge</h2>");
              
              client.println("<div class='status-row'>");
              for(int i = 0; i < 8; i++) {
                  // Core 1 liest (volatile)
                  uint8_t current_pwm = output_pwm_current[i]; 
                  String status_class = "status-indicator";
                  String label = "A" + String(i + 1); // Label A1, A2, ...

                  if (current_pwm == 0) status_class += " status-red";
                  else if (current_pwm == 128) status_class += " status-yellow";
                  else if (current_pwm > 128) status_class += " status-green";
                  else status_class += " status-red"; 

                  client.println("<div class='status-indicator-container'>");
                  client.println("<div class='" + status_class + "'></div>");
                  client.println("<div class='status-indicator-label'>" + label + "</div>");
                  client.println("</div>");
              }
              client.println("</div>");
              client.println("</div>");
              
              // Moduladresse Formular
              client.println("<div class='card'>");
              client.println("<form action='/save' method='get'>");
              client.println("<div class='input-group'>");
              client.println("<label for='Moduladresse'>Moduladresse (0-255):</label>");
              client.println("<input type='number' name='Moduladresse' min='0' max='255' value='" + String(modul_address) + "'>");
              client.println("</div>");
//              client.println("<p><b>Aktueller Wert: " + String(modul_address) + "</b></p>");
              client.println("<button type='submit' class='button-save'>Moduladresse speichern</button>");
              client.println("</form>");
              client.println("</div>");

              //Modus-Einstellungen und Toggle-Schalter in der gewünschten Tabellenstruktur
              client.println("<h2>Ausgangs-Einstellungen</h2>");
              client.println("<div class='card'>");
              
              // Header Zeile
              client.println("<div class='table-header'>");
              client.println("<div class='header-output'>Ausgang</div>"); // Zeile 1/Spalte 1
              client.println("<div class='header-mode'>Blink-Intervall</div>"); // Zeile 2/Spalte 2
              client.println("<div class='header-toggle'>3Pos-Schalter</div>"); // Zeile 3/Spalte 3
              client.println("</div>");
              
              // Datenzeilen
              for(int i = 0; i < 8; i++) {
                  
                  client.println("<div class='setting-row'>");
                  
                  // SPALTE 1: Ausgangsnummer
                  client.println("<div class='col-output'>");
                  client.print(String(i + 1));
                  client.println("</div>");
                  
                  // SPALTE 2: Blinkmodus (Auswahl)
                  client.println("<div class='col-mode'>");
                  client.println("<form action='/save' method='get' style='margin:0; padding:0;'>"); // Individuelles Formular für Select
                  client.println("<select name='AusgangMode" + String(i) + "' onchange='this.form.submit()'>");
                  
                  String options[] = {
                      "0=Dauerlicht an (Default)", "1=8s Intervall", "2=4s Intervall",
                      "3=2s Intervall", "4=1s Intervall", "5=0,5s Intervall",
                      "6=0,25s Intervall", "7=0,125s Intervall", "8=0,0625s Intervall"
                  };

                  for (int j = 0; j <= 8; j++) {
                      client.print("<option value='" + String(j) + "'");
                      if (mode[i] == j) client.print(" selected");
                      client.println(">" + options[j] + "</option>");
                  }
                  client.println("</select>");
                  client.println("<input type='hidden' name='force_save' value='1'>"); // Hiddn-Feld zum Speichern nach Änderung
                  client.println("</form>");
                  client.println("</div>");
                  
                  // SPALTE 3: 3-Positionen-Switch Toggle
                  client.println("<div class='col-toggle'>");
                  uint8_t current_toggle_state = three_pos_state[i]; 
                  String toggle_checked = (current_toggle_state == 1) ? "checked" : "";
                  
                  client.print("<label class='switch'>");
                  // AN-Zustand: Link auf "Aus", AUS-Zustand: Link auf "An"
                  if (current_toggle_state == 1) {
                      client.print("<a href='/toggle?AusgangToggle" + String(i) + "=0'>");
                  } else {
                      client.print("<a href='/toggle?AusgangToggle" + String(i) + "=1'>");
                  }
                  
                  client.print("<input type='checkbox' " + toggle_checked + " disabled>");
                  client.print("<span class='slider round'></span>");
                  client.println("</a></label>");
                  client.println("</div>"); // Ende col-toggle
                  
                  client.println("</div>"); // Ende setting-row
              }
              
              client.println("</div>"); // Ende card

              // Speichern-Button für alle Modes
              client.println("<button type='submit' class='button-save'>Alle Modes speichern</button>");
              client.println("</form>");
              
              // Globaler Save/Reset
              client.println("<div class='card' style='margin-top: 20px;'>");
              client.println("<p>Aktuelle PWM (Standard ON-Wert) Output 1: " + String(pwm_wert[0]) + " (wird im Programm nicht geaendert)</p>");
              client.println("<a href=\"/save\"><button class=\"button-save\">Gesamte Konfiguration Speichern</button></a>");
              client.println("<a href=\"/reset\"><button class=\"button-reset\">Auf Werkseinstellungen zuruecksetzen</button></a>");
              client.println("</div>");
              
              client.println("<p style='font-size:12px; margin-top:20px;'>Version: " + String(Version, 1) + "</p>");
              client.println("</div></body></html>");

              client.println();
              break;
            } else {           
              currentLine = "";
            }
          } else if (c != '\r') {
            currentLine += c;
          }
        }
      }
      
      header = "";
      client.stop();
    }
}




// ==============================================================================
// 7. TASK DEFINITION (CORE 0)
// ==============================================================================

/**
 * @brief FreeRTOS Task für die CRSF-Datenverarbeitung und PWM-Steuerung.
 * Läuft dediziert auf Core 0.
 */
void CrsfPwmTask(void * parameter) {
  // Endlosschleife für die Aufgabe
  for (;;) {
    readCrsfData();
    output_manager();
    // Sehr kleiner Delay, um den Core nicht zu blockieren, aber die CRSF-Verarbeitung 
    // bleibt hochfrequent (420k Baud erfordert schnelle Reaktion).
    vTaskDelay(1 / portTICK_PERIOD_MS); 
  }
}

// ==============================================================================
// 8. SETUP & LOOP
// ==============================================================================

void setup()
{
  Serial.begin(115200);
  Serial.println("CRSF-Switch (Core 0: CRSF/PWM, Core 1: Webserver)");
  
  // EEPROM laden muss vor dem Start der Task erfolgen, damit die Config da ist
  EEprom_Load();

  // Initialize UART for CRSF: 420000 Baud, 8N1, RX on pin 16, TX on pin 17 (nicht verwendet), false = nicht invertiert
  CrsfLink.begin(420000, SERIAL_8N1, 16, 17, false);

  pinMode(WifiPin, INPUT_PULLUP);
  pinMode(LedPin, OUTPUT);
  
  // Ausgaenge (PWM Setup)
  for(int i = 0; i < 8; i++) {
    ledcAttach(OutPin[i], freq, resolution);
    ledcWrite(OutPin[i], 0); // Start mit 0 Duty Cycle
  }

  // 1. CRSF/PWM Task auf Core 0 erstellen und starten
  // Core 0 ist für zeitkritische I/O-Aufgaben reserviert.
  xTaskCreatePinnedToCore(
    CrsfPwmTask,      // Funktion, die ausgeführt wird
    "CRSF_PWM_Task",  // Name der Task
    10000,            // Stack-Größe in Bytes (10kB sollte reichen)
    NULL,             // Parameter (nicht benötigt)
    5,                // Priorität (5 ist eine hohe Priorität)
    NULL,             // Task Handle (nicht benötigt)
    0                 // Core-Nummer (0 = zweiter Kern)
  );
  
  // === Initialisierung der Web-Schnittstelle auf Core 1 (Default) ===
  if (!digitalRead(WifiPin))
  {
    digitalWrite(LedPin, HIGH);  
    Serial.print("AP (Zugangspunkt) einstellen…");
    WiFi.softAP(ssid, password);

    Serial.println("IP Adresse einstellen");
    IPAddress Ip(192, 168, 1, 1);
    IPAddress NMask(255, 255, 255, 0);
    WiFi.softAPConfig(Ip, Ip, NMask);
  
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP-IP-Adresse: ");
    Serial.println(IP);
  
    server.begin();  // Start Webserver
  }
  else
  {
    Serial.println("Kein AP-Normalmodus");
    digitalWrite(LedPin, LOW);    
  }
}

/**
 * @brief Die Haupt-Arduino-Loop, die auf Core 1 läuft.
 * Wird nur für die Webseiten-Verarbeitung im Setup-Modus verwendet.
 * Im Normalmodus wird Core 1 in den Ruhezustand versetzt.
 */
void loop()
{
  // === Setup Modus (GPIO13 LOW) ===
  if (digitalRead(WifiPin) == LOW) 
  {     
    Webpage();
    // Ein kleiner Delay, um dem Webserver-Task genug Zeit zu geben und gleichzeitig 
    // andere Tasks (wie die auf Core 0) nicht zu verhungern lassen.
    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
  
  // === Normal-Modus (GPIO13 HIGH) ===
  else 
  {
    // Da CRSF/PWM nun auf Core 0 läuft, ist Core 1 (die loop-Funktion) im Normalmodus 
    // vollständig frei und wird in den Ruhezustand (max. Delay) versetzt.
    vTaskDelay(portMAX_DELAY); 
  }
}
