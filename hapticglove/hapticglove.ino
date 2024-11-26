/*
Arduino code to run random Tass 3:2 pattern on ESP32 C3
Written by shivajiva101@hotmail.com

Code is written utilising millis() and time slices.
Working on a 3:2 ON:OFF pattern with a random index vector
for the data selected at the start of each of the 3 sequences
before the rest period.

The following code is just one simple example of a finite
state machine that meets the requirement. Syncronisation
between hands is achieved using wifi comms with negotiation
for master/slave and timing offsets.
*/

#include <Adafruit_NeoPixel.h>
#include "EEPROM.h"
#include <esp_now.h>
#include <WiFi.h>

#define FINGER_ON_TIME 167  // motor ON time in ms
#define FINGER_OFF_TIME 66  // motor OFF time in ms
#define FRAME_OFF_TIME (8 * (FINGER_ON_TIME + FINGER_OFF_TIME) - FINGER_OFF_TIME)
#define BRIGHTNESS 50    // LED brightness
#define EEPROM_ADDR 0    // start offset
#define EEPROM_SIZE 128  // number of bytes of mem used as EEPROM
#define PWD (0x60)       // used for initialising settings
#define TR_TIME 67       // transmit time in ms
#define RUNTIME 7200000  // session operation period in ms
#define M_REQ 35         // command used for master REQ
#define M_ACK 36         // command used for master ACK
#define M_SYNC 37        // command used for master SYNC
#define M_TXDLY 38       // command used for master TX time
#define S_ACK 39         // command used for slave ACK
#define S_REQ 40         // command used for slave REQ


// Use the correct MAC address for the receiver, not its own!
//uint8_t broadcastAddress[] = { 0xf0, 0x9e, 0x9e, 0xad, 0x91, 0xc8 };  // slave
uint8_t broadcastAddress[] = { 0xf0, 0x9e, 0x9e, 0xad, 0x9c, 0x38 };  // master


// Variable to store if sending data was successful
String success;

// Data Structure
//Must match the receiver structure!
typedef struct struct_message {
  int command;
  unsigned long val;
} __attribute__((packed)) struct_message;

struct_message tx_cache;
struct_message rx_cache;

int iCmd;
int Hand[4] = { 2, 3, 4, 5 };  // physical board pin assignments for haptic motors
byte mode = 1;                 // you can use 2 for the fixed array tass thesis sequence Seq1[]

// Define all 22 non sequential random sequences for 4 fingers
unsigned int Seq[] = { 0x1243, 0x1324, 0x1342, 0x1423, 0x1432, 0x2134, 0x2143, 0x2314, 0x2341, 0x2413, 0x2431,
                       0x3142, 0x3124, 0x3214, 0x3241, 0x3412, 0x3421, 0x4123, 0x4132, 0x4213, 0x4231, 0x4312 };

// Define the Tass thesis pattern as the alt sequence.
unsigned int Seq1[] = { 0x1432, 0x4132, 0x3142, 0x2341, 0x2134, 0x3214 };

bool cng, initSeq, insync, slave, delayTmrActive, broadcast, sync_pulse, dataIn;

uint8_t Fingers[4];  // array to hold the current sequence

volatile uint8_t nSeq, finger, pin, stage, loops, nIdx, tristate, iSync;
unsigned long prevMillis, txDelayTmr, txDelay, delayTmr, delayMillis, total, iVal, synclock, txMillis, timestamp;
unsigned long masterDelay = 101;  // master is always ahead!
unsigned long slaveDelay = 100;
long tmr;

Adafruit_NeoPixel pixels(1, 8, NEO_GRB + NEO_KHZ800);

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  txMillis = millis() - timestamp;
  if (status == 0) {
    success = "Delivery Success :)";
  } else {
    success = "Delivery Fail :(";
  }
  // Serial.println(success);
}

// Callback when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&rx_cache, incomingData, sizeof(rx_cache));
  dataIn = true;            // set flag
  iCmd = rx_cache.command;  // store data, cache will be overwritten!
  iVal = rx_cache.val;
}

/**** EEPROM Functions****/
void saveByte(int address, byte value) {
  EEPROM.write(address, value);
}

int loadByte(int address) {
  return (int)EEPROM.read(address);
}

void saveInt(int address, int number) {
  EEPROM.write(address, (number >> 24) & 0xFF);
  EEPROM.write(address + 1, (number >> 16) & 0xFF);
  EEPROM.write(address + 2, (number >> 8) & 0xFF);
  EEPROM.write(address + 3, number & 0xFF);
}

int loadInt(int address) {
  return (EEPROM.read(address) << 24) + (EEPROM.read(address + 1) << 16) + (EEPROM.read(address + 2) << 8) + EEPROM.read(address + 3);
}

/**** Settings ****/
void loadSettings() {
  if (loadByte(0) == PWD) {
    mode = loadByte(1);
  } else {
    saveSettings();
    mode = 1;
  }
}

void saveSettings() {
  if (loadByte(0) != PWD) {
    saveByte(0, PWD);   // init first byte with password
    saveByte(1, mode);  // save mode
  }
  EEPROM.commit();  // write shadow copy to flash
}

void newRandomSequence() {
  int idx = random(0, nSeq);
  for (int i = 0; i < 4; i++) {
    Fingers[i] = ((Seq[idx] >> (12 - (i * 4))) & 0xf) - 1;
  }
}

void nextSequence() {
  nIdx = (nIdx + 1) % 4;
  for (int i = 0; i < 4; i++) {
    Fingers[i] = ((Seq1[nIdx] >> (12 - (i * 4))) & 0xf) - 1;
  }
}

/**** WS2812 LED ****/
void setPixel(uint8_t r, uint8_t g, uint8_t b, uint8_t bright) {
  pixels.setBrightness(bright);
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();  // update
}

/**** Helper Functions ****/
void SendMessage() {
  timestamp = millis();  // used by callback function
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&tx_cache, sizeof(tx_cache));
}

void setup() {

  // Serial
  Serial.begin(115200);
  //while (!Serial) { ; }  // dev only!!!
  delay(2000);  // wait a few seconds...

  Serial.println("Haptic Glove - ESP32 C3");
  Serial.println("Loading settings from eeprom...");

  loadSettings();

  Serial.printf("mode = %d", mode);
  Serial.println();

  randomSeed(analogRead(A0));  // seed the RNG

  for (int i = 0; i < 4; i++) { pinMode(Hand[i], OUTPUT); }    // set pin modes
  for (int i = 0; i < 4; i++) { digitalWrite(Hand[i], LOW); }  // set pin states

  // Init variables
  initSeq = false;
  insync = false;
  slave = false;
  delayTmrActive = false;
  broadcast = true;
  stage = 0;
  loops = 1;
  iSync = 0;
  tristate = 2;  // 0 = behind, 1 = ahead, 2 = neither
  tmr = 0;
  txDelayTmr = 0;
  txDelay = 250;
  delayTmr = 0;
  synclock = 0;
  txMillis = 0;
  timestamp = 0;

  // use mode to set array size
  switch (mode) {
    case 1:
      nSeq = sizeof(Seq) / sizeof(int);
      Serial.println("using random sequence method");
      break;
    case 2:
      nSeq = sizeof(Seq1) / sizeof(int);
      Serial.println("using fixed sequence method");
      break;
  }

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_2dBm);  // reduce tx power to minimum!

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callback function for transmit status
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  // Register callback function for data received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  // NeoPixel (WS2812B)
  pixels.begin();                   // init NeoPixel strip object
  pixels.clear();                   // set all pixel colours to 'off'
  setPixel(0, 0, 255, BRIGHTNESS);  // set to blue

  prevMillis = millis();  // store milliseconds since power on
}

void loop() {

  unsigned long delta = 0;  // init
  unsigned long now = millis();

  // Tick check
  if (now != prevMillis) {
    delta = now - prevMillis;  // calc delta ms
    prevMillis = now;          // store for next loop
  }

  tmr = tmr + delta;  // update loop timer variable

  if (stage > 0) {
    total = total + delta;  // update runtime total
    synclock = synclock + delta;
  }

  if (initSeq) {
    initSeq = false;  // unset flag
    switch (mode) {
      case 1:
        newRandomSequence();  // select random pattern
        break;
      case 2:
        nextSequence();  // increment pattern in sequence
        break;
    }
    finger = 0;                       // reset
    pin = Hand[Fingers[finger]];      // select pin
    tmr = 0;                          // reset
    stage = 1;                        // initialise first stage
    sync_pulse = false;               // reset sync pulse flag
    digitalWrite(pin, HIGH);          // start motor
    setPixel(0, 255, 0, BRIGHTNESS);  // green
  }

  switch (stage) {
    case 1:
      if (tmr >= FINGER_ON_TIME) {          // timer check
        digitalWrite(pin, LOW);             // stop motor
        setPixel(255, 0, 255, BRIGHTNESS);  // purple
        stage = 2;                          // init next stage (off duration)
        tmr = 0;                            // reset
      }
      break;
    case 2:
      if (tmr >= FINGER_OFF_TIME) {     // timer check
        tmr = 0;                        // reset
        if (finger < 3) {               // next finger check
          finger = finger + 1;          // increment
          pin = Hand[Fingers[finger]];  // select pin
          digitalWrite(pin, HIGH);      // start motor
          stage = 1;                    // init next stage (on duration)
        } else {                        // Apply the antecedant of the pattern ratio (3 loops)
          if (loops < 4) {              // loop check
            loops = loops + 1;          // increment
            initSeq = true;             // init sequence
          } else {
            if (slave) {
              setPixel(255, 0, 0, BRIGHTNESS);  // slave is red
            } else {
              setPixel(0, 0, 255, BRIGHTNESS);  // master is blue
            }
            stage = 3;  // init stage (rest phase)
          }
        }
      }
      break;
    case 3:
      // Apply the consequent of the pattern ratio
      // i.e. the OFF time at the end of the antecedent pattern ratio
      if (tmr >= FRAME_OFF_TIME) {
        loops = 1;                  // reset
        initSeq = total < RUNTIME;  // used for turning off haptics
      }
      break;
  }

  if (!insync) {

    /* The following code is run on startup when isync is false and initiates
      a negotiation of the 2 devices for control.
    */

    txDelayTmr += delta;  // increment timer

    if (dataIn) {

      switch (iCmd) {
        case M_REQ:
          broadcast = false;         // stop broadcasting
          tx_cache.command = S_ACK;  // set response
          SendMessage();             // respond
          dataIn = false;            // data quenched, reset flag
          break;
        case S_ACK:
          slave = false;  // set master
          insync = true;  // block re-entry
          tx_cache.command = M_ACK;
          SendMessage();              // respond
          delayTmrActive = true;      // activate tmr
          delayMillis = masterDelay;  // set delay
          broadcast = false;          // stop broadcasting
          dataIn = false;             // data quenched, reset flag
          break;
        case M_ACK:
          slave = true;              // set slave
          insync = true;             // block re-entry
          delayTmrActive = true;     // activate tmr
          delayMillis = slaveDelay;  // set delay
          dataIn = false;            // data quenched, reset flag
          break;
      }
    } else {
      // check broadcast conditions
      if (broadcast && txDelayTmr >= txDelay) {
        // No valid data received & delay expired
        tx_cache.command = M_REQ;  // set command
        SendMessage();             // send command
        txDelayTmr = 0;            // reset
      }
    }
  } else if (delayTmrActive) {      // status check
    delayTmr += delta;              // increment
    if (delayTmr >= delayMillis) {  // timer check
      delayTmrActive = false;       // block re-entry
      initSeq = true;               // activate haptics
      delayTmr = 0;                 // reset
    }
  }
}
