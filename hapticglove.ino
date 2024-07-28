// Arduino code to run random Tass 3:2 pattern on RP2040 Mbed
// Written for RP2040 Zero by shivajiva101@hotmail.com
// Write the code to use millis() so it's non blocking.
// Working on a 3:2 ON:OFF pattern with a random
// index vector for the data selected at the start of
// each of the 3 sequences before the rest period.
// The following code is a simple example of a finite
// state machine that meets the requirement. Syncronisation
// between hands is achieved using IR comms with negotiation
// for master/slave and a timing offset.

#include <IRremote.hpp>
#include <Adafruit_NeoPixel.h>
#include "EEPROM.h"

#define FINGER_ON_TIME 167
#define FINGER_OFF_TIME 66
#define FRAME_OFF_TIME (8 * (FINGER_ON_TIME + FINGER_OFF_TIME) - FINGER_OFF_TIME)
#define IR_SEND_PIN 6
#define IR_RECEIVE_PIN 7
#define BRIGHTNESS 50
#define DECODE_NEC
#define EEPROM_ADDR 0
#define EEPROM_SIZE 256
#define PWD (0x60)

int Hand[4] = { 2, 3, 4, 5 };  // physical pin assignments for haptic motors
byte mode = 1;                 // set this to 2 to run the fixed sequence in array Seq1

// Define all 22 non sequential random sequences for 4 fingers
unsigned int Seq[] = { 0x1243, 0x1324, 0x1342, 0x1423, 0x1432, 0x2134, 0x2143, 0x2314, 0x2341, 0x2413, 0x2431,
                       0x3142, 0x3124, 0x3214, 0x3241, 0x3412, 0x3421, 0x4123, 0x4132, 0x4213, 0x4231, 0x4312 };

// Define the Tass thesis pattern as the alt sequence.
unsigned int Seq1[] = { 0x1432, 0x4132, 0x3142, 0x2341, 0x2134, 0x3214 };

bool cng, initSeq, insync, slave, delayTmrActive, broadcast;

uint8_t Fingers[4];  // array to hold the current sequence

volatile uint8_t nSeq, finger, pin, stage, loops, nIdx;
unsigned long prevMillis, tmr, txDelayTmr, txDelay, delayTmr, delayMillis;
unsigned long masterDelay = 1000;
// alter this for sync issues, small value changes only. Upload any changes to both devices!
unsigned long slaveDelay = 850;

Adafruit_NeoPixel pixels(1, 16, NEO_GRB + NEO_KHZ800);


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
    saveByte(0, PWD);  // init first byte with password
    saveByte(1, 1);    // save mode
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

void setPixel(uint8_t r, uint8_t g, uint8_t b, uint8_t bright) {
  pixels.setBrightness(bright);
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();  // update
}

void setup() {

  // Serial
  Serial.begin();
  //while (!Serial) {;} // development only!
  Serial.println("Haptic Glove - RP2040 zero");
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
  tmr = 0;
  txDelayTmr = 0;
  delayTmr = 0;

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

  // InfraRed
  IrReceiver.begin(IR_RECEIVE_PIN);  // start rx
  IrSender.begin(IR_SEND_PIN);       // start tx
  disableLEDFeedback();
  Serial.print("Ready to receive IR signals of protocols: ");
  printActiveIRProtocols(&Serial);
  Serial.println();

  // NeoPixel (WS2812B)
  pixels.begin();                   // init NeoPixel strip object
  pixels.clear();                   // set all pixel colours to 'off'
  setPixel(0, 0, 255, BRIGHTNESS);  // set to blue

  txDelay = random(2000, 4000);  // init IR transmit delay

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

  tmr = tmr + delta;  // update timer variable

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
    stage = 1;                        // init stage (motor duration)
    digitalWrite(pin, HIGH);          // start motor
    setPixel(0, 255, 0, BRIGHTNESS);  // green led
  }

  if (stage == 1) {
    if (tmr >= FINGER_ON_TIME) {  // timer check
      digitalWrite(pin, LOW);     // stop motor
      setPixel(255, 0, 0, 5);     // red led
      stage = 2;                  // init next stage (off duration)
      tmr = 0;                    // reset
    }
  }

  if (stage == 2) {
    if (tmr >= FINGER_OFF_TIME) {     // timer check
      tmr = 0;                        // reset
      if (finger < 3) {               // next finger check
        finger++;                     // increment
        pin = Hand[Fingers[finger]];  // select pin
        digitalWrite(pin, HIGH);      // start motor
        stage = 1;                    // init next stage (on duration)
      } else {                        // Apply the antecedant of the pattern ratio (3 loops)
        if (loops < 4) {              // loop check
          loops++;                    // increment
          initSeq = true;             // init sequence
        } else {
          stage = 3;                        // init stage (rest phase)
          setPixel(0, 0, 255, BRIGHTNESS);  // blue led
        }
      }
    }
  }

  if (stage == 3) {
    // Apply the consequent of the pattern ratio
    // i.e. the OFF time at the end of the antecedent pattern ratio
    if (tmr >= FRAME_OFF_TIME) {
      initSeq = true;  // init randomly selected sequence
      loops = 1;       // reset
    }
  }

  /*
  Master & Slave - for the purpose of sync between gloves
  there is a need to determine which hand is acting as the
  master. Some negotiation is necessary since a unified
  firmware is desirable for simplicity. To achieve this
  requirement the introduction of a random offset to the
  delay before broadcasting should increase the odds of
  a quick sync by reducing the odds of a collision if
  power is applied in sync i.e. single powerbank and a
  split USB lead. 
  */

  if (!insync) {

    txDelayTmr = txDelayTmr + delta;  // increment timer

    if (IrReceiver.decode()) {

      // dev code
      //IrReceiver.printIRResultMinimal(&Serial);
      //Serial.println();

      if (IrReceiver.decodedIRData.command == 0x34) {
        // master REQ received
        broadcast = false;  // stop broadcasting
        IrReceiver.stop();  // turn receiver OFF
        IrSender.sendNEC(0x00, 0x35, 0);  // send ack
        IrReceiver.start();               // turn receiver ON
      } else if (IrReceiver.decodedIRData.command == 0x35) {
        // slave ACK received
        slave = false;      // set master
        insync = true;      // block re-entry
        IrReceiver.stop();  // turn receiver OFF
        IrSender.sendNEC(0x00, 0x36, 0);  // send ack
        IrReceiver.start();               // turn receiver ON
        delayTmrActive = true;            // activate tmr
        delayMillis = masterDelay;        // set delay
        broadcast = false;                // stop broadcasting
      } else if (IrReceiver.decodedIRData.command == 0x36) {
        // master ACK received
        slave = true;              // set slave
        insync = true;             // block re-entry
        delayTmrActive = true;     // activate tmr
        delayMillis = slaveDelay;  // set delay
      }
      IrReceiver.resume();
    }

    // No data received and delay expired yet?
    if (broadcast && txDelayTmr >= txDelay) {
      IrReceiver.stop();                // turn receiver OFF
      IrSender.sendNEC(0x00, 0x34, 0);  // broadcast REQ
      IrReceiver.start();               // turn receiver ON
      txDelayTmr = 0;                   // reset
    }

  } else if (delayTmrActive) {      // status check
    delayTmr = delayTmr + delta;    // increment
    if (delayTmr >= delayMillis) {  // timer check
      delayTmrActive = false;       // block re-entry
      initSeq = true;               // activate haptics
      delayTmr = 0;
    }
  }
}