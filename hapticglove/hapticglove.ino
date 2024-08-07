/*
Arduino code to run random Tass 3:2 pattern on RP2040 Mbed
Written for RP2040 Zero by shivajiva101@hotmail.com

Code is written utilising millis() and time slices.
Working on a 3:2 ON:OFF pattern with a random index vector
for the data selected at the start of each of the 3 sequences
before the rest period.

The following code is just one simple example of a finite
state machine that meets the requirement. Syncronisation
between hands is achieved using IR comms with negotiation
for master/slave and timing offsets plus sync pulses to
maintain sync over time. 
  
Master & Slave Logic

If the sync flag is false broadcast Master REQ periodically
at a random start time between 2 & 4 seconds after powerup
whilst simultaneously listening for decoded responses. If
a Master REQ is received stop broadcasting and return Slave
ACK, the response will be Master ACK and the respective
devices will count down their delays. Master turns
off its IR receiver and broadcasts a sync command in the
consequent off period of the pattern ratio. The slave 
device attempts to sync to the master whenever it receives
a sync command in the consequent off period of the pattern
ratio. NEC Commands used are as follows:

  0x34  Master REQ
  0x35  Slave ACK
  0x36  Master ACK
  0x40  Master SYNC
  
  
*/

#include <IRremote.hpp>
#include <Adafruit_NeoPixel.h>
#include "EEPROM.h"

#define FINGER_ON_TIME 167  // motor ON time in ms
#define FINGER_OFF_TIME 66  // motor OFF time in ms
#define FRAME_OFF_TIME (8 * (FINGER_ON_TIME + FINGER_OFF_TIME) - FINGER_OFF_TIME)
#define IR_SEND_PIN 6     // physical device pin
#define IR_RECEIVE_PIN 7  // physical device pin
#define BRIGHTNESS 50     // LED brightness
#define DECODE_NEC        // reqd for IR
#define EEPROM_ADDR 0     // start offset
#define EEPROM_SIZE 128   // number of bytes of mem used as EEPROM
#define PWD (0x60)        // used for initialising settings
#define TR_TIME 130       // transmit & receive time in ms
#define RUNTIME 7200000   // operation period

int Hand[4] = { 2, 3, 4, 5 };  // physical pin assignments for haptic motors
byte mode = 1;                 // you can use 2 for the fixed array tass thesis sequence Seq1[]

// Define all 22 non sequential random sequences for 4 fingers
unsigned int Seq[] = { 0x1243, 0x1324, 0x1342, 0x1423, 0x1432, 0x2134, 0x2143, 0x2314, 0x2341, 0x2413, 0x2431,
                       0x3142, 0x3124, 0x3214, 0x3241, 0x3412, 0x3421, 0x4123, 0x4132, 0x4213, 0x4231, 0x4312 };

// Define the Tass thesis pattern as the alt sequence.
unsigned int Seq1[] = { 0x1432, 0x4132, 0x3142, 0x2341, 0x2134, 0x3214 };

bool cng, initSeq, insync, slave, delayTmrActive, broadcast, pulsed;

uint8_t Fingers[4];  // array to hold the current sequence

volatile uint8_t nSeq, finger, pin, stage, loops, nIdx;
unsigned long prevMillis, tmr, txDelayTmr, txDelay, delayTmr, delayMillis, total;
unsigned long masterDelay = 1000;
unsigned long slaveDelay = 1000 - TR_TIME;

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

void setPixel(uint8_t r, uint8_t g, uint8_t b, uint8_t bright) {
  pixels.setBrightness(bright);
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();  // update
}

void setup() {

  // Serial
  //Serial.begin();
  //while (!Serial) {;} // development only!
  //Serial.println("Haptic Glove - RP2040 zero");
  //Serial.println("Loading settings from eeprom...");

  loadSettings();

  //Serial.printf("mode = %d", mode);
  //Serial.println();

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
  //Serial.print("Ready to receive IR signals of protocols: ");
  //printActiveIRProtocols(&Serial);
  //Serial.println();

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
  total = total + delta;

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
    pulsed = false;                   // reset sync pulse flag
    digitalWrite(pin, HIGH);          // start motor
    setPixel(0, 255, 0, BRIGHTNESS);  // green
  }

  if (stage == 1) {
    if (tmr >= FINGER_ON_TIME) {          // timer check
      digitalWrite(pin, LOW);             // stop motor
      setPixel(255, 0, 255, BRIGHTNESS);  // purple
      stage = 2;                          // init next stage (off duration)
      tmr = 0;                            // reset
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
          stage = 3;  // init stage (rest phase)
          if (slave) {
            setPixel(255, 0, 0, BRIGHTNESS);  // slave is red
          } else {
            setPixel(0, 0, 255, BRIGHTNESS);  // master is blue
          }
        }
      }
    }
  }

  if (stage == 3) {
    // Apply the consequent of the pattern ratio
    // i.e. the OFF time at the end of the antecedent pattern ratio
    if (tmr >= FRAME_OFF_TIME) {
      if (total >= RUNTIME) {
        initSeq = false;  // timeout reached
      } else {
        initSeq = true;  // init new sequence
      }
      loops = 1;  // reset
    }
    // master mode
    if (!slave) {
      if (!pulsed && tmr >= 500) {
        IrReceiver.stop();                // turn receiver OFF
        unsigned long t = millis();       // store current millis
        IrSender.sendNEC(0x00, 0x40, 0);  // send sync pulse code
        //Serial.printf("sync send time %d", millis() - t);  // send to terminal
        //Serial.println();                                  // newline
        //Serial.flush();      // clear the buffer
        IrReceiver.start();  // turn receiver ON
        pulsed = true;       // set branch flag
      }
    } else {
      // Slave mode!
      // check if IR data was received until frame off time is reached
      if (IrReceiver.decode()) {
        IrReceiver.resume();
        if (IrReceiver.decodedIRData.command == 0x40) {
          /* modify tmr based on value reached when entering this branch
          it should be 500ms plus the transmit and receive time so
          alter by the diff. Longer than the predicted time means
          the master is behind so retarding the timer by the diff
          should bring us closer to sync. Shorter than predicted will
          give a negative result, the timer is advanced by the diff
          to bring the devices in closer sync. The fact entry to this
          branch requires an IR decode event allows the code to continue
          using timers until the next successful sync event.
          */
          int d = tmr - (500 + TR_TIME);  // calc diff
          if (d > -50 && d < 50) {        // sanity check
            tmr -= d;                     // modify timer var
          }
          //Serial.printf("diff is %d", d);
          //Serial.println();
        }
      }
    }
  }

  if (!insync) {

    txDelayTmr += delta;  // increment timer

    if (IrReceiver.decode()) {

      // dev code
      //IrReceiver.printIRResultMinimal(&Serial);
      //Serial.println();
      IrReceiver.resume();

      if (IrReceiver.decodedIRData.command == 0x34) {
        // master REQ received
        broadcast = false;                // stop broadcasting
        IrReceiver.stop();                // turn receiver OFF
        IrSender.sendNEC(0x00, 0x35, 0);  // send ack
        IrReceiver.start();               // turn receiver ON
      } 
      if (IrReceiver.decodedIRData.command == 0x35) {
        // slave ACK received
        slave = false;                    // set master
        insync = true;                    // block re-entry
        IrReceiver.stop();                // turn receiver OFF
        IrSender.sendNEC(0x00, 0x36, 0);  // send ack
        IrReceiver.start();
        delayTmrActive = true;            // activate tmr
        delayMillis = masterDelay;        // set delay
        broadcast = false;                // stop broadcasting
      }
      if (IrReceiver.decodedIRData.command == 0x36) {
        // master ACK received
        slave = true;              // set slave
        insync = true;             // block re-entry
        delayTmrActive = true;     // activate tmr
        delayMillis = slaveDelay;  // set delay
      }
    }

    // No data received and delay expired yet?
    if (broadcast && txDelayTmr >= txDelay) {
      IrReceiver.stop();                // turn receiver OFF
      IrSender.sendNEC(0x00, 0x34, 0);  // broadcast REQ
      IrReceiver.start();               // turn receiver ON
      txDelayTmr = 0;                   // reset
    }
  } else if (delayTmrActive) {      // status check
    delayTmr += delta;    // increment
    if (delayTmr >= delayMillis) {  // timer check
      delayTmrActive = false;       // block re-entry
      initSeq = true;               // activate haptics
      delayTmr = 0;                 // reset
    }
    if (!slave) {
      if (!pulsed && delayTmr >= 200) {
        unsigned long t = millis();     // store current millis
        IrSender.sendNEC(0x00, 0x40, 0);  // send sync pulse code
        pulsed = true;                    // set branch flag
      }
    } else {
      // Slave mode!
      // check if IR data was received until delay time is reached
      if (IrReceiver.decode()) {
        IrReceiver.resume();
        if (IrReceiver.decodedIRData.command == 0x40) {
          int d = delayTmr - (200 + TR_TIME);  // calc diff
          if (d > -50 && d < 50) {             // sanity check
            delayTmr -= d;                     // modify timer var
          }
        }
      }
    }
  }
}
