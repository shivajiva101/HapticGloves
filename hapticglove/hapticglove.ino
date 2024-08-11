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
maintain sync over time with compensation when IR comms
is blocked by physical objects.
  
Master & Slave Logic

If the sync flag is false broadcast Master REQ periodically
at a random start time between 1 & 5 seconds after powerup
whilst simultaneously listening for decoded responses. If
a Master REQ is received stop broadcasting and return Slave
ACK, the response will be Master ACK and the respective
devices will count down their delays. Master turns
off its IR receiver and broadcasts a sync command in the
consequent off period of the pattern ratio. The slave 
device attempts to sync to the master whenever it receives
a sync command in its consequent off period of the pattern
ratio. When that isn't possible i.e. when the devices cannot
see each other, the code compensates by determining if the
slave is ahead or behind master when IR comms is present
during the first 30 seconds and altering the timer every
6th sequence by 1ms. This strategy is based on the event
pattern observed when IR comms is present.
*/

#include <IRremote.hpp>
#include <Adafruit_NeoPixel.h>
#include "EEPROM.h"

#define FINGER_ON_TIME 167  // motor ON time in ms
#define FINGER_OFF_TIME 66  // motor OFF time in ms
#define FRAME_OFF_TIME (8 * (FINGER_ON_TIME + FINGER_OFF_TIME) - FINGER_OFF_TIME)
#define IR_SEND_PIN 6       // physical device pin
#define IR_RECEIVE_PIN 7    // physical device pin
#define BRIGHTNESS 50       // LED brightness
#define DECODE_NEC          // reqd for IR
#define EEPROM_ADDR 0       // start offset
#define EEPROM_SIZE 128     // number of bytes of mem used as EEPROM
#define PWD (0x60)          // used for initialising settings
#define TR_TIME 67          // transmit time in ms
#define RUNTIME 7200000     // session operation period in ms
#define MASTER_REQ (0x34)   // NEC command used for master REQ
#define MASTER_ACK (0x36)   // NEC IR command used for master ACK
#define MASTER_SYNC (0x40)  // NEC IR command used for master SYNC
#define SLAVE_ACK (0x35)    // NEC IR command used for slave ACK

int Hand[4] = { 2, 3, 4, 5 };  // physical board pin assignments for haptic motors
byte mode = 1;                 // you can use 2 for the fixed array tass thesis sequence Seq1[]

// Define all 22 non sequential random sequences for 4 fingers
unsigned int Seq[] = { 0x1243, 0x1324, 0x1342, 0x1423, 0x1432, 0x2134, 0x2143, 0x2314, 0x2341, 0x2413, 0x2431,
                       0x3142, 0x3124, 0x3214, 0x3241, 0x3412, 0x3421, 0x4123, 0x4132, 0x4213, 0x4231, 0x4312 };

// Define the Tass thesis pattern as the alt sequence.
unsigned int Seq1[] = { 0x1432, 0x4132, 0x3142, 0x2341, 0x2134, 0x3214 };

bool cng, initSeq, insync, slave, delayTmrActive, broadcast, pulsed, bSync;

uint8_t Fingers[4];  // array to hold the current sequence

volatile uint8_t nSeq, finger, pin, stage, loops, nIdx, tristate, iSync;
unsigned long prevMillis, txDelayTmr, txDelay, delayTmr, delayMillis, total;
unsigned long masterDelay = 101; // master is always ahead!
unsigned long slaveDelay = 100;
long tmr;

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

/**** WS2812 LED ****/
void setPixel(uint8_t r, uint8_t g, uint8_t b, uint8_t bright) {
  pixels.setBrightness(bright);
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();  // update
}

/**** IR Helper Functions ****/
void send(uint16_t value) {
  IrReceiver.stop();  // turn receiver OFF
  //unsigned long t = millis();       // store current millis
  IrSender.sendNEC(0x00, value, 0);  // send sync pulse code
  //Serial.printf("sync send time %d", millis() - t);  // send to terminal
  //Serial.println();                                  // newline
  //Serial.flush();      // clear the buffer
  IrReceiver.start();  // turn receiver ON
}

void setup() {

  // Serial
  //Serial.begin();
  //while (!Serial) { ; }  // development only!
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
  bSync = false;
  stage = 0;
  loops = 1;
  iSync = 0;
  tristate = 2;  // 0 = behind, 1 = ahead, 2 = neither
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

  txDelay = random(1000, 5000);  // init IR transmit delay

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
    finger = 0;                   // reset
    pin = Hand[Fingers[finger]];  // select pin
    tmr = 0;                      // reset
    stage = 1;                    // init stage (motor duration)
    bSync = true;
    pulsed = false;                   // reset sync pulse flag
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
          finger++;                     // increment
          pin = Hand[Fingers[finger]];  // select pin
          digitalWrite(pin, HIGH);      // start motor
          stage = 1;                    // init next stage (on duration)
        } else {                        // Apply the antecedant of the pattern ratio (3 loops)
          if (loops < 4) {              // loop check
            loops++;                    // increment
            initSeq = true;             // init sequence
            bSync = true;               // reset flag
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
        if (total >= RUNTIME) {
          initSeq = false;  // timeout reached
        } else {
          initSeq = true;  // init new sequence
        }
        loops = 1;  // reset
      } else {
        if (!slave) {
          // master mode
          if (!pulsed && tmr >= 500) {
            send(MASTER_SYNC);  // broadcast
            pulsed = true;      // set branch flag
          }
        } else {
          // Slave mode!
          // check if IR data was received until frame off time is reached
          if (IrReceiver.decode()) {
            if (IrReceiver.decodedIRData.command == 0x40) {
              /* modify tmr based on value reached when entering this branch
              it should be 500ms + tx time so use as the diff ref. Longer than
              predicted time means the master is behind so retarding the
              timer by the diff should bring us closer to sync. Shorter
              than predicted will give a negative result, the timer is
              advanced by the diff to bring the devices closer in sync.
              The fact entry to this branch requires an IR decode event
              allows the code to continue using timers until the next
              successful sync event.
              */

              iSync = 0;                      // reset counter
              int d = tmr - (500 + TR_TIME);  // calc diff
              if (d > -50 && d < 50) {        // sanity check
                tmr -= d;                     // modify timer var
                // update tristate var
                if (d > 0) {
                  tristate = 0;
                } else if (d < 0) {
                  tristate = 1;
                }
              }
              //Serial.printf("diff: %d", d);
              //Serial.println();
              //Serial.printf("tristate: %d", tristate);
              //Serial.println();
              //Serial.printf("tmr: %u", tmr);
              //Serial.println();
              //Serial.flush();
            }
            IrReceiver.resume();
          } else {
            // no decode event! On the 6th cycle adjust the timer by 1ms
            if (bSync == true && tmr >= 500 + TR_TIME) {
              bSync = false;
              iSync += 1;
              //Serial.printf("tmr: %u", tmr);
              //Serial.println();
              //Serial.printf("sync count: %d", iSync);
              //Serial.println();
              //Serial.flush();
              if (iSync == 6) {
                iSync = 0;  // reset
                switch (tristate) {
                  case 0:
                    tmr -= 1;
                    //Serial.println("subtracting 1ms from timer");
                    //Serial.flush();
                    break;
                  case 1:
                    tmr += 1;
                    //Serial.println("adding 1ms to timer");
                    //Serial.flush();
                    break;
                }
              }
            }
          }
        }
      }
      break;
  }

  if (!insync) {

    txDelayTmr += delta;  // increment timer

    if (IrReceiver.decode()) {

      // dev code
      //IrReceiver.printIRResultMinimal(&Serial);
      //Serial.println();
      IrReceiver.resume();

      switch (IrReceiver.decodedIRData.command) {
        case 0x34:
          // master REQ received
          broadcast = false;  // stop broadcasting
          send(SLAVE_ACK);    // respond
          break;
        case 0x35:
          // slave ACK received
          slave = false;              // set master
          insync = true;              // block re-entry
          send(MASTER_ACK);           // respond
          delayTmrActive = true;      // activate tmr
          delayMillis = masterDelay;  // set delay
          broadcast = false;          // stop broadcasting
          break;
        case 0x36:
          // master ACK received
          slave = true;              // set slave
          insync = true;             // block re-entry
          delayTmrActive = true;     // activate tmr
          delayMillis = slaveDelay;  // set delay
          break;
      }
    } else {
      if (broadcast && txDelayTmr >= txDelay) {
        // No valid data received & delay expired
        send(MASTER_REQ);  // send master REQ
        txDelayTmr = 0;    // reset
      }
    }
  } else if (delayTmrActive) {      // status check
    delayTmr += delta;              // increment
    if (delayTmr >= delayMillis) {  // timer check
      delayTmrActive = false;       // block re-entry
      initSeq = true;               // activate haptics
      delayTmr = 0;                 // reset
    }
    if (!slave) {
      if (!pulsed && delayTmr >= 100) {
        send(MASTER_SYNC);  // broadcast
        pulsed = true;      // set branch flag
      }
    } else {
      // Slave mode!
      // check if IR data was received until delay time is reached
      if (IrReceiver.decode()) {
        if (IrReceiver.decodedIRData.command == 0x40) {
          int d = delayTmr - 100;   // calc diff
          if (d > -50 && d < 50) {  // sanity check
            delayTmr -= d;          // modify timer var
          }
          //Serial.printf("initial diff: %d", d);
          //Serial.println();
          //Serial.flush();
        }
        IrReceiver.resume();
      }
    }
  }
}
