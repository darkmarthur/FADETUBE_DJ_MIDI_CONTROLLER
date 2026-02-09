#include <MIDIUSB.h>

/*
  FadeTube – USB-MIDI Controller v1 (ATmega32U4)

  Requerimientos:
  - LOAD LEDs: SIEMPRE PRENDIDOS.
  - PLAY LEDs: prender:
      a) por feedback MIDI del DAW (Note 60 y 62), y/o
      b) localmente al presionar el botón (para test / sin DAW).

  Diagnóstico:
  - Si PLAY LEDs nunca prenden: o no hay feedback del DAW, o el mapeo de pines LED está cruzado.
  - Por eso: swaps separados para switches y LEDs.
*/

//////////////////////
// CONFIG PRINCIPAL
//////////////////////
const bool SWAP_PLAY_LOAD_SWITCHES = true;  // <- si Play funciona como Load (switches cruzados), pon true
const bool SWAP_PLAY_LOAD_LEDS     = true;  // <- si LEDs están cruzados, pon true (puede diferir del de switches)

const bool PLAY_LEDS_LOCAL_ON_PRESS = true; // <- prende LED Play mientras presionas, aunque no haya DAW
const bool PLAY_LEDS_FROM_DAW       = true; // <- permite que el DAW controle LEDs Play (Note 60/62)

const bool LED_ACTIVE_HIGH_PLAY = true;     // <- si tu LED prende con HIGH
const bool LED_ACTIVE_HIGH_LOAD = true;     // <- si tu LED prende con HIGH

//////////////////////
// DEBUG SERIAL
//////////////////////
#define DEBUG_SERIAL 1

//////////////////////
// PINES FÍSICOS (tu pinout)
//////////////////////
// Encoder
const int encA  = 2;
const int encB  = 3;
const int encSW = 4;

// Fader
const int faderPin = A0;

// Switches físicos
const int PHY_playA_sw = 5; // D5  (PLAY 1 BTN)
const int PHY_playB_sw = 6; // D6  (PLAY 2 BTN)
const int PHY_loadA_sw = 7; // D7  (LOAD 1 BTN)
const int PHY_loadB_sw = 8; // D8  (LOAD 2 BTN)

// LEDs físicos
const int PHY_playA_led = 9;   // D9  (PLAY 1 LED)
const int PHY_playB_led = 10;  // D10 (PLAY 2 LED)
const int PHY_loadA_led = 16;  // D16 (LOAD 1 LED)
const int PHY_loadB_led = 14;  // D14 (LOAD 2 LED)

//////////////////////
// PINES LÓGICOS (resueltos por swaps)
//////////////////////
static inline int PIN_PLAY_A_SW() { return SWAP_PLAY_LOAD_SWITCHES ? PHY_loadA_sw : PHY_playA_sw; }
static inline int PIN_PLAY_B_SW() { return SWAP_PLAY_LOAD_SWITCHES ? PHY_loadB_sw : PHY_playB_sw; }
static inline int PIN_LOAD_A_SW() { return SWAP_PLAY_LOAD_SWITCHES ? PHY_playA_sw : PHY_loadA_sw; }
static inline int PIN_LOAD_B_SW() { return SWAP_PLAY_LOAD_SWITCHES ? PHY_playB_sw : PHY_loadB_sw; }

static inline int PIN_PLAY_A_LED() { return SWAP_PLAY_LOAD_LEDS ? PHY_loadA_led : PHY_playA_led; }
static inline int PIN_PLAY_B_LED() { return SWAP_PLAY_LOAD_LEDS ? PHY_loadB_led : PHY_playB_led; }
static inline int PIN_LOAD_A_LED() { return SWAP_PLAY_LOAD_LEDS ? PHY_playA_led : PHY_loadA_led; }
static inline int PIN_LOAD_B_LED() { return SWAP_PLAY_LOAD_LEDS ? PHY_playB_led : PHY_loadB_led; }

//////////////////////
// MIDI CONFIG
//////////////////////
const uint8_t MIDI_CH = 0; // Channel 1
const uint8_t CC_CROSSFADER   = 14;
const uint8_t CC_ENCODER_REL  = 20;
const uint8_t CC_ENCODER_PUSH = 21;

const uint8_t NOTE_PLAY_A = 60;
const uint8_t NOTE_LOAD_A = 61;
const uint8_t NOTE_PLAY_B = 62;
const uint8_t NOTE_LOAD_B = 63;

//////////////////////
// DEBOUNCE / FILTROS
//////////////////////
const unsigned long DEBOUNCE_BTN_MS     = 40;
const unsigned long DEBOUNCE_ENC_MS     = 2;
const int           FADER_DEADBAND      = 1;
const unsigned long MIDI_FLUSH_EVERY_MS = 5;

//////////////////////
// ESTADO INPUTS
//////////////////////
// Encoder
int lastA = HIGH;
unsigned long lastEncStepMs = 0;

// Encoder push
bool lastEncSW = HIGH;
unsigned long lastEncSWMs = 0;
bool scrollToggleState = false;

// Buttons (lógicos)
bool lastPlayA = HIGH;
bool lastPlayB = HIGH;
bool lastLoadA = HIGH;
bool lastLoadB = HIGH;

unsigned long lastPlayAms = 0;
unsigned long lastPlayBms = 0;
unsigned long lastLoadAms = 0;
unsigned long lastLoadBms = 0;

// Fader
int lastFaderMidi = -999;
unsigned long lastMidiFlushMs = 0;

//////////////////////
// ESTADO LEDs (PLAY)
//////////////////////
// Estado por DAW (feedback)
bool dawPlayA = false;
bool dawPlayB = false;

// Estado local por press (para test / uso sin DAW)
bool localPlayA = false;
bool localPlayB = false;

//////////////////////
// HELPERS DEBUG
//////////////////////
static inline void dbg(const __FlashStringHelper* s) {
#if DEBUG_SERIAL
  Serial.print(s);
#endif
}
static inline void dbgln(const __FlashStringHelper* s) {
#if DEBUG_SERIAL
  Serial.println(s);
#endif
}
static inline void dbgPinMap() {
#if DEBUG_SERIAL
  Serial.println(F("=== LOGICAL PIN MAP ==="));
  Serial.print(F("SW: PLAY_A=")); Serial.print(PIN_PLAY_A_SW());
  Serial.print(F(" PLAY_B="));   Serial.print(PIN_PLAY_B_SW());
  Serial.print(F(" LOAD_A="));   Serial.print(PIN_LOAD_A_SW());
  Serial.print(F(" LOAD_B="));   Serial.println(PIN_LOAD_B_SW());

  Serial.print(F("LED: PLAY_A=")); Serial.print(PIN_PLAY_A_LED());
  Serial.print(F(" PLAY_B="));     Serial.print(PIN_PLAY_B_LED());
  Serial.print(F(" LOAD_A="));     Serial.print(PIN_LOAD_A_LED());
  Serial.print(F(" LOAD_B="));     Serial.println(PIN_LOAD_B_LED());
  Serial.println(F("======================="));
#endif
}

//////////////////////
// MIDI SEND
//////////////////////
static inline void sendCC(uint8_t cc, uint8_t value) {
  midiEventPacket_t evt = {0x0B, (uint8_t)(0xB0 | MIDI_CH), cc, value};
  MidiUSB.sendMIDI(evt);
#if DEBUG_SERIAL
  Serial.print(F("[TX CC] CC")); Serial.print(cc);
  Serial.print(F(" = ")); Serial.println(value);
#endif
}

static inline void sendNoteOn(uint8_t note, uint8_t vel) {
  midiEventPacket_t evt = {0x09, (uint8_t)(0x90 | MIDI_CH), note, vel};
  MidiUSB.sendMIDI(evt);
#if DEBUG_SERIAL
  Serial.print(F("[TX NOTE ON] ")); Serial.print(note);
  Serial.print(F(" vel=")); Serial.println(vel);
#endif
}

static inline void sendNoteOff(uint8_t note) {
  midiEventPacket_t evt = {0x08, (uint8_t)(0x80 | MIDI_CH), note, 0x00};
  MidiUSB.sendMIDI(evt);
#if DEBUG_SERIAL
  Serial.print(F("[TX NOTE OFF] ")); Serial.println(note);
#endif
}

static inline void maybeFlush() {
  unsigned long now = millis();
  if (now - lastMidiFlushMs >= MIDI_FLUSH_EVERY_MS) {
    lastMidiFlushMs = now;
    MidiUSB.flush();
  }
}

//////////////////////
// LED WRITE
//////////////////////
static inline void writeLED(int pin, bool on, bool activeHigh) {
  // si activeHigh=true: on => HIGH
  // si activeHigh=false: on => LOW
  digitalWrite(pin, (on ^ !activeHigh) ? HIGH : LOW);
}

static inline void forceLoadLEDsOn() {
  writeLED(PIN_LOAD_A_LED(), true, LED_ACTIVE_HIGH_LOAD);
  writeLED(PIN_LOAD_B_LED(), true, LED_ACTIVE_HIGH_LOAD);
}

static inline void applyLEDs() {
  // PLAY LED final = (DAW feedback) OR (local press)
  bool playA_on = (PLAY_LEDS_FROM_DAW ? dawPlayA : false) || (PLAY_LEDS_LOCAL_ON_PRESS ? localPlayA : false);
  bool playB_on = (PLAY_LEDS_FROM_DAW ? dawPlayB : false) || (PLAY_LEDS_LOCAL_ON_PRESS ? localPlayB : false);

  writeLED(PIN_PLAY_A_LED(), playA_on, LED_ACTIVE_HIGH_PLAY);
  writeLED(PIN_PLAY_B_LED(), playB_on, LED_ACTIVE_HIGH_PLAY);

  // LOAD siempre ON
  forceLoadLEDsOn();

#if DEBUG_SERIAL
  Serial.print(F("[LED] PLAY_A=")); Serial.print(playA_on ? F("ON") : F("OFF"));
  Serial.print(F(" PLAY_B=")); Serial.print(playB_on ? F("ON") : F("OFF"));
  Serial.print(F(" | LOAD_A=ON LOAD_B=ON"));
  Serial.print(F(" | daw(A,B)=")); Serial.print(dawPlayA); Serial.print(F(",")); Serial.print(dawPlayB);
  Serial.print(F(" local(A,B)=")); Serial.print(localPlayA); Serial.print(F(",")); Serial.println(localPlayB);
#endif
}

//////////////////////
// BOOT LED TEST (parpadea pines físicos)
//////////////////////
static inline void bootLedTest() {
#if DEBUG_SERIAL
  Serial.println(F("Boot LED Test: blinking PHYSICAL LED pins D9, D10, D16, D14"));
#endif
  int pins[] = { PHY_playA_led, PHY_playB_led, PHY_loadA_led, PHY_loadB_led };
  for (int i = 0; i < 4; i++) {
#if DEBUG_SERIAL
    Serial.print(F(" Blink pin ")); Serial.println(pins[i]);
#endif
    writeLED(pins[i], true, true);  delay(200);
    writeLED(pins[i], false, true); delay(120);
  }
#if DEBUG_SERIAL
  Serial.println(F("Boot LED Test done."));
#endif
}

//////////////////////
// MIDI IN (feedback) -> PLAY LEDs (Note 60/62)
//////////////////////
void handleMidiIn() {
  midiEventPacket_t rx;
  do {
    rx = MidiUSB.read();
    if (rx.header == 0) break;

    uint8_t status = rx.byte1 & 0xF0;
    uint8_t ch     = rx.byte1 & 0x0F;
    uint8_t data1  = rx.byte2;
    uint8_t data2  = rx.byte3;

    if (ch != MIDI_CH) continue;

#if DEBUG_SERIAL
    Serial.print(F("[RX] status=0x")); Serial.print(status, HEX);
    Serial.print(F(" d1=")); Serial.print(data1);
    Serial.print(F(" d2=")); Serial.println(data2);
#endif

    if (status == 0x90) {
      bool isOn = (data2 > 0);
      if (data1 == NOTE_PLAY_A) { dawPlayA = isOn; applyLEDs(); }
      if (data1 == NOTE_PLAY_B) { dawPlayB = isOn; applyLEDs(); }
    } else if (status == 0x80) {
      if (data1 == NOTE_PLAY_A) { dawPlayA = false; applyLEDs(); }
      if (data1 == NOTE_PLAY_B) { dawPlayB = false; applyLEDs(); }
    }
  } while (rx.header != 0);
}

void setup() {
#if DEBUG_SERIAL
  Serial.begin(115200);
  delay(700);
  Serial.println(F("FadeTube Controller boot..."));
  Serial.print(F("SWAP switches=")); Serial.println(SWAP_PLAY_LOAD_SWITCHES ? F("true") : F("false"));
  Serial.print(F("SWAP leds="));     Serial.println(SWAP_PLAY_LOAD_LEDS ? F("true") : F("false"));
  Serial.print(F("PLAY local on press=")); Serial.println(PLAY_LEDS_LOCAL_ON_PRESS ? F("true") : F("false"));
  Serial.print(F("PLAY from DAW="));       Serial.println(PLAY_LEDS_FROM_DAW ? F("true") : F("false"));
#endif

  // Encoder
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);
  pinMode(encSW, INPUT_PULLUP);
  lastA = digitalRead(encA);

  // Switches físicos como input pullup
  pinMode(PHY_playA_sw, INPUT_PULLUP);
  pinMode(PHY_playB_sw, INPUT_PULLUP);
  pinMode(PHY_loadA_sw, INPUT_PULLUP);
  pinMode(PHY_loadB_sw, INPUT_PULLUP);

  // LEDs físicos como output
  pinMode(PHY_playA_led, OUTPUT);
  pinMode(PHY_playB_led, OUTPUT);
  pinMode(PHY_loadA_led, OUTPUT);
  pinMode(PHY_loadB_led, OUTPUT);

  // Test visual rápido para identificar pines
  bootLedTest();

  // Estado inicial
  dawPlayA = dawPlayB = false;
  localPlayA = localPlayB = false;

  dbgPinMap();

  // Aplicar LEDs: LOAD ON, PLAY según estados (inicialmente OFF)
  applyLEDs();
}

void loop() {
  unsigned long now = millis();

  // Mantener LOAD siempre ON (por si algo lo “pisa”)
  forceLoadLEDsOn();

  // 1) MIDI feedback primero
  handleMidiIn();

  // 2) Encoder rotate -> CC20 relative
  int a = digitalRead(encA);
  if (lastA == HIGH && a == LOW) {
    if (now - lastEncStepMs > DEBOUNCE_ENC_MS) {
      lastEncStepMs = now;
      int b = digitalRead(encB);
      sendCC(CC_ENCODER_REL, (b == HIGH) ? 1 : 127);
      maybeFlush();
    }
  }
  lastA = a;

  // 3) Encoder push -> CC21 toggle
  bool sw = digitalRead(encSW);
  if (lastEncSW == HIGH && sw == LOW) {
    if (now - lastEncSWMs > 150) {
      lastEncSWMs = now;
      scrollToggleState = !scrollToggleState;
      sendCC(CC_ENCODER_PUSH, scrollToggleState ? 127 : 0);
      maybeFlush();
    }
  }
  lastEncSW = sw;

  // 4) Buttons -> Notes + local LED feedback (Play)
  // Play A
  bool pA = digitalRead(PIN_PLAY_A_SW());
  if (lastPlayA == HIGH && pA == LOW) {
    if (now - lastPlayAms > DEBOUNCE_BTN_MS) {
      lastPlayAms = now;
      localPlayA = true;           // <- prende LED local mientras presionas
      applyLEDs();
      sendNoteOn(NOTE_PLAY_A, 127);
      maybeFlush();
    }
  } else if (lastPlayA == LOW && pA == HIGH) {
    localPlayA = false;
    applyLEDs();
    sendNoteOff(NOTE_PLAY_A);
    maybeFlush();
  }
  lastPlayA = pA;

  // Play B
  bool pB = digitalRead(PIN_PLAY_B_SW());
  if (lastPlayB == HIGH && pB == LOW) {
    if (now - lastPlayBms > DEBOUNCE_BTN_MS) {
      lastPlayBms = now;
      localPlayB = true;
      applyLEDs();
      sendNoteOn(NOTE_PLAY_B, 127);
      maybeFlush();
    }
  } else if (lastPlayB == LOW && pB == HIGH) {
    localPlayB = false;
    applyLEDs();
    sendNoteOff(NOTE_PLAY_B);
    maybeFlush();
  }
  lastPlayB = pB;

  // Load A
  bool lA = digitalRead(PIN_LOAD_A_SW());
  if (lastLoadA == HIGH && lA == LOW) {
    if (now - lastLoadAms > DEBOUNCE_BTN_MS) {
      lastLoadAms = now;
      sendNoteOn(NOTE_LOAD_A, 127);
      maybeFlush();
    }
  } else if (lastLoadA == LOW && lA == HIGH) {
    sendNoteOff(NOTE_LOAD_A);
    maybeFlush();
  }
  lastLoadA = lA;

  // Load B
  bool lB = digitalRead(PIN_LOAD_B_SW());
  if (lastLoadB == HIGH && lB == LOW) {
    if (now - lastLoadBms > DEBOUNCE_BTN_MS) {
      lastLoadBms = now;
      sendNoteOn(NOTE_LOAD_B, 127);
      maybeFlush();
    }
  } else if (lastLoadB == LOW && lB == HIGH) {
    sendNoteOff(NOTE_LOAD_B);
    maybeFlush();
  }
  lastLoadB = lB;

  // 5) Crossfader -> CC14 absolute
  int raw = analogRead(faderPin);
  raw = (raw + analogRead(faderPin) + analogRead(faderPin) + analogRead(faderPin)) / 4;

  int midi = map(raw, 0, 1023, 0, 127);

  if (lastFaderMidi == -999 || abs(midi - lastFaderMidi) > FADER_DEADBAND) {
    lastFaderMidi = midi;
    sendCC(CC_CROSSFADER, (uint8_t)midi);
    maybeFlush();
  }

  maybeFlush();
}

