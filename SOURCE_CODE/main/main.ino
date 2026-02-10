#include <MIDIUSB.h>

/*
  FadeTube – USB-MIDI Controller v1 (ATmega32U4)

  OBJETIVO:
  - Los 4 LEDs (PLAY A/B, LOAD A/B) sincronizados con mensajes REMOTOS del DAW (MIDI IN).
  - Opcional: fallback local (prender mientras presionas) para pruebas.
  - NUEVO: LOAD LEDs "siempre ON", pero se apagan SOLO mientras presionas su botón y luego vuelven a ON.

  DAW FEEDBACK (REMOTE):
  - NoteOn/NoteOff (o NoteOn vel=0) para:
      PLAY A = Note 60
      LOAD A = Note 61
      PLAY B = Note 62
      LOAD B = Note 63
*/

//////////////////////
// CONFIG PRINCIPAL
//////////////////////
const bool SWAP_PLAY_LOAD_SWITCHES = true;   // si los switches Play/Load están cruzados
const bool SWAP_PLAY_LOAD_LEDS     = true;   // si los LEDs Play/Load están cruzados

// MODO LED
const bool LEDS_FROM_DAW           = true;   // si true, el DAW puede controlar LEDs vía MIDI IN
const bool LEDS_LOCAL_FALLBACK     = true;   // si true, al presionar se prende local (fallback)

// LOAD behavior:
// - true  = LOAD LEDs normalmente ON, y se apagan SOLO mientras presionas su botón
// - false = LOAD LEDs siguen modo remoto/local como los PLAY
const bool LOAD_LEDS_ALWAYS_ON     = true;
const bool LOAD_LEDS_OFF_WHILE_PRESSED = true;  // solo aplica si LOAD_LEDS_ALWAYS_ON=true

// Polaridad
const bool LED_ACTIVE_HIGH_PLAY = true;
const bool LED_ACTIVE_HIGH_LOAD = true;

// DEBUG
#define DEBUG_SERIAL 1
const unsigned long DEBUG_THROTTLE_MS = 40; // evita spam excesivo

//////////////////////
// PINES
//////////////////////
// Encoder
const int encA  = 2;
const int encB  = 3;
const int encSW = 4;

// Fader
const int faderPin = A0;

// Switches físicos (según tu PCB real)
const int PHY_playA_sw = 5; // D5
const int PHY_playB_sw = 6; // D6
const int PHY_loadA_sw = 7; // D7
const int PHY_loadB_sw = 8; // D8

// LEDs físicos
const int PHY_playA_led = 9;   // D9
const int PHY_playB_led = 10;  // D10
const int PHY_loadA_led = 16;  // D16
const int PHY_loadB_led = 14;  // D14

//////////////////////
// PINES LÓGICOS (swaps)
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
int lastA = HIGH;
unsigned long lastEncStepMs = 0;

bool lastEncSW = HIGH;
unsigned long lastEncSWMs = 0;
bool scrollToggleState = false;

bool lastPlayA = HIGH;
bool lastPlayB = HIGH;
bool lastLoadA = HIGH;
bool lastLoadB = HIGH;

unsigned long lastPlayAms = 0;
unsigned long lastPlayBms = 0;
unsigned long lastLoadAms = 0;
unsigned long lastLoadBms = 0;

int lastFaderMidi = -999;
unsigned long lastMidiFlushMs = 0;

//////////////////////
// ESTADO LEDs
//////////////////////
// Estado remoto (DAW)
bool dawPlayA = false;
bool dawPlayB = false;
bool dawLoadA = false;
bool dawLoadB = false;

// Estado local (fallback)
bool localPlayA = false;
bool localPlayB = false;
bool localLoadA = false;
bool localLoadB = false;

// Estado final aplicado (para log por cambios)
bool appliedPlayA = false;
bool appliedPlayB = false;
bool appliedLoadA = false;
bool appliedLoadB = false;

unsigned long lastDebugMs = 0;

//////////////////////
// HELPERS
//////////////////////
static inline void maybeFlush() {
  unsigned long now = millis();
  if (now - lastMidiFlushMs >= MIDI_FLUSH_EVERY_MS) {
    lastMidiFlushMs = now;
    MidiUSB.flush();
  }
}

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

static inline void writeLED(int pin, bool on, bool activeHigh) {
  digitalWrite(pin, (on ^ !activeHigh) ? HIGH : LOW);
}

static inline void debugLedStateIfChanged(bool pA, bool pB, bool lA, bool lB) {
#if DEBUG_SERIAL
  if (pA != appliedPlayA || pB != appliedPlayB || lA != appliedLoadA || lB != appliedLoadB) {
    unsigned long now = millis();
    if (now - lastDebugMs >= DEBUG_THROTTLE_MS) {
      lastDebugMs = now;

      Serial.print(F("[LED APPLY] PLAY_A=")); Serial.print(pA ? F("ON") : F("OFF"));
      Serial.print(F(" PLAY_B="));          Serial.print(pB ? F("ON") : F("OFF"));
      Serial.print(F(" LOAD_A="));          Serial.print(lA ? F("ON") : F("OFF"));
      Serial.print(F(" LOAD_B="));          Serial.print(lB ? F("ON") : F("OFF"));

      Serial.print(F(" | daw="));
      Serial.print(dawPlayA); Serial.print(F(",")); Serial.print(dawPlayB); Serial.print(F(","));
      Serial.print(dawLoadA); Serial.print(F(",")); Serial.print(dawLoadB);

      Serial.print(F(" local="));
      Serial.print(localPlayA); Serial.print(F(",")); Serial.print(localPlayB); Serial.print(F(","));
      Serial.print(localLoadA); Serial.print(F(",")); Serial.println(localLoadB);
    }
  }
#endif
}

static inline void applyLEDs() {
  // PLAY: remoto OR fallback
  bool playA_on = (LEDS_FROM_DAW ? dawPlayA : false) || (LEDS_LOCAL_FALLBACK ? localPlayA : false);
  bool playB_on = (LEDS_FROM_DAW ? dawPlayB : false) || (LEDS_LOCAL_FALLBACK ? localPlayB : false);

  // LOAD:
  // - Si LOAD_LEDS_ALWAYS_ON: ON por default, y si OFF_WHILE_PRESSED: OFF SOLO mientras presionas.
  // - Si no: remoto OR fallback (como PLAY).
  bool loadA_on;
  bool loadB_on;

  if (LOAD_LEDS_ALWAYS_ON) {
    if (LOAD_LEDS_OFF_WHILE_PRESSED) {
      loadA_on = !localLoadA; // localLoadA=true mientras presionas => LED OFF
      loadB_on = !localLoadB;
    } else {
      loadA_on = true;
      loadB_on = true;
    }
  } else {
    loadA_on = (LEDS_FROM_DAW ? dawLoadA : false) || (LEDS_LOCAL_FALLBACK ? localLoadA : false);
    loadB_on = (LEDS_FROM_DAW ? dawLoadB : false) || (LEDS_LOCAL_FALLBACK ? localLoadB : false);
  }

  debugLedStateIfChanged(playA_on, playB_on, loadA_on, loadB_on);

  writeLED(PIN_PLAY_A_LED(), playA_on, LED_ACTIVE_HIGH_PLAY);
  writeLED(PIN_PLAY_B_LED(), playB_on, LED_ACTIVE_HIGH_PLAY);
  writeLED(PIN_LOAD_A_LED(), loadA_on, LED_ACTIVE_HIGH_LOAD);
  writeLED(PIN_LOAD_B_LED(), loadB_on, LED_ACTIVE_HIGH_LOAD);

  appliedPlayA = playA_on;
  appliedPlayB = playB_on;
  appliedLoadA = loadA_on;
  appliedLoadB = loadB_on;
}

//////////////////////
// MIDI IN (feedback) -> 4 LEDs (Note 60-63)
//////////////////////
void handleMidiIn() {
  midiEventPacket_t rx;
  do {
    rx = MidiUSB.read();
    if (rx.header == 0) break;

    uint8_t status = rx.byte1 & 0xF0;
    uint8_t ch     = rx.byte1 & 0x0F;
    uint8_t note   = rx.byte2;
    uint8_t vel    = rx.byte3;

    if (ch != MIDI_CH) continue;

#if DEBUG_SERIAL
    Serial.print(F("[RX] status=0x")); Serial.print(status, HEX);
    Serial.print(F(" note=")); Serial.print(note);
    Serial.print(F(" vel="));  Serial.println(vel);
#endif

    // NoteOn vel=0 = NoteOff
    if (status == 0x90) {
      bool isOn = (vel > 0);
      if (note == NOTE_PLAY_A) dawPlayA = isOn;
      if (note == NOTE_PLAY_B) dawPlayB = isOn;
      if (note == NOTE_LOAD_A) dawLoadA = isOn;
      if (note == NOTE_LOAD_B) dawLoadB = isOn;
      applyLEDs();
    } else if (status == 0x80) {
      if (note == NOTE_PLAY_A) dawPlayA = false;
      if (note == NOTE_PLAY_B) dawPlayB = false;
      if (note == NOTE_LOAD_A) dawLoadA = false;
      if (note == NOTE_LOAD_B) dawLoadB = false;
      applyLEDs();
    }
  } while (rx.header != 0);
}

//////////////////////
// BOOT LED TEST (PHYSICAL PINS)
//////////////////////
static inline void bootLedTest() {
#if DEBUG_SERIAL
  Serial.println(F("Boot LED Test: blinking PHYSICAL LED pins D9, D10, D16, D14"));
#endif
  int pins[] = { PHY_playA_led, PHY_playB_led, PHY_loadA_led, PHY_loadB_led };
  for (int i = 0; i < 4; i++) {
    writeLED(pins[i], true, true);  delay(200);
    writeLED(pins[i], false, true); delay(120);
  }
#if DEBUG_SERIAL
  Serial.println(F("Boot LED Test done."));
#endif
}

static inline void printPinMap() {
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

void setup() {
#if DEBUG_SERIAL
  Serial.begin(115200);
  delay(700);
  Serial.println(F("FadeTube Controller boot..."));
  Serial.print(F("SWAP switches=")); Serial.println(SWAP_PLAY_LOAD_SWITCHES ? F("true") : F("false"));
  Serial.print(F("SWAP leds="));     Serial.println(SWAP_PLAY_LOAD_LEDS ? F("true") : F("false"));
  Serial.print(F("LEDS_FROM_DAW=")); Serial.println(LEDS_FROM_DAW ? F("true") : F("false"));
  Serial.print(F("LOCAL_FALLBACK=")); Serial.println(LEDS_LOCAL_FALLBACK ? F("true") : F("false"));
  Serial.print(F("LOAD_ALWAYS_ON=")); Serial.println(LOAD_LEDS_ALWAYS_ON ? F("true") : F("false"));
  Serial.print(F("LOAD_OFF_WHILE_PRESSED=")); Serial.println(LOAD_LEDS_OFF_WHILE_PRESSED ? F("true") : F("false"));
#endif

  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);
  pinMode(encSW, INPUT_PULLUP);
  lastA = digitalRead(encA);

  pinMode(PHY_playA_sw, INPUT_PULLUP);
  pinMode(PHY_playB_sw, INPUT_PULLUP);
  pinMode(PHY_loadA_sw, INPUT_PULLUP);
  pinMode(PHY_loadB_sw, INPUT_PULLUP);

  pinMode(PHY_playA_led, OUTPUT);
  pinMode(PHY_playB_led, OUTPUT);
  pinMode(PHY_loadA_led, OUTPUT);
  pinMode(PHY_loadB_led, OUTPUT);

  bootLedTest();
  printPinMap();

  // estados iniciales
  dawPlayA = dawPlayB = dawLoadA = dawLoadB = false;
  localPlayA = localPlayB = localLoadA = localLoadB = false;

  applyLEDs();
}

void loop() {
  unsigned long now = millis();

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

  // 4) Buttons -> Notes + local fallback (si está habilitado)
  // Play A
  bool pA = digitalRead(PIN_PLAY_A_SW());
  if (lastPlayA == HIGH && pA == LOW) {
    if (now - lastPlayAms > DEBOUNCE_BTN_MS) {
      lastPlayAms = now;
      localPlayA = true;
      if (LEDS_LOCAL_FALLBACK) applyLEDs();
      sendNoteOn(NOTE_PLAY_A, 127);
      maybeFlush();
    }
  } else if (lastPlayA == LOW && pA == HIGH) {
    localPlayA = false;
    if (LEDS_LOCAL_FALLBACK) applyLEDs();
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
      if (LEDS_LOCAL_FALLBACK) applyLEDs();
      sendNoteOn(NOTE_PLAY_B, 127);
      maybeFlush();
    }
  } else if (lastPlayB == LOW && pB == HIGH) {
    localPlayB = false;
    if (LEDS_LOCAL_FALLBACK) applyLEDs();
    sendNoteOff(NOTE_PLAY_B);
    maybeFlush();
  }
  lastPlayB = pB;

  // Load A
  bool lA = digitalRead(PIN_LOAD_A_SW());
  if (lastLoadA == HIGH && lA == LOW) {
    if (now - lastLoadAms > DEBOUNCE_BTN_MS) {
      lastLoadAms = now;
      localLoadA = true;                 // PRESIONADO
      if (LEDS_LOCAL_FALLBACK || (LOAD_LEDS_ALWAYS_ON && LOAD_LEDS_OFF_WHILE_PRESSED)) applyLEDs();
      sendNoteOn(NOTE_LOAD_A, 127);
      maybeFlush();
    }
  } else if (lastLoadA == LOW && lA == HIGH) {
    localLoadA = false;                // SUELTO
    if (LEDS_LOCAL_FALLBACK || (LOAD_LEDS_ALWAYS_ON && LOAD_LEDS_OFF_WHILE_PRESSED)) applyLEDs();
    sendNoteOff(NOTE_LOAD_A);
    maybeFlush();
  }
  lastLoadA = lA;

  // Load B
  bool lB = digitalRead(PIN_LOAD_B_SW());
  if (lastLoadB == HIGH && lB == LOW) {
    if (now - lastLoadBms > DEBOUNCE_BTN_MS) {
      lastLoadBms = now;
      localLoadB = true;                 // PRESIONADO
      if (LEDS_LOCAL_FALLBACK || (LOAD_LEDS_ALWAYS_ON && LOAD_LEDS_OFF_WHILE_PRESSED)) applyLEDs();
      sendNoteOn(NOTE_LOAD_B, 127);
      maybeFlush();
    }
  } else if (lastLoadB == LOW && lB == HIGH) {
    localLoadB = false;                // SUELTO
    if (LEDS_LOCAL_FALLBACK || (LOAD_LEDS_ALWAYS_ON && LOAD_LEDS_OFF_WHILE_PRESSED)) applyLEDs();
    sendNoteOff(NOTE_LOAD_B);
    maybeFlush();
  }
  lastLoadB = lB;

  // 5) Crossfader -> CC14
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
