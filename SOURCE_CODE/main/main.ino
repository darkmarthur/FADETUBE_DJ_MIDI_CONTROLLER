#include <MIDIUSB.h>

/*
  FadeTube – USB-MIDI Controller v1 (ATmega32U4)

  + IDLE MODE:
    - Si no hay actividad por 3 minutos => LEDs "breathing" simultáneo (fade in/out).
    - Cualquier actividad (botón/encoder/fader o MIDI IN) sale del idle.

  + BOOT ANIMATION:
    - Al conectar: 5 “respiraciones” rápidas (todos los LEDs al mismo tiempo).

  + NUEVA LÓGICA DJ:
    - Crossfader normal (sin LOAD): CC_CROSSFADER (14)
    - Mantén LOAD A + mueve crossfader: CC_TEMPO_A_ABS (30) ABS 0..127
    - Mantén LOAD B + mueve crossfader: CC_TEMPO_B_ABS (31) ABS 0..127
    - Mantén LOAD A + gira encoder: CC_JOG_A_REL (22) relativo (+1 / -1)
    - Mantén LOAD B + gira encoder: CC_JOG_B_REL (23) relativo (+1 / -1)
      (derecha=1, izquierda=127)

  + SOFT TAKEOVER (CROSSFADER CC14):
    - El CC14 NO se manda al volver del modo LOAD->TEMPO hasta que el fader físico
      alcance (cruce) el último valor “virtual” de CC14 (lastCrossMidi).

  + MEJORA SOLICITADA (LOAD "ON RELEASE" + CANCEL SI HAY SHORTCUT):
    - El LOAD (NOTE_LOAD_A / NOTE_LOAD_B) ya NO se envía en press-down.
    - Se envía SOLO al soltar (release-up).
    - Si mientras está presionado LOAD se detecta un shortcut (encoder jog o crossfader tempo),
      entonces al soltar LOAD NO se envía el NOTE de LOAD (se cancela).
    - Los shortcuts siguen funcionando igual.

  + FIX BUG + MEJORA (TEMPO SOFT TAKEOVER + SOLO CAMBIA SI MUEVES FADER):
    - Al entrar a modo TEMPO (LOAD + fader) NO se manda CC30/31 automáticamente.
    - Solo se manda cuando el usuario MUEVE el fader (gate por movimiento real).
    - Además, soft takeover para tempo: aun moviendo, no se manda hasta “capturar”
      el último tempo virtual del deck (tempoA_val / tempoB_val).
*/

//////////////////////
// CONFIG PRINCIPAL
//////////////////////
const bool SWAP_PLAY_LOAD_SWITCHES = false;
const bool SWAP_PLAY_LOAD_LEDS     = false;

// MODO LED
const bool LEDS_FROM_DAW           = true;
const bool LEDS_LOCAL_FALLBACK     = true;

// LOAD LEDs behavior
const bool LOAD_LEDS_ALWAYS_ON          = true;
const bool LOAD_LEDS_OFF_WHILE_PRESSED  = true;

// Polaridad
const bool LED_ACTIVE_HIGH_PLAY = true;
const bool LED_ACTIVE_HIGH_LOAD = true;

// DEBUG
#define DEBUG_SERIAL 1
const unsigned long DEBUG_THROTTLE_MS = 40;

//////////////////////
// IDLE MODE
//////////////////////
const unsigned long IDLE_TIMEOUT_MS = 180000UL; // 3 minutos
const unsigned long IDLE_BREATH_PERIOD_MS = 4800; // ms por ciclo

unsigned long lastActivityMs = 0;
bool idleActive = false;

//////////////////////
// PINES
//////////////////////
const int encA  = 2;
const int encB  = 3;
const int encSW = 4;

const int faderPin = A0;

const int PHY_playA_sw = 5;
const int PHY_playB_sw = 6;
const int PHY_loadA_sw = 7;
const int PHY_loadB_sw = 8;

const int PHY_playA_led = 9;
const int PHY_playB_led = 10;
const int PHY_loadA_led = 16;
const int PHY_loadB_led = 14;

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

// Crossfader normal
const uint8_t CC_CROSSFADER   = 14;

// Encoder normal (scroll)
const uint8_t CC_ENCODER_REL  = 20;
const uint8_t CC_ENCODER_PUSH = 21;

// Jogwheel (relativo +/-1) - separado por deck (FIX)
const uint8_t CC_JOG_A_REL    = 22;
const uint8_t CC_JOG_B_REL    = 23;

// Tempo (ABS 0..127)
const uint8_t CC_TEMPO_A_ABS  = 30;
const uint8_t CC_TEMPO_B_ABS  = 31;

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
// TEMPO VIRTUAL
//////////////////////
uint8_t tempoA_val = 64;
uint8_t tempoB_val = 64;

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

unsigned long lastMidiFlushMs = 0;

//////////////////////
// LOAD "ON RELEASE" + CANCEL SI HUBO SHORTCUT
//////////////////////
bool loadA_pending = false;
bool loadB_pending = false;
bool loadA_usedShortcut = false;
bool loadB_usedShortcut = false;

static inline void setLoadPendingA(bool pending) {
  loadA_pending = pending;
  if (pending) loadA_usedShortcut = false;
}
static inline void setLoadPendingB(bool pending) {
  loadB_pending = pending;
  if (pending) loadB_usedShortcut = false;
}

//////////////////////
// CROSS / TEMPO POR FADER + SOFT TAKEOVER
//////////////////////
int lastCrossMidi        = -999; // último CC14 “virtual” enviado
int lastTempoAFromFader  = -999; // último CC30 enviado por fader
int lastTempoBFromFader  = -999; // último CC31 enviado por fader

// Soft takeover state (solo para CC14)
bool crossTakeoverActive = false;
int  crossTakeoverTarget = 0;
int  crossPrevPhys       = -999;
bool prevAnyTempoMode    = false;

//////////////////////
// TEMPO TAKEOVER (CC30/CC31) + MOVEMENT GATE
//////////////////////
bool tempoTakeoverActive = false;
int  tempoTakeoverTarget = 0;
int  tempoPrevPhys       = -999;
int  tempoEntryPhys      = -999;
bool tempoMoved          = false;
uint8_t tempoTakeoverDeck = 0; // 1=A, 2=B, 0=none

//////////////////////
// ESTADO LEDs
//////////////////////
bool dawPlayA = false;
bool dawPlayB = false;
bool dawLoadA = false;
bool dawLoadB = false;

bool localPlayA = false;
bool localPlayB = false;
bool localLoadA = false; // pressed flag
bool localLoadB = false;

bool appliedPlayA = false;
bool appliedPlayB = false;
bool appliedLoadA = false;
bool appliedLoadB = false;

unsigned long lastDebugMs = 0;

//////////////////////
// HELPERS
//////////////////////
static inline void markActivity() {
  lastActivityMs = millis();
  if (idleActive) idleActive = false;
}

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

// PWM por software (0..255) usando micros() (sirve en cualquier pin)
static inline void writeLEDPwmSoft(int pin, uint8_t level, bool activeHigh) {
  uint8_t phase = (uint8_t)(micros() & 0xFF);
  bool on = (phase < level);
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
    }
  }
#endif
}

static inline void applyLEDs() {
  if (idleActive) return;

  bool playA_on = (LEDS_FROM_DAW ? dawPlayA : false) || (LEDS_LOCAL_FALLBACK ? localPlayA : false);
  bool playB_on = (LEDS_FROM_DAW ? dawPlayB : false) || (LEDS_LOCAL_FALLBACK ? localPlayB : false);

  bool loadA_on;
  bool loadB_on;

  if (LOAD_LEDS_ALWAYS_ON) {
    if (LOAD_LEDS_OFF_WHILE_PRESSED) {
      loadA_on = !localLoadA;
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
// IDLE ANIMATION (breathing)
//////////////////////
static inline uint8_t breathLevel(unsigned long nowMs, unsigned long periodMs) {
  unsigned long t = nowMs % periodMs;
  unsigned long half = periodMs / 2;
  if (t < half) return (uint8_t)((t * 255UL) / half);
  t -= half;
  return (uint8_t)(255UL - ((t * 255UL) / half));
}

static inline void idleUpdate() {
  unsigned long now = millis();
  if (!idleActive) {
    if (now - lastActivityMs >= IDLE_TIMEOUT_MS) idleActive = true;
    else return;
  }

  uint8_t lvl = breathLevel(now, IDLE_BREATH_PERIOD_MS);

  writeLEDPwmSoft(PIN_PLAY_A_LED(), lvl, LED_ACTIVE_HIGH_PLAY);
  writeLEDPwmSoft(PIN_PLAY_B_LED(), lvl, LED_ACTIVE_HIGH_PLAY);
  writeLEDPwmSoft(PIN_LOAD_A_LED(), lvl, LED_ACTIVE_HIGH_LOAD);
  writeLEDPwmSoft(PIN_LOAD_B_LED(), lvl, LED_ACTIVE_HIGH_LOAD);
}

//////////////////////
// MIDI IN (feedback) -> LEDs
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

    markActivity();

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
// BOOT ANIMATION
//////////////////////
static inline void bootBreath(uint8_t cycles) {
  const unsigned long period = 1200;
  unsigned long start = millis();
  unsigned long total = (unsigned long)cycles * period;

  while (millis() - start < total) {
    uint8_t lvl = breathLevel(millis() - start, period);
    writeLEDPwmSoft(PIN_PLAY_A_LED(), lvl, LED_ACTIVE_HIGH_PLAY);
    writeLEDPwmSoft(PIN_PLAY_B_LED(), lvl, LED_ACTIVE_HIGH_PLAY);
    writeLEDPwmSoft(PIN_LOAD_A_LED(), lvl, LED_ACTIVE_HIGH_LOAD);
    writeLEDPwmSoft(PIN_LOAD_B_LED(), lvl, LED_ACTIVE_HIGH_LOAD);
  }

  writeLED(PIN_PLAY_A_LED(), false, LED_ACTIVE_HIGH_PLAY);
  writeLED(PIN_PLAY_B_LED(), false, LED_ACTIVE_HIGH_PLAY);
  writeLED(PIN_LOAD_A_LED(), false, LED_ACTIVE_HIGH_LOAD);
  writeLED(PIN_LOAD_B_LED(), false, LED_ACTIVE_HIGH_LOAD);
}

//////////////////////
// SOFT TAKEOVER HELPERS (CC14)
//////////////////////
static inline void armCrossTakeover(int currentPhys) {
  if (lastCrossMidi == -999) {
    crossTakeoverActive = false;
    crossPrevPhys = currentPhys;
    return;
  }
  crossTakeoverActive = true;
  crossTakeoverTarget = lastCrossMidi;
  crossPrevPhys = currentPhys;
}

static inline bool takeoverCaptured(int prevPhys, int phys, int target) {
  if (abs(phys - target) <= FADER_DEADBAND) return true;

  if (prevPhys == -999) return false;
  if (prevPhys < target && phys >= target) return true;
  if (prevPhys > target && phys <= target) return true;

  return false;
}

//////////////////////
// TEMPO TAKEOVER HELPERS (CC30/CC31)
//////////////////////
static inline void armTempoTakeover(uint8_t deck, int entryPhys) {
  tempoTakeoverDeck   = deck;
  tempoEntryPhys      = entryPhys;
  tempoPrevPhys       = entryPhys;
  tempoMoved          = false;
  tempoTakeoverTarget = (deck == 1) ? (int)tempoA_val : (int)tempoB_val;
  tempoTakeoverActive = true;
}

static inline void disarmTempoTakeover() {
  tempoTakeoverActive = false;
  tempoTakeoverDeck = 0;
  tempoMoved = false;
  tempoEntryPhys = -999;
  tempoPrevPhys = -999;
  tempoTakeoverTarget = 0;
}

//////////////////////
// SETUP
//////////////////////
void setup() {
#if DEBUG_SERIAL
  Serial.begin(115200);
  delay(700);
  Serial.println(F("FadeTube Controller boot..."));
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

  dawPlayA = dawPlayB = dawLoadA = dawLoadB = false;
  localPlayA = localPlayB = localLoadA = localLoadB = false;

  // pending/cancel init
  loadA_pending = loadB_pending = false;
  loadA_usedShortcut = loadB_usedShortcut = false;

  // tempo takeover init
  disarmTempoTakeover();

  bootBreath(5);

  // Seed inicial de tempo (opcional)
  sendCC(CC_TEMPO_A_ABS, tempoA_val);
  sendCC(CC_TEMPO_B_ABS, tempoB_val);
  maybeFlush();

  lastActivityMs = millis();
  idleActive = false;

  // baseline fader
  int raw = analogRead(faderPin);
  raw = (raw + analogRead(faderPin) + analogRead(faderPin) + analogRead(faderPin)) / 4;
  int phys = map(raw, 0, 1023, 0, 127);
  crossPrevPhys = phys;

  applyLEDs();
}

//////////////////////
// LOOP
//////////////////////
void loop() {
  unsigned long now = millis();

  // 1) MIDI feedback
  handleMidiIn();

  // 2) IDLE update
  idleUpdate();

  // --- Snapshot de estado LOAD (para modos)
  bool loadA_pressed = (digitalRead(PIN_LOAD_A_SW()) == LOW);
  bool loadB_pressed = (digitalRead(PIN_LOAD_B_SW()) == LOW);

  bool tempoModeA    = (loadA_pressed && !loadB_pressed);
  bool tempoModeB    = (loadB_pressed && !loadA_pressed);
  bool anyTempoMode  = (tempoModeA || tempoModeB);

  // 2.1) Transiciones para takeovers (CC14 y TEMPO)
  // A) TEMPO->NORMAL: armar soft takeover de CC14
  if (prevAnyTempoMode && !anyTempoMode) {
    int raw0 = analogRead(faderPin);
    raw0 = (raw0 + analogRead(faderPin) + analogRead(faderPin) + analogRead(faderPin)) / 4;
    int phys0 = map(raw0, 0, 1023, 0, 127);
    armCrossTakeover(phys0);

    // al salir de TEMPO limpiamos estado de tempo takeover
    disarmTempoTakeover();
  }

  // B) NORMAL->TEMPO: armar tempo takeover (gateado por movimiento real)
  if (!prevAnyTempoMode && anyTempoMode) {
    int raw1 = analogRead(faderPin);
    raw1 = (raw1 + analogRead(faderPin) + analogRead(faderPin) + analogRead(faderPin)) / 4;
    int phys1 = map(raw1, 0, 1023, 0, 127);

    if (tempoModeA) armTempoTakeover(1, phys1);
    else if (tempoModeB) armTempoTakeover(2, phys1);
  }

  prevAnyTempoMode = anyTempoMode;

  // 3) Encoder rotate
  int a = digitalRead(encA);
  if (lastA == HIGH && a == LOW) {
    if (now - lastEncStepMs > DEBOUNCE_ENC_MS) {
      lastEncStepMs = now;

      int b = digitalRead(encB);
      bool dirRight = (b == HIGH);

      markActivity();

      if (anyTempoMode) {
        // JOG RELATIVO +/-1 (mientras LOAD) - separado por deck (FIX)
        uint8_t rel = dirRight ? 1 : 127;

        if (tempoModeA) {
          sendCC(CC_JOG_A_REL, rel);
          if (loadA_pending) loadA_usedShortcut = true;
        } else if (tempoModeB) {
          sendCC(CC_JOG_B_REL, rel);
          if (loadB_pending) loadB_usedShortcut = true;
        }
      } else {
        // Scroll/Browse normal
        uint8_t rel = dirRight ? 1 : 127;
        sendCC(CC_ENCODER_REL, rel);
      }

      maybeFlush();
      applyLEDs();
    }
  }
  lastA = a;

  // 4) Encoder push (sin cambios funcionales)
  bool sw = digitalRead(encSW);
  if (lastEncSW == HIGH && sw == LOW) {
    if (now - lastEncSWMs > 150) {
      lastEncSWMs = now;

      markActivity();

      scrollToggleState = !scrollToggleState;
      sendCC(CC_ENCODER_PUSH, scrollToggleState ? 127 : 0);

      maybeFlush();
      applyLEDs();
    }
  }
  lastEncSW = sw;

  // 5) Buttons -> Notes + local fallback
  // Play A
  bool pA = digitalRead(PIN_PLAY_A_SW());
  if (lastPlayA == HIGH && pA == LOW) {
    if (now - lastPlayAms > DEBOUNCE_BTN_MS) {
      lastPlayAms = now;
      markActivity();
      localPlayA = true;
      if (LEDS_LOCAL_FALLBACK) applyLEDs();
      sendNoteOn(NOTE_PLAY_A, 127);
      maybeFlush();
    }
  } else if (lastPlayA == LOW && pA == HIGH) {
    markActivity();
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
      markActivity();
      localPlayB = true;
      if (LEDS_LOCAL_FALLBACK) applyLEDs();
      sendNoteOn(NOTE_PLAY_B, 127);
      maybeFlush();
    }
  } else if (lastPlayB == LOW && pB == HIGH) {
    markActivity();
    localPlayB = false;
    if (LEDS_LOCAL_FALLBACK) applyLEDs();
    sendNoteOff(NOTE_PLAY_B);
    maybeFlush();
  }
  lastPlayB = pB;

  // Load A (ON RELEASE + CANCEL)
  bool lA = digitalRead(PIN_LOAD_A_SW());
  if (lastLoadA == HIGH && lA == LOW) {
    if (now - lastLoadAms > DEBOUNCE_BTN_MS) {
      lastLoadAms = now;
      markActivity();

      localLoadA = true;
      setLoadPendingA(true);

      if (LEDS_LOCAL_FALLBACK || (LOAD_LEDS_ALWAYS_ON && LOAD_LEDS_OFF_WHILE_PRESSED)) applyLEDs();
    }
  } else if (lastLoadA == LOW && lA == HIGH) {
    markActivity();

    localLoadA = false;

    if (loadA_pending && !loadA_usedShortcut) {
      sendNoteOn(NOTE_LOAD_A, 127);
      sendNoteOff(NOTE_LOAD_A);
      maybeFlush();
    }

    setLoadPendingA(false);

    if (LEDS_LOCAL_FALLBACK || (LOAD_LEDS_ALWAYS_ON && LOAD_LEDS_OFF_WHILE_PRESSED)) applyLEDs();
  }
  lastLoadA = lA;

  // Load B (ON RELEASE + CANCEL)
  bool lB = digitalRead(PIN_LOAD_B_SW());
  if (lastLoadB == HIGH && lB == LOW) {
    if (now - lastLoadBms > DEBOUNCE_BTN_MS) {
      lastLoadBms = now;
      markActivity();

      localLoadB = true;
      setLoadPendingB(true);

      if (LEDS_LOCAL_FALLBACK || (LOAD_LEDS_ALWAYS_ON && LOAD_LEDS_OFF_WHILE_PRESSED)) applyLEDs();
    }
  } else if (lastLoadB == LOW && lB == HIGH) {
    markActivity();

    localLoadB = false;

    if (loadB_pending && !loadB_usedShortcut) {
      sendNoteOn(NOTE_LOAD_B, 127);
      sendNoteOff(NOTE_LOAD_B);
      maybeFlush();
    }

    setLoadPendingB(false);

    if (LEDS_LOCAL_FALLBACK || (LOAD_LEDS_ALWAYS_ON && LOAD_LEDS_OFF_WHILE_PRESSED)) applyLEDs();
  }
  lastLoadB = lB;

  // 6) Crossfader (dual-mode: CC14 o TEMPO A/B ABS + soft takeover CC14 + tempo takeover)
  int raw = analogRead(faderPin);
  raw = (raw + analogRead(faderPin) + analogRead(faderPin) + analogRead(faderPin)) / 4;
  int phys = map(raw, 0, 1023, 0, 127);

  if (anyTempoMode) {
    // EN MODO LOAD: el crossfader controla TEMPO del deck seleccionado
    // FIX: no mandar nada hasta que el usuario MUEVA el fader
    // y además: soft takeover contra el último tempo virtual (tempoA_val/tempoB_val)

    // Gate por movimiento real (evita salto al solo presionar LOAD)
    if (!tempoMoved) {
      if (abs(phys - tempoEntryPhys) <= FADER_DEADBAND) {
        // sigue quieto (o ruido mínimo) => NO enviar
        tempoPrevPhys = phys;
        crossPrevPhys = phys; // seguir track para takeover de CC14 al salir
      } else {
        tempoMoved = true; // ya hubo movimiento real
      }
    }

    // Si ya se movió, aplicamos takeover de tempo
    if (tempoMoved) {
      if (tempoTakeoverActive) {
        if (takeoverCaptured(tempoPrevPhys, phys, tempoTakeoverTarget)) {
          tempoTakeoverActive = false; // capturado
        } else {
          // aún no capturado => no enviar tempo
          tempoPrevPhys = phys;
          crossPrevPhys = phys;
          goto end_fader;
        }
      }

      // Ya capturado => enviar tempo solo si cambia
      if (tempoModeA) {
        if (lastTempoAFromFader == -999 || abs(phys - lastTempoAFromFader) > FADER_DEADBAND) {
          lastTempoAFromFader = phys;
          tempoA_val = (uint8_t)phys;
          markActivity();
          sendCC(CC_TEMPO_A_ABS, tempoA_val);

          // shortcut usado => cancelar LOAD al soltar
          if (loadA_pending) loadA_usedShortcut = true;

          maybeFlush();
          applyLEDs();
        }
      } else if (tempoModeB) {
        if (lastTempoBFromFader == -999 || abs(phys - lastTempoBFromFader) > FADER_DEADBAND) {
          lastTempoBFromFader = phys;
          tempoB_val = (uint8_t)phys;
          markActivity();
          sendCC(CC_TEMPO_B_ABS, tempoB_val);

          if (loadB_pending) loadB_usedShortcut = true;

          maybeFlush();
          applyLEDs();
        }
      }

      tempoPrevPhys = phys;
      crossPrevPhys = phys;
    }

    // NO mandar CC14 mientras LOAD
    crossPrevPhys = phys;
  } else {
    // MODO NORMAL: CC14 con soft takeover
    if (crossTakeoverActive) {
      if (takeoverCaptured(crossPrevPhys, phys, crossTakeoverTarget)) {
        crossTakeoverActive = false;
        if (lastCrossMidi == -999 || abs(phys - lastCrossMidi) > FADER_DEADBAND) {
          lastCrossMidi = phys;
          markActivity();
          sendCC(CC_CROSSFADER, (uint8_t)phys);
          maybeFlush();
          applyLEDs();
        }
      }
      crossPrevPhys = phys;
    } else {
      if (lastCrossMidi == -999 || abs(phys - lastCrossMidi) > FADER_DEADBAND) {
        lastCrossMidi = phys;
        markActivity();
        sendCC(CC_CROSSFADER, (uint8_t)phys);
        maybeFlush();
        applyLEDs();
      }
      crossPrevPhys = phys;
    }
  }

end_fader:
  maybeFlush();
}
