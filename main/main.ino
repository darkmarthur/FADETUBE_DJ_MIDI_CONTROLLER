#include <MIDIUSB.h>

// FadeTube – USB-MIDI Controller v1 (ATmega32U4)
// - Sends MIDI (buttons/encoder/fader)
// - Receives MIDI feedback (from Ableton) to drive LEDs
//
// Encoder: A=D2, B=D3, SW=D4 (COM->GND)
// Fader B10K: 1->GND, 2->A0, 3->VCC(5V)
// Play Buttons (momentary):
//   Switch: one pin -> GND, other pin -> Dx (INPUT_PULLUP)
//   LED: C -> GND, R -> Dx (OUTPUT)  [active HIGH]
// Load Buttons (momentary switch only):
//   Switch: one pin -> GND, other pin -> Dx (INPUT_PULLUP)
//
// MIDI mapping (Channel 1):
// - Crossfader: CC14 absolute 0..127
// - Encoder rotate: CC20 relative (CW=1, CCW=127)
// - Encoder push: CC21 toggle (127/0)  [you can also make it momentary]
// - Buttons:
//   Play A: Note 60 (C4)
//   Load A: Note 61 (C#4)
//   Play B: Note 62 (D4)
//   Load B: Note 63 (Eb4)
//
// LED feedback expectations from DAW:
// - Play A LED follows Note 60 (On => velocity >0, Off => NoteOff/vel=0)
// - Play B LED follows Note 62 (On => velocity >0, Off => NoteOff/vel=0)

//////////////////////
// PINES
//////////////////////
const int encA  = 2;
const int encB  = 3;
const int encSW = 4;

const int faderPin = A0; // WIPER (pin 2 del fader)

// PLAY buttons (switch)
const int playA_sw = 5;
const int playB_sw = 6;

// PLAY LEDs (R pin del botón; C a GND)
const int playA_led = 9;
const int playB_led = 10;

// LOAD buttons (switch)
const int loadA_sw = 7;
const int loadB_sw = 8;

//////////////////////
// MIDI CONFIG
//////////////////////
const uint8_t MIDI_CH = 0; // 0 = Channel 1 en MIDIUSB
const uint8_t CC_CROSSFADER = 14;
const uint8_t CC_ENCODER_REL = 20;
const uint8_t CC_ENCODER_PUSH = 21;

const uint8_t NOTE_PLAY_A = 60;
const uint8_t NOTE_LOAD_A = 61;
const uint8_t NOTE_PLAY_B = 62;
const uint8_t NOTE_LOAD_B = 63;

//////////////////////
// DEBOUNCE / FILTROS
//////////////////////
const unsigned long DEBOUNCE_BTN_MS = 40;     // botones
const unsigned long DEBOUNCE_ENC_MS = 2;      // encoder steps
const int FADER_DEADBAND = 1;                 // 1 = ignora jitter de +-1 en MIDI
const unsigned long MIDI_FLUSH_EVERY_MS = 5;  // flush batching

//////////////////////
// ESTADO INPUTS
//////////////////////
// Encoder
int lastA = HIGH;
unsigned long lastEncStepMs = 0;

// Encoder push
bool lastEncSW = HIGH;
unsigned long lastEncSWMs = 0;
bool scrollToggleState = false; // para CC21 toggle

// Buttons
bool lastPlayA = HIGH;
bool lastPlayB = HIGH;
bool lastLoadA = HIGH;
bool lastLoadB = HIGH;

unsigned long lastPlayAms = 0;
unsigned long lastPlayBms = 0;
unsigned long lastLoadAms = 0;
unsigned long lastLoadBms = 0;

// Fader
int lastFaderMidi = -999; // fuerza primer envío
unsigned long lastMidiFlushMs = 0;

//////////////////////
// ESTADO LEDs (por feedback)
//////////////////////
bool ledPlayA = false;
bool ledPlayB = false;

//////////////////////
// HELPERS MIDIUSB
//////////////////////
static inline void sendCC(uint8_t cc, uint8_t value) {
  midiEventPacket_t evt = {0x0B, (uint8_t)(0xB0 | MIDI_CH), cc, value};
  MidiUSB.sendMIDI(evt);
}

static inline void sendNoteOn(uint8_t note, uint8_t vel) {
  midiEventPacket_t evt = {0x09, (uint8_t)(0x90 | MIDI_CH), note, vel};
  MidiUSB.sendMIDI(evt);
}

static inline void sendNoteOff(uint8_t note) {
  midiEventPacket_t evt = {0x08, (uint8_t)(0x80 | MIDI_CH), note, 0x00};
  MidiUSB.sendMIDI(evt);
}

static inline void maybeFlush() {
  unsigned long now = millis();
  if (now - lastMidiFlushMs >= MIDI_FLUSH_EVERY_MS) {
    lastMidiFlushMs = now;
    MidiUSB.flush();
  }
}

//////////////////////
// LED APPLY
//////////////////////
static inline void applyLEDs() {
  digitalWrite(playA_led, ledPlayA ? HIGH : LOW);
  digitalWrite(playB_led, ledPlayB ? HIGH : LOW);
}

//////////////////////
// MIDI IN (feedback)
// - We interpret NoteOn/NoteOff for notes 60 and 62 to drive LEDs.
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

    // Solo canal 1 (opcional; si quieres escuchar todos, quita este if)
    if (ch != MIDI_CH) continue;

    // NOTE ON (0x90) y NOTE OFF (0x80)
    // Nota: NoteOn con vel=0 es equivalente a NoteOff.
    if (status == 0x90) {
      bool isOn = (data2 > 0);
      if (data1 == NOTE_PLAY_A) { ledPlayA = isOn; applyLEDs(); }
      if (data1 == NOTE_PLAY_B) { ledPlayB = isOn; applyLEDs(); }
    } else if (status == 0x80) {
      if (data1 == NOTE_PLAY_A) { ledPlayA = false; applyLEDs(); }
      if (data1 == NOTE_PLAY_B) { ledPlayB = false; applyLEDs(); }
    }

    // (Opcional) también puedes mapear CC->LED aquí si tu DAW manda CC:
    // if (status == 0xB0) { ... }

  } while (rx.header != 0);
}

void setup() {
  // Pines
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);
  pinMode(encSW, INPUT_PULLUP);
  lastA = digitalRead(encA);

  pinMode(playA_sw, INPUT_PULLUP);
  pinMode(playB_sw, INPUT_PULLUP);
  pinMode(loadA_sw, INPUT_PULLUP);
  pinMode(loadB_sw, INPUT_PULLUP);

  pinMode(playA_led, OUTPUT);
  pinMode(playB_led, OUTPUT);

  // LEDs arrancan apagados; el DAW los encenderá vía feedback
  ledPlayA = false;
  ledPlayB = false;
  applyLEDs();
}

void loop() {
  unsigned long now = millis();

  // 1) Recibir feedback MIDI primero (para LEDs responsivos)
  handleMidiIn();

  // 2) Encoder rotate -> CC20 relative
  int a = digitalRead(encA);
  if (lastA == HIGH && a == LOW) {
    if (now - lastEncStepMs > DEBOUNCE_ENC_MS) {
      lastEncStepMs = now;
      int b = digitalRead(encB);

      // Convención: CW=1, CCW=127
      if (b == HIGH) {
        sendCC(CC_ENCODER_REL, 1);
      } else {
        sendCC(CC_ENCODER_REL, 127);
      }
      maybeFlush();
    }
  }
  lastA = a;

  // 3) Encoder push -> CC21 toggle (127/0)
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

  // 4) Buttons -> Notes
  // Play A
  bool pA = digitalRead(playA_sw);
  if (lastPlayA == HIGH && pA == LOW) {
    if (now - lastPlayAms > DEBOUNCE_BTN_MS) {
      lastPlayAms = now;
      sendNoteOn(NOTE_PLAY_A, 127);
      maybeFlush();
    }
  } else if (lastPlayA == LOW && pA == HIGH) {
    // release -> Note Off
    sendNoteOff(NOTE_PLAY_A);
    maybeFlush();
  }
  lastPlayA = pA;

  // Play B
  bool pB = digitalRead(playB_sw);
  if (lastPlayB == HIGH && pB == LOW) {
    if (now - lastPlayBms > DEBOUNCE_BTN_MS) {
      lastPlayBms = now;
      sendNoteOn(NOTE_PLAY_B, 127);
      maybeFlush();
    }
  } else if (lastPlayB == LOW && pB == HIGH) {
    sendNoteOff(NOTE_PLAY_B);
    maybeFlush();
  }
  lastPlayB = pB;

  // Load A (momentary)
  bool lA = digitalRead(loadA_sw);
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

  // Load B (momentary)
  bool lB = digitalRead(loadB_sw);
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

  // deadband anti-jitter
  if (lastFaderMidi == -999 || abs(midi - lastFaderMidi) > FADER_DEADBAND) {
    lastFaderMidi = midi;
    sendCC(CC_CROSSFADER, (uint8_t)midi);
    maybeFlush();
  }

  // flush final
  maybeFlush();
}
