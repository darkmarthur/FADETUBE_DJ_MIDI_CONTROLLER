// FadeTube – Encoder + Crossfader + 2x Play Buttons w/ LED Test (ATmega32U4)
//
// Encoder: A=D2, B=D3, SW=D4 (COM->GND)
// Fader B10K: 1->GND, 2->A0, 3->VCC(5V)
// Play Buttons (momentary):
//   Switch: one pin -> GND, other pin -> Dx (INPUT_PULLUP)
//   LED (using RED channel):
//     C -> GND, R -> Dx (OUTPUT)  [If LED common is actually +5V, invert logic: see NOTE below]
//
// NOTES:
// - INPUT_PULLUP => released=HIGH, pressed=LOW
// - LED pins here use "active HIGH" (HIGH=on) assuming C -> GND (common cathode)

//////////////////////
// PINES
//////////////////////
const int encA  = 2;
const int encB  = 3;
const int encSW = 4;

const int faderPin = A0; // WIPER (pin 2 del fader)

// Botones PLAY (switch)
const int playA_sw = 5;
const int playB_sw = 6;

// LEDs (canal R del botón; C a GND)
const int playA_led = 9;
const int playB_led = 10;

//////////////////////
// ESTADO
//////////////////////
long encCount = 0;
int lastA = HIGH;
unsigned long lastStepMs = 0;

// Encoder push
bool lastEncSW = HIGH;
unsigned long lastEncSWMs = 0;

// Play buttons
bool lastPlayA = HIGH;
bool lastPlayB = HIGH;
unsigned long lastPlayAMs = 0;
unsigned long lastPlayBMs = 0;
bool playA_state = false; // estado lógico (para prueba)
bool playB_state = false;

// Fader
int lastFaderMidi = -1;
unsigned long lastFaderPrintMs = 0;
int minRaw = 1023;
int maxRaw = 0;

// Heartbeat
unsigned long lastAliveMs = 0;

//////////////////////
// HELPERS
//////////////////////
void setPlayLed(int pin, bool on) {
  // Si C->GND (common cathode): HIGH = ON
  digitalWrite(pin, on ? HIGH : LOW);

  // NOTE: Si tu LED es C->+5V (common anode), cambia a:
  // digitalWrite(pin, on ? LOW : HIGH);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // Encoder
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);
  pinMode(encSW, INPUT_PULLUP);
  lastA = digitalRead(encA);

  // Botones PLAY (switch)
  pinMode(playA_sw, INPUT_PULLUP);
  pinMode(playB_sw, INPUT_PULLUP);

  // LEDs
  pinMode(playA_led, OUTPUT);
  pinMode(playB_led, OUTPUT);
  setPlayLed(playA_led, false);
  setPlayLed(playB_led, false);

  Serial.println("=== FadeTube Controller Test ===");
  Serial.println("Encoder: A=D2 B=D3 SW=D4");
  Serial.println("Fader: 1->GND 2->A0 3->5V");
  Serial.println("Play A: SW=D5 LED=D9 (LED: C->GND, R->D9)");
  Serial.println("Play B: SW=D6 LED=D10 (LED: C->GND, R->D10)");
  Serial.println("--------------------------------");
}

void loop() {
  unsigned long now = millis();

  // -------- HEARTBEAT --------
  if (now - lastAliveMs > 1000) {
    lastAliveMs = now;
    Serial.println("Loop vivo...");
  }

  // -------- ENCODER (flanco de bajada en A) --------
  int a = digitalRead(encA);
  if (lastA == HIGH && a == LOW) {
    if (now - lastStepMs > 2) { // debounce simple
      lastStepMs = now;

      int b = digitalRead(encB);
      if (b == HIGH) {
        encCount++;
        Serial.print("Encoder Derecha -> ");
      } else {
        encCount--;
        Serial.print("Encoder Izquierda <- ");
      }
      Serial.println(encCount);
    }
  }
  lastA = a;

  // -------- ENCODER PUSH --------
  bool sw = digitalRead(encSW);
  if (lastEncSW == HIGH && sw == LOW) {
    if (now - lastEncSWMs > 200) {
      lastEncSWMs = now;
      Serial.println("Encoder SW: BOTON PRESIONADO");
    }
  }
  lastEncSW = sw;

  // -------- PLAY BUTTON A (switch + LED) --------
  bool pA = digitalRead(playA_sw);
  if (lastPlayA == HIGH && pA == LOW) {
    if (now - lastPlayAMs > 120) {
      lastPlayAMs = now;

      playA_state = !playA_state;
      setPlayLed(playA_led, playA_state);

      Serial.print("PLAY A: ");
      Serial.println(playA_state ? "ON (LED ON)" : "OFF (LED OFF)");
    }
  }
  lastPlayA = pA;

  // -------- PLAY BUTTON B (switch + LED) --------
  bool pB = digitalRead(playB_sw);
  if (lastPlayB == HIGH && pB == LOW) {
    if (now - lastPlayBMs > 120) {
      lastPlayBMs = now;

      playB_state = !playB_state;
      setPlayLed(playB_led, playB_state);

      Serial.print("PLAY B: ");
      Serial.println(playB_state ? "ON (LED ON)" : "OFF (LED OFF)");
    }
  }
  lastPlayB = pB;

  // -------- FADER (A0) --------
  int raw = analogRead(faderPin);
  raw = (raw + analogRead(faderPin) + analogRead(faderPin) + analogRead(faderPin)) / 4; // reduce jitter

  if (raw < minRaw) minRaw = raw;
  if (raw > maxRaw) maxRaw = raw;

  int midi = map(raw, 0, 1023, 0, 127);

  if (midi != lastFaderMidi || (now - lastFaderPrintMs > 300)) {
    lastFaderPrintMs = now;
    lastFaderMidi = midi;

    Serial.print("Fader RAW=");
    Serial.print(raw);
    Serial.print("  MIDI=");
    Serial.print(midi);
    Serial.print("  (min=");
    Serial.print(minRaw);
    Serial.print(" max=");
    Serial.print(maxRaw);
    Serial.println(")");
  }
}
