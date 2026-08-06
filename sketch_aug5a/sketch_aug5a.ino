#include <Arduino.h>

// ==========================================
// PIN DEFINITIONS (ESP32)
// ==========================================
const int dialPulsePin = 18;  // Rotary pulse switch (NC, to GND)
const int hookPin      = 19;  // Receiver hook switch (NO, to GND)
const int dialStartPin = 21;  // Rotary wheel off-rest switch (to GND)
const int audioPin     = 25;  // Single-pin summed audio out
const int statusLed    = 2;   // Onboard LED (GPIO 2)

// ==========================================
// ROTARY PULSE TRACKING (BUGFIXED TIMINGS)
// ==========================================
volatile int pulseCount = 0;
volatile unsigned long lastPulseTime = 0;
volatile unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 15; // 15ms is optimal for 10Hz rotary leaf switches
const unsigned long dialTimeout   = 350; // 350ms window after last pulse

// ==========================================
// HARDWARE STATE VARIABLES
// ==========================================
bool phoneOffHook    = false;
bool dialWheelActive = false;
bool toneIsPlaying   = false;

// ==========================================
// SOFTWARE DDS AUDIO SYNTHESIS
// ==========================================
#define PWM_FREQ       62500  
#define PWM_RESOLUTION 8      
#define SAMPLE_RATE    8000   

hw_timer_t *audioTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

const uint8_t PROGMEM sineTable[256] = {
  128, 131, 134, 137, 140, 143, 146, 149, 152, 155, 158, 162, 165, 167, 170, 173,
  176, 179, 182, 185, 188, 190, 193, 196, 198, 201, 203, 206, 208, 211, 213, 215,
  218, 220, 222, 224, 226, 228, 230, 232, 234, 235, 237, 238, 240, 241, 243, 244,
  245, 246, 247, 248, 249, 250, 250, 251, 252, 252, 252, 253, 253, 253, 253, 253,
  252, 252, 252, 251, 250, 250, 249, 248, 247, 246, 245, 244, 243, 241, 240, 238,
  237, 235, 234, 232, 230, 228, 226, 224, 222, 220, 218, 215, 213, 211, 208, 206,
  203, 201, 198, 196, 193, 190, 188, 185, 182, 179, 176, 173, 170, 167, 165, 162,
  158, 155, 152, 149, 146, 143, 140, 137, 134, 131, 128, 124, 121, 118, 115, 112,
  109, 106, 103, 100, 97,  93,  90,  88,  85,  82,  79,  76,  73,  70,  67,  65,
  62,  59,  57,  54,  52,  49,  47,  44,  42,  40,  37,  35,  33,  31,  29,  27,
  25,  23,  21,  20,  18,  17,  15,  14,  12,  11,  10,  9,   8,   7,   6,   5,
  5,   4,   3,   3,   3,   2,   2,   2,   2,   2,   3,   3,   3,   4,   5,   5,
  6,   7,   8,   9,   10,  11,  12,  14,  15,  17,  18,  20,  21,  23,  25,  27,
  29,  31,  33,  35,  37,  40,  42,  44,  47,  49,  52,  54,  57,  59,  62,  65,
  67,  70,  73,  76,  79,  82,  85,  88,  90,  93,  97,  100, 103, 106, 109, 112,
  115, 118, 121, 124
};

volatile uint32_t phaseAcc350 = 0;
volatile uint32_t phaseAcc440 = 0;
const uint32_t phaseInc350 = (350ULL * 4294967296ULL) / SAMPLE_RATE;
const uint32_t phaseInc440 = (440ULL * 4294967296ULL) / SAMPLE_RATE;

void ARDUINO_ISR_ATTR onAudioTimer() {
  if (toneIsPlaying) {
    phaseAcc350 += phaseInc350;
    phaseAcc440 += phaseInc440;

    uint8_t sample1 = sineTable[phaseAcc350 >> 24];
    uint8_t sample2 = sineTable[phaseAcc440 >> 24];
    uint8_t mixedSample = (sample1 >> 1) + (sample2 >> 1);

    ledcWrite(audioPin, mixedSample);
  } else {
    ledcWrite(audioPin, 128);
  }
}

// BUGFIXED ISR: Verifies pin state and uses FALLING break edge with 15ms window
void ARDUINO_ISR_ATTR countPulse() {
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    // Confirm the pulse break is real (NC switch pulled HIGH on break)
    if (digitalRead(dialPulsePin) == HIGH) {
      pulseCount++;
      lastPulseTime = currentTime;
      lastDebounceTime = currentTime;
    }
  }
}

void blinkDigit(int count);

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n--- ESP32 Candlestick Phone Controller (Pulse Engine v2) ---");

  pinMode(dialPulsePin, INPUT_PULLUP);
  pinMode(hookPin, INPUT_PULLUP);
  pinMode(dialStartPin, INPUT_PULLUP);
  
  pinMode(statusLed, OUTPUT);
  digitalWrite(statusLed, LOW);

  ledcAttach(audioPin, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(audioPin, 128);

  audioTimer = timerBegin(SAMPLE_RATE);
  timerAttachInterrupt(audioTimer, &onAudioTimer);

  // Trigger on RISING edge (when NC switch opens and pin goes HIGH)
  attachInterrupt(digitalPinToInterrupt(dialPulsePin), countPulse, RISING);

  Serial.println("[SYSTEM] Hardware and DDS Audio Engine Ready.");
}

void loop() {
  bool currentHookState  = (digitalRead(hookPin) == LOW);      // true = Off-Hook
  bool currentWheelState = (digitalRead(dialStartPin) == LOW); // true = Wheel Off Rest

  // -----------------------------------------------------------------
  // 1. HANDSET HOOK STATE
  // -----------------------------------------------------------------
  if (currentHookState) {
    
    if (!phoneOffHook) {
      phoneOffHook = true;
      digitalWrite(statusLed, HIGH);
      
      Serial.println("\n[HOOK] Handset LIFTED (Off-Hook). Starting dial tone...");
      
      portENTER_CRITICAL(&timerMux);
      phaseAcc350 = 0;
      phaseAcc440 = 0;
      toneIsPlaying = true;
      portEXIT_CRITICAL(&timerMux);
    }

    // ---------------------------------------------------------------
    // 2. DIAL WHEEL MOVEMENT STATE
    // ---------------------------------------------------------------
    if (currentWheelState && !dialWheelActive) {
      dialWheelActive = true;
      
      // Clear wind-up noise
      portENTER_CRITICAL(&timerMux);
      pulseCount = 0;
      portEXIT_CRITICAL(&timerMux);
      
      Serial.println("[DIAL] Wheel pulled off rest position.");

      if (toneIsPlaying) {
        portENTER_CRITICAL(&timerMux);
        toneIsPlaying = false;
        portEXIT_CRITICAL(&timerMux);
        
        digitalWrite(statusLed, LOW);
        Serial.println("[AUDIO] Dial tone CUT.");
      }
    } 
    else if (!currentWheelState && dialWheelActive) {
      dialWheelActive = false;
      Serial.println("[DIAL] Wheel returned flush to rest position.");
    }

    // ---------------------------------------------------------------
    // 3. DIGIT COMPLETE CHECK (DUAL-CONDITION TERMINATION)
    // ---------------------------------------------------------------
    int currentCount = 0;
    unsigned long timeSinceLastPulse = 0;
    unsigned long localLastPulse = 0;

    portENTER_CRITICAL(&timerMux);
    currentCount = pulseCount;
    localLastPulse = lastPulseTime;
    portEXIT_CRITICAL(&timerMux);

    if (currentCount > 0) {
      timeSinceLastPulse = millis() - localLastPulse;
      
      // Commit digit if timeout reached OR wheel has snapped back to rest position
      if (timeSinceLastPulse > dialTimeout || (!currentWheelState && timeSinceLastPulse > 100)) {
        
        portENTER_CRITICAL(&timerMux);
        pulseCount = 0;
        portEXIT_CRITICAL(&timerMux);

        int displayDigit = (currentCount == 10) ? 0 : currentCount;
        Serial.printf("[DIAL] Digit Complete! Raw Pulses: %d -> Dialed Number: %d\n", currentCount, displayDigit);

        blinkDigit(currentCount);
      }
    }

  } else {
    // ---------------------------------------------------------------
    // 4. ON-HOOK / RESET STATE
    // ---------------------------------------------------------------
    if (phoneOffHook) {
      phoneOffHook = false;
      dialWheelActive = false;
      
      portENTER_CRITICAL(&timerMux);
      pulseCount = 0;
      toneIsPlaying = false;
      portEXIT_CRITICAL(&timerMux);
      
      digitalWrite(statusLed, LOW);
      Serial.println("\n[HOOK] Handset REPLACED (On-Hook). System reset.");
    }
  }
}

void blinkDigit(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(statusLed, HIGH);
    delay(70);
    digitalWrite(statusLed, LOW);
    delay(70);
  }
}