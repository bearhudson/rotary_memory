#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include <Arduino.h>

// ==========================================
// PIN DEFINITIONS (ESP32)
// ==========================================
const int dialPulsePin = 18;  // Rotary pulse switch (NC, to GND)
const int hookPin      = 19;  // Receiver hook switch (NO, to GND)
const int dialStartPin = 21;  // Rotary wheel off-rest switch (to GND)
const int audioPin     = 25;  // Single-pin summed audio out (DAC/PWM)
const int statusLed    = 2;   // Onboard LED (GPIO 2 on most ESP32 boards)

// ==========================================
// ROTARY PULSE TRACKING
// ==========================================
volatile int pulseCount = 0;
volatile unsigned long lastPulseTime = 0;
volatile unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 30; // 30ms eliminates contact chatter
const unsigned long dialTimeout   = 400; // 400ms pause indicates end of digit

// ==========================================
// HARDWARE STATE VARIABLES
// ==========================================
bool phoneOffHook    = false;
bool dialWheelActive = false;
bool toneIsPlaying   = false;

// ==========================================
// SOFTWARE DDS AUDIO SYNTHESIS (SINGLE PIN)
// ==========================================
// Timer & PWM config for 62.5 kHz PWM carrier frequency
#define PWM_CHANNEL    0
#define PWM_FREQ       62500  
#define PWM_RESOLUTION 8      // 8-bit resolution (0-255)
#define SAMPLE_RATE    8000   // 8 kHz audio sampling rate

hw_timer_t *audioTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Look Up Table (256-point Sine Wave)
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

// DDS Phase accumulators (32-bit fixed point)
volatile uint32_t phaseAcc350 = 0;
volatile uint32_t phaseAcc440 = 0;
// Phase increments calculated for 8 kHz sample rate
const uint32_t phaseInc350 = (350ULL * 4294967296ULL) / SAMPLE_RATE;
const uint32_t phaseInc440 = (440ULL * 4294967296ULL) / SAMPLE_RATE;

// Timer Interrupt Service Routine - Runs at 8000 Hz
void IRAM_ATTR onAudioTimer() {
  if (toneIsPlaying) {
    // Advance phase accumulators
    phaseAcc350 += phaseInc350;
    phaseAcc440 += phaseInc440;

    // Fetch sample values from sine table using upper 8 bits of accumulator
    uint8_t sample1 = sineTable[phaseAcc350 >> 24];
    uint8_t sample2 = sineTable[phaseAcc440 >> 24];

    // Mathematically sum both sine waves and scale to 8-bit PWM range (0-255)
    uint8_t mixedSample = (sample1 >> 1) + (sample2 >> 1);

    // Update hardware PWM duty cycle on GPIO 25
    ledcWrite(PWM_CHANNEL, mixedSample);
  } else {
    // Mute output (center scale 128 to avoid DC click)
    ledcWrite(PWM_CHANNEL, 128);
  }
}

// Interrupt Service Routine for Rotary Pulses
void IRAM_ATTR countPulse() {
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    pulseCount++;
    lastPulseTime = currentTime;
    lastDebounceTime = currentTime;
  }
}

void blinkDigit(int count);

void setup() {
  // 1. Initialize Serial Debugging
  Serial.begin(115200);
  delay(500);
  Serial.println("\n--- ESP32 Candlestick Phone Controller Initializing ---");

  // 2. Configure Hardware Inputs
  pinMode(dialPulsePin, INPUT_PULLUP);
  pinMode(hookPin, INPUT_PULLUP);
  pinMode(dialStartPin, INPUT_PULLUP);
  
  pinMode(statusLed, OUTPUT);
  digitalWrite(statusLed, LOW);

  // 3. Configure Single-Pin Audio PWM (LEDC)
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(audioPin, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 128); // Initialize at zero-crossing offset

  // 4. Configure Hardware Timer for 8 kHz Audio Interrupt
  // ESP32 APB clock is 80 MHz -> Prescaler 80 gives 1 MHz timer clock (1 µs per tick)
  audioTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(audioTimer, &onAudioTimer, true);
  // Trigger every 125 ticks (1000000 µs / 125 = 8000 Hz)
  timerAlarmWrite(audioTimer, 125, true);
  timerAlarmEnable(audioTimer);

  // 5. Attach Rotary Pulse Interrupt
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
    
    // Handset was just lifted
    if (!phoneOffHook) {
      phoneOffHook = true;
      digitalWrite(statusLed, HIGH);
      
      Serial.println("\n[HOOK] Handset LIFTED (Off-Hook). Starting dual dial tone (350 Hz + 440 Hz)...");
      
      // Reset DDS phases and enable tone generator
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
      
      // Clear wind-up noise pulses
      portENTER_CRITICAL(&timerMux);
      pulseCount = 0;
      portEXIT_CRITICAL(&timerMux);
      
      Serial.println("[DIAL] Wheel pulled off rest position (Pin 21 LOW).");

      // Cut dial tone immediately on single output pin
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
    // 3. DIGIT COMPLETE CHECK
    // ---------------------------------------------------------------
    int currentCount = 0;
    unsigned long timeSinceLastPulse = 0;

    portENTER_CRITICAL(&timerMux);
    currentCount = pulseCount;
    if (currentCount > 0) {
      timeSinceLastPulse = millis() - lastPulseTime;
    }
    portEXIT_CRITICAL(&timerMux);

    if (currentCount > 0 && timeSinceLastPulse > dialTimeout) {
      
      // Atomically reset pulseCount before processing
      portENTER_CRITICAL(&timerMux);
      pulseCount = 0;
      portEXIT_CRITICAL(&timerMux);

      // Translate 10 pulses to '0' for serial debugging
      int displayDigit = (currentCount == 10) ? 0 : currentCount;
      Serial.printf("[DIAL] Digit Complete! Raw Pulses: %d -> Dialed Number: %d\n", currentCount, displayDigit);

      // Signal dialed count back on onboard LED
      blinkDigit(currentCount);
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

// Rapid-fire blink function for visual feedback
void blinkDigit(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(statusLed, HIGH);
    delay(70);
    digitalWrite(statusLed, LOW);
    delay(70);
  }
}