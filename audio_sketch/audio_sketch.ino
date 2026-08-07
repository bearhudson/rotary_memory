/**
 * ============================================================================
 * PROJECT: ESP32 Retro Rotary Telephone Controller & Particle Light Engine
 * TARGET HARDWARE: ESP32-WROOM-32 / ESP32-S Breakout
 * ============================================================================
 * 
 * CORE ARCHITECTURE:
 * ------------------
 * - CORE 0 (Dedicated FreeRTOS Task): Handles multi-mode I2S digital audio synthesis 
 *   (Dial Tone, Procedural Star Trek Klaxon, 3-Note SIT Chime, Voicemail Beep) via I2S DMA 
 *   directly to the MAX98357A I2S Class-D Amplifier.
 * 
 * - CORE 1 (Arduino Loop / Main Task): Handles rotary pulse counting, handset hook state 
 *   monitoring, wheel return shunt logic, and the 32-pixel WS2812 NeoPixel particle 
 *   rendering engine.
 * 
 * HARDWARE MAPPING:
 * -----------------
 * - GPIO 16 (RX2)  : Rotary Dial Rotary Pulse Contact Switch (Input Pullup)
 * - GPIO 17 (TX2)  : Handset Off-Hook Switch (Input Pullup, LOW = Off-Hook)
 * - GPIO 21 (SDA)  : Rotary Dial Shunt Contact Switch (Input Pullup, LOW = Dial Pulled)
 * - GPIO 22 (SCL)  : WS2812 NeoPixel Data Out (32-LED Ring)
 * - GPIO 25 (D25)  : MAX98357A I2S LRC / WS (Word Select Clock)
 * - GPIO 26 (D26)  : MAX98357A I2S BCLK / SCK (Bit Clock)
 * - GPIO 27 (D27)  : MAX98357A I2S DIN / SD (Serial Audio Data)
 * 
 * ============================================================================
 */

#include <Arduino.h>
#include <Freenove_WS2812_Lib_for_ESP32.h>
#include "driver/i2s.h"

// ============================================================================
// HARDWARE PIN ASSIGNMENTS (WROOM PIN TRANSLATION)
// ============================================================================
const uint8_t DIAL_PIN  = 16; // RX2 : Rotary dial pulse contacts
const uint8_t HOOK_PIN  = 17; // TX2 : Handset cradle hook switch
const uint8_t SHUNT_PIN = 21; // SDA : Rotary dial mechanical shunt switch
const uint8_t LED_PIN   = 22; // SCL : WS2812 NeoPixel data line

// Dedicated I2S Hardware Pins for MAX98357A Class-D DAC/Amplifier
#define I2S_BCLK_PIN  26
#define I2S_LRC_PIN   25
#define I2S_DOUT_PIN  27

// ============================================================================
// LED RING & CANVAS SYSTEM CONFIGURATION
// ============================================================================
#define NUM_LEDS      32   // Physical 32-Pixel NeoPixel Ring
#define MAX_BRIGHT    128  // Software intensity ceiling (0-255) to cap current draw
#define MAX_PARTICLES 20  // Max simultaneous particle allocations to prevent stack fragmentation

Freenove_ESP32_WS2812 strip(NUM_LEDS, LED_PIN, 0, TYPE_GRB);

struct ColorRGB {
  float r, g, b;
};

ColorRGB pixelCanvas[NUM_LEDS];
ColorRGB currentRenderedRGB = { 0.0f, 0.0f, 0.0f };

// Stores a unique, randomized 10-color assignment map generated per dialing session
ColorRGB currentSessionPalette[10];

// ============================================================================
// CORE 0: I2S AUDIO ENGINE & FREERTOS TASK SYNTHESIS
// ============================================================================
#define I2S_NUM         I2S_NUM_0
#define SAMPLE_RATE     44100
#define AUDIO_BUF_SIZE  128

enum AudioMode { 
  AUDIO_OFF, 
  AUDIO_DIAL_TONE, 
  AUDIO_KLAXON, 
  AUDIO_OPERATOR_CHIME, 
  AUDIO_VOICEMAIL_BEEP 
};

// Thread-safe atomic state flag read by Core 0 audio task
volatile AudioMode currentAudioMode = AUDIO_OFF;

// Audio Phase Accumulators for DDS (Direct Digital Synthesis)
uint32_t phase350 = 0;
uint32_t phase440 = 0;

// Phase step sizes calculated for 44.1kHz sample rate (Phase = (Freq * 2^32) / SampleRate)
const uint32_t INC_350  = (uint32_t)((350.0f  * 4294967296.0f) / SAMPLE_RATE);
const uint32_t INC_440  = (uint32_t)((440.0f  * 4294967296.0f) / SAMPLE_RATE);
const uint32_t INC_880  = (uint32_t)((880.0f  * 4294967296.0f) / SAMPLE_RATE);
const uint32_t INC_950  = (uint32_t)((950.0f  * 4294967296.0f) / SAMPLE_RATE);
const uint32_t INC_1000 = (uint32_t)((1000.0f * 4294967296.0f) / SAMPLE_RATE); // Voicemail Beep
const uint32_t INC_1400 = (uint32_t)((1400.0f * 4294967296.0f) / SAMPLE_RATE);
const uint32_t INC_1800 = (uint32_t)((1800.0f * 4294967296.0f) / SAMPLE_RATE);

// Pre-computed High-Precision 256-Step 16-Bit Signed Sine Lookup Table in Flash (PROGMEM)
const int16_t SINE_TABLE_256[256] PROGMEM = {
      0,   804,  1607,  2410,  3211,  4011,  4807,  5601,
   6392,  7179,  7961,  8739,  9511, 10278, 11038, 11792,
  12539, 13278, 14009, 14732, 15446, 16150, 16845, 17530,
  18204, 18867, 19519, 20159, 20787, 21402, 22004, 22594,
  23169, 23731, 24278, 24811, 25329, 25831, 26318, 26789,
  27244, 27683, 28105, 28510, 28897, 29268, 29621, 29956,
  30272, 30571, 30851, 31113, 31356, 31580, 31785, 31970,
  32137, 32284, 32412, 32520, 32609, 32678, 32727, 32757,
  32767, 32757, 32727, 32678, 32609, 32520, 32412, 32284,
  32137, 31970, 31785, 31580, 31356, 31113, 30851, 30571,
  30272, 29956, 29621, 29268, 28897, 28510, 28105, 27683,
  27244, 26789, 26318, 25831, 25329, 24811, 24278, 23731,
  23169, 22594, 22004, 21402, 20787, 20159, 19519, 18867,
  18204, 17530, 16845, 16150, 15446, 14732, 14009, 13278,
  12539, 11792, 11038, 10278,  9511,  8739,  7961,  7179,
   6392,  5601,  4807,  4011,  3211,  2410,  1607,   804,
      0,  -804, -1607, -2410, -3211, -4011, -4807, -5601,
  -6392, -7179, -7961, -8739, -9511,-10278,-11038,-11792,
 -12539,-13278,-14009,-14732,-15446,-16150,-16845,-17530,
 -18204,-18867,-19519,-20159,-20787,-21402,-22004,-22594,
 -23169,-23731,-24278,-24811,-25329,-25831,-26318,-26789,
 -27244,-27683,-28105,-28510,-28897,-29268,-29621,-29956,
 -30272,-30571,-30851,-31113,-31356,-31580,-31785,-31970,
 -32137,-32284,-32412,-32520,-32609,-32678,-32727,-32757,
 -32767,-32757,-32727,-32678,-32609,-32520,-32412,-32284,
 -32137,-31970,-31785,-31580,-31356,-31113,-30851,-30571,
 -30272,-29956,-29621,-29268,-28897,-28510,-28105,-27683,
 -27244,-26789,-26318,-25831,-25329,-24811,-24278,-23731,
 -23169,-22594,-22004,-21402,-20787,-20159,-19519,-18867,
 -18204,-17530,-16845,-16150,-15446,-14732,-14009,-13278,
 -12539,-11792,-11038,-10278, -9511, -8739, -7961, -7179,
  -6392, -5601, -4807, -4011, -3211, -2410, -1607,  -804
};

/**
 * Initializes ESP32 hardware I2S peripheral in master TX mode.
 */
void initI2SAudio() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 6,
    .dma_buf_len = AUDIO_BUF_SIZE,
    .use_apll = false,
    .tx_desc_auto_clear = true
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK_PIN,
    .ws_io_num = I2S_LRC_PIN,
    .data_out_num = I2S_DOUT_PIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM, &pin_config);
}

/**
 * FREERTOS CORE 0 TASK: Continuously synthesizes audio buffers and streams 
 * them to the MAX98357A via I2S DMA without blocking main execution on Core 1.
 */
void audioTask(void *pvParameters) {
  int16_t buffer[AUDIO_BUF_SIZE * 2]; // Stereo interleaved buffer (L/R)
  static unsigned long chimeStartMs = 0;
  static unsigned long klaxonCycleStartMs = 0;

  for (;;) {
    if (currentAudioMode == AUDIO_DIAL_TONE) {
      chimeStartMs = 0;
      klaxonCycleStartMs = 0;
      
      // Standard US Dual-Tone Multi-Frequency Dial Tone (350 Hz + 440 Hz)
      for (int i = 0; i < AUDIO_BUF_SIZE; i++) {
        phase350 += INC_350;
        phase440 += INC_440;

        uint8_t idx350 = (phase350 >> 24) & 0xFF;
        uint8_t idx440 = (phase440 >> 24) & 0xFF;

        int32_t mixed = (int32_t)pgm_read_word(&SINE_TABLE_256[idx350]) + (int32_t)pgm_read_word(&SINE_TABLE_256[idx440]);
        int16_t sample = (int16_t)(mixed / 64); // Digital attenuation factor for optimal volume

        buffer[i * 2]     = sample; // Left Channel
        buffer[i * 2 + 1] = sample; // Right Channel
      }
      size_t bytesWritten;
      i2s_write(I2S_NUM, buffer, sizeof(buffer), &bytesWritten, portMAX_DELAY);
    } 
    else if (currentAudioMode == AUDIO_KLAXON) {
      chimeStartMs = 0;
      if (klaxonCycleStartMs == 0) klaxonCycleStartMs = millis();

      unsigned long cycleTime = (millis() - klaxonCycleStartMs) % 1200; // 1.2-second total loop duration
      
      // 1. FM Frequency Envelope: Curved rise from 300Hz up to 850Hz over 800ms
      float pitchProgress = min((float)cycleTime / 800.0f, 1.0f);
      float currentFreq = 300.0f + (550.0f * pitchProgress * pitchProgress);
      uint32_t currentInc = (uint32_t)((currentFreq * 4294967296.0f) / SAMPLE_RATE);

      // 2. AM Amplitude Envelope: Double Attack Pulse + Exponential Decay
      float volumeEnv = 0.0f;
      if (cycleTime < 150) {
        volumeEnv = sin(((float)cycleTime / 150.0f) * PI); // Pulse 1
      } else if (cycleTime >= 200 && cycleTime < 400) {
        volumeEnv = sin(((float)(cycleTime - 200) / 200.0f) * PI); // Pulse 2
      } else if (cycleTime >= 400 && cycleTime < 900) {
        float fadeProg = (float)(cycleTime - 400) / 500.0f;
        volumeEnv = (1.0f - fadeProg) * (1.0f - fadeProg); // Decay Tail
      } else {
        volumeEnv = 0.0f; // Silent gap before next blast
      }

      for (int i = 0; i < AUDIO_BUF_SIZE; i++) {
        phase350 += currentInc;
        uint8_t idx = (phase350 >> 24) & 0xFF;

        int32_t sineSample = pgm_read_word(&SINE_TABLE_256[idx]);
        
        // Blend fundamental with 2nd harmonic (octave over) for aggressive horn timbre
        uint8_t idxHarmonic = ((phase350 * 2) >> 24) & 0xFF;
        int32_t harmonicSample = pgm_read_word(&SINE_TABLE_256[idxHarmonic]);

        int32_t combined = (sineSample * 0.7f) + (harmonicSample * 0.3f);
        int16_t finalSample = (int16_t)((combined / 32) * volumeEnv);

        buffer[i * 2]     = finalSample;
        buffer[i * 2 + 1] = finalSample;
      }
      size_t bytesWritten;
      i2s_write(I2S_NUM, buffer, sizeof(buffer), &bytesWritten, portMAX_DELAY);
    }
    else if (currentAudioMode == AUDIO_VOICEMAIL_BEEP) {
      klaxonCycleStartMs = 0;
      chimeStartMs = 0;
      
      // Standard 1000 Hz Voicemail Record Tone
      for (int i = 0; i < AUDIO_BUF_SIZE; i++) {
        phase350 += INC_1000;
        uint8_t idx = (phase350 >> 24) & 0xFF;
        int16_t sample = (int16_t)((int32_t)pgm_read_word(&SINE_TABLE_256[idx]) / 32);

        buffer[i * 2]     = sample;
        buffer[i * 2 + 1] = sample;
      }
      size_t bytesWritten;
      i2s_write(I2S_NUM, buffer, sizeof(buffer), &bytesWritten, portMAX_DELAY);
    }
    else if (currentAudioMode == AUDIO_OPERATOR_CHIME) {
      klaxonCycleStartMs = 0;
      if (chimeStartMs == 0) chimeStartMs = millis();

      unsigned long elapsed = millis() - chimeStartMs;
      uint32_t inc = 0;

      // 3-Note Ascending SIT Tone: 950Hz -> 1400Hz -> 1800Hz (330ms each)
      if (elapsed < 330)       inc = INC_950;
      else if (elapsed < 660)  inc = INC_1400;
      else if (elapsed < 990)  inc = INC_1800;
      else if (elapsed >= 2990) { // 990ms tone + 2000ms pause = 2990ms cycle
        chimeStartMs = millis(); // Reset loop cycle
      }

      if (inc > 0) {
        for (int i = 0; i < AUDIO_BUF_SIZE; i++) {
          phase350 += inc;
          uint8_t idx = (phase350 >> 24) & 0xFF;
          int16_t sample = (int16_t)((int32_t)pgm_read_word(&SINE_TABLE_256[idx]) / 32);

          buffer[i * 2]     = sample;
          buffer[i * 2 + 1] = sample;
        }
      } else {
        memset(buffer, 0, sizeof(buffer)); // Mute buffer during 2-second pause gap
      }

      size_t bytesWritten;
      i2s_write(I2S_NUM, buffer, sizeof(buffer), &bytesWritten, portMAX_DELAY);
    }
    else {
      // Idle Silence Mode
      klaxonCycleStartMs = 0;
      chimeStartMs = 0;
      memset(buffer, 0, sizeof(buffer));
      size_t bytesWritten;
      i2s_write(I2S_NUM, buffer, sizeof(buffer), &bytesWritten, portMAX_DELAY);
      vTaskDelay(pdMS_TO_TICKS(10)); // Yield thread when silent to preserve CPU cycles
    }
  }
}

// ============================================================================
// PARTICLE LIGHT ENGINE DATA STRUCTURES
// ============================================================================
enum ParticlePhase { PHASE_INACTIVE, PHASE_SPARKLE, PHASE_ORBIT, PHASE_POP };

struct Particle {
  bool active = false;
  ParticlePhase phase = PHASE_INACTIVE;
  char digitChar;
  
  ColorRGB color;
  float pos;
  int direction;           // +1 (CW) or -1 (CCW)
  float speed;             // Calculated angular velocity
  float decayRate;         // Individual trail decay factor (0.75 - 0.92)
  unsigned long lifetimeMs; // Lifetime scaling inverse to speed (5,000ms - 20,000ms)
  unsigned long sparkleDurationMs;
  
  unsigned long phaseStartMillis;
  int popRadius;
};

Particle particles[MAX_PARTICLES];

struct Star {
  bool active = false;
  float brightness = 0.0f;
  float fadeSpeed = 0.02f;
  int state = 0; // 0 = Fading In, 1 = Fading Out
};

Star starSky[NUM_LEDS];

// ============================================================================
// TIMING & ROTARY STATE PARAMETERS
// ============================================================================
int PULSE_LEVEL = LOW;

const unsigned long DEBOUNCE_MS        = 10;
const unsigned long DIGIT_GAP_MS       = 250;  // Gap threshold between pulses to finalize digit
const unsigned long NUMBER_GAP_MS      = 3000; // Gap threshold to finalize complete phone number
const unsigned long DIGIT_SPAWN_GAP_MS = 1000; // Constant delay between particle spawns

const bool DEBUG = true;

int stableState = HIGH;
int lastReadState = HIGH;
unsigned long lastChangeTime = 0;

int pulseCount = 0;
bool dialActive = false;
unsigned long lastPulseMillis = 0;

String dialedNumber = "";
bool numberPending = false;

enum PhoneState {
  STATE_IDLE_PULSE,
  STATE_OFF_HOOK,
  STATE_DIALING_SWEEP,
  STATE_DIALING_SOLID,
  STATE_DIGIT_FADE,
  STATE_PLAYBACK,
  STATE_RED_ALERT,
  STATE_EASTER_EGG_ZERO
};

PhoneState currentSystemState = STATE_IDLE_PULSE;
bool lastHookState = false;
bool lastShuntState = false;

unsigned long fadeStartMillis = 0;
const unsigned long FADE_DURATION_MS = 300;

// ============================================================================
// COLOR MATH & PALETTE MANAGEMENT
// ============================================================================

void setRingColorRGB(uint8_t r, uint8_t g, uint8_t b) {
  currentRenderedRGB = { (float)r, (float)g, (float)b };
  for (int i = 0; i < NUM_LEDS; i++) {
    pixelCanvas[i] = currentRenderedRGB;
  }
  strip.setAllLedsColor(r, g, b);
}

void renderCanvas() {
  for (int i = 0; i < NUM_LEDS; i++) {
    uint8_t r = (uint8_t)constrain(pixelCanvas[i].r, 0, MAX_BRIGHT);
    uint8_t g = (uint8_t)constrain(pixelCanvas[i].g, 0, MAX_BRIGHT);
    uint8_t b = (uint8_t)constrain(pixelCanvas[i].b, 0, MAX_BRIGHT);
    strip.setLedColorData(i, r, g, b);
  }
  strip.show();
}

void clearCanvas() {
  for (int i = 0; i < NUM_LEDS; i++) {
    pixelCanvas[i] = { 0.0f, 0.0f, 0.0f };
  }
  strip.setAllLedsColor(0, 0, 0);
  currentRenderedRGB = { 0.0f, 0.0f, 0.0f };
}

ColorRGB hsvToRgb(uint8_t h, uint8_t s, uint8_t v) {
  uint8_t region, remainder, p, q, t;
  uint8_t r, g, b;

  if (s == 0) {
    r = g = b = v;
  } else {
    region = h / 43;
    remainder = (h - (region * 43)) * 6;

    p = (v * (255 - s)) >> 8;
    q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

    switch (region) {
      case 0:  r = v; g = t; b = p; break;
      case 1:  r = q; g = v; b = p; break;
      case 2:  r = p; g = v; b = t; break;
      case 3:  r = p; g = q; b = v; break;
      case 4:  r = t; g = p; b = v; break;
      default: r = v; g = p; b = q; break;
    }
  }

  return { (r * MAX_BRIGHT) / 255.0f, (g * MAX_BRIGHT) / 255.0f, (b * MAX_BRIGHT) / 255.0f };
}

/**
 * Generates 10 distinct, vibrant random colors mapped to digits 0-9 
 * for the current dialing session.
 */
void randomizeSessionPalette() {
  uint8_t startHue = random(0, 256);
  for (int i = 0; i < 10; i++) {
    uint8_t hue = (startHue + (i * 25) + random(-5, 6)) % 256;
    currentSessionPalette[i] = hsvToRgb(hue, 255, 255);
  }
}

ColorRGB getDigitColor(char digitChar) {
  if (digitChar >= '0' && digitChar <= '9') {
    int digit = digitChar - '0';
    return currentSessionPalette[digit];
  }
  return { (float)MAX_BRIGHT, (float)MAX_BRIGHT, (float)MAX_BRIGHT };
}

// ============================================================================
// ANIMATION & TRANSITION ENGINE
// ============================================================================

void runCounterClockwiseSweep(uint8_t r, uint8_t g, uint8_t b, uint16_t frameDelay = 6) {
  for (int i = NUM_LEDS - 1; i >= 0; i--) {
    if (digitalRead(HOOK_PIN) == HIGH) break;

    strip.setAllLedsColor(0, 0, 0);
    strip.setLedColorData(i, r, g, b);
    strip.setLedColorData((i + 1) % NUM_LEDS, r / 2, g / 2, b / 2);
    strip.setLedColorData((i + 2) % NUM_LEDS, r / 4, g / 4, b / 4);
    strip.show();

    vTaskDelay(pdMS_TO_TICKS(frameDelay));
  }
  setRingColorRGB(r, g, b);
}

void runShuntReleasePulse() {
  setRingColorRGB(MAX_BRIGHT, MAX_BRIGHT, MAX_BRIGHT);
  vTaskDelay(pdMS_TO_TICKS(40));

  unsigned long start = millis();
  const unsigned long PULSE_FADE_MS = 150;

  while (millis() - start < PULSE_FADE_MS) {
    if (digitalRead(HOOK_PIN) == HIGH) break;

    float progress = (float)(millis() - start) / (float)PULSE_FADE_MS;
    uint8_t r = (uint8_t)(MAX_BRIGHT * (1.0f - progress));
    uint8_t g = (uint8_t)(MAX_BRIGHT * (1.0f - progress));
    uint8_t b = MAX_BRIGHT;

    setRingColorRGB(r, g, b);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  setRingColorRGB(0, 0, MAX_BRIGHT);
}

void runHookStateFade(uint8_t targetR, uint8_t targetG, uint8_t targetB, unsigned long fadeDurationMs, bool targetHookOff) {
  ColorRGB startColor = currentRenderedRGB;
  unsigned long start = millis();

  while (millis() - start < fadeDurationMs) {
    bool currentHookOff = (digitalRead(HOOK_PIN) == LOW);
    if (currentHookOff != targetHookOff) break;

    float progress = (float)(millis() - start) / (float)fadeDurationMs;

    uint8_t r = (uint8_t)(startColor.r + ((targetR - startColor.r) * progress));
    uint8_t g = (uint8_t)(startColor.g + ((targetG - startColor.g) * progress));
    uint8_t b = (uint8_t)(startColor.b + ((targetB - startColor.b) * progress));

    setRingColorRGB(r, g, b);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  setRingColorRGB(targetR, targetG, targetB);
}

// ============================================================================
// PARTICLE ENGINE RENDER LOOPS
// ============================================================================

void spawnParticle(char digitChar, ColorRGB forcedColor = { -1.0f, -1.0f, -1.0f }) {
  for (int i = 0; i < MAX_PARTICLES; i++) {
    if (!particles[i].active) {
      particles[i].active = true;
      particles[i].phase = PHASE_SPARKLE;
      particles[i].digitChar = digitChar;
      
      if (forcedColor.r >= 0.0f) {
        particles[i].color = forcedColor;
      } else {
        particles[i].color = getDigitColor(digitChar);
      }

      particles[i].pos = random(0, NUM_LEDS);
      particles[i].direction = (random(0, 2) == 0) ? 1 : -1;
      
      int speedRating = random(1, 11);
      particles[i].speed = (speedRating * 0.4f) + random(1, 8) / 10.0f;

      float normSpeed = (speedRating - 1.0f) / 9.0f;
      particles[i].lifetimeMs = 20000 - (normSpeed * 15000);

      particles[i].decayRate = 0.75f + (random(0, 18) / 100.0f);
      particles[i].sparkleDurationMs = random(10, 41) * 10;

      particles[i].phaseStartMillis = millis();
      particles[i].popRadius = 0;
      break;
    }
  }
}

bool hasActiveParticles() {
  for (int i = 0; i < MAX_PARTICLES; i++) {
    if (particles[i].active) return true;
  }
  return false;
}

int countActiveParticles() {
  int count = 0;
  for (int i = 0; i < MAX_PARTICLES; i++) {
    if (particles[i].active) count++;
  }
  return count;
}

void updateAndRenderParticles() {
  unsigned long now = millis();

  for (int i = 0; i < MAX_PARTICLES; i++) {
    if (!particles[i].active) continue;

    unsigned long elapsed = now - particles[i].phaseStartMillis;

    // Apply decay multiplier across canvas frame
    for (int p = 0; p < NUM_LEDS; p++) {
      pixelCanvas[p].r *= particles[i].decayRate;
      pixelCanvas[p].g *= particles[i].decayRate;
      pixelCanvas[p].b *= particles[i].decayRate;
    }

    switch (particles[i].phase) {
      case PHASE_SPARKLE: {
        if (elapsed >= particles[i].sparkleDurationMs) {
          particles[i].phase = PHASE_ORBIT;
          particles[i].phaseStartMillis = now;
        } else {
          int sparkleIdx = random(0, NUM_LEDS);
          pixelCanvas[sparkleIdx].r = max(pixelCanvas[sparkleIdx].r, particles[i].color.r * 0.8f);
          pixelCanvas[sparkleIdx].g = max(pixelCanvas[sparkleIdx].g, particles[i].color.g * 0.8f);
          pixelCanvas[sparkleIdx].b = max(pixelCanvas[sparkleIdx].b, particles[i].color.b * 0.8f);
        }
        break;
      }

      case PHASE_ORBIT: {
        if (elapsed >= particles[i].lifetimeMs) {
          particles[i].phase = PHASE_POP;
          particles[i].phaseStartMillis = now;
          particles[i].popRadius = 0;
        } else {
          particles[i].pos += (particles[i].direction * (particles[i].speed / 10.0f));
          if (particles[i].pos >= NUM_LEDS) particles[i].pos -= NUM_LEDS;
          if (particles[i].pos < 0)         particles[i].pos += NUM_LEDS;

          int currentIdx = (int)particles[i].pos;
          int nextIdx = (currentIdx + 1) % NUM_LEDS;
          float frac = particles[i].pos - currentIdx;

          pixelCanvas[currentIdx].r = max(pixelCanvas[currentIdx].r, particles[i].color.r * (1.0f - frac));
          pixelCanvas[currentIdx].g = max(pixelCanvas[currentIdx].g, particles[i].color.g * (1.0f - frac));
          pixelCanvas[currentIdx].b = max(pixelCanvas[currentIdx].b, particles[i].color.b * (1.0f - frac));

          pixelCanvas[nextIdx].r = max(pixelCanvas[nextIdx].r, particles[i].color.r * frac);
          pixelCanvas[nextIdx].g = max(pixelCanvas[nextIdx].g, particles[i].color.g * frac);
          pixelCanvas[nextIdx].b = max(pixelCanvas[nextIdx].b, particles[i].color.b * frac);
        }
        break;
      }

      case PHASE_POP: {
        int centerIdx = (int)particles[i].pos;
        int radius = particles[i].popRadius;

        if (radius > NUM_LEDS / 2) {
          particles[i].active = false;
          particles[i].phase = PHASE_INACTIVE;
        } else {
          int leftWing  = (centerIdx - radius + NUM_LEDS) % NUM_LEDS;
          int rightWing = (centerIdx + radius) % NUM_LEDS;

          float brightnessFactor = 1.0f - ((float)radius / (NUM_LEDS / 2.0f));
          ColorRGB burstColor = { 
            particles[i].color.r * brightnessFactor, 
            particles[i].color.g * brightnessFactor, 
            particles[i].color.b * brightnessFactor 
          };

          pixelCanvas[leftWing].r  = max(pixelCanvas[leftWing].r, burstColor.r);
          pixelCanvas[leftWing].g  = max(pixelCanvas[leftWing].g, burstColor.g);
          pixelCanvas[leftWing].b  = max(pixelCanvas[leftWing].b, burstColor.b);

          pixelCanvas[rightWing].r = max(pixelCanvas[rightWing].r, burstColor.r);
          pixelCanvas[rightWing].g = max(pixelCanvas[rightWing].g, burstColor.g);
          pixelCanvas[rightWing].b = max(pixelCanvas[rightWing].b, burstColor.b);

          particles[i].popRadius++;
        }
        break;
      }

      default:
        break;
    }
  }

  renderCanvas();
}

/**
 * Standard multi-digit playback animation sequence.
 * Includes explicit handset hook checks to allow instant cancellation on hangup.
 */
void playbackSequence(String sequence) {
  if (sequence.length() == 0) return;

  for (int i = 0; i < MAX_PARTICLES; i++) particles[i].active = false;

  for (size_t i = 0; i < sequence.length(); i++) {
    if (digitalRead(HOOK_PIN) == HIGH) {
      clearCanvas();
      return;
    }

    spawnParticle(sequence.charAt(i));

    unsigned long delayStart = millis();
    while (millis() - delayStart < DIGIT_SPAWN_GAP_MS) {
      if (digitalRead(HOOK_PIN) == HIGH) {
        clearCanvas();
        return;
      }
      updateAndRenderParticles();
      vTaskDelay(pdMS_TO_TICKS(15));
    }
  }

  while (hasActiveParticles()) {
    if (digitalRead(HOOK_PIN) == HIGH) { // Escape immediately on hangup
      clearCanvas();
      for (int i = 0; i < MAX_PARTICLES; i++) particles[i].active = false;
      return;
    }
    updateAndRenderParticles();
    vTaskDelay(pdMS_TO_TICKS(15));
  }

  clearCanvas();

  unsigned long pauseStart = millis();
  while (millis() - pauseStart < 1000) {
    if (digitalRead(HOOK_PIN) == HIGH) return;
    vTaskDelay(pdMS_TO_TICKS(15));
  }

  bool hookOff = (digitalRead(HOOK_PIN) == LOW);
  unsigned long fadeStart = millis();
  const unsigned long TRANSITION_FADE_MS = 1000;

  while (millis() - fadeStart < TRANSITION_FADE_MS) {
    if (digitalRead(HOOK_PIN) != (hookOff ? LOW : HIGH)) break;

    float progress = (float)(millis() - fadeStart) / (float)TRANSITION_FADE_MS;

    uint8_t targetR, targetG, targetB;
    if (hookOff) {
      targetR = 0;
      targetG = MAX_BRIGHT;
      targetB = 0;
    } else {
      targetR = (uint8_t)(255 * 0.12f * (MAX_BRIGHT / 255.0f));
      targetG = (uint8_t)(160 * 0.12f * (MAX_BRIGHT / 255.0f));
      targetB = (uint8_t)(15  * 0.12f * (MAX_BRIGHT / 255.0f));
    }

    uint8_t curR = (uint8_t)(targetR * progress);
    uint8_t curG = (uint8_t)(targetG * progress);
    uint8_t curB = (uint8_t)(targetB * progress);

    setRingColorRGB(curR, curG, curB);
    vTaskDelay(pdMS_TO_TICKS(15));
  }
}

// ============================================================================
// EASTER EGG ENGINES
// ============================================================================

void runRedAlertSwarm() {
  currentAudioMode = AUDIO_KLAXON; // Start Procedural Star Trek Klaxon

  for (int i = 0; i < MAX_PARTICLES; i++) particles[i].active = false;
  clearCanvas();

  ColorRGB pureRed = { (float)MAX_BRIGHT, 0.0f, 0.0f };

  for (int p = 0; p < 16; p++) {
    spawnParticle('!', pureRed);
  }

  while (digitalRead(HOOK_PIN) == LOW) {
    // Non-blocking capacity check to avoid thread spin locks
    if (countActiveParticles() < 16) {
      spawnParticle('!', pureRed);
    }

    updateAndRenderParticles();
    vTaskDelay(pdMS_TO_TICKS(15));
  }

  clearCanvas();
  for (int i = 0; i < MAX_PARTICLES; i++) particles[i].active = false;
  currentAudioMode = AUDIO_OFF; // Stop Klaxon on hangup
}

void runHeartbeatHslWalk() {
  uint8_t currentHue = random(0, 256);
  uint8_t currentSat = random(180, 255);
  int hueDirection = (random(0, 2) == 0) ? 1 : -1;

  unsigned long cycleStart = millis();

  while (digitalRead(HOOK_PIN) == LOW) {
    unsigned long elapsed = (millis() - cycleStart) % 1200;

    currentHue += hueDirection * 1;

    float pulseVal = 0.05f;
    if (elapsed < 150) {
      pulseVal = 0.05f + 0.95f * (sin((float)elapsed / 150.0f * PI));
    } else if (elapsed >= 220 && elapsed < 420) {
      pulseVal = 0.05f + 0.60f * (sin((float)(elapsed - 220) / 200.0f * PI));
    }

    uint8_t targetValue = (uint8_t)(pulseVal * 220);
    ColorRGB hsvColor = hsvToRgb(currentHue, currentSat, targetValue);

    setRingColorRGB((uint8_t)hsvColor.r, (uint8_t)hsvColor.g, (uint8_t)hsvColor.b);
    vTaskDelay(pdMS_TO_TICKS(15));
  }
}

void runNightSkyTwinkle() {
  for (int i = 0; i < NUM_LEDS; i++) {
    starSky[i].active = false;
    starSky[i].brightness = 0.0f;
  }

  ColorRGB startRGB = currentRenderedRGB;
  for (int f = 0; f < 20; f++) {
    float prog = (float)f / 20.0f;
    setRingColorRGB((uint8_t)(startRGB.r * (1.0f - prog)), 
                    (uint8_t)(startRGB.g * (1.0f - prog)), 
                    (uint8_t)(startRGB.b * (1.0f - prog)));
    vTaskDelay(pdMS_TO_TICKS(15));
  }
  clearCanvas();

  while (digitalRead(HOOK_PIN) == LOW) {
    if (random(0, 100) < 15) {
      int idx = random(0, NUM_LEDS);
      if (!starSky[idx].active) {
        starSky[idx].active = true;
        starSky[idx].brightness = 0.0f;
        starSky[idx].fadeSpeed = random(5, 25) / 1000.0f;
        starSky[idx].state = 0;
      }
    }

    for (int i = 0; i < NUM_LEDS; i++) {
      if (starSky[i].active) {
        if (starSky[i].state == 0) {
          starSky[i].brightness += starSky[i].fadeSpeed;
          if (starSky[i].brightness >= 1.0f) {
            starSky[i].brightness = 1.0f;
            starSky[i].state = 1;
          }
        } else {
          starSky[i].brightness -= starSky[i].fadeSpeed;
          if (starSky[i].brightness <= 0.0f) {
            starSky[i].brightness = 0.0f;
            starSky[i].active = false;
          }
        }

        float bFactor = starSky[i].brightness;
        uint8_t r = (uint8_t)(MAX_BRIGHT * bFactor);
        uint8_t g = (uint8_t)(MAX_BRIGHT * 0.82f * bFactor);
        uint8_t b = (uint8_t)(MAX_BRIGHT * 0.45f * bFactor);

        strip.setLedColorData(i, r, g, b);
      } else {
        strip.setLedColorData(i, 0, 0, 0);
      }
    }

    strip.show();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void runZeroEasterEgg() {
  currentAudioMode = AUDIO_OPERATOR_CHIME; // Triggers looping 3-note chime + 2s pause

  int choice = random(0, 2);
  if (choice == 0) {
    runHeartbeatHslWalk();
  } else {
    runNightSkyTwinkle();
  }

  currentAudioMode = AUDIO_OFF;
}

// ============================================================================
// ARDUINO SETUP & MAIN CORE 1 EXECUTION
// ============================================================================

void setup() {
  Serial.begin(115200);

  pinMode(DIAL_PIN, INPUT_PULLUP);
  pinMode(HOOK_PIN, INPUT_PULLUP);
  pinMode(SHUNT_PIN, INPUT_PULLUP);

  // Initialize I2S Hardware Output
  initI2SAudio();

  // Pin audio generation to Core 0 with priority 1
  xTaskCreatePinnedToCore(
    audioTask,    // Task function pointer
    "AudioTask",  // Task identifier string
    4096,         // Allocated stack memory depth
    NULL,         // Task input parameters
    1,            // Priority level
    NULL,         // Task handle pointer
    0             // Target CPU Core 0
  );

  strip.begin();
  strip.setBrightness(255);
  clearCanvas();

  delay(50);
  stableState = digitalRead(DIAL_PIN);
  lastReadState = stableState;

  lastHookState  = (digitalRead(HOOK_PIN) == LOW);
  lastShuntState = (digitalRead(SHUNT_PIN) == LOW);

  if (lastHookState) {
    currentSystemState = STATE_OFF_HOOK;
    currentAudioMode = AUDIO_DIAL_TONE;
    setRingColorRGB(0, MAX_BRIGHT, 0);
  } else {
    currentSystemState = STATE_IDLE_PULSE;
    currentAudioMode = AUDIO_OFF;
  }

  Serial.println("\n[SYSTEM] Rotary Controller Ready.");
}

void loop() {
  unsigned long now = millis();

  // 1. Read Hardware Inputs
  int rawState = digitalRead(DIAL_PIN);
  bool hookOffCradle = (digitalRead(HOOK_PIN) == LOW);
  bool shuntFlipped  = (digitalRead(SHUNT_PIN) == LOW);

  // 2. Handset Hook State Switch Logic
  if (hookOffCradle != lastHookState) {
    vTaskDelay(pdMS_TO_TICKS(20));
    if (hookOffCradle) {
      if (DEBUG) Serial.println("\n[HOOK] Handset LIFTED -> Dial Tone ON & Fade to Green");
      currentAudioMode = AUDIO_DIAL_TONE;
      runHookStateFade(0, MAX_BRIGHT, 0, 250, true);
      currentSystemState = STATE_OFF_HOOK;
    } else {
      if (DEBUG) Serial.println("\n[HOOK] Handset REPLACED -> Audio OFF & Ambient Yellow");
      currentAudioMode = AUDIO_OFF;
      
      for (int i = 0; i < MAX_PARTICLES; i++) particles[i].active = false;
      
      uint8_t targetR = (uint8_t)(255 * 0.12f * (MAX_BRIGHT / 255.0f));
      uint8_t targetG = (uint8_t)(160 * 0.12f * (MAX_BRIGHT / 255.0f));
      uint8_t targetB = (uint8_t)(15  * 0.12f * (MAX_BRIGHT / 255.0f));
      
      runHookStateFade(targetR, targetG, targetB, 350, false);
      currentSystemState = STATE_IDLE_PULSE;

      dialedNumber = "";
      numberPending = false;
      pulseCount = 0;
      dialActive = false;
    }
    lastHookState = hookOffCradle;
  }

  // 3. Shunt Contact Wheel Rotation Monitoring
  if (hookOffCradle && currentSystemState != STATE_PLAYBACK && currentSystemState != STATE_RED_ALERT && currentSystemState != STATE_EASTER_EGG_ZERO) {
    if (shuntFlipped && !lastShuntState) {
      if (DEBUG) Serial.println("[SHUNT] Wheel pulled -> Cut Dial Tone & Trigger CCW Sweep");
      currentAudioMode = AUDIO_OFF;
      runCounterClockwiseSweep(0, 0, MAX_BRIGHT, 6);
      currentSystemState = STATE_DIALING_SOLID;
    }
    else if (!shuntFlipped && lastShuntState) {
      if (DEBUG) Serial.println("[SHUNT] Wheel returned -> Single White Pulse");
      runShuntReleasePulse();
    }
  }
  lastShuntState = shuntFlipped;

  // 4. Core Pulse Counting State Machine (Debounced)
  if (rawState != lastReadState) {
    lastReadState = rawState;
    lastChangeTime = now;
  }

  if ((now - lastChangeTime) >= DEBOUNCE_MS) {
    if (rawState != stableState) {
      stableState = rawState;

      if (stableState != PULSE_LEVEL) {
        pulseCount++;
        dialActive = true;
        lastPulseMillis = now;

        currentAudioMode = AUDIO_OFF; // Cut dial tone instantly on pulse edge

        if (hookOffCradle && currentSystemState != STATE_DIALING_SWEEP && currentSystemState != STATE_RED_ALERT && currentSystemState != STATE_EASTER_EGG_ZERO) {
          currentSystemState = STATE_DIALING_SOLID;
        }

        if (DEBUG) {
          Serial.print("  -> Pulse counted, running total for current digit: ");
          Serial.println(pulseCount);
        }
      }
    }
  }

  // 5. Digit & Full Sequence Timeout Router
  if (dialActive && (now - lastPulseMillis) >= DIGIT_GAP_MS) {
    int digit = (pulseCount == 10) ? 0 : pulseCount;
    if (pulseCount > 0) {
      if (DEBUG) {
        Serial.print("  -> Digit finalized: ");
        Serial.println(digit);
      }
      dialedNumber += String(digit);
      numberPending = true;

      if (hookOffCradle && currentSystemState != STATE_RED_ALERT && currentSystemState != STATE_EASTER_EGG_ZERO) {
        currentSystemState = STATE_DIGIT_FADE;
        fadeStartMillis = now;
      }
    }
    pulseCount = 0;
    dialActive = false;
  }

  if (numberPending && !dialActive && (now - lastPulseMillis) >= NUMBER_GAP_MS) {
    Serial.print("\nNumber dialed: ");
    Serial.println(dialedNumber);

    randomizeSessionPalette();

    // Sequence Routing
    if (dialedNumber == "911") {
      currentSystemState = STATE_RED_ALERT;
      runRedAlertSwarm();

      if (digitalRead(HOOK_PIN) == LOW) {
        currentSystemState = STATE_OFF_HOOK;
      } else {
        currentSystemState = STATE_IDLE_PULSE;
      }
    } 
    else if (dialedNumber == "0") {
      currentSystemState = STATE_EASTER_EGG_ZERO;
      runZeroEasterEgg();

      if (digitalRead(HOOK_PIN) == LOW) {
        currentSystemState = STATE_OFF_HOOK;
      } else {
        currentSystemState = STATE_IDLE_PULSE;
      }
    }
    else {
      // Standard Dialing Playback Sequence
      if (DEBUG) Serial.println("[AUDIO] Playing Voicemail Record Beep...");
      
      currentAudioMode = AUDIO_VOICEMAIL_BEEP;
      vTaskDelay(pdMS_TO_TICKS(500)); // Play 500ms 1kHz beep
      currentAudioMode = AUDIO_OFF;
      vTaskDelay(pdMS_TO_TICKS(100)); // Short silence gap

      currentSystemState = STATE_PLAYBACK;
      playbackSequence(dialedNumber);

      if (digitalRead(HOOK_PIN) == LOW) {
        currentSystemState = STATE_OFF_HOOK;
      } else {
        currentSystemState = STATE_IDLE_PULSE;
      }
    }

    dialedNumber = "";
    numberPending = false;
  }

  // 6. Base Light Machine Transitions
  switch (currentSystemState) {
    case STATE_IDLE_PULSE: {
      float breathVal = 0.12f + 0.88f * ((sin((float)now / 10000.0f * 2.0f * PI) + 1.0f) / 2.0f);
      
      uint8_t r = (uint8_t)(255 * breathVal * (MAX_BRIGHT / 255.0f));
      uint8_t g = (uint8_t)(160 * breathVal * (MAX_BRIGHT / 255.0f));
      uint8_t b = (uint8_t)(15  * breathVal * (MAX_BRIGHT / 255.0f));
      
      setRingColorRGB(r, g, b);
      break;
    }

    case STATE_OFF_HOOK:
      setRingColorRGB(0, MAX_BRIGHT, 0); // Solid Green
      break;

    case STATE_DIALING_SOLID:
      setRingColorRGB(0, 0, MAX_BRIGHT); // Solid Blue
      break;

    case STATE_DIGIT_FADE: {
      unsigned long elapsed = now - fadeStartMillis;
      if (elapsed >= FADE_DURATION_MS) {
        setRingColorRGB(0, 0, 0);
      } else {
        float progress = (float)elapsed / (float)FADE_DURATION_MS;
        uint8_t b = (uint8_t)(MAX_BRIGHT * (1.0f - progress));
        setRingColorRGB(0, 0, b);
      }
      break;
    }

    case STATE_RED_ALERT:
    case STATE_EASTER_EGG_ZERO:
    case STATE_DIALING_SWEEP:
    case STATE_PLAYBACK:
      break;
  }
}