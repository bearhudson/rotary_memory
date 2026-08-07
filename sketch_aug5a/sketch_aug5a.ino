#include <Arduino.h>
#include <Freenove_WS2812_Lib_for_ESP32.h>

// ==========================================
// PIN & LED DEFINITIONS (REMAPPED)
// ==========================================
const uint8_t DIAL_PIN  = 16; // RX2
const uint8_t HOOK_PIN  = 17; // TX2
const uint8_t SHUNT_PIN = 21; // SDA
const uint8_t LED_PIN   = 22; // SCL

#define NUM_LEDS     32   // 32-Pixel NeoPixel Ring
#define MAX_BRIGHT   128  // Half intensity ceiling (0-255)
#define MAX_PARTICLES 20  // Max simultaneous digits/particles on the ring

Freenove_ESP32_WS2812 strip(NUM_LEDS, LED_PIN, 0, TYPE_GRB);

struct ColorRGB {
  float r, g, b;
};

ColorRGB pixelCanvas[NUM_LEDS];

// Track actual rendered RGB levels for seamless fade transitions
ColorRGB currentRenderedRGB = { 0.0f, 0.0f, 0.0f };

// ==========================================
// PARTICLE ENGINE DATA STRUCTURES
// ==========================================
enum ParticlePhase { PHASE_INACTIVE, PHASE_SPARKLE, PHASE_ORBIT, PHASE_POP };

struct Particle {
  bool active = false;
  ParticlePhase phase = PHASE_INACTIVE;
  char digitChar;
  
  ColorRGB color;
  float pos;
  int direction;           // +1 (CW) or -1 (CCW)
  float speed;             // Calculated angular velocity
  float decayRate;         // Individual trail decay (0.75 - 0.92)
  unsigned long lifetimeMs; // Lifetime inverse to speed (5,000ms - 20,000ms)
  unsigned long sparkleDurationMs; // Sparkle duration (100ms - 400ms)
  
  unsigned long phaseStartMillis;
  int popRadius;
};

Particle particles[MAX_PARTICLES];

// Star structure for Option 2 Night Sky Twinkle
struct Star {
  bool active = false;
  float brightness = 0.0f;
  float fadeSpeed = 0.02f;
  int state = 0; // 0 = Fading In, 1 = Fading Out
};

Star starSky[NUM_LEDS];

// ==========================================
// TIMING & DIAL PARAMETERS
// ==========================================
int PULSE_LEVEL = LOW;

const unsigned long DEBOUNCE_MS   = 10;
const unsigned long DIGIT_GAP_MS  = 250;
const unsigned long NUMBER_GAP_MS = 3000;

const unsigned long DIGIT_SPAWN_GAP_MS = 1000; // 1-second constant delay between digit spawns

const bool DEBUG = true;

// --- Pulse Tracking Variables ---
int stableState = HIGH;
int lastReadState = HIGH;
unsigned long lastChangeTime = 0;

int pulseCount = 0;
bool dialActive = false;
unsigned long lastPulseMillis = 0;

String dialedNumber = "";
bool numberPending = false;

// --- System State Machine ---
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

// Animation Tracking
unsigned long fadeStartMillis = 0;
const unsigned long FADE_DURATION_MS = 300;

// Helper RGB set
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

// Convert Integer HSV (0-255) to RGB ColorRGB
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

// Sweep on shunt pull
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

// Single White Pulse on Shunt Release (Spin Finish)
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

// Quick transition fade when lifting or replacing handset
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

// Color generator (Vibrant random colors)
ColorRGB getRandomVibrantColor() {
  uint8_t h = random(0, 256);
  uint8_t region = h / 43;
  uint8_t remainder = (h - (region * 43)) * 6;

  uint8_t p = 0;
  uint8_t q = 255 - remainder;
  uint8_t t = remainder;

  float r, g, b;
  switch (region) {
    case 0:  r = 255; g = t;   b = p;   break;
    case 1:  r = q;   g = 255; b = p;   break;
    case 2:  r = p;   g = 255; b = t;   break;
    case 3:  r = p;   g = q;   b = 255; break;
    case 4:  r = t;   g = p;   b = 255; break;
    default: r = 255; g = p;   b = q;   break;
  }

  return { (r * MAX_BRIGHT) / 255.0f, (g * MAX_BRIGHT) / 255.0f, (b * MAX_BRIGHT) / 255.0f };
}

// ==========================================
// PARTICLE SYSTEM ENGINE
// ==========================================
void spawnParticle(char digitChar, ColorRGB forcedColor = { -1.0f, -1.0f, -1.0f }) {
  for (int i = 0; i < MAX_PARTICLES; i++) {
    if (!particles[i].active) {
      particles[i].active = true;
      particles[i].phase = PHASE_SPARKLE;
      particles[i].digitChar = digitChar;
      
      if (forcedColor.r >= 0.0f) {
        particles[i].color = forcedColor;
      } else {
        particles[i].color = getRandomVibrantColor();
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

      if (DEBUG && forcedColor.r < 0.0f) {
        Serial.printf("Spawned [%c] @ Slot %d | Speed Rating: %d/10 | Lifetime: %lu ms | Direction: %s\n",
                      digitChar, i, speedRating, particles[i].lifetimeMs, (particles[i].direction > 0 ? "CW" : "CCW"));
      }
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

// Standard multi-digit playback sequence
void playbackSequence(String sequence) {
  if (sequence.length() == 0) return;

  if (DEBUG) Serial.println("\n--- Launching Randomized Particle Engine (1s delay) ---");

  for (int i = 0; i < MAX_PARTICLES; i++) particles[i].active = false;

  for (size_t i = 0; i < sequence.length(); i++) {
    if (digitalRead(HOOK_PIN) == HIGH) break;

    spawnParticle(sequence.charAt(i));

    unsigned long delayStart = millis();
    while (millis() - delayStart < DIGIT_SPAWN_GAP_MS) {
      if (digitalRead(HOOK_PIN) == HIGH) break;
      updateAndRenderParticles();
      vTaskDelay(pdMS_TO_TICKS(15));
    }
  }

  while (hasActiveParticles()) {
    if (digitalRead(HOOK_PIN) == HIGH) break;
    updateAndRenderParticles();
    vTaskDelay(pdMS_TO_TICKS(15));
  }

  clearCanvas();

  if (DEBUG) Serial.println("--- Particles finished. 1s Pause ---");
  unsigned long pauseStart = millis();
  while (millis() - pauseStart < 1000) {
    if (digitalRead(HOOK_PIN) == HIGH) break;
    vTaskDelay(pdMS_TO_TICKS(15));
  }

  bool hookOff = (digitalRead(HOOK_PIN) == LOW);
  if (DEBUG) {
    Serial.printf("--- Fading into %s state ---\n", hookOff ? "Green (Off-Hook)" : "Yellow (On-Hook)");
  }

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

  if (DEBUG) Serial.println("--- Playback Sequence Complete ---\n");
}

// ==========================================
// RED ALERT SWARM ENGINE (911 EASTER EGG)
// ==========================================
void runRedAlertSwarm() {
  if (DEBUG) Serial.println("\n[RED ALERT] Launching 16-Pixel Red Particle Swarm...");

  for (int i = 0; i < MAX_PARTICLES; i++) particles[i].active = false;
  clearCanvas();

  ColorRGB pureRed = { (float)MAX_BRIGHT, 0.0f, 0.0f };

  for (int p = 0; p < 16; p++) {
    spawnParticle('!', pureRed);
  }

  while (digitalRead(HOOK_PIN) == LOW) {
    while (countActiveParticles() < 16) {
      spawnParticle('!', pureRed);
    }

    updateAndRenderParticles();
    vTaskDelay(pdMS_TO_TICKS(15));
  }

  if (DEBUG) Serial.println("[RED ALERT] Handset replaced. Ending Swarm.");
}

// ==========================================
// OPTION 1: HEARTBEAT HSL WALK ENGINE
// ==========================================
void runHeartbeatHslWalk() {
  if (DEBUG) Serial.println("\n[EASTER EGG 0] Option 1: Heartbeat HSL Walk Selected");

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

// ==========================================
// OPTION 2: NIGHT SKY STAR TWINKLE ENGINE
// ==========================================
void runNightSkyTwinkle() {
  if (DEBUG) Serial.println("\n[EASTER EGG 0] Option 2: Night Sky Star Twinkle Selected");

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

// Master router for 0 easter egg
void runZeroEasterEgg() {
  int choice = random(0, 2);
  if (choice == 0) {
    runHeartbeatHslWalk();
  } else {
    runNightSkyTwinkle();
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(DIAL_PIN, INPUT_PULLUP);
  pinMode(HOOK_PIN, INPUT_PULLUP);
  pinMode(SHUNT_PIN, INPUT_PULLUP);

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
    setRingColorRGB(0, MAX_BRIGHT, 0);
  } else {
    currentSystemState = STATE_IDLE_PULSE;
  }

  Serial.println("\n[SYSTEM] Rotary Controller Ready (Pure Pulse & LED Logic).");
}

void loop() {
  unsigned long now = millis();

  // 1. READ HARDWARE INPUTS
  int rawState = digitalRead(DIAL_PIN);
  bool hookOffCradle = (digitalRead(HOOK_PIN) == LOW);
  bool shuntFlipped  = (digitalRead(SHUNT_PIN) == LOW);

  // 2. HOOK STATE MONITORING WITH RESET LOGIC
  if (hookOffCradle != lastHookState) {
    vTaskDelay(pdMS_TO_TICKS(20));
    if (hookOffCradle) {
      if (DEBUG) Serial.println("\n[HOOK] Handset LIFTED -> Fade to Green");
      runHookStateFade(0, MAX_BRIGHT, 0, 250, true);
      currentSystemState = STATE_OFF_HOOK;
    } else {
      if (DEBUG) Serial.println("\n[HOOK] Handset REPLACED -> Ambient Yellow");
      
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

  // 3. SHUNT MONITORING (Pull Sweep & Release Pulse)
  if (hookOffCradle && currentSystemState != STATE_PLAYBACK && currentSystemState != STATE_RED_ALERT && currentSystemState != STATE_EASTER_EGG_ZERO) {
    if (shuntFlipped && !lastShuntState) {
      if (DEBUG) Serial.println("[SHUNT] Wheel pulled -> Trigger CCW Sweep");
      runCounterClockwiseSweep(0, 0, MAX_BRIGHT, 6);
      currentSystemState = STATE_DIALING_SOLID;
    }
    else if (!shuntFlipped && lastShuntState) {
      if (DEBUG) Serial.println("[SHUNT] Wheel returned -> Single White Pulse");
      runShuntReleasePulse();
    }
  }
  lastShuntState = shuntFlipped;

  // 4. CORE PULSE COUNTING LOGIC
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

        if (hookOffCradle && currentSystemState != STATE_DIALING_SWEEP && currentSystemState != STATE_RED_ALERT && currentSystemState != STATE_EASTER_EGG_ZERO) {
          currentSystemState = STATE_DIALING_SOLID;
        }

        if (DEBUG) {
          Serial.print("  -> pulse counted, running total for this digit: ");
          Serial.println(pulseCount);
        }
      }
    }
  }

  // 5. DIGIT & NUMBER TIMEOUTS WITH EASTER EGG ROUTING
  if (dialActive && (now - lastPulseMillis) >= DIGIT_GAP_MS) {
    int digit = (pulseCount == 10) ? 0 : pulseCount;
    if (pulseCount > 0) {
      if (DEBUG) {
        Serial.print("  -> digit finalized: ");
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

    // --- EASTER EGG ROUTING ---
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

  // 6. LIGHT ENGINE
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
        setRingColorRGB(0, 0, 0); // Fade to black
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