const int pulsePin = 2; // Blue wire (Pulse switch) -> GND on Green
const int shuntPin = 4; // White wire (Shunt switch) -> GND on Red

// Pulse tracking & State Machine
int pulseCount = 0;
bool isDialing = false;

enum PulseState { IDLE_CLOSED, VALIDATING_HIGH, WAITING_FOR_LOW, VALIDATING_LOW };
PulseState currentState = IDLE_CLOSED;

unsigned long stateTimer = 0;
const unsigned long minPulseTime = 12; // ms stability check

// Phone Number Buffer & Sequence Tracking
String dialedNumberBuffer = "";
unsigned long lastDigitTime = 0;
const unsigned long interDigitTimeout = 2500; // Time (ms) to wait after last digit before outputting full number
bool sequenceInProgress = false;

void setup() {
  Serial.begin(9600);

  pinMode(pulsePin, INPUT_PULLUP);
  pinMode(shuntPin, INPUT_PULLUP);
}

void loop() {
  bool dialIsSpinning = (digitalRead(shuntPin) == LOW);
  bool pulsePinState = digitalRead(pulsePin); // HIGH = Open (Pulse break), LOW = Closed

  // 1. START OF A SINGLE DIGIT DIAL TURN
  if (dialIsSpinning && !isDialing) {
    isDialing = true;
    pulseCount = 0;
    currentState = IDLE_CLOSED;
  }

  // 2. PROCESS PULSES DURING DIAL TURN
  if (isDialing) {
    switch (currentState) {
      case IDLE_CLOSED:
        if (pulsePinState == HIGH) {
          stateTimer = millis();
          currentState = VALIDATING_HIGH;
        }
        break;

      case VALIDATING_HIGH:
        if (pulsePinState == HIGH) {
          if (millis() - stateTimer >= minPulseTime) {
            currentState = WAITING_FOR_LOW;
          }
        } else {
          currentState = IDLE_CLOSED;
        }
        break;

      case WAITING_FOR_LOW:
        if (pulsePinState == LOW) {
          stateTimer = millis();
          currentState = VALIDATING_LOW;
        }
        break;

      case VALIDATING_LOW:
        if (pulsePinState == LOW) {
          if (millis() - stateTimer >= minPulseTime) {
            pulseCount++;
            currentState = IDLE_CLOSED;
          }
        } else {
          currentState = WAITING_FOR_LOW;
        }
        break;
    }
  }

  // 3. END OF A SINGLE DIGIT DIAL TURN (Wheel hits rest)
  if (!dialIsSpinning && isDialing) {
    if (pulseCount > 0) {
      int digit = pulseCount;
      if (digit == 10) digit = 0;

      // Append digit to the running buffer
      dialedNumberBuffer += String(digit);
      
      // Update timers and flags
      lastDigitTime = millis();
      sequenceInProgress = true;
    }

    pulseCount = 0;
    isDialing = false;
    currentState = IDLE_CLOSED;
  }

  // 4. SEQUENCE TIMEOUT (Pause detected: print complete dialed sequence)
  if (sequenceInProgress && !isDialing && (millis() - lastDigitTime > interDigitTimeout)) {
    
    // Output full accumulated number
    Serial.print("Dialed Sequence: ");
    Serial.println(dialedNumberBuffer);

    // Reset buffer for the next sequence
    dialedNumberBuffer = "";
    sequenceInProgress = false;
  }
}