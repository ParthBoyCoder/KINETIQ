#include "Config.h"
#include "Sensors.h"
#include "Physics.h"
#include "Display.h"

enum Mode : uint8_t { MODE_HOME, MODE_FRICTION, MODE_COLLISION };
enum State : uint8_t { STATE_WAITING, STATE_RESULT };

Mode currentMode = MODE_HOME;
State currentState = STATE_WAITING;

uint32_t frictionTimeA = 0;
uint32_t frictionTimeB = 0;

uint32_t collisionA[2] = {0, 0};  // block 1: approach, return
uint32_t collisionB[2] = {0, 0};  // block 2: approach, return
uint8_t countA = 0;
uint8_t countB = 0;

uint32_t lastDisplayMs = 0;
bool oledReady = false;

void resetTrial() {
  Sensors::clear();

  frictionTimeA = 0;
  frictionTimeB = 0;

  collisionA[0] = collisionA[1] = 0;
  collisionB[0] = collisionB[1] = 0;

  countA = 0;
  countB = 0;
  currentState = STATE_WAITING;
}

void selectMode(Mode mode) {
  currentMode = mode;
  resetTrial();

  if (mode == MODE_FRICTION) {
    Serial.println(F("\n[KINETIQ] Friction mode selected."));
    Serial.println(F("Send one block through Gate A, then Gate B."));
  } else if (mode == MODE_COLLISION) {
    Serial.println(F("\n[KINETIQ] Collision mode selected."));
    Serial.println(F("Gate A: block 1 approach, then return."));
    Serial.println(F("Gate B: block 2 approach, then return."));
    Serial.println(F("Both blocks must cross their own gate twice."));
  }

  if (oledReady) UI::home();
}

void handleSerial() {
  while (Serial.available()) {
    const char command = toupper(Serial.read());

    if (command == 'F') {
      selectMode(MODE_FRICTION);
    } else if (command == 'C') {
      selectMode(MODE_COLLISION);
    } else if (command == 'N') {
      resetTrial();
      Serial.println(F("[KINETIQ] Trial reset. Ready."));
    } else if (command == 'H') {
      currentMode = MODE_HOME;
      resetTrial();
      if (oledReady) UI::home();
    }
  }
}

void collectEvents() {
  if (currentMode == MODE_HOME || currentState == STATE_RESULT) return;

  Sensors::Event event;

  if (currentMode == MODE_FRICTION) {
    if (countA == 0 && Sensors::pop(0, event)) {
      frictionTimeA = event.blockedUs;
      countA = 1;
      Serial.print(F("[A] block time us: "));
      Serial.println(frictionTimeA);
    }

    if (countB == 0 && Sensors::pop(1, event)) {
      frictionTimeB = event.blockedUs;
      countB = 1;
      Serial.print(F("[B] block time us: "));
      Serial.println(frictionTimeB);
    }

    if (countA == 1 && countB == 1) {
      const Physics::FrictionResult result =
        Physics::calculateFriction(frictionTimeA, frictionTimeB);

      Serial.println(F("\n--- FRICTION RESULTS ---"));
      if (result.valid) {
        Serial.print(F("vA (m/s): ")); Serial.println(result.velocityA, 5);
        Serial.print(F("vB (m/s): ")); Serial.println(result.velocityB, 5);
        Serial.print(F("a (m/s^2): ")); Serial.println(result.acceleration, 5);
        Serial.print(F("mu_k: ")); Serial.println(result.muK, 5);
      } else {
        Serial.println(F("Invalid result. Check configuration and sensor data."));
      }

      UI::frictionResult(result);
      currentState = STATE_RESULT;
    }
  }

  if (currentMode == MODE_COLLISION) {
    while (countA < 2 && Sensors::pop(0, event)) {
      collisionA[countA] = event.blockedUs;
      Serial.print(F("[A event "));
      Serial.print(countA + 1);
      Serial.print(F("] block time us: "));
      Serial.println(event.blockedUs);
      countA++;
    }

    while (countB < 2 && Sensors::pop(1, event)) {
      collisionB[countB] = event.blockedUs;
      Serial.print(F("[B event "));
      Serial.print(countB + 1);
      Serial.print(F("] block time us: "));
      Serial.println(event.blockedUs);
      countB++;
    }

    if (countA == 2 && countB == 2) {
      const Physics::CollisionResult result =
        Physics::calculateCollision(
          collisionA[0], collisionA[1],
          collisionB[0], collisionB[1]
        );

      Serial.println(F("\n--- COLLISION RESULTS ---"));
      if (result.valid) {
        Serial.print(F("u1 (m/s): ")); Serial.println(result.u1, 5);
        Serial.print(F("v1 (m/s): ")); Serial.println(result.v1, 5);
        Serial.print(F("u2 (m/s): ")); Serial.println(result.u2, 5);
        Serial.print(F("v2 (m/s): ")); Serial.println(result.v2, 5);
        Serial.print(F("Initial momentum: "));
        Serial.println(result.initialMomentum, 6);
        Serial.print(F("Final momentum: "));
        Serial.println(result.finalMomentum, 6);
        Serial.print(F("Momentum difference: "));
        Serial.println(result.momentumDifference, 6);
        Serial.print(F("Momentum error (%): "));
        Serial.println(result.momentumErrorPercent, 3);
        Serial.print(F("Initial KE (J): "));
        Serial.println(result.initialKE, 6);
        Serial.print(F("Final KE (J): "));
        Serial.println(result.finalKE, 6);
        Serial.print(F("Restitution: "));
        Serial.println(result.restitution, 4);
      } else {
        Serial.println(F("Invalid result. Check configuration and sensor data."));
      }

      UI::collisionResult(result);
      currentState = STATE_RESULT;
    }
  }
}

void updateDisplay() {
  if (!oledReady) return;

  const uint32_t now = millis();
  if (now - lastDisplayMs < DISPLAY_CONFIG::REFRESH_MS) return;
  lastDisplayMs = now;

  if (currentMode == MODE_HOME) {
    UI::home();
  } else if (currentState == STATE_WAITING) {
    if (currentMode == MODE_FRICTION) {
      UI::waitingFriction(countA, countB);
    } else if (currentMode == MODE_COLLISION) {
      UI::waitingCollision(countA, countB);
    }
  }
}

void reportOverflows() {
  static uint16_t lastA = 0;
  static uint16_t lastB = 0;

  const uint16_t a = Sensors::overflowCount(0);
  const uint16_t b = Sensors::overflowCount(1);

  if (a != lastA || b != lastB) {
    Serial.print(F("[WARNING] Sensor queue overflow. A="));
    Serial.print(a);
    Serial.print(F(" B="));
    Serial.println(b);
    lastA = a;
    lastB = b;
  }
}

void setup() {
  Serial.begin(HW::SERIAL_BAUD);
  Sensors::begin();

  oledReady = UI::begin();

  Serial.println(F("\n================================"));
  Serial.println(F(" KINETIQ — Experimental Mechanics"));
  Serial.println(F(" Arduino Nano / V1"));
  Serial.println(F("================================"));
  Serial.println(F("Commands:"));
  Serial.println(F(" F = Friction mode"));
  Serial.println(F(" C = Collision mode"));
  Serial.println(F(" N = New trial"));
  Serial.println(F(" H = Home"));
  Serial.println(F("--------------------------------"));

  if (!oledReady) {
    Serial.println(F("[WARNING] OLED not detected. Serial still works."));
  } else {
    UI::home();
  }
}

void loop() {
  handleSerial();
  collectEvents();
  updateDisplay();
  reportOverflows();
}
