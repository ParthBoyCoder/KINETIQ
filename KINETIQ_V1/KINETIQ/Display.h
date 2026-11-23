#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Config.h"
#include "Physics.h"

namespace UI {

Adafruit_SSD1306 oled(HW::OLED_WIDTH, HW::OLED_HEIGHT, &Wire, HW::OLED_RESET);

inline bool begin() {
  if (!oled.begin(SSD1306_SWITCHCAPVCC, HW::OLED_ADDRESS)) return false;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println(F("KINETIQ"));
  oled.println(F("Experimental Mechanics"));
  oled.println(F("Starting..."));
  oled.display();
  return true;
}

inline void header(const __FlashStringHelper *mode) {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println(F("KINETIQ  |  V1"));
  oled.println(mode);
}

inline void home() {
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println(F("KINETIQ"));
  oled.println(F("Experimental Mechanics"));
  oled.println();
  oled.println(F("Serial commands:"));
  oled.println(F("F = Friction mode"));
  oled.println(F("C = Collision mode"));
  oled.println(F("N = New trial"));
  oled.display();
}

inline void waitingFriction(uint8_t countA, uint8_t countB) {
  header(F("FRICTION ANALYZER"));
  oled.println(F("Pass block A -> B"));
  oled.print(F("Gate A: ")); oled.print(countA); oled.println(F("/1"));
  oled.print(F("Gate B: ")); oled.print(countB); oled.println(F("/1"));
  oled.println();
  oled.println(F("N = reset trial"));
  oled.display();
}

inline void waitingCollision(uint8_t countA, uint8_t countB) {
  header(F("COLLISION ANALYZER"));
  oled.println(F("Two blocks approach"));
  oled.println(F("and return through"));
  oled.println(F("their original gates."));
  oled.print(F("Gate A: ")); oled.print(countA); oled.println(F("/2"));
  oled.print(F("Gate B: ")); oled.print(countB); oled.println(F("/2"));
  oled.println(F("N = reset trial"));
  oled.display();
}

inline void frictionResult(const Physics::FrictionResult &r) {
  header(F("FRICTION RESULTS"));
  if (!r.valid) {
    oled.println(F("Invalid measurement"));
  } else {
    oled.print(F("vA: ")); oled.print(r.velocityA, 3); oled.println(F(" m/s"));
    oled.print(F("vB: ")); oled.print(r.velocityB, 3); oled.println(F(" m/s"));
    oled.print(F("a : ")); oled.print(r.acceleration, 3); oled.println(F(" m/s2"));
    oled.print(F("mu: ")); oled.println(r.muK, 4);
  }
  oled.display();
}

inline void collisionResult(const Physics::CollisionResult &r) {
  header(F("COLLISION RESULTS"));
  if (!r.valid) {
    oled.println(F("Invalid measurement"));
  } else {
    oled.print(F("Pi: ")); oled.println(r.initialMomentum, 4);
    oled.print(F("Pf: ")); oled.println(r.finalMomentum, 4);
    oled.print(F("dP: ")); oled.println(r.momentumDifference, 4);
    oled.print(F("KEi: ")); oled.println(r.initialKE, 4);
    oled.print(F("KEf: ")); oled.println(r.finalKE, 4);
    oled.print(F("e: ")); oled.println(r.restitution, 3);
  }
  oled.display();
}

inline void error(const __FlashStringHelper *message) {
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println(F("KINETIQ ERROR"));
  oled.println(message);
  oled.display();
}

} // namespace UI
