#pragma once
#include <Arduino.h>
#include "Config.h"

namespace Sensors {

struct Event {
  uint32_t blockedUs;
  uint32_t endTimeUs;
};

volatile uint32_t queueDuration[2][SENSOR::QUEUE_SIZE];
volatile uint32_t queueEndTime[2][SENSOR::QUEUE_SIZE];
volatile uint8_t queueHead[2] = {0, 0};
volatile uint8_t queueTail[2] = {0, 0};
volatile uint16_t queueOverflows[2] = {0, 0};

volatile bool beamBlocked[2] = {false, false};
volatile uint32_t beamStartUs[2] = {0, 0};

inline void pushFromISR(uint8_t sensor, uint32_t duration, uint32_t endTime) {
  const uint8_t head = queueHead[sensor];
  const uint8_t next = (head + 1) % SENSOR::QUEUE_SIZE;

  if (next == queueTail[sensor]) {
    queueOverflows[sensor]++;
    return;
  }

  queueDuration[sensor][head] = duration;
  queueEndTime[sensor][head] = endTime;
  queueHead[sensor] = next;
}

inline void handleChange(uint8_t sensor, uint8_t pin) {
  const uint32_t now = micros();
  const bool active = (digitalRead(pin) == SENSOR::ACTIVE_LEVEL);

  if (active && !beamBlocked[sensor]) {
    beamBlocked[sensor] = true;
    beamStartUs[sensor] = now;
  } else if (!active && beamBlocked[sensor]) {
    beamBlocked[sensor] = false;
    const uint32_t duration = now - beamStartUs[sensor];

    if (duration >= SENSOR::MIN_BLOCK_US) {
      pushFromISR(sensor, duration, now);
    }
  }
}

inline void isrA() { handleChange(0, HW::SENSOR_A_PIN); }
inline void isrB() { handleChange(1, HW::SENSOR_B_PIN); }

inline void begin() {
  pinMode(HW::SENSOR_A_PIN, INPUT_PULLUP);
  pinMode(HW::SENSOR_B_PIN, INPUT_PULLUP);

  beamBlocked[0] = (digitalRead(HW::SENSOR_A_PIN) == SENSOR::ACTIVE_LEVEL);
  beamBlocked[1] = (digitalRead(HW::SENSOR_B_PIN) == SENSOR::ACTIVE_LEVEL);
  beamStartUs[0] = micros();
  beamStartUs[1] = micros();

  attachInterrupt(digitalPinToInterrupt(HW::SENSOR_A_PIN), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HW::SENSOR_B_PIN), isrB, CHANGE);
}

inline bool pop(uint8_t sensor, Event &event) {
  bool available = false;

  noInterrupts();
  const uint8_t tail = queueTail[sensor];
  if (tail != queueHead[sensor]) {
    event.blockedUs = queueDuration[sensor][tail];
    event.endTimeUs = queueEndTime[sensor][tail];
    queueTail[sensor] = (tail + 1) % SENSOR::QUEUE_SIZE;
    available = true;
  }
  interrupts();

  return available;
}

inline void clear() {
  noInterrupts();

  for (uint8_t s = 0; s < 2; s++) {
    queueHead[s] = 0;
    queueTail[s] = 0;
    queueOverflows[s] = 0;
    beamBlocked[s] = (digitalRead(s == 0 ? HW::SENSOR_A_PIN : HW::SENSOR_B_PIN)
                      == SENSOR::ACTIVE_LEVEL);
    beamStartUs[s] = micros();
  }

  interrupts();
}

inline uint16_t overflowCount(uint8_t sensor) {
  uint16_t count;
  noInterrupts();
  count = queueOverflows[sensor];
  interrupts();
  return count;
}

} // namespace Sensors
