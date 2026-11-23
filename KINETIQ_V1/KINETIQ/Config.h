#pragma once
#include <Arduino.h>

// KINETIQ V1 — edit experimental values here.
// Units: metres, kilograms, seconds, degrees.

namespace HW {
  constexpr uint8_t SENSOR_A_PIN = 2;   // INT0
  constexpr uint8_t SENSOR_B_PIN = 3;   // INT1
  constexpr uint8_t OLED_WIDTH = 128;
  constexpr uint8_t OLED_HEIGHT = 64;
  constexpr uint8_t OLED_ADDRESS = 0x3C;
  constexpr int8_t OLED_RESET = -1;
  constexpr uint32_t SERIAL_BAUD = 115200;
}

namespace SENSOR {
  // Most common IR obstacle modules are active LOW.
  // Change to HIGH if your module behaves the opposite way.
  constexpr uint8_t ACTIVE_LEVEL = LOW;

  // Ignore very short pulses (noise). Tune after testing your sensor.
  constexpr uint32_t MIN_BLOCK_US = 300;

  // Each sensor has a ring queue of 8 slots (7 usable).
  constexpr uint8_t QUEUE_SIZE = 8;
}

namespace FRICTION {
  constexpr float G = 9.80665f;

  // PROVISIONAL VALUES — replace after the physical setup is finalised.
  constexpr float OBJECT_LENGTH_A_M = 0.050f;
  constexpr float OBJECT_LENGTH_B_M = 0.050f;
  constexpr float GATE_SPACING_M = 0.200f;  // distance between beam centres
  constexpr float ANGLE_DEG = 0.0f;         // 0 = horizontal; positive = downhill
}

namespace COLLISION {
  constexpr float G = 9.80665f;

  // PROVISIONAL VALUES — replace after measuring the two blocks.
  constexpr float BLOCK_1_LENGTH_M = 0.050f;
  constexpr float BLOCK_2_LENGTH_M = 0.050f;
  constexpr float BLOCK_1_MASS_KG = 0.100f;
  constexpr float BLOCK_2_MASS_KG = 0.100f;
}

namespace DISPLAY_CONFIG {
  constexpr uint32_t REFRESH_MS = 200;
}
