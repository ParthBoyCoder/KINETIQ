# KINETIQ V1

Arduino Nano firmware for a two-mode experimental mechanics instrument.

## Hardware

- Arduino Nano (ATmega328P)
- IR sensor A output: D2
- IR sensor B output: D3
- SSD1306 128x64 I2C OLED, address 0x3C
- OLED SDA: A4
- OLED SCL: A5
- OLED reset: -1

Install these Arduino libraries:
- Adafruit GFX Library
- Adafruit SSD1306

## Upload

1. Put all five source files in a folder named `KINETIQ`.
2. Rename `KINETIQ.ino` to `KINETIQ.ino` (already named correctly).
3. Open it in Arduino IDE.
4. Select Arduino Nano / ATmega328P and the correct processor/bootloader option.
5. Compile and upload.

## Serial commands (115200 baud)

- `F`: Friction mode
- `C`: Collision mode
- `N`: Reset/start a new trial
- `H`: Return to home screen

There is no physical mode button in V1.

## Friction mode

The block crosses Gate A and then Gate B while travelling in the positive/downhill direction.

The firmware calculates:
- speed at A and B from effective optical length / beam-block time
- acceleration from the two speeds and gate spacing
- estimated kinetic friction coefficient

For a horizontal track, set `FRICTION::ANGLE_DEG = 0`.
For an incline, set it to the measured angle, with positive angle meaning Gate B is downhill from Gate A.

The formula assumes sliding (not rolling), approximately constant acceleration, and kinetic friction.

## Collision mode: required event sequence

This implementation assumes:
- Gate A is left of the collision zone.
- Gate B is right of the collision zone.
- Block 1 approaches the centre through A, collides, then returns through A.
- Block 2 approaches the centre through B, collides, then returns through B.
- Each gate therefore records two block passages.

Coordinate convention: rightward is positive.
- A first event = block 1 moving right (+)
- A second event = block 1 moving left (-)
- B first event = block 2 moving left (-)
- B second event = block 2 moving right (+)

If the actual physical sequence differs, the sign/event mapping in `Physics.h` must be changed before trusting the collision results.

## Calibration / limitations

The speed calculation is `effective optical length / blocked duration`.
For reflective IR obstacle modules, the effective optical length may not equal the physical block length. Calibrate it experimentally for each sensor and use the calibrated values in `Config.h`.

The sensor's active output is assumed LOW and `INPUT_PULLUP` is enabled. Confirm this with your exact sensor module before running experiments. If the output is active HIGH, change `SENSOR::ACTIVE_LEVEL` and use the appropriate electrical input configuration.

This is V1 acquisition and calculation firmware, not a claim of laboratory-grade accuracy. Validate repeated measurements against an independent timing/reference method before presenting results.
