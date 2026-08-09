# Changelog

All notable changes to the easyPID library will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.2] - 2026-08-09

### Fixed
- Relay edge detection no longer uses a function-local `static` inside
  `PIDTuner::detectPeak()`. That variable was shared by every `PIDTuner`
  instance in the sketch and was initialised only once for the lifetime of the
  program, so a second tuner (or a second `start()` on the same tuner) saw
  corrupted edge state. It is now a per-instance member.
- Initialise every `PIDTuner` member in the constructor, so calling `update()`
  before `start()` can no longer read indeterminate values.

---

## [1.0.1] - 2026-08-09

### Fixed
- Stop redefining Arduino's `PI` macro in `PIDTuner.cpp`, which emitted a
  "PI redefined" warning on every compilation (reproduced on AVR and ESP32).
  The tuner now uses a private float constant instead.

---

## [1.0.0] - 2024-01-15

### Added
- **Core PIDController class** with full PID control functionality
  - Proportional, Integral, and Derivative control terms
  - dt-aware integral accumulation and derivative calculation
  - Support for automatic timing (millis-based) and manual dt input
  - Dual update patterns: `update(setpoint, measurement)` and `update(setpoint, measurement, dt)`
  - Alternative pattern: `setSetpoint()`, `setMeasurement()`, `compute()`
  
- **Anti-windup protection** with multiple modes
  - NONE: No anti-windup (for testing)
  - CLAMP: Prevent integral accumulation during saturation
  - BACKCALC: Back-calculation method for advanced applications
  
- **Derivative filtering** to reduce noise sensitivity
  - NONE: Raw derivative (no filtering)
  - EMA: Exponential Moving Average (1st-order low-pass filter)
  - Configurable filter coefficient (alpha parameter)
  
- **Full state introspection** for debugging and monitoring
  - `getError()`: Current control error
  - `getPterm()`: Proportional term contribution
  - `getIterm()`: Integral term contribution
  - `getDterm()`: Derivative term contribution
  - `getOutput()`: Last computed output value
  
- **Runtime configuration** methods
  - `setTunings()`: Update PID gains on-the-fly
  - `setOutputLimits()`: Configure output saturation bounds
  - `setIntegralLimits()`: Set separate integral term limits
  - `setSampleTime()`: Document expected sample period
  - `setDirection()`: DIRECT or REVERSE control action
  - `reset()`: Clear all state (integral, derivative, errors)
  
- **Optional PIDTuner add-on module** for automatic tuning
  - Relay/limit-cycle autotuning method
  - Oscillation detection and parameter extraction
  - Ultimate gain (Ku) and period (Pu) calculation
  - Four tuning rule options:
    - Ziegler-Nichols (classic, aggressive)
    - Tyreus-Luyben (less overshoot)
    - Pessen Integral Rule (fast response)
    - No Overshoot (conservative)
  - Progress monitoring and timeout protection
  - Safe oscillation amplitude and noise band configuration
  
- **Three comprehensive examples**
  - BasicPID: Single controller with simulated first-order process
  - MultiLoopPID: Two independent controllers demonstrating multi-instance capability
  - AutoTunePID: Complete autotuning workflow with multiple tuning rules
  
- **Complete documentation**
  - README.md with quick start, API reference, and feature comparison
  - docs/tuning_guide.md with practical tuning procedures and troubleshooting
  - Inline code documentation with Doxygen-style comments
  
- **Arduino library metadata**
  - library.properties for Arduino Library Manager compatibility
  - keywords.txt for Arduino IDE syntax highlighting
  - MIT License (LICENSE file)
  - Arduino Library Specification 1.5 compliant structure

### Technical Details
- **Multi-instance safe**: No global mutable state, create unlimited controllers
- **Memory efficient**: Uses float (not double) for AVR compatibility
- **Proven algorithm**: Based on field-tested light-tracking robot implementation
- **Hardware agnostic**: No dependencies on specific sensors or actuators
- **AVR optimized**: Tested on Arduino Uno (ATmega328P)
- **Portable**: Compatible with most Arduino architectures (AVR, ARM, ESP8266, ESP32)

### Dependencies
- Arduino Core library (included with Arduino IDE)
- No external dependencies required

### Tested Platforms
- Arduino Uno (ATmega328P)
- Arduino Nano
- Arduino Mega 2560
- Compatible with most Arduino boards

### Known Limitations
- Sample time consistency is user's responsibility (library doesn't enforce it in auto-timing mode)
- Autotuner requires stable process (no major disturbances during tuning)
- Derivative filtering limited to 1st-order (EMA) in v1.0.0

### Migration Notes
This is the initial release. No migration necessary.

---

## Future Roadmap (Not in v1.0.0)

Potential features for future versions:
- 2nd-order derivative filtering
- Derivative-on-measurement (to avoid derivative kick)
- Setpoint weighting/ramping
- Bumpless transfer support
- Additional autotuning algorithms
- EEPROM parameter persistence

---

## Version Format

Version numbers follow [Semantic Versioning](https://semver.org/):
- **MAJOR**: Incompatible API changes
- **MINOR**: New functionality (backward-compatible)
- **PATCH**: Bug fixes (backward-compatible)

Example: v1.2.3
- 1 = Major version
- 2 = Minor version  
- 3 = Patch version
