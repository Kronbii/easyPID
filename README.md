# easyPID

A flexible, hardware‑agnostic PID controller library for Arduino with multi‑instance support, anti‑windup protection, derivative filtering, and an optional autotuning add‑on.

---

## Quick TL;DR

```cpp
#include <easyPID.h>

PIDController pid(2.0, 0.5, 0.1, 0, 255);

void setup() {
  pid.begin();
}

void loop() {
  float output = pid.update(setpoint, measurement);
  applyOutput(output);
}
```

---

## Why easyPID?

`easyPID` is designed for real‑world embedded control applications where multiple independent control loops, timing flexibility, and robust behavior are required.

* Multiple independent PID instances (no global state)
* Flexible timing: automatic `millis()`‑based updates or user‑supplied `dt`
* Built‑in anti‑windup protection
* Optional derivative filtering for noisy signals
* Full internal state introspection for debugging and tuning
* Optional relay‑based autotuner (opt‑in)
* Lightweight and AVR‑friendly

---

## Features

### Core Capabilities

* **Multi‑instance friendly** – Create multiple independent PID controllers
* **Flexible timing modes** – Automatic timing via `millis()` or manual time‑delta input
* **Hardware‑agnostic** – No assumptions about sensors, actuators, or pins
* **AVR‑friendly** – Designed to run efficiently on Arduino Uno‑class boards
* **Practical implementation** – Derived from a real embedded control project and generalized for reuse

### Advanced Control Features

* **Anti‑windup protection** – Prevent integral saturation using selectable modes
* **Derivative filtering** – Reduce noise sensitivity using optional low‑pass filtering
* **Runtime tuning** – Adjust PID gains while the system is running
* **Output limiting** – Clamp controller output to safe bounds
* **Control direction** – Support for DIRECT and REVERSE acting systems
* **State introspection** – Access error, P/I/D terms, and last output for debugging

## Optional Add-On: Autotuner

The autotuner is provided as an optional module and is only included when explicitly requested:

```cpp
#include <PIDTuner.h>
```

---

## Autotuning Method

The autotuner implements a **relay / limit-cycle method (Åström–Hägglund)**:

* Applies bang-bang (relay) control around the setpoint
* Induces sustained oscillations in the system
* Measures oscillation amplitude and period
* Computes:

  * Ultimate gain (**Ku**)
  * Ultimate period (**Pu**)
* Applies classical tuning rules to compute PID gains

---

## Supported Tuning Rules

* **Ziegler–Nichols** – Classic, aggressive response
* **Tyreus–Luyben** – Reduced overshoot, better for lag-dominant systems
* **Pessen Integral Rule** – Faster response with moderate overshoot
* **No Overshoot** – Conservative tuning for sensitive systems

---

## Autotuner Usage Example

```cpp
#include <easyPID.h>
#include <PIDTuner.h>

PIDController pid(1.0, 0.0, 0.0, 0, 255);
PIDTuner tuner(pid);

void setup() {
  pid.begin();
  tuner.start(setpoint, relayAmplitude);  // noiseBand is optional
}

void loop() {
  if (!tuner.isComplete()) {
    // Apply relay output during tuning
    float output = tuner.update(measurement);
    applyOutput(output);
  } else {
    float kp, ki, kd;
    tuner.getTunings(kp, ki, kd, TUNING_ZIEGLER_NICHOLS);
    pid.setTunings(kp, ki, kd);

    // Run normal PID control
    float output = pid.update(setpoint, measurement);
    applyOutput(output);
  }
}
```

---

## ⚠️ Warning

The autotuner intentionally induces oscillations around the setpoint.

Ensure your system can safely tolerate these oscillations before enabling autotuning.

---

## Installation

### Arduino Library Manager (recommended)

Once published, install via:

> **Arduino IDE → Library Manager → Search for “easyPID”**

### Arduino CLI

```bash
arduino-cli lib install easyPID
```

### Manual Installation

1. Download or clone this repository
2. Copy the `easyPID` folder into your Arduino `libraries` directory:

   * Windows: `Documents/Arduino/libraries/`
   * macOS: `~/Documents/Arduino/libraries/`
   * Linux: `~/Arduino/libraries/`
3. Restart the Arduino IDE

---

## Basic Usage

```cpp
#include <easyPID.h>

PIDController pid(2.0, 0.5, 0.1, 0, 255);

void setup() {
  pid.begin();
}

void loop() {
  float output = pid.update(setpoint, measurement);
  applyOutput(output);
}
```

* `setpoint` is the desired target value
* `measurement` is the current process value
* `output` is the computed control signal

---

## Using the Autotuner (Optional)

```cpp
#include <easyPID.h>
#include <PIDTuner.h>

PIDController pid(1.0, 0.0, 0.0, 0, 255);
PIDTuner tuner(pid);

void setup() {
  pid.begin();
  tuner.start(setpoint, relayAmplitude);
}

void loop() {
  if (!tuner.isComplete()) {
    float output = tuner.update(measurement);
    applyOutput(output);
  } else {
    float kp, ki, kd;
    tuner.getTunings(kp, ki, kd);
    pid.setTunings(kp, ki, kd);

    float output = pid.update(setpoint, measurement);
    applyOutput(output);
  }
}
```

The autotuner is intended as a starting point for tuning and may require refinement depending on the system dynamics.

---

## Examples

The library includes the following examples:

1. **BasicPID** – Single PID controller with a simulated first‑order process
2. **MultiLoopPID** – Two independent PID loops running concurrently
3. **AutoTunePID** – Demonstration of the optional autotuning workflow

Access them via:

> **Arduino IDE → File → Examples → easyPID**

---

## How easyPID Is Different

| Feature              | easyPID           | Typical Arduino PID Libraries |
| -------------------- | ----------------- | ----------------------------- |
| Multiple instances   | ✅ Yes             | ⚠️ Often limited              |
| Timing flexibility   | ✅ Auto + manual   | ⚠️ Usually fixed              |
| Anti‑windup          | ✅ Built‑in        | ❌ Often missing               |
| Derivative filtering | ✅ Optional        | ❌ Rare                        |
| State introspection  | ✅ Full access     | ⚠️ Limited                    |
| Autotuning           | ✅ Optional add‑on | ❌ Separate library            |

---

## Tuning Guide

A practical tuning guide is available at:

* `docs/tuning_guide.md`

It covers:

* Understanding Kp, Ki, and Kd
* When to use anti‑windup and filtering
* Step‑by‑step tuning strategies
* Common pitfalls in embedded PID control

---

## Applications

`easyPID` can be used for a wide range of control problems, including:

* Temperature regulation
* Motor speed control
* Position control
* Pan‑tilt and X‑Y systems
* Multi‑zone control setups
* Robotics and automation projects

---

## Requirements

* Arduino IDE 1.6 or newer
* Tested on AVR‑based boards (Arduino Uno, Nano)
* Expected to work on most Arduino‑compatible platforms

---

## License

MIT License – see [LICENSE](LICENSE) for details.

---

## Author

Rami Kronbi

---

## Version History

See [CHANGELOG.md](CHANGELOG.md) for release notes and version history.

---

## Contributing

Contributions, bug reports, and suggestions are welcome via GitHub issues and pull requests.