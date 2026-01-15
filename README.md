# easyPID

A feature-rich PID controller library for Arduino with multi-instance support, flexible timing, anti-windup, derivative filtering, and optional autotuning.

## Features

### Core Capabilities
- **Multi-instance friendly** - No global state, create unlimited independent PID controllers
- **Flexible timing modes** - Automatic timing via `millis()` OR manual time-delta input
- **Hardware-agnostic** - No dependencies on specific sensors or actuators
- **AVR-optimized** - Efficient for Arduino Uno and similar boards
- **Proven math** - Based on working light-tracking robot implementation

### Advanced Features
- **Anti-windup protection** - Multiple modes (NONE, CLAMP, BACKCALC) to prevent integral saturation
- **Derivative filtering** - Reduce noise sensitivity with selectable filter types (NONE, EMA)
- **Full introspection** - Access error, P term, I term, D term for debugging and tuning
- **Runtime tuning** - Change PID gains on-the-fly without restarting
- **Output limiting** - Configurable min/max output bounds with separate integral limits
- **Control direction** - DIRECT or REVERSE action support

### Optional Add-on: Autotuner
- **Relay/limit-cycle method** - Automatic PID parameter discovery
- **Multiple tuning rules** - Ziegler-Nichols, Tyreus-Luyben, Pessen, No-Overshoot
- **Separate module** - Include `<PIDTuner.h>` only when needed
- **Safe oscillation detection** - Configurable amplitude and noise band

## Installation

### Arduino IDE
1. Download the latest release as a ZIP file
2. Open Arduino IDE
3. Go to **Sketch → Include Library → Add .ZIP Library**
4. Select the downloaded ZIP file
5. Restart Arduino IDE

### Arduino CLI
```bash
arduino-cli lib install easyPID
```

### Manual Installation
1. Download or clone this repository
2. Copy the `easyPID` folder to your Arduino `libraries` directory
   - Windows: `Documents\Arduino\libraries\`
   - macOS: `~/Documents/Arduino/libraries/`
   - Linux: `~/Arduino/libraries/`
3. Restart Arduino IDE

## Quick Start

### Basic Usage

```cpp
#include <PIDController.h>

// Create PID controller: Kp, Ki, Kd, OutputMin, OutputMax
PIDController pid(2.0, 0.5, 0.1, 0, 255);

void setup() {
  pid.begin();  // Initialize timing and state
}

void loop() {
  float setpoint = 100.0;
  float measurement = readSensor();
  
  // Update PID (automatic timing)
  float output = pid.update(setpoint, measurement);
  
  applyOutput(output);
  delay(100);
}
```

### With Autotuning

```cpp
#include <PIDController.h>
#include <PIDTuner.h>  // Optional add-on

PIDController pid(1.0, 0.0, 0.0, 0, 255);
PIDTuner tuner(pid);

void setup() {
  pid.begin();
  tuner.start(setpoint, relayAmplitude);  // Start autotuning
}

void loop() {
  float measurement = readSensor();
  
  if (!tuner.isComplete()) {
    // Apply autotuner output during tuning
    float output = tuner.update(measurement);
    applyOutput(output);
  } else {
    // Get tuned parameters and apply them
    float kp, ki, kd;
    tuner.getTunings(kp, ki, kd, TUNING_ZIEGLER_NICHOLS);
    pid.setTunings(kp, ki, kd);
    
    // Run normal PID control
    float output = pid.update(setpoint, measurement);
    applyOutput(output);
  }
}
```

## API Reference

### PIDController Class

#### Constructor
```cpp
PIDController(float kp, float ki, float kd, float outMin, float outMax)
```

#### Initialization
```cpp
void begin()  // Call in setup() before first update
```

#### Update Methods
```cpp
// Automatic timing (uses millis() internally)
float update(float setpoint, float measurement)

// Manual timing (provide time delta in milliseconds)
float update(float setpoint, float measurement, float dtMs)

// Alternative pattern
void setSetpoint(float setpoint)
void setMeasurement(float measurement)
float compute()
```

#### Configuration
```cpp
void setTunings(float kp, float ki, float kd)
void setOutputLimits(float min, float max)
void setIntegralLimits(float min, float max)
void setAntiWindup(AntiWindupMode mode)
void setDerivativeFilter(DerivativeFilterMode mode, float alpha = 0.8)
void setSampleTime(unsigned long ms)
void setDirection(ControlDirection dir)
```

#### State Management
```cpp
void reset()  // Clear integral, derivative, and error history
```

#### Introspection
```cpp
float getError()   // Current error (setpoint - measurement)
float getPterm()   // Proportional term contribution
float getIterm()   // Integral term contribution
float getDterm()   // Derivative term contribution
float getOutput()  // Last computed output
```

### PIDTuner Class (Optional)

#### Constructor
```cpp
PIDTuner(PIDController& pid)
```

#### Tuning Control
```cpp
bool start(float setpoint, float relayAmplitude, float noiseBand = 0.5)
float update(float measurement)
bool isComplete()
void cancel()
```

#### Results
```cpp
bool getTunings(float& kp, float& ki, float& kd, TuningRule rule)
float getUltimateGain()
float getUltimatePeriod()
float getProgress()  // 0.0 to 1.0
```

#### Tuning Rules
- `TUNING_ZIEGLER_NICHOLS` - Classic, aggressive response
- `TUNING_TYREUS_LUYBEN` - Less overshoot, better for lag-dominant processes
- `TUNING_PESSEN` - Fast response, moderate overshoot
- `TUNING_NO_OVERSHOOT` - Conservative, minimal overshoot

## Examples

The library includes three complete examples:

1. **BasicPID** - Single PID controller with simulated first-order process
2. **MultiLoopPID** - Two independent controllers running concurrently
3. **AutoTunePID** - Automatic tuning demonstration with multiple tuning rules

Load examples from Arduino IDE: **File → Examples → easyPID**

## How easyPID is Different

| Feature | easyPID | Typical PID Libraries |
|---------|---------|----------------------|
| Multi-instance | ✅ Unlimited | ❌ Often uses global state |
| Timing flexibility | ✅ Auto + Manual | ⚠️ Usually auto only |
| Anti-windup | ✅ 3 modes | ⚠️ Basic or none |
| Derivative filtering | ✅ Built-in | ❌ Rare |
| Introspection | ✅ Full state access | ⚠️ Limited |
| Autotuner | ✅ Optional add-on | ❌ Separate library |
| Proven algorithm | ✅ Field-tested | ⚠️ Varies |

## Tuning Guide

See [docs/tuning_guide.md](docs/tuning_guide.md) for comprehensive tuning advice, including:
- What Kp, Ki, Kd do and how to tune them
- When to use anti-windup and derivative filtering
- Step-by-step tuning procedures
- Common pitfalls and solutions

## Applications

- Temperature control (heating/cooling systems)
- Motor speed control
- Position control (servos, steppers)
- Dual-axis systems (pan-tilt, X-Y stages)
- Multi-zone control (multiple heaters, motors, etc.)
- Robotics (line following, balancing, tracking)
- Process control (flow, pressure, level)

## Requirements

- Arduino IDE 1.6.x or higher
- Arduino boards: Uno, Nano, Mega, Due, Zero, ESP8266, ESP32, and compatible
- Tested on AVR architecture, compatible with most Arduino platforms

## License

MIT License - see [LICENSE](LICENSE) file for details

## Author

Rami Kronbi - ramykronby@gmail.com

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## Version History

See [CHANGELOG.md](CHANGELOG.md) for version history and release notes.

## Acknowledgments

- Based on proven PID implementation from light-tracking robot project
- Autotuner inspired by Åström-Hägglund relay method
- Tuning rules from classical control theory literature
