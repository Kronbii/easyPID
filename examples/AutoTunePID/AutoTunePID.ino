/**
 * @file AutoTunePID.ino
 * @brief Automatic PID Tuning Example using Relay Method
 * @author Rami Kronbi
 * @date 2024
 * 
 * This example demonstrates:
 * - Automatic PID tuning using relay/limit-cycle method
 * - Multiple tuning rule options (Z-N, Tyreus-Luyben, Pessen, No-Overshoot)
 * - Before/after performance comparison
 * - Practical autotuning workflow
 * 
 * The autotuner works by:
 * 1. Applying relay (bang-bang) control around setpoint
 * 2. Measuring oscillation period and amplitude
 * 3. Calculating ultimate gain (Ku) and period (Pu)
 * 4. Applying tuning rules to get Kp, Ki, Kd
 * 
 * WARNING: Autotuning induces oscillations. Ensure your system
 *          can safely handle oscillations around the setpoint.
 * 
 * Hardware: Arduino Uno (or compatible)
 * No external hardware required - all simulation in software
 */

#include <easyPID.h>
#include <PIDTuner.h>  // Optional add-on module

// Process parameters for simulation.
// The plant settles at PROCESS_GAIN * 100 at full output, so 75 is its ceiling
// and the setpoint must sit below it.
const float SETPOINT = 37.0;
const float PROCESS_GAIN = 0.75;
const float PROCESS_TIME_CONSTANT = 1.0;

// Output limits
const float OUTPUT_MIN = 0.0;
const float OUTPUT_MAX = 255.0;

// Autotuning parameters
const float RELAY_AMPLITUDE = 50.0;  // 20% of output range
const float NOISE_BAND = 5.0;        // Ignore oscillations smaller than this

// The tuner swings its output symmetrically about zero, between
// -RELAY_AMPLITUDE and +RELAY_AMPLITUDE. Most real actuators are unipolar --
// a heater or a PWM pin cannot accept a negative drive -- so the relay has to
// be centred on an operating point that puts the process near the setpoint.
// Here 127 holds the plant at roughly 37, and the relay swings around that.
//
// Without this bias the relay output is negative half the time and clipped to
// zero the rest, the measurement never crosses the setpoint, the relay never
// switches, and tuning aborts on the timeout with no result.
const float OUTPUT_BIAS = 127.0;

// Progress reporting
int lastProgressPercent = -1;

// Timing
const unsigned long SAMPLE_TIME_MS = 100;
unsigned long lastTime = 0;

// Process variables
float measurement = 0.0;
float output = 0.0;

// Tuning results
float tuned_kp = 0.0;
float tuned_ki = 0.0;
float tuned_kd = 0.0;

// State machine
enum State {
  STATE_INIT,
  STATE_TUNING,
  STATE_TUNING_COMPLETE,
  STATE_RUNNING_TUNED
};
State currentState = STATE_INIT;

// Create PID controller with initial conservative gains
PIDController pid(1.0, 0.0, 0.0, OUTPUT_MIN, OUTPUT_MAX);

// Create autotuner (optional add-on)
PIDTuner tuner(pid);

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  Serial.println(F("=== easyPID AutoTune Example ==="));
  Serial.println(F("Automatic PID tuning using relay method"));
  Serial.println();
  Serial.print(F("Setpoint: "));
  Serial.println(SETPOINT);
  Serial.print(F("Relay Amplitude: +/-"));
  Serial.println(RELAY_AMPLITUDE);
  Serial.print(F("Output Bias: "));
  Serial.println(OUTPUT_BIAS);
  Serial.println();
  
  // Initialize PID
  pid.begin();
  
  // Start autotuning
  Serial.println(F("Starting autotuning..."));
  Serial.println(F("The system will oscillate. Please wait..."));
  Serial.println();
  
  if (tuner.start(SETPOINT, RELAY_AMPLITUDE, NOISE_BAND)) {
    currentState = STATE_TUNING;
    Serial.println(F("Autotuner started successfully"));
  } else {
    Serial.println(F("Failed to start autotuner!"));
    currentState = STATE_INIT;
  }
  
  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  
  // Run at fixed sample time
  if (now - lastTime >= SAMPLE_TIME_MS) {
    lastTime = now;
    float dt = SAMPLE_TIME_MS / 1000.0;
    
    switch (currentState) {
      case STATE_INIT:
        // Reached if the tuner refused to start, or if tuning failed. The
        // output is held at zero and the sketch idles rather than pretending
        // to control anything.
        output = 0.0;
        break;
        
      case STATE_TUNING: {
        // Run autotuner. The relay output is centred on zero, so the operating
        // bias is added before it reaches the actuator.
        output = tuner.update(measurement) + OUTPUT_BIAS;
        if (output < OUTPUT_MIN) output = OUTPUT_MIN;
        if (output > OUTPUT_MAX) output = OUTPUT_MAX;

        // Tuning can fail: the process may not respond to the relay, or it may
        // never settle into a consistent limit cycle. The tuner returns to
        // TUNER_IDLE in that case, so a sketch that only ever checks
        // isComplete() would spin here forever.
        if (!tuner.isComplete() && tuner.getState() == TUNER_IDLE) {
          Serial.println();
          Serial.println(F("=== AUTOTUNING FAILED ==="));
          Serial.println(F("No usable limit cycle was measured. Check that:"));
          Serial.println(F("  - the relay amplitude is large enough to move the process"));
          Serial.println(F("  - the noise band is smaller than the expected oscillation"));
          Serial.println(F("  - the setpoint is actually reachable"));
          output = 0.0;
          currentState = STATE_INIT;
          break;
        }

        // Check if tuning is complete
        if (tuner.isComplete()) {
          currentState = STATE_TUNING_COMPLETE;
          Serial.println();
          Serial.println(F("=== AUTOTUNING COMPLETE ==="));
          Serial.println();
          
          // Get ultimate parameters
          Serial.print(F("Ultimate Gain (Ku): "));
          Serial.println(tuner.getUltimateGain(), 4);
          Serial.print(F("Ultimate Period (Pu): "));
          Serial.print(tuner.getUltimatePeriod(), 4);
          Serial.println(F(" seconds"));
          Serial.println();
          
          // Display tuning results for all rules
          Serial.println(F("Tuning Results:"));
          displayTuningRule(TUNING_ZIEGLER_NICHOLS, F("Ziegler-Nichols"));
          displayTuningRule(TUNING_TYREUS_LUYBEN, F("Tyreus-Luyben"));
          displayTuningRule(TUNING_PESSEN, F("Pessen Integral"));
          displayTuningRule(TUNING_NO_OVERSHOOT, F("No Overshoot"));
          Serial.println();
          
          // Apply Ziegler-Nichols tuning (can change to other rules)
          tuner.getTunings(tuned_kp, tuned_ki, tuned_kd, TUNING_ZIEGLER_NICHOLS);
          pid.setTunings(tuned_kp, tuned_ki, tuned_kd);
          
          Serial.println(F("Applied Ziegler-Nichols tuning to PID"));
          Serial.println(F("Now running with tuned parameters..."));
          Serial.println();
          Serial.println(F("Time(s),Setpoint,Measurement,Output,Error"));
          
          // Reset PID state before running with new gains
          pid.reset();
          measurement = 0.0; // Reset process
          
          currentState = STATE_RUNNING_TUNED;
        } else {
          // Print progress only when it actually changes. The previous test
          // fired on every loop iteration whose percentage happened to be a
          // multiple of ten, so the same line was printed hundreds of times.
          int percent = (int)(tuner.getProgress() * 100.0);
          if (percent != lastProgressPercent) {
            lastProgressPercent = percent;
            Serial.print(F("Progress: "));
            Serial.print(percent);
            Serial.println(F("%"));
          }
        }
        break;
      }
        
      case STATE_TUNING_COMPLETE:
        // Transition state (handled above)
        break;
        
      case STATE_RUNNING_TUNED: {
        // Run PID with tuned parameters. No bias here: the controller drives
        // the actuator over its full range directly.
        output = pid.update(SETPOINT, measurement);

        // Print data for Serial Plotter
        float timeSeconds = now / 1000.0;
        Serial.print(timeSeconds, 2);
        Serial.print(F(","));
        Serial.print(SETPOINT, 2);
        Serial.print(F(","));
        Serial.print(measurement, 2);
        Serial.print(F(","));
        Serial.print(output, 2);
        Serial.print(F(","));
        Serial.println(pid.getError(), 2);
        break;
      }
    }
    
    // Simulate first-order process (common to all states)
    float alpha = dt / (PROCESS_TIME_CONSTANT + dt);
    float processInput = output * PROCESS_GAIN / 255.0;
    measurement = measurement + (processInput * 100.0 - measurement) * alpha;
  }
}

void displayTuningRule(TuningRule rule, const __FlashStringHelper* name) {
  float kp, ki, kd;
  if (tuner.getTunings(kp, ki, kd, rule)) {
    Serial.print(F("  "));
    Serial.print(name);
    Serial.print(F(": Kp="));
    Serial.print(kp, 4);
    Serial.print(F(", Ki="));
    Serial.print(ki, 4);
    Serial.print(F(", Kd="));
    Serial.println(kd, 4);
  }
}
