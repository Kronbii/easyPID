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

#include <PIDController.h>
#include <PIDTuner.h>  // Optional add-on module

// Process parameters for simulation
const float SETPOINT = 100.0;
const float PROCESS_GAIN = 0.75;
const float PROCESS_TIME_CONSTANT = 0.15;

// Output limits
const float OUTPUT_MIN = 0.0;
const float OUTPUT_MAX = 255.0;

// Autotuning parameters
const float RELAY_AMPLITUDE = 50.0;  // 20% of output range
const float NOISE_BAND = 2.0;        // Ignore oscillations smaller than this

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
  Serial.println(F("Setpoint: "));
  Serial.print(SETPOINT);
  Serial.println();
  Serial.println(F("Relay Amplitude: "));
  Serial.print(RELAY_AMPLITUDE);
  Serial.println();
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
        // Should not reach here
        break;
        
      case STATE_TUNING:
        // Run autotuner
        output = tuner.update(measurement);
        
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
          displayTuningRule(TUNING_ZIEGLER_NICHOLS, "Ziegler-Nichols");
          displayTuningRule(TUNING_TYREUS_LUYBEN, "Tyreus-Luyben");
          displayTuningRule(TUNING_PESSEN, "Pessen Integral");
          displayTuningRule(TUNING_NO_OVERSHOOT, "No Overshoot");
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
          // Print progress
          float progress = tuner.getProgress();
          if (((int)(progress * 100)) % 10 == 0) {
            Serial.print(F("Progress: "));
            Serial.print((int)(progress * 100));
            Serial.println(F("%"));
          }
        }
        break;
        
      case STATE_TUNING_COMPLETE:
        // Transition state (handled above)
        break;
        
      case STATE_RUNNING_TUNED:
        // Run PID with tuned parameters
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
    
    // Simulate first-order process (common to all states)
    float alpha = dt / (PROCESS_TIME_CONSTANT + dt);
    float processInput = output * PROCESS_GAIN / 255.0;
    measurement = measurement + (processInput * 100.0 - measurement) * alpha;
  }
}

void displayTuningRule(TuningRule rule, const char* name) {
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
