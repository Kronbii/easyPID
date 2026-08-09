/**
 * @file BasicPID.ino
 * @brief Basic PID Controller Example with Simulated Process
 * @author Rami Kronbi
 * @date 2024
 * 
 * This example demonstrates:
 * - Single PID controller instance
 * - Software-simulated first-order process
 * - Real-time monitoring of PID terms
 * - Basic setpoint tracking
 * 
 * The simulated process models a simple system like:
 * - Temperature control (heater + thermal mass)
 * - Motor speed control
 * - Any first-order lag system
 * 
 * Hardware: Arduino Uno (or compatible)
 * No external hardware required - all simulation in software
 */

#include <easyPID.h>

// Setpoint and process parameters.
//
// The simulated plant settles at PROCESS_GAIN * 100 when the output is at its
// maximum of 255, so PROCESS_GAIN * 100 is the highest value it can ever
// reach. Keep SETPOINT below that or the loop can never close: the output
// pins at 255 and the error never reaches zero, which demonstrates windup
// rather than control.
const float SETPOINT = 100.0;            // Target value
const float PROCESS_GAIN = 1.5;          // Plant ceiling = 150, comfortably above setpoint
const float PROCESS_TIME_CONSTANT = 0.1; // Time constant in seconds

// PID tuning parameters (hand-tuned for this simulated process)
const float KP = 2.0;   // Proportional gain
const float KI = 0.5;   // Integral gain
const float KD = 0.1;   // Derivative gain

// Output limits
const float OUTPUT_MIN = 0.0;
const float OUTPUT_MAX = 255.0;

// Timing
const unsigned long SAMPLE_TIME_MS = 100;  // 100ms = 10Hz
unsigned long lastTime = 0;

// Process variables
float measurement = 0.0;    // Current process value
float output = 0.0;         // PID output

// Create PID controller
PIDController pid(KP, KI, KD, OUTPUT_MIN, OUTPUT_MAX);

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port to connect (needed for some boards)
  }
  
  Serial.println(F("=== easyPID Basic Example ==="));
  Serial.println(F("Simulating first-order process with PID control"));
  Serial.println();
  Serial.print(F("Setpoint: "));
  Serial.println(SETPOINT);
  Serial.print(F("PID Gains: Kp="));
  Serial.print(KP);
  Serial.print(F(", Ki="));
  Serial.print(KI);
  Serial.print(F(", Kd="));
  Serial.println(KD);
  Serial.println();
  Serial.println(F("Time(s),Setpoint,Measurement,Output,Error,P,I,D"));
  
  // Initialize PID controller
  pid.begin();
  
  // Configure PID (optional - demonstrate API)
  pid.setAntiWindup(ANTIWINDUP_CLAMP);
  pid.setDerivativeFilter(FILTER_NONE);
  
  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  
  // Run at fixed sample time
  if (now - lastTime >= SAMPLE_TIME_MS) {
    lastTime = now;
    
    // Run PID controller. This overload measures dt from millis() itself, so
    // it is safe to call at whatever cadence the sketch happens to run at.
    output = pid.update(SETPOINT, measurement);
    
    // Simulate first-order process response
    // This models: dY/dt = (K*U - Y) / tau
    // Where: Y = measurement, U = output, K = gain, tau = time constant
    // Discrete approximation: Y(n+1) = Y(n) + (K*U - Y(n)) * alpha
    // Where: alpha = dt / (tau + dt)
    float dt = SAMPLE_TIME_MS / 1000.0; // Convert to seconds
    float alpha = dt / (PROCESS_TIME_CONSTANT + dt);
    float processInput = output * PROCESS_GAIN / 255.0; // Normalize output
    measurement = measurement + (processInput * 100.0 - measurement) * alpha;
    
    // Print data for Serial Plotter / Monitor
    float timeSeconds = now / 1000.0;
    Serial.print(timeSeconds, 2);
    Serial.print(F(","));
    Serial.print(SETPOINT, 2);
    Serial.print(F(","));
    Serial.print(measurement, 2);
    Serial.print(F(","));
    Serial.print(output, 2);
    Serial.print(F(","));
    Serial.print(pid.getError(), 2);
    Serial.print(F(","));
    Serial.print(pid.getPterm(), 2);
    Serial.print(F(","));
    Serial.print(pid.getIterm(), 2);
    Serial.print(F(","));
    Serial.println(pid.getDterm(), 2);
  }
}
