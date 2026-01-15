/**
 * @file MultiLoopPID.ino
 * @brief Multi-Loop PID Controller Example
 * @author Rami Kronbi
 * @date 2024
 * 
 * This example demonstrates:
 * - Multiple independent PID controllers running concurrently
 * - Two simulated processes with different characteristics
 * - Multi-instance capability (no global state interference)
 * - Simultaneous control of independent systems
 * 
 * Real-world applications:
 * - Dual-axis positioning (X-Y stage, pan-tilt mechanism)
 * - Multi-zone temperature control
 * - Multiple motor speed control
 * - Any system with multiple independent control loops
 * 
 * Hardware: Arduino Uno (or compatible)
 * No external hardware required - all simulation in software
 */

#include <easyPID.h>

// Process 1: Fast response system (e.g., small motor, low inertia)
const float SETPOINT_1 = 80.0;
const float PROCESS_1_GAIN = 0.9;
const float PROCESS_1_TIME_CONSTANT = 0.08;  // Fast response
const float KP_1 = 1.5;
const float KI_1 = 0.8;
const float KD_1 = 0.05;

// Process 2: Slow response system (e.g., thermal system, high inertia)
const float SETPOINT_2 = 120.0;
const float PROCESS_2_GAIN = 0.7;
const float PROCESS_2_TIME_CONSTANT = 0.25;  // Slow response
const float KP_2 = 3.0;
const float KI_2 = 0.3;
const float KD_2 = 0.2;

// Common output limits
const float OUTPUT_MIN = 0.0;
const float OUTPUT_MAX = 255.0;

// Timing
const unsigned long SAMPLE_TIME_MS = 100;  // 100ms = 10Hz
unsigned long lastTime = 0;

// Process 1 variables
float measurement1 = 0.0;
float output1 = 0.0;

// Process 2 variables
float measurement2 = 0.0;
float output2 = 0.0;

// Create two independent PID controllers
PIDController pid1(KP_1, KI_1, KD_1, OUTPUT_MIN, OUTPUT_MAX);
PIDController pid2(KP_2, KI_2, KD_2, OUTPUT_MIN, OUTPUT_MAX);

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  Serial.println(F("=== easyPID Multi-Loop Example ==="));
  Serial.println(F("Two independent PID controllers running concurrently"));
  Serial.println();
  Serial.println(F("Process 1 (Fast): Setpoint="));
  Serial.print(SETPOINT_1);
  Serial.print(F(", Kp="));
  Serial.print(KP_1);
  Serial.print(F(", Ki="));
  Serial.print(KI_1);
  Serial.print(F(", Kd="));
  Serial.println(KD_1);
  Serial.println(F("Process 2 (Slow): Setpoint="));
  Serial.print(SETPOINT_2);
  Serial.print(F(", Kp="));
  Serial.print(KP_2);
  Serial.print(F(", Ki="));
  Serial.print(KI_2);
  Serial.print(F(", Kd="));
  Serial.println(KD_2);
  Serial.println();
  Serial.println(F("Time(s),SP1,Meas1,Out1,SP2,Meas2,Out2"));
  
  // Initialize both PID controllers
  pid1.begin();
  pid2.begin();
  
  // Configure PID 1 - demonstrate different settings
  pid1.setAntiWindup(ANTIWINDUP_CLAMP);
  pid1.setDerivativeFilter(FILTER_NONE);
  
  // Configure PID 2 - with derivative filtering for noisy process
  pid2.setAntiWindup(ANTIWINDUP_BACKCALC);
  pid2.setDerivativeFilter(FILTER_EMA, 0.7);  // Smooth derivative
  
  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  
  // Run both controllers at fixed sample time
  if (now - lastTime >= SAMPLE_TIME_MS) {
    lastTime = now;
    float dt = SAMPLE_TIME_MS / 1000.0; // Convert to seconds
    
    // === PROCESS 1: Fast Response System ===
    output1 = pid1.update(SETPOINT_1, measurement1);
    
    // Simulate fast first-order process
    float alpha1 = dt / (PROCESS_1_TIME_CONSTANT + dt);
    float processInput1 = output1 * PROCESS_1_GAIN / 255.0;
    measurement1 = measurement1 + (processInput1 * 100.0 - measurement1) * alpha1;
    
    // === PROCESS 2: Slow Response System ===
    output2 = pid2.update(SETPOINT_2, measurement2);
    
    // Simulate slow first-order process with added noise
    float alpha2 = dt / (PROCESS_2_TIME_CONSTANT + dt);
    float processInput2 = output2 * PROCESS_2_GAIN / 255.0;
    // Add small random noise to demonstrate derivative filtering benefit
    float noise = (random(-10, 10)) / 10.0;
    measurement2 = measurement2 + (processInput2 * 100.0 - measurement2) * alpha2 + noise;
    
    // Print data for Serial Plotter
    float timeSeconds = now / 1000.0;
    Serial.print(timeSeconds, 2);
    Serial.print(F(","));
    Serial.print(SETPOINT_1, 1);
    Serial.print(F(","));
    Serial.print(measurement1, 2);
    Serial.print(F(","));
    Serial.print(output1, 1);
    Serial.print(F(","));
    Serial.print(SETPOINT_2, 1);
    Serial.print(F(","));
    Serial.print(measurement2, 2);
    Serial.print(F(","));
    Serial.println(output2, 1);
    
    // Optional: Print detailed info every 2 seconds
    if (now % 2000 < SAMPLE_TIME_MS) {
      Serial.print(F("  [PID1] Error="));
      Serial.print(pid1.getError(), 2);
      Serial.print(F(" P="));
      Serial.print(pid1.getPterm(), 2);
      Serial.print(F(" I="));
      Serial.print(pid1.getIterm(), 2);
      Serial.print(F(" D="));
      Serial.println(pid1.getDterm(), 2);
      
      Serial.print(F("  [PID2] Error="));
      Serial.print(pid2.getError(), 2);
      Serial.print(F(" P="));
      Serial.print(pid2.getPterm(), 2);
      Serial.print(F(" I="));
      Serial.print(pid2.getIterm(), 2);
      Serial.print(F(" D="));
      Serial.println(pid2.getDterm(), 2);
      Serial.println();
    }
  }
}
