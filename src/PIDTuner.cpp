/**
 * @file PIDTuner.cpp
 * @brief Implementation of PIDTuner class
 * @author Rami Kronbi
 * @date 2024
 */

#include "PIDTuner.h"

#define PI 3.14159265359f
#define MIN_CYCLES_FOR_TUNING 3    // Minimum oscillation cycles needed
#define MAX_WAIT_TIME_MS 60000     // Maximum wait time (60 seconds)

PIDTuner::PIDTuner(PIDController& pid)
    : pid_(pid) {
    state_ = TUNER_IDLE;
    resultsValid_ = false;
    ultimateGain_ = 0.0f;
    ultimatePeriod_ = 0.0f;
}

bool PIDTuner::start(float setpoint, float relayAmplitude, float noiseBand) {
    if (state_ != TUNER_IDLE && state_ != TUNER_COMPLETE) {
        return false; // Already running
    }
    
    // Initialize tuning parameters
    setpoint_ = setpoint;
    relayAmplitude_ = relayAmplitude;
    noiseBand_ = noiseBand;
    
    // Set relay output levels
    outputHigh_ = relayAmplitude_;
    outputLow_ = -relayAmplitude_;
    
    // Reset detection variables
    relayHigh_ = false;
    peakHigh_ = setpoint_;
    peakLow_ = setpoint_;
    peakHighTime_ = 0;
    peakLowTime_ = 0;
    lookingForPeak_ = true;
    peakType_ = 0;
    
    cyclesDetected_ = 0;
    cyclesNeeded_ = MIN_CYCLES_FOR_TUNING + 2; // Extra cycles for stability
    lastPeakTime_ = millis();
    periodSum_ = 0.0f;
    amplitudeSum_ = 0.0f;
    
    resultsValid_ = false;
    ultimateGain_ = 0.0f;
    ultimatePeriod_ = 0.0f;
    
    state_ = TUNER_RELAY_STEP;
    return true;
}

float PIDTuner::update(float measurement) {
    if (state_ != TUNER_RELAY_STEP) {
        return 0.0f; // Not actively tuning
    }
    
    unsigned long now = millis();
    
    // Timeout check
    if (now - lastPeakTime_ > MAX_WAIT_TIME_MS) {
        state_ = TUNER_IDLE;
        return 0.0f;
    }
    
    // Relay feedback control (bang-bang)
    if (measurement > setpoint_ + noiseBand_) {
        relayHigh_ = false; // Switch to low output
    } else if (measurement < setpoint_ - noiseBand_) {
        relayHigh_ = true;  // Switch to high output
    }
    
    float output = relayHigh_ ? outputHigh_ : outputLow_;
    
    // Detect peaks in oscillation
    detectPeak(measurement, now);
    
    // Check if enough cycles collected
    if (cyclesDetected_ >= cyclesNeeded_) {
        calculateResults();
        state_ = TUNER_COMPLETE;
    }
    
    return output;
}

void PIDTuner::detectPeak(float measurement, unsigned long now) {
    // Simple peak detection based on relay state transitions
    
    if (lookingForPeak_) {
        if (relayHigh_ && peakType_ != 1) {
            // Looking for high peak
            if (measurement > peakHigh_) {
                peakHigh_ = measurement;
                peakHighTime_ = now;
            }
        } else if (!relayHigh_ && peakType_ != -1) {
            // Looking for low peak
            if (measurement < peakLow_) {
                peakLow_ = measurement;
                peakLowTime_ = now;
            }
        }
    }
    
    // Detect when relay switches (indicates we passed a peak)
    static bool lastRelayState = relayHigh_;
    if (relayHigh_ != lastRelayState) {
        lastRelayState = relayHigh_;
        
        if (relayHigh_) {
            // Just switched to high, we crossed below setpoint (found low peak)
            if (cyclesDetected_ > 0) { // Skip first transition
                float amplitude = peakHigh_ - peakLow_;
                amplitudeSum_ += amplitude;
                
                unsigned long period = now - lastPeakTime_;
                periodSum_ += (float)period;
                
                cyclesDetected_++;
            }
            lastPeakTime_ = now;
            peakLow_ = measurement;
            peakType_ = -1;
            
        } else {
            // Just switched to low, we crossed above setpoint (found high peak)
            peakHigh_ = measurement;
            peakType_ = 1;
        }
    }
}

void PIDTuner::calculateResults() {
    if (cyclesDetected_ < MIN_CYCLES_FOR_TUNING) {
        resultsValid_ = false;
        return;
    }
    
    // Average period and amplitude over collected cycles
    float avgPeriod = periodSum_ / (float)cyclesDetected_;  // in milliseconds
    float avgAmplitude = amplitudeSum_ / (float)cyclesDetected_;
    
    // Convert period to seconds
    ultimatePeriod_ = avgPeriod / 1000.0f;
    
    // Calculate ultimate gain using relay method formula
    // Ku = 4*d / (π*a)
    // where d = relay amplitude, a = oscillation amplitude
    if (avgAmplitude > 0.0f) {
        ultimateGain_ = (4.0f * relayAmplitude_) / (PI * avgAmplitude);
        resultsValid_ = true;
    } else {
        resultsValid_ = false;
    }
}

bool PIDTuner::isComplete() const {
    return state_ == TUNER_COMPLETE && resultsValid_;
}

bool PIDTuner::getTunings(float& kp, float& ki, float& kd, TuningRule rule) {
    if (!resultsValid_ || ultimatePeriod_ <= 0.0f) {
        return false;
    }
    
    applyTuningRule(kp, ki, kd, rule);
    return true;
}

void PIDTuner::applyTuningRule(float& kp, float& ki, float& kd, TuningRule rule) {
    float Ku = ultimateGain_;
    float Pu = ultimatePeriod_;
    
    switch (rule) {
        case TUNING_ZIEGLER_NICHOLS:
            // Classic Ziegler-Nichols tuning rules
            kp = 0.6f * Ku;
            ki = 1.2f * Ku / Pu;
            kd = 0.075f * Ku * Pu;
            break;
            
        case TUNING_TYREUS_LUYBEN:
            // Tyreus-Luyben: less overshoot, better for processes with lag
            kp = Ku / 2.2f;
            ki = kp / (2.2f * Pu);
            kd = kp * Pu / 6.3f;
            break;
            
        case TUNING_PESSEN:
            // Pessen Integral Rule: faster response, some overshoot
            kp = 0.7f * Ku;
            ki = 1.75f * Ku / Pu;
            kd = 0.105f * Ku * Pu;
            break;
            
        case TUNING_NO_OVERSHOOT:
            // Conservative tuning: minimal overshoot
            kp = 0.2f * Ku;
            ki = 0.4f * Ku / Pu;
            kd = 0.066f * Ku * Pu;
            break;
            
        default:
            // Default to Ziegler-Nichols
            kp = 0.6f * Ku;
            ki = 1.2f * Ku / Pu;
            kd = 0.075f * Ku * Pu;
            break;
    }
}

float PIDTuner::getUltimateGain() const {
    return ultimateGain_;
}

float PIDTuner::getUltimatePeriod() const {
    return ultimatePeriod_;
}

void PIDTuner::cancel() {
    state_ = TUNER_IDLE;
    resultsValid_ = false;
}

TunerState PIDTuner::getState() const {
    return state_;
}

float PIDTuner::getProgress() const {
    if (state_ != TUNER_RELAY_STEP) {
        return 0.0f;
    }
    
    float progress = (float)cyclesDetected_ / (float)cyclesNeeded_;
    if (progress > 1.0f) progress = 1.0f;
    return progress;
}
