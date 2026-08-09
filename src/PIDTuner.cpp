/**
 * @file PIDTuner.cpp
 * @brief Implementation of PIDTuner class
 * @author Rami Kronbi
 * @date 2024
 */

#include "PIDTuner.h"

// Arduino.h already defines PI (as a double). Redefining it here produced a
// "PI redefined" warning on every build, so use a private float constant.
static const float EASYPID_PI = 3.14159265359f;

#define MIN_CYCLES_FOR_TUNING 3    // Minimum oscillation cycles needed
#define MAX_WAIT_TIME_MS 60000     // Maximum wait time (60 seconds)

PIDTuner::PIDTuner(PIDController& pid)
    : pid_(pid) {
    state_ = TUNER_IDLE;
    resultsValid_ = false;
    ultimateGain_ = 0.0f;
    ultimatePeriod_ = 0.0f;

    // Every member gets a defined value at construction so that calling
    // update() before start() cannot read uninitialised memory.
    setpoint_ = 0.0f;
    relayAmplitude_ = 0.0f;
    noiseBand_ = 0.0f;

    relayHigh_ = false;
    relayHighPrev_ = false;
    outputHigh_ = 0.0f;
    outputLow_ = 0.0f;

    cycleMax_ = 0.0f;
    cycleMin_ = 0.0f;
    haveCycleStart_ = false;
    cycleStartTime_ = 0;

    cyclesDetected_ = 0;
    cyclesNeeded_ = 0;
    periodSum_ = 0.0f;
    amplitudeSum_ = 0.0f;
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
    relayHighPrev_ = false;
    cycleMax_ = setpoint_;
    cycleMin_ = setpoint_;
    haveCycleStart_ = false;
    cycleStartTime_ = millis();

    cyclesDetected_ = 0;
    cyclesNeeded_ = MIN_CYCLES_FOR_TUNING + 2; // Extra cycles for stability
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
    
    // Timeout check: no complete relay cycle for too long
    if (now - cycleStartTime_ > MAX_WAIT_TIME_MS) {
        state_ = TUNER_IDLE;
        return 0.0f;
    }

    // Relay feedback control (bang-bang, with the noise band as hysteresis)
    if (measurement > setpoint_ + noiseBand_) {
        relayHigh_ = false; // Switch to low output
    } else if (measurement < setpoint_ - noiseBand_) {
        relayHigh_ = true;  // Switch to high output
    }

    float output = relayHigh_ ? outputHigh_ : outputLow_;

    // Measure the limit cycle induced by the relay
    trackLimitCycle(measurement, now);
    
    // Check if enough cycles collected
    if (cyclesDetected_ >= cyclesNeeded_) {
        calculateResults();
        state_ = TUNER_COMPLETE;
    }
    
    return output;
}

void PIDTuner::trackLimitCycle(float measurement, unsigned long now) {
    // Accumulate the extremes of the cycle currently in progress. The process
    // peak always lags the relay switch (that lag is exactly what makes the
    // loop oscillate), so the extremes must be tracked on every update rather
    // than sampled at the switching instant.
    if (measurement > cycleMax_) {
        cycleMax_ = measurement;
    }
    if (measurement < cycleMin_) {
        cycleMin_ = measurement;
    }

    // A rising edge (output low -> high) delimits one full period of the
    // limit cycle: low peak, high peak, back to the next low crossing.
    bool risingEdge = (relayHigh_ && !relayHighPrev_);
    relayHighPrev_ = relayHigh_;

    if (!risingEdge) {
        return;
    }

    if (haveCycleStart_) {
        // 'a' in the describing-function formula is the half peak-to-peak
        // swing of the oscillation, not the full peak-to-peak span.
        amplitudeSum_ += (cycleMax_ - cycleMin_) * 0.5f;
        periodSum_ += (float)(now - cycleStartTime_);
        cyclesDetected_++;
    } else {
        // The very first edge only establishes the reference point; no
        // complete cycle has elapsed yet.
        haveCycleStart_ = true;
    }

    // Begin a fresh measurement window.
    cycleStartTime_ = now;
    cycleMax_ = measurement;
    cycleMin_ = measurement;
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
        ultimateGain_ = (4.0f * relayAmplitude_) / (EASYPID_PI * avgAmplitude);
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
