#ifndef WAVE_PASSIVE_OBSERVER_H
#define WAVE_PASSIVE_OBSERVER_H

#include <Arduino.h>
#include <math.h>

/*
  WavePassiveObserver
  -------------------
  Lightweight 1-DOF yaw wave passive observer for Arduino.

  Purpose:
    Separate measured heading into:
      psiMeasured = psiLF + psiWave + noise

    The autopilot should use psiLF and rLF, not raw psiMeasured,
    so the rudder does not chase first-order wave-induced yaw motion.

  Units:
    - All angles in radians.
    - Angular rates in rad/s.
    - dt in seconds.
    - Rudder angle deltaRudder in radians.
    - Boat speed in m/s.

  Main states:
    psiW  : wave-induced yaw angle estimate
    rW    : wave-induced yaw rate estimate
    psiLF : low-frequency heading estimate
    rLF   : low-frequency yaw rate estimate
    bias  : slow yaw-rate bias/disturbance estimate

  Optional rIMU:
    Set config.useRimu = true to use a reliable IMU yaw rate measurement.
    The class will then use a secondary innovation:
      eR = rIMU - (rLF + rW)
*/

struct WaveObserverConfig {
    // Nomoto-like low-frequency yaw model.
    float Tnomoto;       // s, first-order yaw time constant. Example: 3..8
    float Knomoto;       // 1/s approximately, yaw-rate gain per rudder rad.

    // Slow environmental bias model.
    float Tbias;         // s, bias decay time. Example: 60..300

    // Initial/adaptive wave model.
    float omega0Initial; // rad/s, initial dominant wave encounter frequency.
    float omega0Min;     // rad/s
    float omega0Max;     // rad/s
    float zetaWave;      // relative damping. Example: 0.05..0.25

    // Observer gains for heading innovation.
    float kPsi;          // heading LF correction gain
    float kR;            // yaw-rate LF correction gain
    float kBias;         // bias correction gain
    float waveGain;      // base wave correction gain

    // Optional rIMU correction gains.
    bool useRimu;        // true: use rIMU if provided in update()
    float kRimuWave;     // correction applied to rW from rate innovation
    float kRimuLF;       // correction applied to rLF from rate innovation

    // Dynamic wave adaptation.
    bool enableAdaptiveOmega;
    float alphaOmega;          // 0.01..0.05, smoothing for omega0 adaptation
    float minWaveAmplitude;    // rad, minimum signal amplitude for zero-cross detection
    float zeroCrossHysteresis; // rad, hysteresis threshold for zero crossing
    float minWavePeriod;       // s, physically allowed min period. Example: 2
    float maxWavePeriod;       // s, physically allowed max period. Example: 15

    // Safety limits.
    float innovationLimit;     // rad, max heading innovation used by observer
    float rateInnovationLimit; // rad/s, max rIMU innovation
    float maxWaveYaw;          // rad
    float maxWaveRate;         // rad/s
    float maxLFRate;           // rad/s
    float maxBias;             // rad/s^2 equivalent term in rLF_dot

    // Internal RMS/confidence filter.
    float waveRmsAlpha;        // 0..1, EWMA factor per update. Example: 0.01
};

class WavePassiveObserver {
public:
    WavePassiveObserver();

    void begin(const WaveObserverConfig& cfg);
    void reset(float psiInitial, float rInitial = 0.0f);

    bool setBoatYawModel(float Tnomoto_s, float Knomoto_rad_s_per_rad);

    float getTnomoto() const;
    float getKnomoto() const;

    // Update without rIMU. If config.useRimu is true, rate correction is simply skipped.
    void update(
        float psiMeasured,
        float deltaRudder,
        bool allowAdaptation,
        float dt
    );

    // Update with rIMU. rIMU is used only if config.useRimu == true.
    void update(
        float psiMeasured,
        float rIMU,
        float deltaRudder,
        bool allowAdaptation,
        float dt
    );

    // Runtime configuration helpers.
    void setUseRimu(bool enabled);
    bool getUseRimu() const;
    void setAdaptiveOmegaEnabled(bool enabled);
    bool getAdaptiveOmegaEnabled() const;

    // Getters.
    float getPsiLF() const;
    float getRLF() const;
    float getPsiWave() const;
    float getRWave() const;
    float getBias() const;
    float getOmega0() const;
    float getWavePeriod() const;
    float getInnovation() const;
    float getRateInnovation() const;
    float getWaveConfidence() const;
    float getWaveRms() const;
    bool  isWaveFilterActive() const;

    // Utility functions.
    static float wrapAngle(float a);
    static float constrainFloat(float x, float lo, float hi);

private:
    WaveObserverConfig _cfg;
    bool _begun;

    // Observer states.
    float _psiW;
    float _rW;
    float _psiLF;
    float _rLF;
    float _bias;
    float _omega0;

    // Diagnostics.
    float _innovation;
    float _rateInnovation;
    float _waveRmsSq;
    float _waveConfidence;
    bool  _waveFilterActive;

    // Time and zero-crossing tracking.
    float _time;
    float _lastCrossTime;
    int8_t _waveSignState; // -1, 0, +1
    uint8_t _validCrossings;

    void updateInternal(
        float psiMeasured,
        float rIMU,
        bool hasRimu,
        float deltaRudder,
        bool allowAdaptation,
        float dt
    );

    void updateAdaptiveOmega(float waveSignal, bool allowAdaptation, float dt);
    void updateWaveConfidence(float waveSignal);
    void validateConfig();
    bool badNumber(float x) const;
};

#endif
