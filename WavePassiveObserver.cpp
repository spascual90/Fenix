#include "WavePassiveObserver.h"

#ifndef TWO_PI
#define TWO_PI 6.28318530717958647692f
#endif

WavePassiveObserver::WavePassiveObserver()
: _begun(false),
  _psiW(0.0f), _rW(0.0f), _psiLF(0.0f), _rLF(0.0f), _bias(0.0f),
  _omega0(1.0f),
  _innovation(0.0f), _rateInnovation(0.0f),
  _waveRmsSq(0.0f), _waveConfidence(0.0f), _waveFilterActive(false),
  _time(0.0f), _lastCrossTime(-1.0f), _waveSignState(0), _validCrossings(0)
{
    // Conservative defaults. User should normally call begin(cfg).
    _cfg.Tnomoto = 5.0f;
    _cfg.Knomoto = 0.25f;
    _cfg.Tbias = 120.0f;

    _cfg.omega0Initial = TWO_PI / 6.0f;
    _cfg.omega0Min = TWO_PI / 15.0f;
    _cfg.omega0Max = TWO_PI / 2.0f;
    _cfg.zetaWave = 0.10f;

    _cfg.kPsi = 0.08f;
    _cfg.kR = 0.04f;
    _cfg.kBias = 0.001f;
    _cfg.waveGain = 0.6f;

    _cfg.useRimu = false;
    _cfg.kRimuWave = 0.5f;
    _cfg.kRimuLF = 0.1f;

    _cfg.enableAdaptiveOmega = true;
    _cfg.alphaOmega = 0.02f;
    _cfg.minWaveAmplitude = radians(0.5f);
    _cfg.zeroCrossHysteresis = radians(0.4f);
    _cfg.minWavePeriod = 2.0f;
    _cfg.maxWavePeriod = 15.0f;
    _cfg.minSpeed = 0.5f;

    _cfg.innovationLimit = radians(10.0f);
    _cfg.rateInnovationLimit = radians(20.0f);
    _cfg.maxWaveYaw = radians(15.0f);
    _cfg.maxWaveRate = radians(25.0f);
    _cfg.maxLFRate = radians(15.0f);
    _cfg.maxBias = radians(5.0f);

    _cfg.waveRmsAlpha = 0.01f;
}

void WavePassiveObserver::begin(const WaveObserverConfig& cfg) {
    _cfg = cfg;
    validateConfig();
    _omega0 = _cfg.omega0Initial;
    _begun = true;
}

void WavePassiveObserver::reset(float psiInitial, float rInitial) {
    _psiW = 0.0f;
    _rW = 0.0f;
    _psiLF = wrapAngle(psiInitial);
    _rLF = rInitial;
    _bias = 0.0f;
    _innovation = 0.0f;
    _rateInnovation = 0.0f;
    _waveRmsSq = 0.0f;
    _waveConfidence = 0.0f;
    _waveFilterActive = false;
    _time = 0.0f;
    _lastCrossTime = -1.0f;
    _waveSignState = 0;
    _validCrossings = 0;
    _omega0 = _cfg.omega0Initial;
}

void WavePassiveObserver::update(
    float psiMeasured,
    float deltaRudder,
    float boatSpeed,
    bool allowAdaptation,
    float dt
) {
    updateInternal(psiMeasured, 0.0f, false, deltaRudder, boatSpeed, allowAdaptation, dt);
}

void WavePassiveObserver::update(
    float psiMeasured,
    float rIMU,
    float deltaRudder,
    float boatSpeed,
    bool allowAdaptation,
    float dt
) {
    updateInternal(psiMeasured, rIMU, true, deltaRudder, boatSpeed, allowAdaptation, dt);
}

void WavePassiveObserver::updateInternal(
    float psiMeasured,
    float rIMU,
    bool hasRimu,
    float deltaRudder,
    float boatSpeed,
    bool allowAdaptation,
    float dt
) {
    if (!_begun) {
        validateConfig();
        _omega0 = _cfg.omega0Initial;
        _begun = true;
    }

    if (dt <= 0.0f || dt > 0.5f) {
        return;
    }

    if (badNumber(psiMeasured) || badNumber(deltaRudder) || badNumber(boatSpeed)) {
        return;
    }

    if (hasRimu && badNumber(rIMU)) {
        hasRimu = false;
    }

    // If the observer ever becomes numerically corrupted, reset safely.
    if (badNumber(_psiW) || badNumber(_rW) || badNumber(_psiLF) ||
        badNumber(_rLF) || badNumber(_bias) || badNumber(_omega0)) {
        reset(psiMeasured, hasRimu ? rIMU : 0.0f);
    }

    _time += dt;

    psiMeasured = wrapAngle(psiMeasured);

    // Heading innovation: measured heading minus estimated measured heading.
    const float yHat = wrapAngle(_psiLF + _psiW);
    float e = wrapAngle(psiMeasured - yHat);
    e = constrainFloat(e, -_cfg.innovationLimit, _cfg.innovationLimit);
    _innovation = e;

    // Wave correction gains scheduled by current omega0.
    const float kw1 = _cfg.waveGain * 2.0f * _cfg.zetaWave * _omega0;
    const float kw2 = _cfg.waveGain * _omega0 * _omega0;

    // Wave model: second-order oscillator plus heading innovation.
    const float psiW_dot = _rW + kw1 * e;
    const float rW_dot =
        -_omega0 * _omega0 * _psiW
        -2.0f * _cfg.zetaWave * _omega0 * _rW
        +kw2 * e;

    // Low-frequency yaw dynamics: simple Nomoto-like model plus slow bias.
    const float psiLF_dot = _rLF + _cfg.kPsi * e;
    const float rLF_dot =
        (-_rLF + _cfg.Knomoto * deltaRudder) / _cfg.Tnomoto
        + _bias
        + _cfg.kR * e;

    const float bias_dot =
        -_bias / _cfg.Tbias
        + _cfg.kBias * e;

    _psiW  += dt * psiW_dot;
    _rW    += dt * rW_dot;
    _psiLF += dt * psiLF_dot;
    _rLF   += dt * rLF_dot;
    _bias  += dt * bias_dot;

    // Optional rIMU correction. This helps when yaw-rate measurement is reliable.
    _rateInnovation = 0.0f;
    if (_cfg.useRimu && hasRimu) {
        float eR = rIMU - (_rLF + _rW);
        eR = constrainFloat(eR, -_cfg.rateInnovationLimit, _cfg.rateInnovationLimit);
        _rateInnovation = eR;

        // Direct correction to rate states. We deliberately split rate innovation:
        // most fast correction goes to rW; a smaller part goes to rLF.
        _rW  += dt * _cfg.kRimuWave * eR;
        _rLF += dt * _cfg.kRimuLF   * eR;
    }

    // Safety saturation.
    _psiW = constrainFloat(_psiW, -_cfg.maxWaveYaw, _cfg.maxWaveYaw);
    _rW   = constrainFloat(_rW, -_cfg.maxWaveRate, _cfg.maxWaveRate);
    _rLF  = constrainFloat(_rLF, -_cfg.maxLFRate, _cfg.maxLFRate);
    _bias = constrainFloat(_bias, -_cfg.maxBias, _cfg.maxBias);

    _psiLF = wrapAngle(_psiLF);

    // Candidate wave signal for adaptation/confidence.
    const float waveCandidate = wrapAngle(psiMeasured - _psiLF);
    const float waveSignal = 0.7f * _psiW + 0.3f * waveCandidate;

    updateWaveConfidence(waveSignal);
    updateAdaptiveOmega(waveSignal, boatSpeed, allowAdaptation, dt);
}

void WavePassiveObserver::updateWaveConfidence(float waveSignal) {
    const float a = constrainFloat(_cfg.waveRmsAlpha, 0.0001f, 1.0f);
    _waveRmsSq += a * (waveSignal * waveSignal - _waveRmsSq);
    if (_waveRmsSq < 0.0f) _waveRmsSq = 0.0f;

    const float rms = sqrtf(_waveRmsSq);

    // Confidence grows if RMS is above minimum amplitude and if we have
    // observed repeated valid crossings. This is intentionally simple.
    float ampConf = 0.0f;
    if (_cfg.minWaveAmplitude > 0.0f) {
        ampConf = rms / (3.0f * _cfg.minWaveAmplitude);
    }
    ampConf = constrainFloat(ampConf, 0.0f, 1.0f);

    float crossConf = ((float)_validCrossings) / 6.0f;
    crossConf = constrainFloat(crossConf, 0.0f, 1.0f);

    _waveConfidence = 0.6f * ampConf + 0.4f * crossConf;
    _waveFilterActive = (_waveConfidence > 0.2f);
}

void WavePassiveObserver::updateAdaptiveOmega(
    float waveSignal,
    float boatSpeed,
    bool allowAdaptation,
    float dt
) {
    (void)dt;

    if (!_cfg.enableAdaptiveOmega || !allowAdaptation) {
        return;
    }

    if (boatSpeed < _cfg.minSpeed) {
        return;
    }

    const float threshold = max(_cfg.zeroCrossHysteresis, _cfg.minWaveAmplitude);
    if (fabsf(waveSignal) < threshold) {
        return;
    }

    int8_t newState = _waveSignState;
    if (waveSignal > threshold) {
        newState = +1;
    } else if (waveSignal < -threshold) {
        newState = -1;
    }

    if (_waveSignState == 0) {
        _waveSignState = newState;
        _lastCrossTime = _time;
        return;
    }

    if (newState != _waveSignState) {
        const float halfPeriod = _time - _lastCrossTime;
        _lastCrossTime = _time;
        _waveSignState = newState;

        const float period = 2.0f * halfPeriod;
        if (period >= _cfg.minWavePeriod && period <= _cfg.maxWavePeriod) {
            float omegaTarget = TWO_PI / period;
            omegaTarget = constrainFloat(omegaTarget, _cfg.omega0Min, _cfg.omega0Max);

            const float alpha = constrainFloat(_cfg.alphaOmega, 0.0f, 1.0f);
            _omega0 += alpha * (omegaTarget - _omega0);
            _omega0 = constrainFloat(_omega0, _cfg.omega0Min, _cfg.omega0Max);

            if (_validCrossings < 255) {
                _validCrossings++;
            }
        }
    }
}

void WavePassiveObserver::setUseRimu(bool enabled) {
    _cfg.useRimu = enabled;
}

bool WavePassiveObserver::getUseRimu() const {
    return _cfg.useRimu;
}

void WavePassiveObserver::setAdaptiveOmegaEnabled(bool enabled) {
    _cfg.enableAdaptiveOmega = enabled;
}

bool WavePassiveObserver::getAdaptiveOmegaEnabled() const {
    return _cfg.enableAdaptiveOmega;
}

float WavePassiveObserver::getPsiLF() const {
    return _psiLF;
}

float WavePassiveObserver::getRLF() const {
    return _rLF;
}

float WavePassiveObserver::getPsiWave() const {
    return _psiW;
}

float WavePassiveObserver::getRWave() const {
    return _rW;
}

float WavePassiveObserver::getBias() const {
    return _bias;
}

float WavePassiveObserver::getOmega0() const {
    return _omega0;
}

float WavePassiveObserver::getWavePeriod() const {
    if (_omega0 <= 0.0f) return 0.0f;
    return TWO_PI / _omega0;
}

float WavePassiveObserver::getInnovation() const {
    return _innovation;
}

float WavePassiveObserver::getRateInnovation() const {
    return _rateInnovation;
}

float WavePassiveObserver::getWaveConfidence() const {
    return _waveConfidence;
}

float WavePassiveObserver::getWaveRms() const {
    return sqrtf(_waveRmsSq);
}

bool WavePassiveObserver::isWaveFilterActive() const {
    return _waveFilterActive;
}

float WavePassiveObserver::wrapAngle(float a) {
    while (a > PI)  a -= TWO_PI;
    while (a < -PI) a += TWO_PI;
    return a;
}

float WavePassiveObserver::constrainFloat(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void WavePassiveObserver::validateConfig() {
    if (_cfg.Tnomoto < 0.1f) _cfg.Tnomoto = 0.1f;
    if (_cfg.Tbias < 1.0f) _cfg.Tbias = 1.0f;

    if (_cfg.omega0Min <= 0.0f) _cfg.omega0Min = TWO_PI / 20.0f;
    if (_cfg.omega0Max <= _cfg.omega0Min) _cfg.omega0Max = TWO_PI / 1.5f;
    if (_cfg.omega0Initial < _cfg.omega0Min) _cfg.omega0Initial = _cfg.omega0Min;
    if (_cfg.omega0Initial > _cfg.omega0Max) _cfg.omega0Initial = _cfg.omega0Max;

    _cfg.zetaWave = constrainFloat(_cfg.zetaWave, 0.01f, 1.0f);
    _cfg.waveGain = constrainFloat(_cfg.waveGain, 0.0f, 5.0f);

    if (_cfg.minWavePeriod < 0.5f) _cfg.minWavePeriod = 0.5f;
    if (_cfg.maxWavePeriod < _cfg.minWavePeriod) _cfg.maxWavePeriod = _cfg.minWavePeriod + 1.0f;

    if (_cfg.innovationLimit <= 0.0f) _cfg.innovationLimit = radians(10.0f);
    if (_cfg.rateInnovationLimit <= 0.0f) _cfg.rateInnovationLimit = radians(20.0f);

    if (_cfg.maxWaveYaw <= 0.0f) _cfg.maxWaveYaw = radians(15.0f);
    if (_cfg.maxWaveRate <= 0.0f) _cfg.maxWaveRate = radians(25.0f);
    if (_cfg.maxLFRate <= 0.0f) _cfg.maxLFRate = radians(15.0f);
    if (_cfg.maxBias <= 0.0f) _cfg.maxBias = radians(5.0f);

    if (_cfg.waveRmsAlpha <= 0.0f || _cfg.waveRmsAlpha > 1.0f) {
        _cfg.waveRmsAlpha = 0.01f;
    }
}

bool WavePassiveObserver::badNumber(float x) const {
    return isnan(x) || isinf(x);
}
