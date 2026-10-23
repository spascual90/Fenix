#include "WindFeedforward.h"


namespace {
constexpr float TWO_PI_F = 6.28318530717958647692f;
constexpr float TABLE_SCALE = 36.0f / TWO_PI_F;
constexpr float MOMENT_EPSILON_NM = 1.0e-5f;

static_assert(WIND_FF_START_SPEED_MPS >= WIND_FF_STOP_SPEED_MPS,
              "WIND_FF_START_SPEED_MPS must be >= WIND_FF_STOP_SPEED_MPS");
static_assert(WIND_FF_MAX_MOMENT_NM > 0.0f,
              "WIND_FF_MAX_MOMENT_NM must be > 0");
static_assert(WIND_FF_FILTER_TIME_MS >= 0,
              "WIND_FF_FILTER_TIME_MS must be >= 0");
static_assert(WIND_FF_OUTPUT_FILTER_TIME_MS >= 0,
              "WIND_FF_OUTPUT_FILTER_TIME_MS must be >= 0");
static_assert(WIND_FF_INVALID_DECAY_TIME_MS >= 0,
              "WIND_FF_INVALID_DECAY_TIME_MS must be >= 0");
} // namespace

/*
 * Example dimensionless Cn curve.
 *
 * IMPORTANT: These values are a generic, smooth and symmetric starting point;
 * they are not identified coefficients for a specific vessel. Replace them
 * with measured or modelled values for the boat.
 *
 * Index i corresponds to i * 10 degrees. The table satisfies:
 *
 *     Cn(360 deg - gamma) = -Cn(gamma)
 *     Cn(0 deg) = Cn(180 deg) = 0
 */
const float WindFeedforward::CN_TABLE[WindFeedforward::kTableSize]
    WIND_FF_PROGMEM = {
     0.00f,  0.04f,  0.10f,  0.18f,  0.30f,  0.45f,
     0.62f,  0.78f,  0.92f,  1.00f,  0.96f,  0.88f,
     0.74f,  0.58f,  0.42f,  0.26f,  0.13f,  0.05f,
     0.00f, -0.05f, -0.13f, -0.26f, -0.42f, -0.58f,
    -0.74f, -0.88f, -0.96f, -1.00f, -0.92f, -0.78f,
    -0.62f, -0.45f, -0.30f, -0.18f, -0.10f, -0.04f
};

WindFeedforward::WindFeedforward(float kAir)
#if WIND_FF_ENABLE_INPUT_FILTER
    : _filteredU(0.0f),
      _filteredV(0.0f),
      _yawMoment(0.0f),
      _kAir(0.0f),
      _status(0U)
#else
    : _yawMoment(0.0f),
      _kAir(0.0f),
      _status(0U)
#endif
{
    setKAir(kAir);
}

void WindFeedforward::reset()
{
#if WIND_FF_ENABLE_INPUT_FILTER
    _filteredU = 0.0f;
    _filteredV = 0.0f;
#endif

    _yawMoment = 0.0f;
    _status = 0U;
}

void WindFeedforward::setKAir(float kAir)
{
    _kAir = (isFiniteValue(kAir) && kAir >= 0.0f) ? kAir : 0.0f;
}

float WindFeedforward::update(float apparentWindSpeed,
							  float apparentWindAngleRad,
                              bool measurementValid) // m/s and (-PI,PI)
{
    static uint32_t previousMillis = millis();

    const uint32_t currentMillis = millis();
    const uint32_t dt = (currentMillis - previousMillis) ; // dt en milisegundos

    if (dt< 1500) return _yawMoment;

	//DEBUG_sprintf("dt,prev,curr", float (dt), float (previousMillis), float (currentMillis));

    previousMillis = currentMillis;

	const bool wasInitialized = (_status & STATUS_INITIALIZED) != 0U;
    bool windActive = (_status & STATUS_WIND_ACTIVE) != 0U;

    const bool validMeasurement =
        measurementValid &&
        isFiniteValue(apparentWindSpeed) &&
        apparentWindSpeed >= 0.0f &&
        isFiniteValue(apparentWindAngleRad);

    if (!validMeasurement) {
        _status = STATUS_INPUT_INVALID;

        if (wasInitialized) {
            const float alpha = calculateAlpha(dt, WIND_FF_INVALID_DECAY_TIME_MS);
//		if (hasFrequencyElapsed(
//			TimerRegister::TR_WINDINPUT,
//			1000)) {
//			DEBUG_sprintf("AWS,AWA,Valid",apparentWindSpeed, apparentWindAngleRad, float(measurementValid));
//			DEBUG_sprintf("wasIni, windAct", int(wasInitialized),int(windActive));
//		}

#if WIND_FF_ENABLE_INPUT_FILTER
            _filteredU += alpha * (0.0f - _filteredU);
            _filteredV += alpha * (0.0f - _filteredV);
#endif

            _yawMoment += alpha * (0.0f - _yawMoment);

            if (fabsf(_yawMoment) < MOMENT_EPSILON_NM) {
                _yawMoment = 0.0f;
            }

            _status |= STATUS_INITIALIZED;
        } else {
            reset();
            _status = STATUS_INPUT_INVALID;
        }

        return _yawMoment;
    }

    _status = STATUS_VALID;

    const float normalizedInputAngle =
        normalizeAnglePositive(apparentWindAngleRad);

    float windSpeedSquared = 0.0f;
    float angleForTable = normalizedInputAngle;

#if WIND_FF_ENABLE_INPUT_FILTER
    const float measuredU =
        apparentWindSpeed * cosf(normalizedInputAngle);
    const float measuredV =
        apparentWindSpeed * sinf(normalizedInputAngle);

    if (!wasInitialized) {
        // Initialize directly from the first valid sample to avoid a long
        // artificial startup transient from a zero wind vector.
        _filteredU = measuredU;
        _filteredV = measuredV;
    } else {
        const float alpha = calculateAlpha(dt, WIND_FF_FILTER_TIME_MS);
        _filteredU += alpha * (measuredU - _filteredU);
        _filteredV += alpha * (measuredV - _filteredV);
    }

    windSpeedSquared =
        (_filteredU * _filteredU) + (_filteredV * _filteredV);

    // atan2f is only evaluated after Cartesian filtering, avoiding angular
    // discontinuity around 0/2*pi.
    angleForTable = normalizeAnglePositive(atan2f(_filteredV, _filteredU));
#else
    windSpeedSquared = apparentWindSpeed * apparentWindSpeed;
#endif

    const float startSpeedSquared =
        WIND_FF_START_SPEED_MPS * WIND_FF_START_SPEED_MPS;
    const float stopSpeedSquared =
        WIND_FF_STOP_SPEED_MPS * WIND_FF_STOP_SPEED_MPS;

    // Hysteresis avoids toggling the feedforward around the minimum wind speed.
    if (windActive) {
        if (windSpeedSquared <= stopSpeedSquared) {
            windActive = false;
        }
    } else if (windSpeedSquared >= startSpeedSquared) {
        windActive = true;
    }

    float targetMoment = 0.0f;

    if (!windActive) {
        _status |= STATUS_LOW_WIND;
    } else {
        _status |= STATUS_WIND_ACTIVE;

        const float cn = interpolateCn(angleForTable);
        targetMoment = _kAir * windSpeedSquared * cn;
        if (targetMoment > WIND_FF_MAX_MOMENT_NM) {
            targetMoment = WIND_FF_MAX_MOMENT_NM;
            _status |= STATUS_SATURATED;
        } else if (targetMoment < -WIND_FF_MAX_MOMENT_NM) {
            targetMoment = -WIND_FF_MAX_MOMENT_NM;
            _status |= STATUS_SATURATED;
        }
    }

#if WIND_FF_ENABLE_OUTPUT_FILTER
    if (wasInitialized) {
        const float alpha =
            calculateAlpha(dt, WIND_FF_OUTPUT_FILTER_TIME_MS);
        _yawMoment += alpha * (targetMoment - _yawMoment);
    } else {
        _yawMoment = targetMoment;
    }
#else
    _yawMoment = targetMoment;
#endif

    _status |= STATUS_INITIALIZED;
    return _yawMoment;
}

float WindFeedforward::normalizeAnglePositive(float angleRad)
{
    // Apparent-wind angles are normally already close to the required range,
    // so loops avoid the extra flash/code cost of a general fmodf-based path.
    while (angleRad < 0.0f) {
        angleRad += TWO_PI_F;
    }

    while (angleRad >= TWO_PI_F) {
        angleRad -= TWO_PI_F;
    }

    return angleRad;
}

float WindFeedforward::interpolateCn(float normalizedAngleRad)
{
    // Convert [0, 2*pi) directly to table position [0, 36).
    const float tablePosition = normalizedAngleRad * TABLE_SCALE;

    uint8_t index0 = static_cast<uint8_t>(tablePosition);
    if (index0 >= kTableSize) {
        // Defensive guard against any floating-point rounding at 2*pi.
        index0 = 0U;
    }

    const float fraction = tablePosition - static_cast<float>(index0);
    const uint8_t index1 =
        (index0 + 1U < kTableSize) ? static_cast<uint8_t>(index0 + 1U) : 0U;

    const float cn0 = readCnValue(index0);
    const float cn1 = readCnValue(index1);

    return cn0 + fraction * (cn1 - cn0);
}

float WindFeedforward::readCnValue(uint8_t index)
{
#if defined(__AVR__)
    return pgm_read_float_near(&CN_TABLE[index]);
#else
    return CN_TABLE[index];
#endif
}

float WindFeedforward::calculateAlpha(float dt, float timeConstant)
{
    if (timeConstant <= 0.0f) {
        return 1.0f;
    }

    return dt / (timeConstant + dt);
}

bool WindFeedforward::isFiniteValue(float value)
{
    return !isnan(value) && !isinf(value);
}
