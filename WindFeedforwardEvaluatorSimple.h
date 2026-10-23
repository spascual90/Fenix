#ifndef WIND_FEEDFORWARD_EVALUATOR_SIMPLE_H
#define WIND_FEEDFORWARD_EVALUATOR_SIMPLE_H

#include <Arduino.h>
#include "Fenix_config.h"
#include <GPSport.h>


/**
 * @brief Estimates the wind feedforward scale during steady autopilot operation.
 *
 * Stability is evaluated using only three exponential moving averages:
 * - Unit wind feedforward rudder contribution.
 * - Heading error.
 * - Integral rudder contribution.
 *
 * The class does not allocate dynamic memory, does not retain sample buffers,
 * and does not store a configuration structure in RAM.
 */
class WindFeedforwardEvaluator {
public:
    enum InputFlags : uint8_t {
        INPUT_PILOT_CONTROLLING      = 1U << 0,
        INPUT_WIND_VALID            = 1U << 1,
        INPUT_WIND_ACTIVE           = 1U << 2,
        INPUT_INTEGRATOR_ENABLED    = 1U << 3,
        INPUT_REFERENCE_CHANGED     = 1U << 4,
        INPUT_INTEGRATOR_LIMITED    = 1U << 5,
        INPUT_FEEDFORWARD_SATURATED = 1U << 6,
        INPUT_RUDDER_SATURATED      = 1U << 7
    };

    enum StabilityFlags : uint8_t {
        STABILITY_NONE        = 0U,
        STABILITY_WIND        = 1U << 0,
        STABILITY_HEADING     = 1U << 1,
        STABILITY_INTEGRATION = 1U << 2,
        STABILITY_ALL         = STABILITY_WIND |
                                STABILITY_HEADING |
                                STABILITY_INTEGRATION
    };

    enum RejectionFlags : uint16_t {
        REJECTION_NONE                  = 0U,
        REJECTION_PILOT_INACTIVE        = 1U << 0,
        REJECTION_REFERENCE_CHANGED     = 1U << 1,
        REJECTION_WIND_INVALID          = 1U << 2,
        REJECTION_LOW_WIND              = 1U << 3,
        REJECTION_WIND_UNSTABLE         = 1U << 4,
        REJECTION_HEADING_UNSTABLE      = 1U << 5,
        REJECTION_INTEGRAL_UNSTABLE     = 1U << 6,
        REJECTION_INTEGRAL_LIMITED      = 1U << 7,
        REJECTION_FEEDFORWARD_SATURATED = 1U << 8,
        REJECTION_RUDDER_SATURATED      = 1U << 9,
        REJECTION_LOW_EXCITATION        = 1U << 10,
        REJECTION_SIGN_MISMATCH         = 1U << 11,
        REJECTION_INVALID_SAMPLE        = 1U << 12,
        REJECTION_SCALE_OUT_OF_RANGE    = 1U << 13
    };

    enum Phase : uint8_t {
        PHASE_IDLE = 0U,
        PHASE_QUALIFYING,
        PHASE_MEASURING,
        PHASE_ESTIMATE_READY
    };

    /**
     * @brief Signals required by the evaluator.
     *
     * All angular values are expressed in radians.
     */
    struct Input {
        uint32_t nowMs;
        uint8_t flags;

        float unitWindFeedforwardRudderDeg;
        float headingErrorDeg;
        float integralRudderDeg;
        float appliedWindFeedforwardRudderDeg;
        float currentWindScale;
    };

    /**
     * @brief Evaluation output owned by the caller.
     */
    struct Result {
        Phase phase;
        uint8_t conditionsValid;
        uint8_t estimateAvailable;
        uint8_t stabilityFlags;

        uint16_t rejectionFlags;
        uint16_t sampleCount;
        uint32_t validTimeMs;
        uint32_t stableTimeMs;

        float windMovingAverageDeg;
        float headingMovingAverageDeg;
        float integralMovingAverageDeg;

        float meanStaticRudderDeg;
        float meanAppliedFeedforwardDeg;
        float meanIntegralRudderDeg;
        float Cwind;
        float quality;
    };

    WindFeedforwardEvaluator();

    void reset();

    Phase update(const Input& input, Result& result);

    void acknowledgeEstimate();

    Phase getPhase() const;

    uint16_t getLastRejectionFlags() const;

    static void printInput(const Input& input);

    static void printResult(const Result& result);

    /**
     * @brief Fixed compile-time configuration.
     *
     * These functions return literals embedded in program code. No Config
     * object and no configuration values are stored in RAM.
     */
    static constexpr uint32_t qualificationTimeMs() {
        return 10000UL;
    }

    static constexpr uint32_t measurementTimeMs() {
        return 7000UL;
    }

    static constexpr uint32_t invalidGraceTimeMs() {
        return 500UL;
    }

    static constexpr uint32_t maxSampleIntervalMs() {
        return 1000UL;
    }

    static constexpr float movingAverageTimeConstantMs() {
        return 3000.0f;
    }

    static constexpr float targetFeedforwardShare() {
        return 0.80f;
    }

    static constexpr float minimumWindContributionDeg() {
        return degrees(0.00873f);
    }

    static constexpr float maximumWindMeanRateDegS() {
        return degrees(0.003f);
    }

    static constexpr float maximumHeadingMeanRateDegS() {
        return degrees(0.017f);
    }

    static constexpr float maximumIntegralMeanRateDegS() {
        return degrees(0.001f);
    }

    static constexpr float minimumStaticRudderDeg() {
        return degrees(0.0087f);
    }

    static constexpr float minimumRegressionExcitation() {
        return 0.0001f;
    }

    static constexpr float minimumWindScale() {
        return 0.10f;
    }

    static constexpr float maximumWindScale() {
        return 10.0f;
    }

private:
    enum StateFlags : uint8_t {
        STATE_HAS_LAST_SAMPLE = 1U << 0,
        STATE_INVALID_ACTIVE  = 1U << 1,
        STATE_AVERAGES_READY  = 1U << 2
    };

    Phase _phase;
    uint8_t _stateFlags;
    uint8_t _stabilityFlags;
    uint8_t _reserved;

    uint16_t _sampleCount;
    uint16_t _lastRejectionFlags;

    uint32_t _lastSampleMs;
    uint32_t _invalidSinceMs;
    uint32_t _validTimeMs;
    uint32_t _stableTimeMs;

    float _windMean;
    float _headingMean;
    float _integralMean;

    float _weightS;
    float _appliedFeedforwardSum;
    float _integralSum;
    float _currentScaleSum;
    float _regressionXX;
    float _regressionXY;

    void clearMeasurement();

    void startQualification(const Input& input);

    void startMeasurement();

    void abortEvaluation(uint16_t rejectionFlags);

    uint16_t getInstantRejectionFlags(const Input& input) const;

    uint8_t updateMovingAverages(const Input& input, uint32_t dtMs);

    void accumulateMeasurement(const Input& input, uint32_t dtMs);

    uint16_t getEstimateRejectionFlags() const;

    void fillResult(uint8_t conditionsValid, Result& result) const;

    float getQuality() const;

    static bool isFiniteInput(const Input& input);

    static float absoluteFloat(float value);

    static float maximumFloat(float a, float b);

    static float clampFloat(float value, float minimum, float maximum);
};

#endif
