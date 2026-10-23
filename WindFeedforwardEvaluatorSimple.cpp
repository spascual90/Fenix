#include "WindFeedforwardEvaluatorSimple.h"

#include <math.h>

namespace {
constexpr float MS_TO_SECONDS = 0.001f;
constexpr float MINIMUM_DENOMINATOR = 1.0e-9f;
}

WindFeedforwardEvaluator::WindFeedforwardEvaluator()
    : _phase(PHASE_IDLE),
      _stateFlags(0U),
      _stabilityFlags(STABILITY_NONE),
      _reserved(0U),
      _sampleCount(0U),
      _lastRejectionFlags(REJECTION_NONE),
      _lastSampleMs(0U),
      _invalidSinceMs(0U),
      _validTimeMs(0U),
      _stableTimeMs(0U),
      _windMean(0.0f),
      _headingMean(0.0f),
      _integralMean(0.0f),
      _weightS(0.0f),
      _appliedFeedforwardSum(0.0f),
      _integralSum(0.0f),
      _currentScaleSum(0.0f),
      _regressionXX(0.0f),
      _regressionXY(0.0f) {
}

void WindFeedforwardEvaluator::reset() {
    _phase = PHASE_IDLE;
    _stateFlags = 0U;
    _stabilityFlags = STABILITY_NONE;
    _reserved = 0U;

    _sampleCount = 0U;
    _lastRejectionFlags = REJECTION_NONE;

    _lastSampleMs = 0U;
    _invalidSinceMs = 0U;
    _validTimeMs = 0U;
    _stableTimeMs = 0U;

    _windMean = 0.0f;
    _headingMean = 0.0f;
    _integralMean = 0.0f;

    clearMeasurement();
}

void WindFeedforwardEvaluator::acknowledgeEstimate() {
    reset();
}

WindFeedforwardEvaluator::Phase WindFeedforwardEvaluator::getPhase() const {
    return _phase;
}

uint16_t WindFeedforwardEvaluator::getLastRejectionFlags() const {
    return _lastRejectionFlags;
}

WindFeedforwardEvaluator::Phase WindFeedforwardEvaluator::update(
    const Input& input,
    Result& result) {

    if (_phase == PHASE_ESTIMATE_READY) {
        fillResult(1U, result);
        return _phase;
    }

    const uint16_t instantFlags = getInstantRejectionFlags(input);

    if (instantFlags != REJECTION_NONE) {
    	//DEBUG_sprintf("rejected", int(instantFlags));
        _lastRejectionFlags = instantFlags;
        _stabilityFlags = STABILITY_NONE;
        _stableTimeMs = 0U;

        const uint16_t immediateAbortMask =
            REJECTION_PILOT_INACTIVE |
            REJECTION_REFERENCE_CHANGED |
            REJECTION_INTEGRAL_LIMITED |
            REJECTION_FEEDFORWARD_SATURATED |
            REJECTION_RUDDER_SATURATED |
            REJECTION_INVALID_SAMPLE;

        if ((instantFlags & immediateAbortMask) != 0U) {
            abortEvaluation(instantFlags);
        } else if (_phase == PHASE_MEASURING) {
            if ((_stateFlags & STATE_INVALID_ACTIVE) == 0U) {
                _invalidSinceMs = input.nowMs;
                _stateFlags |= STATE_INVALID_ACTIVE;
            }

            if ((input.nowMs - _invalidSinceMs) > invalidGraceTimeMs()) {
                abortEvaluation(instantFlags);
            }
        }

        _lastSampleMs = input.nowMs;
        _stateFlags |= STATE_HAS_LAST_SAMPLE;

        fillResult(0U, result);
        return _phase;
    }

    _lastRejectionFlags = REJECTION_NONE;

    if (_phase == PHASE_IDLE) {
        startQualification(input);
        fillResult(1U, result);
        return _phase;
    }

    if ((_stateFlags & STATE_HAS_LAST_SAMPLE) == 0U) {
        _lastSampleMs = input.nowMs;
        _stateFlags |= STATE_HAS_LAST_SAMPLE;
        fillResult(1U, result);
        return _phase;
    }

    const uint32_t dtMs = input.nowMs - _lastSampleMs;
    _lastSampleMs = input.nowMs;

    if (dtMs == 0U) {
        fillResult(1U, result);
        return _phase;
    }

    if (dtMs > maxSampleIntervalMs()) {
        abortEvaluation(REJECTION_INVALID_SAMPLE);
        fillResult(0U, result);
        return _phase;
    }

    _stabilityFlags = updateMovingAverages(input, dtMs);

    if (_stabilityFlags == STABILITY_ALL) {
        _stableTimeMs += dtMs;
        _stateFlags &= static_cast<uint8_t>(~STATE_INVALID_ACTIVE);
    } else {
        _stableTimeMs = 0U;

        if (_phase == PHASE_MEASURING) {
            if ((_stateFlags & STATE_INVALID_ACTIVE) == 0U) {
                _invalidSinceMs = input.nowMs;
                _stateFlags |= STATE_INVALID_ACTIVE;
            }

            if ((input.nowMs - _invalidSinceMs) > invalidGraceTimeMs()) {
                uint16_t stabilityRejection = REJECTION_NONE;

                if ((_stabilityFlags & STABILITY_WIND) == 0U) {
                    stabilityRejection |= REJECTION_WIND_UNSTABLE;
                }

                if ((_stabilityFlags & STABILITY_HEADING) == 0U) {
                    stabilityRejection |= REJECTION_HEADING_UNSTABLE;
                }

                if ((_stabilityFlags & STABILITY_INTEGRATION) == 0U) {
                    stabilityRejection |= REJECTION_INTEGRAL_UNSTABLE;
                }

                abortEvaluation(stabilityRejection);
                fillResult(0U, result);
                return _phase;
            }
        }
    }

    if (_phase == PHASE_QUALIFYING) {
        if (_stableTimeMs >= qualificationTimeMs()) {
            startMeasurement();
        }
    } else if (_phase == PHASE_MEASURING &&
               _stabilityFlags == STABILITY_ALL) {

        accumulateMeasurement(input, dtMs);
        _validTimeMs += dtMs;

        if (_validTimeMs >= measurementTimeMs()) {
            const uint16_t estimateFlags = getEstimateRejectionFlags();

            if (estimateFlags == REJECTION_NONE) {
                _phase = PHASE_ESTIMATE_READY;
                _lastRejectionFlags = REJECTION_NONE;
            } else {
                abortEvaluation(estimateFlags);
            }
        }
    }

    fillResult(1U, result);
    return _phase;
}

void WindFeedforwardEvaluator::clearMeasurement() {
    _sampleCount = 0U;
    _validTimeMs = 0U;

    _weightS = 0.0f;
    _appliedFeedforwardSum = 0.0f;
    _integralSum = 0.0f;
    _currentScaleSum = 0.0f;
    _regressionXX = 0.0f;
    _regressionXY = 0.0f;
}

void WindFeedforwardEvaluator::startQualification(const Input& input) {
    _phase = PHASE_QUALIFYING;
    _stateFlags = STATE_HAS_LAST_SAMPLE | STATE_AVERAGES_READY;
    _stabilityFlags = STABILITY_NONE;

    _lastSampleMs = input.nowMs;
    _invalidSinceMs = 0U;
    _stableTimeMs = 0U;

    _windMean = input.unitWindFeedforwardRudderDeg;
    _headingMean = input.headingErrorDeg;
    _integralMean = input.integralRudderDeg;

    clearMeasurement();
}

void WindFeedforwardEvaluator::startMeasurement() {
    _phase = PHASE_MEASURING;
    _stateFlags &= static_cast<uint8_t>(~STATE_INVALID_ACTIVE);
    _invalidSinceMs = 0U;
    _stableTimeMs = 0U;
    clearMeasurement();
}

void WindFeedforwardEvaluator::abortEvaluation(uint16_t rejectionFlags) {
    _phase = PHASE_IDLE;
    _stateFlags = 0U;
    _stabilityFlags = STABILITY_NONE;

    _lastRejectionFlags = rejectionFlags;
    _lastSampleMs = 0U;
    _invalidSinceMs = 0U;
    _stableTimeMs = 0U;

    _windMean = 0.0f;
    _headingMean = 0.0f;
    _integralMean = 0.0f;

    clearMeasurement();
}

uint16_t WindFeedforwardEvaluator::getInstantRejectionFlags(
    const Input& input) const {

    if (!isFiniteInput(input)) {
        return REJECTION_INVALID_SAMPLE;
    }

    uint16_t flags = REJECTION_NONE;

    if ((input.flags & INPUT_PILOT_CONTROLLING) == 0U) {
        flags |= REJECTION_PILOT_INACTIVE;
    }

    if ((input.flags & INPUT_REFERENCE_CHANGED) != 0U) {
        flags |= REJECTION_REFERENCE_CHANGED;
    }

    if ((input.flags & INPUT_WIND_VALID) == 0U ||
        (input.flags & INPUT_WIND_ACTIVE) == 0U) {
        flags |= REJECTION_WIND_INVALID;
    }

    if (absoluteFloat(input.unitWindFeedforwardRudderDeg) <
        minimumWindContributionDeg()) {
        flags |= REJECTION_LOW_WIND;
    }

    if ((input.flags & INPUT_INTEGRATOR_ENABLED) == 0U) {
        flags |= REJECTION_INTEGRAL_UNSTABLE;
    }

    if ((input.flags & INPUT_INTEGRATOR_LIMITED) != 0U) {
        flags |= REJECTION_INTEGRAL_LIMITED;
    }

    if ((input.flags & INPUT_FEEDFORWARD_SATURATED) != 0U) {
        flags |= REJECTION_FEEDFORWARD_SATURATED;
    }

    if ((input.flags & INPUT_RUDDER_SATURATED) != 0U) {
        flags |= REJECTION_RUDDER_SATURATED;
    }

    return flags;
}

uint8_t WindFeedforwardEvaluator::updateMovingAverages(
    const Input& input,
    uint32_t dtMs) {

    if ((_stateFlags & STATE_AVERAGES_READY) == 0U) {
        _windMean = input.unitWindFeedforwardRudderDeg;
        _headingMean = input.headingErrorDeg;
        _integralMean = input.integralRudderDeg;
        _stateFlags |= STATE_AVERAGES_READY;
        return STABILITY_NONE;
    }

    const float dtFloatMs = static_cast<float>(dtMs);
    const float dtS = dtFloatMs * MS_TO_SECONDS;
    const float alpha =
        dtFloatMs / (movingAverageTimeConstantMs() + dtFloatMs);

    const float previousWindMean = _windMean;
    const float previousHeadingMean = _headingMean;
    const float previousIntegralMean = _integralMean;

    _windMean += alpha *
        (input.unitWindFeedforwardRudderDeg - _windMean);
    _headingMean += alpha *
        (input.headingErrorDeg - _headingMean);
    _integralMean += alpha *
        (input.integralRudderDeg - _integralMean);

    const float inverseDtS = 1.0f / maximumFloat(dtS, MINIMUM_DENOMINATOR);

    const float windMeanRate =
        absoluteFloat(_windMean - previousWindMean) * inverseDtS;
    const float headingMeanRate =
        absoluteFloat(_headingMean - previousHeadingMean) * inverseDtS;
    const float integralMeanRate =
        absoluteFloat(_integralMean - previousIntegralMean) * inverseDtS;

//    if (hasFrequencyElapsed(
//    	TimerRegister::TR_WINDFFavg,
//    	5000)) {
//        DEBUG_sprintf("w,h,i", windMeanRate*100.0f, headingMeanRate*100.0f, integralMeanRate*100.0f);
//    }

    uint8_t flags = STABILITY_NONE;

    if (windMeanRate <= maximumWindMeanRateDegS()) {
        flags |= STABILITY_WIND;
    }

    if (headingMeanRate <= maximumHeadingMeanRateDegS()) {
        flags |= STABILITY_HEADING;
    }

    if (integralMeanRate <= maximumIntegralMeanRateDegS()) {
        flags |= STABILITY_INTEGRATION;
    }

    return flags;
}

void WindFeedforwardEvaluator::accumulateMeasurement(
    const Input& input,
    uint32_t dtMs) {

    const float weightS = static_cast<float>(dtMs) * MS_TO_SECONDS;
    const float staticRudder =
        input.appliedWindFeedforwardRudderDeg +
        input.integralRudderDeg;
    const float targetFeedforward =
        targetFeedforwardShare() * staticRudder;

    _weightS += weightS;
    _appliedFeedforwardSum +=
        weightS * input.appliedWindFeedforwardRudderDeg;
    _integralSum += weightS * input.integralRudderDeg;
    _currentScaleSum += weightS * input.currentWindScale;

    _regressionXX +=
        weightS *
        input.unitWindFeedforwardRudderDeg *
        input.unitWindFeedforwardRudderDeg;

    _regressionXY +=
        weightS *
        input.unitWindFeedforwardRudderDeg *
        targetFeedforward;

    if (_sampleCount < UINT16_MAX) {
        ++_sampleCount;
    }
}

uint16_t WindFeedforwardEvaluator::getEstimateRejectionFlags() const {
    if (_weightS <= MINIMUM_DENOMINATOR ||
        _sampleCount < 2U ||
        _regressionXX < minimumRegressionExcitation()) {
        return REJECTION_LOW_EXCITATION;
    }

    uint16_t flags = REJECTION_NONE;
    const float inverseWeight = 1.0f / _weightS;
    const float meanAppliedFeedforward =
        _appliedFeedforwardSum * inverseWeight;
    const float meanIntegral = _integralSum * inverseWeight;
    const float meanStaticRudder =
        meanAppliedFeedforward + meanIntegral;

    if (absoluteFloat(meanStaticRudder) < minimumStaticRudderDeg()) {
        flags |= REJECTION_LOW_EXCITATION;
    }

    if (_regressionXY <= 0.0f) {
        flags |= REJECTION_SIGN_MISMATCH;
    }

    const float estimatedScale = _regressionXY / _regressionXX;

    if (estimatedScale < minimumWindScale() ||
        estimatedScale > maximumWindScale()) {
        flags |= REJECTION_SCALE_OUT_OF_RANGE;
    }

    return flags;
}

void WindFeedforwardEvaluator::fillResult(
    uint8_t conditionsValid,
    Result& result) const {

    result.phase = _phase;
    result.conditionsValid = conditionsValid;
    result.estimateAvailable =
        (_phase == PHASE_ESTIMATE_READY) ? 1U : 0U;
    result.stabilityFlags = _stabilityFlags;

    result.rejectionFlags = _lastRejectionFlags;
    result.sampleCount = _sampleCount;
    result.validTimeMs = _validTimeMs;
    result.stableTimeMs = _stableTimeMs;

    result.windMovingAverageDeg = _windMean;
    result.headingMovingAverageDeg = _headingMean;
    result.integralMovingAverageDeg = _integralMean;

    result.meanStaticRudderDeg = 0.0f;
    result.meanAppliedFeedforwardDeg = 0.0f;
    result.meanIntegralRudderDeg = 0.0f;
    result.Cwind = 1.0f;
    result.quality = getQuality();

    if (_weightS <= MINIMUM_DENOMINATOR) {
        return;
    }

    const float inverseWeight = 1.0f / _weightS;

    result.meanAppliedFeedforwardDeg =
        _appliedFeedforwardSum * inverseWeight;
    result.meanIntegralRudderDeg =
        _integralSum * inverseWeight;
    result.meanStaticRudderDeg =
        result.meanAppliedFeedforwardDeg +
        result.meanIntegralRudderDeg;

//    result.targetFeedforwardRudderRad =
//        targetFeedforwardShare() *
//        result.meanStaticRudderRad;
//    result.targetIntegralRudderRad =
//        (1.0f - targetFeedforwardShare()) *
//        result.meanStaticRudderRad;

    if (absoluteFloat(result.meanAppliedFeedforwardDeg) > MINIMUM_DENOMINATOR) {
    	result.Cwind = 0.8 * (result.meanStaticRudderDeg/result.meanAppliedFeedforwardDeg);

    }

//    result.measuredWindScale =
//        _currentScaleSum * inverseWeight;
//
//    if (_regressionXX > MINIMUM_DENOMINATOR) {
//        result.estimatedWindScale =
//            _regressionXY / _regressionXX;
//        result.scaleCorrection =
//            result.estimatedWindScale -
//            result.measuredWindScale;
//    }
}

float WindFeedforwardEvaluator::getQuality() const {
    if (_phase == PHASE_IDLE) {
        return 0.0f;
    }

    uint8_t stableCount = 0U;

    if ((_stabilityFlags & STABILITY_WIND) != 0U) {
        ++stableCount;
    }

    if ((_stabilityFlags & STABILITY_HEADING) != 0U) {
        ++stableCount;
    }

    if ((_stabilityFlags & STABILITY_INTEGRATION) != 0U) {
        ++stableCount;
    }

    return static_cast<float>(stableCount) / 3.0f;
}

bool WindFeedforwardEvaluator::isFiniteInput(const Input& input) {
    return isfinite(input.unitWindFeedforwardRudderDeg) &&
           isfinite(input.headingErrorDeg) &&
           isfinite(input.integralRudderDeg) &&
           isfinite(input.appliedWindFeedforwardRudderDeg) &&
           isfinite(input.currentWindScale);
}

float WindFeedforwardEvaluator::absoluteFloat(float value) {
    return (value < 0.0f) ? -value : value;
}

float WindFeedforwardEvaluator::maximumFloat(float a, float b) {
    return (a > b) ? a : b;
}

float WindFeedforwardEvaluator::clampFloat(
    float value,
    float minimum,
    float maximum) {

    if (value < minimum) {
        return minimum;
    }

    if (value > maximum) {
        return maximum;
    }

    return value;
}

void WindFeedforwardEvaluator::printInput(const Input& input) {
    //DEBUG_sprintf("nowMs", float(input.nowMs));
    //DEBUG_sprintf("flags", float(input.flags));

    DEBUG_sprintf(
        "INPUT\nOp",
        static_cast<uint8_t>(
            (input.flags & INPUT_PILOT_CONTROLLING) != 0U));
    DEBUG_sprintf(
        "wVal",
        static_cast<uint8_t>(
            (input.flags & INPUT_WIND_VALID) != 0U));
    DEBUG_sprintf(
        "wAct",
        static_cast<uint8_t>(
            (input.flags & INPUT_WIND_ACTIVE) != 0U));
    DEBUG_sprintf(
        "iEn",
        static_cast<uint8_t>(
            (input.flags & INPUT_INTEGRATOR_ENABLED) != 0U));
    DEBUG_sprintf(
        "refCh",
        static_cast<uint8_t>(
            (input.flags & INPUT_REFERENCE_CHANGED) != 0U));
    DEBUG_sprintf(
        "iLim",
        static_cast<uint8_t>(
            (input.flags & INPUT_INTEGRATOR_LIMITED) != 0U));
    DEBUG_sprintf(
        "FFSat",
        static_cast<uint8_t>(
            (input.flags & INPUT_FEEDFORWARD_SATURATED) != 0U));
    DEBUG_sprintf(
        "rudSat",
        static_cast<uint8_t>(
            (input.flags & INPUT_RUDDER_SATURATED) != 0U));

    DEBUG_sprintf(
        "WFFr",
        input.unitWindFeedforwardRudderDeg);
    DEBUG_sprintf(
        "hErr",
        input.headingErrorDeg);
    DEBUG_sprintf(
        "iRud",
        input.integralRudderDeg);
    DEBUG_sprintf(
        "wFFR",
        input.appliedWindFeedforwardRudderDeg);
    //DEBUG_sprintf(
    //    "currentWindScale",
    //    input.currentWindScale);
}

void WindFeedforwardEvaluator::printResult(const Result& result) {
    DEBUG_sprintf("RESULT\nph", static_cast<uint8_t>(result.phase));
    DEBUG_sprintf("cVal", result.conditionsValid);
    DEBUG_sprintf("eAv", result.estimateAvailable);
    //DEBUG_sprintf("stabilityFlags", result.stabilityFlags);

    DEBUG_sprintf(
        "wSt",
        static_cast<uint8_t>(
            (result.stabilityFlags & STABILITY_WIND) != 0U));
    DEBUG_sprintf(
        "hSt",
        static_cast<uint8_t>(
            (result.stabilityFlags & STABILITY_HEADING) != 0U));
    DEBUG_sprintf(
        "iSt",
        static_cast<uint8_t>(
            (result.stabilityFlags & STABILITY_INTEGRATION) != 0U));

    DEBUG_sprintf("rFl", int(result.rejectionFlags));
    DEBUG_sprintf("sCnt", int(result.sampleCount));
    DEBUG_sprintf("vMs", float(result.validTimeMs));
    DEBUG_sprintf("sMs", float(result.stableTimeMs));

    DEBUG_sprintf(
        "wMovAvg",
        result.windMovingAverageDeg);
    DEBUG_sprintf(
        "hMovAvg",
        result.headingMovingAverageDeg);
    DEBUG_sprintf(
        "iMovAvg",
        result.integralMovingAverageDeg);
    DEBUG_sprintf(
        "mR",
        result.meanStaticRudderDeg);
    DEBUG_sprintf(
        "mFF",
        result.meanAppliedFeedforwardDeg);
    DEBUG_sprintf(
        "mIR",
        result.meanIntegralRudderDeg);
    DEBUG_sprintf(
        "Cw",
        result.Cwind);
    DEBUG_sprintf("q", result.quality);
}
