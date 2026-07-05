#ifndef ONLINE_NOMOTO_IDENTIFIER_H
#define ONLINE_NOMOTO_IDENTIFIER_H

#include <Arduino.h>

/**
 * @class OnlineNomotoIdentifier
 * @brief Online least-squares identifier for a first-order Nomoto yaw model.
 *
 * Identified discrete model:
 *
 *     r[k+1] = A * r[k] + B * delta[k]
 *
 * Equivalent continuous Nomoto model:
 *
 *     T * r_dot + r = K * delta
 *
 * Conversion:
 *
 *     T = -dt / ln(A)
 *     K = B / (1 - A)
 *
 * Units:
 *   - delta: real measured rudder angle, rad
 *   - r: yaw rate, rad/s
 *   - T: seconds
 *   - K: (rad/s)/rad
 *
 * No arrays, no dynamic memory, constant RAM usage.
 */
class OnlineNomotoIdentifier {
public:
    OnlineNomotoIdentifier();

    void reset();

    void setForgettingFactor(float lambda);
    void setMinSamples(unsigned long minSamples);
    void setMinDeterminant(float minDet);
    void setPhysicalLimits(float minT, float maxT, float maxAbsK);
    void setUseDeltaAverage(bool enabled);

    bool update(unsigned long timeMs, float deltaNow, float rNow);
    bool updateDt(float dt, float deltaNow, float rNow);

    bool isValid() const;
    bool hasPreviousSample() const;

    float getA() const;
    float getB() const;
    float getT() const;
    float getK() const;
    float getKff() const;
    float getDtMean() const;

    unsigned long getSampleCount() const;
    float getWeightSum() const;

    float predictNextR(float rCurrent, float deltaCurrent) const;

    float getLastResidual() const;
    float getResidualRms() const;

private:
    // Normal-equation accumulated sums:
    // [S00 S01] [A] = [q0]
    // [S01 S11] [B]   [q1]
    float S00;
    float S01;
    float S11;
    float q0;
    float q1;
    float yy;

    float dtSum;
    float weightSum;

    float A;
    float B;
    float T;
    float K;
    float Kff;

    unsigned long lastTimeMs;
    float lastR;
    float lastDelta;

    float lastResidual;
    float residualRms;

    float forgettingFactor;
    float minDeterminant;
    float minT;
    float maxT;
    float maxAbsK;
    unsigned long minimumSamples;
    bool useDeltaAverage;

    unsigned long sampleCount;
    bool valid;
    bool hasPrevious;

    bool addRegressionSample(float dt, float deltaNow, float rNow);
    bool solve();
    bool isFiniteFloat(float value) const;
};

#endif // ONLINE_NOMOTO_IDENTIFIER_H
