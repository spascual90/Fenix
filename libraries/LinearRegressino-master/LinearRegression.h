#ifndef LinearRegression_h
#define LinearRegression_h

#include "Arduino.h"

class LinearRegression
{
  public:
    LinearRegression(double min, double max);
    void learn(double x, double y);
    double calculate(double x);
    double correlation();
    void fixN(double maxN);
    void getValues(double values[]);
    double getstdX ();
    double getstdY ();
  private:
    double meanX = 0;
    double meanX2 = 0; //mean x²
    double varX = 0;
    double meanY = 0;
    double meanY2 = 0; //mean y²
    double varY = 0;
    double meanXY = 0; //mean x*y
    double covarXY = 0;
    double n = 0;
    bool fixedN = false;
    double minX;
    double maxX;
    // m*x + b = y;
    double m = 0;
    double b = 0;
    double r = 0; //correlation
};

#endif
