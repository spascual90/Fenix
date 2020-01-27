#include "Arduino.h"
#include "LinearRegression.h"


LinearRegression::LinearRegression(double min, double max){
  minX = min;
  maxX = max;
}

void LinearRegression::learn(double x, double y){
  if(!fixedN){
    n++;
  }

  if(x < minX){
    return;
  } else if(x > maxX){
    return;
  }

  meanX = meanX + ((x-meanX)/n);
  meanX2 = meanX2 + (((x*x)-meanX2)/n);
  varX = meanX2 - (meanX*meanX);

  meanY = meanY + ((y-meanY)/n);
  meanY2 = meanY2 + (((y*y)-meanY2)/n);
  varY = meanY2 - (meanY*meanY);

  meanXY = meanXY + (((x*y)-meanXY)/n);

  covarXY = meanXY - (meanX*meanY);

  m = covarXY / varX;
  b = meanY-(m*meanX);
}

double LinearRegression::correlation() {
  double stdX = sqrt(varX);
  double stdY = sqrt(varY);
  double stdXstdY = stdX*stdY;
  double correlation;

  if(stdXstdY == 0){
    correlation = 1;
  } else {
    correlation = covarXY / stdXstdY;
  }
  return correlation;
}

double LinearRegression::calculate(double x) {
  return (m*x) + b;
}

void LinearRegression::fixN(double maxN) {
  fixedN = true;
  n = maxN;
}

void LinearRegression::getValues(double values[]){
  values[0] = m;
  values[1] = b;
  values[2] = n;
}

double LinearRegression::getstdX () {
	return sqrt(varX);
}

double LinearRegression::getstdY () {
	return sqrt(varY);
}

