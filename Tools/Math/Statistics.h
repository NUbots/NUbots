#ifndef STATISTICS_H
#define STATISTICS_H
#include "Matrix.h"

float MultiVariateNormalDistribution(const Matrix& covariance, const Matrix& value, const Matrix& mean);

#endif // STATISTICS_H
