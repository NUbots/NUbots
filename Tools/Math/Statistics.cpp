#include "Statistics.h"
#include "Matrix.h"
#include "General.h"
#include <cmath>

using namespace mathGeneral;

/*!
  Calculation if the multi-variate normal distribution. Taken from: http://en.wikipedia.org/wiki/Multivariate_normal_distribution
  @param covariance matrix
  @param individual value
  @param mean value.
  @return The calculated multi-variate normal distribution.
  */

float MultiVariateNormalDistribution(const Matrix& covariance, const Matrix& value, const Matrix& mean)
{
    const float det = determinant(covariance);
    const double k = covariance.getm();
    const Matrix inv_cov = InverseMatrix(covariance);
    const Matrix delta = value - mean;

    return 1 / sqrt(pow(2*PI, k) * det) * exp(-0.5*convDble(delta.transp()*inv_cov*delta));
}
