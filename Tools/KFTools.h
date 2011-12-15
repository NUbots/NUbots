#ifndef KFTOOLS_H
#define KFTOOLS_H

#include "Math/Matrix.h"

class KFTools
{
public:
    KFTools();
    static Matrix CovarianceIntersectionMean(Matrix meanA, Matrix CovarianceA, Matrix meanB, Matrix CovarianceB, float weight);
    static Matrix CovarianceIntersectionCovariance(Matrix meanA, Matrix CovarianceA, Matrix meanB, Matrix CovarianceB, float weight);
};

#endif // KFTOOLS_H
