// Savitzky-Golay filter
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "GolayDerivative.h"

namespace utility {
namespace math {
    namespace filter {

        constexpr double GolayCoeff<0, 1>::Coefficients[];
        constexpr double GolayCoeff<0, 1>::Normalization;

        constexpr double GolayCoeff<0, 5>::Coefficients[];
        constexpr double GolayCoeff<0, 5>::Normalization;

        constexpr double GolayCoeff<0, 7>::Coefficients[];
        constexpr double GolayCoeff<0, 7>::Normalization;

        constexpr double GolayCoeff<0, 9>::Coefficients[];
        constexpr double GolayCoeff<0, 9>::Normalization;

        constexpr double GolayCoeff<1, 5>::Coefficients[];
        constexpr double GolayCoeff<1, 5>::Normalization;

        constexpr double GolayCoeff<1, 7>::Coefficients[];
        constexpr double GolayCoeff<1, 7>::Normalization;

        constexpr double GolayCoeff<1, 9>::Coefficients[];
        constexpr double GolayCoeff<1, 9>::Normalization;

        constexpr double GolayCoeff<2, 5>::Coefficients[];
        constexpr double GolayCoeff<2, 5>::Normalization;

        constexpr double GolayCoeff<2, 7>::Coefficients[];
        constexpr double GolayCoeff<2, 7>::Normalization;

        constexpr double GolayCoeff<2, 9>::Coefficients[];
        constexpr double GolayCoeff<2, 9>::Normalization;
    }  // namespace filter
}  // namespace math
}  // namespace utility
