#ifndef UTILITY_MATH_STATISTICS_STATISTICS_H
#define UTILITY_MATH_STATISTICS_STATISTICS_H

#include <cmath>
#include <vector>

namespace utility
{
    namespace math
    {
        namespace statistics
        {
            // Calculate the mean of the data points.
            template<int n=2>
            arma::vec::fixed<n> calculateMean(const std::vector<arma::vec::fixed<n> >& points)
            {
                arma::vec::fixed<n> mean;
                mean.zeros();

                for (const auto& point : points)
                {
                    mean += point;
                }

                return(mean / points.size());
            }

            // Create a covariance matrix for all points in the window.
            // https://en.wikipedia.org/wiki/Covariance_matrix#Generalization_of_the_variance
            template<int n=2>
            arma::mat::fixed<n, n> calculateCovarianceMatrix(const std::vector<arma::vec::fixed<n> >& points, const arma::vec::fixed<n>& mean)
            {
                arma::mat::fixed<n, n> covariance;
                covariance.zeros();

                for (const auto& point : points)
                {
                    arma::vec::fixed<n> offset = point - mean;
                    covariance += offset * offset.t();
                }

                return(covariance / (points.size() - 1));
            }

            // Create a correlation matrix for all points in the window.
            // https://en.wikipedia.org/wiki/Covariance_matrix#Correlation_matrix
            template<int n=2>
            arma::mat::fixed<n, n> calculateCorrelationMatrix(const arma::mat::fixed<n, n>& covariance)
            {
                arma::mat::fixed<n, n> diag = arma::diagmat(covariance);
                diag.for_each(
                    [] (arma::mat::elem_type& val) -> void
                    {
                        if (val > 0.0)
                        {
                            val = 1.0 / std::sqrt(val);
                        }
                    });

                return(diag * covariance * diag);
            }
        }
    }
}

#endif  // UTILITY_MATH_STATISTICS_STATISTICS_H
