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
            Eigen::Matrix<double, n, 1> calculateMean(const std::vector<Eigen::Matrix<double, n, 1> >& points)
            {
                Eigen::Matrix<double, n, 1> mean;
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
            arma::mat::fixed<n, n> calculateCovarianceMatrix(const std::vector<Eigen::Matrix<double, n, 1> >& points, const Eigen::Matrix<double, n, 1>& mean)
            {
                arma::mat::fixed<n, n> covariance;
                covariance.zeros();

                for (const auto& point : points)
                {
                    Eigen::Matrix<double, n, 1> offset = point - mean;
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
