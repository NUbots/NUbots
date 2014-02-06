#ifndef UTILITY_MATH_KALMAN_IMUMODEL_H
#define UTILITY_MATH_KALMAN_IMUMODEL_H

/* Inertial Motion Unit*/
#include <armadillo>
namespace utility {
    namespace math {
        namespace kalman {

            class IMUModel {
                // Number of dimensions
                // Store unit vector pointing globally down (gravity)
                // Coordinate system (same as CM730 coords):
                //                  x = forward, out of chest
                //                  y = leftwards
                //                  z = robot upward, towards head
            public:
                static constexpr size_t size = 3;
                
                IMUModel() {} // empty constructor
                
                arma::vec::fixed<size> timeUpdate(const arma::vec::fixed<size>& state, double deltaT, const arma::vec3& measurement);
                
                arma::vec predictedObservation(const arma::vec::fixed<size>& state, const arma::vec& measurement);
                
                arma::vec observationDifference(const arma::vec& a, const arma::vec& b);
                
                arma::vec::fixed<size> limitState(const arma::vec::fixed<size>& state);
                
                arma::mat::fixed<size, size> processNoise();
            };


        }
    }
}
#endif