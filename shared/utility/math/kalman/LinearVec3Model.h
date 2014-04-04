#ifndef UTILITY_MATH_KALMAN_LINEARVEC3MODEL_H
#define UTILITY_MATH_KALMAN_LINEARVEC3MODEL_H

/* Inertial Motion Unit*/
#include <armadillo>
namespace utility {
    namespace math {
        namespace kalman {

            class LinearVec3Model {
                // Number of dimensions
                // State is velocity of torso relative to the robot local ground matrix
                // 
            public:            
                static constexpr size_t size = 3;
                
                LinearVec3Model() {} // empty constructor
                
                arma::vec::fixed<size> timeUpdate(const arma::vec::fixed<size>& state, double deltaT, const arma::vec3& measurement);
                
                arma::vec predictedObservation(const arma::vec::fixed<size>& state, std::nullptr_t);
                
                arma::vec observationDifference(const arma::vec& a, const arma::vec& b);
                
                arma::vec::fixed<size> limitState(const arma::vec::fixed<size>& state);
                
                arma::mat::fixed<size, size> processNoise();

                const double processNoiseFactor = 1e-6;
            };


        }
    }
}
#endif