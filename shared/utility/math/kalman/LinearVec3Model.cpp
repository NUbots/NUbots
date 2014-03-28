#include <math.h> //needed for normalisation function
#include <assert.h>
#include "LinearVec3Model.h" //includes armadillo

#include <iostream>
#include <nuclear>

namespace utility {
    namespace math {
        namespace kalman {
            arma::vec::fixed<LinearVec3Model::size> LinearVec3Model::limitState(const arma::vec::fixed<size>& state) {
                return state;            
            }

            
            arma::vec::fixed<LinearVec3Model::size> LinearVec3Model::timeUpdate(const arma::vec::fixed<size>& state, double deltaT, const arma::vec3& dState) {
                return state + deltaT * dState;
            }


            arma::vec LinearVec3Model::predictedObservation(const arma::vec::fixed<size>& state, std::nullptr_t) {
                return state;
            }


            arma::vec LinearVec3Model::observationDifference(const arma::vec& a, const arma::vec& b) {
                return a - b;
            }

            arma::mat::fixed<LinearVec3Model::size, LinearVec3Model::size> LinearVec3Model::processNoise() {
                return arma::eye(size, size) * processNoiseFactor; //std::numeric_limits<double>::epsilon();
            }

        }
    }
}