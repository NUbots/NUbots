#include <math.h> //needed for normalisation function
#include <assert.h>
#include "AdaptiveIMUModel.h" //includes armadillo

#include <iostream>

namespace utility {
    namespace math {
        namespace kalman {
            arma::vec::fixed<AdaptiveIMUModel::size> AdaptiveIMUModel::limitState(const arma::vec::fixed<size>& state) {
                
                arma::mat stateMatrix = arma::reshape(state,3,3);
                double normDown = arma::norm(stateMatrix.col(0),2);
                if(normDown == 0){
                    //TODO: RESTART FILTER
                    return state;
                }                
                
                stateMatrix.col(0) = stateMatrix.col(0) / normDown;     //Normalise down    

                double dotProd = arma::dot(stateMatrix.col(0), stateMatrix.col(1));
                stateMatrix.col(1) = stateMatrix.col(1) - stateMatrix.col(0)*dotProd;       //Orthogonalise forward and down     
                
                double normForward = arma::norm(stateMatrix.col(1),2);     //Normalise forward
                if(normForward == 0){
                    //TODO: RESTART FILTER
                    return state;
                }  
                stateMatrix.col(1) = stateMatrix.col(1) / normForward;
                
                
                return static_cast<arma::vec::fixed<size>>(arma::vectorise(stateMatrix));            
            }

            // @brief The process equation is used to update the systems state using the process euquations of the system. 
            // @param sigma_point The sigma point representing a system state. 
            // @param deltaT The amount of time that has passed since the previous update, in seconds. 
            // @param measurement The reading from the rate gyroscope in rad/s used to update the orientation. 
            // @return The new estimated system state.
            arma::vec::fixed<AdaptiveIMUModel::size> AdaptiveIMUModel::timeUpdate(const arma::vec::fixed<size>& state, double deltaT, const arma::vec3& measurement) { 
                //new universal rotation code for gyro (SORA)
                //See: http://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation#Simultaneous_orthogonal_rotation_angle
                arma::mat stateMatrix = arma::reshape(state,3,3);
                arma::vec3 omega = ( measurement - stateMatrix.col(2) )* deltaT;    //Offset applied
                double phi = arma::norm(omega, 2);
                if (phi == 0) {
                    return state;
                }
                arma::vec3 unitOmega = omega / phi;
                
                
                const auto omegaCrossStateDown = arma::cross(unitOmega,stateMatrix.col(0));
                stateMatrix.col(0) = stateMatrix.col(0) * cos(phi) + omegaCrossStateDown * sin(phi) + unitOmega * arma::dot(unitOmega, stateMatrix.col(0)) * (1.0 - cos(phi));
                
                const auto omegaCrossStateForward = arma::cross(unitOmega,stateMatrix.col(1));
                stateMatrix.col(1) = stateMatrix.col(1) * cos(phi) + omegaCrossStateForward * sin(phi) + unitOmega * arma::dot(unitOmega, stateMatrix.col(1)) * (1.0 - cos(phi));
                
                return static_cast<arma::vec::fixed<size>>(arma::vectorise(stateMatrix));
            }


            arma::vec AdaptiveIMUModel::predictedObservation(const arma::vec::fixed<size>& state, const arma::vec& measurement) {
                return state.rows(0,2) * 9.807;
            }


            arma::vec AdaptiveIMUModel::observationDifference(const arma::vec& a, const arma::vec& b) {
                return a - b;
            }

            arma::mat::fixed<AdaptiveIMUModel::size, AdaptiveIMUModel::size> AdaptiveIMUModel::processNoise() {
                return arma::eye(size, size) * processNoiseFactor; //std::numeric_limits<double>::epsilon();
            }

        }
    }
}