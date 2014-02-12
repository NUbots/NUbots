#include <math.h> //needed for normalisation function
#include <assert.h>
#include "IMUModel.h" //includes armadillo

#include <iostream>

namespace utility {
    namespace math {
        namespace kalman {
            arma::vec::fixed<IMUModel::size> IMUModel::limitState(const arma::vec::fixed<size>& state) {
                // const float pi_2 = 0.5 * M_PI;
                // if(fabs(state(kstates_body_angle_x, 0)) > pi_2 and fabs(state(kstates_body_angle_y, 0)) > pi_2) { // This part checks for the event where a large roll and large pitch puts the robot back upright
                //     state(kstates_body_angle_x, 0) = state(kstates_body_angle_x, 0) - sign(state(kstates_body_angle_x, 0)) * M_PI;
                //     state(kstates_body_angle_y, 0) = state(kstates_body_angle_y, 0) - sign(state(kstates_body_angle_y, 0)) * M_PI;
                //     state(kstates_body_angle_z, 0) = state(kstates_body_angle_z, 0) - sign(state(kstates_body_angle_z, 0)) * M_PI;
                // }

                // Regular unwrapping.
                //state(kstates_body_angle_x, 0) = normaliseAngle(state(kstates_body_angle_x, 0));
                //state(kstates_body_angle_y, 0) = normaliseAngle(state(kstates_body_angle_y, 0));
                //state(kstates_body_angle_z, 0) = normaliseAngle(state(kstates_body_angle_z, 0));

                return state;///arma::norm(state, 2);
            }

            // @brief The process equation is used to update the systems state using the process euquations of the system. 
            // @param sigma_point The sigma point representing a system state. 
            // @param deltaT The amount of time that has passed since the previous update, in seconds. 
            // @param measurement The reading from the rate gyroscope in rad/s used to update the orientation. 
            // @return The new estimated system state.
            arma::vec::fixed<IMUModel::size> IMUModel::timeUpdate(const arma::vec::fixed<size>& state, double deltaT, const arma::vec3& measurement) { 
                //new universal rotation code for gyro (SORA)
                //See: http://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation#Simultaneous_orthogonal_rotation_angle
                arma::vec3 omega = measurement * deltaT;
                double phi = arma::norm(omega, 2);
                if (phi == 0) {
                    return state;
                }
                arma::vec3 unitOmega = omega / phi;
                const auto omegaCrossState = arma::cross(unitOmega,state);
                return state * cos(phi) + omegaCrossState * sin(phi) + unitOmega * arma::dot(unitOmega, state) * (1.0 - cos(phi));
            }


            arma::vec IMUModel::predictedObservation(const arma::vec::fixed<size>& state, const arma::vec& measurement) {
                return state ;
            }


            arma::vec IMUModel::observationDifference(const arma::vec& a, const arma::vec& b) {
                return a - b;
            }

            arma::mat::fixed<IMUModel::size, IMUModel::size> IMUModel::processNoise() {

                return arma::eye(size, size) *1e-6; //std::numeric_limits<double>::epsilon();
            }

        }
    }
}