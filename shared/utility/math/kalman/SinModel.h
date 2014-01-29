#ifndef SINMODEL_H
#define SINMODEL_H

#include <armadillo>

class SinModel {
public:

    SinModel(); // empty constructor
   
    arma::mat processEquation(const arma::mat& state, double deltaT, const arma::mat& measurement);      // The process equation, this describes the transition of the estimate due to time and inputs applied. @param state The state determined frim the previous estimate. @param deltaT The elapsed time since the previous update was performed. @param measurement Measurment data obtained from the inputs to the system. @return The new updated measurement.
    arma::mat measurementEquation(const arma::mat& state, const arma::mat& measurementArgs);    // The measurement equation, this is used to calculate the expected measurement given a state of the system. @param state The estimated state of the system. @param measurementArgs Additional information about the measurement. @return The expected measurment for the given conditions.
    arma::mat measurementDistance(const arma::mat& measurement1, const arma::mat& measurement2);

    void limitState(arma::mat &state);
    unsigned int totalStates();
};

#endif