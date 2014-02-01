#include "SinModel.h"


arma::mat SinModel::processEquation(const arma::mat& state, double deltaT, const arma::mat& measurement) {
    
    arma::mat result(state); // Start at original state.
    result(0, 0) = state(1,0)*asin(measurement(0,0)) - deltaT; // Add measurement + offset.
    result(1, 0) += (deltaT+result(0,0))/asin(measurement(0,0)); //result is the period
    limitState(result);
    return result;
    
}

// The process equation, this describes the transition of the estimate due to time and inputs applied. @param state The state determined frim the previous estimate. @param deltaT The elapsed time since the previous update was performed. @param measurement Measurment data obtained from the inputs to the system. @return The new updated measurement.
arma::mat SinModel::measurementEquation(const arma::mat& state, const arma::mat& measurementArgs) {
    
    arma::mat result = { sin(state(0,0) / state(1,0)) };
    
    return result;
    
}// The measurement equation, this is used to calculate the expected measurement given a state of the system. @param state The estimated state of the system. @param measurementArgs Additional information about the measurement. @return The expected measurment for the given conditions.
arma::mat SinModel::measurementDistance(const arma::mat& measurement1, const arma::mat& measurement2) {
    
    arma::mat result = { arma::accu(arma::abs(measurement1 - measurement2)) };
    
    return result;
};

void SinModel::limitState(arma::mat &state) {
    if (state(0,0) > 1) {
        state(0,0) = 1;
    }
    else if (state(0,0) < -1) {
        state(0,0) = -1;
    }
}

unsigned int SinModel::totalStates() {
    return 1;
}