#include "SinModel.h"

// The process equation, this describes the change in state over time. 
//  @param state The internal state of the system. 
//  @param deltaT The elapsed time since the previous update was performed. 
//  @param measurement Time measurment data obtained from the inputs to the system - ie velocity (null if these don't exist for the model). 
//  @return The new updated internal state.
arma::vec::fixed<size> SinModel::processEquation(const arma::vec::fixed<size>& state, double deltaT, const arma::vec::fixed<size>& measurement) {
    
    arma::vec::fixed<size> result(state); // Start at original state.
    result(0, 0) = state(0,0)+deltaT/state(1,0); // Add measurement + offset.
    limitState(result);
    return result;
    
}

// The measurement equation, this is used to calculate the expected observation for the current state. This lets us compare it to real observations.
// NOTE: there are allowed to be multiple measurement equations for different observations - ie obs(goalpost) != obs(centrecircle)
//  @param state The estimated internal state of the system. 
//  @param measurementArgs Additional information about the measurement (currently unused). 
//  @return The expected observation for the given state.
arma::mat SinModel::measurementEquation(const arma::vec::fixed<size>& state, const arma::mat& measurementArgs) {
    
    arma::mat result = { sin(state(0,0)) };
    
    return result;
    
}

//A generic L1 distance calculation - fast for single state systems
//XXX: pretty sure measurementDistance should return a scalar - check original code
arma::mat SinModel::measurementDistance(const arma::mat& measurement1, const arma::mat& measurement2) {
    
    arma::mat result = { arma::accu(arma::abs(measurement1 - measurement2)) };
    
    return result;
}

//this limits the part of the state which is an angle to [-pi,pi]
void SinModel::limitState(arma::mat &state) {
    if (state(0,0) > 3.1415926) { //XXX: replace with arma const for pi
        state(0,0) -= 2*3.1415926;
    }
    else if (state(0,0) < -3.1415926) {
        state(0,0) += 2*3.1415926;
    }
}