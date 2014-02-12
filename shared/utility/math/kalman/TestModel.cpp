#include "TestModel.h"

// The process equation, this describes the change in state over time. 
//  @param state The internal state of the system. 
//  @param deltaT The elapsed time since the previous update was performed. 
//  @param measurement Time measurment data obtained from the inputs to the system - ie velocity (null if these don't exist for the model). 
//  @return The new updated internal state.
arma::vec::fixed<TestModel::size> TestModel::timeUpdate(const arma::vec::fixed<size>& state, double deltaT, const arma::vec& measurement) {
    
    arma::vec::fixed<size> result(state);

    result += deltaT * measurement;
    
    return result;
    
}

// The measurement equation, this is used to calculate the expected observation for the current state. This lets us compare it to real observations.
// NOTE: there are allowed to be multiple measurement equations for different observations - ie obs(goalpost) != obs(centrecircle)
//  @param state The estimated internal state of the system. 
//  @param measurementArgs Additional information about the measurement (currently unused). 
//  @return The expected observation for the given state.
arma::vec TestModel::predictedObservation(const arma::vec::fixed<size>& state, const arma::mat& measurementArgs) {

    return state;
    
}

//A generic L1 distance calculation - fast for single state systems
//XXX: pretty sure measurementDistance should return a scalar - check original code
arma::vec TestModel::observationDifference(const arma::vec& a, const arma::vec& b) {
    
    return a - b;
}


//A generic L1 distance calculation - fast for single state systems
//XXX: pretty sure measurementDistance should return a scalar - check original code
arma::mat::fixed<TestModel::size, TestModel::size> TestModel::processNoise() {
    return arma::zeros(size, size);
}

//this limits the part of the state which is an angle to [-pi,pi]
arma::vec::fixed<TestModel::size> TestModel::limitState(const arma::vec::fixed<size>& state) {
    return state;
}