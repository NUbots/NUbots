#ifndef TESTMODEL_H
#define TESTMODEL_H

#include <armadillo>

class TestModel {

public:
    // Number of dimensions
    static constexpr size_t size = 1;
    
    TestModel() {} // empty constructor
    
    arma::vec::fixed<size> timeUpdate(const arma::vec::fixed<size>& state, double deltaT, const arma::vec& measurement);
    
    arma::vec predictedObservation(const arma::vec::fixed<size>& state, const arma::mat& measurementArgs);
    
    arma::vec observationDifference(const arma::vec& a, const arma::vec& b);
    
    arma::vec::fixed<size> limitState(const arma::vec::fixed<size>& state);
    
    arma::mat::fixed<size, size> processNoise();
    
    unsigned int totalStates();
};

#endif