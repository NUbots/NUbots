#ifndef SINMODEL_H
#define SINMODEL_H

#include <armadillo>

class SinModel {
    //float period;
    //float angle;
public:
    
    // Number of dimensions
    static constexpr size_t size = 1;
    
    SinModel() {} // empty constructor
    
    arma::mat processEquation(const arma::vec::fixed<size> state, double deltaT, const arma::vec::fixed<size>& measurement);
    
    arma::mat measurementEquation(const arma::vec::fixed<size>& state, const arma::mat& measurementArgs);
    
    arma::mat measurementDistance(const arma::mat& measurement1, const arma::mat& measurement2);
    
    void limitState(arma::mat &state);
    
    unsigned int totalStates();
};

#endif