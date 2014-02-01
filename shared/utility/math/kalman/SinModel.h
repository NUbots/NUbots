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
    
    arma::mat processEquation(const arma::mat& state, double deltaT,
                              const arma::mat& measurement);
    
    arma::mat measurementEquation(const arma::mat& state,
                                  const arma::mat& measurementArgs);
    
    arma::mat measurementDistance(const arma::mat& measurement1,
                                  const arma::mat& measurement2);
    
    void limitState(arma::mat &state);
    
    unsigned int totalStates();
};

#endif