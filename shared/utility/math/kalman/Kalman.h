#include <armadillo>

#include "UnscentedTransform.h"

template <typename Model> //model is is a template parameter that Kalman also inherits
class Kalman {
private:
    // The model
    Model model;
    
    // Our estimate and covariance
    arma::vec::fixed<Model::size> mean;
    arma::mat::fixed<Model::size, Model::size> covariance;
    
    // Our sigma variables for UKF
    arma::vec::fixed<Model::size> sigmaMean;
    arma::mat sigmaPoints;
    
    // Don't know what these are
    arma::mat x;
    arma::mat d;
    
    
    // The covariance weights
    arma::mat c;

    // Does unscented transform logic (sigma points)
    UnscentedTransform unscentedTransform;

    // TODO Who knows
    bool outlierFiltering;
    bool outlierThreshold;

public:
    Kalman() : model() {}

    void timeUpdate(double delta_t, const arma::vec::fixed<Model::size>& measurement, const arma::mat& processNoise, const arma::mat& measurementNoise) {

        // Calculate the current sigma points, and write to member variable.
        sigmaPoints = unscentedTransform.generateSigmaPoints(estimate.mean(), estimate.covariance());
        
        // Write the propagated version of the sigma point
        for(uint i = 0; i < sigmaPoints.n_rows; ++i) {
            sigmaPoints.col(i) = model.processEquation(sigmaPoints.col(i), delta_t, measurement);
        }

        // Calculate the new mean and covariance values.
        mean = model.limitState(unscentedTransform.calculateMeanFromSigmas(sigmaPoints));
        covariance = unscentedTransform.calculateCovarianceFromSigmas(sigmaPoints, predictedMean) + processNoise;
        
        // Re calculate our sigma points
        sigmaMean = mean;
        sigmaPoints = unscentedTransform.generateSigmaPoints(estimate.mean(), estimate.covariance());

        // Calculate our TODO what are these again?
        c = arma::diagmat(unscentedTransform.covarianceWeights());
        d = arma::mat(numSigmaPoints, 1);
        x = arma::mat(estimate.totalStates(), numSigmaPoints);
        
        for(uint i = 0; i < numSigmaPoints; ++i) {
            x.col(i) = (sigmaPoints.col(i) - sigmaMean);
        }
    }

    void measurementUpdate(const arma::mat& measurement, const arma::mat& noise, const arma::mat& args) {

        auto totalPoints = unscentedTransform.totalSigmaPoints();
        auto totalMeasurements = measurement.n_rows; //.getm()

        // Want to know what this is...
        arma::mat Yprop(totalMeasurements, totalPoints);

        // First step is to calculate the expected measurement for each sigma point.
        for(uint i = 0; i < totalPoints; ++i) {
            Yprop.col(i)  = model.measurementEquation(sigmaPoints.col(i), args);
        }

        // Now calculate the mean of these measurement sigmas.
        arma::mat Ymean = unscentedTransform.calculateMeanFromSigmas(Yprop);
        arma::mat Y(totalMeasurements, totalPoints);
        arma::mat Pyy(noise);

        const arma::mat covWeights = unscentedTransform.covarianceWeights();

        // Calculate the Y vector.
        for(uint i = 0; i < totalPoints; ++i) {

            // Get our mean normalized point
            arma::mat point = Yprop.col(i) - Ymean;

            Y.col(i) = point;
            Pyy = Pyy + covWeights(0, i) * point * point.t();
        }

        // Transpose the Y matrix

        arma::mat Ytransp = Y.t();
        const arma::mat innovation = model.measurementDistance(measurement, Ymean);

        // Check for outlier, if outlier return without updating estimate.
        // TODO implement this
        //if(evaluateMeasurement(innovation, Pyy - noise, noise) == false)   
        //    return false;

        c = c - c.t() * Ytransp * (noise + Y * c * Ytransp).i() * Y * c; //ox previously InverseMatrix(noise + ...)
        d = d + Ytransp * noise.i() * innovation;

        // Update our mean and covariance
        mean = model.limitState(sigmaMean + x * c * d);
        covariance = x * c * x.t();
    }
};