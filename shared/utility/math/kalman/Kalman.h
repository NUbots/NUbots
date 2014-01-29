#include <armadillo>
#include "MultivariateGaussian.h"

#include "UnscentedTransform.h"

template <typename Model> //model is is a template parameter that Kalman also inherits
class Kalman {
private:
    // The model
    Model model;

    // TODO remove this, only contains a matrix
    MultivariateGaussian estimate;

    arma::mat sigmaPoints;
    arma::mat sigmaMean;
    arma::mat x;
    arma::mat d;
    arma::mat c;

    // Does unscented transform logic (sigma points)
    UnscentedTransform unscentedTransform;

    // TODO Who knows
    bool outlierFiltering;
    bool outlierThreshold;

public:
    Kalman() : model(), estimate(model.totalStates()) {}

    void timeUpdate(double delta_t, const arma::mat& measurement, const arma::mat& processNoise, const arma::mat& measurementNoise) {

        auto numSigmaPoints = unscentedTransform.totalSigmaPoints();

        // Calculate the current sigma points, and write to member variable.
        sigmaPoints = unscentedTransform.generateSigmaPoints(estimate.mean(), estimate.covariance());
        
        // Update each sigma point.
        for(uint i = 0; i < numSigmaPoints; ++i) {
            // Write the propagated version of the sigma point
            sigmaPoints.col(i) = model.processEquation(sigmaPoints.col(i), delta_t, measurement);
        }

        // Calculate the new mean and covariance values.
        arma::mat predictedMean = unscentedTransform.calculateMeanFromSigmas(sigmaPoints);
        arma::mat predictedCovariance = unscentedTransform.calculateCovarianceFromSigmas(sigmaPoints, predictedMean) + processNoise;

        model.limitState(predictedMean);
        estimate.setMean(predictedMean);
        estimate.setCovariance(predictedCovariance);

        sigmaMean = estimate.mean();
        sigmaPoints = unscentedTransform.generateSigmaPoints(estimate.mean(), estimate.covariance());

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

        arma::mat updatedMean = sigmaMean + x * c * d;    // Update mean and covariance.
        arma::mat updatedCovariance = x * c * x.t();

        model.limitState(updatedMean);
        estimate.setMean(updatedMean);
        estimate.setCovariance(updatedCovariance);
    }
};