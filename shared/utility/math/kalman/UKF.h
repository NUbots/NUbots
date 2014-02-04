#include <armadillo>

template <typename Model> //model is is a template parameter that Kalman also inherits
class UKF {
private:
    // The model
    Model model;
    
    // Dimension types for vectors and square matricies
    using ModelVec = arma::vec::fixed<Model::size>;
    using ModelMat = arma::mat::fixed<Model::size, Model::size>;
    
    // Our estimate and covariance
    ModelVec mean;
    ModelMat covariance;
    
    // Our sigma points for UKF
    ModelVec sigmaMean;
    arma::mat sigmaPoints;
    
    // Don't know what these are
    arma::mat x;
    arma::mat d;
    ModelMat c;
    
    // The mean and covariance weights
    arma::mat meanWeights;
    arma::mat covarianceWeights;
    
    // Parameters for UKF sigma points
    uint l;
    double alpha;
    double kappa;
    double beta;
    
    // Calculated variables for UKF sigma points
    double lambda;
    uint numSigmaPoints;
    
    // TODO Who knows
    bool outlierFiltering;
    bool outlierThreshold;
    
    arma::mat generateSigmaPoints(const ModelVec& mean, const ModelMat& covariance) {
        
        // Allocate memory for our points
        arma::mat points(Model::size, numSigmaPoints);
        
        // Our first row is always the mean
        points.col(0) = mean;
        
        // Get our cholskey decomposition
        arma::mat chol = arma::chol(covarianceWeights * covariance);
        
        for (uint i = 1; i < Model::size + 1; ++i) {
            int negIndex = i + Model::size;
            
            arma::vec deviation = chol.col(i - 1);
            
            points.col(i) = mean + deviation;
            points.col(negIndex) = mean - deviation;
        }
        
        return points;
        
        // Matrix of number of points x dimensionality
        
        // First point is the mean
        
        // Calculate the cholskey matrix of the covariance
        
        // Points from 1 to number of points
        
        /*
        
        const unsigned int numPoints = totalSigmaPoints();
        const arma::mat current_mean = mean;
        const unsigned int num_states = mean.n_rows; //ox--mean.getm()
        arma::mat points(num_states, numPoints); //ox , false
        points.col(0) = current_mean; //ox---points.setCol(0, current_mean); (changes the 0th column in points to column vecotr: current_mean) // First sigma point is the current mean with no deviation
        
        arma::mat weightedCov = covarianceSigmaWeight() * covariance;
        arma::mat sqtCovariance = chol(weightedCov); //originally cholskey() for matrix class instead of arma::mat class
        arma::mat deviation;
        
        for(unsigned int i = 1; i < num_states + 1; i++){
            int negIndex = i+num_states;
            deviation = sqtCovariance.col(i-1); //ox deviation = sqtCovariance.getCol(i - 1);              // Get deviation from weighted covariance
            points.col(i) = (current_mean + deviation); //ox points.setCol(i, (current_mean + deviation));         // Add mean + deviation
            points.col(negIndex) = (current_mean - deviation); //ox points.setCol(negIndex, (current_mean - deviation));  // Add mean - deviation
        }
        return points;*/
    }
    
    ModelVec meanFromSigmas(const arma::mat& sigmaPoints) const
    {
        
        // Sigma points * mean weights gives a vector return, therefore mean weights must be a vector
        
        
        return sigmaPoints * meanWeights;
    }
    
    arma::mat covarianceFromSigmas(const arma::mat& sigmaPoints, const ModelVec& mean) const
    {
        const unsigned int numPoints = totalSigmaPoints();
        const unsigned int numStates = mean.n_rows; //ox getm();
        
        
        ///
        ModelMat covariance;
        
        for(int i = 0; i < numSigmaPoints; ++i) {
            
        }
        
        ///
        
        arma::mat covariance(numStates, numStates);  //ox , false ... Blank covariance matrix.
        arma::mat diff;
        for(unsigned int i = 0; i < numPoints; ++i)
        {
            const double weight = m_covariance_weights(0, i); //ox [0][i]
            diff = sigmaPoints.col(i) - mean; //ox sigmaPoints.getCol(i)
            covariance = covariance + weight*diff*diff.t(); //ox ..t*diff*diff.transp();
        }
        return covariance;
    }

public:
    UKF(uint l, double alpha = 1e-2, double kappa = 0.f, double beta = 2.f) :
    l(l), alpha(alpha), kappa(kappa), beta(beta), lambda(pow(alpha, 2) * (l + kappa) - l) {
    
        // Calculate the weights
    
    // TODO calculate alpha beta kappa lambda number of sigma points
    
    
    
    }

    void timeUpdate(double delta_t, const arma::vec::fixed<Model::size>& measurement, const arma::mat& processNoise, const arma::mat& measurementNoise) {

        // TODO currently measurement noise is not used at all
        
        // Generate our sigma points
        sigmaPoints = generateSigmaPoints();
        
        // Write the propagated version of the sigma point
        for(uint i = 0; i < sigmaPoints.n_rows; ++i) {
            sigmaPoints.col(i) = model.processEquation(sigmaPoints.col(i), delta_t, measurement);
        }

        // Calculate the new mean and covariance values.
        mean = model.limitState(calculateMeanFromSigmas(sigmaPoints));
        covariance = covarianceFromSigmas(sigmaPoints, predictedMean) + processNoise;
        
        // Re calculate our sigma points
        sigmaMean = mean;
        sigmaPoints = generateSigmaPoints();
        
        // Calculate our TODO what are these again?
        
        c = defaultC;
        
        c = arma::diagmat(unscentedTransform.covarianceWeights());
        d = arma::mat(numSigmaPoints, 1);
        x = sigmaPoints.each_col() - sigmaMean;
    
        //x = arma::mat(estimate.totalStates(), numSigmaPoints);
        //for(uint i = 0; i < numSigmaPoints; ++i) {
        //    x.col(i) = (sigmaPoints.col(i) - sigmaMean);
        //}
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