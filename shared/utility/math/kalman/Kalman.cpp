#include "Utils.h"
#include "Kalman.h" //should not be here - its a template class
//#include "IMUModel.h" //IKFModel.h
//#include "Tools/Math/General.h"
//*

template <typename Model> //: m_estimate(source.estimate()), m_unscented_transform(source.m_unscented_transform)
Kalman<Model>::Kalman(const Kalman<Model>& source): m_estimate(source.estimate()), m_unscented_transform(source.m_unscented_transform) { // IKalmanFilter
        m_model = source.m_model->Clone();
        m_outlier_filtering_enabled = source.m_outlier_filtering_enabled;
        m_outlier_threshold = source.m_outlier_threshold;

        m_previous_decisions = source.m_previous_decisions;  //IWeightedKalmanFilter
        m_parent_id = source.id(); //IWeightedKalmanFilter
        m_active = source.m_active; //IWeightedKalmanFilter
        m_id = m_id; //IWeightedKalmanFilter
        m_creation_time = source.m_creation_time; //IWeightedKalmanFilter
        //m_parent_history_buffer = source.m_parent_history_buffer; //IWeightedKalmanFilter
        
        m_weighting_enabled = source.m_weighting_enabled;
        m_filter_weight = source.m_filter_weight;
        m_sigma_points = source.m_sigma_points;
        m_sigma_mean = source.m_sigma_mean;
        m_C = source.m_C;
        m_d = source.m_d;
        m_X = source.m_X;
}



template <typename Model>
bool Kalman<Model>::timeUpdate(double delta_t, const arma::mat& measurement, const arma::mat& process_noise, const arma::mat& measurement_noise) { //@brief Performs the time update of the filter. @param deltaT The time that has passed since the previous update. @param measurement The measurement/s (if any) that can be used to measure a change in the system. @param linearProcessNoise The linear process noise that will be added. @return True if the time update was performed successfully. False if it was not.

    const unsigned int total_points = m_unscented_transform.totalSigmaPoints();

    // Calculate the current sigma points, and write to member variable.
    m_sigma_points = m_unscented_transform.GenerateSigmaPoints(m_estimate.mean(), m_estimate.covariance());
    arma::mat currentPoint; // temporary storage.

    // update each sigma point.
    for (unsigned int i = 0; i < total_points; ++i) {
        currentPoint = m_sigma_points.col(i);    // Get the sigma point.
        m_sigma_points.col(i) = m_model->processEquation(currentPoint, delta_t, measurement); // Write the propagated version of it.
    }

    // Calculate the new mean and covariance values.
    arma::mat predictedMean = m_unscented_transform.CalculateMeanFromSigmas(m_sigma_points);
    arma::mat predictedCovariance = m_unscented_transform.CalculateCovarianceFromSigmas(m_sigma_points, predictedMean) + process_noise;

    // Set the new mean and covariance values.
    MultivariateGaussian new_estimate = m_estimate;

    if(not duti::isMatrixValid(predictedCovariance)) {
        std::cout << "predicted covariance is not valid!" << std::endl;
        std::cout << "ID: " << id() << " " ;
        std::cout << "Weight: " << this->getFilterWeight() << " ";
        std::cout << "Mean:\n" << m_estimate.mean() << std::endl;
        std::cout << "Covariance:\n" << m_estimate.covariance() << std::endl;
        std::cout << "Sqrt Covariance:\n" << arma::chol(m_estimate.covariance()) << std::endl;
        std::cout << "m_sigma_points:\n" << m_sigma_points << std::endl;

        std::cout << "cov = [";
        arma::mat cov = m_estimate.covariance();
        for(unsigned int i = 0; i < cov.n_rows; ++i) {
            if(i!=0) std::cout << "; ";
            for(unsigned int j = 0; j < cov.n_cols; ++j) { //pretty sure theres a mistake in the original, should be .n_cols, not .n_rows
                if(j!=0) std::cout << ",";
                std::cout << std::setprecision(9) << cov(i, j);
            }
        }
        std::cout << "]" << std::endl;
    }

    m_model->limitState(predictedMean);
    new_estimate.setMean(predictedMean);
    new_estimate.setCovariance(predictedCovariance);
    initialiseEstimate(new_estimate);
    return true;
}

   
// @brief Performs the measurement update of the filter. @param measurement The measurement to be used for the update. @param measurementNoise The linear measurement noise that will be added. @param measurementArgs Any additional information about the measurement, if required. @return True if the measurement update was performed successfully. False if it was not.
template <typename Model>
bool Kalman<Model>::measurementUpdate(const arma::mat& measurement, const arma::mat& noise, const arma::mat& args, unsigned int type) {

    const unsigned int total_points = m_unscented_transform.totalSigmaPoints();
    const unsigned int totalMeasurements = measurement.n_rows; //.getm()
    arma::mat current_point; // temporary storage.
    arma::mat Yprop(totalMeasurements, total_points);

    // First step is to calculate the expected measurement for each sigma point.
    for (unsigned int i = 0; i < total_points; ++i) {
        current_point = m_sigma_points.col(i);    // Get the sigma point.
        Yprop.col(i)  = m_model->measurementEquation(current_point, args, type);
    }

    // Now calculate the mean of these measurement sigmas.
    arma::mat Ymean = m_unscented_transform.CalculateMeanFromSigmas(Yprop);
    arma::mat Y(totalMeasurements, total_points,false);
    arma::mat Pyy(noise);

    const arma::mat cov_weights = m_unscented_transform.covarianceWeights();
    // Calculate the Y vector.
    for(unsigned int i = 0; i < total_points; ++i) {
        arma::mat point = Yprop.col(i) - Ymean;
        Y.col(i) = point;
        Pyy = Pyy + cov_weights(0, i) * point * point.t();
    }


    arma::mat Ytransp = Y.t();    // Calculate the new C and d values.
    const arma::mat innovation = m_model->measurementDistance(measurement, Ymean, type);
    if(evaluateMeasurement(innovation, Pyy - noise, noise) == false)    // Check for outlier, if outlier return without updating estimate.
        return false;

    m_C = m_C - m_C.t() * Ytransp * (noise + Y*m_C*Ytransp).i() * Y * m_C; //ox previously InverseMatrix(noise + ...)
    m_d = m_d + Ytransp * noise.i() * innovation;

    arma::mat updated_mean = m_sigma_mean + m_X * m_C * m_d;    // Update mean and covariance.
    arma::mat updated_covariance = m_X * m_C * m_X.t();

    if(not duti::isMatrixValid(updated_covariance) ) { //not updated_covariance.isValid()
        std::cout << "ID: " << id() << std::endl;
        std::cout << "Sigma mean:\n" << m_X << std::endl;
        std::cout << "measurement:\n" << measurement << std::endl;
        std::cout << "noise:\n" << noise << std::endl;
        std::cout << "args:\n" << args << std::endl;
        std::cout << "type:n" << type << std::endl;
        std::cout << "m_sigma_points:\n" << m_sigma_points << std::endl;
        std::cout << "innovation:\n" << innovation << std::endl;
        std::cout << "m_C After:\n" << m_C << std::endl;
        std::cout << "m_d after:\n" << m_d << std::endl;
        std::cout << "New mean:\n" << updated_mean << std::endl;
        std::cout << "New covariance:\n" << updated_covariance << std::endl;
    }

    m_model->limitState(updated_mean);
    m_estimate.setMean(updated_mean);
    m_estimate.setCovariance(updated_covariance);
    return true;
} //*/

template <typename Model>
void Kalman<Model>::initialiseEstimate(const MultivariateGaussian& estimate) {
    // This is more complicated than you might expect because of all of the buffered values that
    // must be kept up to date.
    const unsigned int total_points = m_unscented_transform.totalSigmaPoints();
    const unsigned int num_states = m_estimate.totalStates();


    m_estimate = estimate;    // Assign the estimate.


    m_sigma_mean = estimate.mean();    // Redraw sigma points to cover this new estimate.
    m_sigma_points = m_unscented_transform.GenerateSigmaPoints(m_estimate.mean(), m_estimate.covariance());

    m_C = arma::diagmat(m_unscented_transform.covarianceWeights());    // Initialise the variables for sequential measurement updates. m_C = Matrix(total_points, total_points, true);
    m_d = arma::mat(total_points, 1);

    m_X = arma::mat(num_states, total_points);    // Calculate X vector.
    for(unsigned int i = 0; i < total_points; ++i) {
        m_X.col(i) = (m_sigma_points.col(i) - m_sigma_mean);
    }

    return;
}

template <typename Model>
double Kalman<Model>::convDble(arma::mat X) {
    return X(0, 0); //cannot use A(n, m) notation directly when assigning a matrix calculation to a float
}

template <typename Model>
bool Kalman<Model>::evaluateMeasurement(const arma::mat& innovation, const arma::mat& estimate_variance, const arma::mat& measurement_variance) {
    if(!m_outlier_filtering_enabled and !m_weighting_enabled) return true;

    arma::mat innov_transp = innovation.t();
    arma::mat innov_variance = estimate_variance + measurement_variance;
    
    if(m_outlier_filtering_enabled) {
        float innovation_2 = convDble(innov_transp * innov_variance.i() * innovation);
        if(m_outlier_threshold > 0 and innovation_2 > m_outlier_threshold) {
            m_filter_weight *= 0.0005;
            return false;
        }
    }

    if(m_weighting_enabled) {
        int measurement_dimensions = measurement_variance.n_rows; //.getm()
        const float outlier_probability = 0.05;
        double exp_term = -0.5 * convDble(innovation.t() * innov_variance.i() *  innovation);
        double fract = 1 / sqrt( pow(2 * duti::PI, measurement_dimensions) * arma::det(innov_variance));
        m_filter_weight *= (1.f - outlier_probability) * fract * exp(exp_term) + outlier_probability;
    }

    return true;
}

template <typename Model>
std::string Kalman<Model>::summary(bool detailed) const {
    std::stringstream str_strm;
    str_strm << "ID: " << m_id <<  " Weight: " << m_filter_weight << std::endl;
    str_strm << m_estimate.string() << std::endl;
    return str_strm.str();
}

/* //new system does not read/write binary stream.
//template <typename Model>
//std::ostream& Kalman<Model>::writeStreamBinary (std::ostream& output) const {
//    m_model->writeStreamBinary(output);
//    m_unscented_transform.writeStreamBinary(output);
//    m_estimate.writeStreamBinary(output);
//    output.write((char*)&m_weighting_enabled, sizeof(m_weighting_enabled));
//    output.write((char*)&m_filter_weight, sizeof(m_filter_weight));
//    return output;
//}

//template <typename Model>
//std::istream& Kalman<Model>::readStreamBinary (std::istream& input) {
//    m_model->readStreamBinary(input);
//    m_unscented_transform.readStreamBinary(input);
//    MultivariateGaussian temp;
//    temp.readStreamBinary(input);
//    input.read((char*)&m_weighting_enabled, sizeof(m_weighting_enabled));
//    input.read((char*)&m_filter_weight, sizeof(m_filter_weight));
//    initialiseEstimate(temp);
//    return input;
//}

//*/