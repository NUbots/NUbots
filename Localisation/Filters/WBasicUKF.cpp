#include "WBasicUKF.h"
#include "Tools/Math/General.h"
#include "IKFModel.h"
#include <sstream>

WBasicUKF::WBasicUKF(IKFModel *model): IWeightedKalmanFilter(model), m_unscented_transform(model->totalStates())
{
    init();
}

WBasicUKF::WBasicUKF(const WBasicUKF& source): IWeightedKalmanFilter(source), m_unscented_transform(source.m_unscented_transform)
{
    m_outlier_filtering_enabled = source.m_outlier_filtering_enabled;
    m_outlier_threshold = source.m_outlier_threshold;
    m_weighting_enabled = source.m_weighting_enabled;
    m_filter_weight = source.m_filter_weight;
}

WBasicUKF::~WBasicUKF()
{
}

void WBasicUKF::init()
{
    initialiseEstimate(m_estimate);
    m_outlier_filtering_enabled = false;
    m_weighting_enabled = false;
    m_outlier_threshold = 15.f;
    m_filter_weight = 1.f;
}

/*!
 * @brief Performs the time update of the filter.
 * @param deltaT The time that has passed since the previous update.
 * @param measurement The measurement/s (if any) that can be used to measure a change in the system.
 * @param linearProcessNoise The linear process noise that will be added.
 * @return True if the time update was performed successfully. False if it was not.
 */
bool WBasicUKF::timeUpdate(double delta_t, const Matrix& measurement, const Matrix& process_noise, const Matrix& measurement_noise)
{
    const unsigned int total_points = m_unscented_transform.totalSigmaPoints();

    // Calculate the current sigma points, and write to member variable.
    Matrix sigma_points = m_unscented_transform.GenerateSigmaPoints(m_estimate.mean(), m_estimate.covariance());
    Matrix current_point; // temporary storage.

    // update each sigma point.
    for (unsigned int i = 0; i < total_points; ++i)
    {
        current_point = sigma_points.getCol(i);    // Get the sigma point.
        sigma_points.setCol(i, m_model->processEquation(current_point, delta_t, measurement));   // Write the propagated version of it.
    }

    // Calculate the new mean and covariance values.
    Matrix predicted_mean = m_unscented_transform.CalculateMeanFromSigmas(sigma_points);
    Matrix predicted_covariance = m_unscented_transform.CalculateCovarianceFromSigmas(sigma_points, predicted_mean) + process_noise;

    // Set the new mean and covariance values.
    initialiseEstimate(MultivariateGaussian(predicted_mean, predicted_covariance));
    return false;
}

/*!
 * @brief Performs the measurement update of the filter.
 * @param measurement The measurement to be used for the update.
 * @param measurementNoise The linear measurement noise that will be added.
 * @param measurementArgs Any additional information about the measurement, if required.
 * @return True if the measurement update was performed successfully. False if it was not.
 */
bool WBasicUKF::measurementUpdate(const Matrix& measurement, const Matrix& noise, const Matrix& args, unsigned int type)
{
    const unsigned int total_points = m_unscented_transform.totalSigmaPoints();
    const unsigned int numStates = m_model->totalStates();
    const unsigned int totalMeasurements = measurement.getm();

    Matrix sigma_points = m_unscented_transform.GenerateSigmaPoints(m_estimate.mean(), m_estimate.covariance());
    Matrix sigma_mean = m_estimate.mean();

    Matrix current_point; // temporary storage.
    Matrix Yprop(totalMeasurements, total_points);

    // First step is to calculate the expected measurmenent for each sigma point.
    for (unsigned int i = 0; i < total_points; ++i)
    {
        current_point = sigma_points.getCol(i);    // Get the sigma point.
        Yprop.setCol(i, m_model->measurementEquation(current_point, args, type));
    }

    // Now calculate the mean of these measurement sigmas.
    Matrix Ymean = m_unscented_transform.CalculateMeanFromSigmas(Yprop);

    Matrix Pyy(noise);   // measurement noise is added, so just use as the beginning value of the sum.
    Matrix Pxy(numStates, totalMeasurements, false); // initialised as 0

    Matrix cov_weights = m_unscented_transform.covarianceWeights();

    // Calculate the Pyy and Pxy variance matrices.
    for(unsigned int i = 0; i < total_points; ++i)
    {
        double weight = cov_weights[0][i];
        // store difference between prediction and measurement.
        current_point = Yprop.getCol(i) - Ymean;
        // Innovation covariance - Add Measurement noise
        Pyy = Pyy + weight * current_point * current_point.transp();
        // Cross correlation matrix
        Pxy = Pxy + weight * (sigma_points.getCol(i) - sigma_mean) * current_point.transp();    // Important: Use mean from estimate, not current mean.
    }

    const Matrix innovation = m_model->measurementDistance(measurement, Ymean, type);

    // Check for outlier, if outlier return without updating estimate.
    if(evaluateMeasurement(innovation, Pyy-noise, noise) == false)
        return false;

    // Calculate the Kalman filter gain
    Matrix K = Pxy * InverseMatrix(Pyy);

    Matrix new_mean = m_estimate.mean() + K * innovation;
    Matrix new_covariance = m_estimate.covariance() - K*Pyy*K.transp();

    initialiseEstimate(MultivariateGaussian(new_mean, new_covariance));
    return true;
}

void WBasicUKF::initialiseEstimate(const MultivariateGaussian& estimate)
{
    // This is pretty simple.
    // Assign the estimate.
    m_estimate = estimate;

    return;
}

bool WBasicUKF::evaluateMeasurement(const Matrix& innovation, const Matrix& estimate_variance, const Matrix& measurement_variance)
{
    if(!m_outlier_filtering_enabled and !m_weighting_enabled) return true;

    const Matrix innov_transp = innovation.transp();
    const Matrix innov_variance = estimate_variance + measurement_variance;

    if(m_outlier_filtering_enabled)
    {
        float innovation_2 = convDble(innov_transp * InverseMatrix(innov_variance) * innovation);
        if(m_outlier_threshold > 0 and innovation_2 > m_outlier_threshold)
        {
            m_filter_weight *= 0.0005;
            return false;
        }
    }

//    if(m_weighting_enabled)
//    {
//        m_filter_weight *= 1 / ( 1 + convDble(innov_transp * InverseMatrix(measurement_variance) * innovation));
//    }

    if(m_weighting_enabled)
    {
        int measurement_dimensions = measurement_variance.getm();
        const float outlier_probability = 0.05;
        double exp_term = -0.5 * convDble(innovation.transp() * InverseMatrix(innov_variance) *  innovation);
        double fract = 1 / sqrt( pow(2 * mathGeneral::PI, measurement_dimensions) * determinant(innov_variance));
        m_filter_weight *= (1.f - outlier_probability) * fract * exp(exp_term) + outlier_probability;
     }

    return true;
}

std::string WBasicUKF::summary(bool detailed) const
{
    std::stringstream str_strm;
    str_strm << "ID: " << m_id <<  " Weight: " << m_filter_weight << std::endl;
    str_strm << m_estimate.string() << std::endl;
    return str_strm.str();
}

std::ostream& WBasicUKF::writeStreamBinary (std::ostream& output) const
{
    m_model->writeStreamBinary(output);
    m_unscented_transform.writeStreamBinary(output);
    m_estimate.writeStreamBinary(output);
    return output;
}

std::istream& WBasicUKF::readStreamBinary (std::istream& input)
{
    m_model->readStreamBinary(input);
    m_unscented_transform.readStreamBinary(input);
    MultivariateGaussian temp;
    temp.readStreamBinary(input);
    initialiseEstimate(temp);
    return input;
}

