#include "WSrBasicUKF.h"
#include "Tools/Math/General.h"
#include "IKFModel.h"
#include <sstream>

WSrBasicUKF::WSrBasicUKF(IKFModel *model): IWeightedKalmanFilter(model), m_estimate(model->totalStates()), m_unscented_transform(model->totalStates())
{
    init();
}

WSrBasicUKF::WSrBasicUKF(const WSrBasicUKF& source): IWeightedKalmanFilter(source.m_model), m_estimate(source.m_estimate), m_unscented_transform(source.m_unscented_transform)
{
    init();
    m_filter_weight = source.m_filter_weight;
}

WSrBasicUKF::~WSrBasicUKF()
{
}

void WSrBasicUKF::init()
{
    Matrix covariance = m_estimate.covariance();
    covariance[0][0] = 1;
    covariance[1][1] = 1;
    covariance[2][2] = 1;
    m_estimate.setCovariance(covariance);
    initialiseEstimate(m_estimate);
    m_weighting_enabled = false;
    m_filter_weight = 1.f;

    m_sqrt_covariance_weights = Matrix(m_unscented_transform.covarianceWeights());
    for(unsigned int m = 0; m < m_sqrt_covariance_weights.getm(); ++m)
    {
        for(unsigned int n = 0; n < m_sqrt_covariance_weights.getm(); ++n)
        {
            m_sqrt_covariance_weights[m][n] = sqrt(m_sqrt_covariance_weights[m][n]);
        }
    }
}

Matrix WSrBasicUKF::GenerateSqrtSigmaPoints() const
{
    const unsigned int numPoints = m_unscented_transform.totalSigmaPoints();
    const Matrix current_mean = m_estimate.mean();
    const unsigned int num_states = m_estimate.totalStates();
    Matrix points(num_states, numPoints, false);

    const float weight = sqrt(m_unscented_transform.covarianceSigmaWeight());
    points.setCol(0, current_mean); // First sigma point is the current mean with no deviation
    Matrix deviation;
    for(unsigned int i = 1; i < num_states + 1; i++){
        int negIndex = i+num_states;
        deviation =  weight * m_sqrt_covariance.getCol(i - 1);        // Get deviation from weighted covariance
        points.setCol(i, (current_mean + deviation));         // Add mean + deviation
        points.setCol(negIndex, (current_mean - deviation));  // Add mean - deviation
    }
    return points;
}


bool WSrBasicUKF::timeUpdate(double delta_t, const Matrix& measurement, const Matrix& process_noise, const Matrix& measurement_noise)
{
    const unsigned int total_points = m_unscented_transform.totalSigmaPoints();
    // Calculate the current sigma points, and write to member variable.
    Matrix sigma_points = GenerateSqrtSigmaPoints();

    Matrix current_point; // temporary storage.

    // propogate each sigma point.
    for (unsigned int i = 0; i < total_points; ++i)
    {
        current_point = sigma_points.getCol(i);    // Get the sigma point.
        sigma_points.setCol(i, m_model->processEquation(current_point, delta_t, measurement));   // Write the propagated version of it.
    }

    // Calculate the new mean and covariance values.
    Matrix predicted_mean = m_unscented_transform.CalculateMeanFromSigmas(sigma_points);
    //Matrix predicted_covariance = CalculateCovarianceFromSigmas(sigma_points, predicted_mean) + process_noise;

    Matrix centre_x = predicted_mean;//sigma_points.getCol(0);

    Matrix temp(sigma_points.getm(), sigma_points.getn()-1);
    for (unsigned int i = 1; i < total_points; ++i)
    {
        temp.setCol(i-1, m_sqrt_covariance_weights[0][i] * (sigma_points.getCol(i)-centre_x));
    }

    //Matrix concat = horzcat(temp, cholesky(measurement_noise));
    Matrix concat = temp;

    Matrix Sy = QR_Householder(concat.transp());

    Matrix U0 = m_sqrt_covariance_weights[0][0] * (sigma_points.getCol(0)-centre_x);
    Sy = CholeskyUpdate(Sy, U0, mathGeneral::sign(m_unscented_transform.Wc(0)));

    // Set the new mean and covariance values.
    Matrix predicted_covariance = Sy * Sy.transp();
    m_model->limitState(predicted_mean);
    m_estimate.setMean(predicted_mean);
    m_estimate.setCovariance(predicted_covariance);
    m_sqrt_covariance = Sy;

    return true;
}

bool WSrBasicUKF::measurementUpdate(const Matrix& measurement, const Matrix& noise, const Matrix& args, unsigned int type)
{
    const unsigned int total_points = m_unscented_transform.totalSigmaPoints();
    const unsigned int numStates = m_model->totalStates();
    const unsigned int totalMeasurements = measurement.getm();

    Matrix sigma_points = GenerateSqrtSigmaPoints();
    Matrix sigma_mean = m_estimate.mean();

    Matrix current_point; // temporary storage.
    Matrix Yprop(totalMeasurements, total_points);

    // First step is to calculate the expected measurmenent for each sigma point.
    for (unsigned int i = 0; i < total_points; ++i)
    {
        current_point = sigma_points.getCol(i);    // Get the sigma point.
        Yprop.setCol(i, m_model->measurementEquation(current_point, args, type));
    }

    Matrix sqrt_noise = cholesky(noise).transp();

    // Now calculate the mean of these measurement sigmas.
    Matrix Ymean = m_unscented_transform.CalculateMeanFromSigmas(Yprop);

    Matrix Pxy(numStates, totalMeasurements, false); // initialised as 0

    const Matrix cov_weights = m_unscented_transform.covarianceWeights();

    // Calculate the Pyy and Pxy variance matrices.
    for(unsigned int i = 0; i < total_points; ++i)
    {
        double weight = cov_weights[0][i];
        // store difference between prediction and measurement.
        current_point = Yprop.getCol(i) - Ymean;
        // Cross correlation matrix
        Pxy = Pxy + weight * (sigma_points.getCol(i) - sigma_mean) * current_point.transp();    // Important: Use mean from estimate, not current mean.
    }

    Matrix Ydif(totalMeasurements, total_points-1);
    for(unsigned int i = 1; i < total_points; ++i)
    {
        current_point = m_sqrt_covariance_weights[0][i] * (Yprop.getCol(i) - Ymean);
        Ydif.setCol(i-1, current_point);
    }

    Matrix concat = horzcat(Ydif, sqrt_noise);

    Matrix Sy = QR_Householder(concat.transp());
    Sy = CholeskyUpdate(Sy, m_sqrt_covariance_weights[0][0]*(Yprop.getCol(0) - Ymean), mathGeneral::sign(cov_weights[0][0]));

    const Matrix innovation = m_model->measurementDistance(measurement, Ymean, type);
    // Check for outlier, if outlier return without updating estimate.
    if(evaluateMeasurement(innovation, Sy*Sy.transp(), noise) == false) // Y * Y^T is the estimate variance, by definition.
        return false;

    Matrix SyInv = InverseMatrix(Sy);
    // Calculate the Kalman filter gain
    Matrix K = Pxy * SyInv * SyInv.transp();

    Matrix new_mean = m_estimate.mean() + K * innovation;
    Matrix U = K * Sy.transp();

    Matrix new_sqrt_cov = CholeskyUpdate(m_sqrt_covariance.transp(), U, -1);
    Matrix new_covariance = new_sqrt_cov * new_sqrt_cov.transp();

    m_sqrt_covariance = new_sqrt_cov;
    m_model->limitState(new_mean);
    m_estimate.setMean(new_mean);
    m_estimate.setCovariance(new_covariance);
    return true;
}

void WSrBasicUKF::initialiseEstimate(const MultivariateGaussian& estimate)
{
    // This is pretty simple.
    // Assign the estimate.
    m_estimate = estimate;
    m_sqrt_covariance = cholesky(estimate.covariance());
    return;
}

bool WSrBasicUKF::evaluateMeasurement(const Matrix& innovation, const Matrix& estimate_variance, const Matrix& measurement_variance)
{
    if(!m_outlier_filtering_enabled and !m_weighting_enabled) return true;

    const Matrix innov_transp = innovation.transp();
    const Matrix sum_variance = estimate_variance + measurement_variance;

    if(m_outlier_filtering_enabled)
    {
        float innovation_2 = convDble(innov_transp * InverseMatrix(sum_variance) * innovation);
        if(m_outlier_threshold > 0 and innovation_2 > m_outlier_threshold)
            return false;
    }

    if(m_weighting_enabled)
    {
        m_filter_weight *= 1 / ( 1 + convDble(innov_transp * InverseMatrix(measurement_variance) * innovation));
    }

    return true;
}

std::string WSrBasicUKF::summary(bool detailed) const
{
    std::stringstream str_strm;
    str_strm << "ID: " << m_id <<  " Weight: " << m_filter_weight << std::endl;
    str_strm << m_estimate.string() << std::endl;
    return str_strm.str();
}

std::ostream& WSrBasicUKF::writeStreamBinary (std::ostream& output) const
{
    m_model->writeStreamBinary(output);
    m_unscented_transform.writeStreamBinary(output);
    m_estimate.writeStreamBinary(output);
    return output;
}

std::istream& WSrBasicUKF::readStreamBinary (std::istream& input)
{
    m_model->readStreamBinary(input);
    m_unscented_transform.readStreamBinary(input);
    init();
    MultivariateGaussian temp;
    temp.readStreamBinary(input);
    initialiseEstimate(temp);
    return input;
}

