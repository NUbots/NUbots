#include "SeqUKF.h"
#include "Tools/Math/General.h"
#include "IKFModel.h"
#include <sstream>

SeqUKF::SeqUKF(IKFModel *model): IKalmanFilter(model), m_unscented_transform(model->totalStates())
{
    init();
}

SeqUKF::SeqUKF(const SeqUKF& source): IKalmanFilter(source), m_unscented_transform(source.m_unscented_transform)
{
    m_sigma_points = source.m_sigma_points;
    m_sigma_mean = source.m_sigma_mean;
    m_C = source.m_C;
    m_d = source.m_d;
    m_X = source.m_X;
}

SeqUKF::~SeqUKF()
{
}

void SeqUKF::init()
{
    initialiseEstimate(m_estimate);
}

/*!
 * @brief Performs the time update of the filter.
 * @param deltaT The time that has passed since the previous update.
 * @param measurement The measurement/s (if any) that can be used to measure a change in the system.
 * @param linearProcessNoise The linear process noise that will be added.
 * @return True if the time update was performed successfully. False if it was not.
 */
bool SeqUKF::timeUpdate(double delta_t, const Matrix& measurement, const Matrix& process_noise, const Matrix& measurement_noise)
{
    const unsigned int total_points = m_unscented_transform.totalSigmaPoints();

    // Calculate the current sigma points, and write to member variable.
    m_sigma_points = m_unscented_transform.GenerateSigmaPoints(m_estimate.mean(), m_estimate.covariance());
    Matrix currentPoint; // temporary storage.

    // update each sigma point.
    for (unsigned int i = 0; i < total_points; ++i)
    {
        currentPoint = m_sigma_points.getCol(i);    // Get the sigma point.
        m_sigma_points.setCol(i, m_model->processEquation(currentPoint, delta_t, measurement));   // Write the propagated version of it.
    }
    // Calculate the new mean and covariance values.
    Matrix predictedMean = m_unscented_transform.CalculateMeanFromSigmas(m_sigma_points);
    Matrix predictedCovariance = m_unscented_transform.CalculateCovarianceFromSigmas(m_sigma_points, predictedMean) + process_noise;

    // Set the new mean and covariance values.
    MultivariateGaussian new_estimate = m_estimate;

    if(not predictedCovariance.isValid())
    {
        std::cout << "Mean:\n" << m_estimate.mean() << std::endl;
        std::cout << "Covariance:\n" << m_estimate.covariance() << std::endl;
        std::cout << "Sqrt Covariance:\n" << cholesky(m_estimate.covariance()) << std::endl;
        std::cout << "m_sigma_points:\n" << m_sigma_points << std::endl;

        std::cout << "cov = [";
        Matrix cov = m_estimate.covariance();
        for(unsigned int i = 0; i < cov.getm(); ++i)
        {
            if(i!=0) std::cout << "; ";
            for(unsigned int j = 0; j < cov.getm(); ++j)
            {
                if(j!=0) std::cout << ",";
                std::cout << std::setprecision(9) << cov[i][j];
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

/*!
 * @brief Performs the measurement update of the filter.
 * @param measurement The measurement to be used for the update.
 * @param measurementNoise The linear measurement noise that will be added.
 * @param measurementArgs Any additional information about the measurement, if required.
 * @return True if the measurement update was performed successfully. False if it was not.
 */
bool SeqUKF::measurementUpdate(const Matrix& measurement, const Matrix& noise, const Matrix& args, unsigned int type)
{
    const unsigned int total_points = m_unscented_transform.totalSigmaPoints();
    const unsigned int totalMeasurements = measurement.getm();
    Matrix current_point; // temporary storage.
    Matrix Yprop(totalMeasurements, total_points);

    // First step is to calculate the expected measurmenent for each sigma point.
    for (unsigned int i = 0; i < total_points; ++i)
    {
        current_point = m_sigma_points.getCol(i);    // Get the sigma point.
        Yprop.setCol(i, m_model->measurementEquation(current_point, args, type));
    }

    // Now calculate the mean of these measurement sigmas.
    Matrix Ymean = m_unscented_transform.CalculateMeanFromSigmas(Yprop);

    Matrix Y(totalMeasurements, total_points,false);
    Matrix Pyy(noise);

    const Matrix cov_weights = m_unscented_transform.covarianceWeights();
    // Calculate the Y vector.
    for(unsigned int i = 0; i < total_points; ++i)
    {
        Matrix point = Yprop.getCol(i) - Ymean;
        Y.setCol(i, point);
        Pyy = Pyy + cov_weights[0][i] * point * point.transp();
    }

    // Calculate the new C and d values.
    Matrix Ytransp = Y.transp();

    const Matrix innovation = m_model->measurementDistance(measurement, Ymean, type);

    // Check for outlier, if outlier return without updating estimate.
    if(evaluateMeasurement(innovation, Pyy - noise, noise) == false)
        return false;

    m_C = m_C - m_C.transp() * Ytransp * InverseMatrix(noise + Y*m_C*Ytransp) * Y * m_C;
    m_d = m_d + Ytransp * InverseMatrix(noise) * innovation;

    // Update mean and covariance.
    Matrix updated_mean = m_sigma_mean + m_X * m_C * m_d;
    Matrix updated_covariance = m_X * m_C * m_X.transp();

    if(not updated_covariance.isValid())
    {
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
}

void SeqUKF::initialiseEstimate(const MultivariateGaussian& estimate)
{
    // This is more complicated than you might expect because of all of the buffered values that
    // must be kept up to date.
    const unsigned int total_points = m_unscented_transform.totalSigmaPoints();
    const unsigned int num_states = m_estimate.totalStates();

    // Assign the estimate.
    m_estimate = estimate;

    // Redraw sigma points to cover this new estimate.
    m_sigma_mean = estimate.mean();
    m_sigma_points = m_unscented_transform.GenerateSigmaPoints(m_estimate.mean(), m_estimate.covariance());

    // Initialise the variables for sequential measurement updates.
    //m_C = Matrix(total_points, total_points, true);
    m_C = diag(m_unscented_transform.covarianceWeights());
    m_d = Matrix(total_points, 1, false);

    // Calculate X vector.
    m_X = Matrix(num_states, total_points, false);
    for(unsigned int i = 0; i < total_points; ++i)
    {
        m_X.setCol(i, (m_sigma_points.getCol(i) - m_sigma_mean));
    }

    return;
}

bool SeqUKF::evaluateMeasurement(const Matrix& innovation, const Matrix& estimate_variance, const Matrix& measurement_variance)
{
    if(!m_outlier_filtering_enabled) return true;

    Matrix innov_transp = innovation.transp();
    Matrix innov_variance = estimate_variance + measurement_variance;

    if(m_outlier_filtering_enabled)
    {
        float innovation_2 = convDble(innov_transp * InverseMatrix(innov_variance) * innovation);
        if(m_outlier_threshold > 0 and innovation_2 > m_outlier_threshold)
        {
            return false;
        }
    }

    return true;
}

std::string SeqUKF::summary(bool detailed) const
{
    std::stringstream str_strm;
    str_strm << m_estimate.string() << std::endl;
    return str_strm.str();
}

std::ostream& SeqUKF::writeStreamBinary (std::ostream& output) const
{
    m_model->writeStreamBinary(output);
    m_unscented_transform.writeStreamBinary(output);
    m_estimate.writeStreamBinary(output);
    return output;
}

std::istream& SeqUKF::readStreamBinary (std::istream& input)
{
    m_model->readStreamBinary(input);
    m_unscented_transform.readStreamBinary(input);
    MultivariateGaussian temp;
    temp.readStreamBinary(input);
    initialiseEstimate(temp);
    return input;
}
