#include "SeqUKF.h"
#include "Tools/Math/General.h"
#include "IKFModel.h"
#include <sstream>

SeqUKF::SeqUKF(IKFModel *model): IKalmanFilter(model), m_estimate(model->totalStates()), m_unscented_transform(model->totalStates())
{
    init();
}

SeqUKF::SeqUKF(const SeqUKF& source): IKalmanFilter(source), m_estimate(source.m_estimate), m_unscented_transform(source.m_unscented_transform)
{
    m_outlier_filtering_enabled = source.m_outlier_filtering_enabled;
    m_outlier_threshold = source.m_outlier_threshold;
    m_weighting_enabled = source.m_weighting_enabled;
    m_filter_weight = source.m_filter_weight;
    m_mean_weights = source.m_mean_weights;
    m_covariance_weights = source.m_covariance_weights;
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
    CalculateWeights();
    initialiseEstimate(m_estimate);
    m_outlier_filtering_enabled = false;
    m_weighting_enabled = false;
    m_outlier_threshold = 15.f;
    m_filter_weight = 1.f;
}

/*!
 * @brief Pre-calculate the sigma point weightings, based on the current unscented transform parameters.
 */
void SeqUKF::CalculateWeights()
{
    const unsigned int totalWeights = m_unscented_transform.totalSigmaPoints();

    // initialise to correct sizes. These are row vectors.
    m_mean_weights = Matrix(1, totalWeights, false);
    m_covariance_weights = Matrix(1, totalWeights, false);

    // Calculate the weights.
    for (unsigned int i = 0; i < totalWeights; ++i)
    {
        m_mean_weights[0][i] = m_unscented_transform.Wm(i);
        m_covariance_weights[0][i] = m_unscented_transform.Wc(i);
    }
    return;
}

/*!
 * @brief Calculates the sigma points from the current mean an covariance of the filter.
 * @return The sigma points that describe the current mean and covariance.
 */
Matrix SeqUKF::GenerateSigmaPoints() const
{
    const unsigned int numPoints = m_unscented_transform.totalSigmaPoints();
    const Matrix current_mean = m_estimate.mean();
    const unsigned int num_states = m_estimate.totalStates();
    Matrix points(num_states, numPoints, false);

    points.setCol(0, current_mean); // First sigma point is the current mean with no deviation
    Matrix weightedCov = m_unscented_transform.covarianceSigmaWeight() * m_estimate.covariance();
    Matrix sqtCovariance = cholesky(weightedCov);
    if(not sqtCovariance.isValid())
    {
        sqtCovariance = cholesky(weightedCov.transp());
    }
    Matrix deviation;

    for(unsigned int i = 1; i < num_states + 1; i++){
        int negIndex = i+num_states;
        deviation = sqtCovariance.getCol(i - 1);        // Get deviation from weighted covariance
        points.setCol(i, (current_mean + deviation));         // Add mean + deviation
        points.setCol(negIndex, (current_mean - deviation));  // Add mean - deviation
    }
    return points;
}

/*!
 * @brief Calculate the mean from a set of sigma points.
 * @param sigmaPoints The sigma points.
 * @return The mean of the given sigma points.
 */
Matrix SeqUKF::CalculateMeanFromSigmas(const Matrix& sigmaPoints) const
{
    return sigmaPoints * m_mean_weights.transp();
}

/*!
 * @brief Calculate the mean from a set of sigma points, given the mean of these points.
 * @param sigmaPoints The sigma points.
 * @param mean The mean of the sigma points.
 * @return The covariance of the given sigma points.
 */
Matrix SeqUKF::CalculateCovarianceFromSigmas(const Matrix& sigmaPoints, const Matrix& mean) const
{
    const unsigned int numPoints = m_unscented_transform.totalSigmaPoints();
    const unsigned int numStates = m_estimate.totalStates();
    Matrix covariance(numStates, numStates, false);  // Blank covariance matrix.
    Matrix diff;
    for(unsigned int i = 0; i < numPoints; ++i)
    {
        const double weight = m_covariance_weights[0][i];
        diff = sigmaPoints.getCol(i) - mean;
        covariance = covariance + weight*diff*diff.transp();
    }
    return covariance;
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
    m_sigma_points = GenerateSigmaPoints();
    Matrix currentPoint; // temporary storage.

    // update each sigma point.
    for (unsigned int i = 0; i < total_points; ++i)
    {
        currentPoint = m_sigma_points.getCol(i);    // Get the sigma point.
        m_sigma_points.setCol(i, m_model->processEquation(currentPoint, delta_t, measurement));   // Write the propagated version of it.
    }

    // Calculate the new mean and covariance values.
    Matrix predictedMean = CalculateMeanFromSigmas(m_sigma_points);
    Matrix predictedCovariance = CalculateCovarianceFromSigmas(m_sigma_points, predictedMean) + process_noise;

    // Set the new mean and covariance values.
    Moment new_estimate = m_estimate;

    if(not predictedCovariance.isValid())
    {
        std::cout << this->getFilterWeight() << std::endl;
        std::cout << this->active() << std::endl;
        std::cout << "Mean:\n" << m_estimate.mean() << std::endl;
        std::cout << "Covariance:\n" << m_estimate.covariance() << std::endl;
        std::cout << "Sqrt Covariance:\n" << cholesky(m_estimate.covariance()) << std::endl;
        std::cout << "m_sigma_points:\n" << m_sigma_points << std::endl;
    }

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
    Matrix Ymean = CalculateMeanFromSigmas(Yprop);

    Matrix Y(totalMeasurements, total_points,false);

    // Calculate the Y vector.
    for(unsigned int i = 0; i < total_points; ++i)
    {
        Y.setCol(i, Yprop.getCol(i) - Ymean);
    }

    // Calculate the new C and d values.
    Matrix Yaug = Y;
    //Yaug.setCol(0, mathGeneral::sign(m_covariance_weights[0][0]) * Yaug.getCol(0));
    Matrix Ytransp = Yaug.transp();

    const Matrix innovation = m_model->measurementDistance(measurement, Ymean, type);

    // Check for outlier, if outlier return without updating estimate.
    if(evaluateMeasurement(innovation, Y * Ytransp, noise) == false) // Y * Y^T is the estimate variance, by definition.
        return false;

    m_C = m_C - m_C * Ytransp * InverseMatrix(noise + Y*m_C*Ytransp) * Y * m_C;
    m_d = m_d + Ytransp * InverseMatrix(noise) * innovation;

    // Update mean and covariance.
    Matrix updated_mean = m_sigma_mean + m_X * m_C * m_d;
    Matrix updated_covariance = m_X * m_C * m_X.transp();

    if(not updated_covariance.isValid())
    {
        std::cout << "ID: " << id() << std::endl;
        std::cout << "measurement: " << measurement << std::endl;
        std::cout << "noise: " << noise << std::endl;
        std::cout << "args: " << args << std::endl;
        std::cout << "type: " << type << std::endl;
        std::cout << "m_sigma_points: " << m_sigma_points << std::endl;
        std::cout << "innovation: " << innovation << std::endl;
        std::cout << "m_C After: " << m_C << std::endl;
        std::cout << "m_d after: " << m_d << std::endl;
        std::cout << "New mean: " << updated_mean << std::endl;
        std::cout << "New covariance: " << updated_covariance << std::endl;
    }

    m_estimate.setMean(updated_mean);
    m_estimate.setCovariance(updated_covariance);
    return true;
}

void SeqUKF::initialiseEstimate(const Moment& estimate)
{
    // This is more complicated than you might expect because of all of the buffered values that
    // must be kept up to date.
    const unsigned int total_points = m_unscented_transform.totalSigmaPoints();
    const unsigned int num_states = m_estimate.totalStates();

    // Assign the estimate.
    m_estimate = estimate;

    // Redraw sigma points to cover this new estimate.
    m_sigma_mean = estimate.mean();
    m_sigma_points = GenerateSigmaPoints();

    // Initialise the variables for sequential measurement updates.
    //m_C = Matrix(total_points, total_points, true);
    m_C = diag(m_covariance_weights);
    m_d = Matrix(total_points, 1, false);

    // Calculate X vector.
    m_X = Matrix(num_states, total_points, false);
    for(unsigned int i = 0; i < total_points; ++i)
    {
        m_X.setCol(i, (m_sigma_points.getCol(i) - m_sigma_mean));
    }

    return;
}

const Moment& SeqUKF::estimate() const
{
    return m_estimate;
}

bool SeqUKF::evaluateMeasurement(const Matrix& innovation, const Matrix& estimate_variance, const Matrix& measurement_variance)
{
    if(!m_outlier_filtering_enabled and !m_weighting_enabled) return true;

    Matrix innov_transp = innovation.transp();
    Matrix sum_variance = estimate_variance + measurement_variance;

    if(m_outlier_filtering_enabled)
    {
        float innovation_2 = convDble(innov_transp * InverseMatrix(sum_variance) * innovation);
        if(m_outlier_threshold > 0 and innovation_2 > m_outlier_threshold)
        {
            m_filter_weight *= 0.0005;
            return false;
        }
    }

    if(m_weighting_enabled)
    {
        m_filter_weight *= 1 / ( 1 + convDble(innov_transp * InverseMatrix(measurement_variance) * innovation));
    }

    return true;
}

std::string SeqUKF::summary(bool detailed) const
{
    std::stringstream str_strm;
    str_strm << "ID: " << m_id <<  " Weight: " << m_filter_weight << std::endl;
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
    Moment temp;
    temp.readStreamBinary(input);
    CalculateWeights();
    initialiseEstimate(temp);
    return input;
}
