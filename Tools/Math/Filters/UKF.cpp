#include "UKF.h"
#include "debug.h"
#include "Tools/Math/General.h"

using namespace std;


UKF::UKF(): Moment(0), UnscentedTransform(0)
{
    CalculateWeights();
}

UKF::UKF(unsigned int numStates): Moment(numStates), UnscentedTransform(numStates)
{
    CalculateWeights();
}

UKF::UKF(const UKF& source): Moment(source), UnscentedTransform(source)
{
    CalculateWeights();
}

UKF::~UKF()
{
}

/*!
 * @brief Pre-calculate the sigma point weightings, based on the current unscented transform parameters.
 */
void UKF::CalculateWeights()
{
    const unsigned int totalWeights = totalSigmaPoints();

    // initialise to correct sizes. These are row vectors.
    m_mean_weights = Matrix(1, totalWeights, false);
    m_covariance_weights = Matrix(1, totalWeights, false);
    m_sqrt_covariance_weights = Matrix(1, totalWeights, false);

    // Calculate the weights.
    for (unsigned int i = 0; i < totalWeights; ++i)
    {
        m_mean_weights[0][i] = Wm(i);
        m_covariance_weights[0][i] = Wc(i);
        m_sqrt_covariance_weights[0][i] = sqrt(fabs(m_covariance_weights[0][i]));
    }
    return;
}

/*!
 * @brief Calculates the sigma points from the current mean an covariance of the filter.
 * @return The sigma points that describe the current mean and covariance.
 */
Matrix UKF::GenerateSigmaPoints() const
{
    const unsigned int numPoints = totalSigmaPoints();
    const Matrix current_mean = mean();
    Matrix points(totalStates(), numPoints, false);

    points.setCol(0, current_mean); // First sigma point is the current mean with no deviation
    Matrix sqtCovariance = cholesky(covarianceSigmaWeight() * covariance());
    Matrix deviation;

    for(unsigned int i = 1; i < totalStates() + 1; i++){
        int negIndex = i+totalStates();
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
Matrix UKF::CalculateMeanFromSigmas(const Matrix& sigmaPoints) const
{
    return sigmaPoints * m_mean_weights.transp();
}

/*!
 * @brief Calculate the mean from a set of sigma points, given the mean of these points.
 * @param sigmaPoints The sigma points.
 * @param mean The mean of the sigma points.
 * @return The covariance of the given sigma points.
 */
Matrix UKF::CalculateCovarianceFromSigmas(const Matrix& sigmaPoints, const Matrix& mean) const
{
    const unsigned int numPoints = totalSigmaPoints();
    const unsigned int numStates = totalStates();
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
bool UKF::timeUpdate(double deltaT, const Matrix& measurement, const Matrix& linearProcessNoise, const Matrix& measurementNoise)
{
    const unsigned int totalPoints = totalSigmaPoints();

    // Calculate the current sigma points, and write to member variable.
    m_sigma_points = GenerateSigmaPoints();

    Matrix currentPoint; // temporary storage.

    // update each sigma point.
    for (unsigned int i = 0; i < totalPoints; ++i)
    {
        currentPoint = m_sigma_points.getCol(i);    // Get the sigma point.
        m_sigma_points.setCol(i, processEquation(currentPoint, deltaT, measurement));   // Write the propagated version of it.
    }

    // Calculate the new mean and covariance values.
    Matrix predictedMean = CalculateMeanFromSigmas(m_sigma_points);
    Matrix predictedCovariance = CalculateCovarianceFromSigmas(m_sigma_points, predictedMean) + linearProcessNoise;

    // Set the new mean and covariance values.
    m_sigma_mean = predictedMean;
    setMean(predictedMean);
    setCovariance(predictedCovariance);

    // Redraw sigma point to include process noise.
    m_sigma_points = GenerateSigmaPoints();

    // Initialise the variables for sequential measurement updates.
    m_C = Matrix(totalPoints, totalPoints, true);
    m_d = Matrix(totalPoints, 1, false);
    return true;
}

/*!
 * @brief Performs the measurement update of the filter.
 * @param measurement The measurement to be used for the update.
 * @param measurementNoise The linear measurement noise that will be added.
 * @param measurementArgs Any additional information about the measurement, if required.
 * @return True if the measurement update was performed successfully. False if it was not.
 */
bool UKF::measurementUpdate(const Matrix& measurement, const Matrix& measurementNoise, const Matrix& measurementArgs)
{
    const unsigned int totalPoints = totalSigmaPoints();
    const unsigned int numStates = totalStates();
    const unsigned int totalMeasurements = measurement.getm();
    Matrix currentPoint; // temporary storage.

    Matrix Yprop(totalMeasurements, totalPoints);

    // First step is to calculate the expected measurmenent for each sigma point.
    for (unsigned int i = 0; i < totalPoints; ++i)
    {
        currentPoint = m_sigma_points.getCol(i);    // Get the sigma point.
        Yprop.setCol(i, measurementEquation(currentPoint, measurementArgs));
    }

    // Now calculate the mean of these measurement sigmas.
    Matrix Ymean = CalculateMeanFromSigmas(Yprop);

    Matrix Y(totalMeasurements,totalPoints,false);
    Matrix X(numStates, totalPoints, false);

    // Calculate the Y and X vectors.
    for(unsigned int i = 0; i < totalPoints; ++i)
    {
        double weight = m_sqrt_covariance_weights[0][i];
        Y.setCol(i, weight*(Yprop.getCol(i) - Ymean));
        X.setCol(i, weight*(m_sigma_points.getCol(i) - m_sigma_mean));
    }

    // Calculate the new C and d values.
    Matrix Yaug = Y;
    Yaug.setCol(0, mathGeneral::sign(m_covariance_weights[0][0]) * Yaug.getCol(0));
    Matrix Ytransp = Yaug.transp();
    m_C = m_C - m_C * Ytransp * InverseMatrix(measurementNoise+Y*m_C*Ytransp) * Y * m_C;
    m_d = m_d + Ytransp * InverseMatrix(measurementNoise) * (measurement - Ymean);

    Matrix newMean = m_sigma_mean + X * m_C * m_d;
    Matrix newCovariance = X * m_C * X.transp();

    setMean(newMean);
    setCovariance(newCovariance);
    return true;
}

void UKF::initialiseModel(const Matrix& mean, const Matrix& covariance)
{
    setMean(mean);
    setCovariance(covariance);
    m_sigma_points = GenerateSigmaPoints();
    m_sigma_mean = CalculateMeanFromSigmas(m_sigma_points);
    return;
}

bool UKF::operator ==(const UKF& b) const
{
    // Check Moment portions are equal
    const Moment* this_moment = this;
    const Moment* other_moment = &b;
    if(*this_moment != *other_moment)
    {
        return false;
    }

    // Check UnscentedTransform portions are equal
    const UnscentedTransform* this_ut = this;
    const UnscentedTransform* other_ut = &b;
    if(*this_ut != *other_ut)
    {
        return false;
    }

    // Check other memebr variables.
   if(m_mean_weights != b.m_mean_weights) return false;
   if(m_covariance_weights != b.m_covariance_weights) return false;
   if(m_sigma_points != b.m_sigma_points) return false;
   if(m_sigma_mean != b.m_sigma_mean) return false;
    return true;
}

std::ostream& UKF::writeStreamBinary (std::ostream& output) const
{
    UnscentedTransform::writeStreamBinary(output);
    Moment::writeStreamBinary(output);
    WriteMatrix(output, m_sigma_points);
    return output;
}

std::istream& UKF::readStreamBinary (std::istream& input)
{
    UnscentedTransform::readStreamBinary(input);
    Moment::readStreamBinary(input);
    CalculateWeights();

    m_sigma_points = ReadMatrix(input);
    // make sure that the sigma points are not empty
    if((m_sigma_points.getm() > 0) and (m_sigma_points.getn() > 0))
    {
        m_sigma_mean = CalculateMeanFromSigmas(m_sigma_points);
    }
    return input;
}
