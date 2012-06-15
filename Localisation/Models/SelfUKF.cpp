#include "SelfUKF.h"
#include "Tools/Math/General.h"
#include "Tools/Math/Statistics.h"
#include "Localisation/odometryMotionModel.h"
#include "Localisation/MeasurementError.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include <sstream>
#include <iostream>

/*! @brief Default constructor
 */
SelfUKF::SelfUKF(): UKF(states_total), WeightedModel(0.0)
{
    m_previous_decisions.resize(FieldObjects::NUM_AMBIGUOUS_FIELD_OBJECTS, FieldObjects::NUM_STAT_FIELD_OBJECTS);
}

/*! @brief Default time constructor

    This constructor requires a creation time.
 */
SelfUKF::SelfUKF(double time): UKF(states_total), WeightedModel(time)
{
    m_previous_decisions.resize(FieldObjects::NUM_AMBIGUOUS_FIELD_OBJECTS, FieldObjects::NUM_STAT_FIELD_OBJECTS);
}

/*! @brief Copy constructor

 */
SelfUKF::SelfUKF(const SelfUKF& source): UKF(states_total), WeightedModel(0.0)
{
    *this = source;
}

/*! @brief Split constructor

Takes a parent filter and performs a split on an ambiguous object using the given split option.

@param parent The parent model from which the split orignates.
@param object The ambiguous object belong evaluated by the split.
@param splitOption The option to be evaluated within this model.
@param time The current time of the update
*/
SelfUKF::SelfUKF(const SelfUKF& parent, const AmbiguousObject& object, const StationaryObject& splitOption, const MeasurementError& error, float time):
    UKF(parent), WeightedModel(parent,time)
{
    StationaryObject updateObject(splitOption);
    updateObject.CopyObject(object);

    // Save the information about the model
    m_split_option = splitOption.getID();
    assert(object.getID() < FieldObjects::NUM_AMBIGUOUS_FIELD_OBJECTS);
    m_previous_decisions = parent.m_previous_decisions;
    m_previous_decisions[object.getID()] = splitOption.getID();

    bool good = MeasurementUpdate(updateObject, error);
    if(good) setActive(true);
    else setActive(false);
    return;
}

void SelfUKF::setMean(const Matrix& newMean)
{

    m_mean = newMean;
    // normalise angle.
    m_mean[states_heading][0] = mathGeneral::normaliseAngle(m_mean[states_heading][0]);
}


/*!
 * @brief The process equation is used to update the systems state using the process euquations of the system.
 * @param sigma_point The sigma point representing a system state.
 * @param deltaT The amount of time that has passed since the previous update, in seconds.
 * @param measurement The measurement of the odometry used to update the objects new relative position.
 * @return The new estimated system state.
 */
Matrix SelfUKF::processEquation(const Matrix& sigma_point, double deltaT, const Matrix& measurement)
{
    Matrix result(sigma_point); // Start at original state.
    double tempx, tempy;

    assert(measurement.getm()==3); // Check the correct number of measurements have been given.

    OdometryMotionModel odom_Model(0.07,0.00005,0.00005,0.000005);
    Pose2D sigma, diffOdom;

    sigma.X = sigma_point[states_x][0];
    sigma.Y = sigma_point[states_y][0];
    sigma.Theta = sigma_point[states_heading][0];

    diffOdom.X = measurement[0][0];
    diffOdom.Y = measurement[1][0];
    diffOdom.Theta = measurement[2][0];

    double *newPose = odom_Model.getNextSigma(diffOdom, sigma);

    result[states_x][0] = *newPose;
    result[states_y][0] = *(newPose+1);
    result[states_heading][0] = *(newPose+2);

    return result;
}

/*!
 * @brief The measurement equation is used to calculate the expected measurement given a system state.
 * @param sigma_point The sigma point representing a system state.
 * @param measurementArgs Additional arguments used to calculate the measurement. In this implementation it is unused.
 * @return The expected measurement for the given states.
 */
Matrix SelfUKF::measurementEquation(const Matrix& sigma_point, const Matrix& measurementArgs)
{
    // measurementArgs not required, since the measurements are only reliant on the current state.
    // Measurement is to be in polar coordinates (distance, theta).

    // Get position from sigma point.
    const double my_x = sigma_point[states_x][0];
    const double my_y = sigma_point[states_x][0];
    const double my_theta = sigma_point[states_heading][0];

    const double obj_x = measurementArgs[0][0];
    const double obj_y = measurementArgs[1][0];

    const double dx = obj_x - my_x;
    const double dy = obj_y - my_y;

    // Convert from cartesian to polar coordinates.
    double distance = sqrt(dx*dx + dy*dy);
    double angle = mathGeneral::normaliseAngle(atan2(dy, dx) - my_theta);

    // Write to matrix for return.
    Matrix expected_measurement(2,1,false);
    expected_measurement[0][0] = distance;
    expected_measurement[1][0] = angle;

    return expected_measurement;
}

bool SelfUKF::MeasurementUpdate(const StationaryObject& object, const MeasurementError& error)
{
    Matrix meas_noise = error.errorCovariance();

    Matrix args(2,1,false);
    args[0][0] = object.X();
    args[1][0] = object.Y();

    Matrix measurement(2,1,false);
    measurement[0][0] = object.measuredDistance() * cos(object.measuredElevation());
    measurement[1][0] = object.measuredBearing();
    return measurementUpdate(measurement, meas_noise, args);
}

/*!
 * @brief Performs the measurement update of the filter.
 * @param measurement The measurement to be used for the update.
 * @param measurementNoise The linear measurement noise that will be added.
 * @param measurementArgs Any additional information about the measurement, if required.
 * @return True if the measurement update was performed successfully. False if it was not.
 */
bool SelfUKF::measurementUpdate(const Matrix& measurement, const Matrix& measurementNoise, const Matrix& measurementArgs)
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

    Matrix Pyy(measurementNoise);   // measurement noise is added, so just use as the beginning value of the sum.
    Matrix Pxy(numStates, totalMeasurements, false);

    // Calculate the Pyy and Pxy variance matrices.
    for(unsigned int i = 0; i < totalPoints; ++i)
    {
        double weight = m_covariance_weights[0][i];
        // store difference between prediction and measurement.
        currentPoint = Yprop.getCol(i) - Ymean;
        // Innovation covariance - Add Measurement noise
        Pyy = Pyy + weight * currentPoint * currentPoint.transp();
        // Cross correlation matrix
        Pxy = Pxy + weight * (m_sigma_points.getCol(i) - m_sigma_mean) * currentPoint.transp();    // Important: Use mean from estimate, not current mean.
    }


    // This is the new part for calculating the new model weighting.
    double innovation2 = convDble((Ymean - measurement).transp() * Invert22(Pyy) * (Ymean - measurement));
    float new_alpha = WeightedModel::alpha() * 1 / (1 + innovation2);
    setAlpha(new_alpha);

    // Calculate the Kalman filter gain
    Matrix K;
    // If we have a 2 dimensional measurement, use the faster shortcut function.
    if(totalMeasurements == 2)
    {
        K = Pxy * Invert22(Pyy);
    }
    else
    {
        K = Pxy * InverseMatrix(Pyy);
    }

    Matrix newMean = mean() + K * (measurement - Ymean);
    Matrix newCovariance = covariance() - K*Pyy*K.transp();

    setMean(newMean);
    setCovariance(newCovariance);
    return true;
}

bool SelfUKF::measurementUpdateAngleBetweenTwoObjects(double angle, double x1, double y1, double x2, double y2, double angle_variance)
{
    const unsigned int totalPoints = totalSigmaPoints();
    const unsigned int numStates = totalStates();
    const unsigned int totalMeasurements = 1;
    double measurement = angle;
    Matrix currentPoint; // temporary storage.

    Matrix Yprop(totalMeasurements, totalPoints);

    double angleToObj1;
    double angleToObj2;
    double sigma_x;
    double sigma_y;
    // First step is to calculate the expected measurmenent for each sigma point.
    for (unsigned int i = 0; i < totalPoints; ++i)
    {
        currentPoint = m_sigma_points.getCol(i);    // Get the sigma point.
        sigma_x = currentPoint[states_x][0];
        sigma_y = currentPoint[states_y][0];
        angleToObj1 = atan2 ( y1 - sigma_y, x1 - sigma_x );
        angleToObj2 = atan2 ( y2 - sigma_y, x2 - sigma_x );
        Yprop[0][i] = mathGeneral::normaliseAngle(angleToObj1 - angleToObj2);
    }

    // Now calculate the mean of these measurement sigmas.
    float Ymean = convDble(CalculateMeanFromSigmas(Yprop));

    float Pyy(angle_variance);   // measurement noise is added, so just use as the beginning value of the sum.
    Matrix Pxy(numStates, totalMeasurements, false);

    // Calculate the Pyy and Pxy variance matrices.
    for(unsigned int i = 0; i < totalPoints; ++i)
    {
        double weight = m_covariance_weights[0][i];
        // store difference between prediction and measurement.
        currentPoint = Yprop.getCol(i) - Ymean;
        // Innovation covariance - Add Measurement noise
        Pyy = Pyy + weight * convDble(currentPoint * currentPoint.transp());
        // Cross correlation matrix
        Pxy = Pxy + weight * (m_sigma_points.getCol(i) - m_sigma_mean) * currentPoint.transp();    // Important: Use mean from estimate, not current mean.
    }


    // This is the new part for calculating the new model weighting.
    double innovation2 = (Ymean - measurement) * (1/Pyy) * (Ymean - measurement);
    float new_alpha = WeightedModel::alpha() * 1 / (1 + innovation2);
    setAlpha(new_alpha);

    // Calculate the Kalman filter gain
    Matrix K;
    // If we have a 2 dimensional measurement, use the faster shortcut function.
    K = Pyy * Pxy;

    Matrix newMean = mean() + K * (measurement - Ymean);
    Matrix newCovariance = covariance() - K*Pyy*K.transp();

    setMean(newMean);
    setCovariance(newCovariance);
    return true;
}

bool SelfUKF::isLost() const
{
    const float lost_self_pos_sd = 150.0f;
    const float lost_self_heading_sd = 1.5f;
    float max_sd = 2 * std::max(sd(states_x), sd(states_y));
    return (max_sd > lost_self_pos_sd) or (sd(states_x) > lost_self_heading_sd);
}

/*! @brief Generate the current self state from the current state estimate.
    @return Self object describing the current state estimate.
*/
Self SelfUKF::GenerateSelfState() const
{
    Self result;
    result.updateLocationOfSelf(mean(states_x), mean(states_y), mean(states_heading), sd(states_x), sd(states_y), sd(states_heading),isLost());
    return result;
}

/*! @brief Get the previous decision path when this ambiguous object was last encountered.
    @param theObject The ambiguous object.
    @return The object id of the unique object that was last chosen.
*/
unsigned int SelfUKF::previousSplitOption(const AmbiguousObject& theObject) const
{
    unsigned int result;
    unsigned int objectIndex = theObject.getID();
    assert(objectIndex < FieldObjects::NUM_AMBIGUOUS_FIELD_OBJECTS);
    result = m_previous_decisions.at(objectIndex);
    return result;
}

/*!
@brief Outputs a binary representation of the UKF object to a stream.
@param output The output stream.
@return The output stream.
*/
std::ostream& SelfUKF::writeStreamBinary (std::ostream& output) const
{
    UKF::writeStreamBinary(output);
    WeightedModel::writeStreamBinary(output);
    return output;
}

/*!
@brief Reads in a UKF object from the input stream.
@param input The input stream.
@return The input stream.
*/
std::istream& SelfUKF::readStreamBinary (std::istream& input)
{
    UKF::readStreamBinary(input);
    WeightedModel::readStreamBinary(input);
    return input;
}
