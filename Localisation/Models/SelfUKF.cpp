#include "SelfUKF.h"
#include "Tools/Math/General.h"
#include "Tools/Math/Statistics.h"
#include "Localisation/odometryMotionModel.h"
#include <sstream>
#include <iostream>

/*! @brief Default constructor
 */
SelfUKF::SelfUKF(): UKF(states_total), WeightedModel(0.0)
{
}

/*! @brief Default time constructor

    This constructor requires a creation time.
 */
SelfUKF::SelfUKF(double time): UKF(states_total), WeightedModel(time)
{
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
    StationaryObject update(splitOption);
    update.CopyObject(object);

    //SelfUKF::updateResult result = MeasurementUpdate(update, error);
//    if(result == RESULT_OUTLIER) setActive(false);
//    else setActive(true);
    return;
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
//    double innovation2 = convDble((yBar - y).transp() * Invert22(Py + R_obj_rel) * (yBar - y));
//    float new_alpha = alpha() * 1 / (1 + innovation2);
//    setAlpha(new_alpha);

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
