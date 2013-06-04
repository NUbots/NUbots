#include "MobileObjectModel.h"
#include <assert.h>
#include "Tools/Math/General.h"

MobileObjectModel::MobileObjectModel()
{
    m_velocity_decay = 0.96;    // Randomly guessed decay -- may not represent the real world.
}

MobileObjectModel::MobileObjectModel(const MobileObjectModel& source)
{
    *this = source;
}

/*!
 * @brief The process equation is used to update the systems state using the process euquations of the system.
 * @param sigma_point The sigma point representing a system state.
 * @param deltaT The amount of time that has passed since the previous update, in seconds.
 * @param measurement The measurement of the odometry used to update the objects new relative position.
 * @return The new estimated system state.
 */
Matrix MobileObjectModel::processEquation(const Matrix& sigma_point, double delta_t, const Matrix& measurement)
{
    Matrix result(sigma_point); // Start at original state.
    double tempx, tempy;

    assert(measurement.getm()==3); // Check the correct number of measurements have been given.

    // In this case the measurement data is the odometry of the robot. This Should follow the format delta X, delta Y, delta Theta.
    double deltaX = measurement[0][0];
    double deltaY = measurement[1][0];
    double deltaTheta = measurement[2][0];

    // First we have the change in the object position due to the object velocity.
    result[kstates_x_pos][0] += delta_t * sigma_point[kstates_x_vel][0];
    result[kstates_y_pos][0] += delta_t * sigma_point[kstates_y_vel][0];

    // Next add a little bit of decay due to friction.
    result[kstates_x_vel][0] *= m_velocity_decay;
    result[kstates_y_vel][0] *= m_velocity_decay;

    // pre-calculate these since they are used more than once.
    const double cosTheta = cos(deltaTheta);
    const double sinTheta = sin(deltaTheta);

    tempx = result[kstates_x_pos][0];
    tempy = result[kstates_y_pos][0];

    // Now we have to re-orientate the position due to the motion of the observer.
    // Start with turn
    // Apply to position - turning counter clockwise (+ve angle) means that the object should move clockwise relatively.
    tempx = result[kstates_x_pos][0] *  cosTheta + result[kstates_y_pos][0] * sinTheta;
    tempy = -result[kstates_x_pos][0] *  sinTheta + result[kstates_y_pos][0] * cosTheta;

    result[kstates_x_pos][0] = tempx;
    result[kstates_y_pos][0] = tempy;

    // Apply to velocity
    tempx = result[kstates_x_vel][0] *  cosTheta - result[kstates_y_vel][0] * sinTheta;
    tempy = result[kstates_x_vel][0] *  sinTheta + result[kstates_y_vel][0] * cosTheta;

    result[kstates_x_vel][0] = tempx;
    result[kstates_y_vel][0] = tempy;

    // Now add observer translation
    result[kstates_x_pos][0] -= deltaX;  // moving forward brings you closer, so decreases relative position.
    result[kstates_y_pos][0] -= deltaY;

    return result;
}

/*!
 * @brief The measurement equation is used to calculate the expected measurement given a system state.
 * @param sigma_point The sigma point representing a system state.
 * @param measurementArgs Additional arguments used to calculate the measurement. In this implementation it is unused.
 * @return The expected measurement for the given states.
 */
Matrix MobileObjectModel::measurementEquation(const Matrix& state, const Matrix& measurementArgs, unsigned int type)
{
    if(type == kobserved_measurement)
    {
        return observedMeasurementEquation(state, measurementArgs);
    }
    else if(type == kshared_measurement)
    {
        return sharedMeasurementEquation(state, measurementArgs);
    }
}

Matrix MobileObjectModel::observedMeasurementEquation(const Matrix& state, const Matrix& measurementArgs)
{
    // measurementArgs not required, since the measurements are only reliant on the current state.
    // Measurement is to be in polar coordinates (distance, theta).

    // measurementArgs not required, since the measurements are only reliant on the current state.
    // Measurement is to be in polar coordinates (distance, theta).

    // Get position from sigma point.
    const double x = state[kstates_x_pos][0];
    const double y = state[kstates_y_pos][0];

    // Convert from cartesian to polar coordinates.
    double distance = sqrt(x*x + y*y);
    double angle = atan2(y, x);

    // Write to matrix for return.
    Matrix expected_measurement(2,1,false);
    expected_measurement[0][0] = distance;
    expected_measurement[1][0] = angle;

    return expected_measurement;
}

Matrix MobileObjectModel::sharedMeasurementEquation(const Matrix& state, const Matrix& measurementArgs)
{
    // measurementArgs not required, since the measurements are only reliant on the current state.
    // Measurement is to be in polar coordinates (distance, theta).

    // measurementArgs not required, since the measurements are only reliant on the current state.
    // Measurement is to be in polar coordinates (distance, theta).

    // Get position from sigma point.
    const double x = state[kstates_x_pos][0];
    const double y = state[kstates_y_pos][0];

    // Write to matrix for return.
    Matrix expected_measurement(2,1,false);
    expected_measurement[0][0] = x;
    expected_measurement[1][0] = y;

    return expected_measurement;
}

Matrix MobileObjectModel::measurementDistance(const Matrix& measurement1, const Matrix& measurement2, unsigned int type)
{
    Matrix result = measurement1 - measurement2;
    result[1][0] = mathGeneral::normaliseAngle(result[1][0]);
    return result;
}

/*!
@brief Outputs a binary representation of the UKF object to a stream.
@param output The output stream.
@return The output stream.
*/
std::ostream& MobileObjectModel::writeStreamBinary (std::ostream& output) const
{
    return output;
}

/*!
@brief Reads in a UKF object from the input stream.
@param input The input stream.
@return The input stream.
*/
std::istream& MobileObjectModel::readStreamBinary (std::istream& input)
{
    return input;
}
