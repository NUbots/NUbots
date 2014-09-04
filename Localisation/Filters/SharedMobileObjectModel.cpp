#include "SharedMobileObjectModel.h"
#include <assert.h>
#include "Tools/Math/General.h"

SharedMobileObjectModel::SharedMobileObjectModel()
{
}

SharedMobileObjectModel::SharedMobileObjectModel(const SharedMobileObjectModel& source)
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
Matrix SharedMobileObjectModel::processEquation(const Matrix& sigma_point, double delta_t, const Matrix& measurement)
{
    Matrix result(sigma_point); // Start at original state.
    return result;
}

/*!
 * @brief The measurement equation is used to calculate the expected measurement given a system state.
 * @param sigma_point The sigma point representing a system state.
 * @param measurementArgs Additional arguments used to calculate the measurement. In this implementation it is unused.
 * @return The expected measurement for the given states.
 */
Matrix SharedMobileObjectModel::measurementEquation(const Matrix& state, const Matrix& measurementArgs, unsigned int type)
{
    if(type == kshared_measurement)
    {
        return sharedMeasurementEquation(state, measurementArgs);
    }
}

Matrix SharedMobileObjectModel::sharedMeasurementEquation(const Matrix& state, const Matrix& measurementArgs)
{
    // measurementArgs not required, since the measurements are only reliant on the current state.
    // Measurement is to be in field coordinated (x,y)

    // Get position from sigma point.
    const double x = state[kstates_x_pos][0];
    const double y = state[kstates_y_pos][0];

    // Write to matrix for return.
    Matrix expected_measurement(2,1,false);
    expected_measurement[0][0] = x;
    expected_measurement[1][0] = y;

    return expected_measurement;
}

Matrix SharedMobileObjectModel::measurementDistance(const Matrix& measurement1, const Matrix& measurement2, unsigned int type)
{
    Matrix result = measurement1 - measurement2;
    return result;
}

/*!
@brief Outputs a binary representation of the UKF object to a stream.
@param output The output stream.
@return The output stream.
*/
std::ostream& SharedMobileObjectModel::writeStreamBinary (std::ostream& output) const
{
    return output;
}

/*!
@brief Reads in a UKF object from the input stream.
@param input The input stream.
@return The input stream.
*/
std::istream& SharedMobileObjectModel::readStreamBinary (std::istream& input)
{
    return input;
}
