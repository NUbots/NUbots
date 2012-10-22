#include "RobotModel.h"
#include "Tools/Math/General.h"
#include <assert.h>
RobotModel::RobotModel()
{
    m_time_process_matrix = Matrix(3,3,false);

    m_time_process_matrix[states_x][states_x] = 0.5;

    m_time_process_matrix[states_y][states_y] = 0.5;

    m_time_process_matrix[states_heading][states_x] = 0.003;
    m_time_process_matrix[states_heading][states_y] = 0.003;
    m_time_process_matrix[states_heading][states_heading] = 0.5;
}

/*!
 * @brief The process equation is used to update the systems state using the process euquations of the system.
 * @param sigma_point The sigma point representing a system state.
 * @param deltaT The amount of time that has passed since the previous update, in seconds.
 * @param measurement The measurement of the odometry used to update the objects new relative position.
 * @return The new estimated system state.
 */
Matrix RobotModel::processEquation(const Matrix& state, double deltaT, const Matrix& measurement)
{
    Matrix result(state); // Start at original state.

    double interp_heading = state[states_heading][0] + 0.5 * measurement[states_heading][0];

    double cos_theta = cos(interp_heading);
    double sin_theta = sin(interp_heading);

    result[states_x][0] += measurement[states_x][0]*cos_theta - measurement[states_y][0]*sin_theta;
    result[states_y][0] +=  measurement[states_x][0]*sin_theta + measurement[states_y][0]*cos_theta;
    result[states_heading][0] += measurement[states_heading][0];

    return result;
}

/*!
 * @brief The measurement equation is used to calculate the expected measurement given a system state.
 * @param sigma_point The sigma point representing a system state.
 * @param measurementArgs Additional arguments used to calculate the measurement. In this implementation it contain the vector [x,y]^T location of the object observed.
 * @return The expected measurement for the given states.
 */
Matrix RobotModel::measurementEquation(const Matrix& state, const Matrix& measurementArgs)
{
    // measurementArgs contain the vector [x,y]^T location of the object observed.
    // Measurement is returned in polar coordinates [distance, theta]^T.

    unsigned int total_objects = measurementArgs.getm() / 2;

    Matrix expected_measurement(2*total_objects,1,false);

    // variables required.
    double dx,dy,distance,angle;
    unsigned int current_index;

    for (unsigned int object_number = 0; object_number < total_objects; ++object_number)
    {
        dx = measurementArgs[0][0] - state[states_x][0];
        dy = measurementArgs[1][0] - state[states_y][0];
        distance = sqrt(dx*dx + dy*dy);
        angle = mathGeneral::normaliseAngle(atan2(dy,dx) - state[states_heading][0]);

        // 2 measurements per object
        current_index = 2*object_number;

        // Write to matrix for return.
        expected_measurement[current_index][0] = distance;
        expected_measurement[current_index+1][0] = angle;
    }

    return expected_measurement;
}

/*!
@brief Outputs a binary representation of the UKF object to a stream.
@param output The output stream.
@return The output stream.
*/
std::ostream& RobotModel::writeStreamBinary (std::ostream& output) const
{
    return output;
}

/*!
@brief Reads in a UKF object from the input stream.
@param input The input stream.
@return The input stream.
*/
std::istream& RobotModel::readStreamBinary (std::istream& input)
{
    return input;
}
