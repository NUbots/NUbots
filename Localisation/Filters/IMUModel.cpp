#include "IMUModel.h"
#include "Tools/Math/General.h"
#include <assert.h>

IMUModel::IMUModel()
{
}

IMUModel::IMUModel(const IMUModel& source)
{
    *this = source;
}

/*!
 * @brief The process equation is used to update the systems state using the process euquations of the system.
 * @param sigma_point The sigma point representing a system state.
 * @param deltaT The amount of time that has passed since the previous update, in seconds.
 * @param measurement The reading from the rate gyroscope in rad/s used to update the orientation.
 * @return The new estimated system state.
 */
Matrix IMUModel::processEquation(const Matrix& state, double deltaT, const Matrix& measurement)
{
    Matrix result(state); // Start at original state.
    result[kstates_body_angle_x][0] += (measurement[0][0] - state[kstates_gyro_offset_x][0]) * deltaT; // Add measurement + offset.
    result[kstates_body_angle_y][0] += (measurement[1][0] - state[kstates_gyro_offset_y][0]) * deltaT;
    limitState(result);
    return result;
}

/*!
 * @brief The measurement equation is used to calculate the expected measurement given a system state.
 * @param sigma_point The sigma point representing a system state.
 * @param measurementArgs Additional arguments used to calculate the measurement. In this implementation it remains empty.
 * @return The expected measurement for the given states. Either accelerometer measurement, or kinematic measurement.
 */
Matrix IMUModel::measurementEquation(const Matrix& state, const Matrix& measurementArgs, unsigned int type)
{
    Matrix result;
    switch(type)
    {
        case kmeasurement_accelerometer:
            result = accelerometerMeasurementEquation(state, measurementArgs);
            break;
        case kmeasurement_kinematic:
            result = kinematicMeasurementEquation(state, measurementArgs);
            break;
    };
    return result;
}

Matrix IMUModel::measurementDistance(const Matrix& measurement1, const Matrix& measurement2, unsigned int type)
{
    Matrix result;
    switch(type)
    {
        case kmeasurement_accelerometer:
            result = measurement1 - measurement2;
            break;
        case kmeasurement_kinematic:
            result = measurement1 - measurement2;
            result[0][0] = mathGeneral::normaliseAngle(result[0][0]);
            result[1][0] = mathGeneral::normaliseAngle(result[1][0]);
            break;
    };
    return result;
}

void IMUModel::limitState(Matrix &state)
{
    const float pi_2 = 0.5 * mathGeneral::PI;
    // This part checks for the event where a large roll and large pitch puts the robot back upright
    if(fabs(state[kstates_body_angle_x][0]) > pi_2 and fabs(state[kstates_body_angle_y][0]) > pi_2)
    {
        state[kstates_body_angle_x][0] = state[kstates_body_angle_x][0] - mathGeneral::sign(state[kstates_body_angle_x][0]) * mathGeneral::PI;
        state[kstates_body_angle_y][0] = state[kstates_body_angle_y][0] - mathGeneral::sign(state[kstates_body_angle_y][0]) * mathGeneral::PI;
    }

    // Regular unwrapping.
    state[kstates_body_angle_x][0] = mathGeneral::normaliseAngle(state[kstates_body_angle_x][0]);
    state[kstates_body_angle_y][0] = mathGeneral::normaliseAngle(state[kstates_body_angle_y][0]);
    return;
}

Matrix IMUModel::kinematicMeasurementEquation(const Matrix& state, const Matrix& measurementArgs)
{
    // measurementArgs contain no data.
    // Measurement is returned in angle [theta_x, theta_y]^T.

    Matrix result(2,1,false);
    result[0][0] = state[kstates_body_angle_x][0];
    result[1][0] = state[kstates_body_angle_y][0];
    return result;
}

Matrix IMUModel::accelerometerMeasurementEquation(const Matrix& state, const Matrix& measurementArgs)
{
    // measurementArgs contain no data.
    // Measurement is returned as accelerations [x, y, z]^T.

    // Gravity vector - Starts as pointing down, z =s 980.7 cm/s^2 as
    Matrix g_vec(3,1,false);
    g_vec[0][0] = 0.f;
    g_vec[1][0] = 0.f;
    g_vec[2][0] = -980.7f;

    const double body_roll = state[kstates_body_angle_x][0];
    const double body_pitch = state[kstates_body_angle_y][0];
    float sinA, cosA;

    Matrix body_roll_rot = Matrix(3,3,true);
    sinA = sin(body_roll);
    cosA = cos(body_roll);
    body_roll_rot[1][1] = cosA;
    body_roll_rot[1][2] = sinA;
    body_roll_rot[2][1] = -sinA;
    body_roll_rot[2][2] = cosA;

    Matrix body_pitch_rot = Matrix(3,3,true);
    sinA = sin(body_pitch);
    cosA = cos(body_pitch);
    body_pitch_rot[0][0] = cosA;
    body_pitch_rot[0][2] = -sinA;
    body_pitch_rot[2][0] = sinA;
    body_pitch_rot[2][2] = cosA;

    Matrix result = body_roll_rot * body_pitch_rot * g_vec;
    return result;
}

/*!
@brief Outputs a binary representation of the UKF object to a stream.
@param output The output stream.
@return The output stream.
*/
std::ostream& IMUModel::writeStreamBinary (std::ostream& output) const
{
    return output;
}

/*!
@brief Reads in a UKF object from the input stream.
@param input The input stream.
@return The input stream.
*/
std::istream& IMUModel::readStreamBinary (std::istream& input)
{
    return input;
}
