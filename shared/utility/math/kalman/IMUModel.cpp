#include <math.h> //needed for normalisation function
#include <assert.h>
#include "IMUModel.h" //includes armadillo


//#include "Tools/Math/General.h" //replace with armadillo library
//#include <assert.h>

//---------------------originally included in Math/general.h ::these functions can (and should) be moved to utilites if used by other files
//const double PI = 4.0*atan(1.0);          //this already exists as M_PI in the <cmath> aka <math.h> library, redundant calculation
inline double normaliseAngle(double theta){ //a helper function (originally in Math/general.h - required, can be ported to utilities)
    return atan2(sin(theta), cos(theta));
}
template <class T> inline int sign(T x){
  if(x < 0.0) return -1;
  else if(x > 0.0) return 1;
  else return 0;
}
//---------------------

IMUModel::IMUModel() {}

IMUModel::IMUModel(const IMUModel& source) {
    *this = source;
}

void IMUModel::limitState(arma::mat &state) {
    const float pi_2 = 0.5 * M_PI;
    if(fabs(state(kstates_body_angle_x, 0)) > pi_2 and fabs(state(kstates_body_angle_y, 0)) > pi_2) { // This part checks for the event where a large roll and large pitch puts the robot back upright
        state(kstates_body_angle_x, 0) = state(kstates_body_angle_x, 0) - sign(state(kstates_body_angle_x, 0)) * M_PI;
        state(kstates_body_angle_y, 0) = state(kstates_body_angle_y, 0) - sign(state(kstates_body_angle_y, 0)) * M_PI;
        state(kstates_body_angle_z, 0) = state(kstates_body_angle_z, 0) - sign(state(kstates_body_angle_z, 0)) * M_PI;
    }

    // Regular unwrapping.
    state(kstates_body_angle_x, 0) = normaliseAngle(state(kstates_body_angle_x, 0));
    state(kstates_body_angle_y, 0) = normaliseAngle(state(kstates_body_angle_y, 0));
    state(kstates_body_angle_z, 0) = normaliseAngle(state(kstates_body_angle_z, 0));
    return;
}

// @brief The process equation is used to update the systems state using the process euquations of the system. 
// @param sigma_point The sigma point representing a system state. 
// @param deltaT The amount of time that has passed since the previous update, in seconds. 
// @param measurement The reading from the rate gyroscope in rad/s used to update the orientation. 
// @return The new estimated system state.
arma::mat IMUModel::processEquation(const arma::mat& state, double deltaT, const arma::mat& measurement) { 
    arma::mat result(state); // Start at original state.
    result(kstates_body_angle_x, 0) += (measurement(0, 0) - state(kstates_gyro_offset_x, 0)) * deltaT; // Add measurement + offset.
    result(kstates_body_angle_y, 0) += (measurement(1, 0) - state(kstates_gyro_offset_y, 0)) * deltaT;
    result(kstates_body_angle_z, 0) += (measurement(2, 0) - state(kstates_gyro_offset_z, 0)) * deltaT;
    limitState(result); 
    return result;
}

// @brief The measurement equation is used to calculate the expected measurement given a system state.
// @param sigma_point The sigma point representing a system state.
// @param measurementArgs Additional arguments used to calculate the measurement. In this implementation it remains empty.
// @return The expected measurement for the given states. Either accelerometer measurement, or kinematic measurement.
arma::mat IMUModel::kinematicMeasurementEquation(const arma::mat& state, const arma::mat& measurementArgs) {
    // measurementArgs contain no data.
    // Measurement is returned in angle [theta_x, theta_y]^T.

    arma::mat result(3,1);
    result(0, 0) = state(kstates_body_angle_x, 0);
    result(1, 0) = state(kstates_body_angle_y, 0);
    result(2, 0) = state(kstates_body_angle_z, 0);
    return result;
}

arma::mat IMUModel::accelerometerMeasurementEquation(const arma::mat& state, const arma::mat& measurementArgs) {
    // measurementArgs contain no data.
    // Measurement is returned as accelerations [x, y, z]^T.
    // Gravity vector - Starts as pointing down, z =s 980.7 cm/s^2 as
    
    arma::vec g_vec(3, 1);
    gVec(0, 0) = 0.f;
    gVec(1, 0) = 0.f;
    gVec(2, 0) = -9.807f; //changed to m/s^2 from cm/s^2

    const double body_roll  = state(kstates_body_angle_x, 0); //state ~ input parameter
    const double body_pitch = state(kstates_body_angle_y, 0);
    const double body_yaw = state(kstates_body_angle_z, 0);
    
    //new universal rotation code for gyro
    //See: http://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation#Simultaneous_orthogonal_rotation_angle
    const double phi = arma::sqrt(body_roll*body_roll,body_pitch*body_pitch,body_yaw*body_yaw);
    const arma::vec omega({body_roll/phi,body_pitch/phi,body_yaw/phi});
    const double cosPhi = cos(phi);
    const auto omegaCrossG = arma::cross(omega,gVec);
    
    arma::vec result = gVec*cosPhi + omegaCrossG*sin(Phi) + omega*omegaCrossG*(1.0-cosPhi);
    
    /* //this is BAD CODE - do not use.
    arma::mat body_roll_rot = arma::mat(3,3);
    body_roll_rot.eye(); //body_roll_rot should be initialised as an identity matrix
    sinA = sin(body_roll);
    cosA = cos(body_roll);
    body_roll_rot(1, 1) = cosA;
    body_roll_rot(1, 2) = sinA;
    body_roll_rot(2, 1) = -sinA;
    body_roll_rot(2, 2) = cosA;

    arma::mat body_pitch_rot = arma::mat(3,3);
    body_pitch_rot.eye(); //generate identity matrix
    sinA = sin(body_pitch);
    cosA = cos(body_pitch);
    body_pitch_rot(0, 0) = cosA;
    body_pitch_rot(0, 2) = -sinA;
    body_pitch_rot(2, 0) = sinA;
    body_pitch_rot(2, 2) = cosA;
    arma::mat result = body_roll_rot * body_pitch_rot * g_vec; //this line armadillo hates ..........................................!
    */
    return result;
}

arma::mat IMUModel::measurementEquation(const arma::mat& state, const arma::mat& measurementArgs, unsigned int type) { // @brief The measurement equation is used to calculate the expected measurement given a system state. @param sigma_point The sigma point representing a system state. @param measurementArgs Additional arguments used to calculate the measurement. In this implementation it remains empty. @return The expected measurement for the given states. Either accelerometer measurement, or kinematic measurement.
    arma::mat result;
    switch(type) {
        case kmeasurement_accelerometer:
            result = accelerometerMeasurementEquation(state, measurementArgs);
            break;
        case kmeasurement_kinematic:
            result = kinematicMeasurementEquation(state, measurementArgs);
            break;
    };
    return result;
}

arma::mat IMUModel::measurementDistance(const arma::mat& measurement1, const arma::mat& measurement2, unsigned int type) {
    arma::mat result;
    switch(type)
    {
        case kmeasurement_accelerometer:
            result = measurement1 - measurement2;
            break;
        case kmeasurement_kinematic:
            result = measurement1 - measurement2;
            result(0, 0) = normaliseAngle(result(0, 0));
            result(1, 0) = normaliseAngle(result(1, 0));
            result(1, 0) = normaliseAngle(result(2, 0));
            break;
    };
    return result;
}

// @brief Outputs a binary representation of the UKF object to a stream. 
// @param output The output stream. 
// @return The output stream.
std::ostream& IMUModel::writeStreamBinary (std::ostream& output) const {
    return output;
}

// @brief Reads in a UKF object from the input stream. 
// @param input The input stream. 
// @return The input stream.
std::istream& IMUModel::readStreamBinary (std::istream& input) {
    return input;
}