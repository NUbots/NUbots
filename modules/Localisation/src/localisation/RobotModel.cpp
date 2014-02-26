#ifndef UTILITY_MATH_KALMAN_ADAPTIVEIMUMODEL_H
#define UTILITY_MATH_KALMAN_ADAPTIVEIMUMODEL_H

#include "RobotModel.h"

#include <armadillo>

#include "utility/math/angle.h"
#include "utility/math/coordinates.h"

namespace modules {
namespace localisation {

arma::vec::fixed<RobotModel::size> RobotModel::timeUpdate(
    const arma::vec::fixed<RobotModel::size>& state, double deltaT, 
    const arma::vec3& measurement) {

    auto result = state;

    // double interp_heading = state[kHeading] + 0.5 * measurement[kHeading];

    // double cos_theta = cos(interp_heading);
    // double sin_theta = sin(interp_heading);

    // result[kX]       += deltaT * (measurement[kX] * cos_theta - measurement[kY] * sin_theta);
    // result[kY]       += deltaT * (measurement[kX] * sin_theta + measurement[kY] * cos_theta);
    // result[kHeading] += deltaT * (measurement[kHeading]);
    
    return result;
}

/// Return the predicted observation of an object at the given position
arma::vec RobotModel::predictedObservation(
    const arma::vec::fixed<RobotModel::size>& state, const arma::vec2& actual_position) {
    
    arma::vec2 diff = actual_position - state.rows(0, 1);

    // auto distance = arma::norm(diff, 2);

    // auto angle = utility::math::angle::normalizeAngle(atan2(diff[1], diff[0]) - state[kHeading]);

    // return { distance, angle };

    arma::vec2 radial = utility::math::coordinates::Cartesian2Radial(diff);

    radial(1) = utility::math::angle::normalizeAngle(radial[1] - state[kHeading]);

    return radial;

    // switch(type)
    // {
    //     case MeasurementType::kLandmarkMeasurement:
    //         return landmarkMeasurementEquation(state);

    //     case MeasurementType::kAngleBetweenLandmarksMeasurement:
    //         return angleBetweenLandmarkMeasurementEquation(state);

    //     default:
    //         return arma::vec::fixed<RobotModel::size>();
    // };
}

arma::vec RobotModel::observationDifference(const arma::vec& a, 
                                            const arma::vec& b){
    
    arma::vec2 result = a - b;

    result(1) = utility::math::angle::normalizeAngle(result[1]);

    return result;


    // Matrix result;
    // switch(type)
    // {
    //     case MeasurementType::kLandmarkMeasurement:
    //         result = measurement1 - measurement2;
    //         result[1] = utility::math::angle::normalizeAngle(result[1]);
    //         break;
    //     case MeasurementType::kAngleBetweenLandmarksMeasurement:
    //         result = measurement1 - measurement2;
    //         result[0] = utility::math::angle::normalizeAngle(result[0]);
    //         break;
    // };
    // return result;
}




arma::vec::fixed<RobotModel::size> RobotModel::limitState(
    const arma::vec::fixed<RobotModel::size>& state) {
 
    auto result = state;

    // How to get clipping values from config system?
    result[kX] = std::max(std::min(result[kX], 4.5 + 0.7) , -4.5 -0.7);
    result[kY] = std::max(std::min(result[kY], 3 + 0.7) , -3 -0.7);
    result[kHeading] = utility::math::angle::normalizeAngle(result[kHeading]);
    
    return result;
}

arma::mat::fixed<RobotModel::size, RobotModel::size> RobotModel::processNoise() {
    return arma::eye(RobotModel::size, RobotModel::size) * processNoiseFactor;
}

}
}
#endif

