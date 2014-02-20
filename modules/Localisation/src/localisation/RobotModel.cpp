#ifndef UTILITY_MATH_KALMAN_ADAPTIVEIMUMODEL_H
#define UTILITY_MATH_KALMAN_ADAPTIVEIMUMODEL_H

#include "RobotModel.h"

#include <armadillo>

#include "utility/math/angle.h"

namespace modules {
namespace localisation {

arma::vec::fixed<RobotModel::size> RobotModel::timeUpdate(
    const arma::vec::fixed<RobotModel::size>& state, double deltaT, 
    const arma::vec3& measurement) {

    auto result = state;

    double interp_heading = state[kHeading] + 0.5 * measurement[kHeading];

    double cos_theta = cos(interp_heading);
    double sin_theta = sin(interp_heading);

    result[kX] += measurement[kX] * cos_theta - measurement[kY] * sin_theta;
    result[kY] += measurement[kX] * sin_theta + measurement[kY] * cos_theta;
    result[kHeading] += measurement[kHeading];
    
    return result;
}

arma::vec RobotModel::predictedObservation(
    const arma::vec::fixed<RobotModel::size>& state, MeasurementType type) {
    
    return state;

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
    return a - b;

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

    result[kHeading] = utility::math::angle::normalizeAngle(result[kHeading]);
    
    return result;
}

arma::mat::fixed<RobotModel::size, RobotModel::size> RobotModel::processNoise() {

}

}
}
#endif
