#include "BallModel.h"

#include <armadillo>

#include "utility/math/angle.h"
#include "utility/math/coordinates.h"

namespace modules {
namespace localisation {

arma::vec::fixed<BallModel::size> BallModel::timeUpdate(
    const arma::vec::fixed<BallModel::size>& state, double deltaT, 
    const arma::vec3& measurement) {

    auto result = state;

    // Apply ball velocity
    result[kX] += state[kVx] * deltaT;
    result[kY] += state[kVy] * deltaT;
    const double kDragCoefficient = 1.0; // TODO: Config system
    result[kVx] *= kDragCoefficient;
    result[kVy] *= kDragCoefficient;

    // result[kY] = utility::math::angle::normalizeAngle(state[kY]);

    // Apply robot odometry / robot position change

    // double interp_heading = state[kHeading] + 0.5 * measurement[kHeading];
    // double cos_theta = cos(interp_heading);
    // double sin_theta = sin(interp_heading);
    // result[kX]       += (measurement[kX] * cos_theta - measurement[kY] * sin_theta);
    // result[kY]       += (measurement[kX] * sin_theta + measurement[kY] * cos_theta);
    // result[kHeading] += (measurement[kHeading]);
    
    return result;
}

/// Return the predicted observation of an object at the given position
arma::vec BallModel::predictedObservation(
    const arma::vec::fixed<BallModel::size>& state, std::nullptr_t unused) {

    // // Robot-relative cartesian
    // return { state[kX], state[kY] };

    // Distance and unit vector heading
    arma::vec2 radial = utility::math::coordinates::Cartesian2Radial(state.rows(0, 1));
    auto heading_angle = radial[1];
    auto heading_x = std::cos(heading_angle);
    auto heading_y = std::sin(heading_angle);
    return {radial[0], heading_x, heading_y};
}

arma::vec BallModel::observationDifference(const arma::vec& a, 
                                            const arma::vec& b){
    // Distance and unit vector heading
    return a - b;
}

arma::vec::fixed<BallModel::size> BallModel::limitState(
    const arma::vec::fixed<BallModel::size>& state) {
 
    return { state[kX], state[kY], state[kVx], state[kVy] };
    

    // // Radial coordinates
    // return { state[kX], 
    //     utility::math::angle::normalizeAngle(state[kY]),
    //     state[kVx], state[kVy] };
}

arma::mat::fixed<BallModel::size, BallModel::size> BallModel::processNoise() {
    arma::mat noise = arma::eye(BallModel::size, BallModel::size) * processNoiseFactor;

    // noise(kX, kX) = processNoiseFactor * 100;
    // noise(kY, kY) = processNoiseFactor * 100;
    noise(kVx, kVx) = processNoiseFactor * 10;
    noise(kVy, kVy) = processNoiseFactor * 10;

    return noise;
}

}
}
