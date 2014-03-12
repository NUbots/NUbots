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

    arma::vec2 radial = utility::math::coordinates::Cartesian2Radial(state.rows(0, 1));

    auto heading_angle = radial[1];
    auto heading_x = std::cos(heading_angle);
    auto heading_y = std::sin(heading_angle);

    // arma::vec2 heading = arma::normalise(diff);
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
}

arma::mat::fixed<BallModel::size, BallModel::size> BallModel::processNoise() {
    return arma::eye(BallModel::size, BallModel::size) * processNoiseFactor;
}

}
}
