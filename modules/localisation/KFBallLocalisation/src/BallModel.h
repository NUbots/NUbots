#ifndef MODULES_LOCALISATION_BALLMODEL_H
#define MODULES_LOCALISATION_BALLMODEL_H

#include <armadillo>
#include "messages/localisation/FieldObject.h"

namespace modules {
namespace localisation {
namespace ball {
    // Number of dimensions
    // The state consists of 3 components:
    //    1. The x position in robot space
    //    2. The y position in robot space
    //    3. The x component of velocity
    //    4. The y component of velocity
    enum BallModelStateComponents {
        kX = 0,
        kY = 1,
        kVx = 2,
        kVy = 3,
    };

    class BallModel {
    public:
        static constexpr size_t size = 4;
        
        BallModel() {} // empty constructor
        
        arma::vec::fixed<size> timeUpdate(
            const arma::vec::fixed<size>& state, double deltaT,
            const messages::localisation::FakeOdometry& odom);

        arma::vec::fixed<size> timeUpdate(
            const arma::vec::fixed<size>& state, double deltaT, std::nullptr_t foo);

        arma::vec predictedObservation(const arma::vec::fixed<size>& state, std::nullptr_t unused);

        arma::vec observationDifference(const arma::vec& a, const arma::vec& b);
        
        arma::vec::fixed<size> limitState(const arma::vec::fixed<size>& state);
        
        arma::mat::fixed<size, size> processNoise();

        // TODO: Add to config system?
        // static constexpr double processNoiseFactor = 1e-6;
        static constexpr double processNoiseFactor = 1e-3;
    };
}
}
}
#endif
