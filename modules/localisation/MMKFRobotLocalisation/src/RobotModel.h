#ifndef MODULES_LOCALISATION_ROBOTMODEL_H
#define MODULES_LOCALISATION_ROBOTMODEL_H

#include <armadillo>
#include "messages/localisation/FieldObject.h"

namespace modules {
namespace localisation {
namespace robot {
    // Number of dimensions
    // The state consists of 3 components:
    //    1. The x position on the field
    //    2. The y position on the field
    //    3. The robot's heading (in radians)
    enum RobotModelStateComponents {
        kX = 0,
        kY = 1,
        // kHeading = 2,
        kHeadingX = 2,
        kHeadingY = 3,
    };

    enum class MeasurementType {
        kBRGoalMeasurement,
        kBLGoalMeasurement,
        kLandmarkMeasurement,
        kAngleBetweenLandmarksMeasurement,
    };

    class RobotModel {
    public:
        static constexpr size_t size = 4;
        
        RobotModel() {} // empty constructor
        
        arma::vec::fixed<RobotModel::size> timeUpdate(
            const arma::vec::fixed<RobotModel::size>& state, double deltaT,
            std::nullptr_t foo);

        arma::vec::fixed<RobotModel::size> timeUpdate(
            const arma::vec::fixed<RobotModel::size>& state, double deltaT, 
            const messages::localisation::FakeOdometry& odom);
        
        arma::vec predictedObservation(
            const arma::vec::fixed<RobotModel::size>& state, 
            const arma::vec2& actual_position);

        arma::vec observationDifference(const arma::vec& a, const arma::vec& b);
        
        arma::vec::fixed<size> limitState(const arma::vec::fixed<size>& state);
        
        arma::mat::fixed<size, size> processNoise();

        // static constexpr double processNoiseFactor = 1e-6;
        static constexpr double processNoiseFactor = 1e-3;
    };
}
}
}
#endif
