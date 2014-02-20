#ifndef MODULES_LOCALISATION_ROBOTMODEL_H
#define MODULES_LOCALISATION_ROBOTMODEL_H

#include <armadillo>
namespace modules {
    namespace localisation {

        // Number of dimensions
        // The state consists of 3 components:
        //    1. The x position on the field
        //    2. The y position on the field
        //    3. The robot's heading (in radians)
        enum RobotModelStateComponents {
            kX = 0,
            kY = 1,
            kHeading = 2,
        };

        enum class MeasurementType {
            kLandmarkMeasurement,
            kAngleBetweenLandmarksMeasurement,
        };

        class RobotModel {
        public:            
            static constexpr size_t size = 3;
            
            RobotModel() {} // empty constructor
            
            arma::vec::fixed<size> timeUpdate(const arma::vec::fixed<size>& state, double deltaT, const arma::vec3& measurement);
            
            arma::vec predictedObservation(const arma::vec::fixed<size>& state, MeasurementType type);
            
            arma::vec observationDifference(const arma::vec& a, const arma::vec& b);
            
            arma::vec::fixed<size> limitState(const arma::vec::fixed<size>& state);
            
            arma::mat::fixed<size, size> processNoise();

            static constexpr double processNoiseFactor = 1e-6;
        };
    }
}
#endif
