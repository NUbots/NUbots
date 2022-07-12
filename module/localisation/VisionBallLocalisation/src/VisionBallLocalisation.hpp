#ifndef MODULE_LOCALISATION_VISIONBALLLOCALISATION_HPP
#define MODULE_LOCALISATION_VISIONBALLLOCALISATION_HPP

#include <nuclear>

#include "message/input/Sensors.hpp"
#include "message/vision/Ball.hpp"


namespace module::localisation {
    using VisionBalls = message::vision::Balls;
    using VisionBall  = message::vision::Ball;
    using message::input::Sensors;

    class VisionBallLocalisation : public NUClear::Reactor {
    private:
        struct Config {
            Config() = default;

        } cfg;

        /// @brief Current estimate of cartesian ball position
        Eigen::Vector3f filtered_rBTt = Eigen::Vector3f(1.0, 0.0, 0.0);

        /// @brief Get distance to ball in x-y plance
        float get_distance(Eigen::Matrix<float, 3, 1> v);


    public:
        /// @brief Called by the powerplant to build and setup the VisionBallLocalisation reactor.
        explicit VisionBallLocalisation(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_BALLLOCALISATION_HPP
