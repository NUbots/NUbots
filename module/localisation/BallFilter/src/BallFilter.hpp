
#ifndef MODULE_LOCALISATION_BALLFILTER_HPP
#define MODULE_LOCALISATION_BALLFILTER_HPP

#include <nuclear>

#include "message/input/Sensors.hpp"
#include "message/vision/Ball.hpp"


namespace module::localisation {

    class BallFilter : public NUClear::Reactor {
    private:
        struct Config {
            Config()               = default;
            float smoothing_factor = 0.0f;
        } cfg;

        /// @brief Current estimate of cartesian ball position
        Eigen::Vector3f filtered_rBCc = Eigen::Vector3f(1.0, 0.0, 0.0);

    public:
        /// @brief Called by the powerplant to build and setup the BallFilter reactor.
        explicit BallFilter(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_BALLFILTER_HPP
