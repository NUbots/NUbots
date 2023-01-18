
#ifndef MODULE_LOCALISATION_BALLFILTER_HPP
#define MODULE_LOCALISATION_BALLFILTER_HPP

#include <nuclear>

#include "message/input/Sensors.hpp"
#include "message/vision/Ball.hpp"

#include "utility/math/filter/KalmanFilter.hpp"


namespace module::localisation {

    class BallFilter : public NUClear::Reactor {
    private:
        struct Config {
            Config() = default;
        } cfg;

        /// @brief Get distance to ball in x-y plane
        float get_distance(Eigen::Matrix<float, 3, 1> v);

    public:
        /// @brief Called by the powerplant to build and setup the BallFilter reactor.
        explicit BallFilter(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Number of states in the filter
        static const size_t n_states = 3;

        /// @brief Number of inputs in the filter
        static const size_t n_inputs = 0;

        /// @brief Number of outputs (measurements) in the filter
        static const size_t n_outputs = 3;

        /// @brief The filter for the ball
        utility::math::filter::KalmanFilter<float, n_states, n_inputs, n_outputs> filter{};

        /// @brief The previous time that the filter was updated
        NUClear::clock::time_point last_update_time = NUClear::clock::now();
    };
}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_BALLFILTER_HPP
