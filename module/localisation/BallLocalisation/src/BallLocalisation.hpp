#ifndef MODULE_LOCALISATION_BALLLOCALISATION_HPP
#define MODULE_LOCALISATION_BALLLOCALISATION_HPP

#include <nuclear>

#include "BallModel.hpp"

#include "utility/math/filter/ParticleFilter.hpp"
#include "utility/math/filter/UKF.hpp"

namespace module::localisation {

    class BallLocalisation : public NUClear::Reactor {
    private:
        utility::math::filter::ParticleFilter<float, BallModel> filter;
        NUClear::clock::time_point last_time_update_time;
        NUClear::clock::time_point last_measurement_update_time;
        bool ball_pos_log;

        struct {
            std::vector<Eigen::Vector2f> start_state{};
            Eigen::Vector2f start_variance = Eigen::Vector2f::Zero();
        } config{};

    public:
        /// @brief Called by the powerplant to build and setup the BallLocalisation reactor.
        explicit BallLocalisation(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_BALLLOCALISATION_HPP
