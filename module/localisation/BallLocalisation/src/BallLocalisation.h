#ifndef MODULE_LOCALISATION_BALLLOCALISATION_H
#define MODULE_LOCALISATION_BALLLOCALISATION_H

#include <nuclear>

#include "BallModel.h"
#include "utility/math/filter/ParticleFilter.h"
#include "utility/math/filter/UKF.h"

namespace module {
namespace localisation {

    class BallLocalisation : public NUClear::Reactor {
    private:
        utility::math::filter::ParticleFilter<BallModel> filter;
        NUClear::clock::time_point last_time_update_time;
        NUClear::clock::time_point last_measurement_update_time;
        bool ball_pos_log;


    public:
        /// @brief Called by the powerplant to build and setup the BallLocalisation reactor.
        explicit BallLocalisation(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace localisation
}  // namespace module

#endif  // MODULE_LOCALISATION_BALLLOCALISATION_H
