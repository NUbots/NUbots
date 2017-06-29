#ifndef MODULE_LOCALISATION_ROBOTPARTICLELOCALISATION_H
#define MODULE_LOCALISATION_ROBOTPARTICLELOCALISATION_H

#include <nuclear>
#include "RobotModel.h"

namespace module {
namespace localisation {

    class RobotParticleLocalisation : public NUClear::Reactor {
    private:
        utility::math::filter::ParticleFilter<RobotModel> filter;
        NUClear::clock::time_point last_time_update_time;
        NUClear::clock::time_point last_measurement_update_time;
    public:
        /// @brief Called by the powerplant to build and setup the RobotParticleLocalisation reactor.
        explicit RobotParticleLocalisation(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_LOCALISATION_ROBOTPARTICLELOCALISATION_H
