#ifndef MODULE_BEHAVIOUR_PLANNING_RANDOMHEAD_H
#define MODULE_BEHAVIOUR_PLANNING_RANDOMHEAD_H

#include <nuclear>
#include <random>

namespace module {
namespace behaviour {
    namespace planning {

        class RandomHead : public NUClear::Reactor {

        public:
            /// @brief Called by the powerplant to build and setup the RandomHead reactor.
            explicit RandomHead(std::unique_ptr<NUClear::Environment> environment);

        private:
            std::random_device rd;  // Will be used to obtain a seed for the random number engine
            std::mt19937 gen;       // Standard mersenne_twister_engine seeded with rd()

            float freq, pitch_limit, yaw_limit;
        };

    }  // namespace planning
}  // namespace behaviour
}  // namespace module

#endif  // MODULE_BEHAVIOUR_PLANNING_RANDOMHEAD_H
