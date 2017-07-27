#ifndef MODULE_GAMECONTROLLERSIMULATOR_H
#define MODULE_GAMECONTROLLERSIMULATOR_H

#include <nuclear>

namespace module {

    class GameControllerSimulator : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the GameControllerSimulator reactor.
        explicit GameControllerSimulator(std::unique_ptr<NUClear::Environment> environment);
    };

}

#endif  // MODULE_GAMECONTROLLERSIMULATOR_H
