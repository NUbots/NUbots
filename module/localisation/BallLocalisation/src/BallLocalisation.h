#ifndef MODULE_LOCALISATION_BALLLOCALISATION_H
#define MODULE_LOCALISATION_BALLLOCALISATION_H

#include <nuclear>

namespace module {
namespace localisation {

    class BallLocalisation : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the BallLocalisation reactor.
        explicit BallLocalisation(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_LOCALISATION_BALLLOCALISATION_H
