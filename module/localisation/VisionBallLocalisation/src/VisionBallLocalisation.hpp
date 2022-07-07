#ifndef MODULE_LOCALISATION_BALLLOCALISATION_HPP
#define MODULE_LOCALISATION_BALLLOCALISATION_HPP

#include <nuclear>

namespace module::localisation {

    class VisionBallLocalisation : public NUClear::Reactor {
    private:
    public:
        /// @brief Called by the powerplant to build and setup the VisionBallLocalisation reactor.
        explicit VisionBallLocalisation(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_BALLLOCALISATION_HPP
