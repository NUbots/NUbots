#ifndef MODULE_ACTUATION_MOCAPTOREAL_HPP
#define MODULE_ACTUATION_MOCAPTOREAL_HPP

#include <nuclear>

namespace module::actuation {

    class MocapToReal : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            bool output_to_servos = true;
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the MocapToReal reactor.
        explicit MocapToReal(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::actuation

#endif  // MODULE_ACTUATION_MOCAPTOREAL_HPP
