#ifndef MODULE_NETWORK_RERUN_HPP
#define MODULE_NETWORK_RERUN_HPP

#include <nuclear>
#include <rerun.hpp>
#include <string>

namespace module::network {

    class Rerun : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the Rerun reactor.
        explicit Rerun(std::unique_ptr<NUClear::Environment> environment);

    private:
        /// @brief The robot's current hostname (included in data sent, to enable filtering)
        std::string hostname;

        /// @brief Handle to the reaction that forwards DataPoints to Rerun
        ReactionHandle forwarder_reaction{};

        /// @brief Rerun recording session pointer
        std::unique_ptr<rerun::RecordingStream> recording;
    };

}  // namespace module::network

#endif  // MODULE_NETWORK_RERUN_HPP
