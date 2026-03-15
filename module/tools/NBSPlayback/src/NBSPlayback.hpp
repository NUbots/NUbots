#ifndef MODULE_TOOLS_NBSPLAYBACK_HPP
#define MODULE_TOOLS_NBSPLAYBACK_HPP

#include <nuclear>

#include "message/eye/Scrubber.hpp"

#include "utility/support/ProgressBar.hpp"

namespace module::tools {

    class NBSPlayback : public NUClear::Reactor {
    private:
        /// The configuration variables for this reactor
        struct {
            /// @brief Playback mode
            message::eye::ScrubberState::Mode mode;
            /// @brief Messages to play
            std::vector<std::string> messages;
            /// @brief Progress bar mode
            std::string progress_bar_mode = "COUNT";
        } config;

        /// @brief Progress bar for the NBS file playback
        utility::support::ProgressBar progress_bar;


    public:
        /// @brief Called by the powerplant to build and setup the NBSPlayback reactor.
        explicit NBSPlayback(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::tools

#endif  // MODULE_TOOLS_NBSPLAYBACK_HPP
