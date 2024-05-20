#ifndef MODULE_TOOLS_NBSPLAYBACK_HPP
#define MODULE_TOOLS_NBSPLAYBACK_HPP

#include <nuclear>

#include "message/nbs/player/Player.hpp"

#include "utility/support/ProgressBar.hpp"

namespace module::tools {

    class NBSPlayback : public NUClear::Reactor {
    private:
        /// The configuration variables for this reactor
        struct {
            /// @brief Playback mode
            message::nbs::player::PlaybackMode mode;
            /// @brief Messages to play
            std::vector<std::string> messages;
        } config;

        /// @brief Progress bar for the NBS file playback
        utility::support::ProgressBar progress_bar;


    public:
        /// @brief Called by the powerplant to build and setup the NBSPlayback reactor.
        explicit NBSPlayback(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::tools

#endif  // MODULE_TOOLS_NBSPLAYBACK_HPP
