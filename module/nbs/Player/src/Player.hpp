#ifndef MODULE_NBS_PLAYER_HPP
#define MODULE_NBS_PLAYER_HPP

#include <filesystem>
#include <fstream>
#include <memory>
#include <nuclear>

#include "message/eye/Scrubber.hpp"

#include "utility/nbs/Decoder.hpp"

namespace module::nbs {

    class Player : public NUClear::Reactor {
    public:
        /// Called by the powerplant to build and setup the Player reactor.
        explicit Player(std::unique_ptr<NUClear::Environment> environment);

        /// NBS file decoder
        utility::nbs::Decoder decoder;
        /// NBS file iterator
        utility::nbs::Decoder::Iterator decoder_iterator = decoder.begin();

        /// Player state
        message::eye::ScrubberState state;

        /// The task id that is used for the chrono task
        NUClear::id_t chrono_task_id = 0;

        /// Returns the current real time factor
        double rtf() const;

        /// Updates the clock
        void update_clock();

        /// Binds the player to the chrono task
        void bind_player();

        /// Unbinds the player from the chrono task
        void unbind_player();
    };

}  // namespace module::nbs

#endif  // MODULE_NBS_PLAYER_HPP
