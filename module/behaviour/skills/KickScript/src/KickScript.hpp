/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef MODULE_BEHAVIOUR_SKILLS_KICKSCRIPT_HPP
#define MODULE_BEHAVIOUR_SKILLS_KICKSCRIPT_HPP

#include <nuclear>

#include "message/motion/KickCommand.hpp"


namespace module::behaviour::skills {

    class KickScript : public NUClear::Reactor {

    private:
        /// @brief The id registered in the subsumption system for this module
        const size_t subsumption_id;

        /// @brief Stores configuration values
        struct Config {
            Config() = default;
            /// @brief Value that priority is set to when kick is requested
            double kick_priority = 0.0;
            /// @brief Time between kick command message and kicking before kick is discarded (milliseconds)
            int message_timeout = 0;
        } cfg;

        /// @brief Time when the last kick command message was received
        NUClear::clock::time_point time_since_message{NUClear::clock::now()};

        /// @brief The last kick command received, nullptr if none received yet
        std::shared_ptr<message::motion::KickScriptCommand> kick_command{nullptr};

        /// @brief Updates the priority of the module by emitting an ActionPriorities message
        /// @param priority The priority used in the ActionPriorities message
        void update_priority(const double& priority);

    public:
        /// @brief Called by the powerplant to build and setup the KickScript reactor.
        explicit KickScript(std::unique_ptr<NUClear::Environment> environment);
    };


}  // namespace module::behaviour::skills


#endif
