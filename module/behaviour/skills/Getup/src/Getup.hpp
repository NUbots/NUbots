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

#ifndef MODULES_BEHAVIOUR_REFLEX_GETUP_HPP
#define MODULES_BEHAVIOUR_REFLEX_GETUP_HPP

#include <nuclear>

namespace module::behaviour::skills {

    /**
     * Executes a getup script if the robot falls over.
     *
     * @author Josiah Walker
     * @author Trent Houliston
     */
    class Getup : public NUClear::Reactor {
    private:
        /// @brief The id registered in the subsumption system for this module
        const size_t subsumption_id;

        /// @brief Stores configuration values
        struct Config {
            Config() = default;
            /// @brief Value that priority is set to when getup is requested
            float getup_priority = 0.0f;
            /// @brief Threshold angle for executing getup, between torso z axis and world z axis
            float fallen_angle = 0.0f;
            /// @brief Script sequence to run when getting from lying on the front to standing
            std::vector<std::string> getup_front;
            /// @brief Script sequence to run when getting from lying on the back to standing
            std::vector<std::string> getup_back;
        } cfg;

        /// @brief Bool to indicate if the robot has fallen on its front or back
        bool is_front = false;

        /// @brief Bool to indicate if the robot is currently getting up
        bool getting_up = false;

        /// @brief Updates the priority of the module by emitting an ActionPriorities message
        /// @param priority The priority used in the ActionPriorities message
        void update_priority(const float& priority);

    public:
        explicit Getup(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::behaviour::skills

#endif  // MODULES_BEHAVIOUR_REFLEX_GETUP_HPP
