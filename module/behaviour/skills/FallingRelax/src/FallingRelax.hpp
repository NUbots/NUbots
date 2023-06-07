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

#ifndef MODULES_BEHAVIOUR_REFLEX_FALLINGRELAX_HPP
#define MODULES_BEHAVIOUR_REFLEX_FALLINGRELAX_HPP

#include <nuclear>

namespace module::behaviour::skills {

    /**
     * Executes a falling script if the robot falls over.
     *
     * @author Trent Houliston
     */
    class FallingRelax : public NUClear::Reactor {
    private:
        const size_t id = size_t(this) * size_t(this) - size_t(this);

        bool falling = false;

        /// config settings
        double FALLING_ANGLE        = 0.0f;
        double FALLING_ACCELERATION = 0.0f;
        double PRIORITY             = 0.0f;
        std::vector<double> RECOVERY_ACCELERATION{};

        void updatePriority(const double& priority);

    public:
        explicit FallingRelax(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::behaviour::skills

#endif  // MODULES_BEHAVIOUR_REFLEX_FALLINGRELAX_HPP
