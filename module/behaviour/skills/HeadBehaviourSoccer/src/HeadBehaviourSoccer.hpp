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

#ifndef MODULES_BEHAVIOUR_REFLEX_HEADBEHAVIOURSOCCER_HPP
#define MODULES_BEHAVIOUR_REFLEX_HEADBEHAVIOURSOCCER_HPP

#include <Eigen/Core>
#include <nuclear>
#include <set>

namespace module::behaviour::skills {

    /**
     * Executes a HeadBehaviourSoccer action.
     *
     * @author Thomas O'Brien
     */
    class HeadBehaviourSoccer : public NUClear::Reactor {
    public:
    private:
        // Time before starting to search for ball after its lost
        float search_timeout_ms = 0.0f;
        // Time lingering at each position in lost ballsearch
        float fixation_time_ms = 0.0f;
        // Index in the list of search positions
        long unsigned int searchIdx = 0;

        // Flag for if the robot is currently getting up
        bool isGettingUp = false;

        // Time between last search position transition
        NUClear::clock::time_point searchLastMoved = NUClear::clock::now();

        // List of positions for search
        std::vector<Eigen::Vector2d> search_positions;

        // Time since last ball seen
        NUClear::clock::time_point ballLastMeasured = NUClear::clock::now();

        // Position of last seen ball
        Eigen::Vector3d rBCc = Eigen::Vector3d::Zero();

    public:
        explicit HeadBehaviourSoccer(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::behaviour::skills

#endif  // MODULES_BEHAVIOURS_REFLEX_HEADBEHAVIOURSOCCER_HPP
