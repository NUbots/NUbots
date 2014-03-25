/*
 * This file is part of MockRobot.
 *
 * MockRobot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * MockRobot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MockRobot.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_LOCALISATION_MOCKROBOT_H
#define MODULES_LOCALISATION_MOCKROBOT_H

#include <nuclear>
#include <armadillo>
#include "utility/localisation/FieldDescription.h"

namespace modules {
namespace localisation {

    class MockRobot : public NUClear::Reactor {
    private:
        arma::vec2 ball_position_ = { 0, 0 };
        arma::vec2 ball_velocity_ = { 0, 0 };
        arma::vec2 robot_position_ = { 0, 0 };
        arma::vec2 robot_velocity_ = { 0, 0 };
        arma::vec2 robot_heading_ = { 1, 0 };

        std::shared_ptr<utility::localisation::FieldDescription> field_description_;

    public:
        /// @brief Called by the powerplant to build and setup the MockRobot reactor.
        explicit MockRobot(std::unique_ptr<NUClear::Environment> environment);
    };

}
}


#endif