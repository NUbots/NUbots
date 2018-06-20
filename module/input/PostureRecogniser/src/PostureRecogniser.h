/*
 * This file is part of NUbots Codebase.
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
 * Copyright 2015 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_INPUT_POSTURERECOGNISER_H
#define MODULES_INPUT_POSTURERECOGNISER_H

#include <yaml-cpp/yaml.h>
#include <armadillo>
#include <chrono>
#include <nuclear>

#include "extension/Configuration.h"

#include "message/input/PostureRecognition.h"
#include "message/input/Sensors.h"

#include "utility/input/ServoLoadModel.h"

#include "utility/math/filter/UKF.h"

#include "utility/time/time.h"

#include "utility/nusight/NUhelpers.h"

namespace module {
namespace input {
    class PostureRecogniser : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the PostureRecogniser reactor.
        explicit PostureRecogniser(std::unique_ptr<NUClear::Environment> environment);

        std::vector<utility::math::filter::UKF<utility::input::ServoLoadModel>> loadFilters;
        NUClear::clock::time_point lastTimeUpdateTime;

    private:
        /**
         * Temporary debugging variables for local output logging...
         */
        bool DEBUG;      //
        int DEBUG_ITER;  //

        /**
         * NUsight feedback initialized from configuration script, see config file for documentation...
         */
        bool emitAccelerometer;
        bool emitGyroscope;
        bool emitFallingScaleFactor;

        /**
         * @brief [brief description]
         * @details [long description]
         *
         * @param inTorsoPosition [description]
         */
        void configure(const YAML::Node& config);
    };
}  // namespace input
}  // namespace module

#endif  // MODULES_INPUT_POSTURERECOGNISER_H
