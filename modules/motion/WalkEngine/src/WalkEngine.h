/*
 * This file is part of WalkEngine.
 *
 * WalkEngine is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * WalkEngine is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with WalkEngine.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_MOTION_WALKENGINE_H
#define MODULES_MOTION_WALKENGINE_H

#include <nuclear>
#include <armadillo>

#include "utility/configuration/ConfigurationNode.h"

namespace modules {
    namespace motion {

        /**
         * TODO
         * 
         * @author Trent Houliston
         */
        class WalkEngine : public NUClear::Reactor {
        public:
            static constexpr const char* CONFIGURATION_PATH = "WalkEngine.json";
            explicit WalkEngine(std::unique_ptr<NUClear::Environment> environment);
        private:
            utility::configuration::ConfigurationNode config;
            bool active;
            bool started;
            bool moving;
            time_t tLastStep;
            float ph0 = 0;
            float ph = 0;
            int iStep0 = -1;
            int iStep = 0;
            int stopRequest = 2;
            enum {
                LEFT,
                RIGHT
            } supportLeg = LEFT;
            // TODO: uLeft1
            // TODO: uRight1
            // TODO: uTorso1
            arma::vec2 supportMod;
            float shiftFactor;

            void update();
            void advanceMotion();
            void updateVelocity();
            float getFootX();
        };
    
    }  // motion
}  // modules

#endif  // MODULES_MOTION_WALKENGINE_H

