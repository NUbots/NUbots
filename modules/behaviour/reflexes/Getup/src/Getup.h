/*
 * This file is part of ScriptRunner.
 *
 * ScriptRunner is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ScriptRunner is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ScriptRunner.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_BEHAVIOUR_REFLEX_GETUP_H
#define MODULES_BEHAVIOUR_REFLEX_GETUP_H

#include <nuclear>
#include <queue>

#include "messages/platform/darwin/DarwinServoCommand.h"
#include "messages/motion/Script.h"

namespace modules {
    namespace behaviour {
        namespace reflexes {

            /**
             * Executes a getup script if the robot falls over.
             *
             * @author Josiah Walker
             */
            class Getup : public NUClear::Reactor {
            private:
                /// config settings
                double FALLEN_ANGLE,GETUP_PRIORITY,EXECUTION_PRIORITY;

                ///reactionhandles let us disable reactions temporarily
                ReactionHandle fallenDetector;
                ReactionHandle getupDetector;

                double currentPriority;
                void updateAction(const double& priority);

            public:
                explicit Getup(std::unique_ptr<NUClear::Environment> environment);
                static constexpr const char* CONFIGURATION_PATH = "Getup.json";
            };
        }  // reflexes
    }  // behaviours
}  // modules

#endif  // MODULES_BEHAVIOURS_UTILITY_SCRIPTRUNNER_H

