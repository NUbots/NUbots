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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_BEHAVIOUR_TOOLS_SCRIPTTUNER_H
#define MODULES_BEHAVIOUR_TOOLS_SCRIPTTUNER_H

#include <nuclear>
#include "message/motion/Script.h"


namespace module {
    namespace behaviour {
        namespace tools {

            /**
             * Provides a Curses interface to let the user customize scripts
             *
             * @author Trent Houliston
             */
            class ScriptTuner : public NUClear::Reactor {
            private:
                const size_t id;
                /// The path to the script we are editing
                std::string scriptPath;
                /// The script object we are editing
                message::motion::Script script;
                /// The index of the frame we are currently editing
                size_t frame;
                /// The index of the item we are selecting
                size_t selection;
                /// If we are selecting the angle or gain for this item
                bool angleOrGain;
                const size_t defaultGain = 80;
                const size_t defaultDuration = 1000;

                std::string userInput();

                void refreshView();
                void loadScript(const std::string& path);
                void saveScript();
                void editDuration();
                void editSelection();
                void activateFrame(int frame);
                void toggleLockMotor();
                void newFrame();
                void deleteFrame();
                void playScript();
                void jumpToFrame();
                void help();
                void editGainInput();
                void mirrorScript();
                void saveScriptAs();
                void editGain();
                void userInputToFrame();
                float userInputToGain();

                volatile bool running;

            public:
                explicit ScriptTuner(std::unique_ptr<NUClear::Environment> environment);
            };

        }  // tools
    }  // behaviours
}  // modules

#endif  // MODULES_BEHAVIOURS_UTILITY_SCRIPTTUNER_H
