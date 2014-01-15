/*
 * This file is part of ScriptTuner.
 *
 * ScriptTuner is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ScriptTuner is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ScriptTuner.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_BEHAVIOURS_TOOLS_SCRIPTTUNER_H
#define MODULES_BEHAVIOURS_TOOLS_SCRIPTTUNER_H

#include <nuclear>
#include "messages/motion/Script.h"


namespace modules {
    namespace behaviours {
        namespace tools {
            
            /**
             * Provides a Curses interface to let the user customize scripts
             * 
             * @author Trent Houliston
             */
            class ScriptTuner : public NUClear::Reactor {
            private:
                /// The path to the script we are editing
                std::string scriptPath;
                /// The script object we are editing
                messages::motion::Script script;
                /// The index of the frame we are currently editing
                size_t frame;
                /// The index of the item we are selecting
                size_t selection;
                /// If we are selecting the angle or gain for this item
                bool angleOrGain;

                void refreshView();

                std::string userInput();
                void loadScript(const std::string& path);
                void saveScript();
                void editDuration();
                void editSelection();

                void activateFrame(int frame);

                void toggleLockMotor();
                void newFrame();
                void deleteFrame();

                volatile bool running;
                void run();
                void kill();
                void playScript();
                void jumpToFrame();
                //void help();
                //void userLoadScript();


               
            public:
                explicit ScriptTuner(std::unique_ptr<NUClear::Environment> environment);
            };
            
        }  // tools
    }  // behaviours
}  // modules

#endif  // MODULES_BEHAVIOURS_UTILITY_SCRIPTTUNER_H
