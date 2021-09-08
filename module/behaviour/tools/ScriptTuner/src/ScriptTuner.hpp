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

#ifndef MODULES_BEHAVIOUR_TOOLS_SCRIPTTUNER_HPP
#define MODULES_BEHAVIOUR_TOOLS_SCRIPTTUNER_HPP

#include <nuclear>

#include "extension/Script.hpp"

namespace module::behaviour::tools {

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
        ::extension::Script script;
        /// The index of the frame we are currently editing
        size_t frame;
        /// The index of the item we are selecting
        size_t selection;
        /// If we are selecting the angle or gain for this item
        bool angleOrGain;
        const size_t defaultGain     = 30;
        const size_t defaultDuration = 1000;

        static std::string userInput();

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
        static float userInputToGain();

        volatile bool running;

    public:
        explicit ScriptTuner(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::behaviour::tools

#endif  // MODULES_BEHAVIOURS_UTILITY_SCRIPTTUNER_HPP
