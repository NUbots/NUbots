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

#ifndef MODULES_SCRIPTTUNER_H
#define MODULES_SCRIPTTUNER_H

#include <NUClear.h>
#include "messages/Script.h"

namespace modules {

    class ScriptTuner : public NUClear::Reactor {
    private:
        std::string scriptPath;
        messages::Script script;
        size_t frame;
        size_t selection;
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
    public:
        explicit ScriptTuner(NUClear::PowerPlant* plant);
    };
}
#endif

