/*!  @file ScriptTunerProvider.h
    @brief Provider of Head behaviour Testing behaviour. Darwin simply stands, observes and localises.
    @author Jake Fountain

    @class ChaseBallProvider
    @brief A simple chase ball behaviour for testing and demonstration purposes

    @author Jake Fountain

 Copyright (c) 2012 Jake Fountain

    This file is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This file is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SCRIPT_TUNER_BEHAVIOUR_H
#define SCRIPT_TUNER_BEHAVIOUR_H

#include "Behaviour/BehaviourFSMProvider.h"

class ScriptTunerState;

#include <vector>
#include <string>

class ScriptTunerProvider : public BehaviourFSMProvider
{
public:
    ScriptTunerProvider(Behaviour* manager, bool pauseable = false);
    ~ScriptTunerProvider();
protected:
    BehaviourState* nextStateCommons();
private:
    bool m_pauseable;

    friend class ScriptTunerState;
    BehaviourState* m_paused_state;
};


#endif

