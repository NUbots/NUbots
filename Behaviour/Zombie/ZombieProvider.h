/*! @file ChaseBallProvider.h
    @brief Declaration of simple chase ball behaviour for testing and demonstration purposes

    @class ChaseBallProvider
    @brief A simple chase ball behaviour for testing and demonstration purposes

    @author Jason Kulk

  Copyright (c) 2010 Jason Kulk

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

#ifndef ZOMBIEBEHAVIOUR_H
#define ZOMBIEBEHAVIOUR_H

#include "Behaviour/BehaviourFSMProvider.h"

class ZombieState;

#include <vector>
#include <string>

class ZombieProvider : public BehaviourFSMProvider
{
public:
    ZombieProvider(Behaviour* manager, bool pauseable = false);
    ~ZombieProvider();
protected:
    BehaviourState* nextStateCommons();
private:
    bool m_pauseable;

    friend class ZombieState;
    BehaviourState* m_paused_state;
};


#endif

