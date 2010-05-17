/*! @file BehaviourState.h
    @brief Declaration of an abstract behaviour state class for other states to inherit from
 
    @class BehaviourState
    @brief Declaration of an abstract behaviour state class for other states to inherit from

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

#ifndef BEHAVIOURSTATE_H
#define BEHAVIOURSTATE_H

class BehaviourProvider;

class BehaviourState
{
public:
    BehaviourState(BehaviourProvider* provider);
    virtual ~BehaviourState();
    virtual BehaviourState* nextState() = 0;
    virtual void doState() = 0;
protected:
    BehaviourProvider* m_provider;
};


#endif

