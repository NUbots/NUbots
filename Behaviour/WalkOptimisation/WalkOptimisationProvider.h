/*! @file WalkOptimisationProvider.h
    @brief Declaration of walk optimisation behaviour for testing and demonstration purposes 
 
    @class WalkOptimisationProvider
    @brief A walk optimisation behaviour provider

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

#ifndef WALKOPTIMISATIONBEHAVIOUR_H
#define WALKOPTIMISATIONBEHAVIOUR_H

#include "Behaviour/BehaviourFSMProvider.h"
class Optimiser;

#include <vector>
#include <string>

class WalkOptimisationProvider : public BehaviourFSMProvider
{
public:
    WalkOptimisationProvider(Behaviour* manager);
    ~WalkOptimisationProvider();
protected:
    BehaviourState* nextStateCommons();
public:
    BehaviourState* m_generate;             //!< the state in which the parameter generation is done, and preparations for its evaluation
    BehaviourState* m_evaluate;             //!< the state in which the parameter evaluation is done
    BehaviourState* m_paused;               //!< the optimisation process is paused in this state.
    
    Optimiser* m_optimiser;
};


#endif

