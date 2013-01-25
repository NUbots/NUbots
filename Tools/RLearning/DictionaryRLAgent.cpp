/*! @file DictionaryRLAgent.cpp
    @brief Standard implementation of reinforcement learning agent using Dictionary Approximator.
    @author Jake Fountain

    ------------------------------------------------------------------------------------------
    Make your own Reinforcement Learning Agent:

    TEMPLATE
    ========

    DictionaryRLAgent rlagent;
    rlagent.setParameters(0.1,0.9,0.5,0.01,1,5);//Example values.
    rlagent.initialiseAgent(2,5,1);

    for(number of iterations){
         action = mrlagent.getAction(world.getObservation());
         giveReward(world.getReward());
         doLearning();

         world.update(action);
    }
    ------------------------------------------------------------------------------------------

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


#include "DictionaryRLAgent.h"
#include "LinearApproximator.h"
#include "FourierApproximator.h"

DictionaryRLAgent::DictionaryRLAgent():RLAgent()
{
    FunctionApproximator = (ApproximatorInterface*)(new DictionaryRLAgent());
}
