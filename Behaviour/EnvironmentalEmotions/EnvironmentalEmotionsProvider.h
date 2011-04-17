/*! @file RoboPedestrianProvider.h
    @brief Declaration of simple chase ball behaviour for testing and demonstration purposes 
 
    @class RoboPedestrianProvider
    @brief A behaviour used in the Aritifical Life Paper, Mimics a pedestrian on a street with the ability to be distracted.

    @author Jason Kulk, Aaron Wong
 
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

#ifndef ENVIRONMENTALEMOTIONBEHAVIOUR_H
#define ENVIRONMENTALEMOTIONBEHAVIOUR_H

#include "Behaviour/BehaviourFSMProvider.h"
//class ChaseBlueGoal;
//class ChaseYellowGoal;
//class SearchForBlueGoal;
//class SearchForYellowGoal;
class Paused;
class AnalyseEnvironmentState;

#include <vector>
#include <string>

class EnvironmentalEmotionsProvider : public BehaviourFSMProvider
{
public:
    EnvironmentalEmotionsProvider(Behaviour* manager);
    ~EnvironmentalEmotionsProvider();
protected:
    BehaviourState* nextStateCommons();
private:

    friend class Paused;
    BehaviourState* m_paused;
    friend class AnalyseEnvironmentState;
    BehaviourState* m_analyse_environment;
};


#endif

