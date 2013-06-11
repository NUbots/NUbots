/*! @file SoccerProvider.h
    @brief 
 
    @class SoccerProvider
    @brief 

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

#ifndef SOCCERPROVIDER_H
#define SOCCERPROVIDER_H

class Behaviour;
#include "Behaviour/BehaviourFSMProvider.h"
#include "Behaviour/Common/HeadBehaviour.h"

#include <vector>


class SoccerProvider : public BehaviourFSMProvider
{
public:
    SoccerProvider(Behaviour* manager);
    virtual ~SoccerProvider();
    
protected:
    void doBehaviourCommons();
    BehaviourState* nextStateCommons();
public:
    BehaviourState* m_initial;
    BehaviourState* m_ready;
    BehaviourState* m_set;
    BehaviourState* m_playing;
    BehaviourState* m_finished;
    BehaviourState* m_penalised;
    std::vector<float> m_led_white;
    std::vector<float> m_led_red;
    std::vector<float> m_led_green;
    std::vector<float> m_led_blue;
    std::vector<float> m_led_orange;
    std::vector<float> m_led_yellow;
    std::vector<float> m_led_off;

    HeadBehaviour* head_behaviour;
};


#endif

