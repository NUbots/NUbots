/*! @file Behaviour.cpp
    @brief Implementation of behaviour class

    @author Jason Kulk
 
 Copyright (c) 2009 Jason Kulk
 
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

#include "Behaviour.h"

#include "Jobs/JobList.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "Vision/FieldObjects/FieldObjects.h"
#include "GameController/GameInformation.h"
#include "TeamInformation.h"

#include "debug.h"
#include "debugverbositybehaviour.h"
using namespace std;

// Add avaliable behaviours to this list:
static string temp_names[] = {"play_soccer", "chase_ball"};
vector<string> Behaviour::m_avaliable_behaviours(temp_names, temp_names + sizeof(temp_names)/sizeof(*temp_names));



Behaviour::Behaviour()
{
    m_introduction_done = false;
    m_selection_index = 0;
}


Behaviour::~Behaviour()
{
    
}

void Behaviour::doBehaviour()
{
    cout << "Behaviour::doBehaviour()" << endl;
    if (not m_introduction_done)
        doIntroduction();
    else
    {
        vector<float> buttontriggers;
        if (m_data->getButtonTriggers(buttontriggers))
        {
            if (buttontriggers[0] < 20)
            {
                voiceCurrentSelection();
            }
            else if (buttontriggers[1] < 20)
            {
                m_selection_index = (m_selection_index + 1)%(m_avaliable_behaviours.size());
                voiceCurrentSelection();
            }
            else if (buttontriggers[2] < 20)
            {
                //m_selection_index = (m_selection_index - 1)%(m_avaliable_behaviours.size());
                //voiceCurrentSelection();
            }
        }
    }
}

void Behaviour::doIntroduction()
{
    m_actions->addSound(m_current_time + 500, "select_behaviour.wav");
    m_introduction_done = true;
}

void Behaviour::voiceCurrentSelection()
{
    cout << "voiceCurrentSelection" << endl;
    m_actions->addSound(m_current_time, m_avaliable_behaviours[m_selection_index] + ".wav");
}

