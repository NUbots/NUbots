/*! @file SelectBehaviourProvider.cpp
    @brief Implementation of select behaviour class

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

#include "SelectBehaviourProvider.h"
#include "../Behaviour.h"

#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

using namespace std;

SelectBehaviourProvider::SelectBehaviourProvider(Behaviour* manager) : BehaviourProvider(manager)
{
    m_introduction_done = false;
    m_selection_index = 0;
    string names[] = {"play_soccer", "chase_ball", "walk_optimiser", "save_images", "kicker","pose"};
    m_available_behaviours = vector<string>(names, names + sizeof(names)/sizeof(*names));
}


SelectBehaviourProvider::~SelectBehaviourProvider()
{
    
}

void SelectBehaviourProvider::doBehaviour()
{
    if (not m_introduction_done)
        doIntroduction();
    else
    {
        if (singleLeftBumperClick())
        {
            m_selection_index = (m_selection_index + 1) % m_available_behaviours.size();
            voiceCurrentSelection();
        }        
        if (singleRightBumperClick())
        {
            m_selection_index = (m_selection_index + m_available_behaviours.size()-1) % m_available_behaviours.size();
            voiceCurrentSelection();
        }        
        if (singleChestClick() or longChestClick())
        {
            voiceCurrentSelection();
            m_manager->setNextBehaviour(m_available_behaviours[m_selection_index]);
        }
    }
}

void SelectBehaviourProvider::doIntroduction()
{
    m_actions->addSound(m_current_time, "select_behaviour.wav");
    m_introduction_done = true;
}

void SelectBehaviourProvider::voiceCurrentSelection()
{
    m_actions->addSound(m_current_time, m_available_behaviours[m_selection_index] + ".wav");
}
    


