/*! @file SoccerProvider.cpp
    @brief Implementation of soccer behaviour class

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

#include "SoccerProvider.h"

#include "Initial/InitialState.h"
#include "Ready/ReadyState.h"
#include "Set/SetState.h"
#include "Playing/PlayingState.h"
#include "Finished/FinishedState.h"
#include "Penalised/PenalisedState.h"
/*#include "Substitute/SubstituteState.h"
#include "Substitute/RequiresSubstituteState.h"*/

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/GameInformation/GameInformation.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "NUPlatform/NUPlatform.h"

#include "Infrastructure/Jobs/VisionJobs/SaveImagesJob.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

using namespace std;

/*! @brief Construct a behaviour provider with the given manager
 */
SoccerProvider::SoccerProvider(Behaviour* manager) : BehaviourFSMProvider(manager)
{
    m_initial = new InitialState(this);
    m_ready = new ReadyState(this);
    m_set = new SetState(this);
    m_playing = new PlayingState(this);
    m_finished = new FinishedState(this);
    m_penalised = new PenalisedState(this);
    /*m_substitute = new SubstituteState(this);
    m_requires_substitution = new RequiresSubstituteState(this);*/
    
    m_state = m_initial;
    
    m_led_white = vector<float>(3,1);
    m_led_off = vector<float>(3,0);
    m_led_red = m_led_off; m_led_red[0] = 1;
    m_led_green = m_led_off; m_led_green[1] = 1;
    m_led_blue = m_led_off; m_led_blue[2] = 1;
    m_led_orange = m_led_off; m_led_orange[0] = 1; m_led_orange[1] = 0.3;
    m_led_yellow = m_led_off; m_led_yellow[0] = 1; m_led_yellow[1] = 1;
}

/*! @brief Destroys the behaviour provider as well as all of the associated states
 */
SoccerProvider::~SoccerProvider()
{
    delete m_initial;
    delete m_ready;
    delete m_set;
    delete m_playing;
    delete m_finished;
    delete m_penalised;
    /*delete m_substitute;
    delete m_requires_substitution;*/
}

/*! @brief Performs behaviour that is common to all states in the soccer behaviour provider
 
    The rules specify the following:
        - the left foot led mush display the team colour 
 
    The right eye indicate the following:
        - top sector whether the the robot is lost, or the ball is lost
        - botton sector whether the ball is visible
        - left sector whether a yellow post is visible
        - right sector whether a blu post is visible
 */
void SoccerProvider::doBehaviourCommons()
{
    // In every state the left foot led must display the team colour
    if (m_game_info->getTeamColour() == GameInformation::BlueTeam)
    {
        vector<float> blue(3,1);
        blue[0] = 0;
        m_actions->add(NUActionatorsData::LFootLed, m_current_time, blue);
    }
    else
    {
        vector<float> pink(3,1);
        pink[1] = 0;
        m_actions->add(NUActionatorsData::LFootLed, m_current_time, pink);
    }
    
    // set the right eyes to indicate lost states
    bool balllost = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].lost();
    bool selflost = m_field_objects->self.lost();
    if (balllost and selflost)
        Platform->add(NUPlatform::Led6, m_actions->CurrentTime, m_led_yellow);
    else if (balllost)
        Platform->add(NUPlatform::Led6, m_actions->CurrentTime, m_led_red);
    else if (selflost)
        Platform->add(NUPlatform::Led6, m_actions->CurrentTime, m_led_green);
    else
        Platform->add(NUPlatform::Led6, m_actions->CurrentTime, m_led_off);
    
    // set the right eyes to indicate the ball visibility
    if (m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].isObjectVisible())
        Platform->add(NUPlatform::Led4, m_actions->CurrentTime, m_led_orange);
    else
        Platform->add(NUPlatform::Led4, m_actions->CurrentTime, m_led_off);
    
    // set the right eyes to indicate the goal visibility
    StationaryObject& yellow_left = m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST];
    StationaryObject& yellow_right = m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST];
    int unknown_yellow_posts = 0;
    for (size_t i=0; i<m_field_objects->ambiguousFieldObjects.size(); i++)
    {
        if (m_field_objects->ambiguousFieldObjects[i].getID() == FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN)
            unknown_yellow_posts++;
    }
    
    StationaryObject& blue_left = m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST];
    StationaryObject& blue_right = m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST];
    int unknown_blue_posts = 0;
    for (size_t i=0; i<m_field_objects->ambiguousFieldObjects.size(); i++)
    {
        if (m_field_objects->ambiguousFieldObjects[i].getID() == FieldObjects::FO_BLUE_GOALPOST_UNKNOWN)
            unknown_blue_posts++;
    }
    
    if (unknown_yellow_posts > 0 or yellow_left.isObjectVisible() or yellow_right.isObjectVisible())
        Platform->add(NUPlatform::Led5, m_actions->CurrentTime, m_led_yellow);
    else
        Platform->add(NUPlatform::Led5, m_actions->CurrentTime, m_led_off);
    
    if (unknown_blue_posts > 0 or blue_left.isObjectVisible() or blue_right.isObjectVisible())
        Platform->add(NUPlatform::Led7, m_actions->CurrentTime, m_led_blue);
    else
        Platform->add(NUPlatform::Led7, m_actions->CurrentTime, m_led_off);
}

/*! @brief Checks for state transitions that are common to all states in this behaviour provider
 
        That is, the state is selected via the game controler here.
 */
BehaviourState* SoccerProvider::nextStateCommons()
{
    if (singleChestClick() or longChestClick())
        m_game_info->doManualStateChange();
    
    GameInformation::RobotState game_state = m_game_info->getCurrentState();
    switch (game_state) 
    {
        case GameInformation::InitialState:
            return m_initial;
            break;
        case GameInformation::ReadyState:
            return m_ready;
            break;
        case GameInformation::SetState:
            return m_set;
            break;
        case GameInformation::PlayingState:
            return m_playing;
            break;
        case GameInformation::FinishedState:
            return m_finished;
            break;
        case GameInformation::PenalisedState:
            return m_penalised;
            break;/*
        case GameInformation::SubstituteState:
            return m_substitute;
            break;
        case GameInformation::RequiresSubstitutionState:
            return m_requires_substitution;
            break;*/
        default:
            break;
    }
    return m_state;
}


