#include "GameInformation.h"
#include "NUPlatform/NUSensors.h"
#include <vector>
#include "debug.h"

GameInformation::GameInformation(int playerNumber, int teamNumber):
        m_myPlayerNumber(playerNumber),
        m_myTeamNumber(teamNumber)
{
    m_myStateChanged = true;
    // Setup initial state
    // If  a player number, set to initial state.
    if((playerNumber >= 1) && (playerNumber <= 3))
    {
        m_myCurrentState = state_initial;
        m_myPreviousState = state_initial;
    }
    // If a non-player number, set to sustitute state.
    else
    {
        m_myCurrentState = state_substitute;
        m_myPreviousState = state_substitute;
    }
    for (int i = 0; i < numPacketBuffers; i++)
    {
        m_gamePacketBuffers[i] = new RoboCupGameControlData;
    }
    m_currentControlData = m_gamePacketBuffers[0];
    m_previousControlData = m_gamePacketBuffers[1];
    m_unprocessedControlData = m_gamePacketBuffers[2];
}

GameInformation::~GameInformation()
{
    for (int i = 0; i < numPacketBuffers; i++)
    {
        delete m_gamePacketBuffers[i];
    }
    m_currentControlData = 0;
    m_previousControlData = 0;
    m_unprocessedControlData = 0;
}

void GameInformation::process()
{
    // Check if there is a new network packet
    if(!m_hasUnprocessedControlData)
    {
        // Start current update with the most current data.
        memcpy(m_unprocessedControlData, m_currentControlData, sizeof(RoboCupGameControlData));
        if(m_hasUnprocessedStateTrigger)
        {
            doManualStateChange();
        }
        if(m_hasUnprocessedTeamTrigger)
        {
            doManualTeamChange();
        }
        if(m_hasUnprocessedKickoffTrigger)
        {
            doManualKickoffChange();
        }
    }

    m_hasUnprocessedControlData = false;
    m_hasUnprocessedStateTrigger = false;
    m_hasUnprocessedTeamTrigger = false;
    m_hasUnprocessedKickoffTrigger = false;
    updateData();

    if(getPlayer(m_myTeamNumber,m_myPlayerNumber)->penalty != PENALTY_NONE)
    {
        if(m_myCurrentState != state_penalised)
        {
            m_myPreviousState = m_myCurrentState;
            m_myCurrentState = state_penalised;
        }
    }
    else if(m_myCurrentState != m_currentControlData->state)
    {
        m_myPreviousState = m_myCurrentState;
        m_myCurrentState = robotState(m_currentControlData->state);
    }
    return;
}

void GameInformation::addNewNetworkData(const RoboCupGameControlData& gameControllerPacket)
{
    // Copy the new data
    memcpy(m_unprocessedControlData, &gameControllerPacket, sizeof(RoboCupGameControlData));
    m_hasUnprocessedControlData = true;
}

/*
void GameInformation::updateSensorData(NUSensorsData* sensorData)
{
    std::vector<float> tempButtonValues;
    std::vector<float> tempButtonTriggers;
    bool chestPressed = false;
    bool leftFootPressed = false;
    bool rightFootPressed = false;
    bool stateChangeTriggered = false;
    m_myStateChanged = false;

    if(sensorData->getButtonValues(NUSensorsData::MainButton, tempButtonValues) && tempButtonValues.size() >= 1)
    {
        chestPressed = (tempButtonValues[0] > 0.0f);
    }

    if(sensorData->getFootBumperValues(NUSensorsData::AllFeet,tempButtonValues) && tempButtonValues.size() >= 2)
    {
        leftFootPressed = (tempButtonValues[0] > 0.0f);
        rightFootPressed = (tempButtonValues[1] > 0.0f);
    }

    if(sensorData->getButtonTriggers(tempButtonTriggers) && tempButtonTriggers.size() >= 3)
    {
       // Trigger if the button has been pressed for less than a full frame.
        if(chestPressed && (tempButtonTriggers[0] < 33.0f ))
        {
            stateChangeTriggered = true;
        }
    }

    if(stateChangeTriggered)
    {
        m_myPreviousState = m_myCurrentState;
        m_myCurrentState = getNextState(m_myCurrentState);
        debug << "State Changed: " << stateName(m_myPreviousState) << " -> " << stateName(m_myCurrentState) << endl;
        m_myStateChanged = true;
    }
}
*/

void GameInformation::updateData()
{
    RoboCupGameControlData* temp;
    temp = m_previousControlData;
    m_previousControlData = m_currentControlData;
    m_currentControlData = m_unprocessedControlData;
    m_unprocessedControlData = temp;
    return;

}

void GameInformation::doManualStateChange()
{
    m_myPreviousState = m_myCurrentState;
    m_myCurrentState = getNextState(m_myCurrentState);
    if(m_myCurrentState = state_penalised)
    {
        if(m_unprocessedControlData->teams[TEAM_BLUE].teamNumber == m_myTeamNumber)
        {
            m_unprocessedControlData->teams[TEAM_BLUE].players[m_myPlayerNumber].penalty = PENALTY_MANUAL;
        }
        else if(m_unprocessedControlData->teams[TEAM_RED].teamNumber == m_myTeamNumber)
        {
            m_unprocessedControlData->teams[TEAM_RED].players[m_myPlayerNumber].penalty = PENALTY_MANUAL;
        }
    }
}

void GameInformation::doManualTeamChange()
{
    rawSwapTeams(m_unprocessedControlData);
    m_unprocessedControlData->teams[TEAM_RED].teamColour = TEAM_BLUE;
    m_unprocessedControlData->teams[TEAM_BLUE].teamColour = TEAM_RED;
}

void GameInformation::rawSwapTeams(RoboCupGameControlData* data) {
    size_t    teamSize = sizeof(TeamInfo);
    TeamInfo* blueTeam = &(data->teams[TEAM_BLUE]);
    TeamInfo* redTeam  = &(data->teams[TEAM_RED]);

    TeamInfo tempTeam;
    memcpy(&tempTeam, blueTeam, teamSize);

    // swap the teams
    memcpy(blueTeam, redTeam, teamSize);
    memcpy(redTeam, &tempTeam, teamSize);
}

void GameInformation::doManualKickoffChange()
{
    m_unprocessedControlData->kickOffTeam = !m_unprocessedControlData->kickOffTeam;
}

bool GameInformation::isGameState(robotState theState)
{
    bool result = false;
    switch(theState)
    {
        case state_initial:
        case state_ready:
        case state_set:
        case state_playing:
        case state_finished:
            result = true;
        default:
            result = false;
    }
    return result;

}

std::string GameInformation::stateName(robotState theState)
{
    std::string stateName;
    switch(theState)
    {
        case state_initial:
            stateName = "Initial";
            break;
        case state_ready:
            stateName = "Ready";
            break;
        case state_set:
            stateName = "Set";
            break;
        case state_playing:
            stateName = "Playing";
            break;
        case state_finished:
            stateName = "Finished";
            break;
        case state_penalised:
            stateName = "Penalised";
            break;
        case state_substitute:
            stateName = "Sustitute";
            break;
        case state_requires_substitution:
            stateName = "Requires Substitution";
            break;
        default:
            stateName = "Unknown";
            break;
    }
    return stateName;
}

robotState GameInformation::getNextState(robotState currentState)
{
    robotState nextState(currentState);
    switch(currentState)
    {
        case state_initial:
            nextState = state_ready;
            break;
        case state_ready:
            nextState = state_set;
            break;
        case state_set:
            nextState = state_playing;
            break;
        case state_playing:
            nextState = state_penalised;
            break;
        case state_finished:
            nextState = state_finished;
            break;
        case state_penalised:
            nextState = state_playing;
            break;
        case state_substitute:
            nextState = state_substitute;
            break;
        case state_requires_substitution:
            nextState = state_requires_substitution;
            break;
        default:
            nextState = currentState;
            break;
    }
    return nextState;
}
