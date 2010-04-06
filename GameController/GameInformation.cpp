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
}

void GameInformation::process()
{
    // Get info from network packet.
    if(m_hasUnprocessedControlData)
    {
        updateNetworkData(m_unprocessedControlData);
        m_hasUnprocessedControlData = false;
        m_hasUnprocessedStateTrigger = false;
        m_hasUnprocessedTeamTrigger = false;
        m_hasUnprocessedKickoffTrigger = false;
    }
    // Get info from manual commands.
    else
    {
        if(m_hasUnprocessedStateTrigger)
        {
            doManualStateChange();
            m_hasUnprocessedStateTrigger = false;
        }
        if(m_hasUnprocessedTeamTrigger)
        {
            doManualTeamChange();
            m_hasUnprocessedTeamTrigger = false;
        }
        if(m_hasUnprocessedKickoffTrigger)
        {
            doManualKickoffChange();
            m_hasUnprocessedKickoffTrigger = false;
        }
    }
    return;
}

void GameInformation::addNewNetworkData(const RoboCupGameControlData& gameControllerPacket)
{
    m_unprocessedControlData = gameControllerPacket;    //!< The new game packet.
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

void GameInformation::updateNetworkData(const RoboCupGameControlData& gameControllerPacket)
{
    m_previousControlData = m_currentControlData;
    m_currentControlData = gameControllerPacket;
}

void GameInformation::doManualStateChange()
{

}

void GameInformation::doManualTeamChange()
{

}

void GameInformation::doManualKickoffChange()
{

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
