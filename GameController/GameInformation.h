#ifndef GAME_INFORMATION_H
#define GAME_INFORMATION_H

#include "RoboCupGameControlData.h"

enum robotState
{
    state_initial,
    state_ready,
    state_set,
    state_playing,
    state_finished,
    state_penalised,
    state_substitute,
    state_requires_substitution
};

class NUSensorsData;

class GameInformation
{
public:
    GameInformation(int playerNumber = 0, int teamNumber = 0);
    void update(NUSensorsData* sensorData, const RoboCupGameControlData* gameControllerPacket);

    // My information
    int playerId() const
    {
        return m_myPlayerNumber;
    }
    robotState getCurrentState() const
    {
        return m_myCurrentState;
    }
    robotState getPreviousState() const
    {
        return m_myPreviousState;
    }
    bool robotStateChanged() const
    {
        return m_myStateChanged;
    }

    // Team information
    int teamId() const
    {
        return m_myTeamNumber;
    }

    // Game information
    robotState getCurrentGameState() const
    {
        return robotState(m_currentControlData.state);
    }
    robotState getPreviousGameState() const
    {
        return robotState(m_previousControlData.state);
    }
    bool gameStateChanged() const
    {
        return (m_currentControlData.state != m_previousControlData.state);
    }

private:
    static robotState getNextState(robotState currentState);
    void updateSensorData(NUSensorsData* sensorData);
    void updateNetworkData(const RoboCupGameControlData* gameControllerPacket);
    // My information
    int m_myPlayerNumber;
    int m_myTeamNumber;
    robotState m_myCurrentState;
    robotState m_myPreviousState;
    bool m_myStateChanged;

    // Team Information

    // Game Information
    RoboCupGameControlData m_currentControlData;
    RoboCupGameControlData m_previousControlData;

};

#endif
