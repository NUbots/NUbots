/*! @file GameInformation.h
    @brief Declaration of GameInformation class.

    @class GameInformation
    @brief  GameInformation uses the sensor inputs, game packets and team
            packets to keep track of the current state of the game. As well
            as the state of the player.

    @author Steven Nicklin

    Copyright (c) 2010 Steven Nicklin

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

#ifndef GAME_INFORMATION_H
#define GAME_INFORMATION_H

#include "RoboCupGameControlData.h"
#include <string>
#include <cstring>

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
    /*!
      @brief Constructor.
      @param playerNumber The initial player number of the robot.
      @param teamNumber The team number of the robot.
      */
    GameInformation(int playerNumber = 0, int teamNumber = 0);

    /*!
      @brief Destructor.
      */
    ~GameInformation();

    void process();

    // My information
    /*!
      @brief Get the player ID.
      @return The current player ID.
      */
    int playerId() const
    {
        return m_myPlayerNumber;
    }

    /*!
      @brief Get the player state.
      @return The current player state.
      */
    robotState getCurrentState() const
    {
        return m_myCurrentState;
    }

    /*!
      @brief Get the previous player state.
      @return The previous player state.
      */
    robotState getPreviousState() const
    {
        return m_myPreviousState;
    }

    /*!
      @brief Determine if robots state has changed.
      @return true if the state has changes, false if it has not.
      */
    bool robotStateChanged() const
    {
        return m_myStateChanged;
    }

    // Team information
    /*!
      @brief Get the team ID.
      @return The team ID.
      */
    int teamId() const
    {
        return m_myTeamNumber;
    }

    // Game information
    /*!
      @brief Get the total number of players per team.
      @return The number of players per team.
      */
    unsigned char getNumPlayers()
    {
        return m_currentControlData->playersPerTeam;
    }

    /*!
      @brief Determine which half the game is in.
      @return true if the game is within its first half, false if game is in the second half.
      */
    bool isFirstHalf()
    {
        return (bool)m_currentControlData->firstHalf;
    }

    /*!
      @brief Determine if our team has the kickoff
      @return true if our team has kicking off, false if the opponent team has kick off.
      */
    bool haveKickoff()
    {
        return (m_currentControlData->kickOffTeam == m_myTeamNumber);
    }

    /*!
      @brief Get the amout of time left in the current half.
      @return The number of seconds remaining in the current half.
      */
    int timeRemaining()
    {
        return m_currentControlData->secsRemaining;
    }

    /*!
      @brief Get the number of goals that our team has scored.
      @return The goal tally for our team.
      */
    int ourScore()
    {
        return getTeam(teamId())->score;
    }

    /*!
      @brief Get the number of goals that our opponents have scored.
      @return The goal tally for the opposing team.
      */
    int opponentScore()
    {
        return 0;
    }

    const RobotInfo* getPlayer(int teamNumber, int playerNumber)
    {
        const TeamInfo* team = getTeam(teamNumber);
        if(team)
        {
            if((playerNumber > getNumPlayers()) && (playerNumber > 0))
            {
                return 0;
            }
            else
            {
                return &team->players[playerNumber-1];
            }
        }
        return 0;
    }

    const TeamInfo* getTeam(int teamNumber)
    {
        if(m_currentControlData->teams[TEAM_BLUE].teamNumber == teamNumber)
        {
            return &m_currentControlData->teams[TEAM_BLUE];
        }
        else if (m_currentControlData->teams[TEAM_RED].teamNumber == teamNumber)
        {
            return &m_currentControlData->teams[TEAM_RED];
        }
        else
        {
            return 0;
        }
    }

    /*!
      @brief Get the current game state.
      @return Curent game state.
      */
    robotState getCurrentGameState() const
    {
        return robotState(m_currentControlData->state);
    }

    /*!
      @brief Get the previous game state.
      @return Previous game state.
      */
    robotState getPreviousGameState() const
    {
        return robotState(m_previousControlData->state);
    }


    /*!
      @brief Determine if the game state has changed.
      @return true if the game state has changed. false if it has not.
      */
    bool gameStateChanged() const
    {
        return (m_currentControlData->state != m_previousControlData->state);
    }

    /*!
      @brief Queues a new network packet to be processed.
      @param gameControllerPacket The new packet data.
      */
    void addNewNetworkData(const RoboCupGameControlData& gameControllerPacket);

    /*!
      @brief Signal whether a manual state change is requested.
      @param hasChange True if the state should be manually changed. False if it should not.
      */
    void triggerManualStateChange(bool hasChange = true)
    {
        m_hasUnprocessedStateTrigger = hasChange;
    }

    /*!
      @brief Signal whether a manual team change is requested.
      @param hasChange True if the state should be manually changed. False if it should not.
      */
    void triggerManualTeamChange(bool hasChange = true)
    {
        m_hasUnprocessedTeamTrigger = hasChange;
    }

    /*!
      @brief Signal whether a manual kick-off change is requested.
      @param hasChange True if the state should be manually changed. False if it should not.
      */
    void triggerManualKickoffChange(bool hasChange = true)
    {
        m_hasUnprocessedKickoffTrigger = hasChange;
    }

private:
    /*!
      @brief Get the next state that the robot should go to on a state toggle.
      @param currentState The current robot state.
      @return The next robot state.
      */
    static robotState getNextState(robotState currentState);

    /*!
      @brief Find the name of a given state.
      @param theState The robot/game state.
      @return A string of the name of the state.
      */
    static std::string stateName(robotState theState);

    /*!
      @brief Determine if the state is a game playing state.
      @param theState The state in question.
      */
    static bool isGameState(robotState theState);

    /*!
      @brief Update the current information based on a new game controller packet.
      */
    void updateData();

    /*!
      @brief Update the current information based on a manual state change.
      */
    void doManualStateChange();

    /*!
      @brief Update the current information based on a manual team change.
      */
    void doManualTeamChange();

    /*!
      @brief Update the current information based on a manual kick-off change.
      */
    void doManualKickoffChange();

    void rawSwapTeams(RoboCupGameControlData* data);

    // My information
    int m_myPlayerNumber;           //!< Player number.
    int m_myTeamNumber;             //!< Team number.
    robotState m_myCurrentState;    //!< Current robot state.
    robotState m_myPreviousState;   //!< Previous robot state.
    bool m_myStateChanged;          //!< Player state has changed. true if state has changed, false if not.

    // game packet buffers

    enum parameters
    {
        numPacketBuffers = 3
    };

    RoboCupGameControlData* m_gamePacketBuffers[numPacketBuffers];

    // Game Information
    RoboCupGameControlData* m_currentControlData;        //!< The current game info.
    RoboCupGameControlData* m_previousControlData;       //!< The previous game info.

    // Unprocessed info holding area
    bool m_hasUnprocessedControlData;
    RoboCupGameControlData* m_unprocessedControlData;    //!< The new game packet.    
    bool m_hasUnprocessedStateTrigger;      //!< New state trigger.
    bool m_hasUnprocessedTeamTrigger;       //!< New team trigger.
    bool m_hasUnprocessedKickoffTrigger;    //!< New kick-off trigger.

};

#endif
