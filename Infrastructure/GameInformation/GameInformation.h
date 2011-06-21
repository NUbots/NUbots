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

#include "NUPlatform/NUIO/RoboCupGameControlData.h"
#include <string>
#include <cstring>
#include <vector>
using namespace std;

class NUSensorsData;
class NUActionatorsData;
class GameControllerPort;

class GameInformation
{
public:
    enum RobotState
    {
        InitialState = STATE_INITIAL,
        ReadyState = STATE_READY,
        SetState = STATE_SET,
        PlayingState = STATE_PLAYING,
        FinishedState = STATE_FINISHED,
        PenalisedState
    };
    enum TeamColour
    {
        BlueTeam = TEAM_BLUE,
        RedTeam = TEAM_RED
    };
public:
    GameInformation(int playerNumber, int teamNumber);
    ~GameInformation();

    // My information
    int getPlayerNumber() const;
    int getTeamNumber() const;
    TeamColour getTeamColour() const;
    RobotState getCurrentState() const;
    int getPenaltyReason() const;
    bool amIASubstitute() const;
    
    // Game Information
    bool gameControllerWorking() const;
    int getNumberOfPlayers() const;
    bool isFirstHalf() const;
    bool isSecondHalf() const;
    bool haveKickoff() const;
    int secondsRemaining() const;
    int ourScore() const;      
    int opponentScore() const;   
    int getNumberOfPlayersPenalised() const;
    vector<vector<int> > getPenaltyReasons() const;
    
    // GameController packets
    void addNetworkPort(GameControllerPort* port);
    friend GameInformation& operator<< (GameInformation& info, RoboCupGameControlData* data);
    void process(RoboCupGameControlData* data);
    void sendAlivePacket();
    void requestForPickup();

    // Manual game control
    void doManualStateChange();
    void doManualTeamChange();

private:
    const RobotInfo* getRobotInfo(int teamNumber, int playerNumber) const;
    const RobotInfo* getMyRobotInfo() const;
    const TeamInfo* getTeamInfo(int teamNumber) const;
    const TeamInfo* getMyTeamInfo() const;
    const TeamInfo* getOpponentTeamInfo() const;

    static std::string stateName(RobotState theState);

    void doGameControllerUpdate();

    NUSensorsData* m_data;          
    NUActionatorsData* m_actions;  //!< local copy of pointer to actions 
    // My information
    int m_player_number;           //!< Player number
    int m_team_number;             //!< Team number
    RobotState m_state;            //!< Current robot state

    // Game Information
    RoboCupGameControlData* m_currentControlData;        //!< The current game info.
    double m_last_packet_time;                           //!< The time the last game controller packet was received
    GameControllerPort* m_port;
    RoboCupGameControlReturnData* m_currentReturnData;   //!< The current return packet
    
    vector<float> m_led_red;
};

#endif
