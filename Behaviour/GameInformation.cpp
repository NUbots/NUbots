#include "GameInformation.h"

#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "NUPlatform/NUSystem.h"

#include <memory.h>
#include "debug.h"

GameInformation::GameInformation(int playerNumber, int teamNumber, NUSensorsData* data, NUActionatorsData* actions)
{
    m_data = data;
    m_actions = actions;
    
    m_player_number = playerNumber;
    m_team_number = teamNumber;
    m_state = InitialState;
    
    m_currentControlData = new RoboCupGameControlData();
    m_last_packet_time = 0;
    
    m_currentReturnData = new RoboCupGameControlReturnData();
    memcpy(m_currentReturnData->header, GAMECONTROLLER_RETURN_STRUCT_HEADER, sizeof(m_currentReturnData->header));
    m_currentReturnData->version = GAMECONTROLLER_RETURN_STRUCT_VERSION;
}

GameInformation::~GameInformation()
{
    delete m_currentControlData;
}

/*! @brief Returns the player number */
int GameInformation::getPlayerNumber() const 
{
    return m_player_number;
}

/*! @brief Returns the team number */
int GameInformation::getTeamNumber() const 
{
    return m_team_number;
}

/*! @brief Returns the team colour */
GameInformation::TeamColour GameInformation::getTeamColour() const
{
    const TeamInfo* team = getMyTeamInfo();
    if (team)
        return static_cast<TeamColour>(team->teamColour);
    else
        return BlueTeam;
}

/*! @brief Returns the current state */
GameInformation::RobotState GameInformation::getCurrentState() const 
{
    return m_state;
}

/*! @brief Returns true if the game controller is working */
bool GameInformation::gameControllerWorking() const
{
    if (m_data)
        return (m_data->CurrentTime - m_last_packet_time) > 10000;
    else
        return (memcmp(m_currentControlData->header, GAMECONTROLLER_STRUCT_HEADER, sizeof(m_currentControlData->header)) == 0);
}

/*! @brief Returns the number of players per team */
int GameInformation::getNumberOfPlayers() const 
{
    return (int)m_currentControlData->playersPerTeam;
}

/*! @brief Returns true if it is the first half */
bool GameInformation::isFirstHalf() const 
{
    return (bool)m_currentControlData->firstHalf;
}

/*! @brief Returns true if it is the second half */
bool GameInformation::isSecondHalf() const 
{
    return not isFirstHalf();
}
    
/*! @brief Returns true if we have the kick off */
bool GameInformation::haveKickoff() const 
{
    const TeamInfo* my_info = getMyTeamInfo();
    if (my_info and my_info->teamColour == m_currentControlData->kickOffTeam)
        return true;
    else
        return false;
}

/*! @brief Returns the number of seconds remaining in the half */
int GameInformation::secondsRemaining() const 
{
    return m_currentControlData->secsRemaining;
}                          

/*! @brief Returns the number of goals we have scored */
int GameInformation::ourScore() const                                
{
    const TeamInfo* team = getMyTeamInfo();
    if (team)
        return team->score;
    else
        return 0;
}                         

/*! @brief Returns the number of goals they have scored */
int GameInformation::opponentScore() const                           
{
    const TeamInfo* team = getOpponentTeamInfo();
    if (team)
        return team->score;
    else
        return 0;
}

/*! @brief Updates the GameInformation based on the m_currentControlData (ie. sets m_state)
 */
void GameInformation::doGameControllerUpdate()
{
    if (m_state == SubstituteState)
    {   // we don't listen to game controller if we are a subsitute player
        return;
    }
    else if (m_state == RequiresSubstitutionState)
    {   // if we are in the requires substitution state we are do penalty, finish, initial
        const RobotInfo* info = getMyRobotInfo();
        if (info and info->penalty != PENALTY_NONE)
            m_state = PenalisedState;
        else if (m_currentControlData->state == InitialState || m_currentControlData->state == FinishedState)
        {
            m_state = static_cast<RobotState>(m_currentControlData->state); 
        }
    }
    else
    {   // in other states we always do what the game controller says
        const RobotInfo* info = getMyRobotInfo();
        if (info and info->penalty != PENALTY_NONE)
            m_state = PenalisedState;
        else
            m_state = static_cast<RobotState>(m_currentControlData->state);
    }
}

/*! @brief Does a manual state change. The logic is simple if not in penalised then penalise, otherwise go to playing
 */
void GameInformation::doManualStateChange()
{
    m_currentReturnData->team = m_team_number;
    m_currentReturnData->player = m_player_number;
    if (m_state != PenalisedState)
    {
        m_state = PenalisedState;
        m_currentReturnData->message = GAMECONTROLLER_RETURN_MSG_MAN_PENALISE;
    }
    else
    {
        m_state = PlayingState;
        m_currentReturnData->message = GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE;
    }
    
    //! @todo send game controller return packet
}

/*! @brief Does a manual team change
 */
void GameInformation::doManualTeamChange()
{
    size_t teamSize = sizeof(TeamInfo);
    TeamInfo* blueTeam = &(m_currentControlData->teams[TEAM_BLUE]);
    TeamInfo* redTeam  = &(m_currentControlData->teams[TEAM_RED]);
    
    TeamInfo tempTeam;
    memcpy(&tempTeam, blueTeam, teamSize);
    memcpy(blueTeam, redTeam, teamSize);
    memcpy(redTeam, &tempTeam, teamSize);
    
    m_currentControlData->teams[TEAM_RED].teamColour = TEAM_RED;
    m_currentControlData->teams[TEAM_BLUE].teamColour = TEAM_BLUE;
}

/*! @brief Returns the RobotInfo structure for the robot with teamNumber and playerNumber. If there is no robot with those numbers NULL is returned */
const RobotInfo* GameInformation::getRobotInfo(int teamNumber, int playerNumber) const
{
    const TeamInfo* team = getTeamInfo(teamNumber);
    if(team)
    {
        if((playerNumber > getNumberOfPlayers()) || (playerNumber <= 0))
            return 0;
        else
            return &team->players[playerNumber-1];
    }
    else
        return 0;
}

/*! @brief Returns the RoboInfo structure for this robot. If this robot is configured incorrectly NULL will be returned */
const RobotInfo* GameInformation::getMyRobotInfo() const
{
    return getRobotInfo(m_team_number, m_player_number);
}

/*! @brief Returns the TeamInfo structure for teamNumber. If teamNumber is not in last packet then NULL is returned */
const TeamInfo* GameInformation::getTeamInfo(int teamNumber) const
{
    if(m_currentControlData->teams[TEAM_BLUE].teamNumber == teamNumber)
        return &m_currentControlData->teams[TEAM_BLUE];
    else if (m_currentControlData->teams[TEAM_RED].teamNumber == teamNumber)
        return &m_currentControlData->teams[TEAM_RED];
    else
        return 0;
}

/*! @brief Returns the TeamInfo structure for my team */
const TeamInfo* GameInformation::getMyTeamInfo() const
{
    return getTeamInfo(m_team_number);
}

/*! @brief Returns the TeamInfo structure for my opponent */
const TeamInfo* GameInformation::getOpponentTeamInfo() const
{
    if (m_currentControlData->teams[TEAM_BLUE].teamNumber == m_team_number)
        return &m_currentControlData->teams[TEAM_RED];
    else
        return &m_currentControlData->teams[TEAM_BLUE];
}

/*! @brief Find the name of a given state.
    @param theState The robot/game state.
    @return A string of the name of the state.
 */
std::string GameInformation::stateName(RobotState theState)
{
    std::string stateName;
    switch(theState)
    {
        case InitialState:
            stateName = "Initial";
            break;
        case ReadyState:
            stateName = "Ready";
            break;
        case SetState:
            stateName = "Set";
            break;
        case PlayingState:
            stateName = "Playing";
            break;
        case FinishedState:
            stateName = "Finished";
            break;
        case PenalisedState:
            stateName = "Penalised";
            break;
        case SubstituteState:
            stateName = "Sustitute";
            break;
        case RequiresSubstitutionState:
            stateName = "Requires Substitution";
            break;
        default:
            stateName = "Unknown";
            break;
    }
    return stateName;
}

GameInformation& operator<< (GameInformation& info, RoboCupGameControlData* data)
{
    info.process(data);
    return info;
}

void GameInformation::process(RoboCupGameControlData* data)
{
    if (data and memcmp(data->header, GAMECONTROLLER_STRUCT_HEADER, sizeof(data->header)) == 0)
    {
        if (data->teams[0].teamNumber == m_team_number || data->teams[1].teamNumber == m_team_number)
        {
            if (m_data)
                m_last_packet_time = m_data->CurrentTime;
            memcpy(m_currentControlData, data, sizeof(RoboCupGameControlData));
            doGameControllerUpdate();
            nusystem->displayGamePacketReceived(m_actions);
        }
    }
}

