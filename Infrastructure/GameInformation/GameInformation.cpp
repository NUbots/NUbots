#include "GameInformation.h"

#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/NUBlackboard.h"
#include "NUPlatform/NUIO/GameControllerPort.h"
#include "NUPlatform/NUPlatform.h"

#include <memory.h>
#include <sstream>
#include "debug.h"

GameInformation::GameInformation(int playerNumber, int teamNumber): TimestampedData()
{
    m_data = Blackboard->Sensors;
    m_actions = Blackboard->Actions;
    m_port = 0;
    
    m_player_number = playerNumber;
    m_team_number = teamNumber;
    m_state = InitialState;
    
    // Initialise the game controller packet (this will only be used if no game control packet is actually received)
    m_currentControlData = new RoboCupGameControlData();
    m_currentControlData->teams[TEAM_BLUE].teamNumber = m_team_number;
    m_last_packet_time = 0;
    
    // Initialise the game controller return packet (this tells the game controller of manual penalties)
    m_currentReturnData = new RoboCupGameControlReturnData();
    memcpy(m_currentReturnData->header, GAMECONTROLLER_RETURN_STRUCT_HEADER, sizeof(m_currentReturnData->header));
    m_currentReturnData->version = GAMECONTROLLER_RETURN_STRUCT_VERSION;
    
    m_led_red = vector<float>(3,0);
    m_led_red[0] = 1;
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
        return static_cast<TeamColour>(team->goalColour);
    else
        return BlueTeam;
}

/*! @brief Returns the current state */
GameInformation::RobotState GameInformation::getCurrentState() const 
{
    return m_state;
}

/*! @brief Returns the reason the robot is penalised */
int GameInformation::getPenaltyReason() const
{
    const RobotInfo* info = getMyRobotInfo();
    if (info)
        return info->penalty;
    else
        return PENALTY_NONE;
}

/*! @brief Returns true if this robot is currently a substitute */
bool GameInformation::amIASubstitute() const
{
    return getPenaltyReason() == PENALTY_SPL_SUBSTITUTE;
}

/*! @brief Returns the player number of the substitute player */
int GameInformation::getSubstituteNumber() const
{
    vector<vector<int> > penalties = getPenaltyReasons();
    for (size_t i=0; i<penalties.size(); i++)
    {
        if (penalties[i][1] == PENALTY_SPL_SUBSTITUTE)
            return penalties[i][0];
    }
    return -1;
}

/*! @brief Returns true if the game controller is working */
bool GameInformation::gameControllerWorking() const
{
    if (m_data)
        return (m_data->CurrentTime - m_last_packet_time) < 10000;
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

/*! @brief Returns the number of players on my team that are penalised */
int GameInformation::getNumberOfPlayersPenalised() const
{
    return getPenaltyReasons().size();
}

/*! @brief Returns a list containing the penalty reasons for each that is penalised 
           The list is formatted as [[playernumber, reason], [playernumber, reason], ...]
 */
vector<vector<int> > GameInformation::getPenaltyReasons() const
{
    vector<vector<int> > result;
    const TeamInfo* team = getMyTeamInfo();
    if(team)
    {
        for (int i=0; i<MAX_NUM_PLAYERS; i++)
        {
            if (team->players[i].penalty != PENALTY_NONE)
            {
                vector<int> num_reason;
                num_reason.push_back(i+1);
                num_reason.push_back(team->players[i].penalty);
                result.push_back(num_reason);
            }
        }
    }
    return result;
}

/*! @brief Updates the GameInformation based on the m_currentControlData (ie. sets m_state)
 */
void GameInformation::doGameControllerUpdate()
{
    const RobotInfo* info = getMyRobotInfo();
    if (info and info->penalty != PENALTY_NONE)
        m_state = PenalisedState;
    else
        m_state = static_cast<RobotState>(m_currentControlData->state);
}

/*! @brief Sends the 'alive' return packet to the game controller
 *		   This function should be called about once a second
 */
void GameInformation::sendAlivePacket()
{
	m_currentReturnData->team = m_team_number;
	m_currentReturnData->player = m_player_number;
	m_currentReturnData->message = GAMECONTROLLER_RETURN_MSG_ALIVE;

	if (m_port)
		m_port->sendReturnPacket(m_currentReturnData);
}

/*! @brief Sends a request for pick up to the game controller
 */
void GameInformation::requestForPickup()
{
	m_currentReturnData->team = m_team_number;
	m_currentReturnData->player = m_player_number;
	m_currentReturnData->message = GAMECONTROLLER_RETURN_MSG_REQUEST_FOR_SUBSTITUTION;
    
	if (m_port)
		m_port->sendReturnPacket(m_currentReturnData);
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
    
    if (m_port)
        m_port->sendReturnPacket(m_currentReturnData);
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
        if((playerNumber > MAX_NUM_PLAYERS) || (playerNumber <= 0))
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
            Platform->toggle(NUPlatform::Led2, m_actions->CurrentTime, m_led_red);
        }
    }
}

void GameInformation::addNetworkPort(GameControllerPort* port)
{
    m_port = port;
}

std::string stateToString(int state)
{
    std::string result;
    switch(state)
    {
    case GameInformation::InitialState:
        result = "Initial";
        break;
    case GameInformation::ReadyState:
        result = "Ready";
        break;
    case GameInformation::SetState:
        result = "Set";
        break;
    case GameInformation::PlayingState:
        result = "Playing";
        break;
    case GameInformation::FinishedState:
        result = "Finished";
        break;
    case GameInformation::PenalisedState:
        result = "Penalised";
        break;
    default:
        result = "Unknown";
    }
    return result;
}

std::string teamToString(int team)
{
    std::string result;
    switch(team)
    {
    case TEAM_BLUE:
        result = "Blue";
        break;
    case TEAM_RED:
        result = "Red";
        break;
    default:
        result = "Unknown";
    }
    return result;
}

std::string goalToString(int goal)
{
    std::string result;
    switch(goal)
    {
    case GOAL_BLUE:
        result = "Blue";
        break;
    case GOAL_YELLOW:
        result = "Yellow";
        break;
    default:
        result = "Unknown";
    }
    return result;
}

/*!
@brief Produce human readable string summary of the data.
@return Formatted string summary of the current data.
*/
std::string GameInformation::toString() const
{
    const int NUM_TEAMS = 2;

    std::stringstream result;
    result << "Game Information Time: " << GetTimestamp() << std::endl;
    result << "Last Packet Time: " << m_last_packet_time << std::endl;
    result << "Player Number: " << m_player_number << std::endl;
    result << "Team Number: " << m_team_number << std::endl;
    result << "State: " << stateToString(m_state) << std::endl;
    result << std::endl;
    if(m_currentControlData)
    {
        result << "Game Controller Info" << std::endl;
        result << "Version: " << m_currentControlData->version << std::endl;
        result << "Players Per Team: " << (int)m_currentControlData->playersPerTeam << std::endl;
        result << "State: " << stateToString(m_currentControlData->state) << std::endl;
        result << "First Half: " << (m_currentControlData->firstHalf ? "True" : "False") << std::endl;
        result << "Kick-off Team: " << (int)m_currentControlData->kickOffTeam << std::endl;
        result << "Penalty Shootout: " << (m_currentControlData->secondaryState ? "True" : "False") << std::endl;
        result << "Drop in Team: " << (int)m_currentControlData->dropInTeam << std::endl;
        result << "Drop in Time: " << (int)m_currentControlData->dropInTime << std::endl;
        result << "Seconds Remaining: " << (int)m_currentControlData->secsRemaining << std::endl;
        for (int t=0; t < NUM_TEAMS; ++t)
        {
            result << std::endl;
            result << teamToString(m_currentControlData->teams[t].teamColour) << " Team (" << (int)m_currentControlData->teams[t].teamNumber << ")" << std::endl;
            result << "Goal Colour: " << goalToString(m_currentControlData->teams[t].goalColour) << std::endl;
            result << "Score: " << (int)m_currentControlData->teams[t].score << std::endl;
            for(int p=0; p < m_currentControlData->playersPerTeam; ++p)
            {
                result << "Player " << p+1 << " - ";
                result << "Penalised: " << (m_currentControlData->teams[t].players[p].penalty ? "True" : "False");
                if(m_currentControlData->teams[t].players[p].penalty)
                    result << "\t (" << (int)m_currentControlData->teams[t].players[p].secsTillUnpenalised << ")";
                 result << std::endl;
            }
        }
    }
    else
    {
        result << "No game information available." << std::endl;
    }
    return result.str();
}

std::ostream& operator<< (std::ostream& output, const GameInformation& p_game)
{
    output.write(reinterpret_cast<const char*>(&p_game.m_timestamp), sizeof(p_game.m_timestamp));
    output.write(reinterpret_cast<const char*>(&p_game.m_last_packet_time), sizeof(p_game.m_last_packet_time));
    output.write(reinterpret_cast<const char*>(&p_game.m_player_number), sizeof(p_game.m_player_number));
    output.write(reinterpret_cast<const char*>(&p_game.m_team_number), sizeof(p_game.m_team_number));
    output.write(reinterpret_cast<const char*>(&p_game.m_state), sizeof(p_game.m_state));
    output.write(reinterpret_cast<const char*>(p_game.m_currentControlData), sizeof(*p_game.m_currentControlData));
    return output;
}

std::istream& operator>> (std::istream& input, GameInformation& p_game)
{
    input.read(reinterpret_cast<char*>(&p_game.m_timestamp), sizeof(p_game.m_timestamp));
    input.read(reinterpret_cast<char*>(&p_game.m_last_packet_time), sizeof(p_game.m_last_packet_time));
    input.read(reinterpret_cast<char*>(&p_game.m_player_number), sizeof(p_game.m_player_number));
    input.read(reinterpret_cast<char*>(&p_game.m_team_number), sizeof(p_game.m_team_number));
    input.read(reinterpret_cast<char*>(&p_game.m_state), sizeof(p_game.m_state));
    input.read(reinterpret_cast<char*>(p_game.m_currentControlData), sizeof(*p_game.m_currentControlData));
    return input;
}


