/*!
@file GameState.h 
@brief Declaration of NUbots GameState class.
*/

#ifndef GAMECONTROLLER_H
#define GAMECONTROLLER_H

/*!
@brief Class used to store the current game state of the robot.

Used to store the current state of the robot, as well as the current state of the game.
*/
class GameState
{
public:
	GameState(int newRobotNumber = 2, int newTeamNumber = 13): myRobotNumber(newRobotNumber), myTeamNumber(newTeamNumber)
	{
		return;
	}

	bool updateRobotSettings(int newRobotNumber, int newTeamNumber)
	{
		myRobotNumber = newRobotNumber;
		myTeamNumber = newteamNumber;
		myTeam = &currentState.teams[0];
		oppTeam = &currentState.teams[1];
		return true;
	}

	void UpdateFromGameControllerPacket(RoboCupGameControlData newPacket)
	{
		int myTeamIndex = 0;
		if(currentState.teams[0].teamNumber == myTeamNumber)
		{
			myTeamIndex	= 0;
		}
		else if (currentState.teams[1].teamNumber == myTeamNumber)
		{
			myTeamIndex	= 1;
		}
		else 
		{
			return;
		}
		previousState = currentState;
		currentState = newPacket;
		myTeam = currentState.teams[myTeamIndex];
		oppTeam = currentState.teams[!myTeamIndex];
		return;
	}

	int getPlayerNumber()
	{
		return myRobotNumber;
	}

	int getTeamNumber()
	{
		return myTeamNumber;
	}

	unsigned char getNumPlayers()
	{
		return currentState.playersPerTeam;
	}

	unsigned char getCurrentGameState()
	{
		return currentState.state;
	}

	bool isFirstHalf()
	{
		return (bool)currentState.firstHalf;
	}

	unsigned char kickoffTeam()
	{
		return currentState.kickOffTeam;
	}

	bool haveKickoff()
	{
		return (getKickoffTeam() == myTeamNumber);
	}

	unsigned char getSecondaryState()
	{
		return currentState.secondaryState;
	}

	unsigned char lastDropInTeam()
	{
		return currentState.dropInTeam;
	}

	bool wasLastDropinAgainstUs()
	{
		return (lastDropInTeam() == myTeamNumber);
	}

	int timeSinceLastDropin()
	{
		return currentState.dropInTime;
	}

	int getTimeRemaining()
	{
		return currentState.secsRemaining;
	}

	int ourScore()
	{
		return myTeam->score;
	}

	int opponentScore()
	{
		return oppTeam->score;
	}

	int goalDifference()
	{
		return ourScore() - opponentScore();
	}

	unsigned char getOpponentTeamNumber()
	{
		return oppTeam->teamNumber;
	}

	RobotInfo getPlayer(int teamNumber, int playerNumber)
	{
		if(currentState.teams[0].teamNumber == teamNumber)
		{
			return currentState.teams[0].players[playerNumber];
		}
		else
		{
			return currentState.teams[1].players[playerNumber];
		}
	}

	int getPlayerPenalty(int teamNumber, int playerNumber)
	{
		return getPlayer(teamNumber,playerNumber).penalty;
	}
	
	unsigned char getPlayerPenaltyTimeRemaining(int teamNumber, int playerNumber)
	{
		return getPlayer(teamNumber,playerNumber).secsTillUnpenalised;
	}

	bool amIPenlised()
	{
		return getPlayerPenalty(getTeamNumber(), getPlayerNumber());
	}

private:
	int myRobotNumber;
	int myTeamNumber;
	TeamInfo* myTeam;
	TeamInfo* oppTeam;
	RoboCupGameControlData currentState;
	RoboCupGameControlData previousState;
}

#endif