/*!
@file GameController.h 
@brief Declaration of NUbots GameController class.
*/

#ifndef GAMECONTROLLER_H
#define GAMECONTROLLER_H


// Class forward declerations.
class GameState;
class RoboCupGameControlData;
class sensor
{
public:
	vector<float> data;
};

enum buttonIds{
	stateSelect,
	teamSelect,
	kickoffSelect
};

class Button
{
public:
	enum buttonTriggerType{
		None,
		PositiveEdge,
		NegativeEdge
	};
	float previousValue;
	float currentValue;
	long int timeOfLastChange;
	buttonTriggerType currentTrigger;
	long int timeSinceLastTrigger;
	void Update(float sensorValue);
};

/*!
@brief Class used to determine the current state of the robot and the soccer game.

Used to determine the current state of the robot and the soccer game through the inputs from the robots buttons,
ase well as game packets received from the game controller application and the previous state.
*/
class GameController 
{
public:
	GameController();
	~GameController();
	GameState UpdateStates(const GameState &previousState, const sensor &buttonValues, const RoboCupGameControlData &GameControllerReceivedPacket);
private:
	vector<Button> buttons;
};


#endif