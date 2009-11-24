#include "GameController.h"
#include <time>

const int c_minStateSelectPressTime = 1000;

void Button::Update(float sensorValue)
{
	long int currTime = (long int) time(NULL);
	timeSinceLastTrigger = currTime - buttonHistory[s].timeOfLastChange;
	// Update any trigger Data
	tempTrigger.buttonNumber = s;
	if(sensorValue < previousValue))
	{
		currentTrigger = buttonTriggerType::NegativeEdge;
	}
	else if(sensorValue > previousValue)
	{
		currentTrigger = buttonTriggerType::PositiveEdge;
	}
	else 
	{
		currentTrigger = buttonTriggerType::None;
	}

	// Update History
	previousValue = sensorValue;
	timeOfLastChange = currTime;
}

	//! Contructor - Standard constructor.
	GameController()
	{
		buttonHistory.clear(); // Start at size of Zero
	};

	//! Destructor - tidy up before deletion.
	~GameController()
	{
		buttonHistory.clear(); // Clear out any existing data.
	};

	GameState GameController::UpdateStates(const GameState &previousState, const sensor &buttonValues, const RoboCupGameControlData &GameControllerReceivedPacket)
	{
		GameState newState = previousState;
		DoButtonUpdate(buttonValues, newState);
		newState.SetFromPacketData(GameControllerReceivedPacket);
		return newState;
	};

void GameController::DoButtonUpdate(const sensor &buttonValues, GameState &state)
{
	bool buttonsValid = true;
	int numberOfButtons = sensor.data.size();
	if(buttons.size() != numberOfButtons)
	{
		buttons.resize(numberOfButtons);
		buttonsValid = false;
	}
	for(int i = 0; i < numberOfButtons; i++)
	{
		buttons[i].Update(sensor.data[i]);
	}
	if(buttonsValid == false)
	{
		return;
	}
		
	// Increment the state if state select button has been pressed.
	if((buttons[buttonIds::stateSelect].currentTrigger == buttonTriggerType::NegativeEdge) && (buttons[buttonIds::stateSelect].timeSinceLastTrigger > c_minStateSelectPressTime))
	{
		state.incrementState();
	}

	// Swap Teams if team select button has been pressed.
	if((buttons[buttonIds::teamSelect].currentTrigger == buttonTriggerType::NegativeEdge) && (buttons[buttonIds::teamSelect].timeSinceLastTrigger > c_minTeamSelectPressTime))
	{
		state.swapTeams();
	}

	// Swap Kickoff if kickoff select button has been pressed.
	if((buttons[buttonIds::teamSelect].currentTrigger == buttonTriggerType::NegativeEdge) && (buttons[buttonIds::teamSelect].timeSinceLastTrigger > c_minTeamSelectPressTime))
	{
		state.toggleKickoff();
	}
	return;
}