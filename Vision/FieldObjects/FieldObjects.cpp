#include "FieldObjects.h"

FieldObjects::FieldObjects()
{
	InitStationaryFieldObjects();
	InitMobileFieldObjects();
}
FieldObjects::~FieldObjects()
{
	
}

void FieldObjects::InitStationaryFieldObjects()
{
	for(int ID =0; ID < NUM_STAT_FIELD_OBJECTS; ID++)
	{
		StationaryObject statObject;
		switch(ID)
		{
		// Blue Goal.
                //case FO_BLUE_GOAL:
                //statObject = StationaryObject(-300.0f, 0.0f);
                //break;
		case FO_BLUE_LEFT_GOALPOST:
		statObject = StationaryObject(-300.0f, -70.0f);
		break;
		case FO_BLUE_RIGHT_GOALPOST:
		statObject = StationaryObject(-300.0f, 70.0f);
		break;
		// Yellow Goal.
                //case FO_YELLOW_GOAL:
                //statObject = StationaryObject(300.0f, 0.0f);
                //break;
		case FO_YELLOW_LEFT_GOALPOST:
		statObject = StationaryObject(300.0f, 70.0f);
		break;
		case FO_YELLOW_RIGHT_GOALPOST:
		statObject = StationaryObject(300.0f, -70.0f);
		break;
		// Corners
		// Yellow half
		// Back Line
		case FO_CORNER_YELLOW_FIELD_LEFT:
		statObject = StationaryObject(300.0f, 200.0f);
		break;
		case FO_CORNER_YELLOW_T_LEFT:
		statObject = StationaryObject(300.0f, 150.0f);
		break;        
		case FO_CORNER_YELLOW_T_RIGHT:
		statObject = StationaryObject(300.0f, -150.0f);
		break;
		case FO_CORNER_YELLOW_FIELD_RIGHT:
		statObject = StationaryObject(300.0f, -200.0f);
		break;
		// Yellow Penalty Box
		case FO_CORNER_YELLOW_PEN_LEFT:
		statObject = StationaryObject(237.5f, 150.0f);
		break;
		case FO_CORNER_YELLOW_PEN_RIGHT:
		statObject = StationaryObject(237.5f, -150.0f);
		break;
		// Half-Way Line
		case FO_CORNER_CENTRE_T_LEFT:
		statObject = StationaryObject(0.0f, 200.0f);
		break;
		case FO_CORNER_CENTRE_CIRCLE:
		statObject = StationaryObject(0.0f, 0.0f);
		break;
		case FO_CORNER_CENTRE_T_RIGHT:
		statObject = StationaryObject(0.0f, -200.0f);
		break;
		// Blue half
		// Back Line
		case FO_CORNER_BLUE_FIELD_LEFT:
		statObject = StationaryObject(-300.0f, -200.0f);
		break;
		case FO_CORNER_BLUE_T_LEFT:
		statObject = StationaryObject(-300.0f, -150.0f);
		break;        
		case FO_CORNER_BLUE_T_RIGHT:
		statObject = StationaryObject(-300.0f, 150.0f);
		break;
		case FO_CORNER_BLUE_FIELD_RIGHT:
		statObject = StationaryObject(-300.0f, 200.0f);
		break;
		// Yellow Penalty Box
		case FO_CORNER_BLUE_PEN_LEFT:
		statObject = StationaryObject(-237.5f, -150.0f);
		break;
		case FO_CORNER_BLUE_PEN_RIGHT:
		statObject = StationaryObject(-237.5f, 150.0f);
		break;
		}
		stationaryFieldObjects.push_back(statObject);
	}
}
void FieldObjects::InitMobileFieldObjects()
{
	for(int ID; ID < NUM_MOBILE_FIELD_OBJECTS; ID++)
	{
                MobileObject mobileObject;
                mobileFieldObjects.push_back(mobileObject);
	}
}
