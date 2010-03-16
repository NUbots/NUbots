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
        float x,y;
        for(int ID =0; ID < NUM_STAT_FIELD_OBJECTS; ID++)
	{
		switch(ID)
		{
		// Blue Goal.
                //case FO_BLUE_GOAL:
                //    x = -300.0f;
                //    y = 0.0f;
                //    break;
		case FO_BLUE_LEFT_GOALPOST:
                    x = -300.0f;
                    y = -70.0f;
                    break;
		case FO_BLUE_RIGHT_GOALPOST:
                    x = -300.0f;
                    y = 70.0f;
                    break;
		// Yellow Goal.
                //case FO_YELLOW_GOAL:
                //    x = 300.0f;
                //    y = 0.0f;
                //    break;
		case FO_YELLOW_LEFT_GOALPOST:
                    x = 300.0f;
                    y = 70.0f;
                    break;
		case FO_YELLOW_RIGHT_GOALPOST:
                    x = 300.0f;
                    y = -70.0f;
                    break;
		// Corners
		// Yellow half
		// Back Line
		case FO_CORNER_YELLOW_FIELD_LEFT:
                    x = 300.0f;
                    y = 200.0f;
                    break;
		case FO_CORNER_YELLOW_T_LEFT:
                    x = 300.0f;
                    y = 150.0f;
                    break;
		case FO_CORNER_YELLOW_T_RIGHT:
                    x = 300.0f;
                    y = -150.0f;
                    break;
		case FO_CORNER_YELLOW_FIELD_RIGHT:
                    x = 300.0f;
                    y = -200.0f;
                    break;
		// Yellow Penalty Box
		case FO_CORNER_YELLOW_PEN_LEFT:
                    x = 237.5f;
                    y = 150.0f;
                    break;
		case FO_CORNER_YELLOW_PEN_RIGHT:
                    x = 237.5f;
                    y = -150.0f;
                    break;
		// Half-Way Line
		case FO_CORNER_CENTRE_T_LEFT:
                    x = 0.0f;
                    y = 200.0f;
                    break;
		case FO_CORNER_CENTRE_CIRCLE:
                    x = 0.0f;
                    y = 0.0f;
                    break;
		case FO_CORNER_CENTRE_T_RIGHT:
                    x = 0.0f;
                    y = -200.0f;
                    break;
		// Blue half
		// Back Line
		case FO_CORNER_BLUE_FIELD_LEFT:
                    x = -300.0f;
                    y = -200.0f;
                    break;
		case FO_CORNER_BLUE_T_LEFT:
                    x = -300.0f;
                    y = -150.0f;
                    break;
		case FO_CORNER_BLUE_T_RIGHT:
                    x = -300.0f;
                    y = 150.0f;
                    break;
		case FO_CORNER_BLUE_FIELD_RIGHT:
                    x = -300.0f;
                    y = 200.0f;
                    break;
		// Yellow Penalty Box
		case FO_CORNER_BLUE_PEN_LEFT:
                    x = -237.5f;
                    y = -150.0f;
                    break;
		case FO_CORNER_BLUE_PEN_RIGHT:
                    x = -237.5f;
                    y = 150.0f;
                    break;
                default:
                    x = y = 0.0f;
                    break;
		}
                stationaryFieldObjects.push_back(StationaryObject(ID,x,y));
	}
}

void FieldObjects::InitMobileFieldObjects()
{
        for(int ID =0; ID < NUM_MOBILE_FIELD_OBJECTS; ID++)
	{
                MobileObject mobileObject = MobileObject();
                mobileFieldObjects.push_back(mobileObject);
	}
}

