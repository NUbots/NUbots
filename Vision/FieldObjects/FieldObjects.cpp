#include "FieldObjects.h"
#include <string>

FieldObjects::FieldObjects()
{
	InitStationaryFieldObjects();
	InitMobileFieldObjects();
}
FieldObjects::~FieldObjects()
{
	
}

/*! @brief Preprocesses each field object
    @param timestamp the current timestamp in ms
 
    This calls preprocess on each object, and clears all of the ambiguous objects
 */
void FieldObjects::preProcess(const float timestamp)
{
    for (unsigned int i=0; i<stationaryFieldObjects.size(); i++)
        stationaryFieldObjects[i].preProcess(timestamp);
    for (unsigned int i=0; i<mobileFieldObjects.size(); i++)
        mobileFieldObjects[i].preProcess(timestamp);
    ambiguousFieldObjects.clear();
}

/*! @brief Postprocesses each field object
    @brief timestamp the current timestamp in ms
 
    This calls the postprocess on each object, including the ambiguous objects
 */
void FieldObjects::postProcess(const float timestamp)
{
    for (unsigned int i=0; i<stationaryFieldObjects.size(); i++)
        stationaryFieldObjects[i].postProcess(timestamp);
    for (unsigned int i=0; i<mobileFieldObjects.size(); i++)
        mobileFieldObjects[i].postProcess(timestamp);
    for (unsigned int i=0; i<ambiguousFieldObjects.size(); i++)
        ambiguousFieldObjects[i].postProcess(timestamp);
}

void FieldObjects::InitStationaryFieldObjects()
{
        float x,y;
        std::string objectName;
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
                    objectName = "Left Blue Goal Post";
                    break;
		case FO_BLUE_RIGHT_GOALPOST:
                    x = -300.0f;
                    y = 70.0f;
                    objectName = "Right Blue Goal Post";
                    break;
		// Yellow Goal.
                //case FO_YELLOW_GOAL:
                //    x = 300.0f;
                //    y = 0.0f;
                //    break;
		case FO_YELLOW_LEFT_GOALPOST:
                    x = 300.0f;
                    y = 70.0f;
                    objectName = "Left Yellow Goal Post";
                    break;
		case FO_YELLOW_RIGHT_GOALPOST:
                    x = 300.0f;
                    y = -70.0f;
                    objectName = "Right Yellow Goal Post";
                    break;
		// Corners
		// Yellow half
		// Back Line
		case FO_CORNER_YELLOW_FIELD_LEFT:
                    x = 300.0f;
                    y = 200.0f;
                    objectName = "Left Yellow Field Corner";
                    break;
		case FO_CORNER_YELLOW_T_LEFT:
                    x = 300.0f;
                    y = 150.0f;
                    objectName = "Left Yellow T Intersect";
                    break;
		case FO_CORNER_YELLOW_T_RIGHT:
                    x = 300.0f;
                    y = -150.0f;
                    objectName = "Right Yellow T Intersect";
                    break;
		case FO_CORNER_YELLOW_FIELD_RIGHT:
                    x = 300.0f;
                    y = -200.0f;
                    objectName = "Right Yellow Field Corner";
                    break;
		// Yellow Penalty Box
		case FO_CORNER_YELLOW_PEN_LEFT:
                    x = 237.5f;
                    y = 150.0f;
                    objectName = "Left Yellow Penalty Box Corner";
                    break;
		case FO_CORNER_YELLOW_PEN_RIGHT:
                    x = 237.5f;
                    y = -150.0f;
                    objectName = "Right Yellow Penalty Box Corner";
                    break;
		// Half-Way Line
		case FO_CORNER_CENTRE_T_LEFT:
                    x = 0.0f;
                    y = 200.0f;
                    objectName = "Left Half-Way T Intersect";
                    y = 0.0f;
                    break;
		case FO_CORNER_CENTRE_T_RIGHT:
                    x = 0.0f;
                    y = -200.0f;
                    objectName = "Right Half-Way T Intersect";
                    break;
		// Blue half
		// Back Line
		case FO_CORNER_BLUE_FIELD_LEFT:
                    x = -300.0f;
                    y = -200.0f;
                    objectName = "Left Blue Field Corner";
                    break;
		case FO_CORNER_BLUE_T_LEFT:
                    x = -300.0f;
                    y = -150.0f;
                    objectName = "Left Blue T intersect";
                    break;
		case FO_CORNER_BLUE_T_RIGHT:
                    x = -300.0f;
                    y = 150.0f;
                    objectName = "Right Blue T intersect";
                    break;
		case FO_CORNER_BLUE_FIELD_RIGHT:
                    x = -300.0f;
                    y = 200.0f;
                    objectName = "Left Blue Field Corner";
                    break;
		// Yellow Penalty Box
		case FO_CORNER_BLUE_PEN_LEFT:
                    x = -237.5f;
                    y = -150.0f;
                    objectName = "Left Blue Penalty Box Corner";
                    break;
		case FO_CORNER_BLUE_PEN_RIGHT:
                    x = -237.5f;
                    y = 150.0f;
                    objectName = "Right Blue Penalty Box Corner";
                    break;
                default:
                    x = y = 0.0f;
                    objectName = "Undefined";
                    break;
		}
                stationaryFieldObjects.push_back(StationaryObject(x, y, ID, objectName));
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

