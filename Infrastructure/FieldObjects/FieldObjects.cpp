#include "FieldObjects.h"
#include <string>
#include <debug.h>
#include "Tools/Math/FieldCalculations.h"

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
    {
        stationaryFieldObjects[i].preProcess(timestamp);
    }
    for (unsigned int i=0; i<mobileFieldObjects.size(); i++)
    {
        mobileFieldObjects[i].preProcess(timestamp);
    }
    ambiguousFieldObjects.clear();
}

/*! @brief Postprocesses each field object
    @brief timestamp the current timestamp in ms
 
    This calls the postprocess on each object, including the ambiguous objects
 */
void FieldObjects::postProcess(const float timestamp)
{
    m_timestamp = timestamp;
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
                case FO_PENALTY_BLUE:
                    x = -120.0f;
                    y = 0.0f;
                    objectName = "Blue Penalty Spot";
                    break;
                case FO_PENALTY_YELLOW:
                    x = 120.0f;
                    y = 0.0f;
                    objectName = "Yellow Penalty Spot";
                    break;
		case FO_CORNER_CENTRE_CIRCLE:
		    x = 0.0f;
                    y = 0.0f;
                    objectName = "Centre Circle";
                    break;
                case FO_CORNER_PROJECTED_T_YELLOW_LEFT:
                    x = 240.0f;
                    y = 200.0f;
                    objectName = "Projected T Yellow Left";
                case FO_CORNER_PROJECTED_T_YELLOW_RIGHT:
                    x = 240.0f;
                    y = -200.0f;
                    objectName = "Projected T Yellow Right";
                case FO_CORNER_PROJECTED_T_BLUE_LEFT:
                    x = -240.0f;
                    y = -200.0f;
                    objectName = "Projected T Blue Left";
                case FO_CORNER_PROJECTED_T_BLUE_RIGHT:
                    x = -240.0f;
                    y = 200.0f;
                    objectName = "Projected T Blue Right";
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
                std::string objectName;
                switch (ID)
                {
                case FO_BALL:
                    objectName = "Ball";
                    break;
                case FO_BLUE_ROBOT_1:
                    objectName = "Blue Robot 1";
                    break;
                case FO_BLUE_ROBOT_2:
                    objectName = "Blue Robot 2";
                    break;
                case FO_BLUE_ROBOT_3:
                    objectName = "Blue Robot 3";
                    break;
                case FO_BLUE_ROBOT_4:
                    objectName = "Blue Robot 4";
                    break;
                case FO_PINK_ROBOT_1:
                    objectName = "Pink Robot 1";
                    break;
                case FO_PINK_ROBOT_2:
                    objectName = "Pink Robot 2";
                    break;
                case FO_PINK_ROBOT_3:
                    objectName = "Pink Robot 3";
                    break;
                case FO_PINK_ROBOT_4:
                    objectName = "Pink Robot 4";
                    break;
                default:
                    objectName = "Unknown Mobile Object";
                    break;
                }

                MobileObject mobileObject = MobileObject(ID, objectName);
                mobileFieldObjects.push_back(mobileObject);
	}
}

/*! @brief Determines the IDs of stationary objects that should be visible for a given field position.
 @param x x position of observer.
 @param y y position of observer.
 @param heading Heading of observer.
 @param headYaw Additional yaw caused by head position relative to body.
 @param headPitch Additional pitch caused by head position relative to body.
 @param FoV_x Horizontal field of view of the observer.
 @param FoV_y Vertical field of view of the observer.
 @return A vector of IDs representing the stationary field objects expected to be visible.
 */
std::vector<FieldObjects::StationaryFieldObjectID> FieldObjects::GetPossibleObservationIds(float x, float y, float heading, 
                                                                            float headYaw, float headPitch, 
                                                                            float FoV_x, float FoV_y)
{
    // Calculate limits.
    // Note: These limits assume that the camera is flat horizontally.
    float maxAngle = heading + headYaw + FoV_x / 2.0f; // Radians
    float minAngle = heading + headYaw - FoV_x / 2.0f; // Radians
    float minDistance = 0;      // cm
    float maxDistance = 200;    // cm
    
    // Create temporary variables for use inside loop.
    std::vector<StationaryFieldObjectID> visibleIds;
    Vector2<float> basePos(x,y), targetPosition;
    float distance, angle;
    bool expectedVisible;
    
    visibleIds.clear(); // Make sure list is empty.
    
    std::vector<StationaryObject>::iterator stat_obj_iterator;

    // Check all objects.
    for(stat_obj_iterator = stationaryFieldObjects.begin();
        stat_obj_iterator != stationaryFieldObjects.end();
        stat_obj_iterator++)
	{
        // Get position of current field object.
        targetPosition = stat_obj_iterator->getFieldLocation();
        // Calculate expected measurments.
        distance = DistanceBetweenPoints(basePos,targetPosition);
        angle = AngleBetweenPoints(basePos,targetPosition);
        // Check if within limits.
        expectedVisible = (angle > minAngle) and (angle < maxAngle) and (distance > minDistance) and (distance < maxDistance);
        // If expected to be visible put on list.
        if(expectedVisible)
            visibleIds.push_back(StationaryFieldObjectID(stat_obj_iterator->getID()));
    }
    return visibleIds;
}

/*! @brief Determines the IDs of mobile objects that should be visible for a given field position.
 @param x x position of observer.
 @param y y position of observer.
 @param heading Heading of observer.
 @param headYaw Additional yaw caused by head position relative to body.
 @param headPitch Additional pitch caused by head position relative to body.
 @param FoV_x Horizontal field of view of the observer.
 @param FoV_y Vertical field of view of the observer.
 @return A vector of IDs representing the stationary field objects expected to be visible.
 */
std::vector<FieldObjects::MobileFieldObjectID> FieldObjects::GetPossibleMobileObservationIds(float x, float y, float heading,
                                                                            float headYaw, float headPitch,
                                                                            float FoV_x, float FoV_y)
{
    // Calculate limits.
    // Note: These limits assume that the camera is flat horizontally.
    float maxAngle = heading + headYaw + FoV_x / 2.0f; // Radians
    float minAngle = heading + headYaw - FoV_x / 2.0f; // Radians
    float minDistance = 0;      // cm
    float maxDistance = 200;    // cm

    // Create temporary variables for use inside loop.
    std::vector<MobileFieldObjectID> visibleIds;
    Vector2<float> basePos(x,y), targetPosition;
    float distance, angle;
    bool expectedVisible;

    visibleIds.clear(); // Make sure list is empty.

    std::vector<MobileObject>::iterator stat_obj_iterator;

    // Check all objects.
    for(stat_obj_iterator = mobileFieldObjects.begin();
        stat_obj_iterator != mobileFieldObjects.end();
        stat_obj_iterator++)
        {
        // Get position of current field object.
        targetPosition = stat_obj_iterator->getEstimatedFieldLocation();
        // Calculate expected measurments.
        distance = DistanceBetweenPoints(basePos,targetPosition);
        angle = AngleBetweenPoints(basePos,targetPosition);
        // Check if within limits.
        expectedVisible = (angle > minAngle) and (angle < maxAngle) and (distance > minDistance) and (distance < maxDistance);
        // If expected to be visible put on list.
        if(expectedVisible)
            visibleIds.push_back(MobileFieldObjectID(stat_obj_iterator->getID()));
    }
    return visibleIds;
}

std::ostream& operator<< (std::ostream& output, const FieldObjects& p_fob)
{
    int size;
    output.write(reinterpret_cast<const char*>(&p_fob.m_timestamp), sizeof(p_fob.m_timestamp));
    output << p_fob.self;
    size = p_fob.stationaryFieldObjects.size();
    output.write(reinterpret_cast<const char*>(&size), sizeof(size));
    for(unsigned int i=0; i < p_fob.stationaryFieldObjects.size(); i++)
        output << p_fob.stationaryFieldObjects[i];

    size = p_fob.mobileFieldObjects.size();
    output.write(reinterpret_cast<const char*>(&size), sizeof(size));
    for(unsigned int i=0; i < p_fob.mobileFieldObjects.size(); i++)
        output << p_fob.mobileFieldObjects[i];

    size = p_fob.ambiguousFieldObjects.size();
    output.write(reinterpret_cast<const char*>(&size), sizeof(size));
    for(unsigned int i=0; i < p_fob.ambiguousFieldObjects.size(); i++)
        output << p_fob.ambiguousFieldObjects[i];
    return output;
}

std::istream& operator>> (std::istream& input, FieldObjects& p_fob)
{
    input.read(reinterpret_cast<char*>(&p_fob.m_timestamp), sizeof(p_fob.m_timestamp));
    input >> p_fob.self;

    int size;
    input.read(reinterpret_cast<char*>(&size), sizeof(size));
    for(int i=0; i < size; i++)
    {
        input >> p_fob.stationaryFieldObjects[i];
    }

    input.read(reinterpret_cast<char*>(&size), sizeof(size));
    for(int i=0; i < size; i++)
    {
        input >> p_fob.mobileFieldObjects[i];
    }

    input.read(reinterpret_cast<char*>(&size), sizeof(size));
    p_fob.ambiguousFieldObjects.resize(size);
    for(int i=0; i < size; i++)
    {
        input >> p_fob.ambiguousFieldObjects[i];
    }
    return input;
}
