#include "FieldObjects.h"
#include <string>
#include <debug.h>
#include "Tools/Math/FieldCalculations.h"
#include <sstream>
#include "Tools/FileFormats/FileFormatException.h"
#include "Tools/Math/General.h"

FieldObjects::FieldObjects()
{
        InitStationaryFieldObjects();
	InitMobileFieldObjects();
        m_timestamp = 0.0;
}
FieldObjects::~FieldObjects()
{
	
}

FieldObjects::FieldObjects(const FieldObjects& source): m_timestamp(source.m_timestamp), self(source.self),
stationaryFieldObjects(source.stationaryFieldObjects), mobileFieldObjects(source.mobileFieldObjects), ambiguousFieldObjects(source.ambiguousFieldObjects)
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
                    break;
                case FO_CORNER_PROJECTED_T_YELLOW_RIGHT:
                    x = 240.0f;
                    y = -200.0f;
                    objectName = "Projected T Yellow Right";
                    break;
                case FO_CORNER_PROJECTED_T_BLUE_LEFT:
                    x = -240.0f;
                    y = -200.0f;
                    objectName = "Projected T Blue Left";
                    break;
                case FO_CORNER_PROJECTED_T_BLUE_RIGHT:
                    x = -240.0f;
                    y = 200.0f;
                    objectName = "Projected T Blue Right";
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

/*! @brief Determines the most likely option for the current ambiguous objects, given a specific field location for the observer.
 @param x x position of observer.
 @param y y position of observer.
 @param heading Heading of observer.
 @return A vector of stationary objects representing the most likely stationary field objects.
 */
std::vector<StationaryObject*> FieldObjects::getExpectedAmbiguousDecisions(float x, float y, float heading)
{
    std::vector<StationaryObject*> expectedObjects;
    const float c_distance_weight = 1.0f;
    const float c_heading_weight = 100.0f;

    // Loop through each of the ambiguous objects and determine the best election option based on the given field position.
    for(std::vector<AmbiguousObject>::iterator amb_it = ambiguousFieldObjects.begin(); amb_it != ambiguousFieldObjects.end(); ++amb_it)
    {
        Self given_location;
        given_location.updateLocationOfSelf(x, y, heading, 1.0f, 1.0f, 1.0f, false);
        float measured_distance = amb_it->measuredDistance();
        float measured_heading = amb_it->measuredBearing();
        float minimum_error = 1000.0f;
        StationaryObject* bestObject = NULL;

        for(std::vector<int>::iterator obj_id_it = amb_it->getPossibleObjectIDs().begin(); obj_id_it != amb_it->getPossibleObjectIDs().end(); ++amb_it)
        {
            StationaryObject* temp_object = &stationaryFieldObjects[(*obj_id_it)];
            float estimated_distance = given_location.CalculateDistanceToStationaryObject(*temp_object);
            float estimated_heading = given_location.CalculateBearingToStationaryObject(*temp_object);

            float distanceError = c_distance_weight * (estimated_distance - measured_distance);
            float headingError = c_heading_weight * (estimated_heading - measured_heading);

            float error_magnitude = sqrt(pow(distanceError,2) + pow(headingError,2));
            if( (!bestObject) or (error_magnitude < minimum_error) )
            {
                bestObject = temp_object;
                minimum_error = error_magnitude;
            }
        }

        expectedObjects.push_back(bestObject);
    }
    return expectedObjects;
}

int FieldObjects::getClosestStationaryOption(const Self& location, const AmbiguousObject& amb_object)
{
    float min_err = 100000, min_err_id = -1;
    std::vector<int> options = amb_object.getPossibleObjectIDs();

    for(std::vector<int>::const_iterator option_it = options.begin(); option_it != options.end(); ++option_it)
    {
        StationaryObject* obj = &stationaryFieldObjects[*option_it];
        float expectedDist = location.CalculateDistanceToStationaryObject(*obj);
        float expectedBear = location.CalculateBearingToStationaryObject(*obj);

        // Use total distance between the two relative points as the error.
        float x_meas = amb_object.measuredDistance() * cos(amb_object.measuredBearing());
        float y_meas = amb_object.measuredDistance() * sin(amb_object.measuredBearing());

        float x_exp = expectedDist * cos(expectedBear);
        float y_exp = expectedDist * sin(expectedBear);

        float x_diff = x_meas - x_exp;
        float y_diff = y_meas - y_exp;

        float total_error = sqrt(x_diff*x_diff + y_diff*y_diff);

//        std::cout << "option: " << obj->getName() << std::endl;
//        std::cout << "error: " << total_weighted_error << " curr min: " << min_err << std::endl;
        // Get smalest value for error
        if(total_error < min_err)
        {
            min_err_id = *option_it;
            min_err = total_error;
        }
    }
    return min_err_id;
}

vector<StationaryObject*> FieldObjects::filterToVisible(const Self& location, const AmbiguousObject& amb_object, float headPan, float fovX)
{
    const float c_view_direction = location.Heading() + headPan;
    const float c_view_range = fovX + location.sdHeading();
    const float c_minHeading =  c_view_direction - c_view_range;
    const float c_maxHeading =  c_view_direction + c_view_range;

    vector<int> poss_ids = amb_object.getPossibleObjectIDs();
    vector<StationaryObject*> result;

//    std::cout << "Ambiguous object: " << amb_object.getName() << std::endl;
//    std::cout << "View direction: " << c_view_direction << " heading: " << location.Heading() << " pan: " << headPan << std::endl;
//    std::cout << "View range: "<< c_view_range << " sd: " << location.sdHeading() << " fov: " << fovX << std::endl;
//    std::cout << "Min heading: "<< c_minHeading << " Max heading: " << c_maxHeading << std::endl;
    for(vector<int>::iterator pos_it = poss_ids.begin(); pos_it != poss_ids.end(); ++pos_it)
    {
        unsigned int index = *pos_it;
        StationaryObject* object = &stationaryFieldObjects.at(index);
        float obj_heading = self.CalculateBearingToStationaryObject(*object);
//        std::cout << "* Object - " << object->getName() << " heading: " << obj_heading << " -> ";
        // Calculate the distance from the viewing direction to the object.
        float delta_angle = mathGeneral::normaliseAngle(obj_heading - c_view_direction);

        // If the distance to the object heading is within the viewing range the object may be seen,
        if(fabs(delta_angle) < c_view_range)
        {
            //std::cout << "Visible" << std::endl;
            result.push_back(object);
        }
        else
        {
            //std::cout << "Not visible" << std::endl;
        }
    }
    return result;
}


std::string FieldObjects::ambiguousName(unsigned int id)
{
    std::string name;
    switch(id)
    {
        case FO_ROBOT_UNKNOWN:
            name = "Unknown Robot";
            break;
        case FO_BLUE_ROBOT_UNKNOWN:
            name = "Unknown Blue Robot";
            break;
        case FO_PINK_ROBOT_UNKNOWN:
            name = "Unknown Pink Robot";
            break;
        case FO_BLUE_GOALPOST_UNKNOWN:
            name = "Unknown Blue Goal Post";
            break;
        case FO_YELLOW_GOALPOST_UNKNOWN:
            name = "Unknown Yellow Goal Post";
            break;
        case FO_CORNER_UNKNOWN_INSIDE_L:
            name = "Unknown Inside L";
            break;
        case FO_CORNER_UNKNOWN_OUTSIDE_L:
            name = "Unknown Outside L";
            break;
        case FO_CORNER_UNKNOWN_T:
            name = "Unknown T";
            break;
        case FO_PENALTY_UNKNOWN:
            name = "Unknown Penalty Spot";
            break;
        default:
            name = "Unknown";
    }
    return name;
}

std::string FieldObjects::toString(bool visibleOnly) const
{
    std::stringstream result;
    result << "Timestamp: " << m_timestamp << std::endl;
    result << std::endl << self.toString() << std::endl;
    int printCount;

    result << "Landmarks" << std::endl;
    printCount = 0;
    for(unsigned int i=0; i < stationaryFieldObjects.size(); i++)
    {
        if(visibleOnly and stationaryFieldObjects[i].isObjectVisible()==false) continue;
        result << stationaryFieldObjects[i].toString() << std::endl;
        ++printCount;
    }
    if(printCount <= 0) result << "None" << std::endl;

    result << std::endl << "Objects" << std::endl;
    printCount = 0;
    for(unsigned int i=0; i < mobileFieldObjects.size(); i++)
    {
        if(visibleOnly and mobileFieldObjects[i].isObjectVisible()==false) continue;
        result << mobileFieldObjects[i].toString() << std::endl;
        ++printCount;
    }
    if(printCount <= 0) result << "None" << std::endl;

    result << std::endl << "Ambiguous" << std::endl;
    printCount = 0;
    for(unsigned int i=0; i < ambiguousFieldObjects.size(); i++)
    {
        result << ambiguousFieldObjects[i].toString() << std::endl;
        ++printCount;
    }
    if(printCount <= 0) result << "None" << std::endl;

    return result.str();
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
        if(input.bad() || input.eof())
        {
            std::stringstream error_msg;
            error_msg << "Error loading stationary object " << i << " of " << size << " - end of file reached." << std::endl;
            throw FileFormatException(error_msg.str());
        }
        input >> p_fob.stationaryFieldObjects[i];
    }

    input.read(reinterpret_cast<char*>(&size), sizeof(size));
    for(int i=0; i < size; i++)
    {
        if(input.bad() || input.eof())
        {
            std::stringstream error_msg;
            error_msg << "Error loading mobile object " << i << " of " << size << " - end of file reached." << std::endl;
            throw FileFormatException(error_msg.str());
        }
        input >> p_fob.mobileFieldObjects[i];
    }

    input.read(reinterpret_cast<char*>(&size), sizeof(size));
    p_fob.ambiguousFieldObjects.resize(size);
    for(int i=0; i < size; i++)
    {
        if(input.bad() || input.eof())
        {
            std::stringstream error_msg;
            error_msg << "Error loading ambiguous object " << i << " of " << size << " - end of file reached." << std::endl;
            throw FileFormatException(error_msg.str());
        }
        input >> p_fob.ambiguousFieldObjects[i];
    }
    return input;
}
