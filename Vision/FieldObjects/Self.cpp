#include "Self.h"
#include "StationaryObject.h"
#include "MobileObject.h"
#include "Tools/Math/General.h"
#include "debug.h"
using namespace mathGeneral;


Self::Self()
{
	WorldModelLocation[0] = 0;
	WorldModelLocation[1] = 0;
	WorldModelLocation[2] = 0;
    
    WorldModelLocationError[0] = 600;
    WorldModelLocationError[1] = 600;
    WorldModelLocationError[0] = 6.283;
}
Self::~Self()
{
}


Self::Self(float wmX, float wmY)
{
	WorldModelLocation[0] = wmX;
    WorldModelLocation[1] = wmY;
    WorldModelLocation[2] = 0;
    
    WorldModelLocationError[0] = 600;
    WorldModelLocationError[1] = 600;
    WorldModelLocationError[0] = 6.283;
}
void Self::updateLocationOfSelf(float wmX, float wmY, float heading, float sdX, float sdY, float sdHeading,bool lost)
{
	WorldModelLocation[0] = wmX;
    WorldModelLocation[1] = wmY;
    WorldModelLocation[2] = heading;
    
    WorldModelLocationError[0] = sdX;
    WorldModelLocationError[1] = sdY;
    WorldModelLocationError[2] = sdHeading;
	amILost = lost;
}

bool Self::lost()
{
    if (WorldModelLocationError[2]*2 > PI/4)      // if heading is really uncertain we are lost
        return true;
    else if (WorldModelLocationError[0]*2 > 150 or WorldModelLocationError[1]*2 > 100)
        return true;
    else
        return false;
}

/*! @brief Calculate the difference from the given field state 
    @param targetState the desired field state [x(cm), y(cm), heading(rad)] 
    @return the difference between the current field state and the target state, otherwise known as the relative location of the target state [distance, bearing, heading]
 */
std::vector<float> Self::CalculateDifferenceFromFieldState(const std::vector<float> targetState)
{
    float selfX = WorldModelLocation[0];
    float selfY = WorldModelLocation[1];
    float selfHeading = WorldModelLocation[2];

    float targetX = targetState[0];
    float targetY = targetState[1];
    float targetHeading = targetState[2];

    float diffX = targetX - selfX;
    float diffY = targetY - selfY;
    if( (diffX == 0) && (diffY == 0)) diffY = 0.0001;
    float positionHeading = atan2(diffY, diffX);

    float distance = sqrt( diffX * diffX + diffY * diffY );
    float bearing = normaliseAngle(positionHeading - selfHeading);
    float headingDifference = normaliseAngle(targetHeading - selfHeading);

    std::vector<float> result(3,0.0f);
    result[0] = distance;
    result[1] = bearing;
    result[2] = headingDifference;

    return result;
}

/*! @brief Calculate the difference from the given field location [x,y]
    @param desiredLocation the desired field location [x(cm), y(cm)]
    @return the difference between the current location and the target location [distance, bearing]
 */
std::vector<float> Self::CalculateDifferenceFromFieldLocation(const std::vector<float> desiredLocation)
{
    float selfX = WorldModelLocation[0];
    float selfY = WorldModelLocation[1];
    float selfHeading = WorldModelLocation[2];
    
    float targetX = desiredLocation[0];
    float targetY = desiredLocation[1];
    
    float diffX = targetX - selfX;
    float diffY = targetY - selfY;
    if( (diffX == 0) && (diffY == 0)) diffY = 0.0001;
    float positionHeading = atan2(diffY, diffX);
    
    float distance = sqrt( diffX * diffX + diffY * diffY );
    float bearing = normaliseAngle(positionHeading - selfHeading);
    
    std::vector<float> result(2,0);
    result[0] = distance;
    result[1] = bearing;
    return result;
}

/*! @brief Calculates the difference between the current state and the stationary object
    @param theObject the stationary field object you want the relative coordinates
    @return the difference between the current field state and theObject, otherwise known as the relative location of theObject [distance, bearing]
 */
std::vector<float> Self::CalculateDifferenceFromStationaryObject(const StationaryObject& theObject)
{
    std::vector<float> fieldlocation(2,0);
    fieldlocation[0] = theObject.X();
    fieldlocation[1] = theObject.Y();
    
    return CalculateDifferenceFromFieldLocation(fieldlocation);
}

/*! @brief Calculate the distance to a stationary object */
float Self::CalculateDistanceToStationaryObject(const StationaryObject& theObject)
{
    float selfX = WorldModelLocation[0];
    float selfY = WorldModelLocation[1];
    float diffX = theObject.X() - selfX;
    float diffY = theObject.Y() - selfY;
    float distance = sqrt( diffX * diffX + diffY * diffY );
    return distance;
}

/*! @brief Calculate the bearing to a stationary object */
float Self::CalculateBearingToStationaryObject(const StationaryObject& theObject)
{
    float selfX = WorldModelLocation[0];
    float selfY = WorldModelLocation[1];
    float selfHeading = WorldModelLocation[2];
    float diffX = theObject.X() - selfX;
    float diffY = theObject.Y() - selfY;
    float positionHeading = atan2(diffY, diffX);
    float bearing = normaliseAngle(positionHeading - selfHeading);
    return bearing;
}

/*! @brief Calculates the difference from a goal 
    @return the difference between the current position and the goal [distance, bearing] */
std::vector<float> Self::CalculateDifferenceFromGoal(const StationaryObject& goalpost)
{
    std::vector<float> fieldlocation(2,0);
    fieldlocation[0] = goalpost.X();
    return CalculateDifferenceFromFieldLocation(fieldlocation);
}

/*! @brief Calculates the angular width of the goal (in radians) from the current state */
float Self::CalculateAngularWidthOfGoal(const StationaryObject& goalpost)
{
    std::vector<float> location = CalculateDifferenceFromGoal(goalpost);
    float distance = location[0];
    float bearing = normaliseAngle(location[1] + Heading());    // bearing from the field x-axis!
    float width = 2*fabs(goalpost.Y());

    // python: angularwidth = numpy.arctan2(width*numpy.cos(bearing), 2*distance + width*numpy.sin(bearing)) + numpy.arctan2(width*numpy.cos(bearing), 2*distance - width*numpy.sin(bearing))
    float angularwidth = atan2(width*cos(bearing), 2*distance + width*sin(bearing)) + atan2(width*cos(bearing), 2*distance - width*sin(bearing));
    return angularwidth;
}

/*! @brief Calculates the angular width (in radians) of the goal from the mobile object */
float Self::CalculateAngularWidthOfGoalFromMobileObject(const StationaryObject& goalpost, const MobileObject& mobileobject)
{
    float mobileX = mobileobject.X();
    float mobileY = mobileobject.Y();
    
    float goalX = goalpost.X();
    float goalY = 0;
    
    float diffX = goalX - mobileX;
    float diffY = goalY - mobileY;
    if( (diffX == 0) && (diffY == 0)) diffY = 0.0001;
    
    float distance = sqrt( diffX * diffX + diffY * diffY );
    float bearing = atan2(diffY, diffX);                        // bearing FROM the mobile object to the goal (from the field x-axis)
    float width = 2*fabs(goalpost.Y());
    
    // python: angularwidth = numpy.arctan2(width*numpy.cos(bearing), 2*distance + width*numpy.sin(bearing)) + numpy.arctan2(width*numpy.cos(bearing), 2*distance - width*numpy.sin(bearing))
    float angularwidth = atan2(width*cos(bearing), 2*distance + width*sin(bearing)) + atan2(width*cos(bearing), 2*distance - width*sin(bearing));
    return angularwidth;
}

/*! @brief Returns true if the standard deviation on the robot's heading is less than the angular width of the goal for the current location
    @param goalpost either opponent goal post
    @param num_stddev the number of standard deviations to be less than the angular width (1 stddev = 68.2% and 2 stddev = 95.4%)
 */
bool Self::sdHeadingLessThanGoalWidth(const StationaryObject& goalpost, float num_stddev)
{
    float angularwidth = CalculateAngularWidthOfGoal(goalpost);
    if (angularwidth < WorldModelLocationError[2])
        return true;
    else
        return false;
}

/*! @brief Returns the [time, x,y] of the closest intercept point to a moving object. 
           Deacceleration is not taken into account, so the moving object may never reach the calculated point.
           In that case the time will be (very) large.
    @param theObject the moving mobile object
 */
std::vector<float> Self::CalculateClosestInterceptToMobileObject(const MobileObject& theObject)
{
    Vector2<float> velocity = theObject.getEstimatedVelocity();     // get velocity in world coords
    float heading = Heading();                                      // get heading in world coords
    
    // Convert the velocity to a relative one
    float velocity_mag = sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1]);
    float velocity_heading = atan2(velocity[1],velocity[0]);
    float v_x = velocity_mag*cos(velocity_heading - heading);
    float v_y = velocity_mag*sin(velocity_heading - heading);
    
    // Get the object position in relative coords
    float b_r = theObject.estimatedDistance()*cos(theObject.estimatedElevation());
    float b_b = theObject.estimatedBearing();
    float b_x = b_r*cos(b_b);
    float b_y = b_r*sin(b_b);
    
    // Now I am going to calculate the point on the velocity vector which is closest to me
    vector<float> intercept(3,0);
    intercept[1] = (b_x*v_y*v_y - b_y*v_x*v_y)/(v_x*v_x + v_y*v_y);                                       // intercept x
    intercept[2] = -(v_x/v_y)*intercept[1];                                                               // intercept y
    intercept[0] = sqrt(pow(b_x - intercept[1], 2) + pow(b_y - intercept[2], 2))/velocity_mag;            // intercept time (s)
    return intercept;
}

/*! @brief Returns the intercept point with the my own y-axis
    @param theObject the mobile object
 */
float Self::CalculateYInterceptOfMobileObject(const MobileObject& theObject)
{
    Vector2<float> velocity = theObject.getEstimatedVelocity();     // get velocity in world coords
    float heading = Heading();                                      // get heading in world coords
    
    // Convert the world coords velocity to a relative one
    float velocity_mag = sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1]);
    float velocity_heading = atan2(velocity[1],velocity[0]);
    float v_x = velocity_mag*cos(velocity_heading - heading);
    float v_y = velocity_mag*sin(velocity_heading - heading);
    
    // Get the ball position in relative coords
    float b_r = theObject.estimatedDistance()*cos(theObject.estimatedElevation());
    float b_b = theObject.estimatedBearing();
    float b_x = b_r*cos(b_b);
    float b_y = b_r*sin(b_b);
    
    // Now we can calculate the points where the ball will intersect our x and y axes
    float i_y = b_y - (v_y/v_x)*b_x;
    return i_y;
}

/*! @brief Returns the intercept point with my own x-axis
    @param theObject the mobile object
 */
float Self::CalculateXInterceptOfMobileObject(const MobileObject& theObject)
{
    Vector2<float> velocity = theObject.getEstimatedVelocity();     // get velocity in world coords
    float heading = Heading();                                      // get heading in world coords
    
    // Convert the world coords velocity to a relative one
    float velocity_mag = sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1]);
    float velocity_heading = atan2(velocity[1],velocity[0]);
    float v_x = velocity_mag*cos(velocity_heading - heading);
    float v_y = velocity_mag*sin(velocity_heading - heading);
    
    // Get the ball position in relative coords
    float b_r = theObject.estimatedDistance()*cos(theObject.estimatedElevation());
    float b_b = theObject.estimatedBearing();
    float b_x = b_r*cos(b_b);
    float b_y = b_r*sin(b_b);
    
    // Now we can calculate the points where the ball will intersect our x and y axes
    float i_x = b_x - (v_x/v_y)*b_y;
    return i_x;
}

/*! @brief Calculates a position between a mobile object and a goal at distancefromgoal from the goal
    @param mobileobject the object you are protecting the goal from
    @param goalpost either goal post
    @param distancefrommobile the desired distance from the mobile object
    @return [x, y] of the position relative to the current state
 */
std::vector<float> Self::CalculatePositionBetweenMobileObjectAndGoal(const MobileObject& mobileobject, const StationaryObject& goalpost, float distancefrommobile)
{
    // get the relative x,y of the mobile object
    float b_r = mobileobject.estimatedDistance()*cos(mobileobject.estimatedElevation());          // get the flat distance!
    float b_b = mobileobject.estimatedBearing();
    float b_x = b_r*cos(b_b);
    float b_y = b_r*sin(b_b);
    
    // get the relative x,y of the goal
    vector<float> goallocation = CalculateDifferenceFromGoal(goalpost);
    float g_x = goallocation[0]*cos(goallocation[1]);
    float g_y = goallocation[0]*sin(goallocation[1]);
    
    // get the relative x,y of the calculated position
    float diffX = b_x - g_x;
    float diffY = b_y - g_y;
    if (diffX == 0) diffX = 0.001;
    float p_x = b_x - distancefrommobile*cos(atan2(diffY, diffX));
    float p_y = b_y + (diffY/diffX)*(p_x - b_x);
    
    vector<float> result(2,0);
    result[0] = p_x;
    result[1] = p_y;
    return result;
}

std::vector<float> Self::CalculatePositionToProtectGoalFromMobileObject(const MobileObject& mobileobject, const StationaryObject& goalpost, float blockingwidth)
{
    float goal_angular_width = fabs(CalculateAngularWidthOfGoalFromMobileObject(goalpost, mobileobject));
    float goal_width = 2*fabs(goalpost.Y());
    
    float distancebetween = sqrt(pow(mobileobject.X() - goalpost.X(), 2) + pow(mobileobject.Y(),2));
    float distancefrommobile = (0.5*blockingwidth)/tan(0.5*goal_angular_width);
    
    vector<float> position(2,0);
    if (distancebetween < 0.7*goal_width)
    {   // if the mobile object is inside the goal then the calculation will break --- just go to the mobile object
        float b_r = mobileobject.estimatedDistance()*cos(mobileobject.estimatedElevation());          // get the flat distance!
        float b_b = mobileobject.estimatedBearing();
        position[0] = b_r*cos(b_b);
        position[1] = b_r*sin(b_b);
        return position;
    }
    
    // clip the distancefrommobile so that we don't go back into the goal.
    if (distancebetween - distancefrommobile < 0.7*goal_width)
        distancefrommobile = distancebetween - 0.7*goal_width;
    
    return CalculatePositionBetweenMobileObjectAndGoal(mobileobject, goalpost, distancefrommobile);
    
}

std::ostream& operator<< (std::ostream& output, const Self& p_self)
{
    output << p_self.WorldModelLocation.x << ' ' << p_self.WorldModelLocation.y << ' ' << p_self.WorldModelLocation.z << ' ';
    output << p_self.WorldModelLocationError.x << ' ' << p_self.WorldModelLocationError.y << ' ' << p_self.WorldModelLocationError.z << ' ';
    return output;
}

std::istream& operator>> (std::istream& input, Self& p_self)
{
    input >> p_self.WorldModelLocation.x >> p_self.WorldModelLocation.y >> p_self.WorldModelLocation.z;
    input >> p_self.WorldModelLocationError.x >> p_self.WorldModelLocationError.y >> p_self.WorldModelLocationError.z;
    return input;
}
