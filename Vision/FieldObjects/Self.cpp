#include "Self.h"
#include "StationaryObject.h"
#include "MobileObject.h"
#include "Tools/Math/General.h"
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
void Self::updateLocationOfSelf(float wmX, float wmY, float heading, float sdX, float sdY, float sdHeading)
{
	WorldModelLocation[0] = wmX;
    WorldModelLocation[1] = wmY;
    WorldModelLocation[2] = heading;
    
    WorldModelLocationError[0] = sdX;
    WorldModelLocationError[1] = sdY;
    WorldModelLocationError[2] = sdHeading;
}

bool Self::lost()
{
    if (WorldModelLocationError[2]*2 > 1.5708)      // if heading is really uncertain we are lost
        return true;
    else if (WorldModelLocationError[0]*2 > 3.0 or WorldModelLocationError[1]*2 > 2.0)
        return true;
    else
        return false;
}

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

float Self::CalculateDistanceToStationaryObject(const StationaryObject& theObject)
{
    float selfX = WorldModelLocation[0];
    float selfY = WorldModelLocation[1];
    float diffX = theObject.X() - selfX;
    float diffY = theObject.Y() - selfY;
    float distance = sqrt( diffX * diffX + diffY * diffY );
    return distance;
}

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
    float b_r = theObject.estimatedDistance();
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
    float b_r = theObject.estimatedDistance();
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
    float b_r = theObject.estimatedDistance();
    float b_b = theObject.estimatedBearing();
    float b_x = b_r*cos(b_b);
    float b_y = b_r*sin(b_b);
    
    // Now we can calculate the points where the ball will intersect our x and y axes
    float i_x = b_x - (v_x/v_y)*b_y;
    return i_x;
}
