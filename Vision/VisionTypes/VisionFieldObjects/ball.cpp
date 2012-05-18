#include "ball.h"

Ball::Ball()
{
    Ball(0);
}

Ball::Ball(int radius)
{
    m_radius = radius;
}


Vector3<float> Ball::getRelativeFieldCoords() const
{
    return m_spherical_position;
}

bool Ball::addToExternalFieldObjects(FieldObjects *fieldobjects, float timestamp) const
{
    return false;
}

void Ball::calculatePositions()
{
    //! @todo implement 
    m_spherical_position[0] = 0; //dist
    m_spherical_position[1] = 0; //bearing
    m_spherical_position[2] = 0; //elevation
    
    m_location_angular.x = 0; //bearing
    m_location_angular.y = 0; //elevation
}

/*! @brief Stream insertion operator for a single ColourSegment.
 *      The segment is terminated by a newline.
 */
ostream& operator<< (ostream& output, const Ball& b)
{
    output << "Ball - pixelloc:" << b.getLocationPixels();
    output << " angularloc: [" << b.getLocationAngular().x << ", " << b.getLocationAngular().y << "]";
    output << " relative field coords: [" << b.getRelativeFieldCoords().x << ", " << b.getRelativeFieldCoords().y << ", " << b.getRelativeFieldCoords().z << "]";
    return output;
}

/*! @brief Stream insertion operator for a vector of ColourSegments.
 *      Each segment is terminated by a newline.
 *  @relates ColourSegment
 */
ostream& operator<< (ostream& output, const vector<Ball>& b)
{
    for (size_t i=0; i<b.size(); i++)
        output << b[i] << endl;
    return output;
}
