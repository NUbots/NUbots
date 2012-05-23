#include "ball.h"

Ball::Ball()
{
    m_radius = 0;
    m_location_pixels.x = 0;
    m_location_pixels.y = 0;
    calculatePositions();
}

Ball::Ball(const PointType& centre, int radius)
{
    m_radius = radius;
    m_location_pixels.x = centre.x;
    m_location_pixels.y = centre.y;
    calculatePositions();
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
    output << "Ball - pixelloc: [" << b.getLocationPixels().x << ", " << b.getLocationPixels().y << "]";
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
