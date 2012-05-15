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

