#include "centrecircle.h"

CentreCircle::CentreCircle()
{
    m_location = Point(0,0);
    m_size_on_screen = Vector2<double>(0,0),
    m_ground_radius = 0;
    //need more here
}

CentreCircle::CentreCircle(Point centre, double ground_radius, Vector2<double> screen_size)
{
    m_location = centre;
    m_size_on_screen = screen_size,
    m_ground_radius =ground_radius;
    //need more here
}

CentreCircle::~CentreCircle()
{
}

bool CentreCircle::addToExternalFieldObjects(FieldObjects* fieldobjects, float timestamp) const
{

}

//! @brief Stream output for labelling purposes
void CentreCircle::printLabel(ostream& out) const
{
    out << getShortLabel();
}

//! @brief Brief stream output for labelling purposes
Vector2<double> CentreCircle::getShortLabel() const
{
    return m_location.screen;
}

//! @brief Calculation of error for optimisation
double CentreCircle::findError(const Vector2<double>& measured) const
{
    return (m_location.screen - measured).abs();
}
