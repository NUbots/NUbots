#include "centrecircle.h"

CentreCircle::CentreCircle(Circle ground_equation)
{
    m_ground_circle = ground_equation;

    //need more here
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
    return m_location_pixels;
}

//! @brief Calculation of error for optimisation
double CentreCircle::findError(const Vector2<double>& measured) const
{
    return (m_location_pixels - measured).abs();
}
