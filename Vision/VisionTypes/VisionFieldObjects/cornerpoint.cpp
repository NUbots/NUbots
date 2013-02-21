#include "cornerpoint.h"

CornerPoint::CornerPoint(TYPE type, GroundPoint location)
{
    m_type = type;
    m_location = location;
}

bool CornerPoint::addToExternalFieldObjects(FieldObjects* fieldobjects, float timestamp) const
{

}

//! @brief Stream output for labelling purposes
void CornerPoint::printLabel(ostream& out) const
{
    out << getShortLabel();
}

//! @brief Brief stream output for labelling purposes
Vector2<double> CornerPoint::getShortLabel() const
{
    return m_location.screen;
}

//! @brief Calculation of error for optimisation
double CornerPoint::findError(const Vector2<double>& measured) const
{
    return (m_location.screen - measured).abs();
}
