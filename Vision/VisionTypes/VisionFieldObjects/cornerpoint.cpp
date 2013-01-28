#include "cornerpoint.h"

CornerPoint::CornerPoint(TYPE type, Point screen_location, Vector2<float> relative_location)
{
    m_type = type;
    m_location_pixels = screen_location;
    m_location_angular = relative_location;
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
    return m_location_pixels;
}

//! @brief Calculation of error for optimisation
double CornerPoint::findError(const Vector2<double>& measured) const
{
    return (m_location_pixels - measured).abs();
}

void CornerPoint::render(cv::Mat& mat) const
{
    cv::circle(mat, cv::Point(m_location_pixels.x, m_location_pixels.y), 3, cv::Scalar(255,255,0));
}
