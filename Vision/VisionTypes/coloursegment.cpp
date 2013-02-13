#include "coloursegment.h"

void ColourSegment::set(const Point& start, const Point& end, Colour colour)
{
    m_colour = colour;
    bool flip;

    //check if the points are in correct order
    if(start.x < end.x)
        flip = false;
    else if(start.x > end.x)
        flip = true;
    else
        flip = start.y > end.y;

    //swap them if not
    if(flip) {
        m_start = end;
        m_end = start;
    }
    else {
        m_start = start;
        m_end = end;
    }
    m_length_pixels = (start - end).abs();
    m_centre.x = 0.5*(m_start.x + m_end.x);
    m_centre.y = 0.5*(m_start.y + m_end.y);

}

void ColourSegment::setColour(Colour colour)
{
    m_colour = colour;
}

bool ColourSegment::join(const ColourSegment &other)
{
    if(m_colour != other.m_colour)
        return false;   //colours don't match - segments cannot be joined
    if(m_start == other.m_end) {
        m_start = other.m_start;
    }
    else if(m_end == other.m_start) {
        m_end = other.m_end;
    }
    else {
        return false;   //there are no matching endpoints
    }
    m_length_pixels = (m_start - m_end).abs();
    m_centre.x = 0.5*(m_start.x + m_end.x);

    return true;
}

/*! @brief Stream insertion operator for a single ColourSegment.
 *      The segment is terminated by a newline.
 */
ostream& operator<< (ostream& output, const ColourSegment& c)
{
    output << c.m_start << " - " << c.m_end << " length(pixels): " << c.m_length_pixels << " colour: " << getColourName(c.m_colour) << std::endl;
    return output;
}

/*! @brief Stream insertion operator for a vector of ColourSegments.
 *      Each segment is terminated by a newline.
 *  @relates ColourSegment
 */
ostream& operator<< (ostream& output, const vector<ColourSegment>& c)
{
    for (size_t i=0; i<c.size(); i++)
        output << c[i];
    return output;
}
