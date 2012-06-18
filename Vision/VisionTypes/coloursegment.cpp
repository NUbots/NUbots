#include "coloursegment.h"

unsigned int dist(const PointType &x1, const PointType &x2)
{
    if(x1.x == x2.x)
        return std::abs(x1.y - x2.y);
    else if(x1.y == x2.y)
        return std::abs(x1.x - x2.x);
    else
        return (x1 - x2).abs();
}

void ColourSegment::set(const PointType &start, const PointType &end, ClassIndex::Colour colour)
{
    m_colour = colour;
    m_start = start;
    m_end = end;
    m_length_pixels = dist(start, end);
}

void ColourSegment::setColour(ClassIndex::Colour colour)
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
    m_length_pixels = dist(m_start, m_end);

    return true;
}

/*! @brief Stream insertion operator for a single ColourSegment.
 *      The segment is terminated by a newline.
 */
ostream& operator<< (ostream& output, const ColourSegment& c)
{
    output << c.m_start << " - " << c.m_end << " length(pixels): " << c.m_length_pixels << " colour: " << ClassIndex::getColourName(c.m_colour) << endl;
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

///*! @brief Assignment operator for a ColourSegment.
// */
//ColourSegment& ColourSegment::operator= (const ColourSegment& rhs)
//{    
//    // Check for self-assignment!
//    if (this != &rhs) {
//        m_colour = rhs.m_colour;
//        m_start = this->m_start;
//        m_end = this->m_end;
//        m_length_pixels = this->m_length_pixels;
//    }
//    return *this;
//}
