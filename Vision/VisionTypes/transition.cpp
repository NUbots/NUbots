#include "transition.h"
#include "debug.h"
#include "debugverbosityvision.h"

Transition::Transition()
{
}

Transition::Transition(PointType &location, ClassIndex::Colour before_colour, ClassIndex::Colour after_colour, ScanDirection &direction)
{
    set(location, before_colour, after_colour, direction);
}

Transition::Transition(ColourSegment before, ColourSegment after, ScanDirection direction)
{
    set(before, after, direction);
}

void Transition::set(const PointType &location, ClassIndex::Colour before_colour, ClassIndex::Colour after_colour, ScanDirection &direction)
{
    m_location = location;
    m_before_colour = before_colour;
    m_after_colour = after_colour;
    m_direction = direction;
}

void Transition::set(const ColourSegment& before, const ColourSegment& after, ScanDirection direction)
{
    
    #if VISION_SCAN_VERBOSITY > 0
        if(before.getEnd() != after.getStart())
            debug << "Transition::set(): segments not adjacent" << endl;
    #endif
    
    m_location = before.getEnd();
    m_before_colour = before.getColour();
    m_after_colour = after.getColour();
    m_direction = direction;
}

/*! @brief Returns the transition pixel location.
*
*/
const PointType& Transition::getLocation() const
{
    return m_location;
}

/*! @brief Returns the colour of the before segment.
*
*/
ClassIndex::Colour Transition::getBefore() const
{
    return m_before_colour;
}

/*! @brief Returns the colour of the after segment.
*
*/
ClassIndex::Colour Transition::getAfter() const
{
    return m_after_colour;
}

/*! @brief Returns the direction of the scan.
*
*/
ScanDirection Transition::getDirection() const
{
    return m_direction;
}

/*! @brief Less than operator for a pair of Transitions.
 *  @relates Transition
 *  Orders geometrically based on x then y.
 */
bool Transition::operator< (const Transition& rhs) const
{
    return (m_location.x < rhs.m_location.x || (m_location.x == rhs.m_location.x && m_location.y < rhs.m_location.y));
}

/*! @brief Stream insertion operator for a single Transition
 *  @relates Transition
 */
ostream& operator<< (ostream& output, const Transition& t)
{
    output << t.m_location << "[";
    switch(t.m_direction) {
    case VERTICAL:
        output << "DOWN] ";
        break;
    case HORIZONTAL:
        output << "RIGHT] ";
        break;
    }

    output << ClassIndex::getColourName(t.m_before_colour) << " -> " << ClassIndex::getColourName(t.m_after_colour) << endl;

    return output;
}

/*! @brief Stream insertion operator for a vector of Transitions.
 *  @relates Transition
 */
ostream& operator<< (ostream& output, const vector<Transition>& c)
{
    for (size_t i=0; i<c.size(); i++)
        output << c[i];
    return output;
}

