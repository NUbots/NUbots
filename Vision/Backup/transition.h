#ifndef TRANSITION_H
#define TRANSITION_H

#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTools/classificationcolours.h"
#include "Vision/VisionTypes/coloursegment.h"

using namespace VisionID;

class Transition
{
public:
    
    Transition();
    Transition(const PointType& location, ClassIndex::Colour before, ClassIndex::Colour after, ScanDirection& direction);
    Transition(ColourSegment before, ColourSegment after, ScanDirection direction);

    /**
      * Sets the transition values.
      * @param location The pixel location of the transition.
      * @param before The colour of the segment before the transition.
      * @param after The colour of the segment after the transition.
      * @param direction The alignment of the producing segments (vertical/horizontal).
      */
    void set(const PointType& location, ClassIndex::Colour before, ClassIndex::Colour after, ScanDirection& direction);
    /**
      * Sets the transition values from supplied segments.
      * @param before The segment before the transition.
      * @param after The segment after the transition.
      * @param direction The alignment of the producing segments (vertical/horizontal).
      */
    void set(const ColourSegment& before, const ColourSegment& after, ScanDirection direction);
    
    const PointType& getLocation() const;   //! Returns the location of the transition in pixel coordinates.
//    ClassIndex::Colour getBefore() const;   //! Returns the colour of the segment before the transition.
//    ClassIndex::Colour getAfter() const;    //! Returns the colour of the segment after the transition.
//    ScanDirection getDirection() const;     //! Returns the alignment of the segments that produce the transition.
    
    bool operator< (const Transition& rhs) const;   //! GT operator for a pair of transitions.
    
    friend ostream& operator<< (ostream& output, const Transition& c);          //! output stream operator
    friend ostream& operator<< (ostream& output, const vector<Transition>& c);  //! output stream operator for a vector of transitions
    
private:
    PointType m_location;   //! @variable The pixel location of the transition.
    ClassIndex::Colour m_before_colour, //! @variable The colour of the segment before the transition.
                       m_after_colour;  //! @variable The colour of the segment after the transition.
    ScanDirection m_direction;  //! @variable The alignment of the segments that produced this transition.
    
    
};

#endif // TRANSITION_H
