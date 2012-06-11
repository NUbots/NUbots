#ifndef TRANSITION_H
#define TRANSITION_H

#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTools/classificationcolours.h"
#include "Vision/VisionTypes/coloursegment.h"

using namespace VisionID;

class Transition
{
public:
    //static PointType correctDistortion(const PointType& pt);
    
    Transition();
    Transition(const PointType& location, ClassIndex::Colour before, ClassIndex::Colour after, ScanDirection& direction);
    Transition(ColourSegment before, ColourSegment after, ScanDirection direction);
    
    void set(const PointType& location, ClassIndex::Colour before, ClassIndex::Colour after, ScanDirection& direction);
    void set(const ColourSegment& before, const ColourSegment& after, ScanDirection direction);
    
    const PointType& getLocation() const;
    ClassIndex::Colour getBefore() const;
    ClassIndex::Colour getAfter() const;
    ScanDirection getDirection() const;
    
    bool operator< (const Transition& rhs) const;
    
    friend ostream& operator<< (ostream& output, const Transition& c);
    friend ostream& operator<< (ostream& output, const vector<Transition>& c);
    
private:
    PointType m_location;
    ClassIndex::Colour m_before_colour, m_after_colour;
    ScanDirection m_direction;
    
    
};

#endif // TRANSITION_H
