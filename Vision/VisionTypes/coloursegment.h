/*!
  * @file coloursegment.h
  * @class ColourSegment
  * @author Shannon Fenn
  * @date 23-03-12
  *
  * @brief Class to hold segment data for matching and classification.
  *
  */

#ifndef COLOURSEGMENT_H
#define COLOURSEGMENT_H

#include <vector>
#include <iostream>

#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTools/classificationcolours.h"

using namespace std;

class ColourSegment
{
public:
    ColourSegment() {set(PointType(0,0), PointType(0,0), ClassIndex::invalid);}
    ColourSegment(const PointType& start, const PointType& end, const ClassIndex::Colour& colour) {set(start, end, colour);}

    unsigned int getLengthPixels() const {return m_length_pixels;}
    ClassIndex::Colour getColour() const {return m_colour;}
    const PointType& getStart() const {return m_start;}
    const PointType& getEnd() const {return m_end;}

    void set(const PointType& start, const PointType& end, ClassIndex::Colour colour);
    void setColour(ClassIndex::Colour colour);
    bool join(const ColourSegment& other);

    friend ostream& operator<< (ostream& output, const ColourSegment& c);
    friend ostream& operator<< (ostream& output, const vector<ColourSegment>& c);
    
    //ColourSegment& operator= (const ColourSegment& rhs);
    

private:
    ClassIndex::Colour m_colour;
    unsigned int m_length_pixels;
    PointType m_start, m_end;
};

#endif // COLOURSEGMENT_H
