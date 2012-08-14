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

    //! Returns the length of the segment in pixels.
    unsigned int getLengthPixels() const {return m_length_pixels;}
    //! Returns the colour of the segment.
    ClassIndex::Colour getColour() const {return m_colour;}
    //! Returns the start location of the segment in pixel coordinates.
    const PointType& getStart() const {return m_start;}
    //! Returns the end location of the segment in pixel coordinates.
    const PointType& getEnd() const {return m_end;}
    //! Returns the end location of the segment in pixel coordinates.
    const PointType& getCentre() const {return (m_start + m_end)*0.5;}


    /**
      * Sets the parameters for the segment.
      * @param start The start location of the segment.
      * @param end The end location of the segment.
      * @param colour The colour of the segment.
      */
    void set(const PointType& start, const PointType& end, ClassIndex::Colour colour);
    //! Set the colour of the segment.
    void setColour(ClassIndex::Colour colour);
    /**
      * Joins the given segment to this one. This segment will now be as long as the total
      * length of the two provided the following are true:
      *     - The start of this segment matches the end of the provided segment, or vice versa.
      *     - The colours of the two segments are the same.
      * @param other The segment to join onto this one.
      * @return Whether the segments were joined - will be false if the conditions are not met.
      */
    bool join(const ColourSegment& other);

    //! output stream operator.
    friend ostream& operator<< (ostream& output, const ColourSegment& c);
    //! output stream operator for a vector of segments.
    friend ostream& operator<< (ostream& output, const vector<ColourSegment>& c);
    
    //ColourSegment& operator= (const ColourSegment& rhs);
    

private:
    ClassIndex::Colour m_colour;    //! @variable The colour of the segment.
    unsigned int m_length_pixels;   //! @variable The length of the segment in pixels.
    PointType m_start,              //! @variable The start screen location.
              m_end;                //! @variable The end  screenlocation.
};

#endif // COLOURSEGMENT_H
