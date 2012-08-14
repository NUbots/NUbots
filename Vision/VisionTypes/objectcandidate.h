#ifndef OBJECTCANDIDATE_H
#define OBJECTCANDIDATE_H

#include <vector>

#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTypes/coloursegment.h"

using namespace std;

class ObjectCandidate
{
public:
    void setTopLeft(const PointType& point);
    void setBottomRight(const PointType& point);
    void setColour(ClassIndex::Colour c);
    
    
    const PointType& getTopLeft() const;
    const PointType& getBottomRight() const;
    const PointType& getCentre() const;
    int getWidth() const;
    int getHeight() const;
    float getAspect() const;
    ClassIndex::Colour getColour()  const;
    const vector<ColourSegment>& getSegments() const;
    void addColourSegments(const vector<ColourSegment>& new_segments);
    void addColourSegment(const ColourSegment& new_segment);

    ObjectCandidate();
    ObjectCandidate(const PointType& top_left, const PointType& bottom_right);
    ObjectCandidate(const PointType& top_left, const PointType& bottom_right, ClassIndex::Colour colour);
    ObjectCandidate(const PointType& top_left, const PointType& bottom_right, ClassIndex::Colour colour, const vector<ColourSegment>& candidate_segments);
    ~ObjectCandidate();


protected:
    void recalculate();
    
    PointType topLeft;
    PointType bottomRight;
    PointType centre;
    int width;
    int height;
    float aspect;
    vector<ColourSegment> segments;
    ClassIndex::Colour colour;


};

#endif // OBJECTCANDIDATE_H
