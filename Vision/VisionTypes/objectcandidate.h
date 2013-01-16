#ifndef OBJECTCANDIDATE_H
#define OBJECTCANDIDATE_H

#include <vector>

#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTypes/coloursegment.h"

using namespace std;

class ObjectCandidate
{
public:
    void setTopLeft(const Vector2<double>& point);
    void setBottomRight(const Vector2<double>& point);
    void setColour(ClassIndex::Colour c);
    
    
    const Vector2<double>& getTopLeft() const;
    const Vector2<double>& getBottomRight() const;
    const Vector2<double>& getCentre() const;
    int getWidth() const;
    int getHeight() const;
    float getAspect() const;
    ClassIndex::Colour getColour()  const;
    const vector<ColourSegment>& getSegments() const;
    void addColourSegments(const vector<ColourSegment>& new_segments);
    void addColourSegment(const ColourSegment& new_segment);

    ObjectCandidate();
    ObjectCandidate(const Vector2<double>& top_left, const Vector2<double>& bottom_right);
    ObjectCandidate(const Vector2<double>& top_left, const Vector2<double>& bottom_right, ClassIndex::Colour colour);
    ObjectCandidate(const Vector2<double>& top_left, const Vector2<double>& bottom_right, ClassIndex::Colour colour, const vector<ColourSegment>& candidate_segments);
    ~ObjectCandidate();


protected:
    void recalculate();
    
    Vector2<double> topLeft;
    Vector2<double> bottomRight;
    Vector2<double> centre;
    int width;
    int height;
    float aspect;
    vector<ColourSegment> segments;
    ClassIndex::Colour colour;


};

#endif // OBJECTCANDIDATE_H
