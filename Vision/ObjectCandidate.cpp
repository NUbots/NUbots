#include "ObjectCandidate.h"

 /*
        unclassified,   //!< Colour has not be given a category.
        white,          //!< Colour is in the White region.
        green,          //!< Colour is in the Green region.
        shadow_object,  //!< Colour is part of a shadowed area.
        red,            //!< Colour is in the Red region.
        red_orange,     //!< Colour is in the region of overlap between Red and Orange.
        orange,         //!< Colour is in the Orange region.
        yellow_orange,  //!< Colour is in the region of overlap between Yellow and Orange.
        yellow,         //!< Colour is in the Yellow region.
        blue,           //!< Colour is in the Sky Blue region.
        shadow_blue,    //!< Colour is in the Dark Blue region.
    //*/

ObjectCandidate::ObjectCandidate()
{
    topLeft.x = 0;
    topLeft.y = 0;
    bottomRight.x = 0;
    bottomRight.y = 0;
    colour = 3;
}

ObjectCandidate::ObjectCandidate(int left, int top, int right, int bottom)
{
    topLeft.x = left;
    topLeft.y = top;
    bottomRight.x = right;
    bottomRight.y = bottom;
    colour = 3;
}

ObjectCandidate::ObjectCandidate(int left, int top, int right, int bottom, unsigned char colour): colour(colour)
{
    topLeft.x = left;
    topLeft.y = top;
    bottomRight.x = right;
    bottomRight.y = bottom;
}

ObjectCandidate::ObjectCandidate(int left, int top, int right, int bottom, unsigned char colour, std::vector<TransitionSegment> candidate_segments): colour(colour)
{
    topLeft.x = left;
    topLeft.y = top;
    bottomRight.x = right;
    bottomRight.y = bottom;
    segments = candidate_segments;
}//*/

std::vector<TransitionSegment> ObjectCandidate::getSegments() const
{
    return segments;
}


ObjectCandidate::~ObjectCandidate()
{
    return;
}

Vector2<int> ObjectCandidate::getTopLeft() const
{
    return topLeft;
}
Vector2<int> ObjectCandidate::getBottomRight() const
{
    return bottomRight;
}

int ObjectCandidate::width()
{
    return (bottomRight.x - topLeft.x);
}
int ObjectCandidate::height()
{
    return (bottomRight.y - topLeft.y);
}
float ObjectCandidate::aspect()
{
    return (float)(bottomRight.x - topLeft.x) / (float)(bottomRight.y - topLeft.y);
}

unsigned char ObjectCandidate::getColour() const
{
    return colour;
}

void ObjectCandidate::setColour(unsigned char c)
{
    colour = c;
}
void ObjectCandidate::setTopLeft(Vector2<int> point)
{
    topLeft.x = point.x;
    topLeft.y = point.y;
}

void ObjectCandidate::setBottomRight(Vector2<int> point)
{
    bottomRight.x = point.x;
    bottomRight.y = point.y;
}
int ObjectCandidate::getCentreX()
{
    return (int)(round((bottomRight.x + topLeft.x)/2));
}
int ObjectCandidate::getCentreY()
{
    return (int)(round((bottomRight.y + topLeft.y)/2));
}
