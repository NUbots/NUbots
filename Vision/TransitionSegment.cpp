#include "TransitionSegment.h"
#include <math.h>

TransitionSegment::TransitionSegment()
{
   beforeColour = 0;
   colour = 0;
   afterColour =  0;
   isUsed = false;
}

TransitionSegment::~TransitionSegment()
{
    return;
}
TransitionSegment::TransitionSegment(Vector2<int> newStart, Vector2<int> newEnd, unsigned char newBeforeColour, unsigned char newColour, unsigned char newAfterColour)
{
    start = newStart;
    end = newEnd;
    beforeColour = newBeforeColour;
    colour = newColour;
    afterColour = newAfterColour;
    isUsed = false;
}
Vector2<int> TransitionSegment::getEndPoint() const
{
    return end;
}
Vector2<int> TransitionSegment::getMidPoint() const\
{
    Vector2<int> position;
    position.x = (int)(start.x+end.x)/2;
    position.y = (int)(start.y+end.y)/2;
    return position;
}
Vector2<int> TransitionSegment::getStartPoint() const
{
    return start;
}
int TransitionSegment::getSize() const
{
    return (int)(round(sqrt((end.x-start.x)*(end.x-start.x) + (end.y-start.y)*(end.y-start.y))));
}
unsigned char TransitionSegment::getBeforeColour() const
{
    return beforeColour;
}
unsigned char TransitionSegment::getColour() const
{
    return colour;
}
unsigned char TransitionSegment::getAfterColour() const
{
    return afterColour;
}
