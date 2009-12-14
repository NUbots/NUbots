#include "TransitionSegment.h"
#include <math.h>

TransitionSegment::TransitionSegment()
{
   beforeColour = 0;
   colour = 0;
   afterColour =  0;
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
}
Vector2<int> TransitionSegment::getEndPoint()
{
    return end;
}
Vector2<int> TransitionSegment::getStartPoint()
{
    return start;
}
int TransitionSegment::getSize()
{
    return (int)sqrt((end.x-start.x)^2 + (end.y-start.y)^2);
}
unsigned char TransitionSegment::getBeforeColour()
{
    return beforeColour;
}
unsigned char TransitionSegment::getColour()
{
    return colour;
}
unsigned char TransitionSegment::getAfterColour()
{
    return afterColour;
}
