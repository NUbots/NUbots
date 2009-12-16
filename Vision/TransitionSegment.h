#ifndef TRANSISTIONSEGMENT_H
#define TRANSISTIONSEGMENT_H


#include "Tools/Math/Vector2.h"

class TransitionSegment
{
    public:
        TransitionSegment();
        ~TransitionSegment();
        TransitionSegment(Vector2<int> newStart, Vector2<int> newEnd, unsigned char newBeforeColour, unsigned char newColour, unsigned char newAfterColour);
        Vector2<int> getEndPoint() const;
        Vector2<int> getStartPoint() const;
        int getSize() const;
        unsigned char getBeforeColour() const;
        unsigned char getColour() const;
        unsigned char getAfterColour() const;

    private:
        Vector2<int> start;
        Vector2<int> end;
        unsigned char beforeColour;
        unsigned char colour;
        unsigned char afterColour;
};

#endif // TRANSISTIONSEGMENT_H
