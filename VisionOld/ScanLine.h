#ifndef SCANLINE_H
#define SCANLINE_H

#include <vector>
#include "TransitionSegment.h"

class ScanLine
{
    public:
        enum ScanDirection
        {
            UP,
            DOWN,
            LEFT,
            RIGHT,
            num_directions,
        };
        ScanLine();
        ~ScanLine();
        ScanLine(Vector2<int> newStartPoint, int length);
        ScanLine(Vector2<int> newStartPoint, int newLength, int newDirection);
        int getLength();
        int getDirection();
        void setDirection(int newDirection);
        int getNumberOfSegments();
        void addSegement(const TransitionSegment& segment);
        TransitionSegment* getSegment(int position);
        float getFill();
        float getFill(Vector2<int> start, Vector2<int> end);
        float getFill(float start, float end);
        Vector2<int> getStart();
    private:
        std::vector<TransitionSegment> segments;
        Vector2<int> start;
        int length;
        int direction;

};
#endif // SCANLINE_H
