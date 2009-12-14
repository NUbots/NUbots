#ifndef SCANLINE_H
#define SCANLINE_H

#include <vector>
#include "TransitionSegment.h"

class ScanLine
{
    public:
        ScanLine();
        ~ScanLine();
        ScanLine(Vector2<int> newStartPoint, int length);
        int getLength();
        int getNumberOfSegments();
        void addSegement(TransitionSegment* segment);
        TransitionSegment* getSegment(int position);
    private:
        std::vector<TransitionSegment> segments;
        Vector2<int> start;
        int length;
};
#endif // SCANLINE_H
