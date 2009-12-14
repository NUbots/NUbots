#include "ScanLine.h"

ScanLine::ScanLine()
{
    //segments = new std::vector<TransitionSegment>;
    //Vector2<int> start;
    length = 0;
}

ScanLine::~ScanLine()
{
    return;
}

ScanLine::ScanLine(Vector2<int> newStartPoint, int newLength)
{
    //std::vector<TransitionSegment>;
    length = newLength;
}

int ScanLine::getLength()
{
    return length;
}

int  ScanLine::getNumberOfSegments()
{
    return segments.size();
}

void ScanLine::addSegement(TransitionSegment* segment)
{
    segments.push_back(*segment);
    return;
}

TransitionSegment* ScanLine::getSegment(int position)
{
    return &(segments[position]);
}

