#include "ScanLine.h"
#include "ClassificationColours.h"

ScanLine::ScanLine()
{
    //segments = new std::vector<TransitionSegment>;
    //Vector2<int> start;
    length = 0;
    direction = 0;
}

ScanLine::~ScanLine()
{
    return;
}

ScanLine::ScanLine(Vector2<int> newStartPoint, int newLength)
{
    //std::vector<TransitionSegment>;
    start = newStartPoint;
    length = newLength;
    direction = 0;
}

ScanLine::ScanLine(Vector2<int> newStartPoint, int newLength, int newDirection)
{
    //std::vector<TransitionSegment>;
    start = newStartPoint;
    length = newLength;
    direction = newDirection;
}

int ScanLine::getLength()
{
    return length;
}

int ScanLine::getDirection()
{
    return direction;
}

void ScanLine::setDirection(int newDirection)
{
    direction = newDirection;
}

int  ScanLine::getNumberOfSegments()
{
    return segments.size();
}

void ScanLine::addSegement(const TransitionSegment& segment)
{
    segments.push_back(segment);
    return;
}

TransitionSegment* ScanLine::getSegment(int position)
{
    return &(segments[position]);
}
Vector2<int> ScanLine::getStart()
{
    return start;
}

float ScanLine::getFill()
{
    //get the fill for the complete line and no subsection
    return getFill(0,1);
}

float ScanLine::getFill(Vector2<int> start, Vector2<int> end)
{

    if (length > 0)
        switch (direction)
        {
            case ScanLine::UP:
                //check the 'start' and 'end' points are in the right order and on the line
                if (start.x == end.x && start.y >= end.y && start.x == this->start.x)
                    return getFill( (float)(this->start.y - start.y)/(float)length,
                             (float)(this->start.y -   end.y)/(float)length);
                else
                    return 0;
            break;
            case ScanLine::DOWN:
                //check the 'start' and 'end' points are in the right order and on the line
                if (start.x == end.x && start.y <= end.y && start.x == this->start.x)
                    return getFill( (float)(start.y - this->start.y)/(float)length,
                             (float)(  end.y - this->start.y)/(float)length);
                else
                    return 0;
            break;
            case ScanLine::LEFT:
                //check the 'start' and 'end' points are in the right order and on the line
                if (start.x >= end.x && start.y == end.y && start.y == this->start.y)
                    return getFill( (float)(this->start.x - start.x)/(float)length,
                             (float)(this->start.x -   end.x)/(float)length);
                else
                    return 0;
            break;
            case ScanLine::RIGHT:
                //check the 'start' and 'end' points are in the right order and on the line
                if (start.x <= end.x && start.y == end.y && start.y == this->start.y)
                    return getFill( (float)(start.x - this->start.x)/(float)length,
                             (float)(  end.x - this->start.x)/(float)length);
                else
                    return 0;
            break;
            default:
                //No direction = No fill
                return 0;
            break;
        }
     else
         return 0;
}

float ScanLine::getFill(float start, float end)
{
    int fillCount = 0;
    int x, y;
    int sradius, eradius;
    int sbound = (int)(start*length);
    int ebound = (int)(  end*length);
    if ((int)segments.size() <= 0 && length <= 0) return 0;

    std::vector<TransitionSegment>::iterator it;
    for (it = segments.begin(); it != segments.end(); it++)
    {
        if ( (*it).getColour() == ClassIndex::unclassified )
        {
            continue;
        }

        x = (*it).getStartPoint().x;
        y = (*it).getStartPoint().y;
        sradius = (int)sqrt( (x-this->start.x)*(x-this->start.x) + (y-this->start.y)*(y-this->start.y));
        x = (*it).getEndPoint().x;
        y = (*it).getEndPoint().y;
        eradius = (int)sqrt( (x-this->start.x)*(x-this->start.x) + (y-this->start.y)*(y-this->start.y));

        //if the whole segment is within bounds
        if ( sradius >= sbound && sradius <= ebound &&
             eradius >= sbound && eradius <= ebound )
        {
            fillCount += (*it).getSize();
        }
        //if the start is not within bounds but the tail is inside
        else if(sradius <  start*length &&
                eradius >= start*length && eradius <= end*length)
        {
            fillCount += eradius - sbound;
        }
        //if the start is inside the bounds but the tail hangs out
        else if (sradius >= sbound && sradius <= ebound &&
                                      eradius >  ebound )
        {
            fillCount += ebound - sradius;
        }
        else if (sradius < sbound &&
                 eradius > ebound ) //there is a segment that is outside both boundaries
                                    //because it is too big and overlaps all of the boundary
        {
            fillCount = ebound-sbound;
        }
        else
            continue;
    }
    return (float)fillCount / (float)length;
}

