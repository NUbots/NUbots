#include "ClassifiedSection.h"

ClassifiedSection::ClassifiedSection()
{
    direction = 0;
    return;
}

ClassifiedSection::ClassifiedSection(int newDirection)
{
    direction = newDirection;
    return;
}
ClassifiedSection::~ClassifiedSection()
{
    return;
}

void ClassifiedSection::setDirection(int newDirection)
{
    direction = newDirection;
    return;
}

int ClassifiedSection::getDirection()
{
    return direction;
}

void ClassifiedSection::addScanLine(ScanLine* line)
{
    scanLines.push_back(*line);
    return;
}
ScanLine* ClassifiedSection::getScanLine(int position)
{
    return &(scanLines[position]);
}
int ClassifiedSection::getNumberOfScanLines()
{
    return scanLines.size();
}
