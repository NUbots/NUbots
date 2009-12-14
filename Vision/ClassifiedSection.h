#ifndef CLASSIFIEDSECTION_H
#define CLASSIFIEDSECTION_H

#include <vector>
#include "ScanLine.h"




class ClassifiedSection
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

    ClassifiedSection();
    ClassifiedSection(int direction);
    ~ClassifiedSection();

    void setDirection(int newDirection);
    int getDirection();
    void addScanLine(ScanLine* line);
    ScanLine* getScanLine(int posistion);

private:
    int direction;
    std::vector< ScanLine > scanLines;

};


#endif // CLASSIFIEDSECTION_H
