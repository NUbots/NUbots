#ifndef CLASSIFIEDSECTION_H
#define CLASSIFIEDSECTION_H

#include <vector>
#include "ScanLine.h"




class ClassifiedSection
{

public:


    ClassifiedSection();
    ClassifiedSection(int direction);
    ~ClassifiedSection();

    void setDirection(int newDirection);
    int getDirection();
    void addScanLine(const ScanLine& line);
    ScanLine* getScanLine(int position);
    int getNumberOfScanLines();

private:
    int direction;
    std::vector< ScanLine > scanLines;

};


#endif // CLASSIFIEDSECTION_H
