#ifndef CONTROLWRAPPER_H
#define CONTROLWRAPPER_H

#include "Vision/visioncontroller.h"

class NUSensorsData;
class NUActionatorsData;

class VisionControlWrapper
{
public:
    static VisionControlWrapper* getInstance();
    
    int runFrame();
    
private:
    VisionControlWrapper();
    
    static VisionControlWrapper* instance;
    
    VisionController* controller;
};

#endif // CONTROLWRAPPER_H
