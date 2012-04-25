#ifndef CONTROLWRAPPER_H
#define CONTROLWRAPPER_H

#include "visioncontroller.h"

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
