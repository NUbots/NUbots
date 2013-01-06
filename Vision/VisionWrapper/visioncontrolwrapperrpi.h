#ifndef CONTROLWRAPPER_H
#define CONTROLWRAPPER_H

#include "Vision/visioncontroller.h"
#include "Vision/VisionWrapper/datawrapperrpi.h"

class NUSensorsData;
class NUActionatorsData;

class VisionControlWrapper
{
public:
    static VisionControlWrapper* getInstance(bool disp_on=false, bool cam=true);
    
    int runFrame();
    
private:
    VisionControlWrapper(bool disp_on=false, bool cam=true);
    
    static VisionControlWrapper* instance;
    
    VisionController* controller;
    DataWrapper* wrapper;
};

#endif // CONTROLWRAPPER_H
