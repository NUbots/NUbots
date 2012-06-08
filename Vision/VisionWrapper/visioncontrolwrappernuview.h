#ifndef VISIONCONTROLWRAPPERNUVIEW_H
#define VISIONCONTROLWRAPPERNUVIEW_H

#include "Vision/visioncontroller.h"
#include "Vision/VisionWrapper/datawrappernuview.h"

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
    DataWrapper* wrapper;
    //for virtualnubot
    void setRawImage(NUImage* image);
    void setSensorData(NUSensorsData* sensors);
    void setFieldObjects(FieldObjects* fieldObjects);
};

#endif // VISIONCONTROLWRAPPERNUVIEW_H
