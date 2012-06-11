#ifndef VISIONCONTROLWRAPPERNUVIEW_H
#define VISIONCONTROLWRAPPERNUVIEW_H

#include "Vision/visioncontroller.h"
#include "Vision/VisionWrapper/datawrappernuview.h"

class NUSensorsData;
class NUActionatorsData;

class VisionControlWrapper
{
    friend class virtualNUbot;
public:
    static VisionControlWrapper* getInstance();
    
    //void setCallBack(virtualNUbot* virtual_nubot);

    int runFrame();
    void saveAnImage() const;
    
private:
    VisionControlWrapper();
    
    static VisionControlWrapper* instance;
    
    VisionController* controller;
    DataWrapper* wrapper;
    //for virtualnubot
    void setRawImage(const NUImage* image);
    void setSensorData(NUSensorsData* sensors);
    void setFieldObjects(FieldObjects* field_objects);
    void setLUT(unsigned char* vals);
    void classifyImage(ClassifiedImage& classed_image);
    void classifyPreviewImage(ClassifiedImage &target,unsigned char* temp_vals) const;

};

#endif // VISIONCONTROLWRAPPERNUVIEW_H
