#ifndef CONTROLWRAPPER_H
#define CONTROLWRAPPER_H

#include "Infrastructure/Jobs/JobList.h"

#include "Vision/visioncontroller.h"

class NUSensorsData;
class NUActionatorsData;

class VisionControlWrapper
{
public:
    static VisionControlWrapper* getInstance();
    
    int runFrame();
    
    void process(JobList* jobs);
    
private:
    VisionControlWrapper();
    
    static VisionControlWrapper* instance;
    
    VisionController* controller;
    
    //! SavingImages:
    bool isSavingImages;
    bool isSavingImagesWithVaryingSettings;
    ofstream imagefile;
    ofstream sensorfile;
    NUSensorsData* m_sensor_data;               //!< pointer to shared sensor data object
    NUActionatorsData* m_actions;               //!< pointer to shared actionators data object
    CameraSettings currentSettings;
};

#endif // CONTROLWRAPPER_H
