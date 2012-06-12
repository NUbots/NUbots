#ifndef CONTROLWRAPPER_H
#define CONTROLWRAPPER_H

#include "Infrastructure/Jobs/JobList.h"

#include "Vision/visioncontroller.h"
#include "Vision/VisionWrapper/datawrappercurrent.h"

class VisionControlWrapper
{
    friend class SaveImagesThread;
public:
    static VisionControlWrapper* getInstance();
    
    //! Vision Control Interface
    int runFrame();
    void process(JobList* jobs);
    
    //! Vision Save Images Interface
    void saveAnImage() const;
    
    //! Vision Performace Data Interface    
    int getNumFramesDropped() const {return data_wrapper->getNumFramesDropped();}      //! @brief Returns the number of dropped frames since start.
    int getNumFramesProcessed() const {return data_wrapper->getNumFramesProcessed();}  //! @brief Returns the number of processed frames since start.
    
private:
    VisionControlWrapper();
    
    static VisionControlWrapper* instance;
    
    VisionController* controller;
    DataWrapper* data_wrapper;
    SaveImagesThread* m_saveimages_thread;      //!< an external thread to do saving images in parallel with vision processing
};

#endif // CONTROLWRAPPER_H
