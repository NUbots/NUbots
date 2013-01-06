#include "visioncontrolwrapperrpi.h"

VisionControlWrapper* VisionControlWrapper::instance = 0;

VisionControlWrapper* VisionControlWrapper::getInstance(bool disp_on, bool cam)
{
    if(!instance)
        instance = new VisionControlWrapper(disp_on, cam);
    else
        instance->wrapper = DataWrapper::getInstance(disp_on, cam);
    return instance;
}

VisionControlWrapper::VisionControlWrapper(bool disp_on, bool cam)
{
    controller = VisionController::getInstance();
    wrapper = DataWrapper::getInstance(disp_on, cam);
}

int VisionControlWrapper::runFrame()
{
    static int frame = 0;
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "VisionControlWrapper::runFrame(): - frame " << frame << endl;
    #endif
    frame++;
    cout << "frame: " << frame << endl;
    if(!wrapper->updateFrame()) {
        #if VISION_WRAPPER_VERBOSITY > 1
            debug << "VisionControlWrapper::runFrame() - updateFrame() failed" << endl;
        #endif
        return -1;  //failure - do not run vision
    }
    return controller->runFrame(true, true);
}
