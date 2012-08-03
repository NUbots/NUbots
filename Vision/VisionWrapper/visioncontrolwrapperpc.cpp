#include "visioncontrolwrapperpc.h"

VisionControlWrapper* VisionControlWrapper::instance = 0;

VisionControlWrapper* VisionControlWrapper::getInstance()
{
    if(!instance)
        instance = new VisionControlWrapper();
    return instance;
}

VisionControlWrapper::VisionControlWrapper()
{
    controller = VisionController::getInstance();
    wrapper = DataWrapper::getInstance();
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
