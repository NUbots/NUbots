#include "visioncontrolwrapperqt.h"
#include <QApplication>

VisionControlWrapper* VisionControlWrapper::instance = 0;

VisionControlWrapper* VisionControlWrapper::getInstance()
{
    if(!instance)
        instance = new VisionControlWrapper();
    return instance;
}

VisionControlWrapper::VisionControlWrapper()
{
    wrapper = new DataWrapper(&gui);
    DataWrapper::instance = wrapper;
    controller = VisionController::getInstance();
}

int VisionControlWrapper::run()
{
    int frame = 1,
        error = 0;
    bool finished = false,
         next;

    gui.show();
    while(error == 0 && !finished) {
        gui.resetFlags();
        gui.setFrameNo(frame);
        next = false;
        error = runFrame();
        while(!next && !finished && error == 0) {
            QApplication::processEvents();
            next = gui.next();
            finished = gui.finished();
        }
        frame++;
    }
    gui.hide();
    return error;
}

int VisionControlWrapper::runFrame()
{
    if(!wrapper->updateFrame()) {
        #if VISION_WRAPPER_VERBOSITY > 1
            debug << "VisionControlWrapper::runFrame() - updateFrame() failed" << endl;
        #endif
        return -1;  //failure - do not run vision
    }
    return controller->runFrame(true, true);
}

