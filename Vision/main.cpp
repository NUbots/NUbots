#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#ifdef TARGET_IS_RPI
    #include "VisionWrapper/visioncontrolwrapperrpi.h"
#elif TARGET_IS_PC
    #include "VisionWrapper/visioncontrolwrapperpc.h"
#elif TARGET_IS_NUVIEW
    #include "Vision/VisionWrapper/visioncontrolwrappernuview.h"
#elif TARGET_IS_TRAINING
    #include "Vision/VisionWrapper/visioncontrolwrappertraining.h"
#else
    #include "Vision/VisionWrapper/visioncontrolwrapperdarwin.h"
#endif

//#include <QApplication>

using namespace std;
using namespace cv;

int pc();
int rpi(bool disp_on);

int main(int argc, char** argv)
{
    #ifdef TARGET_IS_RPI
    return rpi((argc > 1 ? argv[1] : false));   //run with user option if given or display off by default
    #elif TARGET_IS_PC
    return pc();
    #else
    cout << "Error not a valid define! Must be TARGET_IS_RPI or TARGET_IS_PC" << endl;
    return 0;
    #endif
}

int rpi(bool disp_on)
{
#ifdef TARGET_IS_RPI
    char ESC_KEY            = 27,
         STEP_KEY           = ' ',
         STEP_TOGGLE_KEY    = 's';

    VisionControlWrapper* vision = VisionControlWrapper::getInstance(disp_on);

    char c=0;
    int error=0;
    bool stepping = true;
    while(c!=ESC_KEY && error==0) {
        //visiondata->updateFrame();
        error = vision->runFrame();
        if(stepping) {
            c=0;
            while(c!=STEP_KEY && c!=ESC_KEY && c!=STEP_TOGGLE_KEY) {
                c=waitKey();
            }
        }
        else {
            c = waitKey(30);
        }
        if(c==STEP_TOGGLE_KEY) {
            stepping = !stepping;
        }
    }
    if(error != 0)
        cout << "Error: " << error << endl;
#endif
}

int pc()
{
    //QApplication app(NULL);
    char ESC_KEY            = 27,
         STEP_KEY           = ' ',
         STEP_TOGGLE_KEY    = 's';

    VisionControlWrapper* vision = VisionControlWrapper::getInstance();

    char c=0;
    int error=0;
    bool stepping = false;
    while(c!=ESC_KEY && error==0) {
        //visiondata->updateFrame();
        error = vision->runFrame();
        if(stepping) {
            c=0;
            while(c!=STEP_KEY && c!=ESC_KEY && c!=STEP_TOGGLE_KEY) {
                c=waitKey();
            }
        }
        else {
            c = waitKey(1);
        }
        if(c==STEP_TOGGLE_KEY) {
            stepping = !stepping;
        }
    }
    if(error != 0)
        cout << "Error: " << error << endl;
}
