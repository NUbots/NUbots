#include <iostream>

#ifdef TARGET_IS_RPI
    #include "VisionWrapper/visioncontrolwrapperrpi.h"
#elif TARGET_IS_PC
    //#include "VisionWrapper/visioncontrolwrapperpc.h"
    #include "VisionWrapper/visioncontrolwrapperqt.h"
#elif TARGET_IS_NUVIEW
    #include "Vision/VisionWrapper/visioncontrolwrappernuview.h"
#elif TARGET_IS_TRAINING
    #include "Vision/VisionWrapper/visioncontrolwrappertraining.h"
#else
    #include "Vision/VisionWrapper/visioncontrolwrapperdarwin.h"
#endif

#ifndef TARGET_IS_RPI
#include <QApplication>
#include <QFileDialog>
#endif

using namespace std;
using namespace cv;

int qt();
int pc();
int rpi(bool disp_on, bool cam);

int main(int argc, char** argv)
{
    #ifdef TARGET_IS_RPI
    //run with user option if given or display off by default
    switch(argc) {
    case 2:
        return rpi(string(argv[1]).compare("1") == 0, true);
    case 3:
        return rpi(string(argv[1]).compare("1") == 0, string(argv[2]).compare("1"));
    default:
        return rpi(false, true);
    }

    #elif TARGET_IS_PC
    //return pc();
    return qt();
    #else
    cout << "Error not a valid define! Must be TARGET_IS_RPI or TARGET_IS_PC" << endl;
    return 0;
    #endif
}

int rpi(bool disp_on, bool cam)
{
#ifdef TARGET_IS_RPI
    char ESC_KEY            = 27,
         STEP_KEY           = ' ',
         STEP_TOGGLE_KEY    = 's';

    VisionControlWrapper* vision = VisionControlWrapper::getInstance(disp_on, cam);

    if(disp_on)
        cout << "Running with display on" << endl;

    char c=0;
    int error=0;
    bool stepping = disp_on;
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
    return 0;
}

//int pc()
//{
//#ifndef TARGET_IS_RPI
//    QApplication app(NULL);
//#endif
//    char ESC_KEY            = 27,
//         STEP_KEY           = ' ',
//         STEP_TOGGLE_KEY    = 's';

//    VisionControlWrapper* vision = VisionControlWrapper::getInstance();

//    char c=0;
//    int error=0;
//    bool stepping = true;
//    int frame = 0;
//    while(c!=ESC_KEY && error==0) {
//        //visiondata->updateFrame();
//        cout << "frame: " << ++frame << endl;
//        error = vision->runFrame();
//        if(stepping) {
//            c=0;
//            while(c!=STEP_KEY && c!=ESC_KEY && c!=STEP_TOGGLE_KEY) {
//                c=waitKey();
//            }
//        }
//        else {
//            c = waitKey(1000);
//        }
//        if(c==STEP_TOGGLE_KEY) {
//            stepping = !stepping;
//        }
//    }
//    if(error != 0)
//        cout << "Error: " << error << endl;
//    return 0;
//}

int qt()
{
#ifndef TARGET_IS_RPI
    QApplication app(NULL);
#endif
    VisionControlWrapper* vision = VisionControlWrapper::getInstance();

    int error = vision->run();
    if(error != 0)
        cout << "Error: " << error << endl;
    return 0;
}
