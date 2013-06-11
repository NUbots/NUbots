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
#include <QMessageBox>

//for catching exceptions
class MyApplication : public QApplication {
public:
    MyApplication(int& argc, char ** argv) : QApplication(argc, argv) { }
    MyApplication(Display* dpy, Qt::HANDLE visual = 0, Qt::HANDLE cmap = 0, int flags = ApplicationFlags) : QApplication(dpy, visual, cmap, flags) { }
    virtual ~MyApplication() { }

    // reimplemented from QApplication so we can throw exceptions in slots
    virtual bool notify(QObject * receiver, QEvent * event) {
        try {
        return QApplication::notify(receiver, event);
        }
        catch(const std::exception& e) {
            QMessageBox::warning(NULL, "Exception", QString("Exception thrown: ") + e.what());
        }
        catch (const std::string& ex) {
            QMessageBox::warning(NULL, "Exception", QString(("Manual exception thrown: " + ex).c_str()));
        }
        catch (const char* str) {
            QMessageBox::warning(NULL, "Exception", QString("Manual exception thrown: ") + str);
        }
        catch (...) {
            QMessageBox::warning(NULL, "Exception", "Unknown exception thrown.");
        }
        return false;
    }
};
#endif



int qt();
int pc();
int rpi(bool disp_on, bool cam);

//#include "Infrastructure/NUSensorsData/NUSensorsData.h"
//#include "Infrastructure/NUImage/NUImage.h"
//#include <vector>
int main(int argc, char** argv)
{
//    std::vector<NUImage> imgs;
//    std::vector<NUSensorsData> sensors;
//    ifstream i((std::string(getenv("HOME")) + "/nubot/image.strm").c_str()),
//            s((std::string(getenv("HOME")) + "/nubot/sensor.strm").c_str());

//    while(i.good()) {
//        NUImage im;
//        try {
//            i >> im;
//        }
//        catch(exception e) {
//            break;
//        }
//        imgs.push_back(im);
//    }

//    while(s.good()) {
//        NUSensorsData sd;
//        try {
//            s >> sd;
//        }
//        catch(exception e) {
//            break;
//        }

//        sensors.push_back(sd);
//    }

//    std::cout << imgs.size() << " " << sensors.size() << std::endl;

//    return 0;

    #ifdef TARGET_IS_RPI
    //run with user option if given or display off by default
    switch(argc) {
    case 2:
        return rpi(std::string(argv[1]).compare("1") == 0, true);
    case 3:
        return rpi(std::string(argv[1]).compare("1") == 0, std::string(argv[2]).compare("1"));
    default:
        return rpi(false, true);
    }

    #elif TARGET_IS_PC
    //return pc();
    return qt();
    #else
    std::cout << "Error not a valid define! Must be TARGET_IS_RPI or TARGET_IS_PC" << std::endl;
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
        std::cout << "Running with display on" << std::endl;

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
        std::cout << "Error: " << error << std::endl;
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
//        std::cout << "frame: " << ++frame << std::endl;
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
//        std::cout << "Error: " << error << std::endl;
//    return 0;
//}

int qt()
{
#ifndef TARGET_IS_RPI
    MyApplication app(NULL);
#endif
    VisionControlWrapper* vision = VisionControlWrapper::getInstance();

    int error = vision->run();
    if(error != 0)
        std::cout << "Error: " << error << std::endl;
    return 0;
}
