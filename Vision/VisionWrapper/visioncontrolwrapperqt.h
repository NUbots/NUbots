#ifndef CONTROLWRAPPER_H
#define CONTROLWRAPPER_H

#include "Vision/visioncontroller.h"
#include "Vision/VisionWrapper/datawrapperqt.h"
#include "mainwindow.h"

class NUSensorsData;
class NUActionatorsData;

class VisionControlWrapper
{
public:
    static VisionControlWrapper* getInstance();

    int run();

private:
    int runFrame(bool forward = true, int frame_no = 0);
    VisionControlWrapper();
    void getOptions(DataWrapper::INPUT_METHOD& method, bool& ok, std::string& istrm, std::string& sstrm, std::string& cfg, std::string& lname);

    static VisionControlWrapper* instance;

    MainWindow gui;
    VisionController controller;
    DataWrapper* wrapper;
};

#endif // CONTROLWRAPPER_H
