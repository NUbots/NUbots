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
    int runFrame(bool forward = true);
    VisionControlWrapper();
    void getOptions(DataWrapper::INPUT_METHOD& method, bool& ok, string& istrm, string& sstrm, string& cfg, string& lname);

    static VisionControlWrapper* instance;

    MainWindow gui;
    VisionController controller;
    DataWrapper* wrapper;
};

#endif // CONTROLWRAPPER_H
