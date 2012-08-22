#ifndef CONTROLWRAPPERTRAINING_H
#define CONTROLWRAPPERTRAINING_H

#include "Vision/visioncontroller.h"
#include "Vision/VisionWrapper/datawrappertraining.h"

class NUSensorsData;
class NUActionatorsData;

class VisionControlWrapper
{
public:
    static VisionControlWrapper* getInstance();

    int runFrame();
    bool setLUT(const string& filename);
    bool setStream(const string& filename);
    void restartStream();
    void resetHistory();
    void writeDetections(ostream& out);

private:
    VisionControlWrapper();

    static VisionControlWrapper* instance;

    VisionController* controller;
    DataWrapper* data_wrapper;
    int frame_no;
};

#endif // CONTROLWRAPPERTRAINING_H
