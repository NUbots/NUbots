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
    bool setImageStream(const string& filename);
    bool setSensorStream(const string& filename);
    void restartStream();
    void resetHistory();

    //batch outputs
    void writeBatchDetections(ostream& out);
    void writeBatchResults(ostream& out);
    void writeBatchGroundTruth(ostream& out);

    void printLabels(ostream& out) const;
    bool readLabels(istream& in, vector<VisionFieldObject*>& labels) const;

    float evaluateFrame();

private:
    VisionControlWrapper();

    static VisionControlWrapper* instance;

    VisionController* controller;
    DataWrapper* data_wrapper;
    int frame_no;
};

#endif // CONTROLWRAPPERTRAINING_H
