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
    int runFrame(NUImage& img);
    bool setLUT(const string& filename);
    bool setImageStream(const string& filename);
    bool setSensorStream(const string& filename);
    void restartStream();
    void resetHistory();

    bool renderFrame(cv::Mat& mat);

    //batch outputs
    void writeBatchDetections(ostream& out);
    void writeBatchResults(ostream& out);

    void printLabels(ostream& out) const;
    bool readLabels(istream& in, vector< vector<VisionFieldObject*> >& labels) const;
    bool readLabels(istream& in, vector< vector< pair<VisionFieldObject::VFO_ID, Vector2<double> > > >& labels) const;

    map<VisionFieldObject::VFO_ID, float> evaluateFrame(const vector<pair<VisionFieldObject::VFO_ID, Vector2<double> > >& ground_truth, float false_pos_cost, float false_neg_cost);

private:
    VisionControlWrapper();

    bool objectTypesMatch(VisionFieldObject::VFO_ID id0, VisionFieldObject::VFO_ID id1) const;

    static VisionControlWrapper* instance;

    VisionController* controller;
    DataWrapper* data_wrapper;
    int frame_no;
};

#endif // CONTROLWRAPPERTRAINING_H
