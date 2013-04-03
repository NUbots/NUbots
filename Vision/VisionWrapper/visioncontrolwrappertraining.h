#ifndef CONTROLWRAPPERTRAINING_H
#define CONTROLWRAPPERTRAINING_H

#include "Vision/visioncontroller.h"
#include "Vision/VisionWrapper/datawrappertraining.h"

#include <QImage>

class NUSensorsData;
class NUActionatorsData;

class VisionControlWrapper
{
public:
    static VisionControlWrapper* getInstance();

    int runFrame();
    int runFrame(NUImage& img, NUSensorsData &sensors);
    bool setLUT(const string& filename);
    bool setImageStream(const string& filename);
    bool setSensorStream(const string& filename);
    void restartStream();
    void resetHistory();
    bool renderFrame(QImage& mat, bool lines_only=false);
    void printLabels(ostream& out) const;
    bool readLabels(istream& in, vector< vector<VisionFieldObject*> >& labels) const;
   // bool readLabels(istream& in, vector< vector< pair<VFO_ID, Vector2<double> > > >& labels) const;

    map<VFO_ID, pair<float, int> > evaluateFrame(const vector<VisionFieldObject *> &ground_truth,
                                                 const map<VFO_ID, float>& false_pos_costs,
                                                 const map<VFO_ID, float>& false_neg_costs,
                                                 bool use_ground_errors);

    map<VFO_ID, Vector3<double> > precisionRecall(const vector<VisionFieldObject *>& ground_truth, bool use_ground_errors);


private:
    VisionControlWrapper();

    bool objectTypesMatch(VFO_ID id0, VFO_ID id1) const;

    static VisionControlWrapper* instance;  //! @var static singleton instance

    VisionController controller;           //! @var the system controller
    DataWrapper* data_wrapper;              //! @var the data wrapper
};

#endif // CONTROLWRAPPERTRAINING_H
