#include "visioncontrolwrappertraining.h"
#include <boost/foreach.hpp>

VisionControlWrapper* VisionControlWrapper::instance = 0;

VisionControlWrapper* VisionControlWrapper::getInstance()
{
    if(!instance)
        instance = new VisionControlWrapper();
    return instance;
}

VisionControlWrapper::VisionControlWrapper()
{
    controller = VisionController::getInstance();
    data_wrapper = DataWrapper::getInstance();
    frame_no = 1;
}

int VisionControlWrapper::runFrame()
{
    frame_no++;
    if(!data_wrapper->updateFrame()) {
        return -1;  //failure - do not run vision
    }
    return controller->runFrame(true, true);
}

bool VisionControlWrapper::setLUT(const string& filename)
{
    return data_wrapper->loadLUTFromFile(filename);
}

bool VisionControlWrapper::setImageStream(const string& filename)
{
    frame_no = 1;
    return data_wrapper->setImageStream(filename);
}

bool VisionControlWrapper::setSensorStream(const string& filename)
{
    return data_wrapper->setSensorStream(filename);
}

void VisionControlWrapper::restartStream()
{
    frame_no = 1;
    data_wrapper->resetStream();
}

void VisionControlWrapper::writeBatchDetections(ostream &out)
{
    for(unsigned int i=0; i<data_wrapper->ball_detection_history.size(); i++) {
        out << !data_wrapper->ball_detection_history.at(i).empty() << " ";
        int y = 0, b = 0;
        BOOST_FOREACH(Goal g, data_wrapper->goal_detection_history.at(i)) {
            b += VisionFieldObject::isBlueGoal(g.getID());
            y += VisionFieldObject::isYellowGoal(g.getID());
        }
        out << (y == 1) << " ";
        out << (y > 1) << " ";
        out << (b == 1) << " ";
        out << (b > 1) << " ";
        b = 0; y = 0;
        BOOST_FOREACH(Beacon be, data_wrapper->beacon_detection_history.at(i)) {
            b += be.getID() == VisionFieldObject::BEACON_B;
            y += be.getID() == VisionFieldObject::BEACON_Y;
        }
        out << (y == 1) << " ";
        out << (b == 1) << endl;
    }
}

void VisionControlWrapper::writeBatchResults(ostream& out)
{
    for(unsigned int i=0; i<data_wrapper->ball_detection_history.size(); i++) {
        out << !data_wrapper->ball_detection_history.at(i).empty() << " ";
        int y = 0, b = 0;
        BOOST_FOREACH(Goal g, data_wrapper->goal_detection_history.at(i)) {
            b += VisionFieldObject::isBlueGoal(g.getID());
            y += VisionFieldObject::isYellowGoal(g.getID());
        }
        out << (y == 1) << " ";
        out << (y > 1) << " ";
        out << (b == 1) << " ";
        out << (b > 1) << " ";
        b = 0; y = 0;
        BOOST_FOREACH(Beacon be, data_wrapper->beacon_detection_history.at(i)) {
            b += be.getID() == VisionFieldObject::BEACON_B;
            y += be.getID() == VisionFieldObject::BEACON_Y;
        }
        out << (y == 1) << " ";
        out << (b == 1) << endl;
    }
}

void VisionControlWrapper::printLabels(ostream& out) const
{
    data_wrapper->printLabels(out);
}

bool VisionControlWrapper::readLabels(istream &in, vector<vector<VisionFieldObject *> > &labels) const
{
    return data_wrapper->readLabels(in, labels);
}

bool VisionControlWrapper::readLabels(istream& in, vector< vector< pair<VisionFieldObject::VFO_ID, Vector2<double> > > >& labels) const
{
    return data_wrapper->readLabels(in, labels);
}

map<VisionFieldObject::VFO_ID, float> VisionControlWrapper::evaluateFrame(const vector<pair<VisionFieldObject::VFO_ID, Vector2<double> > >& ground_truth, float false_pos_cost, float false_neg_cost)
{
    //average distance error 3D for now
    vector<bool> matched_detections(false, data_wrapper->detections.size());
    //initi
    map<VisionFieldObject::VFO_ID, float> errors;

    errors[VisionFieldObject::BALL] = 0;
    errors[VisionFieldObject::GOAL_Y_L] = 0;
    errors[VisionFieldObject::GOAL_Y_R] = 0;
    errors[VisionFieldObject::GOAL_Y_U] = 0;
    errors[VisionFieldObject::GOAL_B_L] = 0;
    errors[VisionFieldObject::GOAL_B_R] = 0;
    errors[VisionFieldObject::GOAL_B_U] = 0;
    errors[VisionFieldObject::BEACON_Y] = 0;
    errors[VisionFieldObject::BEACON_B] = 0;
    errors[VisionFieldObject::BEACON_U] = 0;
    errors[VisionFieldObject::FIELDLINE] = 0;
    errors[VisionFieldObject::OBSTACLE] = 0;

    for(unsigned int gt=0; gt<ground_truth.size(); gt++) {
        int d_num = 0;
        pair<int,float> best_match = pair<int,float>(-1,numeric_limits<float>::max());
        BOOST_FOREACH(const VisionFieldObject* d_vfo, data_wrapper->detections) {
            if(objectTypesMatch(d_vfo->getID(), ground_truth[gt].first)) {
                float err = d_vfo->findError(ground_truth[gt].second);
                if(err < best_match.second) {
                    best_match = pair<int, float>(d_num, err);
                }
            }
            d_num++;
        }
        if(best_match.first == -1) {
            //no detection for gt item - false negative
            errors[ground_truth[gt].first] += false_neg_cost;
        }
        else {
            errors[ground_truth[gt].first] += best_match.second;
            matched_detections[best_match.first] = true;
        }
    }
    for(unsigned int d=0; d<matched_detections.size(); d++) {
        if(!matched_detections[d]) {
            //false positive
            errors[data_wrapper->detections[d]->getID()] += false_pos_cost;
        }
    }

    return errors;
}

bool VisionControlWrapper::objectTypesMatch(VisionFieldObject::VFO_ID id0, VisionFieldObject::VFO_ID id1) const
{
    switch(id0) {
    case VisionFieldObject::BALL:
        return id1 == id0;
    case VisionFieldObject::OBSTACLE:
        return id1 == id0;
    case VisionFieldObject::FIELDLINE:
        return id1 == id0;
    case VisionFieldObject::BEACON_B:
        return id1 == id0 || id1 == VisionFieldObject::BEACON_U;    //can match unknown beacon
    case VisionFieldObject::BEACON_Y:
        return id1 == id0 || id1 == VisionFieldObject::BEACON_U;    //can match unknown beacon
    case VisionFieldObject::BEACON_U:
        return VisionFieldObject::isBeacon(id1);    //can match any beacon
    case VisionFieldObject::GOAL_B_L:
        return id1 == id0 || id1 == VisionFieldObject::GOAL_B_U;    //can match unknown blue post
    case VisionFieldObject::GOAL_B_R:
        return id1 == id0 || id1 == VisionFieldObject::GOAL_B_U;    //can match unknown blue post
    case VisionFieldObject::GOAL_B_U:
        return VisionFieldObject::isBlueGoal(id1);  //can match any blue post
    case VisionFieldObject::GOAL_Y_L:
        return id1 == id0 || id1 == VisionFieldObject::GOAL_Y_U;    //can match unknown yellow post
    case VisionFieldObject::GOAL_Y_R:
        return id1 == id0 || id1 == VisionFieldObject::GOAL_Y_U;    //can match unknown yello post
    case VisionFieldObject::GOAL_Y_U:
        return VisionFieldObject::isYellowGoal(id1);  //can match any yellow post
    default:
        errorlog << "VisionControlWrapper::objectTypesMatch - invalid id" << endl;
        return false;
    }
}

void VisionControlWrapper::resetHistory()
{
    data_wrapper->resetHistory();
}
