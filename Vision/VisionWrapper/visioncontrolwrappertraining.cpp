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
            b += g.getID() == Goal::BlueLeftGoal || g.getID() == Goal::BlueRightGoal || g.getID() == Goal::BlueUnknownGoal;
            y += g.getID() == Goal::YellowLeftGoal || g.getID() == Goal::YellowRightGoal || g.getID() == Goal::YellowUnknownGoal;
        }
        out << (y == 1) << " ";
        out << (y > 1) << " ";
        out << (b == 1) << " ";
        out << (b > 1) << " ";
        b = 0; y = 0;
        BOOST_FOREACH(Beacon be, data_wrapper->beacon_detection_history.at(i)) {
            b += be.getID() == Beacon::BlueBeacon;
            y += be.getID() == Beacon::YellowBeacon;
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
            b += g.getID() == Goal::BlueLeftGoal || g.getID() == Goal::BlueRightGoal || g.getID() == Goal::BlueUnknownGoal;
            y += g.getID() == Goal::YellowLeftGoal || g.getID() == Goal::YellowRightGoal || g.getID() == Goal::YellowUnknownGoal;
        }
        out << (y == 1) << " ";
        out << (y > 1) << " ";
        out << (b == 1) << " ";
        out << (b > 1) << " ";
        b = 0; y = 0;
        BOOST_FOREACH(Beacon be, data_wrapper->beacon_detection_history.at(i)) {
            b += be.getID() == Beacon::BlueBeacon;
            y += be.getID() == Beacon::YellowBeacon;
        }
        out << (y == 1) << " ";
        out << (b == 1) << endl;
    }
}

void VisionControlWrapper::writeBatchGroundTruth(ostream& out)
{

}

float VisionControlWrapper::evaluateFrame()
{
    //average distance error 3D for now

}

void VisionControlWrapper::generateLabels(const string& directory)
{
    setStream(directory + string("/image.strm"));
}

void VisionControlWrapper::resetHistory()
{
    data_wrapper->resetHistory();
}
