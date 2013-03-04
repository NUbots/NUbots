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
    data_wrapper = DataWrapper::getInstance();
}

/*!
  * @brief runs the vision system for a single frame - reading from the current stream
  * @return error code.
  */
int VisionControlWrapper::runFrame()
{
    if(!data_wrapper->updateFrame()) {
        return -1;  //failure - do not run vision
    }
    return controller.runFrame(true, true);
}

/*!
  * @brief runs the vision system for a single frame - using the supplied image
  * @param img The image to use.
  * @return error code.
  */
int VisionControlWrapper::runFrame(NUImage& img)
{
    data_wrapper->updateFrame(img);
    return controller->runFrame(true, true);
}

/*!
  * @brief sets the lookup table to use.
  * @param filename The file to load the LUT from
  * @return success.
  */
bool VisionControlWrapper::setLUT(const string& filename)
{
    return data_wrapper->loadLUTFromFile(filename);
}

/*!
  * @brief sets the image stream to use.
  * @param filename The file to load the images from
  * @return success.
  */
bool VisionControlWrapper::setImageStream(const string& filename)
{
    return data_wrapper->setImageStream(filename);
}

/*!
  * @brief resets the image stream.
  */
void VisionControlWrapper::restartStream()
{
    data_wrapper->resetStream();
}

/*!
  * @brief writes the detections seen so far to file in a boolean (seen|notseen) format
  * @param out The stream to print to.
  */
void VisionControlWrapper::writeBatchDetections(ostream &out)
{
    //for all frames
    for(unsigned int i=0; i<data_wrapper->getNumFramesProcessed(); i++) {
        //balls
        out << !data_wrapper->ball_detection_history.at(i).empty() << " ";

        //goals
        int y = 0, b = 0;
        BOOST_FOREACH(Goal g, data_wrapper->goal_detection_history.at(i)) {
            b += VisionFieldObject::isBlueGoal(g.getID());
            y += VisionFieldObject::isYellowGoal(g.getID());
        }
        out << (y == 1) << " " << (y > 1) << " " << (b == 1) << " " << (b > 1);

        //beacons
        b = 0; y = 0;
        BOOST_FOREACH(Beacon be, data_wrapper->beacon_detection_history.at(i)) {
            b += be.getID() == VisionFieldObject::BEACON_B;
            y += be.getID() == VisionFieldObject::BEACON_Y;
        }
        out << (y == 1) << " " << (b == 1) << endl;
    }
}

/*!
  * @brief prints labels to stream
  * @param out The stream to print to.
  */
void VisionControlWrapper::printLabels(ostream& out) const
{
    data_wrapper->printLabels(out);
}

/*!
  * @brief reads labels from stream in VisionFieldObject format
  * @param in The stream to read from.
  * @param labels The resulting labels (output parameter)
  */
bool VisionControlWrapper::readLabels(istream &in, vector<vector<VisionFieldObject *> > &labels) const
{
    return data_wrapper->readLabels(in, labels);
}

/*!
  * @brief reads labels from stream in ID/parameters format
  * @param in The stream to read from.
  * @param labels The resulting labels (output parameter)
  */
bool VisionControlWrapper::readLabels(istream& in, vector< vector< pair<VisionFieldObject::VFO_ID, Vector2<double> > > >& labels) const
{
    return data_wrapper->readLabels(in, labels);
}

/*!
  * @brief evaluates a frame given costs for total error and incidence count.
  * @param ground_truth The labels to compare with.
  * @param false_pos_costs A map between field object IDs and false positive costs
  * @param false_neg_costs A map between field object IDs and false negative costs
  * @return a map between field object IDs and total cost|incident number
  */
map<VisionFieldObject::VFO_ID, pair<float, int> > VisionControlWrapper::evaluateFrame(const vector<pair<VisionFieldObject::VFO_ID, Vector2<double> > >& ground_truth,
                                                                          const map<VisionFieldObject::VFO_ID, float>& false_pos_costs,
                                                                          const map<VisionFieldObject::VFO_ID, float>& false_neg_costs)
{
    vector<bool> matched_detections(data_wrapper->detections.size(), false);    //tracks what detections have been matched
    //initi
    map<VisionFieldObject::VFO_ID, pair<float, int> > errors;                   //the current erors and incidence counts
    vector<pair<VisionFieldObject::VFO_ID, Vector2<double> > >::const_iterator gt;

    //initialise errors map
    for(int i=0; i<VisionFieldObject::INVALID; i++)
        errors[VisionFieldObject::getVFOFromNum(i)] = pair<float, int>(0,0);

    //for each label
    for(gt=ground_truth.begin(); gt!=ground_truth.end(); gt++) {
        int d_num = 0;
        pair<int,float> best_match = pair<int,float>(-1,numeric_limits<float>::max());
        //look for the best matching detection
        BOOST_FOREACH(const VisionFieldObject* d_vfo, data_wrapper->detections) {
            //if the object is a match and the detection has not already been matched
            if(objectTypesMatch(d_vfo->getID(), gt->first) && !matched_detections.at(d_num)) {
                float err = d_vfo->findError(gt->second);
                //keep the best match
                if(err < best_match.second) {
                    best_match = pair<int, float>(d_num, err);
                }
            }
            d_num++;
        }

        if(best_match.first == -1) {
            //no detection for gt item - false negative
            //only apply if costs have been provided
            if(!false_neg_costs.empty()) {
                errors.at(gt->first).first += false_neg_costs.at(gt->first);
                errors.at(gt->first).second++;
            }
        }
        else {
            //detection - add it's cost to the map and mark it as matched
            errors.at(gt->first).first += best_match.second;
            errors.at(gt->first).second++;
            matched_detections.at(best_match.first) = true;
        }
    }

    //all remaining unmatched detections are false positives
    for(unsigned int d=0; d<matched_detections.size(); d++) {
        if(!matched_detections.at(d)) {
            //false positive
            //only apply if costs have been provided
            if(!false_pos_costs.empty()) {
                VisionFieldObject::VFO_ID id = data_wrapper->detections.at(d)->getID();
                errors.at(id).first += false_pos_costs.at(id);
                errors.at(id).second++;
            }
        }
    }

    return errors;
}

/*!
  * @brief evaluates a frame for precision and recall.
  * @param ground_truth The labels to compare with.
  * @return a map between field object IDs and total detections | false_pos | false_neg
  */
map<VisionFieldObject::VFO_ID, Vector3<double> > VisionControlWrapper::precisionRecall(const vector<pair<VisionFieldObject::VFO_ID, Vector2<double> > >& ground_truth)
{
    //average distance error 3D for now
    vector<bool> matched_detections(data_wrapper->detections.size(), false);
    map<VisionFieldObject::VFO_ID, Vector3<double> > result;    //final result
    map<VisionFieldObject::VFO_ID, double> matches;             //accumulator for detections
    map<VisionFieldObject::VFO_ID, double> false_pos;           //accumulator for false positives
    map<VisionFieldObject::VFO_ID, double> false_neg;           //accumulator for false negatives
    vector<pair<VisionFieldObject::VFO_ID, Vector2<double> > >::const_iterator gt;

    //initialise accumulators
    for(int i=0; i<VisionFieldObject::INVALID; i++) {
        VisionFieldObject::VFO_ID vfo_id = VisionFieldObject::getVFOFromNum(i);
        matches[vfo_id] = 0;
        false_pos[vfo_id] = 0;
        false_neg[vfo_id] = 0;
    }

    //for each label
    for(gt=ground_truth.begin(); gt!=ground_truth.end(); gt++) {
        int d_num = 0;
        pair<int,float> best_match = pair<int,float>(-1,numeric_limits<float>::max());
        //find the best match
        BOOST_FOREACH(const VisionFieldObject* d_vfo, data_wrapper->detections) {
            //if the object is a match and the detection has not already been matched
            if(objectTypesMatch(d_vfo->getID(), gt->first) && !matched_detections.at(d_num)) {
                float err = d_vfo->findError(gt->second);
                if(err < best_match.second) {
                    best_match = pair<int, float>(d_num, err);
                }
            }
            d_num++;
        }
        if(best_match.first == -1) {
            //no detection for gt item - false negative
            false_neg.at(gt->first)++;
        }
        else {
            //true positive
            matches.at(gt->first)++;
            matched_detections.at(best_match.first) = true;
        }
    }

    //all remaining unmatched detections are false positives
    for(unsigned int d=0; d<matched_detections.size(); d++) {
        if(!matched_detections.at(d)) {
            //false positive
            VisionFieldObject::VFO_ID id = data_wrapper->detections.at(d)->getID();
            false_pos.at(id)++;
        }
    }

    //build result from accumulators
    for(int i=0; i<VisionFieldObject::INVALID; i++) {
        VisionFieldObject::VFO_ID id = VisionFieldObject::getVFOFromNum(i);
        result[id].x = matches[id];
        result[id].y = false_pos[id];
        result[id].z = false_neg[id];
    }

    return result;
}

/*!
  * @brief returns whether two IDs are a match
  */
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
        return VisionFieldObject::isBeacon(id1);                    //can match any beacon
    case VisionFieldObject::GOAL_B_L:
        return id1 == id0 || id1 == VisionFieldObject::GOAL_B_U;    //can match unknown blue post
    case VisionFieldObject::GOAL_B_R:
        return id1 == id0 || id1 == VisionFieldObject::GOAL_B_U;    //can match unknown blue post
    case VisionFieldObject::GOAL_B_U:
        return VisionFieldObject::isBlueGoal(id1);                  //can match any blue post
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

/*!
  * @brief clears the detection history
  */
void VisionControlWrapper::resetHistory()
{
    data_wrapper->resetHistory();
}

/*!
  * @brief renders the current image and detections to a cv::Mat.
  * @param mat The cv::Mat to render to.
  * @return success
  */
bool VisionControlWrapper::renderFrame(cv::Mat& mat, bool lines_only)
{
    return data_wrapper->renderFrame(mat, lines_only);
}
